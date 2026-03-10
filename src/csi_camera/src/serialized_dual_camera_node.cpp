#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

#define BUFFER_COUNT 4

/**
 * ================================================================================
 * SERIALIZED DUAL CSI CAMERA NODE - V4L2 API
 * ================================================================================
 * 
 * Direct V4L2 implementation for dual camera support:
 * - Synchronous ioctl-based API (no async complexity)
 * - Direct /dev/videoN device access
 * - BGR24 format (no conversion needed!)
 * - mmap buffer management (zero-copy)
 * - Serialized processing (alternating capture)
 * 
 * This is the PROPER way to do dual camera in C++ on Linux.
 * ================================================================================
 */

class V4L2DualCameraNode : public rclcpp::Node
{
public:
    V4L2DualCameraNode() : Node("serialized_dual_camera_node"), current_camera_(0)
    {
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "🎥 V4L2 Dual Camera Node (C++)");
        RCLCPP_INFO(this->get_logger(), "========================================");

        // Parameters
        this->declare_parameter("fps", 8);
        this->declare_parameter("width", 640);
        this->declare_parameter("height", 480);

        fps_ = this->get_parameter("fps").as_int();
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();

        RCLCPP_INFO(this->get_logger(), "Config: %dx%d @ %d FPS per camera", 
                    width_, height_, fps_);

        // Publishers
        pub_cam0_ = this->create_publisher<sensor_msgs::msg::Image>("/cam0HP/image_raw", 10);
        pub_cam1_ = this->create_publisher<sensor_msgs::msg::Image>("/cam1HP/image_raw", 10);

        // Initialize cameras
        try {
            RCLCPP_INFO(this->get_logger(), "Opening Camera 0 (/dev/video0)...");
            init_camera(cam0_, "/dev/video0", 0);
            RCLCPP_INFO(this->get_logger(), "✓ Camera 0 ready");

            RCLCPP_INFO(this->get_logger(), "Opening Camera 1 (/dev/video8)...");
            init_camera(cam1_, "/dev/video8", 1);
            RCLCPP_INFO(this->get_logger(), "✓ Camera 1 ready");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Initialization failed: %s", e.what());
            throw;
        }

        // Timer for serialized capture (alternating cameras)
        double timer_period = 1.0 / (fps_ * 2);  // 2x rate for alternation
        capture_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timer_period),
            std::bind(&V4L2DualCameraNode::capture_tick, this));

        // Stats timer
        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&V4L2DualCameraNode::log_stats, this));

        frame_count_[0] = 0;
        frame_count_[1] = 0;
        last_log_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "✅ Ready! Serialized capture at %d Hz per camera", fps_);
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

    ~V4L2DualCameraNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down...");
        cleanup_camera(cam0_);
        cleanup_camera(cam1_);
        RCLCPP_INFO(this->get_logger(), "✅ Shutdown complete");
    }

private:
    struct V4L2Camera {
        int fd = -1;
        void* buffers[BUFFER_COUNT];
        size_t buffer_sizes[BUFFER_COUNT];
        int id = -1;
        bool streaming = false;
    };

    void init_camera(V4L2Camera& cam, const char* device, int id)
    {
        cam.id = id;

        // Open device
        cam.fd = open(device, O_RDWR | O_NONBLOCK);
        if (cam.fd < 0) {
            throw std::runtime_error(std::string("Failed to open ") + device + 
                                    ": " + strerror(errno));
        }

        // Query capabilities
        struct v4l2_capability cap;
        if (ioctl(cam.fd, VIDIOC_QUERYCAP, &cap) < 0) {
            throw std::runtime_error("VIDIOC_QUERYCAP failed: " + std::string(strerror(errno)));
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
            throw std::runtime_error("Device does not support video capture");
        }

        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
            throw std::runtime_error("Device does not support streaming");
        }

        RCLCPP_INFO(this->get_logger(), "  Device: %s", cap.card);

        // Set format (BGR24)
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = width_;
        fmt.fmt.pix.height = height_;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;  // BGR3 format
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (ioctl(cam.fd, VIDIOC_S_FMT, &fmt) < 0) {
            throw std::runtime_error("VIDIOC_S_FMT failed: " + std::string(strerror(errno)));
        }

        // Verify format was set correctly
        if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_BGR24) {
            RCLCPP_WARN(this->get_logger(), "  BGR24 not supported, got format: %c%c%c%c",
                       (fmt.fmt.pix.pixelformat >> 0) & 0xFF,
                       (fmt.fmt.pix.pixelformat >> 8) & 0xFF,
                       (fmt.fmt.pix.pixelformat >> 16) & 0xFF,
                       (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
        }

        RCLCPP_INFO(this->get_logger(), "  Format: %dx%d", 
                   fmt.fmt.pix.width, fmt.fmt.pix.height);

        // Request buffers
        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));
        req.count = BUFFER_COUNT;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (ioctl(cam.fd, VIDIOC_REQBUFS, &req) < 0) {
            throw std::runtime_error("VIDIOC_REQBUFS failed: " + std::string(strerror(errno)));
        }

        RCLCPP_INFO(this->get_logger(), "  Allocated %d buffers", req.count);

        // mmap buffers and queue them
        for (unsigned int i = 0; i < req.count; i++) {
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (ioctl(cam.fd, VIDIOC_QUERYBUF, &buf) < 0) {
                throw std::runtime_error("VIDIOC_QUERYBUF failed: " + std::string(strerror(errno)));
            }

            cam.buffers[i] = mmap(NULL, buf.length,
                                 PROT_READ | PROT_WRITE,
                                 MAP_SHARED,
                                 cam.fd, buf.m.offset);

            if (cam.buffers[i] == MAP_FAILED) {
                throw std::runtime_error("mmap failed: " + std::string(strerror(errno)));
            }

            cam.buffer_sizes[i] = buf.length;

            // Queue buffer for capture
            if (ioctl(cam.fd, VIDIOC_QBUF, &buf) < 0) {
                throw std::runtime_error("VIDIOC_QBUF failed: " + std::string(strerror(errno)));
            }
        }

        // Start streaming
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(cam.fd, VIDIOC_STREAMON, &type) < 0) {
            throw std::runtime_error("VIDIOC_STREAMON failed: " + std::string(strerror(errno)));
        }

        cam.streaming = true;
        RCLCPP_INFO(this->get_logger(), "  ✓ Streaming started");
    }

    void cleanup_camera(V4L2Camera& cam)
    {
        if (cam.fd < 0) return;

        // Stop streaming
        if (cam.streaming) {
            enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ioctl(cam.fd, VIDIOC_STREAMOFF, &type);
        }

        // Unmap buffers
        for (int i = 0; i < BUFFER_COUNT; i++) {
            if (cam.buffers[i] != nullptr && cam.buffers[i] != MAP_FAILED) {
                munmap(cam.buffers[i], cam.buffer_sizes[i]);
            }
        }

        close(cam.fd);
        cam.fd = -1;
    }

    void capture_tick()
    {
        try {
            // Alternate between cameras
            V4L2Camera& cam = (current_camera_ == 0) ? cam0_ : cam1_;
            auto& publisher = (current_camera_ == 0) ? pub_cam0_ : pub_cam1_;
            
            capture_and_publish(cam, publisher);

            // Switch to next camera
            current_camera_ = 1 - current_camera_;

        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Capture error: %s", e.what());
        }
    }

    void capture_and_publish(V4L2Camera& cam,
                            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& publisher)
    {
        if (cam.fd < 0) return;

        // Use select() for non-blocking check
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(cam.fd, &fds);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 50000;  // 50ms timeout

        int ready = select(cam.fd + 1, &fds, NULL, NULL, &timeout);
        
        if (ready < 0) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                 "select() error");
            return;
        }
        if (ready == 0) {
            // No data ready yet
            return;
        }

        // Dequeue buffer
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(cam.fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno != EAGAIN) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "VIDIOC_DQBUF failed: %s", strerror(errno));
            }
            return;
        }

        // Frame data is in mmap buffer - create cv::Mat (BGR already!)
        cv::Mat bgr_frame(height_, width_, CV_8UC3, cam.buffers[buf.index]);

        // Create ROS message
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = (cam.id == 0) ? "camera_input_tray" : "camera_output_tray";

        auto msg = cv_bridge::CvImage(header, "bgr8", bgr_frame).toImageMsg();
        publisher->publish(*msg);

        frame_count_[cam.id]++;

        // Requeue buffer for next capture
        if (ioctl(cam.fd, VIDIOC_QBUF, &buf) < 0) {
            RCLCPP_ERROR(this->get_logger(), "VIDIOC_QBUF failed: %s", strerror(errno));
        }
    }

    void log_stats()
    {
        auto now = this->now();
        double dt = (now - last_log_time_).seconds();

        if (dt > 0) {
            double fps0 = frame_count_[0] / dt;
            double fps1 = frame_count_[1] / dt;
            RCLCPP_INFO(this->get_logger(), "📊 FPS: Cam0=%.1f, Cam1=%.1f", fps0, fps1);
        }

        frame_count_[0] = 0;
        frame_count_[1] = 0;
        last_log_time_ = now;
    }

    // Members
    V4L2Camera cam0_;
    V4L2Camera cam1_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_cam0_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_cam1_;
    
    rclcpp::TimerBase::SharedPtr capture_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;

    int current_camera_;
    int fps_;
    int width_;
    int height_;

    int frame_count_[2];
    rclcpp::Time last_log_time_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<V4L2DualCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
