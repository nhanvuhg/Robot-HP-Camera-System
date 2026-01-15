/**
 * @file vision_decision_node.cpp
 * @brief Handles all YOLO/AI vision processing and ROI-based decision making
 * 
 * Responsibilities:
 * - Subscribe to YOLO bounding boxes (cam0, cam1)
 * - Process ROI detection for input tray rows
 * - Process ROI detection for output tray slots
 * - Publish selected row/slot decisions
 * 
 * Topics Published:
 * - /vision/input_tray/selected_row (Int32)
 * - /vision/input_tray/row_status (Int32MultiArray) - 5 values: 0=empty, 1=full
 * - /vision/input_tray/empty (Bool)
 * - /vision/output_tray/selected_slot (Int32)
 * - /vision/output_tray/slot_status (Int32MultiArray) - 9 values: 0=empty, 1=occupied
 * 
 * Topics Subscribed:
 * - cam0HP/yolo/bounding_boxes (Detection2DArray) - Input tray
 * - cam1HP/yolo/bounding_boxes (Detection2DArray) - Output tray
 * - /robot/set_mode (Int32) - To know current mode (1=AUTO, 2=AI, 3=MANUAL)
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

#include <vector>
#include <array>
#include <deque>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <atomic>
#include <chrono>

using vision_msgs::msg::Detection2D;
using vision_msgs::msg::Detection2DArray;

// ============================================================================
// UTILITY STRUCTURES
// ============================================================================

struct Point2 {
    float x{}, y{};
};

struct ROIQuad {
    std::array<Point2, 4> pts;
    int min_x{}, max_x{}, min_y{}, max_y{};

    static ROIQuad FromCorners(const std::vector<std::pair<int, int>>& corners) {
        ROIQuad r{};
        for (size_t i = 0; i < 4; ++i) {
            r.pts[i] = Point2{static_cast<float>(corners[i].first),
                              static_cast<float>(corners[i].second)};
        }
        r.min_x = std::min({corners[0].first, corners[1].first,
                            corners[2].first, corners[3].first});
        r.max_x = std::max({corners[0].first, corners[1].first,
                            corners[2].first, corners[3].first});
        r.min_y = std::min({corners[0].second, corners[1].second,
                            corners[2].second, corners[3].second});
        r.max_y = std::max({corners[0].second, corners[1].second,
                            corners[2].second, corners[3].second});
        return r;
    }

    inline bool bbox_contains(float x, float y) const {
        return (x >= min_x && x <= max_x && y >= min_y && y <= max_y);
    }

    inline bool contains(float x, float y) const {
        if (!bbox_contains(x, y)) return false;
        auto cross = [](const Point2& a, const Point2& b, const Point2& c) {
            return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        };
        const Point2 p{x, y};
        float c0 = cross(pts[0], pts[1], p);
        float c1 = cross(pts[1], pts[2], p);
        float c2 = cross(pts[2], pts[3], p);
        float c3 = cross(pts[3], pts[0], p);
        
        bool all_nonneg = (c0 >= 0 && c1 >= 0 && c2 >= 0 && c3 >= 0);
        bool all_nonpos = (c0 <= 0 && c1 <= 0 && c2 <= 0 && c3 <= 0);
        return all_nonneg || all_nonpos;
    }
};

struct RowFilter {
    size_t window = 5;
    int max_fall = 2;
    int ready_consec = 3;

    std::deque<int> hist;
    int last_filtered = 0;
    int ready_streak = 0;

    int filter_count(int raw_count) {
        hist.push_back(raw_count);
        if (hist.size() > window) hist.pop_front();

        std::vector<int> tmp(hist.begin(), hist.end());
        std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
        int med = tmp[tmp.size() / 2];

        int y = med;
        if (y < last_filtered - max_fall) {
            y = last_filtered - max_fall;
        }
        last_filtered = y;
        return y;
    }

    bool update_ready(bool raw_ready) {
        if (raw_ready) ready_streak++;
        else ready_streak = 0;
        return ready_streak >= ready_consec;
    }

    void clear() {
        hist.clear();
        last_filtered = 0;
        ready_streak = 0;
    }
};

enum class SlotStableState : int {
    EMPTY = 0,
    OCC_OK = 1,
    MIS = 2
};

enum class ControlMode : uint8_t {
    AUTO = 1,
    AI = 2,
    MANUAL = 3
};

// ============================================================================
// VISION DECISION NODE
// ============================================================================

class VisionDecisionNode : public rclcpp::Node {
public:
    VisionDecisionNode() : Node("vision_decision_node") {
        RCLCPP_INFO(get_logger(), "[VISION] === Vision Decision Node Starting ===");
        
        initInputTrayROIs();
        initOutputTrayROIs();
        
        // Initialize filters
        row_filters_.assign(5, RowFilter{});
        row_full_.assign(5, false);
        
        // Publishers
        pub_selected_row_ = create_publisher<std_msgs::msg::Int32>(
            "/vision/input_tray/selected_row", 10);
        pub_row_status_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            "/vision/input_tray/row_status", 10);
        pub_input_empty_ = create_publisher<std_msgs::msg::Bool>(
            "/vision/input_tray/empty", 10);
        pub_selected_slot_ = create_publisher<std_msgs::msg::Int32>(
            "/vision/output_tray/selected_slot", 10);
        pub_slot_status_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            "/vision/output_tray/slot_status", 10);
        
        // Subscriptions with SensorDataQoS for low latency
        sub_cam0_ = create_subscription<Detection2DArray>(
            "cam0HP/yolo/bounding_boxes", rclcpp::SensorDataQoS(),
            std::bind(&VisionDecisionNode::camera1Callback, this, std::placeholders::_1));
            
        sub_cam1_ = create_subscription<Detection2DArray>(
            "cam1HP/yolo/bounding_boxes", rclcpp::SensorDataQoS(),
            std::bind(&VisionDecisionNode::camera2Callback, this, std::placeholders::_1));
        
        // Mode subscription
        sub_mode_ = create_subscription<std_msgs::msg::Int32>(
            "/robot/set_mode", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                current_mode_ = static_cast<ControlMode>(msg->data);
                RCLCPP_INFO(get_logger(), "[VISION] Mode changed to: %d", msg->data);
            });
        
        RCLCPP_INFO(get_logger(), "[VISION] === Vision Decision Node Ready ===");
    }

private:
    // ========================================================================
    // CONSTANTS
    // ========================================================================
    static constexpr int INPUT_ROW_THRESHOLD = 8;
    static constexpr float DETECTION_SCORE_THRESH = 0.6f;
    static constexpr int SLOT_CONFIRM_FRAMES = 3;

    // ========================================================================
    // MODE STATE
    // ========================================================================
    std::atomic<ControlMode> current_mode_{ControlMode::AUTO};

    // ========================================================================
    // INPUT TRAY STATE
    // ========================================================================
    std::vector<ROIQuad> input_tray_rois_;
    std::vector<RowFilter> row_filters_;
    std::vector<bool> row_full_;
    bool input_tray_empty_{false};
    int selected_input_row_{-1};

    // ========================================================================
    // OUTPUT TRAY STATE
    // ========================================================================
    std::array<ROIQuad, 8> output_tray_rois_;
    std::array<SlotStableState, 9> slot_stable_state_;
    std::array<int, 9> slot_empty_streak_;
    std::array<int, 9> slot_occ_streak_;
    std::array<int, 9> slot_mis_streak_;
    int selected_output_slot_{-1};
    std::mutex slot_detection_mutex_;

    // ========================================================================
    // PERFORMANCE TRACKING
    // ========================================================================
    std::atomic<uint64_t> callback_count_{0};
    std::atomic<uint64_t> total_callback_time_us_{0};

    // ========================================================================
    // ROS INTERFACES
    // ========================================================================
    rclcpp::Subscription<Detection2DArray>::SharedPtr sub_cam0_;
    rclcpp::Subscription<Detection2DArray>::SharedPtr sub_cam1_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_mode_;
    
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_selected_row_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_row_status_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_input_empty_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_selected_slot_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_slot_status_;

    // ========================================================================
    // INITIALIZATION
    // ========================================================================
    void initInputTrayROIs() {
        std::vector<std::vector<std::pair<int, int>>> row_corners = {
            {{212, 554}, {341, 200}, {428, 200}, {334, 560}},
            {{335, 563}, {427, 200}, {522, 200}, {467, 564}},
            {{473, 567}, {529, 201}, {614, 201}, {607, 567}},
            {{609, 568}, {612, 198}, {705, 198}, {748, 568}},
            {{744, 573}, {703, 196}, {803, 196}, {900, 573}}
        };
        
        input_tray_rois_.clear();
        for (const auto& corners : row_corners) {
            input_tray_rois_.push_back(ROIQuad::FromCorners(corners));
        }
        RCLCPP_INFO(get_logger(), "[VISION] Initialized %zu input tray ROIs", input_tray_rois_.size());
    }

    void initOutputTrayROIs() {
        std::vector<std::vector<std::pair<int, int>>> slot_corners = {
            {{280, 505}, {280, 415}, {580, 430}, {584, 519}},
            {{275, 415}, {295, 319}, {579, 327}, {576, 424}},
            {{297, 315}, {317, 228}, {582, 229}, {580, 320}},
            {{349, 249}, {362, 172}, {619, 168}, {625, 249}},
            {{585, 527}, {590, 242}, {690, 240}, {690, 525}},
            {{699, 517}, {696, 423}, {1010, 415}, {1026, 502}},
            {{696, 420}, {700, 328}, {989, 328}, {1011, 405}},
            {{694, 323}, {696, 230}, {966, 230}, {985, 325}},
        };
        
        for (size_t i = 0; i < slot_corners.size() && i < output_tray_rois_.size(); ++i) {
            output_tray_rois_[i] = ROIQuad::FromCorners(slot_corners[i]);
        }
        
        slot_stable_state_.fill(SlotStableState::EMPTY);
        slot_empty_streak_.fill(0);
        slot_occ_streak_.fill(0);
        slot_mis_streak_.fill(0);
        
        RCLCPP_INFO(get_logger(), "[VISION] Initialized %zu output tray slots", slot_corners.size());
    }

    // ========================================================================
    // INPUT TRAY CALLBACK (from robot_logic_node.cpp)
    // ========================================================================
    void camera1Callback(const Detection2DArray::SharedPtr msg) {
        auto start = std::chrono::high_resolution_clock::now();
        
        // Skip processing in MANUAL mode
        if (current_mode_ == ControlMode::MANUAL) return;
        
        std::vector<int> row_counts(5, 0);
        
        // Count detections per ROI
        for (const auto& det : msg->detections) {
            if (det.results.empty()) continue;
            
            const std::string& class_id = det.results[0].hypothesis.class_id;
            float score = det.results[0].hypothesis.score;
            
            if (class_id != "0" || score < DETECTION_SCORE_THRESH) continue;
            
            float cx = det.bbox.center.position.x;
            float cy = det.bbox.center.position.y;
            
            for (size_t i = 0; i < input_tray_rois_.size(); ++i) {
                if (!input_tray_rois_[i].bbox_contains(cx, cy)) continue;
                
                if (input_tray_rois_[i].contains(cx, cy)) {
                    row_counts[i]++;
                    break;
                }
            }
        }
        
        // Update row status
        bool use_ai = (current_mode_ == ControlMode::AI);
        
        for (size_t i = 0; i < row_counts.size(); ++i) {
            if (!use_ai) {
                row_full_[i] = true;  // Auto Mode: Assume all rows full
            } else {
                int filtered_count = row_filters_[i].filter_count(row_counts[i]);
                bool raw_ready = (filtered_count >= INPUT_ROW_THRESHOLD);
                bool stable_ready = row_filters_[i].update_ready(raw_ready);
                row_full_[i] = stable_ready;
            }
        }
        
        input_tray_empty_ = std::none_of(row_full_.begin(), row_full_.end(),
                                          [](bool full) { return full; });
        
        // Find first available (full) row for picking
        selected_input_row_ = -1;
        for (size_t i = 0; i < row_full_.size(); ++i) {
            if (row_full_[i]) {
                selected_input_row_ = static_cast<int>(i + 1);  // 1-indexed
                break;
            }
        }
        
        // Publish results
        auto row_msg = std_msgs::msg::Int32();
        row_msg.data = selected_input_row_;
        pub_selected_row_->publish(row_msg);
        
        auto empty_msg = std_msgs::msg::Bool();
        empty_msg.data = input_tray_empty_;
        pub_input_empty_->publish(empty_msg);
        
        auto status_msg = std_msgs::msg::Int32MultiArray();
        status_msg.data.resize(5);
        for (size_t i = 0; i < 5; ++i) {
            status_msg.data[i] = row_full_[i] ? 1 : 0;
        }
        pub_row_status_->publish(status_msg);
        
        // Performance tracking
        auto end = std::chrono::high_resolution_clock::now();
        auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        callback_count_.fetch_add(1);
        total_callback_time_us_.fetch_add(duration_us);
        
        if (callback_count_ % 100 == 0) {
            uint64_t avg_us = total_callback_time_us_ / callback_count_;
            RCLCPP_INFO(get_logger(), "[PERF] Cam0 avg: %lu µs", avg_us);
        }
    }

    // ========================================================================
    // OUTPUT TRAY CALLBACK (from robot_logic_node.cpp)
    // ========================================================================
    void camera2Callback(const Detection2DArray::SharedPtr msg) {
        if (current_mode_ == ControlMode::MANUAL) return;
        
        // Step 1: Detect instant state
        std::array<SlotStableState, 9> instant_state;
        instant_state.fill(SlotStableState::EMPTY);
        
        for (const auto& det : msg->detections) {
            if (det.results.empty()) continue;
            
            const std::string& class_id = det.results[0].hypothesis.class_id;
            float score = det.results[0].hypothesis.score;
            
            int cid = -1;
            try { cid = std::stoi(class_id); } catch (...) { continue; }
            if (cid == 0) continue;  // Skip tray
            if (cid != 1 && cid != 2 && cid != 3) continue;
            if (score < DETECTION_SCORE_THRESH) continue;
            
            float cx = det.bbox.center.position.x;
            float cy = det.bbox.center.position.y;
            
            for (size_t i = 0; i < 8 && i < output_tray_rois_.size(); ++i) {
                if (!output_tray_rois_[i].bbox_contains(cx, cy)) continue;
                
                if (output_tray_rois_[i].contains(cx, cy)) {
                    instant_state[i] = SlotStableState::OCC_OK;
                    break;
                }
            }
        }
        
        // Auto Mode override
        if (current_mode_ == ControlMode::AUTO) {
            instant_state.fill(SlotStableState::EMPTY);
        }
        
        // Step 2: Update debouncing with mutex
        int local_selected_slot = -1;
        
        {
            std::lock_guard<std::mutex> lock(slot_detection_mutex_);
            
            for (size_t s = 0; s < 9; ++s) {
                if (instant_state[s] == SlotStableState::EMPTY) {
                    slot_empty_streak_[s]++;
                    slot_occ_streak_[s] = 0;
                    slot_mis_streak_[s] = 0;
                    if (slot_empty_streak_[s] >= SLOT_CONFIRM_FRAMES)
                        slot_stable_state_[s] = SlotStableState::EMPTY;
                } else if (instant_state[s] == SlotStableState::OCC_OK) {
                    slot_occ_streak_[s]++;
                    slot_empty_streak_[s] = 0;
                    slot_mis_streak_[s] = 0;
                    if (slot_occ_streak_[s] >= SLOT_CONFIRM_FRAMES)
                        slot_stable_state_[s] = SlotStableState::OCC_OK;
                } else {
                    slot_mis_streak_[s]++;
                    slot_empty_streak_[s] = 0;
                    slot_occ_streak_[s] = 0;
                    if (slot_mis_streak_[s] >= SLOT_CONFIRM_FRAMES)
                        slot_stable_state_[s] = SlotStableState::MIS;
                }
            }
            
            // Select first EMPTY slot (contiguous fill)
            for (size_t i = 0; i < 9; ++i) {
                if (slot_stable_state_[i] == SlotStableState::EMPTY) {
                    bool all_before_occupied = true;
                    for (size_t j = 0; j < i; ++j) {
                        if (slot_stable_state_[j] == SlotStableState::EMPTY) {
                            all_before_occupied = false;
                            break;
                        }
                    }
                    if (all_before_occupied) {
                        local_selected_slot = static_cast<int>(i + 1);
                        break;
                    }
                }
            }
        }
        
        selected_output_slot_ = local_selected_slot;
        
        // Publish results
        auto slot_msg = std_msgs::msg::Int32();
        slot_msg.data = local_selected_slot;
        pub_selected_slot_->publish(slot_msg);
        
        auto status_msg = std_msgs::msg::Int32MultiArray();
        status_msg.data.resize(9);
        for (size_t i = 0; i < 9; ++i) {
            status_msg.data[i] = (slot_stable_state_[i] == SlotStableState::EMPTY) ? 0 : 1;
        }
        pub_slot_status_->publish(status_msg);
    }
};

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionDecisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
