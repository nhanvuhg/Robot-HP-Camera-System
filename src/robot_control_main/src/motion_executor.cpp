/**
 * @file motion_executor.cpp
 * @brief Handles all Dobot motion commands and digital outputs
 * 
 * Responsibilities:
 * - Execute motion commands from state machine
 * - Call Dobot services (JointMovJ, RelMovL, DO, Sync)
 * - Report motion completion/failure
 * 
 * Topics Subscribed:
 * - /robot/motion_command (String) - Command format: "TYPE:PARAM" (e.g., "PICK_ROW:3")
 * 
 * Topics Published:
 * - /robot/motion_result (Bool) - True = success, False = failure
 * - /robot/motion_status (String) - Current motion status
 * - /robot/motion_busy (Bool) - Is motion in progress
 * - /robot/motion_heartbeat (Header) - Node aliveness with timestamp

 * - /robot/gripper_cmd (Bool) - Gripper state feedback
 * - /robot/picker_cmd (Bool) - Picker state feedback
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/header.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_control_interfaces/action/execute_motion.hpp"



// Dobot Messages
#include "dobot_msgs_v3/srv/enable_robot.hpp"
#include "dobot_msgs_v3/srv/get_pose.hpp"
#include "dobot_msgs_v3/srv/get_angle.hpp"
#include "dobot_msgs_v3/srv/joint_mov_j.hpp"
#include "dobot_msgs_v3/srv/mov_l.hpp"
#include "dobot_msgs_v3/srv/mov_j.hpp"
#include "dobot_msgs_v3/srv/rel_mov_l.hpp"
#include "dobot_msgs_v3/srv/rel_mov_l_user.hpp"
#include "dobot_msgs_v3/srv/do.hpp"
#include "dobot_msgs_v3/srv/robot_mode.hpp"
#include "dobot_msgs_v3/srv/speed_l.hpp"
#include "dobot_msgs_v3/srv/speed_factor.hpp"
#include "dobot_msgs_v3/srv/acc_l.hpp"
#include "dobot_msgs_v3/srv/speed_j.hpp"
#include "dobot_msgs_v3/srv/acc_j.hpp"
#include "dobot_msgs_v3/srv/sync.hpp"
#include "dobot_msgs_v3/srv/clear_error.hpp"
#include "dobot_msgs_v3/srv/get_error_id.hpp"

#include <vector>
#include <string>
#include <sstream>
#include <mutex>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <thread>

using namespace std::chrono_literals;

using EnableRobot = dobot_msgs_v3::srv::EnableRobot;
using GetPose = dobot_msgs_v3::srv::GetPose;
using GetAngle = dobot_msgs_v3::srv::GetAngle;
using JointMovJ = dobot_msgs_v3::srv::JointMovJ;
using MovL = dobot_msgs_v3::srv::MovL;
using MovJ = dobot_msgs_v3::srv::MovJ;
using RelMovL = dobot_msgs_v3::srv::RelMovL;
using RelMovLUser = dobot_msgs_v3::srv::RelMovLUser;
using DO = dobot_msgs_v3::srv::DO;
using RobotMode = dobot_msgs_v3::srv::RobotMode;
using SpeedL = dobot_msgs_v3::srv::SpeedL;
using SpeedFactor = dobot_msgs_v3::srv::SpeedFactor;
using AccL = dobot_msgs_v3::srv::AccL;
using SpeedJ = dobot_msgs_v3::srv::SpeedJ;
using AccJ = dobot_msgs_v3::srv::AccJ;
using SyncSrv = dobot_msgs_v3::srv::Sync;
using ClearError = dobot_msgs_v3::srv::ClearError;
using GetErrorID = dobot_msgs_v3::srv::GetErrorID;

// ============================================================================
// MOTION EXECUTOR NODE
// ============================================================================

class MotionExecutorNode : public rclcpp::Node {
public:
    MotionExecutorNode() : Node("motion_executor") {
        RCLCPP_INFO(get_logger(), "[MOTION] === Motion Executor Node Starting ===");
        
        loadMotionParameters();
        initServiceClients();
        
        // Publishers
        pub_busy_ = create_publisher<std_msgs::msg::Bool>("/robot/motion_busy", 10);

        pub_heartbeat_ = create_publisher<std_msgs::msg::Header>("/robot/motion_heartbeat", 10);
        pub_gripper_ = create_publisher<std_msgs::msg::Bool>("/robot/gripper_cmd", 10);
        pub_picker_ = create_publisher<std_msgs::msg::Bool>("/robot/picker_cmd", 10);

        
        // Subscriptions
        sub_gripper_status_ = create_subscription<std_msgs::msg::Bool>(
            "/robot/gripper_status", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                last_gripper_status_ = msg->data;
            });
            
        sub_picker_status_ = create_subscription<std_msgs::msg::Bool>(
            "/robot/picker_status", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                last_picker_status_ = msg->data;
            });

        speed_ratio_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/robot/speed_ratio", rclcpp::QoS(10).reliable().transient_local(),
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                current_speed_ratio_ = std::clamp(msg->data, 1, 100);
                RCLCPP_INFO(get_logger(), "[SPEED] Speed ratio updated: %d%% (GUI already sent SpeedFactor to hardware)", current_speed_ratio_);
            });

        // Heartbeat Timer (500ms)
        heartbeat_timer_ = create_wall_timer(500ms, [this]() {
            std_msgs::msg::Header h;
            h.stamp = this->now();
            h.frame_id = "motion_executor";
            pub_heartbeat_->publish(h);
        });

        // Action Server
        action_server_ = rclcpp_action::create_server<ExecuteMotion>(
            this,
            "/robot/execute_motion",
            std::bind(&MotionExecutorNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionExecutorNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&MotionExecutorNode::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "[MOTION] === Motion Executor Node Ready (Action Server Enabled) ===");
    }


private:
    // ========================================================================
    // MOTION DATA
    // ========================================================================
    std::vector<std::vector<double>> joint_sequences_;
    std::vector<std::vector<double>> relmovl_sequences_;
    std::vector<std::pair<int, int>> digital_output_steps_;
    
    std::vector<double> safe_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    std::atomic<bool> motion_in_progress_{false};
    // Abort flag — set bởi handle_cancel (STOP), check trong tất cả motion helpers
    // để thoát NGAY khi STOP thay vì chạy hết sequence.
    std::atomic<bool> abort_motion_{false};

    int current_fail_slot_{1};

    // Speed ratio from GUI (set via /robot/speed_ratio topic)
    int current_speed_ratio_{14};  // default matches GUI saved value

    // ========================================================================
    // ROS INTERFACES
    // ========================================================================    // ROS INTERFACES
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_busy_;

    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_heartbeat_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_gripper_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_picker_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_gripper_status_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_picker_status_;
    std::atomic<bool> last_gripper_status_{false};
    std::atomic<bool> last_picker_status_{false};

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr speed_ratio_sub_;
    
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    // Action Server Members
    using ExecuteMotion = robot_control_interfaces::action::ExecuteMotion;
    using GoalHandleExecuteMotion = rclcpp_action::ServerGoalHandle<ExecuteMotion>;
    rclcpp_action::Server<ExecuteMotion>::SharedPtr action_server_;

    
    // Service Clients
    rclcpp::Client<EnableRobot>::SharedPtr enable_client_;
    rclcpp::Client<ClearError>::SharedPtr clear_error_client_;
    rclcpp::Client<GetPose>::SharedPtr pose_client_;
    rclcpp::Client<GetAngle>::SharedPtr angle_client_;
    rclcpp::Client<JointMovJ>::SharedPtr joint_client_;
    rclcpp::Client<MovL>::SharedPtr movl_client_;
    rclcpp::Client<MovJ>::SharedPtr movj_client_;
    rclcpp::Client<RelMovL>::SharedPtr relmovl_client_;
    rclcpp::Client<RelMovLUser>::SharedPtr relmovluser_client_;
    rclcpp::Client<DO>::SharedPtr do_client_;
    rclcpp::Client<RobotMode>::SharedPtr robot_mode_client_;
    rclcpp::Client<SpeedL>::SharedPtr speedl_client_;
    rclcpp::Client<SpeedFactor>::SharedPtr speedfactor_client_;
    rclcpp::Client<AccL>::SharedPtr accl_client_;
    rclcpp::Client<SpeedJ>::SharedPtr speedj_client_;
    rclcpp::Client<AccJ>::SharedPtr accj_client_;
    rclcpp::Client<SyncSrv>::SharedPtr sync_client_;
    rclcpp::Client<GetErrorID>::SharedPtr error_client_;

    // ========================================================================
    // INITIALIZATION
    // ========================================================================
    void initServiceClients() {
        auto qos = rclcpp::ServicesQoS();
        
        enable_client_ = create_client<EnableRobot>("/nova5/dobot_bringup/EnableRobot", qos);
        clear_error_client_ = create_client<ClearError>("/nova5/dobot_bringup/ClearError", qos);
        pose_client_ = create_client<GetPose>("/nova5/dobot_bringup/GetPose", qos);
        angle_client_ = create_client<GetAngle>("/nova5/dobot_bringup/GetAngle", qos);
        joint_client_ = create_client<JointMovJ>("/nova5/dobot_bringup/JointMovJ", qos);
        movl_client_ = create_client<MovL>("/nova5/dobot_bringup/MovL", qos);
        movj_client_ = create_client<MovJ>("/nova5/dobot_bringup/MovJ", qos);
        relmovl_client_ = create_client<RelMovL>("/nova5/dobot_bringup/RelMovL", qos);
        relmovluser_client_ = create_client<RelMovLUser>("/nova5/dobot_bringup/RelMovLUser", qos);
        do_client_ = create_client<DO>("/nova5/dobot_bringup/DO", qos);
        robot_mode_client_ = create_client<RobotMode>("/nova5/dobot_bringup/RobotMode", qos);
        speedl_client_ = create_client<SpeedL>("/nova5/dobot_bringup/SpeedL", qos);
        speedfactor_client_ = create_client<SpeedFactor>("/nova5/dobot_bringup/SpeedFactor", qos);
        accl_client_ = create_client<AccL>("/nova5/dobot_bringup/AccL", qos);
        speedj_client_ = create_client<SpeedJ>("/nova5/dobot_bringup/SpeedJ", qos);
        accj_client_ = create_client<AccJ>("/nova5/dobot_bringup/AccJ", qos);
        sync_client_ = create_client<SyncSrv>("/nova5/dobot_bringup/Sync", qos);
        error_client_ = create_client<GetErrorID>("/nova5/dobot_bringup/GetErrorID", qos);
        
        RCLCPP_INFO(get_logger(), "[MOTION] Service clients initialized");
    }
    
    void loadMotionParameters() {
        this->declare_parameter("motion_sequence", std::vector<std::string>{});
        std::vector<std::string> seq_lines;
        this->get_parameter("motion_sequence", seq_lines);
        
        this->declare_parameter("safe_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        this->get_parameter("safe_pose", safe_pose_);


        joint_sequences_.clear();
        relmovl_sequences_.clear();
        digital_output_steps_.clear();

        for (const auto& line : seq_lines) {
            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;

            while (std::getline(ss, token, ',')) {
                tokens.push_back(token);
            }

            if (tokens.empty()) continue;

            const std::string& type = tokens[0];

            if (type == "J") {
                std::vector<double> joints;
                for (size_t i = 1; i < tokens.size(); ++i) {
                    try { 
                        joints.push_back(std::stod(tokens[i])); 
                    } catch (...) {}
                }
                if (joints.size() == 6) {
                    joint_sequences_.push_back(joints);
                }
            } else if (type == "R" && tokens.size() >= 4) {
                try {
                    relmovl_sequences_.emplace_back(std::vector<double>{
                        std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3])});
                } catch (...) {}
            } else if (type == "D" && tokens.size() >= 3) {
                try {
                    int do_index = std::stoi(tokens[1]);
                    int do_status = std::stoi(tokens[2]);
                    digital_output_steps_.emplace_back(do_index, do_status);
                } catch (...) {}
            }
        }

        RCLCPP_INFO(get_logger(), "[MOTION] Loaded: %zu joints, %zu relmovl, %zu DO",
                    joint_sequences_.size(), relmovl_sequences_.size(), digital_output_steps_.size());
    }

    // ========================================================================
    // SERVICE CALL HELPER
    // ========================================================================
    template <typename ServiceT>
    typename ServiceT::Response::SharedPtr callService(
        typename rclcpp::Client<ServiceT>::SharedPtr client,
        typename ServiceT::Request::SharedPtr request,
        const std::string& service_name) 
    {
        if (!client->wait_for_service(2s)) {
            RCLCPP_ERROR(get_logger(), "[SERVICE] %s not available", service_name.c_str());
            return nullptr;
        }

        auto future = client->async_send_request(request);
        
        if (future.wait_for(5s) == std::future_status::ready) {
            try {
                return future.get();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "[SERVICE] %s exception: %s", service_name.c_str(), e.what());
                return nullptr;
            }
        }
        
        RCLCPP_ERROR(get_logger(), "[SERVICE] %s timeout", service_name.c_str());
        return nullptr;
    }

    // ========================================================================
    // MOTION PRIMITIVES
    // ========================================================================
    bool moveToIndex(size_t index, int speed_override = -1) {
        if (shouldAbort()) { RCLCPP_WARN(get_logger(), "[moveToIndex] aborted"); return false; }
        if (index >= joint_sequences_.size()) {
            RCLCPP_ERROR(get_logger(), "[MOTION] Invalid index: %zu (max: %zu)", 
                         index, joint_sequences_.size() - 1);
            return false;
        }

        if (!prepareJointMotion(speed_override)) {
            RCLCPP_WARN(get_logger(), "[MOTION] Failed to prepare Joint Motion (Speed/Acc)");
        }

        auto req = std::make_shared<JointMovJ::Request>();
        const auto& joints = joint_sequences_[index];
        req->j1 = joints[0];
        req->j2 = joints[1];
        req->j3 = joints[2];
        req->j4 = joints[3];
        req->j5 = joints[4];
        req->j6 = joints[5];

        RCLCPP_INFO(get_logger(), "[MOTION] JointMovJ -> Index %zu", index);

        auto res = callService<JointMovJ>(joint_client_, req, "JointMovJ");
        if (!res) return false;
        if (res->res != 0) {
            RCLCPP_ERROR(get_logger(), "[MOTION] JointMovJ failed (err: %d)", res->res);
            return false;
        }

        return sync();
    }

    bool moveR(double dx, double dy, double dz, int speed_override = -1) {
        if (shouldAbort()) { RCLCPP_WARN(get_logger(), "[moveR] aborted"); return false; }
        if (!prepareLinearMotion(speed_override)) {
            RCLCPP_ERROR(get_logger(), "[moveR] Prepare failed");
            return false;
        }

        auto current_pose = getCurrentPose();
        if (current_pose.size() < 6) {
            RCLCPP_ERROR(get_logger(), "[moveR] No current pose");
            return false;
        }

        // Full pose log for debugging
        RCLCPP_INFO(get_logger(),
            "[moveR] Current: X=%.2f Y=%.2f Z=%.2f Rx=%.2f Ry=%.2f Rz=%.2f",
            current_pose[0], current_pose[1], current_pose[2],
            current_pose[3], current_pose[4], current_pose[5]);

        auto req = std::make_shared<MovL::Request>();
        req->x  = current_pose[0] + dx;
        req->y  = current_pose[1] + dy;
        req->z  = current_pose[2] + dz;
        req->rx = current_pose[3];
        req->ry = current_pose[4];
        req->rz = current_pose[5];
        req->param_value.clear();

        RCLCPP_INFO(get_logger(),
            "[moveR] Target:  X=%.2f Y=%.2f Z=%.2f Rx=%.2f Ry=%.2f Rz=%.2f",
            req->x, req->y, req->z, req->rx, req->ry, req->rz);

        auto res = callService<MovL>(movl_client_, req, "MovL");
        if (!res) {
            RCLCPP_ERROR(get_logger(), "[moveR] Service call returned nullptr (timeout/unavailable)");
            return false;
        }
        if (res->res != 0) {
            RCLCPP_ERROR(get_logger(), "[moveR] Rejected, code=%d", res->res);
            return false;
        }

        return sync();
    }


    bool moveJ_Absolute(const std::vector<double>& pose) {
        if (shouldAbort()) { RCLCPP_WARN(get_logger(), "[moveJ_Absolute] aborted"); return false; }
        if (pose.size() < 6) return false;
        
        if (!prepareJointMotion()) {
            RCLCPP_WARN(get_logger(), "[MOTION] Failed to prepare Joint Motion (Speed/Acc)");
        }

        auto req = std::make_shared<MovJ::Request>();
        req->x = pose[0];
        req->y = pose[1];
        req->z = pose[2];
        req->rx = pose[3];
        req->ry = pose[4];
        req->rz = pose[5];
        req->param_value.clear();
        
        auto res = callService<MovJ>(movj_client_, req, "MovJ");
        if (!res) return false;
        if (res->res != 0) {
            RCLCPP_ERROR(get_logger(), "[moveJ] Failed (err: %d)", res->res);
            return false;
        }

        return sync();
    }

    bool moveL_Absolute(const std::vector<double>& pose) {
        if (shouldAbort()) { RCLCPP_WARN(get_logger(), "[moveL_Absolute] aborted"); return false; }
        if (pose.size() < 6) return false;
        
        if (!prepareLinearMotion()) {
            RCLCPP_WARN(get_logger(), "[MOTION] Failed to prepare Linear Motion (Speed/Acc)");
        }

        auto req = std::make_shared<MovL::Request>();
        req->x = pose[0];
        req->y = pose[1];
        req->z = pose[2];
        req->rx = pose[3];
        req->ry = pose[4];
        req->rz = pose[5];
        req->param_value.clear();
        
        auto res = callService<MovL>(movl_client_, req, "MovL");
        if (!res) return false;
        if (res->res != 0) {
            RCLCPP_ERROR(get_logger(), "[moveL] Failed (err: %d)", res->res);
            return false;
        }

        return sync();
    }


    bool setDigitalOutput(int index, bool status) {
        if (shouldAbort()) { RCLCPP_WARN(get_logger(), "[setDigitalOutput] aborted"); return false; }
        // Publish to ROS topics first for the Festo Gripper Node
        if (index == 1 && pub_gripper_) {
            auto msg = std_msgs::msg::Bool();
            msg.data = status;
            pub_gripper_->publish(msg);
            
            // Wait for feedback from Python node
            auto start = std::chrono::steady_clock::now();
            while (last_gripper_status_ != status && rclcpp::ok()) {
                if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(2000)) {
                    RCLCPP_WARN(get_logger(), "[DO] Timeout waiting for Gripper status feedback!");
                    break;
                }
                rclcpp::sleep_for(std::chrono::milliseconds(10));
            }
        }
        if (index == 2 && pub_picker_) {
            auto msg = std_msgs::msg::Bool();
            msg.data = status;
            pub_picker_->publish(msg);
            
            // Wait for feedback from Python node
            auto start = std::chrono::steady_clock::now();
            while (last_picker_status_ != status && rclcpp::ok()) {
                if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(2000)) {
                    RCLCPP_WARN(get_logger(), "[DO] Timeout waiting for Picker status feedback!");
                    break;
                }
                rclcpp::sleep_for(std::chrono::milliseconds(10));
            }
        }

        // Try to trigger Dobot's hardware DO (optional, might fail if not configured)
        auto req = std::make_shared<DO::Request>();
        req->index = index;
        req->status = status ? 1 : 0;

        auto res = callService<DO>(do_client_, req, "DO");
        if (!res) {
            RCLCPP_WARN(get_logger(), "[DO] Hardware DO service call failed.");
        } else if (res->res != 0) {
            RCLCPP_WARN(get_logger(), "[DO] Hardware DO Failed (err: %d), but ROS topic published.", res->res);
        }

        // Always return true to keep the motion sequence running
        return true;
    }

    bool sync() {
        // Sync() returns -10000 on this firmware (command unsupported).
        // Instead, poll RobotMode until robot is no longer in motion (mode != 7).
        // Mode values: 5=standby, 7=running, 9=error
        if (!robot_mode_client_->service_is_ready()) {
            RCLCPP_WARN(get_logger(), "[SYNC] RobotMode service not ready, using sleep fallback");
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            return true;
        }

        // VITAL FIX: Give the Dobot controller time to transition into mode 7
        // After sending a move command, the controller is still in mode 5 for ~50-150ms 
        // before it officially starts trajectory execution.
        rclcpp::sleep_for(std::chrono::milliseconds(250));

        const int max_attempts = 600; // 60 seconds max
        for (int i = 0; i < max_attempts; ++i) {
            // Thoát ngay nếu STOP/cancel — không chờ thêm
            if (shouldAbort()) {
                RCLCPP_WARN(get_logger(), "[SYNC] aborted (STOP) — return false");
                return false;
            }
            auto req = std::make_shared<RobotMode::Request>();
            auto future = robot_mode_client_->async_send_request(req);
            if (future.wait_for(1s) != std::future_status::ready) {
                RCLCPP_WARN(get_logger(), "[SYNC] RobotMode request timed out, retrying...");
                continue;
            }
            try {
                auto res = future.get();
                if (res && res->res == 0) {
                    int mode = std::stoi(res->mode);
                    if (mode == 5) {
                        RCLCPP_DEBUG(get_logger(), "[SYNC] Robot idle (mode=5) after %d polls - SUCCESS", i);
                        return true;
                    } else if (mode != 7) {
                        RCLCPP_ERROR(get_logger(), "[SYNC] Motion INTERRUPTED! Mode=%d (not 7 or 5)", mode);
                        return false;
                    }
                }
            } catch (...) {}
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        RCLCPP_ERROR(get_logger(), "[SYNC] Timeout waiting for motion to complete! Robot still in mode 7.");
        return false;
    }

    std::vector<double> getCurrentPose() {
        auto req = std::make_shared<GetPose::Request>();
        req->user = 0;
        req->tool = 0;  // tool=0 matches MovL/MovJ base frame
        
        auto res = callService<GetPose>(pose_client_, req, "GetPose");
        
        if (!res) {
            RCLCPP_ERROR(get_logger(), "[getCurrentPose] Service call failed");
            return {};
        }
        
        std::string pose_str = res->pose;
        pose_str.erase(std::remove(pose_str.begin(), pose_str.end(), '{'), pose_str.end());
        pose_str.erase(std::remove(pose_str.begin(), pose_str.end(), '}'), pose_str.end());
        
        std::vector<double> pose;
        std::stringstream ss(pose_str);
        std::string item;
        
        while (std::getline(ss, item, ',')) {
            try {
                pose.push_back(std::stod(item));
            } catch (...) {}
        }
        
        if (pose.size() < 6) {
            RCLCPP_ERROR(get_logger(), "[getCurrentPose] Invalid pose size: %zu", pose.size());
            return {};
        }

        return pose;
    }

    // Wait (delay) — chèn vào giữa các motion command, vd:
    //   moveToIndex(5);
    //   wait(2.0);            // chờ 2 giây
    //   moveR(0, 0, -30);
    // Interruptible: thoát sớm khi rclcpp shutdown. Returns false nếu bị shutdown.
    bool wait(double seconds) {
        if (seconds <= 0.0) return true;
        if (shouldAbort()) { RCLCPP_WARN(get_logger(), "[wait] aborted at start"); return false; }
        RCLCPP_INFO(get_logger(), "[MOTION] wait %.2fs", seconds);
        auto deadline = std::chrono::steady_clock::now()
                      + std::chrono::milliseconds(static_cast<int>(seconds * 1000));
        while (std::chrono::steady_clock::now() < deadline) {
            if (shouldAbort()) {
                RCLCPP_WARN(get_logger(), "[wait] aborted mid-sleep");
                return false;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
        return true;
    }


    bool prepareLinearMotion(int speed_override = -1) {
        int spd = (speed_override > 0) ? speed_override : current_speed_ratio_;  // Use system or override speed
        RCLCPP_INFO(get_logger(), "[MOTION] prepareLinearMotion speed=%d%%", spd);
        // Set SpeedL
        auto speed_req = std::make_shared<SpeedL::Request>();
        speed_req->r = spd;
        if (!callService<SpeedL>(speedl_client_, speed_req, "SpeedL")) return false;

        // Set AccL
        auto acc_req = std::make_shared<AccL::Request>();
        acc_req->r = spd;
        if (!callService<AccL>(accl_client_, acc_req, "AccL")) return false;
        
        return true;
    }

    bool prepareJointMotion(int speed_override = -1) {
        int spd = (speed_override > 0) ? speed_override : current_speed_ratio_;  // Use system or override speed
        RCLCPP_INFO(get_logger(), "[MOTION] prepareJointMotion speed=%d%%", spd);
        // Set SpeedJ
        auto speed_req = std::make_shared<SpeedJ::Request>();
        speed_req->r = spd;
        if (!callService<SpeedJ>(speedj_client_, speed_req, "SpeedJ")) return false;

        // Set AccJ
        auto acc_req = std::make_shared<AccJ::Request>();
        acc_req->r = spd;
        if (!callService<AccJ>(accj_client_, acc_req, "AccJ")) return false;
        
        return true;
    }

    // RAII Guard for motion_busy
    struct MotionBusyGuard {
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub;
        MotionBusyGuard(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr p) : pub(p) {
            std_msgs::msg::Bool msg; msg.data = true; pub->publish(msg);
        }
        ~MotionBusyGuard() {
            std_msgs::msg::Bool msg; msg.data = false; pub->publish(msg);
        }
    };

    // ========================================================================
    // COMMAND HANDLER

    // ========================================================================



    // ========================================================================
    // ACTION SERVER CALLBACKS
    // ========================================================================
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ExecuteMotion::Goal> goal)
    {
        (void)uuid;
        // CAS-claim cờ ngay tại handle_goal — chặn TOCTOU giữa 2 goal đến song song.
        // executeAction sẽ KHÔNG set lại motion_in_progress_, chỉ clear khi xong.
        bool expected = false;
        if (!motion_in_progress_.compare_exchange_strong(expected, true)) {
            RCLCPP_WARN(get_logger(), "[ACTION] Rejecting goal: motion in progress");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(get_logger(), "[ACTION] Received goal: %s (slot: %d)",
            goal->command.c_str(), goal->slot);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_WARN(get_logger(), "[ACTION] Received request to cancel goal — set abort flag");
        // Set abort flag NGAY — motion helpers đang chạy sẽ thoát ở check tiếp theo.
        // KHÔNG gọi rollbackSafe() từ đây: handle_cancel chạy trên executor thread,
        // còn motion đang chạy trên thread riêng (handle_accepted detach). Gọi rollback
        // ở đây sẽ chạy moveL song song với motion đang dở → Dobot reject hoặc race.
        // Rollback được executeAction tự gọi ở fail path khi nó thoát do abort.
        abort_motion_.store(true);
        return rclcpp_action::CancelResponse::ACCEPT;
    }


    void handle_accepted(const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
    {
        // Execute in separate thread
        std::thread{std::bind(&MotionExecutorNode::executeAction, this, std::placeholders::_1), goal_handle}.detach();
    }

    // Helper: motion helpers gọi để check có cần thoát ngay không.
    // True = ROS shutdown HOẶC handle_cancel đã set abort_motion_ (STOP).
    bool shouldAbort() const {
        return abort_motion_.load() || !rclcpp::ok();
    }

    void executeAction(const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
    {
    // Wrap toàn bộ thân trong try/catch: thread này detach, nếu exception unwinds
    // mà không reset motion_in_progress_/publishBusy thì node treo busy vĩnh viễn.
    try {
    RCLCPP_INFO(get_logger(), "[ACTION] Executing motion goal...");
    // Reset abort flag mỗi lần goal mới — flag chỉ giữ trong scope của 1 goal.
    abort_motion_.store(false);

    // ✅ Chỉ clear error khi robot đang ở mode lỗi (mode 9)
    {
        auto mode_req = std::make_shared<RobotMode::Request>();
        auto mode_future = robot_mode_client_->async_send_request(mode_req);
        
        if (mode_future.wait_for(2s) == std::future_status::ready) {
            try {
                auto mode_res = mode_future.get();
                int mode = (mode_res && mode_res->res == 0) ? std::stoi(mode_res->mode) : -1;
                
                if (mode == 9) {
                    RCLCPP_WARN(get_logger(), "[ACTION] Robot in error mode (9) — clearing...");
                    
                    auto clr = std::make_shared<ClearError::Request>();
                    callService<ClearError>(clear_error_client_, clr, "ClearError");
                    std::this_thread::sleep_for(200ms);
                    
                    auto en = std::make_shared<EnableRobot::Request>();
                    en->load = 0.0;
                    callService<EnableRobot>(enable_client_, en, "EnableRobot");
                    std::this_thread::sleep_for(300ms);
                    
                    RCLCPP_INFO(get_logger(), "[ACTION] Robot re-enabled after error clear");
                }
                // mode 5 = standby, mode 7 = running — không cần làm gì
            } catch (...) {
                RCLCPP_WARN(get_logger(), "[ACTION] Failed to parse robot mode — proceeding anyway");
            }
        } else {
            RCLCPP_WARN(get_logger(), "[ACTION] RobotMode check timeout — proceeding anyway");
        }
    }

    auto feedback = std::make_shared<ExecuteMotion::Feedback>();
    auto result = std::make_shared<ExecuteMotion::Result>();
    const auto goal = goal_handle->get_goal();

    // motion_in_progress_ đã được claim atomic ở handle_goal — không set lại.
    publishBusy(true);

    feedback->state = "RUNNING";
    feedback->progress = 0.1f;
    goal_handle->publish_feedback(feedback);

    bool success = false;
    std::string type = goal->command;
    int param = goal->slot;

    if (type == "INPUT_TRAY_CHAMBER")  success = executeInputTrayChamber(param);
    else if (type == "INPUT_TRAY_BUFFER") success = executeInputTrayBuffer(param);
    else if (type == "CHAMBER_SCALE")  success = executeChamberScale();
    else if (type == "SCALE_OUTPUT")   success = executeScaleOutput(param);
    else if (type == "SCALE_FAIL")     success = executeScaleFail();
    else if (type == "BUFFER_CHAMBER") success = executeBufferChamber();
    else if (type == "HOME")           success = moveToIndex(0);
    else RCLCPP_ERROR(get_logger(), "[ACTION] Unknown command: %s", type.c_str());

    if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "CANCELLED";
        goal_handle->canceled(result);
        publishBusy(false);
        motion_in_progress_ = false;
        return;
    }

    result->success = success;

    if (success) {
        result->message = "COMPLETED";
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "[ACTION] Goal succeeded");
    } else {
        result->message = "FAILED";
        if (goal->allow_rollback) {
            feedback->state = "ROLLBACK";
            goal_handle->publish_feedback(feedback);
            rollbackSafe();
            result->message = "ROLLED_BACK";
        }
        goal_handle->abort(result);
        RCLCPP_ERROR(get_logger(), "[ACTION] Goal aborted/failed");
    }

    publishBusy(false);
    motion_in_progress_ = false;
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "[ACTION] Exception in executeAction: %s", e.what());
        try {
            auto r = std::make_shared<ExecuteMotion::Result>();
            r->success = false;
            r->message = std::string("EXCEPTION: ") + e.what();
            if (goal_handle && goal_handle->is_active()) goal_handle->abort(r);
        } catch (...) {}
        publishBusy(false);
        motion_in_progress_ = false;
    }
    catch (...) {
        RCLCPP_ERROR(get_logger(), "[ACTION] Unknown exception in executeAction");
        try {
            auto r = std::make_shared<ExecuteMotion::Result>();
            r->success = false;
            r->message = "UNKNOWN_EXCEPTION";
            if (goal_handle && goal_handle->is_active()) goal_handle->abort(r);
        } catch (...) {}
        publishBusy(false);
        motion_in_progress_ = false;
    }
    }

    void rollbackSafe()
    {
        RCLCPP_ERROR(get_logger(), "[ROLLBACK] Executing SAFE rollback");
        
        // 1. Move to Safe Coordinates (Absolute)
        if (safe_pose_.size() == 6 && (std::abs(safe_pose_[0]) > 0.1 || std::abs(safe_pose_[1]) > 0.1 || std::abs(safe_pose_[2]) > 0.1)) {
            RCLCPP_INFO(get_logger(), "[ROLLBACK] Moving to safe coordinates: %.1f, %.1f, %.1f", 
                        safe_pose_[0], safe_pose_[1], safe_pose_[2]);
            moveL_Absolute(safe_pose_);
        } else {
            RCLCPP_WARN(get_logger(), "[ROLLBACK] Safe pose not set, moving to HOME index 0");
            moveToIndex(0);
        }

        // 2. KHÔNG reset gripper/picker khi rollback — giữ nguyên trạng thái hiện tại
        //    (operator nhấn STOP không được làm rơi khay đang kẹp).
        //    Nếu cần release sau STOP, operator chọn NHẢ trên GUI thủ công.

        RCLCPP_WARN(get_logger(), "[ROLLBACK] Completed (gripper/picker giữ nguyên state)");
    }


    void publishBusy(bool busy)
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = busy;
        pub_busy_->publish(msg);
    }


    // Hàm di chuyển theo trục chuẩn của mặt bàn (Base/User 0)
    bool moveBase(double dx, double dy, double dz) {
        if (shouldAbort()) { RCLCPP_WARN(get_logger(), "[moveBase] aborted"); return false; }
        if (!prepareLinearMotion()) return false;
        auto current_pose = getCurrentPose();
        if (current_pose.size() < 6) return false;
        
        auto req = std::make_shared<MovL::Request>();
        req->x  = current_pose[0] + dx;
        req->y  = current_pose[1] + dy;
        req->z  = current_pose[2] + dz;
        req->rx = current_pose[3];
        req->ry = current_pose[4];
        req->rz = current_pose[5];
        req->param_value.clear();

        auto res = callService<MovL>(movl_client_, req, "MovL");
        if (!res || res->res != 0) return false;
        return sync();
    }

    bool executeInputTrayChamber(int row) {
        RCLCPP_INFO(get_logger(), "[MOTION] Input Tray Row %d → Chamber", row);
        if (row < 1 || row > 5) return false;
        if (!moveToIndex(0)) return false;
        // MoveJ đến row1new1 (Index 1)
        if (!moveToIndex(6)) return false;
        if (!moveToIndex(1)) return false;
        
        // Tính tiến theo row index DỰA TRÊN TRỤC CỦA TAY MÁY (Khay đặt theo góc của tay)
        if (row > 1) {
            double dx = (row - 1) * (-104.75); // Đi dọc theo khay (hướng đâm thẳng của tay)
            double dy = (row - 1) * -8.8;      // Đi ngang khay (hướng vuông góc với tay)
            double dz = (row - 1) * 1.0;
            if (!moveR(dx, dy, dz)) return false;
        }
        
        // --- INPUT ROW → CHAMBER: Picker bốc khay từ input stack rồi nhả vào chamber ---
        if (!moveR(0, 0, -110,5)) return false;
        if (!setDigitalOutput(1, true)) return false;   // Picker GẮP — kẹp khay tại input row
        if (!wait(1.0)) return false;
        if (!moveR(0, 0, 120,5)) return false;
        if (!moveToIndex(27)) return false;
        if (!moveToIndex(28)) return false;
        if (!moveToIndex(7)) return false;
        if (!wait(1.0)) return false;
        if (!moveR(0, 139, 0,3)) return false;
        if (!setDigitalOutput(1, false)) return false;  // Picker NHẢ — thả khay vào chamber
        if (!wait(2.0)) return false;
        if (!moveR(1, -60, 0)) return false;
        if (!moveR(-10, 29, 0)) return false;
        if (!wait(0.5)) return false;
        if (!moveR(0, -230, 0)) return false;
        if (!moveToIndex(28)) return false;
        return true;
    }

    bool executeInputTrayBuffer(int row) {
        RCLCPP_INFO(get_logger(), "[MOTION] Input Tray Row %d → Buffer", row);
        if (row < 1 || row > 5) return false;
        // if (!moveToIndex(6)) return false; // Tạm thời bỏ move đến vị trí an toàn
        if (!moveToIndex(27)) return false;
        // MoveJ đến row1new1 (Index 1)
        if (!moveToIndex(1)) return false;
        
        // Tính tiến theo row index DỰA TRÊN TRỤC CỦA TAY MÁY (Khay đặt theo góc của tay)
        if (row > 1) {
            double dx = (row - 1) * (-104.75); // Đi dọc theo khay (hướng đâm thẳng của tay)
            double dy = (row - 1) * -8.8;      // Đi ngang khay (hướng vuông góc với tay)
            double dz = (row - 1) * 1.0;
            if (!moveR(dx, dy, dz)) return false;
        }
        
        // --- INPUT ROW → BUFFER: Picker bốc cart từ input stack rồi nhả vào buffer ---
        if (!moveR(0, 0, -110,5)) return false;
        if (!setDigitalOutput(1, true)) return false;   // Picker GẮP — kẹp khay tại input row
        if (!wait(1.0)) return false;
        if (!moveR(0, 0, 120,5)) return false;
        if (!moveToIndex(27)) return false;
        if (!moveToIndex(8)) return false;
        if (!moveR(0, 0, -55)) return false;
        if (!wait(1.0)) return false;
        if (!setDigitalOutput(1, false)) return false;  // Picker NHẢ — thả khay vào buffer
        if (!wait(1.0)) return false;
        if (!moveR(0, 0, 70)) return false;
        if (!moveToIndex(0)) return false;
        return true;
    }

    bool executeChamberScale() {
        RCLCPP_INFO(get_logger(), "[MOTION] Chamber → Scale");
        if (!moveToIndex(7)) return false;
        if (!moveR(0, 140.5, 0)) return false;
        if (!setDigitalOutput(1, true)) return false;   // Picker GẮP — kẹp khay tại chamber
        if (!wait(2.0)) return false;
        if (!moveR(0, -170, 0)) return false;
        if (!moveToIndex(9)) return false;
        if (!moveR(0, 0, -60)) return false;
        if (!setDigitalOutput(1, false)) return false;  // Picker NHẢ — thả khay lên scale
        if (!wait(1.0)) return false;
        if (!moveR(0, 0, 110)) return false;
        return true;
    }

    bool executeScaleOutput(int slot) {
        RCLCPP_INFO(get_logger(), "[MOTION] Scale → Output Slot %d", slot);
        if (slot < 1 || slot > 9) return false;
        if (!moveToIndex(30)) return false;
        if (!moveToIndex(10)) return false;
        if (!moveR(40, 0, 0)) return false;
        if (!setDigitalOutput(1, true)) return false;   // Picker GẮP — kẹp khay đang ở scale
        if (!moveR(0, 0, 40,5)) return false;
        if (!moveToIndex(12)) return false;
        if (!moveR(0, 0, -60,5)) return false;
        if (!wait(2.0)) return false;
        if (!setDigitalOutput(1, false)) return false;  // Picker NHẢ — đặt khay xuống vị trí trung gian
        if (!wait(1.0)) return false;
        if (!setDigitalOutput(2, false)) return false;  // Gripper NHẢ — đảm bảo gripper mở trước khi pick up
        if (!moveR(0, 0, 60,5)) return false;
        if (!moveToIndex(11)) return false;
        if (!moveR(0, 0, -60,5)) return false;
        if (!setDigitalOutput(2, true)) return false;   // Gripper GẮP — kẹp khay tại Index 11
        if (!wait(1.0)) return false;
        if (!moveR(0, 0, 60,5)) return false;
        if (!moveToIndex(13)) return false;
        if (!moveToIndex(13 + slot)) return false;
        if (!moveR(0, 0, -30,5)) return false;
        if (!setDigitalOutput(2, false)) return false;  // Gripper NHẢ — thả khay vào output slot
        if (!moveR(0, 0, 30,5)) return false;
        if (!moveToIndex(13)) return false;
        if (!moveToIndex(0)) return false;
        return true;
    }

    bool executeScaleFail() {
        RCLCPP_INFO(get_logger(), "[MOTION] Scale → Fail Position %d", current_fail_slot_);
        if (!moveToIndex(23 + current_fail_slot_)) return false;
        current_fail_slot_++;
        if (current_fail_slot_ > 4) current_fail_slot_ = 1;
        if (!moveR(0, 0, -30)) return false;
        if (!setDigitalOutput(1, false)) return false;  // Picker NHẢ — thả khay fail vào ngăn loại
        if (!moveR(0, 0, 30)) return false;
        if (!moveToIndex(0)) return false;
        return true;
    }

    bool executeBufferChamber() {
        RCLCPP_INFO(get_logger(), "[MOTION] Buffer → Chamber");
        if (!moveToIndex(28)) return false; 
        if (!moveToIndex(8)) return false;
        if (!moveR(0, 0, -58)) return false;
        if (!setDigitalOutput(1, true)) return false;   // Picker GẮP — kẹp khay tại buffer
        if (!moveR(0, 0, 90)) return false;
        if (!moveToIndex(28)) return false;
        if (!moveToIndex(7)) return false;
        if (!wait(1.0)) return false;
        if (!moveR(0, 139, 0,3)) return false;
        if (!setDigitalOutput(1, false)) return false;  // Picker NHẢ — thả khay vào chamber
        if (!wait(2.0)) return false;
        if (!moveR(-1, -60, 0)) return false;
        if (!moveR(-10, 29, 0)) return false;
        if (!wait(0.5)) return false;
        if (!moveR(0, -230, 0)) return false;
        if (!moveToIndex(28)) return false;
        return true;
    }
};

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionExecutorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
