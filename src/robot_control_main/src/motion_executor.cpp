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
    bool moveToIndex(size_t index) {
        if (index >= joint_sequences_.size()) {
            RCLCPP_ERROR(get_logger(), "[MOTION] Invalid index: %zu (max: %zu)", 
                         index, joint_sequences_.size() - 1);
            return false;
        }

        if (!prepareJointMotion()) {
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

    bool moveR(double dx, double dy, double dz) {
        if (!prepareLinearMotion()) {
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
            RCLCPP_ERROR(get_logger(), "[moveR] MovL failed (err: %d)", res->res);
            return false;
        }

        return sync();
    }


    bool moveJ_Absolute(const std::vector<double>& pose) {
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
        auto req = std::make_shared<DO::Request>();
        req->index = index;
        req->status = status ? 1 : 0;

        auto res = callService<DO>(do_client_, req, "DO");
        if (!res) return false;
        if (res->res != 0) {
            RCLCPP_ERROR(get_logger(), "[DO] Failed (err: %d)", res->res);
            return false;
        }

        // Publish feedback
        if (index == 1 && pub_gripper_) {
            auto msg = std_msgs::msg::Bool();
            msg.data = status;
            pub_gripper_->publish(msg);
        }
        if (index == 2 && pub_picker_) {
            auto msg = std_msgs::msg::Bool();
            msg.data = status;
            pub_picker_->publish(msg);
        }

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

        const int max_attempts = 100; // 10 seconds max
        for (int i = 0; i < max_attempts; ++i) {
            auto req = std::make_shared<RobotMode::Request>();
            auto future = robot_mode_client_->async_send_request(req);
            if (future.wait_for(1s) != std::future_status::ready) {
                RCLCPP_WARN(get_logger(), "[SYNC] RobotMode timeout");
                break;
            }
            try {
                auto res = future.get();
                if (res && res->res == 0) {
                    // mode field is a string like "5" or "7"
                    int mode = std::stoi(res->mode);
                    if (mode != 7) { // 7 = in motion
                        RCLCPP_DEBUG(get_logger(), "[SYNC] Robot idle (mode=%d) after %d polls", mode, i);
                        return true;
                    }
                }
            } catch (...) {}
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        RCLCPP_WARN(get_logger(), "[SYNC] Max polls reached, proceeding anyway");
        return true;
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

    bool prepareLinearMotion() {
        int spd = 100;  // Always 100% — actual speed is controlled by SpeedFactor (global)
        RCLCPP_INFO(get_logger(), "[MOTION] prepareLinearMotion speed=100%% (SpeedFactor=%d%%)", current_speed_ratio_);
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

    bool prepareJointMotion() {
        int spd = 100;  // Always 100% — actual speed is controlled by SpeedFactor (global)
        RCLCPP_INFO(get_logger(), "[MOTION] prepareJointMotion speed=100%% (SpeedFactor=%d%%)", current_speed_ratio_);
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
        if (motion_in_progress_) {
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
        RCLCPP_WARN(get_logger(), "[ACTION] Received request to cancel goal");
        
        // Safety: If rollback is allowed, execute it
        if (goal_handle->get_goal()->allow_rollback) {
            rollbackSafe();
        }

        return rclcpp_action::CancelResponse::ACCEPT;
    }


    void handle_accepted(const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
    {
        // Execute in separate thread
        std::thread{std::bind(&MotionExecutorNode::executeAction, this, std::placeholders::_1), goal_handle}.detach();
    }

    void executeAction(const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
    {
    RCLCPP_INFO(get_logger(), "[ACTION] Executing motion goal...");

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

    motion_in_progress_ = true;
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

        // 2. Release Gripper/Picker
        setDigitalOutput(1, false); // Gripper off
        setDigitalOutput(2, false); // Picker off
        
        RCLCPP_WARN(get_logger(), "[ROLLBACK] Completed");
    }


    void publishBusy(bool busy)
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = busy;
        pub_busy_->publish(msg);
    }


    bool executeInputTrayChamber(int row) {
        RCLCPP_INFO(get_logger(), "[MOTION] Input Tray Row %d → Chamber", row);
        if (row < 1 || row > 5) return false;
        // if (!moveToIndex(6)) return false; // Tạm thời bỏ move đến vị trí an toàn nếu bạn muốn đi thẳng đến row1new1
        
        // MoveJ đến row1new1 (Index 1)
        if (!moveToIndex(1)) return false;
        
        // Tính tiến theo row index
        if (row > 1) {
            double dx = (row - 1) * (-105.0);
            double dy = (row - 1) * 9.0;
            double dz = (row - 1) * 1.0;
            if (!moveR(dx, dy, dz)) return false;
        }
        
        // --- CÁC LỆNH MOVE TIẾP THEO BẠN CÓ THỂ TỰ THÊM HOẶC CHỈNH SỬA Ở ĐÂY ---
        if (!moveR(0, 0, -101)) return false;
        if (!setDigitalOutput(1, true)) return false;
        if (!moveR(0, 0, 101)) return false;
        if (!moveToIndex(7)) return false;
        if (!moveR(0, 30, 0)) return false;
        if (!setDigitalOutput(1, false)) return false;
        if (!moveR(0, -30, 0)) return false;
        return true;
    }

    bool executeInputTrayBuffer(int row) {
        RCLCPP_INFO(get_logger(), "[MOTION] Input Tray Row %d → Buffer", row);
        if (row < 1 || row > 5) return false;
        // if (!moveToIndex(6)) return false; // Tạm thời bỏ move đến vị trí an toàn
        
        // MoveJ đến row1new1 (Index 1)
        if (!moveToIndex(1)) return false;
        
        // Tính tiến theo row index
        if (row > 1) {
            double dx = (row - 1) * (-105.0);
            double dy = (row - 1) * 9.0;
            double dz = (row - 1) * 1.0;
            if (!moveR(dx, dy, dz)) return false;
        }
        
        // --- CÁC LỆNH MOVE TIẾP THEO BẠN CÓ THỂ TỰ THÊM HOẶC CHỈNH SỬA Ở ĐÂY ---
        if (!moveR(0, 0, -101)) return false;
        if (!setDigitalOutput(1, true)) return false;
        if (!moveR(0, 0, 101)) return false;
        if (!moveToIndex(8)) return false;
        if (!moveR(0, 0, -30)) return false;
        if (!setDigitalOutput(1, false)) return false;
        if (!moveR(0, 0, 30)) return false;
        if (!moveToIndex(0)) return false;
        return true;
    }

    bool executeChamberScale() {
        RCLCPP_INFO(get_logger(), "[MOTION] Chamber → Scale");
        if (!moveToIndex(7)) return false;
        if (!moveR(0, 30, 0)) return false;
        if (!moveR(0, -30, 0)) return false;
        if (!moveToIndex(9)) return false;
        if (!moveToIndex(10)) return false;
        if (!moveR(0, 0, -30)) return false;
        if (!setDigitalOutput(1, false)) return false;
        if (!moveR(0, 0, 30)) return false;
        return true;
    }

    bool executeScaleOutput(int slot) {
        RCLCPP_INFO(get_logger(), "[MOTION] Scale → Output Slot %d", slot);
        if (slot < 1 || slot > 9) return false;
        if (!moveToIndex(11)) return false;
        if (!moveR(0, 0, -30)) return false;
        if (!setDigitalOutput(2, true)) return false;
        if (!moveR(0, 0, 30)) return false;
        if (!moveToIndex(13)) return false;
        if (!moveToIndex(13 + slot)) return false;
        if (!moveR(0, 0, -30)) return false;
        if (!setDigitalOutput(2, false)) return false;
        if (!moveR(0, 0, 30)) return false;
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
        if (!setDigitalOutput(2, false)) return false;
        if (!moveR(0, 0, 30)) return false;
        if (!moveToIndex(0)) return false;
        return true;
    }

    bool executeBufferChamber() {
        RCLCPP_INFO(get_logger(), "[MOTION] Buffer → Chamber");
        if (!moveToIndex(8)) return false;
        if (!moveR(0, 0, -30)) return false;
        if (!setDigitalOutput(2, true)) return false;
        if (!moveR(0, 0, 30)) return false;
        if (!moveToIndex(7)) return false;
        if (!moveR(0, 30, 0)) return false;
        if (!setDigitalOutput(2, false)) return false;
        if (!moveR(0, -30, 0)) return false;
        if (!moveToIndex(0)) return false;
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
