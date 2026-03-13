#include "unified_control_gui/robot_controller.hpp"
#include <QDebug>
#include <sstream>

RobotController::RobotController(rclcpp::Node::SharedPtr node, QObject *parent)
    : QObject(parent)
    , node_(node)
{
    // Init joint angles to 6 zeros
    for (int i = 0; i < 6; i++) joint_angles_.append(0.0);
    for (int i = 0; i < 6; i++) cartesian_pose_.append(0.0);

    // Create service clients
    enable_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/enable_system");
    emergency_stop_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/emergency_stop");
    manual_mode_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/set_manual_mode");
    ai_mode_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/set_ai_mode");
    
    // Dobot jog service clients
    jog_client_ = node_->create_client<dobot_msgs_v3::srv::MoveJog>("/nova5/dobot_bringup/MoveJog");
    get_angle_client_ = node_->create_client<dobot_msgs_v3::srv::GetAngle>("/nova5/dobot_bringup/GetAngle");
    get_pose_client_ = node_->create_client<dobot_msgs_v3::srv::GetPose>("/nova5/dobot_bringup/GetPose");
    joint_movj_client_ = node_->create_client<dobot_msgs_v3::srv::JointMovJ>("/nova5/dobot_bringup/JointMovJ");
    movl_client_ = node_->create_client<dobot_msgs_v3::srv::MovL>("/nova5/dobot_bringup/MovL");
    do_client_ = node_->create_client<dobot_msgs_v3::srv::DO>("/nova5/dobot_bringup/DO");
    pause_client_ = node_->create_client<dobot_msgs_v3::srv::Pause>("/nova5/dobot_bringup/Pause");
    clear_error_client_ = node_->create_client<dobot_msgs_v3::srv::ClearError>("/nova5/dobot_bringup/ClearError");
    reset_robot_client_ = node_->create_client<dobot_msgs_v3::srv::ResetRobot>("/nova5/dobot_bringup/ResetRobot");
    speed_factor_client_ = node_->create_client<dobot_msgs_v3::srv::SpeedFactor>("/nova5/dobot_bringup/SpeedFactor");
    get_error_id_client_ = node_->create_client<dobot_msgs_v3::srv::GetErrorID>("/nova5/dobot_bringup/GetErrorID");

    // Create publishers
    camera_select_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robot/command_camera", 10);
    command_row_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robot/command_row", 10);
    command_slot_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robot/command_slot", 10);
    goto_state_pub_ = node_->create_publisher<std_msgs::msg::String>("/robot/goto_state", 10);
    feed_chamber_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/revpi/feed_chamber", 10);
    fill_done_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/fill_machine/fill_done", 10);
    
    // Create subscribers
    system_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot/system_status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            system_status_ = QString::fromStdString(msg->data);
            emit systemStatusChanged();
        });
    
    error_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot/error", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            error_message_ = QString::fromStdString(msg->data);
            emit errorMessageChanged();
        });
    
    selected_row_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/robot/selected_input_row", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            selected_row_ = msg->data;
            emit selectedRowChanged();
        });
    
    selected_slot_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/robot/selected_output_slot", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            selected_slot_ = msg->data;
            emit selectedSlotChanged();
        });
    
    system_uptime_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot/system_uptime", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            system_uptime_ = QString::fromStdString(msg->data);
            emit systemUptimeChanged();
        });
    
    tray_count_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/robot/tray_count", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            tray_count_ = msg->data;
            emit trayCountChanged();
        });
    
    qDebug() << "RobotController initialized";
    
    // Auto-poll angles/pose every 500ms for realtime display
    poll_timer_ = new QTimer(this);
    connect(poll_timer_, &QTimer::timeout, this, &RobotController::pollRobotState);
    poll_timer_->start(500);
}

void RobotController::callServiceAsync(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, bool value)
{
    if (!client->wait_for_service(std::chrono::seconds(1))) {
        qWarning() << "Service not available";
        emit serviceCallResult(false, "Service not available");
        return;
    }
    
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = value;
    client->async_send_request(request);
    emit serviceCallResult(true, "Request sent");
}

void RobotController::enableSystem(bool enable)
{
    qDebug() << "Enable system:" << enable;
    callServiceAsync(enable_client_, enable);
}

void RobotController::emergencyStop(bool stop)
{
    qDebug() << "Emergency stop:" << stop;
    callServiceAsync(emergency_stop_client_, stop);
}

void RobotController::setManualMode(bool enable)
{
    qDebug() << "Manual mode:" << enable;
    callServiceAsync(manual_mode_client_, enable);
}

void RobotController::setAiMode(bool enable)
{
    qDebug() << "AI mode:" << enable;
    callServiceAsync(ai_mode_client_, enable);
}

void RobotController::switchCamera(int cameraId)
{
    qDebug() << "Switch camera:" << cameraId;
    auto msg = std_msgs::msg::Int32();
    msg.data = cameraId;
    camera_select_pub_->publish(msg);
}

void RobotController::selectRow(int row)
{
    qDebug() << "Select row:" << row;
    selected_row_ = row;
    emit selectedRowChanged();
    auto msg = std_msgs::msg::Int32();
    msg.data = row;
    command_row_pub_->publish(msg);
}

void RobotController::selectSlot(int slot)
{
    qDebug() << "Select slot:" << slot;
    selected_slot_ = slot;
    emit selectedSlotChanged();
    auto msg = std_msgs::msg::Int32();
    msg.data = slot;
    command_slot_pub_->publish(msg);
}

void RobotController::gotoState(const QString& state)
{
    qDebug() << "Goto state:" << state;
    auto msg = std_msgs::msg::String();
    msg.data = state.toStdString();
    goto_state_pub_->publish(msg);
}

// ═══════════════════════════════════════════════════════════════
// AUTO-POLL REALTIME STATE
// ═══════════════════════════════════════════════════════════════

void RobotController::pollRobotState()
{
    getAngles();
    getPose();
    pollErrorID();
}

// ═══════════════════════════════════════════════════════════════
// JOG CONTROL
// ═══════════════════════════════════════════════════════════════

void RobotController::jogStart(const QString& axisId)
{
    qDebug() << "Jog start:" << axisId;
    auto request = std::make_shared<dobot_msgs_v3::srv::MoveJog::Request>();
    request->axis_id = axisId.toStdString();
    // No extra params needed for basic jog
    
    jog_client_->async_send_request(request,
        [this, axisId](rclcpp::Client<dobot_msgs_v3::srv::MoveJog>::SharedFuture future) {
            try {
                auto result = future.get();
                qDebug() << "Jog" << axisId << "result:" << result->res;
            } catch (const std::exception& e) {
                qWarning() << "Jog failed:" << e.what();
            }
        });
}

void RobotController::jogStop()
{
    qDebug() << "Jog stop";
    auto request = std::make_shared<dobot_msgs_v3::srv::MoveJog::Request>();
    request->axis_id = "";  // Empty = stop
    
    jog_client_->async_send_request(request,
        [this](rclcpp::Client<dobot_msgs_v3::srv::MoveJog>::SharedFuture future) {
            try {
                auto result = future.get();
                qDebug() << "Jog stop result:" << result->res;
            } catch (const std::exception& e) {
                qWarning() << "Jog stop failed:" << e.what();
            }
        });
}

void RobotController::getAngles()
{
    qDebug() << "Get angles";
    auto request = std::make_shared<dobot_msgs_v3::srv::GetAngle::Request>();
    
    get_angle_client_->async_send_request(request,
        [this](rclcpp::Client<dobot_msgs_v3::srv::GetAngle>::SharedFuture future) {
            try {
                auto result = future.get();
                // Parse angle string: "{j1, j2, j3, j4, j5, j6}"
                std::string angle_str = result->angle;
                qDebug() << "Angles raw:" << QString::fromStdString(angle_str);
                
                // Remove braces and parse
                std::string clean = angle_str;
                clean.erase(std::remove(clean.begin(), clean.end(), '{'), clean.end());
                clean.erase(std::remove(clean.begin(), clean.end(), '}'), clean.end());
                
                std::istringstream iss(clean);
                QVariantList angles;
                std::string token;
                while (std::getline(iss, token, ',')) {
                    try {
                        angles.append(std::stod(token));
                    } catch (...) {
                        angles.append(0.0);
                    }
                }
                
                // Pad to 6 if needed
                while (angles.size() < 6) angles.append(0.0);
                
                joint_angles_ = angles;
                emit jointAnglesChanged();
            } catch (const std::exception& e) {
                qWarning() << "GetAngle failed:" << e.what();
            }
        });
}

void RobotController::getPose()
{
    qDebug() << "Get pose";
    auto request = std::make_shared<dobot_msgs_v3::srv::GetPose::Request>();
    request->user = 0;
    request->tool = 0;
    
    get_pose_client_->async_send_request(request,
        [this](rclcpp::Client<dobot_msgs_v3::srv::GetPose>::SharedFuture future) {
            try {
                auto result = future.get();
                // Parse pose string: "{x, y, z, rx, ry, rz}"
                std::string pose_str = result->pose;
                qDebug() << "Pose raw:" << QString::fromStdString(pose_str);
                
                std::string clean = pose_str;
                clean.erase(std::remove(clean.begin(), clean.end(), '{'), clean.end());
                clean.erase(std::remove(clean.begin(), clean.end(), '}'), clean.end());
                
                std::istringstream iss(clean);
                QVariantList pose;
                std::string token;
                while (std::getline(iss, token, ',')) {
                    try {
                        pose.append(std::stod(token));
                    } catch (...) {
                        pose.append(0.0);
                    }
                }
                
                while (pose.size() < 6) pose.append(0.0);
                
                cartesian_pose_ = pose;
                emit cartesianPoseChanged();
            } catch (const std::exception& e) {
                qWarning() << "GetPose failed:" << e.what();
            }
        });
}

// ═══════════════════════════════════════════════════════════════
// MOVE TO EXACT POSITION
// ═══════════════════════════════════════════════════════════════

void RobotController::moveJoint(double j1, double j2, double j3, double j4, double j5, double j6)
{
    qDebug() << "MoveJoint:" << j1 << j2 << j3 << j4 << j5 << j6;
    auto request = std::make_shared<dobot_msgs_v3::srv::JointMovJ::Request>();
    request->j1 = j1; request->j2 = j2; request->j3 = j3;
    request->j4 = j4; request->j5 = j5; request->j6 = j6;
    joint_movj_client_->async_send_request(request,
        [this](rclcpp::Client<dobot_msgs_v3::srv::JointMovJ>::SharedFuture future) {
            try { auto r = future.get(); qDebug() << "MoveJoint result:" << r->res; }
            catch (const std::exception& e) { qWarning() << "MoveJoint failed:" << e.what(); }
        });
}

void RobotController::moveLinear(double x, double y, double z, double rx, double ry, double rz)
{
    qDebug() << "MoveLinear:" << x << y << z << rx << ry << rz;
    auto request = std::make_shared<dobot_msgs_v3::srv::MovL::Request>();
    request->x = x; request->y = y; request->z = z;
    request->rx = rx; request->ry = ry; request->rz = rz;
    movl_client_->async_send_request(request,
        [this](rclcpp::Client<dobot_msgs_v3::srv::MovL>::SharedFuture future) {
            try { auto r = future.get(); qDebug() << "MoveLinear result:" << r->res; }
            catch (const std::exception& e) { qWarning() << "MoveLinear failed:" << e.what(); }
        });
}

// ═══════════════════════════════════════════════════════════════
// DIGITAL OUTPUT CONTROL
// ═══════════════════════════════════════════════════════════════

void RobotController::setDigitalOutput(int index, bool status)
{
    qDebug() << "SetDO:" << index << status;
    auto request = std::make_shared<dobot_msgs_v3::srv::DO::Request>();
    request->index = index;
    request->status = status ? 1 : 0;
    do_client_->async_send_request(request,
        [this, index, status](rclcpp::Client<dobot_msgs_v3::srv::DO>::SharedFuture future) {
            try { auto r = future.get(); qDebug() << "DO" << index << "=" << status << "result:" << r->res; }
            catch (const std::exception& e) { qWarning() << "SetDO failed:" << e.what(); }
        });
}

// ═══════════════════════════════════════════════════════════════
// PAUSE / RESUME / CLEAR ERROR
// ═══════════════════════════════════════════════════════════════

void RobotController::pauseRobot()
{
    qDebug() << "PauseRobot";
    auto request = std::make_shared<dobot_msgs_v3::srv::Pause::Request>();
    pause_client_->async_send_request(request,
        [](rclcpp::Client<dobot_msgs_v3::srv::Pause>::SharedFuture future) {
            try { qDebug() << "Pause result:" << future.get()->res; }
            catch (const std::exception& e) { qWarning() << "Pause failed:" << e.what(); }
        });
}

void RobotController::resumeRobot()
{
    qDebug() << "ResumeRobot (ResetRobot)";
    auto request = std::make_shared<dobot_msgs_v3::srv::ResetRobot::Request>();
    reset_robot_client_->async_send_request(request,
        [](rclcpp::Client<dobot_msgs_v3::srv::ResetRobot>::SharedFuture future) {
            try { qDebug() << "Resume result:" << future.get()->res; }
            catch (const std::exception& e) { qWarning() << "Resume failed:" << e.what(); }
        });
}

void RobotController::clearError()
{
    qDebug() << "ClearError";
    auto request = std::make_shared<dobot_msgs_v3::srv::ClearError::Request>();
    clear_error_client_->async_send_request(request,
        [](rclcpp::Client<dobot_msgs_v3::srv::ClearError>::SharedFuture future) {
            try { qDebug() << "ClearError result:" << future.get()->res; }
            catch (const std::exception& e) { qWarning() << "ClearError failed:" << e.what(); }
        });
}

// ═══════════════════════════════════════════════════════════════
// SPEED FACTOR
// ═══════════════════════════════════════════════════════════════

void RobotController::setSpeedRatio(int ratio)
{
    qDebug() << "SetSpeedRatio:" << ratio;
    auto request = std::make_shared<dobot_msgs_v3::srv::SpeedFactor::Request>();
    request->ratio = ratio;
    speed_factor_client_->async_send_request(request,
        [this, ratio](rclcpp::Client<dobot_msgs_v3::srv::SpeedFactor>::SharedFuture future) {
            try {
                auto r = future.get();
                qDebug() << "SpeedFactor result:" << r->res;
                if (r->res == 0) {
                    speed_ratio_ = ratio;
                    emit speedRatioChanged();
                }
            } catch (const std::exception& e) { qWarning() << "SpeedFactor failed:" << e.what(); }
        });
}

// ═══════════════════════════════════════════════════════════════
// ERROR ID POLLING
// ═══════════════════════════════════════════════════════════════

void RobotController::pollErrorID()
{
    auto request = std::make_shared<dobot_msgs_v3::srv::GetErrorID::Request>();
    get_error_id_client_->async_send_request(request,
        [this](rclcpp::Client<dobot_msgs_v3::srv::GetErrorID>::SharedFuture future) {
            try {
                auto r = future.get();
                QString newLog = QString::fromStdString(r->error_id);
                if (newLog != error_log_) {
                    error_log_ = newLog;
                    emit errorLogChanged();
                }
            } catch (const std::exception& e) {
                // silently ignore poll errors
            }
        });
}

// ═══════════════════════════════════════════════════════════════
// SIMULATION TRIGGERS
// ═══════════════════════════════════════════════════════════════

void RobotController::simulateFeedChamber()
{
    qDebug() << "SimulateFeedChamber -> goto INIT_LOAD_CHAMBER_DIRECT + feed_chamber=true";
    // First transition robot to correct state
    auto state_msg = std_msgs::msg::String();
    state_msg.data = "INIT_LOAD_CHAMBER_DIRECT";
    goto_state_pub_->publish(state_msg);
    // Then send feed_chamber signal
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    feed_chamber_pub_->publish(msg);
}

void RobotController::simulateFillDone()
{
    qDebug() << "SimulateFillDone -> /fill_machine/fill_done = true";
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    fill_done_pub_->publish(msg);
}
