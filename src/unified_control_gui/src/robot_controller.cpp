#include "unified_control_gui/robot_controller.hpp"
#include <QDebug>
#include <QFile>
#include <QRegularExpression>
#include <QSettings>
#include <QTcpSocket>
#include <QThread>
#include <sstream>
#include <thread>

RobotController::RobotController(rclcpp::Node::SharedPtr node, QObject *parent)
    : QObject(parent)
    , node_(node)
{
    // Init joint angles to 6 zeros
    for (int i = 0; i < 6; i++) joint_angles_.append(0.0);
    for (int i = 0; i < 6; i++) cartesian_pose_.append(0.0);
    for (int i = 0; i < 5; i++) row_ready_.append(false);
    for (int i = 0; i < 9; i++) slot_ready_.append(false);

    // Create service clients
    enable_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/enable_system");
    start_system_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/start_system");
    emergency_stop_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/emergency_stop");
    pause_system_client_   = node_->create_client<std_srvs::srv::SetBool>("/robot/pause_system");  // NEW

    manual_mode_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/set_manual_mode");
    ai_mode_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/set_ai_mode");
    
    // Dobot jog service clients
    jog_client_ = node_->create_client<dobot_msgs_v3::srv::MoveJog>("/nova5/dobot_bringup/MoveJog");
    get_angle_client_ = node_->create_client<dobot_msgs_v3::srv::GetAngle>("/nova5/dobot_bringup/GetAngle");
    get_pose_client_ = node_->create_client<dobot_msgs_v3::srv::GetPose>("/nova5/dobot_bringup/GetPose");
    joint_movj_client_ = node_->create_client<dobot_msgs_v3::srv::JointMovJ>("/nova5/dobot_bringup/JointMovJ");
    movl_client_ = node_->create_client<dobot_msgs_v3::srv::MovL>("/nova5/dobot_bringup/MovL");
    servo_p_client_ = node_->create_client<dobot_msgs_v3::srv::ServoP>("/nova5/dobot_bringup/ServoP");
    do_client_ = node_->create_client<dobot_msgs_v3::srv::DO>("/nova5/dobot_bringup/DO");
    pause_client_ = node_->create_client<dobot_msgs_v3::srv::Pause>("/nova5/dobot_bringup/Pause");
    dobot_emergency_stop_client_ = node_->create_client<dobot_msgs_v3::srv::EmergencyStop>("/nova5/dobot_bringup/EmergencyStop");
    dobot_stop_script_client_ = node_->create_client<dobot_msgs_v3::srv::StopScript>("/nova5/dobot_bringup/StopScript");
    clear_error_client_ = node_->create_client<dobot_msgs_v3::srv::ClearError>("/nova5/dobot_bringup/ClearError");
    reset_robot_client_ = node_->create_client<dobot_msgs_v3::srv::ResetRobot>("/nova5/dobot_bringup/ResetRobot");
    speed_factor_client_ = node_->create_client<dobot_msgs_v3::srv::SpeedFactor>("/nova5/dobot_bringup/SpeedFactor");
    get_error_id_client_ = node_->create_client<dobot_msgs_v3::srv::GetErrorID>("/nova5/dobot_bringup/GetErrorID");
    reset_state_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/reset_state");
    soft_stop_to_manual_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/soft_stop_to_manual");

    // Create publishers
    camera_select_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robot/command_camera", 10);
    command_row_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robot/command_row", 10);
    command_slot_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robot/command_slot", 10);
    goto_state_pub_ = node_->create_publisher<std_msgs::msg::String>("/robot/goto_state", 10);
    set_mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robot/set_mode", 10);
    feed_chamber_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/revpi/feed_chamber", 10);
    fill_done_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/revpi/fill_done", 10);
    input_tray_ready_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cartridge_providesystem/new_tray_loaded", 10);
    output_tray_ready_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cartridge_providesystem/new_trayoutput_loaded", 10);
    scale_result_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/scale/result", 10);
    speed_ratio_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robot/speed_ratio", rclcpp::QoS(10).reliable().transient_local());
    system_start_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/system/start_button", 10);  // shared with cartridge
    system_pause_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/system/pause_button", 10);  // sync sang cartridge
    system_resume_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/system/resume_button", 10); // sync sang cartridge
    ignore_scale_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/robot/ignore_scale", 10);
    gripper_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/robot/gripper_cmd", 10);
    picker_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/robot/picker_cmd", 10);
    
    // Create subscribers
    system_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot/system_status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                system_status_ = QString::fromStdString(msg->data);
                emit systemStatusChanged();
            }, Qt::QueuedConnection);
        });
    
    error_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot/error", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                error_message_ = QString::fromStdString(msg->data);
                emit errorMessageChanged();
            }, Qt::QueuedConnection);
        });
    
    selected_row_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/robot/selected_input_row", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                selected_row_ = msg->data;
                emit selectedRowChanged();
            }, Qt::QueuedConnection);
        });
    
    selected_slot_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/robot/selected_output_slot", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                selected_slot_ = msg->data;
                emit selectedSlotChanged();
            }, Qt::QueuedConnection);
        });

    row_status_sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/vision/input_tray/row_status", 10,
        [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QVariantList next;
                for (int i = 0; i < 5; ++i) {
                    next.append(i < (int)msg->data.size() && msg->data[i] == 1);
                }
                if (next != row_ready_) {
                    row_ready_ = next;
                    emit rowReadyChanged();
                }
            }, Qt::QueuedConnection);
        });

    slot_status_sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/vision/output_tray/slot_status", 10,
        [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QVariantList next;
                // slot_status: 0=empty (ready to place), 1=occupied (blank)
                for (int i = 0; i < 9; ++i) {
                    next.append(i < (int)msg->data.size() && msg->data[i] == 0);
                }
                if (next != slot_ready_) {
                    slot_ready_ = next;
                    emit slotReadyChanged();
                }
            }, Qt::QueuedConnection);
        });
    
    system_uptime_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot/system_uptime", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                system_uptime_ = QString::fromStdString(msg->data);
                emit systemUptimeChanged();
            }, Qt::QueuedConnection);
        });

    in_ready_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/cartridge_providesystem/new_tray_loaded", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                if (in_ready_ != msg->data) {
                    in_ready_ = msg->data;
                    emit inReadyChanged();
                }
            }, Qt::QueuedConnection);
        });

    out_ready_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/cartridge_providesystem/new_trayoutput_loaded", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                if (out_ready_ != msg->data) {
                    out_ready_ = msg->data;
                    emit outReadyChanged();
                }
            }, Qt::QueuedConnection);
        });

    tray_count_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/robot/tray_count", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                tray_count_ = msg->data;
                emit trayCountChanged();
            }, Qt::QueuedConnection);
        });
    
    // Subscribe to hardware speed factor readback from Dobot feedback node
    hw_speed_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/robot/hw_speed_factor", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                if (hw_speed_ratio_ != msg->data) {
                    hw_speed_ratio_ = msg->data;
                    emit hwSpeedRatioChanged();
                }
            }, Qt::QueuedConnection);
        });

    // Realtime joint angles from feedback streaming (Dobot port 30004, ~100Hz)
    // Replaces GetAngle service polling — eliminates 500ms display lag.
    // feedback.py publishes JointState in radians; convert to degrees for GUI.
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/nova5/joint_states_robot", sensor_qos,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            QVariantList angles;
            constexpr double RAD2DEG = 57.29577951308232;
            for (double p : msg->position) angles.append(p * RAD2DEG);
            while (angles.size() < 6) angles.append(0.0);
            
            std::lock_guard<std::mutex> lock(status_mutex_);
            next_joint_angles_ = angles;
            has_new_joints_ = true;
        });

    // Realtime TCP pose from feedback streaming — already in mm/deg.
    tool_vector_sub_ = node_->create_subscription<dobot_msgs_v3::msg::ToolVectorActual>(
        "/nova5/dobot_msgs_v3/msg/ToolVectorActual", sensor_qos,
        [this](const dobot_msgs_v3::msg::ToolVectorActual::SharedPtr msg) {
            QVariantList pose;
            pose.append(msg->x); pose.append(msg->y); pose.append(msg->z);
            pose.append(msg->rx); pose.append(msg->ry); pose.append(msg->rz);
            
            std::lock_guard<std::mutex> lock(status_mutex_);
            next_cartesian_pose_ = pose;
            has_new_pose_ = true;
        });

    // High-frequency GUI Update Timer (20Hz / 50ms)
    // Decouples high-frequency ROS streaming (~100Hz) from Qt Main Thread rendering.
    // Prevents GUI event loop starvation, drastically reduces CPU usage, and guarantees thread safety.
    QTimer* gui_update_timer = new QTimer(this);
    connect(gui_update_timer, &QTimer::timeout, this, [this]() {
        bool update_joints = false;
        bool update_pose = false;
        QVariantList joints_to_set;
        QVariantList pose_to_set;

        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            if (has_new_joints_) {
                joints_to_set = next_joint_angles_;
                has_new_joints_ = false;
                update_joints = true;
            }
            if (has_new_pose_) {
                pose_to_set = next_cartesian_pose_;
                has_new_pose_ = false;
                update_pose = true;
            }
        }

        if (update_joints) {
            joint_angles_ = joints_to_set;
            emit jointAnglesChanged();
        }
        if (update_pose) {
            cartesian_pose_ = pose_to_set;
            emit cartesianPoseChanged();
        }
    });
    gui_update_timer->start(50); // 50ms (20Hz)

    qDebug() << "RobotController initialized";

    // Load persisted speed ratio
    QSettings settings("RobotControl", "ManualMode");
    int savedSpeed = settings.value("speedRatio", 100).toInt();
    speed_ratio_ = qBound(1, savedSpeed, 100);
    qDebug() << "Loaded speed ratio:" << speed_ratio_;

    // Publish initial speed ratio so motion_executor AND Dobot hardware get it on startup
    // Use a single-shot timer to ensure services/publishers are ready before sending
    QTimer::singleShot(3000, this, [this]() {
        // 1. Sync to motion_executor (for SpeedJ/SpeedL before each move)
        std_msgs::msg::Int32 msg;
        msg.data = speed_ratio_;
        speed_ratio_pub_->publish(msg);
        qDebug() << "[STARTUP] Published initial speed ratio:" << speed_ratio_;

        // 2. Sync to Dobot hardware (SpeedFactor — persists until power cycle)
        if (speed_factor_client_ && speed_factor_client_->service_is_ready()) {
            auto req = std::make_shared<dobot_msgs_v3::srv::SpeedFactor::Request>();
            req->ratio = speed_ratio_;
            speed_factor_client_->async_send_request(req,
                [this](rclcpp::Client<dobot_msgs_v3::srv::SpeedFactor>::SharedFuture f) {
                    try {
                        auto r = f.get();
                        qDebug() << "[STARTUP] SpeedFactor synced to hardware:" << speed_ratio_ << "% (res:" << r->res << ")";
                    } catch (...) {
                        qWarning() << "[STARTUP] SpeedFactor sync failed — hardware may use default speed";
                    }
                });
        } else {
            qWarning() << "[STARTUP] SpeedFactor service not ready — hardware speed may differ from GUI";
        }
    });

    // Position/pose stream in at ~100Hz via joint_state_sub_ + tool_vector_sub_.
    // Only error ID still needs service polling — slower cadence is fine.
    poll_timer_ = new QTimer(this);
    connect(poll_timer_, &QTimer::timeout, this, &RobotController::pollRobotState);
    poll_timer_->start(1000);
}

void RobotController::callServiceAsync(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, bool value)
{
    if (!client->service_is_ready()) {
        qWarning() << "Service not available yet, sending anyway...";
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

void RobotController::stopAndResetRobot()
{
    qDebug() << "Stop & Reset Robot";

    // ✅ HARD HALT motion: EmergencyStop + StopScript TRƯỚC Pause/ResetRobot.
    // Pause/ResetRobot KHÔNG halt được motion đang chạy mid-MovJ; chỉ Dobot
    // EmergencyStop service moi cat duong di chuyen ngay lap tuc.
    auto eStopReq = std::make_shared<dobot_msgs_v3::srv::EmergencyStop::Request>();
    if (dobot_emergency_stop_client_ && dobot_emergency_stop_client_->service_is_ready()) {
        qDebug() << "  -> EmergencyStop (hard halt motion)";
        dobot_emergency_stop_client_->async_send_request(eStopReq);
    } else {
        qWarning() << "  -> EmergencyStop service NOT ready — motion may NOT halt!";
    }

    auto stopScriptReq = std::make_shared<dobot_msgs_v3::srv::StopScript::Request>();
    if (dobot_stop_script_client_ && dobot_stop_script_client_->service_is_ready()) {
        qDebug() << "  -> StopScript (abort script)";
        dobot_stop_script_client_->async_send_request(stopScriptReq);
    }

    // FORCE INSTANT STOP: Pause + ResetRobot (sau EmergencyStop để clear state)
    auto pauseReq = std::make_shared<dobot_msgs_v3::srv::Pause::Request>();
    if (pause_client_ && pause_client_->service_is_ready()) {
        pause_client_->async_send_request(pauseReq);
    }

    auto resetRobotReq = std::make_shared<dobot_msgs_v3::srv::ResetRobot::Request>();
    if (reset_robot_client_ && reset_robot_client_->service_is_ready()) {
        reset_robot_client_->async_send_request(resetRobotReq);
    }

    // Step 1: Reset state machine → IDLE (stops auto mode, clears all flags)
    auto resetReq = std::make_shared<std_srvs::srv::SetBool::Request>();
    resetReq->data = true;
    if (reset_state_client_->service_is_ready()) {
        reset_state_client_->async_send_request(resetReq,
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture f) {
                try { qDebug() << "Reset state:" << f.get()->message.c_str(); } catch (...) {}
            });
    } else {
        qWarning() << "/robot/reset_state not available (robot_logic_node not running?)";
    }

    // Step 2: Clear E-stop flag
    auto estopReq = std::make_shared<std_srvs::srv::SetBool::Request>();
    estopReq->data = false;
    if (emergency_stop_client_->service_is_ready()) {
        emergency_stop_client_->async_send_request(estopReq);
    }

    // Step 3: ClearError + EnableRobot (after 500ms for reset_state to complete)
    QTimer::singleShot(500, this, [this]() {
        auto clearReq = std::make_shared<dobot_msgs_v3::srv::ClearError::Request>();
        clear_error_client_->async_send_request(clearReq,
            [this](rclcpp::Client<dobot_msgs_v3::srv::ClearError>::SharedFuture f) {
                try { qDebug() << "ClearError:" << f.get()->res; } catch (...) {}
                QMetaObject::invokeMethod(this, [this]() {
                    QTimer::singleShot(300, this, [this]() {
                        callServiceAsync(enable_client_, true);
                        qDebug() << "Robot re-enabled — ready for new commands";
                    });
                }, Qt::QueuedConnection);
            });
    });
}

void RobotController::softStopAndManual()
{
    qDebug() << "Soft Stop & Switch to MANUAL (keep state + CPX)";

    // 1. Pause Dobot ngay → robot motion ngừng vật lý
    auto pauseReq = std::make_shared<dobot_msgs_v3::srv::Pause::Request>();
    if (pause_client_ && pause_client_->service_is_ready()) {
        pause_client_->async_send_request(pauseReq);
    }

    // 1b. ResetRobot: clear motion buffer để Dobot accept lệnh manual mới (MovJ, JointMovJ)
    //     Nếu chỉ Pause thì robot ở paused state, các lệnh motion sau sẽ bị reject.
    auto resetRobotReq = std::make_shared<dobot_msgs_v3::srv::ResetRobot::Request>();
    if (reset_robot_client_ && reset_robot_client_->service_is_ready()) {
        reset_robot_client_->async_send_request(resetRobotReq);
    }

    // 2. Gọi /robot/soft_stop_to_manual: cancel motion action + manual mode, giữ state
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = true;
    if (soft_stop_to_manual_client_ && soft_stop_to_manual_client_->service_is_ready()) {
        soft_stop_to_manual_client_->async_send_request(req,
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture f) {
                try { qDebug() << "Soft stop:" << f.get()->message.c_str(); } catch (...) {}
            });
    } else {
        qWarning() << "/robot/soft_stop_to_manual not available";
    }

    // 3. ClearError sau 300ms (drive có thể vào error mode khi pause giữa motion)
    QTimer::singleShot(300, this, [this]() {
        auto clearReq = std::make_shared<dobot_msgs_v3::srv::ClearError::Request>();
        if (clear_error_client_ && clear_error_client_->service_is_ready()) {
            clear_error_client_->async_send_request(clearReq);
        }
    });
}

void RobotController::startSystem(bool start)
{
    qDebug() << "Start system:" << start;
    // Call robot service
    callServiceAsync(start_system_client_, start);
    // Also publish to shared topic so cartridge system starts too
    if (start) {
        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        system_start_pub_->publish(msg);
        qDebug() << "Published /system/start_button = true (cartridge trigger)";
    }
}

void RobotController::emergencyStop(bool stop)
{
    qDebug() << "Emergency stop:" << stop;
    callServiceAsync(emergency_stop_client_, stop);
}

void RobotController::setManualMode(bool enable)
{
    qDebug() << "Set Manual mode";
    if (enable) {
        auto msg = std_msgs::msg::Int32();
        msg.data = 3; // 3 = MANUAL
        set_mode_pub_->publish(msg);
    }
}

void RobotController::setAiMode(bool enable)
{
    qDebug() << "Set AI mode";
    if (enable) {
        auto msg = std_msgs::msg::Int32();
        msg.data = 2; // 2 = AI
        set_mode_pub_->publish(msg);
    }
}

void RobotController::setAutoMode(bool enable)
{
    qDebug() << "Set Auto mode";
    if (enable) {
        auto msg = std_msgs::msg::Int32();
        msg.data = 1; // 1 = AUTO
        set_mode_pub_->publish(msg);
    }
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
    pollErrorID();
}

// ═══════════════════════════════════════════════════════════════
// JOG CONTROL
// ═══════════════════════════════════════════════════════════════

void RobotController::setIgnoreScale(bool ignore)
{
    if (ignore_scale_ != ignore) {
        ignore_scale_ = ignore;
        emit ignoreScaleChanged();
        
        auto msg = std_msgs::msg::Bool();
        msg.data = ignore;
        if (ignore_scale_pub_) ignore_scale_pub_->publish(msg);
        qDebug() << "Ignore Scale set to:" << ignore;
    }
}

// Helper: parse "{v1,v2,...}" into vector<double>
static std::vector<double> parseDobot(const std::string& raw) {
    std::string s = raw;
    s.erase(std::remove(s.begin(), s.end(), '{'), s.end());
    s.erase(std::remove(s.begin(), s.end(), '}'), s.end());
    std::istringstream iss(s);
    std::vector<double> v;
    std::string tok;
    while (std::getline(iss, tok, ',')) v.push_back(std::stod(tok));
    return v;
}

// Helper: get joint axis index from "j1"-"j6"
static int jointIdx(const QString& axis) {
    if (axis == "j1") return 0; if (axis == "j2") return 1;
    if (axis == "j3") return 2; if (axis == "j4") return 3;
    if (axis == "j5") return 4; if (axis == "j6") return 5;
    return -1;
}

// Helper: get Cartesian axis index from "x","y","z","rx","ry","rz"
static int cartIdx(const QString& axis) {
    if (axis == "x") return 0; if (axis == "y") return 1;
    if (axis == "z") return 2; if (axis == "rx") return 3;
    if (axis == "ry") return 4; if (axis == "rz") return 5;
    return -1;
}

void RobotController::setJogContinuous(bool c) {
    if (jog_continuous_ != c) { jog_continuous_ = c; emit jogContinuousChanged(); }
}

void RobotController::setJogStepSize(double s) {
    if (jog_step_size_ != s) { jog_step_size_ = s; emit jogStepSizeChanged(); }
}

void RobotController::jogStart(const QString& axisId)
{
    QString fixedId = axisId;
    bool isJoint = fixedId.startsWith("j", Qt::CaseInsensitive) && fixedId.length() > 1 && fixedId[1].isDigit();

    if (isJoint) {
        fixedId[0] = 'J';
    }

    qDebug() << "Jog start:" << axisId << "→" << fixedId << (isJoint ? "JOINT" : "CART");
    jog_axis_ = fixedId;
    jog_moving_ = true;
    jog_start_time_ = std::chrono::steady_clock::now();

    if (isJoint) {
        // Joint: MoveJog native
        auto req = std::make_shared<dobot_msgs_v3::srv::MoveJog::Request>();
        req->axis_id = fixedId.toStdString();
        req->param_value = {};
        jog_client_->async_send_request(req,
            [this, fixedId](rclcpp::Client<dobot_msgs_v3::srv::MoveJog>::SharedFuture f) {
                try { qDebug() << "MoveJog:" << fixedId << "res:" << f.get()->res; }
                catch (...) {}
            });
    } else {
        // Cartesian: ServoP streaming via QTimer
        QString ax = fixedId.toLower();
        jog_cart_positive_ = ax.endsWith("+");
        ax.chop(1);
        jog_cart_idx_ = -1;
        if (ax == "x") jog_cart_idx_ = 0;
        else if (ax == "y") jog_cart_idx_ = 1;
        else if (ax == "z") jog_cart_idx_ = 2;
        else if (ax == "rx") jog_cart_idx_ = 3;
        else if (ax == "ry") jog_cart_idx_ = 4;
        else if (ax == "rz") jog_cart_idx_ = 5;
        if (jog_cart_idx_ < 0) return;

        // Initialize target from cached pose
        for (int i = 0; i < 6 && i < cartesian_pose_.size(); i++)
            jog_cart_target_[i] = cartesian_pose_[i].toDouble();

        // Start streaming timer: 33ms interval (~30Hz), 0.5mm/tick = ~15mm/s
        if (!jog_timer_) {
            jog_timer_ = new QTimer(this);
            connect(jog_timer_, &QTimer::timeout, this, &RobotController::sendCartesianStep);
        }
        jog_timer_->start(33);
        sendCartesianStep();  // first tick immediately
    }
}

void RobotController::sendCartesianStep()
{
    if (!jog_moving_ || jog_cart_idx_ < 0) {
        if (jog_timer_) jog_timer_->stop();
        return;
    }

    double step = (jog_cart_idx_ < 3) ? 0.5 : 0.2;  // 0.5mm for XYZ, 0.2° for rotation
    double delta = jog_cart_positive_ ? step : -step;

    jog_cart_target_[jog_cart_idx_] += delta;

    auto req = std::make_shared<dobot_msgs_v3::srv::ServoP::Request>();
    req->x  = jog_cart_target_[0];
    req->y  = jog_cart_target_[1];
    req->z  = jog_cart_target_[2];
    req->rx = jog_cart_target_[3];
    req->ry = jog_cart_target_[4];
    req->rz = jog_cart_target_[5];

    servo_p_client_->async_send_request(req);
}

void RobotController::sendMoveJog(const QString&) {}
void RobotController::sendJogStep() {}

void RobotController::jogStop()
{
    qDebug() << "Jog stop";
    bool wasJoint = jog_axis_.startsWith("J");
    jog_moving_ = false;
    jog_axis_.clear();

    if (wasJoint) {
        // Joint: MoveJog("") stop
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - jog_start_time_).count();
        int delay = (elapsed < 200) ? (200 - elapsed) : 0;
        QTimer::singleShot(delay, this, [this]() {
            auto req = std::make_shared<dobot_msgs_v3::srv::MoveJog::Request>();
            req->axis_id = "";
            req->param_value = {};
            jog_client_->async_send_request(req,
                [](rclcpp::Client<dobot_msgs_v3::srv::MoveJog>::SharedFuture) {});
        });
    } else {
        // Cartesian: stop ServoP timer = robot stops immediately
        if (jog_timer_) jog_timer_->stop();
        jog_cart_idx_ = -1;
    }
}

void RobotController::getAngles()
{
    auto request = std::make_shared<dobot_msgs_v3::srv::GetAngle::Request>();

    get_angle_client_->async_send_request(request,
        [this](rclcpp::Client<dobot_msgs_v3::srv::GetAngle>::SharedFuture future) {
            try {
                auto result = future.get();
                // Response format: "0,{j1,j2,j3,j4,j5,j6},GetAngle();"
                // Extract ONLY content between { and }
                const std::string& raw = result->angle;
                auto beg = raw.find('{');
                auto end = raw.find('}');
                if (beg == std::string::npos || end == std::string::npos || end <= beg) return;
                std::string inner = raw.substr(beg + 1, end - beg - 1);

                std::istringstream iss(inner);
                QVariantList angles;
                std::string token;
                while (std::getline(iss, token, ',')) {
                    try { angles.append(std::stod(token)); }
                    catch (...) { angles.append(0.0); }
                }
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
    auto request = std::make_shared<dobot_msgs_v3::srv::GetPose::Request>();
    request->user = 0;
    request->tool = 0;

    get_pose_client_->async_send_request(request,
        [this](rclcpp::Client<dobot_msgs_v3::srv::GetPose>::SharedFuture future) {
            try {
                auto result = future.get();
                // Response format: "0,{x,y,z,rx,ry,rz},GetPose();"
                // Extract ONLY content between { and }
                const std::string& raw = result->pose;
                auto beg = raw.find('{');
                auto end = raw.find('}');
                if (beg == std::string::npos || end == std::string::npos || end <= beg) return;
                std::string inner = raw.substr(beg + 1, end - beg - 1);

                std::istringstream iss(inner);
                QVariantList pose;
                std::string token;
                while (std::getline(iss, token, ',')) {
                    try { pose.append(std::stod(token)); }
                    catch (...) { pose.append(0.0); }
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

void RobotController::saveJointPose(const QString& name, double j1, double j2, double j3, double j4, double j5, double j6)
{
    // Path to YAML config
    QString yaml_path = QString::fromStdString(
        std::string(std::getenv("HOME") ? std::getenv("HOME") : "/home/pi") +
        "/ros2_ws/src/robot_control_main/config/joint_pose_params.yaml");

    QFile file(yaml_path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        emit jointPoseSaved(false, "Cannot open YAML: " + yaml_path);
        return;
    }
    QString content = QString::fromUtf8(file.readAll());
    file.close();

    // Build new pose line: "J,j1,j2,j3,j4,j5,j6   # name"
    QString comment = name.trimmed().isEmpty() ? "" : ("     # " + name.trimmed());
    QString newLine = QString("      - \"J,%1,%2,%3,%4,%5,%6\"%7")
        .arg(j1, 0, 'f', 4)
        .arg(j2, 0, 'f', 4)
        .arg(j3, 0, 'f', 4)
        .arg(j4, 0, 'f', 4)
        .arg(j5, 0, 'f', 4)
        .arg(j6, 0, 'f', 4)
        .arg(comment);

    // Find insertion point: last "- \"J," entry in the joints list
    int lastJIdx = content.lastIndexOf(QRegularExpression("      - \"J,"));
    if (lastJIdx < 0) {
        emit jointPoseSaved(false, "Could not find joint pose list in YAML");
        return;
    }
    // Find end of that line
    int lineEnd = content.indexOf('\n', lastJIdx);
    if (lineEnd < 0) lineEnd = content.length();

    // Insert new line after the last J, entry
    content.insert(lineEnd + 1, newLine + "\n");

    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        emit jointPoseSaved(false, "Cannot write YAML: " + yaml_path);
        return;
    }
    file.write(content.toUtf8());
    file.close();

    QString msg = QString("Saved: \"%1\" → J(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)")
        .arg(name).arg(j1).arg(j2).arg(j3).arg(j4).arg(j5).arg(j6);
    qDebug() << "saveJointPose:" << msg;
    emit jointPoseSaved(true, msg);
}

QVariantList RobotController::getSavedPoses()
{
    QVariantList list;
    QString yaml_path = QString::fromStdString(
        std::string(std::getenv("HOME") ? std::getenv("HOME") : "/home/pi") +
        "/ros2_ws/src/robot_control_main/config/joint_pose_params.yaml");

    QFile file(yaml_path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "Cannot open YAML for reading poses:" << yaml_path;
        return list;
    }

    while (!file.atEnd()) {
        QString line = QString::fromUtf8(file.readLine());
        // Check for joint move string
        if (line.contains("- \"J,") || line.contains("- 'J,")) {
            // Find name comment
            QString name = "";
            int hashIdx = line.indexOf('#');
            if (hashIdx >= 0) {
                name = line.mid(hashIdx + 1).trimmed();
            }

            // Skip placeholders or entries that contain "placeholder"
            if (name.toLower().contains("placeholder")) {
                continue;
            }

            // Extract coordinate string between quotes
            int firstQuote = line.indexOf('"');
            int lastQuote = line.lastIndexOf('"');
            if (firstQuote < 0 || lastQuote < 0 || lastQuote <= firstQuote) {
                firstQuote = line.indexOf('\'');
                lastQuote = line.lastIndexOf('\'');
            }

            if (firstQuote >= 0 && lastQuote > firstQuote) {
                QString coordStr = line.mid(firstQuote + 1, lastQuote - firstQuote - 1);
                // coordStr looks like "J,104.2439,45.5457,-153.7780,62.7823,85.6854,184.4839"
                if (coordStr.startsWith("J,")) {
                    QStringList parts = coordStr.mid(2).split(',');
                    if (parts.size() >= 6) {
                        QVariantMap map;
                        if (name.isEmpty()) {
                            name = QString("Pose (index %1)").arg(list.size());
                        }
                        map["name"] = name;
                        map["j1"] = parts[0].toDouble();
                        map["j2"] = parts[1].toDouble();
                        map["j3"] = parts[2].toDouble();
                        map["j4"] = parts[3].toDouble();
                        map["j5"] = parts[4].toDouble();
                        map["j6"] = parts[5].toDouble();
                        list.append(map);
                    }
                }
            }
        }
    }
    file.close();
    return list;
}



// ═══════════════════════════════════════════════════════════════
// DIGITAL OUTPUT CONTROL
// ═══════════════════════════════════════════════════════════════

void RobotController::setDigitalOutput(int index, bool status)
{
    qDebug() << "SetDO (via Topic):" << index << status;
    auto msg = std_msgs::msg::Bool();
    msg.data = status;
    if (index == 1 && gripper_cmd_pub_) {
        gripper_cmd_pub_->publish(msg);
    } else if (index == 2 && picker_cmd_pub_) {
        picker_cmd_pub_->publish(msg);
    } else {
        qWarning() << "Invalid DO index for GUI control:" << index;
    }
}

// ═══════════════════════════════════════════════════════════════
// PAUSE / RESUME / CLEAR ERROR
// ═══════════════════════════════════════════════════════════════

void RobotController::pauseRobot()
{
    qDebug() << "PauseRobot -> /robot/pause_system true + /system/pause_button";
    // Graceful PAUSE: robot dừng tại ranh giới motion goal hiện tại.
    // Đồng thời publish /system/pause_button để cartridge cũng pause graceful.
    callServiceAsync(pause_system_client_, true);
    if (system_pause_pub_) {
        std_msgs::msg::Bool m;
        m.data = true;
        system_pause_pub_->publish(m);
    }
}

void RobotController::resumeRobot()
{
    qDebug() << "ResumeRobot -> /robot/pause_system false + /system/resume_button";
    callServiceAsync(pause_system_client_, false);
    if (system_resume_pub_) {
        std_msgs::msg::Bool m;
        m.data = true;
        system_resume_pub_->publish(m);
    }
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
                    // Persist across restarts
                    QSettings settings("RobotControl", "ManualMode");
                    settings.setValue("speedRatio", ratio);
                    settings.sync();
                    // Notify motion_executor about the new speed
                    std_msgs::msg::Int32 smsg;
                    smsg.data = ratio;
                    speed_ratio_pub_->publish(smsg);
                    qDebug() << "[SPEED] Published speed_ratio to motion_executor:" << ratio;
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
    qDebug() << "SimulateFeedChamber -> /revpi/feed_chamber = true";
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    feed_chamber_pub_->publish(msg);
    error_log_ = "[SIMULATION] ⚙️ PICK INPUT Signal Sent";
    emit errorLogChanged();
}

void RobotController::simulateFillDone()
{
    qDebug() << "SimulateFillDone -> /revpi/fill_done = true";
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    fill_done_pub_->publish(msg);
    error_log_ = "[SIMULATION] 💧 PICK CHAMBER (Fill Done) Sent";
    emit errorLogChanged();
}

void RobotController::simulateInputTrayReady()
{
    qDebug() << "SimulateInputTrayReady toggling to" << !in_ready_;
    auto msg = std_msgs::msg::Bool();
    msg.data = !in_ready_;
    input_tray_ready_pub_->publish(msg);
    error_log_ = msg.data ? "[SIMULATION] 📥 INPUT TRAY READY (TRUE)" : "[SIMULATION] 📥 INPUT TRAY READY (FALSE)";
    emit errorLogChanged();
}

void RobotController::simulateOutputTrayReady()
{
    qDebug() << "SimulateOutputTrayReady toggling to" << !out_ready_;
    auto msg = std_msgs::msg::Bool();
    msg.data = !out_ready_;
    output_tray_ready_pub_->publish(msg);
    error_log_ = msg.data ? "[SIMULATION] 📤 OUTPUT TRAY READY (TRUE)" : "[SIMULATION] 📤 OUTPUT TRAY READY (FALSE)";
    emit errorLogChanged();
}

void RobotController::publishScaleResult(bool pass)
{
    qDebug() << "Scale result:" << (pass ? "PASS" : "FAIL");
    auto msg = std_msgs::msg::Bool();
    msg.data = pass;
    scale_result_pub_->publish(msg);
    error_log_ = pass ? "[SCALE] ✅ PASS published" : "[SCALE] ❌ FAIL published";
    emit errorLogChanged();
}
