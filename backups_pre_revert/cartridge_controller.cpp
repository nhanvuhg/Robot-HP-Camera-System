#include "unified_control_gui/cartridge_controller.hpp"
#include <QDebug>
#include <QDateTime>

CartridgeController::CartridgeController(rclcpp::Node::SharedPtr node, QObject *parent)
    : QObject(parent), node_(node)
{
    auto qos = rclcpp::QoS(10);

    // Publishers
    jog_pub_         = node_->create_publisher<std_msgs::msg::String>("/providesystem/jog_cmd", qos);
    sim_sensor_pub_  = node_->create_publisher<std_msgs::msg::String>("/providesystem/sim_sensor", qos);
    move_to_pos_pub_ = node_->create_publisher<std_msgs::msg::String>("/providesystem/move_to_pos", qos);
    set_mode_pub_    = node_->create_publisher<std_msgs::msg::String>("/providesystem/set_operation_mode", qos);
    goto_state_pub_  = node_->create_publisher<std_msgs::msg::String>("/providesystem/goto_state", qos);
    set_target_row_pub_ = node_->create_publisher<std_msgs::msg::String>("/providesystem/set_target_row", qos);
    reset_faults_pub_   = node_->create_publisher<std_msgs::msg::String>("/providesystem/reset_faults", qos);
    get_config_pub_     = node_->create_publisher<std_msgs::msg::String>("/providesystem/get_config", qos);
    update_config_pub_  = node_->create_publisher<std_msgs::msg::String>("/providesystem/update_config", qos);
    hmi_resume_pub_     = node_->create_publisher<std_msgs::msg::Bool>("/providesystem/hmi_resume", qos);
    start_button_pub_   = node_->create_publisher<std_msgs::msg::Bool>("/system/start_button", qos);
    stop_button_pub_    = node_->create_publisher<std_msgs::msg::Bool>("/system/stop_button", qos);
    pause_button_pub_   = node_->create_publisher<std_msgs::msg::Bool>("/system/pause_button", qos);
    confirm_button_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/system/confirm_button", qos);
    gui_confirm_pub_    = node_->create_publisher<std_msgs::msg::String>("/providesystem/gui_confirm", qos);
    done_tray_input_pub_  = node_->create_publisher<std_msgs::msg::Bool>("/robot/done_tray_input", qos);
    done_tray_output_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/robot/done_tray_output", qos);

    // Subscribers
    system_state_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/system_state", rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QString raw = QString::fromStdString(msg->data);
            QStringList parts = raw.split("|");
            if (parts.size() >= 4) {
                QString pyMode = parts[3];
                if (current_mode_ != pyMode) {
                    current_mode_ = pyMode;
                    emit currentModeChanged();
                }
                system_state_ = parts[0];
            } else if (parts.size() > 0) {
                system_state_ = parts[0];
            } else {
                system_state_ = raw;
            }
            emit systemStateChanged();
        });

    config_data_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/providesystem/config_data", rclcpp::QoS(10).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            config_data_ = QString::fromStdString(msg->data);
            emit configDataChanged();
        });

    // Notification from cartridge py
    gui_notify_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/providesystem/gui_notify", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            last_notification_ = QString::fromStdString(msg->data);
            // Parse JSON: {"level":"info","title":"...","detail":"..."}
            QJsonDocument doc = QJsonDocument::fromJson(last_notification_.toUtf8());
            if (doc.isObject()) {
                QJsonObject obj = doc.object();
                QString level = obj.value("level").toString("info");
                QString title = obj.value("title").toString();
                QString detail = obj.value("detail").toString();
                QString logMsg = detail.isEmpty() ? title : title + " — " + detail;
                QString type = (level == "error") ? "err" : (level == "warn") ? "err" : "info";
                addLog(logMsg, type);
                // Detect S11 warning → trigger QML dialog
                if (title.startsWith("S11")) {
                    emit s11WarningRequested();
                }
            } else {
                addLog(last_notification_, "info");
            }
            emit notificationReceived();
        });

    servo_pos_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/providesystem/servo_positions", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            servo_positions_ = QString::fromStdString(msg->data);
            emit servoPositionsChanged();
        });

    sensors_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/providesystem/sensors_state", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QString s = QString::fromStdString(msg->data);
            if (sensor_state_ != s) {
                sensor_state_ = s;
                emit sensorStateChanged();
            }
        });

    qDebug() << "CartridgeController initialized";
}

void CartridgeController::publishString(
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub, const QString &data)
{
    auto msg = std_msgs::msg::String();
    msg.data = data.toStdString();
    pub->publish(msg);
}

// === Servo Control ===

void CartridgeController::jogServo(int id, const QString &dir, int velocity)
{
    publishString(jog_pub_, QString("%1 %2 %3").arg(id).arg(dir).arg(velocity));
}

void CartridgeController::jogStop(int id)
{
    publishString(jog_pub_, QString("%1 stop").arg(id));
}

void CartridgeController::homeServo(int id)
{
    publishString(jog_pub_, QString("home %1").arg(id));
    addLog(QString("Home servo %1 sent").arg(id), "info");
}

void CartridgeController::clearServo(int id)
{
    publishString(jog_pub_, QString("clear %1").arg(id));
    addLog(QString("Clear fault servo %1").arg(id), "info");
}

void CartridgeController::moveServo(int id, double position)
{
    publishString(move_to_pos_pub_, QString("%1:%2").arg(id).arg(position));
    addLog(QString("Move servo %1 to %2 mm").arg(id).arg(position), "ok");
}

// === System Control ===

void CartridgeController::setMode(const QString &mode)
{
    current_mode_ = mode;
    publishString(set_mode_pub_, mode);
    addLog(QString("Mode set to %1").arg(mode.toUpper()), "ok");
    emit currentModeChanged();
}

void CartridgeController::gotoState(const QString &state)
{
    publishString(goto_state_pub_, state);
    addLog(QString("Goto state: %1").arg(state), "info");
}

void CartridgeController::setTargetRow(int row)
{
    publishString(set_target_row_pub_, QString::number(row));
    addLog(QString("Target row set to %1").arg(row), "ok");
}

void CartridgeController::startSystem()
{
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    start_button_pub_->publish(msg);
    addLog("System START", "ok");
}

void CartridgeController::stopSystem()
{
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    stop_button_pub_->publish(msg);
    addLog("System STOP", "err");
}

void CartridgeController::pauseSystem()
{
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    pause_button_pub_->publish(msg);
    addLog("System PAUSE", "info");
}

void CartridgeController::hmiResume()
{
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    hmi_resume_pub_->publish(msg);
    addLog("HMI Resume sent", "info");
}

void CartridgeController::resetFaults()
{
    publishString(reset_faults_pub_, "reset");
    addLog("Reset faults sent", "info");
}

void CartridgeController::confirmOutput()
{
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    confirm_button_pub_->publish(msg);  // chỉ set _confirm_load_received
    addLog("Confirm: đã cấp khạy", "ok");
}

void CartridgeController::s11Respond(bool ok)
{
    auto msg = std_msgs::msg::String();
    msg.data = ok ? "S11_OK" : "S11_NO";
    gui_confirm_pub_->publish(msg);
    addLog(ok ? "S11: OK — thực hiện State2 rồi State1" : "S11: NO — chờ restart", ok ? "ok" : "err");
}

// === Sensor Simulation ===

void CartridgeController::simSensor(const QString &cmd)
{
    publishString(sim_sensor_pub_, cmd);
}

void CartridgeController::simAll(int value)
{
    publishString(sim_sensor_pub_, QString("all:%1").arg(value));
}

void CartridgeController::simClear()
{
    publishString(sim_sensor_pub_, "clear");
}

// === Config ===

void CartridgeController::getConfig()
{
    publishString(get_config_pub_, "request");
}

void CartridgeController::saveConfig(const QString &key, const QString &jsonData)
{
    QJsonObject obj;
    obj["key"] = key;
    obj["data"] = jsonData;
    QString payload = QString::fromUtf8(QJsonDocument(obj).toJson(QJsonDocument::Compact));
    publishString(update_config_pub_, payload);
    addLog(QString("Config saved: %1").arg(key), "ok");
}

// === Log ===

void CartridgeController::addLog(const QString &msg, const QString &type)
{
    QVariantMap entry;
    entry["time"] = QDateTime::currentDateTime().toString("HH:mm:ss");
    entry["msg"] = msg;
    entry["type"] = type;
    log_entries_.prepend(entry);
    if (log_entries_.size() > 100)
        log_entries_.removeLast();
    emit logEntriesChanged();
}

void CartridgeController::clearLog()
{
    log_entries_.clear();
    emit logEntriesChanged();
}

void CartridgeController::triggerState2()
{
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    done_tray_input_pub_->publish(msg);
    addLog("Trigger STATE 2: done_tray_input published", "ok");
}

void CartridgeController::triggerState4()
{
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    done_tray_output_pub_->publish(msg);
    addLog("Trigger STATE 4: done_tray_output published", "ok");
}

void CartridgeController::s7Acknowledge()
{
    auto msg = std_msgs::msg::String();
    msg.data = "S7_ACK";
    gui_confirm_pub_->publish(msg);
    addLog("S7: Đã xác nhận cấp khay — tiếp tục kiểm tra", "ok");
}
