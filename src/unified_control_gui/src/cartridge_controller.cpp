#include "unified_control_gui/cartridge_controller.hpp"
#include <QDebug>
#include <QDateTime>
#include <QJsonDocument>
#include <QJsonObject>

CartridgeController::CartridgeController(rclcpp::Node::SharedPtr node, QObject *parent)
    : QObject(parent), node_(node)
{
    auto qos = rclcpp::QoS(10);

    // ── Publishers ───────────────────────────────────────────────
    jog_pub_              = node_->create_publisher<std_msgs::msg::String>("/providesystem/jog_cmd", qos);
    sim_sensor_pub_       = node_->create_publisher<std_msgs::msg::String>("/providesystem/sim_sensor", qos);
    set_mode_pub_         = node_->create_publisher<std_msgs::msg::String>("/providesystem/set_operation_mode", qos);
    goto_state_pub_       = node_->create_publisher<std_msgs::msg::String>("/providesystem/goto_state", qos);
    update_config_pub_    = node_->create_publisher<std_msgs::msg::String>("/providesystem/update_config", qos);
    get_config_pub_       = node_->create_publisher<std_msgs::msg::String>("/providesystem/get_config", qos);
    start_button_pub_     = node_->create_publisher<std_msgs::msg::Bool>("/system/start_button", qos);
    stop_button_pub_      = node_->create_publisher<std_msgs::msg::Bool>("/system/stop_button", qos);
    pause_button_pub_     = node_->create_publisher<std_msgs::msg::Bool>("/system/pause_button", qos);
    gui_confirm_pub_      = node_->create_publisher<std_msgs::msg::String>("/providesystem/gui_confirm", qos);
    set_target_row_pub_   = node_->create_publisher<std_msgs::msg::String>("/providesystem/set_target_row", qos);

    // ── Simulate robot signals (nút STATE 2 / STATE 4 trong GUI) ─
    done_tray_input_pub_  = node_->create_publisher<std_msgs::msg::Bool>("/robot/done_tray_input", qos);
    done_tray_output_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/robot/done_tray_output", qos);

    // ── Subscribers ──────────────────────────────────────────────
    system_state_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/system_state", rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QString raw = QString::fromStdString(msg->data);
            // Format: "global|state_in|state_out"
            QStringList parts = raw.split('|');
            system_state_ = parts.value(0);
            state_in_     = parts.value(1);
            state_out_    = parts.value(2);
            emit systemStateChanged();
        });

    // current_mode từ Python: "jog" | "manual" | "auto"
    current_mode_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/providesystem/current_mode", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QString mode = QString::fromStdString(msg->data);
            if (current_mode_ != mode) {
                current_mode_ = mode;
                emit currentModeChanged();
            }
        });

    config_data_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/providesystem/config_data",
        rclcpp::QoS(10).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            config_data_ = QString::fromStdString(msg->data);
            emit configDataChanged();
        });

    gui_notify_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/providesystem/gui_notify", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            last_notification_ = QString::fromStdString(msg->data);
            QJsonDocument doc  = QJsonDocument::fromJson(last_notification_.toUtf8());
            if (doc.isObject()) {
                QJsonObject obj = doc.object();
                QString level   = obj.value("level").toString("info");
                QString title   = obj.value("title").toString();
                QString detail  = obj.value("detail").toString();
                QString logMsg  = detail.isEmpty() ? title : title + " — " + detail;
                QString type    = (level == "error" || level == "warn") ? "err" : "info";
                addLog(logMsg, type);
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

    qDebug() << "CartridgeController v8 initialized";
}

// ── Helpers ───────────────────────────────────────────────────────

void CartridgeController::publishString(
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub,
    const QString &data)
{
    auto msg = std_msgs::msg::String();
    msg.data = data.toStdString();
    pub->publish(msg);
}

void CartridgeController::publishBool(
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub,
    bool value)
{
    auto msg = std_msgs::msg::Bool();
    msg.data = value;
    pub->publish(msg);
}

// ── Servo JOG ─────────────────────────────────────────────────────

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
    addLog(QString("Home servo %1").arg(id), "info");
}

void CartridgeController::clearServo(int id)
{
    publishString(jog_pub_, QString("clear %1").arg(id));
    addLog(QString("Clear fault servo %1").arg(id), "info");
}

void CartridgeController::moveServo(int id, double position)
{
    publishString(jog_pub_, QString("%1 move %2").arg(id).arg(position));
    addLog(QString("Move S%1 → %2mm").arg(id).arg(position), "ok");
}

// ── System Control ────────────────────────────────────────────────

void CartridgeController::setMode(const QString &mode)
{
    publishString(set_mode_pub_, mode);
    addLog(QString("Mode: %1").arg(mode.toUpper()), "ok");
    // current_mode_ sẽ được cập nhật qua subscriber /providesystem/current_mode
}

void CartridgeController::gotoState(const QString &state)
{
    publishString(goto_state_pub_, state);
    addLog(QString("Goto state: %1").arg(state), "info");
}

void CartridgeController::setTargetRow(int row)
{
    publishString(set_target_row_pub_, QString::number(row));
    addLog(QString("Set Target Row: %1").arg(row), "ok");
}

void CartridgeController::startSystem()
{
    publishBool(start_button_pub_, true);
    addLog("System START → Homing...", "ok");
}

void CartridgeController::stopSystem()
{
    publishBool(stop_button_pub_, true);
    addLog("System STOP", "err");
}

void CartridgeController::pauseSystem()
{
    publishBool(pause_button_pub_, true);
    addLog("System PAUSE", "info");
}

void CartridgeController::hmiResume()
{
    // Không dùng trong v8, giữ để tương thích
    addLog("HMI Resume (no-op in v8)", "info");
}

void CartridgeController::resetFaults()
{
    // Gửi ABORT_TO_JOG để về JOG an toàn
    publishString(goto_state_pub_, "ABORT_TO_JOG");
    addLog("Reset faults → ABORT_TO_JOG", "info");
}

void CartridgeController::abortToJog()
{
    publishString(goto_state_pub_, "ABORT_TO_JOG");
    addLog("ABORT → JOG mode", "info");
}

// ── Robot signal simulation (nút STATE 2 / STATE 4) ───────────────

void CartridgeController::simulateDoneTrayInput()
{
    publishBool(done_tray_input_pub_, true);
    addLog("Simulate: done_tray_input (trigger State 2)", "info");
}

void CartridgeController::simulateDoneTrayOutput()
{
    publishBool(done_tray_output_pub_, true);
    addLog("Simulate: done_tray_output (trigger State 4)", "info");
}

void CartridgeController::confirmOutput()
{
    // Giữ để tương thích QML cũ
    simulateDoneTrayOutput();
}

void CartridgeController::s11Respond(bool ok)
{
    publishString(gui_confirm_pub_, ok ? "s11_confirm" : "s11_cancel");
    addLog(QString("S11 Response: %1").arg(ok ? "CONFIRM" : "CANCEL"), "info");
}

// ── Sensor Simulation ─────────────────────────────────────────────

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

// ── Config ────────────────────────────────────────────────────────

void CartridgeController::getConfig()
{
    publishString(get_config_pub_, "request");
}

void CartridgeController::saveConfig(const QString &key, const QString &jsonData)
{
    QJsonObject obj;
    obj["key"]  = key;
    obj["data"] = jsonData;
    QString payload = QString::fromUtf8(
        QJsonDocument(obj).toJson(QJsonDocument::Compact));
    publishString(update_config_pub_, payload);
    addLog(QString("Config saved: %1").arg(key), "ok");
}

// ── Log ───────────────────────────────────────────────────────────

void CartridgeController::addLog(const QString &msg, const QString &type)
{
    QVariantMap entry;
    entry["time"] = QDateTime::currentDateTime().toString("HH:mm:ss");
    entry["msg"]  = msg;
    entry["type"] = type;
    log_entries_.prepend(entry);
    if (log_entries_.size() > 200)
        log_entries_.removeLast();
    emit logEntriesChanged();
}

void CartridgeController::clearLog()
{
    log_entries_.clear();
    emit logEntriesChanged();
}