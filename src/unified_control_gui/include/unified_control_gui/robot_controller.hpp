#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <QObject>
#include <QVariantList>
#include <QTimer>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "dobot_msgs_v3/srv/move_jog.hpp"
#include "dobot_msgs_v3/srv/get_angle.hpp"
#include "dobot_msgs_v3/srv/get_pose.hpp"
#include "dobot_msgs_v3/srv/joint_mov_j.hpp"
#include "dobot_msgs_v3/srv/mov_l.hpp"
#include "dobot_msgs_v3/srv/do.hpp"
#include "dobot_msgs_v3/srv/pause.hpp"
#include "dobot_msgs_v3/srv/clear_error.hpp"
#include "dobot_msgs_v3/srv/reset_robot.hpp"
#include "dobot_msgs_v3/srv/speed_factor.hpp"
#include "dobot_msgs_v3/srv/get_error_id.hpp"
#include <memory>

class RobotController : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString systemStatus READ systemStatus NOTIFY systemStatusChanged)
    Q_PROPERTY(QString errorMessage READ errorMessage NOTIFY errorMessageChanged)
    Q_PROPERTY(int selectedRow READ selectedRow NOTIFY selectedRowChanged)
    Q_PROPERTY(int selectedSlot READ selectedSlot NOTIFY selectedSlotChanged)
    Q_PROPERTY(QString systemUptime READ systemUptime NOTIFY systemUptimeChanged)
    Q_PROPERTY(int trayCount READ trayCount NOTIFY trayCountChanged)
    // Jog control properties
    Q_PROPERTY(QVariantList jointAngles READ jointAngles NOTIFY jointAnglesChanged)
    Q_PROPERTY(QVariantList cartesianPose READ cartesianPose NOTIFY cartesianPoseChanged)
    Q_PROPERTY(int speedRatio READ speedRatio NOTIFY speedRatioChanged)
    Q_PROPERTY(QString errorLog READ errorLog NOTIFY errorLogChanged)

public:
    explicit RobotController(rclcpp::Node::SharedPtr node, QObject *parent = nullptr);
    
    QString systemStatus() const { return system_status_; }
    QString errorMessage() const { return error_message_; }
    int selectedRow() const { return selected_row_; }
    int selectedSlot() const { return selected_slot_; }
    QString systemUptime() const { return system_uptime_; }
    int trayCount() const { return tray_count_; }
    QVariantList jointAngles() const { return joint_angles_; }
    QVariantList cartesianPose() const { return cartesian_pose_; }
    int speedRatio() const { return speed_ratio_; }
    QString errorLog() const { return error_log_; }

public slots:
    // System control
    void enableSystem(bool enable);
    void emergencyStop(bool stop);
    void setManualMode(bool enable);
    void setAiMode(bool enable);
    void switchCamera(int cameraId);
    void selectRow(int row);
    void selectSlot(int slot);
    void gotoState(const QString& state);
    
    // Jog control (Manual mode)
    void jogStart(const QString& axisId);  // "j1+" "j1-" "x+" "z-" etc
    void jogStop();
    void getAngles();
    void getPose();
    
    // Move to exact position
    void moveJoint(double j1, double j2, double j3, double j4, double j5, double j6);
    void moveLinear(double x, double y, double z, double rx, double ry, double rz);
    
    // IO control
    void setDigitalOutput(int index, bool status);
    
    // Robot control
    void pauseRobot();
    void resumeRobot();
    void clearError();
    void setSpeedRatio(int ratio);
    void pollErrorID();
    
    // Simulation triggers
    void simulateFeedChamber();
    void simulateFillDone();

signals:
    void systemStatusChanged();
    void errorMessageChanged();
    void selectedRowChanged();
    void selectedSlotChanged();
    void systemUptimeChanged();
    void trayCountChanged();
    void serviceCallResult(bool success, QString message);
    void jointAnglesChanged();
    void cartesianPoseChanged();
    void speedRatioChanged();
    void errorLogChanged();

private:
    rclcpp::Node::SharedPtr node_;
    
    // Service clients
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr emergency_stop_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr manual_mode_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ai_mode_client_;
    
    // Dobot service clients (jog)
    rclcpp::Client<dobot_msgs_v3::srv::MoveJog>::SharedPtr jog_client_;
    rclcpp::Client<dobot_msgs_v3::srv::GetAngle>::SharedPtr get_angle_client_;
    rclcpp::Client<dobot_msgs_v3::srv::GetPose>::SharedPtr get_pose_client_;
    rclcpp::Client<dobot_msgs_v3::srv::JointMovJ>::SharedPtr joint_movj_client_;
    rclcpp::Client<dobot_msgs_v3::srv::MovL>::SharedPtr movl_client_;
    rclcpp::Client<dobot_msgs_v3::srv::DO>::SharedPtr do_client_;
    rclcpp::Client<dobot_msgs_v3::srv::Pause>::SharedPtr pause_client_;
    rclcpp::Client<dobot_msgs_v3::srv::ClearError>::SharedPtr clear_error_client_;
    rclcpp::Client<dobot_msgs_v3::srv::ResetRobot>::SharedPtr reset_robot_client_;
    rclcpp::Client<dobot_msgs_v3::srv::SpeedFactor>::SharedPtr speed_factor_client_;
    rclcpp::Client<dobot_msgs_v3::srv::GetErrorID>::SharedPtr get_error_id_client_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr camera_select_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr command_row_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr command_slot_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goto_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr feed_chamber_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fill_done_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr error_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr selected_row_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr selected_slot_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_uptime_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tray_count_sub_;
    
    // State
    QString system_status_{"UNKNOWN"};
    QString error_message_;
    int selected_row_{-1};
    int selected_slot_{-1};
    QString system_uptime_{"00:00:00"};
    int tray_count_{0};
    QVariantList joint_angles_;
    QVariantList cartesian_pose_;
    int speed_ratio_{100};
    QString error_log_;
    QTimer *poll_timer_;
    
    void callServiceAsync(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, bool value);
    void pollRobotState();
};

#endif // ROBOT_CONTROLLER_HPP
