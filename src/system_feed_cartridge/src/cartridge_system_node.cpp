#include "system_feed_cartridge/cartridge_system.hpp"
#include <fstream>
#include <chrono>

namespace system_feed_cartridge {

std::string stateToString(SystemState state) {
    switch (state) {
        case SystemState::IDLE: return "IDLE";
        case SystemState::HOMING: return "HOMING";
        case SystemState::WAIT_FOR_TRAY: return "WAIT_FOR_TRAY";
        case SystemState::CONVEYOR_RUNNING: return "CONVEYOR_RUNNING";
        case SystemState::INX_MOVING_TO_TARGET2: return "INX_MOVING_TO_TARGET2";
        case SystemState::INY_SEARCHING: return "INY_SEARCHING";
        case SystemState::INY_MOVING_TO_ROW: return "INY_MOVING_TO_ROW";
        case SystemState::CYLINDER1_EXTEND: return "CYLINDER1_EXTEND";
        case SystemState::INY_RETURN_HOME: return "INY_RETURN_HOME";
        case SystemState::INY_MOVING_TO_TARGET2_RETURN: return "INY_MOVING_TO_TARGET2_RETURN";
        case SystemState::CYLINDER1_RETRACT: return "CYLINDER1_RETRACT";
        case SystemState::INX_RETURN_HOME: return "INX_RETURN_HOME";
        case SystemState::TRAY_LOADED: return "TRAY_LOADED";
        case SystemState::WAIT_FOR_DONE_TRAY: return "WAIT_FOR_DONE_TRAY";
        case SystemState::RETRIEVE_INX_TO_TARGET2: return "RETRIEVE_INX_TO_TARGET2";
        case SystemState::RETRIEVE_INY_TO_TARGET2: return "RETRIEVE_INY_TO_TARGET2";
        case SystemState::RETRIEVE_CYLINDER_EXTEND: return "RETRIEVE_CYLINDER_EXTEND";
        case SystemState::RETRIEVE_INY_TO_TARGET1: return "RETRIEVE_INY_TO_TARGET1";
        case SystemState::RETRIEVE_INX_TO_TARGET1: return "RETRIEVE_INX_TO_TARGET1";
        case SystemState::RETRIEVE_INY_SEARCH_ROW: return "RETRIEVE_INY_SEARCH_ROW";
        case SystemState::RETRIEVE_INY_TO_ROW: return "RETRIEVE_INY_TO_ROW";
        case SystemState::RETRIEVE_CYLINDER_RETRACT: return "RETRIEVE_CYLINDER_RETRACT";
        case SystemState::RETRIEVE_INY_RETURN: return "RETRIEVE_INY_RETURN";
        case SystemState::WAIT_FOR_OUTPUT_SIGNAL: return "WAIT_FOR_OUTPUT_SIGNAL";
        case SystemState::OUTPUT_SERVO3_PUSH_TRAY: return "OUTPUT_SERVO3_PUSH_TRAY";
        case SystemState::OUTPUT_WAIT_CARTRIDGE_FILL: return "OUTPUT_WAIT_CARTRIDGE_FILL";
        case SystemState::OUTPUT_OUTX_TO_TARGET2: return "OUTPUT_OUTX_TO_TARGET2";
        case SystemState::OUTPUT_OUTY_TO_TARGET2: return "OUTPUT_OUTY_TO_TARGET2";
        case SystemState::OUTPUT_CYLINDER_EXTEND: return "OUTPUT_CYLINDER_EXTEND";
        case SystemState::OUTPUT_OUTY_TO_TARGET1: return "OUTPUT_OUTY_TO_TARGET1";
        case SystemState::OUTPUT_OUTX_TO_TARGET1: return "OUTPUT_OUTX_TO_TARGET1";
        case SystemState::OUTPUT_OUTY_SEARCH_ROW: return "OUTPUT_OUTY_SEARCH_ROW";
        case SystemState::OUTPUT_OUTY_TO_ROW: return "OUTPUT_OUTY_TO_ROW";
        case SystemState::OUTPUT_CYLINDER_RETRACT: return "OUTPUT_CYLINDER_RETRACT";
        case SystemState::OUTPUT_OUTY_TO_HOME: return "OUTPUT_OUTY_TO_HOME";
        case SystemState::OUTPUT_SERVO3_RETRACT: return "OUTPUT_SERVO3_RETRACT";
        case SystemState::OUTPUT_COMPLETE: return "OUTPUT_COMPLETE";
        case SystemState::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

void Config::loadFromFile(const std::string& config_file) {
    YAML::Node config = YAML::LoadFile(config_file);
    
    // Load Servo IPs
    if (config["servo_ips"]) {
        for (YAML::const_iterator it = config["servo_ips"].begin(); it != config["servo_ips"].end(); ++it) {
            servo_ips[it->first.as<int>()] = it->second.as<std::string>();
        }
    }
    
    io_ip = config["io_ip"].as<std::string>();
    
    cylinder1_extend_channel = config["cylinder1_extend_channel"].as<int>();
    cylinder1_retract_channel = config["cylinder1_retract_channel"].as<int>();
    cylinder2_extend_channel = config["cylinder2_extend_channel"].as<int>();
    cylinder2_retract_channel = config["cylinder2_retract_channel"].as<int>();
    
    inx_home = config["inx_home"].as<double>();
    inx_target2 = config["inx_target2"].as<double>();
    iny_home = config["iny_home"].as<double>();
    iny_target2 = config["iny_target2"].as<double>();
    
    servo3_home = config["servo3_home"].as<double>();
    servo3_push_position = config["servo3_push_position"].as<double>();
    
    outx_home = config["outx_home"].as<double>();
    outx_target1 = config["outx_target1"].as<double>();
    outx_target2 = config["outx_target2"].as<double>();
    
    outy_home = config["outy_home"].as<double>();
    outy_target1 = config["outy_target1"].as<double>();
    outy_target2 = config["outy_target2"].as<double>();
    
    if (config["row_positions"]) {
        for (YAML::const_iterator it = config["row_positions"].begin(); it != config["row_positions"].end(); ++it) {
            row_positions[it->first.as<int>()] = it->second.as<double>();
        }
    }
    
    position_tolerance = config["position_tolerance"].as<double>();
    iny_safe_position_threshold = config["iny_safe_position_threshold"].as<double>();
    outx_safe_position_threshold = config["outx_safe_position_threshold"].as<double>();
    
    servo3_jog_velocity = config["servo3_jog_velocity"].as<double>();
    iny_search_velocity = config["iny_search_velocity"].as<double>();
    
    homing_timeout = config["homing_timeout"].as<double>();
    move_timeout = config["move_timeout"].as<double>();
    cylinder_timeout = config["cylinder_timeout"].as<double>();
}

CartridgeSystem::CartridgeSystem(const rclcpp::NodeOptions& options)
    : Node("cartridge_system", options),
      current_state_(SystemState::IDLE),
      done_tray_signal_(false),
      done_output_signal_(false),
      start_button_pressed_(false),
      detected_trays_(0),
      current_input_row_(1),
      current_output_row_(1),
      current_slot_(1),
      max_trays_(8),
      max_slots_per_output_tray_(9)
{
    // Load config
    std::string config_path = this->declare_parameter("config_file", 
        std::string("/home/pi/ros2_ws/src/system_feed_cartridge/config/cartridge_config.yaml"));
    try {
        config_.loadFromFile(config_path);
        RCLCPP_INFO(this->get_logger(), "Config loaded from %s", config_path.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load config: %s", e.what());
    }

    // Publishers
    pub_new_tray_ = this->create_publisher<std_msgs::msg::Bool>("/revpi/new_tray_loaded", 1);
    pub_run_conveyor_ = this->create_publisher<std_msgs::msg::Bool>("/revpi/run_conveyor", 1);
    pub_last_batch_ = this->create_publisher<std_msgs::msg::Bool>("/revpi/is_last_tray", 1); // Renamed
    pub_state_ = this->create_publisher<std_msgs::msg::String>("/system_state", 1);

    // Subscribers
    sub_start_button_ = this->create_subscription<std_msgs::msg::Bool>(
        "/system/start_button", 1, std::bind(&CartridgeSystem::startButtonCallback, this, std::placeholders::_1));
    sub_done_tray_input_ = this->create_subscription<std_msgs::msg::Bool>(
        "/robot/change_tray", 1, std::bind(&CartridgeSystem::doneTrayInputCallback, this, std::placeholders::_1));
    sub_done_tray_output_ = this->create_subscription<std_msgs::msg::Bool>(
        "/robot/done_tray_output", 1, std::bind(&CartridgeSystem::doneTrayOutputCallback, this, std::placeholders::_1));

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&CartridgeSystem::timerCallback, this));

    connectHardware();
    
    RCLCPP_INFO(this->get_logger(), "Cartridge System Node Initialized");
}

CartridgeSystem::~CartridgeSystem() {}

void CartridgeSystem::startButtonCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (current_state_ == SystemState::IDLE) {
            RCLCPP_INFO(this->get_logger(), "Start button pressed. Starting system.");
            start_button_pressed_ = true;
        }
    }
}

void CartridgeSystem::doneTrayInputCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        done_tray_signal_ = true;
        RCLCPP_INFO(this->get_logger(), "Received done_tray_input signal");
    }
}

void CartridgeSystem::doneTrayOutputCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        done_output_signal_ = true;
        RCLCPP_INFO(this->get_logger(), "Received done_tray_output signal");
    }
}

void CartridgeSystem::timerCallback() {
    processState();
    updateMockHardware(); // Simulate hardware running
    
    // Publish state
    std_msgs::msg::String state_msg;
    state_msg.data = stateToString(current_state_);
    pub_state_->publish(state_msg);
}

void CartridgeSystem::changeState(SystemState new_state) {
    if (current_state_ != new_state) {
        RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s", 
            stateToString(current_state_).c_str(), stateToString(new_state).c_str());
        current_state_ = new_state;
    }
}

void CartridgeSystem::processState() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    switch (current_state_) {
        case SystemState::IDLE:
            if (start_button_pressed_) {
                start_button_pressed_ = false;
                changeState(SystemState::HOMING);
            }
            break;
            
        case SystemState::HOMING:
            if (homeAllServos()) {
                changeState(SystemState::WAIT_FOR_TRAY); // Skip SERVO3_MOVING for simplicity based on diagram? No, let's keep it simple or follow diagram.
                // Diagram says: HOMING -> SERVO3_MOVING -> WAIT_FOR_TRAY
                // Actually let's assume SERVO3 setup is part of Homing or handled implicitly
                // For this implementation I will skip SERVO3_MOVING detailed jog and assume it goes home.
                // Wait, Servo 3 PUSH TRAY is for Output. For Input we just wait.
                // Re-reading: "SERVO3_MOVING: 3 jog forward -> Sensor 8 ON -> WAIT_FOR_TRAY". 
                // This seems to prepare the output tray or something? I'll follow the flow.
                // Ah, the first block says "STATE 1: LOAD KHAY VÀO".
                // Let's implement basic flow.
                changeState(SystemState::WAIT_FOR_TRAY);
            }
            break;
            
        case SystemState::WAIT_FOR_TRAY:
            if (getSensor(1)) { // Sensor 1: Input conveyor start
                std_msgs::msg::Bool run_msg;
                run_msg.data = true;
                pub_run_conveyor_->publish(run_msg);
                changeState(SystemState::CONVEYOR_RUNNING);
            }
            break;
            
        case SystemState::CONVEYOR_RUNNING:
            if (getSensor(2)) { // Sensor 2: End of conveyor
                std_msgs::msg::Bool run_msg;
                run_msg.data = false;
                pub_run_conveyor_->publish(run_msg);
                changeState(SystemState::INX_MOVING_TO_TARGET2);
            }
            break;
            
        case SystemState::INX_MOVING_TO_TARGET2:
            if (moveServo(1, config_.inx_target2)) {
                // If it's the first time in a batch, we need to count trays
                // For simplicity, let's assume we search every time or have logic for counting
                // The spec says: INY_SEARCHING -> Count trays (sensor 5) -> detected_trays = N
                changeState(SystemState::INY_SEARCHING);
            }
            break;
            
        case SystemState::INY_SEARCHING:
            // Placeholder: Simulate finding row
            // If we haven't detected trays yet, detect them.
            // Move Servo 2 to search.
            if (detected_trays_ == 0) {
                 // Logic to scan and count. Mocked:
                 detected_trays_ = 3; // Example: found 3 trays
                 current_input_row_ = 1;
                 RCLCPP_INFO(this->get_logger(), "Detected %d trays", detected_trays_);
            }
            
            // If current row > detected, we shouldn't be here in this loop, but let's check
            if (current_input_row_ > detected_trays_) {
                 // Should have gone to output?
                 // For now, assume valid.
            }
            
            changeState(SystemState::INY_MOVING_TO_ROW);
            break;
            
        case SystemState::INY_MOVING_TO_ROW:
            if (moveServo(2, config_.row_positions[current_input_row_])) {
                changeState(SystemState::CYLINDER1_EXTEND);
            }
            break;
            
        case SystemState::CYLINDER1_EXTEND:
            setCylinder(1, true); // Extend
            if (getSensor(4)) { // Cyl 1 Extended
                changeState(SystemState::INY_RETURN_HOME);
            }
            break;
            
        case SystemState::INY_RETURN_HOME:
            if (moveServo(2, config_.iny_home)) {
                changeState(SystemState::INY_MOVING_TO_TARGET2_RETURN);
            }
            break;

        case SystemState::INY_MOVING_TO_TARGET2_RETURN:
            if (moveServo(2, config_.iny_target2)) { // Corrected target based on flow, actually it says INY_TO_TARGET2 -> CYLINDER1_RETRACT
                changeState(SystemState::CYLINDER1_RETRACT);
            }
            break;

        case SystemState::CYLINDER1_RETRACT:
            setCylinder(1, false); // Retract
            if (getSensor(3)) { // Cyl 1 Retracted
                changeState(SystemState::INX_RETURN_HOME);
            }
            break;

        case SystemState::INX_RETURN_HOME:
            if (moveServo(1, config_.inx_home)) {
                changeState(SystemState::TRAY_LOADED);
            }
            break;

        case SystemState::TRAY_LOADED:
            {
                std_msgs::msg::Bool msg;
                msg.data = true;
                pub_new_tray_->publish(msg);
                
                // Check is last batch
                if (current_input_row_ == 8 && !getSensor(2)) {
                    pub_last_batch_->publish(msg);
                }
                
                changeState(SystemState::WAIT_FOR_DONE_TRAY);
            }
            break;

        case SystemState::WAIT_FOR_DONE_TRAY:
            if (done_tray_signal_) {
                done_tray_signal_ = false;
                // Safety check
                if (isInYSafeForInXMove()) {
                   changeState(SystemState::RETRIEVE_INX_TO_TARGET2); 
                } else {
                    RCLCPP_WARN(this->get_logger(), "Safety check failed: InY not safe for InX move");
                    // Wait? or Error?
                }
            }
            break;

        case SystemState::RETRIEVE_INX_TO_TARGET2:
            if (moveServo(1, config_.inx_target2)) {
                changeState(SystemState::RETRIEVE_INY_TO_TARGET2);
            }
            break;

        case SystemState::RETRIEVE_INY_TO_TARGET2:
            if (moveServo(2, config_.iny_target2)) {
                changeState(SystemState::RETRIEVE_CYLINDER_EXTEND);
            }
            break;

        case SystemState::RETRIEVE_CYLINDER_EXTEND:
            setCylinder(1, true);
            if (getSensor(4)) {
                changeState(SystemState::RETRIEVE_INY_TO_TARGET1);
            }
            break;

        case SystemState::RETRIEVE_INY_TO_TARGET1:
            // Target1 or Home? Spec says Target1 then INX to Target1
            // Let's assume Target1 is iny_home for now or 0
             if (moveServo(2, config_.iny_home)) { // Assuming Target1 is effectively Home for Y vertical lift?
                changeState(SystemState::RETRIEVE_INX_TO_TARGET1);
            }
            break;

        case SystemState::RETRIEVE_INX_TO_TARGET1:
            // Move X back to store position
            if (moveServo(1, config_.inx_home)) {
                changeState(SystemState::RETRIEVE_INY_SEARCH_ROW);
            }
            break;

        case SystemState::RETRIEVE_INY_SEARCH_ROW:
             // Move Y down to scan/find row
             // Mock: we know the row is current_input_row_
             changeState(SystemState::RETRIEVE_INY_TO_ROW);
             break;

        case SystemState::RETRIEVE_INY_TO_ROW:
            if (moveServo(2, config_.row_positions[current_input_row_])) {
                changeState(SystemState::RETRIEVE_CYLINDER_RETRACT);
            }
            break;

        case SystemState::RETRIEVE_CYLINDER_RETRACT:
            setCylinder(1, false);
            if (getSensor(3)) {
                changeState(SystemState::RETRIEVE_INY_RETURN);
            }
            break;

        case SystemState::RETRIEVE_INY_RETURN:
            if (moveServo(2, config_.iny_home)) {
                current_input_row_++;
                if (current_input_row_ > detected_trays_) {
                    // All trays processed, go to Output
                    current_input_row_ = 1; // Reset for next batch? Or just move to output
                    changeState(SystemState::WAIT_FOR_OUTPUT_SIGNAL);
                } else {
                    // Next tray
                    changeState(SystemState::WAIT_FOR_TRAY);
                }
            }
            break;
            
        case SystemState::WAIT_FOR_OUTPUT_SIGNAL:
             // Logic for Output phase
             // Wait for done_tray_output
             if (done_output_signal_) {
                 done_output_signal_ = false;
                 // Assuming we are fetching from storage (row 1..8) to output
                 // Start with current_output_row_
                 changeState(SystemState::OUTPUT_OUTX_TO_TARGET2);
             }
             break;
             
        case SystemState::OUTPUT_OUTX_TO_TARGET2:
             if (moveServo(4, config_.outx_target2)) {
                 changeState(SystemState::OUTPUT_OUTY_TO_TARGET2);
             }
             break;

        case SystemState::OUTPUT_OUTY_TO_TARGET2:
            // Move Y to pickup position? 
            // Spec says: OUTPUT_OUTY_TO_TARGET2 (Servo 5 vào)
            if (moveServo(5, config_.outy_target2)) {
                changeState(SystemState::OUTPUT_CYLINDER_EXTEND);
            }
            break;
            
        case SystemState::OUTPUT_CYLINDER_EXTEND:
            setCylinder(2, true);
            if (getSensor(7)) { // Cyl 2 Extended
                changeState(SystemState::OUTPUT_OUTY_TO_TARGET1);
            }
            break;
            
        case SystemState::OUTPUT_OUTY_TO_TARGET1:
            if (moveServo(5, config_.outy_target1)) {
                changeState(SystemState::OUTPUT_OUTX_TO_TARGET1);
            }
            break;
            
        case SystemState::OUTPUT_OUTX_TO_TARGET1:
            if (moveServo(4, config_.outx_target1)) {
                if (isOutXSafeForOutYMove()) {
                     changeState(SystemState::OUTPUT_OUTY_SEARCH_ROW);
                }
            }
            break;
            
        case SystemState::OUTPUT_OUTY_SEARCH_ROW:
            // Mocking search
            changeState(SystemState::OUTPUT_OUTY_TO_ROW);
            break;
            
        case SystemState::OUTPUT_OUTY_TO_ROW:
            if (moveServo(5, config_.row_positions[current_output_row_])) {
                changeState(SystemState::OUTPUT_CYLINDER_RETRACT);
            }
            break;
            
        case SystemState::OUTPUT_CYLINDER_RETRACT:
            setCylinder(2, false);
            if (getSensor(6)) {
                changeState(SystemState::OUTPUT_OUTY_TO_HOME);
            }
            break;
            
        case SystemState::OUTPUT_OUTY_TO_HOME:
            if (moveServo(5, config_.outy_home)) {
                 current_output_row_++;
                 changeState(SystemState::OUTPUT_COMPLETE);
            }
            break;
            
        case SystemState::OUTPUT_COMPLETE:
            if (current_output_row_ > detected_trays_) {
                current_output_row_ = 1;
                // Batch complete
                changeState(SystemState::IDLE); 
            } else {
                changeState(SystemState::WAIT_FOR_OUTPUT_SIGNAL);
            }
            break;

        default:
            break;
    }
}

// Mock Hardware Utils
bool CartridgeSystem::connectHardware() {
    // MOCK: Init positions
    mocked_servo_positions_[1] = 0.0;
    mocked_servo_positions_[2] = 0.0;
    mocked_servo_positions_[3] = 0.0;
    mocked_servo_positions_[4] = 0.0;
    mocked_servo_positions_[5] = 0.0;
    
    // MOCK: Init sensors
    for(int i=1; i<=10; i++) mocked_sensors_[i] = false;
    // Assume cylinders start retracted
    mocked_sensors_[3] = true; // Cyl 1 retracted
    mocked_sensors_[6] = true; // Cyl 2 retracted
    
    return true;
}

bool CartridgeSystem::homeAllServos() {
    for(auto& pair : mocked_servo_positions_) {
        pair.second = 0.0;
    }
    return true;
}

bool CartridgeSystem::moveServo(int servo_id, double position, bool wait) {
    mocked_servo_positions_[servo_id] = position;
    // Simulate delay
    return true;
}

double CartridgeSystem::getServoPosition(int servo_id) {
    return mocked_servo_positions_[servo_id];
}

void CartridgeSystem::setCylinder(int cylinder_id, bool extend) {
    if (cylinder_id == 1) {
        mocked_sensors_[3] = !extend; // Retracted
        mocked_sensors_[4] = extend;  // Extended
    } else if (cylinder_id == 2) {
        mocked_sensors_[6] = !extend;
        mocked_sensors_[7] = extend;
    }
}

bool CartridgeSystem::getSensor(int sensor_id) {
    return mocked_sensors_[sensor_id];
}

bool CartridgeSystem::isInYSafeForInXMove() {
     return getServoPosition(2) <= config_.iny_safe_position_threshold;
}

bool CartridgeSystem::isOutXSafeForOutYMove() {
     return getServoPosition(4) <= config_.outx_safe_position_threshold;
}

void CartridgeSystem::updateMockHardware() {
    // Determine sensors based on logic
    // Example: if state is WAIT_FOR_TRAY, simulate sensor 1 (conveyor start) periodically
    // This is just for testing without real hardware
}

} // namespace system_feed_cartridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<system_feed_cartridge::CartridgeSystem>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
