#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <cerrno>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

extern "C" {
#include <modbus/modbus.h>
}

using namespace std::chrono_literals;

// =========================================================
//  RS485BusNode  —  RevPi A
//
//  TOPICS PUBLISH:
//    /loadcell/weight              Float32   Net weight sau tare + moving average (g), 2 Hz
//    /loadcell/raw_weight          Float32   Raw weight chua filter/tare (g), 2 Hz
//    /loadcell/status              String    "OK" | "ERROR"
//    /loadcell/tare_ack            Bool      true khi tare/reset hoan thanh
//    /loadcell/overload            Bool      true khi vuot max_weight_g (tu dong, lap moi chu ky)
//    /loadcell/zero_drift_warning  Bool      true khi drift phat hien (tu dong)
//    /loadcell/consecutive_fails   Int32     So lan fail lien tiep hien tai
//    /loadcell/batch_stats         String    JSON: {"total":N,"pass":N,"fail":N,"consec_fail":N}
//    /loadcell/cal_status          String    "IDLE"|"WAITING_WEIGHT"|"DONE"|"ERROR"
//    /loadcell/signal_process      Bool      Pass/Fail forwarded to robot_logic_node
//    /weight/monitor_status        String    "IDLE"|"MEASURING"|"PASS"|"FAIL" for GUI
//    /vfd/freq_fb                  Float32   Tan so VFD (Hz)
//    /vfd/state                    Int32     0=disabled 1=enabled 2=running
//
//  TOPICS SUBSCRIBE:
//    /loadcell/tare_cmd            Bool      true -> set tare offset = raw hien tai
//    /loadcell/tare_reset          Bool      true -> xoa tare offset ve 0
//    /loadcell/overload_ack        Bool      true -> GUI acknowledge overload, reset co
//    /loadcell/batch_reset         Bool      true -> reset batch statistics
//    /loadcell/target_min          Float32   Target minimum weight from GUI
//    /loadcell/target_max          Float32   Target maximum weight from GUI
//    /loadcell/target_weight       Float32   Target batch weight from GUI
//    /loadcell/cal_weight          Float32   Known calibration weight from GUI
//
//  SERVICES (goi tu GUI):
//    /loadcell/cal_start           Trigger   Buoc 1: can trong, ghi zero raw
//    /loadcell/cal_set_known       Trigger   Buoc 2-5: dat qua can chuan (5 diem, goi lap lai 4 lan)
//                                            cal_status: "WAITING_WEIGHT" | "CONTINUE_CAL_N/5" | "DONE" | "ERROR"
// =========================================================

static constexpr float DEAD_ZONE_G       = 0.5f;
static constexpr int   MA_WINDOW         = 5;
static constexpr float DRIFT_EPSILON_G   = 1.0f;
static constexpr int   DRIFT_HOLD_CYCLES = 6;    // 6 x 500ms = 3 giay lien tuc
static constexpr int   MAX_CONSEC_FAIL   = 3;

class RS485BusNode : public rclcpp::Node
{
  enum class CalState { IDLE, WAITING_WEIGHT, DONE };

public:
  RS485BusNode()
  : Node("rs485_bus"),
    tare_offset_(0.0f),
    last_raw_weight_(0.0f),
    connection_ok_(false),
    overload_active_(false),
    drift_cycles_(0),
    consec_fail_(0),
    batch_total_(0), batch_pass_(0), batch_fail_(0),
    cal_state_(CalState::IDLE),
    cal_point_count_(0),
    ad_avg_1_(0), ad_avg_2_(0),
    cal_known_weight_g_(500.0),
    target_batch_weight_(0.0f), target_min_weight_(0.0f), target_max_weight_(0.0f),
    monitor_state_("IDLE"),
    evaluating_cartridge_(false),
    waiting_for_removal_(false),
    stable_eval_cycles_(0),
    eval_last_weight_(0.0f),
    running_(false)
  {
    // ── VFD ──────────────────────────────────────────────
    REG_CMD_     = 8501;  REG_LFR_     = 8502;
    CMD_DISABLE_ = 0x0000; CMD_ENABLE_ = 0x0006; CMD_RUN_FWD_ = 0x000F;

    slave_id_  = declare_parameter<int>("slave_id",    1);
    ref_hz_    = declare_parameter<double>("ref_hz",   30.0);
    ref_scale_ = declare_parameter<double>("ref_scale",10.0);

    // ── Loadcell ─────────────────────────────────────────
    loadcell_slave_id_  = declare_parameter<int>("loadcell_slave_id",     2);
    loadcell_reg_       = declare_parameter<int>("loadcell_reg",          0);
    loadcell_scale_     = declare_parameter<double>("loadcell_scale",     0.01);
    loadcell_is_32bit_  = declare_parameter<bool>("loadcell_is_32bit",    false);
    loadcell_period_ms_ = declare_parameter<int>("loadcell_period_ms",    500);
    max_weight_g_       = declare_parameter<double>("max_weight_g",       5000.0);
    cal_known_weight_g_ = declare_parameter<double>("cal_known_weight_g", 500.0);
    config_path_        = declare_parameter<std::string>(
                            "config_path", "/home/pi/loadcell_cal.cfg");

    load_scale_from_file();

    // ── RS485 ────────────────────────────────────────────
    std::string port     = declare_parameter<std::string>("port",       "/dev/ttyRS485");
    int baudrate         = declare_parameter<int>("baudrate",            9600);
    std::string par_str  = declare_parameter<std::string>("parity",     "N");
    int stopbits         = declare_parameter<int>("stopbits",            1);
    int bytesize         = declare_parameter<int>("bytesize",            8);
    double timeout_s     = declare_parameter<double>("timeout",          1.0);
    double turnaround_ms = declare_parameter<double>("turnaround_ms",    5.0);
    turnaround_          = turnaround_ms / 1000.0;
    char parity          = par_str.empty() ? 'N' : static_cast<char>(par_str[0]);

    // ── Modbus init ──────────────────────────────────────
    ctx_ = modbus_new_rtu(port.c_str(), baudrate, parity, bytesize, stopbits);
    if (!ctx_) {
      RCLCPP_ERROR(get_logger(), "[RS485] modbus_new_rtu failed");
      return;
    }
    modbus_set_response_timeout(ctx_,
      static_cast<uint32_t>(timeout_s),
      static_cast<uint32_t>((timeout_s - static_cast<int>(timeout_s)) * 1e6));

    if (modbus_connect(ctx_) == -1) {
      RCLCPP_ERROR(get_logger(), "[RS485] Cannot connect: %s", modbus_strerror(errno));
      modbus_free(ctx_);
      ctx_ = nullptr;
      return;
    }
    connection_ok_ = true;
    RCLCPP_INFO(get_logger(), "[RS485] Connected  port=%s  baud=%d  scale=%.6f",
                port.c_str(), baudrate, loadcell_scale_);

    // ── Publishers ───────────────────────────────────────
    pub_weight_          = create_publisher<std_msgs::msg::Float32>("/loadcell/weight",             10);
    pub_raw_weight_      = create_publisher<std_msgs::msg::Float32>("/loadcell/raw_weight",         10);
    pub_status_          = create_publisher<std_msgs::msg::String> ("/loadcell/status",             10);
    pub_tare_ack_        = create_publisher<std_msgs::msg::Bool>   ("/loadcell/tare_ack",           10);
    pub_overload_        = create_publisher<std_msgs::msg::Bool>   ("/loadcell/overload",           10);
    pub_drift_warn_      = create_publisher<std_msgs::msg::Bool>   ("/loadcell/zero_drift_warning", 10);
    pub_consec_fail_     = create_publisher<std_msgs::msg::Int32>  ("/loadcell/consecutive_fails",  10);
    pub_batch_stats_     = create_publisher<std_msgs::msg::String> ("/loadcell/batch_stats",        10);
    pub_cal_status_      = create_publisher<std_msgs::msg::String> ("/loadcell/cal_status",         10);
    pub_signal_process_  = create_publisher<std_msgs::msg::Bool>   ("/loadcell/signal_process",     10);
    pub_monitor_status_  = create_publisher<std_msgs::msg::String> ("/weight/monitor_status",       10);
    pub_freq_            = create_publisher<std_msgs::msg::Float32>("/vfd/freq_fb",                 10);
    pub_vfd_state_       = create_publisher<std_msgs::msg::Int32>  ("/vfd/state",                   10);
    pub_ink_cap_ack_     = create_publisher<std_msgs::msg::Float32>("/Fill_HP1/ink_capacity_ack",   10);

    // ── Subscribers ──────────────────────────────────────

    sub_tare_cmd_ = create_subscription<std_msgs::msg::Bool>(
      "/loadcell/tare_cmd", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) return;
        // Per RW-ST01D datasheet: write current raw reading into Zero Value register
        // (0x0004-0x0005) for persistent hardware tare (survives power cycle).
        // Software offset reset to 0 since hardware will subtract the zero internally.
        int32_t current_raw = static_cast<int32_t>(last_raw_weight_);
        if (write_2reg(0x0004, current_raw, loadcell_slave_id_)) {
          tare_offset_ = 0.0f; // hardware handles tare from now on
          RCLCPP_INFO(get_logger(),
            "[TARE] Hardware zero set: raw=%d written to reg 0x0004", current_raw);
        } else {
          // Fallback: software tare if hardware write fails
          tare_offset_ = last_raw_weight_;
          RCLCPP_WARN(get_logger(),
            "[TARE] Hardware write failed, using software tare: offset=%.2f g", tare_offset_);
        }
        pub_bool(pub_tare_ack_, true);
      });

    sub_tare_reset_ = create_subscription<std_msgs::msg::Bool>(
      "/loadcell/tare_reset", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) return;
        // Clear hardware zero register (write 0 to 0x0004-0x0005)
        write_2reg(0x0004, 0, loadcell_slave_id_);
        tare_offset_ = 0.0f;
        RCLCPP_INFO(get_logger(), "[TARE] Hardware zero cleared (reg 0x0004 = 0)");
        pub_bool(pub_tare_ack_, true);
      });

    sub_overload_ack_ = create_subscription<std_msgs::msg::Bool>(
      "/loadcell/overload_ack", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) return;
        overload_active_ = false;
        RCLCPP_INFO(get_logger(), "[OVERLOAD] Acknowledged, reset");
        pub_bool(pub_overload_, false);
      });

    sub_batch_reset_ = create_subscription<std_msgs::msg::Bool>(
      "/loadcell/batch_reset", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) return;
        batch_total_ = batch_pass_ = batch_fail_ = consec_fail_ = 0;
        RCLCPP_INFO(get_logger(), "[BATCH] Statistics reset");
        pub_batch_stats();
        pub_int(pub_consec_fail_, 0);
      });

    // GUI pub /loadcell/target_weight khi operator bam APPLY TARGET
    sub_target_weight_ = create_subscription<std_msgs::msg::Float32>(
      "/loadcell/target_weight", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        target_batch_weight_ = msg->data;
        RCLCPP_INFO(get_logger(),
          "[TARGET] Received target_batch_weight=%.2f g from GUI", target_batch_weight_);
      });

    // ── Gating Evaluation Logic from Robot System Status ──
    sub_robot_status_ = create_subscription<std_msgs::msg::String>(
      "/robot/system_status", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        bool processing = (msg->data == "PROCESSING_SCALE");
        if (processing && !robot_processing_scale_) {
            // Just entered weighing state, explicitly reset stability to ensure fresh evaluation
            stable_eval_cycles_ = 0; 
            evaluating_cartridge_ = false;
            waiting_for_removal_ = false;
        }
        robot_processing_scale_ = processing;
      });

    sub_target_min_ = create_subscription<std_msgs::msg::Float32>(
      "/loadcell/target_min", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        target_min_weight_ = msg->data;
      });

    sub_target_max_ = create_subscription<std_msgs::msg::Float32>(
      "/loadcell/target_max", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        target_max_weight_ = msg->data;
      });

    sub_cal_weight_ = create_subscription<std_msgs::msg::Float32>(
      "/loadcell/cal_weight", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        if (msg->data > 0.0f) {
          cal_known_weight_g_ = static_cast<double>(msg->data);
          RCLCPP_INFO(get_logger(),
            "[CAL] Updated cal_known_weight=%.2f g from GUI", cal_known_weight_g_);
        }
      });

    sub_ink_cap_ = create_subscription<std_msgs::msg::Float32>(
      "/Fill_HP1/ink_capacity", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        pub_float(pub_ink_cap_ack_, msg->data);
      });

    // ── VFD Subscriptions ────────────────────────────────
    sub_vfd_run_ = create_subscription<std_msgs::msg::Bool>(
      "/vfd/cmd_run", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !running_) {
            // Start motor
            write_reg(REG_CMD_, 0x0007, slave_id_); // Switch On
            rclcpp::sleep_for(50ms);
            write_reg(REG_CMD_, CMD_RUN_FWD_, slave_id_); // Enable Operation
        } else if (!msg->data && running_) {
            // Stop motor
            write_reg(REG_CMD_, CMD_ENABLE_, slave_id_); // Ready to switch on
        }
        running_ = msg->data;
        RCLCPP_INFO(get_logger(), "[VFD] cmd_run = %s", running_ ? "START" : "STOP");
      });

    sub_vfd_freq_ = create_subscription<std_msgs::msg::Float32>(
      "/vfd/cmd_freq", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        float f = msg->data;
        if (f < 0.0f) f = 0.0f;
        if (f > 60.0f) f = 60.0f;
        int val = static_cast<int>(f * ref_scale_);
        write_reg(REG_LFR_, val, slave_id_);
        RCLCPP_INFO(get_logger(), "[VFD] Target freq = %.1f Hz", f);
      });

    // ── Calibration Services ─────────────────────────────

    srv_cal_start_ = create_service<std_srvs::srv::Trigger>(
      "/loadcell/cal_start",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        if (!connection_ok_) {
          res->success = false;
          res->message = "RS485 not connected";
          return;
        }

        int32_t sum = 0;
        int valid = 0;
        for (int i = 0; i < 32; i++) {
           int32_t raw_ad = 0;
           if (read_2reg(0x1F40, loadcell_slave_id_, raw_ad)) {
               sum += raw_ad;
               valid++;
           }
           rclcpp::sleep_for(20ms);
        }

        if (valid > 0) {
           int32_t zero_ad = sum / valid;
           // AVP1 = 0x0008, PVP1 = 0x001A = 0
           write_2reg(0x0008, zero_ad, loadcell_slave_id_);
           write_2reg(0x001A, 0, loadcell_slave_id_);
           ad_avg_1_ = zero_ad;
           cal_point_count_ = 1; // Diem 1 (zero) da xong
           cal_state_ = CalState::WAITING_WEIGHT;

           RCLCPP_INFO(get_logger(), "[CAL] Step 1 done (zero): ad=%d", zero_ad);
           pub_str(pub_cal_status_, "WAITING_WEIGHT");
           res->success = true;
           res->message = "Zero captured. Dat qua can diem 1 (VD: 100g) len roi goi cal_set_known.";
        } else {
           res->success = false;
           res->message = "Khong the doc duoc gia tri AD.";
        }
      });

    // AVP register: index 0=AVP1(0x0008)...AVP5(0x0010)
    // PVP register: index 0=PVP1(0x001A)...PVP5(0x0022)
    srv_cal_set_known_ = create_service<std_srvs::srv::Trigger>(
      "/loadcell/cal_set_known",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        if (cal_state_ != CalState::WAITING_WEIGHT) {
          res->success = false;
          res->message = "Goi cal_start truoc.";
          return;
        }
        if (cal_point_count_ >= 5) {
          res->success = false;
          res->message = "Da du 5 diem calib. Goi cal_start de bat dau lai.";
          return;
        }

        // AVP/PVP register map: point index 1..4 maps to AVP2..AVP5
        static const int AVP_REG[] = {0x0008, 0x000A, 0x000C, 0x000E, 0x0010};
        static const int PVP_REG[] = {0x001A, 0x001C, 0x001E, 0x0020, 0x0022};

        int32_t sum = 0;
        int valid = 0;
        for (int i = 0; i < 32; i++) {
           int32_t raw_ad = 0;
           if (read_2reg(0x1F40, loadcell_slave_id_, raw_ad)) {
               sum += raw_ad;
               valid++;
           }
           rclcpp::sleep_for(20ms);
        }

        if (valid > 0) {
           int32_t known_ad = sum / valid;
           int idx = cal_point_count_; // 1..4 (diem 2..5)
           write_2reg(AVP_REG[idx], known_ad, loadcell_slave_id_);
           write_2reg(PVP_REG[idx], static_cast<int32_t>(cal_known_weight_g_), loadcell_slave_id_);
           cal_point_count_++; // 2..5

           RCLCPP_INFO(get_logger(), "[CAL] Point %d done: ad=%d  weight=%.1fg  reg_avp=0x%04X",
                       cal_point_count_, known_ad, cal_known_weight_g_, AVP_REG[idx]);

           if (cal_point_count_ < 5) {
             // Tiep tuc — chua du 5 diem
             std::string status = "CONTINUE_CAL_" + std::to_string(cal_point_count_) + "/5";
             pub_str(pub_cal_status_, status);
             res->success = true;
             res->message = "Diem " + std::to_string(cal_point_count_) +
                             "/5 xong. Dat qua can tiep theo len roi goi lai cal_set_known.";
           } else {
             // Du 5 diem — commit vao hardware va ket thuc
             write_2reg(0x0006, 5, loadcell_slave_id_); // CalPoint = 5
             cal_state_ = CalState::DONE;
             pub_str(pub_cal_status_, "DONE");
             RCLCPP_INFO(get_logger(), "[CAL] 5-point calibration DONE.");
             res->success = true;
             res->message = "Calibration 5 diem hoan tat!";
           }
        } else {
           res->success = false;
           res->message = "Khong doc duoc AD cho diem " + std::to_string(cal_point_count_ + 1) + ".";
        }
      });

    // ── Timers ───────────────────────────────────────────
    timer_loadcell_ = create_wall_timer(
      std::chrono::milliseconds(loadcell_period_ms_),
      std::bind(&RS485BusNode::poll_loadcell, this));

    timer_freq_ = create_wall_timer(200ms,
      std::bind(&RS485BusNode::poll_freq, this));

    // Reset VFD Fault
    write_reg(REG_CMD_, 0x0080, slave_id_);
    rclcpp::sleep_for(100ms);
    write_reg(REG_CMD_, CMD_DISABLE_, slave_id_);
    rclcpp::sleep_for(100ms);

    write_reg(REG_LFR_, static_cast<int>(ref_hz_ * ref_scale_), slave_id_);
    write_reg(REG_CMD_, CMD_ENABLE_, slave_id_);
    pub_str(pub_status_, "OK");
    pub_str(pub_monitor_status_, "IDLE");
  }

  ~RS485BusNode()
  {
    if (ctx_) { modbus_close(ctx_); modbus_free(ctx_); }
  }

private:

  // ── Modbus helpers ───────────────────────────────────────

  bool write_reg(int reg, int val, int sid)
  {
    if (!ctx_) return false;
    std::lock_guard<std::mutex> lk(bus_mutex_);
    modbus_set_slave(ctx_, sid);
    if (modbus_write_register(ctx_, reg, val) == -1) {
      RCLCPP_WARN(get_logger(), "[Write] reg=%d err=%s", reg, modbus_strerror(errno));
      return false;
    }
    sleep_ta(); return true;
  }

  bool read_reg(int reg, int sid, int &out)
  {
    if (!ctx_) return false;
    std::lock_guard<std::mutex> lk(bus_mutex_);
    modbus_set_slave(ctx_, sid);
    uint16_t r = 0;
    if (modbus_read_registers(ctx_, reg, 1, &r) == -1) return false;
    out = r; sleep_ta(); return true;
  }

  bool read_2reg(int reg, int sid, int32_t &out)
  {
    if (!ctx_) return false;
    std::lock_guard<std::mutex> lk(bus_mutex_);
    modbus_set_slave(ctx_, sid);
    uint16_t r[2] = {0, 0};
    if (modbus_read_registers(ctx_, reg, 2, r) == -1) return false;
    out = (static_cast<int32_t>(r[0]) << 16) | r[1];
    sleep_ta(); return true;
  }

  bool write_2reg(int reg, int32_t val, int sid)
  {
    if (!ctx_) return false;
    std::lock_guard<std::mutex> lk(bus_mutex_);
    modbus_set_slave(ctx_, sid);
    uint16_t out[2];
    out[0] = static_cast<uint16_t>((static_cast<uint32_t>(val) >> 16) & 0xFFFF);
    out[1] = static_cast<uint16_t>(static_cast<uint32_t>(val) & 0xFFFF);
    if (modbus_write_registers(ctx_, reg, 2, out) == -1) {
      RCLCPP_WARN(get_logger(), "[Write] reg=%d err=%s", reg, modbus_strerror(errno));
      return false;
    }
    sleep_ta(); return true;
  }

  void sleep_ta()
  {
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(turnaround_ * 1000)));
  }

  // ── Publish helpers ──────────────────────────────────────

  void pub_bool(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr &p, bool v)
  { std_msgs::msg::Bool m; m.data = v; p->publish(m); }

  void pub_int(rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr &p, int v)
  { std_msgs::msg::Int32 m; m.data = v; p->publish(m); }

  void pub_float(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr &p, float v)
  { std_msgs::msg::Float32 m; m.data = v; p->publish(m); }

  void pub_str(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &p, const std::string &s)
  { std_msgs::msg::String m; m.data = s; p->publish(m); }

  void pub_batch_stats()
  {
    std::string j =
      "{\"total\":"       + std::to_string(batch_total_)  +
      ",\"pass\":"        + std::to_string(batch_pass_)   +
      ",\"fail\":"        + std::to_string(batch_fail_)   +
      ",\"consec_fail\":" + std::to_string(consec_fail_)  + "}";
    pub_str(pub_batch_stats_, j);
  }

  // ── Calibration file I/O ─────────────────────────────────

  void save_scale_to_file(float scale)
  {
    std::ofstream f(config_path_);
    if (!f.is_open()) {
      RCLCPP_ERROR(get_logger(), "[CAL] Cannot write to %s", config_path_.c_str());
      return;
    }
    f << "loadcell_scale=" << scale << "\n";
    RCLCPP_INFO(get_logger(), "[CAL] Scale %.6f saved to %s", scale, config_path_.c_str());
  }

  void load_scale_from_file()
  {
    std::ifstream f(config_path_);
    if (!f.is_open()) return;
    std::string line;
    while (std::getline(f, line)) {
      auto pos = line.find("loadcell_scale=");
      if (pos != std::string::npos) {
        try {
          float s = std::stof(line.substr(pos + 15));
          if (s > 0.0f) {
            loadcell_scale_ = s;
            RCLCPP_INFO(get_logger(), "[CAL] Loaded scale=%.6f from %s",
                        loadcell_scale_, config_path_.c_str());
          }
        } catch (...) {}
      }
    }
  }

  // ── VFD poll ─────────────────────────────────────────────

  void poll_freq()
  {
    if (!connection_ok_) return;
    int raw = 0;
    if (!read_reg(REG_LFR_, slave_id_, raw)) {
      return;
    }
    pub_float(pub_freq_, raw / 10.0f);
    pub_int(pub_vfd_state_, running_ ? 2 : 1);
  }

  // ── Loadcell poll ────────────────────────────────────────

  void poll_loadcell()
  {
    // Luôn luôn publish status mỗi chu kỳ
    std::string status_msg = "OK";

    if (!connection_ok_ || !ctx_) {
      status_msg = "ERROR";
      pub_str(pub_status_, status_msg);
      // Thử reconnect nếu mất kết nối
      static int reconnect_count = 0;
      reconnect_count++;
      if (reconnect_count % 10 == 0) { // thử lại mỗi 10 chu kỳ
        RCLCPP_WARN(get_logger(), "[RS485] Trying to reconnect...");
        
        std::lock_guard<std::mutex> lk(bus_mutex_); // Lock during reconnect

        // Giải phóng và tạo lại context
        if (ctx_) { modbus_close(ctx_); modbus_free(ctx_); ctx_ = nullptr; }
        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        std::string par_str = this->get_parameter("parity").as_string();
        int stopbits = this->get_parameter("stopbits").as_int();
        int bytesize = this->get_parameter("bytesize").as_int();
        char parity = par_str.empty() ? 'N' : static_cast<char>(par_str[0]);
        ctx_ = modbus_new_rtu(port.c_str(), baudrate, parity, bytesize, stopbits);
        if (ctx_) {
          double timeout_s = this->get_parameter("timeout").as_double();
          modbus_set_response_timeout(ctx_,
            static_cast<uint32_t>(timeout_s),
            static_cast<uint32_t>((timeout_s - static_cast<int>(timeout_s)) * 1e6));
          if (modbus_connect(ctx_) == -1) {
            RCLCPP_ERROR(get_logger(), "[RS485] Cannot reconnect: %s", modbus_strerror(errno));
            modbus_free(ctx_); ctx_ = nullptr;
            connection_ok_ = false;
          } else {
            connection_ok_ = true;
            RCLCPP_INFO(get_logger(), "[RS485] Reconnected RS485!");
          }
        }
      }
      return;
    }

    // 1. Doc raw tu Modbus (Luon luon 32bit nhu HD)
    float net = 0.0f;
    int32_t raw = 0;
    
    static int modbus_read_fail_count = 0; // Đếm số lần lỗi

    if (!read_2reg(0x0000, loadcell_slave_id_, raw)) {
      modbus_read_fail_count++;
      
      if (modbus_read_fail_count >= 2) {
          status_msg = "ERROR";
          pub_str(pub_status_, status_msg);
      }
      
      if (modbus_read_fail_count >= 5) {
        RCLCPP_ERROR(get_logger(), "[RS485] Modbus read failed 5 times continuously. Triggering reconnect!");
        connection_ok_ = false; // Phủ báo trạng thái ngắt để kích hoạt nhảy vào khối reconnect ở chu kỳ sau
        modbus_read_fail_count = 0;
      }
      return; 
    }
    modbus_read_fail_count = 0; // Đọc thành công thì reset bộ đếm

    net = static_cast<float>(raw);
    last_raw_weight_ = net;

    // Apply software tare offset before publishing
    float net_tared = net - tare_offset_;

    // Theo HD: bo qua MA vi Loadcell xu ly san
    pub_float(pub_raw_weight_, net);
    pub_float(pub_weight_, net_tared);
    pub_str(pub_status_, "OK");

    float raw_weight = net_tared;

    // 4. Overload — tu dong, khong can trigger
    if (net > static_cast<float>(max_weight_g_)) {
      if (!overload_active_) {
        overload_active_ = true;
        RCLCPP_ERROR(get_logger(), "[OVERLOAD] %.2f g vuot nguong %.2f g", net, max_weight_g_);
      }
      pub_bool(pub_overload_, true);
    }

    // 5. Zero drift detection — tu dong
    bool in_dead_zone = std::fabs(net) < DEAD_ZONE_G;
    bool drifted      = std::fabs(raw_weight - tare_offset_) > DRIFT_EPSILON_G;

    if (in_dead_zone && drifted) {
      drift_cycles_++;
      if (drift_cycles_ >= DRIFT_HOLD_CYCLES) {
        RCLCPP_WARN(get_logger(),
          "[DRIFT] Zero drift: offset_lenh=%.3f g (lien tuc %d chu ky)",
          raw_weight - tare_offset_, drift_cycles_);
        pub_bool(pub_drift_warn_, true);
        drift_cycles_ = 0;
      }
    } else {
      drift_cycles_ = 0;
    }

    // 6. Local Evaluation Logic (Min/Max checking)
    if (in_dead_zone) {
      if (waiting_for_removal_ || evaluating_cartridge_) {
        RCLCPP_INFO(get_logger(), "[SCALE] Scale emptied. Resetting internal statemachine.");
      }
      evaluating_cartridge_ = false;
      waiting_for_removal_ = false;
      stable_eval_cycles_ = 0;
      eval_last_weight_ = 0.0f;
      if (monitor_state_ != "IDLE") {
        monitor_state_ = "IDLE";
        pub_str(pub_monitor_status_, monitor_state_);
      }
    } else if (target_batch_weight_ > 0.0f && robot_processing_scale_) {
      if (!waiting_for_removal_) {
        evaluating_cartridge_ = true;

        if (monitor_state_ != "MEASURING") {
          monitor_state_ = "MEASURING";
          pub_str(pub_monitor_status_, monitor_state_);
        }

        // Tự động kiểm tra độ ổn định (Stability Check)
        // Nếu dao động giữa 2 lần lấy mẫu liên tiếp nhỏ hơn 0.5g -> Tích luỹ chu kỳ
        if (std::fabs(net - eval_last_weight_) <= 0.5f) {
           stable_eval_cycles_++;
        } else {
           stable_eval_cycles_ = 0; // Dao động mạnh -> Trọng lượng chưa ổn định, reset đếm
        }
        eval_last_weight_ = net;

        // Nếu giữ độ ổn định trong 3 chu kỳ liên tiếp (~1.5 giây ở tần số 2Hz)
        if (stable_eval_cycles_ >= 3) {
          evaluating_cartridge_ = false;
          waiting_for_removal_ = true; // Wait until it drops back to DEAD_ZONE

          bool pass = (net >= target_min_weight_ && net <= target_max_weight_);
          
          batch_total_++;
          if (pass) {
            batch_pass_++;
            consec_fail_ = 0;
            monitor_state_ = "PASS";
            RCLCPP_INFO(get_logger(), "[BATCH] Local Check PASS  total=%d  pass=%d  fail=%d  weight=%.2fg (%.2f - %.2f)",
                        batch_total_, batch_pass_, batch_fail_, net, target_min_weight_, target_max_weight_);
          } else {
            batch_fail_++;
            consec_fail_++;
            monitor_state_ = "FAIL";
            RCLCPP_WARN(get_logger(),
              "[BATCH] Local Check FAIL  weight=%.2fg (not in %.2f - %.2f). consec=%d",
              net, target_min_weight_, target_max_weight_, consec_fail_);
              
            if (consec_fail_ >= MAX_CONSEC_FAIL) {
              RCLCPP_ERROR(get_logger(), "[BATCH] CANH BAO: %d lan fail lien tiep — kiem tra fill muc / co hoc!", consec_fail_);
            }
          }

          pub_batch_stats();
          pub_int(pub_consec_fail_, consec_fail_);
          pub_bool(pub_signal_process_, pass);
          pub_str(pub_monitor_status_, monitor_state_);
        }
      }
    }
  }

  // ── Members ──────────────────────────────────────────────

  modbus_t         *ctx_{nullptr};
  std::mutex        bus_mutex_;
  double            turnaround_;
  bool              connection_ok_;
  bool              running_;

  float             tare_offset_;
  float             last_raw_weight_;
  std::deque<float> ma_buffer_;

  bool              overload_active_;
  int               drift_cycles_;

  int               consec_fail_;
  int               batch_total_, batch_pass_, batch_fail_;

  CalState          cal_state_;
  int               cal_point_count_; // Diem hien tai da calib (1=zero, 2-5=known points)
  int32_t           ad_avg_1_;
  int32_t           ad_avg_2_;
  double            cal_known_weight_g_;
  std::string       config_path_;

  float             target_batch_weight_;
  float             target_min_weight_;
  float             target_max_weight_;
  std::string       monitor_state_;
  bool              evaluating_cartridge_;
  bool              robot_processing_scale_;
  bool              waiting_for_removal_;
  int               stable_eval_cycles_;
  float             eval_last_weight_;

  int    REG_CMD_, REG_LFR_;
  int    CMD_DISABLE_, CMD_ENABLE_, CMD_RUN_FWD_;
  int    slave_id_;
  double ref_hz_, ref_scale_;

  int    loadcell_slave_id_;
  int    loadcell_reg_;
  double loadcell_scale_;
  bool   loadcell_is_32bit_;
  int    loadcell_period_ms_;
  double max_weight_g_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_weight_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_raw_weight_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  pub_status_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_tare_ack_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_overload_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_drift_warn_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr   pub_consec_fail_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  pub_batch_stats_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  pub_cal_status_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_signal_process_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  pub_monitor_status_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_freq_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr   pub_vfd_state_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_ink_cap_ack_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_tare_cmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_tare_reset_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_overload_ack_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_batch_reset_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_weight_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  sub_robot_status_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_min_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_max_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_cal_weight_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_ink_cap_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_vfd_run_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_vfd_freq_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_cal_start_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_cal_set_known_;

  // Timers
  rclcpp::TimerBase::SharedPtr timer_loadcell_;
  rclcpp::TimerBase::SharedPtr timer_freq_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RS485BusNode>());
  rclcpp::shutdown();
  return 0;
}
      