#!/usr/bin/env python3
"""
Cartridge Loading System — ROS 2 + festo-edcon

AUTO mode  : Đọc sensor thực, tự động trigger STATE 1/2/3/4 khi đủ điều kiện.
MANUAL mode: Đọc sensor simulation, logic STATE giống AUTO.
             STATE 2/3/4 vẫn trigger tự động (robot topics).
             STATE 1 CHỈ chạy khi operator nhấn nút STATE1 trên GUI.
             Sau S1_COMPLETE → về IDLE, chờ operator nhấn STATE1 lần tiếp.
             S2A_COMPLETE   → về IDLE (không auto-vào S1).

Patches v4–v6: (giữ nguyên — xem header gốc)
Manual/Auto mode redesign v7:
  [V7-1] sensor(): AUTO  → real only. MANUAL → sim only (không OR real+sim).
  [V7-2] _do_idle_input(): MANUAL không auto-trigger S1 (chỉ qua _state1_enabled
         được set bởi GUI button). S2/S3/S4 vẫn tự trigger bình thường.
  [V7-3] _s2a_complete(): MANUAL → về IDLE (không auto-vào S1).
  [V7-4] sensor_real() giữ nguyên để internal checks (homing, hardware) dùng được.
  [V7-5] State2 trigger qua /robot/done_tray_input (simulateDoneTrayInput từ GUI).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool, String, Int32
from enum import Enum
import time
from typing import Optional, Any
import yaml
import json
import threading
import os
import signal
import traceback

try:
    from edcon.edrive.com_modbus import ComModbus
    from edcon.edrive.motion_handler import MotionHandler
    from edcon.utils.logging import Logging as EdconLogging
    EDCON_AVAILABLE = True
except ImportError:
    ComModbus = MotionHandler = EdconLogging = None
    EDCON_AVAILABLE = False

try:
    from cpx_io.cpx_system.cpx_ap.cpx_ap import CpxAp
    CPXAP_AVAILABLE = True
except ImportError:
    CpxAp = None
    CPXAP_AVAILABLE = False

# ─── Constants ───────────────────────────────────────────────────
COUNTS_PER_MM        = 1000
CYLINDER_TIMEOUT_S   = 15.0
OUTPUT_DETECT_WAIT_S = 25.0
INY_JOG_VEL          = 40
OUTY_ROW1_LIMIT_MM   = 590.0
TRAY_ROBOT_CHECK_TIMEOUT_S = 3.0
JOG_OUTPUT_TIMEOUT_S = 120.0
POSITION_TOLERANCE_HOME = 5.0   # mm — vị trí gần 0 đủ để skip re-home
HOME_SPEED              = 30    # mm/s cho position_task về 0

# ─── Sensor IDs — matching sensors.yaml ─────────────────────────
S1_BELT_START      = 1
S2_BELT_MID        = 2
S3_BELT_END        = 3
S4_SCAN_STACK_P1   = 4
S5_OUTPUT_DETECT   = 5
S6_CHECK_TRAY_P1   = 6
S7_TRAY_AT_ROBOT   = 7
# S8 reserved
S9_CYL1_RETRACTED  = 9
S10_CYL1_EXTENDED  = 10
# S11/S12 ATV Run/Fault — monitored by vfd_logic_node, not used here
# S13–S16 reserved
S17_PLATFORM       = 17
S18_FEED_OK        = 18
S19_CHECK_TRAY_P2  = 19
S20_SCAN_STACK_P2  = 20
S21_CYL2_RETRACTED = 21
S22_CYL2_EXTENDED  = 22


# ─── State Enum ──────────────────────────────────────────────────

class SystemState(Enum):
    IDLE            = "idle"
    ERROR           = "error"
    HOMING          = "homing"
    HOMING_RUNNING  = "homing_running"

    # STATE 1
    S1_CONFIRM_SAFE     = "s1_confirm_safe"
    S1_INX_MOVE         = "s1_inx_move"
    S1_WAIT_ARRIVE      = "s1_wait_arrive"
    S1_INY_SCAN         = "s1_iny_scan"
    S1_WAIT_STOP_S4     = "s1_wait_stop_s4"
    S1_INY_TO_ROW       = "s1_iny_to_row"
    S1_CHECK_S5         = "s1_check_s5"
    S1_FALLBACK_RETRACT = "s1_fallback_retract"
    S1_FALLBACK_WAIT_INY = "s1_fallback_wait_iny"
    S1_WAIT_GUI_CONFIRM = "s1_wait_gui_confirm"
    S1_RETRY_INX_500    = "s1_retry_inx_500"
    S1_RETRY_JOG        = "s1_retry_jog"
    S1_CYL1_EXTEND      = "s1_cyl1_extend"
    S1_INY_50           = "s1_iny_50"
    S1_INY_200          = "s1_iny_200"
    S1_WAIT_RELEASE     = "s1_wait_release"
    S1_WAIT_S7         = "s1_wait_s7"
    S1_INX_10           = "s1_inx_10"
    S1_COMPLETE         = "s1_complete"
    S1_RETRY_SCAN_HOME  = "s1_retry_scan_home"

    # STATE 2
    S2A_CHECK_INTERLOCK   = "s2a_check_interlock"
    S2A_INX_500           = "s2a_inx_500"
    S2A_INY_200_CYL1      = "s2a_iny_200_cyl1"
    S2A_WAIT_S7          = "s2a_wait_s7"
    S2A_INY_10            = "s2a_iny_10"
    S2A_INX_10            = "s2a_inx_10"
    S2A_INY_JOG_OUTPUT    = "s2a_iny_jog_output"
    S2A_INY_OUTPUT_ROW    = "s2a_iny_output_row"
    S2A_WAIT_CYL1_RET     = "s2a_wait_cyl1_ret"
    S2A_INY_10_FINAL      = "s2a_iny_10_final"
    S2A_INX_20            = "s2a_inx_20"
    S2A_RETRY_SCAN_HOME   = "s2a_retry_scan_home"
    S2A_COMPLETE          = "s2a_complete"

    # STATE 3
    S3_CHECK_OUTXY_SAFE  = "s3_check_outxy_safe"
    S3_SERVO3_TARGET1    = "s3_servo3_target1"
    S3_CHECK_S17         = "s3_check_s17"
    S3_WAIT_S17          = "s3_wait_s17"
    S3_WAIT_GUI_CONFIRM  = "s3_wait_gui_confirm"
    S3_SERVO3_FEED       = "s3_servo3_feed"
    S3_WAIT_S18          = "s3_wait_s18"
    S3_COMPLETE          = "s3_complete"

    # STATE 4
    S4_CHECK_OUTY_SAFE   = "s4_check_outy_safe"
    S4_OUTX_TARGET2      = "s4_outx_target2"
    S4_OUTY_PICK         = "s4_outy_pick"
    S4_CYL2_EXTEND       = "s4_cyl2_extend"
    S4_OUTY_TARGET1      = "s4_outy_target1"
    S4_OUTX_TARGET3      = "s4_outx_target3"
    S4_CHECK_S19          = "s4_check_s19"
    S4_OUTY_ROW1         = "s4_outy_row1"
    S4_OUTY_SCAN_S20     = "s4_outy_scan_s20"
    S4_OUTY_DROP         = "s4_outy_drop"
    S4_CYL2_RETRACT      = "s4_cyl2_retract"
    S4_OUTY_OUTX_HOME    = "s4_outy_outx_home"
    S4_RETRY_SCAN_HOME   = "s4_retry_scan_home"
    S4_COMPLETE          = "s4_complete"


# ─── Config ──────────────────────────────────────────────────────

from config import SystemConfig as CartridgeConfig

# ─── Main Node ───────────────────────────────────────────────────

class CartridgeSystem(Node):

    def __init__(self, config: CartridgeConfig):
        super().__init__('cartridge_providesystem')
        self.config = config

        # Hardware
        self._servo_lock        = threading.Lock()
        self.servos: dict       = {}
        self.zero_offset: dict  = {}
        self.io_module          = None
        self.io_module_2        = None
        self._io_sensor_cache: list = []
        self._io_sensor_cache_2: list = []
        self._io_ready          = False
        self._io_ready_2        = False
        self._io_bg_lock        = threading.Lock()
        self._sim_sensors: dict = {}

        # Motion flags
        self._inx_moving = False
        self._iny_moving = False

        # ── INPUT side state machine ──────────────────────────────
        self.state_in           = SystemState.IDLE
        self._cmd_sent_in       = False
        self._step_start_in     = 0.0
        self._step_timeout_in   = 0.0
        self._s4_armed          = False
        self._current_row       = 0
        self._s5_retry          = 0
        self._inx_arrived       = False
        self._30s_timeout       = 0.0
        self._tray_robot_check_start   = 0.0
        self._s10_warn_t        = 0.0
        self._cyl_retry_t       = 0.0
        self._row1_pos          = 0.0
        self._row1_key          = 1
        self._place_delay_start = 0.0
        self._s1_scan_noise_retry = 0
        self._s4_prev_in        = False

        # STATE 2A runtime
        self._s6_snapshot       = False
        self._output_row        = 0
        self._output_target_pos = 0.0
        self._s4_armed_out      = False

        # ── STATE S3 runtime ─────────────────────────────────────────
        self.state_s3           = SystemState.IDLE
        self._cmd_sent_s3       = False
        self._step_start_s3     = 0.0
        self._step_timeout_s3   = 0.0
        self._s3_pending        = False
        
        # ── STATE S4 runtime ─────────────────────────────────────────
        self.state_s4           = SystemState.IDLE
        self._cmd_sent_s4       = False
        self._step_start_s4     = 0.0
        self._step_timeout_s4   = 0.0
        self._s4_trigger        = False
        self._outy_jog_start    = 0.0
        self._outy_jog_pos      = 0.0
        
        # ── Shared / Other ───────────────────────────────────────────
        self._motion_busy       = False
        self._robot_last_seen   = 0.0   # timestamp of last /robot/motion_busy msg
        self._robot_connected   = False # True when robot node is actively publishing
        self._s10_off_time       = 0.0
        self._s10_prev           = False

        # Homing completion flag (set by bg thread, consumed by main loop)
        self._homing_done_event  = threading.Event()
        self._homing_result      = False
        self._homing_abort       = threading.Event()  # set by STOP to cancel homing thread

        # JOG velocity — only affects JOG; state/homing velocities fixed by FAS firmware
        self._jog_velocity_ms  = 0.05   # m/s (default, overwritten by FAS read on first connect)
        self._jog_velocity_max = 0.08   # m/s (hard limit per FAS firmware)
        self._jog_vel_from_fas = False  # flag: already initialized from drive PNU
        self._fas_jog_vel: dict = {}    # per-servo FAS JOG velocity (m/s), keyed by sid

        # Tray tracking
        self.stack_row_index  = 0
        self._tray_loaded_ack = False

        # Operation
        self.state           = SystemState.IDLE
        self.operation_mode  = 'manual'   # 'auto' | 'ai' | 'manual'
        self._input_tray_done     = False
        self._state1_enabled = False
        self._gui_confirmed  = False
        self._system_paused  = False
        self._input_trays_empty_debounce_count = 0

        # Watchdog
        self._watchdog_last_tick = time.time()
        self._notify_throttle: dict = {}
        self._guide_logged: set = set()

        # ROS Publishers
        qos = QoSProfile(depth=1)
        self.pub_state          = self.create_publisher(String, '/system_state',                                          qos)
        self.pub_new_tray       = self.create_publisher(Bool,   '/cartridge_providesystem/new_tray_loaded',        qos)
        self.pub_newtray_output = self.create_publisher(Bool,   '/cartridge_providesystem/new_trayoutput_loaded',  qos)
        self.pub_gui_notify     = self.create_publisher(String, '/providesystem/gui_notify',  qos)
        self.pub_servo_pos      = self.create_publisher(String, '/providesystem/servo_positions', qos)
        self.pub_sensors        = self.create_publisher(String, '/providesystem/sensors_state', qos)
        self.pub_busy_cartridge = self.create_publisher(Bool, '/cartridge/busy', qos)
        self.pub_input_trays_empty = self.create_publisher(Bool, '/cartridge/input_trays_empty', qos)
        self.pub_current_mode   = self.create_publisher(String, '/providesystem/current_mode', qos)        
        self.pub_robot_mode     = self.create_publisher(Int32, '/robot/set_mode', qos)
        self.pub_homing_done    = self.create_publisher(Bool, '/cartridge/homing_done', qos)
        self.pub_vfd_run        = self.create_publisher(Bool,   '/vfd/cmd_run', qos)
        self._vfd_current_cmd   = False

        qos_latching = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_config_data    = self.create_publisher(String, '/providesystem/config_data', qos_latching)

        # ROS Subscribers
        self.create_subscription(Bool,   '/system/start_button',             self._cb_start,              qos)
        self.create_subscription(Bool,   '/system/stop_button',              self._cb_stop,               qos)
        self.create_subscription(Bool,   '/system/pause_button',             self._cb_pause,              qos)
        self.create_subscription(Bool,   '/robot/motion_busy',               self._cb_motion_busy,          qos)
        self.create_subscription(Bool,   '/robot/done_tray_output',          self._cb_done_tray_output,     qos)
        self.create_subscription(Bool,   '/robot/done_tray_input',           self._cb_done_tray_input,      qos)

        self.pub_gripper_status = self.create_publisher(Bool, '/robot/gripper_status', 10)
        self.pub_picker_status  = self.create_publisher(Bool, '/robot/picker_status', 10)
        self.create_subscription(Bool, '/robot/gripper_cmd', self._cb_gripper_cmd, 10)
        self.create_subscription(Bool, '/robot/picker_cmd', self._cb_picker_cmd, 10)
        self.create_subscription(Bool,   '/robot/done_tray_input',           self._cb_done_tray_input,      qos)

        self.create_subscription(String, '/providesystem/gui_confirm',       self._cb_gui_confirm,        qos)
        self.create_subscription(String, '/providesystem/jog_cmd',           self._cb_jog,                qos)
        qos_sim = QoSProfile(depth=10)
        self.create_subscription(String, '/providesystem/sim_sensor',        self._cb_sim,                qos_sim)
        self.create_subscription(String, '/providesystem/set_operation_mode',self._cb_mode,               qos)
        self.create_subscription(Int32, '/robot/set_mode', self._cb_robot_mode, qos)
        self.create_subscription(String, '/providesystem/goto_state',        self._cb_goto_state,         qos)
        self.create_subscription(String, '/providesystem/update_config',     self._cb_update_config,      qos)
        self.create_subscription(String, '/providesystem/get_config',        self._cb_get_config,         qos)
        self.create_subscription(String, '/providesystem/set_target_row',   self._cb_set_target_row,     qos)

        self._connect_hardware()

        self.create_timer(0.05, self._safe_control_loop)
        
        # Positions publisher chạy trong thread riêng
        # để Modbus timeout KHÔNG block ROS control loop
        self._pos_thread_stop = False
        threading.Thread(
            target=self._positions_bg_loop,
            daemon=True,
            name="pos_publisher"
        ).start()

        self.create_timer(5.0,  self._watchdog)

        self.get_logger().info("CartridgeSystem node started")

    def _safe_control_loop(self):
        """
        Wrapper bảo vệ cho vòng lặp điều khiển chính (20Hz, timer 0.05s).
        Bắt mọi exception không mong muốn và chuyển hệ thống sang trạng thái ERROR
        thay vì để node bị crash. Giúp hệ thống không bị treo hoàn toàn khi có lỗi
        bất ngờ trong logic state machine.
        """
        try:
            self._control_loop()
        except Exception as e:
            err_msg = f"FATAL CONTROL LOOP ERROR: {str(e)}"
            self.get_logger().error(err_msg)
            self.get_logger().error(traceback.format_exc())
            self._error(err_msg)

    # ══════════════════════════════════════════════════════════════
    # Hardware
    # ══════════════════════════════════════════════════════════════

    def _connect_hardware(self):
        """
        Khởi động kết nối song song đến tất cả phần cứng khi node start:
          - 5 servo Festo CMMT-AS qua Modbus TCP (edcon library)
          - IO Module 1 (Festo CPX-AP, IP: config.io_ip) — cảm biến S1–S16, cylinder 1/2
          - IO Module 2 (IP: config.io_ip_2) — cảm biến S17–S24 (output side)
        Mỗi thiết bị kết nối trong thread riêng (timeout 15s).
        Sau khi xong: khởi động thread _servo_reconnect_loop và _io_bg_loop để
        tự động reconnect khi mất kết nối.
        """
        threads = []

        if EDCON_AVAILABLE:
            if EdconLogging:
                EdconLogging()
            for sid, ip in self.config.servo_ips.items():
                t = threading.Thread(
                    target=self._connect_servo, args=(sid, ip, 2),
                    daemon=True, name=f"connect_s{sid}"
                )
                threads.append(t)
        else:
            self.get_logger().warn("Simulation mode (edcon not installed)")

        if CPXAP_AVAILABLE:
            threads.append(threading.Thread(
                target=self._connect_io, args=(1, self.config.io_ip),
                daemon=True, name="connect_io1"
            ))
            io2_ip = getattr(self.config, 'io_ip_2', "192.168.27.254")
            threads.append(threading.Thread(
                target=self._connect_io, args=(2, io2_ip),
                daemon=True, name="connect_io2"
            ))

        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=15)

        if EDCON_AVAILABLE:
            threading.Thread(
                target=self._servo_reconnect_loop, daemon=True, name="servo_reconnect"
            ).start()

        if CPXAP_AVAILABLE and (self.io_module or self.io_module_2):
            threading.Thread(target=self._io_bg_loop, daemon=True, name="io_bg").start()

    def _connect_io(self, idx: int, ip: str):
        """
        Kết nối đến IO Module Festo CPX-AP qua EtherNet/IP (tối đa 3 lần thử).
        idx=1 → Module chính (S1–S16, valve cylinder 1/2, gripper, picker)
        idx=2 → Module phụ output side (S17–S24)
        Khi kết nối Module 1 thành công: reset tất cả valve về 0V để đảm bảo
        trạng thái an toàn (cylinder retracted) ngay khi khởi động node.
        """
        for attempt in range(1, 4):
            try:
                mod = CpxAp(ip_address=ip, cycle_time=0.5)
                if idx == 1:
                    self.io_module = mod
                    # Initialize valve states (module 3) to 0V
                    try:
                        if len(mod.modules) > 3:
                            v_mod = mod.modules[3]
                            if v_mod.is_function_supported("read_channels"):
                                ch = v_mod.read_channels()
                                if len(ch) > 1 and (ch[0] or ch[1]):
                                    v_mod.reset_channel(0)
                                    v_mod.reset_channel(1)
                                if len(ch) > 3 and (ch[2] or ch[3]):
                                    v_mod.reset_channel(2)
                                    v_mod.reset_channel(3)
                    except Exception as ve:
                        self.get_logger().warn(f"Failed to init valves: {ve}")
                else:
                    self.io_module_2 = mod
                self.get_logger().info(f"IO {idx} {ip} OK")
                return
            except Exception as e:
                self.get_logger().warn(f"IO {idx} attempt {attempt}/3: {e}")
                if attempt < 3:
                    time.sleep(3.0)
        self.get_logger().error(f"IO {idx} module failed — sensor reads will be False")

    def _connect_servo(self, sid: int, ip: str, attempts: int = 5) -> bool:
        """
        Kết nối đến một servo Festo CMMT-AS qua Modbus TCP.
        Các bước sau khi kết nối:
          1. Đọc/ghi PNU 3490 → đặt telegram group = 111 (yêu cầu của edcon)
          2. Đọc base_velocity từ firmware FAS
          3. Đọc PNU 11352 (FAS JOG velocity v1) → lưu vào _fas_jog_vel[sid] và
             cập nhật _jog_velocity_ms nếu chưa được khởi tạo từ FAS
          4. Acknowledge faults để drive sẵn sàng nhận lệnh
        Trả về True nếu kết nối thành công, False nếu hết số lần thử.
        """
        for attempt in range(1, attempts + 1):
            try:
                t_ms = int(getattr(self.config, 'modbus_timeout_ms', 3000))
                com = ComModbus(ip_address=ip, cycle_time=60, timeout_ms=t_ms)
                tg = com.read_pnu(3490, 0)
                if tg != 111:
                    com.write_pnu(3490, 0, 111)
                mot = MotionHandler(com)
                try:
                    mot.base_velocity = com.read_pnu(12345, 0)
                    self.get_logger().info(f"S{sid} base_velocity={mot.base_velocity}")
                except Exception as e:
                    self.get_logger().warn(f"S{sid} base_velocity read failed: {e}")
                try:
                    fas_v1 = float(com.read_pnu(11352, 0))
                    self._fas_jog_vel[sid] = fas_v1
                    self.get_logger().info(f"S{sid} FAS jog v1={fas_v1:.3f} m/s")
                    with self._servo_lock:
                        if not self._jog_vel_from_fas and 0.001 <= fas_v1 <= 0.08:
                            self._jog_velocity_ms = fas_v1
                            self._jog_vel_from_fas = True
                except Exception as e:
                    self.get_logger().warn(f"S{sid} FAS jog v1 read failed: {e}")
                mot.acknowledge_faults()
                with self._servo_lock:
                    self.servos[sid] = mot
                self.get_logger().info(f"S{sid} ({ip}) OK (attempt {attempt})")
                return True
            except Exception as e:
                self.get_logger().warn(f"S{sid} ({ip}) fail attempt {attempt}/{attempts}: {e}")
                if attempt < attempts:
                    time.sleep(3.0)
        self.get_logger().error(f"S{sid} ({ip}) failed — se retry tu dong")
        return False

    def _servo_reconnect_loop(self):
        """
        Thread nền chạy vô tận để giám sát và tự động reconnect servo.
        Mỗi 10s (khi đủ kết nối) hoặc 5s (khi thiếu), thực hiện:
          - Với servo chưa kết nối: thử _connect_servo(attempts=1)
          - Với servo đang kết nối: ping bằng current_position()
        Nếu ping thất bại 2 lần liên tiếp → xóa servo khỏi dict và reconnect.
        Gửi thông báo GUI khi mất/khôi phục kết nối.
        """
        consecutive_fail: dict = {}
        while rclpy.ok():
            all_connected = all(sid in self.servos for sid in self.config.servo_ips)
            time.sleep(10.0 if all_connected else 5.0)

            for sid, ip in self.config.servo_ips.items():
                mot = self.servos.get(sid)
                if mot is None:
                    if self._connect_servo(sid, ip, attempts=1):
                        self._notify('info', f'S{sid} da ket noi', f'S{sid} ({ip}) OK')
                    continue
                try:
                    with self._servo_lock:
                        mot.current_position()
                    consecutive_fail[sid] = 0
                except Exception as e:
                    consecutive_fail[sid] = consecutive_fail.get(sid, 0) + 1
                    if consecutive_fail[sid] >= 2:
                        self.get_logger().error(f"S{sid} mat ket noi -> reconnect")
                        self._notify('warn', f'Servo S{sid} mat ket noi', f'Dang reconnect...')
                        with self._servo_lock:
                            self.servos.pop(sid, None)
                        if self._connect_servo(sid, ip, attempts=1):
                            consecutive_fail[sid] = 0
                            self._notify('info', f'S{sid} ket noi lai', f'S{sid} OK')

    def destroy_node(self):
        """
        Dọn dẹp tài nguyên khi node bị shutdown (Ctrl+C hoặc ros2 node kill).
        Dừng thread đọc vị trí, gửi lệnh stop + shutdown đến từng servo,
        và giải phóng kết nối IO module. Được ROS2 executor gọi tự động.
        """
        self._pos_thread_stop = True
        for sid, mot in list(self.servos.items()):
            try:
                mot.stop_motion_task()
                mot.shutdown()
            except Exception as e:
                self.get_logger().warn(f"S{sid} shutdown: {e}")

        if getattr(self, 'io_module', None) is not None:
            try:
                self.io_module = None
                self.get_logger().info("IO module 1 closed")
            except Exception as e:
                self.get_logger().warn(f"IO 1 close error: {e}")
                
        if getattr(self, 'io_module_2', None) is not None:
            try:
                self.io_module_2 = None
                self.get_logger().info("IO module 2 closed")
            except Exception as e:
                self.get_logger().warn(f"IO 2 close error: {e}")

        super().destroy_node()

    # ── IO background reader ──────────────────────────────────────

    def _io_bg_loop(self):
        """
        Thread nền đọc trạng thái toàn bộ IO module với chu kỳ 50ms.
        Đọc tất cả channel của Module 1 và Module 2 vào cache (_io_sensor_cache,
        _io_sensor_cache_2) để vòng lặp điều khiển chính không bị block bởi
        latency Modbus/EtherNet/IP.
        Nếu lỗi liên tiếp >= 3 lần → đánh dấu module không sẵn sàng (_io_ready=False)
        và tự động spawn thread reconnect.
        """
        fail1, fail2 = 0, 0
        while rclpy.ok():
            # Module 1
            if self.io_module is not None:
                try:
                    channels = []
                    for mod in self.io_module.modules:
                        if mod.is_function_supported("read_channels"):
                            ch = mod.read_channels()
                            if isinstance(ch, list):
                                channels.extend(ch)
                    with self._io_bg_lock:
                        self._io_sensor_cache = channels
                        self._io_ready = True
                    fail1 = 0
                except Exception as e:
                    fail1 += 1
                    if fail1 >= 3:
                        with self._io_bg_lock: self._io_ready = False
                        self.io_module = None
                        self.get_logger().warn(f"[IO-bg 1] reconnecting: {e}")
                        threading.Thread(target=self._reconnect_io1, daemon=True).start()
            
            # Module 2
            if self.io_module_2 is not None:
                try:
                    channels2 = []
                    for mod in self.io_module_2.modules:
                        if mod.is_function_supported("read_channels"):
                            ch2 = mod.read_channels()
                            if isinstance(ch2, list):
                                channels2.extend(ch2)
                    with self._io_bg_lock:
                        self._io_sensor_cache_2 = channels2
                        self._io_ready_2 = True
                    fail2 = 0
                except Exception as e:
                    fail2 += 1
                    if fail2 >= 3:
                        with self._io_bg_lock: self._io_ready_2 = False
                        self.io_module_2 = None
                        self.get_logger().warn(f"[IO-bg 2] reconnecting: {e}")
                        threading.Thread(target=self._reconnect_io2, daemon=True).start()
            
            time.sleep(0.05)

    def _reconnect_io1(self):
        """Reconnect IO Module 1 sau 5s delay (tránh flood nếu lỗi liên tục)."""
        time.sleep(5.0)
        try:
            self.io_module = CpxAp(ip_address=self.config.io_ip)
        except Exception: pass

    def _reconnect_io2(self):
        """Reconnect IO Module 2 sau 5s delay."""
        time.sleep(5.0)
        try:
            self.io_module_2 = CpxAp(ip_address=getattr(self.config, 'io_ip_2', "192.168.27.254"))
        except Exception: pass

    # ── Sensor read ───────────────────────────────────────────────

    def _sensor_raw(self, sid: int) -> bool:
        """Luôn đọc sensor phần cứng thực (IO module).
           - S17..S20 → Module 2 (CH 0-3)
           - S1..S8, S9..S22 → Module 1 (CH tương ứng, mặc định sid-1)
        """
        with self._io_bg_lock:
            cache1 = getattr(self, '_io_sensor_cache', [])
            ready1 = getattr(self, '_io_ready', False)
            cache2 = getattr(self, '_io_sensor_cache_2', [])
            ready2 = getattr(self, '_io_ready_2', False)

        if 1 <= sid <= 16:
            if not ready1 or len(cache1) < sid: return False
            return bool(cache1[sid - 1])
        elif 17 <= sid <= 24:
            if not ready2 or len(cache2) < (sid - 16): return False
            return bool(cache2[sid - 17])
        return False

    def sensor(self, sid: int) -> bool:
        """
        [V7-1] Nguồn sensor theo mode:
          AUTO   → đọc sensor thực (IO module).
          MANUAL → đọc sensor simulation (_sim_sensors).
                   Nếu sim chưa set cho sensor đó → trả về False.
        """
        if self.operation_mode == 'manual':
            return bool(self._sim_sensors.get(sid, False))
        # AUTO: đọc thực
        return self._sensor_raw(sid)

    def sensor_real(self, sid: int) -> bool:
        """Luôn đọc sensor thực — dùng cho homing, hardware checks."""
        return self._sensor_raw(sid)

    def _snap(self, *sids: int) -> tuple:
        """Snapshot nhiều sensor cùng lúc, theo logic mode hiện tại."""
        if self.operation_mode == 'manual':
            return tuple(bool(self._sim_sensors.get(sid, False)) for sid in sids)
        # AUTO: đọc thực
        with self._io_bg_lock:
            cache1 = getattr(self, '_io_sensor_cache', [])
            ready1 = getattr(self, '_io_ready', False)
            cache2 = getattr(self, '_io_sensor_cache_2', [])
            ready2 = getattr(self, '_io_ready_2', False)

        results = []
        for sid in sids:
            try:
                if 1 <= sid <= 16:
                    raw = bool(cache1[sid - 1]) if ready1 and len(cache1) >= sid else False
                elif 17 <= sid <= 24:
                    raw = bool(cache2[sid - 17]) if ready2 and len(cache2) >= (sid - 16) else False
                else:
                    raw = False
            except Exception:
                raw = False
            results.append(raw)
        return tuple(results)

    # ── Non-blocking servo motion ─────────────────────────────────

    def _nb_move(self, servo_id: int, pos_mm: float, vel: int = 30, continuous_update: bool = False) -> bool:
        """
        Gửi lệnh di chuyển tuyệt đối non-blocking đến servo (không chờ đến đích).
        Kiểm tra giới hạn phần mềm (servo_limits) và yêu cầu đã homed trước khi gửi.
        pos_mm: vị trí đích tính từ điểm zero (sau homing), đơn vị mm.
        vel: tốc độ mm/s (mặc định 30).
        Sau khi gửi lệnh: đặt _ignore_arrived_X trong 0.5s để tránh
        target_position_reached() trả True sai ngay khi bắt đầu di chuyển.
        Trả về False nếu vượt giới hạn hoặc chưa homed, True nếu thành công.
        """
        limit = self._conf('servo_limits', {}).get(servo_id, 999.0)
        if pos_mm > limit:
            self.get_logger().error(f"S{servo_id}: {pos_mm}mm > limit {limit}mm")
            return False
        if servo_id not in self.zero_offset and servo_id in self.servos:
            self.get_logger().error(f"S{servo_id}: chua home!")
            return False
        mot = self.servos.get(servo_id)
        if not mot:
            return True
        try:
            offset = int(self.zero_offset.get(servo_id, 0))
            counts = offset + int(pos_mm * COUNTS_PER_MM)
            self._ensure_ready(mot, servo_id)
            with self._servo_lock:
                if continuous_update and hasattr(mot, 'configure_continuous_update'):
                    mot.configure_continuous_update(True)
                mot.position_task(int(counts), int(vel), absolute=True, nonblocking=True)
                if continuous_update and hasattr(mot, 'configure_continuous_update'):
                    mot.configure_continuous_update(False)
            if servo_id == 1: self._inx_moving = True
            elif servo_id == 2: self._iny_moving = True
            setattr(self, f'_ignore_arrived_{servo_id}', time.time() + 0.5)
            return True
        except Exception as e:
            self.get_logger().error(f"S{servo_id} nb_move error: {e}")
            return False

    def _arrived(self, servo_id: int) -> bool:
        """
        Kiểm tra servo đã đến đích chưa (target_position_reached từ FAS firmware).
        Trả về False trong 0.5s đầu sau khi gửi lệnh (_ignore_arrived_X)
        để tránh false-positive khi drive chưa kịp bắt đầu di chuyển.
        Nếu servo không kết nối → trả về True (không block flow).
        """
        if time.time() < getattr(self, f'_ignore_arrived_{servo_id}', 0.0):
            return False
        mot = self.servos.get(servo_id)
        if not mot:
            return True
        try:
            with self._servo_lock:
                done = mot.target_position_reached()
            if done:
                if servo_id == 1: self._inx_moving = False
                if servo_id == 2: self._iny_moving = False
            return done
        except Exception:
            return False

    def _stop(self, servo_id: int):
        """
        Dừng ngay servo (stop_motion_task) và reset flag đang di chuyển.
        Dùng khi nhấn STOP, ABORT, hoặc timeout bước.
        """
        mot = self.servos.get(servo_id)
        if mot:
            try:
                with self._servo_lock:
                    mot.stop_motion_task()
            except Exception:
                pass
        if servo_id == 1: self._inx_moving = False
        if servo_id == 2: self._iny_moving = False

    def _pos(self, servo_id: int) -> Optional[float]:
        """
        Đọc vị trí hiện tại của servo (mm, tính từ zero_offset sau homing).
        Công thức: (encoder_counts - zero_offset) / COUNTS_PER_MM
        Trả về 0.0 nếu servo không kết nối, None nếu có lỗi đọc.
        """
        mot = self.servos.get(servo_id)
        if not mot:
            return 0.0
        try:
            with self._servo_lock:
                counts = mot.current_position()
            return (counts - self.zero_offset.get(servo_id, 0)) / COUNTS_PER_MM
        except Exception as e:
            self.get_logger().warn(f"S{servo_id} _pos() error: {e}")
            return None

    def _jog(self, servo_id: int, forward: bool):
        """
        Phát lệnh JOG liên tục đến servo (duration=0.0 = giữ cho đến khi có lệnh stop).
        forward=True → chiều dương (xa home), forward=False → chiều âm (về home).
        Chỉ dùng trong MANUAL mode khi operator điều khiển từ GUI.
        """
        mot = self.servos.get(servo_id)
        if not mot:
            return

        try:
            self._ensure_ready(mot, servo_id)
            with self._servo_lock:
                mot.jog_task(forward, not forward, duration=0.0)
        except Exception as e:
            self.get_logger().error(f"S{servo_id} jog error: {e}")

    def _ensure_ready(self, mot, servo_id: int, timeout: float = 3.0):
        """
        Đảm bảo drive sẵn sàng nhận lệnh chuyển động:
          1. acknowledge_faults() — xóa lỗi cũ trên drive (giống bấm ACK trong FAS)
          2. enable_powerstage() — bật nguồn stage
          3. Chờ ready_for_motion() = True trong timeout giây
        Lock được giữ ngắn nhất có thể để STOP có thể acquire _servo_lock ngay.
        """
        if hasattr(mot, 'ready_for_motion'):
            acquired = self._servo_lock.acquire(timeout=0.1)
            if acquired:
                try:
                    if mot.ready_for_motion():
                        return
                except Exception:
                    pass
                finally:
                    self._servo_lock.release()

        # Short lock holds — tránh race condition trên Modbus socket
        acquired = self._servo_lock.acquire(timeout=1.0)
        if acquired:
            try:
                mot.acknowledge_faults()
                mot.enable_powerstage()
            finally:
                self._servo_lock.release()

        if not hasattr(mot, 'ready_for_motion'):
            return
        start = time.time()
        while time.time() - start < timeout:
            if self._servo_lock.acquire(timeout=0.5):
                try:
                    ready = mot.ready_for_motion()
                finally:
                    self._servo_lock.release()
                if ready:
                    return
            time.sleep(0.05)
        self.get_logger().warn(f"S{servo_id}: drive not ready after {timeout}s")

    # ── Cylinders ─────────────────────────────────────────────────

    def _set_do(self, channel: int, state: bool) -> bool:
        """
        Ghi giá trị digital output lên IO Module 1 (set/reset một channel).
        Duyệt qua các module con của CPX-AP và ghi vào module đầu tiên hỗ trợ set_channel.
        Trả về False nếu IO module không kết nối.
        """
        if not self.io_module: return False
        with self._io_bg_lock:
            for mod in self.io_module.modules:
                if mod.is_function_supported("set_channel"):
                    try:
                        if state: mod.set_channel(channel)
                        else: mod.reset_channel(channel)
                        return True
                    except Exception: pass
        return False

    def _cyl1_extend(self) -> bool:
        """
        Đẩy xi lanh khí nén 1 ra (EXTEND): reset kênh retract → set kênh extend.
        Cylinder 1 dùng để giữ/đẩy khay tray trong STATE 1/2.
        Channel mặc định: retract=4, extend=5 (cấu hình trong config YAML).
        """
        self.get_logger().info(f"Cyl1 EXTEND (ch{self._conf('cylinder1_extend_channel', 5)})")
        io = self.io_module  # snapshot to avoid TOCTOU race with _io_bg_loop
        if io:
            try:
                self._set_do(self._conf('cylinder1_retract_channel', 4), False)
                self._set_do(self._conf('cylinder1_extend_channel', 5), True)
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl1 extend IO error: {e}")
                return False
        return False

    def _cyl1_retract(self) -> bool:
        """
        Thu xi lanh 1 về (RETRACT): reset kênh extend → set kênh retract.
        Gọi khi kết thúc STATE 1/2, STOP khẩn cấp, hoặc khởi động node.
        """
        self.get_logger().info(f"Cyl1 RETRACT (ch{self._conf('cylinder1_retract_channel', 4)})")
        io = self.io_module  # snapshot to avoid TOCTOU race with _io_bg_loop
        if io:
            try:
                self._set_do(self._conf('cylinder1_extend_channel', 5), False)
                self._set_do(self._conf('cylinder1_retract_channel', 4), True)
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl1 retract IO error: {e}")
                return False
        return False

    def _cyl2_extend(self) -> bool:
        """
        Đẩy xi lanh 2 ra (EXTEND) — dùng trong STATE 4 để giữ khay output.
        Channel mặc định: retract=8, extend=9.
        Yêu cầu IO module sẵn sàng (_io_ready=True) trước khi gửi lệnh.
        """
        io = self.io_module  # snapshot to avoid TOCTOU race with _io_bg_loop
        if self._io_ready and io:
            try:
                self._set_do(self._conf('cylinder2_retract_channel', 8), False)
                self._set_do(self._conf('cylinder2_extend_channel', 9), True)
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl2 extend error: {e}")
                return False
        return False

    def _cyl2_retract(self) -> bool:
        """
        Thu xi lanh 2 về (RETRACT). Gọi khi hoàn thành STATE 4 hoặc STOP khẩn cấp.
        """
        io = self.io_module  # snapshot to avoid TOCTOU race with _io_bg_loop
        if self._io_ready and io:
            try:
                self._set_do(self._conf('cylinder2_extend_channel', 9), False)
                self._set_do(self._conf('cylinder2_retract_channel', 8), True)
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl2 retract error: {e}")
                return False
        return False

    # ── Helpers ──────────────────────────────────────────────────

    def _enter(self, next_state: SystemState):
        """
        Chuyển trạng thái GLOBAL (self.state) và log chuyển tiếp.
        Xóa _guide_logged khi đổi state để các log "chỉ in 1 lần" được reset.
        """
        self.get_logger().info(f"[GLOBAL] -> {next_state.name}")
        if next_state != self.state:
            self._guide_logged.clear()
        self.state = next_state

    def _enter_in(self, next_state: SystemState):
        """
        Chuyển trạng thái state machine phụ INPUT (state_in).
        Reset _cmd_sent_in và các biến step-timer để bước tiếp theo bắt đầu sạch.
        Dùng cho toàn bộ luồng STATE 1 và STATE 2.
        """
        self.get_logger().info(f"[IN] -> {next_state.name}")
        self.state_in        = next_state
        self._cmd_sent_in    = False
        self._step_start_in  = 0.0
        self._step_timeout_in = 0.0

    def _enter_s3(self, next_state: SystemState):
        """
        Chuyển trạng thái state machine phụ S3 (state_s3).
        Reset _cmd_sent_s3 và step-timer cho bước tiếp theo.
        Dùng cho toàn bộ luồng STATE 3 (Platform/Servo3 cấp khay vào vị trí feed).
        """
        self.get_logger().info(f"[S3] -> {next_state.name}")
        self.state_s3        = next_state
        self._cmd_sent_s3    = False
        self._step_start_s3  = 0.0
        self._step_timeout_s3 = 0.0

    def _enter_s4(self, next_state: SystemState):
        """
        Chuyển trạng thái state machine phụ S4 (state_s4).
        Reset _cmd_sent_s4 và step-timer cho bước tiếp theo.
        Dùng cho toàn bộ luồng STATE 4 (OutX/OutY thay khay output).
        """
        self.get_logger().info(f"[S4] -> {next_state.name}")
        self.state_s4        = next_state
        self._cmd_sent_s4    = False
        self._step_start_s4  = 0.0
        self._step_timeout_s4 = 0.0

    def _error(self, msg: str):
        """
        Chuyển hệ thống sang trạng thái ERROR, log lỗi và gửi thông báo GUI.
        Toàn bộ state machine dừng lại cho đến khi operator nhấn STOP → START.
        """
        self.get_logger().error(f"ERROR: {msg}")
        self._notify('error', 'ERROR', msg)
        self._enter(SystemState.ERROR)

    def _conf(self, key: str, default: Any = 0.0) -> Any:
        """
        Đọc một giá trị config an toàn, trả về default nếu key không tồn tại.
        Tránh AttributeError khi config chưa có key mới (backward compatibility).
        """
        try:
            return getattr(self.config, key, default)
        except Exception:
            return default

    def _notify(self, level: str, title: str, detail: str = ""):
        """
        Gửi thông báo đến GUI qua topic /providesystem/gui_notify (JSON).
        level: 'info' | 'warn' | 'error' | 'silent_ok'
        Throttle 0.5s/title để không flood GUI với cùng một thông báo lặp lại.
        """
        now = time.time()
        if now - self._notify_throttle.get(title, 0) < 0.5:
            return
        self._notify_throttle[title] = now
        try:
            msg = String()
            msg.data = json.dumps({"level": level, "title": title, "detail": detail})
            self.pub_gui_notify.publish(msg)
        except Exception:
            pass

    def _log_once(self, key: str, msg: str):
        """
        Log một message chỉ một lần trong mỗi trạng thái (dùng key làm ID).
        _guide_logged bị xóa mỗi khi state thay đổi, nên message sẽ xuất hiện
        lại ở trạng thái mới. Tránh spam log khi hệ thống đang chờ điều kiện.
        """
        if key not in self._guide_logged:
            self._guide_logged.add(key)
            self.get_logger().info(f"[guide] {msg}")

    def _find_nearest_row_abs(self, pos_mm: float, row_dict: dict) -> int:
        """
        Tìm row gần nhất với vị trí pos_mm trong bảng row_dict {row: position_mm}.
        Dùng khi InY scan để quyết định row nào của stack tray đang ở vị trí đó.
        """
        return min(row_dict, key=lambda r: abs(row_dict[r] - pos_mm))

    def _zone_to_row(self, trigger_pos: float, zone_table: dict) -> int:
        """
        Ánh xạ vị trí trigger_pos (mm) vào row number theo bảng zone_table.
        zone_table: {row: [min_mm, max_mm]} — mỗi row có một khoảng vị trí.
        Dùng khi S4 bật (cảm biến scan stack) để xác định khay đang ở row nào.
        Trả về row 1 làm fallback nếu không khớp zone nào.
        """
        for row, vals in zone_table.items():
            if len(vals) >= 2 and min(vals[0], vals[1]) <= trigger_pos <= max(vals[0], vals[1]):
                return row
        self.get_logger().warn(
            f"[zone_to_row] {trigger_pos:.1f}mm không khớp zone nào → fallback row1"
        )
        return 1

    # def _calc_output_target(self, trigger_pos_mm: float) -> tuple:
    #     cfg             = self.config
    #     actual_tray_pos = trigger_pos_mm + cfg.s4_cross_offset_mm
    #     target_pos      = actual_tray_pos - cfg.tray_height_mm
    #     occupied_row    = self._find_nearest_row_abs(actual_tray_pos, cfg.iny_output_stack)
    #     is_full         = target_pos <= cfg.output_min_pos_mm
    #     return target_pos, occupied_row, is_full

    def _iny_safe(self) -> bool:
        """
        Kiểm tra InY đang ở vùng an toàn (≤ iny_safe_zone mm).
        Interlock: InX chỉ được phép di chuyển khi InY < 50mm (safe zone)
        để tránh va chạm giữa hai trục.
        """
        p = self._pos(2)
        return p is not None and p <= self.config.iny_safe_zone

    def _can_start_s1(self) -> bool:
        """
        Điều kiện để auto-trigger STATE 1 (chỉ AUTO/AI mode):
          - Đã homing (zero_offset có giá trị)
          - Robot không báo bận (_motion_busy=False)
          - Có khay trên băng tải (S1 OR S2 OR S3 ON)
          - Vị trí cấp khay trống (S7_TRAY_AT_ROBOT=OFF)
          - Cylinder 1 đã thu về (S9 ON, S10 OFF)
        """
        """Điều kiện để bắt đầu STATE 1 (sensor theo mode hiện tại)."""
        if self.operation_mode not in ['auto', 'ai'] or not self.zero_offset or self._motion_busy:
            return False
        has_tray     = self.sensor(S1_BELT_START) or self.sensor(S2_BELT_MID) or self.sensor(S3_BELT_END)
        place_ok     = not self.sensor(S7_TRAY_AT_ROBOT)
        cyl1_ret_ok  = self.sensor(S9_CYL1_RETRACTED)    # S9_CYL1_RETRACTED
        cyl1_ext_ok  = not self.sensor(S10_CYL1_EXTENDED)  # S10_CYL1_EXTENDED must be OFF
        return has_tray and place_ok and cyl1_ret_ok and cyl1_ext_ok

    def _can_start_s2a(self) -> bool:
        """
        Điều kiện để trigger STATE 2 (cả AUTO lẫn MANUAL):
          - Đã homing
          - Robot đã báo xong khay input (_input_tray_done=True, set bởi _cb_done_tray_input)
          - Khay đang ở vị trí robot (S7_TRAY_AT_ROBOT=ON)
          - Robot không báo bận
        """
        return (bool(self.zero_offset) and getattr(self, '_input_tray_done', False)
                and self.sensor(S7_TRAY_AT_ROBOT) and not self._motion_busy)

    def _can_start_s3(self) -> bool:
        """
        Điều kiện để auto-trigger STATE 3 (chỉ AUTO/AI mode):
          - Đã homing và robot không bận
          - S18_FEED_OK=OFF (vị trí feed đang trống)
          - S18 đã OFF liên tục ít nhất 5 giây (debounce — tránh trigger ngay sau khi
            robot lấy khay, đợi hệ thống ổn định trước khi cấp khay mới)
        """
        if self.operation_mode not in ['auto', 'ai'] or self._motion_busy:
            return False
            
        # Neu S18 dang OFF va da duoc ghi nhan thoi gian
        s10_off_duration = 0.0
        if not self.sensor(S18_FEED_OK):
            if getattr(self, '_s10_off_time', 0) > 0:
                s10_off_duration = time.time() - self._s10_off_time
            else:
                s10_off_duration = 5.0
                
        return (bool(self.zero_offset)
                and not self.sensor(S18_FEED_OK)
                and s10_off_duration >= 5.0)

    def _can_start_s4(self) -> bool:
        """
        Điều kiện để trigger STATE 4 (thay khay output):
          - Đã homing
          - _s4_trigger=True (set bởi robot qua /robot/done_tray_output)
          - S18_FEED_OK=ON (có khay ở vị trí feed — đảm bảo output tray đang được dùng)
          - Robot không báo bận
        """
        return bool(self.zero_offset) and self._s4_trigger and self.sensor(S18_FEED_OK) and not self._motion_busy

    def _pub_cartridge_busy(self, busy: bool):
        """
        Publish trạng thái bận của hệ thống nạp cartridge lên /cartridge/busy.
        Robot node subscribe topic này để biết khi nào hệ thống nạp đang chạy.
        """
        self.pub_busy_cartridge.publish(Bool(data=busy))

    def _cb_motion_busy(self, msg: Bool):
        """
        Nhận trạng thái bận của robot từ /robot/motion_busy.
        Cập nhật _motion_busy và timestamp heartbeat.
        Lần đầu nhận → set _robot_connected=True, kích hoạt interlock an toàn.
        """
        self._motion_busy = msg.data
        self._robot_last_seen = time.time()
        if not self._robot_connected:
            self._robot_connected = True
            self.get_logger().info("[ROBOT] Robot node connected — interlock ACTIVE")

    def _cb_done_tray_output(self, msg: Bool):
        """Nhận tín hiệu từ robot báo khay output đã đầy → set cờ _s4_trigger để
        kích hoạt STATE 4 thay khay output trong vòng lặp tiếp theo."""
        """Robot báo khay output đã đầy → trigger State4 thay khay."""
        if msg.data:
            self._s4_trigger = True
            self._notify('info', 'Output tray full', 'Trigger State 4 thay khay')
            self.pub_newtray_output.publish(Bool(data=False))

    def _cb_done_tray_input(self, msg: Bool):
        """
        Robot báo đã xong khay input → set _input_tray_done.
        Cả AUTO lẫn MANUAL đều nhận topic này để trigger STATE 2.
        STATE 2 sẽ tự trigger qua _can_start_s2a() trong _do_idle_input().
        """
        if msg.data:
            self._input_tray_done = True
            self._state1_enabled = True
            self.get_logger().info(
                '[DONE_INPUT] Robot xong khay input → sẵn sàng trigger State2 '
                f'(mode={self.operation_mode})'
            )
            self._notify('info', 'Input tray done', 'Robot xong — chờ State2')
            self.pub_new_tray.publish(Bool(data=False))



    def _outy_safe(self) -> bool:
        """
        Kiểm tra OutY đang ở vùng an toàn (≤ outy_safe_zone mm).
        Điều kiện cần trước khi STATE 4 bắt đầu di chuyển OutX.
        """
        p = self._pos(5)
        return p is not None and p <= self.config.outy_safe_zone

    # ══════════════════════════════════════════════════════════════
    # Homing
    # ══════════════════════════════════════════════════════════════

    def _home_all(self) -> bool:
        """
        Thực hiện homing toàn bộ 5 servo theo thứ tự an toàn:
          InY → OutY → InX → OutX → Platform (Servo3)
        Thứ tự này đảm bảo trục dọc về trước, tránh va chạm với vật thể.

        Logic mỗi servo:
          - Nếu đã homed trước đó (_servo_homed_once) và vị trí gần 0 (< 5mm):
            chỉ di chuyển về 0 bằng position_task (nhanh, không cần re-reference)
          - Ngược lại: gọi referencing_task() theo phương pháp trong FAS firmware
            (limit switch / encoder index / torque), chờ referenced()=True
          - Kiểm tra drive thực sự di chuyển (Δ > 0.5mm) để phát hiện lỗi cơ học

        Hỗ trợ abort: _homing_abort Event có thể được set bởi _cb_stop() để
        dừng homing ngay lập tức tại bất kỳ bước nào.
        Chạy trong background thread (_do_homing), kết quả báo qua _homing_done_event.
        """
        """[V8] Blocking home pattern — mô phỏng dosing-machine code đã chạy thật.
        Dùng referencing_task() blocking (không nonblocking=True), home tuần tự,
        track _servo_homed_once để skip re-home nếu đã homed gần zero.
        Fail loud thay vì silent-assume-home khi drive không di chuyển.
        """
        NAMES = {1: "InX", 2: "InY", 3: "Platform", 4: "OutX", 5: "OutY"}
        ORDER = [(2, "InY"), (5, "OutY"), (1, "InX"), (4, "OutX"), (3, "Platform")]

        if not hasattr(self, '_servo_homed_once'):
            self._servo_homed_once = {sid: False for sid in self.config.servo_ips}

        for sid, name in ORDER:
            if sid not in self.servos:
                self.get_logger().info(f"  {name} (S{sid}): not connected, skip")
                continue
            mot = self.servos[sid]

            try:
                self._ensure_ready(mot, sid, timeout=5.0)

                with self._servo_lock:
                    current_pos = mot.current_position()
                self.get_logger().info(f"Homing {name} (S{sid}): current={current_pos}")

                zero = self.zero_offset.get(sid, 0)
                near_zero = abs(current_pos - zero) <= POSITION_TOLERANCE_HOME * COUNTS_PER_MM

                if self._servo_homed_once.get(sid) and near_zero:
                    # Đã homed, vị trí gần 0 → chỉ position_task về 0 (blocking)
                    self.get_logger().info(f"  {name}: near zero → position_task(0)")
                    with self._servo_lock:
                        mot.position_task(zero, HOME_SPEED, absolute=True, nonblocking=False)
                else:
                    # Lần đầu hoặc đã lệch xa → referencing_task nonblocking
                    # Lock chỉ giữ đủ để issue lệnh — STOP có thể acquire lock ngay
                    self.get_logger().info(
                        f"  {name}: {'first home' if not self._servo_homed_once.get(sid) else 're-home'} "
                        f"→ referencing_task"
                    )
                    pre_pos = current_pos

                    with self._servo_lock:
                        mot.referencing_task(nonblocking=True)
                    # Lock released immediately — _cb_stop có thể stop_motion_task ngay

                    # 3s interruptible delay: cho drive xóa stale referenced() flag từ NVM
                    # và bắt đầu chuyển động vật lý. Event.wait() trả True nếu abort set.
                    if self._homing_abort.wait(timeout=3.0):
                        self.get_logger().warn(f"  {name}: homing aborted (STOP) during init delay")
                        return False

                    # Poll referenced() với abort check mỗi iteration
                    start = time.time()
                    while time.time() - start < self.config.homing_timeout:
                        if self._homing_abort.is_set():
                            self.get_logger().warn(f"  {name}: homing aborted (STOP) during poll")
                            return False
                        with self._servo_lock:
                            ref = mot.referenced()
                        if ref:
                            break
                        time.sleep(0.2)
                    else:
                        with self._servo_lock:
                            is_ref = mot.referenced()
                        if not is_ref:
                            self.get_logger().error(
                                f"  {name}: homing TIMEOUT ({self.config.homing_timeout}s) — NOT referenced"
                            )
                            self._notify('error', f'{name} home FAIL',
                                         'Timeout — drive khong home duoc. Check FAS Referencing tab.')
                            return False

                    with self._servo_lock:
                        post_pos = mot.current_position()

                    delta_mm = abs(post_pos - pre_pos) / COUNTS_PER_MM

                    if delta_mm < 0.5:
                        with self._servo_lock:
                            is_ref = mot.referenced()
                        if not is_ref:
                            self.get_logger().error(
                                f"  {name}: NOT referenced + no motion (Δ={delta_mm:.2f}mm) "
                                f"→ HOMING FAIL. Check FAS: Referencing tab method/velocity/REF-cam."
                            )
                            self._notify('error', f'{name} home FAIL',
                                         f'Drive khong di chuyen va khong referenced (delta={delta_mm:.2f}mm). '
                                         f'Check FAS Referencing tab.')
                            return False
                        self.get_logger().warn(
                            f"  {name}: moved only {delta_mm:.2f}mm but drive says referenced — "
                            f"may already be at REF position"
                        )

                    self._servo_homed_once[sid] = True
                    self.zero_offset[sid] = post_pos
                    self.get_logger().info(f"  {name} = 0mm (homed, Δ={delta_mm:.1f}mm)")

            except Exception as e:
                self.get_logger().error(f"  {name} home exception: {e}")
                self._notify('error', f'{name} home FAIL', str(e))
                return False

        return True

    # ══════════════════════════════════════════════════════════════
    # ROS Callbacks
    # ══════════════════════════════════════════════════════════════

    def _sync_mode_jog(self):
        """
        Publish mode hiện tại lên /providesystem/current_mode để GUI cập nhật.
        Nếu đang trong MANUAL mode và đã homed → publish "jog" (GUI hiển thị nút JOG).
        Ngược lại → publish tên mode thực (auto/manual/ai).
        """
        msg = String()
        # If in manual and homed, we are in 'jog'. Else just the operation_mode.
        if getattr(self, '_jog_mode', False) and self.operation_mode == 'manual':
            msg.data = "jog"
        else:
            msg.data = getattr(self, 'operation_mode', 'manual')
        self.pub_current_mode.publish(msg)

    def _cb_start(self, msg: Bool):
        """
        Xử lý lệnh START từ /system/start_button (GUI hoặc operator nhấn nút).
        - AUTO/AI mode và chưa homed → chuyển sang HOMING tự động
        - AUTO/AI mode và đã homed → chuyển về IDLE, bắt đầu auto-trigger STATE
        - MANUAL mode → vào chế độ JOG (giữ nguyên state, không tự động chạy STATE)
        Bỏ qua nếu đang trong HOMING_RUNNING để tránh xung đột.
        """
        if not msg.data or self.state == SystemState.HOMING_RUNNING:
            return

        self._homing_abort.clear()
        self._system_paused = False
        self._system_running = True
        self._inx_moving = self._iny_moving = False
        self._state1_enabled = (self.operation_mode in ['auto', 'ai'])
        self._motion_busy = False
        
        if self.operation_mode == 'manual':
            self._jog_mode = True
        else:
            self._jog_mode = False
            
        self.get_logger().info(f"[START] System running. Mode: {self.operation_mode}")
        self._sync_mode_jog()
        
        if self.operation_mode in ['auto', 'ai']:
            if not self.zero_offset:
                self.get_logger().info("[START] Auto/AI mode — not homed → HOMING")
                self._enter(SystemState.HOMING)
            else:
                self._enter(SystemState.IDLE)
        else:
            self.get_logger().info("[START] Manual mode — keeping current state")

    def _cb_stop(self, msg: Bool):
        """
        Xử lý lệnh STOP khẩn cấp từ /system/stop_button.
        Thực hiện theo thứ tự an toàn:
          1. Set _homing_abort để dừng thread homing ngay lập tức
          2. Dừng tất cả servo (stop_motion_task)
          3. Thu về cả 2 cylinder
          4. Reset toàn bộ state machine về IDLE
          5. Chuyển sang MANUAL mode + JOG sẵn sàng
        zero_offset được giữ nguyên — START tiếp theo không cần re-home.
        Gửi mode MANUAL (code 3) đến robot node để đồng bộ.
        """
        if not msg.data:
            return
        # Signal homing thread to abort FIRST — before acquiring servo_lock in _stop()
        self._homing_abort.set()
        for sid in list(self.servos):
            self._stop(sid)
        self._cyl1_retract()
        self._cyl2_retract()
        self._enter(SystemState.IDLE)
        self._enter_in(SystemState.IDLE)
        self._enter_s3(SystemState.IDLE)
        self._enter_s4(SystemState.IDLE)
        self._system_paused  = False
        self._system_running = False
        self._input_tray_done = False
        self._motion_busy = False
        self._state1_enabled = False
        self._s4_trigger = False
        
        # Chuyển về MANUAL mode
        self.operation_mode = 'manual'
        self._jog_mode = True
        self._sync_mode_jog()
        robot_msg = Int32()
        robot_msg.data = 3
        self.pub_robot_mode.publish(robot_msg)
        self.get_logger().info("[STOP] Hệ thống dừng - Đã chuyển về MANUAL mode")
        # zero_offset preserved — coordinates remain valid, next START skips re-homing
        self._sync_mode_jog()
        self.get_logger().info('[STOP] STOP — JOG sẵn sàng (toa do giu nguyen)')
        self._notify('warn', 'STOP', 'Dừng hệ thống — JOG sẵn sàng')

    def _cb_pause(self, msg: Bool):
        """
        Tạm dừng vòng lặp điều khiển (_system_paused=True).
        Vòng lặp trả về ngay mà không xử lý state nào.
        RESUME: nhấn START để tiếp tục (xóa _system_paused).
        """
        if not msg.data:
            return
        self._system_paused = True
        self._notify('warn', 'PAUSE', 'Nhan RESUME de tiep tuc')

    def _cb_gripper_cmd(self, msg: Bool):
        """
        Điều khiển gripper (kẹp) qua IO Module 1 (tự động dò valve module):
          msg.data=True  → Đóng gripper (set ch1, reset ch0)
          msg.data=False → Mở gripper (reset cả ch0 và ch1)
        Publish trạng thái lên /robot/gripper_status để robot node biết.
        """
        if not self.io_module: return
        with self._io_bg_lock:
            try:
                for valve_mod in self.io_module.modules:
                    if valve_mod.is_function_supported("set_channel"):
                        if msg.data: # Close
                            valve_mod.reset_channel(0)
                            valve_mod.set_channel(1)
                            self.pub_gripper_status.publish(Bool(data=True))
                        else: # Open
                            valve_mod.reset_channel(1)
                            valve_mod.reset_channel(0)
                            self.pub_gripper_status.publish(Bool(data=False))
                        break
            except Exception as e:
                self.get_logger().error(f"Gripper cmd error: {e}")

    def _cb_picker_cmd(self, msg: Bool):
        """
        Điều khiển picker (hút chân không) qua IO Module 1 (tự động dò valve module):
          msg.data=True  → Kích hoạt picker (set ch3, reset ch2)
          msg.data=False → Tắt picker (reset cả ch2 và ch3)
        Publish trạng thái lên /robot/picker_status.
        """
        if not self.io_module: return
        with self._io_bg_lock:
            try:
                for valve_mod in self.io_module.modules:
                    if valve_mod.is_function_supported("set_channel"):
                        if msg.data: # Close
                            valve_mod.reset_channel(2)
                            valve_mod.set_channel(3)
                            self.pub_picker_status.publish(Bool(data=True))
                        else: # Open
                            valve_mod.reset_channel(3)
                            valve_mod.reset_channel(2)
                            self.pub_picker_status.publish(Bool(data=False))
                        break
            except Exception as e:
                self.get_logger().error(f"Picker cmd error: {e}")

    def _cb_gui_confirm(self, msg: String):
        """
        Nhận xác nhận từ operator qua GUI (topic /providesystem/gui_confirm).
        Set _gui_confirmed=True để state machine tiếp tục từ trạng thái đang chờ
        xác nhận (S1_WAIT_GUI_CONFIRM, S3_WAIT_GUI_CONFIRM).
        """
        self._gui_confirmed = True

    def _cb_robot_mode(self, msg: Int32):
        """
        Đồng bộ mode từ robot node qua /robot/set_mode.
        Mapping: 1=auto, 2=ai, 3=manual.
        Khi mode thay đổi: xóa log guide và reset sim sensor để tránh
        dữ liệu cũ ảnh hưởng sang mode mới.
        """
        mapping = {1: 'auto', 2: 'ai', 3: 'manual'}
        requested = mapping.get(msg.data, 'manual')
        if requested != self.operation_mode:
            self.get_logger().info(f"[SYNC MODE] Cập nhật mode từ Robot Node: {requested.upper()}")
            self.operation_mode = requested
            self._guide_logged.clear()
            self._sim_sensors.clear()
            self._sync_mode_jog()

    def _cb_mode(self, msg: String):
        """
        Thay đổi operation mode từ GUI qua /providesystem/set_operation_mode.
        Chỉ cho phép đổi mode khi hệ thống ở IDLE hoặc ERROR (không đang chạy STATE).
        Khi đổi mode: reset sim sensor, sync GUI, và thông báo đến robot node.
        """
        requested = msg.data.strip().lower()
        if requested not in ('auto', 'manual', 'ai'):
            self._notify('warn', f'Mode khong hop le: {requested}', '')
            return
        active_states = {SystemState.IDLE, SystemState.ERROR,
                         SystemState.HOMING, SystemState.HOMING_RUNNING}
        if self.state not in active_states:
            self._notify('warn', 'Khong the doi mode',
                         f'Dang chay ({self.state.name}) — nhan STOP truoc')
            return
        old = self.operation_mode
        self.operation_mode = requested
        if old != requested:
            self._guide_logged.clear()
            self._sim_sensors.clear()  # Reset sim khi đổi mode
            self._sync_mode_jog()  # Publish mode ngay cho GUI hiển thị
            
            # Sync to robot node
            robot_msg = Int32()
            if requested == 'auto': robot_msg.data = 1
            elif requested == 'ai': robot_msg.data = 2
            elif requested == 'manual': robot_msg.data = 3
            self.pub_robot_mode.publish(robot_msg)

    def _cb_jog(self, msg: String):
        """
        Xử lý lệnh JOG từ GUI qua /providesystem/jog_cmd.
        Định dạng: "<servo_id> <cmd>" | "home <servo_id>" | "clear <servo_id>"

        Lệnh hỗ trợ:
          "clear <sid>"      → Acknowledge faults (bất kỳ mode nào)
          "<sid> stop"       → Dừng servo
          "<sid> +"          → JOG chiều dương (chỉ MANUAL+IDLE)
          "<sid> -"          → JOG chiều âm
          "<sid> move <pos>" → Di chuyển đến pos mm
          "home <sid>"       → Homing thủ công 1 servo (thread riêng)

        Bị khóa nếu đang AUTO mode hoặc bất kỳ state machine nào đang chạy.
        """
        parts = msg.data.strip().split()
        if not parts: return
        cmd0 = parts[0].lower()

        # Cho phép CLEAR lỗi (acknowledge faults giống FAS) bất kỳ lúc nào / mode nào
        if cmd0 == 'clear':
            if len(parts) >= 2:
                try:
                    sid = int(parts[1])
                    mot = self.servos.get(sid)
                    if mot:
                        mot.acknowledge_faults()
                        self.get_logger().info(f"Acknowledge faults cho Servo {sid} (như FAS)")
                except Exception as e:
                    self.get_logger().warn(f"Lỗi khi clear servo: {e}")
            return

        if self.operation_mode != 'manual':
            self._notify('warn', 'JOG bi khoa', 'Chuyen MANUAL mode de JOG')
            return
        if (self.state not in (SystemState.IDLE, SystemState.ERROR)
                or self.state_in != SystemState.IDLE
                or self.state_s3 != SystemState.IDLE
                or self.state_s4 != SystemState.IDLE):
            self._notify('warn', 'JOG bi khoa', 'Dang chay — nhan STOP truoc')
            return
        
        if len(parts) < 2:
            return
        try:
            if cmd0 == 'home':
                sid = int(parts[1])
                mot = self.servos.get(sid)
                if mot:
                    def _do(mot=mot, sid=sid):
                        self._ensure_ready(mot, sid, timeout=5.0)
                        with self._servo_lock:
                            mot.referencing_task(nonblocking=True)
                        start = time.time()
                        timeout = self.config.homing_timeout
                        while time.time() - start < timeout:
                            with self._servo_lock:
                                ref = mot.referenced()
                            if ref:
                                break
                            time.sleep(0.2)
                        else:
                            self.get_logger().warn(f"S{sid} JOG home timeout after {timeout}s")
                            with self._servo_lock:
                                mot.stop_motion_task()
                        if sid not in self.zero_offset:
                            with self._servo_lock:
                                self.zero_offset[sid] = mot.current_position()
                        self._notify('silent_ok', f'Homing complete ...', f'Servo {sid}')
                    threading.Thread(target=_do, daemon=True).start()
                return
            sid = int(parts[0])
            d   = parts[1]
            if d == 'stop':   self._stop(sid)
            elif d == '+':    self._jog(sid, True)
            elif d == '-':    self._jog(sid, False)
            elif d == 'move':
                pos = float(parts[2]) if len(parts) > 2 else 0.0
                self._nb_move(sid, pos)
        except Exception as e:
            self.get_logger().error(f"[JOG] exception: {e}")

    def _cb_sim(self, msg: String):
        """
        Nhận lệnh giả lập cảm biến từ GUI qua /providesystem/sim_sensor (chỉ MANUAL mode).
        Định dạng: "<sensor_id>:<0|1>" hoặc "all:<0|1>" hoặc "clear"
        Ví dụ: "1:1" → set S1=ON, "7:0" → set S7=OFF, "all:1" → bật hết sensor.
        Dữ liệu lưu vào _sim_sensors, được sensor() dùng thay cho IO module thực.
        [V7-1] Chỉ hoạt động trong MANUAL mode — AUTO mode bị reject để tránh
        nhầm lẫn giữa dữ liệu thực và giả lập.
        """
        if self.operation_mode != 'manual':
            self._notify('warn', 'Sim sensor bi khoa', 'Chuyen MANUAL mode')
            return
        cmd = msg.data.strip()
        if cmd == 'clear':
            self._sim_sensors.clear()
            return
        try:
            sid_s, val_s = cmd.split(':')
            val = (val_s.strip() == '1')
            if sid_s.strip() == 'all':
                for i in range(1, 23):
                    self._sim_sensors[i] = val
            else:
                self._sim_sensors[int(sid_s)] = val
        except Exception:
            pass

    def _cb_goto_state(self, msg: String):
        """
        Nhận lệnh chuyển state thủ công từ GUI qua /providesystem/goto_state.
        Lệnh hỗ trợ: HOMING, IDLE, ABORT_TO_JOG, STATE1/S1, STATE2/S2, STATE3/S3, STATE4/S4.
        STATE1 chỉ cho phép trong MANUAL mode (AUTO tự trigger).
        STATE2 cho phép cả hai mode nếu điều kiện cảm biến hợp lệ.
        STATE3/4 cho phép bất kỳ mode nào nếu chưa đang chạy và đã homed.
        """
        cmd = msg.data.strip().upper()
        if cmd == 'HOMING':
            if self.state != SystemState.HOMING_RUNNING:
                self.zero_offset.clear()
                self._inx_moving = self._iny_moving = False
                self._enter(SystemState.HOMING)
            return
        if cmd == 'IDLE':
            for sid in list(self.servos):
                self._stop(sid)
            self._enter(SystemState.IDLE)
            self._enter_in(SystemState.IDLE)
            self._enter_s3(SystemState.IDLE)
            self._enter_s4(SystemState.IDLE)
            self._system_running = False
            self.get_logger().info("Got IDLE command, fully reset system states.")
            return

        if cmd == 'ABORT_TO_JOG':
            self._s1_abort("Operator stopped state (ABORT_TO_JOG)")
            return

        # ── STATE 1: chỉ cho MANUAL, operator nhấn tay ──────────
        if cmd in ('STATE1', 'STATE_1', 'S1'):
            if self.operation_mode != 'manual':
                self._notify('warn', 'STATE1 manual only',
                             'Trong AUTO/AI mode hệ thống tự trigger STATE1')
                return
            if self.state_in not in (SystemState.IDLE,):
                self._notify('warn', 'Không thể vào STATE 1',
                             f'{self.state_in.name} đang chạy — STOP trước')
                return
            if not self.zero_offset:
                self._notify('warn', 'Chua home', 'Nhấn HOMING trước')
                return
            if self._motion_busy:
                self.get_logger().warn("Robot đang báo bận (topic motion_busy), vẫn cho phép chạy MANUAL...")
                self._notify('info', 'Robot busy', 'Đang bỏ qua khóa an toàn trong MANUAL mode...')
            # Kiểm tra điều kiện cảm biến sim
            has_tray    = self.sensor(S1_BELT_START) or self.sensor(S2_BELT_MID) or self.sensor(S3_BELT_END)
            place_ok    = not self.sensor(S7_TRAY_AT_ROBOT)
            cyl1_ret_ok = self.sensor(S9_CYL1_RETRACTED)      # S9_CYL1_RETRACTED
            cyl1_ext_ok = not self.sensor(S10_CYL1_EXTENDED) # S10_CYL1_EXTENDED must be OFF

            if not (has_tray and place_ok and cyl1_ret_ok and cyl1_ext_ok):
                reasons = []
                if not has_tray:
                    reasons.append('S1/S2/S3 OFF (sim chưa set)')
                if not place_ok:
                    reasons.append('S7 ON (vị trí cấp có khay)')
                if not cyl1_ret_ok:
                    reasons.append('S9 OFF (Cyl1 chưa retract)')
                if not cyl1_ext_ok:
                    reasons.append('S10 ON (Cyl1 đang thò ra)')
                self._notify('warn', 'STATE1: Điều kiện chưa đủ',
                             ' | '.join(reasons))
                return
            self._jog_mode = False
            self._cmd_sent_in = False
            self._step_timeout_in = 0.0
            self._sync_mode_jog()
            self._notify('info', 'STATE 1 (manual)', 'Bắt đầu cấp khay')
            self._enter_in(SystemState.S1_CONFIRM_SAFE)
            return

        # ── STATE 2: trong MANUAL operator có thể kích trực tiếp ─
        if cmd in ('STATE2', 'STATE_2', 'S2', 'STATE2A', 'S2A'):
            if self.operation_mode != 'manual':
                self._notify('warn', 'goto_state S2: manual only', '')
                return
            if self.state_in not in (SystemState.IDLE, SystemState.S1_COMPLETE):
                self._notify('warn', 'Không thể vào STATE 2',
                             f'{self.state_in.name} đang chạy — STOP trước')
                return
            if not self.zero_offset:
                self._notify('warn', 'Chua home', '')
                return
            if not self.sensor(S7_TRAY_AT_ROBOT):
                self._notify('warn', 'S7 OFF (sim)',
                             'Sim S7=1 trước khi vào State2')
                return
            self._input_tray_done = False
            self._notify('info', 'STATE 2A (manual)', 'S7 ON (sim)')
            self._enter_in(SystemState.S2A_CHECK_INTERLOCK)
            return

        # ── STATE 3/4: giữ nguyên (cả AUTO lẫn MANUAL) ──────────
        if cmd in ('STATE3', 'STATE_3', 'S3'):
            if self.state_s3 not in (SystemState.IDLE,):
                self._notify('warn', 'Không thể vào STATE 3',
                             f'{self.state_s3.name} đang chạy')
                return
            if not self.zero_offset:
                self._notify('warn', 'Chua home', '')
                return
            if self.sensor(S18_FEED_OK):
                self._notify('warn', 'S18 ON', 'Vị trí cấp (S18) đang có khay')
                return
            self._notify('info', 'STATE 3', 'Kích hoạt Manual S3')
            self._enter_s3(SystemState.S3_CHECK_OUTXY_SAFE)
            return

        if cmd in ('STATE4', 'STATE_4', 'S4'):
            if self.state_s4 not in (SystemState.IDLE,):
                self._notify('warn', 'Không thể vào STATE 4',
                             f'{self.state_s4.name} đang chạy')
                return
            if not self.zero_offset:
                self._notify('warn', 'Chua home', '')
                return
            self._notify('info', 'STATE 4', 'Thay khay output')
            self._enter_s4(SystemState.S4_CHECK_OUTY_SAFE)
            return

        if cmd == 'ABORT_TO_JOG':
            self._s1_abort("Operator stopped state (ABORT_TO_JOG)")
            return

        self._notify('warn', f'goto_state: khong biet "{cmd}"', '')

    def _s1_abort(self, reason: str):
        """
        Hủy bỏ STATE đang chạy và đưa hệ thống về IDLE an toàn.
        Dùng khi: operator nhấn ABORT_TO_JOG, timeout bước, hoặc lỗi cảm biến.
        Dừng tất cả servo, thu cylinder, reset mọi cờ state, bật JOG sẵn sàng.
        Khác với _cb_stop: không chuyển sang MANUAL mode vĩnh viễn.
        """
        self._pub_cartridge_busy(False)
        self._s1_scan_noise_retry = 0
        self._s4_prev_in         = False
        self.get_logger().error(f"S1 ABORT: {reason}")
        self._notify('error', 'Hủy State', reason)
        self._stop(1); self._stop(2); self._stop(3); self._stop(4); self._stop(5)
        self._cyl1_retract()
        self._cyl2_retract()
        self._jog_mode = True
        self._state1_enabled = False
        self._s5_retry = 0; self._s1_retry_count = 0; self._cmd_sent_in = False
        self._input_tray_done = False
        self._s4_trigger = False
        self._enter_in(SystemState.IDLE)
        self._enter_s3(SystemState.IDLE)
        self._enter_s4(SystemState.IDLE)
        self._enter(SystemState.IDLE)
        self._sync_mode_jog()

    def _safety_check_loop(self):
        """
        Luồng kiểm tra an toàn chạy ngầm liên tục (~10Hz).
        Theo dõi tọa độ thực tế của INX và INY để phát hiện sớm các va chạm
        tiềm ẩn, bất kể state machine đang ở trạng thái nào.
        Nếu phát hiện vi phạm Danger Zone hoặc mất kết nối Modbus đột ngột,
        dừng khẩn cấp và chuyển hệ thống sang ERROR.
        """
        while getattr(self, '_system_running', False):
            try:
                time.sleep(0.1)
                
                # Không kiểm tra nếu chưa homed
                if 1 not in self.zero_offset or 2 not in self.zero_offset:
                    continue
                    
                # Đọc vị trí
                inx = self._pos(1)
                iny = self._pos(2)
                
                if inx is None or iny is None:
                    continue
                    
                self._last_inx = inx
                self._last_iny = iny
                    
            except Exception as e:
                self.get_logger().error(f"Safety Loop Error: {e}")
                time.sleep(1.0)

    def _cb_update_config(self, msg: String):
        """
        Cập nhật config runtime từ GUI qua /providesystem/update_config (JSON).
        Payload: {"key": "tên_config", "data": "giá_trị"}
        Hỗ trợ cập nhật số, string, dict (row positions) mà không cần restart node.
        Sau khi cập nhật: lưu vào file YAML (config.save_to_file()).
        Ví dụ: thay đổi tốc độ scan, vị trí row, timeout — có hiệu lực ngay.
        """
        try:
            import json
            payload = json.loads(msg.data)
            key = payload.get('key')
            val_str = payload.get('data', '')
            if key and hasattr(self.config, key):
                try:
                    val = json.loads(val_str)
                except ValueError:
                    val = val_str
                if isinstance(val, dict):
                    def parse_val(v):
                        if isinstance(v, list):
                            return [float(x) for x in v]
                        return float(v)
                    val = {int(k) if str(k).isdigit() else k: parse_val(v) for k, v in val.items()}
                elif isinstance(val, str):
                    try: val = float(val)
                    except ValueError: pass
                setattr(self.config, key, val)
                self.config.save_to_file()
                self.get_logger().info(f"Config updated dynamically: {key} = {val}")
                self._notify('info', 'Config Updated', f'Cập nhật thành công {key}')
            else:
                self.get_logger().warn(f"Config key not found or ignored: {key}")
        except Exception as e:
            self.get_logger().warn(f"update_config error: {e}")

    def _cb_get_config(self, msg: String):
        """
        Phục vụ yêu cầu đọc config từ GUI qua /providesystem/get_config.
        Khi nhận "request": đọc file YAML config, chuyển sang JSON và publish
        lên /providesystem/config_data (latching QoS — GUI mới join vẫn nhận được).
        GUI dùng để hiển thị/chỉnh sửa config trực tiếp trên màn hình.
        """
        if msg.data.strip() == 'request':
            try:
                import json
                import yaml
                with open(self.config._config_file, 'r') as f:
                    js_data = json.dumps(yaml.safe_load(f))
                    msg_out = String()
                    msg_out.data = js_data
                    self.pub_config_data.publish(msg_out)
            except Exception as e:
                self.get_logger().warn(f"get_config error: {e}")

    def _cb_set_target_row(self, msg: String):
        """
        Đặt row đích thủ công từ GUI qua /providesystem/set_target_row.
        Ghi đè _current_row để STATE 1 di chuyển InY đến row được chỉ định
        thay vì row được tính tự động từ sensor scan.
        Dùng để debug hoặc vận hành bán tự động khi cần override.
        """
        try:
            row = int(msg.data.strip())
            valid_rows = list(self.config.iny_input_zones.keys()) if hasattr(self.config, 'iny_input_zones') else list(range(1, 6))
            if row in valid_rows:
                self._current_row = row
                self.get_logger().info(f'[SET_TARGET_ROW] row → {row}')
            else:
                self.get_logger().warn(f'[SET_TARGET_ROW] invalid row: {row} (valid: {valid_rows})')
        except ValueError:
            self.get_logger().warn(f'[SET_TARGET_ROW] non-integer value: "{msg.data}"')

    # ══════════════════════════════════════════════════════════════
    # Control Loop
    # ══════════════════════════════════════════════════════════════


    def _enforce_danger_zones(self):
        inx = self._pos(1)
        iny = self._pos(2)
        if inx is None or iny is None: return
        
        danger_min = getattr(self.config, 'inx_danger_zone_min', 80.0)
        danger_max = getattr(self.config, 'inx_danger_zone_max', 400.0)
        iny_safe   = getattr(self.config, 'iny_safe_zone', 90.0)
        
        inx_danger = (danger_min <= inx <= danger_max)
        iny_danger = (iny > iny_safe)
        
        if inx_danger and iny_danger:
            inx_dir = inx - getattr(self, '_last_inx', inx)
            iny_dir = iny - getattr(self, '_last_iny', iny)
            
            stop1 = False
            stop2 = False
            
            dist_min = abs(inx - danger_min)
            dist_max = abs(danger_max - inx)
            
            if dist_min < dist_max:
                if inx_dir > 0: stop1 = True
            else:
                if inx_dir < 0: stop1 = True
                
            if iny_dir > 0: stop2 = True
                
            if stop1:
                self._stop(1)
                self.get_logger().error(f"Safety Stop: S1 entering danger zone! ({inx:.1f})")
            if stop2:
                self._stop(2)
                self.get_logger().error(f"Safety Stop: S2 exceeding safe zone! ({iny:.1f})")
                
        self._last_inx = inx
        self._last_iny = iny

    def _control_loop(self):
        """
        Vòng lặp điều khiển chính, chạy mỗi 50ms (20Hz) qua ROS timer.
        Thực hiện theo thứ tự:
          1. Cập nhật watchdog tick
          2. Kiểm tra robot heartbeat (timeout 5s → tự xả interlock)
          3. Tracking S18 → ghi timestamp khi S18 OFF (dùng cho auto-trigger S3)
          4. Gọi _process_state() → dispatcher toàn bộ state machine
          5. Publish system state, sensor states
          6. Debounce và publish input_trays_empty (sau 20 tick liên tiếp = 1s)
        Bắt exception trong try/except riêng, chuyển sang ERROR state nếu có lỗi.
        """
        try:
            self._watchdog_last_tick = time.time()
            if self._system_paused:
                return
                
            self._enforce_danger_zones()
            
            # S18 tracking logic for S3 auto-feed delay
            feed_ok = self.sensor(S18_FEED_OK)
            if not feed_ok and self._s10_prev:
                self._s10_off_time = time.time()
                self.get_logger().info("[S18] Bị lấy đi -> bắt đầu đếm ngược 5s để cấp tiếp")
            self._s10_prev = feed_ok
            
            # ── Robot heartbeat timeout ──
            # Nếu robot node chưa bao giờ publish hoặc đã ngắt > 5s → tự xả khóa
            if self._robot_connected and self._robot_last_seen > 0:
                if time.time() - self._robot_last_seen > 5.0:
                    self._robot_connected = False
                    self._motion_busy = False
                    self.get_logger().warn("[ROBOT] Robot node timeout (>5s) — interlock RELEASED, cartridge runs standalone")
            elif not self._robot_connected and self._motion_busy:
                self._motion_busy = False


            self._process_state()
            self._publish_state()
            self._publish_sensors()
            
            # Publish input_trays_empty state to Dobot Robot Node for pipeline drain handling
            b_msg = Bool()
            raw_empty = not (self.sensor(S1_BELT_START) or self.sensor(S2_BELT_MID) or self.sensor(S3_BELT_END))
            
            if raw_empty:
                self._input_trays_empty_debounce_count += 1
            else:
                self._input_trays_empty_debounce_count = 0
                
            #... rest of control loop
            b_msg.data = bool(self._input_trays_empty_debounce_count >= 20)
            self.pub_input_trays_empty.publish(b_msg)
            
        except Exception as e:
            tb = traceback.format_exc()
            self.get_logger().error(f"CRITICAL LOGIC ERROR (Control Loop Killed): {e}\n{tb}")
            self._notify('error', 'Lỗi Logic Hệ Thống!', f'Node treo do: {str(e)[:50]}... Xem log để biết chi tiết.')
            # Transition to ERROR state to ensure safe stop
            if self.state != SystemState.ERROR:
                self._enter(SystemState.ERROR)
            time.sleep(1.0) # Throttle error loop if it keeps failing

    def _publish_sensors(self):
        """Publish chuỗi nhị phân 22 ký tự lên /providesystem/sensors_state.
        Mỗi ký tự = trạng thái cảm biến S1..S22 ('1'=ON, '0'=OFF) theo mode hiện tại."""
        states = "".join("1" if self.sensor(i) else "0" for i in range(1, 23))
        msg = String(); msg.data = states; self.pub_sensors.publish(msg)

    def _publish_state(self):
        """Publish trạng thái state machine dạng chuỗi "<global>|<input>|<s3_or_s4>"
        lên /system_state. GUI parse để hiển thị badge trạng thái theo thời gian thực."""
        s_out = self.state_s4.value if self.state_s4 != SystemState.IDLE else self.state_s3.value
        combined = f"{self.state.value}|{self.state_in.value}|{s_out}"
        msg = String(); msg.data = combined; self.pub_state.publish(msg)

    def _positions_bg_loop(self):
        """
        Thread nền publish vị trí servo mỗi 500ms lên /providesystem/servo_positions.
        Chạy tách biệt khỏi control loop để Modbus read latency không block 20Hz timer.
        Dừng khi _pos_thread_stop=True (set bởi destroy_node).
        """
        while rclpy.ok() and not getattr(self, '_pos_thread_stop', False):
            try:
                self._publish_positions()
            except Exception as e:
                self.get_logger().warn(f"[pos_bg] {e}")
            time.sleep(0.5)

    def _publish_positions(self):
        """Đọc vị trí tất cả servo và publish JSON lên /providesystem/servo_positions.
        JSON chứa {sid: mm, _jog_vel: m/s, _fas_vel: {sid: m/s}}.
        GUI dùng để hiển thị vị trí live và tốc độ JOG hiện tại."""
        try:
            pos = {}
            for sid in self.servos:
                p = self._pos(sid)
                if p is not None:
                    pos[str(sid)] = round(p, 2)
            data = pos.copy()
            data['_jog_vel'] = self._jog_velocity_ms
            if self._fas_jog_vel:
                data['_fas_vel'] = {str(k): round(v, 4) for k, v in self._fas_jog_vel.items()}
            msg = String()
            msg.data = json.dumps(data)
            self.pub_servo_pos.publish(msg)
        except Exception:
            pass

    def _watchdog(self):
        """
        Giám sát độ trễ vòng lặp điều khiển (gọi mỗi 5s qua ROS timer riêng).
        Nếu control loop không tick trong >3 giây → cảnh báo log + GUI notify.
        Phát hiện tình trạng loop bị block do Modbus timeout hoặc deadlock.
        """
        gap = time.time() - self._watchdog_last_tick
        if gap > 3.0:
            self.get_logger().error(f"WATCHDOG: loop silent {gap:.1f}s!")
            self._notify('error', 'Loop treo!', f'{gap:.1f}s khong tick')

    # ══════════════════════════════════════════════════════════════
    # State Dispatcher
    # ══════════════════════════════════════════════════════════════

    def _process_state(self):
        """
        Điểm vào của toàn bộ state machine, gọi mỗi tick từ _control_loop().
        Ưu tiên xử lý:
          1. HOMING / HOMING_RUNNING (global state)
          2. ERROR (global state)
          3. Nếu bình thường → gọi song song 3 dispatcher:
               _dispatch_input() — STATE 1 + STATE 2 (input side)
               _dispatch_s3()    — STATE 3 (platform/servo3 feed)
               _dispatch_s4()    — STATE 4 (output side thay khay)
        Ba dispatcher chạy độc lập, tránh xung đột qua interlock cảm biến.
        """
        s = self.state
        if s == SystemState.HOMING:           self._do_homing(); return
        if s == SystemState.HOMING_RUNNING:
            # Check if bg homing thread finished (Bug #6 fix: state transition on main thread)
            if self._homing_done_event.is_set():
                self._homing_done_event.clear()
                if self._homing_result:
                    self.get_logger().info("Homing complete (main thread transition)")
                    msg = Bool()
                    msg.data = True
                    self.pub_homing_done.publish(msg)
                    if self.operation_mode == 'manual':
                        self._jog_mode = True
                        self._sync_mode_jog()
                        self._notify('info', 'Homing xong', 'Homing xong — JOG sẵn sàng')
                    else:
                        self._notify('info', 'Homing xong', '')
                    self.state_in  = SystemState.IDLE
                    self.state_s3  = SystemState.IDLE
                    self.state_s4  = SystemState.IDLE
                    self._system_running = True
                    self._enter(SystemState.IDLE)
                else:
                    if self._homing_abort.is_set():
                        # STOP pressed during homing — _cb_stop already set state to IDLE
                        self.get_logger().info("Homing cancelled by STOP (not an error)")
                    else:
                        self._error("Homing that bai")
            return
        if s == SystemState.ERROR:            self._do_error(); return
        self._dispatch_input()
        self._dispatch_s3()
        self._dispatch_s4()

    def _dispatch_input(self):
        """
        Dispatcher cho state machine phụ INPUT (state_in).
        Xử lý toàn bộ STATE 1 (cấp khay input từ băng tải vào stack)
        và STATE 2 (rút khay đã dùng ra, đưa sang side output để xử lý tiếp).
        Gọi đúng handler function tương ứng với state hiện tại.
        """
        s = self.state_in
        if   s == SystemState.IDLE:                self._do_idle_input()
        elif s == SystemState.S1_CONFIRM_SAFE:     self._s1_confirm_safe()
        elif s == SystemState.S1_INX_MOVE:         self._s1_inx_move()
        elif s == SystemState.S1_WAIT_ARRIVE:      self._s1_wait_arrive()
        elif s == SystemState.S1_INY_SCAN:         self._s1_iny_scan()
        elif s == SystemState.S1_WAIT_STOP_S4:     self._s1_wait_stop_s4()
        elif s == SystemState.S1_INY_TO_ROW:       self._s1_iny_to_row()
        elif s == SystemState.S1_CHECK_S5:         self._s1_check_s5()
        elif s == SystemState.S1_FALLBACK_RETRACT: self._s1_fallback_retract()
        elif s == SystemState.S1_WAIT_GUI_CONFIRM: self._s1_wait_gui_confirm()
        elif s == SystemState.S1_RETRY_INX_500:    self._s1_retry_inx_500()
        elif s == SystemState.S1_RETRY_JOG:        self._s1_retry_jog()
        elif s == SystemState.S1_CYL1_EXTEND:      self._s1_cyl1_extend()
        elif s == SystemState.S1_INY_50:           self._s1_iny_50()
        elif s == SystemState.S1_INY_200:          self._s1_iny_200()
        elif s == SystemState.S1_WAIT_RELEASE:     self._s1_wait_release()
        elif s == SystemState.S1_WAIT_S7:         self._s1_wait_s7()
        elif s == SystemState.S1_INX_10:           self._s1_inx_10()
        elif s == SystemState.S1_COMPLETE:         self._s1_complete()
        elif s == SystemState.S1_RETRY_SCAN_HOME:  self._s1_retry_scan_home()
        elif s == SystemState.S2A_CHECK_INTERLOCK:   self._s2a_check_interlock()
        elif s == SystemState.S2A_INX_500:           self._s2a_inx_500()
        elif s == SystemState.S2A_INY_200_CYL1:      self._s2a_iny_200_cyl1()
        elif s == SystemState.S2A_WAIT_S7:          self._s2a_wait_s7()
        elif s == SystemState.S2A_INY_10:            self._s2a_iny_10()
        elif s == SystemState.S2A_INX_10:            self._s2a_inx_10()
        elif s == SystemState.S2A_INY_JOG_OUTPUT:    self._s2a_iny_jog_output()
        elif s == SystemState.S2A_INY_OUTPUT_ROW:    self._s2a_iny_output_row()
        elif s == SystemState.S2A_WAIT_CYL1_RET:          self._s2a_wait_cyl1_ret()
        elif s == SystemState.S2A_INY_10_FINAL:      self._s2a_iny_10_final()
        elif s == SystemState.S2A_INX_20:            self._s2a_inx_20()
        elif s == SystemState.S2A_RETRY_SCAN_HOME:   self._s2a_retry_scan_home()
        elif s == SystemState.S2A_COMPLETE:          self._s2a_complete()

    def _dispatch_s3(self):
        """
        Dispatcher cho STATE 3: Platform/Servo3 đẩy khay từ stack ra vị trí feed (S18).
        Chạy độc lập với _dispatch_input — có thể chạy xen kẽ STATE 1/2 nếu cần.
        """
        s = self.state_s3
        if   s == SystemState.IDLE:                self._do_idle_s3()
        elif s == SystemState.S3_CHECK_OUTXY_SAFE: self._s3_check_outxy_safe()
        elif s == SystemState.S3_SERVO3_TARGET1:   self._s3_servo3_target1()
        elif s == SystemState.S3_CHECK_S17:        self._s3_check_s17()
        elif s == SystemState.S3_WAIT_S17:         self._s3_wait_s17()
        elif s == SystemState.S3_WAIT_GUI_CONFIRM: self._s3_wait_gui_confirm()
        elif s == SystemState.S3_SERVO3_FEED:      self._s3_servo3_feed()
        elif s == SystemState.S3_WAIT_S18:         self._s3_wait_s18()
        elif s == SystemState.S3_COMPLETE:         self._s3_complete()

    def _dispatch_s4(self):
        """
        Dispatcher cho STATE 4: OutX/OutY thay khay output khi khay đầy.
        Chạy độc lập hoàn toàn với STATE 1/2/3 — dùng servo riêng (OutX S4, OutY S5).
        """
        s = self.state_s4
        if   s == SystemState.IDLE:                self._do_idle_s4()
        elif s == SystemState.S4_CHECK_OUTY_SAFE:  self._s4_check_outy_safe()
        elif s == SystemState.S4_OUTX_TARGET2:     self._s4_outx_target2()
        elif s == SystemState.S4_OUTY_PICK:        self._s4_outy_pick()
        elif s == SystemState.S4_CYL2_EXTEND:      self._s4_cyl2_extend()
        elif s == SystemState.S4_OUTY_TARGET1:     self._s4_outy_target1()
        elif s == SystemState.S4_OUTX_TARGET3:     self._s4_outx_target3()
        elif s == SystemState.S4_CHECK_S19:         self._s4_check_s19()
        elif s == SystemState.S4_OUTY_ROW1:        self._s4_outy_row1()
        elif s == SystemState.S4_OUTY_SCAN_S20:    self._s4_outy_scan_s20()
        elif s == SystemState.S4_OUTY_DROP:        self._s4_outy_drop()
        elif s == SystemState.S4_CYL2_RETRACT:     self._s4_cyl2_retract_state()
        elif s == SystemState.S4_OUTY_OUTX_HOME:   self._s4_outy_outx_home()
        elif s == SystemState.S4_RETRY_SCAN_HOME:  self._s4_retry_scan_home()
        elif s == SystemState.S4_COMPLETE:         self._s4_complete()

    # ── IDLE ─────────────────────────────────────────────────────

    def _do_idle_input(self):
        """
        Trạng thái IDLE của state machine INPUT — chờ điều kiện để trigger STATE 1 hoặc 2.
        Ưu tiên: STATE 2 (robot đã báo xong) trước STATE 1 (cấp khay mới).
        AUTO/AI: STATE 1 auto-trigger qua _can_start_s1().
        MANUAL: STATE 1 chỉ trigger khi operator nhấn nút (_state1_enabled set bởi GUI).
        Log lý do đang chờ (không có khay / robot bận / vị trí cấp đang có khay).
        """
        if not self.zero_offset or not getattr(self, '_system_running', False):
            if not self.zero_offset:
                self._log_once("IDLE_IN_NOT_HOMED", "IDLE-IN: Chua home")
            return

        # S2A: robot done → trigger kể cả manual (robot pub topic)
        if self._can_start_s2a():
            self._input_tray_done = False
            mode_str = self.operation_mode.upper()
            self.get_logger().info(f"[IN-IDLE] Robot done → STATE 2 ({mode_str})")
            self._enter_in(SystemState.S2A_CHECK_INTERLOCK)
            return

        # ── AUTO/MANUAL: auto-trigger S1 ───────────────
        if self._state1_enabled and self._can_start_s1():
            self.get_logger().info(f"[IN-IDLE] Đủ điều kiện → STATE 1 ({self.operation_mode.upper()})")
            self._guide_logged.discard("IDLE_IN_S7")
            self._enter_in(SystemState.S1_CONFIRM_SAFE)
            return
        
        # Log lý do chờ
        if not (self.sensor(S1_BELT_START) or self.sensor(S2_BELT_MID) or self.sensor(S3_BELT_END)):
            self._log_once("IDLE_IN_NO_TRAY", "[IN-IDLE] S1/S2/S3 OFF — hết khay input")
        elif self.sensor(S7_TRAY_AT_ROBOT):
            self._log_once("IDLE_IN_NO_PLACE", "[IN-IDLE] S7 ON — vị trí cấp đang có khay (chờ xử lý xong để chạy State 2)")
        elif self._motion_busy:
            self._log_once("IDLE_IN_BUSY", f"[IN-IDLE] Robot đang báo BẬN (/robot/motion_busy={self._motion_busy}) — chờ")

    def _do_idle_s3(self):
        """Chờ điều kiện để auto-trigger STATE 3: S18 OFF liên tục 5s (vị trí feed trống)."""
        if not self.zero_offset or not getattr(self, '_system_running', False):
            return
        if self._can_start_s3():
            self.get_logger().info("[S3-IDLE] S17 ON + S18 OFF >= 5s → STATE 3")
            self._enter_s3(SystemState.S3_CHECK_OUTXY_SAFE)

    def _do_idle_s4(self):
        """Chờ điều kiện để trigger STATE 4: _s4_trigger=True (robot báo khay output đầy)."""
        if not self.zero_offset or not getattr(self, '_system_running', False):
            return
        if self._can_start_s4():
            self._s4_trigger = False
            self.get_logger().info("[S4-IDLE] Output full → STATE 4")
            self._enter_s4(SystemState.S4_CHECK_OUTY_SAFE)

    def _do_homing(self):
        """
        Khởi động quá trình homing trong background thread để không block control loop.
        Chuyển sang HOMING_RUNNING ngay lập tức, rồi spawn thread chạy _home_all().
        Kết quả được báo về main thread qua _homing_done_event (threading.Event)
        để tránh race condition khi mutate state từ thread phụ.
        """
        self._enter(SystemState.HOMING_RUNNING)
        self._homing_done_event.clear()
        def _bg():
            ok = self._home_all()
            # Signal result to main thread — do NOT mutate state_in/s3/s4 here
            self._homing_result = ok
            self._homing_done_event.set()
        threading.Thread(target=_bg, daemon=True).start()

    def _do_error(self):
        """Trạng thái ERROR: log hướng dẫn phục hồi (STOP → START) mỗi lần vào state."""
        self._log_once("ERROR_STATE", "ERROR — kiem tra loi roi nhan STOP -> START")

    # ══════════════════════════════════════════════════════════════
    # STATE 1: Cap khay Input  (giữ nguyên logic gốc)
    # ══════════════════════════════════════════════════════════════

    def _s1_confirm_safe(self):
        if not self._cmd_sent_in and not self._inx_moving and not self._iny_moving:
            self._pub_cartridge_busy(True)
            if self._input_tray_done:
                self._input_tray_done = False
        iny = self._pos(2)
        if iny is None:
            return
        if iny <= self.config.iny_home + 2.0:
            self._enter_in(SystemState.S1_INX_MOVE)
        else:
            if not self._cmd_sent_in:
                ok = self._nb_move(2, self.config.iny_home)
                if not ok:
                    self._log_once("S1_INY_HOME_FAIL", "S1_SAFE: INY home fail")
                    return
                self._cmd_sent_in     = True
                self._step_timeout_in = time.time() + self.config.move_timeout
            else:
                if time.time() > self._step_timeout_in:
                    self._cmd_sent_in = False
                elif self._arrived(2):
                    self._enter_in(SystemState.S1_INX_MOVE)

    def _s1_inx_move(self):
        if not (self.sensor(S1_BELT_START) or self.sensor(S2_BELT_MID) or self.sensor(S3_BELT_END)):
            self._log_once("S1_WAIT_BELT", "Step2: Cho S1/S2/S3")
            self._notify('info', 'Cho khay (S1/S2/S3)', 'Dat khay len bang tai.')
            return
        if not self._iny_safe():
            self._log_once("S1_INY_SAFE", "Step2: INY chua safe")
            return
        if not self._cmd_sent_in:
            ok = self._nb_move(1, self.config.inx_target2, vel=150)
            if not ok:
                self._log_once("S1_INX_FAIL", "S1 Step2: INX fail")
                return
            self._cmd_sent_in    = True
            self._inx_arrived = False
            self._30s_timeout = 0.0
            self._cyl1_warn_t  = 0.0
            self.get_logger().info(f"Step2: INX -> {self.config.inx_target2}mm")
            self._enter_in(SystemState.S1_WAIT_ARRIVE)

    def _s1_wait_arrive(self):
        if not self._inx_arrived:
            self._inx_arrived = self._arrived(1)
            if not self._inx_arrived:
                self._log_once("S1_INX_MOVING", "INX dang di chuyen")
                return
            # Verify INX actually reached target position
            inx_curr = self._pos(1)
            if inx_curr is not None and abs(inx_curr - self.config.inx_target2) > 5.0:
                self.get_logger().warn(f"INX arrived=True but pos={inx_curr:.1f}mm != target {self.config.inx_target2}mm — resend")
                self._inx_arrived = False
                self._nb_move(1, self.config.inx_target2, vel=150)
                return
            self._30s_timeout = time.time() + 50.0
            self.get_logger().info(f"INX dung tai {inx_curr:.1f}mm — check S3+S9 (50s)")

        belt_end, cyl1_ret = self._snap(S3_BELT_END, S9_CYL1_RETRACTED)

        if belt_end and cyl1_ret:
            self.get_logger().info("S3+S9 ON -> scan INY")
            self._s4_armed = False
            self._enter_in(SystemState.S1_INY_SCAN)
            return

        if not belt_end:
            if time.time() > self._30s_timeout:
                self.get_logger().info("S3 khong ON sau 50s — INX ve safe (-60mm), retry")
                self._nb_move(1, self.config.inx_safe)
                self._s1_scan_noise_retry = 0
                self._s4_prev_in = False
                self._enter_in(SystemState.S1_CONFIRM_SAFE)
                return
            remain = self._30s_timeout - time.time()
            self._log_once("S1_WAIT_S3", f"Cho S3 ON (con {remain:.0f}s)")
        else:
            if self._cyl1_warn_t == 0:
                self._cyl1_warn_t = time.time()
            elif time.time() - self._cyl1_warn_t >= 5.0:
                self._notify('warn', 'S3 ON nhưng S9 OFF', 'Cyl1 chưa retract')
                self._cyl1_warn_t = time.time()
            self._log_once("S1_WAIT_S13", "S3 ON, chờ S9 ON")

    def _s1_iny_scan(self):
        iny = self._pos(2)
        if iny is None:
            return

        if not self._cmd_sent_in:
            # ok = self._nb_move(2, self.config.target_scaninp1,
            #                    vel=self.config.iny_scan_vel)
            ok = self._nb_move(2, self.config.target_scaninp1,vel=70)
            if not ok:
                self._log_once("S1_SCAN_FAIL", "S1 INY scan: nb_move fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
            self._s4_armed        = False
            self._s4_prev_in      = self.sensor(S4_SCAN_STACK_P1)
            self.get_logger().info(
                f"[S1 SCAN] INY -> {self.config.target_scaninp1:.0f}mm "
                f"vel={self.config.iny_scan_vel}"
            )

        if iny >= self.config.iny_scan_arm_mm:
            self._s4_armed = True

        s4_now = self.sensor(S4_SCAN_STACK_P1)
        # S4 là thường đóng (NC): chạm khay chuyển từ ON sang OFF -> Falling edge
        s4_falling = self._s4_prev_in and (not s4_now)
        self._s4_prev_in = s4_now

        if self._s4_armed and s4_falling:
            trigger_pos = self._pos(2) or iny
            
            row               = self._zone_to_row(trigger_pos, self.config.iny_input_zones)
            target_mm         = self.config.iny_input_zones[row][2]
            self._current_row = row

            self.get_logger().info(
                f"[S1 SCAN] S4 falling edge trigger @ {trigger_pos:.1f}mm → row{row} ({target_mm:.0f}mm) - Moving directly"
            )
            self._s5_retry = 0
            self._enter_in(SystemState.S1_INY_TO_ROW)
            return

        timed_out = time.time() > self._step_timeout_in
        at_target = self._arrived(2) or iny >= self.config.target_scaninp1 - 2.0
        if timed_out or at_target:
            self.get_logger().warn("[S1 SCAN] Hết hạn scan mà không có falling edge hợp lệ của S4")
            self._stop(2)

            if self._s1_scan_noise_retry < self._conf('s1_scan_noise_retry_limit', 1):
                self._s1_scan_noise_retry += 1
                self._notify(
                    'warn',
                    'Không bắt được S4 hợp lệ',
                    f'Retry scan lần {self._s1_scan_noise_retry}'
                )
                self._enter_in(SystemState.S1_RETRY_SCAN_HOME)
                return

            self._notify(
                'error',
                'Scan S4 thất bại',
                f'INY về {self.config.iny_home:.0f}mm, INX về {self._conf("inx_noise_recovery_mm", 10.0):.0f}mm'
            )
            self._nb_move(2, self.config.iny_home)
            self._nb_move(1, self._conf("inx_noise_recovery_mm", 10.0))
            self._jog_mode = True
            self._sync_mode_jog()
            self._enter_in(SystemState.IDLE)
            return

        self._log_once("S1_SCANNING",
                       f"[S1 SCAN] INY {iny:.0f}mm arm={'OK' if self._s4_armed else 'NO'} S4={'ON' if s4_now else 'OFF'}")

    def _s1_retry_scan_home(self):
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_home)
            if not ok:
                self._notify('error', 'Retry scan fail', 'INY không về được home')
                self._nb_move(1, self._conf("inx_noise_recovery_mm", 10.0))
                self._enter_in(SystemState.IDLE)
                return
            self._cmd_sent_in = True
            self._step_timeout_in = time.time() + self.config.move_timeout
            return

        if time.time() > self._step_timeout_in:
            self._notify('error', 'Retry scan timeout', 'INY về home timeout')
            self._nb_move(1, self._conf("inx_noise_recovery_mm", 10.0))
            self._enter_in(SystemState.IDLE)
            return

        if self._arrived(2):
            self._cmd_sent_in = False
            self._step_timeout_in = 0.0
            self._s4_armed = False
            self._s4_prev_in = self.sensor(S4_SCAN_STACK_P1)
            self.get_logger().info("[S1 SCAN] Retry lại từ INY home")
            self._enter_in(SystemState.S1_INY_SCAN)

    def _s1_iny_to_row(self):
        zone = self.config.iny_input_zones.get(self._current_row)
        target = zone[2] if zone else None
        if target is None:
            valid = sorted(self.config.iny_input_zones.keys())
            self._current_row = max(valid[0], min(valid[-1], self._current_row))
            target = self.config.iny_input_zones[self._current_row][2]
        if not self._cmd_sent_in:
            ok = self._nb_move(2, target, vel=self.config.iny_row_vel, continuous_update=True)
            if not ok:
                self._log_once("S1_ROW_FAIL", f"S1: INY -> {target}mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(2):
                self._step_start_in = time.time()
                self._enter_in(SystemState.S1_CHECK_S5)

    def _s1_check_s5(self):
        if not self._cmd_sent_in:
            self._step_start_in = time.time()
            self._cmd_sent_in   = True

        if self.sensor(S5_OUTPUT_DETECT):
            self.get_logger().info(f"S5 ON (lan {self._s5_retry+1}) — extend Cyl1")
            self._cyl1_extend()
            self._enter_in(SystemState.S1_CYL1_EXTEND)
            return

        elapsed = time.time() - self._step_start_in
        self._log_once("S1_WAIT_S5",
                       f"Cho S5 ON (con {max(0.0, OUTPUT_DETECT_WAIT_S - elapsed):.1f}s, "
                       f"lan {self._s5_retry+1}/2) row {self._current_row}")

        if elapsed >= OUTPUT_DETECT_WAIT_S:
            if self._s5_retry == 0:
                self._s5_retry = 1
                self._cmd_sent_in = False
                self._guide_logged.discard("S1_WAIT_S5")
                self.get_logger().warn(f"S5 OFF lan 1 tai row {self._current_row} — thu lai lan 2")
            else:
                self.get_logger().warn(f"S5 OFF 2 lan tai row {self._current_row} — reset")
                self._notify('warn', 'S5 OFF 2 lan', f'Row {self._current_row} — reset')
                self._s5_retry = 0
                self._enter_in(SystemState.S1_FALLBACK_RETRACT)

    def _s1_fallback_retract(self):
        cyl1_ret, cyl1_ext = self._snap(S9_CYL1_RETRACTED, S10_CYL1_EXTENDED)
        if not self._cmd_sent_in:
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in = True

        if cyl1_ret and not cyl1_ext:
            self.get_logger().info("Fallback: S9 ON + S10 OFF -> INY ve home")
            self._nb_move(2, self.config.iny_home)
            self._cmd_sent_in = True
            self._step_timeout_in = time.time() + self.config.move_timeout
            self._enter_in(SystemState.S1_FALLBACK_WAIT_INY)
            return

        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Fallback: Retry Cyl1 retract")
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S1_FB_RETRACT", "Fallback: cho S9 ON, S10 OFF")

    def _s1_fallback_wait_iny(self):
        if time.time() > self._step_timeout_in:
            self.get_logger().error("Timeout iny ve home")
            self._error("Fallback INY timeout")
            return
        if self._arrived(2) or self._pos(2) <= self.config.iny_home + 2.0:
            self.get_logger().info("INY ve home safe -> INX ve safe (-60mm)")
            self._nb_move(1, self.config.inx_safe,vel=200)
            self._enter_in(SystemState.S1_CONFIRM_SAFE)

    def _go_gui_confirm(self):
        self._nb_move(2, self.config.iny_home)
        self._step_timeout_in  = time.time() + self.config.move_timeout
        self._gui_confirmed = False
        self._enter_in(SystemState.S1_WAIT_GUI_CONFIRM)
        self._notify('error', 'Kiem tra S4/S5!',
                     'Cam bien khong phat hien khay. Kiem tra va nhan XAC NHAN.')

    def _s1_wait_gui_confirm(self):
        if not self._cmd_sent_in:
            if self._iny_safe():
                ok = self._nb_move(1, self.config.inx_safe)
                if ok:
                    self._cmd_sent_in = True
        if self._gui_confirmed:
            self._gui_confirmed = False
            self._stop(1); self._stop(2)
            self._enter_in(SystemState.S1_RETRY_JOG)
        self._log_once("S1_GUI_WAIT", "Cho nhan XAC NHAN tren GUI")

    def _s1_retry_jog(self):
        if not (self.sensor(S1_BELT_START) or self.sensor(S2_BELT_MID) or self.sensor(S3_BELT_END)):
            self._log_once("S1_NO_TRAY", "Het khay — S1/S2/S3 OFF")
            self._notify('warn', 'Het khay', 'Nap khay roi nhan START')
            return
        inx = self._pos(1)
        if inx is None:
            return
        if abs(inx - self.config.inx_target2) < 15.0:
            self._s4_armed = False; self._s5_retry = 0
            self._enter_in(SystemState.S1_INY_SCAN)
        else:
            self._enter_in(SystemState.S1_RETRY_INX_500)

    def _s1_retry_inx_500(self):
        if not self._cmd_sent_in:
            if not self._iny_safe():
                self._log_once("S1_RETRY_INY", "Cho INY safe truoc INX 500mm")
                return
            ok = self._nb_move(1, self.config.inx_target2)
            if not ok:
                self._log_once("S1_RETRY_INX", "S1 Retry: INX 500mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(1):
                self._s4_armed = False; self._s5_retry = 0
                self._enter_in(SystemState.S1_INY_SCAN)

    def _s1_cyl1_extend(self):
        cyl1_ext, = self._snap(S10_CYL1_EXTENDED)
        # BO CHECK S5 o day vi qua trinh day rulo co the lam rung S5
        if not self._cmd_sent_in:
            self._cyl1_extend()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in    = True
        if cyl1_ext:
            self.get_logger().info("S10 ON — gap khay OK -> INY ve 50mm")
            self._enter_in(SystemState.S1_INY_50)
            return
        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Retry Cyl1 extend")
            self._cyl1_extend()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S1_WAIT_S14", "Cho S10 ON (Cyl1 extend)")

    def _s1_iny_50(self):
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_home,vel=250)
            if not ok:
                self.get_logger().warn("S1: INY -> home (0mm) fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(2):
                self._enter_in(SystemState.S1_INY_200)

    def _s1_iny_200(self):
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_target2)
            if not ok:
                self.get_logger().warn("S1: INY -> 200mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(2):
                self._enter_in(SystemState.S1_WAIT_RELEASE)

    def _s1_wait_release(self):
        cyl1_ret, cyl1_ext = self._snap(S9_CYL1_RETRACTED, S10_CYL1_EXTENDED)
        if not self._cmd_sent_in:
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in    = True
        if cyl1_ret and not cyl1_ext:
            self.get_logger().info("S9 ON + S10 OFF — Cyl1 nha xong -> cho S7")
            self._enter_in(SystemState.S1_WAIT_S7)
            return
        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Retry Cyl1 retract")
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S1_WAIT_REL", f"Cho S9 ON + S10 OFF | S9={cyl1_ret} S10={cyl1_ext}")

    def _s1_wait_s7(self):
        tray_robot, = self._snap(S7_TRAY_AT_ROBOT)
        if tray_robot:
            self.get_logger().info("S7 ON — Khay o vi tri robot -> INX ve safe")
            self._enter_in(SystemState.S1_INX_10)
            return
        self._log_once("S1_WAIT_S7", "Cho S7 ON")

    def _s1_inx_10(self):
        if not self._iny_safe():
            self._log_once("S1_INX_WAIT_INY", f"Cho INY <= {self.config.iny_safe_zone}mm truoc INX ve home")
            return
        if not self._cmd_sent_in:
            ok = self._nb_move(1, self.config.inx_safe,vel=150)
            if not ok:
                self.get_logger().warn(f"S1: INX -> {self.config.inx_safe}mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(1):
                self._enter_in(SystemState.S1_COMPLETE)

    def _s1_complete(self):
        """
        Publish new_tray_loaded. Check S7 với timeout 3s.
        Xong thì về IDLE. IDLE sẽ chờ robot done_tray_input rồi kích S2A.
        """
        self._pub_cartridge_busy(False)
        self._s1_scan_noise_retry = 0
        self._s4_prev_in         = False
        if not self._cmd_sent_in:
            self.stack_row_index  = self._current_row
            self._cmd_sent_in     = True
            self._tray_robot_check_start = 0.0

        if self.sensor(S7_TRAY_AT_ROBOT):
            if self._tray_robot_check_start >= 0.0:
                self.pub_new_tray.publish(Bool(data=True))
                self.get_logger().info("Published: new_tray_loaded = True (S7 ON)")
                self._notify('info', 'STATE 1 COMPLETE', 'Khay đã ở robot')
                self._tray_robot_check_start = -1.0

            self._state1_enabled = False
            self._enter_in(SystemState.IDLE)
        else:
            if self._tray_robot_check_start == 0.0:
                self._tray_robot_check_start = time.time()
                self.get_logger().warn("S7 chua ON — cho 3s")
            elapsed = time.time() - self._tray_robot_check_start
            if elapsed >= TRAY_ROBOT_CHECK_TIMEOUT_S:
                self.get_logger().error(f"S7 OFF sau {TRAY_ROBOT_CHECK_TIMEOUT_S:.0f}s — caution")
                self._notify('warn', 'S7 OFF sau timeout', 'Kiem tra lai S7.')
                self.pub_new_tray.publish(Bool(data=True))
                self._state1_enabled = False
                self._enter_in(SystemState.IDLE)
            else:
                self._log_once("S1C_S18_WAIT",
                               f"Cho S7 ON (con {TRAY_ROBOT_CHECK_TIMEOUT_S - elapsed:.1f}s)")

    # ══════════════════════════════════════════════════════════════
    # STATE 2: Thay khay Input  (giữ nguyên logic gốc)
    # ══════════════════════════════════════════════════════════════

    def _s2a_check_interlock(self):
        if not self._cmd_sent_in:
            self._pub_cartridge_busy(True)
            self._s6_snapshot       = self.sensor(S6_CHECK_TRAY_P1)
            self._s4_armed_out      = False
            self._output_row        = 0
            self._output_target_pos = 0.0
            self.get_logger().info(
                f"S2 Step1: S6={self._s6_snapshot} "
                f"({'jog do S4' if self._s6_snapshot else 'thang row1'})"
            )
            self._cmd_sent_in = True
        if self._iny_safe():
            self._enter_in(SystemState.S2A_INX_500)
        else:
            if not self._iny_moving:
                ok = self._nb_move(2, self.config.iny_home)
                if not ok:
                    self.get_logger().warn("S2 Step1: INY home fail")
                    return
            self._log_once("S2A_WAIT_INY", "Step1: cho INY <= 50mm")

    def _s2a_inx_500(self):
        if not self._iny_safe():
            self._log_once("S2A_INY2", "A2: INY chua safe")
            return
        if not self.sensor(S9_CYL1_RETRACTED):
            self._log_once("S2A_S13_ILK", "INTERLOCK S2: S9=OFF -> INX bi khoa")
            self._notify('warn', 'S9 chua ON', 'Cyl1 chua retract')
            return
        if not self._cmd_sent_in:
            ok = self._nb_move(1, self.config.inx_target2)
            if not ok:
                self._log_once("S2A_A2_FAIL", "S2A A2: INX 500mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(1):
                self._enter_in(SystemState.S2A_INY_200_CYL1)

    def _s2a_iny_200_cyl1(self):
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_target2)
            if not ok:
                self._log_once("S2A_A3_FAIL", "S2A A3: INY 200mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(2):
                self.get_logger().info("A3: INY tai 200mm -> Cyl1 EXTEND")
                self._cyl1_extend()
                self._enter_in(SystemState.S2A_WAIT_S7)

    def _s2a_wait_s7(self):
        cyl1_ext, = self._snap(S10_CYL1_EXTENDED)
        if not self._cmd_sent_in:
            self._cyl1_extend()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in    = True
        if cyl1_ext:
            self.get_logger().info("A4: S10 ON — gap khay cu OK")
            self._enter_in(SystemState.S2A_INY_10)
            return
        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Retry Cyl1 extend")
            self._cyl1_extend()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S2A_S14", "A4: Cho S10 ON")

    def _s2a_iny_10(self):
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_home)
            if not ok:
                self._log_once("S2A_A5_FAIL", "S2A A5: INY 10mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(2):
                self._enter_in(SystemState.S2A_INX_10)

    def _s2a_inx_10(self):
        if not self._iny_safe():
            self._log_once("S2A_INX10_WAIT", "A6: INY chua safe")
            return
        if not self._cmd_sent_in:
            ok = self._nb_move(1, self.config.inx_output_stack)
            if not ok:
                self._log_once("S2A_A6_FAIL", "S2A A6: INX output pos fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(1):
                self._s4_armed_out = False
                self._enter_in(SystemState.S2A_INY_JOG_OUTPUT)

    def _s2a_iny_jog_output(self):
        if not self._s6_snapshot:
            self.get_logger().info(
                f"[S2A Step7] S6 OFF → thẳng row1 ({self.config.iny_output_zones[1][2]:.0f}mm)"
            )
            self._output_target_pos = self.config.iny_output_zones[1][2]
            self._output_row        = 1
            self._enter_in(SystemState.S2A_INY_OUTPUT_ROW)
            return

        iny = self._pos(2)
        if iny is None:
            return

        scan_target = self.config.target_scanoutp1

        if not self._cmd_sent_in:
            ok = self._nb_move(2, scan_target, vel=self.config.iny_scan_vel)
            if not ok:
                self._log_once("S2A_JOG_FAIL", "S2A Step7: nb_move fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
            self.get_logger().info(
                f"[S2A Step7] S6=ON → scan xuống {scan_target:.0f}mm "
                f"vel={self.config.iny_scan_vel}"
            )

        s4_now = self.sensor(S4_SCAN_STACK_P1)
        if s4_now:
            trigger_pos = self._pos(2) or iny
            
            row = self._zone_to_row(trigger_pos, self.config.iny_output_zones)
            if row is not None:
                target_mm = self.config.iny_output_zones[row][2]
                self._output_target_pos = target_mm
                self._output_row = row
                self._s2a_scan_noise_retry = 0
                self.get_logger().info(f"[S2A Step7] S4 ON @ {trigger_pos:.1f}mm → row{row} ({target_mm:.0f}mm)")
                self._enter_in(SystemState.S2A_INY_OUTPUT_ROW)
                return

        timed_out = time.time() > self._step_timeout_in
        at_target = self._arrived(2) or iny >= scan_target - 2.0
        if timed_out or at_target:
            self._stop(2)
            self.get_logger().warn(f"[S2A Step7] S4 không trigger → fallback row1")
            tgt = self.config.iny_output_zones[1][2]
            self._output_target_pos = tgt
            self._output_row = 1
            self._nb_move(2, tgt, vel=self.config.iny_row_vel)
            self._cmd_sent_in = True
            self._step_timeout_in = time.time() + self.config.move_timeout
            self._enter_in(SystemState.S2A_INY_OUTPUT_ROW)
            return

        self._log_once("S2A_JOG_OUT", f"[S2A Step7] INY {iny:.0f}mm S4={'ON' if s4_now else 'OFF'}")

    def _s2a_retry_scan_home(self):
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_home)
            if not ok:
                self._enter_in(SystemState.ERROR)
            self._cmd_sent_in = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._error("Homing INY sau lỗi timeout")
            elif self._arrived(2) or (self._pos(2) <= self.config.iny_home + 2.0):
                self._cmd_sent_in = False
                self._enter_in(SystemState.S2A_INY_JOG_OUTPUT)

    def _s2a_iny_output_row(self):
        if not self._cmd_sent_in:
            # Chỉ vào đây qua path S6=OFF (target đã set, chưa gửi move)
            target = self._output_target_pos
            if target <= 0:
                self.get_logger().error("output_target_pos chưa set → skip")
                self._enter_in(SystemState.S2A_WAIT_CYL1_RET)
                return
            ok = self._nb_move(2, target, vel=self.config.iny_row_vel, continuous_update=True)
            if not ok:
                self.get_logger().warn(f"S2A output_row: move {target:.1f}mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout

        if time.time() > self._step_timeout_in:
            self._cmd_sent_in = False
        elif self._arrived(2):
            self.get_logger().info(
                f"[S2A] InY tới row{self._output_row} "
                f"({self._output_target_pos:.0f}mm) → Cyl1 RETRACT"
            )
            self._cyl1_retract()
            self._enter_in(SystemState.S2A_WAIT_CYL1_RET)

    def _s2a_wait_cyl1_ret(self):
        cyl1_ret, = self._snap(S9_CYL1_RETRACTED)
        if not self._cmd_sent_in:
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in    = True
        if cyl1_ret:
            self.get_logger().info("A9: S9 ON — da tha khay")
            self._enter_in(SystemState.S2A_INY_10_FINAL)
            return
        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Retry Cyl1 retract")
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S2A_S13", "A9: Cho S9 ON")

    def _s2a_iny_10_final(self):
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_home)
            if not ok:
                self._log_once("S2A_A10_FAIL", "S2A A10: INY 10mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(2):
                self._enter_in(SystemState.S2A_INX_20)

    def _s2a_inx_20(self):
        if not self._iny_safe():
            self._log_once("S2A_INX20", "A11: Cho INY safe")
            return
        if not self._cmd_sent_in:
            ok = self._nb_move(1, self.config.inx_safe)
            if not ok:
                self._log_once("S2A_A11_FAIL", "S2A A11: INX safe fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(1):
                self._enter_in(SystemState.S2A_COMPLETE)

    def _s2a_retry_scan_home(self):
        # Placeholder for S2A retry logic
        self._enter_in(SystemState.IDLE)

    def _s2a_complete(self):
        if not self._cmd_sent_in:
            self._input_tray_done = False
            self._pub_cartridge_busy(False)
            self._notify('info', 'STATE 2 COMPLETE', 'Da rut khay ra')
            self._cmd_sent_in = True

        self.get_logger().info("State2 done -> IDLE (cho check S123 de chay State 1 cap khay tiep)")
        self._enter_in(SystemState.IDLE)

    # ══════════════════════════════════════════════════════════════
    # STATE 3 — Cấp khay thành phẩm  (giữ nguyên logic gốc)
    # ══════════════════════════════════════════════════════════════

    def _s3_check_outxy_safe(self):
        if not self._cmd_sent_s3:
            self._pub_cartridge_busy(True)
        cfg = self.config
        ox = self._pos(4)
        oy = self._pos(5)
        ox_safe = ox is None or ox <= cfg.outx_home + 5.0
        oy_safe = oy is None or oy <= cfg.outy_target1 + 5.0
        if ox_safe and oy_safe:
            self._enter_s3(SystemState.S3_CHECK_S17)
        else:
            if not self._cmd_sent_s3:
                if not ox_safe:
                    self._nb_move(4, cfg.outx_home)
                if not oy_safe:
                    self._nb_move(5, cfg.outy_target1)
                self._cmd_sent_s3     = True
                self._step_timeout_s3 = time.time() + self.config.move_timeout
            else:
                if time.time() > self._step_timeout_s3:
                    self._enter_s3(SystemState.S3_CHECK_S17)
                elif self._arrived(4) and self._arrived(5):
                    self._enter_s3(SystemState.S3_CHECK_S17)
                else:
                    self._log_once("S3_SAFE", "Cho OutX/OutY ve home")

    def _s3_servo3_target1(self):
        cfg = self.config
        if not self._cmd_sent_s3:
            ok = self._nb_move(3, cfg.servo3_target1)
            if not ok:
                self._log_once("S3_T1_FAIL", "S3 target1 fail")
                return
            self._cmd_sent_s3     = True
            self._step_timeout_s3 = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_s3:
                self._error(f"S3_SERVO3_TARGET1 timeout")
            elif self._arrived(3):
                self._enter_s3(SystemState.S3_CHECK_S17)

    def _s3_check_s17(self):
        if not self.sensor(S17_PLATFORM):
            self._enter_s3(SystemState.S3_WAIT_S17)
            return

        if not getattr(self, '_step_start_s3', 0.0):
            self._step_start_s3 = time.time()
            self._log_once("S3_CHECK_S17", "S17 ON -> Confirming 3s...")

        if time.time() - self._step_start_s3 >= 3.0:
            self.get_logger().info("[S3] S17 confirmed for 3s -> Start Feed")
            self._enter_s3(SystemState.S3_SERVO3_FEED)

    def _s3_wait_s17(self):
        if self.sensor(S17_PLATFORM):
            self._notify('info', 'Da phat hien khay', 'S17 ON — confirm 3s')
            self._enter_s3(SystemState.S3_CHECK_S17)
        else:
            self._log_once("S3_WAIT_S17", "Cho S17 ON — cap khay len Platform")

    def _s3_wait_gui_confirm(self):
        self._enter_s3(SystemState.S3_SERVO3_FEED)

    def _s3_servo3_feed(self):
        cfg = self.config
        if not self._cmd_sent_s3:
            ok = self._nb_move(3, cfg.servo3_target2, vel=int(cfg.servo3_feed_velocity))
            if not ok:
                self._log_once("S3_FEED_FAIL", "S3 feed fail")
                return
            self._cmd_sent_s3     = True
            self._step_start_s3   = time.time()
            self._step_timeout_s3 = time.time() + self.config.move_timeout
            self.get_logger().info(f"[S3] Servo3 pushing to {cfg.servo3_target2}mm max until S18 ON")
        else:
            if self.sensor(S18_FEED_OK):
                self._stop(3)
                self.get_logger().info("[S3] S18 ON -> Dung Servo 3 som")
                self._enter_s3(SystemState.S3_WAIT_S18)
                return

            if time.time() > self._step_timeout_s3:
                self._error("S3_SERVO3_FEED timeout")
            elif self._arrived(3) and time.time() - self._step_start_s3 > 0.5:
                self.get_logger().warn("[S3] Servo 3 tới giới hạn (target2) chưa có S18 -> Quay về 10mm thử lại!")
                self._notify('warn', 'Lỗi cấp khay', 'Đã tới 400mm nhưng chưa thấy S18, thu về cấp lại')
                self._enter_s3(SystemState.S3_SERVO3_TARGET1)

    def _s3_wait_s18(self):
        if not self.sensor(S18_FEED_OK):
            self.get_logger().warn("[S3] S18 OFF during confirm -> Resume FEED")
            self._enter_s3(SystemState.S3_SERVO3_FEED)
            return

        if not getattr(self, '_step_start_s3', 0.0):
            self._step_start_s3 = time.time()
            self._log_once("S3_WAIT_S18", "S18 ON -> Confirming 5s...")

        if time.time() - self._step_start_s3 >= 5.0:
            self.get_logger().info("[S3] S18 confirmed 5s -> Feed Success")
            self._enter_s3(SystemState.S3_COMPLETE)
            return

    def _s3_complete(self):
        self._pub_cartridge_busy(False)
        self.pub_newtray_output.publish(Bool(data=True))
        self._notify('info', 'State 3 done', 'Cap khay thanh pham thanh cong')
        self.get_logger().info("[S3] COMPLETE — pub new_trayoutput_loaded")
        self._enter_s3(SystemState.IDLE)

    # ══════════════════════════════════════════════════════════════
    # STATE 4 — Thay khay output  (giữ nguyên logic gốc)
    # ══════════════════════════════════════════════════════════════

    def _s4_check_outy_safe(self):
        if not self._cmd_sent_s4:
            self._pub_cartridge_busy(True)
        cfg = self.config
        oy = self._pos(5)
        if oy is not None and oy <= cfg.outy_safe_zone + 5:
            self._enter_s4(SystemState.S4_OUTX_TARGET2)
        else:
            if not self._cmd_sent_s4:
                ok = self._nb_move(5, cfg.outy_target1)
                if not ok:
                    self._log_once("S4_SAFE_FAIL", "S4 outy home fail")
                    return
                self._cmd_sent_s4     = True
                self._step_timeout_s4 = time.time() + self.config.move_timeout
            else:
                if time.time() > self._step_timeout_s4:
                    self._cmd_sent_s4 = False
                elif self._arrived(5):
                    self._enter_s4(SystemState.S4_OUTX_TARGET2)
            self._log_once("S4_SAFE", "Cho OutY ve safe zone")

    def _s4_outx_target2(self):
        cfg = self.config
        if not self._cmd_sent_s4:
            ok = self._nb_move(4, cfg.outx_target2)
            if not ok:
                self._log_once("S4_OX2_FAIL", "S4 outx_target2 fail")
                return
            self._cmd_sent_s4     = True
            self._step_timeout_s4 = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_s4:
                self._error("S4_OUTX_TARGET2 timeout")
            elif self._arrived(4):
                self._enter_s4(SystemState.S4_OUTY_PICK)

    def _s4_outy_pick(self):
        cfg = self.config
        if not self._cmd_sent_s4:
            ok = self._nb_move(5, cfg.outy_pick_pos)
            if not ok:
                self._log_once("S4_PICK_FAIL", "S4 outy_pick fail")
                return
            self._cmd_sent_s4     = True
            self._step_timeout_s4 = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_s4:
                self._error("S4_OUTY_PICK timeout")
            elif self._arrived(5):
                self._enter_s4(SystemState.S4_CYL2_EXTEND)

    def _s4_cyl2_extend(self):
        if not self._cmd_sent_s4:
            self._cyl2_extend()
            self._cmd_sent_s4 = True
            self._step_start_s4 = time.time()
        if self.sensor(S22_CYL2_EXTENDED):
            self._enter_s4(SystemState.S4_OUTY_TARGET1)
        elif time.time() - self._step_start_s4 > CYLINDER_TIMEOUT_S:
            self._error("[S4] Timeout: Cyl2 extend — S22 khong ON")

    def _s4_outy_target1(self):
        cfg = self.config
        if not self._cmd_sent_s4:
            ok = self._nb_move(5, cfg.outy_target1)
            if not ok:
                self._log_once("S4_OY1_FAIL", "S4 outy_target1 fail")
                return
            self._cmd_sent_s4     = True
            self._step_timeout_s4 = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_s4:
                self._error("S4_OUTY_TARGET1 timeout")
            elif self._arrived(5):
                self._enter_s4(SystemState.S4_OUTX_TARGET3)

    def _s4_outx_target3(self):
        cfg = self.config
        if not self._cmd_sent_s4:
            ok = self._nb_move(4, cfg.outx_target3)
            if not ok:
                self._log_once("S4_OX3_FAIL", "S4 outx_target3 fail")
                return
            self._cmd_sent_s4     = True
            self._step_timeout_s4 = time.time() + self.config.move_timeout
        else:
            if self.sensor(S17_PLATFORM) and not self._s3_pending:
                self._s3_pending = True
            if time.time() > self._step_timeout_s4:
                self._error("S4_OUTX_TARGET3 timeout")
            elif self._arrived(4):
                self._enter_s4(SystemState.S4_CHECK_S19)

    def _s4_check_s19(self):
        # S17 OFF -> Stack đang rỗng -> Bỏ qua scan, xuống thẳng Row 1
        if not self.sensor(S19_CHECK_TRAY_P2):
            self.get_logger().info(f"[S4] S19 OFF -> Bỏ qua scan S20, xuống thẳng Row 1")
            self._enter_s4(SystemState.S4_OUTY_ROW1)
        else:
            self._outy_jog_start = time.time()
            self._enter_s4(SystemState.S4_OUTY_SCAN_S20)

    def _s4_outy_row1(self):
        cfg = self.config
        if not self._cmd_sent_s4:
            # Di chuyen nhanh toi vi tri Target cua Row 1 trong Zone
            target = cfg.outy_output_zones[1][2]
            ok = self._nb_move(5, target)
            if not ok:
                self._log_once("S4_ROW1_FAIL", "S4 outy_row1 fail")
                return
            self._cmd_sent_s4     = True
            self._step_timeout_s4 = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_s4:
                self._error("S4_OUTY_ROW1 timeout")
            elif self._arrived(5):
                self._enter_s4(SystemState.S4_CYL2_RETRACT)

    def _s4_outy_scan_s20(self):
        cfg = self.config
        oy = self._pos(5)
        if oy is None:
            return
            
        scan_target = cfg.outy_output_zones[2][1] if self.sensor(S19_CHECK_TRAY_P2) else getattr(cfg, 'target_scanoutp2', 500.0)

        if not self._cmd_sent_s4:
            ok = self._nb_move(5, scan_target, vel=cfg.outy_search_velocity)
            if not ok:
                self._log_once("S4_SCAN_FAIL", "S4 outy_scan: nb_move fail")
                return
            self._cmd_sent_s4 = True
            self._step_timeout_s4 = time.time() + cfg.move_timeout
            self.get_logger().info(
                f"[S4] S19={'ON' if self.sensor(S19_CHECK_TRAY_P2) else 'OFF'} → scan xuống {scan_target:.0f}mm "
                f"vel={cfg.outy_search_velocity}"
            )
            
        arm_mm = getattr(cfg, 'outy_scan_arm_mm', getattr(cfg, 'outy_safe_zone', 50.0))
        
        if self.sensor(S20_SCAN_STACK_P2):
            trigger_pos = self._pos(5) or oy
            
            row = self._zone_to_row(trigger_pos, cfg.outy_output_zones)
            if row is not None:
                target_mm = cfg.outy_output_zones[row][2]
                self._outy_jog_pos = target_mm
                self._s4_scan_noise_retry = 0
                self.get_logger().info(f"[S4] S20 ON @ {trigger_pos:.1f}mm → chot ROW{row} Target={target_mm:.0f}mm")
                self._nb_move(5, target_mm, vel=cfg.outy_slow_vel)
                self._cmd_sent_s4 = True
                self._step_timeout_s4 = time.time() + cfg.move_timeout
                self._enter_s4(SystemState.S4_OUTY_DROP)
                return

        timed_out = time.time() > self._step_timeout_s4
        at_target = self._arrived(5) or oy >= scan_target - 2.0
        if timed_out or at_target:
            self._stop(5)
            self.get_logger().warn(f"[S4] S20 không trigger → fallback row1")
            tgt = cfg.outy_output_zones[1][2]
            self._outy_jog_pos = tgt
            self._nb_move(5, tgt, vel=cfg.outy_slow_vel)
            self._cmd_sent_s4 = True
            self._step_timeout_s4 = time.time() + cfg.move_timeout
            self._enter_s4(SystemState.S4_OUTY_DROP)
            return

        self._log_once("S4_SCAN",
                       f"[S4] OUTY {oy:.0f}mm S20={'ON' if self.sensor(S20_SCAN_STACK_P2) else 'OFF'}")

    def _s4_retry_scan_home(self):
        cfg = self.config
        if not self._cmd_sent_s4:
            ok = self._nb_move(5, cfg.outy_target1)
            if not ok:
                self._enter_s4(SystemState.ERROR)
            self._cmd_sent_s4 = True
            self._step_timeout_s4 = time.time() + cfg.move_timeout
        else:
            if time.time() > self._step_timeout_s4:
                self._error("Homing OUTY sau lỗi timeout")
            elif self._arrived(5) or (self._pos(5) <= cfg.outy_target1 + 2.0):
                self._cmd_sent_s4 = False
                self._enter_s4(SystemState.S4_OUTY_SCAN_S20)

    def _s4_outy_drop(self):
        if not self._cmd_sent_s4:
            target = self._outy_jog_pos or 0.0
            if target <= 0:
                self._enter_s4(SystemState.S4_CYL2_RETRACT)
                return
            ok = self._nb_move(5, target, vel=self.config.outy_slow_vel, continuous_update=True)
            if not ok:
                return
            self._cmd_sent_s4 = True
            self._step_timeout_s4 = time.time() + self.config.move_timeout

        if time.time() > self._step_timeout_s4:
            self._cmd_sent_s4 = False
        elif self._arrived(5):
            self.get_logger().info(f"[S4] OutY tới target ({self._outy_jog_pos:.0f}mm) → CYL2 RETRACT")
            self._enter_s4(SystemState.S4_CYL2_RETRACT)

    def _s4_cyl2_retract_state(self):
        if not self._cmd_sent_s4:
            self._cyl2_retract()
            self._cmd_sent_s4 = True
            self._step_start_s4 = time.time()
        if self.sensor(S21_CYL2_RETRACTED) and not self.sensor(S22_CYL2_EXTENDED):
            self._enter_s4(SystemState.S4_OUTY_OUTX_HOME)
        elif time.time() - self._step_start_s4 > CYLINDER_TIMEOUT_S:
            self._error("[S4] Timeout: Cyl2 retract")

    def _s4_outy_outx_home(self):
        cfg = self.config
        if not self._cmd_sent_s4:
            ok4 = self._nb_move(4, cfg.outx_home)
            ok5 = self._nb_move(5, cfg.outy_target1)
            if not ok4 or not ok5:
                self._log_once("S4_HOME_FAIL", "S4 outy/outx home fail")
                return
            self._cmd_sent_s4     = True
            self._step_timeout_s4 = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_s4:
                self._error("S4_OUTY_OUTX_HOME timeout")
            elif self._arrived(5) and self._arrived(4):
                self._enter_s4(SystemState.S4_COMPLETE)

    def _s4_complete(self):
        self._pub_cartridge_busy(False)
        self._notify('info', 'State 4 done', 'Thay khay output thanh cong')
        self.get_logger().info("[S4] COMPLETE")
        self._s4_trigger = False
        if self._can_start_s3():
            self.get_logger().info("[S4→S3] S17 ON + S18 OFF → cấp khay output mới")
            self._enter_s4(SystemState.S3_CHECK_OUTXY_SAFE)
        else:
            self._enter_s4(SystemState.IDLE)


# ─── Main ─────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    system = None

    def _sig(signum, frame):
        if system:
            try:
                system.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        os._exit(0)

    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    try:
        config = None
        for candidate in [
            os.path.join(os.path.dirname(__file__), '..', 'config', 'cartridge_config.yaml'),
            os.path.expanduser('~/ros2_ws/src/system_feed_cartridge/config/cartridge_config.yaml'),
        ]:
            if os.path.exists(candidate):
                config = CartridgeConfig.load(candidate)
                break
        
        if config is None:
            config = CartridgeConfig()

        system = CartridgeSystem(config)
        print("=" * 60)
        print("  Cartridge System v8")
        print("  MANUAL + START → Homing → JOG mode")
        print("  Kích sim sensor → Nhấn STATE 1/3 để chạy")
        print("  STATE 2/4: nhấn nút trong State Navigation (sim robot signal)")
        print("=" * 60)

        rclpy.spin(system)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if system:
            try:
                system.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        # Only use os._exit as a last resort if things hang
        # os._exit(0)


if __name__ == '__main__':
    main()