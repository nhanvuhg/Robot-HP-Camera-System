#!/usr/bin/env python3
"""
Cartridge Loading System — ROS 2 + festo-edcon

Workflow:
  STATE 1  — Cap khay Input  (bang tai Input -> robot)
  STATE 2  — Thay khay Input (robot done -> output stack)

Patches v4 (tất cả rủi ro từ review):
  [FIX-1] CRITICAL: IO module init trong _connect_hardware(), KHONG phai trong
          _servo_reconnect_loop() (truoc day IO dat ngoai while -> khong bao gio chay)
  [FIX-2] BUG: Xoa 10 "double return" (return lien tiep khong co code giua)
  [FIX-3] BUG: _s2a_inx_10() — xoa dong _nb_move(inx_home) du thua, chi giu _nb_move(10.0)
  [FIX-4] LOGIC: _do_homing() kiem tra _state1_enabled truoc khi auto-enter STATE 1
  [FIX-5] LOGIC: Xoa state S1_CHECK_S5_ROW1 mo coi (enum + dispatcher + handler)
                 Doi ten S1_CHECK_S13 -> S1_CHECK_S13_ENTRY (tranh trung voi s1_check_s13)
                 Doi ten S1_INY_JOG  -> S1_INY_SCAN (dung position mode, khong phai jog)
  [FIX-6] DESIGN: _s1_complete() giu lai check S13 co timeout 3s cho safety
  [FIX-7] DESIGN: _s1_iny_scan() comment ro dung position mode, giai thich overshoot
  [FIX-8] DESIGN: _s1_check_s5() giu lai 1 tang retry (2 lan tong cong) truoc khi reset
  [FIX-9] MINOR: _log_once keys ngan gon, ASCII, khong cat giua

Patches v4 (review risks):
  [R-1] RACE: _home_all() — _ensure_ready() goi NGOAI lock (tranh sleep giu lock
        block control loop 20Hz); polling referenced() dung acquire(timeout=1.0)
        thay vi with-lock truc tiep.
  [R-2] TIMEOUT: _s2a_iny_jog_output() — jog S4 co timeout 120s; qua gio -> ERROR.
  [R-3] DEADLOCK: destroy_node() — dung acquire(timeout=2.0) thay vi with-lock;
        tranh deadlock voi _servo_reconnect_loop luc shutdown.
  [R-4] LOG_ONCE: _enter() — clear _guide_logged khi DOI SANG state moi (khong phai
        chi discard key == ten state); dam bao log lai khi re-enter state.
  [R-5] INIT: self._row1_pos = 0.0 trong __init__; bo getattr() workaround.
  [R-6] DELAY: _s1_place_delay() — dung bien _place_delay_start rieng, khong dung
        chung _step_start co the bi nhiem boi state truoc.

Patches v5 (fixes tu code-review):
  [V5-1] CRITICAL: servo_limits THIEU trong CartridgeConfig -> AttributeError moi lan
         goi _nb_move(). Da them self.servo_limits vao __init__.
  [V5-2] CRITICAL: _ensure_ready() duoc goi TRONG lock tai _nb_move() -> block loop
         20Hz (sleep 0.05 x N lan). Da chuyen ra ngoai with-block.
  [V5-3] BUG: _s4_outy_scan_s10() goi _jog() trong if-not-cmd_sent -> servo dung
         ngay sau tick dau. Da chuyen ra ngoai de goi moi tick.
  [V5-4] BUG: _s2a_iny_jog_output() guard chi return, khong ra lenh INY ve home
         -> servo dung yen mai mai. Da them _nb_move(iny_home) vao guard.
  [V5-5] BUG: STATE 3/4 handlers thieu timeout -> treo vinh vien neu servo fault.
         Da them _step_timeout cho toan bo handlers STATE 3/4.
  [V5-6] BUG: _s3_check_outxy_safe() dung outy_safe_zone de check OutX (sai bien)
         va khong doi move hoan tat truoc khi chuyen state. Da sua ca hai.
  [V5-7] MINOR: destroy_node() khong cleanup io_module. Da them.
  [V5-8] MINOR: _cb_goto_state() thong bao 'S12 OFF' nhung check sensor(17).
         Da sua notify text thanh 'S17 OFF'.

Patches v6 (logic STATE 1 INY scan):
  [V6-1] LOGIC: _s1_iny_scan() — loai bo hoan toan moi dang JOG+ trong STATE 1.
         INY chi di chuyen position-mode toi da den row1 (iny_input_stack[1] = 600mm).
         Neu S4 ON trong qua trinh di chuyen -> dung ngay -> tinh nearest row ->
         S1_INY_TO_ROW (giu nguyen).
         Neu den row1 ma S4 van OFF -> fallback: set _current_row = row1 va thu
         S1_INY_TO_ROW (thay vi reset ve home nhu cu). S1_CHECK_S5 se xac nhan
         co khay hay khong (2 lan retry truoc khi reset).
         Log ro khi fallback de operator biet S4 co van de.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, String
from enum import Enum
import time
from typing import Optional
import yaml
import json
import threading
import os
import signal

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
S5_WAIT_S            = 5.0
INY_JOG_VEL          = 40          # mm/s — toc do jog output stack
OUTY_ROW1_LIMIT_MM   = 590.0       # gioi han jog output (mm)
S13_CHECK_TIMEOUT_S  = 3.0         # timeout check S13 tai S1_COMPLETE
JOG_OUTPUT_TIMEOUT_S = 120.0       # [R-2] timeout toan bo jog output S4


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
    S1_INY_TO_ROW       = "s1_iny_to_row"
    S1_CHECK_S5         = "s1_check_s5"
    S1_WAIT_GUI_CONFIRM = "s1_wait_gui_confirm"
    S1_RETRY_INX_500    = "s1_retry_inx_500"
    S1_RETRY_JOG        = "s1_retry_jog"
    S1_CYL1_EXTEND      = "s1_cyl1_extend"
    S1_CHECK_S13_ENTRY  = "s1_check_s13_entry"
    S1_WAIT_S12_SAFE    = "s1_wait_s12_safe"
    S1_INY_50_CYL2      = "s1_iny_50_cyl2"
    S1_WAIT_S13_ON      = "s1_wait_s13_on"
    S1_INY_200          = "s1_iny_200"
    S1_PLACE_DELAY      = "s1_place_delay"
    S1_WAIT_RELEASE     = "s1_wait_release"
    S1_INY_50_FINAL     = "s1_iny_50_final"
    S1_INX_HOME         = "s1_inx_home"
    S1_COMPLETE         = "s1_complete"

    # STATE 2
    S2A_CHECK_INTERLOCK   = "s2a_check_interlock"
    S2A_INX_500           = "s2a_inx_500"
    S2A_INY_200_CYL1      = "s2a_iny_200_cyl1"
    S2A_WAIT_S11          = "s2a_wait_s11"
    S2A_INY_10            = "s2a_iny_10"
    S2A_INX_10            = "s2a_inx_10"
    S2A_INY_JOG_OUTPUT    = "s2a_iny_jog_output"
    S2A_INY_OUTPUT_ROW    = "s2a_iny_output_row"
    S2A_WAIT_S10          = "s2a_wait_s10"
    S2A_INY_10_FINAL      = "s2a_iny_10_final"
    S2A_INX_20            = "s2a_inx_20"
    S2A_WAIT_CYL2_RETRACT = "s2a_wait_cyl2_retract"
    S2A_COMPLETE          = "s2a_complete"

    # STATE 3 — Cấp khay thành phẩm (Servo 3 / Platform)
    S3_CHECK_OUTXY_SAFE  = "s3_check_outxy_safe"
    S3_SERVO3_TARGET1    = "s3_servo3_target1"
    S3_CHECK_S7          = "s3_check_s7"
    S3_WAIT_S7           = "s3_wait_s7"
    S3_WAIT_GUI_CONFIRM  = "s3_wait_gui_s7"
    S3_SERVO3_FEED       = "s3_servo3_feed"
    S3_WAIT_S8           = "s3_wait_s8"
    S3_COMPLETE          = "s3_complete"

    # STATE 4 — Thay khay output (OutX/OutY + Cylinder 3)
    S4_CHECK_OUTY_SAFE   = "s4_check_outy_safe"
    S4_OUTX_TARGET2      = "s4_outx_target2"
    S4_OUTY_PICK         = "s4_outy_pick"
    S4_CYL3_EXTEND       = "s4_cyl3_extend"
    S4_OUTY_TARGET1      = "s4_outy_target1"
    S4_OUTX_TARGET3      = "s4_outx_target3"
    S4_CHECK_S9          = "s4_check_s9"
    S4_OUTY_ROW1         = "s4_outy_row1"
    S4_OUTY_SCAN_S10     = "s4_outy_scan_s10"
    S4_OUTY_DROP         = "s4_outy_drop"
    S4_CYL3_RETRACT      = "s4_cyl3_retract"
    S4_OUTY_OUTX_HOME    = "s4_outy_outx_home"
    S4_COMPLETE          = "s4_complete"


# ─── Config ──────────────────────────────────────────────────────

class CartridgeConfig:
    def __init__(self, config_file: Optional[str] = None):
        self.servo_ips = {1: "192.168.27.248", 2: "192.168.27.249",
                          3: "192.168.27.250",  4: "192.168.27.251", 5: "192.168.27.252"}
        self.io_ip = "192.168.27.253"

        self.cylinder1_extend_channel  = 5
        self.cylinder1_retract_channel = 4
        self.cylinder2_extend_channel  = 9
        self.cylinder2_retract_channel = 8
        self.cylinder3_extend_channel  = 11  # [V5-NOTE] can phan biet voi cyl2 (ch9)
        self.cylinder3_retract_channel = 10  # [V5-NOTE] can phan biet voi cyl2 (ch8)

        self.inx_home       = 20.0
        self.inx_target     = 500.0
        self.inx_output_pos = 10.0
        self.iny_home       = 10.0
        self.iny_safe_zone  = 50.0
        self.iny_place      = 200.0

        self.s4_cross_offset_mm    = 200.0
        self.tray_height_mm        = 50.0
        self.output_approach_vel   = 10
        self.output_min_pos_mm     = 50.0
        self.iny_scan_arm_mm       = 30.0

        self.iny_input_stack = {
            8: 250.0, 7: 300.0, 6: 350.0, 5: 400.0,
            4: 450.0, 3: 500.0, 2: 550.0, 1: 600.0
        }
        self.iny_output_stack = {
            8: 100.0, 7: 170.0, 6: 240.0, 5: 310.0,
            4: 380.0, 3: 450.0, 2: 520.0, 1: 590.0
        }

        self.servo3_home          = 0.0
        self.servo3_target1       = 10.0
        self.servo3_target2       = 400.0
        self.servo3_feed_velocity = 50.0

        self.outx_home         = 0.0
        self.outx_target1      = 100.0
        self.outx_target2      = 400.0
        self.outx_target3      = 20.0
        self.outy_home         = 0.0
        self.outy_target1      = 10.0
        self.outy_target2      = 300.0
        self.outy_pick_pos     = 100.0
        self.outy_safe_zone    = 10.0
        self.outy_search_velocity = 40.0
        self.outy_row1_pos     = 700.0
        self.outy_row_limit    = 680.0
        self.outy_slow_vel     = 10.0
        self.outy_drop_extra   = 20.0
        self.homing_timeout    = 90.0
        self.move_timeout      = 25.0

        # [V5-1] CRITICAL FIX: servo_limits PHAI duoc dinh nghia o day.
        # Truoc day thieu -> _nb_move() crash AttributeError moi lan goi.
        self.servo_limits = {
            1: 600.0,   # InX
            2: 560.0,   # InY
            3: 450.0,   # Platform
            4: 500.0,   # OutX
            5: 800.0,   # OutY
        }

        self._config_file: Optional[str] = None

        if config_file and os.path.exists(config_file):
            self.load_from_file(config_file)

    def load_from_file(self, path: str):
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
            for k, v in data.items():
                if hasattr(self, k):
                    if isinstance(v, dict):
                        v = {int(kk): vv for kk, vv in v.items()}
                    setattr(self, k, v)
            self._config_file = path
            print(f"Config loaded: {path}")
        except Exception as e:
            print(f"Config load error: {e}")

    def save_to_file(self):
        if not self._config_file:
            return
        keys = ['servo_ips', 'io_ip', 'cylinder1_extend_channel', 'cylinder1_retract_channel',
                'cylinder2_extend_channel', 'cylinder2_retract_channel',
                'inx_home', 'inx_target', 'inx_output_pos',
                'iny_home', 'iny_safe_zone', 'iny_place',
                's4_cross_offset_mm', 'tray_height_mm', 'output_approach_vel',
                'output_min_pos_mm', 'iny_scan_arm_mm',
                'iny_input_stack', 'iny_output_stack', 'servo_limits',
                'homing_timeout', 'move_timeout']
        data = {}
        for k in keys:
            v = getattr(self, k, None)
            if isinstance(v, dict):
                v = {int(kk) if isinstance(kk, int) else kk: vv for kk, vv in v.items()}
            data[k] = v
        try:
            with open(self._config_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
        except Exception as e:
            print(f"Config save error: {e}")


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
        self._io_sensor_cache: list = []
        self._io_ready          = False
        self._io_bg_lock        = threading.Lock()
        self._sim_sensors: dict = {}

        # Motion flags
        self._inx_moving = False
        self._iny_moving = False

        # ── INPUT side state machine (S1 / S2A) ──────────────────
        self.state_in           = SystemState.IDLE
        self._cmd_sent_in       = False
        self._step_start_in     = 0.0
        self._step_timeout_in   = 0.0
        self._s4_armed          = False
        self._current_row       = 0
        self._s5_retry          = 0
        self._inx_arrived       = False
        self._30s_timeout       = 0.0
        self._s13_check_start   = 0.0
        self._s10_warn_t        = 0.0
        self._cyl_retry_t       = 0.0
        self._row1_pos          = 0.0
        self._row1_key          = 1
        self._place_delay_start = 0.0

        # STATE 2A runtime
        self._s6_snapshot       = False
        self._output_row        = 0
        self._output_target_pos = 0.0
        self._s4_armed_out      = False

        # ── OUTPUT side state machine (S3 / S4) ─────────────────
        self.state_out          = SystemState.IDLE
        self._cmd_sent_out      = False
        self._step_start_out    = 0.0
        self._step_timeout_out  = 0.0
        self._motion_busy       = False
        self._s4_trigger        = False
        self._s3_pending        = False
        self._outy_jog_start    = 0.0
        self._outy_jog_pos      = 0.0

        # Tray tracking
        self.stack_row_index  = 0
        self._tray_loaded_ack = False

        # Operation
        self.state           = SystemState.IDLE  # global: HOMING, ERROR
        self.operation_mode  = 'auto'
        self._input_tray_done     = False
        self._state1_enabled = False
        self._gui_confirmed  = False
        self._system_paused  = False

        # Watchdog
        self._watchdog_last_tick = time.time()
        self._notify_throttle: dict = {}
        self._guide_logged: set = set()

        # ROS Publishers
        qos = QoSProfile(depth=1)
        self.pub_state          = self.create_publisher(String, '/system_state',                                          qos)
        self.pub_new_tray       = self.create_publisher(Bool,   '/cartridge_providesystem/new_tray_loaded',        qos)  # State1 done
        self.pub_newtray_output = self.create_publisher(Bool,   '/cartridge_providesystem/new_trayoutput_loaded',  qos)  # State3 done
        self.pub_gui_notify     = self.create_publisher(String, '/providesystem/gui_notify',  qos)
        self.pub_servo_pos      = self.create_publisher(String, '/providesystem/servo_positions', qos)
        self.pub_sensors        = self.create_publisher(String, '/providesystem/sensors_state', qos)
        self.pub_busy_cartridge = self.create_publisher(Bool, '/cartridge/busy', qos)

        # ROS Subscribers
        self.create_subscription(Bool,   '/system/start_button',             self._cb_start,              qos)
        self.create_subscription(Bool,   '/system/stop_button',              self._cb_stop,               qos)
        self.create_subscription(Bool,   '/system/pause_button',             self._cb_pause,              qos)
        self.create_subscription(Bool,   '/robot/motion_busy',               self._cb_motion_busy,          qos)
        self.create_subscription(Bool,   '/robot/done_tray_output',          self._cb_done_tray_output,     qos)
        self.create_subscription(Bool,   '/robot/done_tray_input',           self._cb_done_tray_input,      qos)  # NEW
        self.create_subscription(Bool,   '/robot/last_batch_complete',       self._cb_last_batch_complete,  qos)  # NEW
        self.create_subscription(String, '/providesystem/gui_confirm',       self._cb_gui_confirm,        qos)
        self.create_subscription(String, '/providesystem/jog_cmd',           self._cb_jog,                qos)
        self.create_subscription(String, '/providesystem/sim_sensor',        self._cb_sim,                qos)
        self.create_subscription(String, '/providesystem/set_operation_mode',self._cb_mode,               qos)
        self.create_subscription(String, '/providesystem/goto_state',        self._cb_goto_state,         qos)

        self._connect_hardware()

        self.create_timer(0.05, self._control_loop)
        self.create_timer(0.5,  self._publish_positions)
        self.create_timer(5.0,  self._watchdog)

        self.get_logger().info("CartridgeSystem node started")

    # ══════════════════════════════════════════════════════════════
    # Hardware
    # ══════════════════════════════════════════════════════════════

    def _connect_hardware(self):
        """[FIX-1] IO module khoi tao TAI DAY."""
        if EDCON_AVAILABLE:
            if EdconLogging:
                EdconLogging()
            for sid, ip in self.config.servo_ips.items():
                self._connect_servo(sid, ip)
            threading.Thread(
                target=self._servo_reconnect_loop, daemon=True, name="servo_reconnect"
            ).start()
        else:
            self.get_logger().warn("Simulation mode (edcon not installed)")

        if CPXAP_AVAILABLE:
            for attempt in range(1, 4):
                try:
                    self.io_module = CpxAp(ip_address=self.config.io_ip, cycle_time=0.5)
                    self.get_logger().info(f"IO {self.config.io_ip} OK")
                    threading.Thread(target=self._io_bg_loop, daemon=True, name="io_bg").start()
                    break
                except Exception as e:
                    self.get_logger().warn(f"IO attempt {attempt}/3: {e}")
                    if attempt < 3:
                        time.sleep(3.0)
            else:
                self.get_logger().error("IO module failed — sensor reads will be False")

    def _connect_servo(self, sid: int, ip: str, attempts: int = 5) -> bool:
        for attempt in range(1, attempts + 1):
            try:
                com = ComModbus(ip_address=ip, cycle_time=60, timeout_ms=15000)
                with self._servo_lock:
                    tg = com.read_pnu(3490, 0)
                    if tg != 111:
                        com.write_pnu(3490, 0, 111)
                mot = MotionHandler(com)
                with self._servo_lock:
                    mot.acknowledge_faults()
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
        """Chi giam sat va reconnect SERVO."""
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
        [R-3] acquire(timeout=2.0) tranh deadlock voi _servo_reconnect_loop.
        [V5-7] Them cleanup io_module.
        """
        for sid, mot in list(self.servos.items()):
            if self._servo_lock.acquire(timeout=2.0):
                try:
                    mot.stop_motion_task()
                    mot.shutdown()
                except Exception as e:
                    self.get_logger().warn(f"S{sid} shutdown: {e}")
                finally:
                    self._servo_lock.release()
            else:
                self.get_logger().warn(f"S{sid} shutdown: lock timeout (bo qua)")

        # [V5-7] Cleanup IO module de dong TCP connection
        if self.io_module is not None:
            try:
                self.io_module = None
                self.get_logger().info("IO module closed")
            except Exception as e:
                self.get_logger().warn(f"IO module close error: {e}")

        super().destroy_node()

    # ── IO background reader ──────────────────────────────────────

    def _io_bg_loop(self):
        fail = 0
        while rclpy.ok():
            if self.io_module is None:
                time.sleep(0.5)
                continue
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
                fail = 0
            except Exception as e:
                fail += 1
                if fail >= 3:
                    self._io_ready = False
                    self.io_module = None
                    self.get_logger().warn(f"[IO-bg] reconnecting: {e}")
                    time.sleep(5.0)
                    try:
                        self.io_module = CpxAp(ip_address=self.config.io_ip)
                        fail = 0
                    except Exception:
                        pass
            time.sleep(0.05)

    # ── Sensor read ───────────────────────────────────────────────

    def _sensor_raw(self, sid: int) -> bool:
        with self._io_bg_lock:
            cache = self._io_sensor_cache
            ready = self._io_ready
        if not ready or len(cache) < sid:
            return False
        return bool(cache[sid - 1])

    def sensor(self, sid: int) -> bool:
        real = self._sensor_raw(sid)
        if self.operation_mode == 'manual':
            return real or self._sim_sensors.get(sid, False)
        return real

    def sensor_real(self, sid: int) -> bool:
        return self._sensor_raw(sid)

    def _snap(self, *sids: int) -> tuple:
        with self._io_bg_lock:
            cache = self._io_sensor_cache
            ready = self._io_ready
        is_manual = (self.operation_mode == 'manual')
        results = []
        for sid in sids:
            try:
                raw = bool(cache[sid - 1]) if ready and len(cache) >= sid else False
            except Exception:
                raw = False
            if is_manual:
                raw = raw or self._sim_sensors.get(sid, False)
            results.append(raw)
        return tuple(results)

    # ── Non-blocking servo motion ─────────────────────────────────

    def _nb_move(self, servo_id: int, pos_mm: float, vel: int = 30) -> bool:
        limit = self.config.servo_limits.get(servo_id, 999.0)
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
            offset = self.zero_offset.get(servo_id, 0)
            counts = offset + int(pos_mm * COUNTS_PER_MM)
            # [V5-2] CRITICAL FIX: _ensure_ready() NGOAI lock.
            # Ham nay co sleep(0.05) ben trong — neu giu lock se block control loop 20Hz.
            self._ensure_ready(mot, servo_id)
            with self._servo_lock:
                mot.position_task(counts, vel, absolute=True, nonblocking=True)
            if servo_id == 1: self._inx_moving = True
            elif servo_id == 2: self._iny_moving = True
            return True
        except Exception as e:
            self.get_logger().error(f"S{servo_id} nb_move error: {e}")
            return False

    def _arrived(self, servo_id: int) -> bool:
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

    def _jog(self, servo_id: int, vel_mm_s: float):
        if servo_id not in self.zero_offset and servo_id in self.servos:
            if self.operation_mode != 'manual':
                self.get_logger().error(f"S{servo_id} JOG blocked — chua home (AUTO)")
                return
            self.get_logger().warn(f"S{servo_id} JOG chua home")
        mot = self.servos.get(servo_id)
        if not mot:
            return
        try:
            # [V5-2] _ensure_ready NGOAI lock (nhu trong _nb_move)
            self._ensure_ready(mot, servo_id)
            positive = vel_mm_s > 0
            with self._servo_lock:
                mot.jog_task(jog_positive=positive, jog_negative=not positive, duration=0)
        except Exception as e:
            self.get_logger().error(f"S{servo_id} jog error: {e}")

    def _ensure_ready(self, mot, servo_id: int, timeout: float = 3.0):
        """
        [R-1] Ham nay CO sleep(0.05) ben trong.
        Goi tu _home_all(), _nb_move(), _jog() phai o NGOAI servo_lock.
        """
        mot.acknowledge_faults()
        mot.enable_powerstage()
        if not hasattr(mot, 'ready_for_motion'):
            return
        start = time.time()
        while time.time() - start < timeout:
            if mot.ready_for_motion():
                return
            time.sleep(0.05)
        self.get_logger().warn(f"S{servo_id}: drive not ready after {timeout}s")

    # ── Cylinders ─────────────────────────────────────────────────

    def _cyl1_extend(self) -> bool:
        self.get_logger().info("Cyl1 EXTEND (ch5)")
        if self.io_module:
            try:
                self.io_module.reset_channel(self.config.cylinder1_retract_channel)
                self.io_module.set_channel(self.config.cylinder1_extend_channel)
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl1 extend IO error: {e} — se retry")
                return False
        return False

    def _cyl1_retract(self) -> bool:
        self.get_logger().info("Cyl1 RETRACT (ch4)")
        if self.io_module:
            try:
                self.io_module.reset_channel(self.config.cylinder1_extend_channel)
                self.io_module.set_channel(self.config.cylinder1_retract_channel)
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl1 retract IO error: {e} — se retry")
                return False
        return False

    def _cyl2_extend(self) -> bool:
        self.get_logger().info("Cyl2 EXTEND (ch9) — Hold Tray")
        if self.io_module:
            try:
                self.io_module.reset_channel(self.config.cylinder2_retract_channel)
                self.io_module.set_channel(self.config.cylinder2_extend_channel)
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl2 extend IO error: {e} — se retry")
                return False
        return False

    def _cyl2_retract(self) -> bool:
        self.get_logger().info("Cyl2 RETRACT (ch8)")
        if self.io_module:
            try:
                self.io_module.reset_channel(self.config.cylinder2_extend_channel)
                if self.config.cylinder2_retract_channel:
                    self.io_module.set_channel(self.config.cylinder2_retract_channel)
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl2 retract IO error: {e} — se retry")
                return False
        return False

    # ── Helpers ──────────────────────────────────────────────────

    def _enter(self, next_state: SystemState):
        """Global state only: HOMING, HOMING_RUNNING, ERROR, IDLE."""
        self.get_logger().info(f"[GLOBAL] -> {next_state.name}")
        if next_state != self.state:
            self._guide_logged.clear()
        self.state = next_state

    def _enter_in(self, next_state: SystemState):
        """Input side state machine (S1/S2A)."""
        self.get_logger().info(f"[IN] -> {next_state.name}")
        self.state_in        = next_state
        self._cmd_sent_in    = False
        self._step_start_in  = 0.0
        self._step_timeout_in = 0.0

    def _enter_out(self, next_state: SystemState):
        """Output side state machine (S3/S4)."""
        self.get_logger().info(f"[OUT] -> {next_state.name}")
        self.state_out        = next_state
        self._cmd_sent_out    = False
        self._step_start_out  = 0.0
        self._step_timeout_out = 0.0

    def _error(self, msg: str):
        self.get_logger().error(f"ERROR: {msg}")
        self._notify('error', 'ERROR', msg)
        self._enter(SystemState.ERROR)

    def _notify(self, level: str, title: str, detail: str = ""):
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
        if key not in self._guide_logged:
            self._guide_logged.add(key)
            self.get_logger().info(f"[guide] {msg}")

    def _find_nearest_row_abs(self, pos_mm: float, row_dict: dict) -> int:
        return min(row_dict, key=lambda r: abs(row_dict[r] - pos_mm))

    def _calc_output_target(self, trigger_pos_mm: float) -> tuple:
        cfg             = self.config
        actual_tray_pos = trigger_pos_mm + cfg.s4_cross_offset_mm
        target_pos      = actual_tray_pos - cfg.tray_height_mm
        occupied_row    = self._find_nearest_row_abs(actual_tray_pos, cfg.iny_output_stack)
        is_full         = target_pos <= cfg.output_min_pos_mm
        return target_pos, occupied_row, is_full

    def _iny_safe(self) -> bool:
        p = self._pos(2)
        return p is not None and p <= self.config.iny_safe_zone

    def _can_start_s1(self) -> bool:
        if not self.zero_offset or self._motion_busy:
            return False
        has_tray  = self.sensor(1) or self.sensor(2) or self.sensor(3)
        place_ok  = self.sensor(17)
        s16_clear = not self.sensor(16)
        return has_tray and place_ok and s16_clear

    def _can_start_s2a(self) -> bool:
        return (bool(self.zero_offset) and self._input_tray_done
                and self.sensor(18) and not self._motion_busy)

    def _can_start_s3(self) -> bool:
        if self._motion_busy:
            return False
        return (bool(self.zero_offset)
                and self.sensor(7)
                and not self.sensor(8))

    def _can_start_s4(self) -> bool:
        return bool(self.zero_offset) and self._s4_trigger and not self._motion_busy

    def _cb_motion_busy(self, msg: Bool):
        self._motion_busy = msg.data

    def _cb_done_tray_output(self, msg: Bool):
        """Robot báo khay output đã đầy → trigger State4 thay khay."""
        if msg.data:
            self._s4_trigger = True
            self._notify('info', 'Output tray full', 'Trigger State 4 thay khay')
            self.pub_newtray_output.publish(Bool(data=False))

    def _cb_done_tray_input(self, msg: Bool):
        """Robot báo đã xong khay input (Row5 / không còn row) → reset, chờ State1 cấp khay mới."""
        if msg.data:
            self._input_tray_done = True
            self._state1_enabled = True
            self.get_logger().info('[DONE_INPUT] Robot xong khay input → sẵn sàng cấp khay mới (State1)')
            self._notify('info', 'Input tray done', 'Robot xong row5 — chờ cấp khay mới')
            self.pub_new_tray.publish(Bool(data=False))

    def _cb_last_batch_complete(self, msg: Bool):
        """
        Robot báo last batch xong (slot bất kỳ, không nhất thiết slot 9).
        Trigger State4 (thay khay output cũ) rồi State3 (cấp khay output mới).
        Tương tự done_tray_output nhưng xảy ra sớm hơn (khi bất kỳ slot cuối cùng được đặt xong).
        """
        if msg.data:
            self._s4_trigger = True  # trigger State4 thay khay cũ
            # State3 sẽ được trigger sau khi State4 hoàn thành (lòng của State4)
            self.get_logger().info('[LAST_BATCH] Last batch xong → trigger State4 thay khay output')
            self._notify('info', 'Last batch complete', 'Thay khay output — sẽ cấp khay mới (State3)')

    def _outy_safe(self) -> bool:
        p = self._pos(5)
        return p is not None and p <= self.config.outy_safe_zone

    def _cyl3_extend(self) -> bool:
        if self._io_ready and self.io_module:
            try:
                self.io_module.reset_channel(self.config.cylinder3_retract_channel)
                self.io_module.set_channel(self.config.cylinder3_extend_channel)
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl3 extend error: {e}")
                return False
        return False

    def _cyl3_retract(self) -> bool:
        if self._io_ready and self.io_module:
            try:
                self.io_module.reset_channel(self.config.cylinder3_extend_channel)
                self.io_module.set_channel(self.config.cylinder3_retract_channel)
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl3 retract error: {e}")
                return False
        return False

    # ══════════════════════════════════════════════════════════════
    # Homing
    # ══════════════════════════════════════════════════════════════

    def _home_all(self) -> bool:
        """
        [R-1] _ensure_ready() NGOAI lock; polling referenced() dung acquire(timeout=1.0).
        """
        NAMES  = {1: "InX", 2: "InY", 3: "Platform", 4: "OutX", 5: "OutY"}
        phases = [([2, 5], "InY+OutY"), ([1, 4, 3], "InX+OutX+Platform")]

        for servo_ids, phase_name in phases:
            active = [(sid, self.servos[sid]) for sid in servo_ids if sid in self.servos]
            if not active:
                continue
            self.get_logger().info(f"Homing {phase_name}...")

            for sid, mot in active:
                try:
                    self._ensure_ready(mot, sid, timeout=5.0)
                    with self._servo_lock:
                        mot.referencing_task(nonblocking=True)
                except Exception as e:
                    self.get_logger().error(f"{NAMES.get(sid)} home start: {e}")
                    return False

            start = time.time()
            while time.time() - start < self.config.homing_timeout:
                if self._servo_lock.acquire(timeout=1.0):
                    try:
                        done = all(mot.referenced() for _, mot in active)
                    finally:
                        self._servo_lock.release()
                else:
                    self.get_logger().warn(
                        f"Homing poll: lock busy ({phase_name}), retry sau 0.1s"
                    )
                    done = False
                if done:
                    break
                time.sleep(0.1)
            else:
                for sid, mot in active:
                    if self._servo_lock.acquire(timeout=1.0):
                        try:
                            if not mot.referenced():
                                self.get_logger().warn(
                                    f"{NAMES.get(sid)} timeout — assume home"
                                )
                                mot.stop_motion_task()
                        finally:
                            self._servo_lock.release()

            time.sleep(0.3)
            for sid, mot in active:
                if sid not in self.zero_offset:
                    if self._servo_lock.acquire(timeout=1.0):
                        try:
                            self.zero_offset[sid] = mot.current_position()
                        finally:
                            self._servo_lock.release()
                self.get_logger().info(f"  {NAMES.get(sid)} = 0mm")

        return True

    # ══════════════════════════════════════════════════════════════
    # ROS Callbacks
    # ══════════════════════════════════════════════════════════════

    def _cb_start(self, msg: Bool):
        if not msg.data or self.state == SystemState.HOMING_RUNNING:
            return
        self._system_paused = False
        self._inx_moving = self._iny_moving = False
        if self.operation_mode == 'manual':
            self._enter(SystemState.IDLE)
            self._notify('info', 'START (MANUAL)', 'Giu nguyen vi tri — chon HOMING neu can')
        else:
            self.zero_offset.clear()
            self._state1_enabled = True
            self._enter(SystemState.HOMING)
            self._notify('info', 'START — HOMING', '')

    def _cb_stop(self, msg: Bool):
        if not msg.data:
            return
        for sid in list(self.servos):
            self._stop(sid)
        self._enter(SystemState.IDLE)
        self._system_paused  = False
        self._input_tray_done     = False
        self._state1_enabled = False
        self._notify('warn', 'STOP', '')

    def _cb_pause(self, msg: Bool):
        if not msg.data:
            return
        self._system_paused = True
        self._notify('warn', 'PAUSE', 'Nhan RESUME de tiep tuc')

    def _cb_gui_confirm(self, msg: String):
        data = msg.data.strip()
        if data == 'S16_OK':
            # User chọn OK: lấy khay cũ ra (S2A) rồi cấp khay mới (S1)
            self.get_logger().info('[S16] OK → chạy S2A để lấy khay ra, sau đó S1')
            self._notify('info', 'S16: Thực hiện State2', 'Lấy khay ra rồi cấp khay mới')
            self._input_tray_done = False  # reset để S2A tiếp tục đúng flow
            self._state1_enabled = True
            self._enter_in(SystemState.S2A_CHECK_INTERLOCK)
        elif data == 'S16_NO':
            # User chọn NO: dừng, chờ start lại
            self.get_logger().info('[S16] NO → IDLE, chờ start')
            self._notify('warn', 'S16: Dừng', 'Nhấn START lại khi sẵn sàng')
            self._state1_enabled = False
            self._enter_in(SystemState.IDLE)
        else:
            self._gui_confirmed = True

    def _cb_mode(self, msg: String):
        requested = msg.data.strip().lower()
        if requested not in ('auto', 'manual'):
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
        if requested == 'auto':
            self._sim_sensors.clear()
        if old != requested:
            self._guide_logged.clear()
        self._notify('info', f'Mode: {requested.upper()}', '')

    def _cb_jog(self, msg: String):
        if self.operation_mode != 'manual':
            self._notify('warn', 'JOG bi khoa', 'Chuyen MANUAL mode de JOG')
            return
        if self.state not in (SystemState.IDLE, SystemState.ERROR):
            self._notify('warn', 'JOG bi khoa', f'Dang chay — nhan STOP truoc')
            return
        parts = msg.data.strip().split()
        if len(parts) < 2:
            return
        try:
            cmd0 = parts[0].lower()
            if cmd0 == 'home':
                sid = int(parts[1])
                mot = self.servos.get(sid)
                if mot:
                    def _do():
                        # [V5-2] _ensure_ready NGOAI lock
                        self._ensure_ready(mot, sid, timeout=5.0)
                        with self._servo_lock:
                            mot.referencing_task(nonblocking=False)
                        if sid not in self.zero_offset:
                            with self._servo_lock:
                                self.zero_offset[sid] = mot.current_position()
                    threading.Thread(target=_do, daemon=True).start()
                return
            if cmd0 == 'clear':
                sid = int(parts[1])
                mot = self.servos.get(sid)
                if mot:
                    with self._servo_lock:
                        mot.acknowledge_faults()
                return
            sid = int(parts[0])
            d   = parts[1]
            vel = int(parts[2]) if len(parts) > 2 else 30
            if d == 'stop':  self._stop(sid)
            elif d == '+':   self._jog(sid,  vel)
            elif d == '-':   self._jog(sid, -vel)
        except Exception:
            pass

    def _cb_sim(self, msg: String):
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
                for i in range(1, 16):
                    self._sim_sensors[i] = val
            else:
                self._sim_sensors[int(sid_s)] = val
        except Exception:
            pass

    def _cb_goto_state(self, msg: String):
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
            return
        if self.operation_mode != 'manual':
            self._notify('warn', 'goto_state bi khoa', 'Chi dung trong MANUAL mode')
            return
        if cmd in ('STATE1', 'STATE_1', 'S1'):
            if self.state_in not in (SystemState.IDLE,):
                self._notify('warn', 'Không thể vào STATE 1',
                             f'{self.state_in.name} đang chạy — STOP trước')
                return
            if not self.zero_offset:
                self._notify('warn', 'Chua home', '')
                return
            if self._motion_busy:
                self._notify('warn', 'Robot đang bận', 'Chờ robot idle rồi thử lại')
                return
            if not self.sensor(17):
                self._notify('warn', 'S17 OFF', 'Vi tri cap dang co khay — sim S17=1')
                return
            if not (self.sensor(1) or self.sensor(2) or self.sensor(3)):
                self._notify('warn', 'S1/S2/S3 OFF', 'Không có khay trên băng tải')
                return
            if self.sensor(16):
                self._notify('warn', 'S16 ON', 'Còn khay tại extract — dọn trước')
                return
            self._state1_enabled = True
            self._notify('info', 'STATE 1 (manual)', 'Doi S1/S2/S3 trong process')
            self._enter_in(SystemState.S1_CONFIRM_SAFE)
        elif cmd in ('STATE2', 'STATE_2', 'S2', 'STATE2A', 'S2A'):
            if self.state_in not in (SystemState.IDLE, SystemState.S1_COMPLETE):
                self._notify('warn', 'Không thể vào STATE 2',
                             f'{self.state_in.name} đang chạy — STOP trước')
                return
            if not self.zero_offset:
                self._notify('warn', 'Chua home', '')
                return
            if not self.sensor(18):
                self._notify('warn', 'S18 OFF', 'Khong co khay tai robot — sim S18=1')
                return
            self._input_tray_done = False
            self._notify('info', 'STATE 2A (manual)', 'S18 ON')
            self._enter_in(SystemState.S2A_CHECK_INTERLOCK)
        elif cmd in ('STATE3', 'STATE_3', 'S3'):
            if self.state_out not in (SystemState.IDLE,):
                self._notify('warn', 'Không thể vào STATE 3',
                             f'{self.state_out.name} đang chạy — STOP trước')
                return
            if not self.zero_offset:
                self._notify('warn', 'Chua home', '')
                return
            if self._motion_busy:
                self._notify('warn', 'Robot đang bận', 'Chờ robot idle rồi thử lại')
                return
            if not self.sensor(7):
                self._notify('warn', 'S7 OFF', 'Chua co khay tren Platform — sim S7=1')
                return
            if self.sensor(8):
                self._notify('warn', 'S8 ON', 'Vi tri robot da co khay — khong can cap')
                return
            self._notify('info', 'STATE 3 (manual)', 'S7 ON + S8 OFF — cap khay')
            self._enter_out(SystemState.S3_CHECK_OUTXY_SAFE)
        elif cmd in ('STATE4', 'STATE_4', 'S4'):
            if self.state_out not in (SystemState.IDLE,):
                self._notify('warn', 'Không thể vào STATE 4',
                             f'{self.state_out.name} đang chạy — STOP trước')
                return
            if not self.zero_offset:
                self._notify('warn', 'Chua home', '')
                return
            if self._motion_busy:
                self._notify('warn', 'Robot đang bận', 'Chờ robot idle rồi thử lại')
                return
            self._notify('info', 'STATE 4 (manual)', 'Thay khay output')
            self._enter_out(SystemState.S4_CHECK_OUTY_SAFE)
        else:
            self._notify('warn', f'goto_state: khong biet "{cmd}"', '')

    # ══════════════════════════════════════════════════════════════
    # Control Loop
    # ══════════════════════════════════════════════════════════════

    def _control_loop(self):
        self._watchdog_last_tick = time.time()
        if self._system_paused:
            return
        self._process_state()
        self._publish_state()
        self._publish_sensors()

    def _publish_sensors(self):
        # Tạo chuỗi 18 ký tự trạng thái sensor (1-18)
        states = "".join("1" if self.sensor(i) else "0" for i in range(1, 19))
        msg = String(); msg.data = states; self.pub_sensors.publish(msg)

    def _publish_state(self):
        # Publish combined state: global | input_side | output_side
        combined = f"{self.state.value}|{self.state_in.value}|{self.state_out.value}"
        msg = String(); msg.data = combined; self.pub_state.publish(msg)

    def _publish_positions(self):
        try:
            pos = {}
            for sid in self.servos:
                p = self._pos(sid)
                if p is not None:
                    pos[str(sid)] = round(p, 2)
            msg = String(); msg.data = json.dumps(pos); self.pub_servo_pos.publish(msg)
        except Exception:
            pass

    def _watchdog(self):
        gap = time.time() - self._watchdog_last_tick
        if gap > 3.0:
            self.get_logger().error(f"WATCHDOG: loop silent {gap:.1f}s!")
            self._notify('error', 'Loop treo!', f'{gap:.1f}s khong tick')

    # ══════════════════════════════════════════════════════════════
    # State Dispatcher
    # ══════════════════════════════════════════════════════════════

    def _process_state(self):
        s = self.state
        # Global states take full control
        if s == SystemState.HOMING:           self._do_homing(); return
        if s == SystemState.HOMING_RUNNING:   return
        if s == SystemState.ERROR:            self._do_error(); return
        # Normal operation: dispatch BOTH sub-machines in same tick
        self._dispatch_input()
        self._dispatch_output()

    def _dispatch_input(self):
        """Input side: S1 / S2A states (servo 1=InX, 2=InY)."""
        s = self.state_in
        if   s == SystemState.IDLE:                self._do_idle_input()
        elif s == SystemState.S1_CONFIRM_SAFE:     self._s1_confirm_safe()
        elif s == SystemState.S1_INX_MOVE:         self._s1_inx_move()
        elif s == SystemState.S1_WAIT_ARRIVE:      self._s1_wait_arrive()
        elif s == SystemState.S1_INY_SCAN:         self._s1_iny_scan()
        elif s == SystemState.S1_INY_TO_ROW:       self._s1_iny_to_row()
        elif s == SystemState.S1_CHECK_S5:         self._s1_check_s5()
        elif s == SystemState.S1_WAIT_GUI_CONFIRM: self._s1_wait_gui_confirm()
        elif s == SystemState.S1_RETRY_INX_500:    self._s1_retry_inx_500()
        elif s == SystemState.S1_RETRY_JOG:        self._s1_retry_jog()
        elif s == SystemState.S1_CYL1_EXTEND:      self._s1_cyl1_extend()
        elif s == SystemState.S1_CHECK_S13_ENTRY:  self._s1_check_s13_entry()
        elif s == SystemState.S1_WAIT_S12_SAFE:    self._s1_wait_s12_safe()
        elif s == SystemState.S1_INY_50_CYL2:      self._s1_iny_50_cyl2()
        elif s == SystemState.S1_WAIT_S13_ON:      self._s1_wait_s13_on()
        elif s == SystemState.S1_INY_200:          self._s1_iny_200()
        elif s == SystemState.S1_PLACE_DELAY:      self._s1_place_delay()
        elif s == SystemState.S1_WAIT_RELEASE:     self._s1_wait_release()
        elif s == SystemState.S1_INY_50_FINAL:     self._s1_iny_50_final()
        elif s == SystemState.S1_INX_HOME:         self._s1_inx_home()
        elif s == SystemState.S1_COMPLETE:         self._s1_complete()
        elif s == SystemState.S2A_CHECK_INTERLOCK:   self._s2a_check_interlock()
        elif s == SystemState.S2A_INX_500:           self._s2a_inx_500()
        elif s == SystemState.S2A_INY_200_CYL1:      self._s2a_iny_200_cyl1()
        elif s == SystemState.S2A_WAIT_S11:          self._s2a_wait_s11()
        elif s == SystemState.S2A_INY_10:            self._s2a_iny_10()
        elif s == SystemState.S2A_INX_10:            self._s2a_inx_10()
        elif s == SystemState.S2A_INY_JOG_OUTPUT:    self._s2a_iny_jog_output()
        elif s == SystemState.S2A_INY_OUTPUT_ROW:    self._s2a_iny_output_row()
        elif s == SystemState.S2A_WAIT_S10:          self._s2a_wait_s10()
        elif s == SystemState.S2A_INY_10_FINAL:      self._s2a_iny_10_final()
        elif s == SystemState.S2A_INX_20:            self._s2a_inx_20()
        elif s == SystemState.S2A_WAIT_CYL2_RETRACT: self._s2a_wait_cyl2_retract()
        elif s == SystemState.S2A_COMPLETE:          self._s2a_complete()

    def _dispatch_output(self):
        """Output side: S3 / S4 states (servo 3=Platform, 4=OutX, 5=OutY)."""
        s = self.state_out
        if   s == SystemState.IDLE:                self._do_idle_output()
        elif s == SystemState.S3_CHECK_OUTXY_SAFE: self._s3_check_outxy_safe()
        elif s == SystemState.S3_SERVO3_TARGET1:   self._s3_servo3_target1()
        elif s == SystemState.S3_CHECK_S7:         self._s3_check_s7()
        elif s == SystemState.S3_WAIT_S7:          self._s3_wait_s7()
        elif s == SystemState.S3_WAIT_GUI_CONFIRM: self._s3_wait_gui_confirm()
        elif s == SystemState.S3_SERVO3_FEED:      self._s3_servo3_feed()
        elif s == SystemState.S3_WAIT_S8:          self._s3_wait_s8()
        elif s == SystemState.S3_COMPLETE:         self._s3_complete()
        elif s == SystemState.S4_CHECK_OUTY_SAFE:  self._s4_check_outy_safe()
        elif s == SystemState.S4_OUTX_TARGET2:     self._s4_outx_target2()
        elif s == SystemState.S4_OUTY_PICK:        self._s4_outy_pick()
        elif s == SystemState.S4_CYL3_EXTEND:      self._s4_cyl3_extend()
        elif s == SystemState.S4_OUTY_TARGET1:     self._s4_outy_target1()
        elif s == SystemState.S4_OUTX_TARGET3:     self._s4_outx_target3()
        elif s == SystemState.S4_CHECK_S9:         self._s4_check_s9()
        elif s == SystemState.S4_OUTY_ROW1:        self._s4_outy_row1()
        elif s == SystemState.S4_OUTY_SCAN_S10:    self._s4_outy_scan_s10()
        elif s == SystemState.S4_OUTY_DROP:        self._s4_outy_drop()
        elif s == SystemState.S4_CYL3_RETRACT:     self._s4_cyl3_retract_state()
        elif s == SystemState.S4_OUTY_OUTX_HOME:   self._s4_outy_outx_home()
        elif s == SystemState.S4_COMPLETE:         self._s4_complete()


    # ── IDLE ─────────────────────────────────────────────────────

    # ── IDLE (dual) ───────────────────────────────────────────────

    def _do_idle_input(self):
        """Input side IDLE: kiểm tra điều kiện vào S1 hoặc S2A."""
        if not self.zero_offset:
            self._log_once("IDLE_IN_NOT_HOMED", "IDLE-IN: Chua home")
            return
        if self.operation_mode == 'manual':
            return
        # S2A: robot done (khay đầy) → thay khay input
        if self._can_start_s2a():
            self._input_tray_done = False
            self.get_logger().info("[IN-IDLE] Robot done → STATE 2")
            self._enter_in(SystemState.S2A_CHECK_INTERLOCK)
            return
        # S16 ON: hiện dialog, chờ response
        if self._state1_enabled and self.sensor(16):
            self._log_once("IDLE_IN_S16",
                           "[IN-IDLE] S16 ON — còn khay tại extract")
            self._notify('warn', 'S16: Còn khay extract',
                         'Chọn OK để lấy khay ra rồi tiếp tục')
            return
        # S1: cấp khay mới cho robot
        if self._state1_enabled and self._can_start_s1():
            self.get_logger().info("[IN-IDLE] Đủ điều kiện → STATE 1")
            self._guide_logged.discard("IDLE_IN_S16")
            self._enter_in(SystemState.S1_CONFIRM_SAFE)
            return
        # Log reason
        if not (self.sensor(1) or self.sensor(2) or self.sensor(3)):
            self._log_once("IDLE_IN_NO_TRAY", "[IN-IDLE] S1/S2/S3 OFF — hết khay input")
        elif not self.sensor(17):
            self._log_once("IDLE_IN_NO_PLACE", "[IN-IDLE] S17 OFF — vị trí cấp đang có khay")
        elif self._motion_busy:
            self._log_once("IDLE_IN_BUSY", "[IN-IDLE] Robot đang bận — chờ")

    def _do_idle_output(self):
        """Output side IDLE: kiểm tra điều kiện vào S4 hoặc S3."""
        if not self.zero_offset:
            return
        if self.operation_mode == 'manual':
            return
        # S4: thay khay output đầy
        if self._can_start_s4():
            self._s4_trigger = False
            self.get_logger().info("[OUT-IDLE] Output full → STATE 4")
            self._enter_out(SystemState.S4_CHECK_OUTY_SAFE)
            return
        # S3: cấp khay output mới (S7 ON, S8 OFF)
        if self._can_start_s3():
            self.get_logger().info("[OUT-IDLE] S7 ON + S8 OFF → STATE 3")
            self._enter_out(SystemState.S3_CHECK_OUTXY_SAFE)


    def _do_homing(self):
        self._enter(SystemState.HOMING_RUNNING)
        def _bg():
            ok = self._home_all()
            if ok:
                self.get_logger().info("Homing complete — reset state_in/out → IDLE")
                self._notify('info', 'Homing xong',
                             'Tự động State1 (nếu có khay input) + State3 (nếu có khay output)')
                self.state_in  = SystemState.IDLE
                self.state_out = SystemState.IDLE
                self._enter(SystemState.IDLE)
            else:
                self._error("Homing that bai")
        threading.Thread(target=_bg, daemon=True).start()

    def _do_error(self):
        self._log_once("ERROR_STATE", "ERROR — kiem tra loi roi nhan STOP -> START")

    # ══════════════════════════════════════════════════════════════
    # STATE 1: Cap khay Input
    # ══════════════════════════════════════════════════════════════

    def _s1_confirm_safe(self):
        if not self._cmd_sent_in and not self._inx_moving and not self._iny_moving:
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
                    self._log_once("S1_INY_HOME_FAIL", "S1_SAFE: INY home fail — retry")
                    return
                self._cmd_sent_in     = True
                self._step_timeout_in = time.time() + self.config.move_timeout
            else:
                if time.time() > self._step_timeout_in:
                    self._cmd_sent_in = False
                elif self._arrived(2):
                    self._enter_in(SystemState.S1_INX_MOVE)

    def _s1_inx_move(self):
        if not (self.sensor(1) or self.sensor(2) or self.sensor(3)):
            self._log_once("S1_WAIT_BELT",
                           f"Step2: Cho S1/S2/S3 | S1={self.sensor(1)} S2={self.sensor(2)} S3={self.sensor(3)}")
            self._notify('info', 'Cho khay (S1/S2/S3)', 'Dat khay len bang tai.')
            return
        if not self._iny_safe():
            self._log_once("S1_INY_SAFE", "Step2: INY chua safe")
            return
        if not self._cmd_sent_in:
            ok = self._nb_move(1, self.config.inx_target)
            if not ok:
                self._log_once("S1_INX_FAIL", "S1 Step2: INX fail — retry")
                return
            self._cmd_sent_in    = True
            self._inx_arrived = False
            self._30s_timeout = 0.0
            self._s10_warn_t  = 0.0
            self.get_logger().info(f"Step2: INX -> {self.config.inx_target}mm")
            self._enter_in(SystemState.S1_WAIT_ARRIVE)

    def _s1_wait_arrive(self):
        if not self._inx_arrived:
            self._inx_arrived = self._arrived(1)
            if not self._inx_arrived:
                self._log_once("S1_INX_MOVING", "INX dang di chuyen — INY bi BLOCK")
                return
            self._30s_timeout = time.time() + 50.0
            self.get_logger().info("INX dung tai 500mm — check S3+S10 (50s)")

        s3, s10 = self._snap(3, 10)

        if s3 and s10:
            self.get_logger().info("S3+S10 ON -> scan INY")
            self._s4_armed = False
            self._enter_in(SystemState.S1_INY_SCAN)
            return

        if not s3:
            if time.time() > self._30s_timeout:
                self.get_logger().info(
                    "S3 khong ON sau 50s (co the nhieu S1/S2) — INX ve home, retry"
                )
                self._nb_move(1, self.config.inx_home)
                self._enter_in(SystemState.S1_CONFIRM_SAFE)
                return
            remain = self._30s_timeout - time.time()
            self._log_once("S1_WAIT_S3", f"Cho S3 ON (con {remain:.0f}s)")
        else:
            if self._s10_warn_t == 0:
                self._s10_warn_t = time.time()
            elif time.time() - self._s10_warn_t >= 5.0:
                self._notify('warn', 'S3 ON nhung S10 OFF',
                             'Cyl1 chua retract — kiem tra xi lanh hoac sensor S10.')
                self._s10_warn_t = time.time()
            self._log_once("S1_WAIT_S10", "S3 ON, cho S10 ON (Cyl1 chua retract)")

    def _s1_iny_scan(self):
        """
        [V6-1] INY scan INPUT stack — POSITION MODE ONLY, khong JOG.

        Luong xu ly:
          1. Gui lenh position-mode den row1 (600mm = max iny_input_stack).
          2. Moi tick: poll S4. Neu S4 ON -> dung ngay -> tinh nearest row
             -> S1_INY_TO_ROW.
          3. Neu INY den row1 ma S4 van OFF -> fallback: thu row1 truc tiep.
             S1_CHECK_S5 se xac nhan co khay khong (co 2 lan retry truoc khi reset).
             Tranh truong hop S4 bi nhieu / sensor delay ma tu dong reset ve home.

        Khong co jog, khong co reset ngay tai day — moi truong hop deu di qua
        S1_INY_TO_ROW -> S1_CHECK_S5 de xu ly dong nhat.
        """
        iny = self._pos(2)
        if iny is None:
            return

        # ── Khoi dong: gui lenh position-mode den row1 ──────────
        if not self._cmd_sent_in:
            # Row1 = vi tri co mm cao nhat trong iny_input_stack (600mm)
            self._row1_pos = max(self.config.iny_input_stack.values())
            # Row key tuong ung voi row1 (dung lam fallback)
            self._row1_key = max(
                self.config.iny_input_stack,
                key=lambda r: self.config.iny_input_stack[r]
            )
            ok = self._nb_move(2, self._row1_pos)
            if not ok:
                self._log_once("S1_SCAN_FAIL", "S1 INY scan fail — retry")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
            self.get_logger().info(
                f"[S1 SCAN] INY pos-mode -> {self._row1_pos:.0f}mm (row{self._row1_key}) | "
                f"S4 arm >= {self.config.iny_scan_arm_mm:.0f}mm | "
                f"offset = {self.config.s4_cross_offset_mm:.0f}mm"
            )

        # ── Arm S4 sau nguong an toan ───────────────────────────
        if iny >= self.config.iny_scan_arm_mm:
            self._s4_armed = True

        # ── S4 trigger trong luc di chuyen -> nearest row ───────
        if self._s4_armed and self.sensor(4):
            self._stop(2)
            trigger_pos       = self._pos(2) or iny
            actual_tray       = trigger_pos + self.config.s4_cross_offset_mm
            self._current_row = self._find_nearest_row_abs(
                actual_tray, self.config.iny_input_stack
            )
            target_mm = self.config.iny_input_stack[self._current_row]
            self.get_logger().info(
                f"[S1 SCAN] S4 ON tai {trigger_pos:.1f}mm | "
                f"tray ~ {actual_tray:.0f}mm -> Row{self._current_row} ({target_mm:.0f}mm)"
            )
            self._s5_retry = 0
            self._enter_in(SystemState.S1_INY_TO_ROW)
            return

        # ── Da den row1 (hoac timeout) ma S4 chua ON ────────────
        if self._arrived(2) or iny >= self._row1_pos - 2.0 or time.time() > self._step_timeout_in:
            self._stop(2)
            # [V6-1] Fallback: thu row1 thay vi reset ve home.
            # S4 co the bi nhieu hoac delay; S1_CHECK_S5 se xac nhan co khay khong.
            # Neu that su khong co khay: S1_CHECK_S5 retry 2 lan roi moi reset.
            timed_out = time.time() > self._step_timeout_in
            self._current_row = self._row1_key
            self.get_logger().warn(
                f"[S1 SCAN] S4 khong trigger den {iny:.1f}mm "
                f"({'timeout' if timed_out else 'da den row1'}) — "
                f"fallback thu Row{self._current_row} "
                f"({self.config.iny_input_stack[self._current_row]:.0f}mm)"
            )
            self._notify(
                'warn', 'S4 khong trigger',
                f'INY {"timeout" if timed_out else "den row1"} — thu Row{self._current_row}'
            )
            self._s5_retry = 0
            self._enter_in(SystemState.S1_INY_TO_ROW)
            return

        # ── Dang tren duong di ──────────────────────────────────
        self._log_once(
            "S1_SCANNING",
            f"[S1 SCAN] INY pos-mode | pos={iny:.0f}mm "
            f"arm={'OK' if self._s4_armed else f'NO (<{self.config.iny_scan_arm_mm:.0f}mm)'}"
        )

    def _s1_iny_to_row(self):
        target = self.config.iny_input_stack.get(self._current_row)
        if target is None:
            valid = sorted(self.config.iny_input_stack.keys())
            self._current_row = max(valid[0], min(valid[-1], self._current_row))
            target = self.config.iny_input_stack[self._current_row]
        if not self._cmd_sent_in:
            ok = self._nb_move(2, target)
            if not ok:
                self._log_once("S1_ROW_FAIL", f"S1 Step5: INY -> {target}mm fail")
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
        """[FIX-8] 2 lan thu (2 x 5s) truoc khi reset."""
        if not self._cmd_sent_in:
            self._step_start_in = time.time()
            self._cmd_sent_in   = True

        if self.sensor(5):
            self.get_logger().info(f"S5 ON (lan {self._s5_retry+1}) — extend Cyl1")
            self._cyl1_extend()
            self._enter_in(SystemState.S1_CYL1_EXTEND)
            return

        elapsed = time.time() - self._step_start_in
        self._log_once("S1_WAIT_S5",
                       f"Cho S5 ON (con {max(0.0, S5_WAIT_S - elapsed):.1f}s, "
                       f"lan {self._s5_retry+1}/2) row {self._current_row}")

        if elapsed >= S5_WAIT_S:
            if self._s5_retry == 0:
                self._s5_retry = 1
                self._cmd_sent_in = False
                self._guide_logged.discard("S1_WAIT_S5")
                self.get_logger().warn(f"S5 OFF lan 1 tai row {self._current_row} — thu lai lan 2")
            else:
                self.get_logger().warn(f"S5 OFF 2 lan tai row {self._current_row} — reset")
                self._notify('warn', 'S5 OFF 2 lan', f'Row {self._current_row} — reset')
                self._s5_retry = 0
                self._nb_move(2, self.config.iny_home)
                self._nb_move(1, self.config.inx_home)
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
                ok = self._nb_move(1, self.config.inx_home)
                if ok:
                    self._cmd_sent_in = True
        if self._gui_confirmed:
            self._gui_confirmed = False
            self._stop(1); self._stop(2)
            self._enter_in(SystemState.S1_RETRY_JOG)
        self._log_once("S1_GUI_WAIT", "Cho nhan XAC NHAN tren GUI")

    def _s1_retry_jog(self):
        if not (self.sensor(1) or self.sensor(2) or self.sensor(3)):
            self._log_once("S1_NO_TRAY", "Het khay — S1/S2/S3 OFF")
            self._notify('warn', 'Het khay', 'Nap khay roi nhan START')
            return
        inx = self._pos(1)
        if inx is None:
            return
        if abs(inx - self.config.inx_target) < 15.0:
            self._s4_armed = False; self._s5_retry = 0
            self._enter_in(SystemState.S1_INY_SCAN)
        else:
            self._enter_in(SystemState.S1_RETRY_INX_500)

    def _s1_retry_inx_500(self):
        if not self._cmd_sent_in:
            if not self._iny_safe():
                self._log_once("S1_RETRY_INY", "Cho INY safe truoc INX 500mm")
                return
            ok = self._nb_move(1, self.config.inx_target)
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
        s16, = self._snap(16)
        if not self._cmd_sent_in:
            self._cyl1_extend()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in    = True
        if s16:
            self.get_logger().info("S16 ON — gap khay OK -> check S18")
            self._enter_in(SystemState.S1_CHECK_S13_ENTRY)
            return
        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Retry Cyl1 extend")
            self._cyl1_extend()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S1_WAIT_S16", "Cho S16 ON (Cyl1 extend). Kich: '16:1'")

    def _s1_check_s13_entry(self):
        if self.sensor(17):
            self.get_logger().info("S17 ON -> INY ve 50mm")
            self._enter_in(SystemState.S1_INY_50_CYL2)
        else:
            self._enter_in(SystemState.S1_WAIT_S12_SAFE)

    def _s1_wait_s12_safe(self):
        if self.sensor(17):
            self.get_logger().info("S17 ON — Cyl2 da ve")
            self._enter_in(SystemState.S1_INY_50_CYL2)
        else:
            self._log_once("S1_WAIT_S17", "Cho S17 ON. Kich: '17:1'")

    def _s1_iny_50_cyl2(self):
        if not self.sensor(17):
            if self._cmd_sent_in:
                self._stop(2); self._cmd_sent_in = False
                self.get_logger().warn("[INTERLOCK] S17 OFF — DUNG INY!")
            self._log_once("S1_S17_ILK", "INTERLOCK: S17=OFF -> INY bi khoa")
            self._notify('warn', 'S17 chua ON', 'Cyl2 chua retract — INY bi khoa')
            return
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_safe_zone)
            if not ok:
                self.get_logger().warn("S1 Step8: INY -> 50mm fail — retry")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(2):
                self.get_logger().info("INY tai 50mm -> Cyl2 EXTEND (ch9)")
                self._cyl2_extend()
                self._enter_in(SystemState.S1_WAIT_S13_ON)

    def _s1_wait_s13_on(self):
        s18, = self._snap(18)
        if not self._cmd_sent_in:
            self._cyl2_extend()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in    = True
        if s18:
            self.get_logger().info("S18 ON — Cyl2 do khay -> INY -> 200mm")
            self._enter_in(SystemState.S1_INY_200)
            return
        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Retry Cyl2 extend")
            self._cyl2_extend()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S1_WAIT_S18", "Cho S18 ON (Cyl2 do khay). Kich: '18:1'")

    def _s1_iny_200(self):
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_place)
            if not ok:
                self.get_logger().warn("S1 Step10: INY -> 200mm fail — retry")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(2):
                self._enter_in(SystemState.S1_PLACE_DELAY)

    def _s1_place_delay(self):
        """[R-6] Dung _place_delay_start rieng, KHONG dung chung _step_start."""
        if not self._cmd_sent_in:
            self._place_delay_start = time.time()
            self._cmd_sent_in          = True
        if time.time() - self._place_delay_start >= 1.0:
            self._cyl1_retract()
            self._enter_in(SystemState.S1_WAIT_RELEASE)

    def _s1_wait_release(self):
        s15, s16 = self._snap(15, 16)
        if not self._cmd_sent_in:
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in    = True
        if s15 and not s16:
            self.get_logger().info("S15 ON + S16 OFF — Cyl1 nha hoan toan")
            self._enter_in(SystemState.S1_INY_50_FINAL)
            return
        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Retry Cyl1 retract")
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S1_WAIT_REL", f"Cho S15 ON + S16 OFF | S15={s15} S16={s16}")

    def _s1_iny_50_final(self):
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_safe_zone)
            if not ok:
                self.get_logger().warn("S1 Step12: INY -> 50mm fail — retry")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(2):
                self._enter_in(SystemState.S1_INX_HOME)

    def _s1_inx_home(self):
        if not self._iny_safe():
            self._log_once("S1_INX_WAIT_INY", "Cho INY <= 50mm truoc INX ve home")
            return
        if not self._cmd_sent_in:
            ok = self._nb_move(1, self.config.inx_home)
            if not ok:
                self.get_logger().warn("S1 Step13: INX fail — retry")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(1):
                self._enter_in(SystemState.S1_COMPLETE)

    def _s1_complete(self):
        """[FIX-6] Publish new_tray_loaded. Check S18 voi timeout 3s."""
        if not self._cmd_sent_in:
            self.pub_new_tray.publish(Bool(data=True))
            self.get_logger().info("Published: new_tray_loaded = True")
            self._notify('info', 'STATE 1 COMPLETE', 'Cho robot done de thay khay')
            self.stack_row_index  = self._current_row
            self._cmd_sent_in        = True
            self._s13_check_start = 0.0

        if self.operation_mode == 'manual':
            self._enter_in(SystemState.IDLE)
            return

        if not self._input_tray_done:
            self._log_once("S1C_WAIT_ROBOT", "Chờ /robot/done_tray_input")
            return

        if self.sensor(18):
            self.get_logger().info("Input done + S18 ON -> STATE 2")
            self._enter_in(SystemState.S2A_CHECK_INTERLOCK)
        else:
            if self._s13_check_start == 0.0:
                self._s13_check_start = time.time()
                self.get_logger().warn("Robot done nhung S18 OFF — cho 3s (sensor fault?)")
            elapsed = time.time() - self._s13_check_start
            if elapsed >= S13_CHECK_TIMEOUT_S:
                self.get_logger().error(
                    f"S18 OFF sau {S13_CHECK_TIMEOUT_S:.0f}s — tiep tuc STATE 2 (caution)"
                )
                self._notify('warn', 'S18 OFF sau robot_done', 'Kiem tra sensor S18.')
                self._enter_in(SystemState.S2A_CHECK_INTERLOCK)
            else:
                self._log_once("S1C_S18_WAIT",
                               f"Cho S18 ON (con {S13_CHECK_TIMEOUT_S - elapsed:.1f}s)")

    # ══════════════════════════════════════════════════════════════
    # STATE 2: Thay khay Input
    # ══════════════════════════════════════════════════════════════

    def _s2a_check_interlock(self):
        if not self._cmd_sent_in:
            self._pub_cartridge_busy(True)
            self._s6_snapshot       = self.sensor(6)
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
                    self.get_logger().warn("S2 Step1: INY home fail — thu lai vong toi")
                    self._notify('warn', 'INY ve home that bai', 'Dang thu lai...')
                    return
            self._log_once("S2A_WAIT_INY", "Step1: cho INY <= 50mm")

    def _s2a_inx_500(self):
        if not self._iny_safe():
            self._log_once("S2A_INY2", "A2: INY chua safe")
            return
        if not self.sensor(15):
            self._log_once("S2A_S15_ILK", "INTERLOCK S2: S15=OFF -> INX bi khoa")
            self._notify('warn', 'S15 chua ON', 'Cyl1 chua retract — INX bi khoa')
            return
        if not self._cmd_sent_in:
            ok = self._nb_move(1, self.config.inx_target)
            if not ok:
                self._log_once("S2A_A2_FAIL", "S2A A2: INX 500mm fail — retry")
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
            ok = self._nb_move(2, self.config.iny_place)
            if not ok:
                self._log_once("S2A_A3_FAIL", "S2A A3: INY 200mm fail — retry")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(2):
                self.get_logger().info("A3: INY tai 200mm -> Cyl1 EXTEND (ch5)")
                self._cyl1_extend()
                self._enter_in(SystemState.S2A_WAIT_S11)

    def _s2a_wait_s11(self):
        s16, = self._snap(16)
        if not self._cmd_sent_in:
            self._cyl1_extend()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in    = True
        if s16:
            self.get_logger().info("A4: S16 ON — gap khay cu OK")
            self._enter_in(SystemState.S2A_INY_10)
            return
        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Retry Cyl1 extend")
            self._cyl1_extend()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S2A_S16", "A4: Cho S16 ON. Kich: '16:1'")

    def _s2a_iny_10(self):
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_home)
            if not ok:
                self._log_once("S2A_A5_FAIL", "S2A A5: INY 10mm fail — retry")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(2):
                self._enter_in(SystemState.S2A_INX_10)

    def _s2a_inx_10(self):
        """[FIX-3] INX -> inx_output_pos (config), khong hardcode."""
        if not self._iny_safe():
            self._log_once("S2A_INX10_WAIT", "A6: INY chua safe")
            return
        if not self._cmd_sent_in:
            ok = self._nb_move(1, self.config.inx_output_pos)
            if not ok:
                self._log_once("S2A_A6_FAIL", "S2A A6: INX output pos fail — retry")
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
        """
        Jog INY tim vi tri dat khay trong output stack.

        [R-2] Timeout 120s; qua gio -> ERROR.
        [V5-4] Guard INY > home: them _nb_move(iny_home) de servo tu dong ve,
               khong chi log va return (truoc day servo dung yen mai mai).
        """
        if not self._s6_snapshot:
            self.get_logger().info("Step7: S6 OFF -> thang row1 (590mm)")
            self._output_target_pos = self.config.iny_output_stack[1]
            self._output_row        = 0
            self._enter_in(SystemState.S2A_INY_OUTPUT_ROW)
            return

        iny = self._pos(2)
        if iny is None:
            return

        if not self._cmd_sent_in:
            self._step_timeout_in = time.time() + JOG_OUTPUT_TIMEOUT_S
            self._cmd_sent_in     = True
            self.get_logger().info(
                f"Step7: S6=ON — jog tim S4, timeout={JOG_OUTPUT_TIMEOUT_S:.0f}s | "
                f"arm>={self.config.iny_scan_arm_mm}mm"
            )

        # [V5-4] Guard: neu INY chua ve home, ra lenh ve home (khong chi return)
        if iny > self.config.iny_home + 5.0:
            if not self._iny_moving:
                self._nb_move(2, self.config.iny_home)
            self._log_once("S2A_JOG_WAIT_INY",
                           f"Step7: INY ve home ({self.config.iny_home}mm) "
                           f"truoc khi jog | pos={iny:.1f}mm")
            return

        if iny >= self.config.iny_scan_arm_mm:
            self._s4_armed_out = True

        if self._s4_armed_out and self.sensor(4):
            self._stop(2)
            trigger_pos = self._pos(2) or iny
            target_pos, occupied_row, is_full = self._calc_output_target(trigger_pos)

            if is_full:
                self.get_logger().error(
                    f"OUTPUT STACK DAY: S4 trigger {trigger_pos:.1f}mm | "
                    f"occupied~row{occupied_row} | "
                    f"target={target_pos:.1f}mm < min={self.config.output_min_pos_mm}mm"
                )
                self._notify('error', 'Output stack day!',
                             'Khong con cho trong — lay khay ra roi tiep tuc.')
                self._nb_move(2, self.config.iny_home)
                self._nb_move(1, self.config.inx_home)
                self._enter_in(SystemState.ERROR)
                return

            self._output_target_pos = target_pos
            self._output_row        = occupied_row
            actual_tray = trigger_pos + self.config.s4_cross_offset_mm
            self.get_logger().info(
                f"Step7: S4 trigger {trigger_pos:.1f}mm | "
                f"tray~{actual_tray:.0f}mm (row{occupied_row}) | "
                f"dat khay -> {target_pos:.1f}mm (vel={self.config.output_approach_vel})"
            )
            self._enter_in(SystemState.S2A_INY_OUTPUT_ROW)
            return

        if iny >= OUTY_ROW1_LIMIT_MM:
            self._stop(2)
            self.get_logger().info(
                f"Step7: INY den {OUTY_ROW1_LIMIT_MM}mm, S4 khong trigger "
                f"(S6 false alarm hoac stack trong) -> dat row1"
            )
            self._output_target_pos = self.config.iny_output_stack[1]
            self._output_row        = 0
            self._enter_in(SystemState.S2A_INY_OUTPUT_ROW)
            return

        if time.time() > self._step_timeout_in:
            self._stop(2)
            self.get_logger().error(
                f"S2A jog output TIMEOUT ({JOG_OUTPUT_TIMEOUT_S:.0f}s): "
                f"S4 khong trigger tai pos={iny:.1f}mm — dung khan cap"
            )
            self._notify('error', 'Jog output timeout!',
                         f'S4 khong trigger sau {JOG_OUTPUT_TIMEOUT_S:.0f}s. '
                         f'Kiem tra sensor S4 va co khi output stack.')
            self._nb_move(2, self.config.iny_home)
            self._nb_move(1, self.config.inx_home)
            self._enter_in(SystemState.ERROR)
            return

        # Tiep tuc jog moi tick
        self._jog(2, INY_JOG_VEL)
        remain = self._step_timeout_in - time.time()
        arm_mm = self.config.iny_scan_arm_mm
        self._log_once("S2A_JOG_OUT",
                       f"Step7: INY jog tim S4 (S6=ON, pos={iny:.0f}mm, "
                       f"arm={'YES' if self._s4_armed_out else f'NO (<{arm_mm:.0f}mm)'}, "
                       f"con {remain:.0f}s)")

    def _s2a_iny_output_row(self):
        target = self._output_target_pos
        if target <= 0:
            self.get_logger().error("_output_target_pos chua duoc set — skip")
            self._enter_in(SystemState.S2A_WAIT_S10)
            return

        vel = self.config.output_approach_vel if self._s6_snapshot else 30

        if not self._cmd_sent_in:
            self.get_logger().info(
                f"A8: INY -> {target:.1f}mm (vel={vel}) "
                f"| S6={'ON->cham' if self._s6_snapshot else 'OFF->nhanh'} "
                f"| occupied_row={self._output_row if self._output_row else 'N/A'}"
            )
            ok = self._nb_move(2, target, vel=vel)
            if not ok:
                self.get_logger().warn(f"S2A A8: INY {target:.1f}mm fail — retry")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(2):
                self.get_logger().info(f"A8: INY tai {target:.1f}mm -> Cyl1 RETRACT")
                self._cyl1_retract()
                self._enter_in(SystemState.S2A_WAIT_S10)

    def _s2a_wait_s10(self):
        s15, = self._snap(15)
        if not self._cmd_sent_in:
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in    = True
        if s15:
            self.get_logger().info("A9: S15 ON — da tha khay")
            self._enter_in(SystemState.S2A_INY_10_FINAL)
            return
        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Retry Cyl1 retract")
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S2A_S15", "A9: Cho S15 ON. Kich: '15:1'")

    def _s2a_iny_10_final(self):
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_home)
            if not ok:
                self._log_once("S2A_A10_FAIL", "S2A A10: INY 10mm fail — retry")
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
            ok = self._nb_move(1, self.config.inx_home)
            if not ok:
                self._log_once("S2A_A11_FAIL", "S2A A11: INX 20mm fail — retry")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(1):
                self.get_logger().info("A11: INX tai 20mm -> Rut Cyl2")
                self._cyl2_retract()
                self._enter_in(SystemState.S2A_WAIT_CYL2_RETRACT)

    def _s2a_wait_cyl2_retract(self):
        s17, s18 = self._snap(17, 18)
        if not self._cmd_sent_in:
            self._cyl2_retract()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in    = True
        if s17 and not s18:
            self.get_logger().info("A11b: S17 ON + S18 OFF — Cyl2 da retract -> COMPLETE")
            self._enter_in(SystemState.S2A_COMPLETE)
            return
        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Retry Cyl2 retract")
            self._cyl2_retract()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S2A_CYL2", f"A11b: Cho Cyl2 retract | S17={s17} S18={s18}")

    def _s2a_complete(self):
        if not self._cmd_sent_in:
            self._input_tray_done = False
            self._pub_cartridge_busy(False)
            self._notify('info', 'STATE 2 COMPLETE', 'Da thay khay')
            self._cmd_sent_in = True

        if self.operation_mode == 'manual':
            self.get_logger().info("[MANUAL] State 2 done -> IDLE-IN")
            self._guide_logged.discard("IDLE_MANUAL")
            self._enter_in(SystemState.IDLE)
            return

        if self._can_start_s1():
            self.get_logger().info("AUTO State2 → State1")
            self._enter_in(SystemState.S1_CONFIRM_SAFE)
        else:
            s1s2s3 = self.sensor(1) or self.sensor(2) or self.sensor(3)
            if not s1s2s3:
                self._notify('warn', 'Het khay tren bang tai', 'Nap khay — tu chay lai')
            elif not self.sensor(17):
                self._notify('info', 'Cho S17 ON', 'Se tu vao State 1 khi S17 ON')
            self._guide_logged.discard("IDLE_IN_NO_TRAY")
            self._guide_logged.discard("IDLE_IN_NO_PLACE")
            self._enter_in(SystemState.IDLE)

    # ══════════════════════════════════════════════════════════════
    # STATE 3 — Cấp khay thành phẩm (Servo 3 / Platform)
    # ══════════════════════════════════════════════════════════════

    def _s3_check_outxy_safe(self):
        """
        OutX va OutY ve safe zone truoc khi Servo3 di chuyen.
        [V5-6] FIX 1: Dung cfg.outx_home thay vi outy_safe_zone de check OutX.
        [V5-6] FIX 2: Doi arrived(4) va arrived(5) truoc khi chuyen state.
        """
        if not self._cmd_sent_out:
            self._pub_cartridge_busy(True)
        cfg = self.config
        ox = self._pos(4)
        oy = self._pos(5)
        # Dieu kien "da ve home": ox <= outx_home + 5mm va oy <= outy_target1 + 5mm
        ox_safe = ox is None or ox <= cfg.outx_home + 5.0
        oy_safe = oy is None or oy <= cfg.outy_target1 + 5.0
        if ox_safe and oy_safe:
            self._enter_out(SystemState.S3_SERVO3_TARGET1)
        else:
            if not self._cmd_sent_out:
                if not ox_safe:
                    self._nb_move(4, cfg.outx_home)
                if not oy_safe:
                    self._nb_move(5, cfg.outy_target1)
                self._cmd_sent_out     = True
                self._step_timeout_out = time.time() + self.config.move_timeout
                self._step_start_out   = time.time()
            else:
                if time.time() > self._step_timeout_out:
                    self.get_logger().warn("S3_CHECK_OUTXY: timeout doi OutX/OutY ve home — tiep tuc")
                    self._enter_out(SystemState.S3_SERVO3_TARGET1)
                elif self._arrived(4) and self._arrived(5):
                    self._enter_out(SystemState.S3_SERVO3_TARGET1)
                else:
                    self._log_once("S3_SAFE", "Cho OutX/OutY ve home truoc khi Servo3 chay")

    def _s3_servo3_target1(self):
        """
        Servo 3 ve vi tri cho cap khay (target1 = 10mm).
        [V5-5] FIX: Them timeout de khong treo neu servo fault.
        """
        cfg = self.config
        if not self._cmd_sent_out:
            ok = self._nb_move(3, cfg.servo3_target1)
            if not ok:
                self._log_once("S3_T1_FAIL", "S3 target1 fail — retry")
                return
            self._cmd_sent_out     = True
            self._step_timeout_out = time.time() + self.config.move_timeout
            self._step_start_out   = time.time()
        else:
            if time.time() > self._step_timeout_out:
                self._error(f"S3_SERVO3_TARGET1 timeout ({self.config.move_timeout:.0f}s)")
            elif self._arrived(3):
                self._enter_out(SystemState.S3_CHECK_S7)

    def _s3_check_s7(self):
        if self.sensor(7):
            self.get_logger().info("[S3] S7 ON — co khay tren Platform")
            self._enter_out(SystemState.S3_SERVO3_FEED)
        else:
            self.get_logger().info("[S3] S7 OFF — Cho khay duoc cap")
            self._enter_out(SystemState.S3_WAIT_S7)

    def _s3_wait_s7(self):
        if self.sensor(7):
            self._notify('info', 'Da phat hien khay', 'S7 ON — xac nhan tren GUI')
            self._gui_confirmed = False
            self._enter_out(SystemState.S3_WAIT_GUI_CONFIRM)
        else:
            self._log_once("S3_WAIT_S7", "Cho S7 ON — cap khay len Platform")

    def _s3_wait_gui_confirm(self):
        if self._gui_confirmed:
            self._gui_confirmed = False
            self._enter_out(SystemState.S3_SERVO3_FEED)
        else:
            self._log_once("S3_WAIT_GUI", "S7 ON — Xac nhan tren GUI de cap khay")

    def _s3_servo3_feed(self):
        """
        Servo 3 day khay den vi tri robot gap (target2 = 400mm).
        [V5-5] FIX: Them timeout de khong treo neu servo fault.
        """
        cfg = self.config
        if not self._cmd_sent_out:
            ok = self._nb_move(3, cfg.servo3_target2, vel=int(cfg.servo3_feed_velocity))
            if not ok:
                self._log_once("S3_FEED_FAIL", "S3 feed fail — retry")
                return
            self._cmd_sent_out     = True
            self._step_timeout_out = time.time() + self.config.move_timeout
            self._step_start_out   = time.time()
        else:
            if time.time() > self._step_timeout_out:
                self._error(f"S3_SERVO3_FEED timeout ({self.config.move_timeout:.0f}s)")
            elif self._arrived(3):
                self._enter_out(SystemState.S3_WAIT_S8)

    def _s3_wait_s8(self):
        cfg = self.config
        if self.sensor(8):
            self.get_logger().info("[S3] S8 ON — Khay vao vi tri robot gap")
            self._enter_out(SystemState.S3_COMPLETE)
            return
        if not self._step_start_out:
            self._step_start_out = time.time()
        if self.sensor(7) and not self.sensor(8):
            if time.time() - self._step_start_out > CYLINDER_TIMEOUT_S:
                self._notify('warn', 'S8 khong ON', 'Khay ket — Servo3 quay ve target1')
                self._cmd_sent_out = False
                self._enter_out(SystemState.S3_SERVO3_TARGET1)
                return
        self._log_once("S3_WAIT_S8", "Cho S8 ON — khay vao vi tri robot gap")

    def _s3_complete(self):
        """State3 xong — khay thành phẩm mới đã được cấp → báo robot có slot mới."""
        self._pub_cartridge_busy(False)
        self.pub_newtray_output.publish(Bool(data=True))
        self._notify('info', 'State 3 done', 'Cap khay thanh pham thanh cong')
        self.get_logger().info("[S3] COMPLETE — pub new_trayoutput_loaded")
        self._enter_out(SystemState.IDLE)

    # ══════════════════════════════════════════════════════════════
    # STATE 4 — Thay khay output (OutX/OutY + Cylinder 3)
    # ══════════════════════════════════════════════════════════════

    def _s4_check_outy_safe(self):
        """OutY phai < outy_safe_zone truoc khi OutX vao."""
        if not self._cmd_sent_out:
            self._pub_cartridge_busy(True)
        cfg = self.config
        oy = self._pos(5)
        if oy is not None and oy <= cfg.outy_safe_zone + 5:
            self._enter_out(SystemState.S4_OUTX_TARGET2)
        else:
            if not self._cmd_sent_out:
                ok = self._nb_move(5, cfg.outy_target1)
                if not ok:
                    self._log_once("S4_SAFE_FAIL", "S4 outy home fail — retry")
                    return
                self._cmd_sent_out     = True
                self._step_timeout_out = time.time() + self.config.move_timeout
            else:
                if time.time() > self._step_timeout_out:
                    self._cmd_sent_out = False
                elif self._arrived(5):
                    self._enter_out(SystemState.S4_OUTX_TARGET2)
            self._log_once("S4_SAFE", "Cho OutY ve safe zone truoc khi OutX vao")

    def _s4_outx_target2(self):
        """
        OutX vao vi tri lay khay thanh pham (target2 = 400mm).
        [V5-5] FIX: Them timeout.
        """
        cfg = self.config
        if not self._cmd_sent_out:
            ok = self._nb_move(4, cfg.outx_target2)
            if not ok:
                self._log_once("S4_OX2_FAIL", "S4 outx_target2 fail — retry")
                return
            self._cmd_sent_out     = True
            self._step_timeout_out = time.time() + self.config.move_timeout
            self._step_start_out   = time.time()
        else:
            if time.time() > self._step_timeout_out:
                self._error(f"S4_OUTX_TARGET2 timeout ({self.config.move_timeout:.0f}s)")
            elif self._arrived(4):
                self._enter_out(SystemState.S4_OUTY_PICK)

    def _s4_outy_pick(self):
        """
        OutY ha xuong vi tri gap khay (outy_pick_pos = 100mm).
        [V5-5] FIX: Them timeout.
        """
        cfg = self.config
        if not self._cmd_sent_out:
            ok = self._nb_move(5, cfg.outy_pick_pos)
            if not ok:
                self._log_once("S4_PICK_FAIL", "S4 outy_pick fail — retry")
                return
            self._cmd_sent_out     = True
            self._step_timeout_out = time.time() + self.config.move_timeout
            self._step_start_out   = time.time()
        else:
            if time.time() > self._step_timeout_out:
                self._error(f"S4_OUTY_PICK timeout ({self.config.move_timeout:.0f}s)")
            elif self._arrived(5):
                self._enter_out(SystemState.S4_CYL3_EXTEND)

    def _s4_cyl3_extend(self):
        """Kich Cylinder 3 gap khay, cho S20 ON."""
        if not self._cmd_sent_out:
            self._cyl3_extend()
            self._cmd_sent_out = True
            self._step_start_out = time.time()
        if self.sensor(20):
            self.get_logger().info("[S4] S20 ON — Cyl3 da extend, da gap khay")
            self._enter_out(SystemState.S4_OUTY_TARGET1)
        elif time.time() - self._step_start_out > CYLINDER_TIMEOUT_S:
            self._error("[S4] Timeout: Cyl3 extend — S20 khong ON")

    def _s4_outy_target1(self):
        """
        OutY nang len 10mm (giu khay).
        [V5-5] FIX: Them timeout.
        """
        cfg = self.config
        if not self._cmd_sent_out:
            ok = self._nb_move(5, cfg.outy_target1)
            if not ok:
                self._log_once("S4_OY1_FAIL", "S4 outy_target1 fail — retry")
                return
            self._cmd_sent_out     = True
            self._step_timeout_out = time.time() + self.config.move_timeout
            self._step_start_out   = time.time()
        else:
            if time.time() > self._step_timeout_out:
                self._error(f"S4_OUTY_TARGET1 timeout ({self.config.move_timeout:.0f}s)")
            elif self._arrived(5):
                self._enter_out(SystemState.S4_OUTX_TARGET3)

    def _s4_outx_target3(self):
        """
        OutX ve vi tri dat khay (target3 = 20mm).
        [V5-5] FIX: Them timeout.
        """
        cfg = self.config
        if not self._cmd_sent_out:
            ok = self._nb_move(4, cfg.outx_target3)
            if not ok:
                self._log_once("S4_OX3_FAIL", "S4 outx_target3 fail — retry")
                return
            self._cmd_sent_out     = True
            self._step_timeout_out = time.time() + self.config.move_timeout
            self._step_start_out   = time.time()
        else:
            if self.sensor(7) and not self._s3_pending:
                self._s3_pending = True
                self.get_logger().info("[S4] S7 ON trong S4 — S3 se chay sau")
            if time.time() > self._step_timeout_out:
                self._error(f"S4_OUTX_TARGET3 timeout ({self.config.move_timeout:.0f}s)")
            elif self._arrived(4):
                self._enter_out(SystemState.S4_CHECK_S9)

    def _s4_check_s9(self):
        if self.sensor(9):
            self.get_logger().info("[S4] S9 ON — Di chuyen den row1")
            self._enter_out(SystemState.S4_OUTY_ROW1)
        else:
            self.get_logger().info("[S4] S9 OFF — Jog tim S10")
            self._outy_jog_start = time.time()
            self._enter_out(SystemState.S4_OUTY_SCAN_S10)

    def _s4_outy_row1(self):
        """
        S9 ON -> OutY den vi tri row1.
        [V5-5] FIX: Them timeout.
        """
        cfg = self.config
        if not self._cmd_sent_out:
            ok = self._nb_move(5, cfg.outy_row1_pos)
            if not ok:
                self._log_once("S4_ROW1_FAIL", "S4 outy_row1 fail — retry")
                return
            self._cmd_sent_out     = True
            self._step_timeout_out = time.time() + self.config.move_timeout
            self._step_start_out   = time.time()
        else:
            if time.time() > self._step_timeout_out:
                self._error(f"S4_OUTY_ROW1 timeout ({self.config.move_timeout:.0f}s)")
            elif self._arrived(5):
                self._enter_out(SystemState.S4_CYL3_RETRACT)

    def _s4_outy_scan_s10(self):
        """
        S9 OFF -> jog OutY cham trong gioi han 680mm den khi S10 ON.
        [V5-3] BUG FIX: _jog() goi MOI TICK (ngoai if-block), khong chi 1 lan.
               jog_task(duration=0) khong tu duy tri — servo dung sau 1 tick neu
               khong goi lai. So sanh voi _s2a_iny_jog_output() la pattern dung.
        [V5-5] FIX: Dung _outy_jog_start (da set trong _s4_check_s9) lam timeout.
        """
        cfg = self.config
        oy = self._pos(5)

        if self.sensor(10):
            self.get_logger().info("[S4] S10 ON — Tim duoc vi tri dat khay")
            self._stop(5)
            self._outy_jog_pos = oy or 0.0
            self._enter_out(SystemState.S4_OUTY_DROP)
            return

        if oy is not None and oy >= cfg.outy_row_limit:
            self._stop(5)
            self._notify('warn', 'S10 khong ON', 'Da het gioi han — kiem tra stack output')
            self._error("[S4] OutY dat gioi han 680mm ma S10 chua ON")
            return

        # [V5-5] Timeout bao ve: dung _outy_jog_start da set truoc do
        if self._outy_jog_start > 0 and time.time() - self._outy_jog_start > JOG_OUTPUT_TIMEOUT_S:
            self._stop(5)
            self._error(f"[S4] S4_OUTY_SCAN_S10 timeout ({JOG_OUTPUT_TIMEOUT_S:.0f}s) — S10 khong ON")
            return

        # [V5-3] Jog MOI TICK — khong gate sau if-not-cmd_sent
        self._jog(5, int(cfg.outy_slow_vel))
        self._log_once("S4_SCAN", f"Jog OutY tim S10 (gioi han {cfg.outy_row_limit}mm)")

    def _s4_outy_drop(self):
        """Sau S10 ON: them outy_drop_extra mm voi toc do cham roi tha khay."""
        cfg = self.config
        if not self._cmd_sent_out:
            extra_target = (self._outy_jog_pos or 0.0) + cfg.outy_drop_extra
            extra_target = min(extra_target, cfg.outy_row_limit + 30.0)
            self._nb_move(5, extra_target, vel=int(cfg.outy_slow_vel))
            self._cmd_sent_out = True
            self._step_start_out = time.time()
        if self._arrived(5):
            self._enter_out(SystemState.S4_CYL3_RETRACT)
        elif time.time() - self._step_start_out > 5.0:
            self._enter_out(SystemState.S4_CYL3_RETRACT)

    def _s4_cyl3_retract_state(self):
        """Retract Cylinder 3, cho S19 ON va S20 OFF."""
        if not self._cmd_sent_out:
            self._cyl3_retract()
            self._cmd_sent_out = True
            self._step_start_out = time.time()
        if self.sensor(19) and not self.sensor(20):
            self.get_logger().info("[S4] S19 ON/S20 OFF — Cyl3 retracted, khay da dat")
            self._enter_out(SystemState.S4_OUTY_OUTX_HOME)
        elif time.time() - self._step_start_out > CYLINDER_TIMEOUT_S:
            self._error("[S4] Timeout: Cyl3 retract — S19 khong ON hoac S20 con ON")

    def _s4_outy_outx_home(self):
        """
        OutY va OutX ve home (10mm).
        [V5-5] FIX: Them timeout cho ca hai truc.
        """
        cfg = self.config
        if not self._cmd_sent_out:
            ok4 = self._nb_move(4, cfg.outx_home)
            ok5 = self._nb_move(5, cfg.outy_target1)
            if not ok4 or not ok5:
                self._log_once("S4_HOME_FAIL", "S4 outy/outx home fail — retry")
                return
            self._cmd_sent_out     = True
            self._step_timeout_out = time.time() + self.config.move_timeout
            self._step_start_out   = time.time()
        else:
            if time.time() > self._step_timeout_out:
                self._error(f"S4_OUTY_OUTX_HOME timeout ({self.config.move_timeout:.0f}s)")
            elif self._arrived(5) and self._arrived(4):
                self._enter_out(SystemState.S4_COMPLETE)

    def _s4_complete(self):
        self._pub_cartridge_busy(False)
        """State4 xong — khay output cũ đã lấy ra.
        Nếu điều kiện S3 thỏa (Camera AI: khay vừa cấp đã full ngay) → trigger S3 ngay.
        """
        self._notify('info', 'State 4 done', 'Thay khay output thanh cong')
        self.get_logger().info("[S4] COMPLETE")
        self._s4_trigger = False
        if self._can_start_s3():
            self.get_logger().info("[S4→S3] S7 ON + S8 OFF → cấp khay output mới ngay")
            self._enter_out(SystemState.S3_CHECK_OUTXY_SAFE)
        else:
            self._enter_out(SystemState.IDLE)


# ─── Main ─────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    system = None

    def _sig(signum, frame):
        if system:
            try: system.destroy_node()
            except Exception: pass
        try: rclpy.shutdown()
        except Exception: pass
        os._exit(0)

    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT,  _sig)

    try:
        config = CartridgeConfig()
        for candidate in [
            os.path.join(os.path.dirname(__file__), '..', 'config', 'cartridge_config.yaml'),
            os.path.expanduser('~/ros2_ws/src/system_feed_cartridge/config/cartridge_config.yaml'),
        ]:
            if os.path.exists(candidate):
                config.load_from_file(candidate)
                break

        system = CartridgeSystem(config)
        print("=" * 58)
        print("  Cartridge System v6 (S1 INY scan: position-mode only + row1 fallback)")
        print("  STATE 1 : Cap khay Input   (bang tai -> robot)")
        print("  STATE 2 : Thay khay Input  (robot done -> output stack)")
        print("=" * 58)
        rclpy.spin(system)

    except KeyboardInterrupt:
        pass
    finally:
        if system:
            try: system.destroy_node()
            except Exception: pass
        try: rclpy.shutdown()
        except Exception: pass
        os._exit(0)


if __name__ == '__main__':
    main()