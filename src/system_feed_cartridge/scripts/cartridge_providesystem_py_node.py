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
  [V7-5] _cb_gui_confirm('S16_OK'): không set _state1_enabled trong MANUAL
         vì S1 cần operator nhấn tay; chỉ chạy S2A rồi về IDLE.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
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
INY_JOG_VEL          = 40
OUTY_ROW1_LIMIT_MM   = 590.0
S13_CHECK_TIMEOUT_S  = 3.0
JOG_OUTPUT_TIMEOUT_S = 120.0


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
    S1_WAIT_GUI_CONFIRM = "s1_wait_gui_confirm"
    S1_RETRY_INX_500    = "s1_retry_inx_500"
    S1_RETRY_JOG        = "s1_retry_jog"
    S1_CYL1_EXTEND      = "s1_cyl1_extend"
    S1_INY_50           = "s1_iny_50"
    S1_INY_200          = "s1_iny_200"
    S1_WAIT_RELEASE     = "s1_wait_release"
    S1_WAIT_S11         = "s1_wait_s11"
    S1_INY_10_FINAL     = "s1_iny_10_final"
    S1_INX_10           = "s1_inx_10"
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
    S2A_WAIT_S15          = "s2a_wait_s15"
    S2A_INY_10_FINAL      = "s2a_iny_10_final"
    S2A_INX_20            = "s2a_inx_20"
    S2A_COMPLETE          = "s2a_complete"

    # STATE 3
    S3_CHECK_OUTXY_SAFE  = "s3_check_outxy_safe"
    S3_SERVO3_TARGET1    = "s3_servo3_target1"
    S3_CHECK_S7          = "s3_check_s7"
    S3_WAIT_S7           = "s3_wait_s7"
    S3_WAIT_GUI_CONFIRM  = "s3_wait_gui_s7"
    S3_SERVO3_FEED       = "s3_servo3_feed"
    S3_WAIT_S8           = "s3_wait_s8"
    S3_COMPLETE          = "s3_complete"

    # STATE 4
    S4_CHECK_OUTY_SAFE   = "s4_check_outy_safe"
    S4_OUTX_TARGET2      = "s4_outx_target2"
    S4_OUTY_PICK         = "s4_outy_pick"
    S4_CYL2_EXTEND       = "s4_cyl2_extend"
    S4_OUTY_TARGET1      = "s4_outy_target1"
    S4_OUTX_TARGET3      = "s4_outx_target3"
    S4_CHECK_S9          = "s4_check_s9"
    S4_OUTY_ROW1         = "s4_outy_row1"
    S4_OUTY_SCAN_S10     = "s4_outy_scan_s10"
    S4_OUTY_DROP         = "s4_outy_drop"
    S4_CYL2_RETRACT      = "s4_cyl2_retract"
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
        self.cylinder2_extend_channel  = 11
        self.cylinder2_retract_channel = 10

        self.iny_input_zones   = {}
        self.iny_output_zones  = {}
        self.outy_output_zones = {}

        self._config_file: Optional[str] = None

        if config_file and os.path.exists(config_file):
            self.load_from_file(config_file)

    def load_from_file(self, path: str):
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
            for k, v in data.items():
                if isinstance(v, dict):
                    v = {int(kk) if str(kk).isdigit() else kk: vv for kk, vv in v.items()}
                setattr(self, k, v)
            self._config_file = path
            print(f"Config loaded: {path}")
        except Exception as e:
            print(f"Config load error: {e}")

    def save_to_file(self):
        if not self._config_file:
            return
        
        try:
            import yaml
            import os
            data = {}
            if os.path.exists(self._config_file):
                with open(self._config_file, 'r') as f:
                    data = yaml.safe_load(f) or {}

            # Cập nhật data với TẤT CẢ các public attributes của config
            for k, v in self.__dict__.items():
                if not k.startswith('_') and not callable(v):
                    if isinstance(v, dict):
                        v = {int(kk) if str(kk).isdigit() else kk: float(vv) if isinstance(vv, (int, float)) else vv for kk, vv in v.items()}
                    data[k] = v

            with open(self._config_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
            print(f"Config saved safely to: {self._config_file}")
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

        # ── OUTPUT side state machine ────────────────────────────
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
        self.state           = SystemState.IDLE
        self.operation_mode  = 'auto'   # 'auto' | 'manual'
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
        
        qos_latching = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_config_data    = self.create_publisher(String, '/providesystem/config_data', qos_latching)

        # ROS Subscribers
        self.create_subscription(Bool,   '/system/start_button',             self._cb_start,              qos)
        self.create_subscription(Bool,   '/system/stop_button',              self._cb_stop,               qos)
        self.create_subscription(Bool,   '/system/pause_button',             self._cb_pause,              qos)
        self.create_subscription(Bool,   '/robot/motion_busy',               self._cb_motion_busy,          qos)
        self.create_subscription(Bool,   '/robot/done_tray_output',          self._cb_done_tray_output,     qos)
        self.create_subscription(Bool,   '/robot/done_tray_input',           self._cb_done_tray_input,      qos)
        self.create_subscription(Bool,   '/robot/last_batch_complete',       self._cb_last_batch_complete,  qos)
        self.create_subscription(String, '/providesystem/gui_confirm',       self._cb_gui_confirm,        qos)
        self.create_subscription(String, '/providesystem/jog_cmd',           self._cb_jog,                qos)
        self.create_subscription(String, '/providesystem/sim_sensor',        self._cb_sim,                qos)
        self.create_subscription(String, '/providesystem/set_operation_mode',self._cb_mode,               qos)
        self.create_subscription(String, '/providesystem/goto_state',        self._cb_goto_state,         qos)
        self.create_subscription(String, '/providesystem/update_config',     self._cb_update_config,      qos)
        self.create_subscription(String, '/providesystem/get_config',        self._cb_get_config,         qos)

        self._connect_hardware()

        self.create_timer(0.05, self._control_loop)
        self.create_timer(0.5,  self._publish_positions)
        self.create_timer(5.0,  self._watchdog)

        self.get_logger().info("CartridgeSystem node started")

    # ══════════════════════════════════════════════════════════════
    # Hardware
    # ══════════════════════════════════════════════════════════════

    def _connect_hardware(self):
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
        """Luôn đọc sensor phần cứng thực (IO module)."""
        with self._io_bg_lock:
            cache = self._io_sensor_cache
            ready = self._io_ready
        if not ready or len(cache) < sid:
            return False
        return bool(cache[sid - 1])

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
            cache = self._io_sensor_cache
            ready = self._io_ready
        results = []
        for sid in sids:
            try:
                raw = bool(cache[sid - 1]) if ready and len(cache) >= sid else False
            except Exception:
                raw = False
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
            self._ensure_ready(mot, servo_id)
            with self._servo_lock:
                mot.velocity_task(int(vel_mm_s), duration=0.0)
        except Exception as e:
            self.get_logger().error(f"S{servo_id} jog error: {e}")

    def _ensure_ready(self, mot, servo_id: int, timeout: float = 3.0):
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

    def _set_do(self, channel: int, state: bool) -> bool:
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
        self.get_logger().info("Cyl1 EXTEND (ch5)")
        if self.io_module:
            try:
                self._set_do(self.config.cylinder1_retract_channel, False)
                self._set_do(self.config.cylinder1_extend_channel, True)
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl1 extend IO error: {e}")
                return False
        return False

    def _cyl1_retract(self) -> bool:
        self.get_logger().info("Cyl1 RETRACT (ch4)")
        if self.io_module:
            try:
                self._set_do(self.config.cylinder1_extend_channel, False)
                self._set_do(self.config.cylinder1_retract_channel, True)
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl1 retract IO error: {e}")
                return False
        return False

    def _cyl2_extend(self) -> bool:
        if self._io_ready and self.io_module:
            try:
                self._set_do(self.config.cylinder2_retract_channel, False)
                self._set_do(self.config.cylinder2_extend_channel, True)
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl2 extend error: {e}")
                return False
        return False

    def _cyl2_retract(self) -> bool:
        if self._io_ready and self.io_module:
            try:
                self._set_do(self.config.cylinder2_extend_channel, False)
                self._set_do(self.config.cylinder2_retract_channel, True)
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl2 retract error: {e}")
                return False
        return False

    # ── Helpers ──────────────────────────────────────────────────

    def _enter(self, next_state: SystemState):
        self.get_logger().info(f"[GLOBAL] -> {next_state.name}")
        if next_state != self.state:
            self._guide_logged.clear()
        self.state = next_state

    def _enter_in(self, next_state: SystemState):
        self.get_logger().info(f"[IN] -> {next_state.name}")
        self.state_in        = next_state
        self._cmd_sent_in    = False
        self._step_start_in  = 0.0
        self._step_timeout_in = 0.0

    def _enter_out(self, next_state: SystemState):
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

    def _zone_to_row(self, trigger_pos: float, zone_table: dict) -> int:
        for row, (lo, hi) in zone_table.items():
            if lo <= trigger_pos <= hi:
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
        p = self._pos(2)
        return p is not None and p <= self.config.iny_safe_zone

    def _can_start_s1(self) -> bool:
        """Điều kiện để bắt đầu STATE 1 (sensor theo mode hiện tại)."""
        if not self.zero_offset or self._motion_busy:
            return False
        has_tray  = self.sensor(1) or self.sensor(2) or self.sensor(3)
        place_ok  = not self.sensor(11)
        s16_clear = not self.sensor(16)
        return has_tray and place_ok and s16_clear

    def _can_start_s2a(self) -> bool:
        return (bool(self.zero_offset) and self._input_tray_done
                and self.sensor(11) and not self._motion_busy)

    def _can_start_s3(self) -> bool:
        if self._motion_busy:
            return False
        return (bool(self.zero_offset)
                and self.sensor(7)
                and not self.sensor(8))

    def _can_start_s4(self) -> bool:
        return bool(self.zero_offset) and self._s4_trigger and not self._motion_busy

    def _pub_cartridge_busy(self, busy: bool):
        self.pub_busy_cartridge.publish(Bool(data=busy))

    def _cb_motion_busy(self, msg: Bool):
        self._motion_busy = msg.data

    def _cb_done_tray_output(self, msg: Bool):
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

    def _cb_last_batch_complete(self, msg: Bool):
        if msg.data:
            self._s4_trigger = True
            self.get_logger().info('[LAST_BATCH] Trigger State4 thay khay output')
            self._notify('info', 'Last batch complete', 'Thay khay output')

    def _outy_safe(self) -> bool:
        p = self._pos(5)
        return p is not None and p <= self.config.outy_safe_zone

    # ══════════════════════════════════════════════════════════════
    # Homing
    # ══════════════════════════════════════════════════════════════

    def _home_all(self) -> bool:
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
                    done = False
                if done:
                    break
                time.sleep(0.1)
            else:
                for sid, mot in active:
                    if self._servo_lock.acquire(timeout=1.0):
                        try:
                            if not mot.referenced():
                                self.get_logger().warn(f"{NAMES.get(sid)} timeout — assume home")
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
            # MANUAL: không tự homing
            self._system_running = True
            self._state1_enabled = True
            self._enter(SystemState.IDLE)
            self._enter_in(SystemState.IDLE)
            self._enter_out(SystemState.IDLE)
            self._notify('info', 'START (MANUAL)', 'Bật chạy theo cảm biến ảo (Sim)')
        else:
            self.zero_offset.clear()
            self._state1_enabled = True
            self._system_running = False
            self._enter(SystemState.HOMING)
            self._notify('info', 'START — HOMING', '')

    def _cb_stop(self, msg: Bool):
        if not msg.data:
            return
        for sid in list(self.servos):
            self._stop(sid)
        self._enter(SystemState.IDLE)
        self._enter_in(SystemState.IDLE)
        self._enter_out(SystemState.IDLE)
        self._system_paused  = False
        self._system_running = False
        self._input_tray_done     = False
        self._state1_enabled = False
        self._s4_trigger = False
        # Tự động chuyển sang MANUAL khi STOP → cho phép JOG ngay
        self.operation_mode = 'manual'
        self.get_logger().info('[STOP] Auto-switch to MANUAL mode — JOG sẵn sàng')
        self._notify('warn', 'STOP', 'Dừng hệ thống — Đã chuyển MANUAL, có thể JOG')

    def _cb_pause(self, msg: Bool):
        if not msg.data:
            return
        self._system_paused = True
        self._notify('warn', 'PAUSE', 'Nhan RESUME de tiep tuc')

    def _cb_gui_confirm(self, msg: String):
        data = msg.data.strip()
        if data == 'S11_OK':
            self.get_logger().info('[S11] OK → chạy S2A (lấy khay ra)')
            self._notify('info', 'S11: Thực hiện State2', 'Lấy khay ra')
            self._input_tray_done = False
            # [V7-5] MANUAL: KHÔNG set _state1_enabled — S1 cần nhấn tay sau S2 xong
            if self.operation_mode == 'auto':
                self._state1_enabled = True
            self._enter_in(SystemState.S2A_CHECK_INTERLOCK)
        elif data == 'S11_NO':
            self.get_logger().info('[S11] NO → IDLE')
            self._notify('warn', 'S11: Dừng', 'Nhấn START lại khi sẵn sàng')
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
        if old != requested:
            self._guide_logged.clear()
            self._sim_sensors.clear()  # Reset sim khi đổi mode
            self._notify('info', f'Mode: {requested.upper()}', 'Hệ thống tự động chạy theo tín hiệu cảm biến.')

    def _cb_jog(self, msg: String):
        if self.operation_mode != 'manual':
            self._notify('warn', 'JOG bi khoa', 'Chuyen MANUAL mode de JOG')
            return
        if (self.state not in (SystemState.IDLE, SystemState.ERROR)
                or self.state_in != SystemState.IDLE
                or self.state_out != SystemState.IDLE):
            self._notify('warn', 'JOG bi khoa', 'Dang chay — nhan STOP truoc')
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
        """
        [V7-1] Sim sensor chỉ hoạt động trong MANUAL mode.
        AUTO mode → reject.
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
                for i in range(1, 20):
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
            self._enter_in(SystemState.IDLE)
            self._enter_out(SystemState.IDLE)
            self._system_running = False
            self.get_logger().info("Got IDLE command, fully reset system states.")
            return

        # ── STATE 1: chỉ cho MANUAL, operator nhấn tay ──────────
        if cmd in ('STATE1', 'STATE_1', 'S1'):
            if self.operation_mode != 'manual':
                self._notify('warn', 'STATE1 manual only',
                             'Trong AUTO mode hệ thống tự trigger STATE1')
                return
            if self.state_in not in (SystemState.IDLE,):
                self._notify('warn', 'Không thể vào STATE 1',
                             f'{self.state_in.name} đang chạy — STOP trước')
                return
            if not self.zero_offset:
                self._notify('warn', 'Chua home', 'Nhấn HOMING trước')
                return
            if self._motion_busy:
                self._notify('warn', 'Robot đang bận', 'Chờ robot idle rồi thử lại')
                return
            # Kiểm tra điều kiện cảm biến sim
            if not self._can_start_s1():
                reasons = []
                if not (self.sensor(1) or self.sensor(2) or self.sensor(3)):
                    reasons.append('S1/S2/S3 OFF (sim chưa set)')
                if self.sensor(11):
                    reasons.append('S11 ON (vị trí cấp có khay)')
                if self.sensor(16):
                    reasons.append('S16 ON (còn khay tại extract)')
                self._notify('warn', 'STATE1: Điều kiện chưa đủ',
                             ' | '.join(reasons) or 'Kiểm tra sim sensor')
                return
            self._state1_enabled = True
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
            if not self.sensor(11):
                self._notify('warn', 'S11 OFF (sim)',
                             'Sim S11=1 trước khi vào State2')
                return
            self._input_tray_done = False
            self._notify('info', 'STATE 2A (manual)', 'S11 ON (sim)')
            self._enter_in(SystemState.S2A_CHECK_INTERLOCK)
            return

        # ── STATE 3/4: giữ nguyên (cả AUTO lẫn MANUAL) ──────────
        if cmd in ('STATE3', 'STATE_3', 'S3'):
            if self.state_out not in (SystemState.IDLE,):
                self._notify('warn', 'Không thể vào STATE 3',
                             f'{self.state_out.name} đang chạy')
                return
            if not self.zero_offset:
                self._notify('warn', 'Chua home', '')
                return
            if not self.sensor(7):
                self._notify('warn', 'S7 OFF', 'Chua co khay tren Platform')
                return
            if self.sensor(8):
                self._notify('warn', 'S8 ON', 'Vi tri robot da co khay')
                return
            self._notify('info', 'STATE 3', 'S7 ON + S8 OFF')
            self._enter_out(SystemState.S3_CHECK_OUTXY_SAFE)
            return

        if cmd in ('STATE4', 'STATE_4', 'S4'):
            if self.state_out not in (SystemState.IDLE,):
                self._notify('warn', 'Không thể vào STATE 4',
                             f'{self.state_out.name} đang chạy')
                return
            if not self.zero_offset:
                self._notify('warn', 'Chua home', '')
                return
            self._notify('info', 'STATE 4', 'Thay khay output')
            self._enter_out(SystemState.S4_CHECK_OUTY_SAFE)
            return

        self._notify('warn', f'goto_state: khong biet "{cmd}"', '')

    def _cb_update_config(self, msg: String):
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
        
        # Publish input_trays_empty state to Dobot Robot Node for pipeline drain handling
        b_msg = Bool()
        raw_empty = not (self.sensor(1) or self.sensor(2) or self.sensor(3))
        
        if raw_empty:
            self._input_trays_empty_debounce_count += 1
        else:
            self._input_trays_empty_debounce_count = 0
            
        # Require 20 ticks (1.0 second at 50ms/tick) of continuous continuous EMPTY
        b_msg.data = bool(self._input_trays_empty_debounce_count >= 20)
        self.pub_input_trays_empty.publish(b_msg)

    def _publish_sensors(self):
        states = "".join("1" if self.sensor(i) else "0" for i in range(1, 19))
        msg = String(); msg.data = states; self.pub_sensors.publish(msg)

    def _publish_state(self):
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
        if s == SystemState.HOMING:           self._do_homing(); return
        if s == SystemState.HOMING_RUNNING:   return
        if s == SystemState.ERROR:            self._do_error(); return
        self._dispatch_input()
        self._dispatch_output()

    def _dispatch_input(self):
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
        elif s == SystemState.S1_WAIT_S11:         self._s1_wait_s11()
        elif s == SystemState.S1_INY_10_FINAL:     self._s1_iny_10_final()
        elif s == SystemState.S1_INX_10:           self._s1_inx_10()
        elif s == SystemState.S1_COMPLETE:         self._s1_complete()
        elif s == SystemState.S2A_CHECK_INTERLOCK:   self._s2a_check_interlock()
        elif s == SystemState.S2A_INX_500:           self._s2a_inx_500()
        elif s == SystemState.S2A_INY_200_CYL1:      self._s2a_iny_200_cyl1()
        elif s == SystemState.S2A_WAIT_S11:          self._s2a_wait_s11()
        elif s == SystemState.S2A_INY_10:            self._s2a_iny_10()
        elif s == SystemState.S2A_INX_10:            self._s2a_inx_10()
        elif s == SystemState.S2A_INY_JOG_OUTPUT:    self._s2a_iny_jog_output()
        elif s == SystemState.S2A_INY_OUTPUT_ROW:    self._s2a_iny_output_row()
        elif s == SystemState.S2A_WAIT_S15:          self._s2a_wait_s15()
        elif s == SystemState.S2A_INY_10_FINAL:      self._s2a_iny_10_final()
        elif s == SystemState.S2A_INX_20:            self._s2a_inx_20()
        elif s == SystemState.S2A_COMPLETE:          self._s2a_complete()

    def _dispatch_output(self):
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
        elif s == SystemState.S4_CYL2_EXTEND:      self._s4_cyl2_extend()
        elif s == SystemState.S4_OUTY_TARGET1:     self._s4_outy_target1()
        elif s == SystemState.S4_OUTX_TARGET3:     self._s4_outx_target3()
        elif s == SystemState.S4_CHECK_S9:         self._s4_check_s9()
        elif s == SystemState.S4_OUTY_ROW1:        self._s4_outy_row1()
        elif s == SystemState.S4_OUTY_SCAN_S10:    self._s4_outy_scan_s10()
        elif s == SystemState.S4_OUTY_DROP:        self._s4_outy_drop()
        elif s == SystemState.S4_CYL2_RETRACT:     self._s4_cyl2_retract_state()
        elif s == SystemState.S4_OUTY_OUTX_HOME:   self._s4_outy_outx_home()
        elif s == SystemState.S4_COMPLETE:         self._s4_complete()

    # ── IDLE ─────────────────────────────────────────────────────

    def _do_idle_input(self):
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
            self._guide_logged.discard("IDLE_IN_S11")
            self._enter_in(SystemState.S1_CONFIRM_SAFE)
            return
        
        # Log lý do chờ
        if not (self.sensor(1) or self.sensor(2) or self.sensor(3)):
            self._log_once("IDLE_IN_NO_TRAY", "[IN-IDLE] S1/S2/S3 OFF — hết khay input")
        elif self.sensor(11):
            self._log_once("IDLE_IN_NO_PLACE", "[IN-IDLE] S11 ON — vị trí cấp đang có khay (chờ xử lý xong để chạy State 2)")
        elif self._motion_busy:
            self._log_once("IDLE_IN_BUSY", "[IN-IDLE] Robot đang bận — chờ")

    def _do_idle_output(self):
        if not self.zero_offset or not getattr(self, '_system_running', False):
            return

        # ── AUTO/MANUAL auto-trigger S3/S4 ────────────────────────
        if self._can_start_s4():
            self._s4_trigger = False
            self.get_logger().info("[OUT-IDLE] Output full → STATE 4")
            self._enter_out(SystemState.S4_CHECK_OUTY_SAFE)
            return
        if self._can_start_s3():
            self.get_logger().info("[OUT-IDLE] S7 ON + S8 OFF → STATE 3")
            self._enter_out(SystemState.S3_CHECK_OUTXY_SAFE)
            return

    def _do_homing(self):
        self._enter(SystemState.HOMING_RUNNING)
        def _bg():
            ok = self._home_all()
            if ok:
                self.get_logger().info("Homing complete")
                self._notify('info', 'Homing xong', '')
                self.state_in  = SystemState.IDLE
                self.state_out = SystemState.IDLE
                self._system_running = True
                self._enter(SystemState.IDLE)
            else:
                self._error("Homing that bai")
        threading.Thread(target=_bg, daemon=True).start()

    def _do_error(self):
        self._log_once("ERROR_STATE", "ERROR — kiem tra loi roi nhan STOP -> START")

    # ══════════════════════════════════════════════════════════════
    # STATE 1: Cap khay Input  (giữ nguyên logic gốc)
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
        if not (self.sensor(1) or self.sensor(2) or self.sensor(3)):
            self._log_once("S1_WAIT_BELT", "Step2: Cho S1/S2/S3")
            self._notify('info', 'Cho khay (S1/S2/S3)', 'Dat khay len bang tai.')
            return
        if not self._iny_safe():
            self._log_once("S1_INY_SAFE", "Step2: INY chua safe")
            return
        if not self._cmd_sent_in:
            ok = self._nb_move(1, self.config.inx_target)
            if not ok:
                self._log_once("S1_INX_FAIL", "S1 Step2: INX fail")
                return
            self._cmd_sent_in    = True
            self._inx_arrived = False
            self._30s_timeout = 0.0
            self._s15_warn_t  = 0.0
            self.get_logger().info(f"Step2: INX -> {self.config.inx_target}mm")
            self._enter_in(SystemState.S1_WAIT_ARRIVE)

    def _s1_wait_arrive(self):
        if not self._inx_arrived:
            self._inx_arrived = self._arrived(1)
            if not self._inx_arrived:
                self._log_once("S1_INX_MOVING", "INX dang di chuyen")
                return
            self._30s_timeout = time.time() + 50.0
            self.get_logger().info("INX dung tai 500mm — check S3+S15 (50s)")

        s3, s15 = self._snap(3, 15)

        if s3 and s15:
            self.get_logger().info("S3+S15 ON -> scan INY")
            self._s4_armed = False
            self._enter_in(SystemState.S1_INY_SCAN)
            return

        if not s3:
            if time.time() > self._30s_timeout:
                self.get_logger().info("S3 khong ON sau 50s — INX ve home, retry")
                self._nb_move(1, self.config.inx_home)
                self._enter_in(SystemState.S1_CONFIRM_SAFE)
                return
            remain = self._30s_timeout - time.time()
            self._log_once("S1_WAIT_S3", f"Cho S3 ON (con {remain:.0f}s)")
        else:
            if self._s15_warn_t == 0:
                self._s15_warn_t = time.time()
            elif time.time() - self._s15_warn_t >= 5.0:
                self._notify('warn', 'S3 ON nhưng S15 OFF', 'Cyl1 chưa retract')
                self._s15_warn_t = time.time()
            self._log_once("S1_WAIT_S15", "S3 ON, chờ S15 ON")

    def _s1_iny_scan(self):
        iny = self._pos(2)
        if iny is None:
            return

        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.target_scaninp1,
                               vel=self.config.iny_scan_vel)
            if not ok:
                self._log_once("S1_SCAN_FAIL", "S1 INY scan: nb_move fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
            self._s4_armed        = False
            self.get_logger().info(
                f"[S1 SCAN] INY -> {self.config.target_scaninp1:.0f}mm "
                f"vel={self.config.iny_scan_vel}"
            )

        if iny >= self.config.iny_scan_arm_mm:
            self._s4_armed = True

        if self._s4_armed and self.sensor(4):
            self._stop(2)
            trigger_pos       = self._pos(2) or iny
            row               = self._zone_to_row(trigger_pos, self.config.iny_input_zones)
            target_mm         = self.config.iny_input_zones[row][2]
            self._current_row = row
            self.get_logger().info(
                f"[S1 SCAN] S4 ON @ {trigger_pos:.1f}mm → row{row} ({target_mm:.0f}mm)"
            )
            self._nb_move(2, target_mm, vel=self.config.iny_row_vel)
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
            self._s5_retry        = 0
            self._enter_in(SystemState.S1_WAIT_STOP_S4)
            return

        timed_out = time.time() > self._step_timeout_in
        at_target = self._arrived(2) or iny >= self.config.target_scaninp1 - 2.0
        if timed_out or at_target:
            self._current_row = 1
            target_mm = self.config.iny_input_zones[1][2]
            self.get_logger().warn(
                f"[S1 SCAN] S4 không trigger "
                f"({'timeout' if timed_out else 'đến đích'}) → fallback row1 ({target_mm:.0f}mm)"
            )
            self._nb_move(2, target_mm, vel=self.config.iny_row_vel)
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
            self._s5_retry        = 0
            self._enter_in(SystemState.S1_WAIT_STOP_S4)
            return

        self._log_once("S1_SCANNING",
                       f"[S1 SCAN] INY {iny:.0f}mm arm={'OK' if self._s4_armed else 'NO'}")

    def _s1_wait_stop_s4(self):
        if self._arrived(2):
            self.get_logger().info(
                f"[S1] InY đến row{self._current_row} OK → S1_INY_TO_ROW"
            )
            self._enter_in(SystemState.S1_INY_TO_ROW)
            return
        if time.time() > self._step_timeout_in:
            self.get_logger().warn("[S1] S1_WAIT_STOP_S4 timeout → tiếp tục")
            self._enter_in(SystemState.S1_INY_TO_ROW)
            return
        self._log_once("S1_WAIT_ROW", f"[S1] Đang di chuyển tới row{self._current_row}...")

    def _s1_iny_to_row(self):
        zone = self.config.iny_input_zones.get(self._current_row)
        target = zone[2] if zone else None
        if target is None:
            valid = sorted(self.config.iny_input_zones.keys())
            self._current_row = max(valid[0], min(valid[-1], self._current_row))
            target = self.config.iny_input_zones[self._current_row][2]
        if not self._cmd_sent_in:
            ok = self._nb_move(2, target)
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
                self._enter_in(SystemState.S1_FALLBACK_RETRACT)

    def _s1_fallback_retract(self):
        s15, s16 = self._snap(15, 16)
        if not self._cmd_sent_in:
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in = True

        if s15 and not s16:
            self.get_logger().info("Fallback: S15 ON + S16 OFF -> INY ve home")
            self._nb_move(2, self.config.iny_home)
            self._nb_move(1, self.config.inx_home)
            self._enter_in(SystemState.S1_CONFIRM_SAFE)
            return

        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Fallback: Retry Cyl1 retract")
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S1_FB_RETRACT", "Fallback: cho S15 ON, S16 OFF")

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
        s16, s5 = self._snap(16, 5)
        if not s5:
            self.get_logger().warn("S5 OFF bat ngo -> huy extend")
            self._notify('warn', 'S5 OFF', 'Khay bi roi -> Reset')
            self._enter_in(SystemState.S1_FALLBACK_RETRACT)
            return

        if not self._cmd_sent_in:
            self._cyl1_extend()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in    = True
        if s16:
            self.get_logger().info("S16 ON — gap khay OK -> INY ve 50mm")
            self._enter_in(SystemState.S1_INY_50)
            return
        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Retry Cyl1 extend")
            self._cyl1_extend()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S1_WAIT_S16", "Cho S16 ON (Cyl1 extend)")

    def _s1_iny_50(self):
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_safe_zone)
            if not ok:
                self.get_logger().warn("S1: INY -> 50mm fail")
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
            ok = self._nb_move(2, self.config.iny_place)
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
        s15, s16 = self._snap(15, 16)
        if not self._cmd_sent_in:
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in    = True
        if s15 and not s16:
            self.get_logger().info("S15 ON + S16 OFF — Cyl1 nha xong -> cho S11")
            self._enter_in(SystemState.S1_WAIT_S11)
            return
        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Retry Cyl1 retract")
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S1_WAIT_REL", f"Cho S15 ON + S16 OFF | S15={s15} S16={s16}")

    def _s1_wait_s11(self):
        s11, = self._snap(11)
        if s11:
            self.get_logger().info("S11 ON — Khay o vi tri robot -> INY ve 10mm")
            self._enter_in(SystemState.S1_INY_10_FINAL)
            return
        self._log_once("S1_WAIT_S11", "Cho S11 ON")

    def _s1_iny_10_final(self):
        s15, s16 = self._snap(15, 16)
        if s16 or not s15:
            if not getattr(self, '_cmd_sent_in_cyl', False):
                self._cyl1_retract()
                self._cmd_sent_in_cyl = True
                self._cyl_retry_t = time.time() + 3.0
            if time.time() > self._cyl_retry_t:
                self._cyl1_retract()
                self._cyl_retry_t = time.time() + 3.0
            self._log_once("S1_ILK_10", "Interlock: Cho S15 ON, S16 OFF truoc khi ve 10mm")
            return
        self._cmd_sent_in_cyl = False

        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_home)
            if not ok:
                self.get_logger().warn("S1: INY -> 10mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(2):
                self._enter_in(SystemState.S1_INX_10)

    def _s1_inx_10(self):
        if not self._iny_safe():
            self._log_once("S1_INX_WAIT_INY", "Cho INY <= 50mm truoc INX ve 10mm")
            return
        if not self._cmd_sent_in:
            ok = self._nb_move(1, self.config.inx_output_pos)
            if not ok:
                self.get_logger().warn("S1: INX -> 10mm fail")
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
        Publish new_tray_loaded. Check S11 với timeout 3s.
        Xong thì về IDLE. IDLE sẽ chờ robot done_tray_input rồi kích S2A.
        """
        if not self._cmd_sent_in:
            self.stack_row_index  = self._current_row
            self._cmd_sent_in     = True
            self._s13_check_start = 0.0

        if self.sensor(11):
            if self._s13_check_start >= 0.0:
                self.pub_new_tray.publish(Bool(data=True))
                self.get_logger().info("Published: new_tray_loaded = True (S11 ON)")
                self._notify('info', 'STATE 1 COMPLETE', 'Khay đã ở robot')
                self._s13_check_start = -1.0

            self._enter_in(SystemState.IDLE)
        else:
            if self._s13_check_start == 0.0:
                self._s13_check_start = time.time()
                self.get_logger().warn("S11 chua ON — cho 3s")
            elapsed = time.time() - self._s13_check_start
            if elapsed >= S13_CHECK_TIMEOUT_S:
                self.get_logger().error(f"S11 OFF sau {S13_CHECK_TIMEOUT_S:.0f}s — caution")
                self._notify('warn', 'S11 OFF sau timeout', 'Kiem tra lai S11.')
                self.pub_new_tray.publish(Bool(data=True))
                self._state1_enabled = False
                self._enter_in(SystemState.IDLE)
            else:
                self._log_once("S1C_S18_WAIT",
                               f"Cho S11 ON (con {S13_CHECK_TIMEOUT_S - elapsed:.1f}s)")

    # ══════════════════════════════════════════════════════════════
    # STATE 2: Thay khay Input  (giữ nguyên logic gốc)
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
                    self.get_logger().warn("S2 Step1: INY home fail")
                    return
            self._log_once("S2A_WAIT_INY", "Step1: cho INY <= 50mm")

    def _s2a_inx_500(self):
        if not self._iny_safe():
            self._log_once("S2A_INY2", "A2: INY chua safe")
            return
        if not self.sensor(15):
            self._log_once("S2A_S15_ILK", "INTERLOCK S2: S15=OFF -> INX bi khoa")
            self._notify('warn', 'S15 chua ON', 'Cyl1 chua retract')
            return
        if not self._cmd_sent_in:
            ok = self._nb_move(1, self.config.inx_target)
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
            ok = self._nb_move(2, self.config.iny_place)
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
        self._log_once("S2A_S16", "A4: Cho S16 ON")

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
            ok = self._nb_move(1, self.config.inx_output_pos)
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

        # Gioi han quet tuy thuoc vao S6 co ON khong
        scan_target = self.config.iny_output_zones[2][1] if self._s6_snapshot else self.config.target_scanoutp1

        if not self._cmd_sent_in:
            ok = self._nb_move(2, scan_target, vel=self.config.iny_scan_vel)
            if not ok:
                self._log_once("S2A_JOG_FAIL", "S2A Step7: nb_move fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
            self._s4_armed_out    = False
            self.get_logger().info(
                f"[S2A Step7] S6=ON → scan xuống {scan_target:.0f}mm "
                f"vel={self.config.iny_scan_vel}"
            )

        if iny >= self.config.iny_scan_arm_mm:
            self._s4_armed_out = True

        if self._s4_armed_out and self.sensor(4):
            self._stop(2)
            trigger_pos             = self._pos(2) or iny
            row                     = self._zone_to_row(trigger_pos, self.config.iny_output_zones)
            if row is not None:
                target_mm               = self.config.iny_output_zones[row][2]
                self._output_target_pos = target_mm
                self._output_row        = row
                self.get_logger().info(
                    f"[S2A Step7] S4 ON @ {trigger_pos:.1f}mm → row{row} ({target_mm:.0f}mm)"
                )
                self._nb_move(2, target_mm, vel=self.config.iny_row_vel)
                self._cmd_sent_in     = True
                self._step_timeout_in = time.time() + self.config.move_timeout
                self._enter_in(SystemState.S2A_INY_OUTPUT_ROW)
                return

        timed_out = time.time() > self._step_timeout_in
        at_target = self._arrived(2) or iny >= scan_target - 2.0
        if timed_out or at_target:
            self._stop(2)
            if self._s6_snapshot:
                self._error(f"[S2A Step7] Lỗi: S4 không trigger khi quét đến {scan_target}mm (Max Row 2)")
                return
            else:
                self.get_logger().warn(
                    f"[S2A Step7] S4 không trigger "
                    f"({'timeout' if timed_out else 'đến đích'}) → fallback row1"
                )
                self._output_target_pos = self.config.iny_output_zones[1][2]
                self._output_row        = 1
                self._nb_move(2, self.config.iny_output_zones[1][2], vel=self.config.iny_row_vel)
                self._cmd_sent_in     = True
                self._step_timeout_in = time.time() + self.config.move_timeout
                self._enter_in(SystemState.S2A_INY_OUTPUT_ROW)
                return

        self._log_once("S2A_JOG_OUT",
                       f"[S2A Step7] INY {iny:.0f}mm arm={'YES' if self._s4_armed_out else 'NO'}")

    def _s2a_iny_output_row(self):
        if not self._cmd_sent_in:
            # Chỉ vào đây qua path S6=OFF (target đã set, chưa gửi move)
            target = self._output_target_pos
            if target <= 0:
                self.get_logger().error("output_target_pos chưa set → skip")
                self._enter_in(SystemState.S2A_WAIT_S15)
                return
            ok = self._nb_move(2, target, vel=self.config.iny_row_vel)
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
            self._enter_in(SystemState.S2A_WAIT_S15)

    def _s2a_wait_s15(self):
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
        self._log_once("S2A_S15", "A9: Cho S15 ON")

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
            ok = self._nb_move(1, self.config.inx_home)
            if not ok:
                self._log_once("S2A_A11_FAIL", "S2A A11: INX 20mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            elif self._arrived(1):
                self._enter_in(SystemState.S2A_COMPLETE)

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
        if not self._cmd_sent_out:
            self._pub_cartridge_busy(True)
        cfg = self.config
        ox = self._pos(4)
        oy = self._pos(5)
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
            else:
                if time.time() > self._step_timeout_out:
                    self._enter_out(SystemState.S3_SERVO3_TARGET1)
                elif self._arrived(4) and self._arrived(5):
                    self._enter_out(SystemState.S3_SERVO3_TARGET1)
                else:
                    self._log_once("S3_SAFE", "Cho OutX/OutY ve home")

    def _s3_servo3_target1(self):
        cfg = self.config
        if not self._cmd_sent_out:
            ok = self._nb_move(3, cfg.servo3_target1)
            if not ok:
                self._log_once("S3_T1_FAIL", "S3 target1 fail")
                return
            self._cmd_sent_out     = True
            self._step_timeout_out = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_out:
                self._error(f"S3_SERVO3_TARGET1 timeout")
            elif self._arrived(3):
                self._enter_out(SystemState.S3_CHECK_S7)

    def _s3_check_s7(self):
        if self.sensor(7):
            self._enter_out(SystemState.S3_SERVO3_FEED)
        else:
            self._enter_out(SystemState.S3_WAIT_S7)

    def _s3_wait_s7(self):
        if self.sensor(7):
            self._notify('info', 'Da phat hien khay', 'S7 ON — cap khay thanh pham')
            self._enter_out(SystemState.S3_SERVO3_FEED)
        else:
            self._log_once("S3_WAIT_S7", "Cho S7 ON — cap khay len Platform")

    def _s3_wait_gui_confirm(self):
        self._enter_out(SystemState.S3_SERVO3_FEED)

    def _s3_servo3_feed(self):
        cfg = self.config
        if not self._cmd_sent_out:
            ok = self._nb_move(3, cfg.servo3_target2, vel=int(cfg.servo3_feed_velocity))
            if not ok:
                self._log_once("S3_FEED_FAIL", "S3 feed fail")
                return
            self._cmd_sent_out     = True
            self._step_timeout_out = time.time() + self.config.move_timeout
            self.get_logger().info(f"[S3] Servo3 pushing to {cfg.servo3_target2}mm max until S8 ON")
        else:
            if self.sensor(8):
                self._stop(3)
                self.get_logger().info("[S3] S8 ON -> Dung Servo 3 som")
                self._enter_out(SystemState.S3_WAIT_S8)
                return

            if time.time() > self._step_timeout_out:
                self._error("S3_SERVO3_FEED timeout")
            elif self._arrived(3):
                self.get_logger().warn("[S3] Servo 3 tới giới hạn (target2) chưa có S8 -> Quay về 10mm thử lại!")
                self._notify('warn', 'Lỗi cấp khay', 'Đã tới 400mm nhưng chưa thấy S8, thu về cấp lại')
                self._enter_out(SystemState.S3_SERVO3_TARGET1)

    def _s3_wait_s8(self):
        if self.sensor(8):
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
        self._log_once("S3_WAIT_S8", "Cho S8 ON")

    def _s3_complete(self):
        self._pub_cartridge_busy(False)
        self.pub_newtray_output.publish(Bool(data=True))
        self._notify('info', 'State 3 done', 'Cap khay thanh pham thanh cong')
        self.get_logger().info("[S3] COMPLETE — pub new_trayoutput_loaded")
        self._enter_out(SystemState.IDLE)

    # ══════════════════════════════════════════════════════════════
    # STATE 4 — Thay khay output  (giữ nguyên logic gốc)
    # ══════════════════════════════════════════════════════════════

    def _s4_check_outy_safe(self):
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
                    self._log_once("S4_SAFE_FAIL", "S4 outy home fail")
                    return
                self._cmd_sent_out     = True
                self._step_timeout_out = time.time() + self.config.move_timeout
            else:
                if time.time() > self._step_timeout_out:
                    self._cmd_sent_out = False
                elif self._arrived(5):
                    self._enter_out(SystemState.S4_OUTX_TARGET2)
            self._log_once("S4_SAFE", "Cho OutY ve safe zone")

    def _s4_outx_target2(self):
        cfg = self.config
        if not self._cmd_sent_out:
            ok = self._nb_move(4, cfg.outx_target2)
            if not ok:
                self._log_once("S4_OX2_FAIL", "S4 outx_target2 fail")
                return
            self._cmd_sent_out     = True
            self._step_timeout_out = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_out:
                self._error("S4_OUTX_TARGET2 timeout")
            elif self._arrived(4):
                self._enter_out(SystemState.S4_OUTY_PICK)

    def _s4_outy_pick(self):
        cfg = self.config
        if not self._cmd_sent_out:
            ok = self._nb_move(5, cfg.outy_pick_pos)
            if not ok:
                self._log_once("S4_PICK_FAIL", "S4 outy_pick fail")
                return
            self._cmd_sent_out     = True
            self._step_timeout_out = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_out:
                self._error("S4_OUTY_PICK timeout")
            elif self._arrived(5):
                self._enter_out(SystemState.S4_CYL2_EXTEND)

    def _s4_cyl2_extend(self):
        if not self._cmd_sent_out:
            self._cyl2_extend()
            self._cmd_sent_out = True
            self._step_start_out = time.time()
        if self.sensor(20):
            self._enter_out(SystemState.S4_OUTY_TARGET1)
        elif time.time() - self._step_start_out > CYLINDER_TIMEOUT_S:
            self._error("[S4] Timeout: Cyl2 extend — S20 khong ON")

    def _s4_outy_target1(self):
        cfg = self.config
        if not self._cmd_sent_out:
            ok = self._nb_move(5, cfg.outy_target1)
            if not ok:
                self._log_once("S4_OY1_FAIL", "S4 outy_target1 fail")
                return
            self._cmd_sent_out     = True
            self._step_timeout_out = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_out:
                self._error("S4_OUTY_TARGET1 timeout")
            elif self._arrived(5):
                self._enter_out(SystemState.S4_OUTX_TARGET3)

    def _s4_outx_target3(self):
        cfg = self.config
        if not self._cmd_sent_out:
            ok = self._nb_move(4, cfg.outx_target3)
            if not ok:
                self._log_once("S4_OX3_FAIL", "S4 outx_target3 fail")
                return
            self._cmd_sent_out     = True
            self._step_timeout_out = time.time() + self.config.move_timeout
        else:
            if self.sensor(7) and not self._s3_pending:
                self._s3_pending = True
            if time.time() > self._step_timeout_out:
                self._error("S4_OUTX_TARGET3 timeout")
            elif self._arrived(4):
                self._enter_out(SystemState.S4_CHECK_S9)

    def _s4_check_s9(self):
        # S9 OFF -> Stack đang rỗng -> Bỏ qua scan, xuống thẳng Row 1
        if not self.sensor(9):
            self.get_logger().info(f"[S4] S9 OFF -> Bỏ qua scan S10, xuống thẳng Row 1")
            self._enter_out(SystemState.S4_OUTY_ROW1)
        else:
            self._s10_armed = False
            self._outy_jog_start = time.time()
            self._enter_out(SystemState.S4_OUTY_SCAN_S10)

    def _s4_outy_row1(self):
        cfg = self.config
        if not self._cmd_sent_out:
            # Di chuyen nhanh toi vi tri Target cua Row 1 trong Zone
            target = cfg.outy_output_zones[1][2]
            ok = self._nb_move(5, target)
            if not ok:
                self._log_once("S4_ROW1_FAIL", "S4 outy_row1 fail")
                return
            self._cmd_sent_out     = True
            self._step_timeout_out = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_out:
                self._error("S4_OUTY_ROW1 timeout")
            elif self._arrived(5):
                self._enter_out(SystemState.S4_CYL2_RETRACT)

    def _s4_outy_scan_s10(self):
        cfg = self.config
        oy = self._pos(5)
        
        # Doc 2 sensor 10 cung ra cho logic an toan, lay S10 (index 10) 
        s10_out = self.sensor(10)

        # Logic tim row giong iny_scan (S4)
        if hasattr(self, '_s10_armed'):
            if not self._s10_armed and s10_out:
                self._s10_armed = True
            elif self._s10_armed and not s10_out:
                row = self._zone_to_row(oy, cfg.outy_output_zones)
                if row is not None:
                    target = cfg.outy_output_zones[row][2]
                    self._stop(5)
                    self._outy_jog_pos = target
                    self.get_logger().info(f"[S4] S10 EDGE -> Chot ROW {row} Target={target}")
                    self._enter_out(SystemState.S4_OUTY_DROP)
                    return
        else:
            self._s10_armed = False

        # Gioi han quyet dinh boi S9
        limit_pos = cfg.outy_row_limit
        if self.sensor(9):
            limit_pos = cfg.outy_output_zones[2][1]

        if oy is not None and oy >= limit_pos:
            self._stop(5)
            self._notify('warn', 'S10 khong ON', 'Da het gioi han')
            self._error(f"[S4] OutY dat gioi han {limit_pos}mm ma S10 chua ON")
            return

        if self._outy_jog_start > 0 and time.time() - self._outy_jog_start > JOG_OUTPUT_TIMEOUT_S:
            self._stop(5)
            self._error(f"[S4] S4_OUTY_SCAN_S10 timeout — S10 khong chot row")
            return

        self._jog(5, int(cfg.outy_slow_vel))
        self._log_once("S4_SCAN", f"Jog OutY tim S10 zone (gioi han {limit_pos}mm)")

    def _s4_outy_drop(self):
        cfg = self.config
        if not self._cmd_sent_out:
            extra_target = self._outy_jog_pos or 0.0
            
            limit_pos = cfg.outy_row_limit
            if self.sensor(9):
                limit_pos = cfg.outy_output_zones[2][1]

            # Gioi han khong ruot qua row limit qua muc tuc la S10 loi 
            extra_target = min(extra_target, limit_pos + 30.0)
            self._nb_move(5, extra_target, vel=int(cfg.outy_slow_vel))
            self._cmd_sent_out = True
            self._step_start_out = time.time()
        if self._arrived(5):
            self._enter_out(SystemState.S4_CYL2_RETRACT)
        elif time.time() - self._step_start_out > 5.0:
            self._enter_out(SystemState.S4_CYL2_RETRACT)

    def _s4_cyl2_retract_state(self):
        if not self._cmd_sent_out:
            self._cyl2_retract()
            self._cmd_sent_out = True
            self._step_start_out = time.time()
        if self.sensor(19) and not self.sensor(20):
            self._enter_out(SystemState.S4_OUTY_OUTX_HOME)
        elif time.time() - self._step_start_out > CYLINDER_TIMEOUT_S:
            self._error("[S4] Timeout: Cyl2 retract")

    def _s4_outy_outx_home(self):
        cfg = self.config
        if not self._cmd_sent_out:
            ok4 = self._nb_move(4, cfg.outx_home)
            ok5 = self._nb_move(5, cfg.outy_target1)
            if not ok4 or not ok5:
                self._log_once("S4_HOME_FAIL", "S4 outy/outx home fail")
                return
            self._cmd_sent_out     = True
            self._step_timeout_out = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_out:
                self._error("S4_OUTY_OUTX_HOME timeout")
            elif self._arrived(5) and self._arrived(4):
                self._enter_out(SystemState.S4_COMPLETE)

    def _s4_complete(self):
        self._pub_cartridge_busy(False)
        self._notify('info', 'State 4 done', 'Thay khay output thanh cong')
        self.get_logger().info("[S4] COMPLETE")
        self._s4_trigger = False
        if self._can_start_s3():
            self.get_logger().info("[S4→S3] S7 ON + S8 OFF → cấp khay output mới")
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
        print("  Cartridge System v7")
        print("  AUTO  : Sensor thực, tự động trigger STATE 1/2/3/4")
        print("  MANUAL: Sensor sim, STATE1 cần nhấn tay, S2/3/4 auto")
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