#!/usr/bin/env python3
"""
Cartridge Loading System — ROS 2 + festo-edcon

Workflow:
  STATE 1  — Cấp khay Input
               (băng tải Input → robot)
               Điều kiện: S1/S2/S3 ON + S12 ON + đã homing

  STATE 2  — Thay khay Input
               (gắp khay cũ từ robot → output stack custom tray)
               Điều kiện: robot_done + S13 ON
               Kết thúc: tự check lại đk State 1 → loop liên tục
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from enum import Enum, auto
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
COUNTS_PER_MM       = 1000
CYLINDER_TIMEOUT_S  = 15.0
S5_WAIT_S           = 5.0       # Thời gian chờ S5 xác nhận có khay
INY_JOG_VEL         = 40        # mm/s — tốc độ tìm khay
INY_S4_ARM_MM       = 150.0     # INY ≥ 150mm thì mới xét S4
INY_ROW1_LIMIT_MM   = 600.0     # Giới hạn INY khi tìm input
OUTY_ROW1_LIMIT_MM  = 590.0     # Giới hạn INY khi tìm output stack


# ─── State Enum ──────────────────────────────────────────────────

class SystemState(Enum):
    # Hệ thống chung
    IDLE            = "idle"
    ERROR           = "error"
    HOMING          = "homing"
    HOMING_RUNNING  = "homing_running"

    # ── STATE 1: Cấp khay Input ──────────────────────────────────
    S1_CONFIRM_SAFE     = "s1_confirm_safe"     # Step 1 : INY → 10mm nếu > 10mm
    S1_INX_MOVE         = "s1_inx_move"         # Step 2 : INX → 500mm (non-blocking)
    S1_WAIT_ARRIVE      = "s1_wait_arrive"       # Step 3 : Chờ INX dừng + S3 + S10
    S1_INY_JOG          = "s1_iny_jog"           # Step 4 : INY jog tìm khay (S4)
    S1_INY_TO_ROW       = "s1_iny_to_row"        # Step 5 : INY → iny_input_stack[row]
    S1_CHECK_S5         = "s1_check_s5"          # Step 6a: Chờ S5 ON (3s)
    S1_CHECK_S5_ROW1    = "s1_check_s5_row1"     # Step 6b: Thử row+1 (3s)
    S1_WAIT_GUI_CONFIRM = "s1_wait_gui_confirm"  # Step 6c: Popup lỗi S4/S5
    S1_RETRY_INX_500    = "s1_retry_inx_500"     # Step 6d: INX → 500 (sau xác nhận)
    S1_RETRY_JOG        = "s1_retry_jog"         # Step 6e: Quay lại jog
    S1_CYL1_EXTEND      = "s1_cyl1_extend"       # Step 7 : Chờ S11 ON
    S1_CHECK_S13        = "s1_check_s13"          # Step 7b: S13 interlock
    S1_WAIT_S12_SAFE    = "s1_wait_s12_safe"      # Step 7c: Chờ S12 ON (Cyl2 về)
    S1_INY_50_CYL2      = "s1_iny_50_cyl2"       # Step 8 : INY → 50 + Cyl2 extend ch9
    S1_WAIT_S13_ON      = "s1_wait_s13_on"        # Step 9 : Chờ S13 ON (Cyl2 đỡ)
    S1_INY_200          = "s1_iny_200"            # Step 10: INY → 200mm
    S1_PLACE_DELAY      = "s1_place_delay"        # Step 10: 1s → retract Cyl1 ch4
    S1_WAIT_RELEASE     = "s1_wait_release"       # Step 11: Chờ S10 ON + S11 OFF
    S1_INY_50_FINAL     = "s1_iny_50_final"       # Step 12: INY → 50mm
    S1_INX_HOME         = "s1_inx_home"           # Step 13: INX → 20mm
    S1_COMPLETE         = "s1_complete"           # ✅ Publish new_tray_loaded

    # ── STATE 2: Thay khay Input ──────────────────────────────────
    S2A_CHECK_INTERLOCK = "s2a_check_interlock"  # 1: INY ≤ 50, snapshot S5
    S2A_INX_500         = "s2a_inx_500"          # 2: INX → 500mm
    S2A_INY_200_CYL1    = "s2a_iny_200_cyl1"     # 3: INY → 200mm + Cyl1 extend ch5
    S2A_WAIT_S11        = "s2a_wait_s11"         # 4: S11 ON = gắp khay cũ
    S2A_INY_10          = "s2a_iny_10"           # 5: INY → 10mm
    S2A_INX_10          = "s2a_inx_10"           # 6: INX → 10mm (INY ≤ 50mm)
    S2A_INY_JOG_OUTPUT  = "s2a_iny_jog_output"  # 7: INY jog → output stack
    S2A_INY_OUTPUT_ROW  = "s2a_iny_output_row"  # 8: INY → row + Cyl1 retract ch4
    S2A_WAIT_S10        = "s2a_wait_s10"         # 9: S10 ON = thả khay
    S2A_INY_10_FINAL    = "s2a_iny_10_final"     # 10: INY → 10mm
    S2A_INX_20          = "s2a_inx_20"           # 11: INX → 20mm
    S2A_WAIT_CYL2_RETRACT = "s2a_wait_cyl2_retract" # 11b: Chờ Cyl2 retract (S12 ON + S13 OFF)
    S2A_COMPLETE        = "s2a_complete"         # ✅ Check điều kiện → quay lại State 1


# ─── Config ──────────────────────────────────────────────────────

class CartridgeConfig:
    def __init__(self, config_file: Optional[str] = None):
        # Hardware IPs
        self.servo_ips = {1: "192.168.27.248", 2: "192.168.27.249",
                          3: "192.168.27.250",  4: "192.168.27.251", 5: "192.168.27.252"}
        self.io_ip = "192.168.27.253"

        # Cylinder channels
        self.cylinder1_extend_channel  = 5   # ch5
        self.cylinder1_retract_channel = 4   # ch4
        self.cylinder2_extend_channel  = 9   # ch9 — Cyl2 Hold Tray (kẹp khay)
        self.cylinder2_retract_channel = 8   # ch8 — Cyl2 Hold Tray (nhả khay)

        # Servo positions (mm)
        self.inx_home       = 20.0
        self.inx_target     = 500.0   # INX tới conveyor
        self.iny_home       = 10.0    # INY home / confirm safe
        self.iny_safe_zone  = 50.0    # INY ≤ 50mm → INX được di chuyển
        self.iny_place      = 200.0   # INY đặt khay lên Cyl2 (Hold Tray)

        # INY row positions — input stack (row 8 = gần home, row 1 = xa)
        self.iny_input_stack = {
            8: 250.0, 7: 300.0, 6: 350.0, 5: 400.0,
            4: 450.0, 3: 500.0, 2: 550.0, 1: 600.0
        }
        # INY row positions — output stack
        self.iny_output_stack = {
            8: 100.0, 7: 170.0, 6: 240.0, 5: 310.0,
            4: 380.0, 3: 450.0, 2: 520.0, 1: 590.0
        }

        # Soft limits per servo (mm)
        self.servo_limits = {1: 700.0, 2: 700.0, 3: 400.0, 4: 600.0, 5: 600.0}

        # Timeouts
        self.homing_timeout = 90.0
        self.move_timeout   = 25.0

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
            print(f"✅ Config loaded: {path}")
        except Exception as e:
            print(f"⚠️ Config load error: {e}")

    def save_to_file(self):
        if not self._config_file:
            return
        keys = ['servo_ips', 'io_ip', 'cylinder1_extend_channel', 'cylinder1_retract_channel',
                'cylinder2_extend_channel', 'cylinder2_retract_channel',
                'inx_home', 'inx_target', 'iny_home', 'iny_safe_zone', 'iny_place',
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
            print(f"❌ Config save error: {e}")


# ─── Main Node ───────────────────────────────────────────────────

class CartridgeSystem(Node):

    def __init__(self, config: CartridgeConfig):
        super().__init__('cartridge_providesystem')
        self.config = config

        # ── Hardware ──
        self._servo_lock = threading.Lock()
        self.servos: dict      = {}
        self.zero_offset: dict = {}
        self.io_module         = None
        self._io_sensor_cache: list = []
        self._io_ready         = False
        self._io_bg_lock       = threading.Lock()
        self._sim_sensors: dict = {}

        # ── Motion flags ──
        self._inx_moving = False
        self._iny_moving = False

        # ── STATE 1 runtime ──
        self._cmd_sent       = False    # Đã gửi lệnh bước hiện tại chưa
        self._step_start     = 0.0     # Timestamp bắt đầu bước
        self._step_timeout   = 0.0     # Absolute timeout của bước
        self._s4_armed       = False   # S4 được xét khi INY ≥ 150mm
        self._current_row    = 0       # Row hiện tại (input stack)
        self._s5_retry       = 0       # 0=lần đầu, 1=row+1 đã thử
        self._inx_arrived    = False   # Non-blocking INX tracking
        self._30s_timeout    = 0.0     # Timeout 30s chờ S3

        # ── STATE 2A runtime ──
        self._s5_snapshot    = False   # Snapshot S5 tại A1
        self._output_row     = 0       # Output stack row tìm được
        self._s4_armed_out   = False   # S4 armed cho output jog

        # ── Tray tracking ──
        self.stack_row_index = 0
        self._tray_loaded_ack = False

        # ── Operation ──
        # Hai mode duy nhất:
        #   'auto'   — Chạy tự động, dùng sensor thực tế, KHÔNG sim sensor
        #   'manual' — Cùng workflow như auto, thêm: jog được + sim sensor được
        self.state          = SystemState.IDLE
        self.operation_mode = 'auto'   # Mặc định: auto
        self._robot_done    = False    # Tín hiệu robot trả khay xong
        self._state1_enabled = False   # Phải nhấn STATE1 từ GUI để enable (Plan 3)
        self._gui_confirmed = False    # GUI popup xác nhận
        self._system_paused = False

        # ── Watchdog ──
        self._watchdog_last_tick = time.time()
        self._notify_throttle: dict = {}
        self._guide_logged: set = set()

        # ── ROS Publishers ──
        qos = QoSProfile(depth=1)
        self.pub_state          = self.create_publisher(String, '/system_state', qos)
        self.pub_new_tray       = self.create_publisher(Bool,   '/revpi/new_tray_loaded', qos)
        self.pub_gui_notify     = self.create_publisher(String, '/providesystem/gui_notify', qos)
        self.pub_servo_pos      = self.create_publisher(String, '/providesystem/servo_positions', qos)

        # ── ROS Subscribers ──
        self.create_subscription(Bool,   '/system/start_button',  self._cb_start,   qos)
        self.create_subscription(Bool,   '/system/stop_button',   self._cb_stop,    qos)
        self.create_subscription(Bool,   '/system/pause_button',  self._cb_pause,   qos)
        self.create_subscription(Bool,   '/revpi/robot_done',     self._cb_robot_done, qos)
        self.create_subscription(String, '/providesystem/gui_confirm', self._cb_gui_confirm, qos)
        self.create_subscription(String, '/providesystem/jog_cmd',    self._cb_jog,  qos)
        self.create_subscription(String, '/providesystem/change_tray_input',
                                 self._cb_change_tray_input, qos)
        self.create_subscription(String, '/providesystem/sim_sensor', self._cb_sim,  qos)
        self.create_subscription(String, '/providesystem/set_operation_mode',
                                 self._cb_mode, qos)
        self.create_subscription(String, '/providesystem/goto_state',
                                 self._cb_goto_state, qos)

        self._connect_hardware()

        # Timers
        self.create_timer(0.05, self._control_loop)     # 20 Hz control
        self.create_timer(0.5,  self._publish_positions)
        self.create_timer(5.0,  self._watchdog)

        self.get_logger().info("✅ CartridgeSystem node started")

    # ══════════════════════════════════════════════════════════════
    # Hardware
    # ══════════════════════════════════════════════════════════════

    def _connect_hardware(self):
        if not EDCON_AVAILABLE:
            self.get_logger().warn("⚠️ Simulation mode (edcon not installed)")
            return
        if EdconLogging:
            EdconLogging()
        for sid, ip in self.config.servo_ips.items():
            self._connect_servo(sid, ip)
        # Start background reconnect monitor
        threading.Thread(
            target=self._servo_reconnect_loop, daemon=True, name="servo_reconnect"
        ).start()

    def _connect_servo(self, sid: int, ip: str, attempts: int = 3) -> bool:
        """Try to connect one servo, retry on failure. Returns True if connected."""
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
                self.get_logger().info(f"  S{sid} ({ip}) ✅ (attempt {attempt})")
                return True
            except Exception as e:
                self.get_logger().warn(
                    f"  S{sid} ({ip}) ❌ attempt {attempt}/{attempts}: {e}"
                )
                if attempt < attempts:
                    time.sleep(2.0)
        self.get_logger().error(f"  S{sid} ({ip}) ❌ failed after {attempts} attempts")
        return False

    def _servo_reconnect_loop(self):
        """Background thread: detect disconnected servos and reconnect."""
        RECONNECT_INTERVAL_S = 10.0
        consecutive_fail: dict = {}
        while rclpy.ok():
            time.sleep(RECONNECT_INTERVAL_S)
            for sid, ip in self.config.servo_ips.items():
                mot = self.servos.get(sid)
                if mot is None:
                    # Never connected — retry
                    self.get_logger().warn(f"[servo_reconnect] S{sid} not in servos → reconnect")
                    self._connect_servo(sid, ip)
                    continue
                # Quick health check: try to read position
                try:
                    with self._servo_lock:
                        mot.current_position()
                    consecutive_fail[sid] = 0
                except Exception as e:
                    consecutive_fail[sid] = consecutive_fail.get(sid, 0) + 1
                    count = consecutive_fail[sid]
                    self.get_logger().warn(
                        f"[servo_reconnect] S{sid} health fail #{count}: {e}"
                    )
                    if count >= 2:
                        # Servo unresponsive — reconnect
                        self.get_logger().error(
                            f"[servo_reconnect] S{sid} ({ip}) unresponsive → reconnect"
                        )
                        self._notify('warn', f'⚠️ Servo S{sid} mất kết nối',
                                     f'Đang tự reconnect S{sid} ({ip})...')
                        with self._servo_lock:
                            self.servos.pop(sid, None)
                        connected = self._connect_servo(sid, ip)
                        if connected:
                            consecutive_fail[sid] = 0
                            self._notify('info', f'✅ S{sid} kết nối lại',
                                         f'Servo S{sid} ({ip}) đã reconnect OK')

        if CPXAP_AVAILABLE:
            for attempt in range(1, 4):
                try:
                    self.io_module = CpxAp(ip_address=self.config.io_ip, cycle_time=0.5)
                    self.get_logger().info(f"  IO {self.config.io_ip} ✅")
                    threading.Thread(target=self._io_bg_loop, daemon=True, name="io_bg").start()
                    break
                except Exception as e:
                    self.get_logger().warn(f"  IO attempt {attempt}/3: {e}")
                    if attempt < 3:
                        time.sleep(3.0)

    def destroy_node(self):
        self.get_logger().info("Shutdown — stopping servos...")
        for sid, mot in self.servos.items():
            try:
                with self._servo_lock:
                    mot.stop_motion_task()
                    mot.shutdown()
            except Exception as e:
                self.get_logger().warn(f"  S{sid} shutdown: {e}")
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
        """Đọc giá trị cảm biến thực từ phần cứng (bất kể mode)."""
        with self._io_bg_lock:
            cache = self._io_sensor_cache
            ready = self._io_ready
        if not ready or len(cache) < sid:
            return False
        return bool(cache[sid - 1])

    def sensor(self, sid: int) -> bool:
        """
        Đọc giá trị sensor logic dùng trong workflow.
        Cả 2 mode đều đọc phần cứng thực tế.
        Sim sensor là forced-ON (OR) như relay PLC — không đè giá trị thực.
        Kết quả: real_value OR sim_forced
        """
        real = self._sensor_raw(sid)
        sim_forced = self._sim_sensors.get(sid, False)
        return real or sim_forced

    def sensor_real(self, sid: int) -> bool:
        """Luôn trả giá trị cảm biến thực (dùng cho GUI hiển thị đèn)."""
        return self._sensor_raw(sid)

    # ── Non-blocking servo motion ─────────────────────────────────

    def _nb_move(self, servo_id: int, pos_mm: float, vel: int = 30) -> bool:
        """Send non-blocking move command. Returns False on error."""
        limit = self.config.servo_limits.get(servo_id, 999.0)
        if pos_mm > limit:
            self.get_logger().error(f"🚫 S{servo_id}: {pos_mm}mm > limit {limit}mm")
            return False
        if servo_id not in self.zero_offset and servo_id in self.servos:
            self.get_logger().error(f"🚫 S{servo_id}: chưa home!")
            return False
        mot = self.servos.get(servo_id)
        if not mot:
            # Simulation — treat as instant arrive
            return True
        try:
            offset = self.zero_offset.get(servo_id, 0)
            counts = offset + int(pos_mm * COUNTS_PER_MM)
            with self._servo_lock:
                self._ensure_ready(mot, servo_id)
                mot.position_task(counts, vel, absolute=True, nonblocking=True)
            if servo_id == 1:
                self._inx_moving = True
            elif servo_id == 2:
                self._iny_moving = True
            return True
        except Exception as e:
            self.get_logger().error(f"⚠️ S{servo_id} nb_move error: {e} — servo có thể mất kết nối")
            return False

    def _arrived(self, servo_id: int) -> bool:
        """Check target_position_reached via Modbus lock."""
        mot = self.servos.get(servo_id)
        if not mot:
            return True   # sim
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
        """Return current position in mm, or None on error."""
        mot = self.servos.get(servo_id)
        if not mot:
            return 0.0
        try:
            with self._servo_lock:
                counts = mot.current_position()
            return (counts - self.zero_offset.get(servo_id, 0)) / COUNTS_PER_MM
        except Exception as e:
            self.get_logger().warn(f"⚠️ S{servo_id} _pos() error: {e}")
            return None

    def _jog(self, servo_id: int, vel_mm_s: float):
        """Jog servo. Trong manual mode: cho phép JOG kể cả chưa home."""
        # Chỉ block JOG trong AUTO mode nếu chưa home (an toàn hơn)
        if servo_id not in self.zero_offset and servo_id in self.servos:
            if self.operation_mode != 'manual':
                self.get_logger().error(f"🚫 S{servo_id} JOG blocked — chưa home (AUTO)")
                return
            # Manual: cho phép JOG chưa home, chỉ warn
            self.get_logger().warn(f"⚠️ S{servo_id} JOG chưa home — vị trí không đảm bảo")
        mot = self.servos.get(servo_id)
        if not mot:
            return
        try:
            self._ensure_ready(mot, servo_id)
            positive = vel_mm_s > 0
            with self._servo_lock:
                mot.jog_task(jog_positive=positive, jog_negative=not positive, duration=0)
        except Exception as e:
            self.get_logger().error(f"S{servo_id} jog error: {e}")

    def _ensure_ready(self, mot, servo_id: int, timeout: float = 3.0):
        """Enable powerstage and wait for ready (called under _servo_lock)."""
        mot.acknowledge_faults()
        mot.enable_powerstage()
        if not hasattr(mot, 'ready_for_motion'):
            return
        start = time.time()
        while time.time() - start < timeout:
            if mot.ready_for_motion():
                return
            time.sleep(0.05)
        self.get_logger().warn(f"⚠️ S{servo_id}: drive not ready after {timeout}s")

    # ── Cylinders ─────────────────────────────────────────────────

    def _cyl1_extend(self):
        self.get_logger().info("▶ Cyl1 EXTEND (ch5)")
        if self.io_module:
            try:
                self.io_module.reset_channel(self.config.cylinder1_retract_channel)
                self.io_module.set_channel(self.config.cylinder1_extend_channel)
            except Exception as e:
                self.get_logger().warn(f"Cyl1 extend IO: {e}")

    def _cyl1_retract(self):
        self.get_logger().info("◀ Cyl1 RETRACT (ch4)")
        if self.io_module:
            try:
                self.io_module.reset_channel(self.config.cylinder1_extend_channel)
                self.io_module.set_channel(self.config.cylinder1_retract_channel)
            except Exception as e:
                self.get_logger().warn(f"Cyl1 retract IO: {e}")

    def _cyl2_extend(self):
        self.get_logger().info("▶ Cyl2 EXTEND (ch9) — Hold Tray kẹp khay")
        if self.io_module:
            try:
                self.io_module.reset_channel(self.config.cylinder2_retract_channel)
                self.io_module.set_channel(self.config.cylinder2_extend_channel)
            except Exception as e:
                self.get_logger().warn(f"Cyl2 extend IO: {e}")

    def _cyl2_retract(self):
        self.get_logger().info("◀ Cyl2 RETRACT (ch8) — Hold Tray nhả khay")
        if self.io_module:
            try:
                self.io_module.reset_channel(self.config.cylinder2_extend_channel)
                if self.config.cylinder2_retract_channel:
                    self.io_module.set_channel(self.config.cylinder2_retract_channel)
            except Exception as e:
                self.get_logger().warn(f"Cyl2 retract IO: {e}")

    # ── Helper utilities ─────────────────────────────────────────

    def _enter(self, next_state: SystemState):
        """Transition to next_state and reset per-step flags."""
        self.get_logger().info(f"  → {next_state.name}")
        self.state      = next_state
        self._cmd_sent  = False
        self._step_start = 0.0
        self._step_timeout = 0.0
        self._guide_logged.discard(next_state.name)

    def _error(self, msg: str):
        self.get_logger().error(f"❌ {msg}")
        self._notify('error', '❌ ERROR', msg)
        self._enter(SystemState.ERROR)

    def _notify(self, level: str, title: str, detail: str = ""):
        now = time.time()
        key = title
        if now - self._notify_throttle.get(key, 0) < 0.5:
            return
        self._notify_throttle[key] = now
        try:
            msg = String()
            msg.data = json.dumps({"level": level, "title": title, "detail": detail})
            self.pub_gui_notify.publish(msg)
        except Exception:
            pass

    def _log_once(self, key: str, msg: str):
        if key not in self._guide_logged:
            self._guide_logged.add(key)
            self.get_logger().info(f"📋 {msg}")

    def _find_nearest_row(self, pos_mm: float, row_dict: dict) -> int:
        """Return nearest row key in row_dict at position >= pos_mm (hướng +).
        Nếu không có row nào ≥ pos_mm thì lấy row có vị trí lớn nhất."""
        # Chỉ xét row ở phía trước (≥ vị trí hiện tại)
        candidates = {r: p for r, p in row_dict.items() if p >= pos_mm}
        if not candidates:
            # Fallback: row lớn nhất (cuối stack)
            return max(row_dict, key=lambda r: row_dict[r])
        # Gần nhất theo hướng +
        return min(candidates, key=lambda r: candidates[r] - pos_mm)

    def _has_trays(self) -> bool:
        return self.sensor(1) or self.sensor(2) or self.sensor(3) or self.stack_row_index > 1

    def _iny_safe(self) -> bool:
        """INY ≤ iny_safe_zone mm."""
        p = self._pos(2)
        return p is not None and p <= self.config.iny_safe_zone

    # ══════════════════════════════════════════════════════════════
    # Homing
    # ══════════════════════════════════════════════════════════════

    def _home_all(self) -> bool:
        """Home all servos in background thread. Protected by _servo_lock."""
        NAMES = {1: "InX", 2: "InY", 3: "Platform", 4: "OutX", 5: "OutY"}
        phases = [([2, 5], "InY + OutY"), ([1, 4, 3], "InX + OutX + Platform")]

        for servo_ids, phase_name in phases:
            active = [(sid, self.servos[sid]) for sid in servo_ids if sid in self.servos]
            if not active:
                continue
            self.get_logger().info(f"🏠 Homing {phase_name}...")

            for sid, mot in active:
                try:
                    with self._servo_lock:
                        self._ensure_ready(mot, sid, timeout=5.0)
                        mot.referencing_task(nonblocking=True)
                except Exception as e:
                    self.get_logger().error(f"❌ {NAMES.get(sid)} home start: {e}")
                    return False

            start = time.time()
            while time.time() - start < self.config.homing_timeout:
                with self._servo_lock:
                    done = all(mot.referenced() for _, mot in active)
                if done:
                    break
                time.sleep(0.1)
            else:
                for sid, mot in active:
                    with self._servo_lock:
                        if not mot.referenced():
                            self.get_logger().warn(f"⚠️ {NAMES.get(sid)} timeout — assume home")
                            mot.stop_motion_task()

            time.sleep(0.3)
            for sid, mot in active:
                if sid not in self.zero_offset:
                    with self._servo_lock:
                        self.zero_offset[sid] = mot.current_position()
                self.get_logger().info(f"  ✅ {NAMES.get(sid)} = 0mm")

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
            # MANUAL: giữ nguyên vị trí, chỉ vào IDLE
            # Homing phải chọn riêng từ State Navigation
            self._enter(SystemState.IDLE)
            self._notify('info', '▶️ START (MANUAL)',
                         'Giữ nguyên vị trí — chọn HOMING từ State Navigation nếu cần')
        else:
            # AUTO: home trước khi chạy
            self.zero_offset.clear()
            self._state1_enabled = True   # AUTO: enable State 1 auto-watch
            self._enter(SystemState.HOMING)
            self._notify('info', '▶️ START — HOMING', '')

    def _cb_stop(self, msg: Bool):
        if not msg.data:
            return
        for sid in list(self.servos):
            self._stop(sid)
        self._enter(SystemState.IDLE)
        self._system_paused = False
        self._robot_done = False   # Reset: tránh flag cũ trigger S2A sau STOP
        self._state1_enabled = False  # Reset: phải chọn lại STATE 1
        self._notify('warn', '⏹️ STOP', '')

    def _cb_pause(self, msg: Bool):
        if not msg.data:
            return
        self._system_paused = True
        self._notify('warn', '⏸️ PAUSE', 'Nhấn RESUME để tiếp tục')

    def _cb_robot_done(self, msg: Bool):
        if msg.data:
            self._robot_done = True

    def _cb_gui_confirm(self, msg: String):
        self._gui_confirmed = True

    def _cb_mode(self, msg: String):
        """
        Chuyển mode: chỉ chấp nhận 'auto' hoặc 'manual'.
        Chuyển mode chỉ được khi IDLE hoặc ERROR.
        Khi về auto: xóa toàn bộ sim_sensors để đảm bảo đọc sensor thực.
        """
        requested = msg.data.strip().lower()
        if requested not in ('auto', 'manual'):
            self._notify('warn', f'Mode không hợp lệ: {requested}',
                         "Chỉ chấp nhận 'auto' hoặc 'manual'")
            return
        active_states = {SystemState.IDLE, SystemState.ERROR,
                         SystemState.HOMING, SystemState.HOMING_RUNNING}
        if self.state not in active_states:
            self._notify('warn', '⚠️ Không thể đổi mode',
                         f'Hệ thống đang chạy ({self.state.name}) — nhấn STOP trước')
            return
        old_mode = self.operation_mode
        self.operation_mode = requested
        if requested == 'auto':
            # Auto mode không cho phép sim sensor — xóa hết
            self._sim_sensors.clear()
            self.get_logger().info("🔄 Mode → AUTO | Đã xóa toàn bộ sim_sensors")
        else:
            self.get_logger().info("🔄 Mode → MANUAL | Sim sensor + JOG được phép")
        self._notify('info', f'Mode: {requested.upper()}',
                     'Sensor thực tế' if requested == 'auto' else 'Sim sensor + JOG enabled')
        # Reset log cache khi đổi mode để guide messages hiển thị lại
        if old_mode != requested:
            self._guide_logged.clear()

    def _cb_jog(self, msg: String):
        """
        JOG axis — chỉ hoạt động ở MANUAL mode khi state == IDLE.
        Format: '<servo_id> +|-|stop [vel_mm_s]'
                'home <servo_id>'
                'clear <servo_id>'
        """
        if self.operation_mode != 'manual':
            self._notify('warn', '⚠️ JOG bị khóa',
                         'Chuyển sang MANUAL mode để JOG')
            return
        if self.state not in (SystemState.IDLE, SystemState.ERROR):
            self._notify('warn', '⚠️ JOG bị khóa',
                         f'Hệ thống đang chạy ({self.state.name}) — nhấn STOP trước')
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
                    def _do_home():
                        with self._servo_lock:
                            self._ensure_ready(mot, sid, timeout=5.0)
                            mot.referencing_task(nonblocking=False)
                        if sid not in self.zero_offset:
                            with self._servo_lock:
                                self.zero_offset[sid] = mot.current_position()
                        self.get_logger().info(f"✅ S{sid} homed (manual)")
                    threading.Thread(target=_do_home, daemon=True).start()
                return
            if cmd0 == 'clear':
                sid = int(parts[1])
                mot = self.servos.get(sid)
                if mot:
                    with self._servo_lock:
                        mot.acknowledge_faults()
                self.get_logger().info(f"✅ S{sid} faults cleared")
                return
            sid = int(parts[0])
            d   = parts[1]
            vel = int(parts[2]) if len(parts) > 2 else 30
            if d == 'stop':
                self._stop(sid)
            elif d == '+':
                self._jog(sid, vel)
            elif d == '-':
                self._jog(sid, -vel)
        except Exception:
            pass

    def _cb_change_tray_input(self, msg: String):
        """
        Lệnh thay khay thủ công từ GUI (cả 2 mode).
        Equivalent với robot gửi robot_done → cho phép vào STATE 2.
        """
        cmd = msg.data.strip().lower()
        if cmd in ('1', 'true', 'change', 'done'):
            self.get_logger().info("📦 change_tray_input → _robot_done = True")
            self._robot_done = True
            self._notify('info', '📦 Lệnh thay khay', 'robot_done set — vào STATE 2 nếu S13 ON')
        elif cmd in ('0', 'false', 'reset'):
            self._robot_done = False
            self.get_logger().info("🔄 change_tray_input reset → _robot_done = False")

    def _cb_sim(self, msg: String):
        """
        Mô phỏng sensor — chỉ hoạt động ở MANUAL mode.
        Format: 'N:1'|'N:0'|'all:1'|'all:0'|'clear'
        """
        if self.operation_mode != 'manual':
            self._notify('warn', '⚠️ Sim sensor bị khóa',
                         'Chuyển sang MANUAL mode để mô phỏng sensor')
            return
        # Sim sensor có thể dùng cả auto và manual — chỉ là forced-OR relay
        cmd = msg.data.strip()
        if cmd == 'clear':
            self._sim_sensors.clear()
            self.get_logger().info("🧹 Sim sensors cleared")
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
        """
        GUI gửi lệnh nhảy state — dùng trong MANUAL mode.

        MANUAL mode:
          • 'STATE1' → kiểm tra điều kiện (S1/S2/S3 + S12) → vào S1_CONFIRM_SAFE nếu đủ
          • 'STATE2' → kiểm tra điều kiện (robot_done + S13) → vào S2A_CHECK_INTERLOCK nếu đủ
          • 'HOMING' → homing
          • 'IDLE'   → dừng về IDLE

        AUTO mode:
          • 'HOMING' và 'IDLE' vẫn được phép
          • 'STATE1'/'STATE2' bị từ chối (auto tự quản)
        """
        cmd = msg.data.strip().upper()

        # Lệnh HOMING và IDLE luôn được phép
        if cmd == 'HOMING':
            if self.state not in (SystemState.HOMING_RUNNING,):
                self.zero_offset.clear()
                self._inx_moving = self._iny_moving = False
                self._enter(SystemState.HOMING)
                self._notify('info', '🏠 HOMING (manual)', '')
            return
        if cmd == 'IDLE':
            for sid in list(self.servos):
                self._stop(sid)
            self._enter(SystemState.IDLE)
            self._notify('info', '⏹ IDLE', '')
            return

        # STATE1 / STATE2 chỉ cho MANUAL mode
        if self.operation_mode != 'manual':
            self._notify('warn', '⚠️ goto_state bị khóa',
                         'Chỉ dùng trong MANUAL mode — AUTO tự quản lý state')
            return

        if cmd in ('STATE1', 'STATE_1', 'S1'):
            if self.state not in (SystemState.IDLE, SystemState.ERROR):
                self._notify('warn', '⚠️ Không thể vào STATE 1',
                             f'Đang chạy ({self.state.name}) — nhấn STOP trước')
                return
            if not self.zero_offset:
                self._notify('warn', '⚠️ Chưa home', 'Homing trước khi chạy STATE 1')
                return
            # Kiểm tra điều kiện giống auto
            if not self.sensor(12):
                self._notify('warn', '⚠️ Vị trí cấp đang có khay',
                             'S12 OFF — chờ robot lấy khay hoặc sim S12=1')
                return
            self.get_logger().info("▶ [MANUAL] goto STATE 1 — vào ngay, chờ S1/S2/S3 trong process")
            self._notify('info', '▶ STATE 1 (manual)',
                         'Đã vào State 1. S1/S2/S3 chưa ON sẽ chờ tại Step 2.')
            self._state1_enabled = True
            self._enter(SystemState.S1_CONFIRM_SAFE)

        elif cmd in ('STATE2', 'STATE_2', 'S2', 'STATE2A', 'S2A'):
            # Cho phép vào STATE 2 từ IDLE, ERROR, hoặc S1_COMPLETE
            allowed = (SystemState.IDLE, SystemState.ERROR, SystemState.S1_COMPLETE)
            if self.state not in allowed:
                self._notify('warn', '⚠️ Không thể vào STATE 2',
                             f'Đang chạy ({self.state.name}) — nhấn STOP trước hoặc đợi State 1 xong')
                return
            if not self.zero_offset:
                self._notify('warn', '⚠️ Chưa home', 'Homing trước khi chạy STATE 2')
                return
            # Điều kiện: robot_done + S13 ON
            # Ở manual, cho phép sim robot_done bằng cách không bắt buộc _robot_done flag
            # (operator tự confirm bằng sim S13 ON)
            if not self.sensor(13):
                self._notify('warn', '⚠️ Không có khay tại robot',
                             'S13 OFF — sim S13=1 hoặc chờ robot gắn khay')
                return
            self._robot_done = False   # reset flag nếu có
            self.get_logger().info("▶ [MANUAL] goto STATE 2A — S13 OK")
            self._notify('info', '▶ STATE 2A (manual)', 'S13 ON — bắt đầu thay khay')
            self._enter(SystemState.S2A_CHECK_INTERLOCK)

        else:
            self._notify('warn', f'goto_state: không nhận dạng được "{cmd}"', '')

    # ══════════════════════════════════════════════════════════════
    # Control Loop
    # ══════════════════════════════════════════════════════════════

    def _control_loop(self):
        self._watchdog_last_tick = time.time()
        if self._system_paused:
            return
        self._process_state()
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.pub_state.publish(msg)

    def _publish_positions(self):
        try:
            pos = {}
            for sid in self.servos:
                p = self._pos(sid)
                if p is not None:
                    pos[str(sid)] = round(p, 2)
            msg = String()
            msg.data = json.dumps(pos)
            self.pub_servo_pos.publish(msg)
        except Exception:
            pass

    def _watchdog(self):
        gap = time.time() - self._watchdog_last_tick
        if gap > 3.0:
            self.get_logger().error(f"🚨 WATCHDOG: loop silent {gap:.1f}s!")
            self._notify('error', '🚨 Loop treo!', f'{gap:.1f}s không tick')

    # ══════════════════════════════════════════════════════════════
    # Main State Dispatcher
    # ══════════════════════════════════════════════════════════════

    def _process_state(self):
        s = self.state

        if   s == SystemState.IDLE:           self._do_idle()
        elif s == SystemState.HOMING:         self._do_homing()
        elif s == SystemState.HOMING_RUNNING: pass
        elif s == SystemState.ERROR:          self._do_error()

        # STATE 1
        elif s == SystemState.S1_CONFIRM_SAFE:     self._s1_confirm_safe()
        elif s == SystemState.S1_INX_MOVE:         self._s1_inx_move()
        elif s == SystemState.S1_WAIT_ARRIVE:      self._s1_wait_arrive()
        elif s == SystemState.S1_INY_JOG:          self._s1_iny_jog()
        elif s == SystemState.S1_INY_TO_ROW:       self._s1_iny_to_row()
        elif s == SystemState.S1_CHECK_S5:         self._s1_check_s5()
        elif s == SystemState.S1_CHECK_S5_ROW1:   self._s1_check_s5_row1()
        elif s == SystemState.S1_WAIT_GUI_CONFIRM: self._s1_wait_gui_confirm()
        elif s == SystemState.S1_RETRY_INX_500:   self._s1_retry_inx_500()
        elif s == SystemState.S1_RETRY_JOG:       self._s1_retry_jog()
        elif s == SystemState.S1_CYL1_EXTEND:     self._s1_cyl1_extend()
        elif s == SystemState.S1_CHECK_S13:       self._s1_check_s13()
        elif s == SystemState.S1_WAIT_S12_SAFE:   self._s1_wait_s12_safe()
        elif s == SystemState.S1_INY_50_CYL2:     self._s1_iny_50_cyl2()
        elif s == SystemState.S1_WAIT_S13_ON:     self._s1_wait_s13_on()
        elif s == SystemState.S1_INY_200:         self._s1_iny_200()
        elif s == SystemState.S1_PLACE_DELAY:     self._s1_place_delay()
        elif s == SystemState.S1_WAIT_RELEASE:    self._s1_wait_release()
        elif s == SystemState.S1_INY_50_FINAL:    self._s1_iny_50_final()
        elif s == SystemState.S1_INX_HOME:        self._s1_inx_home()
        elif s == SystemState.S1_COMPLETE:        self._s1_complete()

        # STATE 2A
        elif s == SystemState.S2A_CHECK_INTERLOCK: self._s2a_check_interlock()
        elif s == SystemState.S2A_INX_500:         self._s2a_inx_500()
        elif s == SystemState.S2A_INY_200_CYL1:   self._s2a_iny_200_cyl1()
        elif s == SystemState.S2A_WAIT_S11:        self._s2a_wait_s11()
        elif s == SystemState.S2A_INY_10:          self._s2a_iny_10()
        elif s == SystemState.S2A_INX_10:          self._s2a_inx_10()
        elif s == SystemState.S2A_INY_JOG_OUTPUT:  self._s2a_iny_jog_output()
        elif s == SystemState.S2A_INY_OUTPUT_ROW:  self._s2a_iny_output_row()
        elif s == SystemState.S2A_WAIT_S10:        self._s2a_wait_s10()
        elif s == SystemState.S2A_INY_10_FINAL:    self._s2a_iny_10_final()
        elif s == SystemState.S2A_INX_20:          self._s2a_inx_20()
        elif s == SystemState.S2A_WAIT_CYL2_RETRACT: self._s2a_wait_cyl2_retract()
        elif s == SystemState.S2A_COMPLETE:        self._s2a_complete()

    # ── Helpers: entry-condition checks ──────────────────────────

    def _can_start_s1(self) -> bool:
        """
        Điều kiện để vào STATE 1 (cấp khay):
          • Băng tải còn khay : S1 OR S2 OR S3 ON
          • Vị trí cấp khay trống: S12 ON (Cyl2 retracted = chỗ đặt trống)
          • Đã homing xong    : zero_offset không rỗng
        """
        if not self.zero_offset:
            return False
        has_tray_on_belt = self.sensor(1) or self.sensor(2) or self.sensor(3)
        place_is_empty   = self.sensor(12)    # S12 ON = Cyl2 retracted = vị trí đặt trống
        return has_tray_on_belt and place_is_empty

    def _can_start_s2a(self) -> bool:
        """
        Điều kiện để vào STATE 2A (thay khay):
          • Robot đã done_tray  : _robot_done flag
          • Robot đang IDLE     : không busy
          • Có khay tại robot   : S13 ON (Cyl2 extended = đang giữ khay)
          • Đã homing xong
        """
        return (self.zero_offset and
                self._robot_done and
                self.sensor(13))   # S13 ON = có khay ở vị trí robot

    # ── Common states ─────────────────────────────────────────────

    def _do_idle(self):
        """
        AUTO  mode: Tự động theo dõi điều kiện, tự trigger STATE1/STATE2 khi đủ.
        MANUAL mode: Đứng yên chờ operator chọn STATE1/STATE2 từ GUI.
                     JOG và sim sensor được phép.
        """
        if not self.zero_offset:
            self._log_once("IDLE_NOT_HOMED",
                           "⏳ IDLE: Chưa home — nhấn START để bắt đầu")
            return

        if self.operation_mode == 'manual':
            # Manual: chỉ hiển thị guide, không tự trigger
            self._log_once("IDLE_MANUAL",
                           "🖐 MANUAL IDLE — chọn STATE 1 hoặc STATE 2 từ GUI để bắt đầu. "
                           "JOG và sim sensor đang được phép.")
            return

        # ─── AUTO mode: auto-watch ────────────────────────────────

        # Ưu tiên 1: robot done → thay khay (STATE 2)
        # Cyl2 không bao giờ retract khi có khay (cơ khí) → S13 luôn ON khi robot_done
        if self._can_start_s2a():
            self._robot_done = False
            self.get_logger().info("🔄 AUTO IDLE: Robot done → STATE 2 Thay khay")
            self._enter(SystemState.S2A_CHECK_INTERLOCK)
            return

        # Ưu tiên 2: băng tải có khay + vị trí cấp trống → cấp khay
        if self._state1_enabled and self._can_start_s1():
            self.get_logger().info(
                f"▶ AUTO IDLE: S1={self.sensor(1)} S2={self.sensor(2)} "
                f"S3={self.sensor(3)} S12={self.sensor(12)} → STATE 1 Cấp khay"
            )
            self._enter(SystemState.S1_CONFIRM_SAFE)
            return

        # Chờ — log guide
        s1s2s3 = self.sensor(1) or self.sensor(2) or self.sensor(3)
        s12    = self.sensor(12)
        if not s1s2s3:
            self._log_once("IDLE_NO_TRAY",
                           "⏳ AUTO IDLE: S1/S2/S3 OFF — băng tải hết khay, "
                           "nạp khay vào để tiếp tục")
        elif not s12:
            self._log_once("IDLE_NO_PLACE",
                           "⏳ AUTO IDLE: S12 OFF — vị trí cấp đang có khay, "
                           "chờ robot lấy xong")

    def _do_homing(self):
        self._enter(SystemState.HOMING_RUNNING)
        def _bg():
            ok = self._home_all()
            if ok:
                self.get_logger().info("✅ Homing complete")
                self._notify('info', '✅ Homing xong', '')
                # Sau homing: check điều kiện → tự chạy, còn không thì IDLE tự watch
                if self._can_start_s1():
                    self.get_logger().info("▶ Post-home: đủ điều kiện → STATE 1 Cấp khay")
                    self._enter(SystemState.S1_CONFIRM_SAFE)
                else:
                    s1s2s3 = self.sensor(1) or self.sensor(2) or self.sensor(3)
                    reason = "S1/S2/S3 OFF — băng tải chưa có khay" if not s1s2s3 \
                             else "S12 OFF — vị trí cấp đang có khay"
                    self.get_logger().info(f"⏳ Post-home: {reason} → IDLE tự theo dõi")
                    self._notify('info', '✅ Homed — Đang chờ',
                                 'Sẽ tự động chạy khi băng tải có khay + S12 ON')
                    self._enter(SystemState.IDLE)
            else:
                self._error("Homing thất bại")
        threading.Thread(target=_bg, daemon=True).start()

    def _do_error(self):
        self._log_once("ERROR_STATE", "⛔ ERROR — kiểm tra lỗi rồi nhấn STOP → START")

    # ══════════════════════════════════════════════════════════════
    # ─── STATE 1: Cấp khay Input ─────────────────────────────────
    # ══════════════════════════════════════════════════════════════

    # ── Step 1: CONFIRM SAFE ─────────────────────────────────────
    def _s1_confirm_safe(self):
        """INY phải ≤ 10mm. Nếu > 10mm → về 10mm trước."""
        # Reset robot_done khi bắt đầu STATE 1 — tránh flag cũ trigger S2A sớm
        if not self._cmd_sent and not self._inx_moving and not self._iny_moving:
            if self._robot_done:
                self.get_logger().info("🔄 S1 Step1: reset _robot_done (flag cũ)")
                self._robot_done = False

        iny = self._pos(2)
        if iny is None:
            return
        if iny <= self.config.iny_home + 2.0:
            # INY đã safe
            self._enter(SystemState.S1_INX_MOVE)
        else:
            if not self._cmd_sent:
                ok = self._nb_move(2, self.config.iny_home)
                if not ok:
                    self._log_once("S1_INY_HOME_FAIL",
                                   "⚠️ S1 Step1: INY home thất bại — thử lại vòng tới")
                    return
                self._cmd_sent = True
                self._step_timeout = time.time() + self.config.move_timeout
                self._log_once("S1_HOME_INY", f"⏬ INY → {self.config.iny_home}mm")
            else:
                if time.time() > self._step_timeout:
                    # Timeout → retry move thay vì error
                    self.get_logger().warn("⚠️ S1 Step1: INY timeout → retry move")
                    self._cmd_sent = False
                elif self._arrived(2):
                    self._enter(SystemState.S1_INX_MOVE)

    # ── Step 2: INX → 500mm (non-blocking) ───────────────────────
    def _s1_inx_move(self):
        """
        Điều kiện: (S1 OR S2 OR S3 ON) + INY ≤ 50mm.
        INX → 500mm non-blocking — luôn đi vào 500mm bất kể S3 đã ON sẵn hay chưa.
        Sau khi INX DỪng tại 500mm mới check S3 và cho INY di chuyển.
        """
        # Chờ S1/S2/S3 ON trước khi INX di chuyển
        if not (self.sensor(1) or self.sensor(2) or self.sensor(3)):
            self._log_once(
                "S1_WAIT_BELT",
                "⏳ Step2: Chờ S1/S2/S3 ON — đặt khay lên băng tải hoặc sim sensor\n"
                f"  Hiện tại: S1={self.sensor(1)} S2={self.sensor(2)} S3={self.sensor(3)}"
            )
            self._notify('info', '⏳ Chờ khay (S1/S2/S3)',
                         'Đặt khay lên băng tải để tiếp tục.')
            return
        if not self._iny_safe():
            self._log_once("S1_WAIT_INY_SAFE", "⏳ Step2: INY chưa ≤ 50mm")
            return

        if not self._cmd_sent:
            ok = self._nb_move(1, self.config.inx_target)   # 500mm
            if not ok:
                self._log_once("S1_INX_MOVE_FAIL",
                               "⚠️ S1 Step2: INX → 500mm thất bại — thử lại vòng tới")
                return
            self._cmd_sent    = True
            self._inx_arrived = False
            # Timer 30s bắt đầu SAU KHI INX dừng, đặt ở đây để tránh timer sớm
            self._30s_timeout = 0.0
            self.get_logger().info(
                f"▶ Step2: INX → {self.config.inx_target}mm (non-blocking) "
                f"| S1={self.sensor(1)} S2={self.sensor(2)} S3={self.sensor(3)}"
            )
            self._enter(SystemState.S1_WAIT_ARRIVE)

    # ── Step 3: Chờ INX dừng → rồi mới check S3 + S10 ──────────
    def _s1_wait_arrive(self):
        """
        QUAN TRỌNG: INX phải dừng hoàn toàn tại 500mm TRƯỚC KHI check S3.
        Dù S3 đã ON sẵn, INY vẫn KHÔNG được di chuyển cho đến khi INX dừng.

        Sau khi INX dừng:
          • S3 ON + S10 ON → cho INY jog
          • S3 OFF → bắt đầu đếm 30s; hết timeout → INX về home, retry
        """
        # Bước 1: Chờ INX dừng hoàn toàn
        if not self._inx_arrived:
            self._inx_arrived = self._arrived(1)
            if not self._inx_arrived:
                self._log_once("S1_INX_MOVING",
                               "⏳ INX đang di chuyển về 500mm — INY bị BLOCK cho đến khi INX dừng")
                return
            # INX vừa dừng → bắt đầu 30s timer TỪ ĐÂY
            self._30s_timeout = time.time() + 30.0
            self.get_logger().info("✅ INX dừng tại 500mm — bắt đầu check S3 + S10 (timeout 30s)")

        # Bước 2: INX đã dừng → check S3 + S10
        s3  = self.sensor(3)
        s10 = self.sensor(10)

        if s3 and s10:
            self.get_logger().info("✅ S3 ON + S10 ON → jog INY tìm khay")
            self._s4_armed = False
            self._enter(SystemState.S1_INY_JOG)
            return

        if not s3:
            if time.time() > self._30s_timeout:
                self.get_logger().warn("⏱️ 30s — S3 không ON, INX về home, retry")
                self._notify('warn', '⚠️ S3 timeout',
                             'Không có khay tại cuối băng tải sau 30s — quay đầu retry')
                self._nb_move(1, self.config.inx_home)
                self._enter(SystemState.S1_CONFIRM_SAFE)
                return
            remain = self._30s_timeout - time.time()
            self._log_once("S1_WAIT_S3",
                           f"⏳ INX dừng, chờ S3 ON (còn {remain:.0f}s). Kích: '3:1'")
        else:
            # S3 ON nhưng S10 OFF
            self._log_once("S1_WAIT_S10",
                           "⏳ S3 ON, chờ S10 ON (Cyl1 phải retracted). Kích: '10:1'")

    # ── Step 4: INY jog tìm khay ─────────────────────────────────
    def _s1_iny_jog(self):
        """
        INY di chuyển đến vị trí Row 1 (row cuối, xa nhất) tìm S4.
        S4 chỉ xét khi INY ≥ 150mm.
        S4 ON → dừng + snap row gần nhất theo hướng +.
        Đến Row 1 mà S4 OFF:
          S5 ON  → cảnh báo lỗi sensor S4 (khay có nhưng S4 không trigger)
          S5 OFF → không có khay → reset S1_CONFIRM_SAFE.
        """
        iny = self._pos(2)
        if iny is None:
            return

        # Bắt đầu move một lần duy nhất đến vị trí Row 1 (max)
        if not self._cmd_sent:
            row1_pos = max(self.config.iny_input_stack.values())
            ok = self._nb_move(2, row1_pos)
            if not ok:
                self._log_once("S1_INY_JOG_FAIL",
                               "⚠️ S1 Step4: INY move thất bại — thử lại vòng tới")
                return
            self._cmd_sent      = True
            self._row1_pos      = row1_pos
            self._step_timeout  = time.time() + self.config.move_timeout
            self.get_logger().info(
                f"▶ INY → Row1 ({row1_pos:.0f}mm) tìm S4 (arm ≥ {INY_S4_ARM_MM}mm)"
            )

        # Arm S4 sau 150mm
        if iny >= INY_S4_ARM_MM:
            self._s4_armed = True

        # S4 ON → snap row gần nhất theo hướng +
        if self._s4_armed and self.sensor(4):
            self._stop(2)
            current = self._pos(2) or iny
            self._current_row = self._find_nearest_row(current, self.config.iny_input_stack)
            self.get_logger().info(f"✅ S4 ON tại {current:.1f}mm → Row {self._current_row}")
            self._s5_retry = 0
            self._enter(SystemState.S1_INY_TO_ROW)
            return

        # Timeout hoặc đã đến Row 1 → S4 vẫn OFF
        row1_pos = getattr(self, '_row1_pos', max(self.config.iny_input_stack.values()))
        at_row1  = self._arrived(2) or iny >= row1_pos - 2.0
        timed_out = time.time() > self._step_timeout

        if at_row1 or timed_out:
            self._stop(2)
            if self.sensor(5):
                # Có khay nhưng S4 không trigger → S4 lỗi
                self.get_logger().error(
                    f"🚨 S4 FAULT: INY tại {iny:.1f}mm, S5=ON nhưng S4=OFF — sensor S4 hỏng?"
                )
                self._notify('warn', '⚠️ Lỗi Sensor S4',
                             'Có khay (S5=ON) nhưng S4 không phát hiện — kiểm tra S4. Nhấn XÁC NHẬN để tiếp tục.')
                # Không hard error — về home và cho operator xác nhận
                self._nb_move(2, self.config.iny_home)
                self._nb_move(1, self.config.inx_home)
                self._gui_confirmed = False
                self._enter(SystemState.S1_WAIT_GUI_CONFIRM)
            else:
                self.get_logger().warn(f"⚠️ INY tại Row1 ({iny:.1f}mm), S4+S5 đều OFF — không có khay")
                self._notify('warn', '⚠️ Không tìm thấy khay', 'INY đến Row1 không có S4/S5 → reset')
                self._nb_move(2, self.config.iny_home)
                self._nb_move(1, self.config.inx_home)
                self._enter(SystemState.S1_CONFIRM_SAFE)
            return

        self._log_once("S1_JOGGING",
                       f"🔄 INY → Row1 ({row1_pos:.0f}mm), chờ S4 (arm ≥ {INY_S4_ARM_MM}mm)")


    # ── Step 5: INY → row position ───────────────────────────────
    def _s1_iny_to_row(self):
        """Di chuyển INY đến iny_input_stack[row]."""
        target = self.config.iny_input_stack.get(self._current_row)
        if target is None:
            # Clamp row về range hợp lệ trong config (Row 1 max, Row 8 min)
            valid_rows = sorted(self.config.iny_input_stack.keys())  # [1,2,...,8]
            if not valid_rows:
                self._log_once("S1_NO_ROW", "❌ Không có row nào trong config!")
                return
            # Clamp: nếu nhỏ hơn min → dùng min, nếu lớn hơn max → dùng max
            clamped = max(valid_rows[0], min(valid_rows[-1], self._current_row))
            self.get_logger().warn(
                f"⚠️ S1 Step5: row {self._current_row} không có config "
                f"→ clamp → row {clamped} (range row{valid_rows[0]}~row{valid_rows[-1]})"
            )
            self._current_row = clamped
            target = self.config.iny_input_stack[self._current_row]
        if not self._cmd_sent:
            self.get_logger().info(f"▶ INY → {target}mm (row {self._current_row})")
            ok = self._nb_move(2, target)
            if not ok:
                self._log_once("S1_INY_ROW_FAIL",
                               f"⚠️ S1 Step5: INY → {target}mm thất bại — thử lại")
                return
            self._cmd_sent = True
            self._step_timeout = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout:
                # Timeout → retry move (reset cmd_sent)
                self.get_logger().warn("⚠️ S1 Step5: INY timeout → retry move")
                self._cmd_sent = False
            elif self._arrived(2):
                # Bắt đầu đếm 3s check S5
                self._step_start  = time.time()
                self._enter(SystemState.S1_CHECK_S5)

    # ── Step 6a: Check S5 ────────────────────────────────────────
    def _s1_check_s5(self):
        """
        Sau khi INY đến row, chờ tối đa 5s xem S5 có ON không.
        S5 ON  → extend Cyl1 → tiếp tục.
        S5 OFF sau 5s → INY+INX về home → reset S1_CONFIRM_SAFE.
        """
        if not self._cmd_sent:
            self._step_start = time.time()
            self._cmd_sent   = True

        if self.sensor(5):
            self.get_logger().info("✅ S5 ON — có khay tại row, extend Cyl1")
            self._cyl1_extend()
            self._step_timeout = time.time() + CYLINDER_TIMEOUT_S
            self._enter(SystemState.S1_CYL1_EXTEND)
            return

        elapsed   = time.time() - self._step_start
        remaining = max(0.0, S5_WAIT_S - elapsed)
        self._log_once("S1_WAIT_S5", f"⏳ Chờ S5 ON (còn {remaining:.1f}s) tại row {self._current_row}")

        if elapsed >= S5_WAIT_S:
            self.get_logger().warn(
                f"⚠️ S5 OFF sau {S5_WAIT_S:.0f}s tại row {self._current_row} → reset về S1"
            )
            self._notify('warn', '⚠️ Không phát hiện khay',
                         f'S5 OFF sau {S5_WAIT_S:.0f}s tại row {self._current_row} — reset lại')
            self._nb_move(2, self.config.iny_home)   # INY → home
            self._nb_move(1, self.config.inx_home)   # INX → home
            self._enter(SystemState.S1_CONFIRM_SAFE)


    # ── Step 6b: Check S5 tại row+1 ─────────────────────────────
    # ── Step 6b: Check S5 tại row+1 ─────────────────────────────
    def _s1_check_s5_row1(self):
        """Chờ 3s tại row+1 để check S5."""
        if not self._cmd_sent:
            self._step_start = time.time()
            self._cmd_sent   = True

        if self.sensor(5):
            self.get_logger().info(f"✅ S5 ON tại row {self._current_row} — extend Cyl1")
            self._cyl1_extend()
            self._step_timeout = time.time() + CYLINDER_TIMEOUT_S
            self._enter(SystemState.S1_CYL1_EXTEND)
            return

        if time.time() - self._step_start >= S5_WAIT_S:
            self.get_logger().warn(f"⚠️ S5 OFF cả 2 lần (row+1) — show GUI confirm")
            self._go_gui_confirm()

    def _go_gui_confirm(self):
        """INY → home + INX → home + hiện popup."""
        self.get_logger().warn("⚠️ S4/S5 fail — về home + chờ GUI confirm")
        self._nb_move(2, self.config.iny_home)
        self._step_timeout = time.time() + self.config.move_timeout
        self._gui_confirmed = False
        self._enter(SystemState.S1_WAIT_GUI_CONFIRM)
        self._notify(
            'error',
            '⚠️ Kiểm tra S4 hoặc S5!',
            'Cảm biến không phát hiện khay. Kiểm tra và nhấn XÁC NHẬN.'
        )

    # ── Step 6c: Chờ GUI confirm ─────────────────────────────────
    # ── Step 6c: Chờ GUI confirm ─────────────────────────────────
    def _s1_wait_gui_confirm(self):
        """
        Trong khi chờ:
          - Đợi INY về safe → INX → home
          - Hiện popup trên GUI
        Sau xác nhận → kiểm tra S1/S2/S3.
        """
        # Đợi INY về safe trước rồi mới INX về
        if not self._cmd_sent:
            if self._iny_safe():
                ok = self._nb_move(1, self.config.inx_home)  # INX → 20mm
                if ok:
                    self._cmd_sent = True
        # GUI confirm nhận được
        if self._gui_confirmed:
            self._gui_confirmed = False
            self._stop(1)
            self._stop(2)
            self.get_logger().info("✅ GUI confirmed — kiểm tra S1/S2/S3")
            self._enter(SystemState.S1_RETRY_JOG)

        self._log_once("S1_GUI_WAIT",
                       "📺 Chờ nhấn XÁC NHẬN trên GUI để tiếp tục")

    # ── Step 6d: Retry sau GUI confirm ───────────────────────────
    def _s1_retry_jog(self):
        """
        Kiểm tra S1/S2/S3:
          - Bất kỳ ON → kiểm tra INX vị trí → jog lại
          - Tất cả OFF → cảnh báo hết khay
        """
        if not (self.sensor(1) or self.sensor(2) or self.sensor(3)):
            self._log_once("S1_NO_TRAY_RETRY", "⚠️ Hết khay — S1/S2/S3 đều OFF")
            self._notify('warn', '⚠️ Hết khay', 'Nạp khay vào stack rồi nhấn START')
            return

        inx = self._pos(1)
        if inx is None:
            return

        at_target = abs(inx - self.config.inx_target) < 15.0

        if at_target:
            # INX đã ở 500mm → jog ngay
            self.get_logger().info("▶ INX đã ở 500mm → jog INY tìm khay")
            self._s4_armed  = False
            self._s5_retry  = 0
            self._enter(SystemState.S1_INY_JOG)
        else:
            # INX cần về 500mm trước
            self._enter(SystemState.S1_RETRY_INX_500)

    def _s1_retry_inx_500(self):
        """INX → 500mm (non-blocking), sau đó vào jog."""
        if not self._cmd_sent:
            if not self._iny_safe():
                self._log_once("S1_RETRY_WAIT_INY", "⏳ Chờ INY safe trước khi INX đi 500mm")
                return
            ok = self._nb_move(1, self.config.inx_target)
            if not ok:
                self._log_once("S1_RETRY_INX_FAIL",
                               "⚠️ S1 Retry: INX → 500mm thất bại — thử lại vòng tới")
                return
            self._cmd_sent    = True
            self._step_timeout = time.time() + self.config.move_timeout
            self._inx_arrived = False
        else:
            if time.time() > self._step_timeout:
                self.get_logger().warn("⚠️ S1 Retry: INX timeout → retry move")
                self._cmd_sent = False
            elif self._arrived(1):
                self.get_logger().info("✅ INX đến 500mm — jog INY")
                self._s4_armed = False
                self._s5_retry = 0
                self._enter(SystemState.S1_INY_JOG)

    # ── Step 7: Cyl1 extend + chờ S11 ────────────────────────────
    def _s1_cyl1_extend(self):
        """Chờ S11 ON — không timeout, Cyl1 giữ đến khi gắp được khay."""
        if self.sensor(11):
            self.get_logger().info("✅ S11 ON — Cyl1 gắp khay OK → đảm bảo Cyl2 retract")
            self._enter(SystemState.S1_CHECK_S13)
            return
        self._log_once("S1_WAIT_S11", "⏳ Chờ S11 ON (Cyl1 extend). Kích: '11:1'")

    # ── Step 7b: Đảm bảo Cyl2 retract trước khi INY về ──────────
    def _s1_check_s13(self):
        """
        Sau S11 ON: luôn kiểm tra S12 (Cyl2 retract) trước khi INY di chuyển.
        S12 ON  → Cyl2 đã retract → INY an toàn về 50mm.
        S12 OFF → Chờ S12 ON (phải retract Cyl2 trước).
        """
        if self.sensor(12):
            self.get_logger().info("✅ S12 ON (Cyl2 retract) → INY về 50mm")
            self._enter(SystemState.S1_INY_50_CYL2)
        else:
            self.get_logger().info("⚠️ S12 OFF — Cyl2 chưa retract. Đợi S12 ON")
            self._enter(SystemState.S1_WAIT_S12_SAFE)

    # ── Step 7c: Chờ S12 ON (Cyl2 về) ───────────────────────────
    def _s1_wait_s12_safe(self):
        """Chờ S12 ON → Cyl2 đã retract → INY được phép về safe."""
        if self.sensor(12):
            self.get_logger().info("✅ S12 ON — Cyl2 đã về, INY tự do")
            self._enter(SystemState.S1_INY_50_CYL2)
        else:
            self._log_once("S1_WAIT_S12", "⏳ Chờ S12 ON (Cyl2 retract). Kích: '12:1'")

    # ── Step 8: INY → 50mm (S12 phải ON trước) + Cyl2 extend ────
    def _s1_iny_50_cyl2(self):
        """
        BẮT BUỘC S12 ON (Cyl2 retract) trước khi INY di chuyển.
        Nếu S12 OFF → dừng INY ngay, log cảnh báo, chờ S12 ON.
        Sau khi INY tại 50mm → Cyl2 EXTEND → chờ S13 ON.
        """
        # ── Interlock: S12 phải ON ──────────────────────────────
        if not self.sensor(12):
            if self._cmd_sent:
                self._stop(2)
                self._cmd_sent = False
                self.get_logger().warn(
                    "⚠️ [INTERLOCK] S12 OFF trong khi INY đang chạy — DỪNG INY ngay!"
                )
            self._log_once(
                "S1_S12_INTERLOCK",
                "🔒 INTERLOCK ACTIVE: S12=OFF (Cyl2 chưa retract) → INY bị khóa.\n"
                "  Kiểm tra: Cyl2 có đang extend? Van khí? Cảm biến S12 bị lỗi?"
            )
            self._notify('warn', '⚠️ S12 chưa ON',
                         'Cyl2 chưa retract — INY bị khóa. Kiểm tra S12/Cyl2.')
            return

        # S12 ON — an toàn, tiếp tục
        if not self._cmd_sent:
            ok = self._nb_move(2, self.config.iny_safe_zone)
            if not ok:
                self.get_logger().warn("⚠️ S1 Step8: INY → 50mm thất bại — thử lại")
                return
                return
            self._cmd_sent     = True
            self._step_timeout = time.time() + self.config.move_timeout
            self.get_logger().info(
                f"▶ INY → {self.config.iny_safe_zone}mm (safe zone) | S12={self.sensor(12)}"
            )
        else:
            if time.time() > self._step_timeout:
                self.get_logger().warn("⚠️ S1 Step8: INY → 50mm timeout → retry")
                self._cmd_sent = False
            elif self._arrived(2):
                self.get_logger().info("✅ INY tại 50mm → Cyl2 EXTEND (ch9)")
                self._cyl2_extend()
                self._enter(SystemState.S1_WAIT_S13_ON)


    # ── Step 9: Chờ S13 ON (Cyl2 đang đỡ khay) ──────────────────
    def _s1_wait_s13_on(self):
        """Chờ S13 ON xác nhận Cyl2 đã extend và đang đỡ khay.
        Không timeout — Cyl1 giữ vô thời hạn cho đến khi Cyl2 xác nhận."""
        if self.sensor(13):
            self.get_logger().info("✅ S13 ON — Cyl2 đang đỡ → INY → 200mm")
            self._enter(SystemState.S1_INY_200)
            return
        self._log_once("S1_WAIT_S13", "⏳ Chờ S13 ON (Cyl2 đỡ khay). Kích: '13:1'")

    # ── Step 10: INY → 200mm → đặt khay lên Cyl2 Hold Tray ────────────────
    def _s1_iny_200(self):
        """INY → 200mm để đặt khay lên Cyl2 Hold Tray. Đến nơi → chờ 1s."""
        if not self._cmd_sent:
            ok = self._nb_move(2, self.config.iny_place)  # 200mm
            if not ok:
                self.get_logger().warn("⚠️ S1 Step10: INY → 200mm thất bại — thử lại")
                return
                return
            self._cmd_sent     = True
            self._step_timeout = time.time() + self.config.move_timeout
            self.get_logger().info("▶ INY → 200mm (đặt khay lên Cyl2 Hold Tray)")
        else:
            if time.time() > self._step_timeout:
                self.get_logger().warn("⚠️ S1 Step10: INY → 200mm timeout → retry")
                self._cmd_sent = False
            elif self._arrived(2):
                self.get_logger().info("✅ INY tại 200mm → chờ 1s → retract Cyl1")
                self._step_start = time.time()
                self._enter(SystemState.S1_PLACE_DELAY)

    # ── Step 10 (cont): 1s delay rồi retract Cyl1 ───────────────
    def _s1_place_delay(self):
        """Chờ 1 giây → retract Cyl1 (ch4)."""
        if not self._cmd_sent:
            self._step_start = time.time()
            self._cmd_sent   = True
        if time.time() - self._step_start >= 1.0:
            self.get_logger().info("◀ Cyl1 RETRACT (ch4)")
            self._cyl1_retract()
            self._enter(SystemState.S1_WAIT_RELEASE)

    # ── Step 11: Chờ S10 ON + S11 OFF ────────────────────────────
    def _s1_wait_release(self):
        """S10 ON = Cyl1 retracted. S11 OFF = Cyl1 released. Cả 2 → nhả hoàn toàn."""
        s10 = self.sensor(10)
        s11 = self.sensor(11)
        if s10 and not s11:
            self.get_logger().info("✅ S10 ON + S11 OFF — Cyl1 nhả hoàn toàn")
            self._enter(SystemState.S1_INY_50_FINAL)
            return
        self._log_once("S1_WAIT_RELEASE", "⏳ Chờ S10 ON + S11 OFF. Kích: '10:1' '11:0'")

    # ── Step 12: INY → 50mm ──────────────────────────────────────
    def _s1_iny_50_final(self):
        """INY về safe (50mm) trước khi INX về."""
        if not self._cmd_sent:
            ok = self._nb_move(2, self.config.iny_safe_zone)
            if not ok:
                self.get_logger().warn("⚠️ S1 Step12: INY → 50mm thất bại — thử lại")
                return
                return
            self._cmd_sent     = True
            self._step_timeout = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout:
                self.get_logger().warn("⚠️ S1 Step12: INY → 50mm timeout → retry")
                self._cmd_sent = False
            elif self._arrived(2):
                self.get_logger().info("✅ INY tại 50mm → INX về home")
                self._enter(SystemState.S1_INX_HOME)

    # ── Step 13: INX → 20mm ──────────────────────────────────────
    def _s1_inx_home(self):
        """INX → 20mm (điều kiện: INY ≤ 50mm)."""
        if not self._iny_safe():
            self._log_once("S1_INX_WAIT_INY", "⏳ Chờ INY ≤ 50mm trước khi INX về")
            return
        if not self._cmd_sent:
            ok = self._nb_move(1, self.config.inx_home)
            if not ok:
                self.get_logger().warn("⚠️ S1 Step13: INX → 20mm thất bại — thử lại")
                return
                return
            self._cmd_sent     = True
            self._step_timeout = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout:
                self.get_logger().warn("⚠️ S1 Step13: INX timeout → retry")
                self._cmd_sent = False
            elif self._arrived(1):
                self.get_logger().info("✅ INX tại 20mm → STATE 1 COMPLETE")
                self._enter(SystemState.S1_COMPLETE)

    # ── STATE 1 COMPLETE ─────────────────────────────────────────
    def _s1_complete(self):
        """
        Publish new_tray_loaded.
        Chờ robot_done → vào STATE 2 (thay khay).

        Thiết kế cơ khí đảm bảo:
          • Cyl2 KHÔNG bao giờ retract khi đang đỡ khay
          → Khi robot_done = True, S13 chắc chắn ON
          → Không cần check S13 riêng

        robot_done được publish bởi:
          • Robot khi hoàn thành row 5 (full tray)
          • Camera AI khi phát hiện khay + row 5 not ready
        """
        if not self._cmd_sent:
            self.pub_new_tray.publish(Bool(data=True))
            self.get_logger().info("📤 Published: new_tray_loaded = True")
            self._notify('info', '✅ STATE 1 COMPLETE',
                         'Khay đã cấp cho robot — chờ robot done để thay khay')
            self.stack_row_index = self._current_row
            self._cmd_sent = True

        # MANUAL mode: về IDLE ngay sau khi publish — operator chọn STATE 2 khi sẵn sàng
        if self.operation_mode == 'manual':
            self.get_logger().info("ℹ️ [MANUAL] State 1 done → IDLE. Chọn STATE 2 từ GUI khi robot xong.")
            self._enter(SystemState.IDLE)
            return

        # AUTO mode: chờ robot_done → tự vào STATE 2
        if self._robot_done:
            self.get_logger().info("🔄 Robot done → STATE 2 Thay khay")
            self._enter(SystemState.S2A_CHECK_INTERLOCK)
        else:
            self._log_once("S1C_WAIT_ROBOT",
                           "⏳ Chờ robot_done — robot (row 5 done) hoặc "
                           "camera AI (khay detect + row 5 not ready) gửi /revpi/robot_done")


    # ══════════════════════════════════════════════════════════════
    # ─── STATE 2: Thay khay Input ────────────────────────────────
    # ══════════════════════════════════════════════════════════════

    # ── Step 1: Check interlock ───────────────────────────────────
    def _s2a_check_interlock(self):
        """
        Đảm bảo INY ≤ 50mm.
        Snapshot S6 (row 1 output stack custom tray):
          S6 ON  → row 1 đang có khay → jog tìm row tiếp theo bằng S4
          S6 OFF → row 1 trống         → đi thẳng row 1, không cần jog
        """
        if not self._cmd_sent:
            # Snapshot S6: phát hiện có khay ở row 1 output stack không
            self._s6_snapshot  = self.sensor(6)
            self._s4_armed_out = False
            self._output_row   = 1  # default
            self.get_logger().info(
                f"Step1: Check interlock | S6(row1 output) = {self._s6_snapshot} "
                f"({'có khay → jog dò S4' if self._s6_snapshot else 'trống → thẳng row 1'})"
            )
            self._cmd_sent = True

        if self._iny_safe():
            self.get_logger().info(f"✅ INY safe → Step2 INX 500mm")
            self._enter(SystemState.S2A_INX_500)
        else:
            # INY chưa về → về home
            if not self._iny_moving:
                ok = self._nb_move(2, self.config.iny_home)
                if not ok:
                    self._error("S2 Step1: INY về home thất bại")
            self._log_once("S2A_WAIT_INY", "⏳ Step1: chờ INY ≤ 50mm")

    # ── A2: INX → 500mm (S10 interlock) ─────────────────────────
    def _s2a_inx_500(self):
        """INX → 500mm. Điều kiện: INY ≤ 50mm VÀ S10 ON (Cyl1 retracted)."""
        if not self._iny_safe():
            self._log_once("S2A_WAIT_INY2", "⏳ A2: INY chưa safe")
            return
        # S10 interlock: Cyl1 phải retracted trước khi INX di chuyển vào
        if not self.sensor(10):
            self._log_once(
                "S2A_S10_INTERLOCK",
                "🔒 INTERLOCK S2: S10=OFF (Cyl1 chưa retract) → INX bị khóa.\n"
                "  Kiểm tra: Cyl1 đã retract chưa? S10 cảm biến bị lỗi?"
            )
            self._notify('warn', '⚠️ S10 chưa ON',
                         'Cyl1 chưa retract — INX bị khóa. Kiểm tra S10/Cyl1.')
            return
        if not self._cmd_sent:
            ok = self._nb_move(1, self.config.inx_target)
            if not ok:
                self._log_once("S2A_A2_INX_TO_500MM_THẤT_", "⚠️ S2A A2: INX → 500mm thất bại — thử lại vòng tới")
                return
                return
            self._cmd_sent     = True
            self._step_timeout = time.time() + self.config.move_timeout
            self.get_logger().info("▶ A2: INX → 500mm")
        else:
            if time.time() > self._step_timeout:
                self.get_logger().warn("⚠️ S2A A2: INX timeout → retry move")
                self._cmd_sent = False
            elif self._arrived(1):
                self.get_logger().info("✅ A2: INX tại 500mm → A3")
                self._enter(SystemState.S2A_INY_200_CYL1)

    # ── A3: INY → 200mm + Cyl1 extend ────────────────────────────
    def _s2a_iny_200_cyl1(self):
        """INY → 200mm rồi extend Cyl1 (ch5) để gắp khay cũ."""
        if not self._cmd_sent:
            ok = self._nb_move(2, self.config.iny_place)   # 200mm
            if not ok:
                self._log_once("S2A_A3_INY_TO_200MM_THẤT_", "⚠️ S2A A3: INY → 200mm thất bại — thử lại vòng tới")
                return
                return
            self._cmd_sent     = True
            self._step_timeout = time.time() + self.config.move_timeout
            self.get_logger().info("▶ A3: INY → 200mm")
        else:
            if time.time() > self._step_timeout:
                self.get_logger().warn("⚠️ S2A A3: INY → 200mm timeout → retry move")
                self._cmd_sent = False
            elif self._arrived(2):
                self.get_logger().info("✅ A3: INY tại 200mm → Cyl1 EXTEND (ch5)")
                self._cyl1_extend()
                # Không timeout — chờ S11 vô thời hạn
                self._enter(SystemState.S2A_WAIT_S11)

    # ── A4: Chờ S11 ON ───────────────────────────────────────────
    def _s2a_wait_s11(self):
        """S11 ON = Cyl1 đã gắp khay cũ thành công. Không timeout."""
        if self.sensor(11):
            self.get_logger().info("✅ A4: S11 ON — gắp khay cũ OK → A5")
            self._enter(SystemState.S2A_INY_10)
            return
        self._log_once("S2A_WAIT_S11", "⏳ A4: Chờ S11 ON. Kích: '11:1'")

    # ── A5: INY → 10mm ───────────────────────────────────────────
    def _s2a_iny_10(self):
        """INY về 10mm (home) để INX tự do di chuyển."""
        if not self._cmd_sent:
            ok = self._nb_move(2, self.config.iny_home)   # 10mm
            if not ok:
                self._log_once("S2A_A5_INY_TO_10MM_THẤT_B", "⚠️ S2A A5: INY → 10mm thất bại — thử lại vòng tới")
                return
                return
            self._cmd_sent     = True
            self._step_timeout = time.time() + self.config.move_timeout
            self.get_logger().info("▶ A5: INY → 10mm")
        else:
            if time.time() > self._step_timeout:
                self.get_logger().warn("⚠️ S2A A5: timeout → retry")
                self._cmd_sent = False
            elif self._arrived(2):
                self.get_logger().info("✅ A5: INY tại 10mm → A6")
                self._enter(SystemState.S2A_INX_10)

    # ── A6: INX → 10mm ───────────────────────────────────────────
    def _s2a_inx_10(self):
        """INX → 10mm (INY ≤ 50mm)."""
        if not self._iny_safe():
            self._log_once("S2A_INX10_WAIT", "⏳ A6: INY chưa safe")
            return
        if not self._cmd_sent:
            ok = self._nb_move(1, self.config.inx_home)   # 10mm ≈ inx_home = 20mm; spec nói 10mm
            # Theo spec: "INX → 10mm" — dùng min(inx_home, 10mm)
            ok = self._nb_move(1, 10.0)
            if not ok:
                self._log_once("S2A_A6_INX_TO_10MM_THẤT_B", "⚠️ S2A A6: INX → 10mm thất bại — thử lại vòng tới")
                return
                return
            self._cmd_sent     = True
            self._step_timeout = time.time() + self.config.move_timeout
            self.get_logger().info("▶ A6: INX → 10mm")
        else:
            if time.time() > self._step_timeout:
                self.get_logger().warn("⚠️ S2A A6: INX → 10mm timeout → retry move")
                self._cmd_sent = False
            elif self._arrived(1):
                self.get_logger().info("✅ A6: INX tại 10mm → A7 jog output")
                self._s4_armed_out = False
                self._enter(SystemState.S2A_INY_JOG_OUTPUT)

    # ── Step 7: INY jog → output stack custom tray ───────────────
    def _s2a_iny_jog_output(self):
        """
        Dựa vào S6 (snapshot tại Step 1):
          S6 OFF → Row 1 trống → đi thẳng row 1 (không cần jog)
          S6 ON  → Row 1 có khay → jog tìm S4 để snap nearest row
        Giới hạn: row 1 = 590mm.
        """
        # S6 OFF → row 1 output stack trống → đi thẳng, không jog
        if not self._s6_snapshot:
            self.get_logger().info("ℹ️ Step7: S6 OFF (row 1 trống) → đi thẳng output row 1")
            self._output_row = 1
            self._enter(SystemState.S2A_INY_OUTPUT_ROW)
            return

        # S6 ON → row 1 có khay → jog tìm S4
        iny = self._pos(2)
        if iny is None:
            return

        if iny >= INY_S4_ARM_MM:
            self._s4_armed_out = True

        if iny >= OUTY_ROW1_LIMIT_MM:
            self._stop(2)
            self.get_logger().info(f"ℹ️ Step7: INY chạm {OUTY_ROW1_LIMIT_MM}mm → fallback row 1")
            self._output_row = 1
            self._enter(SystemState.S2A_INY_OUTPUT_ROW)
            return

        if self._s4_armed_out and self.sensor(4):
            self._stop(2)
            current = self._pos(2) or iny
            self._output_row = self._find_nearest_row(current, self.config.iny_output_stack)
            self.get_logger().info(f"✅ Step7: S4 ON tại {current:.1f}mm → output row {self._output_row}")
            self._enter(SystemState.S2A_INY_OUTPUT_ROW)
            return

        self._jog(2, INY_JOG_VEL)
        self._log_once("S2A_JOG_OUT", "🔄 Step7: INY jog tìm S4 (output stack, S6 ON)")


    # ── A8: INY → output row + Cyl1 retract ─────────────────────
    def _s2a_iny_output_row(self):
        """INY → iny_output_stack[row], đến nơi → Cyl1 retract (ch4)."""
        target = self.config.iny_output_stack.get(self._output_row)
        if target is None:
            self.get_logger().warn(
                f"⚠️ S2A A8: output row {self._output_row} không có trong config — dùng row 1"
            )
            self._output_row = 1
            target = self.config.iny_output_stack.get(1)
            if target is None:
                return
        if not self._cmd_sent:
            self.get_logger().info(f"▶ A8: INY → {target}mm (output row {self._output_row})")
            ok = self._nb_move(2, target)
            if not ok:
                self.get_logger().warn(f"⚠️ S2A A8: INY → {target}mm nb_move fail — retry")
                return
            self._cmd_sent     = True
            self._step_timeout = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout:
                self.get_logger().warn("⚠️ S2A A8: INY timeout → retry move")
                self._cmd_sent = False
            elif self._arrived(2):
                self.get_logger().info("✅ A8: INY tại output row → Cyl1 RETRACT (ch4)")
                self._cyl1_retract()
                # Không timeout — chờ S10 vô thời hạn
                self._enter(SystemState.S2A_WAIT_S10)

    # ── A9: Chờ S10 ON ───────────────────────────────────────────
    def _s2a_wait_s10(self):
        """S10 ON = Cyl1 đã thả khay vào output stack. Không timeout."""
        if self.sensor(10):
            self.get_logger().info("✅ A9: S10 ON — đã thả khay vào output stack → A10")
            self._enter(SystemState.S2A_INY_10_FINAL)
            return
        self._log_once("S2A_WAIT_S10", "⏳ A9: Chờ S10 ON. Kích: '10:1'")

    # ── A10: INY → 10mm ──────────────────────────────────────────
    def _s2a_iny_10_final(self):
        """INY về 10mm."""
        if not self._cmd_sent:
            ok = self._nb_move(2, self.config.iny_home)
            if not ok:
                self._log_once("S2A_A10_INY_TO_10MM_THẤT_", "⚠️ S2A A10: INY → 10mm thất bại — thử lại vòng tới")
                return
                return
            self._cmd_sent     = True
            self._step_timeout = time.time() + self.config.move_timeout
            self.get_logger().info("▶ A10: INY → 10mm")
        else:
            if time.time() > self._step_timeout:
                self.get_logger().warn("⚠️ S2A A10: INY timeout → retry move")
                self._cmd_sent = False
            elif self._arrived(2):
                self.get_logger().info("✅ A10: INY tại 10mm → A11")
                self._enter(SystemState.S2A_INX_20)

    # ── A11: INX → 20mm ──────────────────────────────────────────
    def _s2a_inx_20(self):
        """INX → 20mm (home)."""
        if not self._iny_safe():
            self._log_once("S2A_INX20_WAIT", "⏳ A11: Chờ INY safe")
            return
        if not self._cmd_sent:
            ok = self._nb_move(1, self.config.inx_home)  # 20mm
            if not ok:
                self._log_once("S2A_A11_INX_TO_20MM_THẤT_", "⚠️ S2A A11: INX → 20mm thất bại — thử lại vòng tới")
                return
                return
            self._cmd_sent     = True
            self._step_timeout = time.time() + self.config.move_timeout
            self.get_logger().info("▶ A11: INX → 20mm")
        else:
            if time.time() > self._step_timeout:
                self.get_logger().warn("⚠️ S2A A11: INX timeout → retry move")
                self._cmd_sent = False
            elif self._arrived(1):
                self.get_logger().info("✅ A11: INX tại 20mm → Rút Cyl2 trước khi complete")
                self._cyl2_retract()
                self._enter(SystemState.S2A_WAIT_CYL2_RETRACT)

    # ── A11b: Chờ Cyl2 retract (S12 ON + S13 OFF) ───────────────
    def _s2a_wait_cyl2_retract(self):
        """Đảm bảo Cyl2 đã về (S12 ON + S13 OFF) trước khi báo complete."""
        s12 = self.sensor(12)
        s13 = self.sensor(13)
        if s12 and not s13:
            self.get_logger().info("✅ A11b: S12 ON + S13 OFF — Cyl2 đã retract → COMPLETE")
            self._enter(SystemState.S2A_COMPLETE)
            return
        self._log_once(
            "S2A_WAIT_CYL2",
            f"⏳ A11b: Chờ Cyl2 retract | S12={s12} S13={s13}. "
            f"Cần: S12=ON + S13=OFF. Kích: '12:1' '13:0'"
        )

    # ── STATE 2 COMPLETE ──────────────────────────────────────────
    def _s2a_complete(self):
        """
        Kết thúc STATE 2 (thay khay).
        Reset _robot_done, sau đó kiểm tra điều kiện để tự động chạy lại STATE 1.

        AUTO  mode: Nếu đủ điều kiện → vào STATE 1 ngay.
                    Nếu chưa đủ     → về IDLE, tự theo dõi và chạy khi đủ.
        MANUAL mode: Luôn về IDLE — operator chọn bước tiếp theo từ GUI.
        """
        if not self._cmd_sent:
            # Reset robot_done ngay khi State 2 hoàn thành
            self._robot_done = False
            self._notify('info', '✅ STATE 2 COMPLETE', 'Đã thay khay — kiểm tra điều kiện State 1')
            self.get_logger().info("✅ State 2 COMPLETE | _robot_done reset")
            self._cmd_sent = True

        if self.operation_mode == 'manual':
            self.get_logger().info("ℹ️ [MANUAL] State 2 done → IDLE. Chọn STATE 1 từ GUI để tiếp.")
            self._notify('info', '🖐 MANUAL: State 2 xong', 'Chọn STATE 1 từ GUI để cấp khay tiếp')
            self._guide_logged.discard("IDLE_MANUAL")
            self._enter(SystemState.IDLE)
            return

        # AUTO mode — kiểm tra ngay điều kiện State 1
        if self._can_start_s1():
            self.get_logger().info(
                f"🔄 AUTO State2 → State1: "
                f"S1={self.sensor(1)} S2={self.sensor(2)} "
                f"S3={self.sensor(3)} S12={self.sensor(12)} → Cấp khay tiếp"
            )
            self._enter(SystemState.S1_CONFIRM_SAFE)
        else:
            s1s2s3 = self.sensor(1) or self.sensor(2) or self.sensor(3)
            s12    = self.sensor(12)
            if not s1s2s3:
                self.get_logger().info("⏳ State2 done — S1/S2/S3 OFF, băng tải hết khay → IDLE tự theo dõi")
                self._notify('warn', '⚠️ Hết khay trên băng tải',
                             'Nạp khay vào — hệ thống tự chạy lại khi phát hiện')
            elif not s12:
                self.get_logger().info("⏳ State2 done — S12 OFF, vị trí cấp chưa trống → IDLE chờ")
                self._notify('info', '⏳ Chờ vị trí cấp trống (S12)',
                             'Sẽ tự vào State 1 khi S12 ON')
            self._guide_logged.discard("IDLE_NO_TRAY")
            self._guide_logged.discard("IDLE_NO_PLACE")
            self._enter(SystemState.IDLE)


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
        print("  Cartridge System")
        print("  STATE 1 : Cấp khay Input   (băng tải → robot)")
        print("  STATE 2 : Thay khay Input  (robot done → output stack custom tray)")
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