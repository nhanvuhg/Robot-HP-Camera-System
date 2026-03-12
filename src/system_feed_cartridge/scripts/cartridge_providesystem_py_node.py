#!/usr/bin/env python3
"""
Cartridge Loading System with 5 Servo Motors (ROS 2 + festo-edcon)
- Servo 1: InX
- Servo 2: InY
- Servo 3: Put Tray
- Servo 4: OutX
- Servo 5: OutY

API: edcon.edrive.com_modbus.ComModbus + MotionHandler
Protocol: Modbus TCP (port 502)
Units: drive position counts (1 mm = 1,000 counts, 1 count = 1 µm)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from enum import Enum
import time
from typing import Optional, Dict, List
import yaml
import json
import threading
import os
import signal

# Festo imports
try:
    from edcon.edrive.com_modbus import ComModbus
    from edcon.edrive.motion_handler import MotionHandler
    from edcon.utils.logging import Logging as EdconLogging
    EDCON_AVAILABLE = True
except ImportError:
    print("Warning: festo-edcon not found. Running in simulation mode.")
    ComModbus = None
    MotionHandler = None
    EdconLogging = None
    EDCON_AVAILABLE = False


def safe_try(func, retries=3, backoff=2, logger=None):
    """Call func(), retrying on ConnectionError with exponential backoff.
    Raises the last ConnectionError if all retries fail."""
    for attempt in range(1, retries + 1):
        try:
            return func()
        except ConnectionError as e:
            msg = f"ConnectionError on attempt {attempt}/{retries}: {e}"
            if logger:
                logger.warning(msg)
            else:
                print(msg)
            if attempt == retries:
                raise
            time.sleep(backoff * attempt)

try:
    from cpx_io.cpx_system.cpx_ap.cpx_ap import CpxAp
    CPXAP_AVAILABLE = True
except ImportError:
    print("Warning: cpx-io not found. IO module in simulation mode.")
    CpxAp = None
    CPXAP_AVAILABLE = False

# Unit conversion constants (verified with FAS display: 409,600 counts = 409.6mm)
COUNTS_PER_MM = 1000       # 1 mm = 1,000 counts (1 count = 1 µm)
DEFAULT_VELOCITY = 30      # velocity for position_task

# [FIX-2] Cylinder timeout — 15 giây (thay vì vô hạn)
CYLINDER_TIMEOUT_S = 15.0

# [FIX-4] Config validation ranges — (min, max) mm
_CONFIG_RANGES = {
    "inx_home":             (0.0,   200.0),
    "inx_safe_zone":        (0.0,   200.0),
    "inx_target2":          (10.0,  700.0),
    "inx_output_stack":     (10.0,  700.0),
    "iny_home":             (0.0,   200.0),
    "iny_safe_zone":        (5.0,   200.0),
    "iny_target2":          (10.0,  700.0),
    "iny_search_velocity":  (1.0,   200.0),
    "iny_slow_velocity":    (1.0,   100.0),
    "servo3_home":          (0.0,   50.0),
    "servo3_target1":       (0.0,   200.0),
    "servo3_push_position": (10.0,  400.0),
    "servo3_jog_velocity":  (1.0,   200.0),
    "outx_home":            (0.0,   200.0),
    "outx_target1":         (0.0,   200.0),
    "outx_target2":         (10.0,  700.0),
    "outx_target3":         (10.0,  700.0),
    "outy_home":            (0.0,   200.0),
    "outy_target1":         (0.0,   200.0),
    "outy_target2":         (10.0,  700.0),
    "outy_safe_zone":       (5.0,   200.0),
    "outy_search_velocity": (1.0,   200.0),
    "position_tolerance":   (0.1,   50.0),
    "homing_timeout":       (10.0,  300.0),
    "move_timeout":         (5.0,   120.0),
}
_ROW_POSITION_RANGE = (0.0, 960.0)  # mm — giới hạn vật lý trục


class SystemState(Enum):
    """State machine states"""
    IDLE = "idle"
    ERROR = "error"
    MOTION_DELAY = "motion_delay"  # Non-blocking 2s pause between motions
    
    # ========== STATE 1: Khởi Tạo + Cấp Khay Input ==========
    # Position 1 + Position 2 chạy song song
    HOMING = "homing"
    HOMING_RUNNING = "homing_running"  # Guard: đang chạy homing trong background thread
    
    # Position 1: InX, InY, Cylinder 1
    S1_INY_CONFIRM_SAFE = "s1_iny_confirm_safe"
    S1_INX_TO_CONVEYOR_END = "s1_inx_to_conveyor_end"
    S1_INX_WAIT_STOP = "s1_inx_wait_stop"  # Chờ InX dừng hoàn toàn trước khi check S3
    S1_INY_SEARCH_TRAY = "s1_iny_search_tray"
    S1_INY_TO_NEAREST_ROW = "s1_iny_to_nearest_row"
    S1_CHECK_S5 = "s1_check_s5"                      # [NEW] Check S5 tại row, timeout 2s
    S1_WAIT_DASHBOARD_CONFIRM = "s1_wait_dashboard_confirm"  # [NEW] Chờ confirm sau S5 fail
    S1_CYLINDER1_EXTEND = "s1_cylinder1_extend"
    S1_WAIT_S12_INTERLOCK = "s1_wait_s12_interlock"  # [NEW] INY interlock chờ S12 ON
    S1_CYL3_EXTEND = "s1_cyl3_extend"               # [NEW] Extend Cyl3 hold tray, chờ S13
    S1_INY_TO_TARGET1 = "s1_iny_to_target1"
    S1_INY_TO_TARGET2 = "s1_iny_to_target2"
    S1_CYLINDER1_RETRACT = "s1_cylinder1_retract"
    S1_INY_RETURN_SAFE = "s1_iny_return_safe"
    S1_INX_RETURN_SAFE = "s1_inx_return_safe"
    
    # Position 2: Servo 3, 4, 5 (song song với Position 1)
    S1_SERVO3_CHECK_S7 = "s1_servo3_check_s7"
    S1_SERVO3_MOVE_TO_S6 = "s1_servo3_move_to_s6"
    S1_SERVO3_WAIT_LOAD = "s1_servo3_wait_load"  # Wait S7 ON + confirm
    
    STATE1_COMPLETE = "state1_complete"
    
    # ========== STATE 2: Thay Khay Input (Position 1) ==========
    S2_WAIT_TRIGGER = "s2_wait_trigger"
    
    # Phase A: Thu hồi khay cũ từ robot → output stack
    S2_CHECK_INTERLOCK = "s2_check_interlock"
    S2_INX_TO_TARGET2 = "s2_inx_to_target2"
    S2_INY_DOWN_TO_TRAY = "s2_iny_down_to_tray"
    S2_CYLINDER1_EXTEND = "s2_cylinder1_extend"
    S2_INY_UP_SAFE = "s2_iny_up_safe"
    S2_INX_TO_TARGET1 = "s2_inx_to_target1"
    S2_INY_SEARCH_STACK = "s2_iny_search_stack"
    S2_INY_TO_NEAREST_ROW = "s2_iny_to_nearest_row"
    S2_CYLINDER1_RETRACT = "s2_cylinder1_retract"
    S2_INY_RETURN_SAFE = "s2_iny_return_safe"
    S2_INX_RETURN_SAFE = "s2_inx_return_safe"
    S2_CHECK_REMAINING = "s2_check_remaining"
    
    # Phase B: Nạp khay mới từ input stack → robot
    S2_LOAD_WAIT_ROBOT = "s2_load_wait_robot"      # Chờ robot idle trước khi INX vào
    S2_LOAD_INX_TO_TARGET2 = "s2_load_inx_to_target2"
    S2_LOAD_WAIT_S3 = "s2_load_wait_s3"            # Chờ S3 ON (khay đến cuối băng tải)
    S2_LOAD_INY_SEARCH_TRAY = "s2_load_iny_search_tray"
    S2_LOAD_INY_TO_ROW = "s2_load_iny_to_row"
    S2_LOAD_CHECK_S5 = "s2_load_check_s5"                        # [NEW] Check S5, timeout 2s
    S2_LOAD_WAIT_CONFIRM = "s2_load_wait_confirm"                 # [NEW] Chờ confirm sau S5 fail
    S2_LOAD_CYLINDER1_EXTEND = "s2_load_cylinder1_extend"
    S2_LOAD_WAIT_S12_INTERLOCK = "s2_load_wait_s12_interlock"    # [NEW] INY interlock chờ S12
    S2_LOAD_CYL3_EXTEND = "s2_load_cyl3_extend"                  # [NEW] Extend Cyl3, chờ S13
    S2_LOAD_INY_UP_SAFE = "s2_load_iny_up_safe"
    S2_LOAD_INY_TO_ROBOT_POS = "s2_load_iny_to_robot_pos"
    S2_LOAD_CYLINDER1_RETRACT = "s2_load_cylinder1_retract"
    S2_LOAD_INY_RETURN_SAFE = "s2_load_iny_return_safe"
    S2_LOAD_INX_RETURN_SAFE = "s2_load_inx_return_safe"
    STATE2_COMPLETE = "state2_complete"
    S2_NO_TRAY = "s2_no_tray"
    
    # ========== STATE 3: Thay Khay Output (Position 2) ==========
    S3_WAIT_TRIGGER = "s3_wait_trigger"
    S3_CHECK_SAFE = "s3_check_safe"
    S3_OUTX_TO_TARGET3 = "s3_outx_to_target3"
    S3_OUTY_TO_TARGET2 = "s3_outy_to_target2"
    S3_CYLINDER2_EXTEND = "s3_cylinder2_extend"
    S3_OUTY_TO_TARGET1 = "s3_outy_to_target1"
    S3_OUTX_TO_TARGET2 = "s3_outx_to_target2"
    S3_OUTY_SEARCH_ROW = "s3_outy_search_row"
    S3_OUTY_TO_ROW = "s3_outy_to_row"
    S3_CYLINDER2_RETRACT = "s3_cylinder2_retract"
    S3_OUTY_RETURN_HOME = "s3_outy_return_home"
    S3_OUTX_RETURN_SAFE = "s3_outx_return_safe"
    S3_SERVO3_MOVE_TO_S6 = "s3_servo3_move_to_s6"
    S3_SERVO3_CHECK_S7 = "s3_servo3_check_s7"
    S3_SERVO3_TO_TARGET1 = "s3_servo3_to_target1"
    S3_SERVO3_WAIT_LOAD = "s3_servo3_wait_load"
    S3_WAIT_HMI_RESUME = "s3_wait_hmi_resume"  # Chờ HMI confirm khi hết khay
    STATE3_COMPLETE = "state3_complete"


class SensorManager:
    """Manages all sensors with expandable structure"""
    def __init__(self):
        self.sensors = {
            # ========== Position 1 - Quang ==========
            1: False,   # S1: Input conveyor (đầu băng tải)
            2: False,   # S2: Input mid conveyor (giữa băng tải)
            3: False,   # S3: Input end conveyor (cuối băng tải)
            4: False,   # S4: Detect stack Traycustom (chồng khay kim loại)
            5: False,   # S5: Detect Traycustom output (khay ở vị trí output)
            # ========== Position 2 - Quang ==========
            6: False,   # S6: Detect Tray plastic robot (vị trí robot thao tác)
            7: False,   # S7: Detect Tray plastic loading (bệ nâng khay)
            8: False,   # S8: Detect Tray plastic output (khay nhựa ở output)
            9: False,   # S9: Detect Tray plastic stack (chồng khay nhựa)
            # ========== Xi-lanh Limits ==========
            10: False,  # S10: Cylinder 1 MIN (retract) - Position 1
            11: False,  # S11: Cylinder 1 MAX (extend) - Position 1
            12: False,  # S12: Cylinder 2 MIN (retract) - Position 2
            13: False,  # S13: Cylinder 2 MAX (extend) - Position 2
        }
        
    def update_sensor(self, sensor_id: int, state: bool):
        """Update sensor state"""
        if sensor_id in self.sensors:
            self.sensors[sensor_id] = state
        else:
            # Auto-expand for new sensors
            self.sensors[sensor_id] = state
            
    def get_sensor(self, sensor_id: int) -> bool:
        """Get sensor state"""
        return self.sensors.get(sensor_id, False)
    
    def get_rising_edge(self, sensor_id: int, prev_state: Dict[int, bool]) -> bool:
        """Detect rising edge (OFF -> ON)"""
        current = self.get_sensor(sensor_id)
        previous = prev_state.get(sensor_id, False)
        return current and not previous
    
    def get_falling_edge(self, sensor_id: int, prev_state: Dict[int, bool]) -> bool:
        """Detect falling edge (ON -> OFF)"""
        current = self.get_sensor(sensor_id)
        previous = prev_state.get(sensor_id, False)
        return not current and previous


class CartridgeConfig:
    """Configuration parameters"""
    def __init__(self, config_file: Optional[str] = None):
        # Servo IP addresses
        self.servo_ips = {
            1: "192.168.27.248",  # InX
            2: "192.168.27.249",  # InY
            3: "192.168.27.250",  # Put Tray
            4: "192.168.27.251",  # OutX
            5: "192.168.27.252",  # OutY
        }
        
        # IO Module
        self.io_ip = "192.168.27.253"
        
        # Cylinder channels
        self.cylinder1_extend_channel = 5
        self.cylinder1_retract_channel = 4
        self.cylinder2_extend_channel = 7
        self.cylinder2_retract_channel = 6
        self.cylinder3_retract_channel = 8   # Hold Tray — nhả khay
        self.cylinder3_extend_channel = 9    # Hold Tray — kẹp giữ khay
        
        # InX positions (mm)
        self.inx_home = 20.0             # Target 1 — vị trí safe gần home
        self.inx_safe_zone = 10.0        # Safe zone — INX dừng đị để INY có thể vào/ra output stack
        self.inx_target2 = 500.0         # Input stack position (cuối băng tải)
        self.inx_output_stack = 100.0    # Output stack position (Position 1 - đặt khay đã dùng)
        
        # InY positions (mm)
        self.iny_home = 10.0             # Target 1 — safe zone position
        self.iny_safe_zone = 30.0        # Safe zone threshold - INX chỉ được di chuyển khi INY < giá trị này
        self.iny_target2 = 200.0         # Robot place position (vị trí đặt khay cho robot)
        
        # ═══════════════════════════════════════════════════════════
        # Position 1 (INY) - Row positions (mm)
        # ═══════════════════════════════════════════════════════════
        # INY có 2 stacks: Input (khay custom chứa cartridge) + Output (khay custom đã dùng)
        
        # Input stack - 8 rows (khay custom input)
        # Row 8 = top (gần home), Row 1 = bottom (xa home)
        self.iny_input_stack = {
            8: 250.0,   # Top (closest to home)
            7: 300.0,
            6: 350.0,
            5: 400.0,
            4: 450.0,
            3: 500.0,
            2: 550.0,
            1: 600.0,   # Bottom (furthest from home)
        }
        
        # Output stack - 8 rows (khay custom output - đặt khay đã lấy cartridge)
        # Vị trí có thể khác input stack
        self.iny_output_stack = {
            8: 100.0,   # Top
            7: 170.0,
            6: 240.0,
            5: 310.0,
            4: 380.0,
            3: 450.0,
            2: 520.0,
            1: 590.0,   # Bottom
        }
        
        # ═══════════════════════════════════════════════════════════
        # Position 2 (OUTY) - Row positions (mm)
        # ═══════════════════════════════════════════════════════════
        # OUTY có 1 table: Output table (khay chứa cartridge đã fill xong)
        
        # Output table - 8 rows (khay đã fill xong)
        self.outy_output_table = {
            8: 80.0,    # Top (closest to home)
            7: 140.0,
            6: 200.0,
            5: 260.0,
            4: 320.0,
            3: 380.0,
            2: 440.0,
            1: 500.0,   # Bottom (furthest from home)
        }
        
        # Servo 3 (LoadTrayRobot) - vít me + platform
        self.servo3_home = 0.0
        self.servo3_target1 = 50.0           # Safe/wait position (Target 1)
        self.servo3_push_position = 300.0   # Max push position (adjustable)
        self.servo3_jog_velocity = 50.0     # Jog speed
        
        # OutX positions (mm)
        self.outx_home = 0.0
        self.outx_target1 = 100.0   # Safe position
        self.outx_target2 = 400.0   # Output stack (dặt chồng khay nhựa)
        self.outx_target3 = 500.0   # Robot tray position (servo 3 load)
        
        # OutY positions (mm)
        self.outy_home = 0.0
        self.outy_target1 = 50.0    # Safe position
        self.outy_target2 = 300.0   # Pick/place position
        self.outy_safe_zone = 50.0  # OUTX chỉ được di chuyển khi OUTY < giá trị này
        self.outy_search_velocity = 40.0  # OUTY search speed (Position 2 riêng)
        
        # Position tolerance (mm)
        self.position_tolerance = 5.0
        
        # Stack configuration
        self.max_trays = 8  # Maximum trays per stack (1-8 possible)
        self.max_slots_per_output_tray = 9  # 9 slots per output tray
        # Note: Each slot = 1 batch = 8 cartridges from input trays
        
        # Jog velocities (mm/s)
        self.servo3_jog_velocity = 50.0  # Put Tray jog speed
        self.iny_search_velocity = 30.0    # InY search speed — JOG+ hướng dương về phía stack
        self.iny_slow_velocity = 10.0    # InY slow approach speed (near stack)
        
        # Safety settings
        self.outx_safe_position_threshold = 100.0  # OutX must be <= this for OutY to move
        
        # ✅ Servo soft limits (mm) - servo chỉ được di chuyển đến giá trị này
        # Có thể thay đổi qua service /providesystem/set_servo_limit
        self.servo_limits = {
            1: 700.0,   # InX max (mm)
            2: 700.0,   # InY max (mm)
            3: 400.0,   # Servo 3 (LoadTrayRobot) max (mm)
            4: 600.0,   # OutX max (mm)
            5: 600.0,   # OutY max (mm)
        }
        
        
        # Timeout settings (seconds)
        self.homing_timeout = 90.0  # 90s — OutY có hành trình dài cần thêm thời gian (cũ: 30s)
        self.move_timeout = 20.0
        self.cylinder_timeout = 5.0
        
        if config_file:
            self.load_from_file(config_file)
    
    def load_from_file(self, config_file: str):
        """Load configuration from YAML file"""
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                if not config:
                    return
                # Update attributes from config
                for key, value in config.items():
                    if hasattr(self, key):
                        # Convert dict keys to int for row tables
                        if isinstance(value, dict) and key in ('iny_input_stack', 'iny_output_stack', 'outy_output_table', 'servo_ips', 'servo_limits'):
                            value = {int(k): v for k, v in value.items()}
                        setattr(self, key, value)
                self._config_file = config_file
                print(f"✅ Config loaded from {config_file}")
        except Exception as e:
            print(f"Warning: Failed to load config file: {e}")
    
    def save_to_file(self, config_file: str = None):
        """Save current configuration to YAML file"""
        path = config_file or getattr(self, '_config_file', None)
        if not path:
            print("Warning: No config file path set, cannot save")
            return False
        try:
            data = {}
            # Export all config attributes (skip private/methods)
            for key in [
                'servo_ips', 'io_ip',
                'cylinder1_extend_channel', 'cylinder1_retract_channel',
                'cylinder2_extend_channel', 'cylinder2_retract_channel',
                'cylinder3_retract_channel', 'cylinder3_extend_channel',
                'inx_home', 'inx_target2', 'inx_output_stack',
                'iny_home', 'iny_safe_zone', 'iny_target2',
                'iny_input_stack', 'iny_output_stack',
                'servo3_home', 'servo3_target1', 'servo3_push_position', 'servo3_jog_velocity',
                'outx_home', 'outx_target1', 'outx_target2', 'outx_target3',
                'outy_home', 'outy_target1', 'outy_target2', 'outy_safe_zone', 'outy_search_velocity',
                'outy_output_table',
                'servo_limits',
                'iny_search_velocity', 'iny_slow_velocity',
                'position_tolerance', 'outx_safe_position_threshold',
                'max_trays', 'max_slots_per_output_tray',
                'homing_timeout', 'move_timeout', 'cylinder_timeout',
            ]:
                if hasattr(self, key):
                    val = getattr(self, key)
                    # Convert int-keyed dicts for clean YAML output
                    if isinstance(val, dict):
                        val = {int(k) if isinstance(k, (int, float)) else k: v for k, v in val.items()}
                    data[key] = val
            
            with open(path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
            print(f"✅ Config saved to {path}")
            return True
        except Exception as e:
            print(f"❌ Failed to save config: {e}")
            return False


class CartridgeSystem(Node):
    """Main cartridge loading system (ROS 2 + festo-edcon MotionHandler)"""
    
    def __init__(self, config: CartridgeConfig):
        super().__init__('cartridge_providesystem')
        
        self.config = config
        self.state = SystemState.IDLE
        self.sensor_manager = SensorManager()
        self.prev_sensor_state = {}

        # [FIX-1] Modbus lock — bảo vệ tất cả Modbus TCP access, tránh race condition
        self._servo_lock = threading.Lock()

        # Tray tracking
        self.max_trays = 8  # Maximum trays per stack
        self.detected_trays = 0  # Number of trays detected by sensor 5 (can be 1-8)
        self.current_tray = 0  # Current tray being processed (1-detected_trays)
        self.current_row = 0  # Current input row being processed (1-5 per tray)
        self.rows_per_tray = 5  # Each tray has 5 rows
        self.current_output_row = 0  # Current output row being processed (1-8)
        self.first_tray_detected = False
        self.load_tray_input_signal = False  # Flag for load_tray_input from camera
        self.load_tray_output_signal = False  # Flag for load_tray_output from camera
        
        # Output tray tracking
        self.max_slots_per_output_tray = 9  # 9 slots per output tray
        self.current_slot = 0  # Current slot being filled (1-9)
        # Note: Each slot contains 1 batch = 8 cartridges from input trays
        
        # ✅ NEW: Auto tray change tracking
        self.output_tray_full = False  # Camera báo tray đầy (9 slots)
        self.last_batch_mode = False   # Batch cuối cùng (hết input, tray chưa đầy)
        self.last_batch_complete = False  # Robot đã xử lý xong batch cuối cùng
        
        # ✅ INTERLOCK: Robot motion busy tracking
        self.robot_motion_busy = False  # True khi robot đang thực hiện motion
        self.change_tray_input_signal = False  # Robot yêu cầu thay khay input
        
        # ✅ Stack tracking (State 2)
        self.stack_row_index = 0       # Row index hiện tại trong stack (từ S4 detection)
        self.output_stack_row = 0      # Row index stack output (để đặt khay đã dùng)
        self.has_trays_remaining = True # Double-check: còn khay trên input stack không
        
        # ✅ Jog control flags (tránh gọi jog 50 lần/giây)
        self._iny_jog_active = False
        self._outy_jog_active = False
        self._servo3_jog_active = False
        
        # ✅ State completion guard (tránh pub topic nhiều lần)
        self._state_completed_published = False
        
        # ✅ Cylinder timeout tracking
        self._cylinder_start_time = None
        
        # ✅ Motion delay: non-blocking 2s pause between motion transitions
        self._motion_delay_start = None
        self._motion_delay_duration = 2.0
        self._motion_delay_next_state = None
        
        # ✅ S4 search gate: chỉ xét S4 sau khi InY đã jog được 100mm
        self._s4_search_active = False
        self._iny_jog_start_pos = 0.0  # Vị trí INY khi bắt đầu jog
        
        # ✅ S5 check + retry logic
        self._s5_check_start = None   # Thời điểm bắt đầu chờ S5
        self._s5_retry_count = 0      # Số lần S5 fail (reset về 0 khi thành công)
        
        # ✅ INX S3 wait: đợi S3 sau khi InX đến target (30s timeout)
        self._inx_s3_wait_start = None
        self._inx_arrived = False
        self._s3_seen_during_move = False  # Buffer: S3 ON trong khi INX chưa dừng
        
        # ✅ ACK flags - Robot xác nhận đã nhận topic
        self._tray_loaded_ack = False
        self._last_tray_ack = False
        self._pub_retry_count = 0
        self._pub_retry_timer = None
        self._pub_retry_max = 3        # Số lần retry publish
        self._pub_retry_interval = 0.2  # Giây giữa mỗi lần retry
        
        # ✅ Position 2 / State 3 flags
        self.output_tray_ready = False       # Interlock: robot chỉ place to output khi True
        self._confirm_load_received = False  # HMI xác nhận đã cấp khay output mới
        self._hmi_resume_confirmed = False   # HMI confirm tiếp tục chu kỳ mới (không cần restart)
        self._servo3_init_done = False       # Servo 3 đã init xong (State 1)
        
        # ✅ Parallel State 1: Position 1 + Position 2 chạy đồng thời
        self._pos1_done = False              # Position 1 (InX/InY) hoàn thành
        self._pos2_done = False              # Position 2 (Servo 3) hoàn thành
        self._pos2_state = "IDLE"            # State tracker riêng cho Position 2
        self._p2_s7_stable_start = None      # Timer đểm 3s s7 ON trước khi jog
        self._p2_gui_confirm_needed = False   # Cần GUI confirm sau khi S6 thất bại
        self._p2_move_cmd_sent = False        # Non-blocking move tracker cho P2
        self._p2_move_cmd_time = 0.0          # Thời điểm gửi lệnh move (để delay 0.5s)
        self.change_tray_output_signal = False  # Trigger thay khay output
        self._output_stack_row = 0           # Row index chồng khay output (Position 2)

        # ✅ Pause / Resume system
        self._system_paused = False          # Khi True: control loop bị tạm dừng
        
        # ✅ Manual testing mode
        self.manual_mode = False      # Manual testing mode (step-by-step)
        self.manual_auto_run = False   # Manual mode but run continuously (goto_state)
        self.step_pending = True      # Allow next step execution
        self._last_state = None       # Track state changes
        self.manual_target_row = None # Manual row selection (1-8, None=auto)
        
        # ✅ Operation mode: 'idle' (chờ chọn mode), 'auto' (chạy tự động), 'jog' (jog từng servo)
        self.operation_mode = 'idle'
        self._jog_active_servo = None  # Servo đang jog (None = không jog)
        self._jog_velocity = 500       # Jog velocity mặc định
        self._position_read_fail_count = {}  # Track consecutive read failures per servo
        
        # ✅ Sensor simulation (for testing without CPX IO)
        self._sim_sensors = {}  # {sensor_id: bool} — simulated sensor overrides
        self._io_read_fail_count = 0  # Hardware IO read failure counter
        # ✅ IO sensor cache for background reader (prevents control loop blocking)
        self._io_sensor_cache: list = []  # Last known channel values from IO module
        self._io_bg_lock = threading.Lock()  # Lock for cache access
        # [FIX-3] IO ready flag — True sau khi bg reader đọc thành công lần đầu
        self._io_ready = False

        # [FIX-6] INX / INY mutual exclusion flags
        self._inx_moving = False  # True khi INX đang di chuyển (non-blocking move)
        self._iny_moving = False  # True khi INY đang di chuyển (non-blocking move)

        # [FIX-8] Watchdog — phát hiện control loop bị block
        self._watchdog_last_tick = time.time()

        self._guide_logged = set()  # Track which guide messages have been logged (prevent spam)
        self._last_notify_time = {}  # {title: timestamp} — rate-limit _notify_gui to 0.5s per title
        
        # Servo connections
        self.servos = {}
        self.zero_offset = {}  # Home position per servo (absolute encoder counts at home = our "0")
        self.io_module = None
        
        # QoS profiles
        qos_default = QoSProfile(depth=1)
        qos_latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        # ROS 2 Publishers
        self.pub_run_conveyor = self.create_publisher(Bool, '/run_conveyor', qos_default)
        self.pub_state = self.create_publisher(String, '/system_state', qos_default)
        self.pub_new_tray_loaded = self.create_publisher(Bool, '/revpi/new_tray_loaded', qos_latch)
        self.pub_is_last_tray = self.create_publisher(Bool, '/revpi/is_last_tray', qos_latch)
        self.pub_output_tray_ready = self.create_publisher(Bool, '/revpi/output_tray_ready', qos_latch)
        
        # ROS 2 Subscribers — hệ thống điều khiển chính
        self.create_subscription(Bool, '/system/start_button',   self.start_button_callback,   qos_default)
        self.create_subscription(Bool, '/system/stop_button',    self.stop_button_callback,    qos_default)
        self.create_subscription(Bool, '/system/pause_button',   self.pause_button_callback,   qos_default)
        self.create_subscription(Bool, '/system/confirm_button', self.confirm_button_callback, qos_default)
        self.create_subscription(String, '/providesystem/set_operation_mode', self.set_operation_mode_callback, qos_default)
        self.create_subscription(String, '/providesystem/jog_cmd', self.jog_cmd_callback, qos_default)
        self.create_subscription(String, '/providesystem/sim_sensor', self.sim_sensor_callback, qos_default)
        self.create_subscription(String, '/providesystem/move_to_pos', self.move_to_pos_callback, qos_default)
        self.create_subscription(String, '/providesystem/update_config', self.update_config_callback, qos_default)
        self.create_subscription(String, '/providesystem/get_config', self.get_config_callback, qos_default)
        self.create_subscription(String, '/providesystem/reset_faults', self.reset_faults_callback, qos_default)
        
        # Config data publisher (for GUI to read current config)
        self.pub_config_data = self.create_publisher(String, '/providesystem/config_data', qos_latch)
        # GUI notification publisher (errors, status updates)
        self.pub_gui_notify = self.create_publisher(String, '/providesystem/gui_notify', qos_default)
        
        # ROS 2 Service Servers
        self.create_service(SetBool, '/providesystem/confirm_output_load', self.confirm_output_load_callback)
        self.create_service(SetBool, '/providesystem/set_manual_mode', self.set_manual_mode_callback)
        self.create_service(Trigger, '/providesystem/next_step', self.next_step_callback)
        self.create_service(SetBool, '/providesystem/set_servo_limit', self.set_servo_limit_callback)
        
        # Initialize hardware
        self.connect_hardware()
        
        # Main control loop timer (50 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop_callback)  # 20Hz — đủ đáp ứng cơ học, giảm tải Python GIL
        
        # Servo position publisher (10 Hz — for GUI real-time display)
        self.pub_servo_pos = self.create_publisher(String, '/providesystem/servo_positions', qos_default)
        self.create_timer(0.5, self._publish_positions)  # 2Hz — giảm tải Modbus reads trên Pi

        # Interlock topic — robot sẽ chặn pick/refill khi cartridge đang hoạt động
        self.pub_cartridge_busy = self.create_publisher(Bool, '/cartridge/busy', qos_default)
        self._last_cartridge_busy = None  # Chỉ publish khi thay đổi (tránh spam)

        # [FIX-8] Watchdog timer — cảnh báo nếu control loop bị block > 3s
        self.create_timer(5.0, self._watchdog_tick)

        self.get_logger().info("Cartridge System Initialized (PATCHED — lock/timeout/io_ready/watchdog)")
        self.get_logger().info("=" * 50)
        self.get_logger().info("📋 Chọn mode trước khi bắt đầu:")
        self.get_logger().info("  AUTO: ros2 topic pub --once /providesystem/set_operation_mode std_msgs/String 'data: auto'")
        self.get_logger().info("  JOG:  ros2 topic pub --once /providesystem/set_operation_mode std_msgs/String 'data: jog'")
        self.get_logger().info("=" * 50)

    
    def connect_hardware(self):
        """Connect to Festo servo drives via Modbus TCP + MotionHandler.
        Uses cycle_time=60, timeout_ms=15000 to prevent Modbus communication errors."""
        if not EDCON_AVAILABLE:
            self.get_logger().warn("Running in simulation mode - edcon not available")
            return
        
        # Enable edcon logging
        if EdconLogging:
            EdconLogging()
        
        try:
            # Connect servos via ComModbus + MotionHandler
            for servo_id, ip in self.config.servo_ips.items():
                try:
                    com = ComModbus(
                        ip_address=ip,
                        cycle_time=60,
                        timeout_ms=15000
                    )
                    # Configure telegram 111
                    current_tg = com.read_pnu(3490, 0)
                    if current_tg != 111:
                        self.get_logger().info(f"Servo {servo_id}: Telegram {current_tg} → 111")
                        com.write_pnu(3490, 0, 111)
                    else:
                        self.get_logger().info(f"Servo {servo_id}: Telegram 111 ✅")
                    mot = MotionHandler(com)
                    
                    # Initialize with retry to handle temporary connectivity issues
                    safe_try(
                        lambda: mot.acknowledge_faults(),
                        retries=3, backoff=2,
                        logger=self.get_logger()
                    )
                    # Defer enable_powerstage to HOMING state
                    self.servos[servo_id] = mot
                    self.get_logger().info(
                        f"✅ Connected to Servo {servo_id} at {ip} "
                        f"(Modbus TCP, cycle=60, timeout=15000ms)"
                    )
                except ConnectionError as e:
                    self.get_logger().error(
                        f"❌ Failed to connect Servo {servo_id} at {ip} "
                        f"after retries: {e}"
                    )
                except Exception as e:
                    self.get_logger().error(f"❌ Failed to connect Servo {servo_id} at {ip}: {e}")
            
            # Connect IO module (retry vì CPX-AP chỉ cho 1 Modbus TCP connection đồng thời)
            if CPXAP_AVAILABLE:
                _io_retries = 5
                _io_connected = False
                for _attempt in range(1, _io_retries + 1):
                    try:
                    # cycle_time=0.5s — tăng từ mặc định 10ms lên 500ms để giảm tải
                    # CPX-AP chỉ chấp nhận 1 kết nối TCP, nếu đọc quá nhanh sẽ drop
                        self.io_module = CpxAp(ip_address=self.config.io_ip, cycle_time=0.5)
                        self.get_logger().info(f"✅ Connected to IO Module at {self.config.io_ip}")
                        # ✅ Start background IO reader (prevents control loop from ever blocking on IO reads)
                        threading.Thread(target=self._io_bg_reader_loop, daemon=True, name="io_bg_reader").start()
                        _io_connected = True
                        break
                    except Exception as e:
                        self.get_logger().warn(
                            f"⚠️ IO Module connect attempt {_attempt}/{_io_retries} failed: {e}"
                        )
                        if _attempt < _io_retries:
                            time.sleep(3.0)
                if not _io_connected:
                    self.get_logger().error(f"❌ Failed to connect IO Module after {_io_retries} attempts")
            
        except Exception as e:
            self.get_logger().error(f"Hardware connection error: {e}")
    
    def destroy_node(self):
        """[FIX-5] Cleanup: stop all servos FIRST, then close connections."""
        self.get_logger().info("Shutting down — stopping servos and closing connections...")
        # Stop all moving servos first (safety)
        for servo_id in list(self.servos.keys()):
            try:
                with self._servo_lock:
                    self.servos[servo_id].stop_motion_task()
                self.get_logger().info(f"  Servo {servo_id} stopped")
            except Exception as e:
                self.get_logger().warn(f"  Servo {servo_id} stop error: {e}")
        # Then close connections
        for servo_id, mot in self.servos.items():
            try:
                with self._servo_lock:
                    mot.shutdown()
                self.get_logger().info(f"  Servo {servo_id} connection closed")
            except Exception as e:
                self.get_logger().warn(f"  Servo {servo_id} shutdown error: {e}")
        super().destroy_node()

    # ════════════════════════════════════════════════════════════════
    # [FIX-1] Thread-safe Modbus wrapper
    # ════════════════════════════════════════════════════════════════

    def _servo_call(self, func):
        """Run func() under _servo_lock. Prevents race conditions on Modbus TCP."""
        with self._servo_lock:
            return func()

    # ════════════════════════════════════════════════════════════════
    # [FIX-8] Watchdog
    # ════════════════════════════════════════════════════════════════

    def _watchdog_tick(self):
        """Called every 5s by timer. Warns if control loop is not ticking."""
        now = time.time()
        gap = now - self._watchdog_last_tick
        if gap > 3.0:
            self.get_logger().error(
                f"🚨 WATCHDOG: Control loop không chạy trong {gap:.1f}s! "
                "Có thể bị block bởi move_servo(wait=True) hoặc Modbus timeout. "
                f"State hiện tại: {self.state.name}"
            )
            self._notify_gui('error', '🚨 WATCHDOG: Loop bị treo!',
                             f'Control loop silent {gap:.1f}s — state={self.state.name}')

    # start_button_callback → định nghĩa đầy đủ bên dưới (~line 1350)
    # stop_button_callback  → định nghĩa đầy đủ bên dưới (~line 1390)
    
    def change_tray_input_callback(self, msg: Bool):
        """Handle /robot/change_tray - Robot yêu cầu thay khay input
        AUTO: sau row 5 refill buffer xong
        AI: khi không còn row nào full 8/8
        """
        if msg.data:
            self.change_tray_input_signal = True
            self.get_logger().info("📥 Received /robot/change_tray → State 2 trigger queued")
    
    def motion_busy_callback(self, msg: Bool):
        """Handle /robot/motion_busy - INTERLOCK tracking
        True = robot đang thực hiện motion (KHÔNG được thay khay)
        False = robot idle (an toàn để thay khay)
        """
        self.robot_motion_busy = msg.data
    
    def output_tray_full_callback(self, msg: Bool):
        """Handle /vision/change_tray_output hoặc /robot/done_tray_output"""
        if msg.data:
            self.output_tray_full = True
            self.change_tray_output_signal = True
            self.get_logger().warn("🚨 Output tray FULL detected → State 3 trigger queued")
    
    def tray_loaded_ack_callback(self, msg: Bool):
        """Handle /robot/tray_loaded_ack - Robot xác nhận đã nhận new_tray_loaded"""
        if msg.data:
            self._tray_loaded_ack = True
            self.get_logger().info("✅ ACK received: Robot confirmed new_tray_loaded")
    
    def last_tray_ack_callback(self, msg: Bool):
        """Handle /robot/last_tray_ack - Robot xác nhận đã nhận is_last_tray"""
        if msg.data:
            self._last_tray_ack = True
            self.get_logger().info("✅ ACK received: Robot confirmed is_last_tray")
    
    def last_batch_complete_callback(self, msg: Bool):
        """Handle /robot/last_batch_complete - Robot đã xử lý xong batch cuối cùng"""
        if msg.data:
            self.last_batch_complete = True
            self.get_logger().info("🏆 Robot báo: LAST BATCH COMPLETE - Sẵn sàng kết thúc sau State 3")
    
    def publish_with_retry(self, publisher, msg, topic_name: str):
        """Publish message với retry (gọi trong from process_state loop).
        Mỗi lần vào state, gửi lại nếu chưa ACK và đủ interval.
        """
        now = time.time()
        
        if self._pub_retry_timer is None:
            # Lần đầu: pub ngay + start timer
            publisher.publish(msg)
            self._pub_retry_count = 1
            self._pub_retry_timer = now
            self.get_logger().info(f"📤 Published {topic_name} (attempt {self._pub_retry_count}/{self._pub_retry_max})")
        elif self._pub_retry_count < self._pub_retry_max:
            # Retry nếu đủ interval
            if now - self._pub_retry_timer >= self._pub_retry_interval:
                publisher.publish(msg)
                self._pub_retry_count += 1
                self._pub_retry_timer = now
                self.get_logger().info(f"📤 Retry {topic_name} (attempt {self._pub_retry_count}/{self._pub_retry_max})")
    
    def reset_pub_retry(self):
        """Reset retry state sau khi nhận ACK hoặc transition"""
        self._pub_retry_count = 0
        self._pub_retry_timer = None
    
    def log_system_status(self):
        """Log current system status for debugging"""
        total_rows_processed = (self.current_tray - 1) * self.rows_per_tray + self.current_row - 1
        total_rows_available = self.detected_trays * self.rows_per_tray
        
        self.get_logger().info(
            f"📊 SYSTEM STATUS:\n"
            f"   Detected Trays: {self.detected_trays}\n"
            f"   Current Tray: {self.current_tray}/{self.detected_trays}\n"
            f"   Current Row: {self.current_row}/{self.rows_per_tray}\n"
            f"   Processed: {total_rows_processed}/{total_rows_available} rows\n"
            f"   Output Slot: {self.current_slot}/{self.max_slots_per_output_tray}\n"
            f"   Last Batch Mode: {self.last_batch_mode}"
        )
    
    def is_iny_safe_for_inx_move(self) -> bool:
        """Check if InY position is in safe zone for INX to move.
        INX chỉ được di chuyển khi INY actual position < iny_safe_zone.
        """
        iny_pos = self.get_servo_position(2)
        if iny_pos is None:
            self.get_logger().warn("Cannot read InY position for safety check")
            return False
        
        safe_zone = self.config.iny_safe_zone
        is_safe = iny_pos <= safe_zone
        if not is_safe:
            self.get_logger().warn(f"InY position {iny_pos:.1f}mm NOT in safe zone (threshold: {safe_zone}mm)")
        
        return is_safe
    
    def is_outx_safe_for_outy_move(self) -> bool:
        """Check if OutX position is safe for OutY to move (collision avoidance)"""
        outx_pos = self.get_servo_position(4)
        if outx_pos is None:
            self.get_logger().warn("Cannot read OutX position for safety check")
            return False
        
        is_safe = outx_pos <= self.config.outx_safe_position_threshold
        if not is_safe:
            self.get_logger().warn(f"OutX position {outx_pos:.1f}mm is not safe for OutY movement (threshold: {self.config.outx_safe_position_threshold}mm)")
        
        return is_safe
    
    def _notify_gui(self, level, title, detail=""):
        """Publish notification to GUI with rate-limiting (max 1 per 0.5s per title)."""
        now = time.time()
        last = self._last_notify_time.get(title, 0)
        if now - last < 0.5:  # Rate limit: skip nếu cùng title gửi < 0.5s trước
            return
        self._last_notify_time[title] = now
        try:
            msg = String()
            msg.data = json.dumps({"level": level, "title": title, "detail": detail})
            self.pub_gui_notify.publish(msg)
        except Exception:
            pass
    
    def _publish_positions(self):
        """Publish live servo positions at 2Hz for GUI display."""
        # Guard: không đọc Modbus khi hệ thống idle và không có ai đang xem
        _active_state = self.state not in (
            SystemState.IDLE, SystemState.S2_WAIT_TRIGGER, SystemState.S3_WAIT_TRIGGER
        )
        if not _active_state and self.operation_mode == 'idle':
            return  # Tiết kiệm 10 Modbus reads/s khi toàn bộ hệ thống nghỉ
        try:
            positions = {}
            for sid in self.servos:
                pos = self.get_servo_position(sid)
                if pos is not None:
                    positions[str(sid)] = round(pos, 2)
            msg = String()
            msg.data = json.dumps(positions)
            self.pub_servo_pos.publish(msg)
        except Exception:
            pass
    
    def home_all_servos(self) -> bool:
        """Home all servo motors in 2 phases:
        Phase 1: INY (2) + OUTY (5) — trục Y home trước
        Phase 2: Servo3 (3) + INX (1) + OUTX (4) — platform + trục X
        """
        self.get_logger().info("Homing all servos...")
        self._notify_gui("info", "🏠 Homing", "Bắt đầu homing tuần tự...")


        SERVO_NAMES = {1: "InX", 2: "InY", 3: "Platform", 4: "OutX", 5: "OutY"}
        
        # Homing: trục Y trước, rồi trục X + Platform
        phases = [
            ([2, 5], "Step 1: Trục Y — InY (Pos1) + OutY (Pos2)"),
            ([1, 4, 3], "Step 2: Trục X + Platform — InX (Pos1) + OutX (Pos2) + Platform"),
        ]
        
        try:
            for phase_servos, phase_name in phases:
                # Filter to only connected servos
                active = [(sid, self.servos[sid]) for sid in phase_servos if sid in self.servos]
                if not active:
                    continue
                
                names = ", ".join(SERVO_NAMES.get(s, f"S{s}") for s, _ in active)
                self.get_logger().info(f"🏠 {phase_name} [{names}]")
                self._notify_gui("info", f"🏠 {phase_name}", names)
                
                # Enable powerstage — skip only InX (servo 1) if PLC control denied
                enabled_active = []
                for servo_id, mot in active:
                    name = SERVO_NAMES.get(servo_id, f"S{servo_id}")
                    try:
                        safe_try(
                            lambda m=mot: m.acknowledge_faults(),
                            retries=2, backoff=1, logger=self.get_logger()
                        )
                        safe_try(
                            lambda m=mot: m.enable_powerstage(),
                            retries=2, backoff=1, logger=self.get_logger()
                        )
                        # Wait briefly for PLC control (2s)
                        _ready = False
                        for _ in range(40):
                            if hasattr(mot, 'ready_for_motion') and mot.ready_for_motion():
                                _ready = True; break
                            time.sleep(0.05)
                        if not _ready and hasattr(mot, 'ready_for_motion'):
                            raise RuntimeError("PLC control denied after 2s")
                        enabled_active.append((servo_id, mot))
                    except Exception as e:
                        if servo_id == 1:  # InX — known hardware issue, skip gracefully
                            self.get_logger().warn(f"⚠️ InX (Servo 1) enable failed ({e}) — SKIP homing, assume at home")
                            self._notify_gui('warn', '⚠️ InX homing skipped', f'PLC control denied: {e}')
                            try:
                                self.zero_offset[1] = mot.current_position()
                            except:
                                self.zero_offset[1] = 0
                        else:
                            # Other servos: fail properly
                            err = f"{name}: Enable failed — {e}"
                            self.get_logger().error(f"❌ {err}")
                            self._notify_gui('error', f'❌ {name} enable failed', err)
                            return False
                
                active = enabled_active
                if not active:
                    continue
                
                # Start homing (nonblocking) for all in phase
                for servo_id, mot in active:
                    try:
                        name = SERVO_NAMES.get(servo_id, f"S{servo_id}")
                        self.get_logger().info(f"  {name} homing...")
                        safe_try(
                            lambda m=mot: m.referencing_task(nonblocking=True),
                            retries=3, backoff=2, logger=self.get_logger()
                        )
                    except Exception as e:
                        name = SERVO_NAMES.get(servo_id, f"S{servo_id}")
                        err = f"Servo {name} (S{servo_id}): Homing start failed — {e}"
                        self.get_logger().error(f"❌ {err}")
                        self._notify_gui("error", "❌ Homing Failed: Start Error", err)
                        return False
                
                # Wait for all in phase to complete (skip those that don't home in time)
                start_time = time.time()
                while time.time() - start_time < self.config.homing_timeout:
                    all_done = True
                    for servo_id, mot in active:
                        if not mot.referenced():
                            all_done = False
                            break
                    if all_done:
                        break
                    time.sleep(0.1)
                else:
                    # Timeout — stop motion and skip servos that didn't complete
                    still_active = []
                    for servo_id, mot in active:
                        name = SERVO_NAMES.get(servo_id, f"S{servo_id}")
                        if not mot.referenced():
                            self.get_logger().warn(f"⚠️ {name} homing TIMEOUT ({self.config.homing_timeout}s) — STOP + SKIP, assume at home")
                            self._notify_gui('warn', f'⚠️ {name} homing timeout', 'Stopped — assume at home')
                            try:
                                mot.stop_motion_task()
                            except Exception:
                                pass
                            try:
                                self.zero_offset[servo_id] = mot.current_position()
                            except Exception:
                                self.zero_offset[servo_id] = 0
                        else:
                            still_active.append((servo_id, mot))
                    active = still_active
                
                # Wait for servos to settle before reading zero offset
                time.sleep(0.5)
                
                # Store zero offsets for this phase
                for servo_id, mot in active:
                    self.zero_offset[servo_id] = mot.current_position()
                    abs_mm = self.zero_offset[servo_id] / COUNTS_PER_MM
                    name = SERVO_NAMES.get(servo_id, f"S{servo_id}")
                    self.get_logger().info(f"  ✅ {name} = 0mm (encoder: {abs_mm:.1f}mm)")
            
            self.get_logger().info("✅ All servos homed successfully")
            self._notify_gui("info", "✅ Homing Complete", "Tất cả servo đã home thành công.")
            return True

        except Exception as e:
            self.get_logger().error(f"Homing error: {e}")
            self._notify_gui("error", "❌ Homing Failed", str(e))
            return False
    
    def reset_faults_callback(self, msg):
        """Topic /providesystem/reset_faults — Reset lỗi tất cả servo."""
        SERVO_NAMES = {1: "InX", 2: "InY", 3: "Platform", 4: "OutX", 5: "OutY"}
        results = []
        for servo_id, mot in self.servos.items():
            name = SERVO_NAMES.get(servo_id, f"S{servo_id}")
            try:
                self._servo_call(lambda m=mot: m.acknowledge_faults())  # [FIX-1]
                results.append(f"{name}: ✅")
            except Exception as e:
                results.append(f"{name}: ❌ {e}")
        self._notify_gui("info", "🔄 Fault Reset", " | ".join(results))
    
    def _ensure_drive_ready(self, mot, servo_id: int, timeout: float = 5.0) -> bool:
        """[FIX-1] Acknowledge faults + enable powerstage, all under _servo_lock."""
        for attempt in range(3):
            try:
                self._servo_call(lambda m=mot: m.acknowledge_faults())
                self._servo_call(lambda m=mot: m.enable_powerstage())
                start = time.time()
                while time.time() - start < timeout:
                    ready = self._servo_call(
                        lambda m=mot: m.ready_for_motion() if hasattr(m, 'ready_for_motion') else True
                    )
                    if ready:
                        return True
                    time.sleep(0.05)
                if not hasattr(mot, 'ready_for_motion'):
                    return True
                self.get_logger().warn(f"⚠️ Servo {servo_id}: attempt {attempt+1}/3 not ready, retrying...")
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().warn(f"⚠️ Servo {servo_id}: _ensure_drive_ready attempt {attempt+1} error: {e}")
                time.sleep(0.5)
        self.get_logger().error(f"❌ Servo {servo_id}: drive not ready after 3 attempts")
        return False

    def move_servo(self, servo_id: int, position: float, wait: bool = True) -> bool:
        """Move servo to position (mm) with soft limit enforcement.
        In manual mode: returns True immediately (no blocking wait)."""
        # SOFT LIMIT CHECK
        limit = self.config.servo_limits.get(servo_id, 999999.0)
        if position > limit:
            self.get_logger().error(f"Servo {servo_id}: target {position:.1f}mm EXCEEDS limit {limit:.1f}mm! BLOCKED.")
            return False
        
        if servo_id not in self.servos:
            if self.manual_mode:
                self.get_logger().info(f"[SIM] Servo {servo_id} -> {position:.1f}mm (no hw)")
            return True  # Simulation — always OK

        # [FIX-6] Set motion flag before non-blocking move
        if servo_id == 1:
            self._inx_moving = True
        elif servo_id == 2:
            self._iny_moving = True

        try:
            mot = self.servos[servo_id]
            offset = self.zero_offset.get(servo_id, 0)
            pos_counts = offset + int(position * COUNTS_PER_MM)

            def attempt_move():
                if not self._ensure_drive_ready(mot, servo_id):
                    raise ConnectionError(f"Servo {servo_id}: drive not ready (PLC control denied)")
                success = self._servo_call(  # [FIX-1]
                    lambda m=mot, p=pos_counts: m.position_task(p, DEFAULT_VELOCITY, absolute=True, nonblocking=True)
                )
                if not success:
                    raise ConnectionError(f"Servo {servo_id} rejected position_task (not ready)")
                return True

            safe_try(attempt_move, retries=3, backoff=1, logger=self.get_logger())

            if wait:
                start_time = time.time()
                while time.time() - start_time < self.config.move_timeout:
                    if self._servo_call(lambda m=mot: m.target_position_reached()):  # [FIX-1]
                        return True
                    try:
                        self._publish_positions()
                    except Exception:
                        pass
                    time.sleep(0.05)
                self.get_logger().error(f"Servo {servo_id} move timeout")
                return False
            return True

        except Exception as e:
            self.get_logger().error(f"Servo {servo_id} move error: {e}")
            return False
        finally:
            # [FIX-6] Clear motion flag after blocking wait (always runs)
            if wait:
                if servo_id == 1:
                    self._inx_moving = False
                elif servo_id == 2:
                    self._iny_moving = False


    def jog_servo(self, servo_id: int, velocity: float):
        """Jog servo using jog_task(). [FIX-1] Uses _servo_call for Modbus safety."""
        limit = self.config.servo_limits.get(servo_id, 999999.0)

        # InX (servo 1): check InY safe zone trước khi jog
        if servo_id == 1:
            iny_pos = self.get_servo_position(2)
            if iny_pos is not None and iny_pos > self.config.iny_safe_zone:
                self.get_logger().error(
                    f"🚨 InX JOG bị chặn — InY đang ở {iny_pos:.1f}mm "
                    f"(phải < {self.config.iny_safe_zone}mm để an toàn)."
                )
                self._notify_gui('warn', '🚨 InX bị chặn', f'InY = {iny_pos:.1f}mm > {self.config.iny_safe_zone}mm safe zone')
                return

        # Soft limit check (only if homed)
        if servo_id in self.zero_offset:
            current_pos = self.get_servo_position(servo_id)
            if current_pos is not None and current_pos >= limit and velocity > 0:
                self.get_logger().error(f"🚨 Servo {servo_id}: {current_pos:.1f}mm AT LIMIT {limit:.1f}mm! Jog BLOCKED.")
                self.stop_servo(servo_id)
                return
        else:
            self.get_logger().warn(f"⚠️ Servo {servo_id} chưa home — soft limit KHÔNG áp dụng, cẩn thận!")

        if servo_id not in self.servos:
            return

        try:
            mot = self.servos[servo_id]
            self._ensure_drive_ready(mot, servo_id)
            if servo_id in (2, 5):
                self._servo_call(lambda m=mot: m.jog_task(jog_positive=(velocity > 0), jog_negative=(velocity < 0), duration=0))  # [FIX-1]
            else:
                self._servo_call(lambda m=mot: m.jog_task(jog_positive=(velocity < 0), jog_negative=(velocity > 0), duration=0))  # [FIX-1]
        except Exception as e:
            self.get_logger().error(f"Servo {servo_id} jog error: {e}")
    
    def stop_servo(self, servo_id: int):
        """Stop servo immediately. [FIX-1/6] Uses _servo_call, clears motion flags."""
        if servo_id not in self.servos:
            return
        try:
            self._servo_call(lambda m=self.servos[servo_id]: m.stop_motion_task())  # [FIX-1]
        except Exception as e:
            self.get_logger().error(f"Servo {servo_id} stop error: {e}")
        finally:
            # [FIX-6] Clear motion flags on stop
            if servo_id == 1:
                self._inx_moving = False
            elif servo_id == 2:
                self._iny_moving = False
    
    def get_servo_position(self, servo_id: int) -> Optional[float]:
        """Get current servo position in mm. No lock — read-only, each servo has own TCP socket."""
        if servo_id not in self.servos:
            return 0.0  # Simulation
        try:
            counts = self.servos[servo_id].current_position()  # Direct call, no lock
            offset = self.zero_offset.get(servo_id, 0)
            return (counts - offset) / COUNTS_PER_MM
        except Exception as e:
            if self.manual_mode:
                return 0.0
            self.get_logger().error(f"Failed to get position from Servo {servo_id}: {e}")
            return None

    def is_inx_safe_for_iny_move(self) -> bool:
        """[FIX-6] Returns True if InX is NOT actively moving (safe to move INY)."""
        return not self._inx_moving

    def is_iny_safe_for_inx_move(self) -> bool:
        """[FIX-6] Returns True if INY is in safe zone AND not actively moving."""
        if self._iny_moving:
            return False  # INY currently moving — block INX
        iny_pos = self.get_servo_position(2)
        return iny_pos is None or iny_pos <= self.config.iny_safe_zone

    def find_nearest_row(self, current_pos: float, row_positions_dict: dict) -> int:
        """Find the next row UP (higher mm) from current position.
        
        When S4 triggers while jogging downward (increasing mm), we snap
        to the row immediately above — i.e., the smallest row position
        that is >= current_pos.
        
        Example: rows = {8:250, 7:300, 6:350, ...}
                 current_pos = 255mm → snap to row 7 (300mm)
        
        Args:
            current_pos: Current servo position (mm)
            row_positions_dict: Dict of row positions (e.g., iny_input_stack)
        
        Returns:
            Row number of the first row at or above current position (by mm)
        """
        # Sort rows by ascending position value (lower rows first by position)
        sorted_rows = sorted(row_positions_dict.items(), key=lambda kv: kv[1])  # (row, pos) sorted by pos
        
        # Find the first row position GREATER THAN current_pos
        for row, pos in sorted_rows:
            if pos >= current_pos:
                self.get_logger().info(
                    f"📐 Row snap: current={current_pos:.1f}mm → Row {row} @ {pos:.1f}mm (next UP)"
                )
                return row
        
        # Fallback: already past all rows → use last (highest mm) row
        last_row, last_pos = sorted_rows[-1]
        self.get_logger().warn(
            f"⚠️ Beyond all rows at {current_pos:.1f}mm → fallback Row {last_row} @ {last_pos:.1f}mm"
        )
        return last_row
    
    def check_has_trays_remaining(self) -> bool:
        """Double-check: còn khay trên input stack không?
        
        Check S1 (input conveyor đầu), S2 (mid), S3 (end):
        - Nếu BẤT KỲ sensor ON → còn khay trên băng tải
        - Nếu TẤT CẢ OFF → hết khay
        
        Backup check: stack_row_index > 1 → còn khay trong stack
        """
        s1_on = self.sensor_manager.get_sensor(1)
        s2_on = self.sensor_manager.get_sensor(2)
        s3_on = self.sensor_manager.get_sensor(3)
        stack_not_at_bottom = (self.stack_row_index > 1)
        
        self.has_trays_remaining = s1_on or s2_on or s3_on or stack_not_at_bottom
        
        self.get_logger().info(f"🔍 Tray check: S1={'ON' if s1_on else 'OFF'}, "
                     f"S2={'ON' if s2_on else 'OFF'}, S3={'ON' if s3_on else 'OFF'}, "
                     f"stack_row={self.stack_row_index}, remaining={self.has_trays_remaining}")
        
        return self.has_trays_remaining
    
    def start_cylinder_timer(self):
        """Start cylinder timeout timer"""
        self._cylinder_start_time = time.time()
    
    def check_cylinder_timeout(self) -> bool:
        """[FIX-2] Check if cylinder operation has timed out (CYLINDER_TIMEOUT_S = 15s).
        Returns True if TIMED OUT.
        """
        if self._cylinder_start_time is None:
            return False
        elapsed = time.time() - self._cylinder_start_time
        if elapsed > CYLINDER_TIMEOUT_S:
            self.get_logger().error(
                f"🚨 CYLINDER TIMEOUT: {elapsed:.1f}s > {CYLINDER_TIMEOUT_S}s! "
                "Xi-lanh có thể bị kẹt hoặc sensor bị lỗi. Chuyển ERROR."
            )
            self._notify_gui('error', '🚨 Cylinder Timeout',
                             f'Xi-lanh không phản hồi sau {CYLINDER_TIMEOUT_S}s')
            return True
        return False


    def start_motion_delay(self, next_state, duration: float = None):
        """Start a non-blocking delay before transitioning to next_state.
        Sets self.state to S1_MOTION_DELAY (or equivalent) and waits.
        """
        self._motion_delay_start = time.time()
        self._motion_delay_duration = duration if duration is not None else 2.0
        self._motion_delay_next_state = next_state
        self.state = SystemState.MOTION_DELAY
    
    def check_motion_delay(self) -> bool:
        """Returns True when delay is complete and transitions to next_state automatically."""
        if self._motion_delay_start is None:
            return True
        elapsed = time.time() - self._motion_delay_start
        if elapsed >= self._motion_delay_duration:
            self._motion_delay_start = None
            if self._motion_delay_next_state is not None:
                self.state = self._motion_delay_next_state
                self._motion_delay_next_state = None
            return True
        return False
    
    def start_iny_jog(self, velocity: float):
        """Start INY jog only if not already jogging (tránh gọi 50 lần/giây)"""
        if not self._iny_jog_active:
            self.jog_servo(2, velocity)
            self._iny_jog_active = True
    
    def stop_iny_jog(self):
        """Stop INY jog and reset flag"""
        self.stop_servo(2)
        self._iny_jog_active = False
    
    def start_outy_jog(self, velocity: float):
        """Start OUTY jog only if not already jogging"""
        if not self._outy_jog_active:
            self.jog_servo(5, velocity)
            self._outy_jog_active = True
    
    def stop_outy_jog(self):
        """Stop OUTY jog and reset flag"""
        self.stop_servo(5)
        self._outy_jog_active = False
    
    def start_servo3_jog(self, velocity: float):
        """Start Servo 3 jog only if not already jogging"""
        if not self._servo3_jog_active:
            self.jog_servo(3, velocity)
            self._servo3_jog_active = True
    
    def stop_servo3_jog(self):
        """Stop Servo 3 jog and reset flag"""
        self.stop_servo(3)
        self._servo3_jog_active = False
    
    def is_outy_safe_for_outx_move(self) -> bool:
        """Check OUTY nhỏ hơn safe zone để OUTX được di chuyển"""
        outy_pos = self.get_servo_position(5)
        if outy_pos is None:
            return False
        return outy_pos <= self.config.outy_safe_zone
    
    def confirm_output_load_callback(self, req, response):
        """Service /providesystem/confirm_output_load - HMI xác nhận đã cấp khay output mới"""
        if req.data:
            self._confirm_load_received = True
            self.get_logger().info("✅ HMI confirmed: Output tray loaded")
            response.success = True
            response.message = "Output load confirmed"
            return response
        response.success = False
        response.message = "Invalid request"
        return response
    
    def hmi_resume_callback(self, msg):
        """Topic /providesystem/hmi_resume - RESUME: tiếp tục sau pause + xác nhận cycle mới"""
        if msg.data:
            self._hmi_resume_confirmed = True   # Tiếp tục cycle (sản phẩm, trạng thái S3...)
            self._system_paused = False          # Unpause nếu đang bị pause
            self.get_logger().info("\u25b6\ufe0f  RESUME: _hmi_resume_confirmed=True, _system_paused=False")

    def confirm_button_callback(self, msg):
        """Topic /system/confirm_button - CONFIRM: xác nhận đã cấp khạy (chỉ Servo 3)"""
        if msg.data:
            self._confirm_load_received = True
            self.get_logger().info("✅ CONFIRM: đã cấp khạy — _confirm_load_received=True")

    def start_button_callback(self, msg):
        """Topic /system/start_button - Chạy lại từ đầu (bắt buộc HOMING trước)"""
        if not msg.data:
            return
        # ✅ Phải chọn mode trước (AUTO hoặc MANUAL) — không cho home khi idle
        if self.operation_mode == 'idle':
            self.get_logger().warn('⚠️ START bị chặn — chọn AUTO hoặc MANUAL mode trước!')
            self._notify_gui('warn', '⚠️ Chưa chọn mode', 'Chọn AUTO hoặc MANUAL trước khi nhấn START')
            return
        # ✅ Chặn double-trigger: nếu đang HOMING_RUNNING thì bỏ qua
        if self.state == SystemState.HOMING_RUNNING:
            self.get_logger().warn('⚠️ START ignored — đang trong HOMING, chờ hoàn tất')
            return
        self._system_paused = False
        # ✅ Reset state flags — KHÔNG stop_servo ở đây (blocking 10s!)
        # home_all_servos() trong HOMING thread sẽ tự stop servo trước khi home
        self.zero_offset = {}
        self._pos1_done = False
        self._pos2_done = False
        self._pos2_state = "IDLE"
        self._p2_move_cmd_sent = False
        self._p2_move_cmd_time = 0.0
        self._servo3_jog_active = False
        self.state = SystemState.HOMING
        self.get_logger().info(f'▶️  START ({self.operation_mode}): HOMING bắt đầu')
        self._notify_gui('info', '▶️ START — HOMING', f'Mode: {self.operation_mode} — Đang homing...')

    def stop_button_callback(self, msg):
        """Topic /system/stop_button - Dừng hết, về IDLE"""
        if not msg.data:
            return
        self._system_paused = False
        # Dừng toàn bộ servo
        for sid in list(self.servos.keys()):
            try:
                self.stop_servo(sid)
            except Exception:
                pass
        self.stop_iny_jog()
        self.stop_servo3_jog()
        self.state = SystemState.IDLE
        self.get_logger().warn("⏹️  STOP: Tất cả servo dừng → IDLE")
        self._notify_gui('warn', '⏹️ STOP', 'Hệ thống dừng — chờ lệnh START')

    def pause_button_callback(self, msg):
        """Topic /system/pause_button - Chỉ PAUSE (không toggle). Dùng RESUME để resume."""
        if not msg.data:
            return
        self._system_paused = True
        self.get_logger().warn(f"⏸\ufe0f  PAUSE: Tạm dừng tại {self.state.name} — nhấn RESUME để tiếp tục")
        self._notify_gui('warn', '\u23f8\ufe0f PAUSE', f'Tạm dừng — nhấn RESUME để tiếp tục')


    def set_operation_mode_callback(self, msg):
        """Topic /providesystem/set_operation_mode - Chọn mode: 'auto', 'jog', 'manual', 'idle'
        
        Modes:
            auto   — Chạy tự động, tự home, không sim sensor
            jog    — Điều khiển tay servo, KHÔNG tự home (dùng 'home N' để home từng servo)
            manual — Test state machine step-by-step, tự home, sim sensor ON
            idle   — Chờ lệnh
        """
        mode = msg.data.strip().lower()
        
        if mode not in ('auto', 'jog', 'manual', 'idle'):
            self.get_logger().error(f"❌ Invalid mode '{mode}'. Use: auto, jog, manual, idle")
            return
        
        old_mode = self.operation_mode
        
        # ===== GUARD: Optionally warn, but allow mode switch to proceed =====
        if mode in ('auto', 'manual') and old_mode not in ('idle', mode):
            self.get_logger().warn(f"⚠️ Switching direct to '{mode}' from '{old_mode}'. (Recommended to stop/idle first but allowed)")
            # Allow flow to continue instead of returning
        
        # Stop any active jog when leaving jog mode
        if old_mode == 'jog' and mode != 'jog' and self._jog_active_servo is not None:
            self.stop_servo(self._jog_active_servo)
            self._jog_active_servo = None
        
        self.operation_mode = mode
        
        # ── AUTO ──────────────────────────────────────────────────────
        if mode == 'auto':
            self.manual_mode = False
            self.manual_auto_run = False
            self._sim_sensors.clear()  # AUTO uses real sensors only
            self.zero_offset = {}  # Will be set during homing (triggered by START)
            self.state = SystemState.IDLE
            self.get_logger().info("🤖 MODE: AUTO — Sẵn sàng. Nhấn START để Homing + bắt đầu chu trình.")
            self._notify_gui('info', '🤖 AUTO mode', 'Nhấn START để Homing và bắt đầu')
        
        # ── JOG ───────────────────────────────────────────────────────
        elif mode == 'jog':
            self.manual_mode = False   # JOG không phải manual mode
            self.manual_auto_run = False
            self.get_logger().info(" MODE: JOG — KHÔNG tự home. Dùng lệnh 'home N' để home từng servo.")
            self.get_logger().info("  Format: '1 +'  '1 -'  '1 stop'  'home 1'  'vel 500'  'pos'")
            self.get_logger().info("  Servo IDs: 1=InX, 2=InY, 3=PutTray, 4=OutX, 5=OutY")
            self._notify_gui('info', ' JOG mode', 'Dùng home N để home servo trước khi jog')
            
            # Log vị trí hiện tại nếu đã home
            if self.zero_offset:
                for sid in self.servos:
                    pos = self.get_servo_position(sid)
                    if pos is not None:
                        self.get_logger().info(f"  Servo {sid} = {pos:.2f}mm")
            else:
                self.get_logger().warn("  ⚠️ Chưa home — vị trí chưa chính xác. Dùng 'home N' để home.")
        
        # ── MANUAL ────────────────────────────────────────────────────
        elif mode == 'manual':
            self.manual_mode = True    # MANUAL mới set manual_mode = True
            self.manual_auto_run = False
            self.step_pending = False
            self.zero_offset = {}
            self.state = SystemState.IDLE
            self.get_logger().info("🔧 MODE: MANUAL — Sẵn sàng. Nhấn START để Homing rồi test state machine.")
            self.get_logger().info("  Sau khi home xong: dùng goto_state để nhảy state")
            self.get_logger().info("  Sim sensor hoạt động trong mode này")
            self._notify_gui('info', '🔧 MANUAL mode', 'Nhấn START để Homing trước')
        
        # ── IDLE ──────────────────────────────────────────────────────
        elif mode == 'idle':
            self.manual_mode = False
            self.get_logger().info("⏸️  MODE: IDLE")
            self._notify_gui('info', '⏸️ IDLE mode', '')
        
        self.get_logger().info(f"  Mode: {old_mode} → {mode}")
    
    def jog_cmd_callback(self, msg):
        """Topic /providesystem/jog_cmd - Jog servo interactively
        
        Format:
            '1 +'     → Jog servo 1 positive (xa home)
            '1 -'     → Jog servo 1 negative (về home)
            '1 stop'  → Stop servo 1
            'stop'    → Stop all
            'vel 500' → Set jog velocity
            'pos'     → Show all positions
        
        Usage:
            ros2 topic pub --once /providesystem/jog_cmd std_msgs/String 'data: "1 +"'
        """
        if self.operation_mode not in ('jog', 'manual'):
            self.get_logger().warn(f"❌ Jog command ignored - cần JOG hoặc MANUAL mode (hiện tại: '{self.operation_mode}')")
            self._notify_gui('warn', '⚠️ Chưa ở JOG/MANUAL mode', f'Chọn JOG hoặc MANUAL mode trước (hiện: {self.operation_mode})')
            return
        
        cmd = msg.data.strip()
        
        # 'pos' - show all positions
        if cmd == 'pos':
            for sid in self.servos:
                pos = self.get_servo_position(sid)
                if pos is not None:
                    self.get_logger().info(f"  Servo {sid} = {pos:.2f}mm")
            return
        
        # 'stop' - stop all
        if cmd == 'stop':
            for sid in self.servos:
                self.stop_servo(sid)
            self._jog_active_servo = None
            self.get_logger().info("⏹️  All servos stopped")
            return
        
        # 'clear N' - clear fault on single servo
        if cmd.startswith('clear'):
            try:
                sid = int(cmd.split()[1])
            except:
                self.get_logger().error("❌ Format: 'clear 1'")
                return
            if sid not in self.servos:
                self.get_logger().error(f"❌ Servo {sid} not connected")
                return
            SERVO_NAMES = {1: "InX", 2: "InY", 3: "Platform", 4: "OutX", 5: "OutY"}
            name = SERVO_NAMES.get(sid, f"S{sid}")
            try:
                self.servos[sid].acknowledge_faults()
                self.get_logger().info(f"✅ {name} faults cleared")
                self._notify_gui("info", f"✅ {name} Faults Cleared", "")
            except Exception as e:
                self.get_logger().error(f"❌ {name} clear fault error: {e}")
                self._notify_gui("error", f"❌ {name} Clear Fault Failed", str(e))
            return
        
        # 'home N' - home single servo
        if cmd.startswith('home'):
            try:
                sid = int(cmd.split()[1])
            except:
                self.get_logger().error("❌ Format: 'home 1'")
                return
            if sid not in self.servos:
                self.get_logger().error(f"❌ Servo {sid} not connected")
                return
            SERVO_NAMES = {1: "InX", 2: "InY", 3: "Platform", 4: "OutX", 5: "OutY"}
            name = SERVO_NAMES.get(sid, f"S{sid}")
            def _home_single(servo_id, servo_name):
                mot = self.servos[servo_id]
                try:
                    self.get_logger().info(f"🏠 Homing {servo_name}...")
                    self._notify_gui("info", f"🏠 Homing {servo_name}...", "")
                    mot.acknowledge_faults()
                    mot.enable_powerstage()
                    mot.referencing_task(nonblocking=True)
                    start = time.time()
                    while time.time() - start < self.config.homing_timeout:
                        if mot.referenced():
                            break
                        time.sleep(0.1)
                    else:
                        self.get_logger().error(f"❌ {servo_name} homing timeout!")
                        self._notify_gui("error", f"❌ {servo_name} Homing Timeout", "Check limit sensor")
                        return
                    time.sleep(0.5)  # settle before reading offset
                    self.zero_offset[servo_id] = mot.current_position()
                    self.get_logger().info(f"✅ {servo_name} homed OK")
                    self._notify_gui("info", f"✅ {servo_name} Homed", "0.00 mm")
                except Exception as e:
                    self.get_logger().error(f"❌ {servo_name} homing error: {e}")
                    self._notify_gui("error", f"❌ {servo_name} Homing Failed", str(e))
            threading.Thread(target=_home_single, args=(sid, name), daemon=True).start()
            return
        
        # 'vel XXX' - set velocity
        if cmd.startswith('vel'):
            try:
                self._jog_velocity = int(cmd.split()[1])
                self.get_logger().info(f"✅ Jog velocity = {self._jog_velocity}")
            except:
                self.get_logger().error("❌ Format: 'vel 500'")
            return
        
        # 'N +' / 'N -' / 'N stop' / 'N + 50' - jog specific servo
        parts = cmd.split()
        if len(parts) < 2 or len(parts) > 3:
            self.get_logger().error(f"❌ Invalid: '{cmd}'. Use: '1 +', '1 - 50', '1 stop'")
            return
        
        try:
            servo_id = int(parts[0])
        except:
            self.get_logger().error(f"❌ Invalid servo ID: '{parts[0]}'")
            return
        
        direction = parts[1]
        
        # Optional velocity in 3rd part
        if len(parts) == 3 and direction != 'stop':
            try:
                self._jog_velocity = int(parts[2])
            except:
                pass
        
        if servo_id not in self.servos:
            self.get_logger().error(f"❌ Servo {servo_id} not connected")
            return
        
        if direction == 'stop':
            self.stop_servo(servo_id)
            pos = self.get_servo_position(servo_id)
            self._jog_active_servo = None
            self.get_logger().info(f"⏹️  Servo {servo_id} stopped at {pos:.2f}mm")
            return
        
        if direction not in ('+', '-'):
            self.get_logger().error(f"❌ Invalid direction: '{direction}'. Use: +, -, stop")
            return
        
        # Stop previous jog if different servo
        if self._jog_active_servo is not None and self._jog_active_servo != servo_id:
            self.stop_servo(self._jog_active_servo)
        
        # Jog direction matches FAS: + = positive, - = negative
        mot = self.servos[servo_id]
        
        # =================================================================
        # SAFETY INTERLOCK: InY downward jog blocked if Cyl1 is EXTENDED
        # =================================================================
        if servo_id == 2 and direction == '+':  # InY đi xuống (dương = xa home = xuống stack)
            s11_extended = self.sensor_manager.get_sensor(11)  # S11 = Cyl1+ Extend
            if s11_extended:
                self.get_logger().warn(
                    "🚫 [INTERLOCK] InY đi xuống bị chặn — Cylinder 1 đang EXTEND (S11 ON)! "
                    "Hãy retract cylinder trước."
                )
                self._notify_gui("warn", "⛔ InY bị chặn", "Cylinder 1 đang extend (S11 ON). Retract trước!")
                return
        # =================================================================
        
        if direction == '+':
            mot.jog_task(jog_positive=True, jog_negative=False, duration=0)
        else:
            mot.jog_task(jog_positive=False, jog_negative=True, duration=0)
        
        self._jog_active_servo = servo_id
        pos = self.get_servo_position(servo_id)
        self.get_logger().info(f"🎮 Jog Servo {servo_id} {direction} (vel={self._jog_velocity}) | pos={pos:.2f}mm")
    
    def pub_output_ready(self, ready: bool):
        """Publish output_tray_ready interlock và update flag"""
        self.output_tray_ready = ready
        self.pub_output_tray_ready.publish(Bool(data=ready))
        if ready:
            self.get_logger().info("✅ Output tray READY - Robot can place to output")
        else:
            self.get_logger().warn("🚫 Output tray NOT READY - Robot BLOCKED from place to output")
    
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    # Manual Testing Mode Callbacks
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    
    def set_manual_mode_callback(self, req, response):
        """Service /providesystem/set_manual_mode - Enable/disable manual testing"""
        self.manual_mode = req.data
        if req.data:
            self.get_logger().warn("🔧 MANUAL MODE ENABLED")
            self.get_logger().warn("Use /providesystem/goto_state to jump to state")
            self.get_logger().warn("Use /providesystem/next_step to execute one step")
            self.get_logger().warn("State machine is PAUSED")
            self.step_pending = False  # Start paused
            response.success = True
            response.message = "Manual mode enabled"
            return response
        else:
            self.get_logger().info("✅ MANUAL MODE DISABLED - Auto execution")
            self.step_pending = True  # Resume auto
            response.success = True
            response.message = "Manual mode disabled"
            return response
    
    def goto_state_topic_callback(self, msg):
        """Topic /providesystem/goto_state - Jump to specific state (String)
        
        Chỉ hoạt động khi ở MANUAL mode.
        Dùng set_operation_mode 'manual' trước, rồi mới dùng goto_state.
        """
        # Yêu cầu MANUAL mode
        if self.operation_mode != 'manual':
            self.get_logger().warn(
                f"❌ goto_state bị chặn — đang ở '{self.operation_mode}'. "
                "Chuyển sang MANUAL mode trước."
            )
            self._notify_gui('warn', f'❌ goto_state bị chặn', f'Mode hiện tại: {self.operation_mode}. Cần MANUAL.')
            return
        
        state_name = msg.data.strip()
        
        # Alias mapping for GUI shortcut buttons
        STATE_ALIASES = {
            'STATE1': 'S1_INY_CONFIRM_SAFE',
            'STATE2': 'S2_WAIT_TRIGGER',   # Đi qua WAIT_TRIGGER — tự kích change_tray_input bên dưới
            'STATE3': 'S3_WAIT_TRIGGER',
        }

        # ✅ Khi nhấn STATE 2 từ GUI: gọi thẳng change_tray_input_callback
        # → Cùng code path với topic /change_tray_input, không cần biến signal riêng
        if msg.data.strip() == 'STATE2':
            from std_msgs.msg import Bool as _Bool
            self.change_tray_input_callback(_Bool(data=True))

        state_name = STATE_ALIASES.get(msg.data.strip(), msg.data.strip())
        
        # Find matching state from enum
        target_state = None
        for state in SystemState:
            if state.value == state_name or state.name == state_name:
                target_state = state
                break
        
        if target_state is None:
            available = ", ".join([s.name for s in SystemState][:10]) + "..."
            self.get_logger().error(f"\u274c Invalid state '{state_name}'. Available: {available}")
            return

        # ✅ GUARD: Chỉ cho vào state machine khi đã home (zero_offset đã được set)
        _state_needs_home = (
            target_state.name.startswith("S1_") or
            target_state.name.startswith("S2_") or
            target_state.name.startswith("S3_") or
            target_state in (SystemState.STATE1_COMPLETE, SystemState.STATE2_COMPLETE,
                             SystemState.STATE3_COMPLETE, SystemState.HOMING)
        )
        if _state_needs_home and not self.zero_offset and target_state != SystemState.HOMING:
            self.get_logger().warn(
                f"\u26a0\ufe0f goto_state '{state_name}' bị chặn — chưa home! "
                "Nhấn START để home trước."
            )
            self._notify_gui('warn', '\u26a0\ufe0f Chưa HOME', f"Goto {state_name} cần home trước — nhấn START")
            return

        self.state = target_state
        self.step_pending = True  # Run continuously (no need for next_step)
        self.manual_auto_run = True  # Don't pause at each step
        self.get_logger().info(f"🎯 Goto: {target_state.name}")
        self._notify_gui('info', f"Goto: {target_state.name}")
        
        # ✅ Nếu goto vào STATE1: reset Position 2 (Servo 3) để chạy song song
        if target_state.name.startswith("S1_") or target_state == SystemState.S1_INY_CONFIRM_SAFE:
            if self._pos2_state in ("IDLE", "DONE", "WAIT_S7"):
                self._pos1_done = False
                self._pos2_done = False
                self._pos2_state = "TO_TARGET1"
                self._p2_move_cmd_sent = False
                self._p2_move_cmd_time = 0.0
                self._p2_s7_stable_start = None
                self.get_logger().info("🔄 [P2] Reset → TO_TARGET1 (Servo3 sẽ di chuyển đến 10mm)")
    
    def next_step_callback(self, req, response):
        """Service /providesystem/next_step - Execute one state transition"""
        if not self.manual_mode:
            response.success = False
            response.message = "Manual mode not enabled"
            return response
        
        old_state = self.state
        self.step_pending = True  # Allow one step
        
        # Wait for state to execute (up to 2s for homing etc.)
        for _ in range(20):
            time.sleep(0.1)
            if self.state != old_state:
                break
        
        new_state = self.state
        if new_state != old_state:
            msg = f"Step: {old_state.name} -> {new_state.name}"
            self.get_logger().info(msg)
            self._notify_gui('info', msg)
            response.success = True
            response.message = msg
        else:
            msg = f"State: {old_state.name} (waiting for sensor/interlock)"
            self.get_logger().warn(msg)
            self._notify_gui('warn', msg)
            response.success = True
            response.message = msg
        return response
    
    def set_target_row_topic_callback(self, msg):
        """Topic /providesystem/set_target_row - Set target row (String: '1'-'8')"""
        if not self.manual_mode:
            self.get_logger().warn("❌ set_target_row: Manual mode not enabled")
            return
        
        try:
            row = int(msg.data.strip())
            if row < 1 or row > 8:
                self.get_logger().error(f"❌ Invalid row {row}. Must be 1-8.")
                return
            
            self.manual_target_row = row
            self.get_logger().info(f"🎯 Manual target row set to: {row}")
        except ValueError:
            self.get_logger().error(f"❌ Invalid row '{msg.data}'. Must be 1-8.")
    
    def set_servo_limit_callback(self, req, response):
        """Service /providesystem/set_servo_limit - Set servo soft limit
        Call: rosservice call /providesystem/set_servo_limit "data: true"
        Sau đó publish: rostopic pub /providesystem/servo_limit_data std_msgs/String 'servo_id:position'
        
        Workaround: Dùng req.data không được (bool), nên dùng topic riêng.
        """
        # List current limits
        limits_str = ", ".join([f"S{k}={v:.0f}mm" for k, v in self.config.servo_limits.items()])
        self.get_logger().info(f"📋 Current servo limits: {limits_str}")
        self.get_logger().info("Set limit via: ros2 topic pub /providesystem/servo_limit_cmd std_msgs/String 'data: servo_id:max_pos'")
        self.get_logger().info("Example: ros2 topic pub -1 /providesystem/servo_limit_cmd std_msgs/String '{data: 3:350}'")
        response.success = True
        response.message = f"Limits: {limits_str}"
        return response
    
    def servo_limit_cmd_callback(self, msg):
        """Topic /providesystem/servo_limit_cmd - Set servo soft limit
        Format: 'servo_id:max_position' e.g. '3:350'
        """
        try:
            parts = msg.data.strip().split(':')
            if len(parts) != 2:
                self.get_logger().error(f"❌ Invalid format '{msg.data}'. Use 'servo_id:max_pos' e.g. '3:350'")
                return
            
            servo_id = int(parts[0])
            max_pos = float(parts[1])
            
            if servo_id not in self.config.servo_limits:
                self.get_logger().error(f"❌ Invalid servo ID {servo_id}. Valid: 1-5")
                return
            
            if max_pos <= 0:
                self.get_logger().error(f"❌ Invalid limit {max_pos}. Must be > 0")
                return
            
            old_limit = self.config.servo_limits[servo_id]
            self.config.servo_limits[servo_id] = max_pos
            self.get_logger().warn(f"✅ Servo {servo_id} limit: {old_limit:.0f}mm → {max_pos:.0f}mm")
            
            # Log all limits
            limits_str = ", ".join([f"S{k}={v:.0f}mm" for k, v in self.config.servo_limits.items()])
            self.get_logger().info(f"📋 All limits: {limits_str}")
            
        except ValueError as e:
            self.get_logger().error(f"❌ Parse error: {e}. Use 'servo_id:max_pos' e.g. '3:350'")
    
    def move_to_pos_callback(self, msg):
        """Topic /providesystem/move_to_pos - Move servo to absolute position
        Format: 'servo_id:position_mm'  e.g. '1:250.0'
        Only works in JOG mode.
        """
        if self.operation_mode not in ('jog', 'manual'):
            self.get_logger().warn(f"❌ move_to_pos: Chỉ dùng được ở JOG hoặc MANUAL mode (hiện tại: '{self.operation_mode}')")
            return
        
        try:
            parts = msg.data.strip().split(':')
            if len(parts) != 2:
                self.get_logger().error(f"❌ Invalid format '{msg.data}'. Use 'servo_id:position'")
                return
            
            servo_id = int(parts[0])
            position = float(parts[1])
            
            if servo_id not in self.servos:
                self.get_logger().error(f"❌ Servo {servo_id} not connected")
                return
            
            if position < 0:
                self.get_logger().error(f"❌ Position must be >= 0")
                return
            
            self.get_logger().info(f"🎯 Moving Servo {servo_id} to {position:.1f}mm...")
            success = self.move_servo(servo_id, position, wait=False)
            if success:
                self.get_logger().info(f"✅ Servo {servo_id} move command sent to {position:.1f}mm")
            else:
                self.get_logger().error(f"❌ Servo {servo_id} move failed")
                
        except ValueError as e:
            self.get_logger().error(f"❌ Parse error: {e}")
    
    def update_config_callback(self, msg):
        """[FIX-4] Topic /providesystem/update_config - validate range before applying."""
        try:
            data = json.loads(msg.data)
            key = data.get('key', '') or data.get('table', '')
            raw_data = data.get('data', '') or data.get('positions', {})
            if not key:
                # Flat JSON dict (from GUI): {"inx_target2": 500.0, ...}
                key  = next((k for k in data if hasattr(self.config, k)), None)
                if key:
                    raw_data = data[key]
                else:
                    self.get_logger().error(f"❌ No valid config key in: {list(data.keys())}")
                    return

            if not hasattr(self.config, key):
                self.get_logger().error(f"❌ Unknown config key: {key}")
                return

            current = getattr(self.config, key)

            # Row table (dict) — parse JSON positions
            if isinstance(current, dict) and key in ('iny_input_stack', 'iny_output_stack', 'outy_output_table'):
                positions = json.loads(raw_data) if isinstance(raw_data, str) else raw_data
                lo, hi = _ROW_POSITION_RANGE  # [FIX-4] row range validation
                updated = 0
                for row_str, pos in positions.items():
                    row = int(row_str)
                    v = float(pos)
                    if not (lo <= v <= hi):
                        self.get_logger().error(f"❌ {key} Row {row}: {v}mm out of [{lo}, {hi}]mm")
                        self._notify_gui('error', '❌ Config Out of Range', f'{key} Row {row}: {v}mm')
                        return
                    if row in current:
                        old_val = current[row]
                        current[row] = v
                        if old_val != v:
                            updated += 1
                self.get_logger().info(f"✅ Updated {key}: {updated} rows changed")

            # Scalar value
            else:
                try:
                    new_val = json.loads(raw_data) if isinstance(raw_data, str) else raw_data
                except (json.JSONDecodeError, ValueError):
                    new_val = raw_data
                if isinstance(current, float):
                    new_val = float(new_val)
                elif isinstance(current, int):
                    new_val = int(new_val)
                # [FIX-4] range validation for scalar
                if key in _CONFIG_RANGES:
                    lo, hi = _CONFIG_RANGES[key]
                    if not (lo <= float(new_val) <= hi):
                        self.get_logger().error(f"❌ {key} = {new_val} out of [{lo}, {hi}]")
                        self._notify_gui('error', '❌ Config Out of Range', f'{key} = {new_val} (limit [{lo}, {hi}])')
                        return
                setattr(self.config, key, new_val)
                self.get_logger().info(f"✅ Updated {key} = {new_val}")

            self.config.save_to_file()
            self._publish_config()

        except (json.JSONDecodeError, ValueError) as e:
            self.get_logger().error(f"❌ Config update error: {e}")

    def get_config_callback(self, msg):
        """Topic /providesystem/get_config - Request current config data
        Publishes full config to /providesystem/config_data
        """
        self._publish_config()
    
    def _publish_config(self):
        """Publish current config as JSON to /providesystem/config_data"""
        config_data = {
            'operation_mode': self.operation_mode,
            'manual_mode': self.manual_mode,
            'iny_input_stack': {str(k): v for k, v in self.config.iny_input_stack.items()},
            'iny_output_stack': {str(k): v for k, v in self.config.iny_output_stack.items()},
            'outy_output_table': {str(k): v for k, v in self.config.outy_output_table.items()},
            'servo_positions': {},
            # InX positions
            'inx_home': self.config.inx_home,
            'inx_target2': self.config.inx_target2,
            'inx_output_stack': self.config.inx_output_stack,
            # InY positions
            'iny_home': self.config.iny_home,
            'iny_target2': self.config.iny_target2,
            'iny_safe_zone': self.config.iny_safe_zone,
            # Servo 3
            'servo3_push_position': self.config.servo3_push_position,
            'servo3_target1': self.config.servo3_target1,
            # OutX positions
            'outx_home': self.config.outx_home,
            'outx_target2': self.config.outx_target2,
            'outx_target3': self.config.outx_target3,
            # OutY positions
            'outy_home': self.config.outy_home,
            'outy_target2': self.config.outy_target2,
            'outy_safe_zone': self.config.outy_safe_zone,
        }
        # Read current servo positions
        for sid in self.servos:
            pos = self.get_servo_position(sid)
            if pos is not None:
                config_data['servo_positions'][str(sid)] = round(pos, 2)
        
        msg = String()
        msg.data = json.dumps(config_data)
        self.pub_config_data.publish(msg)
    
    def get_position_safe(self, servo_id: int) -> tuple:
        """Get servo position with safety check.
        Returns (position, is_valid). If invalid 3 times → should ERROR.
        In manual mode: returns (0.0, True) on failure.
        """
        pos = self.get_servo_position(servo_id)
        if pos is None:
            if self.manual_mode:
                return (0.0, True)  # Simulate position 0 in manual mode
            count = self._position_read_fail_count.get(servo_id, 0) + 1
            self._position_read_fail_count[servo_id] = count
            if count >= 3:
                self.get_logger().error(f"Servo {servo_id}: Position read FAILED {count}x! STOPPING.")
                self.stop_servo(servo_id)
                return (None, False)
            else:
                self.get_logger().warn(f"Servo {servo_id}: Position read failed ({count}/3)")
                return (None, True)  # Not fatal yet
        else:
            self._position_read_fail_count[servo_id] = 0  # Reset on success
            return (pos, True)
    
    def count_trays_in_stack(self, start_row: int) -> int:
        """Count total number of trays in the stack using sensor 5"""
        # After detecting first tray, continue jogging to count more trays
        count = 1  # We already detected the first one
        
        # Continue moving up to check for more trays
        # Simple implementation: check each row position above start_row
        for row in range(start_row + 1, self.config.max_trays + 1):
            target_pos = self.config.iny_input_stack.get(row)
            if target_pos is None:
                break
            
            # Move to next row position
            if self.move_servo(2, target_pos, wait=True):
                # Check if sensor 5 detects a tray
                if self.sensor_manager.get_sensor(5):
                    count += 1
                    self.get_logger().info(f"Tray detected at row {row}")
                else:
                    # No more trays above this position
                    break
            else:
                break
        
        return count
    
    def extend_cylinder1(self):
        """Extend cylinder 1"""
        if self.io_module:
            try:
                self.io_module.reset_channel(self.config.cylinder1_retract_channel)
                self.io_module.set_channel(self.config.cylinder1_extend_channel)
            except Exception as e:
                self.get_logger().warn(f"⚠️ Cylinder 1 extend IO error (ignored): {e}")
        self.get_logger().info("Cylinder 1 extending")
    
    def retract_cylinder1(self):
        """Retract cylinder 1"""
        if self.io_module:
            try:
                self.io_module.reset_channel(self.config.cylinder1_extend_channel)
                self.io_module.set_channel(self.config.cylinder1_retract_channel)
            except Exception as e:
                self.get_logger().warn(f"⚠️ Cylinder 1 retract IO error (ignored): {e}")
        self.get_logger().info("Cylinder 1 retracting")
    
    def extend_cylinder2(self):
        """Extend cylinder 2"""
        if self.io_module:
            try:
                self.io_module.reset_channel(self.config.cylinder2_retract_channel)
                self.io_module.set_channel(self.config.cylinder2_extend_channel)
            except Exception as e:
                self.get_logger().warn(f"⚠️ Cylinder 2 extend IO error (ignored): {e}")
        self.get_logger().info("Cylinder 2 extending")
    
    def retract_cylinder2(self):
        """Retract cylinder 2"""
        if self.io_module:
            try:
                self.io_module.reset_channel(self.config.cylinder2_extend_channel)
                self.io_module.set_channel(self.config.cylinder2_retract_channel)
            except Exception as e:
                self.get_logger().warn(f"⚠️ Cylinder 2 retract IO error (ignored): {e}")
        self.get_logger().info("Cylinder 2 retracting")
    
    def extend_cylinder3(self):
        """Extend cylinder 3 (Hold Tray) — kẹp giữ khay"""
        if self.io_module:
            try:
                self.io_module.reset_channel(self.config.cylinder3_retract_channel)
                self.io_module.set_channel(self.config.cylinder3_extend_channel)
            except Exception as e:
                self.get_logger().warn(f"⚠️ Cylinder 3 extend IO error (ignored): {e}")
        self.get_logger().info("🔒 Cylinder 3 (Hold Tray) EXTEND — đang kẹp khay")

    def retract_cylinder3(self):
        """Retract cylinder 3 (Hold Tray) — nhả khay"""
        if self.io_module:
            try:
                self.io_module.reset_channel(self.config.cylinder3_extend_channel)
                self.io_module.set_channel(self.config.cylinder3_retract_channel)
            except Exception as e:
                self.get_logger().warn(f"⚠️ Cylinder 3 retract IO error (ignored): {e}")
        self.get_logger().info("🔓 Cylinder 3 (Hold Tray) RETRACT — đã nhả khay")
    
    def publish_state(self):
        """Publish current state"""
        msg = String()
        msg.data = self.state.value
        self.pub_state.publish(msg)
    
    def control_loop_callback(self):
        """Timer callback for main control loop (50 Hz)"""
        # [FIX-8] Watchdog heartbeat — cập nhật mỗi tick để watchdog_tick biết loop còn sống
        self._watchdog_last_tick = time.time()

        # ── PAUSE gate: khi bị pause thì chỉ update sensor, không chạy state ──
        self.prev_sensor_state = self.sensor_manager.sensors.copy()
        self.update_sensors()
        if self._system_paused:
            return  # Dừng toàn bộ logic, giữ nguyên state hiện tại

        # ── Publish /cartridge/busy: True khi S2/S3 đang chạy ───────────────
        _sname = self.state.name
        _is_busy = _sname.startswith("S2_") or _sname.startswith("S3_")
        if _is_busy != self._last_cartridge_busy:
            self._last_cartridge_busy = _is_busy
            self.pub_cartridge_busy.publish(Bool(data=_is_busy))
            self.get_logger().info(
                f"{'🔒 CARTRIDGE BUSY → Robot BLOCKED' if _is_busy else '🔓 CARTRIDGE FREE → Robot OK'}"
                f"  (state={_sname})"
            )

        # ── Global State 3 trigger: khi đang chờ ở S2_WAIT_TRIGGER ────────
        if self.state == SystemState.S2_WAIT_TRIGGER:
            if self.change_tray_output_signal or self.last_batch_complete:
                if not self.robot_motion_busy:
                    reason = 'last_batch_complete' if self.last_batch_complete else 'output_tray_full'
                    self.get_logger().info(f"[S3] Trigger: {reason} + Robot IDLE → Starting State 3")
                    self.change_tray_output_signal = False
                    self.output_tray_full = False
                    self.pub_output_ready(False)  # Interlock robot
                    self.state = SystemState.S3_CHECK_SAFE
                else:
                    self.log_guide_once("S3_GLOBAL_WAIT", "[S3] Trigger nhận nhưng Robot BUSY — chờ IDLE...")

        # ✅ GLOBAL SAFETY: InY ngoài safe zone → dừng InX (fire 1 lần)
        # SKIP during HOMING: InY intentionally moves to home, and homing thread
        # uses the same servo 2 Modbus TCP socket → concurrent access = deadlock!
        if self.state not in (SystemState.HOMING, SystemState.HOMING_RUNNING):
            iny_pos = self.get_servo_position(2)
            if iny_pos is not None and iny_pos > self.config.iny_safe_zone:
                if not getattr(self, '_interlock_fired', False):
                    self._interlock_fired = True
                    self.stop_servo(1)
                    self.get_logger().error(
                        f"🚨 INTERLOCK: InY = {iny_pos:.1f}mm > {self.config.iny_safe_zone}mm! InX STOPPED."
                    )
                    self._notify_gui('error', '🚨 INTERLOCK: InX dừng!', f'InY={iny_pos:.1f}mm')
            else:
                self._interlock_fired = False
        
        # State machine
        self.process_state()
        
        # ✅ PARALLEL: Process Position 2 (Servo 3) độc lập
        self.process_pos2_state()
        
        # Publish state
        self.publish_state()
    
    def update_sensors(self):
        """Update sensor readings. [FIX-3] Guard: skip hardware cache until IO module is ready."""
        # Simulated sensors always applied first
        for sensor_id, state in self._sim_sensors.items():
            self.sensor_manager.update_sensor(sensor_id, state)

        # [FIX-3] Only use real cache after first successful IO read
        if not self._io_ready:
            return

        with self._io_bg_lock:
            cached = list(self._io_sensor_cache)

        for sensor_id in range(1, min(16, len(cached) + 1)):
            if sensor_id not in self._sim_sensors:
                self.sensor_manager.update_sensor(sensor_id, cached[sensor_id - 1])

    def _io_bg_reader_loop(self):
        """Background thread: reads IO module channels into cache.
        NEVER called from control loop — runs independently.
        If read blocks, only THIS thread is stuck (control loop keeps running).
        """
        import time as _time
        _reconnect_wait = 0.0
        while rclpy.ok():
            # ── Reconnect if needed ────────────────────────────────────────────
            if self.io_module is None:
                if _time.time() >= _reconnect_wait:
                    try:
                        mod = CpxAp(ip_address=self.config.io_ip)
                        self.io_module = mod
                        self._io_read_fail_count = 0
                        self.get_logger().info("✅ [IO-bg] Module reconnected")
                    except Exception as e:
                        self.get_logger().warn(f"[IO-bg] Reconnect failed: {e}")
                        _reconnect_wait = _time.time() + 10.0
                _time.sleep(0.5)
                continue

            # ── Check io_thread alive ─────────────────────────────────────────
            _iot = getattr(self.io_module, 'io_thread', None)
            if _iot is not None and not _iot.is_alive():
                self.get_logger().warn("[IO-bg] io_thread dead → marking module dead, reconnecting")
                self.io_module = None
                _reconnect_wait = _time.time() + 2.0
                _time.sleep(0.5)
                continue

            # ── Read channels ─────────────────────────────────────────────────
            try:
                all_channels = []
                for mod in self.io_module.modules:
                    if mod.is_function_supported("read_channels"):
                        ch = mod.read_channels()
                        if isinstance(ch, list):
                            all_channels.extend(ch)
                with self._io_bg_lock:
                    self._io_sensor_cache = all_channels
                self._io_read_fail_count = 0
                if not self._io_ready:  # [FIX-3] Set ready flag on first successful read
                    self._io_ready = True
                    self.get_logger().info("✅ [IO-bg] IO ready — sensor data đang chạy")
            except Exception as e:
                self._io_read_fail_count += 1
                if self._io_read_fail_count >= 3:
                    self.get_logger().warn(f"[IO-bg] Read failed {self._io_read_fail_count}x → reconnect")
                    try: self.io_module.close()
                    except Exception: pass
                    self.io_module = None
                    self._io_ready = False  # [FIX-3] Reset ready flag on connection loss
                    _reconnect_wait = _time.time() + 5.0

            _time.sleep(0.05)  # 20Hz read rate


    
    def sim_sensor_callback(self, msg: String):
        """Handle /providesystem/sim_sensor — simulate sensor signals for testing.
        
        NOTE: Only works in MANUAL/JOG/IDLE mode. AUTO mode uses real sensors only.
        
        Format:
            '7:1'     -> S7 = ON
            '7:0'     -> S7 = OFF
            'all:1'   -> All sensors ON
            'all:0'   -> All sensors OFF
            'clear'   -> Remove all overrides (use real IO)
            'clear:7' -> Remove override for S7 only
            'status'  -> Log current sensor states
        """
        # ===== GUARD: No simulation in AUTO mode =====
        if self.operation_mode == 'auto':
            self.get_logger().warn("sim_sensor: Khong cho phep trong AUTO mode.")
            self._notify_gui('warn', 'SIM blocked: AUTO mode uses real sensors')
            return
        
        cmd = msg.data.strip()
        
        if cmd == 'clear':
            self._sim_sensors.clear()
            self.get_logger().info("SIM: All sensor overrides cleared")
            self._notify_gui('info', 'SIM: All overrides cleared')
            return
        
        if cmd == 'status':
            states = {sid: self.sensor_manager.get_sensor(sid) for sid in range(1, 14)}
            sim_ids = list(self._sim_sensors.keys())
            self.get_logger().info(f"📊 Sensors: {states}")
            self.get_logger().info(f"📊 Simulated: {sim_ids}")
            return
        
        if cmd.startswith('clear:'):
            try:
                sid = int(cmd.split(':')[1])
                self._sim_sensors.pop(sid, None)
                self.get_logger().info(f"🔄 SIM: S{sid} override removed")
            except ValueError:
                self.get_logger().warn(f"Invalid sim_sensor command: {cmd}")
            return
        
        try:
            parts = cmd.split(':')
            sensor_str, state_str = parts[0], parts[1]
            state = state_str == '1'
            
            if sensor_str == 'all':
                for sid in range(1, 14):
                    self._sim_sensors[sid] = state
                    self.sensor_manager.update_sensor(sid, state)
                self.get_logger().info(f"SIM: ALL sensors -> {'ON' if state else 'OFF'}")
                self._notify_gui('info', f"SIM: ALL -> {'ON' if state else 'OFF'}")
            else:
                sid = int(sensor_str)
                self._sim_sensors[sid] = state
                self.sensor_manager.update_sensor(sid, state)
                self.get_logger().info(f"SIM: S{sid} -> {'ON' if state else 'OFF'}")
                self._notify_gui('info', f"SIM: S{sid} -> {'ON' if state else 'OFF'}")
        except (ValueError, IndexError):
            self.get_logger().warn(f"Invalid sim_sensor format: '{cmd}'. Use 'ID:0/1', 'all:0/1', 'clear', or 'status'")
    
    def log_guide_once(self, key: str, message: str):
        """Log a guide message only once per state entry (prevent spam at 20Hz)"""
        if key not in self._guide_logged:
            self._guide_logged.add(key)
            self.get_logger().info(f"📋 GUIDE: {message}")
    
    def clear_guide(self):
        """Clear guide log tracking (call on state transition)"""
        self._guide_logged.clear()
    
    def process_pos2_state(self):
        """Process Position 2 (Servo 3 / PutTray) - chạy SONG SONG với Position 1.
        
        Flow:
          IDLE → TO_TARGET1 → WAIT_S7 → S7_DEBOUNCE → JOG_TO_S6 → DONE
                                                            ↓ (limit reached, S6 OFF)
                                                     RETURN_TO_T1 → WAIT_GUI_CONFIRM → WAIT_S7 (retry)
        """
        # ══ GLOBAL CONFIRM INTERRUPT: Kích hoạt Servo3 re-load từ bất kỳ state nào ══
        if self._confirm_load_received or self._hmi_resume_confirmed:
            # Không interrupt khi đang di chuyển TO_TARGET1 / RETURN_TO_T1 (chờ xong)
            _safe_to_interrupt = self._pos2_state not in ("TO_TARGET1", "RETURN_TO_T1")
            if _safe_to_interrupt:
                self._confirm_load_received = False
                self._hmi_resume_confirmed  = False
                self.stop_servo3_jog()          # Dừng jog nếu đang chạy
                self._servo3_jog_active = False
                self._p2_move_cmd_sent   = False
                self._p2_move_cmd_time   = 0.0
                self._p2_s7_stable_start = None
                self._p2_settle_start    = time.time()
                self._pos2_done = False         # Cho phép set lại khi thành công
                self._guide_logged.discard("P2_WAIT_CONFIRM")
                self._guide_logged.discard("P2_SETTLE")
                self._guide_logged.discard("P2_WAIT_S7")
                self._guide_logged.discard("P2_JOG_S6")
                self.get_logger().info(
                    f"✅ [P2] CONFIRM nhận (state={self._pos2_state}) "
                    "→ Servo3 reset, bắt đầu cấp khay mới"
                )
                self._pos2_state = "SETTLED"
                return  # Xử lý từ vòng tiếp theo

        if self._pos2_state in ("IDLE", "DONE"):
            return

        P2_SAFE   = self.config.servo3_target1        # 10mm
        P2_LIMIT  = self.config.servo_limits.get(3, 400.0)   # 400mm
        S7_STABLE = 3.0   # giây S7 phải ON liên tục trước khi jog

        # ── TO_TARGET1: Servo 3 về vị trí safe sau khi home (NON-BLOCKING) ─────────────
        if self._pos2_state == "TO_TARGET1":
            if not self._p2_move_cmd_sent:
                ok = self.move_servo(3, P2_SAFE, wait=False)
                self._p2_move_cmd_sent = True
                self._p2_move_cmd_time = time.time()
                self._p2_last_log_time = time.time()
                if not ok:
                    self.get_logger().error(f"❌ [P2] Servo3 → Target1 ({P2_SAFE}mm) REJECTED by drive! Check drive state.")
                else:
                    self.get_logger().info(f"▶️  [P2] Servo3 → Target1 ({P2_SAFE}mm) cmd sent")
                return

            elapsed = time.time() - self._p2_move_cmd_time

            # Timeout 30s — nếu servo vẫn không tới Target1
            if elapsed > 30.0:
                s3_pos = self.get_servo_position(3)
                self.get_logger().error(
                    f"❌ [P2] Servo3 TO_TARGET1 TIMEOUT (30s)! "
                    f"pos={s3_pos:.1f}mm (target={P2_SAFE}mm). Kiểm tra drive/kết nối."
                )
                self._notify_gui('error', '❌ [P2] Servo3 timeout', f'Không đến {P2_SAFE}mm sau 30s, pos={s3_pos:.1f}mm')
                self._p2_move_cmd_sent = False
                self._p2_move_cmd_time = 0.0
                self.state = SystemState.ERROR
                return

            # Đợi tối thiểu 0.5s để drive xử lý lệnh trước khi kiểm tra vị trí
            if elapsed < 0.5:
                return

            s3_pos = self.get_servo_position(3)

            # Log tiến trình mỗi 2s
            if not hasattr(self, '_p2_last_log_time'):
                self._p2_last_log_time = time.time()
            if time.time() - self._p2_last_log_time >= 2.0:
                self._p2_last_log_time = time.time()
                mot = self.servos.get(3)
                tpr = mot.target_position_reached() if mot else False
                self.get_logger().info(
                    f"⏳ [P2] Servo3 moving → {P2_SAFE}mm | pos={s3_pos:.1f}mm | target_reached={tpr} | elapsed={elapsed:.1f}s"
                )

            # Check arrived: dùng cả target_position_reached() và tolerance 3mm
            mot3 = self.servos.get(3)
            arrived_tpr = mot3.target_position_reached() if mot3 else False
            arrived_pos = (s3_pos is not None and abs(s3_pos - P2_SAFE) <= 3.0)

            if arrived_tpr or arrived_pos:
                self._p2_move_cmd_sent = False
                self._p2_move_cmd_time = 0.0
                self._p2_s7_stable_start = None
                self._p2_settle_start = time.time()  # Bắt đầu đếm 3s settle
                self.get_logger().info(f"✅ [P2] Servo3 @ Target1 ({s3_pos:.1f}mm) — dừng 3s rồi mới check S7")
                self._pos2_state = "SETTLED"
            elif s3_pos is None and 3 not in self.servos:
                # không có phần cứng — pass through ngạy
                self._p2_move_cmd_sent = False
                self._p2_settle_start = time.time()
                self._pos2_state = "SETTLED"

        # ── SETTLED: Đợi 3s sau khi đến 50mm, rồi mới check S7 ──────────────────
        elif self._pos2_state == "SETTLED":
            waited = time.time() - getattr(self, '_p2_settle_start', time.time())
            if waited < 3.0:
                remaining = 3.0 - waited
                self.log_guide_once("P2_SETTLE", f"[P2] Servo3 ở 50mm — chờ {3.0:.0f}s rồi check S7 ({remaining:.1f}s còn lại)")
                return
            self.get_logger().info("⏱️ [P2] Settle 3s xong — bắt đầu check S7")
            self._guide_logged.discard("P2_SETTLE")  # Cho phép log lại nếu cần
            self._pos2_state = "WAIT_S7"

        # ── WAIT_S7: Chờ S7 ON (khay có sẵn) ────────────────────────────────
        elif self._pos2_state == "WAIT_S7":

            if self.sensor_manager.get_sensor(12):  # S12 ON (CYL3 retracted — đã nhả khay)
                if self._p2_s7_stable_start is None:
                    self._p2_s7_stable_start = time.time()
                    self.get_logger().info("⏱️  [P2] S7 ON — đếm 3s ổn định trước khi jog...")
                elif time.time() - self._p2_s7_stable_start >= S7_STABLE:
                    self.get_logger().info(f"✅ [P2] S7 ON {S7_STABLE}s — bắt đầu jog Servo3+")
                    self._p2_s7_stable_start = None
                    self._pos2_state = "JOG_TO_S6"
            else:
                # S7 OFF → reset debounce timer nếu tín hiệu bị mất giữa chừng
                if self._p2_s7_stable_start is not None:
                    self.get_logger().warn("⚠️  [P2] S7 OFF giữa chừng — reset debounce")
                    self._p2_s7_stable_start = None
                self.log_guide_once("P2_WAIT_S7", f"[P2] Chờ S7 ON (khay output có sẵn). Còn {S7_STABLE:.0f}s debounce sau khi ON")

        # ── JOG_TO_S6: Jog Servo3 dương cho đến khi S6 ON ───────────────────
        elif self._pos2_state == "JOG_TO_S6":
            if self.sensor_manager.get_sensor(6):  # S6 ON → đến nơi
                self.stop_servo3_jog()
                self.get_logger().info("✅ [P2] S6 ON — Servo3 đã đẩy khay đến vị trí robot!")
                self.pub_output_ready(True)
                self._servo3_init_done = True
                self._pos2_done = True
                self._pos2_state = "DONE"
                if self._pos1_done:
                    self.get_logger().info("🎉 Cả 2 Position xong → STATE1_COMPLETE")
                    self.state = SystemState.STATE1_COMPLETE
                else:
                    self.get_logger().info("⏳ [P2] Done — chờ Position 1 (InX/InY)...")
                return

            # Đọc vị trí để check giới hạn
            s3_pos, pos_ok = self.get_position_safe(3)
            if s3_pos is None:
                if not pos_ok:
                    self.stop_servo3_jog()
                    self._pos2_state = "IDLE"
                    self.state = SystemState.ERROR
                return

            # 🚨 Giới hạn 400mm — S6 không ON → quay về T1
            if s3_pos >= P2_LIMIT:
                self.stop_servo3_jog()
                self.get_logger().warn(f"⚠️  [P2] Servo3={s3_pos:.1f}mm — S6 không ON tại {P2_LIMIT:.0f}mm! Về Target1")
                self._notify_gui('warn', '⚠️ [Pos2] S6 không phát hiện', f'Servo3 đạt {P2_LIMIT:.0f}mm mà S6 chưa ON — về safe')
                self._pos2_state = "RETURN_TO_T1"
                return

            # Tiếp tục jog
            self.start_servo3_jog(-self.config.servo3_jog_velocity)  # âm = jog_positive (đẩy về phía S6)
            self.log_guide_once("P2_JOG_S6", f"[P2] Servo3 đang jog → Chờ S6 ON (giới hạn {P2_LIMIT:.0f}mm)")

        # ── RETURN_TO_T1: Về 50mm sau khi S6 thất bại (NON-BLOCKING) ────────────
        elif self._pos2_state == "RETURN_TO_T1":
            if not self._p2_move_cmd_sent:
                self.move_servo(3, P2_SAFE, wait=False)
                self._p2_move_cmd_sent = True
                self._p2_move_cmd_time = time.time()
                self.get_logger().info(f"▶️  [P2] Servo3 về {P2_SAFE}mm — INX/INY tiếp tục bình thường")
                return
            if time.time() - self._p2_move_cmd_time < 0.5:
                return
            s3_pos = self.get_servo_position(3)
            arrived = (s3_pos is not None and abs(s3_pos - P2_SAFE) <= 3.0)
            no_hw   = (s3_pos is None and 3 not in self.servos)
            if arrived or no_hw:
                self._p2_move_cmd_sent = False
                self._p2_move_cmd_time = 0.0
                # ✅ INX/INY tiếp tục: đánh dấu pos2 done để STATE1_COMPLETE không bị block
                if not self._pos2_done:
                    self._pos2_done = True
                    if self._pos1_done:
                        self.get_logger().info("🎉 [P2] Pos2 release — cả 2 Position xong → STATE1_COMPLETE")
                        self.state = SystemState.STATE1_COMPLETE
                self.get_logger().info(
                    f"✅ [P2] Servo3 về {P2_SAFE}mm — INX/INY chạy bình thường. "
                    "Nhấn CONFIRM để cấp khạy lại."
                )
                self._notify_gui('warn', '⚠️ [Pos2] Chưa tìm thấy khạy', f'Servo3 về {P2_SAFE}mm — nhấn CONFIRM để cấp khạy mới')
                self._pos2_state = "WAIT_CONFIRM"

        # ── WAIT_CONFIRM: Chờ CONFIRM từ GUI để cấp khạy lại — Độc lập với INX/INY ───
        elif self._pos2_state == "WAIT_CONFIRM":
            if self._confirm_load_received or self._hmi_resume_confirmed:
                # Reset flags
                self._confirm_load_received = False
                self._hmi_resume_confirmed  = False
                self._p2_s7_stable_start    = None
                self._p2_settle_start       = time.time()  # 3s settle trước check S7
                self.get_logger().info("✅ [P2] CONFIRM nhận — Servo3 được phép check S7 và jog lại")
                self._guide_logged.discard("P2_WAIT_CONFIRM")
                self._guide_logged.discard("P2_SETTLE")
                # Reset _pos2_done để JOG_TO_S6 có thể set lại khi thành công
                self._pos2_done = False
                self._pos2_state = "SETTLED"
            else:
                self.log_guide_once("P2_WAIT_CONFIRM", "[P2] Servo3 chờ CONFIRM — nhấn nút CONFIRM trong System Control để cấp khạy lại")

    
    # ══════════════════════════════════════════════════════════════════════
    # process_state() continuation — Main State Machine
    # ══════════════════════════════════════════════════════════════════════
    
    def process_state(self):
        """Process current state - Main state machine (Position 1 + States 2/3)"""
        
        # ✅ MANUAL MODE: Pause execution cho đến khi next_step được gọi
        # ⚡ BYPASS: HOMING/HOMING_RUNNING luôn chạy — không cần step (tự động bắt buộc)
        _is_homing = self.state in (SystemState.HOMING, SystemState.HOMING_RUNNING)
        if self.manual_mode and not self.step_pending and not _is_homing:
            return  # Paused, chờ next_step service call
        
        # Track state before execution (for manual mode)
        state_before = self.state
        
        # ══════════════════════════════════════════════════════════════
        # IDLE - Chờ nút Start
        # ══════════════════════════════════════════════════════════════
        if self.state == SystemState.IDLE:
            pass  # Chờ start_button_callback
        
        # ══════════════════════════════════════════════════════════════
        # HOMING - Home tất cả servo
        # ══════════════════════════════════════════════════════════════
        elif self.state == SystemState.HOMING:
            # Chạy homing trong background thread để không block state machine loop
            self.state = SystemState.HOMING_RUNNING  # Dùng state tạm để tránh gọi lại
            
            def _do_home():
                try:
                    ok = self.home_all_servos()
                except Exception as ex:
                    import traceback; traceback.print_exc()
                    self.get_logger().error(f"❌ _do_home exception: {ex}")
                    self.state = SystemState.ERROR
                    return
                if ok:
                    if self.operation_mode == 'manual':
                        self.get_logger().info("✅ MANUAL Homing xong — Sẵn sàng.")
                        self._notify_gui('info', '✅ MANUAL Homed', 'Sẵn sàng')
                        self.state = SystemState.IDLE
                    else:
                        self.get_logger().info("✅ Homing complete — Bắt đầu chu trình tự động")
                        self._pos1_done = False
                        self._pos2_done = False
                        self._pos2_state = "TO_TARGET1"
                        self.state = SystemState.S1_INY_CONFIRM_SAFE
                else:
                    self.get_logger().error("❌ Homing FAILED")
                    self._notify_gui('error', '❌ Homing FAILED', 'Kiểm tra kết nối servo')
                    self.state = SystemState.ERROR

            threading.Thread(target=_do_home, daemon=True).start()

        
        # ══════════════════════════════════════════════════════════════
        # STATE 1: CẤP KHAY INPUT VÀO VỊ TRÍ ROBOT (Position 1: InX, InY)
        # ══════════════════════════════════════════════════════════════
        
        # ── MOTION DELAY: non-blocking 2s wait between motions ──────────────
        elif self.state == SystemState.MOTION_DELAY:
            self.check_motion_delay()
        
        elif self.state == SystemState.S1_INY_CONFIRM_SAFE:
            # Xác nhận InY đã homing xong và ở vị trí safe
            iny_pos = self.get_servo_position(2)
            if iny_pos is not None and iny_pos <= self.config.iny_safe_zone:
                self.get_logger().info(f"✅ InY safe at {iny_pos:.1f}mm (< {self.config.iny_safe_zone}mm) - Ready for InX")
                self.clear_guide()
                self.start_motion_delay(SystemState.S1_INX_TO_CONVEYOR_END)
            else:
                self.get_logger().warn(f"⏳ InY at {iny_pos}mm - Moving to safe zone...")
                self.move_servo(2, self.config.iny_home, wait=False)
        
        elif self.state == SystemState.S1_INX_TO_CONVEYOR_END:
            # InX di chuyển đến cuối băng tải — S1 ON = có khay ở đầu băng tải
            if not self.sensor_manager.get_sensor(1):  # S1 OFF = chưa có khay
                self.log_guide_once("S1_WAIT_S1", "[S1] Chờ S1 ON (sensor đầu băng tải — có khay). Kích: '1:1'")
                return
            if self.is_iny_safe_for_inx_move():
                if 1 in self.servos:
                    try:
                        mot = self.servos[1]
                        offset = self.zero_offset.get(1, 0)
                        pos_counts = offset + int(self.config.inx_target2 * COUNTS_PER_MM)
                        self._ensure_drive_ready(mot, 1)
                        mot.position_task(pos_counts, DEFAULT_VELOCITY, absolute=True, nonblocking=True)
                    except Exception as e:
                        self.get_logger().error(f"❌ InX non-blocking move error: {e}")
                self.get_logger().info(f"▶️ S1 ON — InX moving to {self.config.inx_target2:.0f}mm (non-blocking) — chờ S3 ON")
                self._inx_s3_wait_start = None  # reset 30s timer
                self._inx_arrived = False
                self._s3_seen_during_move = False
                self.clear_guide()
                self.state = SystemState.S1_INX_WAIT_STOP
                self.log_guide_once("S1_INX_MOVING", "[S1] InX đang di chuyển — chờ S3 ON + InX đến nơi")
            else:
                self.get_logger().warn("⏳ Waiting InY safe for InX move...")
        
        elif self.state == SystemState.S1_INX_WAIT_STOP:
            # ═══ CRITICAL: INX phải dừng hoàn toàn trước khi INY được phép di chuyển ═══
            
            # Bước 1: Ghi nhớ nếu S3 ON trong khi INX chưa dừng (BUFFER, chưa hành động)
            if self.sensor_manager.get_sensor(3) and not self._s3_seen_during_move:
                self._s3_seen_during_move = True
                self.get_logger().info("[ⓢⓣⓧ] S3 ON được ghi nhận (INX chưa dừng — sẽ xử lý sau khi INX dừng)")
            
            # Bước 2: Kiểm tra INX đã dừng hoàn toàn chưa (chỉ qua hardware check)
            if not self._inx_arrived:  # chỉ check nếu chưa dừng
                if 1 in self.servos:
                    try:
                        if self.servos[1].target_position_reached():
                            self._inx_arrived = True
                            self.get_logger().info("✅ INX dừng hoàn toàn (target_position_reached)")
                    except Exception:
                        pass  # Lỗi đọc — chưa xác nhận dừng, chờ tiếp
                # Nếu không có servo hardware — KHÔNG tự động cho quả là đã dừng
            
            # Bước 3: CHỈ khi INX đã dừng mới xét S3 và cho phép INY
            if self._inx_arrived:
                s3_ready = self.sensor_manager.get_sensor(3) or self._s3_seen_during_move
                if s3_ready:
                    # ✅ INTERLOCK S10: Cylinder 1 phải retract (S10 ON) trước khi INY được phép di chuyển
                    if not self.sensor_manager.get_sensor(10):
                        self.log_guide_once(
                            "S1_WAIT_S10",
                            "[S1] S3 ON nhưng S10 chưa ON (Cylinder 1 chưa retract) — Chờ S10 ON mới cho INY di chuyển"
                        )
                        return  # Hold: chờ cylinder retract xong
                    self._inx_s3_wait_start = None
                    self._s3_seen_during_move = False
                    self.get_logger().info("✅ INX dừng + S3 ON + S10 ON (Cyl retract) → INY được phép jog")
                    self.state = SystemState.S1_INY_SEARCH_TRAY
                else:
                    if self._inx_s3_wait_start is None:
                        self._inx_s3_wait_start = time.time()
                        self.get_logger().warn("⏳ INX dừng, S3 chưa ON — đợi tối đa 30s...")
                        self.log_guide_once("S1_WAIT_S3_30S", "[S1] INX đến target, chờ S3 ON (tối đa 30s). Kích: '3:1'")
                    elif time.time() - self._inx_s3_wait_start >= 30.0:
                        self.get_logger().warn("⚠️ S3 không ON sau 30s — INX về Home, thử lại State1")
                        self._notify_gui('warn', '⚠️ Không phát hiện khay', 'S3 không ON sau 30s — đang về Home')
                        self._inx_s3_wait_start = None
                        self._inx_arrived = False
                        self._s3_seen_during_move = False
                        # ✅ Reset Position 2 để Servo 3 chạy lại song song
                        if self._pos2_state in ("WAIT_S7", "DONE", "IDLE"):
                            self._pos2_state = "TO_TARGET1"
                            self._p2_move_cmd_sent = False
                            self._p2_move_cmd_time = 0.0
                            self._p2_s7_stable_start = None
                            self._pos2_done = False
                            self.get_logger().info("🔄 [P2] Reset → TO_TARGET1 (retry State1)")
                        if 1 in self.servos:
                            try:
                                mot = self.servos[1]
                                offset = self.zero_offset.get(1, 0)
                                pos_counts = offset + int(self.config.inx_home * COUNTS_PER_MM)
                                self._ensure_drive_ready(mot, 1)
                                mot.position_task(pos_counts, DEFAULT_VELOCITY, absolute=True, nonblocking=True)
                            except Exception as e:
                                self.get_logger().error(f"\u274c INX return home error: {e}")
                        self.state = SystemState.S1_INY_CONFIRM_SAFE
            else:
                self.log_guide_once("S1_INX_MOVING", "⏳ INX đang di chuyển — chờ dừng để cho phép INY...")
        
        elif self.state == SystemState.S1_INY_SEARCH_TRAY:
            # ✅ MANUAL MODE: Nếu đã set target row, bỏ qua sensor search
            if self.manual_mode and self.manual_target_row is not None:
                self.current_row = self.manual_target_row
                self.stack_row_index = self.manual_target_row
                self.get_logger().info(f"🎯 MANUAL: Row set to {self.manual_target_row} (skipping search)")
                self.manual_target_row = None
                self._s4_search_active = False
                self.state = SystemState.S1_INY_TO_NEAREST_ROW
            
            # ✅ Bước 1: Chờ S3 ON (khay đã tới cuối băng tải)
            elif not self.sensor_manager.get_sensor(3):
                self._s4_search_active = False  # Reset gate: chưa cho phép xét S4
                self.log_guide_once("S1_WAIT_S3", "[S1] Chờ S3 ON (sensor cuối băng tải — khay đã tới). Kích: '3:1'")
                return
            
            # ✅ Bước 2: S3 ON và S10 ON → INY bắt đầu jog
            else:
                # INTERLOCK S10: Cylinder 1 phải retract (S10 ON) — check lại trong trường hợp sensor thay đổi
                if not self.sensor_manager.get_sensor(10):
                    self.stop_iny_jog()  # Dừng INY nếu đang jog
                    self.log_guide_once(
                        "S1_S10_LOST",
                        "[S1] S10 mất tín hiệu (Cylinder 1 chưa retract) — INY dừng, chờ S10 ON"
                    )
                    return  # Hold
                iny_pos, pos_ok = self.get_position_safe(2)
                if iny_pos is None:
                    if not pos_ok:
                        self.state = SystemState.ERROR
                    return
                
                # Layer 2: Dừng khẩn cấp nếu INY vượt 960mm (giới hạn vật lý trục)
                INY_AXIS_MAX = 960.0
                if iny_pos >= INY_AXIS_MAX:
                    self.stop_iny_jog()
                    self.get_logger().error(f"🚨 EMERGENCY: INY={iny_pos:.1f}mm >= 960mm (giới hạn trục)! Dừng khẩn cấp")
                    self._notify_gui('error', '🚨 EMERGENCY STOP INY', f'INY={iny_pos:.1f}mm vượt giới hạn 960mm!')
                    self.state = SystemState.ERROR
                    return
                
                row1_limit = self.config.iny_input_stack.get(1, 600.0)
                S4_MIN_TRAVEL = 100.0  # INY phải jog được 100mm mới bắt đầu đọc S4
                
                # Bắt đầu jog nếu chưa chạy
                if not self._iny_jog_active:
                    self.get_logger().info(f"▶️  S3 ON — InY bắt đầu jog từ {iny_pos:.1f}mm, S4 chỉ xét sau khi INY đạt 150mm")
                    self.start_iny_jog(self.config.iny_search_velocity)
                    self._s4_search_active = False
                    return
                
                # Mở cổng S4 khi INY đạt vị trí tuyệt đối >= 150mm
                S4_ACTIVE_POSITION = 150.0  # mm — vị trí tuyệt đối để bắt đầu xét S4
                if iny_pos >= S4_ACTIVE_POSITION:
                    self._s4_search_active = True
                elif not self._s4_search_active:
                    self.log_guide_once("S4_GATE", f"⏳ INY={iny_pos:.0f}mm — chưa xét S4 (chờ đến 150mm)")
                
                # Xét S4 chỉ khi cổng đã mở (_s4_search_active)
                if self._s4_search_active and self.sensor_manager.get_sensor(4):
                    # S4 ON — phát hiện chồng khay
                    self.stop_iny_jog()
                    current_pos = self.get_servo_position(2)
                    if current_pos is not None:
                        self.current_row = self.find_nearest_row(current_pos, self.config.iny_input_stack)
                        self.stack_row_index = self.current_row
                        self.get_logger().info(f"📍 S4 ON (gate open) - Tray at {current_pos:.1f}mm → Row {self.current_row} (snap UP)")
                        self._s4_search_active = False
                        self.state = SystemState.S1_INY_TO_NEAREST_ROW
                elif iny_pos >= row1_limit:
                    # Layer 1: S4 không ON — INY về safe + INX về home, thử lại
                    self.stop_iny_jog()
                    self._s4_search_active = False
                    self.get_logger().warn(f"⚠️ S4 không ON tại {row1_limit:.0f}mm — INY về safe + INX về home")
                    self._notify_gui('warn', '⚠️ Không thấy khay (S4)', f'INY đến {row1_limit:.0f}mm mà S4 không ON — đang về safe')
                    # INY về safe 10mm (non-blocking, INX cũng về cùng lúc)
                    self.move_servo(2, self.config.iny_home, wait=False)
                    # INX về home (non-blocking)
                    if 1 in self.servos:
                        try:
                            mot = self.servos[1]
                            offset = self.zero_offset.get(1, 0)
                            pos_counts = offset + int(self.config.inx_home * COUNTS_PER_MM)
                            self._ensure_drive_ready(mot, 1)
                            mot.position_task(pos_counts, DEFAULT_VELOCITY, absolute=True, nonblocking=True)
                        except Exception as e:
                            self.get_logger().error(f"❌ INX return home error: {e}")
                    # Reset từ đầu State1
                    self._inx_arrived = False
                    self._s3_seen_during_move = False
                    self._inx_s3_wait_start = None
                    self.state = SystemState.S1_INY_CONFIRM_SAFE
        
        elif self.state == SystemState.S1_INY_TO_NEAREST_ROW:
            # Di chuyển đến vị trí row đã snap UP từ S4
            target_pos = self.config.iny_input_stack.get(self.current_row, self.config.iny_input_stack[1])
            if self.move_servo(2, target_pos):
                self.get_logger().info(f"✅ InY at row {self.current_row} ({target_pos}mm) — check S5 xác nhận có khay")
                self._s5_check_start = None  # Reset timer S5
                self.clear_guide()
                self.state = SystemState.S1_CHECK_S5
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S1_CHECK_S5:
            # ── Kiểm tra S5 xác nhận có khay tại vị trí row (timeout 2s) ──
            if self.sensor_manager.get_sensor(5):  # S5 ON → có khay
                self.get_logger().info("✅ [S1] S5 ON — có khay tại Input Stack. Extend Cyl1 →")
                self._s5_retry_count = 0  # Reset counter khi thành công
                self._s5_check_start = None
                self.extend_cylinder1()
                self.start_cylinder_timer()
                self.clear_guide()
                self.state = SystemState.S1_CYLINDER1_EXTEND
                self.log_guide_once("S1_WAIT_S11", "[S1] Cylinder 1 đang extend → Đợi S11 ON (Cyl1 MAX). Kích: '11:1'")
            else:
                # Bắt đầu đếm timeout
                if self._s5_check_start is None:
                    self._s5_check_start = time.time()
                    self.log_guide_once("S1_WAIT_S5", f"[S1] Chờ S5 ON (xác nhận khay tại stack). Timeout: 2s. Kích: '5:1'")
                
                elapsed = time.time() - self._s5_check_start
                if elapsed >= 2.0:  # Timeout 2s
                    self._s5_check_start = None
                    self._s5_retry_count += 1
                    
                    # INY về safe 10mm, INX về safe 10mm (non-blocking)
                    self.move_servo(2, self.config.iny_home, wait=False)
                    if 1 in self.servos:
                        try:
                            mot = self.servos[1]
                            offset = self.zero_offset.get(1, 0)
                            pos_counts = offset + int(self.config.inx_home * COUNTS_PER_MM)
                            self._ensure_drive_ready(mot, 1)
                            mot.position_task(pos_counts, DEFAULT_VELOCITY, absolute=True, nonblocking=True)
                        except Exception as e:
                            self.get_logger().error(f"❌ INX return safe error: {e}")
                    
                    if self._s5_retry_count >= 3:
                        # ⛔ Fail 3 lần → STOP hệ thống
                        msg = "⛔ S5 fail 3 lần liên tiếp — DỪNG HỆ THỐNG! Kiểm tra cảm biến S5 và vị trí khay"
                        self.get_logger().error(msg)
                        self._notify_gui('error', '⛔ DỪNG HỆ THỐNG', 'S5 fail 3 lần — Kiểm tra cảm biến S5!')
                        self._s5_retry_count = 0
                        self.state = SystemState.ERROR
                    else:
                        # ⚠️ Cảnh báo + chờ dashboard confirm
                        retry_msg = f"⚠️ [S1] Không có khay tại Input Stack (lần {self._s5_retry_count}/3) — kiểm tra S5"
                        self.get_logger().warn(retry_msg)
                        self._notify_gui('warn', f'⚠️ Không có khay Stack (lần {self._s5_retry_count}/3)',
                                         'Kiểm tra S5 rồi nhấn Confirm để thử lại')
                        # Reset về CONFIRM_SAFE để chờ xác nhận
                        self._inx_arrived = False
                        self._s3_seen_during_move = False
                        self._inx_s3_wait_start = None
                        self.state = SystemState.S1_WAIT_DASHBOARD_CONFIRM
        
        elif self.state == SystemState.S1_WAIT_DASHBOARD_CONFIRM:
            # ── Chờ operator confirm trên Dashboard ──
            # Confirm đến qua next_step service hoặc manual_mode step
            if self.step_pending:
                self.step_pending = False
                self.clear_guide()
                # Kiểm tra S3: nếu S3 vẫn ON → còn khay → retry
                if self.sensor_manager.get_sensor(3):
                    self.get_logger().info(f"▶️  [S1] Operator confirmed. S3 ON → Retry lần {self._s5_retry_count + 1}/3")
                    self.state = SystemState.S1_INY_CONFIRM_SAFE  # Retry từ đầu
                else:
                    self.get_logger().warn("⚠️ [S1] S3 OFF — Hết khay trên băng tải. Dừng chờ khay mới")
                    self._notify_gui('warn', '⚠️ S3 OFF', 'Hết khay trên băng tải — đang chờ khay mới')
                    # Giữ state chờ S3 ON lại
                    self.state = SystemState.S1_INY_CONFIRM_SAFE
            else:
                self.log_guide_once("S1_WAIT_CONFIRM",
                    f"[S1] Chờ xác nhận từ Dashboard (lần {self._s5_retry_count}/3). "
                    f"Nhấn 'Next Step' để thử lại. S3={'ON' if self.sensor_manager.get_sensor(3) else 'OFF'}")
        
        elif self.state == SystemState.S1_CYLINDER1_EXTEND:
            # Chờ S11 ON (Cylinder 1 MAX - extend complete = đã gắp khay)
            if self.sensor_manager.get_sensor(11):  # S11 = Cyl1 MAX (extend)
                self.get_logger().info("✅ [S1] Cylinder 1 extended (S11 ON) — đã gắp khay")
                self._cylinder_start_time = None
                self.clear_guide()
                # ── Check S13 (Cyl3 hold tray) ──
                if self.sensor_manager.get_sensor(13):  # S13 ON = Cyl3 đang giữ khay
                    # INTERLOCK: INY không được về safe khi S13 ON + S11 ON + InX ở Target2
                    self.get_logger().warn(
                        "⚠️ [S1] S13 ON (Cyl3 đang GIỮ khay) + S11 ON → INY bị INTERLOCK! "
                        "Chờ S12 ON để nhả giữ rồi mới được về safe"
                    )
                    self._notify_gui('warn', '⚠️ INY INTERLOCK', 'S13 ON: Cyl3 đang giữ khay — Chờ S12 để nhả')
                    self.state = SystemState.S1_WAIT_S12_INTERLOCK
                else:
                    # S13 OFF → không có hold tray → INY về safe trực tiếp
                    self.get_logger().info("[S1] S13 OFF — không có hold tray. INY về safe 50mm →")
                    self.start_motion_delay(SystemState.S1_INY_TO_TARGET1)
            elif self.check_cylinder_timeout():
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S1_WAIT_S12_INTERLOCK:
            # ── INY bị INTERLOCK — chờ S12 ON (Cyl3 retracted = hold tray đã nhả) ──
            if self.sensor_manager.get_sensor(12):  # S12 ON = Cyl3 retracted
                self.get_logger().info("✅ [S1] S12 ON — Cyl3 đã nhả giữ. INY được phép về safe 50mm")
                self.clear_guide()
                # INY về safe 50mm
                self.state = SystemState.S1_INY_TO_TARGET1
            else:
                self.log_guide_once("S1_INTERLOCK_WAIT",
                    "[S1] ⛔ INY INTERLOCK: InX=Target2 + S11 ON + S13 ON. "
                    "Chờ S12 ON (Cyl3 retracted) để nhả giữ. Kích: '12:1'")
        
        elif self.state == SystemState.S1_INY_TO_TARGET1:
            # InY về safe zone 50mm trước khi kích Cyl3 hoặc đi Target2
            if self.move_servo(2, self.config.iny_safe_zone):  # 50mm safe zone
                self.get_logger().info("✅ [S1] InY về safe 50mm — kích Cyl3 extend (hold tray)")
                # ── Kích Cyl3 extend: ch9 ON, ch8 OFF ──
                self.extend_cylinder3()
                self.start_cylinder_timer()
                self.clear_guide()
                self.state = SystemState.S1_CYL3_EXTEND
                self.log_guide_once("S1_WAIT_S13", "[S1] Cyl3 extend → Chờ S13 ON (hold tray kẹp). Kích: '13:1'")
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S1_CYL3_EXTEND:
            # ── Chờ S13 ON (Cyl3 extended = đang giữ khay chặt) ──
            if self.sensor_manager.get_sensor(13):  # S13 ON = Cyl3 extended
                self.get_logger().info("✅ [S1] S13 ON — Cyl3 đang giữ khay. INY hạ xuống 200mm đặt khay")
                self._cylinder_start_time = None
                self.clear_guide()
                self.start_motion_delay(SystemState.S1_INY_TO_TARGET2)
            elif self.check_cylinder_timeout():
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S1_INY_TO_TARGET2:
            # InY đến Target 2 (200mm) — vị trí đặt khay cho robot
            if self.move_servo(2, self.config.iny_target2):
                self.get_logger().info("✅ [S1] InY at 200mm — nhả Cyl1 (retract)")
                self.retract_cylinder1()
                self.start_cylinder_timer()
                self.clear_guide()
                self.start_motion_delay(SystemState.S1_CYLINDER1_RETRACT)
                self.log_guide_once("S1_WAIT_S10", "[S1] Cyl1 retract → Chờ S10 ON + S11 OFF. Kích: '10:1'")
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S1_CYLINDER1_RETRACT:
            # Chờ S10 ON (Cyl1 MIN = retracted) VÀ S11 OFF (xác nhận đã nhả hoàn toàn)
            s10 = self.sensor_manager.get_sensor(10)
            s11 = self.sensor_manager.get_sensor(11)
            if s10 and not s11:  # S10 ON + S11 OFF = nhả khay hoàn toàn
                self.get_logger().info("✅ [S1] Cyl1 retracted (S10 ON, S11 OFF) — khay đã được đặt. INY về safe")
                self._cylinder_start_time = None
                self.clear_guide()
                self.start_motion_delay(SystemState.S1_INY_RETURN_SAFE)
            elif self.check_cylinder_timeout():
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S1_INY_RETURN_SAFE:
            # InY về safe 50mm sau khi nhả khay
            if self.move_servo(2, self.config.iny_safe_zone):
                iny_pos = self.get_servo_position(2)
                if iny_pos is not None and iny_pos > self.config.iny_safe_zone:
                    self.get_logger().warn(f"⏳ InY đang về safe: {iny_pos:.1f}mm → {self.config.iny_safe_zone:.1f}mm")
                    return
                self.get_logger().info(f"✅ [S1] InY về safe {self.config.iny_safe_zone:.1f}mm — InX về safe")
                self.start_motion_delay(SystemState.S1_INX_RETURN_SAFE)
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S1_INX_RETURN_SAFE:
            # InX về safe zone — CHỈ KHI InY đã safe
            if self.is_iny_safe_for_inx_move():
                if self.move_servo(1, self.config.inx_home):
                    self.get_logger().info("✅ [S1] InX về safe — State 1 Pos 1 HOÀN THÀNH")
                    self._s5_retry_count = 0  # Reset S5 retry cho lần sau
                    self._pos1_done = True
                    if self._pos2_done:
                        self.state = SystemState.STATE1_COMPLETE
                    else:
                        self.get_logger().info("⏳ Position 1 done, chờ Position 2 (Servo 3)...")
                else:
                    self.state = SystemState.ERROR
            else:
                self.get_logger().warn("⏳ Chờ InY về safe trước khi InX di chuyển...")
        
        elif self.state == SystemState.STATE1_COMPLETE:
            # ✅ Pub new_tray_loaded với retry, chờ ACK từ robot
            if not self._tray_loaded_ack:
                self.publish_with_retry(self.pub_new_tray_loaded, Bool(data=True), '/revpi/new_tray_loaded')
                
                # Check is_last_tray (pub song song, không cần chờ ACK)
                if not self._state_completed_published:
                    if not self.check_has_trays_remaining():
                        self.pub_is_last_tray.publish(Bool(data=True))
                        self.get_logger().warn("🚨 S1+S2+S3 ALL OFF after State 1 → Published is_last_tray")
                    self._state_completed_published = True
            else:
                # ✅ Robot đã ACK → transition an toàn
                self.get_logger().info("🎉 STATE 1 COMPLETE - Robot ACK received, transitioning")
                self._tray_loaded_ack = False
                self._state_completed_published = False
                self.reset_pub_retry()
                self.state = SystemState.S2_WAIT_TRIGGER
        
        # ══════════════════════════════════════════════════════════════
        # STATE 2: THAY KHAY INPUT (Position 1) - Chờ implement
        # ══════════════════════════════════════════════════════════════
        
        elif self.state == SystemState.S2_WAIT_TRIGGER:
            # Chờ robot báo done tray (xong hết cartridge) + robot IDLE → vào lấy khay
            if self.change_tray_input_signal:
                if not self.robot_motion_busy:
                    self.get_logger().info("[S2] Robot done tray + IDLE → Starting State 2 (thay khay input)")
                    self.change_tray_input_signal = False
                    self.state = SystemState.S2_CHECK_INTERLOCK
                else:
                    self.log_guide_once("S2_WAIT_ROBOT", "[S2] Robot done tray but BUSY → Waiting robot IDLE...")
        
        # ══════════════════════════════════════════════════════════════
        # STATE 2 - PHASE A: Thu hồi khay cũ từ robot → output stack
        # ══════════════════════════════════════════════════════════════
        
        elif self.state == SystemState.S2_CHECK_INTERLOCK:
            # ✅ Đọc S5 SỚM — xác định output pos1 trống hay có khay
            # Lưu vào flag để dùng xuyên suốt Phase A (tránh đọc lại giữa chừng)
            self._s2_output_empty = not self.sensor_manager.get_sensor(5)
            s5_status = 'TRỐNG (row 1)' if self._s2_output_empty else 'CÓ KHAY (search S4)'
            self.get_logger().info(f'📡 [S2] S5={("OFF" if self._s2_output_empty else "ON")} → Output pos1 {s5_status}')

            # Check INY safe, INX safe, S3
            iny_safe = self.is_iny_safe_for_inx_move()
            s3_status = self.sensor_manager.get_sensor(3)
            
            if iny_safe:
                self.get_logger().info(f'✅ [S2-A] Interlock OK (INY safe, S3={"ON" if s3_status else "OFF"}, S5={"OFF→row1" if self._s2_output_empty else "ON→search"})')
                self.state = SystemState.S2_INX_TO_TARGET2
            else:
                self.get_logger().warn("⏳ [S2-A] Waiting INY safe for Phase A...")
                self.move_servo(2, self.config.iny_home, wait=False)
        
        elif self.state == SystemState.S2_INX_TO_TARGET2:
            # INX đi vào vị trí robot pickup (target2)
            if self.is_iny_safe_for_inx_move():
                if self.move_servo(1, self.config.inx_target2):
                    self.get_logger().info("✅ [S2-A] INX at target2 (robot area)")
                    self.state = SystemState.S2_INY_DOWN_TO_TRAY
                else:
                    self.state = SystemState.ERROR
            else:
                self.get_logger().warn("⏳ [S2-A] Waiting INY safe for INX move to target2...")
        
        elif self.state == SystemState.S2_INY_DOWN_TO_TRAY:
            # INY hạ xuống vị trí khay robot đặt (target2)
            if self.move_servo(2, self.config.iny_target2):
                self.get_logger().info("✅ [S2-A] INY at tray position (target2)")
                self.extend_cylinder1()
                self.start_cylinder_timer()
                self.clear_guide()
                self.state = SystemState.S2_CYLINDER1_EXTEND
                self.log_guide_once("S2A_WAIT_S11", "[S2-A] Cylinder 1 đang extend (kẹp khay cũ) → Đợi S11 ON (Cyl1 MAX). Kích: '11:1'")
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_CYLINDER1_EXTEND:
            # Chờ S11 ON (Cylinder 1 MAX - extend = grip khay)
            if self.sensor_manager.get_sensor(11):
                self.get_logger().info("✅ [S2-A] Cylinder 1 extended (S11 ON) - Tray grabbed")
                self._cylinder_start_time = None
                self.clear_guide()
                self.state = SystemState.S2_INY_UP_SAFE
            elif self.check_cylinder_timeout():
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_INY_UP_SAFE:
            # INY nâng khay lên safe position
            if self.move_servo(2, self.config.iny_home):
                self.get_logger().info("✅ [S2-A] INY up with tray (safe)")
                self.state = SystemState.S2_INX_TO_TARGET1
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_INX_TO_TARGET1:
            # INX về safe zone (10mm) — để INY có không gian jog vào output stack
            if self.is_iny_safe_for_inx_move():
                if self.move_servo(1, self.config.inx_safe_zone):
                    self.get_logger().info(f"✅ [S2-A] INX về safe zone ({self.config.inx_safe_zone}mm) — sẵn sàng cho INY search")
                    self.state = SystemState.S2_INY_SEARCH_STACK
                else:
                    self.state = SystemState.ERROR
            else:
                self.get_logger().warn("⏳ [S2-A] Waiting INY safe for INX move to safe zone...")
        
        elif self.state == SystemState.S2_INY_SEARCH_STACK:
            # ✅ Dùng flag đã đọc từ S2_CHECK_INTERLOCK (S5 đã được snapshot trước khi vào Phase A)
            if getattr(self, '_s2_output_empty', False):
                self.stop_iny_jog()
                self.output_stack_row = 1
                self.get_logger().info('📭 [S2-A] Output pos1 trống (S5 OFF lúc vào) → Đặt thẳng row 1')
                self.state = SystemState.S2_INY_TO_NEAREST_ROW
            # ✅ MANUAL MODE: Nếu đã set target row, bỏ qua sensor search
            elif self.manual_mode and self.manual_target_row is not None:
                self.output_stack_row = self.manual_target_row
                self.get_logger().info(f"🎯 MANUAL [S2-A]: Row set to {self.manual_target_row} (skipping search)")
                self.manual_target_row = None  # Clear after use
                self.state = SystemState.S2_INY_TO_NEAREST_ROW
            # INY hạ xuống, chờ S4 ON (phát hiện stack)
            elif not self.sensor_manager.get_sensor(4):  # S4 OFF
                iny_pos, pos_ok = self.get_position_safe(2)
                if iny_pos is None:
                    if not pos_ok:
                        self.state = SystemState.ERROR
                    return
                row1_limit = self.config.iny_output_stack.get(1, 590.0)  # S2 PhaseA: search OUTPUT stack (đặt khay cũ)
                if iny_pos >= row1_limit:
                    # ✅ Đạt row 1 (thấp nhất) mà S4 vẫn OFF → stack rỗng, dùng row 1
                    self.stop_iny_jog()
                    self.output_stack_row = 1
                    self.get_logger().warn(f"⚠️ [S2-A] INY reached row 1 limit ({row1_limit}mm), S4 OFF → Using row 1 (stack empty or first tray)")
                    self.state = SystemState.S2_INY_TO_NEAREST_ROW
                else:
                    # Chưa đến row 1 → tiếp tục jog xuống (gọi 1 lần)
                    self.start_iny_jog(self.config.iny_search_velocity)
            else:
                # S4 ON → phát hiện stack, dừng lại
                self.stop_iny_jog()
                current_pos = self.get_servo_position(2)
                if current_pos is not None:
                    self.output_stack_row = self.find_nearest_row(current_pos, self.config.iny_output_stack)
                    self.get_logger().info(f"📍 [S2-A] S4 ON at {current_pos:.1f}mm → Output stack row {self.output_stack_row}")
                    self.state = SystemState.S2_INY_TO_NEAREST_ROW
        
        elif self.state == SystemState.S2_INY_TO_NEAREST_ROW:
            # Di chuyển đến vị trí row gần nhất (tốc độ chậm)
            target_pos = self.config.iny_output_stack.get(self.output_stack_row, self.config.iny_output_stack[1])
            if self.move_servo(2, target_pos):
                self.get_logger().info(f"✅ [S2-A] INY at output stack row {self.output_stack_row} ({target_pos}mm)")
                self.retract_cylinder1()
                self.start_cylinder_timer()
                self.clear_guide()
                self.state = SystemState.S2_CYLINDER1_RETRACT
                self.log_guide_once("S2A_WAIT_S10", "[S2-A] Cylinder 1 đang retract (thả khay) → Đợi S10 ON (Cyl1 MIN). Kích: '10:1'")
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_CYLINDER1_RETRACT:
            # Chờ S10 ON (Cylinder 1 MIN - retract = thả khay)
            if self.sensor_manager.get_sensor(10):
                self.get_logger().info("✅ [S2-A] Cylinder 1 retracted (S10 ON) - Tray released to output stack")
                self._cylinder_start_time = None
                self.clear_guide()
                self.state = SystemState.S2_INY_RETURN_SAFE
            elif self.check_cylinder_timeout():
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_INY_RETURN_SAFE:
            # INY về safe position
            if self.move_servo(2, self.config.iny_home):
                self.get_logger().info("✅ [S2-A] INY returned to safe position")
                self.state = SystemState.S2_INX_RETURN_SAFE
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_INX_RETURN_SAFE:
            # INX về safe position (sau khi INY safe)
            if self.is_iny_safe_for_inx_move():
                if self.move_servo(1, self.config.inx_home):
                    self.get_logger().info("✅ [S2-A] INX returned to safe - Phase A complete")
                    self.state = SystemState.S2_CHECK_REMAINING
                else:
                    self.state = SystemState.ERROR
            else:
                self.get_logger().warn("⏳ [S2-A] Waiting INY safe for INX return...")
        
        elif self.state == SystemState.S2_CHECK_REMAINING:
            # Double-check: còn khay trên input stack không?
            s1_on = self.sensor_manager.get_sensor(1)
            s2_on = self.sensor_manager.get_sensor(2)
            s3_on = self.sensor_manager.get_sensor(3)
            stack_ok = (self.stack_row_index > 1)

            if s1_on or s2_on or s3_on or stack_ok:
                self.get_logger().info(
                    f"✅ [S2-B] Trays remaining (S1={'ON' if s1_on else 'OFF'}, "
                    f"S2={'ON' if s2_on else 'OFF'}, S3={'ON' if s3_on else 'OFF'}, "
                    f"stack_row={self.stack_row_index}) → Waiting robot idle"
                )
                self.state = SystemState.S2_LOAD_WAIT_ROBOT
            else:
                self.get_logger().warn("🚨 [S2] No trays remaining → Publishing is_last_tray")
                self.state = SystemState.S2_NO_TRAY

        elif self.state == SystemState.S2_LOAD_WAIT_ROBOT:
            # Chờ robot không đang pick (not busy) trước khi INX vào khu vực robot
            if not self.robot_motion_busy:
                self.get_logger().info("✅ [S2-B] Robot IDLE → INX moving to target2 (non-blocking)")
                # INX non-blocking vào target2 (500mm) — tương tự State 1
                if self.is_iny_safe_for_inx_move():
                    if 1 in self.servos:
                        try:
                            mot = self.servos[1]
                            offset = self.zero_offset.get(1, 0)
                            pos_counts = offset + int(self.config.inx_target2 * COUNTS_PER_MM)
                            self._ensure_drive_ready(mot, 1)
                            mot.position_task(pos_counts, DEFAULT_VELOCITY, absolute=True, nonblocking=True)
                        except Exception as e:
                            self.get_logger().error(f"❌ [S2-B] INX non-blocking move error: {e}")
                    self._inx_arrived = False
                    self._inx_s3_wait_start = None
                    self._s3_seen_during_move = False
                    self.state = SystemState.S2_LOAD_WAIT_S3
                else:
                    self.get_logger().warn("⏳ [S2-B] Waiting INY safe for INX move...")
            else:
                self.log_guide_once("S2B_WAIT_ROBOT", "[S2-B] Robot đang BUSY (pick/place) — chờ robot IDLE...")

        elif self.state == SystemState.S2_LOAD_WAIT_S3:
            # Chờ INX dừng + S3 ON — tương tự S1_INX_WAIT_STOP
            
            # Ghi nhớ S3 ON trong khi INX chưa dừng (buffer)
            if self.sensor_manager.get_sensor(3) and not self._s3_seen_during_move:
                self._s3_seen_during_move = True
                self.get_logger().info("[S2-B] S3 ON ghi nhận (INX chưa dừng — xử lý sau)")
            
            # Bước 1: Check INX đã dừng
            if not self._inx_arrived:
                if 1 in self.servos:
                    try:
                        if self.servos[1].target_position_reached():
                            self._inx_arrived = True
                            self.get_logger().info("✅ [S2-B] INX dừng hoàn toàn tại target2")
                    except Exception:
                        pass

            # Bước 2: Khi INX dừng → chờ S3 ON
            if self._inx_arrived:
                s3_ready = self.sensor_manager.get_sensor(3) or self._s3_seen_during_move
                if s3_ready:
                    # ✅ INTERLOCK S10: Cylinder 1 phải retract trước khi INY jog
                    if not self.sensor_manager.get_sensor(10):
                        self.log_guide_once("S2B_WAIT_S10",
                            "[S2-B] S3 ON nhưng S10 chưa ON (Cyl1 chưa retract) — chờ S10 ON")
                        return
                    self._inx_s3_wait_start = None
                    self._s3_seen_during_move = False
                    self.get_logger().info("✅ [S2-B] INX dừng + S3 ON + S10 OK → Starting INY search")
                    self.state = SystemState.S2_LOAD_INY_SEARCH_TRAY
                else:
                    # S3 OFF → chờ tối đa 30s
                    if self._inx_s3_wait_start is None:
                        self._inx_s3_wait_start = time.time()
                        self.log_guide_once("S2B_WAIT_S3", "[S2-B] INX tại target2, chờ S3 ON (tối đa 30s). Kích: '3:1'")
                    elif time.time() - self._inx_s3_wait_start >= 30.0:
                        self.get_logger().warn("⚠️ [S2-B] S3 không ON sau 30s — INX về Home, dùng fallback")
                        self._notify_gui('warn', '⚠️ S3 timeout', 'S3 không ON — dùng last stack row')
                        self._inx_s3_wait_start = None
                        self._inx_arrived = False
                        self._s3_seen_during_move = False
                        self.state = SystemState.S2_NO_TRAY

        
        # ══════════════════════════════════════════════════════════════
        # STATE 2 - PHASE B: Nạp khay mới từ input stack → robot
        # (Giống State 1 nhưng có S4 fallback)
        # ══════════════════════════════════════════════════════════════
        
        elif self.state == SystemState.S2_LOAD_INX_TO_TARGET2:
            # INX đi vào vị trí input stack
            if self.is_iny_safe_for_inx_move():
                if self.move_servo(1, self.config.inx_target2):
                    self.get_logger().info("✅ [S2-B] INX at input stack (target2)")
                    self.state = SystemState.S2_LOAD_INY_SEARCH_TRAY
                else:
                    self.state = SystemState.ERROR
            else:
                self.get_logger().warn("⏳ [S2-B] Waiting INY safe for INX move to input stack...")
        
        elif self.state == SystemState.S2_LOAD_INY_SEARCH_TRAY:
            # ✅ MANUAL MODE: Nếu đã set target row, bỏ qua sensor search
            if self.manual_mode and self.manual_target_row is not None:
                self.stack_row_index = self.manual_target_row
                self.get_logger().info(f"🎯 MANUAL [S2-B]: Row set to {self.manual_target_row} (skipping search)")
                self.manual_target_row = None  # Clear after use
                self.state = SystemState.S2_LOAD_INY_TO_ROW
            # INY tìm khay mới bằng S4, fallback bằng stack_row_index
            elif not self.sensor_manager.get_sensor(4):  # S4 OFF
                iny_pos, pos_ok = self.get_position_safe(2)
                if iny_pos is None:
                    if not pos_ok:
                        self.state = SystemState.ERROR
                    return
                row1_limit = self.config.iny_input_stack.get(1, 600.0)  # S2 PhaseB: search INPUT stack (lấy khay mới)
                if iny_pos >= row1_limit:
                    # Đạt row 1 limit, S4 OFF → dùng stack_row_index fallback
                    self.stop_iny_jog()
                    if self.stack_row_index > 1:
                        # Dùng vị trí stack đã nhớ - 1
                        self.stack_row_index -= 1
                        self.get_logger().warn(f"⚠️ [S2-B] S4 OFF at row 1 limit → Fallback to stack_row_index={self.stack_row_index}")
                    else:
                        # stack_row_index = 1, dùng row 1
                        self.stack_row_index = 1
                        self.get_logger().warn(f"⚠️ [S2-B] S4 OFF, stack at bottom → Using row 1")
                    self.state = SystemState.S2_LOAD_INY_TO_ROW
                else:
                    # Chưa đến row 1 → tiếp tục jog (gọi 1 lần)
                    self.start_iny_jog(self.config.iny_search_velocity)
            else:
                # S4 ON → phát hiện khay mới
                self.stop_iny_jog()
                current_pos = self.get_servo_position(2)
                if current_pos is not None:
                    new_row = self.find_nearest_row(current_pos, self.config.iny_input_stack)
                    self.stack_row_index = new_row  # Update stack tracking
                    self.get_logger().info(f"📍 [S2-B] S4 ON - New tray at {current_pos:.1f}mm → Row {new_row}")
                    self.state = SystemState.S2_LOAD_INY_TO_ROW
        
        elif self.state == SystemState.S2_LOAD_INY_TO_ROW:
            # Di chuyển đến vị trí row chính xác
            target_pos = self.config.iny_input_stack.get(self.stack_row_index, self.config.iny_input_stack[1])
            if self.move_servo(2, target_pos):
                self.get_logger().info(f"✅ [S2-B] INY at row {self.stack_row_index} ({target_pos}mm) — check S5")
                self._s5_check_start = None
                self.clear_guide()
                self.state = SystemState.S2_LOAD_CHECK_S5
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_LOAD_CHECK_S5:
            # ── Kiểm tra S5 xác nhận có khay (timeout 2s) ──
            if self.sensor_manager.get_sensor(5):  # S5 ON → có khay
                self.get_logger().info("✅ [S2-B] S5 ON — có khay. Extend Cyl1")
                self._s5_retry_count = 0
                self._s5_check_start = None
                self.extend_cylinder1()
                self.start_cylinder_timer()
                self.clear_guide()
                self.state = SystemState.S2_LOAD_CYLINDER1_EXTEND
                self.log_guide_once("S2B_WAIT_S11", "[S2-B] Cyl1 extend → Đợi S11 ON (Cyl1 MAX). Kích: '11:1'")
            else:
                if self._s5_check_start is None:
                    self._s5_check_start = time.time()
                    self.log_guide_once("S2B_WAIT_S5", "[S2-B] Chờ S5 ON (xác nhận khay). Timeout: 2s. Kích: '5:1'")
                
                elapsed = time.time() - self._s5_check_start
                if elapsed >= 2.0:
                    self._s5_check_start = None
                    self._s5_retry_count += 1
                    # INY về safe, INX về safe (non-blocking)
                    self.move_servo(2, self.config.iny_home, wait=False)
                    if 1 in self.servos:
                        try:
                            mot = self.servos[1]
                            offset = self.zero_offset.get(1, 0)
                            pos_counts = offset + int(self.config.inx_home * COUNTS_PER_MM)
                            self._ensure_drive_ready(mot, 1)
                            mot.position_task(pos_counts, DEFAULT_VELOCITY, absolute=True, nonblocking=True)
                        except Exception as e:
                            self.get_logger().error(f"❌ [S2-B] INX return safe error: {e}")
                    
                    if self._s5_retry_count >= 3:
                        self.get_logger().error("⛔ [S2-B] S5 fail 3 lần — DỪNG HỆ THỐNG! Kiểm tra S5!")
                        self._notify_gui('error', '⛔ DỪNG HỆ THỐNG [S2-B]', 'S5 fail 3 lần — Kiểm tra cảm biến S5!')
                        self._s5_retry_count = 0
                        self.state = SystemState.ERROR
                    else:
                        self.get_logger().warn(f"⚠️ [S2-B] S5 fail lần {self._s5_retry_count}/3 — chờ confirm")
                        self._notify_gui('warn', f'⚠️ [S2-B] Không có khay (lần {self._s5_retry_count}/3)',
                                         'Kiểm tra S5 rồi nhấn Confirm để thử lại')
                        self._inx_arrived = False
                        self._s3_seen_during_move = False
                        self._inx_s3_wait_start = None
                        self.state = SystemState.S2_LOAD_WAIT_CONFIRM
        
        elif self.state == SystemState.S2_LOAD_WAIT_CONFIRM:
            # ── Chờ operator confirm ──
            if self.step_pending:
                self.step_pending = False
                self.clear_guide()
                if self.sensor_manager.get_sensor(3):
                    self.get_logger().info(f"▶️  [S2-B] S3 ON → Retry lần {self._s5_retry_count + 1}/3")
                    self.state = SystemState.S2_LOAD_WAIT_S3  # Retry từ chờ S3
                else:
                    self.get_logger().warn("⚠️ [S2-B] S3 OFF — Hết khay trên băng tải")
                    self._notify_gui('warn', '⚠️ [S2-B] S3 OFF', 'Hết khay — đang chờ khay mới')
                    self.state = SystemState.S2_LOAD_WAIT_S3
            else:
                self.log_guide_once("S2B_WAIT_CONFIRM",
                    f"[S2-B] Chờ xác nhận (lần {self._s5_retry_count}/3). "
                    f"Nhấn 'Next Step'. S3={'ON' if self.sensor_manager.get_sensor(3) else 'OFF'}")
        
        elif self.state == SystemState.S2_LOAD_CYLINDER1_EXTEND:
            # Chờ S11 ON (grip khay mới)
            if self.sensor_manager.get_sensor(11):
                self.get_logger().info("✅ [S2-B] S11 ON — đã gắp khay mới")
                self._cylinder_start_time = None
                self.clear_guide()
                # Check S13 interlock
                if self.sensor_manager.get_sensor(13):  # S13 ON = Cyl3 đang giữ
                    self.get_logger().warn("⚠️ [S2-B] S13 ON → INY INTERLOCK! Chờ S12 ON")
                    self._notify_gui('warn', '⚠️ [S2-B] INY INTERLOCK', 'S13 ON — Chờ S12 để nhả')
                    self.state = SystemState.S2_LOAD_WAIT_S12_INTERLOCK
                else:
                    self.get_logger().info("[S2-B] S13 OFF — INY về safe 50mm")
                    self.start_motion_delay(SystemState.S2_LOAD_INY_UP_SAFE)
            elif self.check_cylinder_timeout():
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_LOAD_WAIT_S12_INTERLOCK:
            # ── INY bị INTERLOCK — chờ S12 ON ──
            if self.sensor_manager.get_sensor(12):
                self.get_logger().info("✅ [S2-B] S12 ON — Cyl3 nhả giữ. INY về safe")
                self.clear_guide()
                self.state = SystemState.S2_LOAD_INY_UP_SAFE
            else:
                self.log_guide_once("S2B_INTERLOCK",
                    "[S2-B] ⛔ INY INTERLOCK: S11 ON + S13 ON. Chờ S12 ON. Kích: '12:1'")
        
        elif self.state == SystemState.S2_LOAD_INY_UP_SAFE:
            # INY nâng khay mới lên safe 50mm + kích Cyl3 extend
            if self.move_servo(2, self.config.iny_safe_zone):
                self.get_logger().info("✅ [S2-B] INY về safe 50mm — kích Cyl3 extend (hold tray)")
                self.extend_cylinder3()
                self.start_cylinder_timer()
                self.clear_guide()
                self.state = SystemState.S2_LOAD_CYL3_EXTEND
                self.log_guide_once("S2B_WAIT_S13", "[S2-B] Cyl3 extend → Chờ S13 ON. Kích: '13:1'")
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_LOAD_CYL3_EXTEND:
            # Chờ S13 ON (Cyl3 giữ khay)
            if self.sensor_manager.get_sensor(13):
                self.get_logger().info("✅ [S2-B] S13 ON — Cyl3 đang giữ. INY đến robot pos (200mm)")
                self._cylinder_start_time = None
                self.clear_guide()
                self.start_motion_delay(SystemState.S2_LOAD_INY_TO_ROBOT_POS)
            elif self.check_cylinder_timeout():
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_LOAD_INY_TO_ROBOT_POS:
            # INY đến vị trí robot place (target2 = 200mm)
            if self.move_servo(2, self.config.iny_target2):
                self.get_logger().info("✅ [S2-B] INY at 200mm — nhả Cyl1 (retract)")
                self.retract_cylinder1()
                self.start_cylinder_timer()
                self.clear_guide()
                self.state = SystemState.S2_LOAD_CYLINDER1_RETRACT
                self.log_guide_once("S2B_WAIT_S10", "[S2-B] Cyl1 retract → Chờ S10 ON + S11 OFF. Kích: '10:1'")
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_LOAD_CYLINDER1_RETRACT:
            # Chờ S10 ON + S11 OFF (nhả hoàn toàn)
            s10 = self.sensor_manager.get_sensor(10)
            s11 = self.sensor_manager.get_sensor(11)
            if s10 and not s11:
                self.get_logger().info("✅ [S2-B] S10 ON + S11 OFF — khay đặt xong. INY về safe")
                self._cylinder_start_time = None
                self.clear_guide()
                self.state = SystemState.S2_LOAD_INY_RETURN_SAFE
            elif self.check_cylinder_timeout():
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_LOAD_INY_RETURN_SAFE:
            # INY về safe 50mm
            if self.move_servo(2, self.config.iny_safe_zone):
                self.get_logger().info("✅ [S2-B] INY về safe — InX về safe")
                self.state = SystemState.S2_LOAD_INX_RETURN_SAFE
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_LOAD_INX_RETURN_SAFE:
            # INX về safe (sau khi INY safe)
            if self.is_iny_safe_for_inx_move():
                if self.move_servo(1, self.config.inx_home):
                    self.get_logger().info("✅ [S2-B] INX về safe — Phase B HOÀN THÀNH")
                    self._s5_retry_count = 0
                    self.state = SystemState.STATE2_COMPLETE
                else:
                    self.state = SystemState.ERROR
            else:
                self.get_logger().warn("⏳ [S2-B] Chờ INY safe cho INX...")
        
        elif self.state == SystemState.STATE2_COMPLETE:
            # ✅ Pub new_tray_loaded với retry, chờ ACK từ robot
            if not self._tray_loaded_ack:
                self.publish_with_retry(self.pub_new_tray_loaded, Bool(data=True), '/revpi/new_tray_loaded')
                
                # Check is_last_tray (pub song song, không cần chờ ACK)
                if not self._state_completed_published:
                    if not self.check_has_trays_remaining():
                        self.pub_is_last_tray.publish(Bool(data=True))
                        self.get_logger().warn("🚨 S1+S2+S3 ALL OFF after State 2 → Published is_last_tray")
                    self._state_completed_published = True
            else:
                # ✅ Robot đã ACK → transition an toàn
                self.get_logger().info("🎉 STATE 2 COMPLETE - Robot ACK received, transitioning")
                self._tray_loaded_ack = False
                self._state_completed_published = False
                self.reset_pub_retry()
                self.state = SystemState.S2_WAIT_TRIGGER
        
        elif self.state == SystemState.S2_NO_TRAY:
            # S2_CHECK_REMAINING đã phát hiện hết khay
            # → không load khay mới, chỉ pub is_last_tray với retry
            if not self._last_tray_ack:
                self.publish_with_retry(self.pub_is_last_tray, Bool(data=True), '/revpi/is_last_tray')
            else:
                # ✅ Robot đã ACK is_last_tray → quay về hub chờ output tray cuối cùng
                self.get_logger().info("🏁 [S2] No trays left → Waiting for final output tray change")
                self._last_tray_ack = False
                self.has_trays_remaining = False
                self.reset_pub_retry()
                self.state = SystemState.S2_WAIT_TRIGGER
        
        # ══════════════════════════════════════════════════════════════
        # STATE 3: THAY KHAY OUTPUT (Position 2) - Chờ implement
        # ══════════════════════════════════════════════════════════════
        
        elif self.state == SystemState.S3_WAIT_TRIGGER:
            # Chờ 1 trong 2 trigger:
            # 1. change_tray_output_signal (khay output đầy)
            # 2. last_batch_complete (robot xong batch cuối)
            if self.change_tray_output_signal or self.last_batch_complete:
                if not self.robot_motion_busy:
                    reason = 'last_batch_complete' if self.last_batch_complete else 'output_tray_full'
                    self.get_logger().info(f"📥 [S3] {reason} + Robot IDLE → Starting State 3")
                    self.change_tray_output_signal = False
                    self.output_tray_full = False
                    self.pub_output_ready(False)  # Interlock robot
                    self.state = SystemState.S3_CHECK_SAFE
                else:
                    self.log_guide_once("S3_WAIT_ROBOT", "[S3] Trigger nhận nhưng Robot BUSY — chờ IDLE...")
        
        elif self.state == SystemState.S3_CHECK_SAFE:
            # Check OUTX, OUTY ở vị trí safe (Target 1)
            outy_safe = self.is_outy_safe_for_outx_move()
            if outy_safe:
                self.get_logger().info("✅ [S3] OUTX/OUTY safe → Starting output tray change")
                self.state = SystemState.S3_OUTX_TO_TARGET3
            else:
                self.get_logger().warn("⏳ [S3] Waiting OUTY safe...")
                self.move_servo(5, self.config.outy_home, wait=False)
        
        elif self.state == SystemState.S3_OUTX_TO_TARGET3:
            # OUTX → Target 3 (vị trí khay thành phẩm từ servo 3)
            if self.is_outy_safe_for_outx_move():
                if self.move_servo(4, self.config.outx_target3):
                    self.get_logger().info(f"✅ [S3] OUTX at target3 ({self.config.outx_target3}mm) - Robot tray position")
                    self.state = SystemState.S3_OUTY_TO_TARGET2
                else:
                    self.state = SystemState.ERROR
            else:
                self.get_logger().warn("⏳ [S3] Waiting OUTY safe for OUTX move to target3...")
        
        elif self.state == SystemState.S3_OUTY_TO_TARGET2:
            # OUTY → Target 2 (vị trí pick khay)
            if self.move_servo(5, self.config.outy_target2):
                self.get_logger().info(f"✅ [S3] OUTY at target2 ({self.config.outy_target2}mm) - Pick position")
                self.extend_cylinder2()
                self.start_cylinder_timer()
                self.clear_guide()
                self.state = SystemState.S3_CYLINDER2_EXTEND
                self.log_guide_once("S3_WAIT_S15", "[S3] Cylinder 2 đang extend (kẹp khay output) → Đợi S15 ON (Cyl2 MAX). Kích: '15:1'")
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S3_CYLINDER2_EXTEND:
            # Chờ S13 ON (Cylinder 2 MAX - extend = grip khay)
            if self.sensor_manager.get_sensor(15):  # S15 = Cyl2 MAX (extended)
                self.get_logger().info("✅ [S3] Cylinder 2 extended (S15 ON) - Tray gripped")
                self._cylinder_start_time = None
                self.clear_guide()
                self.state = SystemState.S3_OUTY_TO_TARGET1
            elif self.check_cylinder_timeout():
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S3_OUTY_TO_TARGET1:
            # OUTY → Target 1 (safe - nâng khay lên)
            if self.move_servo(5, self.config.outy_target1):
                self.get_logger().info("✅ [S3] OUTY at target1 (safe) - Tray lifted")
                self.state = SystemState.S3_OUTX_TO_TARGET2
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S3_OUTX_TO_TARGET2:
            # OUTX → Target 2 (vị trí output stack - đặt chồng khay)
            if self.is_outy_safe_for_outx_move():
                if self.move_servo(4, self.config.outx_target2):
                    self.get_logger().info(f"✅ [S3] OUTX at target2 ({self.config.outx_target2}mm) - Output stack")
                    self.state = SystemState.S3_OUTY_SEARCH_ROW
                else:
                    self.state = SystemState.ERROR
            else:
                self.get_logger().warn("⏳ [S3] Waiting OUTY safe for OUTX move to output stack...")
        
        elif self.state == SystemState.S3_OUTY_SEARCH_ROW:
            # ✅ MANUAL MODE: Nếu đã set target row, bỏ qua sensor search
            if self.manual_mode and self.manual_target_row is not None:
                self._output_stack_row = self.manual_target_row
                self.get_logger().info(f"🎯 MANUAL [S3]: Row set to {self.manual_target_row} (skipping search)")
                self.manual_target_row = None  # Clear after use
                self.state = SystemState.S3_OUTY_TO_ROW
            # OUTY hạ xuống, chờ S9 ON (phát hiện chồng khay output)
            elif not self.sensor_manager.get_sensor(9):  # S9 OFF
                outy_pos, pos_ok = self.get_position_safe(5)
                if outy_pos is None:
                    if not pos_ok:
                        self.state = SystemState.ERROR
                    return
                row1_limit = self.config.outy_output_table.get(1, 500.0)  # S3: search output table
                if outy_pos >= row1_limit:
                    # Đạt row 1 limit, S9 OFF → stack rỗng, dùng row 1
                    self.stop_outy_jog()
                    self._output_stack_row = 1
                    self.get_logger().warn(f"⚠️ [S3] OUTY reached limit ({row1_limit}mm), S9 OFF → Using row 1")
                    self.state = SystemState.S3_OUTY_TO_ROW
                else:
                    self.start_outy_jog(self.config.outy_search_velocity)
            else:  # S9 ON → phát hiện stack
                self.stop_outy_jog()
                current_pos = self.get_servo_position(5)
                if current_pos is not None:
                    self._output_stack_row = self.find_nearest_row(current_pos, self.config.outy_output_table)
                    self.get_logger().info(f"📍 [S3] S9 ON at {current_pos:.1f}mm → Output stack row {self._output_stack_row}")
                    self.state = SystemState.S3_OUTY_TO_ROW
        
        elif self.state == SystemState.S3_OUTY_TO_ROW:
            # Di chuyển đến vị trí row chính xác
            target_pos = self.config.outy_output_table.get(self._output_stack_row, self.config.outy_output_table[1])
            if self.move_servo(5, target_pos):
                self.get_logger().info(f"✅ [S3] OUTY at row {self._output_stack_row} ({target_pos}mm)")
                self.retract_cylinder2()
                self.start_cylinder_timer()
                self.clear_guide()
                self.state = SystemState.S3_CYLINDER2_RETRACT
                self.log_guide_once("S3_WAIT_S14", "[S3] Cylinder 2 đang retract (thả khay) → Đợi S14 ON (Cyl2 MIN). Kích: '14:1'")
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S3_CYLINDER2_RETRACT:
            # Chờ S12 ON (Cylinder 2 MIN - retract = thả khay)
            if self.sensor_manager.get_sensor(14):  # S14 = Cyl2 MIN (retracted)
                self.get_logger().info("✅ [S3] Cylinder 2 retracted (S14 ON) - Tray released to output stack")
                self._cylinder_start_time = None
                self.clear_guide()
                self.state = SystemState.S3_OUTY_RETURN_HOME
            elif self.check_cylinder_timeout():
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S3_OUTY_RETURN_HOME:
            # OUTY → Target 1 (safe)
            if self.move_servo(5, self.config.outy_target1):
                self.get_logger().info("✅ [S3] OUTY returned to safe position")
                self.state = SystemState.S3_OUTX_RETURN_SAFE
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S3_OUTX_RETURN_SAFE:
            # OUTX → Target 1 (safe)
            if self.is_outy_safe_for_outx_move():
                if self.move_servo(4, self.config.outx_target1):
                    if self.last_batch_complete:
                        # ✅ Last batch: skip S7 check, servo 3 về target1 → IDLE
                        self.get_logger().info("[S3] OUTX safe + last_batch_complete → Servo3 to target1 (skip S7)")
                        self.state = SystemState.S3_SERVO3_TO_TARGET1
                    else:
                        self.get_logger().info("[S3] OUTX returned to safe - Now moving servo 3")
                        self.state = SystemState.S3_SERVO3_MOVE_TO_S6
                else:
                    self.state = SystemState.ERROR
            else:
                self.get_logger().warn("⏳ [S3] Waiting OUTY safe for OUTX return...")
        
        elif self.state == SystemState.S3_SERVO3_MOVE_TO_S6:
            # Servo 3 jog positive → S6 ON (đẩy platform nâng khay mới lên vị trí robot)
            if self.sensor_manager.get_sensor(6):  # S6 ON
                self.stop_servo3_jog()
                self.get_logger().info("✅ [S3] S6 ON - Platform đã đưa khay lên vị trí robot")
                self.clear_guide()
                self.state = SystemState.S3_SERVO3_CHECK_S7
            else:
                # ✅ SAFETY: Check position limit trước khi jog tiếp
                s3_pos, pos_ok = self.get_position_safe(3)
                if s3_pos is not None and s3_pos >= self.config.servo_limits.get(3, 400.0):
                    self.stop_servo3_jog()
                    self.get_logger().error(f"🚨 [S3] Servo 3 reached limit {s3_pos:.1f}mm without S6! STOPPING.")
                    self.state = SystemState.ERROR
                elif s3_pos is None and not pos_ok:
                    self.state = SystemState.ERROR
                else:
                    self.start_servo3_jog(-self.config.servo3_jog_velocity)  # âm = jog_positive
                    self.log_guide_once("S3_WAIT_S6", "[S3] Servo 3 đang jog → Đợi S6 ON (sensor platform). Kích: '6:1'")
        
        elif self.state == SystemState.S3_SERVO3_CHECK_S7:
            # Check S7: còn khay trên platform?
            if self.sensor_manager.get_sensor(12):  # S12 ON (CYL3 hold tray — đứng việc, còn khay)
                self.get_logger().info("✅ [S3] S12 ON - Còn khay output")
                self.clear_guide()
                self.pub_output_ready(True)
                self.state = SystemState.STATE3_COMPLETE
            else:  # S7 OFF → hết khay, cần cấp mới
                self.get_logger().warn("⚠️ [S3] S7 OFF - Hết khay output → Servo 3 về Target 1, chờ cấp")
                self.state = SystemState.S3_SERVO3_TO_TARGET1
        
        elif self.state == SystemState.S3_SERVO3_TO_TARGET1:
            # Servo 3 → Target 1 (vị trí chờ load khay)
            if self.move_servo(3, self.config.servo3_target1):
                if self.last_batch_complete:
                    # ✅ Last batch: servo3 at target1, skip wait → IDLE
                    self.get_logger().info("[S3] Servo3 at target1 + last_batch → STATE3_COMPLETE (skip wait)")
                    self.state = SystemState.STATE3_COMPLETE
                else:
                    self.get_logger().info("[S3] Servo 3 at target1 (wait position)")
                    self.pub_output_ready(False)  # Interlock robot
                    self._confirm_load_received = False
                    self.state = SystemState.S3_SERVO3_WAIT_LOAD
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S3_SERVO3_WAIT_LOAD:
            # Chờ S7 ON + HMI confirm. KHÔNG timeout, chỉ warning
            if self.sensor_manager.get_sensor(12):  # S12 ON (CYL3 — xem sensor 12)
                if self._confirm_load_received:
                    self.get_logger().info("✅ [S3] S7 ON + HMI confirmed → Moving servo 3 to S6")
                    self.clear_guide()
                    self._confirm_load_received = False
                    # Servo 3 jog+ đến S6 ON rồi complete
                    self.state = SystemState.S3_SERVO3_MOVE_TO_S6
                else:
                    self.get_logger().warn("⏳ [S3] S7 ON - Chờ HMI confirm (/providesystem/confirm_output_load)")
            else:
                self.log_guide_once("S3_WAIT_S7_LOAD", "[S3] Chờ S7 ON (cấp khay output mới) + HMI confirm. Kích: '7:1'")
                self.get_logger().warn("⏳ [S3] S7 OFF - Chờ cấp khay output mới...")
        
        elif self.state == SystemState.STATE3_COMPLETE:
            if self.last_batch_complete:
                # ✅ PROCESS COMPLETE: Robot đã về HOME và báo xong
                self.get_logger().info("[STATE3] PROCESS COMPLETE - last_batch_complete → IDLE")
                self.get_logger().info("[STATE3] Nhấn Start để bắt đầu chu kỳ mới từ State 1")
                self.last_batch_complete = False
                self.last_batch_mode = False
                self.has_trays_remaining = True  # Reset for next cycle
                self.state = SystemState.IDLE
            else:
                # Tiếp tục process → quay về Central Hub chờ trigger tiếp
                self.get_logger().info("[STATE3] COMPLETE - Output tray changed → S2_WAIT_TRIGGER")
                self.state = SystemState.S2_WAIT_TRIGGER
        
        elif self.state == SystemState.S3_WAIT_HMI_RESUME:
            # Chờ HMI confirm → chỉ chạy Servo 3 nạp khay output mới (KHÔNG homing lại)
            if self._hmi_resume_confirmed:
                self.get_logger().info("✅ HMI RESUME → Chạy Servo 3 nạp khay output mới")
                self._hmi_resume_confirmed = False
                self.state = SystemState.S3_SERVO3_MOVE_TO_S6  # Chỉ servo 3 push tray, không homing
            else:
                self.get_logger().warn("⏳ [S3] Chờ nạp khay output mới + HMI confirm (/providesystem/hmi_resume)")
        
        # ══════════════════════════════════════════════════════════════
        # ERROR
        # ══════════════════════════════════════════════════════════════
        elif self.state == SystemState.ERROR:
            self.log_guide_once("ERROR_STATE", "SYSTEM ERROR - Check and reset")
        
        # MANUAL MODE: Reset step_pending sau khi execute state
        # (Skip reset if manual_auto_run — goto_state runs continuously)
        if self.manual_mode and not self.manual_auto_run and self.state != state_before:
            self.step_pending = False  # Pause for next step
        
        # Notify GUI on state change
        if self.state != state_before:
            self._notify_gui('info', f"State: {state_before.value} -> {self.state.value}")


def main(args=None):
    rclpy.init(args=args)
    system = None
    
    def _handle_signal(signum, frame):
        print(f"\n  Node received signal {signum} — force exit...")
        try:
            rclpy.shutdown()
        except:
            pass
        os._exit(0)
    
    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    # ✅ Catch ConnectionAbortedError từ cpx_io background IO thread
    # Không để thread crash làm chết cả process — reset io_module chờ reconnect
    _node_ref = [None]  # mutable ref để lambda có thể truy cập system node
    def _thread_exception_handler(args):
        exc = args.exc_value
        tname = args.thread.name if args.thread else '?'
        # Chỉ suppress lỗi kết nối từ cpx_io thread
        if isinstance(exc, (ConnectionAbortedError, ConnectionError, OSError)):
            print(f"[IO-THREAD] Connection error in '{tname}' — resetting IO module (will reconnect): {exc}")
            node = _node_ref[0]
            if node is not None:
                try:
                    node.io_module = None  # update_sensors() sẽ tự reconnect sau 10s
                    node._io_read_fail_count = 0
                except Exception:
                    pass
        else:
            # Lỗi khác: in ra rồi để chấy tiếp (không crash process)
            import traceback
            print(f"[THREAD-CRASH] Thread '{tname}': {exc}")
            traceback.print_exception(args.exc_type, args.exc_value, args.exc_tb)
    threading.excepthook = _thread_exception_handler
    
    try:
        config = CartridgeConfig()
        # Auto-load YAML config if exists
        config_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
        config_file = os.path.join(config_dir, 'cartridge_config.yaml')
        if not os.path.exists(config_file):
            # Try installed location
            config_file = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'system_feed_cartridge', 'config', 'cartridge_config.yaml')
        if os.path.exists(config_file):
            config.load_from_file(config_file)
        else:
            print(f"⚠️  Config file not found, using defaults")
        system = CartridgeSystem(config)
        _node_ref[0] = system  # Cho thread exception handler truy cập node
        rclpy.spin(system)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Fatal error: {e}")
    finally:
        try:
            rclpy.shutdown()
        except:
            pass
        os._exit(0)


if __name__ == '__main__':
    main()