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
Units: encoder counts (1 mm = 100,000 counts)
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

# Unit conversion constants
COUNTS_PER_MM = 100000  # 1 mm = 100,000 encoder counts
DEFAULT_VELOCITY = 30000  # ~300 mm/s


class SystemState(Enum):
    """State machine states"""
    IDLE = "idle"
    ERROR = "error"
    
    # ========== STATE 1: Khởi Tạo + Cấp Khay Input ==========
    # Position 1 + Position 2 chạy song song
    HOMING = "homing"
    
    # Position 1: InX, InY, Cylinder 1
    S1_INY_CONFIRM_SAFE = "s1_iny_confirm_safe"
    S1_INX_TO_CONVEYOR_END = "s1_inx_to_conveyor_end"
    S1_INY_SEARCH_TRAY = "s1_iny_search_tray"
    S1_INY_TO_NEAREST_ROW = "s1_iny_to_nearest_row"
    S1_CYLINDER1_EXTEND = "s1_cylinder1_extend"
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
    S2_LOAD_INX_TO_TARGET2 = "s2_load_inx_to_target2"
    S2_LOAD_INY_SEARCH_TRAY = "s2_load_iny_search_tray"
    S2_LOAD_INY_TO_ROW = "s2_load_iny_to_row"
    S2_LOAD_CYLINDER1_EXTEND = "s2_load_cylinder1_extend"
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
            1: "192.168.0.101",  # InX
            2: "192.168.0.102",  # InY
            3: "192.168.0.103",  # Put Tray
            4: "192.168.0.104",  # OutX
            5: "192.168.0.105",  # OutY
        }
        
        # IO Module
        self.io_ip = "192.168.0.200"
        
        # Cylinder channels
        self.cylinder1_extend_channel = 5
        self.cylinder1_retract_channel = 4
        self.cylinder2_extend_channel = 7
        self.cylinder2_retract_channel = 6
        
        # InX positions (mm)
        self.inx_home = 0.0              # Home/safe position
        self.inx_target2 = 500.0         # Input stack position (cuối băng tải)
        self.inx_output_stack = 100.0    # Output stack position (Position 1 - đặt khay đã dùng)
        
        # InY positions (mm)
        self.iny_home = 0.0              # Home position (top)
        self.iny_safe_zone = 50.0        # Safe zone threshold - INX chỉ được di chuyển khi INY < giá trị này
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
        self.servo3_target1 = 50.0          # Safe/wait position (chờ load khay)
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
        self.iny_search_velocity = 30.0  # InY search speed (normal)
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
        self.homing_timeout = 30.0
        self.move_timeout = 20.0
        self.cylinder_timeout = 5.0
        
        if config_file:
            self.load_from_file(config_file)
    
    def load_from_file(self, config_file: str):
        """Load configuration from YAML file"""
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                # Update attributes from config
                for key, value in config.items():
                    if hasattr(self, key):
                        setattr(self, key, value)
        except Exception as e:
            print(f"Warning: Failed to load config file: {e}")


class CartridgeSystem(Node):
    """Main cartridge loading system (ROS 2 + festo-edcon MotionHandler)"""
    
    def __init__(self, config: CartridgeConfig):
        super().__init__('cartridge_providesystem')
        
        self.config = config
        self.state = SystemState.IDLE
        self.sensor_manager = SensorManager()
        self.prev_sensor_state = {}
        
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
        self.change_tray_output_signal = False  # Trigger thay khay output
        self._output_stack_row = 0           # Row index chồng khay output (Position 2)
        
        # ✅ Manual testing mode
        self.manual_mode = False      # Manual testing mode (step-by-step)
        self.step_pending = True      # Allow next step execution
        self._last_state = None       # Track state changes
        self.manual_target_row = None # Manual row selection (1-8, None=auto)
        self._position_read_fail_count = {}  # Track consecutive read failures per servo
        
        # Servo connections
        self.servos = {}
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
        
        # ROS 2 Subscribers
        self.create_subscription(Bool, '/system/start_button', self.start_button_callback, qos_default)
        self.create_subscription(Bool, '/robot/change_tray', self.change_tray_input_callback, qos_default)
        self.create_subscription(Bool, '/vision/change_tray_input', self.change_tray_input_callback, qos_default)  # ✅ AI Mode: Vision detect empty input
        self.create_subscription(Bool, '/robot/motion_busy', self.motion_busy_callback, qos_default)
        self.create_subscription(Bool, '/vision/change_tray_output', self.output_tray_full_callback, qos_default)
        self.create_subscription(Bool, '/robot/done_tray_output', self.output_tray_full_callback, qos_default)
        self.create_subscription(Bool, '/robot/tray_loaded_ack', self.tray_loaded_ack_callback, qos_default)
        self.create_subscription(Bool, '/robot/last_tray_ack', self.last_tray_ack_callback, qos_default)
        self.create_subscription(String, '/providesystem/goto_state', self.goto_state_topic_callback, qos_default)
        self.create_subscription(String, '/providesystem/set_target_row', self.set_target_row_topic_callback, qos_default)
        self.create_subscription(String, '/providesystem/servo_limit_cmd', self.servo_limit_cmd_callback, qos_default)
        self.create_subscription(Bool, '/providesystem/hmi_resume', self.hmi_resume_callback, qos_default)
        
        # ROS 2 Service Servers
        self.create_service(SetBool, '/providesystem/confirm_output_load', self.confirm_output_load_callback)
        self.create_service(SetBool, '/providesystem/set_manual_mode', self.set_manual_mode_callback)
        self.create_service(Trigger, '/providesystem/next_step', self.next_step_callback)
        self.create_service(SetBool, '/providesystem/set_servo_limit', self.set_servo_limit_callback)
        
        # Initialize hardware
        self.connect_hardware()
        
        # Main control loop timer (50 Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop_callback)
        
        self.get_logger().info("Cartridge System Initialized (ROS 2 + festo-edcon MotionHandler)")
    
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
            
            # Connect IO module
            if CPXAP_AVAILABLE:
                try:
                    self.io_module = CpxAp(self.config.io_ip)
                    self.get_logger().info(f"✅ Connected to IO Module at {self.config.io_ip}")
                except Exception as e:
                    self.get_logger().error(f"❌ Failed to connect IO Module: {e}")
            
        except Exception as e:
            self.get_logger().error(f"Hardware connection error: {e}")
    
    def destroy_node(self):
        """Cleanup: close all MotionHandler connections before destroying node"""
        self.get_logger().info("Shutting down - closing servo connections...")
        for servo_id, mot in self.servos.items():
            try:
                mot.shutdown()
                self.get_logger().info(f"Servo {servo_id} connection closed")
            except Exception as e:
                self.get_logger().warn(f"Error closing Servo {servo_id}: {e}")
        super().destroy_node()
    
    def start_button_callback(self, msg: Bool):
        """Handle /system/start_button"""
        if msg.data and self.state == SystemState.IDLE:
            self.get_logger().info("▶️ Start button pressed - Beginning homing sequence")
            self.state = SystemState.HOMING
    
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
    
    def home_all_servos(self) -> bool:
        """Home all servo motors using MotionHandler.referencing_task()"""
        self.get_logger().info("Homing all servos...")
        
        try:
            # Enable powerstage + start homing for each servo (with retry)
            for servo_id, mot in self.servos.items():
                try:
                    safe_try(
                        lambda m=mot: m.acknowledge_faults(),
                        retries=3, backoff=2, logger=self.get_logger()
                    )
                    safe_try(
                        lambda m=mot: m.enable_powerstage(),
                        retries=3, backoff=2, logger=self.get_logger()
                    )
                    if not mot.referenced():
                        safe_try(
                            lambda m=mot: m.referencing_task(nonblocking=True),
                            retries=3, backoff=2, logger=self.get_logger()
                        )
                    else:
                        self.get_logger().info(f"Servo {servo_id} already referenced, skipping")
                    self.get_logger().info(f"Servo {servo_id} homing initiated")
                except ConnectionError as e:
                    self.get_logger().error(f"Servo {servo_id} homing failed after retries: {e}")
                    return False
                except Exception as e:
                    self.get_logger().error(f"Servo {servo_id} homing start error: {e}")
                    return False
            
            # Wait for all to complete
            start_time = time.time()
            while time.time() - start_time < self.config.homing_timeout:
                all_homed = True
                for servo_id, mot in self.servos.items():
                    if not mot.referenced():
                        all_homed = False
                        break
                
                if all_homed:
                    self.get_logger().info("All servos homed successfully")
                    return True
                
                time.sleep(0.1)
            
            self.get_logger().error("Homing timeout!")
            return False
            
        except Exception as e:
            self.get_logger().error(f"Homing error: {e}")
            return False
    
    def move_servo(self, servo_id: int, position: float, wait: bool = True) -> bool:
        """Move servo to position (mm) with soft limit enforcement.
        Converts mm to encoder counts internally."""
        # SOFT LIMIT CHECK
        limit = self.config.servo_limits.get(servo_id, 999999.0)
        if position > limit:
            self.get_logger().error(f"🚨 Servo {servo_id}: target {position:.1f}mm EXCEEDS limit {limit:.1f}mm! BLOCKED.")
            return False
        
        if servo_id not in self.servos:
            self.get_logger().warn(f"Servo {servo_id} not available (simulation mode)")
            return True
        
        try:
            mot = self.servos[servo_id]
            pos_counts = int(position * COUNTS_PER_MM)
            safe_try(
                lambda: mot.position_task(pos_counts, DEFAULT_VELOCITY, absolute=True, nonblocking=True),
                retries=3, backoff=1, logger=self.get_logger()
            )
            
            if wait:
                start_time = time.time()
                while time.time() - start_time < self.config.move_timeout:
                    if mot.target_position_reached():
                        return True
                    time.sleep(0.05)
                
                self.get_logger().error(f"Servo {servo_id} move timeout")
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Servo {servo_id} move error: {e}")
            return False
    
    def jog_servo(self, servo_id: int, velocity: float):
        """Jog servo using jog_task(). velocity>0 = positive, <0 = negative."""
        # SOFT LIMIT CHECK during jog
        limit = self.config.servo_limits.get(servo_id, 999999.0)
        current_pos = self.get_servo_position(servo_id)
        
        if current_pos is not None and current_pos >= limit and velocity > 0:
            self.get_logger().error(f"🚨 Servo {servo_id}: position {current_pos:.1f}mm AT LIMIT {limit:.1f}mm! Jog BLOCKED.")
            self.stop_servo(servo_id)
            return
        
        if servo_id not in self.servos:
            return
        
        try:
            mot = self.servos[servo_id]
            mot.jog_task(jog_positive=(velocity > 0), jog_negative=(velocity < 0), duration=0)
        except Exception as e:
            self.get_logger().error(f"Servo {servo_id} jog error: {e}")
    
    def stop_servo(self, servo_id: int):
        """Stop servo immediately using stop_motion_task()"""
        if servo_id not in self.servos:
            return
        
        try:
            mot = self.servos[servo_id]
            mot.stop_motion_task()
        except Exception as e:
            self.get_logger().error(f"Servo {servo_id} stop error: {e}")
    
    def get_servo_position(self, servo_id: int) -> Optional[float]:
        """Get current servo position in mm (converts from encoder counts)"""
        if servo_id not in self.servos:
            return 0.0  # Simulation
        
        try:
            counts = self.servos[servo_id].current_position()
            return counts / COUNTS_PER_MM  # Convert to mm
        except Exception as e:
            self.get_logger().error(f"Failed to get position from Servo {servo_id}: {e}")
            return None
    
    
    def find_nearest_row(self, current_pos: float, row_positions_dict: dict) -> int:
        """Find nearest row position from given stack/table
        
        Args:
            current_pos: Current servo position (mm)
            row_positions_dict: Dict of row positions (e.g., iny_input_stack, outy_output_table)
        
        Returns:
            Nearest row number
        """
        min_distance = float('inf')
        nearest_row = 1
        
        for row, pos in row_positions_dict.items():
            distance = abs(current_pos - pos)
            if distance < min_distance:
                min_distance = distance
                nearest_row = row
        
        # Check if within tolerance
        if min_distance > self.config.position_tolerance:
            self.get_logger().warn(f"Position {current_pos} outside tolerance from nearest row")
        
        return nearest_row
    
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
        """Check if cylinder operation has timed out.
        Returns True if TIMED OUT (error condition).
        """
        if self._cylinder_start_time is None:
            self.start_cylinder_timer()
            return False
        
        elapsed = time.time() - self._cylinder_start_time
        if elapsed > self.config.cylinder_timeout:
            self.get_logger().error(f"🚨 Cylinder TIMEOUT after {elapsed:.1f}s (limit: {self.config.cylinder_timeout}s)")
            self._cylinder_start_time = None
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
    
    def confirm_output_load_callback(self, req):
        """Service /providesystem/confirm_output_load - HMI xác nhận đã cấp khay output mới"""
        if req.data:
            self._confirm_load_received = True
            self.get_logger().info("✅ HMI confirmed: Output tray loaded")
            response = SetBool.Response()
            response.success = True
            response.message = "Output load confirmed"
            return response
        response = SetBool.Response()
        response.success = False
        response.message = "Invalid request"
        return response
    
    def hmi_resume_callback(self, msg):
        """Topic /providesystem/hmi_resume - HMI xác nhận tiếp tục chu kỳ mới"""
        if msg.data:
            self._hmi_resume_confirmed = True
            self.get_logger().info("✅ HMI RESUME: Operator confirmed → Resuming cycle")
    
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
    
    def set_manual_mode_callback(self, req):
        """Service /providesystem/set_manual_mode - Enable/disable manual testing"""
        self.manual_mode = req.data
        if req.data:
            self.get_logger().warn("🔧 MANUAL MODE ENABLED")
            self.get_logger().warn("Use /providesystem/goto_state to jump to state")
            self.get_logger().warn("Use /providesystem/next_step to execute one step")
            self.get_logger().warn("State machine is PAUSED")
            self.step_pending = False  # Start paused
            response = SetBool.Response()
            response.success = True
            response.message = "Manual mode enabled"
            return response
        else:
            self.get_logger().info("✅ MANUAL MODE DISABLED - Auto execution")
            self.step_pending = True  # Resume auto
            response = SetBool.Response()
            response.success = True
            response.message = "Manual mode disabled"
            return response
    
    def goto_state_topic_callback(self, msg):
        """Topic /providesystem/goto_state - Jump to specific state (String)"""
        if not self.manual_mode:
            self.get_logger().warn("❌ goto_state: Manual mode not enabled")
            return
        
        state_name = msg.data.strip()
        
        # Find matching state from enum
        target_state = None
        for state in SystemState:
            if state.value == state_name or state.name == state_name:
                target_state = state
                break
        
        if target_state is None:
            available = ", ".join([s.name for s in SystemState][:10]) + "..."
            self.get_logger().error(f"❌ Invalid state '{state_name}'. Available: {available}")
            return
        
        self.state = target_state
        self.step_pending = False  # Paused at new state
        self.get_logger().info(f"🎯 Jumped to {target_state.name}")
    
    def next_step_callback(self, req):
        """Service /providesystem/next_step - Execute one state transition"""
        if not self.manual_mode:
            response = Trigger.Response()
            response.success = False
            response.message = "Manual mode not enabled"
            return response
        
        old_state = self.state
        self.step_pending = True  # Allow one step
        
        # Wait for state to execute (up to 0.5s)
        for _ in range(5):
            time.sleep(0.1)
            if self.state != old_state or not self.step_pending:
                break
        
        response = Trigger.Response()
        if self.state == old_state:
            msg = f"⏳ State unchanged: {old_state.name} (may be waiting for sensor/interlock)"
            self.get_logger().warn(msg)
            response.success = True
            response.message = msg
        else:
            msg = f"✅ Step: {old_state.name} → {self.state.name}"
            self.get_logger().info(msg)
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
    
    def set_servo_limit_callback(self, req):
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
        response = SetBool.Response()
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
    
    def get_position_safe(self, servo_id: int) -> tuple:
        """Get servo position with safety check.
        Returns (position, is_valid). If invalid 3 times → should ERROR.
        """
        pos = self.get_servo_position(servo_id)
        if pos is None:
            count = self._position_read_fail_count.get(servo_id, 0) + 1
            self._position_read_fail_count[servo_id] = count
            if count >= 3:
                self.get_logger().error(f"🚨 Servo {servo_id}: Position read FAILED {count}x! STOPPING.")
                self.stop_servo(servo_id)
                return (None, False)
            else:
                self.get_logger().warn(f"⚠️ Servo {servo_id}: Position read failed ({count}/3)")
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
            self.io_module.reset_channel(self.config.cylinder1_retract_channel)
            self.io_module.set_channel(self.config.cylinder1_extend_channel)
        self.get_logger().info("Cylinder 1 extending")
    
    def retract_cylinder1(self):
        """Retract cylinder 1"""
        if self.io_module:
            self.io_module.reset_channel(self.config.cylinder1_extend_channel)
            self.io_module.set_channel(self.config.cylinder1_retract_channel)
        self.get_logger().info("Cylinder 1 retracting")
    
    def extend_cylinder2(self):
        """Extend cylinder 2"""
        if self.io_module:
            self.io_module.reset_channel(self.config.cylinder2_retract_channel)
            self.io_module.set_channel(self.config.cylinder2_extend_channel)
        self.get_logger().info("Cylinder 2 extending")
    
    def retract_cylinder2(self):
        """Retract cylinder 2"""
        if self.io_module:
            self.io_module.reset_channel(self.config.cylinder2_extend_channel)
            self.io_module.set_channel(self.config.cylinder2_retract_channel)
        self.get_logger().info("Cylinder 2 retracting")
    
    def publish_state(self):
        """Publish current state"""
        msg = String()
        msg.data = self.state.value
        self.pub_state.publish(msg)
    
    def control_loop_callback(self):
        """Timer callback for main control loop (50 Hz)"""
        # Save previous sensor state for edge detection
        self.prev_sensor_state = self.sensor_manager.sensors.copy()
        
        # Update sensors (in real system, read from hardware)
        self.update_sensors()
        
        # State machine
        self.process_state()
        
        # ✅ PARALLEL: Process Position 2 (Servo 3) độc lập
        self.process_pos2_state()
        
        # Publish state
        self.publish_state()
    
    def update_sensors(self):
        """Update sensor readings from hardware"""
        if self.io_module is None:
            return  # Simulation mode
        
        try:
            # Read all digital inputs from Festo IO
            # Assuming sensors are connected to input channels 0-15
            for sensor_id in range(1, 16):  # Support up to 15 sensors
                channel_state = self.io_module.get_channel(sensor_id - 1)  # 0-indexed
                self.sensor_manager.update_sensor(sensor_id, channel_state)
                
        except Exception as e:
            self.get_logger().error(f"Failed to read sensors: {e}")
    
    def process_pos2_state(self):
        """Process Position 2 (Servo 3) state machine - Chạy SONG SONG với Position 1"""
        if self._pos2_state == "IDLE" or self._pos2_state == "DONE":
            return  # Không cần xử lý
        
        # ✅ MANUAL MODE: Pause theo manual mode chung
        if self.manual_mode and not self.step_pending:
            return
        
        if self._pos2_state == "CHECK_S7":
            # Servo 3 → Target 1 (safe) rồi check S7
            if self.move_servo(3, self.config.servo3_target1):
                if self.sensor_manager.get_sensor(7):  # S7 ON → còn khay
                    self.get_logger().info("✅ [P2] S7 ON - Khay output có sẵn → Moving servo 3 to S6")
                    self._pos2_state = "MOVE_TO_S6"
                else:  # S7 OFF → chưa cấp khay
                    self.get_logger().warn("⚠️ [P2] S7 OFF - Chưa có khay output → Waiting...")
                    self.pub_output_ready(False)
                    self._pos2_state = "WAIT_LOAD"
        
        elif self._pos2_state == "WAIT_LOAD":
            # Chờ S7 ON + confirm từ HMI
            if self.sensor_manager.get_sensor(7):  # S7 ON
                if self._confirm_load_received:
                    self.get_logger().info("✅ [P2] S7 ON + HMI confirmed → Moving servo 3 to S6")
                    self._confirm_load_received = False
                    self._pos2_state = "MOVE_TO_S6"
                else:
                    self.get_logger().warn("⏳ [P2] S7 ON - Chờ HMI confirm (/providesystem/confirm_output_load)")
            else:
                self.get_logger().warn("⏳ [P2] S7 OFF - Chờ cấp khay output mới...")
        
        elif self._pos2_state == "MOVE_TO_S6":
            # Servo 3 jog positive → S6 ON
            if self.sensor_manager.get_sensor(6):  # S6 ON
                self.stop_servo3_jog()
                self.get_logger().info("✅ [P2] S6 ON - Khay output sẵn sàng cho robot")
                self.pub_output_ready(True)
                self._servo3_init_done = True
                self._pos2_done = True
                self._pos2_state = "DONE"
                # ✅ Nếu Position 1 đã xong → chuyển STATE1_COMPLETE
                if self._pos1_done:
                    self.get_logger().info("🎉 Cả 2 Position xong → STATE1_COMPLETE")
                    self.state = SystemState.STATE1_COMPLETE
                else:
                    self.get_logger().info("⏳ Position 2 done, chờ Position 1 (InX/InY)...")
            else:
                # ✅ SAFETY: Check position limit
                s3_pos, pos_ok = self.get_position_safe(3)
                if s3_pos is not None and s3_pos >= self.config.servo_limits.get(3, 400.0):
                    self.stop_servo3_jog()
                    self.get_logger().error(f"🚨 [P2] Servo 3 reached limit {s3_pos:.1f}mm without S6!")
                    self._pos2_state = "IDLE"
                    self.state = SystemState.ERROR
                elif s3_pos is None and not pos_ok:
                    self._pos2_state = "IDLE"
                    self.state = SystemState.ERROR
                else:
                    self.start_servo3_jog(self.config.servo3_jog_velocity)
    
    # ══════════════════════════════════════════════════════════════════════
    # process_state() continuation — Main State Machine
    # ══════════════════════════════════════════════════════════════════════
    
    def process_state(self):
        """Process current state - Main state machine (Position 1 + States 2/3)"""
        
        # ✅ MANUAL MODE: Pause execution cho đến khi next_step được gọi
        if self.manual_mode and not self.step_pending:
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
            if self.home_all_servos():
                self.get_logger().info("✅ Homing complete - Starting Position 1 + Position 2 IN PARALLEL")
                # ✅ PARALLEL: Khởi động cả 2 track đồng thời
                self._pos1_done = False
                self._pos2_done = False
                self._pos2_state = "CHECK_S7"  # Position 2: Servo 3 bắt đầu
                self.state = SystemState.S1_INY_CONFIRM_SAFE  # Position 1: InX/InY bắt đầu
            else:
                self.state = SystemState.ERROR
        
        # ══════════════════════════════════════════════════════════════
        # STATE 1: CẤP KHAY INPUT VÀO VỊ TRÍ ROBOT (Position 1: InX, InY)
        # ══════════════════════════════════════════════════════════════
        
        elif self.state == SystemState.S1_INY_CONFIRM_SAFE:
            # Xác nhận InY đã homing xong và ở vị trí safe
            iny_pos = self.get_servo_position(2)
            if iny_pos is not None and iny_pos <= self.config.iny_safe_zone:
                self.get_logger().info(f"✅ InY safe at {iny_pos:.1f}mm (< {self.config.iny_safe_zone}mm) - Ready for InX")
                self.state = SystemState.S1_INX_TO_CONVEYOR_END
            else:
                self.get_logger().warn(f"⏳ InY at {iny_pos}mm - Moving to safe zone...")
                self.move_servo(2, self.config.iny_home, wait=False)
        
        elif self.state == SystemState.S1_INX_TO_CONVEYOR_END:
            # InX di chuyển đến cuối băng tải (CHỈ KHI InY safe)
            if self.is_iny_safe_for_inx_move():
                if self.move_servo(1, self.config.inx_target2):
                    self.get_logger().info("✅ InX at conveyor end")
                    self.state = SystemState.S1_INY_SEARCH_TRAY
                else:
                    self.state = SystemState.ERROR
            else:
                self.get_logger().warn("⏳ Waiting InY safe for InX move...")
        
        elif self.state == SystemState.S1_INY_SEARCH_TRAY:
            # ✅ MANUAL MODE: Nếu đã set target row, bỏ qua sensor search
            if self.manual_mode and self.manual_target_row is not None:
                self.current_row = self.manual_target_row
                self.stack_row_index = self.manual_target_row
                self.get_logger().info(f"🎯 MANUAL: Row set to {self.manual_target_row} (skipping search)")
                self.manual_target_row = None  # Clear after use
                self.state = SystemState.S1_INY_TO_NEAREST_ROW
            # InY di chuyển xuống, dựa vào S4 (Detect stack Traycustom)
            elif not self.sensor_manager.get_sensor(4):  # S4 OFF - chưa phát hiện khay
                iny_pos, pos_ok = self.get_position_safe(2)
                if iny_pos is None:
                    if not pos_ok:
                        self.state = SystemState.ERROR
                    return  # Wait for next cycle
                row1_limit = self.config.iny_input_stack.get(1, 600.0)  # Row 1 = bottom
                if iny_pos >= row1_limit:
                    # Đạt row 1 limit, S4 vẫn OFF → dùng row 1
                    self.stop_iny_jog()
                    self.current_row = 1
                    self.stack_row_index = 1
                    self.get_logger().warn(f"⚠️ S4 OFF at row 1 limit ({row1_limit}mm) → Using row 1")
                    self.state = SystemState.S1_INY_TO_NEAREST_ROW
                else:
                    # Tiếp tục jog xuống (gọi 1 lần)
                    self.start_iny_jog(self.config.iny_search_velocity)
            else:  # S4 ON - phát hiện chồng khay
                self.stop_iny_jog()
                current_pos = self.get_servo_position(2)
                if current_pos is not None:
                    self.current_row = self.find_nearest_row(current_pos, self.config.iny_input_stack)
                    self.stack_row_index = self.current_row  # ✅ Lưu vị trí stack cho State 2
                    self.get_logger().info(f"📍 S4 ON - Tray detected at {current_pos:.1f}mm → Row {self.current_row} (stack saved)")
                    self.state = SystemState.S1_INY_TO_NEAREST_ROW
        
        elif self.state == SystemState.S1_INY_TO_NEAREST_ROW:
            # Di chuyển đến vị trí row gần nhất từ YAML
            target_pos = self.config.iny_input_stack.get(self.current_row, self.config.iny_input_stack[1])
            if self.move_servo(2, target_pos):
                self.get_logger().info(f"✅ InY at row {self.current_row} ({target_pos}mm)")
                self.extend_cylinder1()
                self.start_cylinder_timer()
                self.state = SystemState.S1_CYLINDER1_EXTEND
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S1_CYLINDER1_EXTEND:
            # Chờ S11 ON (Cylinder 1 MAX - extend complete)
            if self.sensor_manager.get_sensor(11):  # S11 = Cyl1 MAX (extend)
                self.get_logger().info("✅ Cylinder 1 extended (S11 ON) - Tray grabbed")
                self._cylinder_start_time = None
                self.state = SystemState.S1_INY_TO_TARGET1
            elif self.check_cylinder_timeout():
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S1_INY_TO_TARGET1:
            # InY lên Target 1 (safe position)
            if self.move_servo(2, self.config.iny_home):
                self.get_logger().info("✅ InY at Target 1 (safe)")
                self.state = SystemState.S1_INY_TO_TARGET2
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S1_INY_TO_TARGET2:
            # InY lên Target 2 (robot place position)
            if self.move_servo(2, self.config.iny_target2):
                self.get_logger().info("✅ InY at Target 2 (robot place)")
                self.retract_cylinder1()
                self.start_cylinder_timer()
                self.state = SystemState.S1_CYLINDER1_RETRACT
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S1_CYLINDER1_RETRACT:
            # Chờ S10 ON (Cylinder 1 MIN - retract complete)
            if self.sensor_manager.get_sensor(10):  # S10 = Cyl1 MIN (retract)
                self.get_logger().info("✅ Cylinder 1 retracted (S10 ON) - Tray released")
                self._cylinder_start_time = None
                self.state = SystemState.S1_INY_RETURN_SAFE
            elif self.check_cylinder_timeout():
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S1_INY_RETURN_SAFE:
            # InY về Target 1 (safe)
            if self.move_servo(2, self.config.iny_home):
                self.get_logger().info("✅ InY returned to safe position")
                self.state = SystemState.S1_INX_RETURN_SAFE
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S1_INX_RETURN_SAFE:
            # InX về Target 1 (safe) - CHỈ KHI InY safe
            if self.is_iny_safe_for_inx_move():
                if self.move_servo(1, self.config.inx_home):
                    self.get_logger().info("✅ InX returned to safe position")
                    self._pos1_done = True  # ✅ Position 1 hoàn thành
                    if self._pos2_done:
                        self.state = SystemState.STATE1_COMPLETE
                    else:
                        self.get_logger().info("⏳ Position 1 done, chờ Position 2 (Servo 3)...")
                else:
                    self.state = SystemState.ERROR
            else:
                self.get_logger().warn("⏳ Waiting InY safe for InX return...")
        
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
            # Chờ /robot/change_tray + INTERLOCK motion_busy
            if self.change_tray_input_signal:
                if not self.robot_motion_busy:
                    # ✅ SAFE: Robot đã xong motion, cho phép thay khay
                    self.get_logger().info("📥 change_tray received + Robot IDLE → Starting State 2")
                    self.change_tray_input_signal = False
                    self.state = SystemState.S2_CHECK_INTERLOCK
                else:
                    # ⚠️ INTERLOCK: Robot vẫn đang busy, chờ
                    self.get_logger().warn("⏳ change_tray received but Robot BUSY → Waiting...")
            # ✅ Cũng check output tray signal khi đang chờ input
            elif self.change_tray_output_signal:
                if not self.robot_motion_busy:
                    self.get_logger().info("📤 output_tray_full received while S2_WAIT → Starting State 3")
                    self.change_tray_output_signal = False
                    self.state = SystemState.S3_CHECK_SAFE
                else:
                    self.get_logger().warn("⏳ output_tray_full received but Robot BUSY → Waiting...")
        
        # ══════════════════════════════════════════════════════════════
        # STATE 2 - PHASE A: Thu hồi khay cũ từ robot → output stack
        # ══════════════════════════════════════════════════════════════
        
        elif self.state == SystemState.S2_CHECK_INTERLOCK:
            # Check INY safe, INX safe, S3
            iny_safe = self.is_iny_safe_for_inx_move()
            s3_status = self.sensor_manager.get_sensor(3)
            
            if iny_safe:
                self.get_logger().info(f"✅ [S2-A] Interlock OK (INY safe, S3={'ON' if s3_status else 'OFF'})")
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
                self.state = SystemState.S2_CYLINDER1_EXTEND
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_CYLINDER1_EXTEND:
            # Chờ S11 ON (Cylinder 1 MAX - extend = grip khay)
            if self.sensor_manager.get_sensor(11):
                self.get_logger().info("✅ [S2-A] Cylinder 1 extended (S11 ON) - Tray grabbed")
                self._cylinder_start_time = None
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
            # INX về vị trí output stack (Position 1 - TÁCH RIÊNG khỏi inx_home)
            if self.is_iny_safe_for_inx_move():
                if self.move_servo(1, self.config.inx_output_stack):
                    self.get_logger().info(f"✅ [S2-A] INX at output stack ({self.config.inx_output_stack}mm)")
                    self.state = SystemState.S2_INY_SEARCH_STACK
                else:
                    self.state = SystemState.ERROR
            else:
                self.get_logger().warn("⏳ [S2-A] Waiting INY safe for INX move to output stack...")
        
        elif self.state == SystemState.S2_INY_SEARCH_STACK:
            # ✅ MANUAL MODE: Nếu đã set target row, bỏ qua sensor search
            if self.manual_mode and self.manual_target_row is not None:
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
                row1_limit = self.config.iny_input_stack.get(1, 600.0)  # S2 PhaseA: search input stack
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
                    self.output_stack_row = self.find_nearest_row(current_pos, self.config.iny_input_stack)
                    self.get_logger().info(f"📍 [S2-A] S4 ON at {current_pos:.1f}mm → Output stack row {self.output_stack_row}")
                    self.state = SystemState.S2_INY_TO_NEAREST_ROW
        
        elif self.state == SystemState.S2_INY_TO_NEAREST_ROW:
            # Di chuyển đến vị trí row gần nhất (tốc độ chậm)
            target_pos = self.config.iny_output_stack.get(self.output_stack_row, self.config.iny_output_stack[1])
            if self.move_servo(2, target_pos):
                self.get_logger().info(f"✅ [S2-A] INY at output stack row {self.output_stack_row} ({target_pos}mm)")
                self.retract_cylinder1()
                self.start_cylinder_timer()
                self.state = SystemState.S2_CYLINDER1_RETRACT
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_CYLINDER1_RETRACT:
            # Chờ S10 ON (Cylinder 1 MIN - retract = thả khay)
            if self.sensor_manager.get_sensor(10):
                self.get_logger().info("✅ [S2-A] Cylinder 1 retracted (S10 ON) - Tray released to output stack")
                self._cylinder_start_time = None
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
            remaining = self.check_has_trays_remaining()
            if remaining:
                self.get_logger().info(f"✅ [S2-B] Trays remaining (S3={'ON' if self.sensor_manager.get_sensor(3) else 'OFF'}, "
                             f"stack_row={self.stack_row_index}) → Starting Phase B (load new tray)")
                self.state = SystemState.S2_LOAD_INX_TO_TARGET2
            else:
                self.get_logger().warn("🚨 [S2] No trays remaining → Publishing is_last_tray")
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
                row1_limit = self.config.iny_output_stack.get(1, 590.0)  # S2 PhaseB: search output stack
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
                    new_row = self.find_nearest_row(current_pos, self.config.iny_output_stack)
                    self.stack_row_index = new_row  # Update stack tracking
                    self.get_logger().info(f"📍 [S2-B] S4 ON - New tray at {current_pos:.1f}mm → Row {new_row}")
                    self.state = SystemState.S2_LOAD_INY_TO_ROW
        
        elif self.state == SystemState.S2_LOAD_INY_TO_ROW:
            # Di chuyển đến vị trí row chính xác
            target_pos = self.config.iny_output_stack.get(self.stack_row_index, self.config.iny_output_stack[1])
            if self.move_servo(2, target_pos):
                self.get_logger().info(f"✅ [S2-B] INY at row {self.stack_row_index} ({target_pos}mm)")
                self.extend_cylinder1()
                self.start_cylinder_timer()
                self.state = SystemState.S2_LOAD_CYLINDER1_EXTEND
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_LOAD_CYLINDER1_EXTEND:
            # Chờ S11 ON (grip khay mới)
            if self.sensor_manager.get_sensor(11):
                self.get_logger().info("✅ [S2-B] Cylinder 1 extended - New tray grabbed")
                self._cylinder_start_time = None
                self.state = SystemState.S2_LOAD_INY_UP_SAFE
            elif self.check_cylinder_timeout():
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_LOAD_INY_UP_SAFE:
            # INY nâng khay mới lên safe
            if self.move_servo(2, self.config.iny_home):
                self.get_logger().info("✅ [S2-B] INY up with new tray (safe)")
                self.state = SystemState.S2_LOAD_INY_TO_ROBOT_POS
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_LOAD_INY_TO_ROBOT_POS:
            # INY đến vị trí robot place (target2)
            if self.move_servo(2, self.config.iny_target2):
                self.get_logger().info("✅ [S2-B] INY at robot position (target2)")
                self.retract_cylinder1()
                self.start_cylinder_timer()
                self.state = SystemState.S2_LOAD_CYLINDER1_RETRACT
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_LOAD_CYLINDER1_RETRACT:
            # Chờ S10 ON (thả khay mới)
            if self.sensor_manager.get_sensor(10):
                self.get_logger().info("✅ [S2-B] Cylinder 1 retracted - New tray placed for robot")
                self._cylinder_start_time = None
                self.state = SystemState.S2_LOAD_INY_RETURN_SAFE
            elif self.check_cylinder_timeout():
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_LOAD_INY_RETURN_SAFE:
            # INY về safe
            if self.move_servo(2, self.config.iny_home):
                self.get_logger().info("✅ [S2-B] INY returned to safe")
                self.state = SystemState.S2_LOAD_INX_RETURN_SAFE
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S2_LOAD_INX_RETURN_SAFE:
            # INX về safe (sau khi INY safe)
            if self.is_iny_safe_for_inx_move():
                if self.move_servo(1, self.config.inx_home):
                    self.get_logger().info("✅ [S2-B] INX returned to safe - Phase B complete")
                    self.state = SystemState.STATE2_COMPLETE
                else:
                    self.state = SystemState.ERROR
            else:
                self.get_logger().warn("⏳ [S2-B] Waiting INY safe for INX return...")
        
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
            # Chờ tín hiệu change_tray_output + INTERLOCK motion_busy
            if self.change_tray_output_signal:
                if not self.robot_motion_busy:
                    self.get_logger().info("📥 [S3] change_tray_output + Robot IDLE → Starting State 3")
                    self.change_tray_output_signal = False
                    self.output_tray_full = False
                    self.pub_output_ready(False)  # Interlock robot trong lúc thay khay
                    self.state = SystemState.S3_CHECK_SAFE
                else:
                    self.get_logger().warn("⏳ [S3] change_tray_output nhưng Robot BUSY → Waiting...")
        
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
                self.state = SystemState.S3_CYLINDER2_EXTEND
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S3_CYLINDER2_EXTEND:
            # Chờ S13 ON (Cylinder 2 MAX - extend = grip khay)
            if self.sensor_manager.get_sensor(13):  # S13 = Cyl2 MAX
                self.get_logger().info("✅ [S3] Cylinder 2 extended (S13 ON) - Tray gripped")
                self._cylinder_start_time = None
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
                self.state = SystemState.S3_CYLINDER2_RETRACT
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S3_CYLINDER2_RETRACT:
            # Chờ S12 ON (Cylinder 2 MIN - retract = thả khay)
            if self.sensor_manager.get_sensor(12):  # S12 = Cyl2 MIN
                self.get_logger().info("✅ [S3] Cylinder 2 retracted (S12 ON) - Tray released to output stack")
                self._cylinder_start_time = None
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
                    self.get_logger().info("✅ [S3] OUTX returned to safe - Now moving servo 3")
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
                    self.start_servo3_jog(self.config.servo3_jog_velocity)
        
        elif self.state == SystemState.S3_SERVO3_CHECK_S7:
            # Check S7: còn khay trên platform?
            if self.sensor_manager.get_sensor(7):  # S7 ON → còn khay
                self.get_logger().info("✅ [S3] S7 ON - Còn khay output")
                self.pub_output_ready(True)
                self.state = SystemState.STATE3_COMPLETE
            else:  # S7 OFF → hết khay, cần cấp mới
                self.get_logger().warn("⚠️ [S3] S7 OFF - Hết khay output → Servo 3 về Target 1, chờ cấp")
                self.state = SystemState.S3_SERVO3_TO_TARGET1
        
        elif self.state == SystemState.S3_SERVO3_TO_TARGET1:
            # Servo 3 → Target 1 (vị trí chờ load khay)
            if self.move_servo(3, self.config.servo3_target1):
                self.get_logger().info("✅ [S3] Servo 3 at target1 (wait position)")
                self.pub_output_ready(False)  # Interlock robot
                self._confirm_load_received = False
                self.state = SystemState.S3_SERVO3_WAIT_LOAD
            else:
                self.state = SystemState.ERROR
        
        elif self.state == SystemState.S3_SERVO3_WAIT_LOAD:
            # Chờ S7 ON + HMI confirm. KHÔNG timeout, chỉ warning
            if self.sensor_manager.get_sensor(7):  # S7 ON
                if self._confirm_load_received:
                    self.get_logger().info("✅ [S3] S7 ON + HMI confirmed → Moving servo 3 to S6")
                    self._confirm_load_received = False
                    # Servo 3 jog+ đến S6 ON rồi complete
                    self.state = SystemState.S3_SERVO3_MOVE_TO_S6
                else:
                    self.get_logger().warn("⏳ [S3] S7 ON - Chờ HMI confirm (/providesystem/confirm_output_load)")
            else:
                self.get_logger().warn("⏳ [S3] S7 OFF - Chờ cấp khay output mới...")
        
        elif self.state == SystemState.STATE3_COMPLETE:
            if not self.has_trays_remaining and self.last_batch_mode:
                # ✅ PROCESS DONE: Last batch + hết khay → IDLE, cần Start button + Homing cho chu kỳ mới
                self.get_logger().info("🏆 PROCESS COMPLETE - Last batch done, cần Start button để bắt đầu chu kỳ mới")
                self.last_batch_mode = False
                self.state = SystemState.IDLE
            elif not self.has_trays_remaining:
                # ⚠️ Hết khay output nhưng process vẫn chạy → Chờ HMI confirm nạp khay output mới
                self.get_logger().info("📋 Hết khay output - Chờ HMI confirm nạp khay mới (/providesystem/hmi_resume)")
                self._hmi_resume_confirmed = False
                self.state = SystemState.S3_WAIT_HMI_RESUME
            else:
                self.get_logger().info("🎉 STATE 3 COMPLETE - Output tray changed → Back to central hub")
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
            self.get_logger().error("❌ SYSTEM ERROR - Please check and reset")
        
        # ✅ MANUAL MODE: Reset step_pending sau khi execute state
        if self.manual_mode and self.state != state_before:
            self.step_pending = False  # Pause for next step


def main(args=None):
    rclpy.init(args=args)
    system = None
    try:
        config = CartridgeConfig()
        system = CartridgeSystem(config)
        rclpy.spin(system)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Fatal error: {e}")
    finally:
        if system is not None:
            system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()