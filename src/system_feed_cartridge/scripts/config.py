import os
import yaml
from typing import Dict, List, Optional
from pydantic import BaseModel, Field

class SensorConfig(BaseModel):
    id: int
    name: str
    gui_label: str
    cpx_ip: str
    cpx_channel: str
    desc: str
    pos: str
    role: str
    used_in: List[str]
    active: bool

class CylinderOutputs(BaseModel):
    cylinder1_retract_channel: int
    cylinder1_extend_channel: int
    cylinder2_retract_channel: int
    cylinder2_extend_channel: int

class SystemConfig(BaseModel):
    # Base Network
    servo_ips: Dict[int, str]
    io_ip: str
    io_ip_2: str

    # Cylinder Outputs
    cylinder1_extend_channel: int = 5
    cylinder1_retract_channel: int = 4
    cylinder2_extend_channel: int = 9
    cylinder2_retract_channel: int = 8

    # Limits & Timing
    servo_limits: Dict[int, float]
    homing_timeout: float = 30.0
    move_timeout: float = 60.0
    cylinder_timeout: float = 15.0
    modbus_timeout_ms: int = 3000

    # Velocity (m/s) — synced with FAS
    fas_position_velocity: float = 0.05
    jog_velocity: float = 0.05
    jog_velocity_max: float = 0.08

    # Axes
    inx_home: float = 0.0
    inx_safe: float = -60.0
    inx_target2: float = 500.0
    inx_output_stack: float = 100.0
    iny_home: float = 0.0
    iny_safe_zone: float = 50.0
    iny_push_clearance_mm: float = 5.0   # INY dừng tạm để kẹp trước khi đẩy vào robot (S1)

    # INX Danger Zone — INY không được ra ngoài safe zone khi INX trong vùng này
    inx_danger_zone_min: float = 0.0
    inx_danger_zone_max: float = 400.0

    # Scan
    target_scaninp1: float = 550.0
    target_scanoutp1: float = 500.0
    iny_scan_vel: float = 30.0
    iny_row_vel: float = 20.0
    iny_scan_valid_min_mm: float = 200.0
    iny_scan_valid_max_mm: float = 550.0
    s1_scan_noise_retry_limit: int = 1
    inx_noise_recovery_mm: float = 10.0

    # Zones
    iny_input_zones: Dict[int, List[float]]
    iny_output_zones: Dict[int, List[float]]
    outy_output_zones: Dict[int, List[float]]

    # Pos 2
    iny_target2: float = 60.0
    servo3_home: float = 0.0
    servo3_target1: float = 10.0
    servo3_target2: float = 400.0
    servo3_push_position: float = 300.0
    servo3_jog_velocity: float = 50.0
    servo3_feed_velocity: float = 50.0

    outx_home: float = 0.0
    outx_target1: float = 100.0
    outx_target2: float = 400.0
    outx_target3: float = 20.0
    outy_home: float = 0.0
    outy_target1: float = 10.0
    outy_target2: float = 300.0
    outy_pick_pos: float = 100.0
    outy_safe_zone: float = 10.0
    outy_search_velocity: float = 40.0
    outy_row_limit: float = 680.0
    outy_slow_vel: float = 10.0

    target_scanoutp2: float = 500.0
    outy_scan_arm_mm: float = 50.0
    iny_search_velocity: float = 30.0
    iny_slow_velocity: float = 10.0
    position_tolerance: float = 2.0   # mm — sai số servo "đã đến target", dùng cho _at_position()
    outx_safe_position_threshold: float = 100.0
    max_trays: int = 8
    max_slots_per_output_tray: int = 9

    # Sensors section
    num_sensors: int = 22
    sensor_type: str = "PNP"
    sensor_logic: str = "NO"
    sensors: List[SensorConfig] = Field(default_factory=list)

    @classmethod
    def load(cls, path: str) -> "SystemConfig":
        if not os.path.exists(path):
            raise FileNotFoundError(f"Config file not found: {path}")
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
            
        def convert_keys_to_int(d):
            if not isinstance(d, dict): return d
            return {int(k) if str(k).isdigit() else k: v for k, v in d.items()}
            
        if 'servo_ips' in data: data['servo_ips'] = convert_keys_to_int(data['servo_ips'])
        if 'servo_limits' in data: data['servo_limits'] = convert_keys_to_int(data['servo_limits'])
        if 'iny_input_zones' in data: data['iny_input_zones'] = convert_keys_to_int(data['iny_input_zones'])
        if 'iny_output_zones' in data: data['iny_output_zones'] = convert_keys_to_int(data['iny_output_zones'])
        if 'outy_output_zones' in data: data['outy_output_zones'] = convert_keys_to_int(data['outy_output_zones'])
        
        # Handle YAML parsing "NO" as False
        if isinstance(data.get('sensor_logic'), bool):
            data['sensor_logic'] = "NO" if not data['sensor_logic'] else "NC"

        obj = cls(**data)
        obj._config_file = path
        return obj

    def save(self, path: str):
        with open(path, "w", encoding="utf-8") as f:
            yaml.dump(self.dict(exclude_unset=False), f, default_flow_style=False, sort_keys=False, allow_unicode=True)

    def save_to_file(self):
        if hasattr(self, '_config_file') and self._config_file:
            self.save(self._config_file)
            print(f"Config saved safely to: {self._config_file}")

