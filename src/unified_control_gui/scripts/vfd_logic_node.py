#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import time

try:
    import revpimodio2
    HAS_REVPI = True
except ImportError:
    HAS_REVPI = False

class VfdLogicNode(Node):
    def __init__(self):
        super().__init__('vfd_logic_node')
        
        self.get_logger().info("VFD Logic Node Started (Direct IO mode priority)")
        
        self.pub_run = self.create_publisher(Bool, '/vfd/cmd_run', 10)
        
        # Configuration parameters
        self.use_direct_io = self.declare_parameter('use_direct_io', True).value
        
        # If using direct IO, specify the pin names (default to I_1 and I_3)
        self.pin_s1 = self.declare_parameter('pin_s1', 'I_1').value
        self.pin_s2 = self.declare_parameter('pin_s2', 'I_2').value
        self.pin_s3 = self.declare_parameter('pin_s3', 'I_3').value
        
        # If using ROS topics (fallback)
        self.sub_sensors = self.create_subscription(String, '/hardware/sensors_d1', self.sensors_cb, 10)
        
        self.s1_state = False
        self.s2_state = False
        self.s3_state = False
        self.current_cmd = False
        self.publish_cmd(self.current_cmd)

    def sensors_cb(self, msg):
        if len(msg.data) >= 3:
            s1 = msg.data[0] == '1'
            s2 = msg.data[1] == '1'
            s3 = msg.data[2] == '1'
            
            if s1 != self.s1_state or s2 != self.s2_state or s3 != self.s3_state:
                self.s1_state = s1
                self.s2_state = s2
                self.s3_state = s3
                self.evaluate_logic()

    def evaluate_logic(self):
        new_cmd = self.current_cmd
        
        # Priority logic: 
        # S3 has highest priority: if s3 -> stop.
        if self.s3_state:
            new_cmd = False
        # If not S3, but S1 (or S2) is ON -> run.
        elif self.s1_state or self.s2_state:
            new_cmd = True
            
        if new_cmd != self.current_cmd:
            self.current_cmd = new_cmd
            self.publish_cmd(self.current_cmd)
            
            state_str = "RUN" if self.current_cmd else "STOP"
            reason = "S3 ON" if self.s3_state else "S1/S2 ON"
            self.get_logger().info(f"VFD State changed to {state_str}. Reason: {reason}")
            
    def publish_cmd(self, run_state):
        msg = Bool()
        msg.data = run_state
        self.pub_run.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VfdLogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
