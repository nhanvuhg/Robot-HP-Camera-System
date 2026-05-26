#!/usr/bin/env python3
"""
VFD Logic Node — điều khiển biến tần ATV320 theo cảm biến băng tải.

Logic priority:
    S3 ON                 → STOP (ưu tiên cao nhất)
    S1 ON hoặc S2 ON      → RUN
    Khác                  → giữ trạng thái cũ

Input:  /providesystem/sensors_state (String, 22 ký tự — char 0/1/2 = S1/S2/S3)
Output: /vfd/cmd_run (Bool) → rs485_bus_node trên RevPi A nhận → đẩy Modbus RTU sang ATV320
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class VfdLogicNode(Node):
    def __init__(self):
        super().__init__('vfd_logic_node')

        self.pub_run = self.create_publisher(Bool, '/vfd/cmd_run', 10)
        self.create_subscription(
            String, '/providesystem/sensors_state', self.sensors_cb, 10)

        self.s1_state = False
        self.s2_state = False
        self.s3_state = False
        self.current_cmd = False

        self.publish_cmd(self.current_cmd)
        self.create_timer(0.5, self._heartbeat)

        self.get_logger().info(
            "VFD Logic Node started — listening /providesystem/sensors_state")

    def sensors_cb(self, msg):
        if len(msg.data) < 3:
            return
        s1 = msg.data[0] == '1'
        s2 = msg.data[1] == '1'
        s3 = msg.data[2] == '1'

        if (s1, s2, s3) == (self.s1_state, self.s2_state, self.s3_state):
            return
        self.s1_state, self.s2_state, self.s3_state = s1, s2, s3
        self.evaluate_logic()

    def evaluate_logic(self):
        if self.s3_state:
            new_cmd = False
        elif self.s1_state or self.s2_state:
            new_cmd = True
        else:
            new_cmd = self.current_cmd

        if new_cmd != self.current_cmd:
            self.current_cmd = new_cmd
            self.publish_cmd(self.current_cmd)
            reason = "S3 ON" if self.s3_state else "S1/S2 ON"
            self.get_logger().info(
                f"VFD → {'RUN' if new_cmd else 'STOP'} ({reason})")

    def publish_cmd(self, run_state):
        msg = Bool()
        msg.data = run_state
        self.pub_run.publish(msg)

    def _heartbeat(self):
        # Re-publish hiện tại để rs485_bus_node trên RevPi A đồng bộ
        # sau reconnect/restart.
        self.publish_cmd(self.current_cmd)


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
