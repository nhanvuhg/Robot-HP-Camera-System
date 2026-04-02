import time
from rclpy.node import Node
import rclpy

def _fake_mot():
    class FakeMot:
        def target_position_reached(self):
            time.sleep(3.0) # simulate timeout
            return False
        def stop_motion_task(self): pass
        def velocity_task(self, *a, **k): pass
        def position_task(self, *a, **k):
            time.sleep(3.0) # simulate write timeout
            pass
        def acknowledge_faults(self): time.sleep(3.0)
        def enable_powerstage(self): time.sleep(3.0)
        def ready_for_motion(self):
            time.sleep(3.0)
            return False
    return FakeMot()

print("Mock generated")
