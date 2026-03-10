#!/usr/bin/env python3
"""
Sim Auto Test — Chạy STATE1 tự động với simulated sensors.

Usage:
    source ~/ros2_ws/install/setup.bash
    python3 sim_auto_test.py

Yêu cầu: Node cartridge_providesystem_py đang chạy.
Script sẽ tự: MANUAL → START → HOME → STATE1 → full cycle → kiểm tra node alive.

Flow STATE1 đầy đủ:
  1. S1 ON → InX chạy tới input stack (500mm)
  2. InX dừng → check S3
  3. S3 ON → InY jog tìm tray (hướng +)
  4. S4 ON → Tray detected → InY stop → move to nearest row
  5. S11 ON → Cylinder 1 extended (gắp khay)
  6. InY về Target 1 (10mm — safe zone)
  7. InY tiếp tục đi tới Target 2 (200mm — robot place)
  8. Cylinder retract → S10 ON (nhả khay)
  9. InY về Target 1 (10mm — safe zone)
  10. InX về Target 1 (20mm — gần home)
  → STATE1_COMPLETE
"""

import time
import subprocess
import sys
import json

def pub(topic, msg_type, data):
    """Publish once to a ROS2 topic."""
    try:
        subprocess.run(
            ['ros2', 'topic', 'pub', '--once', topic, msg_type, data],
            capture_output=True, timeout=5
        )
    except:
        pass

def get_state(timeout=3):
    """Get current system state."""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '/system_state', '--once'],
            capture_output=True, text=True, timeout=timeout
        )
        for line in result.stdout.splitlines():
            if line.startswith('data:'):
                return line.split(':', 1)[1].strip()
    except:
        pass
    return None

def get_positions():
    """Get servo positions as dict."""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '/providesystem/servo_positions', '--once'],
            capture_output=True, text=True, timeout=3
        )
        for line in result.stdout.splitlines():
            if line.startswith('data:'):
                raw = line.split(':', 1)[1].strip().strip("'\"")
                return json.loads(raw)
    except:
        pass
    return {}

def sim_sensor(sensor_id, value=1):
    """Simulate sensor ON/OFF."""
    pub('/providesystem/sim_sensor', 'std_msgs/String', f"data: '{sensor_id}:{value}'")
    state = 'ON' if value else 'OFF'
    print(f"    📡 SIM: S{sensor_id} → {state}")

def wait_state(target, timeout=120, poll=2):
    """Wait until state contains target string."""
    start = time.time()
    last_st = None
    while time.time() - start < timeout:
        st = get_state()
        if st and st != last_st:
            print(f"    ⏳ State: {st} ({int(time.time()-start)}s)")
            last_st = st
        if st and target in st:
            return st
        time.sleep(poll)
    print(f"    ⚠️ Timeout ({timeout}s) waiting for '{target}'")
    return None

def wait_any_state(targets, timeout=120, poll=2):
    """Wait until state matches any of targets."""
    start = time.time()
    last_st = None
    while time.time() - start < timeout:
        st = get_state()
        if st and st != last_st:
            print(f"    ⏳ State: {st} ({int(time.time()-start)}s)")
            last_st = st
        if st:
            for t in targets:
                if t in st:
                    return st
        time.sleep(poll)
    print(f"    ⚠️ Timeout ({timeout}s) waiting for {targets}")
    return None

def check_node_alive():
    """Check if cartridge node is running."""
    result = subprocess.run(['pgrep', '-c', '-f', 'cartridge_providesystem_py'],
                          capture_output=True, text=True)
    return result.stdout.strip() not in ('0', '')

def step(num, name):
    print(f"\n{'─'*55}")
    print(f"  Step {num}: {name}")
    print(f"{'─'*55}")


def main():
    print()
    print("╔═══════════════════════════════════════════════════════╗")
    print("║  🚀 SIM AUTO TEST — STATE1 Full Cycle                ║")
    print("╚═══════════════════════════════════════════════════════╝")

    if not check_node_alive():
        print("❌ Node not running! Start with: bash ~/ros2_ws/src/unified_control_gui/scripts/start_all.sh")
        sys.exit(1)

    # ═══ 1. MANUAL + START ═══
    step(1, "MANUAL mode + START → Homing")
    pub('/providesystem/set_operation_mode', 'std_msgs/String', 'data: manual')
    time.sleep(1)
    pub('/system/start_button', 'std_msgs/Bool', 'data: true')
    print("    ✅ MANUAL + START sent")

    # ═══ 2. Wait homing ═══
    step(2, "Chờ homing xong...")
    st = wait_state('idle', timeout=120, poll=5)
    if not st:
        print("    ❌ FAIL: Homing timeout!")
        sys.exit(1)
    print("    ✅ HOMED!")

    # ═══ 3. goto STATE1 + S1 ON ═══
    step(3, "goto STATE1 + S1 ON → InX chạy tới input stack")
    pub('/providesystem/goto_state', 'std_msgs/String', 'data: STATE1')
    time.sleep(1)
    sim_sensor(1)  # S1 ON — tray present
    time.sleep(3)
    st = get_state()
    print(f"    State: {st}")

    # ═══ 4. Wait InX stop ═══
    step(4, "Chờ InX dừng → check S3")
    st = wait_any_state(['s1_inx_wait_stop', 's1_iny_search'], timeout=30)
    if not st:
        print("    ❌ FAIL: InX never started")
        sys.exit(1)
    print(f"    ✅ InX position reached: {st}")

    # ═══ 5. S3 ON → InY jog tìm tray ═══
    step(5, "S3 ON → InY jog+ tìm tray")
    sim_sensor(3)
    time.sleep(3)
    pos = get_positions()
    print(f"    InY pos: {pos.get('2', '?')}mm (phải tăng + positive)")
    st = get_state()
    print(f"    State: {st}")

    # ═══ 6. S4 ON → Tray detected ═══
    step(6, "S4 ON → Tray detected → InY stop → move to row")
    time.sleep(2)
    pos = get_positions()
    print(f"    InY pos trước S4: {pos.get('2', '?')}mm")
    sim_sensor(4)  # S4 ON — tray detected
    time.sleep(5)
    st = get_state()
    print(f"    State sau S4: {st}")
    if not check_node_alive():
        print("    ❌ FAIL: Node crashed after S4!")
        sys.exit(1)
    print("    ✅ Node alive after S4!")

    # ═══ 7. S11 ON → Cylinder 1 extended (gắp khay) ═══
    step(7, "S11 ON → Cylinder 1 extended (gắp khay)")
    st = wait_any_state(['s1_cylinder1_extend'], timeout=15)
    if st:
        sim_sensor(11)  # S11 ON — cylinder 1 MAX
        time.sleep(3)
        st = get_state()
        print(f"    State sau S11: {st}")
        print("    ✅ Tray grabbed!")
    else:
        print("    ⚠️ Skipped — state không phải cylinder1_extend")

    # ═══ 8. InY → Target 1 (10mm safe zone) ═══
    step(8, "InY về Target 1 (10mm — safe zone)")
    st = wait_any_state(['s1_iny_to_target1', 's1_iny_to_target2'], timeout=10)
    print(f"    State: {st}")
    pos = get_positions()
    print(f"    InY pos: {pos.get('2', '?')}mm → target: 10mm")

    # ═══ 9. InY → Target 2 (200mm — robot place) ═══
    step(9, "InY tiếp tục → Target 2 (200mm — vị trí đặt khay)")
    st = wait_any_state(['s1_iny_to_target2', 's1_cylinder1_retract'], timeout=15)
    print(f"    State: {st}")
    pos = get_positions()
    print(f"    InY pos: {pos.get('2', '?')}mm → target: 200mm")

    # ═══ 10. S10 ON → Cylinder retract (nhả khay) ═══
    step(10, "S10 ON → Cylinder 1 retracted (nhả khay)")
    st = wait_any_state(['s1_cylinder1_retract'], timeout=15)
    if st:
        sim_sensor(10)  # S10 ON — cylinder 1 MIN
        time.sleep(3)
        st = get_state()
        print(f"    State sau S10: {st}")
        print("    ✅ Tray released!")
    else:
        print("    ⚠️ Skipped — state không phải cylinder1_retract")

    # ═══ 11. InY về Target 1 (10mm safe) ═══
    step(11, "InY về Target 1 (10mm — safe zone)")
    st = wait_any_state(['s1_iny_return_safe', 's1_inx_return'], timeout=10)
    print(f"    State: {st}")
    pos = get_positions()
    print(f"    InY pos: {pos.get('2', '?')}mm → target: 10mm")

    # ═══ 12. InX về Target 1 (20mm gần home) ═══
    step(12, "InX về Target 1 (20mm — gần home)")
    st = wait_any_state(['s1_inx_return_safe', 'state1_complete', 'idle'], timeout=15)
    print(f"    State: {st}")
    pos = get_positions()
    print(f"    InX pos: {pos.get('1', '?')}mm → target: 20mm")
    print(f"    InY pos: {pos.get('2', '?')}mm")

    # ═══ FINAL ═══
    print()
    print("╔═══════════════════════════════════════════════════════╗")
    print("║  📊 KẾT QUẢ TEST                                     ║")
    print("╠═══════════════════════════════════════════════════════╣")
    st = get_state()
    pos = get_positions()
    alive = check_node_alive()
    print(f"║  State:  {st:<44}║")
    print(f"║  InX:    {str(pos.get('1', '?')) + 'mm':<44}║")
    print(f"║  InY:    {str(pos.get('2', '?')) + 'mm':<44}║")
    print(f"║  Node:   {'✅ ALIVE' if alive else '❌ DEAD':<44}║")
    if alive and st and ('complete' in st or 'idle' in st):
        print("║                                                       ║")
        print("║  🎉 TEST PASSED!                                      ║")
    else:
        print("║                                                       ║")
        print("║  ⚠️  TEST INCOMPLETE — check state                    ║")
    print("╚═══════════════════════════════════════════════════════╝")


if __name__ == '__main__':
    main()
