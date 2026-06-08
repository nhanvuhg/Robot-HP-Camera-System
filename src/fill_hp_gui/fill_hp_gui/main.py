"""Fill HP GUI launcher: ROS2 node + QML preview.

Wire toi thieu (read-only status + start/stop/mode). Cac topic con lai
duoc them dan dan — UI mockup van hien voi du lieu mac dinh khi chua wire.
"""
from __future__ import annotations

import os
import sys
import threading
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Float32MultiArray, Int32, String

from PyQt5.QtCore import QObject, QUrl, QVariant, Qt, pyqtProperty, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QGuiApplication
from PyQt5.QtQml import QQmlApplicationEngine


# ---------------------------------------------------------------------------
# Bridge: QObject expose properties cho QML binding, slot publish ROS msgs.
# ---------------------------------------------------------------------------
class FillHpBridge(QObject):
    sysStateChanged = pyqtSignal()
    modeChanged = pyqtSignal()
    cycleClockChanged = pyqtSignal()
    runningChanged = pyqtSignal()
    pressuresChanged = pyqtSignal()
    cartridgePressuresChanged = pyqtSignal()
    manualResponseChanged = pyqtSignal()
    hwStatusChanged = pyqtSignal()

    # Signals cho QML invoke publish
    requestMode = pyqtSignal(int)
    requestScreen = pyqtSignal(str)
    requestManual = pyqtSignal(str, str)
    requestReconnect = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        self._sys_state = "IDLE"
        self._mode = "MANUAL"
        self._cycle_clock = "00:00"
        self._running = False
        self._pressure_s1 = 0.0
        self._pressure_s2 = 0.0
        self._pressure_s3 = 0.0
        self._cartridge_pressures: list[float] = []
        self._manual_response = "-"
        self._hw_status = ""

    # ----- properties -----
    @pyqtProperty(str, notify=sysStateChanged)
    def sysState(self) -> str:
        return self._sys_state

    def set_sys_state(self, value: str) -> None:
        if value != self._sys_state:
            self._sys_state = value
            self.sysStateChanged.emit()

    @pyqtProperty(str, notify=modeChanged)
    def mode(self) -> str:
        return self._mode

    def set_mode(self, value: str) -> None:
        if value != self._mode:
            self._mode = value
            self.modeChanged.emit()

    @pyqtProperty(str, notify=cycleClockChanged)
    def cycleClock(self) -> str:
        return self._cycle_clock

    def set_cycle_clock(self, value: str) -> None:
        if value != self._cycle_clock:
            self._cycle_clock = value
            self.cycleClockChanged.emit()

    @pyqtProperty(bool, notify=runningChanged)
    def running(self) -> bool:
        return self._running

    def set_running(self, value: bool) -> None:
        if value != self._running:
            self._running = value
            self.runningChanged.emit()

    @pyqtProperty(float, notify=pressuresChanged)
    def pressureS1(self) -> float:
        return self._pressure_s1

    @pyqtProperty(float, notify=pressuresChanged)
    def pressureS2(self) -> float:
        return self._pressure_s2

    @pyqtProperty(float, notify=pressuresChanged)
    def pressureS3(self) -> float:
        return self._pressure_s3

    def set_pressure(self, idx: int, value: float) -> None:
        attr = ("_pressure_s1", "_pressure_s2", "_pressure_s3")[idx - 1]
        if getattr(self, attr) != value:
            setattr(self, attr, value)
            self.pressuresChanged.emit()

    @pyqtProperty(QVariant, notify=cartridgePressuresChanged)
    def cartridgePressures(self) -> QVariant:
        return self._cartridge_pressures

    def set_cartridge_pressures(self, values: list[float]) -> None:
        self._cartridge_pressures = list(values)
        self.cartridgePressuresChanged.emit()

    @pyqtProperty(str, notify=manualResponseChanged)
    def manualResponse(self) -> str:
        return self._manual_response

    def set_manual_response(self, value: str) -> None:
        if value != self._manual_response:
            self._manual_response = value
            self.manualResponseChanged.emit()

    @pyqtProperty(str, notify=hwStatusChanged)
    def hwStatus(self) -> str:
        return self._hw_status

    def set_hw_status(self, value: str) -> None:
        if value != self._hw_status:
            self._hw_status = value
            self.hwStatusChanged.emit()

    # ----- slots: QML goi de publish ROS -----
    @pyqtSlot(int)
    def setMode(self, mode_int: int) -> None:
        self.requestMode.emit(mode_int)

    @pyqtSlot(str)
    def screenControl(self, action: str) -> None:
        self.requestScreen.emit(action)

    @pyqtSlot(str, str)
    def manualCommand(self, name: str, action: str) -> None:
        self.requestManual.emit(name, action)

    @pyqtSlot(str)
    def reconnect(self, target: str) -> None:
        self.requestReconnect.emit(target)


# ---------------------------------------------------------------------------
# ROS Node: subscribe status, publish control.
# ---------------------------------------------------------------------------
class FillHpGuiNode(Node):
    def __init__(self, bridge: FillHpBridge) -> None:
        super().__init__("fill_hp_gui")
        self.bridge = bridge

        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # --- subscriptions ---
        self.create_subscription(String, "system_status", self._on_system_status, 10)
        self.create_subscription(String, "mode_status", self._on_mode_status, qos_latched)
        self.create_subscription(String, "ball_cycle_time", self._on_cycle_clock, 10)
        self.create_subscription(String, "manual_response", self._on_manual_response, 10)
        self.create_subscription(String, "hw_status", self._on_hw_status, 10)
        self.create_subscription(Float32, "pressure_s1", lambda m: self.bridge.set_pressure(1, float(m.data)), 10)
        self.create_subscription(Float32, "pressure_s2", lambda m: self.bridge.set_pressure(2, float(m.data)), 10)
        self.create_subscription(Float32, "pressure_s3", lambda m: self.bridge.set_pressure(3, float(m.data)), 10)
        self.create_subscription(
            Float32MultiArray, "cartridge_pressures",
            lambda m: self.bridge.set_cartridge_pressures(list(m.data)), 10,
        )

        # --- publishers ---
        self._pub_mode = self.create_publisher(Int32, "mode_switch", 10)
        self._pub_screen = self.create_publisher(String, "screen_control", 10)
        self._pub_manual = self.create_publisher(String, "manual_command", 10)
        self._pub_reconnect = self.create_publisher(String, "reconnect_cmd", 10)

        # --- bridge signals -> publishers ---
        bridge.requestMode.connect(self._publish_mode)
        bridge.requestScreen.connect(self._publish_screen)
        bridge.requestManual.connect(self._publish_manual)
        bridge.requestReconnect.connect(self._publish_reconnect)

        self.get_logger().info("fill_hp_gui node ready")

    # ----- ROS -> bridge -----
    def _on_system_status(self, msg: String) -> None:
        self.bridge.set_sys_state(msg.data)

    def _on_mode_status(self, msg: String) -> None:
        # raw format vd "MANUAL|RUNNING=False|..." -> lay segment dau
        token = msg.data.split("|", 1)[0].strip().upper() or "-"
        self.bridge.set_mode(token)
        running = "RUNNING=TRUE" in msg.data.upper()
        self.bridge.set_running(running)

    def _on_cycle_clock(self, msg: String) -> None:
        self.bridge.set_cycle_clock(msg.data or "00:00")

    def _on_manual_response(self, msg: String) -> None:
        self.bridge.set_manual_response(msg.data or "-")

    def _on_hw_status(self, msg: String) -> None:
        self.bridge.set_hw_status(msg.data)

    # ----- bridge -> ROS -----
    def _publish_mode(self, mode_int: int) -> None:
        msg = Int32()
        msg.data = int(mode_int)
        self._pub_mode.publish(msg)
        self.get_logger().info(f"mode_switch -> {mode_int}")

    def _publish_screen(self, action: str) -> None:
        msg = String()
        msg.data = action
        self._pub_screen.publish(msg)
        self.get_logger().info(f"screen_control -> {action}")

    def _publish_manual(self, name: str, action: str) -> None:
        msg = String()
        msg.data = f"{name}:{action}"
        self._pub_manual.publish(msg)
        self.get_logger().info(f"manual_command -> {msg.data}")

    def _publish_reconnect(self, target: str) -> None:
        msg = String()
        msg.data = target
        self._pub_reconnect.publish(msg)
        self.get_logger().info(f"reconnect_cmd -> {target}")


# ---------------------------------------------------------------------------
# Entry point.
# ---------------------------------------------------------------------------
def _find_qml() -> Path:
    # Tim Main.qml: uu tien share/, fallback source khi run uninstalled
    try:
        share = Path(get_package_share_directory("fill_hp_gui"))
        candidate = share / "qml" / "Main.qml"
        if candidate.exists():
            return candidate
    except Exception:
        pass
    src = Path(__file__).resolve().parent / "qml" / "Main.qml"
    return src


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)

    app = QGuiApplication(sys.argv if argv is None else argv)
    app.setApplicationName("Fill HP Control")

    bridge = FillHpBridge()
    node = FillHpGuiNode(bridge)

    engine = QQmlApplicationEngine()
    engine.rootContext().setContextProperty("bridge", bridge)

    qml_path = _find_qml()
    if not qml_path.exists():
        node.get_logger().error(f"Khong thay QML: {qml_path}")
        rclpy.shutdown()
        return 2
    node.get_logger().info(f"Loading QML: {qml_path}")
    engine.load(QUrl.fromLocalFile(str(qml_path)))
    if not engine.rootObjects():
        node.get_logger().error("QML load failed")
        rclpy.shutdown()
        return 3

    # rclpy spin tren thread phu, Qt event loop o thread chinh
    stop_event = threading.Event()

    def spin() -> None:
        while not stop_event.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    try:
        rc = app.exec_()
    finally:
        stop_event.set()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
