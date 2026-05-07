#!/usr/bin/env python3
"""
Loadcell Node — chạy trên RevPi A
ROS_DOMAIN_ID=22, kết nối với Pi 5 qua LAN

Hardware: HX711 hoặc Modbus RTU
Fallback: simulation mode (use_simulation:=true)

Deploy:
  scp -r ~/ros2_ws/src/loadcell_ros2 pi@revpi-a:~/ros2_ws/src/
  ssh pi@revpi-a "cd ~/ros2_ws && colcon build --packages-select loadcell_ros2"
  ssh pi@revpi-a "source ~/ros2_ws/install/setup.bash && ros2 run loadcell_ros2 loadcell_node"
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import Trigger


class LoadcellNode(Node):
    def __init__(self):
        super().__init__('loadcell_node')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('use_simulation', True)
        self.declare_parameter('hx711_dout_pin', 5)
        self.declare_parameter('hx711_sck_pin', 6)
        self.declare_parameter('hx711_gain', 128)
        self.declare_parameter('overload_threshold_g', 5000.0)
        self.declare_parameter('zero_drift_threshold_g', 10.0)
        self.declare_parameter('stabilize_samples', 5)
        self.declare_parameter('stabilize_tolerance_g', 2.0)
        self.declare_parameter('empty_tray_threshold_g', 30.0)
        self.declare_parameter('publish_rate_hz', 10.0)

        self._use_sim      = self.get_parameter('use_simulation').value
        self._overload_thr = self.get_parameter('overload_threshold_g').value
        self._drift_thr    = self.get_parameter('zero_drift_threshold_g').value
        self._stab_n       = self.get_parameter('stabilize_samples').value
        self._stab_tol     = self.get_parameter('stabilize_tolerance_g').value
        self._empty_thr    = self.get_parameter('empty_tray_threshold_g').value
        pub_rate           = self.get_parameter('publish_rate_hz').value

        # ── Hardware init ────────────────────────────────────────────
        self._hx = None
        self._zero_offset = 0.0
        self._tare_base   = 0.0
        self._tared       = False
        self._cal_factor  = 1.0
        self._cal_zero    = 0.0
        self._cal_weight_known = 0.0
        self._cal_state   = 'IDLE'

        if not self._use_sim:
            self._init_hx711()
        else:
            self.get_logger().warn('[LOADCELL] Simulation mode ON — không dùng phần cứng thật')

        # ── State ────────────────────────────────────────────────────
        self._lock          = threading.Lock()
        self._target_weight = 0.0   # tổng batch (8 hộp)
        self._target_min    = 0.0
        self._target_max    = 0.0
        self._raw_weight    = 0.0   # gram sau tare+cal
        self._overloaded    = False
        self._overload_acked = False
        self._batch_total   = 0
        self._batch_pass    = 0
        self._batch_fail    = 0
        self._consec_fails  = 0
        self._prev_raw      = 0.0
        self._zero_drift_warned = False

        # ── Publishers ───────────────────────────────────────────────
        qos = QoSProfile(depth=10)
        self._pub_weight   = self.create_publisher(Float32, '/loadcell/weight', qos)
        self._pub_status   = self.create_publisher(String,  '/loadcell/status', qos)
        self._pub_cal_st   = self.create_publisher(String,  '/loadcell/cal_status', qos)
        self._pub_batch    = self.create_publisher(String,  '/loadcell/batch_stats', qos)
        self._pub_cfails   = self.create_publisher(Int32,   '/loadcell/consecutive_fails', qos)
        self._pub_overload = self.create_publisher(Bool,    '/loadcell/overload', qos)
        self._pub_drift    = self.create_publisher(Bool,    '/loadcell/zero_drift_warning', qos)
        self._pub_signal      = self.create_publisher(Bool,    '/loadcell/signal_process', qos)
        self._pub_mon         = self.create_publisher(String,  '/weight/monitor_status', qos)
        self._pub_ink_cap_ack = self.create_publisher(Float32, '/Fill_HP1/ink_capacity_ack', qos)

        # ── Subscribers ──────────────────────────────────────────────
        self.create_subscription(Float32, '/loadcell/target_weight', self._cb_target, qos)
        self.create_subscription(Float32, '/loadcell/target_min',    self._cb_tmin,   qos)
        self.create_subscription(Float32, '/loadcell/target_max',    self._cb_tmax,   qos)
        self.create_subscription(Float32, '/loadcell/cal_weight',    self._cb_cal_w,  qos)
        self.create_subscription(Bool,    '/loadcell/tare_cmd',      self._cb_tare,      qos)
        self.create_subscription(Bool,    '/loadcell/tare_reset',    self._cb_tare_r,    qos)
        self.create_subscription(Bool,    '/loadcell/overload_ack',  self._cb_ol_ack,    qos)
        self.create_subscription(Bool,    '/loadcell/batch_reset',   self._cb_breset,    qos)
        self.create_subscription(String,  '/weight/active_profile',  self._cb_profile,   qos)
        self.create_subscription(Float32, '/Fill_HP1/ink_capacity',  self._cb_ink_cap,   qos)

        # ── Services ─────────────────────────────────────────────────
        self.create_service(Trigger, '/loadcell/cal_start',     self._srv_cal_start)
        self.create_service(Trigger, '/loadcell/cal_set_known', self._srv_cal_set)

        # ── Timers ───────────────────────────────────────────────────
        period = 1.0 / pub_rate
        self._weight_timer  = self.create_timer(period, self._publish_weight)
        self._decision_timer = self.create_timer(0.5, self._check_decision)

        self._pub_status.publish(self._str('READY'))
        self._pub_cal_st.publish(self._str('IDLE'))
        self._publish_batch_stats()

        # simulation state machine
        self._sim_weight    = 0.0
        self._sim_cartridge = False
        self._sim_timer     = self.create_timer(2.0, self._sim_tick) if self._use_sim else None

        self.get_logger().info('[LOADCELL] Node started — topics: /loadcell/*, /weight/monitor_status')

    # ── Hardware ─────────────────────────────────────────────────────

    def _init_hx711(self):
        try:
            from hx711 import HX711
            dout = self.get_parameter('hx711_dout_pin').value
            sck  = self.get_parameter('hx711_sck_pin').value
            gain = self.get_parameter('hx711_gain').value
            self._hx = HX711(dout, sck)
            self._hx.set_gain(gain)
            self._hx.reset()
            self._zero_offset = self._hx.get_raw_data_mean(10)
            self.get_logger().info(f'[HX711] Init OK — dout={dout} sck={sck} zero={self._zero_offset:.0f}')
        except Exception as e:
            self.get_logger().error(f'[HX711] Init FAILED: {e} — switching to simulation')
            self._use_sim = True
            self._hx = None

    def _read_raw_gram(self) -> float:
        if self._use_sim:
            return self._sim_weight
        try:
            raw = self._hx.get_raw_data_mean(3)
            gram = (raw - self._zero_offset - self._cal_zero) * self._cal_factor
            if self._tared:
                gram -= self._tare_base
            return gram
        except Exception as e:
            self.get_logger().warn(f'[HX711] Read error: {e}')
            return self._raw_weight

    # ── Simulation tick ───────────────────────────────────────────────

    def _sim_tick(self):
        """Mô phỏng đặt cartridge lên cân rồi lấy ra, PASS hoặc FAIL."""
        with self._lock:
            if not self._sim_cartridge:
                target_per_cart = (self._target_weight / 8.0) if self._target_weight > 0 else 150.0
                import random
                ok = random.random() > 0.15
                self._sim_weight = target_per_cart * (1.0 if ok else 0.88)
                self._sim_cartridge = True
            else:
                self._sim_weight = 0.0
                self._sim_cartridge = False

    # ── Publish weight ────────────────────────────────────────────────

    def _publish_weight(self):
        gram = self._read_raw_gram()
        with self._lock:
            self._raw_weight = gram

        self._pub_weight.publish(Float32(data=float(gram)))

        # overload check
        if gram > self._overload_thr and not self._overloaded:
            self._overloaded = True
            self._overload_acked = False
            self._pub_overload.publish(Bool(data=True))
            self._pub_mon.publish(self._str('OVERLOAD'))
        elif gram <= self._overload_thr and self._overloaded and self._overload_acked:
            self._overloaded = False
            self._pub_overload.publish(Bool(data=False))

        # zero drift check (only when scale should be empty)
        if gram < self._empty_thr:
            drift = abs(gram)
            if drift > self._drift_thr and not self._zero_drift_warned:
                self._zero_drift_warned = True
                self._pub_drift.publish(Bool(data=True))
            elif drift <= self._drift_thr and self._zero_drift_warned:
                self._zero_drift_warned = False
                self._pub_drift.publish(Bool(data=False))

    # ── Decision logic ────────────────────────────────────────────────

    def _check_decision(self):
        """Khi phát hiện cartridge trên cân (weight ổn định > empty_thr), đưa ra PASS/FAIL."""
        with self._lock:
            w = self._raw_weight
            target_min = self._target_min
            target_max = self._target_max
            target_wt  = self._target_weight

        if w < self._empty_thr:
            return  # không có cartridge

        # Tính target_min/max từ target_weight nếu chưa set
        if target_min <= 0 and target_max <= 0 and target_wt > 0:
            per_cart   = target_wt / 8.0
            target_min = per_cart * 0.95
            target_max = per_cart * 1.05

        if target_min <= 0:
            return  # chưa nhận target

        # Kiểm tra ổn định (đọc thêm vài lần)
        samples = []
        for _ in range(self._stab_n):
            samples.append(self._read_raw_gram())
            time.sleep(0.05)

        spread = max(samples) - min(samples)
        if spread > self._stab_tol:
            return  # chưa ổn định

        stable_w = sum(samples) / len(samples)
        passed = (target_min <= stable_w <= target_max)

        self.get_logger().info(
            f'[DECISION] w={stable_w:.1f}g min={target_min:.1f} max={target_max:.1f} → {"PASS" if passed else "FAIL"}'
        )

        # Publish kết quả
        self._pub_signal.publish(Bool(data=passed))

        with self._lock:
            self._batch_total += 1
            if passed:
                self._batch_pass += 1
                self._consec_fails = 0
            else:
                self._batch_fail += 1
                self._consec_fails += 1

        self._pub_cfails.publish(Int32(data=self._consec_fails))
        self._publish_batch_stats()
        status = 'PASS' if passed else 'FAIL'
        self._pub_mon.publish(self._str(f'LAST:{status} total={self._batch_total}'))

    # ── Subscribers callbacks ─────────────────────────────────────────

    def _cb_target(self, msg: Float32):
        with self._lock:
            self._target_weight = msg.data
        self.get_logger().info(f'[TARGET] weight={msg.data:.1f}g')

    def _cb_tmin(self, msg: Float32):
        with self._lock:
            self._target_min = msg.data

    def _cb_tmax(self, msg: Float32):
        with self._lock:
            self._target_max = msg.data

    def _cb_cal_w(self, msg: Float32):
        with self._lock:
            self._cal_weight_known = msg.data
        self.get_logger().info(f'[CAL] Known weight = {msg.data:.1f}g')

    def _cb_tare(self, msg: Bool):
        if not msg.data:
            return
        with self._lock:
            self._tare_base = self._raw_weight
            self._tared = True
        self.get_logger().info(f'[TARE] base={self._tare_base:.1f}g')

    def _cb_tare_r(self, msg: Bool):
        if not msg.data:
            return
        with self._lock:
            self._tare_base = 0.0
            self._tared = False
        self.get_logger().info('[TARE] Reset — back to raw')

    def _cb_ol_ack(self, msg: Bool):
        if msg.data:
            self._overload_acked = True

    def _cb_breset(self, msg: Bool):
        if not msg.data:
            return
        with self._lock:
            self._batch_total = 0
            self._batch_pass  = 0
            self._batch_fail  = 0
            self._consec_fails = 0
        self._pub_cfails.publish(Int32(data=0))
        self._publish_batch_stats()
        self.get_logger().info('[BATCH] Reset stats')

    def _cb_profile(self, msg: String):
        self.get_logger().info(f'[PROFILE] Active: {msg.data}')

    def _cb_ink_cap(self, msg: Float32):
        self._pub_ink_cap_ack.publish(Float32(data=msg.data))

    # ── Services ─────────────────────────────────────────────────────

    def _srv_cal_start(self, _req, res):
        with self._lock:
            self._cal_zero  = self._raw_weight
            self._cal_state = 'WAITING_WEIGHT'
        self._pub_cal_st.publish(self._str('WAITING_WEIGHT'))
        self.get_logger().info(f'[CAL] Step1 done — zero={self._cal_zero:.1f}g')
        res.success = True
        res.message = 'Zero recorded. Place known weight then call cal_set_known.'
        return res

    def _srv_cal_set(self, _req, res):
        with self._lock:
            known = self._cal_weight_known
            raw_now = self._raw_weight

        if known <= 0:
            self._pub_cal_st.publish(self._str('ERROR'))
            res.success = False
            res.message = 'cal_weight not received yet'
            return res

        span = raw_now - self._cal_zero
        if abs(span) < 1.0:
            self._pub_cal_st.publish(self._str('ERROR'))
            res.success = False
            res.message = f'Span too small ({span:.1f}g) — check hardware'
            return res

        with self._lock:
            self._cal_factor = known / span
            self._cal_state  = 'DONE'

        self._pub_cal_st.publish(self._str('DONE'))
        self.get_logger().info(f'[CAL] factor={self._cal_factor:.4f} (span={span:.1f}→{known:.1f}g)')
        res.success = True
        res.message = f'Calibration done. factor={self._cal_factor:.4f}'
        return res

    # ── Helpers ───────────────────────────────────────────────────────

    def _str(self, text: str) -> String:
        m = String()
        m.data = text
        return m

    def _publish_batch_stats(self):
        stats = json.dumps({
            'total': self._batch_total,
            'pass':  self._batch_pass,
            'fail':  self._batch_fail,
        })
        self._pub_batch.publish(self._str(stats))


def main(args=None):
    rclpy.init(args=args)
    node = LoadcellNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
