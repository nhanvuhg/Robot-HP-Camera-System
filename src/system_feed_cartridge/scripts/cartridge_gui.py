#!/usr/bin/env python3
"""
Cartridge System GUI — Web Dashboard v3 (PATCHED)
Fixes applied:
  [FIX-GUI-1] Token authentication (X-Auth-Token header + ?token= query)
  [FIX-GUI-2] Security headers (X-Frame-Options, CSP, no-sniff)
  [FIX-GUI-3] Request body size limit (64KB) để tránh DoS
  [FIX-GUI-4] Config value range validation trước khi gửi lên ROS
"""

import http.server
import json
import secrets
import threading
import time
import os
import signal
import sys
from urllib.parse import urlparse, parse_qs

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool

PORT = 8080

# ============================================================================
# [FIX-GUI-1] AUTH TOKEN — đổi giá trị trong biến môi trường trước khi deploy:
# export GUI_AUTH_TOKEN=your_secret_here
# ============================================================================
_DEFAULT_TOKEN = "cartridge-gui-secret"
AUTH_TOKEN = os.environ.get("GUI_AUTH_TOKEN", _DEFAULT_TOKEN)

# [FIX-GUI-4] Config validation ranges — (min, max)
CONFIG_RANGES = {
    "inx_home":             (0.0,   200.0),
    "inx_safe_zone":        (0.0,   200.0),
    "inx_target2":          (10.0,  700.0),
    "inx_output_stack":     (10.0,  700.0),
    "iny_home":             (0.0,   200.0),
    "iny_safe_zone":        (5.0,   200.0),
    "iny_target2":          (10.0,  700.0),
    "servo3_push_position": (10.0,  400.0),
    "servo3_target1":       (0.0,   200.0),
    "outx_home":            (0.0,   200.0),
    "outx_target2":         (10.0,  700.0),
    "outx_target3":         (10.0,  700.0),
    "outy_home":            (0.0,   200.0),
    "outy_target2":         (10.0,  700.0),
    "outy_safe_zone":       (5.0,   200.0),
}
ROW_POSITION_RANGE = (0.0, 960.0)  # mm — giới hạn vật lý trục


def validate_config(config_dict: dict) -> tuple:
    """[FIX-GUI-4] Validate config values before forwarding to ROS.
    Returns (ok: bool, error_message: str)
    """
    table_key = config_dict.get("table", "")
    positions = config_dict.get("positions", {})
    if table_key and positions:
        lo, hi = ROW_POSITION_RANGE
        for row_str, val in positions.items():
            try:
                v = float(val)
            except (TypeError, ValueError):
                return False, f"Row {row_str}: giá trị không hợp lệ '{val}'"
            if not (lo <= v <= hi):
                return False, f"Row {row_str}: {v}mm ngoài phạm vi [{lo}, {hi}]mm"
        return True, ""
    for key, val in config_dict.items():
        if key in CONFIG_RANGES:
            try:
                v = float(val)
            except (TypeError, ValueError):
                return False, f"'{key}': giá trị không hợp lệ '{val}'"
            lo, hi = CONFIG_RANGES[key]
            if not (lo <= v <= hi):
                return False, f"'{key}': {v} ngoài phạm vi [{lo}, {hi}]"
    return True, ""


# ============================================================================
# ROS2 DIRECT PUBLISHER NODE (low latency)
# ============================================================================

class GuiRosNode(Node):
    def __init__(self):
        super().__init__('cartridge_gui')
        qos_default = QoSProfile(depth=10)
        qos_latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._pubs = {
            '/providesystem/jog_cmd':             self.create_publisher(String, '/providesystem/jog_cmd', qos_default),
            '/providesystem/sim_sensor':           self.create_publisher(String, '/providesystem/sim_sensor', qos_default),
            '/providesystem/move_to_pos':          self.create_publisher(String, '/providesystem/move_to_pos', qos_default),
            '/providesystem/update_config':        self.create_publisher(String, '/providesystem/update_config', qos_default),
            '/providesystem/get_config':           self.create_publisher(String, '/providesystem/get_config', qos_default),
            '/providesystem/set_operation_mode':   self.create_publisher(String, '/providesystem/set_operation_mode', qos_default),
            '/providesystem/goto_state':           self.create_publisher(String, '/providesystem/goto_state', qos_default),
            '/providesystem/set_target_row':       self.create_publisher(String, '/providesystem/set_target_row', qos_default),
            '/providesystem/hmi_resume':           self.create_publisher(Bool, '/providesystem/hmi_resume', qos_default),
            '/providesystem/reset_faults':         self.create_publisher(String, '/providesystem/reset_faults', qos_default),
            '/system/start_button':                self.create_publisher(Bool, '/system/start_button', qos_default),
            '/system/stop_button':                 self.create_publisher(Bool, '/system/stop_button', qos_default),
            '/system/pause_button':                self.create_publisher(Bool, '/system/pause_button', qos_default),
            '/system/confirm_button':              self.create_publisher(Bool, '/system/confirm_button', qos_default),
        }

        self._config_data = None
        self.create_subscription(String, '/providesystem/config_data', self._on_config, qos_default)
        self._system_state = 'UNKNOWN'
        self.create_subscription(String, '/system_state', self._on_state, qos_default)
        self._notifications = []
        self.create_subscription(String, '/providesystem/gui_notify', self._on_notify, qos_default)
        self._servo_positions = {}
        self.create_subscription(String, '/providesystem/servo_positions', self._on_positions, qos_default)
        self.get_logger().info('GUI ROS2 node initialized')

    def _on_config(self, msg):    self._config_data = msg.data
    def _on_state(self, msg):     self._system_state = msg.data
    def _on_positions(self, msg):
        try: self._servo_positions = json.loads(msg.data)
        except: pass

    def _on_notify(self, msg):
        import time as _t
        try:
            data = json.loads(msg.data)
            data['time'] = _t.strftime('%H:%M:%S')
            self._notifications.append(data)
            if len(self._notifications) > 20:
                self._notifications = self._notifications[-20:]
        except: pass

    def get_notifications(self, since_idx=0):
        return self._notifications[since_idx:]

    def pub_string(self, topic, data):
        pub = self._pubs.get(topic)
        if pub:
            msg = String(); msg.data = data; pub.publish(msg)
            return {"ok": True}
        return {"ok": False, "err": f"Unknown topic: {topic}"}

    def pub_bool(self, topic, data):
        pub = self._pubs.get(topic)
        if pub:
            msg = Bool(); msg.data = data; pub.publish(msg)
            return {"ok": True}
        return {"ok": False, "err": f"Unknown topic: {topic}"}

_ros_node = None
def get_ros_node(): return _ros_node

def init_ros():
    global _ros_node
    rclpy.init()
    _ros_node = GuiRosNode()
    t = threading.Thread(target=lambda: rclpy.spin(_ros_node), daemon=True)
    t.start()
    time.sleep(0.3)

def fast_pub_string(topic, data):
    node = get_ros_node()
    return node.pub_string(topic, data) if node else {"ok": False, "err": "ROS2 not init"}

def fast_pub_bool(topic, data):
    node = get_ros_node()
    return node.pub_bool(topic, data) if node else {"ok": False, "err": "ROS2 not init"}

# ============================================================================
# HTTP SERVER
# ============================================================================

class GUIHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args): pass

    # ----------------------------------------------------------------
    # [FIX-GUI-1] Token authentication
    # ----------------------------------------------------------------
    def _is_authenticated(self) -> bool:
        token = self.headers.get("X-Auth-Token", "")
        if secrets.compare_digest(token, AUTH_TOKEN):
            return True
        qs = parse_qs(urlparse(self.path).query)
        token_param = qs.get("token", [""])[0]
        return secrets.compare_digest(token_param, AUTH_TOKEN)

    def _send_security_headers(self):
        """[FIX-GUI-2] Security headers on every response."""
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy",
                         "default-src 'self' 'unsafe-inline' https://fonts.googleapis.com "
                         "https://fonts.gstatic.com; connect-src 'self'")
        self.send_header("Referrer-Policy", "no-referrer")

    def _send_unauthorized(self):
        self.send_response(401)
        self.send_header("Content-Type", "application/json")
        self.send_header("WWW-Authenticate", 'Bearer realm="CartridgeGUI"')
        self._send_security_headers()
        self.end_headers()
        self.wfile.write(json.dumps({"ok": False, "err": "Unauthorized"}).encode())

    def do_GET(self):
        if not self._is_authenticated():  # [FIX-GUI-1]
            self._send_unauthorized()
            return
        p = urlparse(self.path).path
        if p in ('/', '/index.html'): self._html(HTML)
        elif p == '/api/status':      self._json(self._get_status())
        elif p == '/api/config':      self._json(self._get_config())
        else: self.send_error(404)

    def do_POST(self):
        if not self._is_authenticated():  # [FIX-GUI-1]
            self._send_unauthorized()
            return

        # [FIX-GUI-3] Limit body to 64KB
        content_length = int(self.headers.get('Content-Length', 0))
        if content_length > 65536:
            self.send_response(413)
            self.end_headers()
            self.wfile.write(b'{"ok":false,"err":"Request too large"}')
            return

        body = self.rfile.read(content_length)
        try:
            d = json.loads(body.decode()) if body else {}
        except json.JSONDecodeError:
            self.send_response(400)
            self.end_headers()
            self.wfile.write(b'{"ok":false,"err":"Invalid JSON"}')
            return

        p = urlparse(self.path).path

        # [FIX-GUI-4] Validate config before forwarding
        if p == '/api/update_config':
            config_payload = d.get('config', {})
            ok, err_msg = validate_config(config_payload)
            if not ok:
                self._json({"ok": False, "err": f"Validation: {err_msg}"})
                return

        routes = {
            '/api/start':          lambda: fast_pub_bool('/system/start_button', True),
            '/api/stop':           lambda: fast_pub_bool('/system/stop_button', True),
            '/api/pause':          lambda: fast_pub_bool('/system/pause_button', True),
            '/api/confirm':        lambda: fast_pub_bool('/system/confirm_button', True),
            '/api/hmi_resume':     lambda: fast_pub_bool('/providesystem/hmi_resume', True),
            '/api/goto_state':     lambda: fast_pub_string('/providesystem/goto_state', d.get('state','IDLE')),
            '/api/sim_sensor':     lambda: fast_pub_string('/providesystem/sim_sensor', d.get('cmd','status')),
            '/api/jog':            lambda: fast_pub_string('/providesystem/jog_cmd', d.get('cmd','1 stop')),
            '/api/set_mode':       lambda: fast_pub_string('/providesystem/set_operation_mode', d.get('mode','auto')),
            '/api/set_target_row': lambda: fast_pub_string('/providesystem/set_target_row', str(d.get('row','1'))),
            '/api/move_servo':     lambda: fast_pub_string('/providesystem/move_to_pos', d.get('cmd','')),
            '/api/update_config':  lambda: fast_pub_string('/providesystem/update_config',
                                                           json.dumps(d.get('config',{}))),
            '/api/get_config':     lambda: fast_pub_string('/providesystem/get_config', 'request'),
            '/api/reset_faults':   lambda: fast_pub_string('/providesystem/reset_faults', 'reset'),
            '/api/notifications':  lambda: self._get_notifications(d),
            '/api/positions':      lambda: self._get_positions(),
        }
        handler = routes.get(p)
        if handler: self._json(handler())
        else: self.send_error(404)

    def _html(self, c):
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Cache-Control', 'no-cache')
        self._send_security_headers()  # [FIX-GUI-2]
        self.end_headers()
        self.wfile.write(c.encode())

    def _json(self, data):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self._send_security_headers()  # [FIX-GUI-2]
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def _get_status(self):
        node = get_ros_node()
        return {"state": node._system_state} if node else {"state": "UNKNOWN"}

    def _get_config(self):
        node = get_ros_node()
        if node and node._config_data:
            try: return {"ok": True, "config": json.loads(node._config_data)}
            except: pass
        return {"ok": False, "config": {}}

    def _get_notifications(self, d):
        node = get_ros_node()
        if node:
            since = d.get('since', 0)
            return {"ok": True, "notifications": node.get_notifications(since),
                    "total": len(node._notifications)}
        return {"ok": True, "notifications": [], "total": 0}

    def _get_positions(self):
        node = get_ros_node()
        return {"ok": True, "positions": node._servo_positions} if node else {"ok": True, "positions": {}}

# ============================================================================
# HTML — matches QML CartridgePage.qml layout
# ============================================================================

HTML = r"""<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Cartridge System Control</title>
<link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;600;700&display=swap" rel="stylesheet">
<style>
:root{--bg:#0c0c1d;--bg2:#1a1a35;--card:#141428;--border:#2a2a50;--accent:#4f6cff;
--green:#00e676;--red:#ff5252;--orange:#ffa726;--yellow:#ffd740;
--cyan:#26c6da;--text:#e8e8f0;--dim:#8888aa;--purple:#bb86fc;}
*{margin:0;padding:0;box-sizing:border-box;}
body{font-family:'JetBrains Mono',monospace;background:var(--bg);color:var(--text);height:100vh;overflow:hidden;font-size:12px;}
::-webkit-scrollbar{width:5px;}::-webkit-scrollbar-track{background:var(--bg2);}::-webkit-scrollbar-thumb{background:var(--accent);border-radius:3px;}

.hdr{background:var(--card);border-bottom:1px solid var(--border);padding:0 12px;height:44px;display:flex;align-items:center;justify-content:space-between;}
.hdr h1{font-size:18px;font-weight:700;letter-spacing:1px;}.hdr h1 span{color:var(--accent);}
.hdr-right{display:flex;align-items:center;gap:10px;}
.state-badge{background:var(--bg);border:1px solid var(--border);border-radius:8px;padding:4px 12px;font-size:14px;font-weight:600;display:flex;align-items:center;gap:8px;}
.state-dot{width:9px;height:9px;border-radius:50%;background:#666;animation:pulse 2s infinite;}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.35}}
.mode-pill{padding:3px 13px;border-radius:20px;font-size:13px;font-weight:700;text-transform:uppercase;letter-spacing:1px;border:1px solid;}
.mode-auto{background:#0a332e;border-color:var(--green);color:var(--green);}
.mode-jog{background:#332e0a;border-color:var(--orange);color:var(--orange);}
.mode-manual{background:#1a0a33;border-color:var(--purple);color:var(--purple);}
.mode-idle{background:#1a1a2e;border-color:var(--dim);color:var(--dim);}

.tab-bar{display:flex;gap:2px;background:var(--card);border-bottom:1px solid var(--border);padding:3px 16px 0;height:32px;}
.tab{padding:3px 18px;font-size:14px;font-weight:600;border-radius:6px 6px 0 0;cursor:pointer;border:1px solid transparent;border-bottom:none;color:var(--dim);transition:all .15s;background:transparent;}
.tab:hover{color:var(--text);background:var(--card);}.tab.active{color:var(--accent);background:var(--card);border-color:var(--border);}
.page{display:none;height:calc(100vh - 76px);}.page.active{display:block;}

.page-grid{display:grid;grid-template-columns:210px 1fr 250px;grid-template-rows:3fr 2.5fr;grid-template-areas:"ctrl center sensor" "log log sensor";gap:4px;padding:6px;height:100%;overflow:hidden;}
.ctrl-col{grid-area:ctrl;overflow:hidden;display:flex;flex-direction:column;gap:4px;}
.center-col{grid-area:center;overflow:hidden;display:flex;flex-direction:column;gap:4px;}
.sensor-col{grid-area:sensor;overflow-y:auto;}
.log-area{grid-area:log;}

.card{background:var(--bg2);border:1px solid var(--border);border-radius:6px;padding:8px;transition:border-color .15s;}
.card:hover{border-color:var(--accent);}
.card-title{font-size:11px;font-weight:700;color:var(--accent);text-transform:uppercase;letter-spacing:1.5px;margin-bottom:5px;}

.btn{padding:5px 12px;border:1px solid var(--border);border-radius:4px;background:var(--card);color:var(--text);cursor:pointer;font-family:inherit;font-size:12px;font-weight:600;transition:all .12s;text-transform:uppercase;letter-spacing:.4px;white-space:nowrap;}
.btn:hover{border-color:var(--accent);background:#222248;}.btn:active{transform:scale(.95);}
.btn-green{background:#0a332e;border-color:var(--green);color:var(--green);}
.btn-red{background:#4d1a1a;border-color:var(--red);color:var(--red);}
.btn-orange{background:#4d3a0a;border-color:var(--orange);color:var(--orange);}
.btn-purple{background:#1a0a33;border-color:var(--purple);color:var(--purple);}
.btn-accent{background:var(--accent);border-color:var(--accent);color:#fff;}
.btn-blue{background:#1a2050;border-color:var(--accent);color:var(--accent);}
.bgrid{display:grid;gap:4px;margin-bottom:4px;}.bgrid-2x2{grid-template-columns:1fr 1fr;}.bgrid-3{grid-template-columns:1fr 1fr 1fr;}.bgrid-2{grid-template-columns:1fr 1fr;}
.bgrid .btn{width:100%;padding:8px 4px;}
.btn.sel{opacity:1;box-shadow:inset 0 0 0 1px rgba(255,255,255,.15);}.btn.dim{opacity:.45;}

.vel-in{background:var(--bg);border:1px solid var(--border);border-radius:6px;padding:3px 6px;width:50px;color:var(--text);font-size:11px;font-family:inherit;text-align:center;}
.vel-in:focus{outline:none;border-color:var(--accent);}
#servoList{display:flex;gap:4px;flex:1;min-height:0;overflow:hidden;}
.scard{flex:1;background:var(--card);border:1px solid var(--border);border-radius:4px;padding:6px;display:flex;flex-direction:column;align-items:center;gap:6px;overflow:hidden;min-width:0;transition:border-color .15s;}
.scard:hover{border-color:var(--accent);}
.scard-name{font-size:14px;font-weight:700;color:var(--cyan);text-align:center;}
.scard-desc{font-size:11px;color:var(--dim);text-align:center;}
.scard-pos{font-size:22px;font-weight:700;color:var(--yellow);text-align:center;}
.jrow{display:flex;gap:4px;width:100%;}.jrow .btn{flex:1;font-size:18px;font-weight:700;padding:10px 2px;}
.sfull{width:100%;}.sfull .btn{width:100%;font-size:16px;padding:12px 4px;}
.prow{display:flex;gap:4px;width:100%;align-items:center;}
.pin{flex:1;background:var(--bg);border:1px solid var(--border);border-radius:6px;padding:6px 4px;color:var(--text);font-size:15px;font-family:inherit;text-align:center;min-width:0;height:34px;}
.pin:focus{outline:none;border-color:var(--accent);}

.sensor-card{height:100%;display:flex;flex-direction:column;}
.stb{display:flex;gap:3px;margin-bottom:5px;flex-shrink:0;}.stb .btn{flex:1;padding:3px 8px;font-size:10px;}
.sgrid{display:grid;grid-template-columns:repeat(2,1fr);gap:4px;flex:1;align-content:start;}
.sbtn{display:flex;flex-direction:column;align-items:center;justify-content:center;padding:5px 3px;border-radius:4px;border:1px solid var(--border);background:var(--card);cursor:pointer;transition:all .15s;min-height:30px;}
.sbtn:hover{border-color:var(--cyan);}.sbtn.on{background:#0a332e;border-color:var(--green);}
.sbtn .sid{font-size:11px;font-weight:700;color:var(--text);}.sbtn.on .sid{color:var(--green);}
.sbtn .slb{font-size:8px;color:var(--dim);}
.sbtn .sdot{width:6px;height:6px;border-radius:50%;background:#333;margin-top:2px;transition:all .15s;}
.sbtn.on .sdot{background:var(--green);box-shadow:0 0 6px var(--green);}
.sleg{font-size:8px;color:var(--dim);line-height:1.5;margin-top:5px;flex-shrink:0;}

.log-hdr{display:flex;justify-content:space-between;align-items:center;margin-bottom:4px;}
.log-box{background:#0a0a18;border:1px solid var(--border);border-radius:4px;padding:6px 8px;height:calc(100% - 28px);overflow-y:auto;font-size:13px;font-family:monospace;line-height:1.5;}
.log-box .ok{color:var(--green);}.log-box .err{color:var(--red);}.log-box .info{color:var(--cyan);}

.notify-bar{display:none;padding:7px 16px;font-size:11px;font-weight:500;align-items:center;gap:10px;border-bottom:1px solid;}
.notify-bar.show{display:flex;}
.notify-bar.error{background:linear-gradient(90deg,#3a0a0a,#2a0808);border-color:var(--red);color:var(--red);}
.notify-bar.warn{background:linear-gradient(90deg,#3a2e0a,#2a2008);border-color:var(--orange);color:var(--orange);}
.notify-bar.info{background:linear-gradient(90deg,#0a2a33,#081e28);border-color:var(--cyan);color:var(--cyan);}
.notify-bar .ntitle{font-weight:700;white-space:nowrap;}.notify-bar .ndetail{flex:1;color:var(--text);opacity:.9;}
.notify-bar .ntime{font-size:10px;color:var(--dim);}.notify-bar .nact{display:flex;gap:6px;flex-shrink:0;}

.toast-ct{position:fixed;bottom:14px;right:14px;z-index:999;}
.toast{background:var(--card);border:1px solid var(--green);border-radius:6px;padding:7px 12px;margin-top:5px;font-size:11px;animation:si .25s;max-width:280px;}
.toast.error{border-color:var(--red);}
@keyframes si{from{transform:translateX(80px);opacity:0}to{transform:translateX(0);opacity:1}}

.config-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:14px;padding:14px;overflow-y:auto;height:100%;}
.config-table{width:100%;border-collapse:collapse;}
.config-table th{font-size:10px;text-transform:uppercase;color:var(--dim);padding:5px 8px;border-bottom:1px solid var(--border);text-align:left;}
.config-table td{padding:3px 8px;border-bottom:1px solid #1e1e3a;}
.config-table input{background:var(--bg);border:1px solid var(--border);border-radius:4px;padding:3px 5px;width:90px;color:var(--yellow);font-size:11px;font-family:monospace;text-align:right;}
.config-table input:focus{outline:none;border-color:var(--accent);}
.config-table .rl{font-size:11px;font-weight:600;color:var(--cyan);}
</style>
</head>
<body>

<div class="hdr">
  <h1><span>Cartridge</span> System</h1>
  <div class="hdr-right">
    <div class="state-badge"><div class="state-dot" id="stateDot"></div><span id="stateText">UNKNOWN</span></div>
    <div class="mode-pill mode-idle" id="modePill">IDLE</div>
  </div>
</div>

<div class="notify-bar" id="notifyBar">
  <span class="ntitle" id="notifyTitle"></span><span class="ndetail" id="notifyDetail"></span><span class="ntime" id="notifyTime"></span>
  <div class="nact">
    <button class="btn btn-orange" style="padding:2px 8px;font-size:10px" onclick="resetFaults()">Reset</button>
    <button class="btn" style="padding:2px 8px;font-size:10px" onclick="dismissNotify()">✕</button>
  </div>
</div>

<div class="tab-bar">
  <div class="tab active" onclick="switchTab('control',this)">Control Dashboard</div>
  <div class="tab" onclick="switchTab('config',this)">Technical System</div>
</div>

<div class="page active" id="page-control">
<div class="page-grid">

  <div class="ctrl-col">
    <div class="card" style="flex:1"><div class="card-title">Mode Selection</div>
      <div class="bgrid bgrid-2x2">
        <button class="btn btn-green" id="btnAuto" onclick="setMode('auto')">AUTO</button>
        <button class="btn btn-orange" id="btnJog" onclick="setMode('jog')">JOG</button>
        <button class="btn btn-purple" id="btnManual" onclick="setMode('manual')">MANUAL</button>
        <button class="btn" id="btnIdle" onclick="setMode('idle')">IDLE</button>
      </div>
    </div>
    <div class="card" style="flex:1"><div class="card-title">System Control</div>
      <div class="bgrid bgrid-3">
        <button class="btn btn-green" onclick="startSystem()">START</button>
        <button class="btn btn-red" onclick="stopSystem()">STOP</button>
        <button class="btn btn-orange" onclick="pauseSystem()">PAUSE</button>
      </div>
      <div class="bgrid bgrid-2">
        <button class="btn btn-blue" onclick="confirmButton()">Confirm</button>
        <button class="btn btn-green" onclick="hmiResume()">Resume</button>
      </div>
    </div>
    <div class="card" style="flex:1"><div class="card-title">State Navigation</div>
      <div class="bgrid bgrid-2x2">
        <button class="btn" onclick="gotoState('HOMING')">HOMING</button>
        <button class="btn" onclick="gotoState('IDLE')">IDLE</button>
        <button class="btn btn-blue" onclick="gotoState('STATE1')">STATE 1</button>
        <button class="btn btn-blue" onclick="gotoState('STATE2')">STATE 2</button>
        <button class="btn btn-blue" onclick="gotoState('STATE3')">STATE 3</button>
        <button class="btn btn-red" onclick="gotoState('ERROR')">ERROR</button>
      </div>
    </div>
  </div>

  <div class="center-col">
    <div class="card" style="flex-shrink:0"><div class="card-title">Target Row</div><div style="display:flex;gap:4px" id="targetRowBar"></div></div>
    <div class="card" style="flex:1;display:flex;flex-direction:column;min-height:0;overflow:hidden">
      <div style="display:flex;align-items:center;gap:6px;margin-bottom:5px;flex-shrink:0">
        <span class="card-title" style="margin-bottom:0">Servo Control</span>
        <span style="font-size:10px;color:var(--dim)">Vel:</span>
        <input type="number" class="vel-in" id="jogVel" value="30" min="1" max="200">
        <span style="font-size:10px;color:var(--dim)">mm/s</span>
      </div>
      <div id="servoList"></div>
    </div>
  </div>

  <div class="card log-area">
    <div class="log-hdr"><div class="card-title" style="margin-bottom:0">Log Activity</div>
    <button class="btn" style="padding:4px 10px;font-size:11px" onclick="document.getElementById('logBox').innerHTML=''">Clear</button></div>
    <div class="log-box" id="logBox"></div>
  </div>

  <div class="sensor-col"><div class="card sensor-card">
    <div class="card-title">Sensor Simulation</div>
    <div class="stb">
      <button class="btn btn-green" onclick="simAll(1)">All ON</button>
      <button class="btn btn-red" onclick="simAll(0)">All OFF</button>
      <button class="btn" onclick="simClear()">Clear</button>
    </div>
    <div style="font-size:10px;font-weight:700;color:var(--dim);text-transform:uppercase;letter-spacing:1px;margin-bottom:5px">STATUS</div>
    <div class="sgrid" id="sensorGrid"></div>
    <div class="sleg"><b>S1-S3</b> Stack · <b>S4</b> Detect · <b>S6</b> Platform · <b>S7</b> Output<br><b>S10</b> Cyl1 Ret · <b>S11</b> Cyl1 Ext · <b>S12</b> Cyl2 Ret · <b>S13</b> Cyl2 Ext</div>
  </div></div>

</div></div>

<div class="page" id="page-config"><div class="config-grid">
  <div class="card"><div class="card-title">Pos 1: Input Stack (InY)</div>
    <table class="config-table"><tr><th>Row</th><th>Position (mm)</th><th>Mô tả</th></tr></table>
    <table class="config-table" id="tblInputStack"></table>
    <div style="margin-top:8px;display:flex;gap:6px">
      <button class="btn btn-green" style="padding:4px 10px;font-size:11px" onclick="saveConfig('iny_input_stack','tblInputStack')">Save</button>
      <button class="btn" style="padding:4px 10px;font-size:11px" onclick="loadConfig()">↻ Reload</button></div></div>
  <div class="card"><div class="card-title">Pos 1: Output Stack (InY)</div>
    <table class="config-table"><tr><th>Row</th><th>Position (mm)</th><th>Mô tả</th></tr></table>
    <table class="config-table" id="tblOutputStack"></table>
    <div style="margin-top:8px;display:flex;gap:6px">
      <button class="btn btn-green" style="padding:4px 10px;font-size:11px" onclick="saveConfig('iny_output_stack','tblOutputStack')">Save</button>
      <button class="btn" style="padding:4px 10px;font-size:11px" onclick="loadConfig()">↻ Reload</button></div></div>
  <div class="card"><div class="card-title">Pos 2: Output Table (OutY)</div>
    <table class="config-table"><tr><th>Row</th><th>Position (mm)</th><th>Mô tả</th></tr></table>
    <table class="config-table" id="tblOutputTable"></table>
    <div style="margin-top:8px;display:flex;gap:6px">
      <button class="btn btn-green" style="padding:4px 10px;font-size:11px" onclick="saveConfig('outy_output_table','tblOutputTable')">Save</button>
      <button class="btn" style="padding:4px 10px;font-size:11px" onclick="loadConfig()">↻ Reload</button></div></div>
  <div class="card"><div class="card-title">Servo Key Positions (mm)</div>
    <table class="config-table" id="tblServoPositions"><tr><th>Parameter</th><th>Value (mm)</th><th>Description</th></tr></table></div>
</div></div>

<div class="toast-ct" id="toasts"></div>

<script>
const SS={};for(let i=1;i<=15;i++)SS[i]=false;
let currentMode='idle',currentState='unknown';
const SERVOS=[{id:1,name:'InX',desc:'Trục X đầu vào'},{id:2,name:'InY',desc:'Trục Y đầu vào'},{id:3,name:'PutTray',desc:'Đẩy khay'},{id:4,name:'OutX',desc:'Trục X đầu ra'},{id:5,name:'OutY',desc:'Trục Y đầu ra'}];
const SLB={1:'Stack',2:'Stack',3:'Stack',4:'Detect',5:'',6:'Platform',7:'Output',8:'',9:'Safety',10:'Cyl1 Retract',11:'Cyl1 Extend',12:'Cyl2 Retract',13:'Cyl2 Extend',14:'',15:''};

function switchTab(n,el){document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));document.querySelectorAll('.page').forEach(p=>p.classList.remove('active'));document.getElementById('page-'+n).classList.add('active');el.classList.add('active');if(n==='config')loadConfig();}

function setMode(mode){const s=currentState;if(s&&s!=='idle'&&s!=='unknown'&&s!=='error'){toast('⚠️ Stop system first','error');return;}api('/api/set_mode',{mode});currentMode=mode;updateModeUI();}
function updateModeUI(){const p=document.getElementById('modePill');p.className='mode-pill mode-'+currentMode;p.textContent=currentMode.toUpperCase();
['Auto','Jog','Manual','Idle'].forEach(m=>{const b=document.getElementById('btn'+m);if(b){b.classList.toggle('sel',currentMode===m.toLowerCase());b.classList.toggle('dim',currentMode!==m.toLowerCase());}});}

async function api(ep,data={}){try{const r=await fetch(ep,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});const j=await r.json();const l=ep.split('/').pop().replace(/_/g,' ');if(j.ok)toast('✅ '+l);else toast('❌ '+(j.err||'Error'),'error');addLog(l+': '+(j.ok?'OK':j.err),j.ok?'ok':'err');return j;}catch(e){toast('❌ Connection error','error');return{ok:false};}}

function startSystem(){api('/api/start');}
function stopSystem(){api('/api/stop');}
function pauseSystem(){api('/api/pause');}
function confirmButton(){api('/api/confirm');}
function hmiResume(){api('/api/hmi_resume');}
function resetFaults(){api('/api/reset_faults');addLog('Reset faults','info');}
function gotoState(s){api('/api/goto_state',{state:s});addLog('Goto: '+s,'ok');}
function setTargetRow(r){api('/api/set_target_row',{row:r});}

function toggleSensor(id){SS[id]=!SS[id];api('/api/sim_sensor',{cmd:id+':'+(SS[id]?1:0)});refreshSensor(id);}
function simAll(v){for(let i=1;i<=15;i++){SS[i]=!!v;refreshSensor(i);}api('/api/sim_sensor',{cmd:'all:'+v});}
function simClear(){for(let i=1;i<=15;i++){SS[i]=false;refreshSensor(i);}api('/api/sim_sensor',{cmd:'clear'});}
function refreshSensor(id){const el=document.getElementById('s'+id);if(el)el.className='sbtn'+(SS[id]?' on':'');}

function jog(id,dir){const v=document.getElementById('jogVel').value||30;fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:id+' '+dir+' '+v})});}
function jogStop(id){fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:id+' stop'})});}
function homeServo(id){fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:'home '+id})});addLog('Home S'+id,'info');}
function clearServo(id){fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:'clear '+id})});addLog('Clear S'+id,'info');}
function moveServo(id){const inp=document.getElementById('pi'+id);const pos=parseFloat(inp?.value);if(isNaN(pos)){toast('❌ Invalid','error');return;}api('/api/move_servo',{cmd:id+':'+pos});}

async function loadConfig(){await api('/api/get_config');await new Promise(r=>setTimeout(r,500));const res=await fetch('/api/config');const j=await res.json();if(!j.ok||!j.config)return;const c=j.config;if(c.iny_input_stack)buildConfigTable('tblInputStack',c.iny_input_stack);if(c.iny_output_stack)buildConfigTable('tblOutputStack',c.iny_output_stack);if(c.outy_output_table)buildConfigTable('tblOutputTable',c.outy_output_table);if(c.operation_mode){currentMode=c.operation_mode;updateModeUI();}buildServoInfoTable(c);}
function buildConfigTable(id,data){const tbl=document.getElementById(id);tbl.innerHTML='';for(let r=8;r>=1;r--){const val=data[r]||data[String(r)]||0;const tr=document.createElement('tr');tr.innerHTML=`<td class="rl">Row ${r}</td><td><input type="number" step="0.1" value="${val}" id="${id}_r${r}"></td><td style="font-size:10px;color:var(--dim)">${r===8?'Top (gần home)':r===1?'Bottom (xa home)':''}</td>`;tbl.appendChild(tr);}}
function buildServoInfoTable(c){const tbl=document.getElementById('tblServoPositions');tbl.innerHTML='<tr><th>Parameter</th><th>Value (mm)</th><th>Description</th></tr>';[['InX Home',c.inx_home,'Servo 1 home'],['InX Target',c.inx_target2,'Servo 1 lấy khay'],['InX OutStack',c.inx_output_stack,'Đặt khay đã dùng'],['InY Home',c.iny_home,'Servo 2 home'],['InY Target',c.iny_target2,'Robot place'],['InY SafeZone',c.iny_safe_zone,'INY safe zone'],['Servo3 Push',c.servo3_push_position,'Push pos'],['OutX Home',c.outx_home,'Servo 4 home'],['OutX Target',c.outx_target2,'Output stack'],['OutX Robot',c.outx_target3,'Robot tray'],['OutY Home',c.outy_home,'Servo 5 home'],['OutY Target',c.outy_target2,'Pick/place'],['OutY SafeZone',c.outy_safe_zone,'OUTY safe zone']].forEach(([n,v,d])=>{if(v===undefined)return;const tr=document.createElement('tr');tr.innerHTML=`<td class="rl">${n}</td><td style="color:var(--yellow);font-family:monospace">${v}</td><td style="font-size:10px;color:var(--dim)">${d}</td>`;tbl.appendChild(tr);});}
async function saveConfig(table,tblId){const pos={};for(let r=1;r<=8;r++){const inp=document.getElementById(tblId+'_r'+r);if(inp)pos[r]=parseFloat(inp.value)||0;}await api('/api/update_config',{config:{table,positions:pos}});}

function buildTargetRow(){const bar=document.getElementById('targetRowBar');for(let r=1;r<=8;r++){const b=document.createElement('button');b.className='btn';b.style.cssText='flex:1;padding:4px 10px;font-size:11px';b.textContent='R'+r;b.onclick=()=>setTargetRow(r);bar.appendChild(b);}}
function buildSensors(){const g=document.getElementById('sensorGrid');for(let i=1;i<=15;i++){const d=document.createElement('div');d.className='sbtn';d.id='s'+i;d.onclick=()=>toggleSensor(i);const lb=SLB[i];d.innerHTML=`<span class="sid">S${i}</span>${lb?`<span class="slb">${lb}</span>`:''}<span class="sdot"></span>`;g.appendChild(d);}}
function buildServos(){const list=document.getElementById('servoList');SERVOS.forEach(s=>{const d=document.createElement('div');d.className='scard';d.innerHTML=`<div style="text-align:center"><div class="scard-name">S${s.id}: ${s.name}</div><div class="scard-desc">${s.desc}</div></div><div class="scard-pos" id="spos${s.id}">-- mm</div><div class="jrow"><button class="btn" onmousedown="jog(${s.id},'-')" onmouseup="jogStop(${s.id})" onmouseleave="jogStop(${s.id})" ontouchstart="jog(${s.id},'-')" ontouchend="jogStop(${s.id})">−</button><button class="btn btn-red" onclick="jogStop(${s.id})" style="font-size:14px">STOP</button><button class="btn" onmousedown="jog(${s.id},'+')" onmouseup="jogStop(${s.id})" onmouseleave="jogStop(${s.id})" ontouchstart="jog(${s.id},'+')" ontouchend="jogStop(${s.id})">+</button></div><div class="sfull"><button class="btn btn-green" onclick="homeServo(${s.id})">HOMING</button></div><div class="sfull"><button class="btn btn-orange" onclick="clearServo(${s.id})">CLEAR</button></div><div class="prow"><input type="number" class="pin" id="pi${s.id}" placeholder="0.0" step="0.1"><span style="font-size:9px;color:var(--dim)">mm</span><button class="btn btn-accent" onclick="moveServo(${s.id})" style="padding:6px 14px;font-size:16px">RUN</button></div>`;list.appendChild(d);});}

function toast(msg,type='ok'){const c=document.getElementById('toasts');const t=document.createElement('div');t.className='toast'+(type==='error'?' error':'');t.textContent=msg;c.appendChild(t);setTimeout(()=>t.remove(),3000);}
function addLog(msg,type='info'){const box=document.getElementById('logBox');const time=new Date().toLocaleTimeString('vi-VN');box.innerHTML+=`<div class="${type}">[${time}] ${msg}</div>`;box.scrollTop=box.scrollHeight;while(box.children.length>100)box.removeChild(box.firstChild);}

async function pollStatus(){try{const j=await(await fetch('/api/status')).json();const st=j.state||'';currentState=st.toLowerCase();document.getElementById('stateText').textContent=st||'UNKNOWN';const dot=document.getElementById('stateDot');if(!st||st==='UNKNOWN'){dot.style.background='#666';dot.style.boxShadow='none';}else if(st.includes('ERROR')){dot.style.background='var(--red)';dot.style.boxShadow='0 0 8px var(--red)';}else if(st==='IDLE'){dot.style.background='var(--orange)';dot.style.boxShadow='0 0 8px var(--orange)';}else{dot.style.background='var(--green)';dot.style.boxShadow='0 0 8px var(--green)';}}catch(e){document.getElementById('stateText').textContent='OFFLINE';}}
async function pollPositions(){try{const j=await(await fetch('/api/positions',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'})).json();if(j.ok&&j.positions){for(let id=1;id<=5;id++){const el=document.getElementById('spos'+id);if(el){const v=j.positions[String(id)];el.textContent=v!==undefined?Number(v).toFixed(1)+' mm':'-- mm';}}}}catch(e){}}
let _ni=0,_nt=null;
async function pollNotifications(){try{const j=await(await fetch('/api/notifications',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({since:_ni})})).json();if(j.ok&&j.notifications&&j.notifications.length>0){_ni=j.total;const last=j.notifications[j.notifications.length-1];showNotify(last.level,last.title,last.detail,last.time);j.notifications.forEach(n=>addLog(n.title+(n.detail?': '+n.detail:''),n.level==='error'?'err':'info'));}}catch(e){}}
function showNotify(lvl,title,detail,time){const bar=document.getElementById('notifyBar');bar.className='notify-bar show '+lvl;document.getElementById('notifyTitle').textContent=title;document.getElementById('notifyDetail').textContent=detail||'';document.getElementById('notifyTime').textContent=time||'';clearTimeout(_nt);if(lvl==='info')_nt=setTimeout(dismissNotify,5000);}
function dismissNotify(){document.getElementById('notifyBar').className='notify-bar';}

buildTargetRow();buildSensors();buildServos();updateModeUI();pollStatus();pollPositions();
setInterval(pollStatus,2000);setInterval(pollNotifications,1500);setInterval(pollPositions,500);
addLog('GUI Connected — Ready','ok');
</script>
</body>
</html>
"""

# ============================================================================
# MAIN
# ============================================================================

def main():
    import socket

    print("  Initializing ROS2 node...")
    init_ros()
    print("  ROS2 node ready — direct publishing enabled")

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
    except:
        local_ip = "localhost"

    http.server.HTTPServer.allow_reuse_address = True
    server = http.server.HTTPServer(('0.0.0.0', PORT), GUIHandler)

    def _handle_signal(signum, frame):
        print(f"\n  GUI received signal {signum} — exiting...")
        try: server.shutdown()
        except: pass
        try: rclpy.shutdown()
        except: pass
        os._exit(0)

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT,  _handle_signal)

    print("=" * 50)
    print("  Cartridge System GUI v3 (Direct ROS2) — PATCHED")
    print("=" * 50)
    print(f"  Local:   http://localhost:{PORT}?token={AUTH_TOKEN}")
    print(f"  Network: http://{local_ip}:{PORT}?token={AUTH_TOKEN}")
    print(f"  Token:   {AUTH_TOKEN}")
    print(f"  Đổi token: export GUI_AUTH_TOKEN=<mật_khẩu_mới>")
    print("=" * 50)
    print("  Press Ctrl+C to stop")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        print("\n  GUI Server stopped.")
        try: server.shutdown()
        except: pass
        try: rclpy.shutdown()
        except: pass

if __name__ == '__main__':
    main()