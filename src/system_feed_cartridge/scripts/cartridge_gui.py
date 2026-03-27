#!/usr/bin/env python3
"""
Cartridge System GUI — Web Dashboard v4
Mode redesign:
  AUTO   — Đọc sensor thực tế, tự động chạy workflow. JOG/Sim bị khóa.
  MANUAL — Cùng workflow/điều kiện như auto, thêm: JOG + sim sensor.
           Operator chọn STATE 1/2 để kích hoạt workflow.
"""

import http.server
import json
import secrets
import threading
import time
import os
import signal
from urllib.parse import urlparse, parse_qs

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Bool

PORT = 8080

_DEFAULT_TOKEN = "cartridge-gui-secret"
AUTH_TOKEN = os.environ.get("GUI_AUTH_TOKEN", _DEFAULT_TOKEN)

CONFIG_RANGES = {
    "inx_home":             (0.0,   200.0),
    "inx_safe_zone":        (0.0,   200.0),
    "inx_target":           (10.0,  700.0),
    "iny_home":             (0.0,   200.0),
    "iny_safe_zone":        (5.0,   200.0),
    "iny_place":            (10.0,  700.0),
    "servo3_push_position": (10.0,  400.0),
    "outx_home":            (0.0,   200.0),
    "outx_target":          (10.0,  700.0),
    "outy_home":            (0.0,   200.0),
    "outy_target":          (10.0,  700.0),
    "outy_safe_zone":       (5.0,   200.0),
}
ROW_POSITION_RANGE = (0.0, 960.0)


def validate_config(config_dict: dict) -> tuple:
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


class GuiRosNode(Node):
    def __init__(self):
        super().__init__('cartridge_gui')
        qos = QoSProfile(depth=10)

        self._pubs = {
            '/providesystem/jog_cmd':           self.create_publisher(String, '/providesystem/jog_cmd', qos),
            '/providesystem/sim_sensor':         self.create_publisher(String, '/providesystem/sim_sensor', qos),
            '/providesystem/move_to_pos':        self.create_publisher(String, '/providesystem/move_to_pos', qos),
            '/providesystem/update_config':      self.create_publisher(String, '/providesystem/update_config', qos),
            '/providesystem/get_config':         self.create_publisher(String, '/providesystem/get_config', qos),
            '/providesystem/set_operation_mode': self.create_publisher(String, '/providesystem/set_operation_mode', qos),
            '/providesystem/goto_state':         self.create_publisher(String, '/providesystem/goto_state', qos),
            '/providesystem/set_target_row':     self.create_publisher(String, '/providesystem/set_target_row', qos),
            '/providesystem/hmi_resume':         self.create_publisher(Bool,   '/providesystem/hmi_resume', qos),
            '/providesystem/reset_faults':       self.create_publisher(String, '/providesystem/reset_faults', qos),
            '/system/start_button':              self.create_publisher(Bool,   '/system/start_button', qos),
            '/system/stop_button':               self.create_publisher(Bool,   '/system/stop_button', qos),
            '/system/pause_button':              self.create_publisher(Bool,   '/system/pause_button', qos),
            '/system/confirm_button':            self.create_publisher(Bool,   '/system/confirm_button', qos),
        }

        self._config_data = None
        self._system_state = 'UNKNOWN'
        self._notifications = []
        self._servo_positions = {}

        self.create_subscription(String, '/providesystem/config_data',  self._on_config,    qos)
        self.create_subscription(String, '/system_state',               self._on_state,     qos)
        self.create_subscription(String, '/providesystem/gui_notify',   self._on_notify,    qos)
        self.create_subscription(String, '/providesystem/servo_positions', self._on_pos,    qos)

    def _on_config(self, msg):  self._config_data = msg.data
    def _on_state(self, msg):   self._system_state = msg.data
    def _on_pos(self, msg):
        try: self._servo_positions = json.loads(msg.data)
        except: pass

    def _on_notify(self, msg):
        try:
            data = json.loads(msg.data)
            data['time'] = time.strftime('%H:%M:%S')
            self._notifications.append(data)
            if len(self._notifications) > 30:
                self._notifications = self._notifications[-30:]
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
    threading.Thread(target=lambda: rclpy.spin(_ros_node), daemon=True).start()
    time.sleep(0.3)


def fast_pub_string(topic, data):
    node = get_ros_node()
    return node.pub_string(topic, data) if node else {"ok": False, "err": "ROS2 not init"}


def fast_pub_bool(topic, data):
    node = get_ros_node()
    return node.pub_bool(topic, data) if node else {"ok": False, "err": "ROS2 not init"}


class GUIHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args): pass

    def _auth(self) -> bool:
        token = self.headers.get("X-Auth-Token", "")
        if secrets.compare_digest(token, AUTH_TOKEN): return True
        qs = parse_qs(urlparse(self.path).query)
        return secrets.compare_digest(qs.get("token", [""])[0], AUTH_TOKEN)

    def _sec_headers(self):
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy",
                         "default-src 'self' 'unsafe-inline' https://fonts.googleapis.com "
                         "https://fonts.gstatic.com; connect-src 'self'")

    def _unauth(self):
        self.send_response(401)
        self.send_header("Content-Type", "application/json")
        self._sec_headers(); self.end_headers()
        self.wfile.write(b'{"ok":false,"err":"Unauthorized"}')

    def do_GET(self):
        if not self._auth(): self._unauth(); return
        p = urlparse(self.path).path
        if p in ('/', '/index.html'): self._html(HTML)
        elif p == '/api/status':      self._json(self._get_status())
        elif p == '/api/config':      self._json(self._get_config())
        else: self.send_error(404)

    def do_POST(self):
        if not self._auth(): self._unauth(); return
        cl = int(self.headers.get('Content-Length', 0))
        if cl > 65536:
            self.send_response(413); self.end_headers()
            self.wfile.write(b'{"ok":false,"err":"Too large"}'); return
        body = self.rfile.read(cl)
        try:
            d = json.loads(body.decode()) if body else {}
        except json.JSONDecodeError:
            self.send_response(400); self.end_headers()
            self.wfile.write(b'{"ok":false,"err":"Invalid JSON"}'); return
        p = urlparse(self.path).path
        if p == '/api/update_config':
            ok, err = validate_config(d.get('config', {}))
            if not ok:
                self._json({"ok": False, "err": f"Validation: {err}"}); return
        routes = {
            '/api/start':          lambda: fast_pub_bool('/system/start_button', True),
            '/api/stop':           lambda: fast_pub_bool('/system/stop_button', True),
            '/api/pause':          lambda: fast_pub_bool('/system/pause_button', True),
            '/api/confirm':        lambda: fast_pub_bool('/system/confirm_button', True),
            '/api/hmi_resume':     lambda: fast_pub_bool('/providesystem/hmi_resume', True),
            '/api/goto_state':     lambda: fast_pub_string('/providesystem/goto_state', d.get('state', 'IDLE')),
            '/api/sim_sensor':     lambda: fast_pub_string('/providesystem/sim_sensor', d.get('cmd', '')),
            '/api/jog':            lambda: fast_pub_string('/providesystem/jog_cmd', d.get('cmd', '1 stop')),
            '/api/set_mode':       lambda: fast_pub_string('/providesystem/set_operation_mode', d.get('mode', 'auto')),
            '/api/set_target_row': lambda: fast_pub_string('/providesystem/set_target_row', str(d.get('row', '1'))),
            '/api/move_servo':     lambda: fast_pub_string('/providesystem/move_to_pos', d.get('cmd', '')),
            '/api/update_config':  lambda: fast_pub_string('/providesystem/update_config', json.dumps(d.get('config', {}))),
            '/api/get_config':     lambda: fast_pub_string('/providesystem/get_config', 'request'),
            '/api/reset_faults':   lambda: fast_pub_string('/providesystem/reset_faults', 'reset'),
            '/api/notifications':  lambda: self._get_notifs(d),
            '/api/positions':      lambda: self._get_pos(),
        }
        h = routes.get(p)
        if h: self._json(h())
        else: self.send_error(404)

    def _html(self, c):
        self.send_response(200); self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Cache-Control', 'no-cache'); self._sec_headers(); self.end_headers()
        self.wfile.write(c.encode())

    def _json(self, data):
        self.send_response(200); self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*'); self._sec_headers(); self.end_headers()
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

    def _get_notifs(self, d):
        node = get_ros_node()
        if node:
            since = d.get('since', 0)
            return {"ok": True, "notifications": node.get_notifications(since),
                    "total": len(node._notifications)}
        return {"ok": True, "notifications": [], "total": 0}

    def _get_pos(self):
        node = get_ros_node()
        return {"ok": True, "positions": node._servo_positions} if node else {"ok": True, "positions": {}}


# ============================================================
# HTML
# ============================================================

HTML = r"""<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Cartridge System v4</title>
<link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;600;700&display=swap" rel="stylesheet">
<style>
:root{--bg:#0c0c1d;--bg2:#1a1a35;--card:#141428;--border:#2a2a50;--accent:#4f6cff;
  --green:#00e676;--red:#ff5252;--orange:#ffa726;--yellow:#ffd740;
  --cyan:#26c6da;--text:#e8e8f0;--dim:#8888aa;--purple:#bb86fc;}
*{margin:0;padding:0;box-sizing:border-box;}
body{font-family:'JetBrains Mono',monospace;background:var(--bg);color:var(--text);
  height:100vh;overflow:hidden;font-size:12px;}
::-webkit-scrollbar{width:5px;}
::-webkit-scrollbar-thumb{background:var(--accent);border-radius:3px;}

/* ── Header ── */
.hdr{background:var(--card);border-bottom:1px solid var(--border);padding:0 14px;
  height:46px;display:flex;align-items:center;justify-content:space-between;}
.hdr h1{font-size:17px;font-weight:700;letter-spacing:.5px;}
.hdr h1 span{color:var(--accent);}
.hdr-r{display:flex;align-items:center;gap:10px;}
.state-badge{background:var(--bg);border:1px solid var(--border);border-radius:8px;
  padding:4px 14px;font-size:14px;font-weight:700;display:flex;align-items:center;gap:8px;}
.sdot{width:9px;height:9px;border-radius:50%;background:#555;animation:pulse 2s infinite;}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
/* mode pill */
.mpill{padding:3px 14px;border-radius:20px;font-size:12px;font-weight:700;
  text-transform:uppercase;letter-spacing:1px;border:1px solid;
  display:flex;align-items:center;gap:6px;}
.m-auto{background:#0a2e22;border-color:var(--green);color:var(--green);}
.m-manual{background:#1a0a33;border-color:var(--purple);color:var(--purple);}

/* ── Notify bar ── */
.nbar{display:none;padding:6px 16px;font-size:11px;font-weight:500;
  align-items:center;gap:10px;border-bottom:1px solid;}
.nbar.show{display:flex;}
.nbar.error{background:#2a0808;border-color:var(--red);color:var(--red);}
.nbar.warn{background:#2a1e08;border-color:var(--orange);color:var(--orange);}
.nbar.info{background:#081e28;border-color:var(--cyan);color:var(--cyan);}
.nbar .ntit{font-weight:700;white-space:nowrap;}
.nbar .ndet{flex:1;color:var(--text);opacity:.85;}
.nbar .ntm{font-size:10px;color:var(--dim);}
.nbar .nact{display:flex;gap:5px;flex-shrink:0;}

/* ── Tabs ── */
.tabs{display:flex;gap:2px;background:var(--card);border-bottom:1px solid var(--border);
  padding:3px 16px 0;height:31px;}
.tab{padding:3px 18px;font-size:13px;font-weight:600;border-radius:5px 5px 0 0;
  cursor:pointer;border:1px solid transparent;border-bottom:none;color:var(--dim);}
.tab.active{color:var(--accent);background:var(--card);border-color:var(--border);}
.page{display:none;height:calc(100vh - 77px);}
.page.active{display:block;}

/* ── Grid ── */
.grid{display:grid;grid-template-columns:215px 1fr 252px;
  grid-template-rows:3fr 2.2fr;
  grid-template-areas:"lc cc sc" "log log sc";
  gap:4px;padding:5px;height:100%;overflow:hidden;}
.lc{grid-area:lc;display:flex;flex-direction:column;gap:4px;overflow:hidden;}
.cc{grid-area:cc;display:flex;flex-direction:column;gap:4px;overflow:hidden;}
.sc{grid-area:sc;overflow-y:auto;}
.la{grid-area:log;}

/* ── Card ── */
.card{background:var(--bg2);border:1px solid var(--border);border-radius:6px;padding:8px;}
.ct{font-size:10px;font-weight:700;color:var(--accent);text-transform:uppercase;
  letter-spacing:1.5px;margin-bottom:5px;}

/* ── Buttons ── */
.btn{padding:5px 10px;border:1px solid var(--border);border-radius:4px;background:var(--card);
  color:var(--text);cursor:pointer;font-family:inherit;font-size:12px;font-weight:600;
  transition:all .12s;text-transform:uppercase;letter-spacing:.3px;white-space:nowrap;}
.btn:hover{border-color:var(--accent);background:#222248;}
.btn:active{transform:scale(.95);}
.g{background:#0a2e22;border-color:var(--green);color:var(--green);}
.r{background:#3a1010;border-color:var(--red);color:var(--red);}
.o{background:#3a2a08;border-color:var(--orange);color:var(--orange);}
.pu{background:#1a0a33;border-color:var(--purple);color:var(--purple);}
.ac{background:var(--accent);border-color:var(--accent);color:#fff;}
.bl{background:#141e44;border-color:var(--accent);color:var(--accent);}
.gr2{display:grid;grid-template-columns:1fr 1fr;gap:4px;margin-bottom:4px;}
.gr3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:4px;margin-bottom:4px;}
.gr2 .btn,.gr3 .btn{width:100%;padding:8px 4px;}
.sel{box-shadow:inset 0 0 0 1.5px rgba(255,255,255,.18);}
/* locked state for auto mode */
.lk{opacity:.32;cursor:not-allowed !important;filter:grayscale(50%);}
.lk:hover{border-color:var(--border) !important;background:var(--card) !important;}
.lk:active{transform:none !important;}
/* lock notice */
.lkn{display:none;align-items:center;gap:6px;padding:4px 7px;
  background:var(--card);border:1px solid #333;border-radius:4px;
  font-size:10px;color:var(--dim);margin-top:3px;}
.lkn.show{display:flex;}

/* ── Servo cards ── */
.vel{background:var(--bg);border:1px solid var(--border);border-radius:5px;
  padding:3px 5px;width:50px;color:var(--text);font-size:11px;font-family:inherit;text-align:center;}
.vel:focus{outline:none;border-color:var(--accent);}
#svlist{display:flex;gap:4px;flex:1;min-height:0;overflow:hidden;}
.svc{flex:1;background:var(--card);border:1px solid var(--border);border-radius:4px;
  padding:6px;display:flex;flex-direction:column;align-items:center;gap:5px;
  overflow:hidden;min-width:0;}
.svn{font-size:14px;font-weight:700;color:var(--cyan);}
.svd{font-size:10px;color:var(--dim);}
.svp{font-size:20px;font-weight:700;color:var(--yellow);}
.jr{display:flex;gap:3px;width:100%;}
.jr .btn{flex:1;font-size:17px;font-weight:700;padding:9px 2px;}
.sfw{width:100%;}.sfw .btn{width:100%;font-size:15px;padding:10px 4px;}
.pr{display:flex;gap:3px;width:100%;align-items:center;}
.pi{flex:1;background:var(--bg);border:1px solid var(--border);border-radius:5px;
  padding:5px 3px;color:var(--text);font-size:14px;font-family:inherit;text-align:center;
  min-width:0;height:32px;}
.pi:focus{outline:none;border-color:var(--accent);}

/* ── Sensor panel ── */
.sgrid{display:grid;grid-template-columns:repeat(2,1fr);gap:4px;margin-top:4px;}
.sb{display:flex;flex-direction:column;align-items:center;justify-content:center;
  padding:5px 3px;border-radius:4px;border:1px solid var(--border);background:var(--card);
  cursor:pointer;min-height:30px;transition:all .12s;}
.sb:hover{border-color:var(--cyan);}
.sb.on{background:#0a2e22;border-color:var(--green);}
.sb.on .sn{color:var(--green);}
.sn{font-size:11px;font-weight:700;}
.slb{font-size:8px;color:var(--dim);}
.sdt{width:6px;height:6px;border-radius:50%;background:#2a2a2a;margin-top:2px;}
.sb.on .sdt{background:var(--green);box-shadow:0 0 5px var(--green);}
/* auto mode: sensor shows read-only badge */
.rs-badge{display:none;align-items:center;gap:5px;padding:4px 7px;
  background:#0a1f0a;border:1px solid var(--green);border-radius:4px;
  font-size:10px;color:var(--green);font-weight:700;margin-bottom:4px;}
.rs-badge.show{display:flex;}

/* ── Log ── */
.lh{display:flex;justify-content:space-between;align-items:center;margin-bottom:4px;}
.lb{background:#080816;border:1px solid var(--border);border-radius:4px;
  padding:6px 8px;height:calc(100% - 26px);overflow-y:auto;
  font-size:12px;font-family:monospace;line-height:1.5;}
.lb .ok{color:var(--green);}.lb .er{color:var(--red);}
.lb .in{color:var(--cyan);}.lb .wn{color:var(--orange);}

/* ── Toast ── */
.tc{position:fixed;bottom:12px;right:12px;z-index:999;}
.ts{background:var(--card);border:1px solid var(--green);border-radius:6px;
  padding:6px 12px;margin-top:4px;font-size:11px;animation:si .2s;max-width:270px;}
.ts.er{border-color:var(--red);}.ts.wn{border-color:var(--orange);}
@keyframes si{from{transform:translateX(70px);opacity:0}to{transform:none;opacity:1}}

/* ── Config tab ── */
.cgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));
  gap:12px;padding:12px;overflow-y:auto;height:100%;}
.ct2{width:100%;border-collapse:collapse;}
.ct2 th{font-size:10px;text-transform:uppercase;color:var(--dim);
  padding:4px 7px;border-bottom:1px solid var(--border);}
.ct2 td{padding:3px 7px;border-bottom:1px solid #1a1a30;}
.ct2 input{background:var(--bg);border:1px solid var(--border);border-radius:3px;
  padding:2px 4px;width:85px;color:var(--yellow);font-size:11px;
  font-family:monospace;text-align:right;}
.ct2 input:focus{outline:none;border-color:var(--accent);}
.rl{font-size:11px;font-weight:600;color:var(--cyan);}

/* mode feature table */
.mft{width:100%;border-collapse:collapse;font-size:10px;margin-top:6px;}
.mft th{padding:4px 6px;border-bottom:1px solid var(--border);color:var(--accent);}
.mft td{padding:3px 6px;border-bottom:1px solid #151525;}
.yes{color:var(--green);font-weight:700;}.no{color:var(--red);}.ok{color:var(--yellow);}
</style>
</head>
<body>

<div class="hdr">
  <h1><span>Cartridge</span> System</h1>
  <div class="hdr-r">
    <div class="state-badge"><div class="sdot" id="sdot"></div><span id="stTxt">UNKNOWN</span></div>
    <div class="mpill m-auto" id="mpill"><span id="micon">🤖</span><span id="mtxt">AUTO</span></div>
  </div>
</div>

<div class="nbar" id="nb">
  <span class="ntit" id="ntit"></span>
  <span class="ndet" id="ndet"></span>
  <span class="ntm" id="ntm"></span>
  <div class="nact">
    <button class="btn o" style="padding:2px 7px;font-size:10px" onclick="resetFaults()">Reset</button>
    <button class="btn" style="padding:2px 7px;font-size:10px" onclick="dnb()">✕</button>
  </div>
</div>

<div class="tabs">
  <div class="tab active" onclick="tab('ctrl',this)">Control</div>
  <div class="tab" onclick="tab('cfg',this)">Config</div>
</div>

<!-- ════════════════ CONTROL PAGE ════════════════ -->
<div class="page active" id="page-ctrl">
<div class="grid">

  <!-- Left col -->
  <div class="lc">

    <!-- Mode: only AUTO / MANUAL -->
    <div class="card">
      <div class="ct">Operation Mode</div>
      <div class="gr2">
        <button class="btn g sel" id="bAuto" onclick="setMode('auto')"
          title="Đọc sensor thực, tự động chạy. JOG + Sim bị khóa.">🤖 AUTO</button>
        <button class="btn pu" id="bMan" onclick="setMode('manual')"
          title="JOG + Sim sensor. Chọn STATE 1/2 để chạy.">🖐 MANUAL</button>
      </div>
      <div id="dAuto" style="font-size:9px;color:var(--dim);line-height:1.6;padding:2px">
        ✅ Sensor thực tế &nbsp;·&nbsp; ✅ Tự động trigger<br>🔒 JOG &nbsp;·&nbsp; 🔒 Sim sensor
      </div>
      <div id="dMan" style="display:none;font-size:9px;color:var(--purple);line-height:1.6;padding:2px">
        ✅ JOG tự do &nbsp;·&nbsp; ✅ Sim sensor<br>✅ Chọn STATE 1/2 từ nút bên dưới
      </div>
    </div>

    <!-- System control -->
    <div class="card">
      <div class="ct">System Control</div>
      <div class="gr3">
        <button class="btn g" onclick="sys('start')">START</button>
        <button class="btn r" onclick="sys('stop')">STOP</button>
        <button class="btn o" onclick="sys('pause')">PAUSE</button>
      </div>
      <div class="gr2">
        <button class="btn bl" onclick="api('/api/confirm')">Confirm</button>
        <button class="btn g" onclick="api('/api/hmi_resume')">Resume</button>
      </div>
    </div>

    <!-- State nav -->
    <div class="card" style="flex:1">
      <div class="ct">State Navigation</div>
      <!-- Always available -->
      <div class="gr2" style="margin-bottom:6px">
        <button class="btn" onclick="gotoState('HOMING')">🏠 HOMING</button>
        <button class="btn" onclick="gotoState('IDLE')">⏹ IDLE</button>
      </div>
      <!-- Workflow: manual only -->
      <div style="font-size:9px;text-transform:uppercase;letter-spacing:.8px;margin-bottom:3px"
           id="wfLabel">— Workflow (manual only) —</div>
      <div class="gr2">
        <button class="btn bl lk" id="bS1" onclick="gotoWf('STATE1')"
          title="Cấp khay Input: kiểm tra S1/S2/S3 + S12">
          ▶ STATE 1<br><span style="font-size:9px;font-weight:400;text-transform:none">Cấp khay</span>
        </button>
        <button class="btn bl lk" id="bS2" onclick="gotoWf('STATE2')"
          title="Thay khay Input: kiểm tra S13">
          ▶ STATE 2<br><span style="font-size:9px;font-weight:400;text-transform:none">Thay khay</span>
        </button>
      </div>
      <div class="lkn show" id="wfLock">🔒 AUTO mode — hệ thống tự chọn STATE</div>
      <div style="margin-top:4px">
        <button class="btn r" onclick="gotoState('ERROR')" style="width:100%;padding:4px">⛔ FORCE ERROR</button>
      </div>
    </div>

  </div>

  <!-- Center col -->
  <div class="cc">
    <div class="card" style="flex-shrink:0">
      <div class="ct">Target Row</div>
      <div style="display:flex;gap:4px" id="trbar"></div>
    </div>
    <div class="card" style="flex:1;display:flex;flex-direction:column;min-height:0;overflow:hidden">
      <div style="display:flex;align-items:center;gap:6px;margin-bottom:5px;flex-shrink:0">
        <span class="ct" style="margin-bottom:0">Servo Control</span>
        <span style="font-size:10px;color:var(--dim)">Vel:</span>
        <input type="number" class="vel" id="jv" value="30" min="1" max="200">
        <span style="font-size:10px;color:var(--dim)">mm/s</span>
        <span id="jlk" style="display:none;font-size:10px;color:var(--dim)">🔒 AUTO</span>
      </div>
      <div id="svlist"></div>
    </div>
  </div>

  <!-- Log -->
  <div class="card la">
    <div class="lh">
      <div class="ct" style="margin-bottom:0">Log</div>
      <button class="btn" style="padding:3px 9px;font-size:11px"
        onclick="document.getElementById('lb').innerHTML=''">Clear</button>
    </div>
    <div class="lb" id="lb"></div>
  </div>

  <!-- Sensor col -->
  <div class="sc"><div class="card" style="height:100%;display:flex;flex-direction:column">
    <div class="ct">Sensor Panel</div>

    <!-- Auto: real sensor badge -->
    <div class="rs-badge show" id="rsbadge">📡 REAL SENSORS (sim khóa)</div>

    <!-- Manual: sim controls -->
    <div id="simCtrl" style="display:none">
      <div style="display:flex;gap:3px;margin-bottom:3px">
        <button class="btn g" style="flex:1;padding:3px 4px;font-size:10px" onclick="sAll(1)">All ON</button>
        <button class="btn r" style="flex:1;padding:3px 4px;font-size:10px" onclick="sAll(0)">All OFF</button>
        <button class="btn"   style="flex:1;padding:3px 4px;font-size:10px" onclick="sClear()">Clear</button>
      </div>
      <div style="font-size:9px;color:var(--dim);margin-bottom:2px;text-transform:uppercase;letter-spacing:.8px">Quick Preset</div>
      <div style="display:flex;gap:3px;margin-bottom:3px">
        <button class="btn bl" style="flex:1;padding:3px 4px;font-size:9px"
          title="S1+S12+S3+S10 ON — đủ điều kiện vào State 1 và pass Step 3"
          onclick="simPreset([1,3,10,12])">S1 Entry</button>
        <button class="btn pu" style="flex:1;padding:3px 4px;font-size:9px"
          title="S1+S3+S4+S5+S10+S11+S12+S13 ON — full State 1 workflow"
          onclick="simPreset([1,3,4,5,10,11,12,13])">S1 Full</button>
      </div>
    </div>


    <div style="font-size:9px;color:var(--dim);text-transform:uppercase;letter-spacing:1px;margin-bottom:4px">Status</div>
    <div class="sgrid" id="sg"></div>

    <div style="font-size:8px;color:var(--dim);line-height:1.5;margin-top:6px">
      <b>S1-S3</b> Băng tải · <b>S4</b> Stack In · <b>S5</b> Output Det<br>
      <b>S6</b> In Stack · <b>S7</b> Platform · <b>S8</b> Feed OK · <b>S9</b> Out Finish<br>
      <b>S10</b> Stack Out · <b>S11</b> Tray@Robot<br>
      <b>S15/16</b> Cyl1 Ret/Ext<br>
      <b>S19/20</b> Cyl2 Ret/Ext
    </div>

    <!-- Feature compare table -->
    <div style="margin-top:8px;padding:5px;background:var(--card);border-radius:4px;border:1px solid var(--border)">
      <table class="mft">
        <tr><th>Tính năng</th><th>AUTO</th><th>MANUAL</th></tr>
        <tr><td>Sensor đọc</td><td class="yes">Thực tế</td><td class="ok">Thực+Sim</td></tr>
        <tr><td>Sim sensor</td><td class="no">🔒</td><td class="yes">✅</td></tr>
        <tr><td>JOG trục</td><td class="no">🔒</td><td class="yes">✅</td></tr>
        <tr><td>Tự trigger</td><td class="yes">✅ liên tục</td><td class="no">Manual</td></tr>
        <tr><td>Chọn STATE</td><td class="ok">Auto</td><td class="yes">GUI nút</td></tr>
      </table>
    </div>
  </div></div>

</div></div>

<!-- ════════════════ CONFIG PAGE ════════════════ -->
<div class="page" id="page-cfg"><div class="cgrid">
  <div class="card"><div class="ct">Input Stack — InY (mm)</div>
    <table class="ct2"><tr><th>Row</th><th>Pos (mm)</th><th>Note</th></tr></table>
    <table class="ct2" id="tIS"></table>
    <div style="margin-top:7px;display:flex;gap:5px">
      <button class="btn g" style="padding:3px 9px;font-size:11px" onclick="saveCfg('iny_input_stack','tIS')">Save</button>
      <button class="btn" style="padding:3px 9px;font-size:11px" onclick="loadCfg()">↻ Reload</button>
    </div>
  </div>
  <div class="card"><div class="ct">Output Stack — InY (mm)</div>
    <table class="ct2"><tr><th>Row</th><th>Pos (mm)</th><th>Note</th></tr></table>
    <table class="ct2" id="tOS"></table>
    <div style="margin-top:7px;display:flex;gap:5px">
      <button class="btn g" style="padding:3px 9px;font-size:11px" onclick="saveCfg('iny_output_stack','tOS')">Save</button>
      <button class="btn" style="padding:3px 9px;font-size:11px" onclick="loadCfg()">↻ Reload</button>
    </div>
  </div>
  <div class="card"><div class="ct">Output Table — OutY (mm)</div>
    <table class="ct2"><tr><th>Row</th><th>Pos (mm)</th><th>Note</th></tr></table>
    <table class="ct2" id="tOT"></table>
    <div style="margin-top:7px;display:flex;gap:5px">
      <button class="btn g" style="padding:3px 9px;font-size:11px" onclick="saveCfg('outy_output_table','tOT')">Save</button>
      <button class="btn" style="padding:3px 9px;font-size:11px" onclick="loadCfg()">↻ Reload</button>
    </div>
  </div>
  <div class="card"><div class="ct">Key Positions (mm)</div>
    <table class="ct2" id="tKP"><tr><th>Parameter</th><th>Value</th><th>Desc</th></tr></table>
  </div>
</div></div>

<div class="tc" id="tc"></div>

<script>
// ─── State ──────────────────────────────────────────────
const SS={};for(let i=1;i<=20;i++)SS[i]=false;
let mode='auto', sysState='unknown';

const SERVOS=[
  {id:1,name:'InX',d:'Trục X đầu vào'},
  {id:2,name:'InY',d:'Trục Y đầu vào'},
  {id:3,name:'Platform',d:'Đỡ/đẩy khay'},
  {id:4,name:'OutX',d:'Trục X đầu ra'},
  {id:5,name:'OutY',d:'Trục Y đầu ra'}
];
const SLB={
  1:'Belt start',2:'Belt mid',3:'Belt end',4:'Stack In',5:'Output det.',
  6:'In stack tray',7:'Platform tray',8:'Feed OK',9:'Out finish',
  10:'Stack Out',11:'Tray@Robot',12:'[reserved]',13:'[reserved]',
  14:'[reserved]',15:'Cyl1 Ret',16:'Cyl1 Ext',17:'Dự phòng',
  18:'Dự phòng',19:'Cyl2 Ret',20:'Cyl2 Ext'
};

// ─── Mode ───────────────────────────────────────────────
function setMode(m) {
  if(m!=='auto'&&m!=='manual') return;
  fetch('/api/set_mode',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({mode:m})})
  .then(r=>r.json()).then(j=>{
    if(j.ok){
      mode=m; updateModeUI();
      log('Mode → '+m.toUpperCase(), m==='auto'?'ok':'in');
    } else {
      toast('❌ '+(j.err||'Mode change failed'),'er');
      log(j.err||'Mode blocked','er');
    }
  }).catch(()=>toast('❌ Connection error','er'));
}

function updateModeUI() {
  const ia=mode==='auto', im=mode==='manual';
  // Header pill
  const p=document.getElementById('mpill');
  p.className='mpill '+(ia?'m-auto':'m-manual');
  document.getElementById('micon').textContent=ia?'🤖':'🖐';
  document.getElementById('mtxt').textContent=ia?'AUTO':'MANUAL';
  // Mode btns
  document.getElementById('bAuto').classList.toggle('sel',ia);
  document.getElementById('bMan').classList.toggle('sel',im);
  // Description
  document.getElementById('dAuto').style.display=ia?'':'none';
  document.getElementById('dMan').style.display=im?'':'none';
  // JOG lock
  document.getElementById('jlk').style.display=ia?'':'none';
  document.querySelectorAll('.jb').forEach(b=>b.classList.toggle('lk',ia));
  // Sensor panel
  document.getElementById('rsbadge').classList.toggle('show',ia);
  document.getElementById('simCtrl').style.display=im?'':'none';
  document.querySelectorAll('.sb').forEach(b=>b.classList.toggle('lk',ia));
  // Workflow buttons
  document.getElementById('bS1').classList.toggle('lk',ia);
  document.getElementById('bS2').classList.toggle('lk',ia);
  document.getElementById('wfLabel').style.color=im?'var(--purple)':'var(--dim)';
  document.getElementById('wfLabel').textContent=ia?'— Workflow (auto — read only) —':'— Workflow — chọn để chạy —';
  const lkn=document.getElementById('wfLock');
  lkn.classList.toggle('show',ia);
  lkn.textContent=ia?'🔒 AUTO mode — hệ thống tự trigger STATE 1/2':'✅ Nhấn STATE 1 hoặc STATE 2 để bắt đầu';
}

// ─── API ────────────────────────────────────────────────
async function api(ep,data={}) {
  try {
    const r=await fetch(ep,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});
    const j=await r.json();
    const lbl=ep.split('/').pop().replace(/_/g,' ');
    if(j.ok) toast('✅ '+lbl);
    else toast('❌ '+(j.err||'Error'),'er');
    log(lbl+': '+(j.ok?'OK':j.err), j.ok?'ok':'er');
    return j;
  } catch(e){ toast('❌ Connection','er'); return {ok:false}; }
}

function sys(a) { api('/api/'+a); }
function resetFaults() { api('/api/reset_faults'); }

function gotoState(s) { api('/api/goto_state',{state:s}); log('goto '+s,'in'); }

// STATE1/STATE2 workflow — manual only
function gotoWf(s) {
  if(mode!=='manual'){
    toast('🔒 MANUAL mode only','wn');
    log('STATE nav blocked — switch to MANUAL','wn');
    return;
  }
  api('/api/goto_state',{state:s});
  log('Manual → '+s,'ok');
}

function setRow(r) { api('/api/set_target_row',{row:r}); }

// ─── Sensor sim ─────────────────────────────────────────
function tog(id) {
  if(mode!=='manual'){ toast('🔒 Sim: MANUAL mode only','wn'); return; }
  SS[id]=!SS[id];
  fetch('/api/sim_sensor',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({cmd:id+':'+(SS[id]?1:0)})});
  rfs(id);
}
function sAll(v) {
  if(mode!=='manual'){ toast('🔒 MANUAL only','wn'); return; }
  for(let i=1;i<=20;i++){ SS[i]=!!v; rfs(i); }
  fetch('/api/sim_sensor',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:'all:'+v})});
}
function sClear() {
  if(mode!=='manual'){ toast('🔒 MANUAL only','wn'); return; }
  for(let i=1;i<=20;i++){ SS[i]=false; rfs(i); }
  fetch('/api/sim_sensor',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:'clear'})});
}
function simPreset(ids) {
  if(mode!=='manual'){ toast('🔒 MANUAL only','wn'); return; }
  // Clear all first
  for(let i=1;i<=20;i++){ SS[i]=false; rfs(i); }
  fetch('/api/sim_sensor',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:'clear'})});
  // Set preset sensors ON
  ids.forEach(id=>{ SS[id]=true; rfs(id); });
  const cmd=ids.map(id=>id+':1').join(',');
  // Send each ON
  ids.forEach(id=>{
    fetch('/api/sim_sensor',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({cmd:id+':1'})});
  });
  log('Preset: S['+ids.join(',')+'] ON','ok');
  toast('✅ Preset: S'+ids.join('+'),'ok');
}
function rfs(id) {
  const el=document.getElementById('s'+id);
  if(el) el.className='sb'+(SS[id]?' on':'')+(mode==='auto'?' lk':'');
}

// ─── JOG ────────────────────────────────────────────────
function jog(id,dir) {
  if(mode!=='manual'){ toast('🔒 JOG: MANUAL mode only','wn'); return; }
  const v=document.getElementById('jv').value||30;
  fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:id+' '+dir+' '+v})});
}
function jStop(id) { // STOP always works (safety)
  fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:id+' stop'})});
}
function hSv(id) {
  if(mode!=='manual'){ toast('🔒 MANUAL only','wn'); return; }
  fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:'home '+id})});
  log('Home S'+id,'in');
}
function clSv(id) {
  if(mode!=='manual'){ toast('🔒 MANUAL only','wn'); return; }
  fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:'clear '+id})});
  log('Clear S'+id,'in');
}
function mvSv(id) {
  if(mode!=='manual'){ toast('🔒 MANUAL only','wn'); return; }
  const pos=parseFloat(document.getElementById('pi'+id)?.value);
  if(isNaN(pos)){ toast('❌ Invalid pos','er'); return; }
  api('/api/move_servo',{cmd:id+':'+pos});
}

// ─── Config ─────────────────────────────────────────────
async function loadCfg() {
  await fetch('/api/get_config',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
  await new Promise(r=>setTimeout(r,500));
  const j=await(await fetch('/api/config')).json();
  if(!j.ok||!j.config) return;
  const c=j.config;
  if(c.iny_input_stack)  buildTbl('tIS',c.iny_input_stack);
  if(c.iny_output_stack) buildTbl('tOS',c.iny_output_stack);
  if(c.outy_output_table)buildTbl('tOT',c.outy_output_table);
  if(c.operation_mode)   { mode=c.operation_mode; updateModeUI(); }
  buildKP(c);
}
function buildTbl(id,data) {
  const tbl=document.getElementById(id); tbl.innerHTML='';
  for(let r=8;r>=1;r--) {
    const v=data[r]||data[String(r)]||0;
    const tr=document.createElement('tr');
    tr.innerHTML=`<td class="rl">R${r}</td><td><input type="number" step="0.1" value="${v}" id="${id}_r${r}"></td><td style="font-size:9px;color:var(--dim)">${r===8?'Top':r===1?'Bot':''}</td>`;
    tbl.appendChild(tr);
  }
}
function buildKP(c) {
  const t=document.getElementById('tKP');
  t.innerHTML='<tr><th>Parameter</th><th>Value</th><th>Desc</th></tr>';
  [['InX Home',c.inx_home,'S1 home'],['InX Target',c.inx_target,'500mm'],
   ['InY Home',c.iny_home,'S2 home'],['InY Place',c.iny_place,'200mm'],
   ['InY Safe',c.iny_safe_zone,'50mm']
  ].forEach(([n,v,d])=>{
    if(v===undefined) return;
    const tr=document.createElement('tr');
    tr.innerHTML=`<td class="rl">${n}</td><td style="color:var(--yellow)">${v}</td><td style="font-size:9px;color:var(--dim)">${d}</td>`;
    t.appendChild(tr);
  });
}
async function saveCfg(tbl,id) {
  const pos={};
  for(let r=1;r<=8;r++) {
    const el=document.getElementById(id+'_r'+r);
    if(el) pos[r]=parseFloat(el.value)||0;
  }
  await api('/api/update_config',{config:{table:tbl,positions:pos}});
}

// ─── Build UI ───────────────────────────────────────────
function buildRows() {
  const bar=document.getElementById('trbar');
  for(let r=1;r<=8;r++) {
    const b=document.createElement('button');
    b.className='btn';b.style.cssText='flex:1;padding:4px 5px;font-size:11px';
    b.textContent='R'+r;b.onclick=()=>setRow(r);bar.appendChild(b);
  }
}
function buildSensors() {
  const g=document.getElementById('sg');
  for(let i=1;i<=20;i++) {
    const d=document.createElement('div');
    d.className='sb';d.id='s'+i;d.onclick=()=>tog(i);
    const lb=SLB[i]||'';
    d.innerHTML=`<span class="sn">S${i}</span>${lb?`<span class="slb">${lb}</span>`:''}<span class="sdt"></span>`;
    g.appendChild(d);
  }
}
function buildServos() {
  const list=document.getElementById('svlist');
  SERVOS.forEach(s=>{
    const d=document.createElement('div');
    d.className='svc';
    d.innerHTML=`
      <div style="text-align:center"><div class="svn">S${s.id}: ${s.name}</div><div class="svd">${s.d}</div></div>
      <div class="svp" id="sp${s.id}">-- mm</div>
      <div class="jr">
        <button class="btn jb" onmousedown="jog(${s.id},'-')" onmouseup="jStop(${s.id})" onmouseleave="jStop(${s.id})" ontouchstart="jog(${s.id},'-')" ontouchend="jStop(${s.id})">−</button>
        <button class="btn r" onclick="jStop(${s.id})" style="font-size:13px">STOP</button>
        <button class="btn jb" onmousedown="jog(${s.id},'+')" onmouseup="jStop(${s.id})" onmouseleave="jStop(${s.id})" ontouchstart="jog(${s.id},'+')" ontouchend="jStop(${s.id})">+</button>
      </div>
      <div class="sfw"><button class="btn g jb" onclick="hSv(${s.id})">HOMING</button></div>
      <div class="sfw"><button class="btn o jb" onclick="clSv(${s.id})">CLEAR</button></div>
      <div class="pr">
        <input type="number" class="pi" id="pi${s.id}" placeholder="0.0" step="0.1">
        <span style="font-size:9px;color:var(--dim)">mm</span>
        <button class="btn ac jb" onclick="mvSv(${s.id})" style="padding:5px 12px;font-size:15px">GO</button>
      </div>`;
    list.appendChild(d);
  });
}

// ─── Toast & Log ────────────────────────────────────────
function toast(msg,type='ok') {
  const c=document.getElementById('tc');
  const t=document.createElement('div');
  t.className='ts'+(type==='er'?' er':type==='wn'?' wn':'');
  t.textContent=msg;c.appendChild(t);
  setTimeout(()=>t.remove(),3000);
}
function log(msg,type='in') {
  const box=document.getElementById('lb');
  const tm=new Date().toLocaleTimeString('vi-VN');
  box.innerHTML+=`<div class="${type}">[${tm}] ${msg}</div>`;
  box.scrollTop=box.scrollHeight;
  while(box.children.length>200) box.removeChild(box.firstChild);
}

// ─── Polling ────────────────────────────────────────────
async function pollSt() {
  try {
    const j=await(await fetch('/api/status')).json();
    const s=j.state||'';
    sysState=s.toLowerCase();
    document.getElementById('stTxt').textContent=s||'UNKNOWN';
    const dot=document.getElementById('sdot');
    if(!s||s==='UNKNOWN'){dot.style.background='#555';dot.style.boxShadow='';}
    else if(s.includes('ERROR')){dot.style.background='var(--red)';dot.style.boxShadow='0 0 7px var(--red)';}
    else if(s==='IDLE'){dot.style.background='var(--orange)';dot.style.boxShadow='0 0 7px var(--orange)';}
    else{dot.style.background='var(--green)';dot.style.boxShadow='0 0 7px var(--green)';}
  } catch(e) { document.getElementById('stTxt').textContent='OFFLINE'; }
}
async function pollPos() {
  try {
    const j=await(await fetch('/api/positions',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'})).json();
    if(j.ok&&j.positions) {
      for(let i=1;i<=5;i++) {
        const el=document.getElementById('sp'+i);
        if(el){ const v=j.positions[String(i)];el.textContent=v!==undefined?Number(v).toFixed(1)+' mm':'-- mm'; }
      }
    }
  }catch(e){}
}
let _ni=0,_nt=null;
async function pollN() {
  try {
    const j=await(await fetch('/api/notifications',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({since:_ni})})).json();
    if(j.ok&&j.notifications&&j.notifications.length>0) {
      _ni=j.total;
      const last=j.notifications[j.notifications.length-1];
      showN(last.level,last.title,last.detail,last.time);
      j.notifications.forEach(n=>log(n.title+(n.detail?': '+n.detail:''),
        n.level==='error'?'er':n.level==='warn'?'wn':'in'));
    }
  }catch(e){}
}
function showN(lvl,title,det,tm) {
  const bar=document.getElementById('nb');
  const lmap={error:'error',warn:'warn',info:'info'};
  bar.className='nbar show '+(lmap[lvl]||'info');
  document.getElementById('ntit').textContent=title;
  document.getElementById('ndet').textContent=det||'';
  document.getElementById('ntm').textContent=tm||'';
  clearTimeout(_nt);
  if(lvl==='info') _nt=setTimeout(dnb,5000);
}
function dnb() { document.getElementById('nb').className='nbar'; }

function tab(n,el) {
  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));
  document.querySelectorAll('.page').forEach(p=>p.classList.remove('active'));
  document.getElementById('page-'+n).classList.add('active');
  el.classList.add('active');
  if(n==='cfg') loadCfg();
}

// ─── Init ───────────────────────────────────────────────
buildRows(); buildSensors(); buildServos();
updateModeUI();
pollSt(); pollPos();
setInterval(pollSt,2000);
setInterval(pollN,1500);
setInterval(pollPos,500);
log('GUI v4 — AUTO mode active','ok');
log('💡 Chuyển MANUAL để dùng JOG + Sim sensor','in');
</script>
</body>
</html>
"""


def main():
    import socket
    print("  Initializing ROS2...")
    init_ros()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80)); local_ip = s.getsockname()[0]; s.close()
    except:
        local_ip = "localhost"

    http.server.HTTPServer.allow_reuse_address = True
    server = http.server.HTTPServer(('0.0.0.0', PORT), GUIHandler)

    def _sig(sn, f):
        try: server.shutdown()
        except: pass
        try: rclpy.shutdown()
        except: pass
        os._exit(0)

    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    print("=" * 55)
    print("  Cartridge System GUI v4 — Auto / Manual Mode")
    print("=" * 55)
    print(f"  Local  : http://localhost:{PORT}?token={AUTH_TOKEN}")
    print(f"  Network: http://{local_ip}:{PORT}?token={AUTH_TOKEN}")
    print(f"  Token  : {AUTH_TOKEN}")
    print("=" * 55)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        try: server.shutdown()
        except: pass
        try: rclpy.shutdown()
        except: pass


if __name__ == '__main__':
    main()