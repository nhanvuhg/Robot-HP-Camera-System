#!/usr/bin/env python3
"""
Cartridge System GUI — Web Dashboard v2
Features: Mode lock, Position move, Technical System config page
Direct rclpy publishing for low-latency jog control.
Run:   python3 cartridge_gui.py
Access: http://<IP>:8080
"""

import http.server
import json
import subprocess
import threading
import time
import os
import signal
import sys
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool

PORT = 8080

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
            '/providesystem/servo_limit_cmd':      self.create_publisher(String, '/providesystem/servo_limit_cmd', qos_default),
            '/providesystem/set_target_row':       self.create_publisher(String, '/providesystem/set_target_row', qos_default),
            '/providesystem/hmi_resume':           self.create_publisher(Bool, '/providesystem/hmi_resume', qos_default),
            '/system/start_button':                self.create_publisher(Bool, '/system/start_button', qos_default),
            '/system/stop_button':                 self.create_publisher(Bool, '/system/stop_button', qos_default),
            '/providesystem/reset_faults':         self.create_publisher(String, '/providesystem/reset_faults', qos_default),
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

def ros2_srv(service, srv_type, data, timeout=5):
    return _run(f"ros2 service call {service} {srv_type} \"{data}\"", timeout)

def _run(cmd, timeout=5):
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                           timeout=timeout, env=os.environ.copy())
        return {"ok": True, "out": r.stdout.strip(), "err": r.stderr.strip()}
    except subprocess.TimeoutExpired:
        return {"ok": False, "err": "Timeout"}
    except Exception as e:
        return {"ok": False, "err": str(e)}

# ============================================================================
# HTTP SERVER
# ============================================================================

class GUIHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args): pass

    def do_GET(self):
        p = urlparse(self.path).path
        if p in ('/', '/index.html'): self._html(HTML)
        elif p == '/api/status':      self._json(self._get_status())
        elif p == '/api/config':      self._json(self._get_config())
        else: self.send_error(404)

    def do_POST(self):
        body = self.rfile.read(int(self.headers.get('Content-Length', 0)))
        d = json.loads(body.decode()) if body else {}
        p = urlparse(self.path).path

        routes = {
            '/api/start':          lambda: fast_pub_bool('/system/start_button', True),
            '/api/stop':           lambda: fast_pub_bool('/system/stop_button', True),
            '/api/manual_mode':    lambda: ros2_srv('/providesystem/set_manual_mode', 'std_srvs/srv/SetBool',
                                                    '{data: '+str(d.get('on',True)).lower()+'}'),
            '/api/next_step':      lambda: ros2_srv('/providesystem/next_step', 'std_srvs/srv/Trigger', '{}'),
            '/api/goto_state':     lambda: fast_pub_string('/providesystem/goto_state', d.get('state','IDLE')),
            '/api/sim_sensor':     lambda: fast_pub_string('/providesystem/sim_sensor', d.get('cmd','status')),
            '/api/jog':            lambda: fast_pub_string('/providesystem/jog_cmd', d.get('cmd','1 stop')),
            '/api/set_mode':       lambda: fast_pub_string('/providesystem/set_operation_mode', d.get('mode','auto')),
            '/api/confirm_output': lambda: ros2_srv('/providesystem/confirm_output_load', 'std_srvs/srv/SetBool',
                                                    '{data: true}'),
            '/api/hmi_resume':     lambda: fast_pub_bool('/providesystem/hmi_resume', True),
            '/api/servo_limit':    lambda: fast_pub_string('/providesystem/servo_limit_cmd', d.get('cmd','')),
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
        self.end_headers()
        self.wfile.write(c.encode())

    def _json(self, data):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
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
# HTML
# ============================================================================

HTML = r"""<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Cartridge System Control</title>
<link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;600;700&display=swap" rel="stylesheet">
<style>
:root {
  --bg:#0c0c1d; --bg2:#141428; --card:#1a1a35; --card2:#141428;
  --border:#2a2a50; --accent:#4f6cff;
  --green:#00e676; --red:#ff5252; --orange:#ffa726; --yellow:#ffd740;
  --cyan:#26c6da; --text:#e8e8f0; --dim:#8888aa;
}
*{margin:0;padding:0;box-sizing:border-box;}
body{font-family:'JetBrains Mono',monospace;background:var(--bg);color:var(--text);
     height:100vh;overflow:hidden;font-size:12px;}
::-webkit-scrollbar{width:5px;}
::-webkit-scrollbar-track{background:var(--bg2);}
::-webkit-scrollbar-thumb{background:var(--accent);border-radius:3px;}

/* HEADER */
.hdr{background:var(--bg2);border-bottom:1px solid var(--border);
     padding:0 14px;height:36px;display:flex;align-items:center;justify-content:space-between;}
.hdr h1{font-size:14px;font-weight:700;letter-spacing:1px;text-transform:uppercase;}
.hdr h1 span{color:var(--accent);}
.hdr-right{display:flex;align-items:center;gap:10px;}
.state-badge{background:var(--bg);border:1px solid var(--border);border-radius:8px;
             padding:4px 12px;font-size:11px;font-weight:600;
             display:flex;align-items:center;gap:7px;}
.state-dot{width:8px;height:8px;border-radius:50%;background:#666;animation:pulse 2s infinite;}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.35}}
.mode-pill{padding:3px 11px;border-radius:12px;font-size:11px;font-weight:700;
           text-transform:uppercase;letter-spacing:1px;border:1px solid;}
.mode-auto {background:#0a332e;border-color:var(--green); color:var(--green);}
.mode-jog  {background:#332e0a;border-color:var(--orange);color:var(--orange);}
.mode-idle {background:#1a1a2e;border-color:var(--dim);   color:var(--dim);}

/* TAB BAR */
.tab-bar{display:flex;gap:2px;background:#0f0f22;border-bottom:1px solid var(--border);
         padding:3px 10px 0;height:30px;}
.tab{padding:3px 14px;font-size:11px;font-weight:600;border-radius:4px 4px 0 0;
     cursor:pointer;border:1px solid transparent;border-bottom:none;
     color:var(--dim);transition:all .15s;background:transparent;}
.tab:hover{color:var(--text);background:var(--card);}
.tab.active{color:var(--accent);background:var(--card);border-color:var(--border);}
.page{display:none;height:calc(100vh - 66px);}
.page.active{display:block;}

/* MAIN GRID — exact QML layout */
.page-grid{
  display:grid;
  grid-template-columns:210px 1fr 200px;
  grid-template-rows:1fr 120px;
  grid-template-areas:"ctrl center sensor" "log log sensor";
  gap:4px;padding:5px;height:100%;overflow:hidden;
}
.ctrl-col  {grid-area:ctrl;  overflow:hidden;display:flex;flex-direction:column;gap:4px;}
.center-col{grid-area:center;overflow:hidden;display:flex;flex-direction:column;gap:4px;}
.sensor-col{grid-area:sensor;overflow-y:auto;}
.log-area  {grid-area:log;}

/* CARDS */
.card{background:var(--card);border:1px solid var(--border);border-radius:4px;padding:6px 8px;}
.card-title{font-size:11px;font-weight:700;color:var(--accent);
            text-transform:uppercase;letter-spacing:1.5px;margin-bottom:5px;}

/* BUTTONS */
.btn{padding:5px 10px;border:1px solid var(--border);border-radius:4px;
     background:var(--card);color:var(--text);cursor:pointer;
     font-family:inherit;font-size:11px;font-weight:600;
     transition:all .12s;text-transform:uppercase;letter-spacing:.4px;white-space:nowrap;}
.btn:hover{border-color:var(--accent);background:#222248;}
.btn:active{transform:scale(.95);}
.btn:disabled{opacity:.3;cursor:not-allowed;transform:none!important;}
.btn-sm    {padding:3px 8px;font-size:10px;}
.btn-green {background:#0a332e;border-color:var(--green); color:var(--green);}
.btn-red   {background:#4d1a1a;border-color:var(--red);   color:var(--red);}
.btn-orange{background:#4d3a0a;border-color:var(--orange);color:var(--orange);}
.btn-accent{background:var(--accent);border-color:var(--accent);color:#fff;}
.btn-blue  {background:#1a2050;border-color:var(--accent);color:#8899ff;}
.brow{display:flex;gap:4px;margin-bottom:4px;}
.brow .btn{flex:1;}

/* STATE GRID */
.sg{display:grid;grid-template-columns:repeat(3,1fr);gap:4px;}

/* SERVO */
.servo-panel-inner{flex:1;display:flex;flex-direction:column;min-height:0;overflow:hidden;}
.vel-row{display:flex;align-items:center;gap:7px;margin-bottom:5px;flex-shrink:0;}
.vel-in{background:var(--bg);border:1px solid var(--border);border-radius:4px;
        padding:3px 6px;width:50px;color:var(--text);font-size:11px;
        font-family:inherit;text-align:center;}
.vel-in:focus{outline:none;border-color:var(--accent);}
#servoList{display:flex;gap:4px;flex:1;min-height:0;overflow:hidden;}
.scard{flex:1;background:var(--card2);border:1px solid var(--border);border-radius:4px;
       padding:5px 4px;display:flex;flex-direction:column;
       align-items:center;gap:4px;overflow:hidden;min-width:0;}
.scard:hover{border-color:var(--accent);}
.scard-title{font-size:10px;font-weight:600;color:var(--cyan);text-align:center;
             white-space:nowrap;overflow:hidden;text-overflow:ellipsis;width:100%;}
.scard-title .sd{color:var(--dim);font-weight:400;}
.scard-pos{font-size:16px;font-weight:700;color:var(--yellow);text-align:center;}
.jrow{display:flex;gap:2px;width:100%;}
.jrow .btn{flex:1;font-size:14px;font-weight:700;padding:5px 2px;}
.sfull{width:100%;}
.sfull .btn{width:100%;font-size:11px;padding:5px 2px;}
.prow{display:flex;gap:2px;width:100%;align-items:center;}
.pin{flex:1;background:var(--bg);border:1px solid var(--border);border-radius:4px;
     padding:4px 3px;color:var(--text);font-size:11px;font-family:inherit;
     text-align:center;min-width:0;}
.pin:focus{outline:none;border-color:var(--accent);}
.pin:disabled{opacity:.3;}

/* SENSOR */
.sensor-card{height:100%;display:flex;flex-direction:column;}
.stb{display:flex;gap:4px;margin-bottom:5px;flex-shrink:0;}
.stb .btn{flex:1;}
.slabel-hdr{font-size:11px;font-weight:700;color:var(--accent);
            text-transform:uppercase;letter-spacing:1.5px;margin-bottom:5px;flex-shrink:0;}
.sgrid{display:grid;grid-template-columns:repeat(2,1fr);gap:4px;flex:1;align-content:start;}
.sbtn{display:flex;flex-direction:column;align-items:center;justify-content:center;
      padding:5px 3px;border-radius:4px;border:1px solid var(--border);
      background:var(--card2);cursor:pointer;transition:all .12s;min-height:44px;}
.sbtn:hover{border-color:var(--cyan);}
.sbtn.on{background:#0a332e;border-color:var(--green);}
.sbtn .sid{font-size:11px;font-weight:700;color:var(--text);}
.sbtn.on .sid{color:var(--green);}
.sbtn .slb{font-size:8px;color:var(--dim);}
.sbtn .sdot{width:6px;height:6px;border-radius:50%;background:#333355;margin-top:2px;}
.sbtn.on .sdot{background:var(--green);box-shadow:0 0 6px var(--green);}
.sleg{font-size:8px;color:var(--dim);line-height:1.5;margin-top:5px;flex-shrink:0;}

/* LOG */
.log-hdr{display:flex;justify-content:space-between;align-items:center;margin-bottom:4px;}
.log-box{background:#0a0a18;border:1px solid var(--border);border-radius:4px;
         padding:6px 8px;height:calc(100% - 28px);overflow-y:auto;
         font-size:11px;line-height:1.5;}
.log-box .ok  {color:var(--green);}
.log-box .err {color:var(--red);}
.log-box .info{color:var(--cyan);}

/* LOCKED */
.locked-ov{position:relative;}
.locked-ov.locked::after{content:'AUTO MODE';display:flex;align-items:center;justify-content:center;
  position:absolute;inset:0;background:rgba(12,12,29,.82);border-radius:4px;
  color:var(--dim);font-size:11px;font-weight:700;letter-spacing:2px;z-index:10;}

/* NOTIFY */
.notify-bar{display:none;padding:7px 16px;font-size:11px;font-weight:500;
            align-items:center;gap:10px;border-bottom:1px solid;}
.notify-bar.show{display:flex;}
.notify-bar.error{background:linear-gradient(90deg,#3a0a0a,#2a0808);border-color:var(--red);   color:var(--red);}
.notify-bar.warn {background:linear-gradient(90deg,#3a2e0a,#2a2008);border-color:var(--orange);color:var(--orange);}
.notify-bar.info {background:linear-gradient(90deg,#0a2a33,#081e28);border-color:var(--cyan);  color:var(--cyan);}
.notify-bar .ntitle{font-weight:700;white-space:nowrap;}
.notify-bar .ndetail{flex:1;color:var(--text);opacity:.9;word-break:break-word;}
.notify-bar .ntime{font-size:10px;color:var(--dim);white-space:nowrap;}
.notify-bar .nact{display:flex;gap:6px;flex-shrink:0;}

/* TOAST */
.toast-ct{position:fixed;bottom:14px;right:14px;z-index:999;}
.toast{background:var(--card);border:1px solid var(--green);border-radius:6px;
       padding:7px 12px;margin-top:5px;font-size:11px;animation:si .25s;max-width:260px;}
.toast.error{border-color:var(--red);}
@keyframes si{from{transform:translateX(80px);opacity:0}to{transform:translateX(0);opacity:1}}

/* CONFIG */
.config-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));
             gap:12px;padding:12px;overflow-y:auto;height:100%;}
.config-table{width:100%;border-collapse:collapse;}
.config-table th{font-size:10px;text-transform:uppercase;color:var(--dim);
                 padding:5px 8px;border-bottom:1px solid var(--border);text-align:left;}
.config-table td{padding:3px 8px;border-bottom:1px solid #1e1e3a;}
.config-table input{background:var(--bg);border:1px solid var(--border);border-radius:4px;
                    padding:3px 5px;width:80px;color:var(--yellow);font-size:11px;
                    font-family:monospace;text-align:right;}
.config-table input:focus{outline:none;border-color:var(--accent);}
.config-table .rl{font-size:11px;font-weight:600;color:var(--cyan);}
</style>
</head>
<body>

<!-- HEADER -->
<div class="hdr">
  <h1><span>Cartridge</span> System</h1>
  <div class="hdr-right">
    <div class="mode-pill mode-idle" id="modePill">IDLE</div>
    <div class="state-badge">
      <div class="state-dot" id="stateDot"></div>
      <span id="stateText">UNKNOWN</span>
    </div>
  </div>
</div>

<!-- NOTIFY BAR -->
<div class="notify-bar" id="notifyBar">
  <span class="ntitle"  id="notifyTitle"></span>
  <span class="ndetail" id="notifyDetail"></span>
  <span class="ntime"   id="notifyTime"></span>
  <div class="nact">
    <button class="btn btn-sm btn-orange" onclick="resetFaults()">Reset Faults</button>
    <button class="btn btn-sm"            onclick="dismissNotify()">Dismiss</button>
  </div>
</div>

<!-- TAB BAR -->
<div class="tab-bar">
  <div class="tab active" onclick="switchTab('control',this)">Control Dashboard</div>
  <div class="tab"        onclick="switchTab('config',this)">Technical System</div>
</div>

<!-- PAGE 1: CONTROL DASHBOARD -->
<div class="page active" id="page-control">
<div class="page-grid">

  <!-- LEFT: ctrl-col -->
  <div class="ctrl-col">

    <!-- Mode Selection -->
    <div class="card">
      <div class="card-title">Mode Selection</div>
      <div class="brow">
        <button class="btn btn-green"  id="btnAuto" onclick="setMode('auto')">AUTO</button>
        <button class="btn btn-orange" id="btnJog"  onclick="setMode('jog')">JOG / MANUAL</button>
      </div>
      <button class="btn" id="btnIdle" style="width:100%" onclick="setMode('idle')">IDLE</button>
    </div>

    <!-- System Control -->
    <div class="card">
      <div class="card-title">System Control</div>
      <div class="brow">
        <button class="btn btn-green"  onclick="api('/api/start')">START</button>
        <button class="btn btn-red"    onclick="api('/api/stop')">STOP</button>
        <button class="btn btn-orange" onclick="api('/api/next_step')">NEXT</button>
      </div>
      <div class="brow">
        <button class="btn" id="btnManual" onclick="toggleManual()">MANUAL: OFF</button>
        <button class="btn"               onclick="api('/api/confirm_output')">CONFIRM</button>
      </div>
      <button class="btn" style="width:100%" onclick="api('/api/hmi_resume')">RESUME</button>
    </div>

    <!-- State Navigation -->
    <div class="card">
      <div class="card-title">State Navigation <span id="manualBadge" style="font-size:9px;color:var(--orange);display:none">(MANUAL)</span></div>
      <div style="font-size:9px;color:var(--dim);margin-bottom:4px">Goto state = auto MANUAL mode + sim sensors</div>
      <div class="sg">
        <button class="btn"          onclick="gotoState('homing')">HOMING</button>
        <button class="btn"          onclick="gotoState('idle')">IDLE</button>
      </div>
      <div style="font-size:9px;color:var(--dim);margin:4px 0 2px">State 1 (Cap khay)</div>
      <div class="sg">
        <button class="btn btn-blue" onclick="gotoState('s1_inx_to_conveyor_end')">S1: InX</button>
        <button class="btn btn-blue" onclick="gotoState('s1_iny_search_tray')">S1: InY</button>
        <button class="btn btn-blue" onclick="gotoState('s1_cylinder1_extend')">S1: Cyl1</button>
        <button class="btn btn-blue" onclick="gotoState('state1_complete')">S1: Done</button>
      </div>
      <div style="font-size:9px;color:var(--dim);margin:4px 0 2px">State 2-3</div>
      <div class="sg">
        <button class="btn btn-blue" onclick="gotoState('s2_wait_trigger')">STATE 2</button>
        <button class="btn btn-blue" onclick="gotoState('s3_wait_trigger')">STATE 3</button>
        <button class="btn btn-red"  onclick="gotoState('error')">ERROR</button>
      </div>
    </div>

  </div><!-- /ctrl-col -->

  <!-- CENTER -->
  <div class="center-col">

    <!-- Target Row -->
    <div class="card" style="flex-shrink:0">
      <div class="card-title">Target Row</div>
      <div class="brow" style="flex-wrap:nowrap">
        <button class="btn btn-sm" onclick="setTargetRow(1)">R1</button>
        <button class="btn btn-sm" onclick="setTargetRow(2)">R2</button>
        <button class="btn btn-sm" onclick="setTargetRow(3)">R3</button>
        <button class="btn btn-sm" onclick="setTargetRow(4)">R4</button>
        <button class="btn btn-sm" onclick="setTargetRow(5)">R5</button>
        <button class="btn btn-sm" onclick="setTargetRow(6)">R6</button>
        <button class="btn btn-sm" onclick="setTargetRow(7)">R7</button>
        <button class="btn btn-sm" onclick="setTargetRow(8)">R8</button>
      </div>
    </div>

    <!-- Servo Control -->
    <div class="card locked-ov servo-panel-inner" id="servoPanel">
      <div class="card-title" style="flex-shrink:0">Servo Control</div>
      <div class="vel-row">
        <span style="font-size:10px;color:var(--dim)">Vel:</span>
        <input type="number" class="vel-in" id="jogVel" value="30" min="1" max="200">
        <span style="font-size:10px;color:var(--dim)">mm/s</span>
      </div>
      <div id="servoList"></div>
    </div>

  </div><!-- /center-col -->

  <!-- LOG -->
  <div class="card log-area">
    <div class="log-hdr">
      <div class="card-title" style="margin-bottom:0">Log Activity</div>
      <button class="btn btn-sm" onclick="document.getElementById('logBox').innerHTML=''">CLEAR</button>
    </div>
    <div class="log-box" id="logBox"></div>
  </div>

  <!-- SENSOR SIMULATION (full-height right column) -->
  <div class="sensor-col">
    <div class="card sensor-card">
      <div class="card-title">Sensor Simulation</div>
      <div class="stb">
        <button class="btn btn-sm btn-green" onclick="simAll(1)">ALL ON</button>
        <button class="btn btn-sm btn-red"   onclick="simAll(0)">ALL OFF</button>
        <button class="btn btn-sm"           onclick="simClear()">CLEAR</button>
      </div>
      <div class="slabel-hdr">STATUS</div>
      <div class="sgrid" id="sensorGrid"></div>
      <div class="sleg">
        <b>S1</b> Conv.dau &nbsp;<b>S2</b> giua &nbsp;<b>S3</b> cuoi<br>
        <b>S4</b> Detect &nbsp;<b>S5</b> Output &nbsp;<b>S6</b> Robot<br>
        <b>S7</b> Loading &nbsp;<b>S10</b> Cyl1- &nbsp;<b>S11</b> Cyl1+<br>
        <b>S12</b> Cyl2- &nbsp;<b>S13</b> Cyl2+
      </div>
      <div id="simNote" style="font-size:9px;color:var(--orange);margin-top:4px;display:none">⚠ AUTO mode: sim blocked</div>
    </div>
  </div>

</div><!-- /page-grid -->
</div><!-- /page-control -->

<!-- PAGE 2: TECHNICAL SYSTEM -->
<div class="page" id="page-config">
<div class="config-grid">

  <div class="card">
    <div class="card-title">Pos 1: Input Stack (InY)</div>
    <table class="config-table"><tr><th>Row</th><th>Position (mm)</th><th>Mô tả</th></tr></table>
    <table class="config-table" id="tblInputStack"></table>
    <div style="margin-top:8px;display:flex;gap:6px">
      <button class="btn btn-green btn-sm" onclick="saveConfig('iny_input_stack','tblInputStack')">Save</button>
      <button class="btn btn-sm" onclick="loadConfig()">↻ Reload</button>
    </div>
  </div>

  <div class="card">
    <div class="card-title">Pos 1: Output Stack (InY)</div>
    <table class="config-table"><tr><th>Row</th><th>Position (mm)</th><th>Mô tả</th></tr></table>
    <table class="config-table" id="tblOutputStack"></table>
    <div style="margin-top:8px;display:flex;gap:6px">
      <button class="btn btn-green btn-sm" onclick="saveConfig('iny_output_stack','tblOutputStack')">Save</button>
      <button class="btn btn-sm" onclick="loadConfig()">↻ Reload</button>
    </div>
  </div>

  <div class="card">
    <div class="card-title">Pos 2: Output Table (OutY)</div>
    <table class="config-table"><tr><th>Row</th><th>Position (mm)</th><th>Mô tả</th></tr></table>
    <table class="config-table" id="tblOutputTable"></table>
    <div style="margin-top:8px;display:flex;gap:6px">
      <button class="btn btn-green btn-sm" onclick="saveConfig('outy_output_table','tblOutputTable')">Save</button>
      <button class="btn btn-sm" onclick="loadConfig()">↻ Reload</button>
    </div>
  </div>

  <div class="card">
    <div class="card-title">Servo Key Positions (mm)</div>
    <table class="config-table" id="tblServoPositions">
      <tr><th>Parameter</th><th>Current</th><th>Description</th></tr>
    </table>
    <p style="font-size:10px;color:var(--dim);margin-top:8px">
      * Chỉ đọc — chỉnh trong cartridge_config.yaml
    </p>
  </div>

</div>
</div>

<div class="toast-ct" id="toasts"></div>

<script>
// ── STATE ──────────────────────────────────────────────────────────────────
const SS = {};  // sensorState
for(let i=1;i<=15;i++) SS[i]=false;
let manualOn=false, currentMode='idle';

const SERVOS=[
  {id:1,name:'InX',    desc:'Trục X đầu vào'},
  {id:2,name:'InY',    desc:'Trục Y đầu vào'},
  {id:3,name:'PutTray',desc:'Đẩy khay'},
  {id:4,name:'OutX',   desc:'Trục X đầu ra'},
  {id:5,name:'OutY',   desc:'Trục Y đầu ra'},
];
const SLB={1:'Conv.dau',2:'Conv.giua',3:'Conv.cuoi',4:'Detect',5:'Output',6:'Robot',
           7:'Loading',8:'Plastic',9:'Stack',10:'Cyl1-',11:'Cyl1+',12:'Cyl2-',13:'Cyl2+',14:'',15:''};

// ── TABS ───────────────────────────────────────────────────────────────────
function switchTab(name,el){
  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));
  document.querySelectorAll('.page').forEach(p=>p.classList.remove('active'));
  document.getElementById('page-'+name).classList.add('active');
  el.classList.add('active');
  if(name==='config') loadConfig();
}

// ── MODE ───────────────────────────────────────────────────────────────────
let currentState='idle';
function setMode(mode){
  if(currentState!=='idle'){
    toast('⚠️ STOP system first before changing mode','error');
    return;
  }
  api('/api/set_mode',{mode});
  currentMode=mode; updateModeLock();
}
function updateModeLock(){
  const pill=document.getElementById('modePill');
  pill.className='mode-pill mode-'+currentMode;
  pill.textContent=currentMode.toUpperCase();
  document.getElementById('servoPanel').classList.toggle('locked',currentMode==='auto');
  const lk=currentMode==='auto';
  document.querySelectorAll('.jog-btn,.pin,.run-btn').forEach(el=>{el.disabled=lk;});
  ['Auto','Jog','Idle'].forEach(m=>{
    const b=document.getElementById('btn'+m);
    if(b) b.style.opacity=(m.toLowerCase()===currentMode)?'1':'0.45';
  });
  // Show sim blocked note in AUTO, hide manual badge
  const sn=document.getElementById('simNote');
  const mb=document.getElementById('manualBadge');
  if(sn) sn.style.display=lk?'block':'none';
  if(lk && mb) mb.style.display='none';
}

// ── API ────────────────────────────────────────────────────────────────────
async function api(ep,data={}){
  try{
    const r=await fetch(ep,{method:'POST',headers:{'Content-Type':'application/json'},
                            body:JSON.stringify(data)});
    const j=await r.json();
    const lbl=ep.split('/').pop().replace(/_/g,' ');
    if(j.ok) toast('✅ '+lbl); else toast('❌ '+(j.err||'Error'),'error');
    addLog(lbl+': '+(j.ok?'OK':j.err),j.ok?'ok':'err');
    return j;
  }catch(e){toast('❌ Connection error','error');return{ok:false};}
}

// ── ACTIONS ────────────────────────────────────────────────────────────────
function gotoState(s){
  api('/api/goto_state',{state:s});
  document.getElementById('manualBadge').style.display='inline';
  addLog('Goto: '+s+' (manual mode ON)','ok');
}
function setTargetRow(r) {api('/api/set_target_row',{row:r});}
function toggleManual(){
  manualOn=!manualOn;
  api('/api/manual_mode',{on:manualOn});
  const b=document.getElementById('btnManual');
  b.textContent='MANUAL: '+(manualOn?'ON':'OFF');
  b.className='btn'+(manualOn?' btn-orange':'');
}

// ── SENSORS ────────────────────────────────────────────────────────────────
function toggleSensor(id){
  SS[id]=!SS[id];
  api('/api/sim_sensor',{cmd:id+':'+(SS[id]?1:0)});
  refreshSensor(id);
}
function simAll(v){for(let i=1;i<=15;i++){SS[i]=!!v;refreshSensor(i);}api('/api/sim_sensor',{cmd:'all:'+v});}
function simClear(){for(let i=1;i<=15;i++){SS[i]=false;refreshSensor(i);}api('/api/sim_sensor',{cmd:'clear'});}
function refreshSensor(id){
  const el=document.getElementById('s'+id);
  if(el) el.className='sbtn'+(SS[id]?' on':'');
}

// ── SERVO ──────────────────────────────────────────────────────────────────
function jog(id,dir){
  if(currentMode==='auto') return;
  const v=document.getElementById('jogVel').value||30;
  fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({cmd:id+' '+dir+' '+v})});
}
function jogStop(id){
  if(currentMode==='auto') return;
  fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({cmd:id+' stop'})});
}
function homeServo(id){
  fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({cmd:'home '+id})});
  addLog('Home S'+id,'info');
}
function clearServo(id){
  fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({cmd:'clear '+id})});
  addLog('Clear fault S'+id,'info');
}
function moveServo(id){
  if(currentMode==='auto') return;
  const inp=document.getElementById('pi'+id);
  const pos=parseFloat(inp?.value);
  if(isNaN(pos)){toast('❌ Invalid position','error');return;}
  api('/api/move_servo',{cmd:id+':'+pos});
}

// ── CONFIG ─────────────────────────────────────────────────────────────────
async function loadConfig(){
  await api('/api/get_config');
  await new Promise(r=>setTimeout(r,500));
  const res=await fetch('/api/config');
  const j=await res.json();
  if(!j.ok||!j.config){
    toast('⚠️ Config not available','error');
    buildConfigTable('tblInputStack', defPos('i'));
    buildConfigTable('tblOutputStack',defPos('o'));
    buildConfigTable('tblOutputTable',defPos('y'));
    return;
  }
  const c=j.config;
  if(c.iny_input_stack)   buildConfigTable('tblInputStack',  c.iny_input_stack);
  if(c.iny_output_stack)  buildConfigTable('tblOutputStack', c.iny_output_stack);
  if(c.outy_output_table) buildConfigTable('tblOutputTable', c.outy_output_table);
  if(c.operation_mode){currentMode=c.operation_mode;updateModeLock();}
  buildServoInfoTable(c);
}
function defPos(t){const d={};for(let i=1;i<=8;i++)d[i]=t==='i'?650-i*50:t==='o'?630-i*70:540-i*60;return d;}
function buildConfigTable(id,data){
  const tbl=document.getElementById(id); tbl.innerHTML='';
  for(let r=8;r>=1;r--){
    const val=data[r]||data[String(r)]||0;
    const tr=document.createElement('tr');
    tr.innerHTML=`<td class="rl">Row ${r}</td>
      <td><input type="number" step="0.1" value="${val}" id="${id}_r${r}"></td>
      <td style="font-size:10px;color:var(--dim)">${r===8?'Top':r===1?'Bottom':''}</td>`;
    tbl.appendChild(tr);
  }
}
function buildServoInfoTable(c){
  const tbl=document.getElementById('tblServoPositions');
  tbl.innerHTML='<tr><th>Parameter</th><th>Value</th><th>Description</th></tr>';
  [['InX Home',c.inx_home,'Servo 1 home'],['InX Target',c.inx_target2,'Servo 1 pick'],
   ['InY Home',c.iny_home,'Servo 2 home'],['InY Target',c.iny_target2,'Servo 2 safe'],
   ['S3 Push',c.servo3_push_position,'Push pos'],['OutX Home',c.outx_home,'Servo 4 home'],
   ['OutX Target',c.outx_target2,'Servo 4 stack'],['OutY Home',c.outy_home,'Servo 5 home'],
   ['OutY Target',c.outy_target2,'Servo 5 pick']
  ].forEach(([n,v,d])=>{
    if(v===undefined) return;
    const tr=document.createElement('tr');
    tr.innerHTML=`<td class="rl">${n}</td>
      <td style="color:var(--yellow);font-family:monospace">${v??'-'}</td>
      <td style="font-size:10px;color:var(--dim)">${d}</td>`;
    tbl.appendChild(tr);
  });
}
async function saveConfig(table,tblId){
  const pos={};
  for(let r=1;r<=8;r++){
    const inp=document.getElementById(tblId+'_r'+r);
    if(inp) pos[r]=parseFloat(inp.value)||0;
  }
  await api('/api/update_config',{config:{table,positions:pos}});
}

// ── BUILD UI ───────────────────────────────────────────────────────────────
function buildSensors(){
  const g=document.getElementById('sensorGrid');
  for(let i=1;i<=15;i++){
    const d=document.createElement('div');
    d.className='sbtn'; d.id='s'+i; d.onclick=()=>toggleSensor(i);
    const lb=SLB[i];
    d.innerHTML=`<span class="sid">S${i}</span>
      ${lb?`<span class="slb">${lb}</span>`:''}
      <span class="sdot"></span>`;
    g.appendChild(d);
  }
}

function buildServos(){
  const list=document.getElementById('servoList');
  SERVOS.forEach(s=>{
    const d=document.createElement('div');
    d.className='scard';
    d.innerHTML=`
      <div class="scard-title">S${s.id}: ${s.name} <span class="sd">– ${s.desc}</span></div>
      <div class="scard-pos" id="spos${s.id}">-- mm</div>
      <div class="jrow">
        <button class="btn jog-btn"
          onmousedown="jog(${s.id},'-')" onmouseup="jogStop(${s.id})" onmouseleave="jogStop(${s.id})"
          ontouchstart="jog(${s.id},'-')" ontouchend="jogStop(${s.id})">−</button>
        <button class="btn btn-red jog-btn" onclick="jogStop(${s.id})" style="font-size:10px">STOP</button>
        <button class="btn jog-btn"
          onmousedown="jog(${s.id},'+')" onmouseup="jogStop(${s.id})" onmouseleave="jogStop(${s.id})"
          ontouchstart="jog(${s.id},'+')" ontouchend="jogStop(${s.id})">+</button>
      </div>
      <div class="sfull">
        <button class="btn btn-green jog-btn" onclick="homeServo(${s.id})">HOMING</button>
      </div>
      <div class="sfull">
        <button class="btn btn-orange" onclick="clearServo(${s.id})">CLEAR</button>
      </div>
      <div class="prow">
        <input type="number" class="pin jog-btn" id="pi${s.id}" placeholder="0.0" step="0.1">
        <span style="font-size:9px;color:var(--dim)">mm</span>
        <button class="btn btn-accent run-btn" onclick="moveServo(${s.id})"
                style="padding:4px 8px;font-size:10px">RUN</button>
      </div>`;
    list.appendChild(d);
  });
}

// ── TOAST & LOG ────────────────────────────────────────────────────────────
function toast(msg,type='ok'){
  const c=document.getElementById('toasts');
  const t=document.createElement('div');
  t.className='toast'+(type==='error'?' error':'');
  t.textContent=msg; c.appendChild(t);
  setTimeout(()=>t.remove(),3000);
}
function addLog(msg,type='info'){
  const box=document.getElementById('logBox');
  const time=new Date().toLocaleTimeString('vi-VN');
  box.innerHTML+=`<div class="${type}">[${time}] ${msg}</div>`;
  box.scrollTop=box.scrollHeight;
  while(box.children.length>80) box.removeChild(box.firstChild);
}

// ── POLLING ────────────────────────────────────────────────────────────────
async function pollStatus(){
  try{
    const j=await (await fetch('/api/status')).json();
    const st=j.state||'';
    currentState=st.toLowerCase();
    document.getElementById('stateText').textContent=st||'UNKNOWN';
    const dot=document.getElementById('stateDot');
    if(!st||st==='UNKNOWN'){dot.style.background='#666';dot.style.boxShadow='none';}
    else if(st.includes('ERROR')){dot.style.background='var(--red)';dot.style.boxShadow='0 0 8px var(--red)';}
    else if(st==='IDLE'){dot.style.background='var(--orange)';dot.style.boxShadow='0 0 8px var(--orange)';}
    else{dot.style.background='var(--green)';dot.style.boxShadow='0 0 8px var(--green)';}
    updateModeButtons();
  }catch(e){document.getElementById('stateText').textContent='OFFLINE';}
}
function updateModeButtons(){
  const running=currentState!=='idle';
  ['btnAuto','btnJog','btnIdle'].forEach(id=>{
    const b=document.getElementById(id);
    if(b){b.disabled=running; b.style.opacity=running?'0.35':(id.replace('btn','').toLowerCase()===currentMode?'1':'0.45');}
  });
}

async function pollPositions(){
  try{
    const j=await (await fetch('/api/positions',{method:'POST',
      headers:{'Content-Type':'application/json'},body:'{}'})).json();
    if(j.ok&&j.positions){
      for(let id=1;id<=5;id++){
        const el=document.getElementById('spos'+id);
        if(el){const v=j.positions[String(id)];el.textContent=v!==undefined?v.toFixed(2)+' mm':'-- mm';}
      }
    }
  }catch(e){}
}

let _ni=0,_nt=null;
async function pollNotifications(){
  try{
    const j=await(await fetch('/api/notifications',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify({since:_ni})})).json();
    if(j.ok&&j.notifications&&j.notifications.length>0){
      _ni=j.total;
      const last=j.notifications[j.notifications.length-1];
      showNotify(last.level,last.title,last.detail,last.time);
      j.notifications.forEach(n=>addLog(n.title+': '+n.detail,n.level==='error'?'err':'info'));
    }
  }catch(e){}
}

function showNotify(lvl,title,detail,time){
  const bar=document.getElementById('notifyBar');
  bar.className='notify-bar show '+lvl;
  document.getElementById('notifyTitle').textContent=title;
  document.getElementById('notifyDetail').textContent=detail||'';
  document.getElementById('notifyTime').textContent=time||'';
  clearTimeout(_nt);
  if(lvl==='info') _nt=setTimeout(dismissNotify,5000);
}
function dismissNotify(){document.getElementById('notifyBar').className='notify-bar';}
function resetFaults(){
  fetch('/api/reset_faults',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
  addLog('Reset faults sent','info');
}

// ── INIT ───────────────────────────────────────────────────────────────────
buildSensors();
buildServos();
updateModeLock();
pollStatus();
pollPositions();
setInterval(pollStatus,        3000);
setInterval(pollNotifications, 1500);
setInterval(pollPositions,      500);
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
    print("  Cartridge System GUI v2 (Direct ROS2)")
    print("=" * 50)
    print(f"  Local:   http://localhost:{PORT}")
    print(f"  Network: http://{local_ip}:{PORT}")
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