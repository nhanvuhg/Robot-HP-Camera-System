// ─────────────────────────────────────────────────────────────────────────────
// FillHpTab.qml — replacement cho PAGE 4 (FILL HP CONTROL) trong CartridgePage.
// Read-only display + cac control AN TOAN (Start/Stop/Mode/Reconnect/Clear Error).
// CO Y BO QUA cac control rui ro cao: manual valve/cylinder, settings editor,
// servo jog. Khi can them lai, mo file nay, KHONG dung sang CartridgePage.qml.
// ─────────────────────────────────────────────────────────────────────────────
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: tab
    anchors.fill: parent

    // ---- Theme tokens (đồng bộ liquid-glass với CameraPage/CartridgePage/InkTab) ----
    readonly property color cBg:          "transparent"
    readonly property color cPanel:       "#990d1e32"
    readonly property color cPanel2:      "#8806101d"
    readonly property color cBorder:      "#1affffff"
    readonly property color cText:        "#c7dcef"
    readonly property color cMuted:       "#9fb3c8"
    readonly property color cAccent:      "#7fcdf5"
    readonly property color cAccentSoft:  Qt.rgba(0.42, 0.75, 0.95, 0.16)
    readonly property color cOk:          "#3ed0b4"
    readonly property color cOkBg:        Qt.rgba(0.0, 0.90, 0.46, 0.15)
    readonly property color cField:       "#cc081627"
    readonly property color cWarn:        "#f5a623"
    readonly property color cWarnBg:      Qt.rgba(1.0, 0.65, 0.15, 0.15)
    readonly property color cBad:         "#f0735c"
    readonly property color cBadBg:       Qt.rgba(1.0, 0.32, 0.32, 0.15)
    readonly property color cIdle:        "#74899f"
    readonly property color cIdleBg:      Qt.rgba(0.32, 0.38, 0.44, 0.15)

    readonly property string monoFamily:  "JetBrains Mono, DejaVu Sans Mono, Consolas, monospace"

    // ---- Parsers (mirror page4Root logic) ----
    function parseKvPipe(raw) {
        var out = {};
        if (!raw) return out;
        var parts = String(raw).split("|");
        for (var i = 0; i < parts.length; i++) {
            var part = parts[i];
            var idx = part.indexOf('=');
            var alt = part.indexOf(':');
            var pos = idx >= 0 ? idx : alt;
            if (pos >= 0) out[part.substring(0, pos).trim()] = part.substring(pos + 1).trim();
        }
        return out;
    }
    function parseKvComma(raw) {
        var out = {};
        if (!raw) return out;
        var parts = String(raw).split(",");
        for (var i = 0; i < parts.length; i++) {
            var part = parts[i];
            var idx = part.indexOf('=');
            var alt = part.indexOf(':');
            var pos = idx >= 0 ? idx : alt;
            if (pos >= 0) {
                var k = part.substring(0, pos).trim();
                var v = part.substring(pos + 1).trim();
                if (k === "safety_ch1") {
                    k = "safety_i_4_i04";
                } else if (k === "safety_ch2") {
                    k = "safety_i_5_i04";
                }
                out[k] = v;
            }
        }
        return out;
    }
    function classifyState(rawVal) {
        var v = String(rawVal || "").toLowerCase();
        if (v === "on" || v === "true" || v === "ok" || v === "ready" || v === "connected") return "on";
        if (v === "off" || v === "false" || v === "error" || v === "fault" || v === "disconnected") return "off";
        return "mid";
    }
    function getSensorLabel(key) {
        var mapping = {
            "start_button": "Start button",
            "stop_button": "Stop button",
            "optical_sensor": "Optical sensor",
            "ball_feed_down": "Ball feed down",
            "ball_feed_up": "Ball feed up",
            "ball_push_down": "Ball push down",
            "ball_push_up": "Ball push up",
            "chamber_open": "Chamber open",
            "chamber_closed": "Chamber closed",
            "seal_pin_down": "Seal pin down",
            "seal_pin_up": "Seal pin up",
            "fix_cylinder_down": "Fix cyl down",
            "fix_cylinder_up": "Fix cyl up",
            "ball_box_empty": "Ball box empty",
            "safety_i_4_i04": "Safety I_4_i04",
            "safety_i_5_i04": "Safety I_5_i04",
            "safety_area_clear": "Safety area clear"
        };
        if (mapping[key] !== undefined) {
            return mapping[key];
        }
        if (key.indexOf("mag_") === 0) {
            return "Mag " + key.substring(4);
        }
        if (key.indexOf("tube_") === 0) {
            return "Tube " + key.substring(5);
        }
        return key;
    }
    function classifyPressure(val, lowT, highT, limitT) {
        if (val < lowT) return "low";
        if (val >= limitT) return "limit";
        if (val >= highT) return "high";
        return "ok";
    }
    // Parse valveState format "v1=open:on,v2=ink:on,..." -> { v1: {label, state} }
    function parseValveState(rawStr) {
        var out = {};
        if (!rawStr) return out;
        var parts = String(rawStr).split(",");
        for (var i = 0; i < parts.length; i++) {
            var p = parts[i].split("=");
            if (p.length === 2) {
                var chunks = p[1].trim().split(":");
                out[p[0].trim()] = { label: chunks[0] || "-", state: chunks[1] || "mid" };
            }
        }
        return out;
    }
    // Parse alert string format "LEVEL:WARN|TIME:08:52:03|AREA:CHAMBER_SAFETY|MESSAGE:..."
    function parseAlertString(rawStr) {
        var out = { level: "", time: "", area: "", message: "", sev: "info", raw: rawStr };
        if (!rawStr) return out;
        var parts = String(rawStr).split("|");
        for (var i = 0; i < parts.length; i++) {
            var p = parts[i];
            var idx = p.indexOf(':');
            if (idx < 0) continue;
            var key = p.substring(0, idx).trim().toUpperCase();
            var val = p.substring(idx + 1).trim();
            if (key === "LEVEL") out.level = val;
            else if (key === "TIME") out.time = val;
            else if (key === "AREA") out.area = val;
            else if (key === "MESSAGE") out.message = val;
        }
        // Classify severity
        var lvl = out.level.toUpperCase();
        var msg = (out.message + " " + out.area).toLowerCase();
        if (lvl === "CRITICAL" || msg.indexOf("critical") >= 0 || msg.indexOf("emergency") >= 0) out.sev = "critical";
        else if (lvl === "ERROR" || lvl === "FAULT" || msg.indexOf("error") >= 0 || msg.indexOf("fault") >= 0 || msg.indexOf("timeout") >= 0) out.sev = "error";
        else if (lvl === "WARN" || lvl === "WARNING" || msg.indexOf("warn") >= 0) out.sev = "warning";
        else out.sev = "info";
        if (!out.message) out.message = rawStr;
        return out;
    }

    // Action label matching with aliases (open/close/on/off/air/ink/up/down/chamber/waste)
    function actionMatches(label, action) {
        if (!label || !action) return false;
        var text = String(label).toLowerCase().trim();
        var act  = String(action).toLowerCase().trim();
        var aliases = {
            "open":["open","mở"], "close":["close","đóng"],
            "on":["on","bật"], "off":["off","tắt"],
            "air":["air"], "ink":["ink"], "chamber":["chamber"], "waste":["waste"],
            "up":["up","lên"], "down":["down","xuống"]
        };
        var list = aliases[act] || [act];
        return list.indexOf(text) !== -1;
    }

    // ---- Derived data from hpController ----
    property var sysMap:      parseKvPipe(hpController.systemStatus)
    property var hwMap:       parseKvPipe(hpController.hwStatus)
    property var inputsMap:   parseKvComma(hpController.inputState)

    // ---- Alert center state (accumulate alerts client-side) ----
    property var alertHistory: []          // [{level, time, area, message, sev, raw}, ...]
    property string lastAlertRaw: ""
    property string alertFilter: "all"     // all | critical | error | warning | info
    readonly property var filteredAlerts: alertHistory.filter(function(a) {
        if (tab.alertFilter === "all") return true;
        return a.sev === tab.alertFilter;
    })
    readonly property int countError:    alertHistory.filter(function(a) { return a.sev === "error" || a.sev === "critical" }).length
    readonly property int countWarning:  alertHistory.filter(function(a) { return a.sev === "warning" }).length
    readonly property int countInfo:     alertHistory.filter(function(a) { return a.sev === "info" }).length

    Connections {
        target: hpController
        function onErrorStatusChanged() {
            var raw = hpController.errorStatus;
            if (!raw || raw === "OK" || raw === "-" || raw === tab.lastAlertRaw) return;
            tab.lastAlertRaw = raw;
            var parsed = tab.parseAlertString(raw);
            var hist = tab.alertHistory.slice();
            hist.unshift(parsed);
            if (hist.length > 50) hist = hist.slice(0, 50);
            tab.alertHistory = hist;
        }
        function onManualResponseChanged() {
            var msg = hpController.manualResponse;
            if (!msg || msg === "-" || msg === tab.lastActionRaw) return;
            tab.lastActionRaw = msg;
            var now = new Date();
            var hh = String(now.getHours()).padStart(2, '0');
            var mm = String(now.getMinutes()).padStart(2, '0');
            var ss = String(now.getSeconds()).padStart(2, '0');
            var line = hh + ":" + mm + ":" + ss + " · " + msg;
            var log = tab.actionLog.slice();
            log.unshift(line);
            if (log.length > 30) log = log.slice(0, 30);
            tab.actionLog = log;
        }
    }

    // ---- Action log state ----
    property var actionLog: []
    property string lastActionRaw: ""
    // Lay danh sach key inputs, loc bo mag_index va tube_index de lay 33 binary sensors
    property var filteredInputKeys: Object.keys(inputsMap).filter(function(k) {
        return k !== "mag_index" && k !== "tube_index"
    })
    property var valvesMap:   parseValveState(hpController.valveState)
    property var settingsMap: parseKvComma(hpController.pressureThresholds)
    property int activeSettingsTab: 0

    // ---- Static models ported from PAGE 4 (DO NOT change id/statusKey/a/b
    //      — must match hp_controller publishManual topic format) ----
    property var valveModel: [
        { id: "valve1",        statusKey: "v1",      label: "V1 (Main)",       a: "open",    b: "close" },
        { id: "valve2",        statusKey: "v2",      label: "V2 (Air/Ink)",    a: "air",     b: "ink" },
        { id: "valve4",        statusKey: "v4",      label: "V4 (Main Air)",   a: "on",      b: "off" },
        { id: "valve5",        statusKey: "v5",      label: "V5 (Tank ret)",   a: "open",    b: "close" },
        { id: "valve6",        statusKey: "v6",      label: "V6 (Clean)",      a: "open",    b: "close" },
        { id: "valve7",        statusKey: "v7",      label: "V7 (Purge Ink)",  a: "on",      b: "off" },
        { id: "valve8",        statusKey: "v8",      label: "V8 (Purge Air)",  a: "on",      b: "off" },
        { id: "valve9",        statusKey: "v9",      label: "V9 (Waste/Ch)",   a: "chamber", b: "waste" },
        { id: "valve_chamber", statusKey: "vchamber",label: "V Chamber",       a: "on",      b: "off" },
        { id: "vacuum",        statusKey: "pump",    label: "Vacuum Pump",     a: "on",      b: "off" }
    ]
    property var cylinderModel: [
        { id: "chamber",   statusKey: "cyl_ch",   label: "Chamber Cyl",   a: "close", b: "open" },
        { id: "cartridge", statusKey: "cyl_cart", label: "Cartridge Cyl", a: "down",  b: "up" },
        { id: "ball_feed", statusKey: "cyl_bf",   label: "Ball Feed Cyl", a: "up",    b: "down" },
        { id: "ball_push", statusKey: "cyl_bp",   label: "Ball Push Cyl", a: "up",    b: "down" },
        { id: "seal_pin",  statusKey: "cyl_seal", label: "Seal Pin Cyl",  a: "up",    b: "down" },
        { id: "fix_cyl",   statusKey: "cyl_fix",  label: "Fix Cylinder",  a: "up",    b: "down" }
    ]
    property var settingGroups: [
        { id: "auto", label: "Auto", items: [
            { topic: "cart_fix",          type: "threshold", label: "Cart Fix Pressure", min: 0,    max: 1200, unit: "mbar" },
            { topic: "chamber_vac",       type: "threshold", label: "Chamber Vacuum",    min: 0,    max: 1200, unit: "mbar" },
            { topic: "chamber_leak",      type: "threshold", label: "Chamber Leak",      min: 0,    max: 1200, unit: "mbar" },
            { topic: "cart_vac",          type: "threshold", label: "Cartridge Vacuum",  min: 0,    max: 1200, unit: "mbar" },
            { topic: "cart_leak",         type: "threshold", label: "Cartridge Leak",    min: 0,    max: 1200, unit: "mbar" },
            { topic: "pressure_balance",  type: "threshold", label: "Pressure Balance",  min: 0,    max: 1200, unit: "mbar" },
            { topic: "chamber_vent",      type: "threshold", label: "Chamber Vent",      min: 0,    max: 1200, unit: "mbar" },
            { topic: "fill_compensation", type: "float",     label: "Fill Compensation", min: -999, max: 999,  unit: "ml" }
        ]},
        { id: "clean", label: "Clean", items: [
            { topic: "cr_cart_vac",         type: "threshold", label: "Cartridge Vacuum", min: 0, max: 1200, unit: "mbar" },
            { topic: "cr_cart_leak",        type: "threshold", label: "Cartridge Leak",   min: 0, max: 1200, unit: "mbar" },
            { topic: "cr_pressure_balance", type: "threshold", label: "Pressure Balance", min: 0, max: 1200, unit: "mbar" },
            { topic: "cr_volume",           type: "float",     label: "CR Volume",        min: 0, max: 9999, unit: "ml" },
            { topic: "cr_flow_rate",        type: "float",     label: "CR Flow",          min: 0, max: 100,  unit: "ml/s" },
            { topic: "cr_loading_rate",     type: "float",     label: "CR Loading",       min: 0, max: 100,  unit: "ml/s" },
            { topic: "cr_cycles",           type: "int",       label: "CR Cycles",        min: 1, max: 999,  unit: "" }
        ]},
        { id: "dosing", label: "Dosing", items: [
            { topic: "dosing_volume",       type: "float", label: "Dosing Volume",  min: 0, max: 9999, unit: "ml" },
            { topic: "dosing_flow_rate",    type: "float", label: "Dosing Flow",    min: 0, max: 100,  unit: "ml/s" },
            { topic: "dosing_loading_rate", type: "float", label: "Dosing Loading", min: 0, max: 100,  unit: "ml/s" }
        ]},
        { id: "pwm", label: "PWM/Tank", items: [
            { topic: "base_pwm",            type: "int",   label: "Base PWM",         min: 0,   max: 100,  unit: "%" },
            { topic: "chamber_vent_pwm",    type: "int",   label: "Chamber Vent PWM", min: 0,   max: 100,  unit: "%" },
            { topic: "cr_valve10_pwm",      type: "int",   label: "CR Valve10 PWM",   min: 0,   max: 100,  unit: "%" },
            { topic: "cr_valve10_duration", type: "float", label: "CR Valve10 Time",  min: 0.5, max: 30,   unit: "s" },
            { topic: "cr_valve5_duration",  type: "float", label: "CR Valve5 Time",   min: 0.5, max: 30,   unit: "s" },
            { topic: "cr_return_duration",  type: "float", label: "CR Return Time",   min: 0.5, max: 30,   unit: "s" },
            { topic: "tank_min",            type: "float", label: "Tank Min",         min: 0,   max: 1000, unit: "mbar" },
            { topic: "tank_max",            type: "float", label: "Tank Max",         min: 0,   max: 1000, unit: "mbar" }
        ]}
    ]
    // Helper: publish setting based on type (matches old PAGE 4 wire-up)
    function publishSetting(item, valStr) {
        var n = parseFloat(valStr);
        if (isNaN(n)) return;
        if (item.type === "threshold") hpController.publishString("pressure_thresholds_set", item.topic + ":" + valStr);
        else if (item.type === "int")  hpController.publishInt(item.topic, Math.round(n));
        else                            hpController.publishFloat(item.topic, n);
    }
    property string modeStr:   (sysMap["MODE"] || hpController.modeStatus || "-").toString().toUpperCase()
    property string stateStr:  sysMap["STATE"] || hpController.systemStatus || "-"
    property string cycleStr:  sysMap["CYCLE"] || "-"
    property string volumeStr: sysMap["VOLUME"] || "-"
    property bool   running:   String(sysMap["RUNNING"]).toLowerCase() === "true"
    property bool   hasError:  hpController.errorStatus && hpController.errorStatus !== "OK"
                              && hpController.errorStatus !== "-"

    Rectangle { anchors.fill: parent; color: cBg }

    // ====================================================================
    //  HEADER BAR (sticky, top)
    // ====================================================================
    Rectangle {
        id: headerBar
        anchors { top: parent.top; left: parent.left; right: parent.right }
        height: 60
        color: "#06101d"

        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 14
            anchors.rightMargin: 14
            spacing: 8

            Text {
                text: "Fill HP Control"
                color: cText; font.pixelSize: 26; font.bold: true
            }

            Item { Layout.fillWidth: true }

            TbBtn {
                lbl: "Start"; variant: "primary"
                onClicked: hpController.publishScreenControl("start")
            }

            CycleChip {
                value: hpController.ballCycleTime || "00:00"
                isRun: tab.running
                Layout.leftMargin: 6
                Layout.rightMargin: 6
            }

            TbBtn {
                lbl: "Stop"; variant: "danger"
                onClicked: hpController.publishScreenControl("stop")
            }

            Item { width: 8 }

            ModeBtn { lbl: "Manual"; active: tab.modeStr === "MANUAL"; onClicked: hpController.publishMode(2) }
            ModeBtn { lbl: "Auto";   active: tab.modeStr === "AUTO";   onClicked: hpController.publishMode(0) }
            ModeBtn { lbl: "Clean";  active: tab.modeStr === "CLEAN";  onClicked: hpController.publishMode(1) }
            ModeBtn { lbl: "Prefill"; active: tab.modeStr === "PREFILL"; onClicked: hpController.publishMode(3) }

            Item { width: 8 }

            TbBtn {
                lbl: "Reconnect CPX"; variant: "warn"
                onClicked: hpController.publishString("reconnect_cmd", "cpx")
            }

            TbBtn {
                lbl: "Clear Error"; variant: "warn"
                visible: tab.hasError
                onClicked: hpController.publishString("error_control", "clear")
            }
        }

        Rectangle {
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            height: 1
            color: cBorder
        }
    }

    // ====================================================================
    //  BODY (scrollable)
    // ====================================================================
    Flickable {
        id: bodyScrollView
        anchors { top: headerBar.bottom; left: parent.left; right: parent.right; bottom: parent.bottom }
        contentWidth: width
        contentHeight: mainCol.implicitHeight + 24
        clip: true
        ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

        ColumnLayout {
            id: mainCol
            x: 12; y: 12
            width: parent.width - 24
            spacing: 12

            // -- Error banner --
            Sect {
                Layout.fillWidth: true
                visible: tab.hasError
                bgColor: Qt.rgba(1.0, 0.32, 0.32, 0.12)
                borderColor: cBad
                noTitle: true

                RowLayout {
                    spacing: 12
                    width: parent.width

                    Text { text: "⛔"; font.pixelSize: 30 }
                    ColumnLayout {
                        Layout.fillWidth: true; spacing: 2
                        Text { text: "CANH BAO HE THONG"; color: cBad; font.bold: true; font.pixelSize: 22 }
                        Text {
                            text: hpController.errorStatus || "-"
                            color: cText; font.pixelSize: 21
                            wrapMode: Text.WrapAtWordBoundaryOrAnywhere; Layout.fillWidth: true
                        }
                    }
                    TbBtn {
                        lbl: "Xac nhan & xoa"; variant: "danger"
                        onClicked: hpController.publishString("error_control", "clear")
                    }
                }
            }

            // -- Body row: sidebar + content --
            RowLayout {
                Layout.fillWidth: true
                spacing: 12

                // ====== SIDEBAR 340 ======
                ColumnLayout {
                    Layout.preferredWidth: 340
                    Layout.minimumWidth: 340
                    Layout.maximumWidth: 340
                    Layout.fillWidth: false
                    Layout.alignment: Qt.AlignTop | Qt.AlignLeft
                    spacing: 12

                    Sect {
                        title: "Tong quan"
                        Layout.fillWidth: true
                        ColumnLayout {
                            width: parent.width; spacing: 7
                            Kv { lbl: "Mode";          val: tab.modeStr }
                            Kv { lbl: "State";         val: tab.stateStr }
                            Kv { lbl: "Cycle";         val: tab.cycleStr }
                            Kv { lbl: "Volume";        val: tab.volumeStr }
                            Kv {
                                lbl: "Running"
                                chip: StatusChip {
                                    state: tab.running ? "on" : "off"
                                    label: tab.running ? "In Process" : "Stop"
                                }
                            }
                            Kv {
                                lbl: "Phan hoi cuoi"
                                val: hpController.manualResponse || "-"
                            }
                        }
                    }

                    Sect {
                        title: "Trang thai may"
                        Layout.fillWidth: true
                        ColumnLayout {
                            width: parent.width; spacing: 7
                            Kv { lbl: "Dosing";      val: hpController.dosingStatus || "-" }
                            Kv { lbl: "Fill";        val: hpController.fillStatus || "-" }
                            Kv { lbl: "Fix";         val: hpController.fixStatus || "-" }
                            Kv { lbl: "Clean refill"; val: hpController.crStatus || "-" }
                            Kv { lbl: "Servo raw";   val: hpController.servoPositionRaw.toFixed(0) }
                            Kv { lbl: "Servo mm";    val: hpController.servoPosition.toFixed(2) + " mm" }
                            Kv { lbl: "Base PWM";    val: hpController.basePwmStatus + " %" }
                        }
                    }

                    Sect {
                        title: "Qua trinh"
                        Layout.fillWidth: true
                        ColumnLayout {
                            width: parent.width; spacing: 7
                            Kv { lbl: "Auto fill";    val: hpController.fillStatus || "-" }
                            Kv { lbl: "Dosing";       val: hpController.dosingStatus || "-" }
                            Kv { lbl: "Clean refill"; val: hpController.crStatus || "-" }
                            Kv { lbl: "Cycle / Vol";  val: tab.cycleStr + "  ·  " + tab.volumeStr }
                        }
                    }

                    Sect {
                        title: "Hardware"
                        Layout.fillWidth: true
                        visible: Object.keys(tab.hwMap).length > 0
                        Flow {
                            spacing: 6; width: parent.width
                            Repeater {
                                model: Object.keys(tab.hwMap)
                                Chip {
                                    name:  modelData
                                    state: classifyState(tab.hwMap[modelData])
                                    label: String(tab.hwMap[modelData])
                                }
                            }
                        }
                    }
                }

                // ====== CONTENT ======
                ColumnLayout {
                    Layout.fillWidth: true
                    Layout.minimumWidth: 400
                    Layout.maximumWidth: 1500
                    Layout.alignment: Qt.AlignTop
                    spacing: 12

                    // ─── TOP BLOCK: [Alert + (Analog|Cartridge)] | Valves | Cylinders ───
                    RowLayout {
                        Layout.fillWidth: true
                        Layout.alignment: Qt.AlignLeft
                        spacing: 12

                        // LEFT column: Alert center (full width 812) + Analog|Cart row
                        ColumnLayout {
                            Layout.preferredWidth: 812
                            Layout.maximumWidth: 812
                            Layout.alignment: Qt.AlignTop
                            spacing: 12

                            // Alert center
                            Sect {
                                title: "Trung tam canh bao"
                                Layout.fillWidth: true

                                ColumnLayout {
                                    width: parent.width; spacing: 8

                                    RowLayout {
                                        width: parent.width; spacing: 8
                                        Text { text: "Tat ca: " + tab.alertHistory.length;     color: cMuted; font.pixelSize: 16 }
                                        Text { text: "Loi: "    + tab.countError;              color: cBad;   font.pixelSize: 16; font.bold: true }
                                        Text { text: "Canh bao: " + tab.countWarning;          color: cWarn;  font.pixelSize: 16; font.bold: true }
                                        Text { text: "Thong tin: " + tab.countInfo;            color: cAccent; font.pixelSize: 16 }
                                        Item { Layout.fillWidth: true }
                                        TbBtn { lbl: "Xoa lich su"; onClicked: { tab.alertHistory = []; tab.lastAlertRaw = "" } }
                                    }

                                    RowLayout {
                                        width: parent.width; spacing: 6
                                        Repeater {
                                            model: [
                                                { key: "all",      lbl: "Tat ca" },
                                                { key: "critical", lbl: "Nghiem trong" },
                                                { key: "error",    lbl: "Loi" },
                                                { key: "warning",  lbl: "Canh bao" },
                                                { key: "info",     lbl: "Thong tin" }
                                            ]
                                            TbBtn {
                                                lbl: modelData.lbl
                                                variant: tab.alertFilter === modelData.key ? "primary" : "default"
                                                onClicked: tab.alertFilter = modelData.key
                                            }
                                        }
                                    }

                                    Rectangle {
                                        visible: tab.alertHistory.length === 0
                                        Layout.fillWidth: true
                                        implicitHeight: 60
                                        radius: 6
                                        color: cOkBg
                                        border.color: cOk; border.width: 1
                                        Text {
                                            anchors.centerIn: parent
                                            text: "✅ Khong co canh bao. He thong hoat dong binh thuong."
                                            color: cOk; font.pixelSize: 16; font.bold: true
                                        }
                                    }

                                    Item {
                                        visible: tab.alertHistory.length > 0
                                        width: parent.width
                                        implicitHeight: Math.min(360, alertColumn.implicitHeight)
                                        ScrollView {
                                            anchors.fill: parent
                                            clip: true
                                            ColumnLayout {
                                                id: alertColumn
                                                width: parent.width
                                                spacing: 6
                                                Repeater {
                                                    model: tab.filteredAlerts
                                                    AlertRow {
                                                        sev:     modelData.sev
                                                        time:    modelData.time
                                                        area:    modelData.area
                                                        message: modelData.message
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            // Cylinders & Servos row
                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 12

                                // Cylinders (60% width of 812 = 487.2)
                                Sect {
                                    title: "Cylinders (manual only)"
                                    Layout.preferredWidth: 487
                                    Layout.fillHeight: true
                                    Layout.alignment: Qt.AlignTop

                                    Item {
                                        anchors.fill: parent
                                        enabled: tab.modeStr === "MANUAL"
                                        opacity: tab.modeStr === "MANUAL" ? 1.0 : 0.4
                                        Behavior on opacity { NumberAnimation { duration: 150 } }
                                        ColumnLayout {
                                            anchors.fill: parent
                                            spacing: 6
                                            Repeater {
                                                model: tab.cylinderModel
                                                IoToggle {
                                                    Layout.fillWidth: true
                                                    Layout.fillHeight: true
                                                    ioId:      modelData.id
                                                    statusKey: modelData.statusKey
                                                    ioLabel:   modelData.label
                                                    actA:      modelData.a
                                                    actB:      modelData.b
                                                }
                                            }
                                        }
                                    }
                                }

                                // Servo & Motor (40% width of 812 = 324.8)
                                Sect {
                                    title: "Servo & Motor"
                                    Layout.preferredWidth: 325
                                    Layout.fillHeight: true
                                    Layout.alignment: Qt.AlignTop

                                    ColumnLayout {
                                        width: parent.width
                                        spacing: 8

                                        // Position display
                                        Text {
                                            text: "Pos: " + hpController.servoPosition.toFixed(2) + " mm"
                                            color: cText
                                            font.pixelSize: 18
                                            font.bold: true
                                            font.family: monoFamily
                                            Layout.alignment: Qt.AlignHCenter
                                        }

                                        // Commands Grid (Vertical stack of buttons)
                                        ColumnLayout {
                                            spacing: 5
                                            Layout.fillWidth: true

                                            TbBtn {
                                                lbl: "Enable"
                                                Layout.fillWidth: true
                                                onClicked: hpController.publishString("servo_command", "enable")
                                            }
                                            TbBtn {
                                                lbl: "Disable"
                                                Layout.fillWidth: true
                                                onClicked: hpController.publishString("servo_command", "disable")
                                            }
                                            TbBtn {
                                                lbl: "Home"
                                                variant: "primary"
                                                Layout.fillWidth: true
                                                onClicked: hpController.publishString("servo_command", "home")
                                            }
                                            TbBtn {
                                                lbl: "Reset Fault"
                                                variant: "danger"
                                                Layout.fillWidth: true
                                                onClicked: hpController.publishString("servo_command", "reset_fault")
                                            }
                                        }

                                        // Jog Controls (Vertical layout with STOP button at the bottom)
                                        ColumnLayout {
                                            spacing: 5
                                            Layout.fillWidth: true
                                            enabled: tab.modeStr === "MANUAL"
                                            opacity: tab.modeStr === "MANUAL" ? 1.0 : 0.4
                                            Behavior on opacity { NumberAnimation { duration: 150 } }

                                            RowLayout {
                                                spacing: 6
                                                Layout.fillWidth: true
                                                JogBtn {
                                                    lbl: "◀ JOG REV"
                                                    dir: "rev"
                                                    variant: "warn"
                                                    Layout.fillWidth: true
                                                }
                                                JogBtn {
                                                    lbl: "JOG FWD ▶"
                                                    dir: "fwd"
                                                    variant: "primary"
                                                    Layout.fillWidth: true
                                                }
                                            }
                                            TbBtn {
                                                lbl: "STOP"
                                                variant: "danger"
                                                Layout.fillWidth: true
                                                onClicked: hpController.publishString("servo_jog", "stop")
                                            }
                                        }

                                        // PWM Controls (Compact vertical stacks)
                                        ColumnLayout {
                                            Layout.fillWidth: true
                                            spacing: 4

                                            RowLayout {
                                                width: parent.width; spacing: 6
                                                Text { text: "Base PWM"; color: cText; font.pixelSize: 14; font.bold: true; Layout.fillWidth: true }
                                                PwmInput { id: basePwmIn; valueText: hpController.basePwmStatus.toString(); Layout.preferredWidth: 60 }
                                                TbBtn {
                                                    lbl: "Set"; variant: "primary"
                                                    Layout.preferredWidth: 50
                                                    onClicked: {
                                                        var v = parseInt(basePwmIn.valueText);
                                                        if (!isNaN(v)) hpController.publishInt("base_pwm", Math.max(0, Math.min(100, v)));
                                                    }
                                                }
                                            }
                                            RowLayout {
                                                width: parent.width; spacing: 6
                                                enabled: tab.modeStr === "MANUAL"
                                                opacity: tab.modeStr === "MANUAL" ? 1.0 : 0.4
                                                Text { text: "V10 PWM"; color: cText; font.pixelSize: 14; font.bold: true; Layout.fillWidth: true }
                                                PwmInput { id: v10In; valueText: (tab.valvesMap["v10"] && tab.valvesMap["v10"].label) ? tab.valvesMap["v10"].label.replace("%","") : "0"; Layout.preferredWidth: 60 }
                                                TbBtn {
                                                    lbl: "Set"; variant: "primary"
                                                    Layout.preferredWidth: 50
                                                    onClicked: {
                                                        var v = parseInt(v10In.valueText);
                                                        if (!isNaN(v)) hpController.publishManual("valve10", String(Math.max(0, Math.min(100, v))));
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                        } // end LEFT column

                        // Valves (MIDDLE, fills height of left column)
                        Sect {
                            title: "Valves (manual only)"
                            Layout.preferredWidth: 320
                            Layout.maximumWidth: 320
                            Layout.fillHeight: true
                            Layout.minimumHeight: implicitHeight
                            Layout.alignment: Qt.AlignTop
                            Item {
                                anchors.fill: parent
                                enabled: tab.modeStr === "MANUAL"
                                opacity: tab.modeStr === "MANUAL" ? 1.0 : 0.4
                                Behavior on opacity { NumberAnimation { duration: 150 } }
                                ColumnLayout {
                                    anchors.fill: parent
                                    spacing: 3
                                    Repeater {
                                        model: tab.valveModel
                                        IoToggle {
                                            Layout.fillWidth: true
                                            Layout.fillHeight: true
                                            ioId:      modelData.id
                                            statusKey: modelData.statusKey
                                            ioLabel:   modelData.label
                                            actA:      modelData.a
                                            actB:      modelData.b
                                        }
                                    }
                                }
                            }
                        }

                        // Inputs (read-only grid) - SENSOR SIGNAL FILL MACHINE
                        Sect {
                            id: sensorSect
                            title: "SENSOR SIGNAL FILL MACHINE"
                            Layout.preferredWidth: 400
                            Layout.maximumWidth: 400
                            Layout.fillHeight: true
                            Layout.minimumHeight: implicitHeight
                            Layout.alignment: Qt.AlignTop

                            Grid {
                                width: sensorSect.width - 24
                                columns: 3
                                spacing: 6

                                Repeater {
                                    model: [
                                        "start_button",
                                        "stop_button",
                                        "optical_sensor",
                                        "ball_feed_down",
                                        "ball_feed_up",
                                        "ball_push_down",
                                        "ball_push_up",
                                        "chamber_open",
                                        "chamber_closed",
                                        "seal_pin_down",
                                        "seal_pin_up",
                                        "fix_cylinder_down",
                                        "fix_cylinder_up",
                                        "ball_box_empty",
                                        "safety_i_4_i04",
                                        "safety_i_5_i04",
                                        "safety_area_clear",
                                        "mag_1",
                                        "mag_2",
                                        "mag_3",
                                        "mag_4",
                                        "mag_5",
                                        "mag_6",
                                        "mag_7",
                                        "mag_8",
                                        "tube_1",
                                        "tube_2",
                                        "tube_3",
                                        "tube_4",
                                        "tube_5",
                                        "tube_6",
                                        "tube_7",
                                        "tube_8"
                                    ]
                                    delegate: Rectangle {
                                        id: sBtn
                                        property bool on_: {
                                            var rawVal = tab.inputsMap[modelData];
                                            return classifyState(rawVal) === "on";
                                        }

                                        width: Math.floor((sensorSect.width - 36) / 3)
                                        height: 48
                                        radius: 4
                                        color: on_ ? "#081627" : "transparent"
                                        border.color: on_ ? tab.cAccent : tab.cBorder
                                        border.width: 1
                                        Behavior on color       { ColorAnimation { duration: 150 } }
                                        Behavior on border.color { ColorAnimation { duration: 150 } }

                                        Column {
                                            anchors.centerIn: parent
                                            width: parent.width - 6
                                            spacing: 4

                                            Text {
                                                width: parent.width
                                                text: getSensorLabel(modelData)
                                                color: sBtn.on_ ? tab.cAccent : tab.cText
                                                font.pixelSize: 10
                                                font.bold: true
                                                wrapMode: Text.WrapAnywhere
                                                horizontalAlignment: Text.AlignHCenter
                                            }

                                            Rectangle {
                                                id: dotIndicator
                                                width: 6; height: 6; radius: 3
                                                color: sBtn.on_ ? tab.cAccent : "#163a52"
                                                anchors.horizontalCenter: parent.horizontalCenter

                                                Repeater {
                                                    model: 2
                                                    delegate: Rectangle {
                                                        id: ripple
                                                        anchors.centerIn: parent
                                                        width: 8; height: 8; radius: 4
                                                        color: "transparent"
                                                        border.color: tab.cAccent
                                                        border.width: 1
                                                        opacity: 0
                                                        visible: sBtn.on_

                                                        SequentialAnimation {
                                                            running: sBtn.on_
                                                            loops: Animation.Infinite
                                                            PauseAnimation { duration: index * 1000 }
                                                            ParallelAnimation {
                                                                 NumberAnimation {
                                                                     target: ripple
                                                                     property: "opacity"
                                                                     from: 0.8; to: 0.0
                                                                     duration: 1500
                                                                     easing.type: Easing.OutQuad
                                                                 }
                                                                 NumberAnimation {
                                                                     target: ripple
                                                                     property: "scale"
                                                                     from: 1.0; to: 4.0
                                                                     duration: 1500
                                                                     easing.type: Easing.OutQuad
                                                                 }
                                                            }
                                                            PauseAnimation { duration: (1 - index) * 1000 }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    } // end TOP BLOCK





                    // -- Settings and Raw Status row --
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 12

                        // Settings (tabs + grid)
                        Sect {
                            title: "Thong so dieu khien"
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            Layout.alignment: Qt.AlignTop

                            ColumnLayout {
                                width: parent.width; spacing: 8

                                RowLayout {
                                    width: parent.width; spacing: 6
                                    Repeater {
                                        model: tab.settingGroups
                                        TbBtn {
                                            lbl: modelData.label
                                            variant: tab.activeSettingsTab === index ? "primary" : "default"
                                            Layout.fillWidth: true
                                            onClicked: tab.activeSettingsTab = index
                                        }
                                    }
                                }

                                // Restore defaults dashed-like box
                                Rectangle {
                                    Layout.fillWidth: true
                                    implicitHeight: 85
                                    color: "transparent"
                                    border.color: "#2a1c08" // Dark warning color
                                    border.width: 1
                                    radius: 6

                                    ColumnLayout {
                                        anchors.centerIn: parent
                                        spacing: 6

                                        TbBtn {
                                            lbl: "♻️ Khoi phuc mac dinh"
                                            variant: "warn"
                                            Layout.alignment: Qt.AlignHCenter
                                            onClicked: hpController.publishString("parameters_control", "reset_defaults")
                                        }
                                        Text {
                                            text: "Dua TAT CA thong so ve gia tri goc trong code (can quyen Admin)"
                                            color: cMuted
                                            font.pixelSize: 13
                                            Layout.alignment: Qt.AlignHCenter
                                        }
                                    }
                                }

                                // Grid auto-fit settings rows for active tab
                                Grid {
                                    id: settingsGrid
                                    width: parent.width
                                    columns: Math.max(1, Math.floor(width / 320))
                                    spacing: 6
                                    Repeater {
                                        model: tab.settingGroups[tab.activeSettingsTab].items
                                        SettingRow {
                                            width: (settingsGrid.width - settingsGrid.spacing * (settingsGrid.columns - 1)) / settingsGrid.columns
                                            item: modelData
                                            currentVal: tab.settingsMap[modelData.topic] || ""
                                        }
                                    }
                                }
                            }
                        }

                        // Raw Status
                        Sect {
                            title: "RAW STATUS"
                            Layout.preferredWidth: 350
                            Layout.fillHeight: true
                            Layout.alignment: Qt.AlignTop

                            Rectangle {
                                width: parent.width
                                implicitHeight: 200
                                color: "#081627"
                                border.color: "#163a52"
                                border.width: 1
                                radius: 6

                                ScrollView {
                                    anchors.fill: parent
                                    clip: true
                                    padding: 8

                                    Text {
                                        width: parent.width - 16
                                        text: "servo_position_raw=" + hpController.servoPositionRaw + "\n" +
                                              "servo_position=" + hpController.servoPosition + "\n" +
                                              "pressure_thresholds=" + hpController.pressureThresholds
                                        color: "#9fb3c8"
                                        font.pixelSize: 12
                                        font.family: monoFamily
                                        wrapMode: Text.Wrap
                                    }
                                }
                            }
                        }
                    }

                    // -- Action log (timestamped manual command responses) --
                    Sect {
                        title: "Action log"
                        Layout.fillWidth: true
                        visible: tab.actionLog.length > 0
                        ColumnLayout {
                            width: parent.width; spacing: 4
                            RowLayout {
                                width: parent.width
                                Text { text: tab.actionLog.length + " thao tac gan day"; color: cMuted; font.pixelSize: 14 }
                                Item { Layout.fillWidth: true }
                                TbBtn { lbl: "Xoa log"; onClicked: { tab.actionLog = []; tab.lastActionRaw = "" } }
                            }
                            Item {
                                width: parent.width
                                implicitHeight: Math.min(240, logColumn.implicitHeight + 4)
                                ScrollView {
                                    anchors.fill: parent
                                    clip: true
                                    ColumnLayout {
                                        id: logColumn
                                        width: parent.width
                                        spacing: 2
                                        Repeater {
                                            model: tab.actionLog
                                            Text {
                                                text: modelData; color: cText
                                                font.pixelSize: 14; font.family: "monospace"
                                                width: parent.width; elide: Text.ElideRight
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // ====================================================================
    //  COMPONENTS
    // ====================================================================

    component Sect: Rectangle {
        property alias title: ttl.text
        property color bgColor: cPanel
        property color borderColor: cBorder
        property bool  noTitle: false
        default property alias contentChildren: ci.children

        color: bgColor; radius: 8
        border.color: borderColor; border.width: 1
        implicitHeight: inner.implicitHeight + 24
        Layout.preferredHeight: implicitHeight

        ColumnLayout {
            id: inner
            x: 12; y: 12
            width: parent.width - 24
            spacing: 10
            Text {
                id: ttl
                visible: !noTitle && text.length > 0
                color: cMuted; font.pixelSize: 20; font.bold: true
                font.letterSpacing: 0.6
                text: ""
            }
            Item {
                id: ci
                Layout.fillWidth: true
                Layout.fillHeight: false
                implicitHeight: childrenRect.height
            }
        }
    }

    component TbBtn: Rectangle {
        property string lbl: "Btn"
        property string variant: "default"
        signal clicked

        readonly property color baseBorder: variant === "primary" ? cAccent
                                          : variant === "warn"    ? cWarn
                                          : variant === "danger"  ? cBad : cBorder
        readonly property color baseBg:     variant === "primary" ? "#081627"
                                          : variant === "warn"    ? "#2a1c08"
                                          : variant === "danger"  ? "#2c0f0e" : cPanel2
        readonly property color baseFg:     variant === "primary" ? cAccent
                                          : variant === "warn"    ? cWarn
                                          : variant === "danger"  ? cBad : cText

        implicitWidth: Math.max(80, t.implicitWidth + 26)
        implicitHeight: 38
        radius: 6
        color: ma.pressed ? Qt.darker(baseBg, 1.35)
             : ma.containsMouse ? Qt.darker(baseBg, 0.85) : baseBg
        border.color: ma.pressed ? Qt.lighter(baseBorder, 1.15) : baseBorder
        border.width: 1
        Behavior on color { ColorAnimation { duration: 90 } }

        Text {
            id: t; anchors.centerIn: parent
            text: lbl; color: baseFg
            font.pixelSize: 21; font.bold: true
        }
        MotionMouseArea {
            id: ma; anchors.fill: parent
            hoverEnabled: true; cursorShape: Qt.PointingHandCursor
            onClicked: parent.clicked()
        }
    }

    component ModeBtn: TbBtn {
        property bool active: false
        variant: "default"
        color: active ? "#081627" : cPanel2
        border.color: active ? cOk : cBorder
        Text {
            anchors.centerIn: parent; text: parent.lbl
            color: parent.active ? cOk : cText
            font.pixelSize: 21; font.bold: true
        }
    }

    component CycleChip: Rectangle {
        property string value: "00:00"
        property bool   isRun: false
        implicitWidth: Math.max(120, valText.implicitWidth + 24)
        implicitHeight: 38
        radius: 6
        color: isRun ? cOkBg : cIdleBg
        border.color: isRun ? cOk : cBorder; border.width: 1
        Text {
            id: valText
            anchors.centerIn: parent; text: value
            color: parent.isRun ? cOk : cText
            font.pixelSize: 21; font.bold: true; font.family: "monospace"
        }
    }

    component StatusChip: Rectangle {
        property string state: "mid"
        property string label: state
        implicitWidth: txt.implicitWidth + 20; implicitHeight: 28
        radius: 14
        color:        state === "on" ? cOkBg : state === "off" ? cBadBg : state === "mid" ? cWarnBg : cIdleBg
        border.color: state === "on" ? cOk   : state === "off" ? cBad   : state === "mid" ? cWarn   : cIdle
        border.width: 1
        Text {
            id: txt; anchors.centerIn: parent; text: label
            color: parent.state === "on" ? cOk : parent.state === "off" ? cBad : parent.state === "mid" ? cWarn : cIdle
            font.pixelSize: 19; font.bold: true
        }
    }

    component Kv: RowLayout {
        property string lbl: ""
        property string val: ""
        property Item   chip: null
        width: parent.width; spacing: 8
        Text {
            text: lbl; color: cMuted
            font.pixelSize: 20; font.bold: true
            Layout.preferredWidth: 170
        }
        Loader {
            Layout.fillWidth: true
            sourceComponent: chip ? chipWrap : textComp
            Component {
                id: textComp
                Text {
                    text: val; color: cText
                    font.pixelSize: 21
                    wrapMode: Text.WrapAtWordBoundaryOrAnywhere
                }
            }
            Component {
                id: chipWrap
                Item { width: childrenRect.width; height: childrenRect.height; children: chip ? [chip] : [] }
            }
        }
    }

    component Chip: Rectangle {
        property string name: ""
        property string state: "mid"
        property string label: state
        implicitWidth: Math.min(260, Math.max(170, nameTxt.implicitWidth + sChip.implicitWidth + 32))
        implicitHeight: 40
        radius: 6
        color: cPanel2; border.color: cBorder; border.width: 1
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 8; anchors.rightMargin: 8
            spacing: 6
            Text {
                id: nameTxt
                text: name; color: cText
                font.pixelSize: 20
                elide: Text.ElideRight; Layout.fillWidth: true
            }
            StatusChip { id: sChip; state: parent.parent.state; label: parent.parent.label }
        }
    }

    component PCard: Rectangle {
        property string lbl: ""
        property real   val: 0
        property real   maxVal: 1000
        Layout.fillWidth: true
        implicitHeight: pc.implicitHeight + 24
        radius: 8
        color: cPanel2; border.color: cBorder; border.width: 1
        ColumnLayout {
            id: pc; x: 12; y: 12
            width: parent.width - 24; spacing: 6
            RowLayout {
                width: parent.width
                Text { text: lbl; color: cMuted; font.pixelSize: 21; font.bold: true }
                Item { Layout.fillWidth: true }
                RowLayout {
                    spacing: 3
                    Text { text: val.toFixed(1); color: cText; font.pixelSize: 30; font.bold: true; font.family: "monospace" }
                    Text { text: "mbar"; color: cMuted; font.pixelSize: 20; font.bold: true }
                }
            }
            Rectangle {
                Layout.fillWidth: true; height: 8; radius: 4
                color: cBorder
                Rectangle {
                    height: parent.height; radius: parent.radius
                    width: parent.width * Math.max(0, Math.min(1, val / maxVal))
                    color: cAccent
                }
            }
        }
    }

    component CartRow: Rectangle {
        property string cartName: ""
        property real   cartVal: 0
        readonly property string cls: classifyPressure(cartVal, 280, 400, 600)
        Layout.fillWidth: true
        implicitHeight: 50
        radius: 6
        color:        cls === "ok"    ? cOkBg
                    : cls === "high"  ? cWarnBg
                    : cls === "limit" ? cBadBg
                    : Qt.rgba(0.01, 0.51, 0.78, 0.15)
        border.color: cls === "ok"    ? cOk
                    : cls === "high"  ? cWarn
                    : cls === "limit" ? cBad
                    : "#1f86e0"
        border.width: 1
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 10; anchors.rightMargin: 10
            spacing: 8
            Text {
                text: cartName; color: cText
                font.pixelSize: 20; font.bold: true
                Layout.preferredWidth: 95
            }
            ColumnLayout {
                Layout.fillWidth: true; spacing: 2
                RowLayout {
                    width: parent.width
                    Item { Layout.fillWidth: true }
                    Text { text: cartVal.toFixed(0); color: cText; font.pixelSize: 22; font.bold: true; font.family: "monospace" }
                    Text { text: "mbar"; color: cMuted; font.pixelSize: 18; font.bold: true }
                }
                Rectangle {
                    Layout.fillWidth: true; height: 6; radius: 3
                    color: cBorder
                    Rectangle {
                        height: parent.height; radius: parent.radius
                        width: parent.width * Math.max(0, Math.min(1, cartVal / 1000))
                        color: cls === "ok"    ? cOk
                             : cls === "high"  ? cWarn
                             : cls === "limit" ? cBad
                             : "#1f86e0"
                    }
                }
            }
        }
    }

    component PCardBox: Rectangle {
        property string lbl: ""
        property string val: ""
        property bool   active: false
        Layout.preferredHeight: 92
        radius: 8
        color: active ? cAccentSoft : cPanel2
        border.color: active ? cAccent : cBorder
        border.width: active ? 3 : 1
        Behavior on border.width { NumberAnimation { duration: 180 } }
        // Glow effect when active
        Rectangle {
            visible: parent.active
            anchors.fill: parent
            radius: parent.radius
            color: "transparent"
            border.color: cAccent
            border.width: 1
            opacity: 0.5
            anchors.margins: -2
        }
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 10
            spacing: 5
            Text { text: lbl; color: parent.parent.active ? cAccent : cMuted; font.pixelSize: 20; font.bold: parent.parent.active }
            Text { text: val; color: cText; font.pixelSize: 23; font.bold: true; wrapMode: Text.Wrap }
        }
    }

    // Alert row component for Trung tam canh bao
    component AlertRow: Rectangle {
        property string sev: "info"
        property string time: ""
        property string area: ""
        property string message: ""
        Layout.fillWidth: true
        implicitHeight: arCol.implicitHeight + 18
        radius: 6
        color:        sev === "critical" ? Qt.rgba(1.0, 0.15, 0.15, 0.18)
                    : sev === "error"    ? Qt.rgba(1.0, 0.32, 0.32, 0.10)
                    : sev === "warning"  ? Qt.rgba(1.0, 0.65, 0.15, 0.10)
                    : Qt.rgba(0.42, 0.75, 0.95, 0.10)
        border.color: sev === "critical" ? cBad
                    : sev === "error"    ? cBad
                    : sev === "warning"  ? cWarn
                    : cAccent
        border.width: 1

        RowLayout {
            id: arCol
            anchors.fill: parent
            anchors.margins: 10
            spacing: 10

            Text {
                text: sev === "critical" ? "🔥" : sev === "error" ? "⛔" : sev === "warning" ? "⚠️" : "ℹ️"
                font.pixelSize: 22
            }
            ColumnLayout {
                Layout.fillWidth: true; spacing: 2
                RowLayout {
                    width: parent.width
                    Text {
                        text: (area ? area + " · " : "") + sev.toUpperCase()
                        color: parent.parent.parent.parent.parent.sev === "warning" ? cWarn
                             : parent.parent.parent.parent.parent.sev === "info"    ? cAccent : cBad
                        font.pixelSize: 14; font.bold: true
                    }
                    Item { Layout.fillWidth: true }
                    Text {
                        text: time; color: cMuted
                        font.pixelSize: 13; font.family: "monospace"
                    }
                }
                Text {
                    text: message; color: cText
                    font.pixelSize: 15
                    wrapMode: Text.WrapAtWordBoundaryOrAnywhere
                    Layout.fillWidth: true
                }
            }
        }
    }

    // Single valve/cylinder row: label + 2 toggle buttons (a/b) — compact
    component IoToggle: Rectangle {
        property string ioId: ""
        property string statusKey: ""
        property string ioLabel: ""
        property string actA: "on"
        property string actB: "off"
        readonly property var entry: tab.valvesMap[statusKey] || null
        readonly property string curLabel: entry ? entry.label : ""
        readonly property bool aActive: actionMatches(curLabel, actA)
        readonly property bool bActive: actionMatches(curLabel, actB)
        implicitWidth: 360
        implicitHeight: 42
        radius: 5
        color: cPanel2; border.color: cBorder; border.width: 1
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 10; anchors.rightMargin: 8
            spacing: 6
            Text {
                text: ioLabel; color: cText
                font.pixelSize: 20; font.bold: true
                Layout.fillWidth: true; elide: Text.ElideRight
            }
            // Action A button
            Rectangle {
                implicitWidth: 80; implicitHeight: 32; radius: 5
                color: parent.parent.aActive ? cOk : cPanel
                border.color: parent.parent.aActive ? cOk : cBorder; border.width: 1
                Text {
                    anchors.centerIn: parent; text: actA
                    color: parent.parent.parent.aActive ? "#06101d" : cText
                    font.pixelSize: 19; font.bold: true
                }
                MotionMouseArea {
                    anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                    onClicked: hpController.publishManual(ioId, actA)
                }
            }
            Rectangle {
                implicitWidth: 80; implicitHeight: 32; radius: 5
                color: parent.parent.bActive ? cOk : cPanel
                border.color: parent.parent.bActive ? cOk : cBorder; border.width: 1
                Text {
                    anchors.centerIn: parent; text: actB
                    color: parent.parent.parent.bActive ? "#06101d" : cText
                    font.pixelSize: 19; font.bold: true
                }
                MotionMouseArea {
                    anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                    onClicked: hpController.publishManual(ioId, actB)
                }
            }
        }
    }

    // Press-and-hold jog button: publishes "rev"/"fwd" on press, "stop" on release
    component JogBtn: Rectangle {
        property string lbl: "JOG"
        property string dir: "fwd"  // "fwd" | "rev"
        property string variant: "default"
        property bool _pressed: false
        readonly property color baseBorder: variant === "primary" ? cAccent
                                          : variant === "warn"    ? cWarn
                                          : variant === "danger"  ? cBad : cBorder
        readonly property color baseFg:     variant === "primary" ? cAccent
                                          : variant === "warn"    ? cWarn
                                          : variant === "danger"  ? cBad : cText
        implicitWidth: 140; implicitHeight: 46
        radius: 6
        color: _pressed ? Qt.rgba(0.25, 0.65, 0.85, 0.28) : cPanel2
        border.color: baseBorder; border.width: 1
        Text {
            anchors.centerIn: parent; text: lbl; color: baseFg
            font.pixelSize: 21; font.bold: true
        }
        MotionMouseArea {
            anchors.fill: parent
            cursorShape: Qt.PointingHandCursor
            onPressed:   { parent._pressed = true;  hpController.publishString("servo_jog", dir) }
            onReleased:  { parent._pressed = false; hpController.publishString("servo_jog", "stop") }
            onCanceled:  { parent._pressed = false; hpController.publishString("servo_jog", "stop") }
        }
    }

    // Numeric input rectangle (exposes valueText for parent to read)
    component PwmInput: Rectangle {
        property string valueText: "0"
        implicitWidth: 90; implicitHeight: 34
        radius: 4
        color: cField; border.color: cBorder; border.width: 1
        TextInput {
            anchors.fill: parent; anchors.margins: 6
            text: valueText
            onTextChanged: valueText = text
            color: cText; font.pixelSize: 21; font.bold: true; font.family: "monospace"
            selectByMouse: true; horizontalAlignment: TextInput.AlignHCenter
        }
    }

    // Setting row (label + numeric input + unit + Set button)
    component SettingRow: Rectangle {
        property var item: ({})
        property string currentVal: ""
        height: 46
        radius: 6
        color: cPanel2; border.color: cBorder; border.width: 1
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 10; anchors.rightMargin: 10
            spacing: 6
            Text {
                text: item.label || ""; color: cText
                font.pixelSize: 20; font.bold: true
                Layout.fillWidth: true; elide: Text.ElideRight
            }
            Rectangle {
                implicitWidth: 95; implicitHeight: 32; radius: 4
                color: cField; border.color: cBorder; border.width: 1
                TextInput {
                    id: inp
                    anchors.fill: parent; anchors.margins: 6
                    text: currentVal
                    color: cText; font.pixelSize: 20; font.family: "monospace"
                    selectByMouse: true; horizontalAlignment: TextInput.AlignRight
                }
            }
            Text { text: item.unit || ""; color: cMuted; font.pixelSize: 19; Layout.preferredWidth: 38 }
            Rectangle {
                implicitWidth: 60; implicitHeight: 32; radius: 4
                color: "#081627"; border.color: cAccent; border.width: 1
                Text {
                    anchors.centerIn: parent; text: "Set"; color: cAccent
                    font.pixelSize: 19; font.bold: true
                }
                MotionMouseArea {
                    anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                    onClicked: tab.publishSetting(item, inp.text)
                }
            }
        }
    }
}
