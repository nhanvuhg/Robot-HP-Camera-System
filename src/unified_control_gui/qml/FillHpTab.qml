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

    // ---- Theme tokens (mirror CartridgePage palette) ----
    readonly property color cBg:          "#0c0c1d"
    readonly property color cPanel:       "#081e29"
    readonly property color cPanel2:      "#051a1a"
    readonly property color cBorder:      "#134357"
    readonly property color cText:        "#e8e8f0"
    readonly property color cMuted:       "#8888aa"
    readonly property color cAccent:      "#4f6cff"
    readonly property color cAccentSoft:  Qt.rgba(0.31, 0.42, 1.0, 0.18)
    readonly property color cOk:          "#00e676"
    readonly property color cOkBg:        Qt.rgba(0.0, 0.90, 0.46, 0.15)
    readonly property color cWarn:        "#ffa726"
    readonly property color cWarnBg:      Qt.rgba(1.0, 0.65, 0.15, 0.15)
    readonly property color cBad:         "#ff5252"
    readonly property color cBadBg:       Qt.rgba(1.0, 0.32, 0.32, 0.15)
    readonly property color cIdle:        "#526070"
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
            if (pos >= 0) out[part.substring(0, pos).trim()] = part.substring(pos + 1).trim();
        }
        return out;
    }
    function classifyState(rawVal) {
        var v = String(rawVal || "").toLowerCase();
        if (v === "on" || v === "true" || v === "ok" || v === "ready" || v === "connected") return "on";
        if (v === "off" || v === "false" || v === "error" || v === "fault" || v === "disconnected") return "off";
        return "mid";
    }
    function classifyPressure(val, lowT, highT, limitT) {
        if (val < lowT) return "low";
        if (val >= limitT) return "limit";
        if (val >= highT) return "high";
        return "ok";
    }

    // ---- Derived data from hpController ----
    property var sysMap:    parseKvPipe(hpController.systemStatus)
    property var hwMap:     parseKvPipe(hpController.hwStatus)
    property var inputsMap: parseKvComma(hpController.inputState)
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
        color: "#141428"

        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 14
            anchors.rightMargin: 14
            spacing: 8

            Text {
                text: "Fill HP Control"
                color: cText; font.pixelSize: 18; font.bold: true; font.family: monoFamily
            }

            Item { Layout.fillWidth: true }

            TbBtn {
                lbl: "Start"; variant: "primary"
                onClicked: hpController.publishScreenControl("start")
            }

            CycleChip {
                value: hpController.ballCycleTime || "00:00"
                isRun: tab.running
            }

            TbBtn {
                lbl: "Stop"; variant: "danger"
                onClicked: hpController.publishScreenControl("stop")
            }

            Item { width: 8 }

            ModeBtn { lbl: "Manual"; active: tab.modeStr === "MANUAL"; onClicked: hpController.publishMode(2) }
            ModeBtn { lbl: "Auto";   active: tab.modeStr === "AUTO";   onClicked: hpController.publishMode(0) }
            ModeBtn { lbl: "Clean";  active: tab.modeStr === "CLEAN";  onClicked: hpController.publishMode(1) }

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

                    Text { text: "⛔"; font.pixelSize: 22 }
                    ColumnLayout {
                        Layout.fillWidth: true; spacing: 2
                        Text { text: "CANH BAO HE THONG"; color: cBad; font.bold: true; font.pixelSize: 14 }
                        Text {
                            text: hpController.errorStatus || "-"
                            color: cText; font.pixelSize: 13; font.family: monoFamily
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
                    Layout.alignment: Qt.AlignTop
                    spacing: 12

                    // -- Pressure cards (full row) --
                    Sect {
                        title: "Analog / Ap suat"
                        Layout.fillWidth: true
                        ColumnLayout {
                            width: parent.width; spacing: 8
                            PCard { lbl: "S1 Chamber";   val: hpController.pressureS1; maxVal: 1200 }
                            PCard { lbl: "S2 Cartridge"; val: hpController.pressureS2; maxVal: 1200 }
                            PCard { lbl: "S3 Tank";      val: hpController.pressureS3; maxVal: 1000 }
                        }
                    }

                    // -- Cartridge pressures --
                    Sect {
                        title: "Cartridge pressure"
                        Layout.fillWidth: true
                        visible: hpController.cartridgePressures && hpController.cartridgePressures.length > 0
                        ColumnLayout {
                            width: parent.width; spacing: 5
                            Repeater {
                                model: hpController.cartridgePressures
                                CartRow {
                                    cartName: "Cart " + (index + 1)
                                    cartVal:  Number(modelData) || 0
                                }
                            }
                        }
                    }

                    // -- Process cards --
                    Sect {
                        title: "Qua trinh"
                        Layout.fillWidth: true
                        GridLayout {
                            columns: 4
                            rowSpacing: 8; columnSpacing: 8
                            width: parent.width
                            PCardBox { Layout.fillWidth: true; lbl: "Auto fill";    val: hpController.fillStatus    || "-"; active: tab.modeStr === "AUTO" && tab.running }
                            PCardBox { Layout.fillWidth: true; lbl: "Dosing";       val: hpController.dosingStatus  || "-" }
                            PCardBox { Layout.fillWidth: true; lbl: "Clean refill"; val: hpController.crStatus      || "-"; active: tab.modeStr === "CLEAN" && tab.running }
                            PCardBox { Layout.fillWidth: true; lbl: "Cycle / Vol";  val: tab.cycleStr + "  ·  " + tab.volumeStr }
                        }
                    }

                    // -- Inputs (read-only chips) --
                    Sect {
                        title: "Ngo vao Input"
                        Layout.fillWidth: true
                        visible: Object.keys(tab.inputsMap).length > 0
                        Flow {
                            spacing: 6; width: parent.width
                            Repeater {
                                model: Object.keys(tab.inputsMap)
                                Chip {
                                    name:  modelData
                                    state: classifyState(tab.inputsMap[modelData])
                                    label: String(tab.inputsMap[modelData])
                                }
                            }
                        }
                    }

                    // -- Note ve cac phan da bo --
                    Sect {
                        title: "Ghi chu"
                        Layout.fillWidth: true
                        Text {
                            text: "Cac control rui ro cao (valve/cylinder manual, settings editor, servo jog) duoc giu trong file PAGE 4 cu — neu can mo lai, dung lai phien ban Fill HP cu."
                            color: cMuted; font.pixelSize: 12; font.family: monoFamily
                            wrapMode: Text.WrapAtWordBoundaryOrAnywhere
                            width: parent.width
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
                color: cMuted; font.pixelSize: 12; font.bold: true
                font.letterSpacing: 0.6; font.family: monoFamily
                text: ""
            }
            Item {
                id: ci
                Layout.fillWidth: true
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
        readonly property color baseBg:     variant === "primary" ? "#0a1a3a"
                                          : variant === "warn"    ? "#3a2a08"
                                          : variant === "danger"  ? "#3a1010" : cPanel2
        readonly property color baseFg:     variant === "primary" ? cAccent
                                          : variant === "warn"    ? cWarn
                                          : variant === "danger"  ? cBad : cText

        implicitWidth: Math.max(70, t.implicitWidth + 22)
        implicitHeight: 32
        radius: 6
        color: ma.containsMouse ? Qt.darker(baseBg, 0.85) : baseBg
        border.color: baseBorder; border.width: 1

        Text {
            id: t; anchors.centerIn: parent
            text: lbl; color: baseFg
            font.pixelSize: 13; font.bold: true; font.family: monoFamily
        }
        MouseArea {
            id: ma; anchors.fill: parent
            hoverEnabled: true; cursorShape: Qt.PointingHandCursor
            onClicked: parent.clicked()
        }
    }

    component ModeBtn: TbBtn {
        property bool active: false
        variant: "default"
        color: active ? "#0a2e22" : cPanel2
        border.color: active ? cOk : cBorder
        Text {
            anchors.centerIn: parent; text: parent.lbl
            color: parent.active ? cOk : cText
            font.pixelSize: 13; font.bold: true; font.family: monoFamily
        }
    }

    component CycleChip: Rectangle {
        property string value: "00:00"
        property bool   isRun: false
        implicitWidth: 90; implicitHeight: 32
        radius: 6
        color: isRun ? cOkBg : cIdleBg
        border.color: isRun ? cOk : cBorder; border.width: 1
        Text {
            anchors.centerIn: parent; text: value
            color: parent.isRun ? cOk : cText
            font.pixelSize: 13; font.bold: true; font.family: monoFamily
        }
    }

    component StatusChip: Rectangle {
        property string state: "mid"
        property string label: state
        implicitWidth: txt.implicitWidth + 16; implicitHeight: 22
        radius: 11
        color:        state === "on" ? cOkBg : state === "off" ? cBadBg : state === "mid" ? cWarnBg : cIdleBg
        border.color: state === "on" ? cOk   : state === "off" ? cBad   : state === "mid" ? cWarn   : cIdle
        border.width: 1
        Text {
            id: txt; anchors.centerIn: parent; text: label
            color: parent.state === "on" ? cOk : parent.state === "off" ? cBad : parent.state === "mid" ? cWarn : cIdle
            font.pixelSize: 11; font.bold: true; font.family: monoFamily
        }
    }

    component Kv: RowLayout {
        property string lbl: ""
        property string val: ""
        property Item   chip: null
        width: parent.width; spacing: 8
        Text {
            text: lbl; color: cMuted
            font.pixelSize: 12; font.bold: true; font.family: monoFamily
            Layout.preferredWidth: 130
        }
        Loader {
            Layout.fillWidth: true
            sourceComponent: chip ? chipWrap : textComp
            Component {
                id: textComp
                Text {
                    text: val; color: cText
                    font.pixelSize: 13; font.family: monoFamily
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
        implicitWidth: Math.min(220, Math.max(140, nameTxt.implicitWidth + sChip.implicitWidth + 28))
        implicitHeight: 34
        radius: 6
        color: cPanel2; border.color: cBorder; border.width: 1
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 8; anchors.rightMargin: 8
            spacing: 6
            Text {
                id: nameTxt
                text: name; color: cText
                font.pixelSize: 12; font.family: monoFamily
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
                Text { text: lbl; color: cMuted; font.pixelSize: 13; font.bold: true; font.family: monoFamily }
                Item { Layout.fillWidth: true }
                RowLayout {
                    spacing: 3
                    Text { text: val.toFixed(1); color: cText; font.pixelSize: 22; font.bold: true; font.family: monoFamily }
                    Text { text: "mbar"; color: cMuted; font.pixelSize: 12; font.bold: true }
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
        implicitHeight: 42
        radius: 6
        color:        cls === "ok"    ? cOkBg
                    : cls === "high"  ? cWarnBg
                    : cls === "limit" ? cBadBg
                    : Qt.rgba(0.01, 0.51, 0.78, 0.15)
        border.color: cls === "ok"    ? cOk
                    : cls === "high"  ? cWarn
                    : cls === "limit" ? cBad
                    : "#0284c7"
        border.width: 1
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 10; anchors.rightMargin: 10
            spacing: 8
            Text {
                text: cartName; color: cText
                font.pixelSize: 12; font.bold: true; font.family: monoFamily
                Layout.preferredWidth: 70
            }
            ColumnLayout {
                Layout.fillWidth: true; spacing: 2
                RowLayout {
                    width: parent.width
                    Item { Layout.fillWidth: true }
                    Text { text: cartVal.toFixed(0); color: cText; font.pixelSize: 14; font.bold: true; font.family: monoFamily }
                    Text { text: "mbar"; color: cMuted; font.pixelSize: 10; font.bold: true }
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
                             : "#0284c7"
                    }
                }
            }
        }
    }

    component PCardBox: Rectangle {
        property string lbl: ""
        property string val: ""
        property bool   active: false
        Layout.preferredHeight: 76
        radius: 8
        color: active ? cAccentSoft : cPanel2
        border.color: active ? cAccent : cBorder
        border.width: 1
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 10
            spacing: 5
            Text { text: lbl; color: cMuted; font.pixelSize: 12; font.family: monoFamily }
            Text { text: val; color: cText; font.pixelSize: 15; font.bold: true; font.family: monoFamily; wrapMode: Text.Wrap }
        }
    }
}
