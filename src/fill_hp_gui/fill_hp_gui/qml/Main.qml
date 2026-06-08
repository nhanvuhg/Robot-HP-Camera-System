import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {
    id: root
    visible: true
    width: 1600
    height: 950
    title: "Fill HP Control — Mockup"
    color: cBg

    // ---- Theme tokens (mirror fill_hp_web1.py :root) ----
    readonly property color cBg:          "#0c0c1d"
    readonly property color cPanel:       "#081e29"
    readonly property color cPanel2:      "#051a1a"
    readonly property color cText:        "#e8e8f0"
    readonly property color cMuted:       "#8888aa"
    readonly property color cLine:        "#134357"
    readonly property color cPrimary:     "#4f6cff"
    readonly property color cPrimarySoft: Qt.rgba(0.31, 0.42, 1.0, 0.15)
    readonly property color cOk:          "#00e676"
    readonly property color cOkBg:        Qt.rgba(0.0, 0.90, 0.46, 0.15)
    readonly property color cWarn:        "#ffa726"
    readonly property color cWarnBg:      Qt.rgba(1.0, 0.65, 0.15, 0.15)
    readonly property color cBad:         "#ff5252"
    readonly property color cBadBg:       Qt.rgba(1.0, 0.32, 0.32, 0.15)
    readonly property color cIdle:        "#526070"
    readonly property color cIdleBg:      Qt.rgba(0.32, 0.38, 0.44, 0.15)

    readonly property string monoFamily:  "JetBrains Mono, DejaVu Sans Mono, Consolas, monospace"

    // ---- State: bind sang `bridge` neu duoc set boi launcher Python,
    //      fallback dummy data khi run standalone (qmlscene / preview.py) ----
    readonly property bool hasBridge: typeof bridge !== "undefined" && bridge !== null
    property string mode:        hasBridge ? bridge.mode       : "MANUAL"
    property string sysState:    hasBridge ? bridge.sysState   : "IDLE"
    property int    cycleCount:  17
    property string cycleClock:  hasBridge ? bridge.cycleClock : "00:42"
    property bool   running:     hasBridge ? bridge.running    : false

    // Helpers de QML goi bridge an toan
    function pubMode(m)         { if (hasBridge) bridge.setMode(m) }
    function pubScreen(action)  { if (hasBridge) bridge.screenControl(action) }
    function pubManual(n, a)    { if (hasBridge) bridge.manualCommand(n, a) }
    function pubReconnect(t)    { if (hasBridge) bridge.reconnect(t) }

    // ====================================================================
    //  HEADER
    // ====================================================================
    header: Rectangle {
        height: 64
        color: "#141428"
        border.color: cLine
        border.width: 0

        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 16
            anchors.rightMargin: 16
            spacing: 10

            Text {
                text: "Fill HP Control"
                color: cText
                font.pixelSize: 20
                font.bold: true
                font.family: monoFamily
            }

            Item { Layout.fillWidth: true }

            ToolbarButton { label: "Start";  variant: "primary"; onClicked: pubScreen("start") }
            CycleTimerChip { value: cycleClock; running: root.running }
            CycleCountChip { count: cycleCount }
            ToolbarButton { label: "Stop";   variant: "danger";  onClicked: pubScreen("stop") }

            Item { width: 8 }
            ModeButton { label: "Manual"; active: mode === "MANUAL"; onClicked: { if (!hasBridge) mode = "MANUAL"; pubMode(2) } }
            ModeButton { label: "Auto";   active: mode === "AUTO";   onClicked: { if (!hasBridge) mode = "AUTO";   pubMode(0) } }
            ModeButton { label: "Clean";  active: mode === "CLEAN";  onClicked: { if (!hasBridge) mode = "CLEAN";  pubMode(1) } }

            Item { width: 8 }
            ToolbarButton { label: "Reconnect CPX"; variant: "warn"; onClicked: pubReconnect("cpx") }

            BellButton { unread: 3 }

            Text { text: "admin";  color: cText; font.pixelSize: 13; font.family: monoFamily }
            ToolbarButton { label: "Dang xuat"; variant: "warn" }
        }

        Rectangle { anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: parent.bottom; height: 1; color: cLine }
    }

    // ====================================================================
    //  MAIN BODY (sidebar + content)
    // ====================================================================
    Flickable {
        anchors.fill: parent
        contentWidth: width
        contentHeight: mainCol.implicitHeight + 24
        clip: true
        ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

        ColumnLayout {
            id: mainCol
            x: 12; y: 12
            width: parent.width - 24
            spacing: 12

            // ---- Error banner (sample) ----
            Section {
                Layout.fillWidth: true
                visible: true
                bgColor: Qt.rgba(1.0, 0.32, 0.32, 0.10)
                borderColor: cBad
                noTitle: true

                RowLayout {
                    spacing: 12
                    width: parent.width

                    Text { text: "⛔"; font.pixelSize: 24 }
                    ColumnLayout {
                        Layout.fillWidth: true
                        spacing: 2
                        RowLayout {
                            spacing: 8
                            Text { text: "CANH BAO HE THONG"; color: cBad; font.bold: true; font.pixelSize: 14 }
                            Rectangle {
                                width: badge.implicitWidth + 12; height: 20
                                color: Qt.rgba(1.0, 0.32, 0.32, 0.2); radius: 10
                                border.color: cBad; border.width: 1
                                Text { id: badge; anchors.centerIn: parent; text: "ERROR"; color: cBad; font.pixelSize: 11; font.bold: true }
                            }
                        }
                        Text { text: "Chamber pressure timeout sau 8.2s"; color: cText; font.pixelSize: 13; font.family: monoFamily }
                        Text { text: "13:42:15 · S1 = 102 mbar · target 350 mbar"; color: cMuted; font.pixelSize: 11; font.family: monoFamily }
                    }
                    ToolbarButton { label: "Chi tiet" }
                    ToolbarButton { label: "Xac nhan & xoa"; variant: "danger" }
                }
            }

            // ---- Body row: sidebar (340) + content ----
            RowLayout {
                Layout.fillWidth: true
                spacing: 12

                // ====== SIDEBAR ======
                ColumnLayout {
                    Layout.preferredWidth: 340
                    Layout.alignment: Qt.AlignTop
                    spacing: 12

                    Section {
                        title: "Tong quan"
                        Layout.fillWidth: true
                        ColumnLayout {
                            spacing: 7; width: parent.width
                            KvRow { label: "Ket noi web"; valueChip: StatusChip { state: "off"; label: "offline" } }
                            KvRow { label: "Mode";        valueText: root.mode }
                            KvRow { label: "State";       valueText: root.sysState }
                            KvRow { label: "Cycle";       valueText: "3 / 5" }
                            KvRow { label: "Volume";      valueText: "42.5 ml" }
                            KvRow { label: "Running";     valueChip: StatusChip { state: root.running ? "on" : "off"; label: root.running ? "In Process" : "Stop" } }
                            KvRow {
                                label: "Phan hoi cuoi"
                                valueText: "valve1:open OK\nat 13:42:08"
                            }
                        }
                    }

                    Section {
                        title: "Hardware"
                        Layout.fillWidth: true
                        Flow {
                            spacing: 7
                            width: parent.width
                            Repeater {
                                model: [
                                    {name: "CPX",        state: "on"},
                                    {name: "Modbus",     state: "on"},
                                    {name: "Servo",      state: "on"},
                                    {name: "Pressure S1",state: "on"},
                                    {name: "Pressure S2",state: "mid"},
                                    {name: "Tank S3",    state: "on"},
                                ]
                                ChipItem {
                                    name:  modelData.name
                                    state: modelData.state
                                }
                            }
                        }
                    }

                    Section {
                        title: "Trang thai may"
                        Layout.fillWidth: true
                        ColumnLayout {
                            spacing: 7; width: parent.width
                            KvRow { label: "Dosing";      valueText: "IDLE" }
                            KvRow { label: "Fill";        valueText: "READY" }
                            KvRow { label: "Fix";         valueText: "RELEASED" }
                            KvRow { label: "Clean refill"; valueText: "IDLE" }
                            KvRow { label: "Servo raw";   valueText: "12 845" }
                            KvRow { label: "Servo mm";    valueText: "85.32 mm" }
                            KvRow { label: "Base PWM";    valueText: "62 %" }
                        }
                    }
                }

                // ====== CONTENT GRID ======
                ColumnLayout {
                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignTop
                    spacing: 12

                    // -- Alert center (wide) --
                    Section {
                        title: "Trung tam canh bao"
                        Layout.fillWidth: true

                        ColumnLayout {
                            width: parent.width
                            spacing: 8

                            RowLayout {
                                spacing: 6
                                Repeater {
                                    model: ["Tat ca", "Dang hoat dong", "Nghiem trong", "Loi", "Canh bao", "Thong tin"]
                                    ToolbarButton {
                                        label: modelData
                                        variant: index === 0 ? "primary" : "default"
                                    }
                                }
                                Item { Layout.fillWidth: true }
                                ToolbarButton { label: "Danh dau da doc" }
                                ToolbarButton { label: "Xoa lich su" }
                            }

                            Repeater {
                                model: [
                                    {sev: "error",   time: "13:42:15", title: "Chamber pressure timeout",  text: "S1 102 mbar < target 350 mbar sau 8.2s"},
                                    {sev: "warning", time: "13:38:02", title: "Vent PWM dat gioi han",     text: "chamber_vent_pwm clamp at 100%"},
                                    {sev: "info",    time: "13:35:41", title: "Auto mode da chuyen sang Idle", text: "User press Stop"},
                                ]
                                AlertRow {
                                    sev:   modelData.sev
                                    time:  modelData.time
                                    title: modelData.title
                                    text:  modelData.text
                                }
                            }
                        }
                    }

                    // -- Qua trinh (wide, 4 cards) --
                    Section {
                        title: "Qua trinh"
                        Layout.fillWidth: true
                        GridLayout {
                            columns: 4
                            rowSpacing: 8
                            columnSpacing: 8
                            width: parent.width
                            ProcessCard { Layout.fillWidth: true; label: "Auto fill";    valueText: "READY";     active: true }
                            ProcessCard { Layout.fillWidth: true; label: "Dosing";       valueText: "IDLE" }
                            ProcessCard { Layout.fillWidth: true; label: "Clean refill"; valueText: "IDLE" }
                            ProcessCard { Layout.fillWidth: true; label: "Cycle / Volume"; valueText: "3 / 5  · 42.5 ml" }
                        }
                    }

                    // -- Auto-fit grid for the rest --
                    Grid {
                        id: contentGrid
                        Layout.fillWidth: true
                        columns: Math.max(1, Math.floor(width / 360))
                        spacing: 12

                        // Analog/Ap suat
                        Section {
                            title: "Analog / Ap suat"
                            width: (contentGrid.width - contentGrid.spacing * (contentGrid.columns - 1)) / contentGrid.columns
                            ColumnLayout {
                                spacing: 8; width: parent.width
                                PressureCard {
                                    label: "S1 Chamber"; unit: "mbar"
                                    valueText: hasBridge ? bridge.pressureS1.toFixed(0) : "342"
                                    ratio: hasBridge ? bridge.pressureS1 / 1200 : 0.58
                                }
                                PressureCard {
                                    label: "S2 Cartridge"; unit: "mbar"
                                    valueText: hasBridge ? bridge.pressureS2.toFixed(0) : "118"
                                    ratio: hasBridge ? bridge.pressureS2 / 1200 : 0.20
                                }
                                PressureCard {
                                    label: "S3 Tank"; unit: "mbar"
                                    valueText: hasBridge ? bridge.pressureS3.toFixed(0) : "812"
                                    ratio: hasBridge ? bridge.pressureS3 / 1000 : 0.81
                                }

                                Item { height: 6 }
                                RowLayout {
                                    width: parent.width
                                    Text { text: "CARTRIDGE PRESSURE"; color: cMuted; font.pixelSize: 11; font.bold: true; font.letterSpacing: 0.5 }
                                    Item { Layout.fillWidth: true }
                                    Text { text: "ok: 6  · high: 1  · limit: 0"; color: cMuted; font.pixelSize: 11 }
                                }
                                Repeater {
                                    model: hasBridge && bridge.cartridgePressures.length > 0
                                            ? bridge.cartridgePressures
                                            : [318, 322, 410, 308, 312, 296]
                                    CartridgeRow {
                                        name:  "Cart " + (index + 1)
                                        val:   Math.round(modelData)
                                        ratio: modelData / 1000
                                        cls:   modelData < 300 ? "low"
                                             : modelData > 400 ? (modelData > 600 ? "limit" : "high")
                                             : "ok"
                                    }
                                }
                            }
                        }

                        // Inputs
                        Section {
                            title: "Ngo vao Input"
                            width: (contentGrid.width - contentGrid.spacing * (contentGrid.columns - 1)) / contentGrid.columns
                            Flow {
                                spacing: 6; width: parent.width
                                Repeater {
                                    model: [
                                        {name: "Start button",  state: "off"},
                                        {name: "Stop button",   state: "off"},
                                        {name: "Optical sensor",state: "on"},
                                        {name: "Ball feed up",  state: "on"},
                                        {name: "Ball feed down",state: "off"},
                                        {name: "Ball push up",  state: "on"},
                                        {name: "Ball push down",state: "off"},
                                        {name: "Chamber open",  state: "off"},
                                        {name: "Chamber closed",state: "on"},
                                        {name: "Seal pin up",   state: "on"},
                                        {name: "Seal pin down", state: "off"},
                                        {name: "Fix cyl up",    state: "on"},
                                        {name: "Fix cyl down",  state: "off"},
                                        {name: "Ball box empty",state: "off"},
                                    ]
                                    ChipItem {
                                        name:  modelData.name
                                        state: modelData.state
                                    }
                                }
                            }
                        }

                        // Valve
                        Section {
                            title: "Valve"
                            width: (contentGrid.width - contentGrid.spacing * (contentGrid.columns - 1)) / contentGrid.columns
                            ColumnLayout {
                                spacing: 0; width: parent.width
                                Repeater {
                                    model: [
                                        {name: "Valve 1",          state: "on",  on: "open",   off: "close"},
                                        {name: "Valve 2",          state: "mid", on: "air",    off: "ink"},
                                        {name: "Valve 4",          state: "off", on: "on",     off: "off"},
                                        {name: "Valve 5",          state: "off", on: "open",   off: "close"},
                                        {name: "Valve 6",          state: "off", on: "open",   off: "close"},
                                        {name: "Valve 7",          state: "off", on: "on",     off: "off"},
                                        {name: "Valve 8",          state: "off", on: "on",     off: "off"},
                                        {name: "Valve 9",          state: "mid", on: "chamber",off: "waste"},
                                        {name: "Valve chamber O_5",state: "off", on: "on",     off: "off"},
                                        {name: "Vacuum pump O_6",  state: "on",  on: "on",     off: "off"},
                                    ]
                                    IoControlRow {
                                        ioName:  modelData.name
                                        ioState: modelData.state
                                        onLabel: modelData.on
                                        offLabel: modelData.off
                                    }
                                }
                            }
                        }

                        // Cylinder
                        Section {
                            title: "Cylinder"
                            width: (contentGrid.width - contentGrid.spacing * (contentGrid.columns - 1)) / contentGrid.columns
                            ColumnLayout {
                                spacing: 0; width: parent.width
                                Repeater {
                                    model: [
                                        {name: "Chamber cylinder",  state: "off", on: "close", off: "open"},
                                        {name: "Cartridge cylinder",state: "on",  on: "down",  off: "up"},
                                        {name: "Ball feed cylinder",state: "on",  on: "up",    off: "down"},
                                        {name: "Ball push cylinder",state: "off", on: "up",    off: "down"},
                                        {name: "Seal pin cylinder", state: "on",  on: "up",    off: "down"},
                                        {name: "Fix cylinder",      state: "off", on: "up",    off: "down"},
                                    ]
                                    IoControlRow {
                                        ioName:  modelData.name
                                        ioState: modelData.state
                                        onLabel: modelData.on
                                        offLabel: modelData.off
                                    }
                                }
                            }
                        }

                        // Motor / PWM / Servo
                        Section {
                            title: "Motor / PWM / Servo"
                            width: (contentGrid.width - contentGrid.spacing * (contentGrid.columns - 1)) / contentGrid.columns
                            ColumnLayout {
                                spacing: 10; width: parent.width

                                RowLayout {
                                    width: parent.width
                                    Text { text: "Main PWM";  color: cText; font.pixelSize: 13; Layout.fillWidth: true }
                                    StatusChip { state: "on"; label: "62%" }
                                }
                                RowLayout {
                                    width: parent.width
                                    Text { text: "Tank PWM";  color: cText; font.pixelSize: 13; Layout.fillWidth: true }
                                    StatusChip { state: "mid"; label: "idle" }
                                }
                                RowLayout {
                                    width: parent.width
                                    Text { text: "Servo";     color: cText; font.pixelSize: 13; Layout.fillWidth: true }
                                    StatusChip { state: "on"; label: "85.32 mm" }
                                }

                                Rectangle { Layout.fillWidth: true; height: 1; color: cLine }

                                RowLayout {
                                    width: parent.width; spacing: 6
                                    Text { text: "Servo jog"; color: cMuted; font.pixelSize: 12; Layout.fillWidth: true }
                                    ToolbarButton { label: "−"; }
                                    ToolbarButton { label: "+"; }
                                    ToolbarButton { label: "HOME"; variant: "primary" }
                                }
                            }
                        }

                        // Settings (tabs)
                        Section {
                            title: "Thong so dieu khien"
                            width: (contentGrid.width - contentGrid.spacing * (contentGrid.columns - 1)) / contentGrid.columns
                            ColumnLayout {
                                spacing: 8; width: parent.width
                                RowLayout {
                                    spacing: 6
                                    Repeater {
                                        model: ["Auto", "Clean", "Dosing", "PWM / Tank"]
                                        ToolbarButton {
                                            label: modelData
                                            variant: index === 0 ? "primary" : "default"
                                        }
                                    }
                                }
                                Repeater {
                                    model: [
                                        {name: "Cart fix pressure",   val: "850",  unit: "mbar"},
                                        {name: "Chamber vacuum",      val: "120",  unit: "mbar"},
                                        {name: "Chamber leak",        val: "60",   unit: "mbar"},
                                        {name: "Cartridge vacuum",    val: "180",  unit: "mbar"},
                                        {name: "Cartridge leak",      val: "80",   unit: "mbar"},
                                        {name: "Pressure balance",    val: "250",  unit: "mbar"},
                                        {name: "Chamber vent",        val: "200",  unit: "mbar"},
                                        {name: "Fill compensation",   val: "0.4",  unit: "ml"},
                                    ]
                                    SettingRow {
                                        settingName: modelData.name
                                        settingVal:  modelData.val
                                        settingUnit: modelData.unit
                                    }
                                }
                            }
                        }

                        // Action log
                        Section {
                            title: "Action log"
                            width: (contentGrid.width - contentGrid.spacing * (contentGrid.columns - 1)) / contentGrid.columns
                            ColumnLayout {
                                spacing: 4; width: parent.width
                                Repeater {
                                    model: [
                                        "13:42:08 · valve1 → open",
                                        "13:42:01 · cylinder chamber → close",
                                        "13:41:55 · base_pwm = 62",
                                        "13:41:40 · mode → MANUAL",
                                        "13:41:21 · reconnect_cmd → cpx",
                                    ]
                                    Text {
                                        text: modelData; color: cText
                                        font.pixelSize: 12; font.family: monoFamily
                                        width: parent.width; elide: Text.ElideRight
                                    }
                                }
                            }
                        }

                        // Raw status
                        Section {
                            title: "Raw status"
                            width: (contentGrid.width - contentGrid.spacing * (contentGrid.columns - 1)) / contentGrid.columns
                            Text {
                                text: "system_status: IDLE | mode: MANUAL | cycle: 3/5\nvolume: 42.5 | running: False"
                                color: cMuted
                                font.pixelSize: 11
                                font.family: monoFamily
                                width: parent.width
                                wrapMode: Text.WrapAtWordBoundaryOrAnywhere
                            }
                        }
                    }
                }
            }
        }
    }

    // ====================================================================
    //  REUSABLE COMPONENTS
    // ====================================================================

    component Section: Rectangle {
        property alias title:    titleText.text
        property color bgColor:  cPanel
        property color borderColor: cLine
        property bool  noTitle:  false
        default property alias contentChildren: contentItem.children

        color: bgColor
        radius: 8
        border.color: borderColor
        border.width: 1
        implicitHeight: inner.implicitHeight + 24
        Layout.preferredHeight: implicitHeight

        ColumnLayout {
            id: inner
            x: 12; y: 12
            width: parent.width - 24
            spacing: 10

            Text {
                id: titleText
                visible: !noTitle && text.length > 0
                color: cMuted
                font.pixelSize: 12
                font.bold: true
                font.letterSpacing: 0.7
                font.family: monoFamily
                text: ""
            }

            Item {
                id: contentItem
                Layout.fillWidth: true
                implicitHeight: childrenRect.height
            }
        }
    }

    component ToolbarButton: Rectangle {
        property string label: "Btn"
        property string variant: "default"     // default | primary | warn | danger
        signal clicked

        readonly property color baseBorder: variant === "primary" ? cPrimary
                                          : variant === "warn"    ? cWarn
                                          : variant === "danger"  ? cBad : cLine
        readonly property color baseBg:     variant === "primary" ? "#0a1a3a"
                                          : variant === "warn"    ? "#3a2a08"
                                          : variant === "danger"  ? "#3a1010" : cPanel2
        readonly property color baseFg:     variant === "primary" ? cPrimary
                                          : variant === "warn"    ? cWarn
                                          : variant === "danger"  ? cBad : cText

        implicitWidth: Math.max(70, txt.implicitWidth + 22)
        implicitHeight: 30
        radius: 6
        color: hovered ? Qt.darker(baseBg, 0.85) : baseBg
        border.color: baseBorder
        border.width: 1
        property bool hovered: ma.containsMouse

        Text {
            id: txt; anchors.centerIn: parent
            text: label; color: baseFg
            font.pixelSize: 12; font.bold: true; font.family: monoFamily
        }
        MouseArea {
            id: ma; anchors.fill: parent
            hoverEnabled: true
            cursorShape: Qt.PointingHandCursor
            onClicked: parent.clicked()
        }
    }

    component ModeButton: ToolbarButton {
        property bool active: false
        variant: "default"
        color: active ? "#0a2e22" : cPanel2
        border.color: active ? cOk : cLine
        Text {
            anchors.centerIn: parent; text: parent.label
            color: parent.active ? cOk : cText
            font.pixelSize: 12; font.bold: true; font.family: monoFamily
        }
    }

    component BellButton: Rectangle {
        property int unread: 0
        implicitWidth: 44; implicitHeight: 30
        radius: 6
        color: cPanel2
        border.color: cLine; border.width: 1
        Text { anchors.centerIn: parent; text: "🔔"; font.pixelSize: 14 }
        Rectangle {
            visible: unread > 0
            width: 18; height: 16; radius: 8
            anchors.right: parent.right; anchors.top: parent.top
            anchors.rightMargin: -4; anchors.topMargin: -4
            color: cBad
            Text { anchors.centerIn: parent; text: unread; color: "#fff"; font.pixelSize: 10; font.bold: true }
        }
    }

    component CycleTimerChip: Rectangle {
        property string value: "00:00"
        property bool   running: false
        implicitWidth: 80; implicitHeight: 30
        radius: 6
        color: running ? cOkBg : cIdleBg
        border.color: running ? cOk : cLine; border.width: 1
        Text {
            anchors.centerIn: parent; text: value
            color: parent.running ? cOk : cText
            font.pixelSize: 13; font.bold: true; font.family: monoFamily
        }
    }

    component CycleCountChip: Rectangle {
        property int count: 0
        implicitWidth: row.implicitWidth + 20; implicitHeight: 30
        radius: 8; color: cIdleBg; border.color: cLine; border.width: 1
        RowLayout {
            id: row; anchors.centerIn: parent; spacing: 4
            Text { text: "Chu trinh:"; color: cMuted; font.pixelSize: 12 }
            Text { text: count;        color: cText;  font.pixelSize: 14; font.bold: true; font.family: monoFamily }
        }
    }

    component KvRow: RowLayout {
        property string label: ""
        property string valueText: ""
        property Item   valueChip: null
        width: parent.width
        spacing: 8
        Text {
            text: label; color: cMuted
            font.pixelSize: 12; font.bold: true; font.family: monoFamily
            Layout.preferredWidth: 130
        }
        Loader {
            Layout.fillWidth: true
            sourceComponent: valueChip ? chipWrap : textComp
            Component {
                id: textComp
                Text {
                    text: valueText; color: cText
                    font.pixelSize: 13; font.family: monoFamily
                    wrapMode: Text.WrapAtWordBoundaryOrAnywhere
                }
            }
            Component {
                id: chipWrap
                Item { width: childrenRect.width; height: childrenRect.height; children: valueChip ? [valueChip] : [] }
            }
        }
    }

    component StatusChip: Rectangle {
        property string state: "mid"  // on | off | mid
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

    component ChipItem: Rectangle {
        property string name: ""
        property string state: "mid"
        implicitWidth: 150; implicitHeight: 36
        radius: 6
        color: cPanel2; border.color: cLine; border.width: 1

        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 8; anchors.rightMargin: 8
            spacing: 6
            Text {
                text: name; color: cText
                font.pixelSize: 12; font.family: monoFamily
                elide: Text.ElideRight; Layout.fillWidth: true
            }
            StatusChip { state: parent.parent.state; label: parent.parent.state }
        }
    }

    component PressureCard: Rectangle {
        property string label: ""
        property string valueText: ""
        property string unit: "mbar"
        property real   ratio: 0.0

        Layout.fillWidth: true
        implicitHeight: pc.implicitHeight + 24
        radius: 8
        color: cPanel2; border.color: cLine; border.width: 1

        ColumnLayout {
            id: pc; x: 12; y: 12
            width: parent.width - 24
            spacing: 6
            RowLayout {
                width: parent.width
                Text { text: label; color: cMuted; font.pixelSize: 13; font.bold: true }
                Item { Layout.fillWidth: true }
                RowLayout {
                    spacing: 3
                    Text { text: valueText; color: cText; font.pixelSize: 22; font.bold: true; font.family: monoFamily }
                    Text { text: unit;      color: cMuted; font.pixelSize: 12; font.bold: true }
                }
            }
            Rectangle {
                Layout.fillWidth: true; height: 8; radius: 4
                color: "#134357"
                Rectangle {
                    height: parent.height
                    width: parent.width * Math.max(0, Math.min(1, ratio))
                    radius: parent.radius
                    color: cPrimary
                }
            }
        }
    }

    component CartridgeRow: Rectangle {
        property string name: ""
        property int    val: 0
        property real   ratio: 0
        property string cls: "ok"   // low | ok | high | limit

        Layout.fillWidth: true
        implicitHeight: 44
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
            anchors.leftMargin: 9; anchors.rightMargin: 9
            spacing: 8

            Text {
                text: name; color: cText
                font.pixelSize: 12; font.bold: true; font.family: monoFamily
                Layout.preferredWidth: 70
            }
            ColumnLayout {
                Layout.fillWidth: true; spacing: 2
                RowLayout {
                    width: parent.width
                    Item { Layout.fillWidth: true }
                    Text { text: val; color: cText; font.pixelSize: 14; font.bold: true; font.family: monoFamily }
                    Text { text: "mbar"; color: cMuted; font.pixelSize: 10; font.bold: true }
                }
                Rectangle {
                    Layout.fillWidth: true; height: 6; radius: 3
                    color: "#134357"
                    Rectangle {
                        height: parent.height; radius: parent.radius
                        width: parent.width * Math.max(0, Math.min(1, ratio))
                        color: cls === "ok"    ? cOk
                             : cls === "high"  ? cWarn
                             : cls === "limit" ? cBad
                             : "#0284c7"
                    }
                }
            }
        }
    }

    component ProcessCard: Rectangle {
        property string label: ""
        property string valueText: ""
        property bool   active: false
        Layout.preferredHeight: 76
        radius: 8
        color: active ? cPrimarySoft : cPanel2
        border.color: active ? cPrimary : cLine
        border.width: 1
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 10
            spacing: 5
            Text { text: label; color: cMuted; font.pixelSize: 12 }
            Text { text: valueText; color: cText; font.pixelSize: 16; font.bold: true; font.family: monoFamily; wrapMode: Text.Wrap }
        }
    }

    component IoControlRow: RowLayout {
        property string ioName: ""
        property string ioState: "mid"
        property string onLabel: "on"
        property string offLabel: "off"
        width: parent.width
        spacing: 8
        Layout.fillWidth: true

        Item {
            Layout.fillWidth: true
            Layout.topMargin: 6; Layout.bottomMargin: 6
            implicitHeight: nameCol.implicitHeight
            ColumnLayout {
                id: nameCol; width: parent.width; spacing: 2
                RowLayout {
                    spacing: 6
                    Text { text: ioName; color: cText; font.pixelSize: 12; font.bold: true; font.family: monoFamily }
                    StatusChip { state: ioState; label: ioState }
                }
            }
        }
        ToolbarButton { label: onLabel  }
        ToolbarButton { label: offLabel }
    }

    component SettingRow: RowLayout {
        property string settingName: ""
        property string settingVal:  ""
        property string settingUnit: ""
        width: parent.width
        spacing: 8
        Rectangle {
            Layout.fillWidth: true
            radius: 6
            color: cPanel2; border.color: cLine; border.width: 1
            implicitHeight: 36
            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 10; anchors.rightMargin: 10
                spacing: 8
                Text {
                    text: settingName; color: cText
                    font.pixelSize: 12; font.bold: true; font.family: monoFamily
                    Layout.fillWidth: true; elide: Text.ElideRight
                }
                Rectangle {
                    implicitWidth: 80; implicitHeight: 24
                    radius: 4; color: cBg; border.color: cLine; border.width: 1
                    TextInput {
                        anchors.fill: parent; anchors.margins: 4
                        text: settingVal; color: cText
                        font.pixelSize: 12; font.family: monoFamily
                        selectByMouse: true
                        horizontalAlignment: TextInput.AlignRight
                    }
                }
                Text { text: settingUnit; color: cMuted; font.pixelSize: 11; Layout.preferredWidth: 36 }
                ToolbarButton { label: "SET"; variant: "primary" }
            }
        }
    }

    component AlertRow: Rectangle {
        property string sev: "info"   // critical | error | warning | info
        property string time: ""
        property string title: ""
        property string text: ""
        Layout.fillWidth: true
        implicitHeight: arCol.implicitHeight + 18
        radius: 6
        color:        sev === "error"    ? Qt.rgba(1.0, 0.32, 0.32, 0.10)
                    : sev === "warning"  ? Qt.rgba(1.0, 0.65, 0.15, 0.10)
                    : sev === "critical" ? Qt.rgba(1.0, 0.15, 0.15, 0.18)
                    : Qt.rgba(0.31, 0.42, 1.0, 0.08)
        border.color: sev === "error"    ? cBad
                    : sev === "warning"  ? cWarn
                    : sev === "critical" ? cBad
                    : cPrimary
        border.width: 1

        RowLayout {
            id: arCol
            anchors.fill: parent
            anchors.margins: 10
            spacing: 10

            Text {
                text: sev === "error" ? "⛔" : sev === "warning" ? "⚠️" : sev === "critical" ? "🔥" : "ℹ️"
                font.pixelSize: 18
            }
            ColumnLayout {
                Layout.fillWidth: true; spacing: 2
                RowLayout {
                    Text { text: title; color: cText; font.pixelSize: 13; font.bold: true }
                    Item { Layout.fillWidth: true }
                    Text { text: time;  color: cMuted; font.pixelSize: 11; font.family: monoFamily }
                }
                Text { text: parent.parent.parent.text; color: cText; font.pixelSize: 12; font.family: monoFamily; wrapMode: Text.WrapAtWordBoundaryOrAnywhere; Layout.fillWidth: true }
            }
        }
    }
}
