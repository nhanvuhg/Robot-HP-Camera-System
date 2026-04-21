    import QtQuick 2.15
    import QtQuick.Controls 2.15
    import QtQuick.Layouts 1.15

    // ─── CSS VARIABLES (from cartridge_gui.py) ──────────────────────────────────
    // --bg:       #0c0c1d   page background
    // --bg2:      #1a1a35   card background
    // --card:     #141428   servo/sensor item bg
    // --border:   #2a2a50
    // --accent:   #4f6cff
    // --green:    #00e676   --red: #ff5252   --orange: #ffa726
    // --cyan:     #26c6da   --yellow: #ffd740   --dim: #8888aa   --text: #e8e8f0
    //
    // GRID: 230px | 1fr | 220px  /  rows: 3fr top + 2.5fr log
    // areas: "ctrl center servo" / "log log servo"
    // gap:4px padding:6px  height:calc(100vh - 56px)
    // ────────────────────────────────────────────────────────────────────────────
    Item {
        id: root
        anchors.fill: parent

        readonly property int headerH:  44
        readonly property int tabbarH:  32
        readonly property int gap:       4
        readonly property int pad:       6
        readonly property int ctrlW:   210
        readonly property int sensorW: 250
        readonly property real rowRatio: 3.0 / (3.0 + 2.5)

        property int gridH:   height - headerH - tabbarH
        property int outerW:  width  - pad * 2
        property int outerH:  gridH  - pad * 2
        property int centerW: outerW - ctrlW - sensorW - gap * 2
        property int topH:    Math.floor(outerH * rowRatio) - gap
        property int logH:    outerH - topH - gap

        readonly property color cBg:     "#0c0c1d"
        readonly property color cBg2:    "#081e29"
        readonly property color cCard:   "#051a1a"
        readonly property color cBorder: "#134357"
        readonly property color cAccent: "#4f6cff"
        readonly property color cGreen:  "#00e676"
        readonly property color cRed:    "#ff5252"
        readonly property color cOrange: "#ffa726"
        readonly property color cCyan:   "#26c6da"
        readonly property color cYellow: "#ffd740"
        readonly property color cDim:    "#8888aa"
        readonly property color cText:   "#e8e8f0"

        Rectangle { anchors.fill: parent; color: root.cBg }

        // ════════════════════════════════════════════════════════════
        // HEADER
        // ════════════════════════════════════════════════════════════
        Rectangle {
            id: header
            anchors { top: parent.top; left: parent.left; right: parent.right }
            height: root.headerH
            color: "#141428"; border.color: root.cBorder; z: 10

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 12
                anchors.rightMargin: 12
                spacing: 0
                Button {
                    text: "\u25c0"; Layout.preferredWidth: 30; Layout.preferredHeight: 24
                    font.pixelSize: 12; font.bold: true; onClicked: stackView.pop()
                    background: Rectangle { radius: 4; color: "#2a2a50"; border.color: root.cAccent }
                    contentItem: Text { text: parent.text; font: parent.font; color: root.cAccent; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
                Item { width: 10 }
                Row { spacing: 0
                    Text { text: "Cartridge"; color: root.cAccent; font.pixelSize: 18; font.bold: true; font.letterSpacing: 1 }
                    Text { text: " System";   color: root.cText;   font.pixelSize: 18; font.bold: true; font.letterSpacing: 1 }
                }
                Item { Layout.fillWidth: true }
                Rectangle {
                    id: stateBadge
                    Layout.preferredHeight: 28; radius: 8
                    Layout.preferredWidth: sbRow.implicitWidth + 22
                    color: root.cBg
                    border.color: {
                        var s = cartridgeController.systemState.toUpperCase()
                        if (s.indexOf("ERROR") !== -1) return root.cRed
                        if (s === "IDLE" || s === "UNKNOWN" || s === "") return root.cOrange
                        return root.cGreen
                    }
                    Behavior on border.color { ColorAnimation { duration: 200 } }
                    Row { id: sbRow; anchors.centerIn: parent; spacing: 8
                        Rectangle {
                            id: stateDot
                            width: 9; height: 9; radius: 4.5
                            anchors.verticalCenter: parent.verticalCenter
                            color: stateBadge.border.color
                            Behavior on color { ColorAnimation { duration: 200 } }
                            SequentialAnimation on opacity { loops: Animation.Infinite
                                NumberAnimation { to: 0.35; duration: 900 }
                                NumberAnimation { to: 1.0;  duration: 900 } }
                        }
                        Text { text: cartridgeController.systemState.replace(/\|/g, "    •    "); color: root.cText
                            font.pixelSize: 14; font.bold: true; font.letterSpacing: 1
                            anchors.verticalCenter: parent.verticalCenter }
                    }
                }
                Item { width: 10 }
                Rectangle {
                    Layout.preferredHeight: 28; radius: 6
                    Layout.preferredWidth: hmRow.implicitWidth + 16
                    color: cartridgeController.systemState.toLowerCase().indexOf("homing") !== -1 ? "#2a1a00"
                         : (cartridgeController.systemState === "idle" && cartridgeController.currentMode !== "") ? "#0a2a0a"
                         : "#081e29"
                    border.color: cartridgeController.systemState.toLowerCase().indexOf("homing") !== -1 ? root.cOrange
                                : (cartridgeController.systemState === "idle" && cartridgeController.currentMode !== "") ? root.cGreen
                                : root.cBorder
                    border.width: 1
                    Row {
                        id: hmRow
                        anchors.centerIn: parent; spacing: 6
                        Text {
                            text: cartridgeController.systemState.toLowerCase().indexOf("homing") !== -1 ? "⟳ HOMING..."
                                : (cartridgeController.currentMode !== "" && cartridgeController.systemState === "idle") ? "✓ HOMED"
                                : "○ NOT HOMED"
                            color: cartridgeController.systemState.toLowerCase().indexOf("homing") !== -1 ? root.cOrange
                                 : (cartridgeController.currentMode !== "" && cartridgeController.systemState === "idle") ? root.cGreen
                                 : root.cDim
                            font.pixelSize: 11; font.bold: true
                        }
                    }
                }
                Item { Layout.fillWidth: true }
                Rectangle {
                    id: modePill; Layout.preferredHeight: 28; radius: 20
                    property string m: cartridgeController.currentMode
                    property bool isIdle: m === "idle" || m === ""
                    Layout.preferredWidth: mpLbl.implicitWidth + 26
                    color: isIdle ? "#2a1a00" : m === "auto" ? "#0a332e" : m === "jog" ? "#332e0a" : "#051a25"
                    border.color: isIdle ? "#ffd740" : m === "auto" ? root.cGreen : m === "jog" ? root.cOrange : "#5cf4f1"
                    border.width: isIdle ? 2 : 1

                    // Nhấp nháy khi chưa chọn mode
                    SequentialAnimation on opacity {
                        loops: Animation.Infinite; running: modePill.isIdle
                        NumberAnimation { to: 0.4; duration: 600 }
                        NumberAnimation { to: 1.0; duration: 600 }
                    }
                    opacity: modePill.isIdle ? 1.0 : 1.0

                    Text {
                        id: mpLbl
                        anchors.centerIn: parent
                        text: modePill.isIdle ? "⚠  SELECT MODE" : modePill.m.toUpperCase()
                        color: modePill.isIdle ? "#ffd740" : modePill.border.color
                        font.pixelSize: modePill.isIdle ? 12 : 13
                        font.bold: true; font.letterSpacing: 1
                    }
                }

                // ── Reset Faults button trong header ──
                Item { width: 8 }
                Button {
                    text: "🔄 Faults"
                    Layout.preferredHeight: 26
                    font.pixelSize: 11; font.bold: true
                    onClicked: cartridgeController.resetFaults()
                    background: Rectangle { radius: 4; color: "#3a1a0a"; border.color: root.cOrange; border.width: 1 }
                    contentItem: Text { text: parent.text; font: parent.font; color: root.cOrange;
                        horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
                Item { width: 4 }
            }
        }

        // ════════════════════════════════════════════════════════════
        // OUTPUT TRAY TIMEOUT WARNING
        // ════════════════════════════════════════════════════════════

        Popup {
            id: outputWarningPopup
            width: 320; height: 160
            anchors.centerIn: parent
            modal: true; focus: true
            closePolicy: Popup.NoAutoClose
            background: Rectangle {
                color: "#1a0f05"
                border.color: root.cOrange
                border.width: 2
                radius: 10
            }
            contentItem: ColumnLayout {
                spacing: 15
                Text {
                    text: "⚠️ CẢNH BÁO"
                    color: root.cOrange
                    font.pixelSize: 18; font.bold: true
                    Layout.alignment: Qt.AlignHCenter
                }
                Text {
                    text: "Hệ thống đang chờ khay cấp khay thành phẩm.\ Bạn đã cấp khay mới chưa?"
                    color: "white"
                    font.pixelSize: 14
                    horizontalAlignment: Text.AlignHCenter
                    Layout.alignment: Qt.AlignHCenter
                }
                Item { Layout.fillHeight: true }
                RowLayout {
                    Layout.alignment: Qt.AlignHCenter
                    spacing: 20
                    Button {
                        text: "ĐÃ CẤP KHAY"
                        font.bold: true; font.pixelSize: 12
                        Layout.preferredWidth: 120; Layout.preferredHeight: 35
                        background: Rectangle { color: root.cOrange; radius: 5 }
                        contentItem: Text { text: parent.text; font: parent.font; color: "#000"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        onClicked: {
                            cartridgeController.confirmOutput();
                            // outputWarningPopup.close();
                        }
                    }
                    Button {
                        text: "CHỜ THÊM"
                        font.bold: true; font.pixelSize: 12
                        Layout.preferredWidth: 100; Layout.preferredHeight: 35
                        background: Rectangle { color: "#134357"; radius: 5; border.color: "#2a3a4a"; border.width: 1 }
                        contentItem: Text { text: parent.text; font: parent.font; color: "#fff"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        onClicked: {
                            // outputWarningPopup.close();
                        }
                    }
                }
            }
        }

        // ════════════════════════════════════════════════════════════
        // NOTIFICATION BANNER — gui_notify từ node (watchdog, cylinder timeout, errors)
        // ════════════════════════════════════════════════════════════
        Rectangle {
            id: notifyBanner
            anchors { top: header.bottom; left: parent.left; right: parent.right }
            height: visible ? 36 : 0
            visible: false
            z: 9
            clip: true
            Behavior on height { NumberAnimation { duration: 200 } }

            property string lvl: "info"   // "info" | "warn" | "error"
            property string ttl: ""
            property string dtl: ""

            color: {
                if (lvl === "error") return "#2a0808"
                if (lvl === "warn")  return "#2a2008"
                return "#081e28"
            }
            border.color: {
                if (lvl === "error") return root.cRed
                if (lvl === "warn")  return root.cOrange
                return root.cCyan
            }
            border.width: 1

            Connections {
                target: cartridgeController
                function onNotificationReceived() {
                    try {
                        var obj = JSON.parse(cartridgeController.lastNotification)
                        if (obj.level && obj.level.indexOf("silent") === 0) return;
                        // notifyBanner.lvl = obj.level  || "info"
                        // notifyBanner.ttl = obj.title  || ""
                        // notifyBanner.dtl = obj.detail || ""
                        // notifyBanner.visible = true
                        // bannerTimer.restart()

                        if (obj.title === "Da phat hien khay") {
                            // outputWarningPopup.open()
                        }
                    } catch(e) {}
                }
            }

            Timer {
                id: bannerTimer
                // info/warn tự ẩn sau 6s — error ở lại cho đến khi bấm ✕
                interval: notifyBanner.lvl === "error" ? 30000 : 6000
                onTriggered: { if (notifyBanner.lvl !== "error") notifyBanner.visible = false }
            }

            RowLayout {
                anchors { fill: parent; leftMargin: 12; rightMargin: 6 }
                spacing: 8

                // Level icon
                Text {
                    text: {
                        if (notifyBanner.lvl === "error") return "🚨"
                        if (notifyBanner.lvl === "warn")  return "⚠"
                        return "ℹ"
                    }
                    font.pixelSize: 13
                }
                // Title
                Text {
                    text: notifyBanner.ttl
                    color: {
                        if (notifyBanner.lvl === "error") return root.cRed
                        if (notifyBanner.lvl === "warn")  return root.cOrange
                        return root.cCyan
                    }
                    font.pixelSize: 12; font.bold: true
                }
                // Detail (fills remaining space)
                Text {
                    text: notifyBanner.dtl
                    color: root.cText; font.pixelSize: 11
                    Layout.fillWidth: true
                    elide: Text.ElideRight; opacity: 0.9
                }
                // Reset Faults shortcut (chỉ hiện khi error)
                Button {
                    visible: notifyBanner.lvl === "error"
                    text: "Reset"
                    Layout.preferredHeight: 22
                    font.pixelSize: 10; font.bold: true
                    onClicked: { cartridgeController.resetFaults(); notifyBanner.visible = false }
                    background: Rectangle { radius: 3; color: "#3a0a0a"; border.color: root.cOrange; border.width: 1 }
                    contentItem: Text { text: parent.text; font: parent.font; color: root.cOrange;
                        horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
                // Dismiss button
                Button {
                    text: "✕"
                    Layout.preferredWidth: 22; Layout.preferredHeight: 22
                    font.pixelSize: 11
                    onClicked: notifyBanner.visible = false
                    background: Rectangle { radius: 3; color: "transparent"; border.color: root.cBorder; border.width: 1 }
                    contentItem: Text { text: parent.text; font: parent.font; color: root.cDim;
                        horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
            }
        }

        // ════════════════════════════════════════════════════════════
        // TAB BAR
        // ════════════════════════════════════════════════════════════
        Rectangle {
            id: tabbar
            anchors { top: notifyBanner.bottom; left: parent.left; right: parent.right }
            height: root.tabbarH
            color: "#141428"; border.color: root.cBorder

            Row { anchors { fill: parent; leftMargin: 16 }
            spacing: 2
                Repeater {
                    model: ListModel {
                        ListElement { t: "Control Dashboard"; k: "control" }
                        ListElement { t: "Technical System";  k: "config"  }
                        ListElement { t: "Robot Control";     k: "robot"   }
                    }
                    delegate: Rectangle {
                        height: root.tabbarH - 2; width: tabLbl.width + 36; y: 1; radius: 6
                        color: stack.currentIndex === index ? root.cCard : "transparent"
                        border.color: stack.currentIndex === index ? root.cBorder : "transparent"
                        Text { id: tabLbl; anchors.centerIn: parent; text: model.t; font.pixelSize: 14; font.bold: true
                            color: stack.currentIndex === index ? root.cAccent : root.cDim }
                        MouseArea { anchors.fill: parent; onClicked: stack.currentIndex = index }
                    }
                }
            }
        }

        // ════════════════════════════════════════════════════════════
        // STACK
        // ════════════════════════════════════════════════════════════
        StackLayout {
            id: stack
            anchors { top: tabbar.bottom; left: parent.left; right: parent.right; bottom: parent.bottom }
            currentIndex: 0

            // ── PAGE 1: CONTROL DASHBOARD ────────────────────────
            Item {
                Item {
                    id: pageGrid
                    anchors { fill: parent; margins: root.pad }

                    // ─ CTRL COL ──────────────────────────────────
                    ColumnLayout {
                        x: 0; y: 0; width: root.ctrlW; height: root.topH
                        spacing: root.gap

                        // ── Mode Selection ──────────────────────
                        Rectangle {
                            Layout.fillWidth: true
                            Layout.fillHeight: true          // ← chia đều 1/3
                            color: root.cBg2; border.color: root.cBorder; radius: 6
                            HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cAccent : root.cBorder }

                            ColumnLayout { id: modeSelCol
                                anchors.fill: parent; anchors.margins: 8
                                spacing: 4

                                // modeBlocked: đang chạy (state machine busy) HOẶC chưa chọn mode (idle)
                                property bool modeIsIdle: cartridgeController.currentMode === "idle" || cartridgeController.currentMode === ""
                                property bool modeBlocked: {
                                    var s = cartridgeController.systemState.toLowerCase()
                                    return s !== "" && s !== "idle" && s !== "unknown"
                                }

                                Text {
                                    text: "MODE SELECTION"; color: root.cAccent
                                    font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.5
                                }

                                // ── Dropdown Mode Selector ──────────
                                Item {
                                    id: modeDropdown
                                    Layout.fillWidth: true
                                    Layout.fillHeight: true
                                    property bool expanded: modeSelCol.modeIsIdle

                                    // ── Thanh hiển thị (header) ──
                                    Rectangle {
                                        id: modeHeader
                                        width: parent.width
                                        height: 36
                                        radius: 6
                                        color: root.cCard
                                        border.color: {
                                            var m = cartridgeController.currentMode
                                            if (m === "auto")   return root.cGreen
                                            if (m === "manual") return "#5cf4f1"
                                            if (m === "jog")    return root.cOrange
                                            return root.cBorder
                                        }
                                        border.width: modeSelCol.modeIsIdle ? 1 : 2
                                        Behavior on border.color { ColorAnimation { duration: 150 } }

                                        Row {
                                            anchors { left: parent.left; leftMargin: 10; verticalCenter: parent.verticalCenter }
                                            spacing: 8
                                            Text {
                                                id: modeIcon
                                                text: {
                                                    var m = cartridgeController.currentMode
                                                    if (m === "auto")   return "●"
                                                    if (m === "manual" || m === "jog") return "●"
                                                    return "○"
                                                }
                                                color: {
                                                    var m = cartridgeController.currentMode
                                                    if (m === "auto")   return root.cGreen
                                                    if (m === "manual") return "#5cf4f1"
                                                    if (m === "jog")    return root.cOrange
                                                    return root.cDim
                                                }
                                                font.pixelSize: 12
                                                anchors.verticalCenter: parent.verticalCenter
                                            }
                                            Text {
                                                text: {
                                                    var m = cartridgeController.currentMode
                                                    if (m === "auto")   return "AUTO"
                                                    if (m === "manual") return "MANUAL"
                                                    if (m === "jog")    return "MANUAL (JOG)"
                                                    return "Chọn chế độ..."
                                                }
                                                color: {
                                                    var m = cartridgeController.currentMode
                                                    if (m === "auto")   return root.cGreen
                                                    if (m === "manual") return "#5cf4f1"
                                                    if (m === "jog")    return root.cOrange
                                                    return root.cDim
                                                }
                                                font.pixelSize: 12; font.bold: !modeSelCol.modeIsIdle
                                                anchors.verticalCenter: parent.verticalCenter
                                            }
                                        }
                                        // Mũi tên
                                        Text {
                                            anchors { right: parent.right; rightMargin: 10; verticalCenter: parent.verticalCenter }
                                            text: modeDropdown.expanded ? "▲" : "▼"
                                            color: root.cDim; font.pixelSize: 10
                                        }

                                        MouseArea {
                                            anchors.fill: parent
                                            onClicked: {
                                                if (!modeSelCol.modeBlocked)
                                                    modeDropdown.expanded = !modeDropdown.expanded
                                            }
                                        }
                                    }

                                    // ── Options (expanded) ──
                                    Column {
                                        id: modeOptions
                                        visible: modeDropdown.expanded
                                        anchors { top: modeHeader.bottom; topMargin: 4; left: parent.left; right: parent.right }
                                        spacing: 4

                                        // AUTO
                                        Rectangle {
                                            width: parent.width; height: 32; radius: 5
                                            color: "#0d3d2e"; border.color: root.cGreen; border.width: 1
                                            HoverHandler { onHoveredChanged: parent.opacity = hovered ? 0.85 : 1.0 }
                                            MouseArea {
                                                anchors.fill: parent
                                                onClicked: {
                                                    cartridgeController.setMode("auto")
                                                    modeDropdown.expanded = false
                                                }
                                            }
                                            Row {
                                                anchors { left: parent.left; leftMargin: 10; verticalCenter: parent.verticalCenter }
                                                spacing: 8
                                                Text { text: "●"; color: root.cGreen; font.pixelSize: 11; anchors.verticalCenter: parent.verticalCenter }
                                                Column {
                                                    anchors.verticalCenter: parent.verticalCenter
                                                    Text { text: "AUTO"; color: root.cGreen; font.pixelSize: 11; font.bold: true }
                                                    Text { text: "Camera / Robot tín hiệu"; color: root.cDim; font.pixelSize: 8 }
                                                }
                                            }
                                        }

                                        // MANUAL
                                        Rectangle {
                                            width: parent.width; height: 32; radius: 5
                                            color: "#051a1a"; border.color: "#5cf4f1"; border.width: 1
                                            HoverHandler { onHoveredChanged: parent.opacity = hovered ? 0.85 : 1.0 }
                                            MouseArea {
                                                anchors.fill: parent
                                                onClicked: {
                                                    cartridgeController.setMode("manual")
                                                    modeDropdown.expanded = false
                                                }
                                            }
                                            Row {
                                                anchors { left: parent.left; leftMargin: 10; verticalCenter: parent.verticalCenter }
                                                spacing: 8
                                                Text { text: "●"; color: "#5cf4f1"; font.pixelSize: 11; anchors.verticalCenter: parent.verticalCenter }
                                                Column {
                                                    anchors.verticalCenter: parent.verticalCenter
                                                    Text { text: "MANUAL"; color: "#5cf4f1"; font.pixelSize: 11; font.bold: true }
                                                    Text { text: "Điều khiển tay trực tiếp"; color: root.cDim; font.pixelSize: 8 }
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                        }

                        // ── System Control ───────────────────────
                        Rectangle {
                            Layout.fillWidth: true
                            Layout.fillHeight: true          // ← chia đều 1/3
                            color: root.cBg2; border.color: root.cBorder; radius: 6
                            HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cAccent : root.cBorder }

                            ColumnLayout {
                                anchors.fill: parent; anchors.margins: 8
                                spacing: 4
                                // Không cho chạy khi chưa chọn mode
                                enabled: !parent.parent.parent.modeIsIdle
                                opacity: parent.parent.parent.modeIsIdle ? 0.35 : 1.0
                                Behavior on opacity { NumberAnimation { duration: 200 } }

                                Text {
                                    text: "SYSTEM CONTROL"; color: root.cAccent
                                    font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.5
                                }

                                // Hàng 1: START / STOP / PAUSE
                                RowLayout {
                                    Layout.fillWidth: true; Layout.fillHeight: true; spacing: 4
                                    CBtn {
                                        Layout.fillWidth: true; Layout.fillHeight: true
                                        lbl: "START"
                                        bg: "#0a332e"; bc: root.cGreen;  tc: root.cGreen
                                        onClicked: cartridgeController.startSystem()
                                    }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; lbl: "STOP";   bg: "#4d1a1a"; bc: root.cRed;    tc: root.cRed;    onClicked: { robotController.stopAndResetRobot(); cartridgeController.stopSystem() } }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; lbl: "PAUSE";  bg: "#4d3a0a"; bc: root.cOrange; tc: root.cOrange; onClicked: cartridgeController.pauseSystem() }
                                }

                                // Hàng 2: RESUME
                                RowLayout {
                                    Layout.fillWidth: true; Layout.fillHeight: true; spacing: 4
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; lbl: "Resume";  bg: "#0a332e"; bc: root.cGreen;  tc: root.cGreen;  onClicked: cartridgeController.hmiResume() }
                                }


                            }
                        }

                        // ── State Navigation ─────────────────────
                        Rectangle {
                            Layout.fillWidth: true
                            Layout.fillHeight: true          // ← chia đều 1/3
                            color: root.cBg2; border.color: root.cBorder; radius: 6
                            HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cAccent : root.cBorder }

                            ColumnLayout {
                                anchors.fill: parent; anchors.margins: 8
                                spacing: 4
                                // Không cho chạy khi chưa chọn mode
                                enabled: !parent.parent.parent.modeIsIdle
                                opacity: parent.parent.parent.modeIsIdle ? 0.35 : 1.0
                                Behavior on opacity { NumberAnimation { duration: 200 } }

                                Text {
                                    text: "STATE NAVIGATION"; color: root.cAccent
                                    font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.5
                                }

                                // Grid 2 cột × 4 hàng
                                GridLayout {
                                    Layout.fillWidth: true; Layout.fillHeight: true
                                    columns: 2; columnSpacing: 4; rowSpacing: 4

                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; lbl: "HOMING";  bg: root.cCard;   bc: root.cBorder; tc: root.cText;   onClicked: cartridgeController.gotoState("HOMING") }
                                    CBtn {
                                        Layout.fillWidth: true; Layout.fillHeight: true
                                        lbl: cartridgeController.currentMode === "jog" ? "JOG MODE" : "STOP STATE"
                                        bg: cartridgeController.currentMode === "jog" ? "#0a332e" : "#4d1a1a"
                                        bc: cartridgeController.currentMode === "jog" ? root.cGreen  : root.cRed
                                        tc: cartridgeController.currentMode === "jog" ? root.cGreen  : root.cRed
                                        onClicked: {
                                            if (cartridgeController.currentMode === "jog") {
                                                console.log("JOG mode verified")
                                            } else {
                                                cartridgeController.gotoState("ABORT_TO_JOG")
                                            }
                                        }
                                    }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; lbl: "STATE 1\nNạp khay In"; bg: "#1a2050"; bc: "#00ffff"; tc: root.cAccent; isSelected: cartridgeController.systemState.indexOf("S1_") !== -1 || cartridgeController.systemState.indexOf("STATE1") !== -1; onClicked: cartridgeController.gotoState("STATE1") }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; lbl: "STATE 2\nThay khay In"; bg: "#1a2050"; bc: "#00ffff"; tc: root.cAccent; isSelected: cartridgeController.systemState.indexOf("S2A_") !== -1 || cartridgeController.systemState.indexOf("STATE2") !== -1; onClicked: cartridgeController.simulateDoneTrayInput() }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; lbl: "STATE 3\nCấp khay Out"; bg: "#1a2050"; bc: root.cGreen; tc: root.cGreen; isSelected: cartridgeController.systemState.indexOf("S3_") !== -1 || cartridgeController.systemState.indexOf("STATE3") !== -1; onClicked: cartridgeController.gotoState("STATE3") }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; lbl: "STATE 4\nThay khay Out"; bg: "#1a2050"; bc: root.cGreen; tc: root.cGreen; isSelected: cartridgeController.systemState.indexOf("S4_") !== -1 || cartridgeController.systemState.indexOf("STATE4") !== -1; onClicked: cartridgeController.simulateDoneTrayOutput() }
                                }
                            }
                        }
                    }

                    // ─ CENTER COL ────────────────────────────────
                    Item {
                        x: root.ctrlW + root.gap; y: 0
                        width: root.centerW; height: root.topH

                        Column {
                            anchors.fill: parent; spacing: root.gap

                            // Target Row
                            Rectangle {
                                width: parent.width; height: trCol.implicitHeight + 10
                                color: root.cBg2; border.color: root.cBorder; radius: 6
                                HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cAccent : root.cBorder }
                                Column { id: trCol; anchors { fill: parent; margins: 8 }
                                spacing: 4
                                    Text { text: "TARGET ROW"; color: root.cAccent; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.5 }
                                    Row { spacing: 4
                                        Repeater { model: [10,9,8,7,6,5,4,3,2,1]
                                            delegate: CBtn { lbl: "R"+modelData; padV: 4; padH: 10; fontSize: 11; bg: root.cCard; bc: root.cBorder; tc: root.cText; onClicked: cartridgeController.setTargetRow(modelData) }
                                        }
                                    }
                                }
                            }

                            // Servo Control (flex:1 → fills remaining)
                            Rectangle {
                                width: parent.width
                                height: parent.height - (trCol.implicitHeight + 10) - root.gap
                                color: root.cBg2; border.color: root.cBorder; radius: 6; clip: true
                                HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cAccent : root.cBorder }

                                Column {
                                    anchors.fill: parent
                                    anchors.margins: 8
                                    spacing: 4

                                    Row { width: parent.width; height: 20; spacing: 6
                                        Text { text: "SERVO CONTROL"; color: root.cAccent; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.5; anchors.verticalCenter: parent.verticalCenter }
                                        Text { text: "Vel:"; color: root.cDim; font.pixelSize: 10; anchors.verticalCenter: parent.verticalCenter }
                                        Rectangle { width: 50; height: 18; radius: 6; color: root.cBg; border.color: root.cBorder; anchors.verticalCenter: parent.verticalCenter
                                            TextInput { id: velInput; anchors.centerIn: parent; text: "30"; font.pixelSize: 11; color: root.cText; horizontalAlignment: TextInput.AlignHCenter; validator: IntValidator { bottom:1; top:200 } }
                                        }
                                        Text { text: "mm/s"; color: root.cDim; font.pixelSize: 10; anchors.verticalCenter: parent.verticalCenter }
                                    }

                                    // 5 servo cards horizontal
                                    Row {
                                        id: servoRow
                                        width: parent.width
                                        height: parent.height - 20 - 4
                                        spacing: root.gap
                                        property bool jogAllowed: cartridgeController.currentMode === "jog"

                                        Repeater {
                                            model: ListModel {
                                                ListElement { sid: 1; sname: "InX";     sdesc: "Trục X đầu vào" }
                                                ListElement { sid: 2; sname: "InY";     sdesc: "Trục Y đầu vào" }
                                                ListElement { sid: 3; sname: "PutTray"; sdesc: "Đẩy khay" }
                                                ListElement { sid: 4; sname: "OutX";    sdesc: "Trục X đầu ra" }
                                                ListElement { sid: 5; sname: "OutY";    sdesc: "Trục Y đầu ra" }
                                            }
                                            delegate: Rectangle {
                                                width: Math.floor((servoRow.width - 4*root.gap) / 5)
                                                height: servoRow.height
                                                color: root.cCard; border.color: root.cBorder; radius: 4; clip: true
                                                HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cAccent : root.cBorder }

                                                Column {
                                                    anchors.fill: parent
                                                    anchors.margins: 6
                                                    spacing: 6; width: parent.width - 12

                                                    // header: name + desc
                                                    Column { width: parent.width; spacing: 2
                                                        Text { text: "S"+model.sid+": "+model.sname; color: root.cCyan; font.pixelSize: 14; font.bold: true; width: parent.width; horizontalAlignment: Text.AlignHCenter }
                                                        Text { text: model.sdesc; color: root.cDim; font.pixelSize: 11; width: parent.width; horizontalAlignment: Text.AlignHCenter; elide: Text.ElideRight }
                                                    }

                                                    // position display — direct connect + deadband
                                                    Text {
                                                        id: posText
                                                        width: parent.width; horizontalAlignment: Text.AlignHCenter
                                                        text: "--"
                                                        color: root.cYellow; font.pixelSize: 22; font.bold: true
                                                        property real lastVal: -99999
                                                        Connections {
                                                            target: cartridgeController
                                                            function onServoPositionsChanged() {
                                                                try {
                                                                    var p = JSON.parse(cartridgeController.servoPositions)[model.sid]
                                                                    if (p !== undefined && p !== null) {
                                                                        var v = Number(p)
                                                                        if (Math.abs(v - posText.lastVal) >= 0.05) {
                                                                            posText.lastVal = v
                                                                            posText.text = v.toFixed(1) + " mm"
                                                                        }
                                                                    }
                                                                } catch(e) {}
                                                            }
                                                        }
                                                    }

                                                    // − STOP + (jog hoặc manual mode)
                                                    Row { spacing: 4; anchors.horizontalCenter: parent.horizontalCenter
                                                        CBtn { lbl:"−"; padV:10; padH:16; fontSize:18; bg:root.cCard; bc:root.cBorder; tc:root.cText; active: servoRow.jogAllowed
                                                            onPressed: { if(servoRow.jogAllowed) cartridgeController.jogServo(model.sid,"-",parseInt(velInput.text)||30) }
                                                            onReleased: cartridgeController.jogStop(model.sid) }
                                                        CBtn { lbl:"STOP"; padV:10; padH:8; fontSize:14; bg:"#4d1a1a"; bc:root.cRed; tc:root.cRed; onClicked: cartridgeController.jogStop(model.sid) }
                                                        CBtn { lbl:"+"; padV:10; padH:16; fontSize:18; bg:root.cCard; bc:root.cBorder; tc:root.cText; active: servoRow.jogAllowed
                                                            onPressed: { if(servoRow.jogAllowed) cartridgeController.jogServo(model.sid,"+",parseInt(velInput.text)||30) }
                                                            onReleased: cartridgeController.jogStop(model.sid) }
                                                    }

                                                    // HOMING (jog hoặc manual mode)
                                                    CBtn { lbl:"HOMING"; w:parent.width; padV:12; padH:12; fontSize:16; bg:"#0a332e"; bc:root.cGreen; tc:root.cGreen; active:servoRow.jogAllowed; onClicked: { if(servoRow.jogAllowed) cartridgeController.homeServo(model.sid) } }

                                                    // CLEAR (always available)
                                                    CBtn { lbl:"CLEAR"; w:parent.width; padV:12; padH:12; fontSize:16; bg:"#4d3a0a"; bc:root.cOrange; tc:root.cOrange; onClicked: cartridgeController.clearServo(model.sid) }

                                                    // pos-row (jog mode required)
                                                    Row { spacing: 4; anchors.horizontalCenter: parent.horizontalCenter
                                                        Rectangle { width:72; height:34; radius:6; color:root.cBg; border.color:root.cBorder
                                                            TextInput { id:posIn; anchors.fill: parent; anchors.margins: 4; text:"0.0"; font.pixelSize:15; color:root.cText; horizontalAlignment:TextInput.AlignHCenter; verticalAlignment:TextInput.AlignVCenter } }
                                                        Text { text:"mm"; color:root.cDim; font.pixelSize:12; anchors.verticalCenter:parent.verticalCenter; rightPadding:2 }
                                                        CBtn { lbl:"RUN"; padV:10; padH:14; fontSize:16; bg:root.cAccent; bc:root.cAccent; tc:"#fff"; active:servoRow.isJog
                                                            onClicked: { if(servoRow.isJog) { var v=parseFloat(posIn.text); if(!isNaN(v)) cartridgeController.moveServo(model.sid,v) } } }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // ─ LOG AREA ──────────────────────────────────
                    Rectangle {
                        x: 0; y: root.topH + root.gap
                        width: parent.width - root.sensorW - root.gap
                        height: root.logH
                        color: root.cBg2; border.color: root.cBorder; radius: 6
                        HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cAccent : root.cBorder }

                        Column {
                            anchors.fill: parent
                            anchors.margins: 8
                            spacing: 4
                            RowLayout { width: parent.width; height: 18
                                Text { text: "LOG ACTIVITY"; color: root.cAccent; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.5 }
                                Item { Layout.fillWidth: true }
                                CBtn { lbl:"Clear"; padV:4; padH:10; fontSize:11; bg:root.cCard; bc:root.cBorder; tc:root.cText; onClicked: cartridgeController.clearLog() }
                            }
                            Rectangle {
                                width: parent.width; height: parent.height - 18 - 4
                                color: "#0a0a18"; border.color: root.cBorder; radius: 4
                                ListView { anchors { fill: parent; margins: 6 }
                                    model: cartridgeController.logEntries; clip: true; spacing: 2
                                    verticalLayoutDirection: ListView.BottomToTop
                                    delegate: Loader {
                                        property var entry: modelData
                                        active: {
                                            var m = (modelData.msg || "").toLowerCase()
                                            return m.indexOf("no error") === -1
                                        }
                                        width: parent ? parent.width : 100
                                        sourceComponent: Text {
                                            width: parent ? parent.width : 100
                                            text: "[" + entry.time + "] " + entry.msg
                                            font.pixelSize: 18; font.family: "monospace"
                                            color: entry.type==="err" ? root.cRed : entry.type==="ok" ? root.cGreen : root.cCyan
                                            wrapMode: Text.WordWrap
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // ─ SENSOR SIMULATION (grid-area: servo, full height) ──
                    Rectangle {
                        x: parent.width - root.sensorW
                        y: 0; width: root.sensorW; height: root.outerH
                        color: root.cBg2; border.color: root.cBorder; radius: 6
                        HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cAccent : root.cBorder }

                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: 6
                            spacing: 3

                            // ── Tiêu đề ──
                            Text {
                                text: "SENSOR SIMULATION"
                                color: root.cAccent; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.5
                            }

                            // ── Nút All ON / OFF / Clear ──
                            Row {
                                spacing: 3
                                enabled: cartridgeController.currentMode === "jog" || cartridgeController.currentMode === "manual"
                                opacity: enabled ? 1.0 : 0.3
                                CBtn { lbl:"All ON";  padV:3; padH:8; fontSize:10; bg:"#0a332e"; bc:root.cGreen;  tc:root.cGreen;  onClicked: cartridgeController.simAll(1) }
                                CBtn { lbl:"All OFF"; padV:3; padH:8; fontSize:10; bg:"#4d1a1a"; bc:root.cRed;    tc:root.cRed;    onClicked: cartridgeController.simAll(0) }
                                CBtn { lbl:"Clear";   padV:3; padH:6; fontSize:10; bg:root.cCard; bc:root.cBorder; tc:root.cText;  onClicked: cartridgeController.simSensor("clear") }
                            }

                            // ── Quick Preset ──
                            Text { text: "QUICK PRESET"; color: root.cDim; font.pixelSize: 9; font.bold: true; font.letterSpacing: 0.8 }
                            Row {
                                spacing: 3
                                enabled: cartridgeController.currentMode === "jog" || cartridgeController.currentMode === "manual"
                                opacity: enabled ? 1.0 : 0.3
                                // S1 Entry: điều kiện vào State 1
                                CBtn {
                                    lbl: "S1 Entry"
                                    padV: 3; padH: 6; fontSize: 10
                                    bg: "#0a1a4d"; bc: root.cAccent; tc: root.cAccent
                                    onClicked: {
                                        cartridgeController.simSensor("clear")
                                        // S1+S3+S13 ON (băng tải có khay, Cyl 1 rút)
                                        var ids = [1,3,13]
                                        ids.forEach(function(id) {
                                            cartridgeController.simSensor(id + ":1")
                                        })
                                    }
                                }
                                // S1 Full — S4 ON để test scan trigger
                                CBtn {
                                    lbl: "S1 Full"
                                    padV: 3; padH: 6; fontSize: 10
                                    bg: "#051a1a"; bc: "#5cf4f1"; tc: "#5cf4f1"
                                    onClicked: {
                                        cartridgeController.simSensor("clear")
                                        // S1+S3+S4+S13 ON (Cyl 1 rút)
                                        var ids = [1,3,4,13]
                                        ids.forEach(function(id) {
                                            cartridgeController.simSensor(id + ":1")
                                        })
                                    }
                                }
                                // S3 Entry: cấp khay Pos2
                                CBtn {
                                    lbl: "S3 Entry"
                                    padV: 3; padH: 6; fontSize: 10
                                    bg: "#0a3a1a"; bc: root.cGreen; tc: root.cGreen
                                    onClicked: {
                                        cartridgeController.simSensor("clear")
                                        // S7 ON (có khay trên Platform)
                                        cartridgeController.simSensor("7:1")
                                    }
                                }
                            }

                            // ── Status label ──
                            Text { text: "STATUS"; color: root.cDim; font.pixelSize: 10; font.bold: true; font.letterSpacing: 1 }

                            // ── Grid sensor – fillHeight để tự co vừa chiều cao còn lại ──
                            GridLayout {
                                Layout.fillWidth: true
                                Layout.fillHeight: true      // ← KEY: chiếm hết không gian còn lại
                                columns: 2
                                columnSpacing: 4
                                rowSpacing: 3

                                Repeater {
                                    model: ListModel {
                                        // S1-S8: Module 1
                                        ListElement { sid:1;  slabel:"S1";  sdesc:"Belt start" }
                                        ListElement { sid:2;  slabel:"S2";  sdesc:"Belt mid" }
                                        ListElement { sid:3;  slabel:"S3";  sdesc:"Belt end"}
                                        ListElement { sid:4;  slabel:"S4";  sdesc:"Scan Stack Pos1" }
                                        ListElement { sid:5;  slabel:"S5";  sdesc:"Output det." }
                                        ListElement { sid:6;  slabel:"S6";  sdesc:"Check Tray OutP1" }
                                        ListElement { sid:7;  slabel:"S7";  sdesc:"Khay tại Robot" }
                                        ListElement { sid:8;  slabel:"S8";  sdesc:"[Reserved]" }
                                        // S9-S16: Module 2
                                        ListElement { sid:9;  slabel:"S9";  sdesc:"Platform" }
                                        ListElement { sid:10; slabel:"S10"; sdesc:"Feed OK" }
                                        ListElement { sid:11; slabel:"S11"; sdesc:"Check Tray OutP2" }
                                        ListElement { sid:12; slabel:"S12"; sdesc:"Scan Stack Pos2" }
                                        ListElement { sid:13; slabel:"S13"; sdesc:"Cyl1 Ret"}
                                        ListElement { sid:14; slabel:"S14"; sdesc:"Cyl1 Ext" }
                                        ListElement { sid:15; slabel:"S15"; sdesc:"Cyl2 Ret"}
                                        ListElement { sid:16; slabel:"S16"; sdesc:"Cyl2 Ext" }
                                    }
                                    delegate: Rectangle {
                                        id: sBtn
                                        property bool on_: {
                                            var st = cartridgeController.sensorState;
                                            if (model.sid > 0 && model.sid <= st.length) {
                                                return st.charAt(model.sid - 1) === '1';
                                            }
                                            return false;
                                        }

                                        Layout.fillWidth: true
                                        Layout.fillHeight: true          // ← mỗi nút chiếm đều phần chiều cao
                                        Layout.minimumHeight: 30         // ← không nhỏ hơn 30px

                                        radius: 4
                                        color: on_ ? "#0a332e" : root.cCard
                                        border.color: on_ ? root.cGreen : root.cBorder
                                        Behavior on color       { ColorAnimation { duration: 150 } }
                                        Behavior on border.color { ColorAnimation { duration: 150 } }
                                        HoverHandler { onHoveredChanged: if(!sBtn.on_) sBtn.border.color = hovered ? root.cCyan : root.cBorder }
                                        MouseArea {
                                            anchors.fill: parent
                                            onClicked: {
                                                if (cartridgeController.currentMode === "auto") return;
                                                cartridgeController.simSensor(model.sid + ":" + (sBtn.on_ ? "0" : "1"))
                                            }
                                        }
                                        Column {
                                            anchors.centerIn: parent
                                            spacing: 1
                                            Text {
                                                text: model.slabel
                                                color: sBtn.on_ ? root.cGreen : root.cText
                                                font.pixelSize: 11; font.bold: true
                                                anchors.horizontalCenter: parent.horizontalCenter
                                            }
                                            Text {
                                                text: model.sdesc
                                                color: root.cDim; font.pixelSize: 8
                                                anchors.horizontalCenter: parent.horizontalCenter
                                                visible: model.sdesc !== ""
                                            }
                                            Rectangle {
                                                width: 6; height: 6; radius: 3
                                                color: sBtn.on_ ? root.cGreen : "#134357"
                                                anchors.horizontalCenter: parent.horizontalCenter
                                            }
                                        }
                                    }
                                }
                            }

                            // ── Chú thích ──
                            Text {
                                text: "<b>S1-S3</b> Conveyor · <b>S4</b> Scan P1 · <b>S5</b> Out Det · <b>S6</b> Check P1 · <b>S7</b> Robot P1/2 · <b>S8</b> RSV\n<b>S9</b> Platform · <b>S10</b> Feed OK · <b>S11</b> Check P2 · <b>S12</b> Scan P2\n<b>S13</b> Cyl1↩ · <b>S14</b> Cyl1↪ · <b>S15</b> Cyl2↩ · <b>S16</b> Cyl2↪"
                                textFormat: Text.RichText; color: root.cDim; font.pixelSize: 8
                                Layout.fillWidth: true; wrapMode: Text.WordWrap
                            }
                        }
                    }
                }
            } // Page 1

            // ── PAGE 2: TECHNICAL SYSTEM — horizontal 4-column layout ──
            Item {
                id: page2Root
                property var parsedConfig: ({})
                property int configRevision: 0
                property var p2FilteredLog: []

                function reloadConfig() {
                    try { page2Root.parsedConfig = JSON.parse(cartridgeController.configData) } catch(e) {}
                    page2Root.configRevision++
                }

                function refreshP2Log() {
                    var filtered = []
                    var entries = cartridgeController.logEntries
                    for (var i = 0; i < entries.length; i++) {
                        var msg = (entries[i].msg || "").toLowerCase()
                        if (msg.indexOf("config") !== -1 || msg.indexOf("updated") !== -1 || msg.indexOf("saved") !== -1)
                            filtered.push(entries[i])
                    }
                    page2Root.p2FilteredLog = filtered
                }

                Connections {
                    target: cartridgeController
                    function onConfigDataChanged() { page2Root.reloadConfig() }
                    function onLogEntriesChanged() { page2Root.refreshP2Log() }
                }
                Component.onCompleted: cartridgeController.getConfig()

                // ── Layout: cards on top, log bar at bottom ──
                ColumnLayout {
                    anchors { fill: parent; margins: 10 }
                    spacing: 8

                // ── 4 cards trải đều ngang ──
                RowLayout {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    spacing: 10

                    ConfigZoneCard {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        title: "Pos 1: Input Zones (InY)"
                        configKey: "iny_input_zones"
                        configSource: page2Root.parsedConfig
                    }
                    ConfigZoneCard {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        title: "Pos 1: Output Zones (InY)"
                        configKey: "iny_output_zones"
                        configSource: page2Root.parsedConfig
                    }
                    ConfigZoneCard {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        title: "Pos 2: Output Zones (OutY)"
                        configKey: "outy_output_zones"
                        configSource: page2Root.parsedConfig
                    }

                    // Card 4: Servo Key Positions
                    Rectangle {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        color: root.cBg2; border.color: root.cBorder; radius: 6
                        HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cAccent : root.cBorder }

                        property var servoParams: [
                            { key:"inx_home",        label:"InX Home",      desc:"S1 home" },
                            { key:"inx_target2",     label:"InX Target",    desc:"S1 lấy khay (500mm)" },
                            { key:"inx_output_stack",label:"InX OutPos",    desc:"Đặt khay output" },
                            { key:"iny_home",        label:"InY Home",      desc:"S2 home" },
                            { key:"iny_target2",     label:"InY Place",     desc:"Robot place (200mm)" },
                            { key:"iny_safe_zone",   label:"InY SafeZone",  desc:"Safe zone" },
                            { key:"servo3_target2",  label:"S3 Feed",       desc:"Feed pos (400mm)" },
                            { key:"outx_home",       label:"OutX Home",     desc:"S4 home" },
                            { key:"outx_target2",    label:"OutX Target2",  desc:"Lấy khay output" },
                            { key:"outx_target3",    label:"OutX Target3",  desc:"Đặt khay robot" },
                            { key:"outy_target1",    label:"OutY Target1",  desc:"Nâng khay (safe)" },
                            { key:"outy_pick_pos",   label:"OutY Pick",     desc:"Hạ gắp khay" },
                            { key:"target_scanoutp2",label:"OUTY TgtScan",  desc:"Điểm dừng quét S10" },
                            { key:"outy_scan_arm_mm",label:"OUTY Arm S10",  desc:"Giới hạn kích hoạt S10" }
                        ]

                        Column {
                            id: servoInfoCol2
                            anchors { fill: parent; margins: 8 }
                            spacing: 2

                            Text { text: "SERVO KEY POSITIONS (mm)"; color: root.cAccent; font.pixelSize: 15; font.bold: true; font.letterSpacing: 1.5 }
                            Row { width: parent.width
                                Repeater { model: ["Parameter","Value","Desc"]
                                    delegate: Text { text: modelData; color: root.cDim; font.pixelSize: 10; font.bold: true
                                        width: index===0?110:index===1?66:parent.width-176
                                        font.capitalization: Font.AllUppercase; font.letterSpacing: 1 } } }
                            Rectangle { width: parent.width; height: 1; color: root.cBorder }

                            Repeater {
                                id: servoRepeater2
                                model: parent.parent.servoParams
                                delegate: Rectangle {
                                    required property var modelData
                                    required property int index
                                    width: servoInfoCol2.width; height: 38
                                    color: index % 2 === 0 ? "transparent" : "#0d0d22"
                                    property alias inputText: sInput2.text
                                    property string paramKey: modelData.key
                                    Row {
                                        anchors.verticalCenter: parent.verticalCenter; width: parent.width
                                        Text { text: modelData.label; color: root.cCyan; font.pixelSize: 13; font.bold: true
                                               width: 120; anchors.verticalCenter: parent.verticalCenter }
                                        Rectangle { width: 94; height: 30; radius: 4; color: root.cBg; border.color: root.cBorder
                                            TextInput { id: sInput2; anchors { fill: parent; margins: 3 }
                                                text: "0.0"; font.pixelSize: 14; font.family: "monospace"
                                                color: root.cYellow; horizontalAlignment: TextInput.AlignHCenter
                                                validator: DoubleValidator { bottom: -9999; top: 9999; decimals: 2 }
                                                Connections {
                                                    function onConfigRevisionChanged() {
                                                        var v = page2Root.parsedConfig[modelData.key]
                                                        if (v !== undefined) sInput2.text = String(v)
                                                    }
                                                    target: page2Root
                                                }
                                            }
                                        }
                                        Item { width: 4 }
                                        Text { text: modelData.desc; color: root.cDim; font.pixelSize: 12
                                               elide: Text.ElideRight; anchors.verticalCenter: parent.verticalCenter
                                               width: parent.width - 120 - 94 - 4 }
                                    }
                                }
                            }

                            Row { spacing: 6; topPadding: 6
                                CBtn { lbl:"Save All"; padV:8; padH:18; fontSize:13; bg:"#0a332e"; bc:root.cGreen; tc:root.cGreen
                                    onClicked: {
                                        for (var i = 0; i < servoRepeater2.count; i++) {
                                            var item = servoRepeater2.itemAt(i)
                                            if (item && item.inputText !== "")
                                                cartridgeController.saveConfig(item.paramKey, item.inputText)
                                        }
                                    }
                                }
                                CBtn { lbl:"↺ Reset"; padV:8; padH:14; fontSize:13; bg:root.cCard; bc:root.cBorder; tc:root.cText; onClicked: page2Root.reloadConfig() }
                            }
                        }
                    }
                } // RowLayout

                    // ── Log bar ──────────────────────────────────────────
                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 120
                        color: root.cBg2; border.color: root.cBorder; radius: 6

                        Column {
                            anchors { fill: parent; margins: 8 }
                            spacing: 4
                            RowLayout {
                                width: parent.width; height: 18
                                Text { text: "CONFIG LOG"; color: root.cAccent; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.5 }
                                Item { Layout.fillWidth: true }
                                CBtn { lbl:"Clear"; padV:3; padH:8; fontSize:10; bg:root.cCard; bc:root.cBorder; tc:root.cDim; onClicked: cartridgeController.clearLog() }
                            }
                            Rectangle {
                                width: parent.width; height: parent.height - 22
                                color: "#0a0a18"; border.color: root.cBorder; radius: 4
                                ListView {
                                    anchors { fill: parent; margins: 6 }
                                    model: page2Root.p2FilteredLog
                                    clip: true; spacing: 2
                                    verticalLayoutDirection: ListView.BottomToTop
                                    delegate: Text {
                                        width: parent ? parent.width : 100
                                        text: "[" + modelData.time + "] " + modelData.msg
                                        font.pixelSize: 12; font.family: "monospace"
                                        color: modelData.type==="err" ? root.cRed : modelData.type==="ok" ? root.cGreen : root.cCyan
                                        wrapMode: Text.WordWrap
                                    }
                                }
                            }
                        }
                    }

                } // ColumnLayout
            } // Page 2

            // ── PAGE 3: ROBOT CONTROL ──────────────────────────────────
            Item {
                id: page3Root
                property string currentMode: cartridgeController.currentMode  // bind to system mode
                property bool manualEnabled: robotController.systemStatus === "IDLE" || robotController.systemStatus === "UNKNOWN" || robotController.systemStatus === ""
                property real stepValue: 1.0
                property int speedVal: robotController.speedRatio
                property bool rowLocked: false
                property int jogStep: 1               // 0.1, 1, 5, 10

                Item {
                    id: p3Inner
                    anchors { fill: parent; margins: 10 }

                    // ════════════════ MODE HEADER ══════════════════════
                    Rectangle {
                        id: modeToggle
                        anchors { top: parent.top; left: parent.left; right: parent.right }
                        height: 32; radius: 5
                        color: page3Root.manualEnabled ? "#0a1a35" : "#081e29"
                        border.color: page3Root.manualEnabled ? "#5cf4f1" : "#2a3a4a"; border.width: 2
                        Behavior on color { ColorAnimation { duration: 200 } }
                        Behavior on border.color { ColorAnimation { duration: 200 } }
                        Row {
                            anchors.centerIn: parent; spacing: 8
                            Text {
                                text: page3Root.manualEnabled ? "🔓" : "🔒"
                                font.pixelSize: 14; anchors.verticalCenter: parent.verticalCenter
                            }
                            Text {
                                text: page3Root.manualEnabled ? "MANUAL MODE" : "LOCKED — " + page3Root.currentMode.toUpperCase() + " MODE"
                                color: page3Root.manualEnabled ? "#5cf4f1" : "#888"
                                font.pixelSize: 12; font.bold: true; font.letterSpacing: 1
                                anchors.verticalCenter: parent.verticalCenter
                            }
                        }
                    }

                    // ════════════════ CONTENT AREA ════════════════════════
                    Item {
                        id: contentArea
                        anchors { top: modeToggle.bottom; topMargin: 8; left: parent.left; right: parent.right; bottom: robotLogBar.top; bottomMargin: 4 }


                        // ──────────── MANUAL MODE: JOG ────────────────────
                        Item {
                            anchors.fill: parent
                            enabled: page3Root.manualEnabled
                            opacity: page3Root.manualEnabled ? 1.0 : 0.35
                            Behavior on opacity { NumberAnimation { duration: 200 } }

                            property int colGap: 6

                            // ── 3 columns: Cartesian | Joint | IO+Controls ──
                            Row {
                                id: jogRow
                                anchors.fill: parent
                                spacing: parent.colGap

                                // ═══ CARTESIAN ═══
                                Rectangle {
                                    width: (parent.width - parent.parent.colGap * 2) * 0.38; height: parent.height
                                    color: root.cBg2; border.color: root.cBorder; radius: 6
                                    Column {
                                        id: cartCol
                                        anchors { fill: parent; margins: 6 }
                                        spacing: 3
                                        Row { spacing: 5
                                            Rectangle { width: 3; height: 11; radius: 1; color: root.cAccent; anchors.verticalCenter: parent.verticalCenter }
                                            Text { text: "CARTESIAN (mm)"; color: root.cAccent; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1 }
                                        }
                                        Repeater {
                                            id: cartRep
                                            model: [
                                                { axis: "X",  neg: "X-",  pos: "X+" },
                                                { axis: "Y",  neg: "Y-",  pos: "Y+" },
                                                { axis: "Z",  neg: "Z-",  pos: "Z+" },
                                                { axis: "RX", neg: "Rx-", pos: "Rx+" },
                                                { axis: "RY", neg: "Ry-", pos: "Ry+" },
                                                { axis: "RZ", neg: "Rz-", pos: "Rz+" }
                                            ]
                                            delegate: Row {
                                                required property var modelData
                                                required property int index
                                                width: cartCol.width; height: 44; spacing: 2
                                                Rectangle {
                                                    id: negBtn
                                                    width: 46; height: 40; radius: 5
                                                    color: negMA.pressed ? "#6a2222" : root.cCard
                                                    border.color: negMA.pressed ? root.cRed : root.cBorder; border.width: negMA.pressed ? 3 : 2
                                                    scale: negMA.pressed ? 0.92 : 1.0
                                                    Behavior on color { ColorAnimation { duration: 80 } }
                                                    Behavior on scale { NumberAnimation { duration: 60 } }
                                                    Behavior on border.color { ColorAnimation { duration: 80 } }
                                                    Text { anchors.centerIn: parent; text: modelData.axis + "-"; color: negMA.pressed ? Qt.lighter(root.cRed, 1.3) : root.cRed; font.pixelSize: 12; font.bold: true }
                                                    MouseArea { id: negMA; anchors.fill: parent; onPressed: robotController.jogStart(modelData.neg); onReleased: robotController.jogStop(); onCanceled: robotController.jogStop() }
                                                }
                                                Rectangle {
                                                    width: parent.width - 96; height: 40; radius: 5; color: "#0d1117"; border.width: 2; border.color: root.cBorder
                                                    Text { anchors.centerIn: parent; text: robotController.cartesianPose.length > index ? robotController.cartesianPose[index].toFixed(4) : "0.0000"; color: "#f59e0b"; font.pixelSize: 16; font.family: "monospace"; font.bold: true }
                                                }
                                                Rectangle {
                                                    id: posBtn
                                                    width: 46; height: 40; radius: 5
                                                    color: posMA.pressed ? "#1a4a2a" : root.cCard
                                                    border.color: posMA.pressed ? root.cGreen : root.cBorder; border.width: posMA.pressed ? 3 : 2
                                                    scale: posMA.pressed ? 0.92 : 1.0
                                                    Behavior on color { ColorAnimation { duration: 80 } }
                                                    Behavior on scale { NumberAnimation { duration: 60 } }
                                                    Behavior on border.color { ColorAnimation { duration: 80 } }
                                                    Text { anchors.centerIn: parent; text: modelData.axis + "+"; color: posMA.pressed ? Qt.lighter(root.cGreen, 1.3) : root.cGreen; font.pixelSize: 12; font.bold: true }
                                                    MouseArea { id: posMA; anchors.fill: parent; onPressed: robotController.jogStart(modelData.pos); onReleased: robotController.jogStop(); onCanceled: robotController.jogStop() }
                                                }
                                            }
                                        }
                                        Rectangle { width: parent.width; height: 1; color: root.cBorder }
                                        // Input fields for MovL
                                        Row { spacing: 3; width: parent.width
                                            Repeater {
                                                id: cartInputs
                                                model: ["X","Y","Z","RX","RY","RZ"]
                                                delegate: Column { spacing: 1; width: (cartCol.width - 15) / 6
                                                    Text { text: modelData; color: root.cDim; font.pixelSize: 10; anchors.horizontalCenter: parent.horizontalCenter }
                                                    Rectangle {
                                                        width: parent.width; height: 36; radius: 4; color: "#0d1117"; border.color: root.cAccent; border.width: 2
                                                        TextInput { id: cartInp
                                                            anchors { fill: parent; margins: 2 }
                                                            color: root.cText; font.pixelSize: 14; font.family: "monospace"; horizontalAlignment: Text.AlignHCenter; clip: true
                                                            text: robotController.cartesianPose.length > index ? robotController.cartesianPose[index].toFixed(1) : "0"
                                                            selectByMouse: true; verticalAlignment: Text.AlignVCenter; validator: DoubleValidator { notation: DoubleValidator.StandardNotation }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                        Row { spacing: 4; width: parent.width
                                            Rectangle {
                                                width: parent.width / 2 - 2; height: 34; radius: 5
                                                color: gpMA.pressed ? "#1a5a3a" : "#0a2a1a"; border.color: gpMA.pressed ? Qt.lighter(root.cCyan, 1.3) : root.cCyan
                                                border.width: gpMA.pressed ? 2 : 1; scale: gpMA.pressed ? 0.95 : 1.0
                                                Behavior on color { ColorAnimation { duration: 80 } }
                                                Behavior on scale { NumberAnimation { duration: 60 } }
                                                Text { anchors.centerIn: parent; text: "GET POSE"; color: gpMA.pressed ? Qt.lighter(root.cCyan, 1.4) : root.cCyan; font.pixelSize: 12; font.bold: true }
                                                MouseArea { id: gpMA; anchors.fill: parent; onClicked: {
                                                    robotController.getPose()
                                                    for (var i = 0; i < 6; i++) {
                                                        if (robotController.cartesianPose.length > i)
                                                            cartInputs.itemAt(i).children[1].children[0].text = robotController.cartesianPose[i].toFixed(2)
                                                    }
                                                }}
                                            }
                                            Rectangle {
                                                width: parent.width / 2 - 2; height: 34; radius: 5
                                                color: mlMA.pressed ? "#1a3a65" : "#0a1a35"; border.color: mlMA.pressed ? Qt.lighter(root.cAccent, 1.3) : root.cAccent
                                                border.width: mlMA.pressed ? 2 : 1; scale: mlMA.pressed ? 0.95 : 1.0
                                                Behavior on color { ColorAnimation { duration: 80 } }
                                                Behavior on scale { NumberAnimation { duration: 60 } }
                                                Text { anchors.centerIn: parent; text: "SEND MovL"; color: mlMA.pressed ? Qt.lighter(root.cAccent, 1.4) : root.cAccent; font.pixelSize: 12; font.bold: true }
                                                MouseArea { id: mlMA; anchors.fill: parent; onClicked: {
                                                    var vals = []
                                                    for (var i = 0; i < 6; i++) vals.push(parseFloat(cartInputs.itemAt(i).children[1].children[0].text) || 0)
                                                    robotController.moveLinear(vals[0],vals[1],vals[2],vals[3],vals[4],vals[5])
                                                }}
                                            }
                                        }
                                    }
                                }

                                // ═══ JOINT ═══
                                Rectangle {
                                    width: (parent.width - parent.parent.colGap * 2) * 0.38; height: parent.height
                                    color: root.cBg2; border.color: root.cBorder; radius: 6
                                    Column {
                                        id: jointCol
                                        anchors { fill: parent; margins: 6 }
                                        spacing: 3
                                        Row { spacing: 5
                                            Rectangle { width: 3; height: 11; radius: 1; color: "#5cf4f1"; anchors.verticalCenter: parent.verticalCenter }
                                            Text { text: "JOINT (deg)"; color: "#5cf4f1"; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1 }
                                        }
                                        Repeater {
                                            id: jointRep
                                            model: 6
                                            delegate: Row {
                                                property int jn: index + 1
                                                width: jointCol.width; height: 44; spacing: 2
                                                Rectangle {
                                                    width: 46; height: 40; radius: 5
                                                    color: jnMA.pressed ? "#6a2222" : root.cCard
                                                    border.color: jnMA.pressed ? root.cRed : root.cBorder; border.width: jnMA.pressed ? 3 : 2
                                                    scale: jnMA.pressed ? 0.92 : 1.0
                                                    Behavior on color { ColorAnimation { duration: 80 } }
                                                    Behavior on scale { NumberAnimation { duration: 60 } }
                                                    Behavior on border.color { ColorAnimation { duration: 80 } }
                                                    Text { anchors.centerIn: parent; text: "J" + jn + "-"; color: jnMA.pressed ? Qt.lighter(root.cRed, 1.3) : root.cRed; font.pixelSize: 12; font.bold: true }
                                                    MouseArea { id: jnMA; anchors.fill: parent; onPressed: robotController.jogStart("j" + jn + "-"); onReleased: robotController.jogStop(); onCanceled: robotController.jogStop() }
                                                }
                                                Rectangle {
                                                    width: parent.width - 96; height: 40; radius: 5; color: "#0d1117"; border.width: 2; border.color: root.cBorder
                                                    Text { anchors.centerIn: parent; text: robotController.jointAngles.length > index ? robotController.jointAngles[index].toFixed(4) : "0.0000"; color: "#f59e0b"; font.pixelSize: 16; font.family: "monospace"; font.bold: true }
                                                }
                                                Rectangle {
                                                    width: 46; height: 40; radius: 5
                                                    color: jpMA.pressed ? "#1a4a2a" : root.cCard
                                                    border.color: jpMA.pressed ? root.cGreen : root.cBorder; border.width: jpMA.pressed ? 3 : 2
                                                    scale: jpMA.pressed ? 0.92 : 1.0
                                                    Behavior on color { ColorAnimation { duration: 80 } }
                                                    Behavior on scale { NumberAnimation { duration: 60 } }
                                                    Behavior on border.color { ColorAnimation { duration: 80 } }
                                                    Text { anchors.centerIn: parent; text: "J" + jn + "+"; color: jpMA.pressed ? Qt.lighter(root.cGreen, 1.3) : root.cGreen; font.pixelSize: 12; font.bold: true }
                                                    MouseArea { id: jpMA; anchors.fill: parent; onPressed: robotController.jogStart("j" + jn + "+"); onReleased: robotController.jogStop(); onCanceled: robotController.jogStop() }
                                                }
                                            }
                                        }
                                        Rectangle { width: parent.width; height: 1; color: root.cBorder }
                                        Row { spacing: 3; width: parent.width
                                            Repeater {
                                                id: jointInputs
                                                model: ["J1","J2","J3","J4","J5","J6"]
                                                delegate: Column { spacing: 1; width: (jointCol.width - 15) / 6
                                                    Text { text: modelData; color: root.cDim; font.pixelSize: 10; anchors.horizontalCenter: parent.horizontalCenter }
                                                    Rectangle {
                                                        width: parent.width; height: 36; radius: 4; color: "#0d1117"; border.color: "#5cf4f1"; border.width: 2
                                                        TextInput {
                                                            anchors { fill: parent; margins: 2 }
                                                            color: root.cText; font.pixelSize: 14; font.family: "monospace"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; clip: true
                                                            text: robotController.jointAngles.length > index ? robotController.jointAngles[index].toFixed(1) : "0"
                                                            selectByMouse: true; validator: DoubleValidator { notation: DoubleValidator.StandardNotation }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                        Row { spacing: 4; width: parent.width
                                            Rectangle {
                                                width: parent.width / 2 - 2; height: 34; radius: 5
                                                color: gaMA.pressed ? "#1a5a3a" : "#0a2a1a"; border.color: gaMA.pressed ? Qt.lighter(root.cCyan, 1.3) : root.cCyan
                                                border.width: gaMA.pressed ? 2 : 1; scale: gaMA.pressed ? 0.95 : 1.0
                                                Behavior on color { ColorAnimation { duration: 80 } }
                                                Behavior on scale { NumberAnimation { duration: 60 } }
                                                Text { anchors.centerIn: parent; text: "GET ANGLES"; color: gaMA.pressed ? Qt.lighter(root.cCyan, 1.4) : root.cCyan; font.pixelSize: 12; font.bold: true }
                                                MouseArea { id: gaMA; anchors.fill: parent; onClicked: {
                                                    robotController.getAngles()
                                                    for (var i = 0; i < 6; i++) {
                                                        if (robotController.jointAngles.length > i)
                                                            jointInputs.itemAt(i).children[1].children[0].text = robotController.jointAngles[i].toFixed(2)
                                                    }
                                                }}
                                            }
                                            Rectangle {
                                                width: parent.width / 2 - 2; height: 34; radius: 5
                                                color: mjMA.pressed ? "#082f3a" : "#051a25"; border.color: mjMA.pressed ? Qt.lighter("#5cf4f1", 1.3) : "#5cf4f1"
                                                border.width: mjMA.pressed ? 2 : 1; scale: mjMA.pressed ? 0.95 : 1.0
                                                Behavior on color { ColorAnimation { duration: 80 } }
                                                Behavior on scale { NumberAnimation { duration: 60 } }
                                                Text { anchors.centerIn: parent; text: "SEND MovJ"; color: mjMA.pressed ? Qt.lighter("#5cf4f1", 1.3) : "#5cf4f1"; font.pixelSize: 12; font.bold: true }
                                                MouseArea { id: mjMA; anchors.fill: parent; onClicked: {
                                                    var vals = []
                                                    for (var i = 0; i < 6; i++) vals.push(parseFloat(jointInputs.itemAt(i).children[1].children[0].text) || 0)
                                                    robotController.moveJoint(vals[0],vals[1],vals[2],vals[3],vals[4],vals[5])
                                                }}
                                            }
                                        }

                                        // ── SAVE TO YAML ─────────────────────
                                        Rectangle { width: parent.width; height: 1; color: root.cBorder; opacity: 0.5 }
                                        Text { text: "SAVE POSE TO YAML"; color: root.cDim; font.pixelSize: 8; font.bold: true; font.letterSpacing: 0.8 }

                                        Row { spacing: 4; width: parent.width
                                            // Name / comment input
                                            Rectangle {
                                                id: poseNameRect
                                                width: parent.width - 70 - 4; height: 32; radius: 4
                                                color: "#0d1117"; border.color: "#5cf4f1"; border.width: 1
                                                // Placeholder hint
                                                Text {
                                                    anchors { fill: parent; leftMargin: 6; verticalCenter: parent.verticalCenter }
                                                    text: poseNameInput.text.length === 0 ? "pose name / comment..." : ""
                                                    color: "#555"; font.pixelSize: 12; font.family: "monospace"
                                                    verticalAlignment: Text.AlignVCenter
                                                }
                                                TextInput {
                                                    id: poseNameInput
                                                    anchors { fill: parent; leftMargin: 6; rightMargin: 4; topMargin: 4; bottomMargin: 4 }
                                                    color: root.cText; font.pixelSize: 12; font.family: "monospace"
                                                    clip: true; selectByMouse: true
                                                }
                                            }
                                            // SAVE button
                                            Rectangle {
                                                id: savePoseBtn
                                                width: 70; height: 32; radius: 4
                                                property bool saving: false
                                                color: saving ? "#1a4d00" : (saveMA.pressed ? "#2a6a00" : "#0a3a00")
                                                border.color: savePoseBtn.saving ? root.cGreen : (saveMA.pressed ? Qt.lighter(root.cGreen, 1.3) : "#4a8a00")
                                                border.width: saveMA.pressed ? 2 : 1
                                                Behavior on color { ColorAnimation { duration: 100 } }
                                                Text {
                                                    anchors.centerIn: parent
                                                    text: savePoseBtn.saving ? "✓ SAVED" : "💾 SAVE"
                                                    color: savePoseBtn.saving ? root.cGreen : "#88cc44"
                                                    font.pixelSize: 11; font.bold: true
                                                }
                                                MouseArea {
                                                    id: saveMA; anchors.fill: parent
                                                    onClicked: {
                                                        var vals = []
                                                        for (var i = 0; i < 6; i++)
                                                            vals.push(parseFloat(jointInputs.itemAt(i).children[1].children[0].text) || 0)
                                                        robotController.saveJointPose(
                                                            poseNameInput.text,
                                                            vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]
                                                        )
                                                    }
                                                }
                                                Connections {
                                                    target: robotController
                                                    function onJointPoseSaved(success, message) {
                                                        savePoseBtn.saving = success
                                                        saveStatusText.text = message
                                                        saveStatusText.color = success ? root.cGreen : root.cRed
                                                        saveStatusTimer.restart()
                                                    }
                                                }
                                                Timer {
                                                    id: saveStatusTimer; interval: 3000
                                                    onTriggered: { savePoseBtn.saving = false; saveStatusText.text = "" }
                                                }
                                            }
                                        }
                                        // Status toast
                                        Text {
                                            id: saveStatusText
                                            width: parent.width; wrapMode: Text.WordWrap
                                            text: ""; font.pixelSize: 10; font.family: "monospace"
                                            color: root.cGreen
                                        }

                                    }
                                }

                                // ═══ IO + CONTROLS ═══
                                Rectangle {
                                    width: parent.width - (parent.width - parent.parent.colGap * 2) * 0.76 - parent.parent.colGap * 2; height: parent.height
                                    color: root.cBg2; border.color: root.cBorder; radius: 6
                                    Column {
                                        id: ioCol
                                        anchors { fill: parent; margins: 6 }
                                        spacing: 5
                                        Row { spacing: 5
                                            Rectangle { width: 3; height: 11; radius: 1; color: root.cCyan; anchors.verticalCenter: parent.verticalCenter }
                                            Text { text: "I/O CONTROL"; color: root.cCyan; font.pixelSize: 10; font.bold: true; font.letterSpacing: 1 }
                                        }

                                        // Step Value
                                        Text { text: "STEP VALUE"; color: root.cDim; font.pixelSize: 9; font.bold: true }
                                        Row { spacing: 3; width: parent.width
                                            Repeater {
                                                model: [0.1, 1, 5, 10]
                                                delegate: Rectangle {
                                                    required property var modelData
                                                    width: (ioCol.width - 9) / 4; height: 28; radius: 4
                                                    color: page3Root.stepValue === modelData ? root.cAccent : root.cCard
                                                    border.color: page3Root.stepValue === modelData ? root.cAccent : root.cBorder; border.width: 2
                                                    Text { anchors.centerIn: parent; text: modelData; color: page3Root.stepValue === modelData ? "#000" : root.cText; font.pixelSize: 11; font.bold: true }
                                                    MouseArea { anchors.fill: parent; onClicked: page3Root.stepValue = modelData }
                                                }
                                            }
                                        }

                                        // Speed Ratio
                                        Text { text: "SPEED %"; color: root.cDim; font.pixelSize: 9; font.bold: true }
                                        Row { spacing: 4; width: parent.width; height: 30
                                            Slider {
                                                id: speedSlider
                                                width: parent.width - 50; height: 28
                                                from: 1; to: 100; stepSize: 1; value: page3Root.speedVal
                                                onMoved: { page3Root.speedVal = Math.round(value) }
                                                onPressedChanged: { if (!pressed) robotController.setSpeedRatio(Math.round(value)) }
                                                background: Rectangle { x: speedSlider.leftPadding; y: speedSlider.topPadding + speedSlider.availableHeight / 2 - 4; width: speedSlider.availableWidth; height: 8; radius: 4; color: root.cCard; border.color: root.cBorder
                                                    Rectangle { width: speedSlider.visualPosition * parent.width; height: parent.height; radius: 4; color: "#5cf4f1" }
                                                }
                                                handle: Rectangle { x: speedSlider.leftPadding + speedSlider.visualPosition * (speedSlider.availableWidth - width); y: speedSlider.topPadding + speedSlider.availableHeight / 2 - 8; width: 16; height: 16; radius: 8; color: "#5cf4f1"; border.color: "#fff" }
                                            }
                                            Rectangle {
                                                width: 44; height: 28; radius: 4; color: "#0d1117"; border.color: "#5cf4f1"; border.width: 2
                                                TextInput { anchors.centerIn: parent; width: 38; color: root.cText; font.pixelSize: 12; font.family: "monospace"; font.bold: true; horizontalAlignment: Text.AlignHCenter
                                                    text: page3Root.speedVal
                                                    validator: IntValidator { bottom: 1; top: 100 }
                                                    selectByMouse: true
                                                    onEditingFinished: { var v = Math.max(1, Math.min(100, parseInt(text) || 100)); page3Root.speedVal = v; speedSlider.value = v; robotController.setSpeedRatio(v) }
                                                }
                                            }
                                        }

                                        Rectangle { width: parent.width; height: 1; color: root.cBorder }

                                        // Gripper DO1
                                        Text { text: "GRIPPER (DO1)"; color: root.cDim; font.pixelSize: 9; font.bold: true }
                                        Row { spacing: 4; width: parent.width
                                            Rectangle {
                                                width: (parent.width - 4) / 2; height: 34; radius: 4
                                                color: "#0a332e"; border.color: root.cGreen
                                                Text { anchors.centerIn: parent; text: "ON"; color: root.cGreen; font.pixelSize: 12; font.bold: true }
                                                MouseArea { anchors.fill: parent; onClicked: robotController.setDigitalOutput(1, true) }
                                            }
                                            Rectangle {
                                                width: (parent.width - 4) / 2; height: 34; radius: 4
                                                color: root.cCard; border.color: root.cBorder
                                                Text { anchors.centerIn: parent; text: "OFF"; color: root.cDim; font.pixelSize: 12; font.bold: true }
                                                MouseArea { anchors.fill: parent; onClicked: robotController.setDigitalOutput(1, false) }
                                            }
                                        }

                                        // Picker DO2
                                        Text { text: "PICKER (DO2)"; color: root.cDim; font.pixelSize: 9; font.bold: true }
                                        Row { spacing: 4; width: parent.width
                                            Rectangle {
                                                width: (parent.width - 4) / 2; height: 34; radius: 4
                                                color: "#0a332e"; border.color: root.cGreen
                                                Text { anchors.centerIn: parent; text: "ON"; color: root.cGreen; font.pixelSize: 12; font.bold: true }
                                                MouseArea { anchors.fill: parent; onClicked: robotController.setDigitalOutput(2, true) }
                                            }
                                            Rectangle {
                                                width: (parent.width - 4) / 2; height: 34; radius: 4
                                                color: root.cCard; border.color: root.cBorder
                                                Text { anchors.centerIn: parent; text: "OFF"; color: root.cDim; font.pixelSize: 12; font.bold: true }
                                                MouseArea { anchors.fill: parent; onClicked: robotController.setDigitalOutput(2, false) }
                                            }
                                        }

                                        Rectangle { width: parent.width; height: 1; color: root.cBorder }

                                        // Stop & Reset → IDLE
                                        CBtn { lbl: "⏹ STOP"; width: parent.width; bg: "#4a1a00"; bc: "#FF6600"; tc: "#FF6600"; padV: 10; onClicked: { robotController.stopAndResetRobot(); cartridgeController.stopSystem() } }

                                        // Enable
                                        CBtn { lbl: "ENABLE"; width: parent.width; bg: "#0a332e"; bc: root.cGreen; tc: root.cGreen; padV: 8; onClicked: robotController.enableSystem(true) }

                                        // Pause / Resume
                                        Row { spacing: 4; width: parent.width
                                            Rectangle {
                                                width: (parent.width - 4) / 2; height: 34; radius: 4
                                                color: pauseMA.pressed ? "#3a3a00" : "#1a1a00"; border.color: pauseMA.pressed ? Qt.lighter(root.cOrange, 1.3) : root.cOrange
                                                border.width: pauseMA.pressed ? 2 : 1; scale: pauseMA.pressed ? 0.95 : 1.0
                                                Behavior on color { ColorAnimation { duration: 80 } }
                                                Behavior on scale { NumberAnimation { duration: 60 } }
                                                Text { anchors.centerIn: parent; text: "PAUSE"; color: pauseMA.pressed ? Qt.lighter(root.cOrange, 1.3) : root.cOrange; font.pixelSize: 12; font.bold: true }
                                                MouseArea { id: pauseMA; anchors.fill: parent; onClicked: robotController.pauseRobot() }
                                            }
                                            Rectangle {
                                                width: (parent.width - 4) / 2; height: 34; radius: 4
                                                color: resumeMA.pressed ? "#1a5a3a" : "#0a2a1a"; border.color: resumeMA.pressed ? Qt.lighter(root.cCyan, 1.3) : root.cCyan
                                                border.width: resumeMA.pressed ? 2 : 1; scale: resumeMA.pressed ? 0.95 : 1.0
                                                Behavior on color { ColorAnimation { duration: 80 } }
                                                Behavior on scale { NumberAnimation { duration: 60 } }
                                                Text { anchors.centerIn: parent; text: "RESUME"; color: resumeMA.pressed ? Qt.lighter(root.cCyan, 1.3) : root.cCyan; font.pixelSize: 12; font.bold: true }
                                                MouseArea { id: resumeMA; anchors.fill: parent; onClicked: robotController.resumeRobot() }
                                            }
                                        }

                                        // Clear Error
                                        Rectangle {
                                            width: parent.width; height: 34; radius: 4
                                            color: ceMA.pressed ? "#1a3a5a" : "#0a1a2a"; border.color: ceMA.pressed ? Qt.lighter("#4da6ff", 1.3) : "#4da6ff"
                                            border.width: ceMA.pressed ? 2 : 1; scale: ceMA.pressed ? 0.95 : 1.0
                                            Behavior on color { ColorAnimation { duration: 80 } }
                                            Behavior on scale { NumberAnimation { duration: 60 } }
                                            Text { anchors.centerIn: parent; text: "CLEAR ERROR"; color: ceMA.pressed ? Qt.lighter("#4da6ff", 1.3) : "#4da6ff"; font.pixelSize: 12; font.bold: true }
                                            MouseArea { id: ceMA; anchors.fill: parent; onClicked: robotController.clearError() }
                                        }

                                        // E-STOP (biggest button)
                                        Rectangle {
                                            width: parent.width; height: 50; radius: 5
                                            color: esMA.pressed ? "#8a2222" : "#4d1a1a"
                                            border.color: esMA.pressed ? "#ff4444" : root.cRed; border.width: esMA.pressed ? 4 : 2
                                            scale: esMA.pressed ? 0.93 : 1.0
                                            Behavior on color { ColorAnimation { duration: 60 } }
                                            Behavior on scale { NumberAnimation { duration: 50 } }
                                            Behavior on border.color { ColorAnimation { duration: 60 } }
                                            Text { anchors.centerIn: parent; text: "EMERGENCY\nSTOP"; color: esMA.pressed ? "#ff6666" : root.cRed; font.pixelSize: 14; font.bold: true; horizontalAlignment: Text.AlignHCenter }
                                            MouseArea { id: esMA; anchors.fill: parent; onClicked: { robotController.emergencyStop(true); cartridgeController.stopSystem() } }
                                        }
                                    }
                                }
                            }
                        } // manual mode

                        // ════════════ AUTOMATED MODE OVERLAY ════════════
                        Item {
                            visible: !page3Root.manualEnabled
                            anchors.fill: parent; z: 100

                            // Dim background & block clicks to manual elements
                            Rectangle {
                                anchors.fill: parent; color: "#000000"; opacity: 0.65; radius: 6
                                MouseArea { anchors.fill: parent; hoverEnabled: true } 
                            }

                            Column {
                                anchors.centerIn: parent; spacing: 20
                                
                                Text {
                                    text: "🔒 MANUAL CONTROL DISABLED\nChuyển sang MANUAL mode để JOG robot"
                                    color: "#aaa"; font.pixelSize: 16; font.bold: true
                                    horizontalAlignment: Text.AlignHCenter; lineHeight: 1.5
                                    anchors.horizontalCenter: parent.horizontalCenter
                                }

                                Rectangle { width: 300; height: 1; color: "#444"; anchors.horizontalCenter: parent.horizontalCenter }

                                Text {
                                    text: "AUTO MODE ACTIONS"
                                    color: "#5cf4f1"; font.pixelSize: 14; font.bold: true; font.letterSpacing: 2
                                    anchors.horizontalCenter: parent.horizontalCenter
                                }

                                Column {
                                    spacing: 15; anchors.horizontalCenter: parent.horizontalCenter
                                    
                                    // ── ROW 1: TRAY READY SIGNALS ──
                                    Row {
                                        spacing: 15; anchors.horizontalCenter: parent.horizontalCenter
                                        
                                        // TRAY INPUT READY
                                        Rectangle {
                                            width: 160; height: 50; radius: 8
                                            color: robotController.inReady ? "#1a5a3a" : (tiMA.pressed ? "#5a3a1a" : "#351a0a")
                                            border.color: robotController.inReady ? "#00ff00" : (tiMA.pressed ? Qt.lighter("#ffaa4f", 1.2) : "#ffaa4f")
                                            border.width: tiMA.pressed ? 3 : 2
                                            scale: tiMA.pressed ? 0.95 : 1.0
                                            Behavior on color { ColorAnimation { duration: 100 } }
                                            Behavior on scale { NumberAnimation { duration: 100 } }
                                            
                                            Column {
                                                anchors.centerIn: parent; spacing: 2
                                                Text { text: "📥 TRAY INPUT READY"; color: "#fff"; font.pixelSize: 12; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                                Text { text: robotController.inReady ? "(ON)" : "(OFF)"; color: robotController.inReady ? "#00ff00" : "#d29252"; font.pixelSize: 10; anchors.horizontalCenter: parent.horizontalCenter }
                                            }
                                            MouseArea {
                                                id: tiMA; anchors.fill: parent
                                                onClicked: robotController.simulateInputTrayReady()
                                            }
                                        }

                                        // TRAY OUTPUT READY
                                        Rectangle {
                                            width: 160; height: 50; radius: 8
                                            color: robotController.outReady ? "#1a5a3a" : (toMA.pressed ? "#5a3a1a" : "#351a0a")
                                            border.color: robotController.outReady ? "#00ff00" : (toMA.pressed ? Qt.lighter("#ffaa4f", 1.2) : "#ffaa4f")
                                            border.width: toMA.pressed ? 3 : 2
                                            scale: toMA.pressed ? 0.95 : 1.0
                                            Behavior on color { ColorAnimation { duration: 100 } }
                                            Behavior on scale { NumberAnimation { duration: 100 } }
                                            
                                            Column {
                                                anchors.centerIn: parent; spacing: 2
                                                Text { text: "📤 TRAY OUTPUT READY"; color: "#fff"; font.pixelSize: 12; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                                Text { text: robotController.outReady ? "(ON)" : "(OFF)"; color: robotController.outReady ? "#00ff00" : "#d29252"; font.pixelSize: 10; anchors.horizontalCenter: parent.horizontalCenter }
                                            }
                                            MouseArea {
                                                id: toMA; anchors.fill: parent
                                                onClicked: robotController.simulateOutputTrayReady()
                                            }
                                        }
                                    }

                                    // ── ROW 2: ACTION TRIGGERS ──
                                    Row {
                                        spacing: 15; anchors.horizontalCenter: parent.horizontalCenter
                                        
                                        // PICK INPUT
                                        Rectangle {
                                            width: 160; height: 50; radius: 8
                                            color: piMA.pressed ? "#1a3a5a" : "#0a1a35"
                                            border.color: piMA.pressed ? Qt.lighter("#4f6cff", 1.2) : "#4f6cff"
                                            border.width: piMA.pressed ? 3 : 2
                                            scale: piMA.pressed ? 0.95 : 1.0
                                            Behavior on color { ColorAnimation { duration: 100 } }
                                            Behavior on scale { NumberAnimation { duration: 100 } }
                                            
                                            Column {
                                                anchors.centerIn: parent; spacing: 2
                                                Text { text: "↓ PICK INPUT"; color: "#fff"; font.pixelSize: 12; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                                Text { text: "(Load Khay -> Chamber)"; color: "#888"; font.pixelSize: 10; anchors.horizontalCenter: parent.horizontalCenter }
                                            }
                                            MouseArea {
                                                id: piMA; anchors.fill: parent
                                                onClicked: {
                                                    page3Root.rowLocked = true;
                                                    robotController.simulateFeedChamber();
                                                }
                                            }
                                        }

                                        // PICK CHAMBER
                                        Rectangle {
                                            width: 160; height: 50; radius: 8
                                            color: pcMA.pressed ? "#1a3a5a" : "#0a1a35"
                                            border.color: pcMA.pressed ? Qt.lighter("#4f6cff", 1.2) : "#4f6cff"
                                            border.width: pcMA.pressed ? 3 : 2
                                            scale: pcMA.pressed ? 0.95 : 1.0
                                            Behavior on color { ColorAnimation { duration: 100 } }
                                            Behavior on scale { NumberAnimation { duration: 100 } }
                                            
                                            Column {
                                                anchors.centerIn: parent; spacing: 2
                                                Text { text: "⟳ PICK CHAMBER"; color: "#fff"; font.pixelSize: 12; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                                Text { text: "(Chamber -> Output)"; color: "#888"; font.pixelSize: 10; anchors.horizontalCenter: parent.horizontalCenter }
                                            }
                                            MouseArea {
                                                id: pcMA; anchors.fill: parent
                                                onClicked: robotController.simulateFillDone()
                                            }
                                        }
                                    }


                                }
                            }
                        }
                    } // contentArea

                    // ════════════════ ROBOT LOG BAR ══════════════════════
                    Rectangle {
                        id: robotLogBar
                        anchors { left: parent.left; right: parent.right; bottom: parent.bottom }
                        height: 160; radius: 6; color: root.cBg2; border.color: root.cBorder
                        Column {
                            anchors { fill: parent; margins: 8 }
                            spacing: 4
                            RowLayout {
                                width: parent.width; height: 18
                                Text { text: "ROBOT LOG"; color: root.cAccent; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.5 }
                                Item { Layout.fillWidth: true }
                                CBtn { lbl:"Clear"; padV:3; padH:8; fontSize:10; bg:root.cCard; bc:root.cBorder; tc:root.cDim; onClicked: robotController.clearLog() }
                            }
                            Rectangle {
                                width: parent.width; height: parent.height - 22
                                color: "#0a0a18"; border.color: root.cBorder; radius: 4
                                ListView {
                                    anchors { fill: parent; margins: 6 }
                                    model: robotController.logEntries
                                    clip: true; spacing: 2
                                    verticalLayoutDirection: ListView.BottomToTop
                                    delegate: Text {
                                        width: parent ? parent.width : 100
                                        text: "[" + modelData.time + "] " + modelData.msg
                                        font.pixelSize: 12; font.family: "monospace"
                                        color: modelData.type==="err" ? root.cRed : modelData.type==="ok" ? root.cGreen : root.cCyan
                                        wrapMode: Text.WordWrap
                                    }
                                }
                            }
                        }
                    }

                } // p3Inner
            } // Page 3

        } // StackLayout

        // ════════════════════════════════════════════════════════════
        // REUSABLE: CBtn — matches HTML .btn
        // ════════════════════════════════════════════════════════════
        component CBtn: Rectangle {
            id: cbr
            property string lbl: ""
            property color bg:   root.cCard
            property color bc:   root.cBorder
            property color tc:   root.cText
            property bool  active: true
            property int   padV: 6
            property int   padH: 12
            property int   fontSize: 12
            property int   w: 0
            property bool  _pressed: false
            property bool  _hovered: false
            property bool  isSelected: false

            signal clicked(); signal pressed(); signal released()

            width:  w > 0 ? w : cbrT.width + padH * 2
            height: cbrT.height + padV * 2
            radius: 4
            color: {
                if (!active) return bg
                if (_pressed) return Qt.lighter(bg, 1.6)
                if (isSelected) return bc
                if (_hovered) return Qt.lighter(bg, 1.2)
                return bg
            }
            border.color: {
                if (_pressed) return Qt.lighter(bc, 1.5)
                if (isSelected) return Qt.lighter(bc, 1.2)
                if (_hovered) return (bc === root.cBorder ? root.cAccent : Qt.lighter(bc, 1.3))
                return bc
            }
            border.width: _pressed ? 2 : 1
            opacity: active ? 1.0 : 0.4
            scale: isSelected ? 0.96 : (_pressed ? 0.93 : 1.0)

            Behavior on color        { ColorAnimation { duration: 100 } }
            Behavior on border.color { ColorAnimation { duration: 100 } }
            Behavior on scale        { NumberAnimation { duration: 60; easing.type: Easing.OutQuad } }
            Behavior on opacity      { NumberAnimation { duration: 150 } }

            // Glow effect when pressed
            Rectangle {
                anchors.fill: parent; anchors.margins: -2
                radius: parent.radius + 2; color: "transparent"
                border.color: cbr._pressed ? Qt.rgba(cbr.bc.r, cbr.bc.g, cbr.bc.b, 0.4) : "transparent"
                border.width: 3; z: -1
                Behavior on border.color { ColorAnimation { duration: 100 } }
            }

            // Inner shadow to simulate physically sunken pressed state
            Rectangle {
                anchors.fill: parent
                radius: parent.radius
                color: "transparent"
                border.color: "black"
                border.width: cbr.isSelected ? 4 : 0
                opacity: 0.6
                visible: cbr.isSelected
            }

            Text { id: cbrT; anchors.centerIn: parent; text: cbr.lbl; color: cbr.isSelected ? "#0c0c1d" : (cbr._pressed ? Qt.lighter(cbr.tc, 1.4) : cbr.tc)
                font.pixelSize: cbr.fontSize; font.bold: true; font.capitalization: Font.AllUppercase
                anchors.verticalCenterOffset: (cbr.isSelected || cbr._pressed) ? 2 : 0
                Behavior on color { ColorAnimation { duration: 80 } }
            }

            MouseArea { anchors.fill: parent; hoverEnabled: true
                onClicked:       { if(cbr.active) cbr.clicked() }
                onPressed:       { cbr._pressed = true;  cbr.pressed() }
                onReleased:      { cbr._pressed = false; cbr.released() }
                onEntered:       cbr._hovered = true
                onExited:        { cbr._hovered = false; cbr._pressed = false }
                onCanceled:      { cbr._hovered = false; cbr._pressed = false }
            }
        }

        // ════════════════════════════════════════════════════════════
        // REUSABLE: ConfigCard — compact row position table
        // ════════════════════════════════════════════════════════════
        component ConfigCard: Rectangle {
            id: cfgCard
            property string title: ""
            property string configKey: ""
            property var configSource: ({})

            // height fill từ Layout parent
            color: root.cBg2; border.color: root.cBorder; radius: 6
            HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cAccent : root.cBorder }

            Column {
                id: cfgCol
                anchors { fill: parent; margins: 8 }
                spacing: 2

                Text { text: cfgCard.title; color: root.cAccent; font.pixelSize: 15; font.bold: true; font.letterSpacing: 1.5 }

                Row { width: parent.width
                    Repeater { model: ["Row","Position (mm)","Mô tả"]
                        delegate: Text { text: modelData; color: root.cDim; font.pixelSize: 11; font.bold: true
                            width: index===0?46:index===1?94:parent.width-140
                            font.capitalization: Font.AllUppercase; font.letterSpacing: 1 } }
                }
                Rectangle { width: parent.width; height: 1; color: root.cBorder }

                Repeater {
                    id: cfgRepeater
                    model: [10,9,8,7,6,5,4,3,2,1]
                    delegate: Rectangle {
                        required property int modelData
                        required property int index
                        width: cfgCol.width; height: 38
                        color: index % 2 === 0 ? "transparent" : "#0d0d22"
                        property alias inputText: rowInput.text
                        property int rowNum: modelData

                        Row {
                            anchors.verticalCenter: parent.verticalCenter; spacing: 0
                            Text { text: "R"+modelData; color: root.cCyan; font.pixelSize: 13; font.bold: true
                                   width: 46; anchors.verticalCenter: parent.verticalCenter }
                            Rectangle { width: 94; height: 30; radius: 4; color: root.cBg; border.color: root.cBorder
                                TextInput { id: rowInput; anchors { fill: parent; margins: 4 }
                                    text: "0.0"
                                    font.pixelSize: 14; font.family: "monospace"; color: root.cYellow
                                    horizontalAlignment: TextInput.AlignHCenter
                                    validator: DoubleValidator { bottom: -9999; top: 9999; decimals: 2 }
                                    Connections {
                                        function onConfigRevisionChanged() {
                                            var tbl = page2Root.parsedConfig[cfgCard.configKey]
                                            if (tbl && tbl[String(modelData)] !== undefined)
                                                rowInput.text = String(tbl[String(modelData)])
                                        }
                                        target: page2Root
                                    }
                                }
                            }
                            Item { width: 4 }
                            Text {
                                text: modelData===10?"Top":modelData===1?"Bot":""
                                color: root.cDim; font.pixelSize: 12; anchors.verticalCenter: parent.verticalCenter
                            }
                        }
                        Rectangle { width: parent.width; height: 1; color: "#1e1e3a"; anchors.bottom: parent.bottom }
                    }
                }

                Row { spacing: 6; topPadding: 8
                    CBtn { lbl:"Save"; padV:8; padH:18; fontSize:13; bg:"#0a332e"; bc:root.cGreen; tc:root.cGreen
                        onClicked: {
                            var positions = {}
                            for (var i = 0; i < cfgRepeater.count; i++) {
                                var item = cfgRepeater.itemAt(i)
                                if (item) positions[String(item.rowNum)] = parseFloat(item.inputText) || 0.0
                            }
                            cartridgeController.saveConfig(cfgCard.configKey, JSON.stringify(positions))
                        }
                    }
                    CBtn { lbl:"↺ Reset"; padV:8; padH:14; fontSize:13; bg:root.cCard; bc:root.cBorder; tc:root.cText
                        onClicked: page2Root.reloadConfig()
                    }
                }
            }
        }

        // ════════════════════════════════════════════════════════════
        // REUSABLE: ConfigZoneCard — compact min/max/target table
        // ════════════════════════════════════════════════════════════
        component ConfigZoneCard: Rectangle {
            id: cfgZoneCard
            property string title: ""
            property string configKey: ""
            property var configSource: ({})

            color: root.cBg2; border.color: root.cBorder; radius: 6
            HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cAccent : root.cBorder }

            Column {
                id: cfgZoneCol
                anchors { fill: parent; margins: 8 }
                spacing: 2

                Text { text: cfgZoneCard.title; color: root.cAccent; font.pixelSize: 13; font.bold: true; font.letterSpacing: 1.5 }

                Row { width: parent.width; spacing: 0
                    Text { text: "Row"; color: root.cDim; font.pixelSize: 10; font.bold: true; width: 36; font.capitalization: Font.AllUppercase }
                    Text { text: "Min"; color: root.cDim; font.pixelSize: 10; font.bold: true; width: 50; font.capitalization: Font.AllUppercase }
                    Text { text: "Max"; color: root.cDim; font.pixelSize: 10; font.bold: true; width: 50; font.capitalization: Font.AllUppercase }
                    Text { text: "Target"; color: root.cDim; font.pixelSize: 10; font.bold: true; width: 62; font.capitalization: Font.AllUppercase }
                }
                Rectangle { width: parent.width; height: 1; color: root.cBorder }

                Repeater {
                    id: cfgZoneRepeater
                    model: [10,9,8,7,6,5,4,3,2,1]
                    delegate: Rectangle {
                        required property int modelData
                        required property int index
                        width: cfgZoneCol.width; height: 38
                        color: index % 2 === 0 ? "transparent" : "#0d0d22"
                        property alias minText: minInp.text
                        property alias maxText: maxInp.text
                        property alias tgtText: tgtInp.text
                        property int rowNum: modelData

                        Row {
                            anchors.verticalCenter: parent.verticalCenter; spacing: 2
                            Text { text: "R"+modelData; color: root.cCyan; font.pixelSize: 12; font.bold: true; width: 34; anchors.verticalCenter: parent.verticalCenter }

                            Rectangle { width: 48; height: 30; radius: 4; color: root.cBg; border.color: root.cBorder
                                TextInput { id: minInp; anchors { fill: parent; margins: 2 } text: "0.0"; font.pixelSize: 12; font.family: "monospace"; color: root.cYellow; horizontalAlignment: TextInput.AlignHCenter; validator: DoubleValidator { bottom: -9999; top: 9999; decimals: 1 }
                                    Connections { target: page2Root; function onConfigRevisionChanged() { var tbl = page2Root.parsedConfig[cfgZoneCard.configKey]; if (tbl && tbl[String(modelData)]) minInp.text = String(tbl[String(modelData)][0]) } } } }
                            Rectangle { width: 48; height: 30; radius: 4; color: root.cBg; border.color: root.cBorder
                                TextInput { id: maxInp; anchors { fill: parent; margins: 2 } text: "0.0"; font.pixelSize: 12; font.family: "monospace"; color: root.cYellow; horizontalAlignment: TextInput.AlignHCenter; validator: DoubleValidator { bottom: -9999; top: 9999; decimals: 1 }
                                    Connections { target: page2Root; function onConfigRevisionChanged() { var tbl = page2Root.parsedConfig[cfgZoneCard.configKey]; if (tbl && tbl[String(modelData)]) maxInp.text = String(tbl[String(modelData)][1]) } } } }
                            Rectangle { width: 48; height: 30; radius: 4; color: root.cBg; border.color: root.cBorder
                                TextInput { id: tgtInp; anchors { fill: parent; margins: 2 } text: "0.0"; font.pixelSize: 12; font.family: "monospace"; color: root.cYellow; horizontalAlignment: TextInput.AlignHCenter; validator: DoubleValidator { bottom: -9999; top: 9999; decimals: 1 }
                                    Connections { target: page2Root; function onConfigRevisionChanged() { var tbl = page2Root.parsedConfig[cfgZoneCard.configKey]; if (tbl && tbl[String(modelData)]) tgtInp.text = String(tbl[String(modelData)][2]) } } } }

                            Item { width: 2 }
                            Text { text: modelData===10?"Top":modelData===1?"Bot":""; color: root.cDim; font.pixelSize: 10; anchors.verticalCenter: parent.verticalCenter }
                        }
                        Rectangle { width: parent.width; height: 1; color: "#1e1e3a"; anchors.bottom: parent.bottom }
                    }
                }

                Row { spacing: 6; topPadding: 8
                    CBtn { lbl:"Save"; padV:8; padH:18; fontSize:13; bg:"#0a332e"; bc:root.cGreen; tc:root.cGreen
                        onClicked: {
                            var positions = {}
                            for (var i = 0; i < cfgZoneRepeater.count; i++) {
                                var item = cfgZoneRepeater.itemAt(i)
                                if (item) {
                                  var min = parseFloat(item.minText); if(isNaN(min)) min = 0.0;
                                  var max = parseFloat(item.maxText); if(isNaN(max)) max = 0.0;
                                  var tgt = parseFloat(item.tgtText); if(isNaN(tgt)) tgt = 0.0;
                                  positions[String(item.rowNum)] = [min, max, tgt]
                                }
                            }
                            cartridgeController.saveConfig(cfgZoneCard.configKey, JSON.stringify(positions))
                        }
                    }
                    CBtn { lbl:"↺ Reset"; padV:8; padH:14; fontSize:13; bg:root.cCard; bc:root.cBorder; tc:root.cText
                        onClicked: page2Root.reloadConfig()
                    }
                }
            }
        }

    Timer {
        id: outTrayTimer
        interval: 200000
        repeat: false
        onTriggered: outTrayPopup.open()
    }

    function checkOutTrayTimer() {
        var robotActive = robotController.systemStatus !== "IDLE" && robotController.systemStatus !== "ERROR" && robotController.systemStatus !== "UNKNOWN" && robotController.systemStatus !== "EMERGENCY_STOP";
        var isAuto = cartridgeController.currentMode === "auto";
        var isManualS3 = cartridgeController.currentMode === "manual" && cartridgeController.stateOut.indexOf("S3") !== -1;
        
        if (!robotController.outReady && robotActive && (isAuto || isManualS3)) {
            if (!outTrayTimer.running) {
                outTrayTimer.restart();
            }
        } else {
            outTrayTimer.stop();
            outTrayPopup.close();
        }
    }

    Connections {
        target: robotController
        function onOutReadyChanged() { checkOutTrayTimer(); }
        function onSystemStatusChanged() { checkOutTrayTimer(); }
    }

    Connections {
        target: cartridgeController
        function onCurrentModeChanged() { checkOutTrayTimer(); }
        function onSystemStateChanged() { checkOutTrayTimer(); }
    }

    Popup {
        id: outTrayPopup
        width: 440; height: 220
        anchors.centerIn: parent
        modal: true; focus: true
        closePolicy: Popup.NoAutoClose
        background: Rectangle { color: "#1a0a0a"; radius: 10; border.color: "#ffaa00"; border.width: 3 }
        Column {
            anchors.centerIn: parent; spacing: 30
            Text {
                text: "⚠️ CẢNH BÁO CHƯA CÓ KHAY THÀNH PHẨM"
                color: "#ffaa00"; font.pixelSize: 22; font.bold: true
                anchors.horizontalCenter: parent.horizontalCenter
            }
            Text {
                text: "Hệ thống đang chờ khay Output lâu hơn 200s.\nĐã cấp khay chưa?"
                color: "#ffffff"; font.pixelSize: 18; horizontalAlignment: Text.AlignHCenter
                anchors.horizontalCenter: parent.horizontalCenter
            }
            Row {
                spacing: 40; anchors.horizontalCenter: parent.horizontalCenter
                Rectangle {
                    width: 130; height: 46; radius: 6; color: "#aa0000"
                    Text { anchors.centerIn: parent; text: "NO"; color: "white"; font.bold: true; font.pixelSize: 16 }
                    MouseArea { anchors.fill: parent; onClicked: { outTrayPopup.close(); outTrayTimer.restart(); } }
                }
                Rectangle {
                    width: 130; height: 46; radius: 6; color: "#00aa00"
                    Text { anchors.centerIn: parent; text: "YES"; color: "white"; font.bold: true; font.pixelSize: 16 }
                    MouseArea { anchors.fill: parent; onClicked: { robotController.simulateOutputTrayReady(); outTrayPopup.close(); } }
                }
            }
        }
    }
    }
