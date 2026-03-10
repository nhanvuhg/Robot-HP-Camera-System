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
        readonly property color cBg2:    "#1a1a35"
        readonly property color cCard:   "#141428"
        readonly property color cBorder: "#2a2a50"
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
                    height: 28; radius: 8
                    width: sbRow.implicitWidth + 22
                    color: root.cBg; border.color: root.cBorder
                    Row { id: sbRow; anchors.centerIn: parent; spacing: 8
                        Rectangle {
                            width: 9; height: 9; radius: 4.5; color: root.cGreen; anchors.verticalCenter: parent.verticalCenter
                            SequentialAnimation on opacity { loops: Animation.Infinite
                                NumberAnimation { to: 0.4; duration: 1000 }
                                NumberAnimation { to: 1.0; duration: 1000 } }
                        }
                        Text { text: cartridgeController.systemState; color: root.cText
                            font.pixelSize: 14; font.bold: true; font.letterSpacing: 1
                            anchors.verticalCenter: parent.verticalCenter }
                    }
                }
                Item { Layout.fillWidth: true }
                Rectangle {
                    id: modePill; height: 28; radius: 20
                    property string m: cartridgeController.currentMode
                    property bool isIdle: m === "idle" || m === ""
                    width: mpLbl.implicitWidth + 26
                    color: isIdle ? "#2a1a00" : m === "auto" ? "#0a332e" : m === "jog" ? "#332e0a" : "#1a0a33"
                    border.color: isIdle ? "#ffd740" : m === "auto" ? root.cGreen : m === "jog" ? root.cOrange : "#bb86fc"
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
            }
        }

        // ════════════════════════════════════════════════════════════
        // TAB BAR
        // ════════════════════════════════════════════════════════════
        Rectangle {
            id: tabbar
            anchors { top: header.bottom; left: parent.left; right: parent.right }
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

                                // 2 modes: AUTO | MANUAL
                                Row {
                                    id: modeRow
                                    Layout.fillWidth: true
                                    Layout.fillHeight: true
                                    spacing: 4

                                    CBtn {
                                        w: Math.floor((modeRow.width - 4) / 2)
                                        lbl: "AUTO"; bg: "#0a332e"; bc: root.cGreen; tc: root.cGreen
                                        active: !modeSelCol.modeBlocked && cartridgeController.currentMode === "auto"
                                        enabled: !modeSelCol.modeBlocked
                                        onClicked: cartridgeController.setMode("auto")
                                    }
                                    CBtn {
                                        w: Math.floor((modeRow.width - 4) / 2)
                                        lbl: "MANUAL"; bg: "#1a0a33"; bc: "#bb86fc"; tc: "#bb86fc"
                                        active: !modeSelCol.modeBlocked && cartridgeController.currentMode === "manual"
                                        enabled: !modeSelCol.modeBlocked
                                        onClicked: cartridgeController.setMode("manual")
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
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; lbl: "START";  bg: "#0a332e"; bc: root.cGreen;  tc: root.cGreen;  onClicked: cartridgeController.startSystem() }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; lbl: "STOP";   bg: "#4d1a1a"; bc: root.cRed;    tc: root.cRed;    onClicked: cartridgeController.stopSystem() }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; lbl: "PAUSE";  bg: "#4d3a0a"; bc: root.cOrange; tc: root.cOrange; onClicked: cartridgeController.pauseSystem() }
                                }

                                // Hàng 2: CONFIRM / RESUME
                                RowLayout {
                                    Layout.fillWidth: true; Layout.fillHeight: true; spacing: 4
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; lbl: "Confirm"; bg: "#1a2050"; bc: root.cAccent; tc: root.cAccent; onClicked: cartridgeController.confirmOutput() }
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

                                // Grid 2 cột × 3 hàng, fill toàn bộ không gian còn lại
                                GridLayout {
                                    Layout.fillWidth: true; Layout.fillHeight: true
                                    columns: 2; columnSpacing: 4; rowSpacing: 4

                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; lbl: "HOMING";  bg: root.cCard;   bc: root.cBorder; tc: root.cText;   onClicked: cartridgeController.gotoState("HOMING") }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; lbl: "IDLE";    bg: root.cCard;   bc: root.cBorder; tc: root.cText;   onClicked: cartridgeController.gotoState("IDLE") }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; lbl: "STATE 1"; bg: "#1a2050";   bc: root.cAccent; tc: root.cAccent; onClicked: cartridgeController.gotoState("STATE1") }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; lbl: "STATE 2"; bg: "#1a2050";   bc: root.cAccent; tc: root.cAccent; onClicked: cartridgeController.gotoState("STATE2") }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; lbl: "STATE 3"; bg: "#1a2050";   bc: root.cAccent; tc: root.cAccent; onClicked: cartridgeController.gotoState("STATE3") }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; lbl: "ERROR";   bg: "#4d1a1a";   bc: root.cRed;    tc: root.cRed;   onClicked: cartridgeController.gotoState("ERROR") }
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
                                        Repeater { model: [1,2,3,4,5,6,7,8]
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
                                        property bool isJog: cartridgeController.currentMode === "jog"

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

                                                    // − STOP + (jog mode required)
                                                    Row { spacing: 4; anchors.horizontalCenter: parent.horizontalCenter
                                                        CBtn { lbl:"−"; padV:10; padH:16; fontSize:18; bg:root.cCard; bc:root.cBorder; tc:root.cText; active: servoRow.isJog
                                                            onPressed: { if(servoRow.isJog) cartridgeController.jogServo(model.sid,"-",parseInt(velInput.text)||30) }
                                                            onReleased: cartridgeController.jogStop(model.sid) }
                                                        CBtn { lbl:"STOP"; padV:10; padH:8; fontSize:14; bg:"#4d1a1a"; bc:root.cRed; tc:root.cRed; onClicked: cartridgeController.jogStop(model.sid) }
                                                        CBtn { lbl:"+"; padV:10; padH:16; fontSize:18; bg:root.cCard; bc:root.cBorder; tc:root.cText; active: servoRow.isJog
                                                            onPressed: { if(servoRow.isJog) cartridgeController.jogServo(model.sid,"+",parseInt(velInput.text)||30) }
                                                            onReleased: cartridgeController.jogStop(model.sid) }
                                                    }

                                                    // HOMING (jog mode required)
                                                    CBtn { lbl:"HOMING"; w:parent.width; padV:12; padH:12; fontSize:16; bg:"#0a332e"; bc:root.cGreen; tc:root.cGreen; active:servoRow.isJog; onClicked: { if(servoRow.isJog) cartridgeController.homeServo(model.sid) } }

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
                                            font.pixelSize: 12; font.family: "monospace"
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
                            Row { spacing: 3
                                CBtn { lbl:"All ON";  padV:3; padH:8; fontSize:10; bg:"#0a332e"; bc:root.cGreen;  tc:root.cGreen;  onClicked: cartridgeController.simAll(1) }
                                CBtn { lbl:"All OFF"; padV:3; padH:8; fontSize:10; bg:"#4d1a1a"; bc:root.cRed;    tc:root.cRed;    onClicked: cartridgeController.simAll(0) }
                                CBtn { lbl:"Clear";   padV:3; padH:6; fontSize:10; bg:root.cCard; bc:root.cBorder; tc:root.cText;  onClicked: cartridgeController.simSensor("clear") }
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
                                        ListElement { sid:1;  slabel:"S1";  sdesc:"Stack"    }
                                        ListElement { sid:2;  slabel:"S2";  sdesc:"Stack"    }
                                        ListElement { sid:3;  slabel:"S3";  sdesc:"Stack"    }
                                        ListElement { sid:4;  slabel:"S4";  sdesc:"Detect"   }
                                        ListElement { sid:5;  slabel:"S5";  sdesc:"Out.Pos1" }
                                        ListElement { sid:6;  slabel:"S6";  sdesc:"Platform" }
                                        ListElement { sid:7;  slabel:"S7";  sdesc:"Output"   }
                                        ListElement { sid:8;  slabel:"S8";  sdesc:""         }
                                        ListElement { sid:9;  slabel:"S9";  sdesc:"Safety"   }
                                        ListElement { sid:10; slabel:"S10"; sdesc:"Cyl1- Retract"}
                                        ListElement { sid:11; slabel:"S11"; sdesc:"Cyl1+ Extend" }
                                        ListElement { sid:12; slabel:"S12"; sdesc:"Cyl2- Retract"}
                                        ListElement { sid:13; slabel:"S13"; sdesc:"Cyl2+ Extend" }
                                        ListElement { sid:14; slabel:"S14"; sdesc:""         }
                                        ListElement { sid:15; slabel:"S15"; sdesc:""         }
                                    }
                                    delegate: Rectangle {
                                        id: sBtn
                                        property bool on_: false

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
                                                sBtn.on_ = !sBtn.on_
                                                cartridgeController.simSensor(model.sid + ":" + (sBtn.on_ ? "1" : "0"))
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
                                                color: sBtn.on_ ? root.cGreen : "#333333"
                                                anchors.horizontalCenter: parent.horizontalCenter
                                            }
                                        }
                                    }
                                }
                            }

                            // ── Chú thích ──
                            Text {
                                text: "<b>S1-S3</b> Stack · <b>S4</b> Detect · <b>S6</b> Platform · <b>S7</b> Output ·\n<b>S10</b> Cyl1 · <b>S11</b> Cyl1 · <b>S12</b> Cyl2 · <b>S13</b> Cyl2"
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

                    ConfigCard {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        title: "Pos 1: Input Stack (InY)"
                        configKey: "iny_input_stack"
                        configSource: page2Root.parsedConfig
                    }
                    ConfigCard {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        title: "Pos 1: Output Stack (InY)"
                        configKey: "iny_output_stack"
                        configSource: page2Root.parsedConfig
                    }
                    ConfigCard {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        title: "Pos 2: Output Table (OutY)"
                        configKey: "outy_output_table"
                        configSource: page2Root.parsedConfig
                    }

                    // Card 4: Servo Key Positions
                    Rectangle {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        color: root.cBg2; border.color: root.cBorder; radius: 6
                        HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cAccent : root.cBorder }

                        property var servoParams: [
                            { key:"inx_home",            label:"InX Home",      desc:"S1 home" },
                            { key:"inx_target2",         label:"InX Target",    desc:"S1 lấy khay" },
                            { key:"inx_output_stack",    label:"InX OutStack",  desc:"Đặt khay" },
                            { key:"iny_home",            label:"InY Home",      desc:"S2 home" },
                            { key:"iny_target2",         label:"InY Target",    desc:"Robot place" },
                            { key:"iny_safe_zone",       label:"InY SafeZone",  desc:"Safe zone" },
                            { key:"servo3_push_position",label:"S3 Push",       desc:"Push pos" },
                            { key:"outx_home",           label:"OutX Home",     desc:"S4 home" },
                            { key:"outx_target2",        label:"OutX Target",   desc:"Output stack" },
                            { key:"outx_target3",        label:"OutX Robot",    desc:"Robot tray" },
                            { key:"outy_home",           label:"OutY Home",     desc:"S5 home" },
                            { key:"outy_target2",        label:"OutY Target",   desc:"Pick/place" },
                            { key:"outy_safe_zone",      label:"OutY SafeZone", desc:"Safe zone" }
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
                property string currentMode: "manual"  // "auto" | "camera_ai" | "manual"
                property real stepValue: 1.0
                property int speedVal: 100
                property bool rowLocked: false
                property int jogStep: 1               // 0.1, 1, 5, 10

                Item {
                    id: p3Inner
                    anchors { fill: parent; margins: 10 }

                    // ════════════════ MANUAL MODE ══════════════════════
                    Rectangle {
                        id: modeToggle
                        anchors { top: parent.top; left: parent.left; right: parent.right }
                        height: 32; radius: 5; color: "#2a0a4a"; border.color: "#bb86fc"; border.width: 2
                        Text { anchors.centerIn: parent; text: "MANUAL MODE"; color: "#bb86fc"; font.pixelSize: 12; font.bold: true }
                    }

                    // ════════════════ CONTENT AREA ════════════════════════
                    Item {
                        id: contentArea
                        anchors { top: modeToggle.bottom; topMargin: 8; left: parent.left; right: parent.right; bottom: robotLogBar.top; bottomMargin: 4 }


                        // ──────────── MANUAL MODE: JOG ────────────────────
                        Item {
                            anchors.fill: parent

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
                                                    width: 46; height: 40; radius: 5; color: root.cCard; border.color: root.cBorder; border.width: 2
                                                    Text { anchors.centerIn: parent; text: modelData.axis + "-"; color: root.cRed; font.pixelSize: 12; font.bold: true }
                                                    MouseArea { anchors.fill: parent; onPressed: robotController.jogStart(modelData.neg); onReleased: robotController.jogStop() }
                                                }
                                                Rectangle {
                                                    width: parent.width - 96; height: 40; radius: 5; color: "#0d1117"; border.width: 2; border.color: root.cBorder
                                                    Text { anchors.centerIn: parent; text: robotController.cartesianPose.length > index ? robotController.cartesianPose[index].toFixed(4) : "0.0000"; color: "#FFD700"; font.pixelSize: 16; font.family: "monospace"; font.bold: true }
                                                }
                                                Rectangle {
                                                    width: 46; height: 40; radius: 5; color: root.cCard; border.color: root.cBorder; border.width: 2
                                                    Text { anchors.centerIn: parent; text: modelData.axis + "+"; color: root.cGreen; font.pixelSize: 12; font.bold: true }
                                                    MouseArea { anchors.fill: parent; onPressed: robotController.jogStart(modelData.pos); onReleased: robotController.jogStop() }
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
                                                width: parent.width / 2 - 2; height: 34; radius: 5; color: "#0a2a1a"; border.color: root.cCyan
                                                Text { anchors.centerIn: parent; text: "GET POSE"; color: root.cCyan; font.pixelSize: 12; font.bold: true }
                                                MouseArea { anchors.fill: parent; onClicked: {
                                                    robotController.getPose()
                                                    for (var i = 0; i < 6; i++) {
                                                        if (robotController.cartesianPose.length > i)
                                                            cartInputs.itemAt(i).children[1].children[0].text = robotController.cartesianPose[i].toFixed(2)
                                                    }
                                                }}
                                            }
                                            Rectangle {
                                                width: parent.width / 2 - 2; height: 34; radius: 5; color: "#0a1a35"; border.color: root.cAccent
                                                Text { anchors.centerIn: parent; text: "SEND MovL"; color: root.cAccent; font.pixelSize: 12; font.bold: true }
                                                MouseArea { anchors.fill: parent; onClicked: {
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
                                            Rectangle { width: 3; height: 11; radius: 1; color: "#bb86fc"; anchors.verticalCenter: parent.verticalCenter }
                                            Text { text: "JOINT (deg)"; color: "#bb86fc"; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1 }
                                        }
                                        Repeater {
                                            id: jointRep
                                            model: 6
                                            delegate: Row {
                                                property int jn: index + 1
                                                width: jointCol.width; height: 44; spacing: 2
                                                Rectangle {
                                                    width: 46; height: 40; radius: 5; color: root.cCard; border.color: root.cBorder; border.width: 2
                                                    Text { anchors.centerIn: parent; text: "J" + jn + "-"; color: root.cRed; font.pixelSize: 12; font.bold: true }
                                                    MouseArea { anchors.fill: parent; onPressed: robotController.jogStart("j" + jn + "-"); onReleased: robotController.jogStop() }
                                                }
                                                Rectangle {
                                                    width: parent.width - 96; height: 40; radius: 5; color: "#0d1117"; border.width: 2; border.color: root.cBorder
                                                    Text { anchors.centerIn: parent; text: robotController.jointAngles.length > index ? robotController.jointAngles[index].toFixed(4) : "0.0000"; color: "#FFD700"; font.pixelSize: 16; font.family: "monospace"; font.bold: true }
                                                }
                                                Rectangle {
                                                    width: 46; height: 40; radius: 5; color: root.cCard; border.color: root.cBorder; border.width: 2
                                                    Text { anchors.centerIn: parent; text: "J" + jn + "+"; color: root.cGreen; font.pixelSize: 12; font.bold: true }
                                                    MouseArea { anchors.fill: parent; onPressed: robotController.jogStart("j" + jn + "+"); onReleased: robotController.jogStop() }
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
                                                        width: parent.width; height: 36; radius: 4; color: "#0d1117"; border.color: "#bb86fc"; border.width: 2
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
                                                width: parent.width / 2 - 2; height: 34; radius: 5; color: "#0a2a1a"; border.color: root.cCyan
                                                Text { anchors.centerIn: parent; text: "GET ANGLES"; color: root.cCyan; font.pixelSize: 12; font.bold: true }
                                                MouseArea { anchors.fill: parent; onClicked: {
                                                    robotController.getAngles()
                                                    for (var i = 0; i < 6; i++) {
                                                        if (robotController.jointAngles.length > i)
                                                            jointInputs.itemAt(i).children[1].children[0].text = robotController.jointAngles[i].toFixed(2)
                                                    }
                                                }}
                                            }
                                            Rectangle {
                                                width: parent.width / 2 - 2; height: 34; radius: 5; color: "#1a0a33"; border.color: "#bb86fc"
                                                Text { anchors.centerIn: parent; text: "SEND MovJ"; color: "#bb86fc"; font.pixelSize: 12; font.bold: true }
                                                MouseArea { anchors.fill: parent; onClicked: {
                                                    var vals = []
                                                    for (var i = 0; i < 6; i++) vals.push(parseFloat(jointInputs.itemAt(i).children[1].children[0].text) || 0)
                                                    robotController.moveJoint(vals[0],vals[1],vals[2],vals[3],vals[4],vals[5])
                                                }}
                                            }
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
                                                    Rectangle { width: speedSlider.visualPosition * parent.width; height: parent.height; radius: 4; color: "#bb86fc" }
                                                }
                                                handle: Rectangle { x: speedSlider.leftPadding + speedSlider.visualPosition * (speedSlider.availableWidth - width); y: speedSlider.topPadding + speedSlider.availableHeight / 2 - 8; width: 16; height: 16; radius: 8; color: "#bb86fc"; border.color: "#fff" }
                                            }
                                            Rectangle {
                                                width: 44; height: 28; radius: 4; color: "#0d1117"; border.color: "#bb86fc"; border.width: 2
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

                                        // Enable / Disable
                                        Row { spacing: 4; width: parent.width
                                            CBtn { lbl: "ENABLE"; width: (parent.width - 4) / 2; bg: "#0a332e"; bc: root.cGreen; tc: root.cGreen; padV: 8; onClicked: robotController.enableSystem(true) }
                                            CBtn { lbl: "DISABLE"; width: (parent.width - 4) / 2; bg: root.cCard; bc: root.cBorder; tc: root.cDim; padV: 8; onClicked: robotController.enableSystem(false) }
                                        }

                                        // Pause / Resume
                                        Row { spacing: 4; width: parent.width
                                            Rectangle {
                                                width: (parent.width - 4) / 2; height: 34; radius: 4
                                                color: "#1a1a00"; border.color: root.cOrange
                                                Text { anchors.centerIn: parent; text: "PAUSE"; color: root.cOrange; font.pixelSize: 12; font.bold: true }
                                                MouseArea { anchors.fill: parent; onClicked: robotController.pauseRobot() }
                                            }
                                            Rectangle {
                                                width: (parent.width - 4) / 2; height: 34; radius: 4
                                                color: "#0a2a1a"; border.color: root.cCyan
                                                Text { anchors.centerIn: parent; text: "RESUME"; color: root.cCyan; font.pixelSize: 12; font.bold: true }
                                                MouseArea { anchors.fill: parent; onClicked: robotController.resumeRobot() }
                                            }
                                        }

                                        // Clear Error
                                        Rectangle {
                                            width: parent.width; height: 34; radius: 4
                                            color: "#0a1a2a"; border.color: "#4da6ff"
                                            Text { anchors.centerIn: parent; text: "CLEAR ERROR"; color: "#4da6ff"; font.pixelSize: 12; font.bold: true }
                                            MouseArea { anchors.fill: parent; onClicked: robotController.clearError() }
                                        }

                                        // E-STOP (biggest button)
                                        Rectangle {
                                            width: parent.width; height: 50; radius: 5; color: "#4d1a1a"; border.color: root.cRed; border.width: 2
                                            Text { anchors.centerIn: parent; text: "EMERGENCY\nSTOP"; color: root.cRed; font.pixelSize: 14; font.bold: true; horizontalAlignment: Text.AlignHCenter }
                                            MouseArea { anchors.fill: parent; onClicked: robotController.emergencyStop(true) }
                                        }
                                    }
                                }
                            }
                        } // manual mode
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

            signal clicked(); signal pressed(); signal released()

            width:  w > 0 ? w : cbrT.width + padH * 2
            height: cbrT.height + padV * 2
            radius: 4; color: bg; border.color: bc
            opacity: active ? 1.0 : 0.5
            Behavior on border.color { ColorAnimation { duration: 150 } }
            Behavior on color        { ColorAnimation { duration: 150 } }

            Text { id: cbrT; anchors.centerIn: parent; text: cbr.lbl; color: cbr.tc
                font.pixelSize: cbr.fontSize; font.bold: true; font.capitalization: Font.AllUppercase }

            MouseArea { anchors.fill: parent
                onClicked:  { if(cbr.enabled){ cbr.scale=1.0; cbr.clicked() } }
                onPressed:  { if(cbr.enabled) cbr.scale=0.96; cbr.pressed() }
                onReleased: { cbr.scale=1.0; cbr.released() }
            }
            HoverHandler { onHoveredChanged: cbr.border.color = hovered ? (cbr.bc===root.cBorder?root.cAccent:cbr.bc) : cbr.bc }
            Behavior on scale { NumberAnimation { duration: 80 } }
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
                    model: [8,7,6,5,4,3,2,1]
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
                                text: modelData===8?"Top":modelData===1?"Bot":""
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
    }
