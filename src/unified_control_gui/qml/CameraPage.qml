import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls.Material 2.15

Item {
    id: cameraPageRoot

    property string currentTime: Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
    property bool rowLocked: (robotController.systemStatus || "").toUpperCase() === "INIT_LOAD_CHAMBER_DIRECT" && robotController.selectedRow >= 1
    property bool modeLocked: false
    property string ctrlMode: "auto"  // "auto" | "camera_ai"

    readonly property color cPanel:       "#081e29"
    readonly property color cPanel2:      "#0a2238"   // dark navy blue — match camera systems tone (was #051a1a teal-black)
    readonly property color cBorder:      "#134357"
    readonly property color cText:        "#e8e8f0"
    readonly property color cMuted:       "#8888aa"
    readonly property color cAccent:      "#4f6cff"
    readonly property color cOk:          "#00e676"
    readonly property color cOkBg:        Qt.rgba(0.0, 0.90, 0.46, 0.15)
    readonly property color cWarn:        "#ffa726"
    readonly property color cWarnBg:      Qt.rgba(1.0, 0.65, 0.15, 0.15)
    readonly property color cBad:         "#ff5252"
    readonly property color cBadBg:       Qt.rgba(1.0, 0.32, 0.32, 0.15)

    function classifyPressure(val, lowT, highT, limitT) {
        if (val < lowT) return "low";
        if (val >= limitT) return "limit";
        if (val >= highT) return "high";
        return "ok";
    }

    function getCartridgeStats() {
        var list = hpController.cartridgePressures;
        if (!list || list.length === 0) {
            return "Min 0.0 | Avg 0.0 | Max\n0.0 mbar";
        }
        var minVal = Number.MAX_VALUE;
        var maxVal = -Number.MAX_VALUE;
        var sum = 0.0;
        var count = 0;
        for (var i = 0; i < list.length; i++) {
            var v = parseFloat(list[i]);
            if (!isNaN(v)) {
                if (v < minVal) minVal = v;
                if (v > maxVal) maxVal = v;
                sum += v;
                count++;
            }
        }
        if (count === 0) return "Min 0.0 | Avg 0.0 | Max\n0.0 mbar";
        var avgVal = sum / count;
        return "Min " + minVal.toFixed(1) + " | Avg " + avgVal.toFixed(1) + " | Max\n" + maxVal.toFixed(1) + " mbar";
    }

    // Calculated from systemStatus - block mode change when robot is running
    // MANUAL is considered "idle" (not busy) - robot only waits for manual commands, allowing mode change.
    property bool robotBusy: {
        var s = (robotController.systemStatus || "").toLowerCase()
        return s !== "" && s !== "idle" && s !== "ready" && s !== "unknown" && s !== "manual"
    }

    // Unlock row selection khi process idle/done
    Connections {
        target: robotController
        function onSystemStatusChanged() {
            var s = (robotController.systemStatus || "").toLowerCase()
            if (s === "idle" || s === "ready" || s === "") {
                cameraPageRoot.modeLocked = false
            }
        }
    }

    // Sync mode from Cartridge (ensures both UIs are synchronized)
    Connections {
        target: cartridgeController
        function onCurrentModeChanged() {
            var m = (cartridgeController.currentMode || "").toLowerCase()
            if (m === "auto" || m === "ai" || m === "camera_ai" || m === "manual") {
                cameraPageRoot.ctrlMode = (m === "ai") ? "camera_ai" : m;
            }
        }
    }
    Timer {
        interval: 1000
        running: true
        repeat: true
        onTriggered: currentTime = Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        // ── Title Bar ──────────────────────────────────────────
        Item {
            Layout.fillWidth: true
            height: 80

            Rectangle {
                anchors.fill: parent
                color: "transparent"

                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 10

                    Item { Layout.preferredWidth: 60 }

                    Item {
                        Layout.fillWidth: true
                        Text {
                            anchors.centerIn: parent
                            text: "ROS2 - ROBOT CONTROL SYSTEM"
                            font.pixelSize: 24; font.bold: true; color: "#6cf"
                        }
                    }

                    Button {
                        text: "CARTRIDGE SYSTEM  ▸"
                        Layout.preferredHeight: 50
                        font.pixelSize: 16; font.bold: true
                        onClicked: stackView.push(cartridgePage)
                        background: Rectangle {
                            radius: 6
                            gradient: Gradient {
                                GradientStop { position: 0.0; color: "#4f6cff" }
                                GradientStop { position: 1.0; color: "#6c5ce7" }
                            }
                        }
                        contentItem: Text {
                            text: parent.text; font: parent.font
                            color: "#fff"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                        }
                    }

                    RowLayout {
                        spacing: 5
                        Text {
                            text: "IGNORE SCALE"
                            font.pixelSize: 14; font.bold: true
                            color: ignoreScaleToggle.checked ? "#ef4444" : "#94a3b8"
                            verticalAlignment: Text.AlignVCenter
                        }
                        Switch {
                            id: ignoreScaleToggle
                            checked: robotController.ignoreScale
                            onCheckedChanged: {
                                if (robotController.ignoreScale !== checked)
                                    robotController.ignoreScale = checked;
                            }
                        }
                    }

                    Button {
                        Layout.preferredWidth: 50; Layout.preferredHeight: 50
                        onClicked: {
                            var comp = Qt.createComponent("frm_settings.qml")
                            if (comp.status === Component.Ready) {
                                var win = comp.createObject(mainWindow)
                                if (win) {
                                    win.x = mainWindow.x + (mainWindow.width - win.width) / 2
                                    win.y = mainWindow.y + (mainWindow.height - win.height) / 2
                                    win.show()
                                }
                            }
                        }
                        background: Rectangle { radius: 6; color: "transparent"; border.color: "#134357"; border.width: 2 }
                        contentItem: Image { source: "qrc:/icons/qml/icons/settings.svg"; width: 24; height: 24; fillMode: Image.PreserveAspectFit; smooth: true }
                    }

                    Button {
                        Layout.preferredWidth: 50; Layout.preferredHeight: 50
                        onClicked: Qt.quit()
                        background: Rectangle { radius: 6; color: "transparent"; border.color: "#134357"; border.width: 2 }
                        contentItem: Image { source: "qrc:/icons/qml/icons/power_settings.svg"; width: 24; height: 24; fillMode: Image.PreserveAspectFit; smooth: true }
                    }
                }
            }
        }

        // ── Scale Issue Warning Banner ─────────────────────────
        Rectangle {
            id: scaleWarnBanner
            Layout.fillWidth: true
            height: 44
            visible: mainWindow.scaleIssueWarning
            color: "#2d1010"
            border.color: "#ef4444"
            border.width: 1
            radius: 4

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 16
                anchors.rightMargin: 10
                spacing: 12

                Text {
                    text: "⚠"
                    color: "#ef4444"
                    font.pixelSize: 20
                    font.bold: true
                }
                Text {
                    Layout.fillWidth: true
                    text: "SCALE ISSUE — Scale problem or cartridge taken away. Operator intervention was required. Check scale and loadcell before next cycle."
                    color: "#fca5a5"
                    font.pixelSize: 14
                    elide: Text.ElideRight
                }
                Button {
                    Layout.preferredWidth: 110; Layout.preferredHeight: 30
                    text: "✓  Confirm"
                    font.pixelSize: 13; font.bold: true
                    background: Rectangle { color: "#4d1010"; border.color: "#ef4444"; border.width: 1; radius: 4 }
                    contentItem: Text {
                        text: parent.text; color: "#ef4444"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: mainWindow.scaleIssueWarning = false
                }
            }
        }

        // ── Main Content ───────────────────────────────────────
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 10

            // Camera Area — 2 rows stacked, 16:9
            ColumnLayout {
                Layout.fillHeight: true
                Layout.preferredWidth: 770
                Layout.maximumWidth: 770
                spacing: 8

                Rectangle {
                    color: "#081e29"
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    border.color: "#134357"
                    radius: 6
                    clip: true

                    Column {
                        anchors.fill: parent; anchors.margins: 10
                        spacing: 10

                        Repeater {
                            model: camNode.cameraList
                            delegate: CameraView {
                                cameraName: modelData.name
                                topic: modelData.topic
                                providerId: modelData.providerId
                                width: parent.width
                            }
                        }
                    }
                }

            }

            // ── Analog / Pressure Column (Middle) ──────────────────
            Rectangle {
                Layout.fillHeight: true
                Layout.preferredWidth: 400
                Layout.maximumWidth: 400
                color: "#081e29"
                border.color: "#134357"
                radius: 6

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 10

                    Text {
                        text: "ANALOG PRESSURE"
                        color: "#5cf4f1"
                        font.pixelSize: 20
                        font.bold: true
                        font.letterSpacing: 0.6
                        Layout.fillWidth: true
                    }

                    ColumnLayout {
                        Layout.fillWidth: true
                        spacing: 8
                        PCard { lbl: "S1 Chamber";   val: hpController.pressureS1; maxVal: 1200 }
                        PCard { lbl: "S2 Cartridge"; val: hpController.pressureS2; maxVal: 1200 }
                        PCard { lbl: "S3 Tank";      val: hpController.pressureS3; maxVal: 1000 }
                    }

                    Item { height: 10 } // Spacer

                    RowLayout {
                        Layout.fillWidth: true
                        Text {
                            text: "CARTRIDGE\nPRESSURE"
                            color: "#8888aa"
                            font.pixelSize: 18
                            font.bold: true
                            Layout.fillWidth: true
                        }
                        Text {
                            text: getCartridgeStats()
                            color: "#8888aa"
                            font.pixelSize: 15
                            font.bold: true
                            horizontalAlignment: Text.AlignRight
                            wrapMode: Text.Wrap
                            Layout.preferredWidth: 220
                        }
                    }

                    ColumnLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        spacing: 5
                        Repeater {
                            model: 8
                            CartRow {
                                cartName: "Cart " + (index + 1)
                                cartVal:  (hpController.cartridgePressures && hpController.cartridgePressures.length > index) ? (Number(hpController.cartridgePressures[index]) || 0) : 0
                            }
                        }
                    }
                }
            }

            // ── System Monitor + Controls ──────────────────────
            Rectangle {
                Layout.fillHeight: true
                Layout.fillWidth: true
                Layout.minimumWidth: 300
                color: "#081e29"
                border.color: "#134357"
                radius: 6

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 6

                    Text {
                        text: "SYSTEM MONITOR"
                        color: "#5cf4f1"; font.bold: true; font.pixelSize: 23
                        Layout.alignment: Qt.AlignHCenter
                    }
                    Rectangle { Layout.fillWidth: true; height: 1; color: "#134357" }

                    // ── 3-Column Layout: Left (Robot Status), Middle (Ink Info), Right (Control Mode) ──
                    RowLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: false
                        spacing: 20

                        // COLUMN 1: Robot Status
                        GridLayout {
                            Layout.fillWidth: true
                            Layout.alignment: Qt.AlignTop
                            columns: 2; rowSpacing: 8; columnSpacing: 10

                            Text { text: "Mode:"; color: "#94a3b8"; font.pixelSize: 18 }
                            Text { text: cameraPageRoot.ctrlMode.toUpperCase(); color: cameraPageRoot.ctrlMode === "camera_ai" ? "#5cf4f1" : "#4f6cff"; font.bold: true; font.pixelSize: 18; elide: Text.ElideRight; Layout.fillWidth: true }

                            Text { text: "Uptime:"; color: "#94a3b8"; font.pixelSize: 18 }
                            Text { text: robotController.systemUptime; color: "#f59e0b"; font.bold: true; font.pixelSize: 18; elide: Text.ElideRight; Layout.fillWidth: true }

                            Text { text: "State Robot:"; color: "#94a3b8"; font.pixelSize: 18 }
                            Text { text: robotController.systemStatus; color: "#10b981"; font.bold: true; font.pixelSize: 18; elide: Text.ElideRight; Layout.fillWidth: true }
                        }

                        Rectangle { width: 1; Layout.fillHeight: true; color: "#134357" }

                        // COLUMN 2: Ink & Scale Info
                        GridLayout {
                            Layout.fillWidth: true
                            Layout.alignment: Qt.AlignTop
                            columns: 2; rowSpacing: 8; columnSpacing: 10

                            Text { text: "Ink Name:"; color: "#94a3b8"; font.pixelSize: 18 }
                            Text { text: scaleController.activeInkName; color: "#5cf4f1"; font.bold: true; font.pixelSize: 18; elide: Text.ElideRight; Layout.fillWidth: true }

                            Text { text: "Cartridge Type:"; color: "#94a3b8"; font.pixelSize: 18 }
                            Text { text: scaleController.activeCartName; color: "#f59e0b"; font.bold: true; font.pixelSize: 18; elide: Text.ElideRight; Layout.fillWidth: true }

                            Text { text: "Weight Batch:"; color: "#94a3b8"; font.pixelSize: 18 }
                            Text { text: scaleController.totalBatchWeight > 0 ? scaleController.totalBatchWeight.toFixed(2) + " g" : "0.00 g"; color: "#10b981"; font.bold: true; font.pixelSize: 18; elide: Text.ElideRight; Layout.fillWidth: true }
                        }

                        Rectangle { width: 1; Layout.fillHeight: true; color: "#134357" }

                        // COLUMN 3: Control Mode
                        ColumnLayout {
                            Layout.fillWidth: true
                            Layout.alignment: Qt.AlignTop
                            spacing: 6

                            Text { text: "CONTROL MODE "; color: "#5cf4f1"; font.pixelSize: 16; font.bold: true; font.letterSpacing: 1 }
                            Repeater {
                                model: [
                                    { key: "auto",      lbl: "AUTO",      bc: "#4f6cff" },
                                    { key: "camera_ai", lbl: "CAMERA AI", bc: "#5cf4f1" }
                                ]
                                delegate: Rectangle {
                                    required property var modelData
                                    property bool isSelected: cameraPageRoot.ctrlMode === modelData.key
                                    property bool isLocked: cameraPageRoot.modeLocked || cameraPageRoot.robotBusy
                                    Layout.fillWidth: true; height: 35; radius: 5
                                    color: isSelected ? modelData.bc + "33" : "#0d2538"
                                    border.color: isSelected ? modelData.bc : (isLocked ? "#2a3a4a" : "#134357"); border.width: 2
                                    opacity: (!isSelected && isLocked) ? 0.35 : 1.0
                                    Text { anchors.centerIn: parent; text: (isLocked && !isSelected ? "🔒 " : "") + modelData.lbl; color: isSelected ? modelData.bc : "#94a3b8"; font.pixelSize: 16; font.bold: true }
                                    MouseArea {
                                        anchors.fill: parent
                                        enabled: !parent.isLocked
                                        onClicked: {
                                            cameraPageRoot.ctrlMode = modelData.key
                                        }
                                    }
                                }
                            }
                        }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#134357" }

                    // Error indicator
                    Rectangle {
                        Layout.fillWidth: true; Layout.fillHeight: false
                        height: (robotController.errorLog || "").length > 0 ? 24 : 0; radius: 4
                        visible: (robotController.errorLog || "").length > 0
                        color: "#1a0a0a"; border.color: "#ef4444"
                        Row { anchors.fill: parent; anchors.leftMargin: 6; spacing: 4
                            Text { text: "⚠"; color: "#ef4444"; font.pixelSize: 17; anchors.verticalCenter: parent.verticalCenter }
                            Text { text: robotController.errorLog; color: "#FFD700"; font.pixelSize: 17; font.family: "monospace"; anchors.verticalCenter: parent.verticalCenter; elide: Text.ElideRight; width: parent.width - 30 }
                        }
                    }

                    // Input Row
                    RowLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: false
                        Text { text: "INPUT ROW"; color: "#5cf4f1"; font.pixelSize: 17; font.bold: true }
                        Text {
                            text: cameraPageRoot.rowLocked ? "🔒 Waiting for tray..." : (cameraPageRoot.ctrlMode === "camera_ai" ? "(AI auto)" : "Select then press PICK_INPUT")
                            color: cameraPageRoot.rowLocked ? "#ef4444" : "#6b7280"
                            font.pixelSize: 17
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 4
                        Repeater { model: 5
                            delegate: Rectangle {
                                property int rn: index + 1
                                property bool aiMode: cameraPageRoot.ctrlMode === "camera_ai"
                                property bool isReady: aiMode && (robotController.rowReady[index] === true)
                                property bool isActive: robotController.selectedRow === rn
                                // MANUAL cũng được pick row: sau STOP (soft_stop → MANUAL), operator
                                // cần repick row trước khi PICK_INPUT thủ công. Chỉ AI mode tự chọn row.
                                property bool canSelect: !cameraPageRoot.rowLocked && (cameraPageRoot.ctrlMode === "auto" || cameraPageRoot.ctrlMode === "manual")
                                Layout.fillWidth: true; height: 32; radius: 5
                                color: aiMode
                                       ? (isActive ? "#0a4020" : (isReady ? "#0d3320" : "#0d2538"))
                                       : (isActive ? "#0a4020" : "#0d2538")
                                border.color: aiMode
                                              ? (isActive ? "#10b981" : (isReady ? "#10b981" : "#134357"))
                                              : (isActive ? "#10b981" : "#134357")
                                border.width: aiMode
                                              ? (isActive ? 3 : (isReady ? 2 : 1))
                                              : (isActive ? 2 : 1)
                                opacity: aiMode
                                         ? (isReady || isActive ? 1.0 : 0.45)
                                         : (canSelect ? 1.0 : (isActive ? 1.0 : 0.45))
                                Rectangle {
                                    visible: isActive
                                    anchors { top: parent.top; left: parent.left; right: parent.right }
                                    height: 3; radius: 2; color: "#10b981"
                                }
                                Text {
                                    anchors.centerIn: parent; text: "R" + rn
                                    color: aiMode
                                           ? (isActive ? "#34d399" : (isReady ? "#10b981" : "#94a3b8"))
                                           : (isActive ? "#10b981" : "#94a3b8")
                                    font.pixelSize: 18; font.bold: isActive || isReady
                                }
                                MouseArea { anchors.fill: parent; enabled: canSelect; onClicked: robotController.selectRow(rn) }
                            }
                        }
                    }

                    // Output Slot
                    RowLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: false
                        Text { text: "OUTPUT SLOT"; color: "#5cf4f1"; font.pixelSize: 17; font.bold: true }
                        Text {
                            text: robotController.selectedSlot > 0 ? ("Selected slot " + robotController.selectedSlot) : "Select output tray position"
                            color: "#6b7280"; font.pixelSize: 17
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 3
                        Repeater { model: 9
                            delegate: Rectangle {
                                property int sn: index + 1
                                property bool aiMode: cameraPageRoot.ctrlMode === "camera_ai"
                                property bool isReady: aiMode && (robotController.slotReady[index] === true)
                                property bool isActive: robotController.selectedSlot === sn
                                // Tương tự row picker: MANUAL cũng pick được sau STOP.
                                property bool canSelect: cameraPageRoot.ctrlMode === "auto" || cameraPageRoot.ctrlMode === "manual"
                                Layout.fillWidth: true; height: 32; radius: 5
                                color: aiMode
                                       ? (isActive ? "#0a4020" : (isReady ? "#0d3320" : "#0d2538"))
                                       : (isActive ? "#04363a" : "#0d2538")
                                border.color: aiMode
                                              ? (isActive ? "#10b981" : (isReady ? "#10b981" : "#134357"))
                                              : (isActive ? "#00bcd4" : "#134357")
                                border.width: aiMode
                                              ? (isActive ? 3 : (isReady ? 2 : 1))
                                              : (isActive ? 2 : 1)
                                opacity: aiMode
                                         ? (isReady || isActive ? 1.0 : 0.45)
                                         : (canSelect ? 1.0 : (isActive ? 1.0 : 0.45))
                                Rectangle {
                                    visible: isActive
                                    anchors { top: parent.top; left: parent.left; right: parent.right }
                                    height: 3; radius: 2; color: aiMode ? "#10b981" : "#00bcd4"
                                }
                                Text {
                                    anchors.centerIn: parent; text: "O" + sn
                                    color: aiMode
                                           ? (isActive ? "#34d399" : (isReady ? "#10b981" : "#94a3b8"))
                                           : (isActive ? "#00bcd4" : "#94a3b8")
                                    font.pixelSize: 16; font.bold: isActive || isReady
                                }
                                MouseArea { anchors.fill: parent; enabled: canSelect; onClicked: robotController.selectSlot(sn) }
                            }
                        }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#134357" }

                    Text { text: "STATE COMMANDS"; color: "#5cf4f1"; font.pixelSize: 16; font.bold: true; font.letterSpacing: 1 }
                    GridLayout {
                        Layout.fillWidth: true; columns: 2; rowSpacing: 5; columnSpacing: 5
                        Repeater {
                            model: [
                                { lbl: "IN_READY",     icon: "",   bg: "#351a0a", bc: "#ffaa4f" },
                                { lbl: "OUT_READY",    icon: "",   bg: "#351a0a", bc: "#ffaa4f" },
                                { lbl: "PICK_INPUT",   icon: "↓",  bg: "#0a1a35", bc: "#4f6cff" },
                                { lbl: "PICK_CHAMBER", icon: "⟳", bg: "#0a1a35", bc: "#4f6cff" },
                                { lbl: "PLACE_OUTPUT", icon: "",   bg: "#051a1a", bc: "#00bcd4" },
                                { lbl: "PLACE_FAIL",   icon: "",   bg: "#1a1505", bc: "#ff9800" }
                            ]
                            delegate: Rectangle {
                                required property var modelData
                                property bool isActive: (modelData.lbl === "IN_READY" && robotController.inReady) || (modelData.lbl === "OUT_READY" && robotController.outReady)
                                Layout.fillWidth: true; height: 64; radius: 5
                                color: isActive ? Qt.lighter(modelData.bg, 1.8) : (ma.pressed ? Qt.darker(modelData.bg, 1.3) : modelData.bg)
                                border.color: isActive ? "#00ff00" : (ma.pressed ? Qt.lighter(modelData.bc, 1.2) : modelData.bc)
                                border.width: isActive || ma.pressed ? 3 : 2
                                scale: ma.pressed ? 0.95 : 1.0
                                Behavior on color { ColorAnimation { duration: 150 } }
                                Behavior on border.color { ColorAnimation { duration: 150 } }
                                Behavior on scale { NumberAnimation { duration: 100 } }
                                Row { anchors.centerIn: parent; spacing: 6
                                    Rectangle {
                                        width: 14; height: 14; radius: 7
                                        color: parent.parent.isActive ? "#00ff00" : "#444"
                                        border.color: parent.parent.isActive ? "#fff" : "#222"
                                        border.width: 1
                                        visible: modelData.lbl === "IN_READY" || modelData.lbl === "OUT_READY"
                                        anchors.verticalCenter: parent.verticalCenter
                                        Behavior on color { ColorAnimation { duration: 200 } }
                                    }
                                    Text { text: modelData.icon; color: modelData.bc; font.pixelSize: 22; anchors.verticalCenter: parent.verticalCenter }
                                    Text { text: modelData.lbl; color: modelData.bc; font.pixelSize: 20; font.bold: true; anchors.verticalCenter: parent.verticalCenter }
                                }
                                MouseArea { id: ma; anchors.fill: parent; onClicked: {
                                    if (modelData.lbl === "IN_READY") robotController.simulateInputTrayReady()
                                    else if (modelData.lbl === "OUT_READY") robotController.simulateOutputTrayReady()
                                    else if (modelData.lbl === "PICK_INPUT") robotController.simulateFeedChamber()
                                    else if (modelData.lbl === "PICK_CHAMBER") robotController.simulateFillDone()
                                    else if (modelData.lbl === "PLACE_OUTPUT") robotController.gotoState("PLACE_TO_OUTPUT")
                                    else if (modelData.lbl === "PLACE_FAIL") robotController.gotoState("PLACE_TO_FAIL")
                                    else robotController.gotoState(modelData.lbl)
                                }}
                            }
                        }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#134357" }

                    Text { text: "SYSTEM CONTROL"; color: "#5cf4f1"; font.pixelSize: 16; font.bold: true; font.letterSpacing: 1 }

                    Rectangle { Layout.fillWidth: true; height: 56; radius: 5; color: emMA.pressed ? Qt.darker("#4d1a1a", 1.2) : "#4d1a1a"; border.color: "#ef4444"; border.width: 2
                        scale: emMA.pressed ? 0.95 : 1.0
                        Behavior on scale { NumberAnimation { duration: 100 } }
                        Text { anchors.centerIn: parent; text: "⛔ EMERGENCY STOP"; color: "#ef4444"; font.pixelSize: 21; font.bold: true }
                        MouseArea { id: emMA; anchors.fill: parent; onClicked: { cameraPageRoot.modeLocked = false; robotController.emergencyStop(true) } }
                    }

                    GridLayout {
                        Layout.fillWidth: true; columns: 3; rowSpacing: 5; columnSpacing: 5

                        Rectangle { Layout.fillWidth: true; height: 52; radius: 5; color: stopResetMA.pressed ? Qt.darker("#4a1a00", 1.2) : "#4a1a00"; border.color: "#FF6600"; border.width: 2
                            scale: stopResetMA.pressed ? 0.95 : 1.0
                            Behavior on scale { NumberAnimation { duration: 100 } }
                            Text { anchors.centerIn: parent; text: "⏹ STOP"; color: "#FF6600"; font.pixelSize: 20; font.bold: true }
                            MouseArea { id: stopResetMA; anchors.fill: parent; onClicked: { cameraPageRoot.modeLocked = false; robotController.softStopAndManual(); cartridgeController.softStop() } }
                        }

                        Rectangle { Layout.fillWidth: true; height: 52; radius: 5; color: enMA.pressed ? Qt.darker("#0a2a1a", 1.2) : "#0a2a1a"; border.color: "#10b981"; border.width: 2
                            scale: enMA.pressed ? 0.95 : 1.0
                            Behavior on scale { NumberAnimation { duration: 100 } }
                            Text { anchors.centerIn: parent; text: "ENABLE"; color: "#10b981"; font.pixelSize: 21; font.bold: true }
                            MouseArea { id: enMA; anchors.fill: parent; onClicked: robotController.enableSystem(true) }
                        }

                        Rectangle { Layout.fillWidth: true; height: 52; radius: 5; color: startMA.pressed ? Qt.darker("#0d3320", 1.3) : "#0d3320"; border.color: "#22c55e"; border.width: 2
                            scale: startMA.pressed ? 0.95 : 1.0
                            Behavior on scale { NumberAnimation { duration: 100 } }
                            Behavior on color { ColorAnimation { duration: 100 } }
                            Text { anchors.centerIn: parent; text: "🚀 START"; color: "#22c55e"; font.pixelSize: 20; font.bold: true }
                            MouseArea { id: startMA; anchors.fill: parent; onClicked: {
                                if (cameraPageRoot.ctrlMode === "camera_ai") {
                                    robotController.selectRow(0)
                                    robotController.setAiMode(true)
                                    hpController.publishMode(0) // sync Fill HP → Auto
                                } else if (cameraPageRoot.ctrlMode === "auto") {
                                    if (robotController.selectedRow <= 0) {
                                        robotController.selectRow(1)
                                    }
                                    robotController.setAutoMode(true)
                                    hpController.publishMode(0) // sync Fill HP → Auto
                                }
                                cameraPageRoot.modeLocked = true
                                robotController.startSystem(true)
                            } }
                        }

                        Rectangle { Layout.fillWidth: true; height: 52; radius: 5; color: pauseMA.pressed ? Qt.darker("#1a1a00", 1.2) : "#1a1a00"; border.color: "#f59e0b"; border.width: 2
                            scale: pauseMA.pressed ? 0.95 : 1.0
                            Behavior on scale { NumberAnimation { duration: 100 } }
                            Text { anchors.centerIn: parent; text: "PAUSE"; color: "#f59e0b"; font.pixelSize: 21; font.bold: true }
                            MouseArea { id: pauseMA; anchors.fill: parent; onClicked: robotController.pauseRobot() }
                        }

                        Rectangle { Layout.fillWidth: true; height: 52; radius: 5; color: resMA.pressed ? Qt.darker("#0a1a2a", 1.2) : "#0a1a2a"; border.color: "#5cf4f1"; border.width: 2
                            scale: resMA.pressed ? 0.95 : 1.0
                            Behavior on scale { NumberAnimation { duration: 100 } }
                            Text { anchors.centerIn: parent; text: "RESUME"; color: "#5cf4f1"; font.pixelSize: 21; font.bold: true }
                            MouseArea { id: resMA; anchors.fill: parent; onClicked: robotController.resumeRobot() }
                        }

                        Rectangle { Layout.fillWidth: true; height: 52; radius: 5; color: clrMA.pressed ? Qt.darker("#0a1a2a", 1.2) : "#0a1a2a"; border.color: "#4da6ff"; border.width: 2
                            scale: clrMA.pressed ? 0.95 : 1.0
                            Behavior on scale { NumberAnimation { duration: 100 } }
                            Text { anchors.centerIn: parent; text: "CLEAR ERR"; color: "#4da6ff"; font.pixelSize: 21; font.bold: true }
                            MouseArea { id: clrMA; anchors.fill: parent; onClicked: robotController.clearError() }
                        }
                    }
                }
            }
        }

        // ── Footer ─────────────────────────────────────────────
        Item {
            height: 40; Layout.fillWidth: true
            Rectangle {
                anchors.fill: parent; color: "#0d2538"; border.color: "#134357"
                RowLayout {
                    anchors.fill: parent; anchors.margins: 10
                    Text { text: "© 2025 RYNAN TECHNOLOGIES"; color: "#6cf"; font.pixelSize: 16; Layout.alignment: Qt.AlignVCenter }
                    Item { Layout.fillWidth: true }
                    RowLayout { spacing: 6
                        Image { source: "qrc:/icons/qml/icons/app_badging.svg"; width: 24; height: 24; fillMode: Image.PreserveAspectFit; smooth: true; Layout.preferredWidth: 24; Layout.preferredHeight: 24; Layout.alignment: Qt.AlignVCenter }
                        Text { text: "Status: Running"; color: "#00ff99"; font.pixelSize: 16; Layout.alignment: Qt.AlignVCenter }
                    }
                    Rectangle { width: 2; Layout.fillHeight: true; color: "#134357" }
                    RowLayout { spacing: 6; Layout.alignment: Qt.AlignVCenter
                        Image { source: "qrc:/icons/qml/icons/schedule.svg"; fillMode: Image.PreserveAspectFit; smooth: true; Layout.preferredWidth: 24; Layout.preferredHeight: 24; Layout.alignment: Qt.AlignVCenter }
                        Text { text: currentTime; font.pixelSize: 16; color: "#6cf"; Layout.alignment: Qt.AlignVCenter }
                    }
                }
            }
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
        Layout.fillHeight: true
        implicitHeight: 50
        radius: 6
        color:        cls === "ok"    ? cOkBg
                    : cls === "high"  ? cWarnBg
                    : cls === "limit" ? cBadBg
                    : cPanel2
        border.color: cls === "ok"    ? cOk
                    : cls === "high"  ? cWarn
                    : cls === "limit" ? cBad
                    : cBorder
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
                             : "#0284c7"
                    }
                }
            }
        }
    }

}
