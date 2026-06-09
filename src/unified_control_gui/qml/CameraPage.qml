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

    // Tính từ systemStatus — chặn đổi mode khi robot đang chạy
    // MANUAL được coi là "rảnh" (không busy) — robot chỉ chờ lệnh thủ công, cho phép đổi mode.
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

    // Sync mode từ Cartridge (đảm bảo 2 UI đồng bộ)
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

                    Button {
                        text: "INK SYSTEM  ▸"
                        Layout.preferredHeight: 50
                        font.pixelSize: 16; font.bold: true
                        onClicked: stackView.push(inkPage)
                        background: Rectangle {
                            radius: 6
                            gradient: Gradient {
                                GradientStop { position: 0.0; color: "#00bcd4" }
                                GradientStop { position: 1.0; color: "#006064" }
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

                    // ── 3-Column Layout: Trái (Trạng thái Robot), Giữa (Thông tin Mực), Phải (Chế độ Điều khiển) ──
                    RowLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: false
                        spacing: 20

                        // CỘT 1: Trạng thái Robot
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

                        // CỘT 2: Thông tin mực & cân
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

                        // CỘT 3: Control Mode
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
                            text: cameraPageRoot.rowLocked ? "🔒 Chờ lấy khay..." : (cameraPageRoot.ctrlMode === "camera_ai" ? "(AI auto)" : "Chọn rồi nhấn PICK_INPUT")
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
                                property bool canSelect: !cameraPageRoot.rowLocked && cameraPageRoot.ctrlMode === "auto"
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
                            text: robotController.selectedSlot > 0 ? ("Đã chọn slot " + robotController.selectedSlot) : "Chọn vị trí đặt khay output"
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
                                property bool canSelect: cameraPageRoot.ctrlMode === "auto"
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

}
