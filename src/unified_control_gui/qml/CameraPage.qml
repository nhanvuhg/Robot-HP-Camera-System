import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls.Material 2.15

Item {
    id: cameraPageRoot

    property string currentTime: Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
    property bool rowLocked: false
    property string ctrlMode: "auto"  // "auto" | "camera_ai"

    // Unlock row selection khi process idle/done
    Connections {
        target: robotController
        function onSystemStatusChanged() {
            var s = (robotController.systemStatus || "").toLowerCase()
            if (s === "idle" || s === "ready" || s === "") {
                cameraPageRoot.rowLocked = false
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

            // Camera Area + Log stacked vertically
            ColumnLayout {
                Layout.fillHeight: true
                Layout.fillWidth: true
                spacing: 8

                // Camera Grid — fill height, cameras expand
                Rectangle {
                    color: "#081e29"
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    border.color: "#134357"
                    radius: 6

                    GridLayout {
                        id: camGrid
                        columns: 2
                        rowSpacing: 10; columnSpacing: 10
                        anchors.fill: parent; anchors.margins: 10

                        Repeater {
                            model: camNode.cameraList
                            delegate: CameraView {
                                cameraName: modelData.name
                                topic: modelData.topic
                                providerId: modelData.providerId
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                            }
                        }
                    }
                }

                // Log bar below cameras — compact fixed height
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 120
                    color: "#081e29"; border.color: "#134357"; radius: 6

                    Column {
                        anchors { fill: parent; margins: 8 }
                        spacing: 4
                        RowLayout {
                            width: parent.width; height: 18
                            Text { text: "ROBOT LOG"; color: "#5cf4f1"; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.5 }
                            Item { Layout.fillWidth: true }
                            Rectangle { width: 50; height: 18; radius: 4; color: "#0d2538"; border.color: "#134357"
                                Text { anchors.centerIn: parent; text: "Clear"; color: "#94a3b8"; font.pixelSize: 10 }
                                MouseArea { anchors.fill: parent; onClicked: robotController.clearLog() }
                            }
                        }
                        Rectangle {
                            width: parent.width; height: parent.height - 22
                            color: "#040f15"; border.color: "#134357"; radius: 4
                            ListView {
                                anchors { fill: parent; margins: 6 }
                                model: robotController.logEntries
                                clip: true; spacing: 2
                                verticalLayoutDirection: ListView.BottomToTop
                                delegate: Text {
                                    width: parent ? parent.width : 100
                                    text: "[" + modelData.time + "] " + modelData.msg
                                    font.pixelSize: 12; font.family: "monospace"
                                    color: modelData.type==="err" ? "#ef4444" : modelData.type==="ok" ? "#10b981" : "#5cf4f1"
                                    wrapMode: Text.WordWrap
                                }
                            }
                        }
                    }
                }
            }

            // ── System Monitor + Controls ──────────────────────
            Rectangle {
                Layout.fillHeight: true
                Layout.preferredWidth: 350
                Layout.minimumWidth: 300
                color: "#081e29"
                border.color: "#134357"
                radius: 6

                Flickable {
                    anchors.fill: parent
                    anchors.margins: 10
                    contentHeight: sysCol.implicitHeight
                    clip: true

                    Column {
                        id: sysCol
                        width: parent.width
                        spacing: 12

                        // ── Header ──────────────────────────
                        Text {
                            text: "SYSTEM MONITOR"
                            color: "#5cf4f1"; font.bold: true; font.pixelSize: 20
                            anchors.horizontalCenter: parent.horizontalCenter
                        }

                        Rectangle { width: parent.width; height: 1; color: "#134357" }

                        // ── Status Info ──────────────────────
                        GridLayout {
                            width: parent.width
                            columns: 2; rowSpacing: 8; columnSpacing: 10

                            Text { text: "Status:";    color: "#94a3b8"; font.pixelSize: 16 }
                            Text { text: robotController.systemStatus; color: "#10b981"; font.bold: true; font.pixelSize: 16 }

                            Text { text: "Mode:";      color: "#94a3b8"; font.pixelSize: 16 }
                            Text { text: cameraPageRoot.ctrlMode.toUpperCase(); color: cameraPageRoot.ctrlMode === "camera_ai" ? "#5cf4f1" : "#4f6cff"; font.bold: true; font.pixelSize: 16 }

                            Text { text: "Uptime:";    color: "#94a3b8"; font.pixelSize: 16 }
                            Text { text: robotController.systemUptime; color: "#f59e0b"; font.bold: true; font.pixelSize: 16 }

                            Text { text: "Tray:";      color: "#94a3b8"; font.pixelSize: 16 }
                            Text { text: robotController.trayCount; color: "#8b5cf6"; font.bold: true; font.pixelSize: 16 }
                        }

                        // ── Cartridge State ──────────────────
                        Text { text: "CARTRIDGE SYSTEM"; color: "#ffa726"; font.bold: true; font.pixelSize: 15; anchors.horizontalCenter: parent.horizontalCenter }

                        GridLayout {
                            width: parent.width
                            columns: 2; rowSpacing: 6; columnSpacing: 10

                            Text { text: "State:"; color: "#94a3b8"; font.pixelSize: 14 }
                            Text { text: cartridgeController.systemState; color: "#4f6cff"; font.bold: true; font.pixelSize: 14 }

                            Text { text: "Mode:";  color: "#94a3b8"; font.pixelSize: 14 }
                            Text { text: cartridgeController.currentMode.toUpperCase(); color: "#6c5ce7"; font.bold: true; font.pixelSize: 14 }
                        }

                        Rectangle { width: parent.width; height: 1; color: "#134357" }

                        // ══════════════════════════════════════
                        // CONTROL SECTION (below monitor info)
                        // ══════════════════════════════════════

                        // Control Mode Toggle
                        Text { text: "CONTROL MODE"; color: "#5cf4f1"; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1 }
                        Row { spacing: 6; width: parent.width
                            Repeater {
                                model: [
                                    { key: "auto",      lbl: "AUTO",      bc: "#4f6cff" },
                                    { key: "camera_ai", lbl: "CAMERA AI", bc: "#5cf4f1" }
                                ]
                                delegate: Rectangle {
                                    required property var modelData
                                    width: (sysCol.width - 6) / 2; height: 46; radius: 5
                                    color: cameraPageRoot.ctrlMode === modelData.key ? modelData.bc + "33" : "#0d2538"
                                    border.color: cameraPageRoot.ctrlMode === modelData.key ? modelData.bc : "#134357"; border.width: 2
                                    Text { anchors.centerIn: parent; text: modelData.lbl; color: cameraPageRoot.ctrlMode === modelData.key ? modelData.bc : "#94a3b8"; font.pixelSize: 14; font.bold: true }
                                    MouseArea { anchors.fill: parent; onClicked: cameraPageRoot.ctrlMode = modelData.key }
                                }
                            }
                        }

                        // Input Row
                        Row { spacing: 6
                            Text { text: "INPUT ROW"; color: "#5cf4f1"; font.pixelSize: 11; font.bold: true; anchors.verticalCenter: parent.verticalCenter }
                            Text {
                                text: cameraPageRoot.rowLocked ? "🔒 Đang thực hiện" : (cameraPageRoot.ctrlMode === "camera_ai" ? "(AI auto)" : "Chọn rồi nhấn PICK_INPUT")
                                color: cameraPageRoot.rowLocked ? "#ef4444" : "#6b7280"
                                font.pixelSize: 10; anchors.verticalCenter: parent.verticalCenter
                            }
                        }
                        Grid { columns: 5; spacing: 5; width: parent.width
                            Repeater { model: 5
                                delegate: Rectangle {
                                    property int rn: index + 1
                                    property bool isActive: robotController.selectedRow === rn
                                    property bool canSelect: !cameraPageRoot.rowLocked && cameraPageRoot.ctrlMode === "auto"
                                    width: (sysCol.width - 20) / 5; height: 52; radius: 5
                                    // Active = đang xử lý (sáng xanh)
                                    color: isActive ? "#0a4020" : "#0d2538"
                                    border.color: isActive ? "#10b981" : "#134357"
                                    border.width: isActive ? 2 : 1
                                    opacity: canSelect ? 1.0 : (isActive ? 1.0 : 0.45)
                                    Rectangle {
                                        visible: isActive
                                        anchors { top: parent.top; left: parent.left; right: parent.right }
                                        height: 3; radius: 2
                                        color: "#10b981"
                                    }
                                    Column { anchors.centerIn: parent; spacing: 2
                                        Text { text: "R" + rn; color: isActive ? "#10b981" : "#94a3b8"; font.pixelSize: 14; font.bold: isActive; anchors.horizontalCenter: parent.horizontalCenter }
                                    }
                                    MouseArea {
                                        anchors.fill: parent
                                        enabled: canSelect
                                        onClicked: robotController.selectRow(rn)
                                    }
                                }
                            }
                        }

                        // State Commands
                        Text { text: "STATE COMMANDS"; color: "#5cf4f1"; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1 }
                        Grid { columns: 2; spacing: 6; width: parent.width
                            Repeater {
                                model: [
                                    { lbl: "IN_READY",     icon: "📥", bg: "#351a0a", bc: "#ffaa4f" },
                                    { lbl: "OUT_READY",    icon: "📤", bg: "#351a0a", bc: "#ffaa4f" },
                                    { lbl: "PICK_INPUT",   icon: "↓", bg: "#0a1a35", bc: "#4f6cff" },
                                    { lbl: "PICK_CHAMBER", icon: "⟳", bg: "#0a1a35", bc: "#4f6cff" },
                                    { lbl: "HOME",         icon: "⌂", bg: "#0d2538", bc: "#334155" },
                                    { lbl: "IDLE",         icon: "◌", bg: "#0d2538", bc: "#334155" }
                                ]
                                delegate: Rectangle {
                                    required property var modelData
                                    width: (sysCol.width - 6) / 2; height: 48; radius: 5
                                    color: ma.pressed ? Qt.darker(modelData.bg, 1.3) : modelData.bg; border.color: ma.pressed ? Qt.lighter(modelData.bc, 1.2) : modelData.bc; border.width: ma.pressed ? 3 : 2
                                    scale: ma.pressed ? 0.95 : 1.0
                                    Behavior on color { ColorAnimation { duration: 100 } }
                                    Behavior on scale { NumberAnimation { duration: 100 } }
                                    Row { anchors.centerIn: parent; spacing: 5
                                        Text { text: modelData.icon; color: modelData.bc; font.pixelSize: 16; anchors.verticalCenter: parent.verticalCenter }
                                        Text { text: modelData.lbl; color: modelData.bc; font.pixelSize: 12; font.bold: true; anchors.verticalCenter: parent.verticalCenter }
                                    }
                                    MouseArea { id: ma; anchors.fill: parent; onClicked: {
                                        if (modelData.lbl === "IN_READY") robotController.simulateInputTrayReady()
                                        else if (modelData.lbl === "OUT_READY") robotController.simulateOutputTrayReady()
                                        else if (modelData.lbl === "PICK_INPUT") { cameraPageRoot.rowLocked = true; robotController.simulateFeedChamber() }
                                        else if (modelData.lbl === "PICK_CHAMBER") robotController.simulateFillDone()
                                        else robotController.gotoState(modelData.lbl)
                                    }}
                                }
                            }
                        }

                        // System Controls
                        Text { text: "SYSTEM CONTROL"; color: "#5cf4f1"; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1 }

                        Rectangle { width: parent.width; height: 56; radius: 5; color: emMA.pressed ? Qt.darker("#4d1a1a", 1.2) : "#4d1a1a"; border.color: "#ef4444"; border.width: 2
                            scale: emMA.pressed ? 0.95 : 1.0
                            Behavior on scale { NumberAnimation { duration: 100 } }
                            Text { anchors.centerIn: parent; text: "⛔ EMERGENCY STOP"; color: "#ef4444"; font.pixelSize: 15; font.bold: true }
                            MouseArea { id: emMA; anchors.fill: parent; onClicked: robotController.emergencyStop(true) }
                        }

                        Rectangle { width: parent.width; height: 52; radius: 5; color: stopResetMA.pressed ? Qt.darker("#4a1a00", 1.2) : "#4a1a00"; border.color: "#FF6600"; border.width: 2
                            scale: stopResetMA.pressed ? 0.95 : 1.0
                            Behavior on scale { NumberAnimation { duration: 100 } }
                            Text { anchors.centerIn: parent; text: "⏹ STOP & RESET → MODE 4"; color: "#FF6600"; font.pixelSize: 14; font.bold: true }
                            MouseArea { id: stopResetMA; anchors.fill: parent; onClicked: robotController.stopAndResetRobot() }
                        }

                        Row { spacing: 6; width: parent.width
                            Rectangle { width: (parent.width - 6) / 2; height: 46; radius: 5; color: enMA.pressed ? Qt.darker("#0a2a1a", 1.2) : "#0a2a1a"; border.color: "#10b981"; border.width: 2
                                scale: enMA.pressed ? 0.95 : 1.0
                                Behavior on scale { NumberAnimation { duration: 100 } }
                                Text { anchors.centerIn: parent; text: "ENABLE";  color: "#10b981"; font.pixelSize: 13; font.bold: true }
                                MouseArea { id: enMA; anchors.fill: parent; onClicked: robotController.enableSystem(true) }
                            }
                            Rectangle { width: (parent.width - 6) / 2; height: 46; radius: 5; color: disMA.pressed ? Qt.darker("#0d2538", 1.2) : "#0d2538"; border.color: "#334155"; border.width: 2
                                scale: disMA.pressed ? 0.95 : 1.0
                                Behavior on scale { NumberAnimation { duration: 100 } }
                                Text { anchors.centerIn: parent; text: "DISABLE"; color: "#94a3b8"; font.pixelSize: 13; font.bold: true }
                                MouseArea { id: disMA; anchors.fill: parent; onClicked: robotController.enableSystem(false) }
                            }
                        }

                        Rectangle { width: parent.width; height: 52; radius: 5; color: startMA.pressed ? Qt.darker("#0d3320", 1.3) : "#0d3320"; border.color: "#22c55e"; border.width: 2
                            scale: startMA.pressed ? 0.95 : 1.0
                            Behavior on scale { NumberAnimation { duration: 100 } }
                            Behavior on color { ColorAnimation { duration: 100 } }
                            Text { anchors.centerIn: parent; text: "🚀 START"; color: "#22c55e"; font.pixelSize: 15; font.bold: true }
                            MouseArea { id: startMA; anchors.fill: parent; onClicked: robotController.startSystem(true) }
                        }

                        Row { spacing: 6; width: parent.width
                            Rectangle { width: (parent.width - 6) / 2; height: 46; radius: 5; color: pauseMA.pressed ? Qt.darker("#1a1a00", 1.2) : "#1a1a00"; border.color: "#f59e0b"; border.width: 2
                                scale: pauseMA.pressed ? 0.95 : 1.0
                                Behavior on scale { NumberAnimation { duration: 100 } }
                                Text { anchors.centerIn: parent; text: "PAUSE";  color: "#f59e0b"; font.pixelSize: 13; font.bold: true }
                                MouseArea { id: pauseMA; anchors.fill: parent; onClicked: robotController.pauseRobot() }
                            }
                            Rectangle { width: (parent.width - 6) / 2; height: 46; radius: 5; color: resMA.pressed ? Qt.darker("#0a1a2a", 1.2) : "#0a1a2a"; border.color: "#5cf4f1"; border.width: 2
                                scale: resMA.pressed ? 0.95 : 1.0
                                Behavior on scale { NumberAnimation { duration: 100 } }
                                Text { anchors.centerIn: parent; text: "RESUME"; color: "#5cf4f1"; font.pixelSize: 13; font.bold: true }
                                MouseArea { id: resMA; anchors.fill: parent; onClicked: robotController.resumeRobot() }
                            }
                        }

                        Rectangle { width: parent.width; height: 46; radius: 5; color: clrMA.pressed ? Qt.darker("#0a1a2a", 1.2) : "#0a1a2a"; border.color: "#4da6ff"; border.width: 2
                            scale: clrMA.pressed ? 0.95 : 1.0
                            Behavior on scale { NumberAnimation { duration: 100 } }
                            Text { anchors.centerIn: parent; text: "CLEAR ERROR"; color: "#4da6ff"; font.pixelSize: 13; font.bold: true }
                            MouseArea { id: clrMA; anchors.fill: parent; onClicked: robotController.clearError() }
                        }

                        // Error indicator — only shown when there IS an error
                        Rectangle {
                            width: parent.width; height: robotController.errorLog.length > 0 ? 30 : 0; radius: 4
                            visible: robotController.errorLog.length > 0
                            color: "#1a0a0a"; border.color: "#ef4444"
                            Row { anchors.fill: parent; anchors.leftMargin: 6; spacing: 4
                                Text { text: "⚠"; color: "#ef4444"; font.pixelSize: 10; anchors.verticalCenter: parent.verticalCenter }
                                Text { text: robotController.errorLog; color: "#FFD700"; font.pixelSize: 10; font.family: "monospace"; anchors.verticalCenter: parent.verticalCenter; elide: Text.ElideRight; width: parent.width - 30 }
                            }
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
