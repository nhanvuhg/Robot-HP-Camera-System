import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls.Material 2.15
import QtQuick.Window 2.15

ApplicationWindow {
    id: mainWindow
    visible: true
    width: 1920
    height: 1080
    title: "ROS2 - Unified Control System"
    color: "#0c0c1d"
    visibility: Window.FullScreen

    property bool scaleIssueWarning: false

    Shortcut {
        sequence: "F11"
        onActivated: {
            if (mainWindow.visibility === Window.FullScreen)
                mainWindow.visibility = Window.Windowed
            else
                mainWindow.visibility = Window.FullScreen
        }
    }

    StackView {
        id: stackView
        anchors.fill: parent
        initialItem: cameraPage

        Component {
            id: cameraPage
            CameraPage {}
        }
        Component {
            id: cartridgePage
            CartridgePage {}
        }
    }

    // ────────────────────────────────────────────────────────────
    // GLOBAL POPUP — feed_chamber timeout resume choice
    // ────────────────────────────────────────────────────────────
    // GLOBAL POPUP — feed_chamber timeout resume choice
    // Robot_logic_node publishes systemStatus = "WAIT_RESUME_CHOICE" when
    // LOAD_CHAMBER_FROM_BUFFER times out (150s) + SCALE has been drained to
    // PLACE. Operator chooses 1 of 2 ways to resume or stops the system.
    // Popup is displayed globally (on all pages) so operator does not miss it.
    // ────────────────────────────────────────────────────────────
    Connections {
        target: robotController
        function onSystemStatusChanged() {
            var s = robotController.systemStatus
            if (s === "WAIT_RESUME_CHOICE") {
                scaleChoicePopup.close()
                confirmEmptyBufferPopup.close()
                resumeChoicePopup.open()
            } else if (s === "WAIT_SCALE_CHOICE") {
                resumeChoicePopup.close()
                confirmEmptyBufferPopup.close()
                scaleChoicePopup.open()
            } else {
                resumeChoicePopup.close()
                confirmEmptyBufferPopup.close()
                scaleChoicePopup.close()
            }
        }
    }

    Popup {
        id: resumeChoicePopup
        parent: Overlay.overlay
        anchors.centerIn: parent
        modal: true
        closePolicy: Popup.NoAutoClose
        width: 620; height: 400
        background: Rectangle {
            color: "#081e29"
            border.color: "#f59e0b"
            border.width: 2
            radius: 10
        }

        Timer {
            id: resumeStopHomingTimer
            interval: 500
            repeat: false
            onTriggered: cartridgeController.gotoState("HOMING")
        }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 22
            spacing: 14

            Text {
                Layout.alignment: Qt.AlignHCenter
                text: "⏸  RESUME REQUIRED"
                color: "#f59e0b"
                font.pixelSize: 26
                font.bold: true
            }
            Text {
                Layout.fillWidth: true
                text: "feed_chamber timeout 150s — SCALE has been drained to PLACE.\nSelect how to resume the cycle:"
                color: "#e8e8f0"
                font.pixelSize: 17
                wrapMode: Text.WordWrap
                horizontalAlignment: Text.AlignHCenter
            }
            Item { Layout.fillHeight: true }

            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                spacing: 16

                Button {
                    Layout.preferredWidth: 250; Layout.preferredHeight: 60
                    text: "🔁  LOAD CHAMBER\nFROM BUFFER"
                    font.pixelSize: 15; font.bold: true
                    background: Rectangle { color: "#1e2a80"; border.color: "#818cf8"; border.width: 2; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#c7d2fe"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: {
                        robotController.gotoState("LOAD_CHAMBER_FROM_BUFFER")
                        resumeChoicePopup.close()
                    }
                }
                Button {
                    Layout.preferredWidth: 250; Layout.preferredHeight: 60
                    text: "🔂  LOAD CHAMBER\nFROM TRAY"
                    font.pixelSize: 15; font.bold: true
                    background: Rectangle { color: "#7a4a00"; border.color: "#fbbf24"; border.width: 2; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#fde68a"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: confirmEmptyBufferPopup.open()
                }
            }

            Button {
                Layout.alignment: Qt.AlignHCenter
                Layout.preferredWidth: 516; Layout.preferredHeight: 52
                text: "⏹  STOP & HOME  —  Dừng hệ thống và đưa robot về Home"
                font.pixelSize: 15; font.bold: true
                background: Rectangle { color: "#4d1a1a"; border.color: "#ef4444"; border.width: 2; radius: 6 }
                contentItem: Text {
                    text: parent.text; color: "#ef4444"
                    font: parent.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                onClicked: {
                    robotController.stopAndResetRobot()
                    cartridgeController.stopSystem()
                    resumeStopHomingTimer.start()
                    resumeChoicePopup.close()
                }
            }
        }
    }

    Popup {
        id: confirmEmptyBufferPopup
        parent: Overlay.overlay
        anchors.centerIn: parent
        modal: true
        closePolicy: Popup.NoAutoClose
        width: 580; height: 320
        background: Rectangle {
            color: "#081e29"
            border.color: "#ef4444"
            border.width: 2
            radius: 10
        }
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 22
            spacing: 14

            Text {
                Layout.alignment: Qt.AlignHCenter
                text: "⚠  CONFIRM EMPTY BUFFER"
                color: "#ef4444"
                font.pixelSize: 24
                font.bold: true
            }
            Text {
                Layout.fillWidth: true
                text: "LOAD CHAMBER FROM TRAY will restart like a fresh boot:\n" +
                      "  INIT_LOAD → INIT_REFILL_BUFFER → cycle.\n\n" +
                      "Have you manually removed all cartridges from the BUFFER?"
                color: "#e8e8f0"
                font.pixelSize: 16
                wrapMode: Text.WordWrap
                horizontalAlignment: Text.AlignHCenter
            }
            Item { Layout.fillHeight: true }

            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                spacing: 16

                Button {
                    Layout.preferredWidth: 240; Layout.preferredHeight: 56
                    text: "✓  Buffer is empty — CONFIRM"
                    font.pixelSize: 15; font.bold: true
                    background: Rectangle { color: "#10b981"; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#001100"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: {
                        robotController.gotoState("INIT_LOAD_CHAMBER_DIRECT")
                        confirmEmptyBufferPopup.close()
                        resumeChoicePopup.close()
                    }
                }
                Button {
                    Layout.preferredWidth: 240; Layout.preferredHeight: 56
                    text: "✗  Cancel / Back"
                    font.pixelSize: 15; font.bold: true
                    background: Rectangle { color: "#374151"; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#e8e8f0"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: confirmEmptyBufferPopup.close()
                }
            }
        }
    }

    // ────────────────────────────────────────────────────────────
    // GLOBAL POPUP — loadcell silent 150s in PROCESSING_SCALE
    // Operator chọn 1 trong 3 cách xử lý: WAIT_FILLING / PLACE_OUTPUT / PLACE_FAIL
    // ────────────────────────────────────────────────────────────
    Popup {
        id: scaleChoicePopup
        parent: Overlay.overlay
        anchors.centerIn: parent
        modal: true
        closePolicy: Popup.NoAutoClose
        width: 660; height: 430
        background: Rectangle {
            color: "#081e29"
            border.color: "#ef4444"
            border.width: 2
            radius: 10
        }

        Timer {
            id: scaleStopHomingTimer
            interval: 500
            repeat: false
            onTriggered: cartridgeController.gotoState("HOMING")
        }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 22
            spacing: 14

            Text {
                Layout.alignment: Qt.AlignHCenter
                text: "⚠  SCALE ISSUE DETECTED"
                color: "#ef4444"
                font.pixelSize: 26
                font.bold: true
            }
            Text {
                Layout.fillWidth: true
                text: "No loadcell topic received for 150s in PROCESSING_SCALE.\nSelect how to handle this cartridge:"
                color: "#e8e8f0"
                font.pixelSize: 16
                wrapMode: Text.WordWrap
                horizontalAlignment: Text.AlignHCenter
            }
            Item { Layout.fillHeight: true }

            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                spacing: 12

                Button {
                    Layout.preferredWidth: 196; Layout.preferredHeight: 72
                    text: "↩  BACK TO\nWAIT FILLING\n(đã lấy cartridge ra)"
                    font.pixelSize: 13; font.bold: true
                    background: Rectangle { color: "#0f2a4a"; border.color: "#60a5fa"; border.width: 2; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#93c5fd"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: {
                        robotController.gotoState("WAIT_FILLING")
                        mainWindow.scaleIssueWarning = true
                        scaleChoicePopup.close()
                    }
                }
                Button {
                    Layout.preferredWidth: 196; Layout.preferredHeight: 72
                    text: "✓  PLACE TO\nOUTPUT\n(force PASS)"
                    font.pixelSize: 13; font.bold: true
                    background: Rectangle { color: "#052e16"; border.color: "#4ade80"; border.width: 2; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#86efac"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: {
                        robotController.gotoState("PLACE_TO_OUTPUT")
                        mainWindow.scaleIssueWarning = true
                        scaleChoicePopup.close()
                    }
                }
                Button {
                    Layout.preferredWidth: 196; Layout.preferredHeight: 72
                    text: "✗  PLACE TO\nFAIL\n(force FAIL)"
                    font.pixelSize: 13; font.bold: true
                    background: Rectangle { color: "#2d0a0a"; border.color: "#f87171"; border.width: 2; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#fca5a5"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: {
                        robotController.gotoState("PLACE_TO_FAIL")
                        mainWindow.scaleIssueWarning = true
                        scaleChoicePopup.close()
                    }
                }
            }

            Button {
                Layout.alignment: Qt.AlignHCenter
                Layout.preferredWidth: 612; Layout.preferredHeight: 52
                text: "⏹  STOP & HOME  —  Dừng hệ thống và đưa robot về Home"
                font.pixelSize: 15; font.bold: true
                background: Rectangle { color: "#1a0808"; border.color: "#ef4444"; border.width: 2; radius: 6 }
                contentItem: Text {
                    text: parent.text; color: "#ef4444"
                    font: parent.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                onClicked: {
                    robotController.stopAndResetRobot()
                    cartridgeController.stopSystem()
                    mainWindow.scaleIssueWarning = true
                    scaleStopHomingTimer.start()
                    scaleChoicePopup.close()
                }
            }
        }
    }

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

    function checkInkAndRun(callback) {
        var inkMap = parseKvPipe(hpController.inkStatus);
        var code = inkMap["CODE"] || "";
        var lot_ci = inkMap["LOT_CI"] || "";
        
        var sysMap = parseKvPipe(hpController.systemStatus);
        var modeStr = (sysMap["MODE"] || hpController.modeStatus || "").toString().trim().toUpperCase();

        var isAutoOrPrefill = (modeStr === "AUTO" || modeStr === "PREFILL" || modeStr === "1" || modeStr === "2");
        var isInkEmpty = (code.trim() === "" || lot_ci.trim() === "");

        if (isAutoOrPrefill && isInkEmpty) {
            notYetInkSelectedPopup.confirmCallback = callback;
            notYetInkSelectedPopup.open();
        } else {
            callback();
        }
    }

    Popup {
        id: notYetInkSelectedPopup
        parent: Overlay.overlay
        anchors.centerIn: parent
        modal: true
        closePolicy: Popup.NoAutoClose
        width: 600; height: 320
        background: Rectangle {
            color: "#0c0c1d"
            border.color: "#ef4444"
            border.width: 2
            radius: 10
        }
        
        property var confirmCallback: null
        
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 25
            spacing: 15
            
            Text {
                text: "⚠️ WARNING: INK NOT SELECTED"
                color: "#ef4444"
                font.pixelSize: 24
                font.bold: true
                Layout.alignment: Qt.AlignHCenter
            }
            Text {
                text: "Ink or Lot has not been selected for the system.\nIf you continue running, production and consumption logs WILL NOT BE SAVED."
                color: "#e8e8f0"
                font.pixelSize: 16
                Layout.alignment: Qt.AlignHCenter
                horizontalAlignment: Text.AlignHCenter
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }
            Item { Layout.fillHeight: true }
            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                spacing: 20
                Button {
                    text: "✓ RUN (NO LOGGING)"
                    font.pixelSize: 16
                    font.bold: true
                    onClicked: {
                        notYetInkSelectedPopup.close()
                        if (notYetInkSelectedPopup.confirmCallback) notYetInkSelectedPopup.confirmCallback()
                    }
                    background: Rectangle { radius: 6; color: "#ef4444" }
                    contentItem: Text {
                        text: parent.text; color: "#ffffff"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }
                Button {
                    text: "✗ CANCEL"
                    font.pixelSize: 16
                    font.bold: true
                    onClicked: notYetInkSelectedPopup.close()
                    background: Rectangle { radius: 6; color: "#374151" }
                    contentItem: Text {
                        text: parent.text; color: "#e8e8f0"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }
            }
        }
    }
}
