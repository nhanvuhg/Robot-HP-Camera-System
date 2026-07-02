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
    color: "#06101d"
    visibility: Window.FullScreen

    property bool scaleIssueWarning: false

    signal synchronizedModeRequested(string mode)
    signal synchronizedStartRequested(string mode)
    signal synchronizedStopRequested()

    function cartridgeModeFor(mode) {
        var m = (mode || "").toString().trim().toLowerCase()
        if (m === "auto")
            return "auto"
        if (m === "ai" || m === "camera_ai")
            return "ai"
        if (m === "jog")
            return "jog"
        return "manual"
    }

    function robotModeFor(mode) {
        var m = (mode || "").toString().trim().toLowerCase()
        if (m === "camera_ai" || m === "ai")
            return "ai"
        if (m === "auto")
            return "auto"
        return "manual"
    }

    function cartridgeCommandModeFor(mode) {
        var m = (mode || "").toString().trim().toLowerCase()
        return cartridgeModeFor(m)
    }

    function syncOperationMode(mode) {
        var cartridgeMode = cartridgeModeFor(mode)
        var cartridgeCommandMode = cartridgeCommandModeFor(mode)
        var robotMode = robotModeFor(mode)

        synchronizedModeRequested(mode)
        cartridgeController.setMode(cartridgeCommandMode)

        if (robotMode === "ai")
            robotController.setAiMode(true)
        else if (robotMode === "auto")
            robotController.setAutoMode(true)
        else
            robotController.setManualMode(true)

        return cartridgeMode
    }

    function startSynchronizedSystems(mode) {
        var cartridgeMode = syncOperationMode(mode)
        hpController.publishMode((cartridgeMode === "auto" || cartridgeMode === "ai") ? 0 : 2)
        synchronizedStartRequested(mode)
        robotController.startSystem(true)
    }

    function stopSynchronizedSystems() {
        synchronizedStopRequested()
        robotController.stopAndResetRobot()
        cartridgeController.stopSystem()
    }

    function emergencyStopSynchronizedSystems() {
        synchronizedStopRequested()
        robotController.emergencyStop(true)
        cartridgeController.stopSystem()
    }

    Shortcut {
        sequence: "F11"
        onActivated: {
            if (mainWindow.visibility === Window.FullScreen)
                mainWindow.visibility = Window.Windowed
            else
                mainWindow.visibility = Window.FullScreen
        }
    }

    Image {
        id: bgWallpaper
        anchors.fill: parent
        source: "qrc:/icons/qml/icons/bg_servers.jpg"
        fillMode: Image.PreserveAspectCrop
        smooth: true
        z: -10
        onStatusChanged: console.log("[BG]", status, source, "size=", sourceSize)
    }
    Rectangle {
        anchors.fill: parent
        color: "#8006101d"
        z: -9
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
            color: "#081627"
            border.color: "#f5a623"
            border.width: 2
            radius: 10
        }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 22
            spacing: 14

            Text {
                Layout.alignment: Qt.AlignHCenter
                text: "⏸  RESUME REQUIRED"
                color: "#f5a623"
                font.pixelSize: 26
                font.bold: true
            }
            Text {
                Layout.fillWidth: true
                text: "feed_chamber timeout 150s — SCALE has been drained to PLACE.\nSelect how to resume the cycle:"
                color: "#c7dcef"
                font.pixelSize: 17
                wrapMode: Text.WordWrap
                horizontalAlignment: Text.AlignHCenter
            }
            Item { Layout.fillHeight: true }

            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                spacing: 16

                MotionButton {
                    Layout.preferredWidth: 250; Layout.preferredHeight: 60
                    text: "🔁  LOAD CHAMBER\nFROM BUFFER"
                    font.pixelSize: 15; font.bold: true
                    background: Rectangle { color: "#6f4be0"; border.color: "#9b7bff"; border.width: 2; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#bfe0f5"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: {
                        robotController.gotoState("LOAD_CHAMBER_FROM_BUFFER")
                        resumeChoicePopup.close()
                    }
                }
                MotionButton {
                    Layout.preferredWidth: 250; Layout.preferredHeight: 60
                    text: "🔂  LOAD CHAMBER\nFROM TRAY"
                    font.pixelSize: 15; font.bold: true
                    background: Rectangle { color: "#e2761b"; border.color: "#ecc45a"; border.width: 2; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#ecc45a"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: confirmEmptyBufferPopup.open()
                }
            }

            MotionButton {
                Layout.alignment: Qt.AlignHCenter
                Layout.preferredWidth: 516; Layout.preferredHeight: 52
                text: "⏹  STOP  —  Dừng hệ thống và giữ nguyên vị trí hiện tại"
                font.pixelSize: 15; font.bold: true
                background: Rectangle { color: "#3a1614"; border.color: "#f0735c"; border.width: 2; radius: 6 }
                contentItem: Text {
                    text: parent.text; color: "#f0735c"
                    font: parent.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                onClicked: {
                    mainWindow.stopSynchronizedSystems()
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
            color: "#081627"
            border.color: "#f0735c"
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
                color: "#f0735c"
                font.pixelSize: 24
                font.bold: true
            }
            Text {
                Layout.fillWidth: true
                text: "LOAD CHAMBER FROM TRAY will restart like a fresh boot:\n" +
                      "  INIT_LOAD → INIT_REFILL_BUFFER → cycle.\n\n" +
                      "Have you manually removed all cartridges from the BUFFER?"
                color: "#c7dcef"
                font.pixelSize: 16
                wrapMode: Text.WordWrap
                horizontalAlignment: Text.AlignHCenter
            }
            Item { Layout.fillHeight: true }

            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                spacing: 16

                MotionButton {
                    Layout.preferredWidth: 240; Layout.preferredHeight: 56
                    text: "✓  Buffer is empty — CONFIRM"
                    font.pixelSize: 15; font.bold: true
                    background: Rectangle { color: "#3ed0b4"; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#04140d"
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
                MotionButton {
                    Layout.preferredWidth: 240; Layout.preferredHeight: 56
                    text: "✗  Cancel / Back"
                    font.pixelSize: 15; font.bold: true
                    background: Rectangle { color: "#14263c"; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#c7dcef"
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
            color: "#081627"
            border.color: "#f0735c"
            border.width: 2
            radius: 10
        }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 22
            spacing: 14

            Text {
                Layout.alignment: Qt.AlignHCenter
                text: "⚠  SCALE ISSUE DETECTED"
                color: "#f0735c"
                font.pixelSize: 26
                font.bold: true
            }
            Text {
                Layout.fillWidth: true
                text: "No loadcell topic received for 150s in PROCESSING_SCALE.\nSelect how to handle this cartridge:"
                color: "#c7dcef"
                font.pixelSize: 16
                wrapMode: Text.WordWrap
                horizontalAlignment: Text.AlignHCenter
            }
            Item { Layout.fillHeight: true }

            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                spacing: 12

                MotionButton {
                    Layout.preferredWidth: 196; Layout.preferredHeight: 72
                    text: "↩  BACK TO\nWAIT FILLING\n(đã lấy cartridge ra)"
                    font.pixelSize: 13; font.bold: true
                    background: Rectangle { color: "#081627"; border.color: "#36b6ff"; border.width: 2; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#7fcdf5"
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
                MotionButton {
                    Layout.preferredWidth: 196; Layout.preferredHeight: 72
                    text: "✓  PLACE TO\nOUTPUT\n(force PASS)"
                    font.pixelSize: 13; font.bold: true
                    background: Rectangle { color: "#0a2418"; border.color: "#3ed0b4"; border.width: 2; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#3ed0b4"
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
                MotionButton {
                    Layout.preferredWidth: 196; Layout.preferredHeight: 72
                    text: "✗  PLACE TO\nFAIL\n(force FAIL)"
                    font.pixelSize: 13; font.bold: true
                    background: Rectangle { color: "#220c0b"; border.color: "#f0735c"; border.width: 2; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#f5a394"
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

            MotionButton {
                Layout.alignment: Qt.AlignHCenter
                Layout.preferredWidth: 612; Layout.preferredHeight: 52
                text: "⏹  STOP  —  Dừng hệ thống và giữ nguyên vị trí hiện tại"
                font.pixelSize: 15; font.bold: true
                background: Rectangle { color: "#160a09"; border.color: "#f0735c"; border.width: 2; radius: 6 }
                contentItem: Text {
                    text: parent.text; color: "#f0735c"
                    font: parent.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                onClicked: {
                    mainWindow.stopSynchronizedSystems()
                    mainWindow.scaleIssueWarning = true
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
            color: "#06101d"
            border.color: "#f0735c"
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
                color: "#f0735c"
                font.pixelSize: 24
                font.bold: true
                Layout.alignment: Qt.AlignHCenter
            }
            Text {
                text: "Ink or Lot has not been selected for the system.\nIf you continue running, production and consumption logs WILL NOT BE SAVED."
                color: "#c7dcef"
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
                MotionButton {
                    text: "✓ RUN (NO LOGGING)"
                    font.pixelSize: 16
                    font.bold: true
                    onClicked: {
                        notYetInkSelectedPopup.close()
                        if (notYetInkSelectedPopup.confirmCallback) notYetInkSelectedPopup.confirmCallback()
                    }
                    background: Rectangle { radius: 6; color: "#f0735c" }
                    contentItem: Text {
                        text: parent.text; color: "#ffffff"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }
                MotionButton {
                    text: "✗ CANCEL"
                    font.pixelSize: 16
                    font.bold: true
                    onClicked: notYetInkSelectedPopup.close()
                    background: Rectangle { radius: 6; color: "#14263c" }
                    contentItem: Text {
                        text: parent.text; color: "#c7dcef"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }
            }
        }
    }
}
