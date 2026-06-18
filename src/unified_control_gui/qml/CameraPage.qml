import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls.Material 2.15

Item {
    id: cameraPageRoot

    property string currentTime: Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
    property bool rowLocked: (robotController.systemStatus || "").toUpperCase() === "INIT_LOAD_CHAMBER_DIRECT" && robotController.selectedRow >= 1
    property bool modeLocked: false
    property bool startCommandLocked: false
    property bool autoRowIndicatorsActive: false
    property string ctrlMode: "auto"  // "auto" | "camera_ai"
    property string pendingStartMode: ""
    property string pendingStartUiMode: ""

    readonly property color cPanel:       "#990d1e32"
    readonly property color cPanel2:      "#88060f1e"
    readonly property color cBorder:      "#1affffff"
    readonly property color cHover:       "#40ffffff"
    readonly property color cText:        "#ffffff"
    readonly property color cMuted:       "#8eb4d0"
    readonly property color cAccent:      "#7bc8f0"
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

    function dispatchStartAfterModeConfirmed() {
        var requestedUiMode = pendingStartUiMode
        if (requestedUiMode === "")
            return

        pendingStartMode = ""
        pendingStartUiMode = ""
        startModeConfirmTimer.stop()

        if (requestedUiMode === "camera_ai") {
            robotController.selectRow(0)
            robotController.setAiMode(true)
            hpController.publishMode(0)
        } else if (requestedUiMode === "auto") {
            robotController.setAutoMode(true)
            hpController.publishMode(0)
        } else if (requestedUiMode === "manual") {
            robotController.setManualMode(true)
            hpController.publishMode(2)
        }

        modeLocked = true
        autoRowIndicatorsActive = true
        robotController.startSystem(true)
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

    Connections {
        target: mainWindow
        function onSynchronizedStopRequested() {
            startModeConfirmTimer.stop()
            cameraPageRoot.pendingStartMode = ""
            cameraPageRoot.pendingStartUiMode = ""
            cameraPageRoot.modeLocked = false
            cameraPageRoot.startCommandLocked = false
            cameraPageRoot.autoRowIndicatorsActive = false
        }
    }

    // Sync mode from Cartridge (ensures both UIs are synchronized)
    Connections {
        target: cartridgeController
        function onCurrentModeChanged() {
            var m = (cartridgeController.currentMode || "").toLowerCase()
            var requestedUiMode = cameraPageRoot.pendingStartUiMode
            if (cameraPageRoot.pendingStartMode !== "" && m === cameraPageRoot.pendingStartMode)
                cameraPageRoot.dispatchStartAfterModeConfirmed()
            if (requestedUiMode === "camera_ai" && m === "auto") {
                cameraPageRoot.ctrlMode = "camera_ai"
            } else if (m === "auto" || m === "ai" || m === "camera_ai" || m === "manual") {
                cameraPageRoot.ctrlMode = (m === "ai") ? "camera_ai" : m;
            }
        }
    }
    Timer {
        id: startModeConfirmTimer
        interval: 3000
        repeat: false
        onTriggered: {
            console.warn("START cancelled: cartridge mode confirmation timeout")
            cameraPageRoot.pendingStartMode = ""
            cameraPageRoot.pendingStartUiMode = ""
            cameraPageRoot.startCommandLocked = false
            cameraPageRoot.modeLocked = false
            cameraPageRoot.autoRowIndicatorsActive = false
        }
    }
    Timer {
        interval: 1000
        running: true
        repeat: true
        onTriggered: currentTime = Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
    }

    // Ambient glow blobs — glass depth background
    Canvas {
        anchors.fill: parent
        onPaint: {
            var ctx = getContext("2d")
            ctx.clearRect(0, 0, width, height)
            var g1 = ctx.createRadialGradient(width * 0.80, -height * 0.05, 0, width * 0.80, -height * 0.05, height * 0.85)
            g1.addColorStop(0.00, "rgba(20,90,210,0.40)")
            g1.addColorStop(0.30, "rgba(20,90,210,0.15)")
            g1.addColorStop(0.62, "rgba(20,90,210,0.03)")
            g1.addColorStop(1.00, "rgba(20,90,210,0.00)")
            ctx.fillStyle = g1; ctx.fillRect(0, 0, width, height)
            var g2 = ctx.createRadialGradient(width * 0.05, height * 1.10, 0, width * 0.05, height * 1.10, height * 0.65)
            g2.addColorStop(0.00, "rgba(0,175,155,0.30)")
            g2.addColorStop(0.40, "rgba(0,175,155,0.08)")
            g2.addColorStop(0.70, "rgba(0,175,155,0.02)")
            g2.addColorStop(1.00, "rgba(0,175,155,0.00)")
            ctx.fillStyle = g2; ctx.fillRect(0, 0, width, height)
        }
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
                color: cPanel
                border.color: cBorder
                border.width: 1
                radius: 8

                // specular highlight — top edge
                Rectangle {
                    anchors { top: parent.top; left: parent.left; right: parent.right }
                    height: 1
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: "transparent" }
                        GradientStop { position: 0.35; color: "#55ffffff" }
                        GradientStop { position: 0.65; color: "#55ffffff" }
                        GradientStop { position: 1.0;  color: "transparent" }
                    }
                }

                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 10

                    ScreenshotButton {
                        Layout.preferredWidth: 60; Layout.preferredHeight: 50
                        onCaptureRequested: {
                            robotController.captureScreenshot()
                        }
                    }

                    Item {
                        Layout.fillWidth: true
                        Text {
                            anchors.centerIn: parent
                            text: "ROS 2 - INKOBOT MONITORING SYSTEM"
                            font.pixelSize: 24; font.bold: true; color: "#5cf4f1"
                        }
                    }

                    MotionButton {
                        text: "CARTRIDGE SYSTEM  ▸"
                        Layout.preferredHeight: 50
                        font.pixelSize: 16; font.bold: true
                        onClicked: stackView.push(cartridgePage)
                        background: Rectangle {
                            radius: 6
                            color: "transparent"
                            border.color: "#5cf4f1"
                            border.width: 2
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: "#0e5274" }
                                GradientStop { position: 1.0; color: "#041f1f" }
                            }
                        }
                        contentItem: Text {
                            text: parent.text; font: parent.font
                            color: "#5cf4f1"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
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

                    MotionButton {
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
                        contentItem: Item {
                            Image {
                                anchors.centerIn: parent
                                source: "icons/switch_camera.svg"
                                width: 34
                                height: 34
                                fillMode: Image.PreserveAspectFit
                                smooth: true
                            }
                        }
                    }

                    MotionButton {
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
                MotionButton {
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
                    color: cPanel
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    border.color: cBorder
                    radius: 8
                    clip: true
                    HoverHandler { onHoveredChanged: parent.border.color = hovered ? cHover : cBorder }

                    // specular top edge
                    Rectangle {
                        anchors { top: parent.top; left: parent.left; right: parent.right }
                        height: 1; z: 1
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0; color: "transparent" }
                            GradientStop { position: 0.4; color: "#33ffffff" }
                            GradientStop { position: 0.6; color: "#33ffffff" }
                            GradientStop { position: 1.0; color: "transparent" }
                        }
                    }

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
                color: cPanel
                border.color: cBorder
                border.width: 1
                radius: 8
                HoverHandler { onHoveredChanged: parent.border.color = hovered ? cHover : cBorder }

                // specular top edge
                Rectangle {
                    anchors { top: parent.top; left: parent.left; right: parent.right }
                    height: 1; z: 1
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: "transparent" }
                        GradientStop { position: 0.4; color: "#44ffffff" }
                        GradientStop { position: 0.6; color: "#44ffffff" }
                        GradientStop { position: 1.0; color: "transparent" }
                    }
                }

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
                        PCard { lbl: "S1 Chamber";   val: hpController.pressureS1; maxVal: 1200; cardIndex: 0 }
                        PCard { lbl: "S2 Cartridge"; val: hpController.pressureS2; maxVal: 1200; cardIndex: 1 }
                        PCard { lbl: "S3 Tank";      val: hpController.pressureS3; maxVal: 1000; cardIndex: 2 }
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
                                cartIndex: index
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
                color: cPanel
                border.color: cBorder
                border.width: 1
                radius: 8
                HoverHandler { onHoveredChanged: parent.border.color = hovered ? cHover : cBorder }

                // specular top edge
                Rectangle {
                    anchors { top: parent.top; left: parent.left; right: parent.right }
                    height: 1; z: 1
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: "transparent" }
                        GradientStop { position: 0.4; color: "#44ffffff" }
                        GradientStop { position: 0.6; color: "#44ffffff" }
                        GradientStop { position: 1.0; color: "transparent" }
                    }
                }

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
                            Text { text: cameraPageRoot.ctrlMode.toUpperCase(); color: "#d4faff"; font.bold: true; font.pixelSize: 18; elide: Text.ElideRight; Layout.fillWidth: true }

                            Text { text: "Uptime:"; color: "#94a3b8"; font.pixelSize: 18 }
                            Text { text: robotController.systemUptime; color: "#d4faff"; font.bold: true; font.pixelSize: 18; elide: Text.ElideRight; Layout.fillWidth: true }

                            Text { text: "State Robot:"; color: "#94a3b8"; font.pixelSize: 18 }
                            Text { text: robotController.systemStatus; color: "#d4faff"; font.bold: true; font.pixelSize: 18; elide: Text.ElideRight; Layout.fillWidth: true }
                        }

                        Rectangle { width: 1; Layout.fillHeight: true; color: "#134357" }

                        // COLUMN 2: Ink & Scale Info
                        GridLayout {
                            Layout.fillWidth: true
                            Layout.alignment: Qt.AlignTop
                            columns: 2; rowSpacing: 8; columnSpacing: 10

                            Text { text: "Ink Name:"; color: "#94a3b8"; font.pixelSize: 18 }
                            Text { text: scaleController.activeInkName; color: "#d4faff"; font.bold: true; font.pixelSize: 18; elide: Text.ElideRight; Layout.fillWidth: true }

                            Text { text: "Cartridge Type:"; color: "#94a3b8"; font.pixelSize: 18 }
                            Text { text: scaleController.activeCartName; color: "#d4faff"; font.bold: true; font.pixelSize: 18; elide: Text.ElideRight; Layout.fillWidth: true }

                            Text { text: "Weight Batch:"; color: "#94a3b8"; font.pixelSize: 18 }
                            Text { text: scaleController.totalBatchWeight > 0 ? scaleController.totalBatchWeight.toFixed(2) + " g" : "0.00 g"; color: "#d4faff"; font.bold: true; font.pixelSize: 18; elide: Text.ElideRight; Layout.fillWidth: true }
                        }

                        Rectangle { width: 1; Layout.fillHeight: true; color: "#134357" }

                        // COLUMN 3: Control Mode
                        ColumnLayout {
                            Layout.fillWidth: true
                            Layout.alignment: Qt.AlignTop
                            spacing: 6

                            Text { text: "CONTROL MODE "; color: "#5cf4f1"; font.pixelSize: 16; font.bold: true; font.letterSpacing: 1 }

                            Rectangle {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 84
                                radius: 22
                                antialiasing: true
                                color: "#0d2538"
                                border.color: "#134357"
                                border.width: 1

                                ColumnLayout {
                                    anchors.fill: parent
                                    anchors.margins: 4
                                    spacing: 0

                                    Repeater {
                                        model: [
                                            { key: "auto",      lbl: "AUTO" },
                                            { key: "camera_ai", lbl: "CAMERA AI" }
                                        ]
                                        delegate: MotionButton {
                                            id: modeOption
                                            required property var modelData
                                            property bool isSelected: cameraPageRoot.ctrlMode === modelData.key
                                            property bool isLocked: cameraPageRoot.modeLocked || cameraPageRoot.robotBusy
                                            Layout.fillWidth: true
                                            Layout.fillHeight: true
                                            hoverScale: 1.05
                                            pressScale: 0.97
                                            shadowColor: "#66000000"
                                            enabled: !isLocked
                                            opacity: (!isSelected && isLocked) ? 0.35 : 1.0
                                            onClicked: cameraPageRoot.ctrlMode = modelData.key

                                            background: Rectangle {
                                                radius: height / 2
                                                antialiasing: true
                                                color: modeOption.isSelected ? "transparent" : (modeOption.hovered ? "#14334a" : "transparent")
                                                border.color: modeOption.isSelected ? "#63dce7" : (modeOption.hovered ? "#245c75" : "transparent")
                                                border.width: modeOption.isSelected || (modeOption.hovered && !modeOption.isSelected) ? 1 : 0
                                                gradient: modeOption.isSelected ? selectedModeGradient : null
                                                Behavior on color { ColorAnimation { duration: 140 } }
                                                Behavior on border.color { ColorAnimation { duration: 140 } }

                                                Gradient {
                                                    id: selectedModeGradient
                                                    orientation: Gradient.Horizontal
                                                    GradientStop { position: 0.0; color: "#2bb7c2" }
                                                    GradientStop { position: 1.0; color: "#0f7d93" }
                                                }
                                            }

                                            contentItem: Text {
                                                text: (modeOption.isLocked && !modeOption.isSelected ? "🔒 " : "") + modeOption.modelData.lbl
                                                color: modeOption.isSelected ? "#ffffff" : "#d4faff"
                                                font.pixelSize: 16
                                                font.bold: true
                                                horizontalAlignment: Text.AlignHCenter
                                                verticalAlignment: Text.AlignVCenter
                                            }
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
                            Layout.fillWidth: true;
                            font.pixelSize: 13; color: "#8888aa"; font.italic: true
                            text: cameraPageRoot.rowLocked ? "🔒 Waiting for tray..." : (cameraPageRoot.ctrlMode === "camera_ai" ? "(AI auto)" : "Select then press PICK_CARTRIDGE")
                            elide: Text.ElideRight
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 4
                        Repeater { model: 5
                            delegate: Rectangle {
                                property int rn: index + 1
                                property bool aiMode: cameraPageRoot.ctrlMode === "camera_ai"
                                property bool isActive: robotController.selectedRow === rn
                                property bool isAiDetected: aiMode && (robotController.rowReady[index] === true)
                                property bool canSelect: !cameraPageRoot.rowLocked && cameraPageRoot.ctrlMode === "auto"
                                Layout.fillWidth: true; height: 32; radius: 5
                                color: aiMode
                                       ? (isAiDetected ? Qt.rgba(0.36, 0.96, 0.95, 0.22) : "transparent")
                                       : (isActive ? "#0a3b42" : "#0d2538")
                                border.color: aiMode
                                              ? (isAiDetected ? "#5cf4f1" : "#134357")
                                              : (isActive ? "#5cf4f1" : "#134357")
                                border.width: aiMode
                                              ? (isAiDetected ? 2 : 1)
                                              : (isActive ? 2 : 1)
                                opacity: aiMode
                                         ? 1.0
                                         : (canSelect ? 1.0 : (isActive ? 1.0 : 0.45))
                                Rectangle {
                                    visible: isAiDetected || (!aiMode && isActive)
                                    anchors { top: parent.top; left: parent.left; right: parent.right }
                                    height: 3; radius: 2; color: "#5cf4f1"
                                }
                                Text {
                                    anchors.centerIn: parent; text: "R" + rn
                                    color: aiMode
                                           ? (isAiDetected ? "#5cf4f1" : "#6b7280")
                                           : (isActive ? "#5cf4f1" : "#94a3b8")
                                    font.pixelSize: 18; font.bold: (aiMode ? isAiDetected : isActive)
                                }
                                MotionMouseArea { anchors.fill: parent; enabled: canSelect; onClicked: robotController.selectRow(rn) }
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
                                property bool canSelect: cameraPageRoot.ctrlMode === "auto"
                                Layout.fillWidth: true; height: 32; radius: 5
                                color: aiMode
                                       ? (isActive ? "#0b351e" : (isReady ? "#0d2c1d" : "#0d2538"))
                                       : (isActive ? "#073238" : "#0d2538")
                                border.color: aiMode
                                              ? (isActive ? "#0fa36f" : (isReady ? "#0fa36f" : "#134357"))
                                              : (isActive ? "#22a6b8" : "#134357")
                                border.width: aiMode
                                              ? (isActive ? 3 : (isReady ? 2 : 1))
                                              : (isActive ? 2 : 1)
                                opacity: aiMode
                                         ? (isReady || isActive ? 1.0 : 0.45)
                                         : (canSelect ? 1.0 : (isActive ? 1.0 : 0.45))
                                Rectangle {
                                    visible: isActive
                                    anchors { top: parent.top; left: parent.left; right: parent.right }
                                    height: 3; radius: 2; color: aiMode ? "#0fa36f" : "#22a6b8"
                                }
                                Text {
                                    anchors.centerIn: parent; text: "O" + sn
                                    color: aiMode
                                           ? (isActive ? "#6ee7b7" : (isReady ? "#34d399" : "#94a3b8"))
                                           : (isActive ? "#67dce7" : "#94a3b8")
                                    font.pixelSize: 16; font.bold: isActive || isReady
                                }
                                MotionMouseArea {
                                    anchors.fill: parent
                                    enabled: canSelect
                                    hoverScale: 1.05
                                    pressScale: 0.98
                                    shadowColor: "#66000000"
                                    pressedShadowColor: "#80000000"
                                    shimmerColor: "#40ffffff"
                                    onClicked: robotController.selectSlot(sn)
                                }
                            }
                        }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#134357" }

                    Text { text: "STATE COMMANDS"; color: "#5cf4f1"; font.pixelSize: 16; font.bold: true; font.letterSpacing: 1 }
                    GridLayout {
                        Layout.fillWidth: true; columns: 2; rowSpacing: 5; columnSpacing: 5
                        Repeater {
                            model: [
                                { lbl: "IN_READY",     displayLbl: "IN_READY",       icon: "icons/between_horizontal_end.svg",  bgStart: "#0e5274", bgEnd: "#072d42", bc: "#1e7090", tc: "#7bc8f0" },
                                { lbl: "OUT_READY",    displayLbl: "OUT_READY",       icon: "icons/between_horizontal_start.svg",  bgStart: "#0e5274", bgEnd: "#072d42", bc: "#1e7090", tc: "#7bc8f0" },
                                { lbl: "PICK_INPUT",   displayLbl: "PICK_CARTRIDGE",  icon: "icons/arrows_up_from_line.svg", bgStart: "#3ba0cf", bgEnd: "#115c5c", bc: "#5bc8e8", tc: "#d4faff" },
                                { lbl: "PICK_CHAMBER", displayLbl: "PICK_CHAMBER",    icon: "icons/fold_horizontal.svg", bgStart: "#3ba0cf", bgEnd: "#115c5c", bc: "#5bc8e8", tc: "#d4faff" },
                                { lbl: "PLACE_OUTPUT", displayLbl: "PLACE_OUTPUT",    icon: "icons/package.svg",  bgStart: "#0e5274", bgEnd: "#031e1e", bc: "#1e6a8a", tc: "#d4faff" },
                                { lbl: "PLACE_FAIL",   displayLbl: "PLACE_FAIL",      icon: "icons/package_x.svg",  bgStart: "#0b4462", bgEnd: "#042027", bc: "#1a5070", tc: "#d4faff" }
                            ]
                            delegate: Rectangle {
                                required property var modelData
                                property bool isActive: (modelData.lbl === "IN_READY" && robotController.inReady) || (modelData.lbl === "OUT_READY" && robotController.outReady)
                                readonly property bool isReadyBtn: modelData.lbl === "IN_READY" || modelData.lbl === "OUT_READY"
                                property color gStart: (isReadyBtn && isActive) ? "#1a5070" : modelData.bgStart
                                property color gEnd:   (isReadyBtn && isActive) ? "#0a3040" : modelData.bgEnd
                                Layout.fillWidth: true; height: 64; radius: 5
                                color: "transparent"
                                gradient: Gradient {
                                    orientation: Gradient.Horizontal
                                    GradientStop { position: 0.0; color: ma.pressed ? Qt.darker(gStart, 1.2) : gStart }
                                    GradientStop { position: 1.0; color: ma.pressed ? Qt.darker(gEnd, 1.2) : gEnd }
                                }
                                border.color: modelData.bc
                                border.width: 1
                                Item {
                                    anchors.fill: parent

                                    Item {
                                        id: stateIconHost
                                        width: 38; height: 36
                                        anchors.horizontalCenter: parent.horizontalCenter
                                        anchors.top: parent.top
                                        anchors.topMargin: 1

                                        Image {
                                            source: modelData.icon.indexOf(".svg") !== -1 ? modelData.icon : ""
                                            visible: source.toString() !== ""
                                            width: 34; height: 34
                                            sourceSize.width: 80
                                            sourceSize.height: 80
                                            fillMode: Image.PreserveAspectFit
                                            smooth: true
                                            mipmap: true
                                            antialiasing: true
                                            anchors.centerIn: parent
                                        }

                                        Text {
                                            text: modelData.icon.indexOf(".svg") === -1 ? modelData.icon : ""
                                            visible: text !== ""
                                            color: (parent.parent.parent.isReadyBtn && parent.parent.parent.isActive) ? "#090d16" : modelData.tc
                                            font.pixelSize: 22
                                            anchors.centerIn: parent
                                        }

                                    }

                                    Row {
                                        spacing: 6
                                        anchors.horizontalCenter: parent.horizontalCenter
                                        anchors.top: stateIconHost.bottom
                                        anchors.bottom: parent.bottom

                                        Rectangle {
                                            width: 9; height: 9; radius: 4.5
                                            color: parent.parent.parent.isActive ? "#5cf4f1" : "#2a3a4a"
                                            border.color: modelData.bc
                                            border.width: 1
                                            visible: modelData.lbl === "IN_READY" || modelData.lbl === "OUT_READY"
                                            anchors.verticalCenter: parent.verticalCenter
                                            Behavior on color { ColorAnimation { duration: 200 } }
                                        }

                                        Text {
                                            text: modelData.displayLbl
                                            color: (parent.parent.parent.isReadyBtn && parent.parent.parent.isActive) ? "#090d16" : modelData.tc
                                            font.pixelSize: 13
                                            font.bold: true
                                            anchors.verticalCenter: parent.verticalCenter
                                        }
                                    }
                                }
                                MotionMouseArea { id: ma; anchors.fill: parent; onClicked: {
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

                    Rectangle { Layout.fillWidth: true; height: 56; radius: 5; color: "transparent"; border.color: "#ef4444"; border.width: 1
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0; color: emMA.pressed ? Qt.darker("#da2525", 1.15) : "#da2525" }
                            GradientStop { position: 1.0; color: emMA.pressed ? Qt.darker("#ba1b1b", 1.15) : "#ba1b1b" }
                        }
                        Text { anchors.centerIn: parent; text: "⛔ EMERGENCY STOP"; color: "#ffffff"; font.pixelSize: 21; font.bold: true }
                        MotionMouseArea { id: emMA; anchors.fill: parent; onClicked: { cameraPageRoot.modeLocked = false; robotController.emergencyStop(true) } }
                    }

                    GridLayout {
                        Layout.fillWidth: true; columns: 3; rowSpacing: 5; columnSpacing: 5

                        Rectangle { Layout.fillWidth: true; height: 52; radius: 5; color: "transparent"; border.color: "#6a2222"; border.width: 1
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: stopResetMA.pressed ? Qt.darker("#8a2020", 1.15) : "#8a2020" }
                                GradientStop { position: 1.0; color: stopResetMA.pressed ? Qt.darker("#4e0c0c", 1.15) : "#4e0c0c" }
                            }
                            Text { anchors.centerIn: parent; text: "⏹ STOP"; color: "#d4faff"; font.pixelSize: 20; font.bold: true }
                            MotionMouseArea { id: stopResetMA; anchors.fill: parent; onClicked: {
                                mainWindow.stopSynchronizedSystems()
                            } }
                        }

                        Rectangle { Layout.fillWidth: true; height: 52; radius: 5; color: "transparent"; border.color: "#134357"; border.width: 1
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: enMA.pressed ? Qt.darker("#0e5274", 1.15) : "#0e5274" }
                                GradientStop { position: 1.0; color: enMA.pressed ? Qt.darker("#052b3d", 1.15) : "#052b3d" }
                            }
                            Text { anchors.centerIn: parent; text: "ENABLE"; color: "#d4faff"; font.pixelSize: 21; font.bold: true }
                            MotionMouseArea { id: enMA; anchors.fill: parent; onClicked: robotController.enableSystem(true) }
                        }

                        Rectangle { Layout.fillWidth: true; height: 52; radius: 5; color: "transparent"; border.color: "#0d6060"; border.width: 1
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: startMA.pressed ? Qt.darker("#0c7876", 1.15) : "#0c7876" }
                                GradientStop { position: 1.0; color: startMA.pressed ? Qt.darker("#085f5d", 1.15) : "#085f5d" }
                            }
                            Text { anchors.centerIn: parent; text: "▶ START"; color: "#d4faff"; font.pixelSize: 20; font.bold: true }
                            MotionMouseArea { id: startMA; anchors.fill: parent; onClicked: {
                                if (cameraPageRoot.startCommandLocked)
                                    return
                                cameraPageRoot.startCommandLocked = true
                                cameraPageRoot.pendingStartUiMode = cameraPageRoot.ctrlMode
                                cameraPageRoot.pendingStartMode = cameraPageRoot.ctrlMode === "manual" ? "manual" : "auto"

                                var confirmedMode = (cartridgeController.currentMode || "").toLowerCase()
                                if (confirmedMode === cameraPageRoot.pendingStartMode) {
                                    cameraPageRoot.dispatchStartAfterModeConfirmed()
                                } else {
                                    startModeConfirmTimer.restart()
                                    cartridgeController.setMode(cameraPageRoot.pendingStartMode)
                                }
                            } }
                        }

                        Rectangle { Layout.fillWidth: true; height: 52; radius: 5; color: "transparent"; border.color: "#134357"; border.width: 1
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: pauseMA.pressed ? Qt.darker("#0e5274", 1.15) : "#0e5274" }
                                GradientStop { position: 1.0; color: pauseMA.pressed ? Qt.darker("#052b3d", 1.15) : "#052b3d" }
                            }
                            Text { anchors.centerIn: parent; text: "PAUSE"; color: "#d4faff"; font.pixelSize: 21; font.bold: true }
                            MotionMouseArea { id: pauseMA; anchors.fill: parent; onClicked: robotController.pauseRobot() }
                        }

                        Rectangle { Layout.fillWidth: true; height: 52; radius: 5; color: "transparent"; border.color: "#134357"; border.width: 1
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: resMA.pressed ? Qt.darker("#0e5274", 1.15) : "#0e5274" }
                                GradientStop { position: 1.0; color: resMA.pressed ? Qt.darker("#052b3d", 1.15) : "#052b3d" }
                            }
                            Text { anchors.centerIn: parent; text: "RESUME"; color: "#d4faff"; font.pixelSize: 21; font.bold: true }
                            MotionMouseArea { id: resMA; anchors.fill: parent; onClicked: robotController.resumeRobot() }
                        }

                        Rectangle { Layout.fillWidth: true; height: 52; radius: 5; color: "transparent"; border.color: "#134357"; border.width: 1
                            gradient: Gradient {
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: clrMA.pressed ? Qt.darker("#0e5274", 1.15) : "#0e5274" }
                                GradientStop { position: 1.0; color: clrMA.pressed ? Qt.darker("#052b3d", 1.15) : "#052b3d" }
                            }
                            Text { anchors.centerIn: parent; text: "CLEAR ERR"; color: "#d4faff"; font.pixelSize: 21; font.bold: true }
                            MotionMouseArea { id: clrMA; anchors.fill: parent; onClicked: robotController.clearError() }
                        }
                    }
                }
            }
        }

        // ── Footer ─────────────────────────────────────────────
        Item {
            height: 40; Layout.fillWidth: true
            Rectangle {
                anchors.fill: parent; color: cPanel; border.color: cBorder; border.width: 1
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
        property int    cardIndex: 0
        Layout.fillWidth: true
        Layout.fillHeight: true
        implicitHeight: 64
        radius: 10
        color: cPanel2
        clip: true
        border.color: cBorder
        border.width: 1
        HoverHandler { onHoveredChanged: parent.border.color = hovered ? cHover : cBorder }
        ColumnLayout {
            anchors.fill: parent
            anchors.leftMargin: 16
            anchors.rightMargin: 16
            anchors.topMargin: 10
            anchors.bottomMargin: 10
            spacing: 8
            RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: 8
                Text {
                    text: {
                        var parts = lbl.split(" ");
                        var head = parts[0];
                        var tail = parts.length > 1 ? parts.slice(1).join(" ") : "";
                        return tail.length > 0 ? (head + " · " + tail) : head;
                    }
                    color: cMuted
                    font.pixelSize: 15
                    font.bold: true
                    Layout.alignment: Qt.AlignVCenter
                }
                Item { Layout.fillWidth: true }
                RowLayout {
                    spacing: 4
                    Layout.alignment: Qt.AlignVCenter
                    Text {
                        text: val.toFixed(1)
                        color: cText
                        font.pixelSize: 26
                        font.bold: true
                        font.family: "monospace"
                    }
                    Text {
                        text: "mbar"
                        color: cMuted
                        font.pixelSize: 13
                        font.bold: true
                        Layout.alignment: Qt.AlignBottom
                        Layout.bottomMargin: 3
                    }
                }
            }
            Rectangle {
                id: pcBarContainer
                Layout.fillWidth: true
                Layout.preferredHeight: 12
                radius: height / 2
                antialiasing: true
                color: "#0c4663"
                clip: true

                Rectangle {
                    id: pcFilledBar
                    height: parent.height
                    radius: height / 2
                    antialiasing: true
                    width: parent.width * Math.max(0, Math.min(1, val / maxVal))
                    clip: true
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: "#0e5274" }
                        GradientStop { position: 1.0; color: "#041f1f" }
                    }

                    onWidthChanged: {
                        pcShimmerAnim.restart()
                    }

                    Rectangle {
                        id: pcBarShimmer
                        height: parent.height
                        width: 160
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0;  color: "#00ffffff" }
                            GradientStop { position: 0.35; color: "#22ffffff" }
                            GradientStop { position: 0.5;  color: "#44ffffff" }
                            GradientStop { position: 0.65; color: "#22ffffff" }
                            GradientStop { position: 1.0;  color: "#00ffffff" }
                        }
                        PropertyAnimation {
                            id: pcShimmerAnim
                            target: pcBarShimmer
                            property: "x"
                            from: -pcBarShimmer.width
                            to: pcFilledBar.width
                            duration: 2600 + cardIndex * 300
                            loops: Animation.Infinite
                            running: pcFilledBar.width > 0
                        }
                    }
                }
            }
        }
    }

    component CartRow: Rectangle {
        property string cartName: ""
        property real   cartVal: 0
        property int    cartIndex: 0
        readonly property string cls: classifyPressure(cartVal, 280, 400, 600)
        Layout.fillWidth: true
        Layout.fillHeight: true
        implicitHeight: 60
        radius: 6
        color:        cPanel2
        clip:         true
        border.color: cBorder
        border.width: 1
        HoverHandler { onHoveredChanged: parent.border.color = hovered ? cHover : cBorder }
        ColumnLayout {
            anchors.fill: parent
            anchors.leftMargin: 14
            anchors.rightMargin: 14
            anchors.topMargin: 8
            anchors.bottomMargin: 8
            spacing: 2
            RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                ColumnLayout {
                    spacing: 0
                    Layout.alignment: Qt.AlignVCenter
                    Text {
                        text: {
                            var parts = cartName.split(" ");
                            return parts[0].toUpperCase();
                        }
                        color: cMuted
                        font.pixelSize: 11
                        font.bold: true
                    }
                    Text {
                        text: {
                            var parts = cartName.split(" ");
                            return parts.length > 1 ? parts[1] : "";
                        }
                        color: cText
                        font.pixelSize: 16
                        font.bold: true
                    }
                }
                Item { Layout.fillWidth: true }
                RowLayout {
                    spacing: 4
                    Layout.alignment: Qt.AlignVCenter
                    Text {
                        text: cartVal.toFixed(1)
                        color: cText
                        font.pixelSize: 21
                        font.bold: true
                        font.family: "monospace"
                    }
                    Text {
                        text: "mbar"
                        color: cMuted
                        font.pixelSize: 13
                        font.bold: true
                        Layout.alignment: Qt.AlignBottom
                        Layout.bottomMargin: 2
                    }
                }
            }
            Rectangle {
                id: barContainer
                Layout.fillWidth: true
                Layout.preferredHeight: 12
                radius: height / 2
                antialiasing: true
                color: "#0c4663"
                clip: true

                Rectangle {
                    id: cartFilledBar
                    height: parent.height
                    radius: height / 2
                    antialiasing: true
                    width: parent.width * Math.max(0, Math.min(1, cartVal / 1200))
                    clip: true
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop {
                            position: 0.0
                            color: {
                                if (cls === "limit") return "#0e5274";
                                if (cls === "high") return "#3ba0cf";
                                if (cls === "ok") return "#90c0cf";
                                return "#0e5274";
                            }
                        }
                        GradientStop {
                            position: 1.0
                            color: {
                                if (cls === "limit") return "#041f1f";
                                if (cls === "high") return "#125c5c";
                                if (cls === "ok") return "#459bbd";
                                return "#041f1f";
                            }
                        }
                    }

                    onWidthChanged: {
                        cartShimmerAnim.restart()
                    }

                    Rectangle {
                        id: cartBarShimmer
                        height: parent.height
                        width: 160
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0;  color: "#00ffffff" }
                            GradientStop { position: 0.35; color: "#22ffffff" }
                            GradientStop { position: 0.5;  color: "#44ffffff" }
                            GradientStop { position: 0.65; color: "#22ffffff" }
                            GradientStop { position: 1.0;  color: "#00ffffff" }
                        }
                        PropertyAnimation {
                            id: cartShimmerAnim
                            target: cartBarShimmer
                            property: "x"
                            from: -cartBarShimmer.width
                            to: cartFilledBar.width
                            duration: 2600 + cartIndex * 300
                            loops: Animation.Infinite
                            running: cartFilledBar.width > 0
                        }
                    }
                }
            }
        }
    }

}
