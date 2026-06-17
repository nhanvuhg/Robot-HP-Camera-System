    import QtQuick 2.15
    import QtQuick.Controls 2.15
    import QtQuick.Layouts 1.15

    // NOTE: "cartridge systems" refers strictly to this "ROS2 - CARTRIDGE PROVISION SYSTEM" page (CartridgePage.qml).
    // UI styling can change freely, but keep button logic intact unless the request explicitly says to change behavior.

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
        focus: true
        activeFocusOnTab: true

        readonly property int headerH:  70
        readonly property int tabbarH:  44
        readonly property int gap:       4
        readonly property int pad:       6
        readonly property int ctrlW:   245   // rộng hơn để chứa title font 14
        readonly property int sensorW: 250
        readonly property real rowRatio: 5.0 / (5.0 + 1.6)   // top:log = 5:1.6 → log nhỏ hơn nữa

        property int gridH:   height - headerH - tabbarH
        property int outerW:  width  - pad * 2
        property int outerH:  gridH  - pad * 2
        property int centerW: outerW - ctrlW - sensorW - gap * 2
        property int topH:    Math.floor(outerH * rowRatio) - gap
        property int logH:    outerH - topH - gap
        property int previousStackIndex: 0
        property int slideDirection: 1
        property int screenDragStartIndex: 0
        property bool startCommandLocked: false

        readonly property color cBg:     "transparent"
        readonly property color cBg2:    "#990d1e32"
        readonly property color cCard:   "#88060f1e"
        readonly property color cBorder: "#1affffff"
        readonly property color cAccent: "#7bc8f0"
        readonly property color cGreen:  "#5cf4f1"
        readonly property color cRed:    "#ff5252"
        readonly property color cOrange: "#ffa726"
        readonly property color cCyan:   "#26c6da"
        readonly property color cYellow: "#ffd740"
        readonly property color cDim:    "#8888aa"
        readonly property color cText:   "#e8e8f0"
        readonly property color cHover:  "#40ffffff"
        readonly property color cUnifiedBtn: Qt.lighter("#0d2a3a", 1.12)
        readonly property color cBlueWhiteBtn: "#5baeb2"
        readonly property color cBlueWhiteSelected: "#102e42"
        readonly property color cBlueWhiteIdle: "#28949a"
        readonly property color cBlueWhiteBorder: "#7acdd0"
        readonly property color cBlueWhiteText: "#eaffff"
        readonly property color cBlueWhiteSubText: "#b8dde0"
        readonly property color cStateAuxBtn: "#3f8185"
        readonly property color cStateAuxBorder: "#5eabad"
        readonly property color cStateAuxText: "#d4f0f1"
        property bool jogStopStateHint: false
        property bool homingCommandLocked: false

        function homingBusy() {
            return homingCommandLocked || cartridgeController.systemState.toLowerCase().indexOf("homing") !== -1
        }

        function cancelHoming() {
            root.homingCommandLocked = false
            homingLockFailsafeTimer.stop()
            cartridgeController.abortToJog()
            cartridgeController.softStop()
        }

        function showJogStopStateHint() {
            jogStopStateHint = true
            jogStopStateHintTimer.restart()
        }

        function fadeJogStopStateHint() {
            if (jogStopStateHint)
                jogStopStateHintTimer.restart()
        }

        Timer {
            id: jogStopStateHintTimer
            interval: 1200
            repeat: false
            onTriggered: root.jogStopStateHint = false
        }

        Timer {
            id: homingLockFailsafeTimer
            interval: 8000
            repeat: false
            onTriggered: {
                if (cartridgeController.systemState.toLowerCase().indexOf("homing") === -1)
                    root.homingCommandLocked = false
            }
        }

        Connections {
            target: cartridgeController
            function onSystemStateChanged() {
                if (cartridgeController.systemState.toLowerCase().indexOf("homing") === -1)
                    root.homingCommandLocked = false
            }
        }

        function setStackIndex(nextIndex) {
            var clamped = Math.max(0, Math.min(5, nextIndex))
            if (clamped === stack.currentIndex)
                return

            previousStackIndex = stack.currentIndex
            slideDirection = clamped > previousStackIndex ? 1 : -1
            stack.currentIndex = clamped
            stackSlide.x = slideDirection * Math.min(140, Math.max(70, stack.width * 0.10))
            stack.opacity = 0.68
            stackSlideAnim.restart()
        }

        Shortcut {
            sequence: "Left"
            context: Qt.WindowShortcut
            enabled: root.visible && stack.currentIndex > 0
            onActivated: root.setStackIndex(stack.currentIndex - 1)
        }

        Shortcut {
            sequence: "Right"
            context: Qt.WindowShortcut
            enabled: root.visible && stack.currentIndex < 5
            onActivated: root.setStackIndex(stack.currentIndex + 1)
        }

        Component.onCompleted: forceActiveFocus()
        onVisibleChanged: if (visible) forceActiveFocus()

        Rectangle { anchors.fill: parent; color: root.cBg }

        // Ambient glow blobs — creates depth behind glass panels
        Canvas {
            anchors.fill: parent
            onPaint: {
                var ctx = getContext("2d")
                ctx.clearRect(0, 0, width, height)
                var g1 = ctx.createRadialGradient(width * 0.85, -height * 0.08, 0, width * 0.85, -height * 0.08, height * 0.80)
                g1.addColorStop(0.00, "rgba(30,100,220,0.42)")
                g1.addColorStop(0.30, "rgba(30,100,220,0.18)")
                g1.addColorStop(0.60, "rgba(30,100,220,0.04)")
                g1.addColorStop(1.00, "rgba(30,100,220,0.00)")
                ctx.fillStyle = g1; ctx.fillRect(0, 0, width, height)
                var g2 = ctx.createRadialGradient(width * 0.06, height * 1.08, 0, width * 0.06, height * 1.08, height * 0.70)
                g2.addColorStop(0.00, "rgba(0,180,160,0.32)")
                g2.addColorStop(0.38, "rgba(0,180,160,0.10)")
                g2.addColorStop(0.68, "rgba(0,180,160,0.02)")
                g2.addColorStop(1.00, "rgba(0,180,160,0.00)")
                ctx.fillStyle = g2; ctx.fillRect(0, 0, width, height)
            }
        }

        // ════════════════════════════════════════════════════════════
        // HEADER
        // ════════════════════════════════════════════════════════════
        Rectangle {
            id: header
            anchors { top: parent.top; left: parent.left; right: parent.right }
            height: root.headerH
            color: "#cc0d1428"; border.color: root.cBorder; z: 10

            // Top specular line
            Rectangle {
                anchors { top: parent.top; left: parent.left; right: parent.right }
                height: 1
                gradient: Gradient {
                    orientation: Gradient.Horizontal
                    GradientStop { position: 0.0; color: "transparent" }
                    GradientStop { position: 0.4; color: "#55ffffff" }
                    GradientStop { position: 0.6; color: "#55ffffff" }
                    GradientStop { position: 1.0; color: "transparent" }
                }
            }

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 12
                anchors.rightMargin: 12
                spacing: 8
                MotionButton {
                    id: backBtn
                    Layout.preferredWidth: 50; Layout.preferredHeight: 50
                    onClicked: stackView.pop()
                    background: Rectangle { radius: 6; color: "transparent"; border.color: "#134357"; border.width: 2 }
                    contentItem: Image {
                        source: "qrc:/icons/qml/icons/reply_arrow.svg"
                        width: 24; height: 24
                        anchors.centerIn: parent
                        fillMode: Image.PreserveAspectFit; smooth: true
                    }
                }
                MotionButton {
                    Layout.preferredWidth: 50; Layout.preferredHeight: 50
                    onClicked: {
                        robotController.captureScreenshot()
                    }
                    background: Rectangle { radius: 6; color: "transparent"; border.color: "#1565c0"; border.width: 2 }
                    contentItem: Text {
                        text: "📷"; font.pixelSize: 22
                        horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                    }
                }
                Item { width: 6 }
                Text {
                    text: "ROS2 - CARTRIDGE PROVISION SYSTEM"
                    color: "#6cf"
                    font.pixelSize: 24; font.bold: true; font.letterSpacing: 1.5
                }
                Item { Layout.fillWidth: true }
                Rectangle {
                    id: stateBadge
                    Layout.preferredHeight: 50; radius: 6
                    Layout.preferredWidth: sbRow.implicitWidth + 24
                    color: "transparent"; border.color: "#134357"; border.width: 2
                    Row {
                        id: sbRow
                        anchors.centerIn: parent; spacing: 8
                        Rectangle {
                            id: stateDot
                            width: 9; height: 9; radius: 4.5
                            anchors.verticalCenter: parent.verticalCenter
                            color: {
                                var s = cartridgeController.systemState.toUpperCase()
                                if (s.indexOf("ERROR") !== -1) return root.cRed
                                if (s === "IDLE" || s === "UNKNOWN" || s === "") return root.cOrange
                                return root.cGreen
                            }
                            SequentialAnimation on opacity { loops: Animation.Infinite
                                NumberAnimation { to: 0.35; duration: 900 }
                                NumberAnimation { to: 1.0;  duration: 900 }
                            }
                        }
                        Text {
                            text: cartridgeController.systemState.toUpperCase().replace(/\|/g, "   •   ")
                            color: "#6cf"
                            font.pixelSize: 14; font.bold: true; font.letterSpacing: 1
                            anchors.verticalCenter: parent.verticalCenter
                        }
                    }
                }
                Item { width: 4 }
                Rectangle {
                    Layout.preferredHeight: 50; radius: 6
                    Layout.preferredWidth: hmRow.implicitWidth + 24
                    color: "transparent"; border.color: "#134357"; border.width: 2
                    Row {
                        id: hmRow
                        anchors.centerIn: parent; spacing: 6
                        Text {
                            text: cartridgeController.systemState.toLowerCase().indexOf("homing") !== -1 ? "⟳ HOMING..."
                                : (cartridgeController.currentMode !== "" && cartridgeController.systemState === "idle") ? "✓ HOMED"
                                : "○ NOT HOMED"
                            color: "#6cf"
                            font.pixelSize: 14; font.bold: true; font.letterSpacing: 1
                        }
                    }
                }
                Item { width: 4 }
                Rectangle {
                    id: modePill; Layout.preferredHeight: 50; radius: 6
                    property string m: cartridgeController.currentMode
                    property bool isIdle: m === "idle" || m === ""
                    Layout.preferredWidth: mpLbl.implicitWidth + 26
                    color: "transparent"; border.color: "#134357"; border.width: 2

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
                        color: "#6cf"
                        font.pixelSize: 14; font.bold: true; font.letterSpacing: 1
                    }
                }
                Item { width: 4 }
                MotionButton {
                    id: faultsBtn
                    text: "CLEAR ERROR"
                    Layout.preferredWidth: 130; Layout.preferredHeight: 50
                    font.pixelSize: 13; font.bold: true
                    onClicked: cartridgeController.resetFaults()
                    background: Rectangle {
                        radius: 6
                        color: faultsBtn.hovered ? "#332e0a" : "transparent"
                        border.color: "#ffd740"; border.width: 2
                        Behavior on color { ColorAnimation { duration: 120 } }
                    }
                    contentItem: Text {
                        text: parent.text; font: parent.font; color: "#ffd740"
                        horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                    }
                }
                Item { width: 4 }
                MotionButton {
                    id: closeBtn
                    Layout.preferredWidth: 50; Layout.preferredHeight: 50
                    onClicked: Qt.quit()
                    background: Rectangle {
                        radius: 6
                        color: closeBtn.hovered ? "#5a0a0a" : "transparent"
                        border.color: "#134357"; border.width: 2
                        Behavior on color { ColorAnimation { duration: 120 } }
                    }
                    contentItem: Image {
                        source: "qrc:/icons/qml/icons/power_settings.svg"
                        width: 24; height: 24
                        anchors.centerIn: parent
                        fillMode: Image.PreserveAspectFit; smooth: true
                    }
                    ToolTip.visible: hovered
                    ToolTip.delay: 500
                    ToolTip.text: "Tắt giao diện"
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
                    MotionButton {
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
                    MotionButton {
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
                MotionButton {
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
                MotionButton {
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
            anchors.leftMargin: 16
            anchors.rightMargin: 16
            height: root.tabbarH
            radius: height / 2
            antialiasing: true
            color: "#0d2538"
            border.color: "#134357"
            border.width: 1

            // drag-to-switch: press anywhere on tabbar and swipe left/right
            property real _dragPressX: 0
            property real _dragCurrentX: 0
            property bool _wasDrag: false
            MotionMouseArea {
                id: tabDragArea
                anchors.fill: parent
                z: 10
                motionEnabled: false
                shadowEnabled: false
                shimmerEnabled: false
                onPressed: {
                    tabbar._dragPressX = mouseX
                    tabbar._dragCurrentX = mouseX
                    tabbar._wasDrag = false
                }
                onPositionChanged: {
                    tabbar._dragCurrentX = mouseX
                    if (Math.abs(mouseX - tabbar._dragPressX) > 30)
                        tabbar._wasDrag = true
                }
                onReleased: {
                    if (tabbar._wasDrag) {
                        var dx = mouseX - tabbar._dragPressX
                        if (dx < 0 && stack.currentIndex < 5) root.setStackIndex(stack.currentIndex + 1)
                        else if (dx > 0 && stack.currentIndex > 0) root.setStackIndex(stack.currentIndex - 1)
                    } else {
                        var tabWidth = (tabbar.width - 8) / 6
                        var clickedIndex = Math.floor(Math.max(0, Math.min(tabbar.width - 9, mouseX - 4)) / tabWidth)
                        root.setStackIndex(clickedIndex)
                    }
                    tabbar._dragCurrentX = mouseX
                }
                onCanceled: {
                    tabbar._wasDrag = false
                    tabbar._dragCurrentX = tabbar._dragPressX
                }
            }

            Rectangle {
                id: tabGrip
                width: Math.max(44, (tabbar.width - 8) / 6 - 22)
                height: 4
                radius: 2
                y: tabbar.height - 8
                property real tabWidth: (tabbar.width - 8) / 6
                property real dragPreviewOffset: tabDragArea.pressed && tabbar._wasDrag
                                                 ? Math.max(-tabWidth, Math.min(tabWidth, tabbar._dragCurrentX - tabbar._dragPressX)) * 0.28
                                                 : 0
                x: 4 + stack.currentIndex * tabWidth + (tabWidth - width) / 2 + dragPreviewOffset
                z: 3
                color: "#5cf4f1"
                opacity: tabDragArea.pressed ? 1.0 : 0.82
                scale: tabDragArea.pressed ? 1.14 : 1.0
                transformOrigin: Item.Center
                Behavior on x { NumberAnimation { duration: 210; easing.type: Easing.OutCubic } }
                Behavior on opacity { NumberAnimation { duration: 120 } }
                Behavior on scale { NumberAnimation { duration: 120; easing.type: Easing.OutQuad } }
            }

            Row {
                anchors.fill: parent
                anchors.margins: 4
                spacing: 0
                Repeater {
                    model: ListModel {
                        ListElement { t: "Control Dashboard"; k: "control" }
                        ListElement { t: "Technical System";  k: "config"  }
                        ListElement { t: "Robot Control";     k: "robot"   }
                        ListElement { t: "Fill HP Control";   k: "hp"      }
                        ListElement { t: "Ink System";        k: "ink"     }
                        ListElement { t: "Production Output"; k: "prod"    }
                    }
                    delegate: MotionButton {
                        id: tabButton
                        property bool isSelected: stack.currentIndex === index
                        height: parent.height
                        width: (tabbar.width - 8) / 6
                        z: 1
                        hoverScale: 1.05
                        pressScale: 0.976
                        shadowColor: "#66000000"
                        onClicked: if (!tabbar._wasDrag) root.setStackIndex(index)

                        background: Rectangle {
                            radius: height / 2
                            antialiasing: true
                            color: tabButton.isSelected ? "transparent" : (tabButton.hovered ? "#14334a" : "transparent")
                            border.color: tabButton.isSelected ? "#63dce7" : (tabButton.hovered ? "#245c75" : "transparent")
                            border.width: tabButton.isSelected || tabButton.hovered ? 1 : 0
                            gradient: tabButton.isSelected ? selectedTabGradient : null
                            Behavior on color { ColorAnimation { duration: 140 } }
                            Behavior on border.color { ColorAnimation { duration: 140 } }

                            Gradient {
                                id: selectedTabGradient
                                orientation: Gradient.Horizontal
                                GradientStop { position: 0.0; color: "#2ab6c0" }
                                GradientStop { position: 1.0; color: "#0f7688" }
                            }
                        }

                        contentItem: Text {
                            text: model.t
                            font.pixelSize: 17
                            font.bold: true
                            font.letterSpacing: 0.5
                            color: tabButton.isSelected ? "#ffffff" : "#d4faff"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
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
            transform: Translate { id: stackSlide; x: 0 }

            DragHandler {
                id: screenSwipeHandler
                target: null
                acceptedButtons: Qt.LeftButton
                grabPermissions: PointerHandler.CanTakeOverFromAnything
                xAxis.enabled: true
                yAxis.enabled: false

                onActiveChanged: {
                    if (active) {
                        root.screenDragStartIndex = stack.currentIndex
                    } else {
                        var dx = activeTranslation.x
                        if (Math.abs(dx) > 90) {
                            if (dx < 0 && root.screenDragStartIndex < 5)
                                root.setStackIndex(root.screenDragStartIndex + 1)
                            else if (dx > 0 && root.screenDragStartIndex > 0)
                                root.setStackIndex(root.screenDragStartIndex - 1)
                        }
                    }
                }
            }

            ParallelAnimation {
                id: stackSlideAnim
                NumberAnimation {
                    target: stackSlide
                    property: "x"
                    to: 0
                    duration: 240
                    easing.type: Easing.OutCubic
                }
                NumberAnimation {
                    target: stack
                    property: "opacity"
                    to: 1.0
                    duration: 180
                    easing.type: Easing.OutQuad
                }
            }

            // ── PAGE 1: CONTROL DASHBOARD ────────────────────────
            Item {
                Item {
                    id: pageGrid
                    anchors { fill: parent; margins: root.pad }

                    // ─ TOP CARDS ROW ─────────────────────────────
                    RowLayout {
                        id: topCardsRow
                        x: 0; y: 0
                        width: parent.width - root.sensorW - root.gap
                        height: 260
                        spacing: root.gap

                        // ── Mode Selection ──────────────────────
                        Rectangle {
                            Layout.preferredWidth: (parent.width - root.sensorW - root.gap) * 0.166
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            color: root.cBg2; border.color: root.cBorder; radius: 6
                            HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cBorder }

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
                                    text: "MODE SELECTION"; color: "#5cf4f1"
                                    font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5
                                }

                                // ── Mode Items (always visible, no dropdown) ──
                                Column {
                                    Layout.fillWidth: true
                                    Layout.fillHeight: true
                                    spacing: 6

                                    // AUTO
                                    Rectangle {
                                        width: parent.width
                                        height: (parent.height - 6) / 2
                                        radius: 5
                                        property bool isModeSelected: cartridgeController.currentMode === "auto"
                                        color: isModeSelected ? "#5cf4f1" : "transparent"
                                        border.color: isModeSelected ? "#86bccb" : "#b9dfe1"
                                        border.width: 1
                                        Behavior on color { ColorAnimation { duration: 150 } }
                                        Behavior on border.color { ColorAnimation { duration: 150 } }
                                        Rectangle {
                                            anchors.fill: parent
                                            radius: parent.radius
                                            visible: parent.isModeSelected
                                            color: "#8094a3b8"
                                        }
                                        MotionMouseArea {
                                            anchors.fill: parent
                                            enabled: !modeSelCol.modeBlocked
                                            hoverScale: 1.05
                                            pressScale: 0.976
                                            shadowEnabled: false
                                            shimmerEnabled: false
                                            onClicked: {
                                                cartridgeController.setMode("auto")
                                                hpController.publishMode(0)
                                            }
                                        }
                                        Column {
                                            anchors.centerIn: parent
                                            spacing: 2
                                            Text { text: "AUTO"; color: parent.parent.isModeSelected ? "#05303a" : "#8eb4d0"; font.pixelSize: 15; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                            Text { text: "Automatic"; color: parent.parent.isModeSelected ? "#0c5a66" : "#8eb4d0"; font.pixelSize: 11; anchors.horizontalCenter: parent.horizontalCenter }
                                        }
                                    }

                                    // MANUAL
                                    Rectangle {
                                        width: parent.width
                                        height: (parent.height - 6) / 2
                                        radius: 5
                                        property bool isModeSelected: cartridgeController.currentMode === "manual" || cartridgeController.currentMode === "jog"
                                        color: isModeSelected ? "#5cf4f1" : "transparent"
                                        border.color: isModeSelected ? "#86bccb" : "#b9dfe1"
                                        border.width: 1
                                        Behavior on color { ColorAnimation { duration: 150 } }
                                        Behavior on border.color { ColorAnimation { duration: 150 } }
                                        Rectangle {
                                            anchors.fill: parent
                                            radius: parent.radius
                                            visible: parent.isModeSelected
                                            color: "#8094a3b8"
                                        }
                                        MotionMouseArea {
                                            anchors.fill: parent
                                            enabled: !modeSelCol.modeBlocked
                                            hoverScale: 1.05
                                            pressScale: 0.976
                                            shadowEnabled: false
                                            shimmerEnabled: false
                                            onClicked: {
                                                cartridgeController.setMode("manual")
                                                cartridgeController.startSystem()
                                                hpController.publishMode(2)
                                            }
                                        }
                                        Column {
                                            anchors.centerIn: parent
                                            spacing: 2
                                            Text { text: "MANUAL"; color: parent.parent.isModeSelected ? "#05303a" : "#8eb4d0"; font.pixelSize: 15; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                            Text { text: "Direct Control"; color: parent.parent.isModeSelected ? "#0c5a66" : "#8eb4d0"; font.pixelSize: 11; anchors.horizontalCenter: parent.horizontalCenter }
                                        }
                                    }
                                }
                            }
                        }

                        // ── System Control ───────────────────────
                        Rectangle {
                            Layout.preferredWidth: (parent.width - root.sensorW - root.gap) * 0.208
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            color: root.cBg2; border.color: root.cBorder; radius: 6
                            HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cBorder }

                            ColumnLayout {
                                anchors.fill: parent; anchors.margins: 8
                                spacing: 4
                                // Không cho chạy khi chưa chọn mode
                                enabled: !modeSelCol.modeIsIdle
                                opacity: modeSelCol.modeIsIdle ? 0.35 : 1.0
                                Behavior on opacity { NumberAnimation { duration: 200 } }

                                Text {
                                    text: "SYSTEM CONTROL"; color: "#5cf4f1"
                                    font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5
                                }

                                // 2 columns x 2 rows — same GridLayout structure as other cards
                                GridLayout {
                                    Layout.fillWidth: true; Layout.fillHeight: true
                                    columns: 2; columnSpacing: 4; rowSpacing: 4

                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "START";  bg: "#0b7876"; bgEnd: "#095f5d"; bc: root.cAccent; tc: "#d4faff"; clickEnabled: !root.startCommandLocked; onClicked: {
                                            if (root.startCommandLocked)
                                                return
                                            root.startCommandLocked = true
                                            if (cartridgeController.currentMode === "auto") {
                                                cartridgeController.setMode("auto")
                                                robotController.setAutoMode(true)
                                                hpController.publishMode(0)
                                            } else if (cartridgeController.currentMode === "manual" || cartridgeController.currentMode === "jog") {
                                                cartridgeController.setMode("manual")
                                                robotController.setManualMode(true)
                                                hpController.publishMode(2)
                                            }
                                            cartridgeController.startSystem()
                                        } }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "RESUME"; bg: "#0a405c"; bgEnd: "#052b3d"; bc: root.cAccent; tc: "#d4faff"; onClicked: cartridgeController.resumeSystem() }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "STOP";   bg: "#771a1a"; bgEnd: "#4e0c0c"; bc: root.cRed;    tc: "#d4faff"; blinking: cartridgeController.uiHint === "press_stop"; onClicked: { root.startCommandLocked = false; root.cancelHoming(); robotController.stopAndResetRobot(); cartridgeController.stopSystem() } }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "PAUSE";  bg: "#0a405c"; bgEnd: "#052b3d"; bc: root.cAccent; tc: "#d4faff"; onClicked: cartridgeController.pauseSystem() }
                                }
                            }
                        }

                        // ── State Navigation ─────────────────────
                        Rectangle {
                            Layout.preferredWidth: (parent.width - root.sensorW - root.gap) * 0.313
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            color: root.cBg2; border.color: root.cBorder; radius: 6
                            HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cBorder }

                            ColumnLayout {
                                anchors.fill: parent; anchors.margins: 8
                                spacing: 4
                                // Không cho chạy khi chưa chọn mode
                                enabled: !modeSelCol.modeIsIdle
                                opacity: modeSelCol.modeIsIdle ? 0.35 : 1.0
                                Behavior on opacity { NumberAnimation { duration: 200 } }

                                Text {
                                    text: "STATE NAVIGATION"; color: "#5cf4f1"
                                    font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5
                                }

                                // 3 columns x 2 rows square grid
                                GridLayout {
                                    Layout.fillWidth: true; Layout.fillHeight: true
                                    columns: 3; columnSpacing: 4; rowSpacing: 4

                                    CBtn {
                                        Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1
                                        lbl: "HOMING"
                                        bg: root.cStateAuxBtn
                                        bc: root.cStateAuxBorder
                                        tc: root.cStateAuxText
                                        isSelected: root.homingBusy()
                                        clickEnabled: !root.homingBusy()
                                        blinking: cartridgeController.uiHint === "press_homing"
                                        onClicked: {
                                            root.homingCommandLocked = true
                                            homingLockFailsafeTimer.restart()
                                            robotController.gotoState("HOMING")
                                            cartridgeController.gotoState("HOMING")
                                        }
                                    }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "STATE 1\nKhay In"; bg: root.cUnifiedBtn; bc: root.cGreen; tc: root.cGreen; isSelected: cartridgeController.systemState.indexOf("S1_") !== -1 || cartridgeController.systemState.indexOf("STATE1") !== -1; onClicked: cartridgeController.gotoState("STATE1") }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "STATE 3\nKhay Out"; bg: root.cUnifiedBtn; bc: root.cGreen; tc: root.cGreen; isSelected: cartridgeController.systemState.indexOf("S3_") !== -1 || cartridgeController.systemState.indexOf("STATE3") !== -1; onClicked: cartridgeController.gotoState("STATE3") }

                                    CBtn {
                                        Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1
                                        lbl: cartridgeController.currentMode === "jog" ? "STATE MODE" : "STOP STATE"
                                        bg: "#771a1a"
                                        bgEnd: "#4e0c0c"
                                        bc: root.cRed
                                        tc: "#d4faff"
                                        isSelected: cartridgeController.currentMode === "jog" || cartridgeController.systemState.toLowerCase().indexOf("jog") !== -1
                                        onClicked: {
                                            root.jogStopStateHint = false
                                            root.cancelHoming()
                                            if (cartridgeController.currentMode === "jog") {
                                                cartridgeController.setMode("manual")
                                                hpController.publishMode(2)  // sync Fill HP → Manual
                                            } else {
                                                cartridgeController.gotoState("ABORT_TO_JOG")
                                            }
                                        }
                                    }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "STATE 2\nKhay In"; bg: root.cUnifiedBtn; bc: root.cGreen; tc: root.cGreen; isSelected: cartridgeController.systemState.indexOf("S2A_") !== -1 || cartridgeController.systemState.indexOf("STATE2") !== -1; onClicked: cartridgeController.gotoState("STATE2") }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "STATE 4\nKhay Out"; bg: root.cUnifiedBtn; bc: root.cGreen; tc: root.cGreen; isSelected: cartridgeController.systemState.indexOf("S4_") !== -1 || cartridgeController.systemState.indexOf("STATE4") !== -1; onClicked: cartridgeController.gotoState("STATE4") }
                                }
                            }
                        }

                        // ── Control Cylinder ──────────────────────
                        Rectangle {
                            Layout.preferredWidth: (parent.width - root.sensorW - root.gap) * 0.313
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            color: root.cBg2; border.color: root.cBorder; radius: 6
                            HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cBorder }

                            ColumnLayout {
                                anchors.fill: parent; anchors.margins: 8
                                spacing: 4
                                property bool cylEnabled: cartridgeController.currentMode === "jog"
                                enabled: cylEnabled
                                opacity: cylEnabled ? 1.0 : 0.35
                                Behavior on opacity { NumberAnimation { duration: 200 } }

                                Text {
                                    text: "CONTROL CYLINDER"; color: "#6cf"
                                    font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5
                                }

                                // 3 columns x 2 rows — same GridLayout structure as State Navigation
                                GridLayout {
                                    Layout.fillWidth: true; Layout.fillHeight: true
                                    columns: 3; columnSpacing: 4; rowSpacing: 4

                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "INY\nEXTEND";  bg: root.cUnifiedBtn; bc: root.cGreen; tc: root.cGreen; isSelected: cartridgeController.sensorState.length >= 10 && cartridgeController.sensorState.charAt(9) === '1'; onClicked: cartridgeController.cylinderCmd(1, true) }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "OUTY\nEXTEND"; bg: root.cUnifiedBtn; bc: root.cGreen; tc: root.cGreen; isSelected: cartridgeController.sensorState.length >= 22 && cartridgeController.sensorState.charAt(21) === '1'; onClicked: cartridgeController.cylinderCmd(2, true) }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "HOLD\nEXTEND"; bg: root.cUnifiedBtn; bc: root.cGreen; tc: root.cGreen; isSelected: cartridgeController.sensorState.length >= 16 && cartridgeController.sensorState.charAt(15) === '1'; onClicked: cartridgeController.cylinderCmd(3, true) }

                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "INY\nRETRACT";  bg: root.cUnifiedBtn; bc: root.cGreen; tc: root.cGreen; isSelected: cartridgeController.sensorState.length >= 9 && cartridgeController.sensorState.charAt(8) === '1'; onClicked: cartridgeController.cylinderCmd(1, false) }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "OUTY\nRETRACT"; bg: root.cUnifiedBtn; bc: root.cGreen; tc: root.cGreen; isSelected: cartridgeController.sensorState.length >= 21 && cartridgeController.sensorState.charAt(20) === '1'; onClicked: cartridgeController.cylinderCmd(2, false) }
                                    CBtn { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; Layout.preferredHeight: 1; lbl: "HOLD\nRETRACT"; bg: root.cUnifiedBtn; bc: root.cGreen; tc: root.cGreen; isSelected: cartridgeController.sensorState.length >= 15 && cartridgeController.sensorState.charAt(14) === '1'; onClicked: cartridgeController.cylinderCmd(3, false) }
                                }
                            }
                        }
                    }
                    // ─ SERVO CONTROL AREA ────────────────────────
                    Rectangle {
                        x: 0
                        y: topCardsRow.height + root.gap
                        width: parent.width - root.sensorW - root.gap
                        height: root.topH - topCardsRow.height - root.gap
                        color: root.cBg2; border.color: root.cBorder; radius: 6; clip: true
                        HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cBorder }

                        Column {
                            anchors.fill: parent
                            anchors.margins: 8
                            spacing: 4

                            Row { width: parent.width; height: 24; spacing: 6
                                Text { text: "SERVO CONTROL"; color: "#6cf"; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5; anchors.verticalCenter: parent.verticalCenter }
                                // hidden data source — syncs jogVelMms from FAS PNU via _jog_vel topic
                                Item {
                                    id: velDisplay
                                    visible: false; width: 0; height: 0
                                    property int jogVelMms: 30
                                    Connections {
                                        target: cartridgeController
                                        function onServoPositionsChanged() {
                                            try {
                                                var d = JSON.parse(cartridgeController.servoPositions)
                                                if (d["_jog_vel"] !== undefined)
                                                    velDisplay.jogVelMms = Math.round(Number(d["_jog_vel"]) * 1000)
                                            } catch(e) {}
                                        }
                                    }
                                }
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
                                        id: cardItem
                                        property int jogVelMms: 30
                                        Connections {
                                            target: cartridgeController
                                            function onServoPositionsChanged() {
                                                try {
                                                    var fv = JSON.parse(cartridgeController.servoPositions)["_fas_vel"]
                                                    if (fv && fv[String(model.sid)] !== undefined)
                                                        cardItem.jogVelMms = Math.round(Number(fv[String(model.sid)]) * 1000)
                                                } catch(e) {}
                                            }
                                        }
                                        width: Math.floor((servoRow.width - 4*root.gap) / 5)
                                        height: servoRow.height
                                        color: root.cCard; border.color: root.cBorder; radius: 4; clip: true
                                        HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cBorder }

                                        Column {
                                            anchors.fill: parent
                                            anchors.margins: 6
                                            spacing: 6; width: parent.width - 12

                                            // header: name + desc
                                            Column { width: parent.width; spacing: 2
                                                Text { text: "S"+model.sid+": "+model.sname; color: root.cCyan; font.pixelSize: 20; font.bold: true; width: parent.width; horizontalAlignment: Text.AlignHCenter }
                                                Text { text: model.sdesc; color: root.cDim; font.pixelSize: 17; width: parent.width; horizontalAlignment: Text.AlignHCenter; elide: Text.ElideRight }
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

                                            // VELOCITY Row (aligned to left label)
                                            RowLayout {
                                                width: parent.width
                                                spacing: 8
                                                Text {
                                                    text: "VELOCITY"
                                                    color: root.cDim
                                                    font.pixelSize: 16
                                                    font.bold: true
                                                    Layout.alignment: Qt.AlignVCenter
                                                    Layout.preferredWidth: 82
                                                }
                                                Rectangle {
                                                    Layout.fillWidth: true
                                                    Layout.preferredHeight: 42
                                                    radius: 6
                                                    color: "#081e29"; border.color: root.cBorder; border.width: 1
                                                    Text {
                                                        id: velText
                                                        anchors.centerIn: parent
                                                        text: cardItem.jogVelMms > 0 ? (cardItem.jogVelMms / 1000.0).toFixed(3) + " m/s" : "–"
                                                        color: root.cCyan
                                                        font.pixelSize: 16; font.bold: true; font.family: "monospace"
                                                    }
                                                }
                                            }

                                            // JOG Row (with - and + buttons, right-aligned and spanning width)
                                            RowLayout {
                                                width: parent.width
                                                spacing: 8
                                                Text {
                                                    text: "JOG"
                                                    color: root.cDim
                                                    font.pixelSize: 16
                                                    font.bold: true
                                                    Layout.alignment: Qt.AlignVCenter
                                                    Layout.preferredWidth: 82
                                                }
                                                RowLayout {
                                                    Layout.fillWidth: true
                                                    spacing: 8
                                                    CBtn {
                                                        iconSource: "qrc:/icons/qml/icons/jog_neg.png"
                                                        Layout.fillWidth: true; Layout.preferredWidth: 1
                                                        Layout.preferredHeight: 42
                                                        padV: 9; padH: 0; fontSize: 20
                                                       bg: "#0a243a"; bc: "#6cf"; tc: "#6cf"
                                                        active: servoRow.jogAllowed
                                                        onPressed: {
                                                            if(servoRow.jogAllowed) {
                                                                root.showJogStopStateHint()
                                                                cartridgeController.jogServo(model.sid,"-", cardItem.jogVelMms)
                                                            }
                                                        }
                                                        onReleased: {
                                                            cartridgeController.jogStop(model.sid)
                                                            root.fadeJogStopStateHint()
                                                        }
                                                    }
                                                    CBtn {
                                                        iconSource: "qrc:/icons/qml/icons/jog_plus.png"
                                                        Layout.fillWidth: true; Layout.preferredWidth: 1
                                                        Layout.preferredHeight: 42
                                                        padV: 9; padH: 0; fontSize: 20
                                                        bg: "#0a243a"; bc: "#6cf"; tc: "#6cf"
                                                        active: servoRow.jogAllowed
                                                        onPressed: {
                                                            if(servoRow.jogAllowed) {
                                                                root.showJogStopStateHint()
                                                                cartridgeController.jogServo(model.sid,"+", cardItem.jogVelMms)
                                                            }
                                                        }
                                                        onReleased: {
                                                            cartridgeController.jogStop(model.sid)
                                                            root.fadeJogStopStateHint()
                                                        }
                                                    }
                                                }
                                            }

                                            // HOMING & CLEAR combined side-by-side to save vertical space
                                            Row {
                                                spacing: 6
                                                width: parent.width
                                                CBtn { lbl:"HOMING"; w:(parent.width - 6)/2; h:42; padV:9; fontSize: 16; bg:root.cCard; bc:root.cAccent; tc:"#d4faff"; active:servoRow.jogAllowed; onClicked: { if(servoRow.jogAllowed) cartridgeController.homeServo(model.sid) } }
                                                CBtn { lbl:"CLEAR";  w:(parent.width - 6)/2; h:42; padV:9; fontSize: 16; bg:root.cCard; bc:root.cBorder; tc:"#d4faff"; onClicked: cartridgeController.clearServo(model.sid) }
                                            }

                                            // TARGET POSITION Row (with input & RUN button)
                                            RowLayout {
                                                width: parent.width
                                                spacing: 8
                                                Text {
                                                    text: "TARGET\nPOSITION"
                                                    color: root.cDim
                                                    font.pixelSize: 11
                                                    font.bold: true
                                                    Layout.alignment: Qt.AlignVCenter
                                                    Layout.preferredWidth: 82
                                                }
                                                RowLayout {
                                                    Layout.fillWidth: true
                                                    spacing: 4
                                                    Rectangle {
                                                        Layout.fillWidth: true
                                                        Layout.preferredHeight: 42
                                                        radius: 6
                                                        color: "#081e29"; border.color: "#6cf"; border.width: 2
                                                        TextInput {
                                                            id: posIn
                                                            anchors.fill: parent; anchors.margins: 4
                                                            text: "0.0"; font.pixelSize: 20; font.bold: true; font.family: "monospace"
                                                            color: "#6cf"
                                                            horizontalAlignment: TextInput.AlignHCenter; verticalAlignment: TextInput.AlignVCenter
                                                        }
                                                    }
                                                    Text { 
                                                        text: "mm"
                                                        color: "#6cf"
                                                        font.pixelSize: 15
                                                        font.bold: true
                                                        Layout.alignment: Qt.AlignBottom
                                                        Layout.preferredWidth: 24
                                                        horizontalAlignment: Text.AlignHCenter 
                                                    }
                                                    CBtn {
                                                        lbl: "RUN"
                                                        Layout.preferredWidth: 80
                                                        Layout.preferredHeight: 42
                                                        padV: 0; fontSize: 16
                                                        bg: "#0d2a3a"; bc: root.cGreen; tc: root.cGreen; active: servoRow.jogAllowed
                                                        onClicked: { if(servoRow.jogAllowed) { var v=parseFloat(posIn.text); if(!isNaN(v)) cartridgeController.moveServo(model.sid,v) } }
                                                    }
                                                }
                                            }

                                            // Limits display
                                            Text {
                                                text: model.sid === 1 ? "Min: -322 | Max: 560" : (model.sid === 2 ? "Min: -80 | Max: 1025" : "")
                                                color: "#94a3b8"
                                                font.pixelSize: 13
                                                font.bold: true
                                                width: parent.width
                                                horizontalAlignment: Text.AlignHCenter
                                                visible: model.sid === 1 || model.sid === 2
                                            }

                                            // STOP button (full width safety button at bottom)
                                                CBtn {
                                                    lbl: "STOP"
                                                    w: parent.width; h: 42
                                                    padV: 9
                                                    fontSize: 18
                                                    bg: "#771a1a"
                                                    bgEnd: "#4e0c0c"
                                                    bc: root.cRed
                                                    tc: root.cRed
                                                    onClicked: cartridgeController.jogStop(model.sid)
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
                        HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cBorder }

                        Column {
                            anchors.fill: parent
                            anchors.margins: 8
                            spacing: 4
                            RowLayout { width: parent.width; height: 24
                                Text { text: "LOG ACTIVITY"; color: "#6cf"; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5 }
                                Item { Layout.fillWidth: true }
                                CBtn { lbl:"Clear"; padV:4; padH:10; fontSize: 15; bg:root.cWarnBlueBg; bgEnd:root.cWarnBlueBgEnd; bc:root.cWarnBlueBorder; tc:root.cWarnBlueText; onClicked: cartridgeController.clearLog() }
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

                    // ─ SENSOR SIGNALS (grid-area: servo, full height) ──
                    // Hiển thị read-only 20 sensor THẬT (S1-S10, S13-S22) từ IO module qua
                    // topic /providesystem/sensors_state. S11/S12 là VFD status (ATV Run/
                    // Fault) — monitor bởi vfd_logic_node, không hiển thị ở grid này. Cập
                    // nhật real-time, không click. Ô được thu nhỏ để fit vừa card.
                    Rectangle {
                        x: parent.width - root.sensorW
                        y: 0; width: root.sensorW; height: root.outerH
                        color: root.cBg2; border.color: root.cBorder; radius: 6
                        HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cBorder }

                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: 6
                            spacing: 3

                            // ── Tiêu đề ──
                            Text {
                                text: "SENSOR SIGNAL"
                                color: "#6cf"; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5
                            }

                            // ── Status label ──
                            Text { text: "STATUS"; color: root.cDim; font.pixelSize: 10; font.bold: true; font.letterSpacing: 1 }

                            // ── Grid sensor – fillHeight để tự co vừa chiều cao còn lại ──
                            GridLayout {
                                Layout.fillWidth: true
                                Layout.fillHeight: true      // ← KEY: chiếm hết không gian còn lại
                                columns: 2
                                columnSpacing: 4
                                rowSpacing: 2

                                Repeater {
                                    model: ListModel {
                                        // [CPX 253] Module 2: I1.0–I1.7
                                        ListElement { sid:1;  slabel:"S1";  sdesc:"Belt start" }
                                        ListElement { sid:2;  slabel:"S2";  sdesc:"Belt mid" }
                                        ListElement { sid:3;  slabel:"S3";  sdesc:"Belt end" }
                                        ListElement { sid:4;  slabel:"S4";  sdesc:"Scan Stack Pos1" }
                                        ListElement { sid:5;  slabel:"S5";  sdesc:"Output det." }
                                        ListElement { sid:6;  slabel:"S6";  sdesc:"Check Tray OutP1" }
                                        ListElement { sid:7;  slabel:"S7";  sdesc:"Khay tại Robot" }
                                        ListElement { sid:8;  slabel:"S8";  sdesc:"[Reserved]" }
                                        // [CPX 253] Module 3: I2.0–I2.7
                                        // (S11/S12 ATV Run/Fault — VFD status, không hiển thị ở grid này)
                                        ListElement { sid:9;  slabel:"S9";  sdesc:"Cyl1 Ret" }
                                        ListElement { sid:10; slabel:"S10"; sdesc:"Cyl1 Ext" }
                                        ListElement { sid:13; slabel:"S13"; sdesc:"OUT1 TrayPos1" }
                                        ListElement { sid:14; slabel:"S14"; sdesc:"OUT2 TrayPos1" }
                                        ListElement { sid:15; slabel:"S15"; sdesc:"Cyl3 Ret" }
                                        ListElement { sid:16; slabel:"S16"; sdesc:"Cyl3 Ext" }
                                        // [CPX 254] Module 2: I3.0–I3.5
                                        ListElement { sid:17; slabel:"S17"; sdesc:"Platform" }
                                        ListElement { sid:18; slabel:"S18"; sdesc:"Feed OK" }
                                        ListElement { sid:19; slabel:"S19"; sdesc:"Check Tray OutP2" }
                                        ListElement { sid:20; slabel:"S20"; sdesc:"Scan Stack Pos2" }
                                        ListElement { sid:21; slabel:"S21"; sdesc:"Cyl2 Ret" }
                                        ListElement { sid:22; slabel:"S22"; sdesc:"Cyl2 Ext" }
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
                                        Layout.minimumHeight: 20         // ← thu nhỏ để fit đủ 20 sensor

                                        radius: 3
                                        color: on_ ? "#0d2a3a" : root.cCard
                                        border.color: on_ ? "#6cf" : root.cBorder
                                        Behavior on color       { ColorAnimation { duration: 150 } }
                                        Behavior on border.color { ColorAnimation { duration: 150 } }
                                        HoverHandler { onHoveredChanged: if(!sBtn.on_) sBtn.border.color = hovered ? root.cCyan : root.cBorder }
                                        Column {
                                            anchors.centerIn: parent
                                            spacing: 0
                                            Text {
                                                text: model.slabel
                                                color: sBtn.on_ ? "#6cf" : root.cText
                                                font.pixelSize: 13; font.bold: true
                                                anchors.horizontalCenter: parent.horizontalCenter
                                            }
                                            Text {
                                                text: model.sdesc
                                                color: root.cDim; font.pixelSize: 10
                                                anchors.horizontalCenter: parent.horizontalCenter
                                                visible: model.sdesc !== ""
                                            }
                                            Rectangle {
                                                id: dotIndicator
                                                width: 4; height: 4; radius: 2
                                                color: sBtn.on_ ? "#6cf" : "#134357"
                                                anchors.horizontalCenter: parent.horizontalCenter

                                                Repeater {
                                                    model: 2
                                                    delegate: Rectangle {
                                                        id: ripple
                                                        anchors.centerIn: parent
                                                        width: 6; height: 6; radius: 3
                                                        color: "transparent"
                                                        border.color: "#6cf"
                                                        border.width: 1
                                                        opacity: 0
                                                        visible: sBtn.on_

                                                        SequentialAnimation {
                                                            running: sBtn.on_
                                                            loops: Animation.Infinite
                                                            PauseAnimation { duration: index * 1000 }
                                                            ParallelAnimation {
                                                                NumberAnimation {
                                                                    target: ripple
                                                                    property: "opacity"
                                                                    from: 0.8; to: 0.0
                                                                    duration: 1500
                                                                    easing.type: Easing.OutQuad
                                                                }
                                                                NumberAnimation {
                                                                    target: ripple
                                                                    property: "scale"
                                                                    from: 1.0; to: 5.0
                                                                    duration: 1500
                                                                    easing.type: Easing.OutQuad
                                                                }
                                                            }
                                                            PauseAnimation { duration: (1 - index) * 1000 }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
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
                        id: servoParamsCard
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        color: root.cBg2; border.color: root.cBorder; radius: 6
                        HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cBorder }

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
                            { key:"target_scanoutp2",label:"OUTY TgtScan",  desc:"Điểm dừng quét S20" },
                            { key:"outy_scan_arm_mm",label:"OUTY Arm S20",  desc:"Giới hạn kích hoạt S20" }
                        ]

                        Flickable {
                            id: servoFlickable2
                            anchors { fill: parent; margins: 8 }
                            contentWidth: width; contentHeight: servoInfoCol2.height + 20
                            clip: true

                            Column {
                                id: servoInfoCol2
                                width: parent.width
                                spacing: 4

                                Text { text: "SERVO KEY POSITIONS (mm)"; color: root.cAccent; font.pixelSize: 18; font.bold: true; font.letterSpacing: 1.5 }
                                Row { width: parent.width; spacing: parent.width * 0.015
                                    Repeater { model: ["Parameter","Value","Description"]
                                        delegate: Text {
                                            text: modelData
                                            color: root.cDim
                                            font.pixelSize: 14; font.bold: true
                                            width: index===0 ? parent.width * 0.40 : index===1 ? parent.width * 0.22 : parent.width * 0.35
                                            font.capitalization: Font.AllUppercase; font.letterSpacing: 1
                                        }
                                    }
                                }
                                Rectangle { width: parent.width; height: 1; color: root.cBorder }

                                Repeater {
                                    id: servoRepeater2
                                    model: servoParamsCard.servoParams
                                    delegate: Rectangle {
                                        required property var modelData
                                        required property int index
                                        width: servoInfoCol2.width; height: 46
                                        color: index % 2 === 0 ? "transparent" : "#0d0d22"
                                        property alias inputText: sInput2.text
                                        property string paramKey: modelData.key
                                        Row {
                                            anchors.verticalCenter: parent.verticalCenter
                                            width: parent.width
                                            spacing: parent.width * 0.015
                                            Text {
                                                text: modelData.label
                                                color: root.cCyan
                                                font { pixelSize: 18; bold: true }
                                                width: parent.width * 0.40
                                                anchors.verticalCenter: parent.verticalCenter
                                            }
                                            Rectangle {
                                                width: parent.width * 0.22; height: 36; radius: 5
                                                color: root.cBg; border.color: root.cBorder
                                                TextInput {
                                                    id: sInput2
                                                    anchors { fill: parent; margins: 3 }
                                                    text: "0.0"; font.pixelSize: 18; font.family: "monospace"; font.bold: true
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
                                            Text {
                                                text: modelData.desc
                                                color: root.cDim
                                                font.pixelSize: 16
                                                elide: Text.ElideRight
                                                anchors.verticalCenter: parent.verticalCenter
                                                width: parent.width * 0.35
                                            }
                                        }
                                    }
                                }

                                Row { spacing: 8; topPadding: 8
                                    CBtn { lbl:"Save All"; padV:10; padH:22; fontSize: 18; bg:"#0d2a3a"; bc:root.cGreen; tc:root.cGreen
                                        onClicked: {
                                            for (var i = 0; i < servoRepeater2.count; i++) {
                                                var item = servoRepeater2.itemAt(i)
                                                if (item && item.inputText !== "")
                                                    cartridgeController.saveConfig(item.paramKey, item.inputText)
                                            }
                                        }
                                    }
                                    CBtn { lbl:"↺ Reset"; padV:10; padH:18; fontSize: 18; bg:root.cCard; bc:root.cBorder; tc:root.cText; onClicked: page2Root.reloadConfig() }
                                }
                            }
                        }

                        Rectangle {
                            id: servoScroll2
                            anchors { right: parent.right; top: parent.top; bottom: parent.bottom; rightMargin: 2; topMargin: 8; bottomMargin: 8 }
                            width: 4; radius: 2; color: "#1f2937"
                            visible: servoFlickable2.height < servoFlickable2.contentHeight

                            Rectangle {
                                width: parent.width; radius: 2; color: root.cAccent
                                height: Math.max(20, servoFlickable2.height * (servoFlickable2.height / servoFlickable2.contentHeight))
                                y: servoFlickable2.visibleArea.yPosition * servoFlickable2.height
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
                                CBtn { lbl:"Clear"; padV:3; padH:8; fontSize: 14; bg:root.cCard; bc:root.cBorder; tc:root.cDim; onClicked: cartridgeController.clearLog() }
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
                // MANUAL controls (JOG) chỉ enable khi robot rảnh — "MANUAL" và "IDLE" đều coi là rảnh.
                property bool manualEnabled: robotController.systemStatus === "IDLE" || robotController.systemStatus === "MANUAL" || robotController.systemStatus === "UNKNOWN" || robotController.systemStatus === ""
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
                                        anchors { fill: parent; margins: 8 }
                                        spacing: 6
                                        Row { spacing: 6
                                            Rectangle { width: 4; height: 16; radius: 1; color: root.cAccent; anchors.verticalCenter: parent.verticalCenter }
                                            Text { text: "CARTESIAN (mm)"; color: root.cAccent; font.pixelSize: 15; font.bold: true; font.letterSpacing: 1.2 }
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
                                                width: cartCol.width; height: 52; spacing: 4
                                                Rectangle {
                                                    id: negBtn
                                                    width: 58; height: 48; radius: 5
                                                    color: negMA.pressed ? "#4a1d24" : root.cCard
                                                    border.color: negMA.pressed ? "#d44747" : root.cBorder; border.width: negMA.pressed ? 2 : 1
                                                    Behavior on color { ColorAnimation { duration: 80 } }
                                                    Behavior on border.color { ColorAnimation { duration: 80 } }
                                                    Text { anchors.centerIn: parent; text: modelData.axis + "-"; color: negMA.pressed ? "#ff8a8a" : root.cRed; font.pixelSize: 16; font.bold: true }
                                                    MotionMouseArea { id: negMA; anchors.fill: parent; hoverScale: 1.05; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false; onPressed: robotController.jogStart(modelData.neg); onReleased: robotController.jogStop(); onCanceled: robotController.jogStop() }
                                                }
                                                Rectangle {
                                                    width: parent.width - 120; height: 48; radius: 5; color: "#0d1117"; border.width: 2; border.color: root.cBorder
                                                    Text { anchors.centerIn: parent; text: robotController.cartesianPose.length > index ? robotController.cartesianPose[index].toFixed(4) : "0.0000"; color: "#f59e0b"; font.pixelSize: 20; font.family: "monospace"; font.bold: true }
                                                }
                                                Rectangle {
                                                    id: posBtn
                                                    width: 58; height: 48; radius: 5
                                                    color: posMA.pressed ? "#133b26" : root.cCard
                                                    border.color: posMA.pressed ? "#1ecb70" : root.cBorder; border.width: posMA.pressed ? 2 : 1
                                                    Behavior on color { ColorAnimation { duration: 80 } }
                                                    Behavior on border.color { ColorAnimation { duration: 80 } }
                                                    Text { anchors.centerIn: parent; text: modelData.axis + "+"; color: posMA.pressed ? "#6ee7a8" : root.cGreen; font.pixelSize: 16; font.bold: true }
                                                    MotionMouseArea { id: posMA; anchors.fill: parent; hoverScale: 1.05; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false; onPressed: robotController.jogStart(modelData.pos); onReleased: robotController.jogStop(); onCanceled: robotController.jogStop() }
                                                }
                                            }
                                        }
                                        Rectangle { width: parent.width; height: 1; color: root.cBorder }
                                        Row { spacing: 3; width: parent.width
                                            Repeater {
                                                id: cartInputs
                                                model: ["X","Y","Z","RX","RY","RZ"]
                                                delegate: Column { spacing: 2; width: (cartCol.width - 15) / 6
                                                    Text { text: modelData; color: root.cDim; font.pixelSize: 12; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                                    Rectangle {
                                                        width: parent.width; height: 40; radius: 4; color: "#0d1117"; border.color: root.cAccent; border.width: 2
                                                        TextInput { id: cartInp
                                                            anchors { fill: parent; margins: 2 }
                                                            color: root.cText; font.pixelSize: 16; font.family: "monospace"; horizontalAlignment: Text.AlignHCenter; clip: true
                                                            text: robotController.cartesianPose.length > index ? robotController.cartesianPose[index].toFixed(4) : "0"
                                                            selectByMouse: true; verticalAlignment: Text.AlignVCenter; validator: DoubleValidator { notation: DoubleValidator.StandardNotation }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                        Row { spacing: 4; width: parent.width
                                            Rectangle {
                                                width: parent.width / 2 - 2; height: 42; radius: 5
                                                color: gpMA.pressed ? "#123c2a" : "#0a2a1a"; border.color: gpMA.pressed ? "#42d8e5" : root.cCyan
                                                border.width: gpMA.pressed ? 2 : 1
                                                Behavior on color { ColorAnimation { duration: 80 } }
                                                Text { anchors.centerIn: parent; text: "GET POSE"; color: gpMA.pressed ? "#8ceef4" : root.cCyan; font.pixelSize: 14; font.bold: true }
                                                MotionMouseArea { id: gpMA; anchors.fill: parent; hoverScale: 1.05; shadowEnabled: false; shimmerEnabled: false; onClicked: {
                                                    robotController.getPose()
                                                    for (var i = 0; i < 6; i++) {
                                                        if (robotController.cartesianPose.length > i)
                                                            cartInputs.itemAt(i).children[1].children[0].text = robotController.cartesianPose[i].toFixed(4)
                                                    }
                                                }}
                                            }
                                            Rectangle {
                                                width: parent.width / 2 - 2; height: 42; radius: 5
                                                color: mlMA.pressed ? "#132d4f" : "#0a1a35"; border.color: mlMA.pressed ? "#6f82ff" : root.cAccent
                                                border.width: mlMA.pressed ? 2 : 1
                                                Behavior on color { ColorAnimation { duration: 80 } }
                                                Text { anchors.centerIn: parent; text: "SEND MovL"; color: mlMA.pressed ? "#aab6ff" : root.cAccent; font.pixelSize: 14; font.bold: true }
                                                MotionMouseArea { id: mlMA; anchors.fill: parent; hoverScale: 1.05; shadowEnabled: false; shimmerEnabled: false; onClicked: {
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
                                        anchors { fill: parent; margins: 8 }
                                        spacing: 6
                                        Row { spacing: 6
                                            Rectangle { width: 4; height: 16; radius: 1; color: "#5cf4f1"; anchors.verticalCenter: parent.verticalCenter }
                                            Text { text: "JOINT (deg)"; color: "#5cf4f1"; font.pixelSize: 15; font.bold: true; font.letterSpacing: 1.2 }
                                        }
                                        Repeater {
                                            id: jointRep
                                            model: 6
                                            delegate: Row {
                                                property int jn: index + 1
                                                width: jointCol.width; height: 52; spacing: 4
                                                Rectangle {
                                                    id: jnBtn
                                                    width: 58; height: 48; radius: 5
                                                    color: jnMA.pressed ? "#4a1d24" : root.cCard
                                                    border.color: jnMA.pressed ? "#d44747" : root.cBorder; border.width: jnMA.pressed ? 2 : 1
                                                    Behavior on color { ColorAnimation { duration: 80 } }
                                                    Behavior on border.color { ColorAnimation { duration: 80 } }
                                                    Text { anchors.centerIn: parent; text: "J" + jn + "-"; color: jnMA.pressed ? "#ff8a8a" : root.cRed; font.pixelSize: 16; font.bold: true }
                                                    MotionMouseArea { id: jnMA; anchors.fill: parent; hoverScale: 1.05; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false; onPressed: robotController.jogStart("j" + jn + "-"); onReleased: robotController.jogStop(); onCanceled: robotController.jogStop() }
                                                }
                                                Rectangle {
                                                    width: parent.width - 120; height: 48; radius: 5; color: "#0d1117"; border.width: 2; border.color: root.cBorder
                                                    Text { anchors.centerIn: parent; text: robotController.jointAngles.length > index ? robotController.jointAngles[index].toFixed(4) : "0.0000"; color: "#f59e0b"; font.pixelSize: 20; font.family: "monospace"; font.bold: true }
                                                }
                                                Rectangle {
                                                    id: jpBtn
                                                    width: 58; height: 48; radius: 5
                                                    color: jpMA.pressed ? "#133b26" : root.cCard
                                                    border.color: jpMA.pressed ? "#1ecb70" : root.cBorder; border.width: jpMA.pressed ? 2 : 1
                                                    Behavior on color { ColorAnimation { duration: 80 } }
                                                    Behavior on border.color { ColorAnimation { duration: 80 } }
                                                    Text { anchors.centerIn: parent; text: "J" + jn + "+"; color: jpMA.pressed ? "#6ee7a8" : root.cGreen; font.pixelSize: 16; font.bold: true }
                                                    MotionMouseArea { id: jpMA; anchors.fill: parent; hoverScale: 1.05; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false; onPressed: robotController.jogStart("j" + jn + "+"); onReleased: robotController.jogStop(); onCanceled: robotController.jogStop() }
                                                }
                                            }
                                        }
                                        Rectangle { width: parent.width; height: 1; color: root.cBorder }
                                        Row { spacing: 3; width: parent.width
                                            Repeater {
                                                id: jointInputs
                                                model: ["J1","J2","J3","J4","J5","J6"]
                                                delegate: Column { spacing: 2; width: (jointCol.width - 15) / 6
                                                    Text { text: modelData; color: root.cDim; font.pixelSize: 12; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                                    Rectangle {
                                                        width: parent.width; height: 40; radius: 4; color: "#0d1117"; border.color: "#5cf4f1"; border.width: 2
                                                        TextInput {
                                                            anchors { fill: parent; margins: 2 }
                                                            color: root.cText; font.pixelSize: 16; font.family: "monospace"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; clip: true
                                                            text: robotController.jointAngles.length > index ? robotController.jointAngles[index].toFixed(4) : "0"
                                                            selectByMouse: true; validator: DoubleValidator { notation: DoubleValidator.StandardNotation }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                        Row { spacing: 4; width: parent.width
                                            Rectangle {
                                                width: parent.width / 2 - 2; height: 42; radius: 5
                                                color: gaMA.pressed ? "#123c2a" : "#0a2a1a"; border.color: gaMA.pressed ? "#42d8e5" : root.cCyan
                                                border.width: gaMA.pressed ? 2 : 1
                                                Behavior on color { ColorAnimation { duration: 80 } }
                                                Text { anchors.centerIn: parent; text: "GET ANGLES"; color: gaMA.pressed ? "#8ceef4" : root.cCyan; font.pixelSize: 14; font.bold: true }
                                                MotionMouseArea { id: gaMA; anchors.fill: parent; hoverScale: 1.05; shadowEnabled: false; shimmerEnabled: false; onClicked: {
                                                    robotController.getAngles()
                                                    for (var i = 0; i < 6; i++) {
                                                        if (robotController.jointAngles.length > i)
                                                            jointInputs.itemAt(i).children[1].children[0].text = robotController.jointAngles[i].toFixed(4)
                                                    }
                                                }}
                                            }
                                            Rectangle {
                                                width: parent.width / 2 - 2; height: 42; radius: 5
                                                color: mjMA.pressed ? "#07303a" : "#051a25"; border.color: mjMA.pressed ? "#63dce7" : "#5cf4f1"
                                                border.width: mjMA.pressed ? 2 : 1
                                                Behavior on color { ColorAnimation { duration: 80 } }
                                                Text { anchors.centerIn: parent; text: "SEND MovJ"; color: mjMA.pressed ? "#99f3f5" : "#5cf4f1"; font.pixelSize: 14; font.bold: true }
                                                MotionMouseArea { id: mjMA; anchors.fill: parent; hoverScale: 1.05; shadowEnabled: false; shimmerEnabled: false; onClicked: {
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
                                                width: parent.width - 90 - 4; height: 42; radius: 6
                                                color: "#0d1117"; border.color: "#5cf4f1"; border.width: 1
                                                // Placeholder hint
                                                Text {
                                                    anchors { fill: parent; leftMargin: 8; verticalCenter: parent.verticalCenter }
                                                    text: poseNameInput.text.length === 0 ? "pose name / comment..." : ""
                                                    color: "#555"; font.pixelSize: 14; font.family: "monospace"
                                                    verticalAlignment: Text.AlignVCenter
                                                }
                                                TextInput {
                                                    id: poseNameInput
                                                    anchors { fill: parent; leftMargin: 8; rightMargin: 4; topMargin: 4; bottomMargin: 4 }
                                                    color: root.cText; font.pixelSize: 14; font.family: "monospace"
                                                    clip: true; selectByMouse: true; verticalAlignment: Text.AlignVCenter
                                                }
                                            }
                                            // SAVE button
                                            Rectangle {
                                                id: savePoseBtn
                                                width: 90; height: 42; radius: 6
                                                property bool saving: false
                                                color: saving ? "#0d2a3a" : (saveMA.pressed ? "#0a405c" : root.cCard)
                                                border.color: savePoseBtn.saving ? "#1ecb70" : (saveMA.pressed ? "#1ecb70" : root.cBorder)
                                                border.width: saveMA.pressed ? 2 : 1
                                                Behavior on color { ColorAnimation { duration: 100 } }
                                                Text {
                                                    anchors.centerIn: parent
                                                    text: savePoseBtn.saving ? "✓ SAVED" : "💾 SAVE"
                                                    color: savePoseBtn.saving ? root.cGreen : root.cCyan
                                                    font.pixelSize: 13; font.bold: true
                                                }
                                                    MotionMouseArea {
                                                        id: saveMA; anchors.fill: parent
                                                        hoverScale: 1.05
                                                        pressScale: 0.976
                                                        shadowEnabled: false
                                                        shimmerEnabled: false
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
                                            text: ""; font.pixelSize: 12; font.family: "monospace"
                                            color: root.cGreen
                                        }

                                        Rectangle { width: parent.width; height: 1; color: root.cBorder; opacity: 0.5 }
                                        Text { text: "LOAD SAVED POSE"; color: root.cDim; font.pixelSize: 11; font.bold: true; font.letterSpacing: 0.8 }

                                        Rectangle {
                                            id: savedPosesLoaderRect
                                            width: parent.width; height: 42; radius: 6
                                            color: loadMA.pressed ? "#0f2c3d" : "#0d1117"
                                            border.color: "#5cf4f1"; border.width: 1

                                            property var savedPoses: []
                                            function refreshPoses() {
                                                savedPoses = robotController.getSavedPoses()
                                            }
                                            Component.onCompleted: refreshPoses()

                                            Text {
                                                anchors { left: parent.left; leftMargin: 8; verticalCenter: parent.verticalCenter }
                                                text: "📋 TOẠ ĐỘ ĐÃ LƯU (" + parent.savedPoses.length + ")"
                                                color: "#5cf4f1"; font.pixelSize: 14; font.bold: true
                                            }

                                            Text {
                                                anchors { right: parent.right; rightMargin: 8; verticalCenter: parent.verticalCenter }
                                                text: "▼"
                                                color: "#5cf4f1"; font.pixelSize: 12
                                            }

                                            MotionMouseArea {
                                                id: loadMA
                                                anchors.fill: parent
                                                onClicked: {
                                                    savedPosesLoaderRect.refreshPoses()
                                                    poseSelectorPopup.open()
                                                }
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
                                        anchors { fill: parent; margins: 8 }
                                        spacing: 6
                                        Row { spacing: 6
                                            Rectangle { width: 4; height: 16; radius: 1; color: root.cCyan; anchors.verticalCenter: parent.verticalCenter }
                                            Text { text: "I/O CONTROL"; color: root.cCyan; font.pixelSize: 15; font.bold: true; font.letterSpacing: 1.2 }
                                        }

                                        // Step Value
                                        Text { text: "STEP VALUE"; color: root.cDim; font.pixelSize: 12; font.bold: true }
                                        Row { spacing: 4; width: parent.width
                                            Repeater {
                                                model: [0.1, 1, 5, 10]
                                                delegate: Rectangle {
                                                    required property var modelData
                                                    width: (ioCol.width - 12) / 4; height: 34; radius: 5
                                                    color: page3Root.stepValue === modelData ? root.cAccent : root.cCard
                                                    border.color: page3Root.stepValue === modelData ? root.cAccent : root.cBorder; border.width: 2
                                                    Text { anchors.centerIn: parent; text: modelData; color: page3Root.stepValue === modelData ? "#000" : root.cText; font.pixelSize: 14; font.bold: true }
                                                    MotionMouseArea { anchors.fill: parent; onClicked: page3Root.stepValue = modelData }
                                                }
                                            }
                                        }

                                        // Hardware Speed (read-only from Dobot)
                                        Text { text: "HW SPEED"; color: root.cDim; font.pixelSize: 12; font.bold: true }
                                        Rectangle {
                                            width: parent.width; height: 36; radius: 5
                                            color: "#0d1117"; border.color: "#2a3a4a"; border.width: 1
                                            Row {
                                                anchors { fill: parent; leftMargin: 10; rightMargin: 10 }
                                                spacing: 6
                                                Text {
                                                    text: "⚙ Dobot:"
                                                    color: root.cDim; font.pixelSize: 14
                                                    anchors.verticalCenter: parent.verticalCenter
                                                }
                                                Text {
                                                    text: robotController.hwSpeedRatio + "%"
                                                    color: "#f59e0b"; font.pixelSize: 18; font.bold: true; font.family: "monospace"
                                                    anchors.verticalCenter: parent.verticalCenter
                                                }
                                            }
                                        }

                                        // Set Speed (interactive slider)
                                        Text { text: "SET SPEED %"; color: root.cDim; font.pixelSize: 12; font.bold: true }
                                        Row { spacing: 4; width: parent.width; height: 36
                                            Slider {
                                                id: speedSlider
                                                width: parent.width - 56; height: 34
                                                from: 1; to: 100; stepSize: 1; value: page3Root.speedVal
                                                onMoved: { page3Root.speedVal = Math.round(value) }
                                                onPressedChanged: { if (!pressed) robotController.setSpeedRatio(Math.round(value)) }
                                                background: Rectangle { x: speedSlider.leftPadding; y: speedSlider.topPadding + speedSlider.availableHeight / 2 - 4; width: speedSlider.availableWidth; height: 8; radius: 4; color: root.cCard; border.color: root.cBorder
                                                    Rectangle { width: speedSlider.visualPosition * parent.width; height: parent.height; radius: 4; color: "#5cf4f1" }
                                                }
                                                handle: Rectangle { x: speedSlider.leftPadding + speedSlider.visualPosition * (speedSlider.availableWidth - width); y: speedSlider.topPadding + speedSlider.availableHeight / 2 - 8; width: 18; height: 18; radius: 9; color: "#5cf4f1"; border.color: "#fff" }
                                            }
                                            Rectangle {
                                                width: 50; height: 34; radius: 5; color: "#0d1117"; border.color: "#5cf4f1"; border.width: 2
                                                TextInput { anchors.centerIn: parent; width: 44; color: root.cText; font.pixelSize: 14; font.family: "monospace"; font.bold: true; horizontalAlignment: Text.AlignHCenter
                                                    text: page3Root.speedVal
                                                    validator: IntValidator { bottom: 1; top: 100 }
                                                    selectByMouse: true
                                                    onEditingFinished: { var v = Math.max(1, Math.min(100, parseInt(text) || 100)); page3Root.speedVal = v; speedSlider.value = v; robotController.setSpeedRatio(v) }
                                                }
                                            }
                                        }

                                        Rectangle { width: parent.width; height: 1; color: root.cBorder }

                                        // Gripper DO1 — valve 5/3: GẮP (ch0=T,ch1=F) / NHẢ (ch0=F,ch1=T)
                                        Text { text: "GRIPPER (DO1)"; color: root.cDim; font.pixelSize: 12; font.bold: true }
                                        Row {
                                            id: rowGripper
                                            property bool isOn: false  // false = NHẢ (startup safe state)
                                            spacing: 6; width: parent.width
                                            Rectangle {
                                                width: (parent.width - 6) / 2; height: 42; radius: 6
                                                color: rowGripper.isOn ? "#0d2a3a" : root.cCard
                                                border.color: rowGripper.isOn ? root.cGreen : root.cBorder
                                                Text { anchors.centerIn: parent; text: "GẮP"; color: rowGripper.isOn ? root.cGreen : root.cDim; font.pixelSize: 15; font.bold: true }
                                                MotionMouseArea { anchors.fill: parent; onClicked: { robotController.setDigitalOutput(1, true); rowGripper.isOn = true } }
                                            }
                                            Rectangle {
                                                width: (parent.width - 6) / 2; height: 42; radius: 6
                                                color: !rowGripper.isOn ? "#1a3a5a" : root.cCard
                                                border.color: !rowGripper.isOn ? "#6aaeff" : root.cBorder
                                                Text { anchors.centerIn: parent; text: "NHẢ"; color: !rowGripper.isOn ? "#6aaeff" : root.cDim; font.pixelSize: 15; font.bold: true }
                                                MotionMouseArea { anchors.fill: parent; onClicked: { robotController.setDigitalOutput(1, false); rowGripper.isOn = false } }
                                            }
                                        }

                                        // Picker DO2 — valve 5/3: GẮP (ch2=T,ch3=F) / NHẢ (ch2=F,ch3=T)
                                        Text { text: "PICKER (DO2)"; color: root.cDim; font.pixelSize: 12; font.bold: true }
                                        Row {
                                            id: rowPicker
                                            property bool isOn: false  // false = NHẢ (startup safe state)
                                            spacing: 6; width: parent.width
                                            Rectangle {
                                                width: (parent.width - 6) / 2; height: 42; radius: 6
                                                color: rowPicker.isOn ? "#0d2a3a" : root.cCard
                                                border.color: rowPicker.isOn ? root.cGreen : root.cBorder
                                                Text { anchors.centerIn: parent; text: "GẮP"; color: rowPicker.isOn ? root.cGreen : root.cDim; font.pixelSize: 15; font.bold: true }
                                                MotionMouseArea { anchors.fill: parent; onClicked: { robotController.setDigitalOutput(2, true); rowPicker.isOn = true } }
                                            }
                                            Rectangle {
                                                width: (parent.width - 6) / 2; height: 42; radius: 6
                                                color: !rowPicker.isOn ? "#1a3a5a" : root.cCard
                                                border.color: !rowPicker.isOn ? "#6aaeff" : root.cBorder
                                                Text { anchors.centerIn: parent; text: "NHẢ"; color: !rowPicker.isOn ? "#6aaeff" : root.cDim; font.pixelSize: 15; font.bold: true }
                                                MotionMouseArea { anchors.fill: parent; onClicked: { robotController.setDigitalOutput(2, false); rowPicker.isOn = false } }
                                            }
                                        }

                                        Rectangle { width: parent.width; height: 1; color: root.cBorder }

                                        // Stop & Reset → IDLE
                                        Rectangle {
                                            width: parent.width; height: 46; radius: 5
                                            color: "transparent"; border.width: 0
                                            gradient: Gradient {
                                                orientation: Gradient.Horizontal
                                                GradientStop { position: 0.0; color: scStopMA.pressed ? Qt.darker("#771a1a", 1.15) : "#771a1a" }
                                                GradientStop { position: 1.0; color: scStopMA.pressed ? Qt.darker("#4e0c0c", 1.15) : "#4e0c0c" }
                                            }
                                            Text { anchors.centerIn: parent; text: "⏹ STOP"; color: "#d4faff"; font.pixelSize: 15; font.bold: true }
                                            MotionMouseArea { id: scStopMA; anchors.fill: parent; onClicked: { robotController.stopAndResetRobot(); cartridgeController.stopSystem() } }
                                        }

                                        // Enable
                                        Rectangle {
                                            width: parent.width; height: 42; radius: 5
                                            color: "transparent"; border.width: 0
                                            gradient: Gradient {
                                                orientation: Gradient.Horizontal
                                                GradientStop { position: 0.0; color: scEnMA.pressed ? Qt.darker("#0a405c", 1.15) : "#0a405c" }
                                                GradientStop { position: 1.0; color: scEnMA.pressed ? Qt.darker("#052b3d", 1.15) : "#052b3d" }
                                            }
                                            Text { anchors.centerIn: parent; text: "ENABLE"; color: "#d4faff"; font.pixelSize: 15; font.bold: true }
                                            MotionMouseArea { id: scEnMA; anchors.fill: parent; onClicked: robotController.enableSystem(true) }
                                        }

                                        // Pause / Resume
                                        Row { spacing: 6; width: parent.width
                                            Rectangle {
                                                width: (parent.width - 6) / 2; height: 42; radius: 5
                                                color: "transparent"; border.width: 0
                                                gradient: Gradient {
                                                    orientation: Gradient.Horizontal
                                                    GradientStop { position: 0.0; color: scPauseMA.pressed ? Qt.darker("#0a405c", 1.15) : "#0a405c" }
                                                    GradientStop { position: 1.0; color: scPauseMA.pressed ? Qt.darker("#052b3d", 1.15) : "#052b3d" }
                                                }
                                                Text { anchors.centerIn: parent; text: "PAUSE"; color: "#d4faff"; font.pixelSize: 14; font.bold: true }
                                                MotionMouseArea { id: scPauseMA; anchors.fill: parent; onClicked: robotController.pauseRobot() }
                                            }
                                            Rectangle {
                                                width: (parent.width - 6) / 2; height: 42; radius: 5
                                                color: "transparent"; border.width: 0
                                                gradient: Gradient {
                                                    orientation: Gradient.Horizontal
                                                    GradientStop { position: 0.0; color: scResMA.pressed ? Qt.darker("#0b7876", 1.15) : "#0b7876" }
                                                    GradientStop { position: 1.0; color: scResMA.pressed ? Qt.darker("#095f5d", 1.15) : "#095f5d" }
                                                }
                                                Text { anchors.centerIn: parent; text: "RESUME"; color: "#d4faff"; font.pixelSize: 14; font.bold: true }
                                                MotionMouseArea { id: scResMA; anchors.fill: parent; onClicked: robotController.resumeRobot() }
                                            }
                                        }

                                        // Clear Error
                                        Rectangle {
                                            width: parent.width; height: 34; radius: 4
                                            color: "transparent"; border.width: 0
                                            gradient: Gradient {
                                                orientation: Gradient.Horizontal
                                                GradientStop { position: 0.0; color: scCeMA.pressed ? Qt.darker("#0a405c", 1.15) : "#0a405c" }
                                                GradientStop { position: 1.0; color: scCeMA.pressed ? Qt.darker("#052b3d", 1.15) : "#052b3d" }
                                            }
                                            Text { anchors.centerIn: parent; text: "CLEAR ERROR"; color: "#d4faff"; font.pixelSize: 12; font.bold: true }
                                            MotionMouseArea { id: scCeMA; anchors.fill: parent; hoverScale: 1.05; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false; onClicked: robotController.clearError() }
                                        }

                                        // E-STOP (biggest button)
                                        Rectangle {
                                            width: parent.width; height: 50; radius: 5
                                            color: "transparent"
                                            border.color: "#ef4444"; border.width: 1
                                            gradient: Gradient {
                                                orientation: Gradient.Horizontal
                                                GradientStop { position: 0.0; color: scEsMA.pressed ? Qt.darker("#da2525", 1.15) : "#da2525" }
                                                GradientStop { position: 1.0; color: scEsMA.pressed ? Qt.darker("#ba1b1b", 1.15) : "#ba1b1b" }
                                            }
                                            Text { anchors.centerIn: parent; text: "EMERGENCY\nSTOP"; color: "#ffffff"; font.pixelSize: 14; font.bold: true; horizontalAlignment: Text.AlignHCenter }
                                            MotionMouseArea { id: scEsMA; anchors.fill: parent; onClicked: { robotController.emergencyStop(true); cartridgeController.stopSystem() } }
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
                                MotionMouseArea {
                                    anchors.fill: parent
                                    hoverEnabled: true
                                    motionEnabled: false
                                    shadowEnabled: false
                                    shimmerEnabled: false
                                }
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
                                            border.color: robotController.inReady ? "#22c55e" : (tiMA.pressed ? "#ffc16f" : "#ffaa4f")
                                            border.width: tiMA.pressed ? 3 : 2
                                            Behavior on color { ColorAnimation { duration: 100 } }
                                            
                                            Column {
                                                anchors.centerIn: parent; spacing: 2
                                                Text { text: "📥 TRAY INPUT READY"; color: "#fff"; font.pixelSize: 12; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                                Text { text: robotController.inReady ? "(ON)" : "(OFF)"; color: robotController.inReady ? "#6ee7a8" : "#d29252"; font.pixelSize: 10; anchors.horizontalCenter: parent.horizontalCenter }
                                            }
                                            MotionMouseArea {
                                                id: tiMA; anchors.fill: parent
                                                hoverScale: 1.05; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false
                                                onClicked: robotController.simulateInputTrayReady()
                                            }
                                        }

                                        // TRAY OUTPUT READY
                                        Rectangle {
                                            width: 160; height: 50; radius: 8
                                            color: robotController.outReady ? "#1a5a3a" : (toMA.pressed ? "#5a3a1a" : "#351a0a")
                                            border.color: robotController.outReady ? "#22c55e" : (toMA.pressed ? "#ffc16f" : "#ffaa4f")
                                            border.width: toMA.pressed ? 3 : 2
                                            Behavior on color { ColorAnimation { duration: 100 } }
                                            
                                            Column {
                                                anchors.centerIn: parent; spacing: 2
                                                Text { text: "📤 TRAY OUTPUT READY"; color: "#fff"; font.pixelSize: 12; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                                Text { text: robotController.outReady ? "(ON)" : "(OFF)"; color: robotController.outReady ? "#6ee7a8" : "#d29252"; font.pixelSize: 10; anchors.horizontalCenter: parent.horizontalCenter }
                                            }
                                            MotionMouseArea {
                                                id: toMA; anchors.fill: parent
                                                hoverScale: 1.05; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false
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
                                            border.color: piMA.pressed ? "#7085ff" : "#4f6cff"
                                            border.width: piMA.pressed ? 3 : 2
                                            Behavior on color { ColorAnimation { duration: 100 } }
                                            
                                            Column {
                                                anchors.centerIn: parent; spacing: 2
                                                Text { text: "↓ PICK INPUT"; color: "#fff"; font.pixelSize: 12; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                                Text { text: "(Load Khay -> Chamber)"; color: "#888"; font.pixelSize: 10; anchors.horizontalCenter: parent.horizontalCenter }
                                            }
                                            MotionMouseArea {
                                                id: piMA; anchors.fill: parent
                                                hoverScale: 1.05; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false
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
                                            border.color: pcMA.pressed ? "#7085ff" : "#4f6cff"
                                            border.width: pcMA.pressed ? 3 : 2
                                            Behavior on color { ColorAnimation { duration: 100 } }
                                            
                                            Column {
                                                anchors.centerIn: parent; spacing: 2
                                                Text { text: "⟳ PICK CHAMBER"; color: "#fff"; font.pixelSize: 12; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                                Text { text: "(Chamber -> Output)"; color: "#888"; font.pixelSize: 10; anchors.horizontalCenter: parent.horizontalCenter }
                                            }
                                            MotionMouseArea {
                                                id: pcMA; anchors.fill: parent
                                                hoverScale: 1.05; pressScale: 0.976; shadowEnabled: false; shimmerEnabled: false
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
                                CBtn { lbl:"Clear"; padV:3; padH:8; fontSize: 14; bg:root.cCard; bc:root.cBorder; tc:root.cDim; onClicked: robotController.clearLog() }
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

                // ── POPUP: SELECT SAVED POSE ──
                Rectangle {
                    id: poseSelectorPopup
                    anchors.fill: parent
                    color: "#cc000000" // dim background
                    visible: false
                    z: 9999 // ensure it is on top of everything

                    // Close on clicking background
                    MotionMouseArea {
                        anchors.fill: parent
                        motionEnabled: false
                        shadowEnabled: false
                        shimmerEnabled: false
                        onClicked: poseSelectorPopup.visible = false
                    }

                    Rectangle {
                        id: popupBg
                        width: parent.width * 0.9; height: parent.height * 0.8
                        anchors.centerIn: parent
                        color: "#0a0d14"
                        border.color: "#5cf4f1"; border.width: 2
                        radius: 8

                        // Prevent clicking inside from closing
                        MotionMouseArea {
                            anchors.fill: parent
                            preventStealing: true
                            motionEnabled: false
                            shadowEnabled: false
                            shimmerEnabled: false
                        }

                        Column {
                            anchors { fill: parent; margins: 14 }
                            spacing: 10

                            Item {
                                width: parent.width; height: 34
                                Text {
                                    text: "📋 CHỌN TOẠ ĐỘ ROBOT ĐÃ LƯU"
                                    color: "#5cf4f1"; font.pixelSize: 18; font.bold: true; font.letterSpacing: 1.2
                                    anchors.verticalCenter: parent.verticalCenter
                                    anchors.left: parent.left
                                }
                                Rectangle {
                                    width: 34; height: 34; radius: 17
                                    color: closeMA.pressed ? "#552222" : "#221111"
                                    border.color: "#ff4444"
                                    anchors.right: parent.right
                                    anchors.verticalCenter: parent.verticalCenter
                                    Text { anchors.centerIn: parent; text: "✕"; color: "#ff4444"; font.pixelSize: 18; font.bold: true }
                                    MotionMouseArea {
                                        id: closeMA; anchors.fill: parent; onClicked: poseSelectorPopup.visible = false
                                    }
                                }
                            }

                            Rectangle { width: parent.width; height: 1; color: "#223344" }

                            Item {
                                width: parent.width; height: popupBg.height - 83
                                
                                ListView {
                                    id: poseListView
                                    anchors.fill: parent
                                    clip: true
                                    spacing: 8
                                    model: []

                                    delegate: Rectangle {
                                        width: poseListView.width; height: 72
                                        color: itemMA.pressed ? "#0f2c3d" : (itemMA.containsMouse ? "#0a1a26" : "#0d1117")
                                        border.color: itemMA.containsMouse ? "#5cf4f1" : "#1a2a3a"; border.width: 1
                                        radius: 6

                                        Row {
                                            anchors { fill: parent; leftMargin: 12; rightMargin: 12 }
                                            spacing: 16

                                            // Left side: Comment & Grid coordinates
                                            Column {
                                                anchors.verticalCenter: parent.verticalCenter
                                                width: parent.width - 104
                                                spacing: 4
                                                
                                                Text {
                                                    text: modelData.name ? modelData.name : "Không có tên/ghi chú"
                                                    color: "#f59e0b"; font.pixelSize: 22; font.bold: true
                                                    elide: Text.ElideRight
                                                }
                                                
                                                // Grid-like layout for coordinates with vertical alignment and clear margins
                                                Row {
                                                    spacing: 8
                                                    width: parent.width
                                                    
                                                    Repeater {
                                                        model: [
                                                            { label: "J1", val: modelData.j1 },
                                                            { label: "J2", val: modelData.j2 },
                                                            { label: "J3", val: modelData.j3 },
                                                            { label: "J4", val: modelData.j4 },
                                                            { label: "J5", val: modelData.j5 },
                                                            { label: "J6", val: modelData.j6 }
                                                        ]
                                                        
                                                        delegate: Row {
                                                            spacing: 4
                                                            Text {
                                                                text: model.modelData.label + ":"
                                                                color: "#a1a1aa"
                                                                font { pixelSize: 18; bold: true }
                                                                anchors.verticalCenter: parent.verticalCenter
                                                            }
                                                            Text {
                                                                text: model.modelData.val.toFixed(2)
                                                                color: "#e4e4e7"
                                                                font { pixelSize: 19; family: "monospace"; bold: true }
                                                                anchors.verticalCenter: parent.verticalCenter
                                                            }
                                                        }
                                                    }
                                                }
                                            }

                                            // Apply button
                                            Rectangle {
                                                width: 88; height: 42; radius: 6
                                                anchors.verticalCenter: parent.verticalCenter
                                                color: applyMA.pressed ? "#1a5a3a" : "#0a2a1a"
                                                border.color: applyMA.pressed ? "#1ecb70" : "#168f52"
                                                
                                                Text {
                                                    anchors.centerIn: parent
                                                    text: "APPLY"
                                                    color: "#6ee7a8"
                                                    font.pixelSize: 16; font.bold: true; font.letterSpacing: 1
                                                }
                                                
                                                MotionMouseArea {
                                                    id: applyMA; anchors.fill: parent
                                                    onClicked: {
                                                        jointInputs.itemAt(0).children[1].children[0].text = modelData.j1.toFixed(4)
                                                        jointInputs.itemAt(1).children[1].children[0].text = modelData.j2.toFixed(4)
                                                        jointInputs.itemAt(2).children[1].children[0].text = modelData.j3.toFixed(4)
                                                        jointInputs.itemAt(3).children[1].children[0].text = modelData.j4.toFixed(4)
                                                        jointInputs.itemAt(4).children[1].children[0].text = modelData.j5.toFixed(4)
                                                        jointInputs.itemAt(5).children[1].children[0].text = modelData.j6.toFixed(4)
                                                        poseSelectorPopup.visible = false
                                                    }
                                                }
                                            }
                                        }

                                        MotionMouseArea {
                                            id: itemMA
                                            anchors.fill: parent
                                            hoverEnabled: true
                                            onClicked: {
                                                jointInputs.itemAt(0).children[1].children[0].text = modelData.j1.toFixed(4)
                                                jointInputs.itemAt(1).children[1].children[0].text = modelData.j2.toFixed(4)
                                                jointInputs.itemAt(2).children[1].children[0].text = modelData.j3.toFixed(4)
                                                jointInputs.itemAt(3).children[1].children[0].text = modelData.j4.toFixed(4)
                                                jointInputs.itemAt(4).children[1].children[0].text = modelData.j5.toFixed(4)
                                                jointInputs.itemAt(5).children[1].children[0].text = modelData.j6.toFixed(4)
                                                poseSelectorPopup.visible = false
                                            }
                                        }
                                    }
                                }
                                
                                // Custom Scrollbar
                                Rectangle {
                                    id: scrollbar
                                    anchors { right: parent.right; top: parent.top; bottom: parent.bottom; rightMargin: 2 }
                                    width: 4; radius: 2; color: "#1f2937"
                                    visible: poseListView.height < poseListView.contentHeight
                                    
                                    Rectangle {
                                        width: parent.width; radius: 2; color: "#5cf4f1"
                                        height: Math.max(20, poseListView.height * (poseListView.height / poseListView.contentHeight))
                                        y: poseListView.visibleArea.yPosition * poseListView.height
                                    }
                                }
                            }
                        }
                    }

                    function open() {
                        poseListView.model = robotController.getSavedPoses()
                        visible = true
                    }
                }
            } // Page 3

            // ── PAGE 4: FILL HP CONTROL (redesigned — see FillHpTab.qml) ──
            FillHpTab { }

            // ── PAGE 5: INK SYSTEM ──────────────────────────────────────
            InkTab { }

            // ── PAGE 6: PRODUCTION OUTPUT ───────────────────────────────
            ProductionTab { }

        } // StackLayout

        // ════════════════════════════════════════════════════════════
        // REUSABLE: CBtn — matches HTML .btn
        // ════════════════════════════════════════════════════════════
        component CBtn: Rectangle {
            id: cbr
            property string lbl: ""
            property string iconSource: ""
            property color bg:    root.cCard
            property color bgEnd: bg       // when bgEnd != bg, use horizontal gradient
            property color bc:    root.cBorder
            property color tc:    root.cText
            property bool  active: true
            property bool  clickEnabled: active
            property int   padV: 6
            property int   padH: 12
            property int   fontSize: 16
            property int   w: 0
            property int   h: 0
            property bool  _pressed: false
            property bool  _hovered: false
            property bool  isSelected: false
            // Khi blinking=true: viền + nền nhấp nháy thu hút sự chú ý (vd hint từ
            // node Python qua uiHint). Auto-stop khi blinking=false. Animation 600ms/chu kỳ.
            property bool  blinking: false

            signal clicked(); signal pressed(); signal released()

            implicitWidth:  w > 0 ? w : cbrT.implicitWidth + padH * 2
            implicitHeight: h > 0 ? h : cbrT.implicitHeight + padV * 2
            radius: 4
            color: bgEnd !== bg ? "transparent" : (
                !active ? bg :
                _pressed ? Qt.darker(bg, 2.15) :
                isSelected ? Qt.darker(bg, 2.05) :
                _hovered ? Qt.lighter(bg, 1.06) :
                bg
            )
            border.color: {
                if (_pressed) return Qt.darker(bc, 1.32)
                if (isSelected) return Qt.darker(bc, 1.25)
                if (_hovered) return Qt.lighter(bc, 1.08)
                return bc
            }
            border.width: _pressed ? 2 : 1
            opacity: active ? 1.0 : 0.4

            Behavior on color        { ColorAnimation { duration: 100 } }
            Behavior on border.color { ColorAnimation { duration: 100 } }
            Behavior on opacity      { NumberAnimation { duration: 150 } }

            // Gradient background (only active when bgEnd != bg)
            Rectangle {
                anchors.fill: parent
                radius: parent.radius
                visible: cbr.bgEnd !== cbr.bg
                z: -1
                gradient: Gradient {
                    orientation: Gradient.Horizontal
                    GradientStop { position: 0.0; color: cbr._pressed ? Qt.darker(cbr.bg, 1.28) : cbr.bg }
                    GradientStop { position: 1.0; color: cbr._pressed ? Qt.darker(cbr.bgEnd, 1.28) : cbr.bgEnd }
                }
            }

            // Glow effect when pressed
            Rectangle {
                anchors.fill: parent; anchors.margins: -2
                radius: parent.radius + 2; color: "transparent"
                border.color: cbr._pressed ? Qt.rgba(cbr.bc.r, cbr.bc.g, cbr.bc.b, 0.18) : "transparent"
                border.width: 3; z: -1
                Behavior on border.color { ColorAnimation { duration: 100 } }
            }

            // Blink overlay khi blinking=true (UI hint từ node Python)
            Rectangle {
                id: blinkOverlay
                anchors.fill: parent; anchors.margins: -3
                radius: parent.radius + 3
                color: "transparent"
                border.color: root.cAccent
                border.width: 4
                opacity: 0
                visible: cbr.blinking
                z: 2
                SequentialAnimation on opacity {
                    loops: Animation.Infinite
                    running: cbr.blinking
                    NumberAnimation { to: 1.0; duration: 350; easing.type: Easing.InOutQuad }
                    NumberAnimation { to: 0.2; duration: 350; easing.type: Easing.InOutQuad }
                }
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

            Text { id: cbrT; anchors.centerIn: parent; text: cbr.lbl; color: cbr.isSelected ? "#e8fff3" : (cbr.active ? cbr.tc : root.cDim)
                font.pixelSize: cbr.fontSize; font.weight: Font.DemiBold; font.capitalization: Font.MixedCase
                anchors.verticalCenterOffset: (cbr.isSelected || cbr._pressed) ? 2 : 0
                Behavior on color { ColorAnimation { duration: 80 } }
                visible: cbr.iconSource === ""
            }

            Image {
                id: cbrImg
                anchors.centerIn: parent
                source: cbr.iconSource
                width: cbr.fontSize * 1.5; height: cbr.fontSize * 1.5
                fillMode: Image.PreserveAspectFit
                smooth: true
                visible: cbr.iconSource !== ""
            }

            MotionMouseArea { anchors.fill: parent; hoverEnabled: true
                enabled: cbr.clickEnabled
                hoverScale: 1.05
                pressScale: 0.976
                shadowEnabled: false
                shimmerEnabled: cbr.active
                shimmerWhilePressed: true
                shimmerColor: "#55d4faff"
                onClicked:       { if(cbr.clickEnabled) cbr.clicked() }
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
            HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cBorder }

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
                    CBtn { lbl:"Save"; padV:8; padH:18; fontSize: 17; bg:"#0d2a3a"; bc:root.cGreen; tc:root.cGreen
                        onClicked: {
                            var positions = {}
                            for (var i = 0; i < cfgRepeater.count; i++) {
                                var item = cfgRepeater.itemAt(i)
                                if (item) positions[String(item.rowNum)] = parseFloat(item.inputText) || 0.0
                            }
                            cartridgeController.saveConfig(cfgCard.configKey, JSON.stringify(positions))
                        }
                    }
                    CBtn { lbl:"↺ Reset"; padV:8; padH:14; fontSize: 17; bg:root.cCard; bc:root.cBorder; tc:root.cText
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
            HoverHandler { onHoveredChanged: parent.border.color = hovered ? root.cHover : root.cBorder }

            Flickable {
                id: cfgZoneFlick
                anchors { fill: parent; margins: 8 }
                contentWidth: width; contentHeight: cfgZoneCol.height + 20
                clip: true

                Column {
                    id: cfgZoneCol
                    width: parent.width
                    spacing: 6

                    Text { text: cfgZoneCard.title; color: root.cAccent; font.pixelSize: 18; font.bold: true; font.letterSpacing: 1.5 }

                    Row { width: parent.width; spacing: parent.width * 0.02
                        Text { text: "Row"; color: root.cDim; font.pixelSize: 14; font.bold: true; width: parent.width * 0.12; font.capitalization: Font.AllUppercase }
                        Text { text: "Max"; color: root.cDim; font.pixelSize: 14; font.bold: true; width: parent.width * 0.23; font.capitalization: Font.AllUppercase }
                        Text { text: "Min"; color: root.cDim; font.pixelSize: 14; font.bold: true; width: parent.width * 0.23; font.capitalization: Font.AllUppercase }
                        Text { text: "Target"; color: root.cDim; font.pixelSize: 14; font.bold: true; width: parent.width * 0.23; font.capitalization: Font.AllUppercase }
                        Text { text: "Loc"; color: root.cDim; font.pixelSize: 14; font.bold: true; width: parent.width * 0.10; font.capitalization: Font.AllUppercase }
                    }
                    Rectangle { width: parent.width; height: 1; color: root.cBorder }

                    Repeater {
                        id: cfgZoneRepeater
                        model: [10,9,8,7,6,5,4,3,2,1]
                        delegate: Rectangle {
                            required property int modelData
                            required property int index
                            width: cfgZoneCol.width; height: 46
                            color: index % 2 === 0 ? "transparent" : "#0d0d22"
                            property alias minText: minInp.text
                            property alias maxText: maxInp.text
                            property alias tgtText: tgtInp.text
                            property int rowNum: modelData

                            Row {
                                anchors.verticalCenter: parent.verticalCenter
                                width: parent.width
                                spacing: parent.width * 0.02
                                Text { text: "R"+modelData; color: root.cCyan; font.pixelSize: 18; font.bold: true; width: parent.width * 0.12; anchors.verticalCenter: parent.verticalCenter }

                                Rectangle { width: parent.width * 0.23; height: 36; radius: 5; color: root.cBg; border.color: root.cBorder
                                    TextInput { id: minInp; anchors { fill: parent; margins: 3 } text: "0.0"; font.pixelSize: 18; font.family: "monospace"; font.bold: true; color: root.cYellow; horizontalAlignment: TextInput.AlignHCenter; validator: DoubleValidator { bottom: -9999; top: 9999; decimals: 1 }
                                        Connections { target: page2Root; function onConfigRevisionChanged() { var tbl = page2Root.parsedConfig[cfgZoneCard.configKey]; if (tbl && tbl[String(modelData)]) minInp.text = String(tbl[String(modelData)][0]) } } } }
                                Rectangle { width: parent.width * 0.23; height: 36; radius: 5; color: root.cBg; border.color: root.cBorder
                                    TextInput { id: maxInp; anchors { fill: parent; margins: 3 } text: "0.0"; font.pixelSize: 18; font.family: "monospace"; font.bold: true; color: root.cYellow; horizontalAlignment: TextInput.AlignHCenter; validator: DoubleValidator { bottom: -9999; top: 9999; decimals: 1 }
                                        Connections { target: page2Root; function onConfigRevisionChanged() { var tbl = page2Root.parsedConfig[cfgZoneCard.configKey]; if (tbl && tbl[String(modelData)]) maxInp.text = String(tbl[String(modelData)][1]) } } } }
                                Rectangle { width: parent.width * 0.23; height: 36; radius: 5; color: root.cBg; border.color: root.cBorder
                                    TextInput { id: tgtInp; anchors { fill: parent; margins: 3 } text: "0.0"; font.pixelSize: 18; font.family: "monospace"; font.bold: true; color: root.cYellow; horizontalAlignment: TextInput.AlignHCenter; validator: DoubleValidator { bottom: -9999; top: 9999; decimals: 1 }
                                        Connections { target: page2Root; function onConfigRevisionChanged() { var tbl = page2Root.parsedConfig[cfgZoneCard.configKey]; if (tbl && tbl[String(modelData)]) tgtInp.text = String(tbl[String(modelData)][2]) } } } }

                                Text { text: modelData===10?"Top":modelData===1?"Bot":""; color: root.cDim; font.pixelSize: 14; font.bold: true; width: parent.width * 0.10; anchors.verticalCenter: parent.verticalCenter }
                            }
                            Rectangle { width: parent.width; height: 1; color: "#1e1e3a"; anchors.bottom: parent.bottom }
                        }
                    }

                    Row { spacing: 8; topPadding: 8
                        CBtn { lbl:"Save"; padV:10; padH:22; fontSize: 18; bg:"#0d2a3a"; bc:root.cGreen; tc:root.cGreen
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
                        CBtn { lbl:"↺ Reset"; padV:10; padH:18; fontSize: 18; bg:root.cCard; bc:root.cBorder; tc:root.cText
                            onClicked: page2Root.reloadConfig()
                        }
                    }
                }
            }

            Rectangle {
                id: cfgZoneScroll
                anchors { right: parent.right; top: parent.top; bottom: parent.bottom; rightMargin: 2; topMargin: 8; bottomMargin: 8 }
                width: 4; radius: 2; color: "#1f2937"
                visible: cfgZoneFlick.height < cfgZoneFlick.contentHeight

                Rectangle {
                    width: parent.width; radius: 2; color: root.cAccent
                    height: Math.max(20, cfgZoneFlick.height * (cfgZoneFlick.height / cfgZoneFlick.contentHeight))
                    y: cfgZoneFlick.visibleArea.yPosition * cfgZoneFlick.height
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
        id: velPopup
        width: 280; height: 370
        anchors.centerIn: parent
        modal: true; focus: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        background: Rectangle {
            color: "#0e0e22"; border.color: root.cAccent; border.width: 2; radius: 10
        }

        property string inputStr: ""
        property var targetCard: null

        onOpened: { velPopup.inputStr = "" }

        function openForCard(card) {
            velPopup.targetCard = card
            velPopup.inputStr = ""
            velPopup.open()
        }

        function velPopupApply() {
            var v = parseInt(velPopup.inputStr) || (velPopup.targetCard ? velPopup.targetCard.jogVelMms : velDisplay.jogVelMms)
            v = Math.max(1, Math.min(v, 80))
            if (velPopup.targetCard) velPopup.targetCard.jogVelMms = v
            else velDisplay.jogVelMms = v
            velPopup.close()
        }

        function numpadPress(ch) {
            if (ch === "←") {
                if (velPopup.inputStr.length > 0)
                    velPopup.inputStr = velPopup.inputStr.slice(0, -1)
            } else if (ch === "✓") {
                velPopupApply()
            } else {
                if (velPopup.inputStr.length < 3)
                    velPopup.inputStr += ch
            }
        }

        contentItem: Column {
            anchors.fill: parent; anchors.margins: 14; spacing: 7

            Text {
                text: "Đặt tốc độ JOG"
                color: root.cText; font.pixelSize: 14; font.bold: true
                width: parent.width; horizontalAlignment: Text.AlignHCenter
            }

            Rectangle {
                width: parent.width; height: 44; radius: 6
                color: "#1a1a3a"; border.color: root.cAccent; border.width: 1
                Row {
                    anchors.centerIn: parent; spacing: 6
                    Text {
                        text: velPopup.inputStr.length > 0 ? velPopup.inputStr : "–"
                        color: root.cText; font.pixelSize: 26; font.bold: true
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    Text {
                        text: "mm/s"; color: root.cDim; font.pixelSize: 13
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }
            }

            Text {
                text: "max 80 mm/s — chỉ áp dụng JOG"
                color: root.cOrange; font.pixelSize: 9
                width: parent.width; horizontalAlignment: Text.AlignHCenter
            }

            Repeater {
                model: [["7","8","9"],["4","5","6"],["1","2","3"],["←","0","✓"]]
                delegate: Row {
                    property var keys: modelData
                    spacing: 7
                    Repeater {
                        model: keys
                        delegate: Rectangle {
                            width: 79; height: 44; radius: 6
                            color: modelData === "✓" ? root.cGreen
                                 : modelData === "←" ? "#3a2a2a" : "#1a1a3a"
                            border.color: root.cBorder; border.width: 1
                            Text {
                                anchors.centerIn: parent
                                text: modelData; color: root.cText
                                font.pixelSize: 20; font.bold: true
                            }
                            MotionMouseArea {
                                anchors.fill: parent
                                onClicked: velPopup.numpadPress(modelData)
                            }
                        }
                    }
                }
            }

            Rectangle {
                width: parent.width; height: 38; radius: 6
                color: root.cBg; border.color: root.cBorder
                Text { anchors.centerIn: parent; text: "Hủy"; color: root.cDim; font.pixelSize: 13 }
                MotionMouseArea { anchors.fill: parent; onClicked: velPopup.close() }
            }
        }
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
                    MotionMouseArea { anchors.fill: parent; onClicked: { outTrayPopup.close(); outTrayTimer.restart(); } }
                }
                Rectangle {
                    width: 130; height: 46; radius: 6; color: "#00aa00"
                    Text { anchors.centerIn: parent; text: "YES"; color: "white"; font.bold: true; font.pixelSize: 16 }
                    MotionMouseArea { anchors.fill: parent; onClicked: { robotController.simulateOutputTrayReady(); outTrayPopup.close(); } }
                }
            }
        }
    }
    }
