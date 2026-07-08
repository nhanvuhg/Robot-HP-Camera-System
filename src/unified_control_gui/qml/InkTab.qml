import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls.Material 2.15

Item {
    id: inkTab

    property string currentTime: Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
    property bool calActive: scaleController.calStatus === "WAITING_WEIGHT" || scaleController.calStatus.startsWith("CONTINUE_CAL")


    Timer {
        interval: 1000
        running: true
        repeat: true
        onTriggered: currentTime = Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
    }

    Component.onCompleted: {
        loadProfiles()
    }

    function loadProfiles() {
        inkModel.clear()
        cartModel.clear()
        var inks = scaleController.getInkProfiles()
        var carts = scaleController.getCartProfiles()
        for (var i = 0; i < inks.length; i++) inkModel.append(inks[i])
        for (var j = 0; j < carts.length; j++) cartModel.append(carts[j])
    }

    ListModel { id: inkModel }
    ListModel { id: cartModel }
    
    Connections {
        target: scaleController
        function onProfilesChanged() { loadProfiles() }
    }

    property var numpadTarget: null

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

    Connections {
        target: hpController
        function onInkStatusChanged() {
            var map = parseKvPipe(hpController.inkStatus);
            if (map["CODE"] !== undefined && mapInkNameInput && !mapInkNameInput.activeFocus) {
                mapInkNameInput.text = map["CODE"];
            }
            if (map["LOT_PI"] !== undefined && mapLotPiInput && !mapLotPiInput.activeFocus) {
                mapLotPiInput.text = map["LOT_PI"];
            }
            if (map["LOT_CI"] !== undefined && mapLotCiInput && !mapLotCiInput.activeFocus) {
                mapLotCiInput.text = map["LOT_CI"];
            }
        }
    }

    // CartridgePage visual tokens — colors only; behavior and layout stay unchanged.
    readonly property color cBg:               "transparent"
    readonly property color cBorder:           "#1affffff"
    readonly property color cText:             "#ffffff"
    readonly property color cAccent:           "#67d0ff"
    readonly property color cPanel:            "#990d1e32"
    readonly property color cCard:             "#8806101d"
    readonly property color cField:            Qt.rgba(0.06, 0.19, 0.26, 0.82)
    readonly property color cFieldStrong:      "#081627"
    readonly property color cMuted:            "#74899f"
    readonly property color cSubText:          "#ffffff"
    readonly property color cDisabled:         "#74899f"
    readonly property color cSelectedText:     "#04080f"
    readonly property color cActionStart:      "#1a4a6e"
    readonly property color cActionEnd:        "#0c1726"
    readonly property color cActionHoverStart: "#1a4a6e"
    readonly property color cActionHoverEnd:   "#163a52"
    readonly property color cActionPressStart: "#163a52"
    readonly property color cActionPressEnd:   "#04080f"
    readonly property color cResumeStart:      "#1C4D8D"
    readonly property color cResumeEnd:        "#0c1726"
    readonly property color cResumeHoverStart: "#245fa8"
    readonly property color cResumeHoverEnd:   "#163a52"
    readonly property color cResumePressStart: "#173e72"
    readonly property color cResumePressEnd:   "#081627"
    readonly property color cPauseStart:       "#8a4210"
    readonly property color cPauseEnd:         "#E68457"
    readonly property color cPauseHoverStart:  "#a65315"
    readonly property color cPauseHoverEnd:    "#f09a6d"
    readonly property color cPausePressStart:  "#6f350d"
    readonly property color cPausePressEnd:    "#c76c3f"
    readonly property color cPauseBorder:      "#E68457"
    readonly property color cSuccess:          "#3ed0b4"
    readonly property color cDanger:           "#f0735c"
    readonly property color cWarning:          "#f5a623"

    Rectangle { anchors.fill: parent; color: cBg }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        // ── Main Content ───────────────────────────────────────
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.leftMargin: 15
            Layout.rightMargin: 15
            Layout.topMargin: 15
            Layout.bottomMargin: 24
            spacing: 15

            // ════════ LEFT PANEL: Monitor & Calibration ════════
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredWidth: 600
                Layout.fillHeight: true
                color: cPanel
                border.color: cBorder
                border.width: 1
                radius: 8

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 20
                    spacing: 12

                    Text { text: "LIVE WEIGHT DISPLAY"; color: cAccent; font.pixelSize: 22; font.bold: true; Layout.alignment: Qt.AlignHCenter }
                    
                    Rectangle { Layout.fillWidth: true; height: 1; color: cBorder }
                    
                    RowLayout {
                        Layout.alignment: Qt.AlignHCenter
                        spacing: 20
                        Text {
                            text: "● LOADCELL: " + scaleController.loadcellStatus
                            color: (scaleController.loadcellStatus == "OK" || scaleController.loadcellStatus == "SIM") ? cSuccess : cDanger
                            font.pixelSize: 16
                            font.bold: true
                        }
                        Text {
                            text: "Scale node: ● " + (scaleController.scaleNodeConnected ? "CONNECTED" : "DISCONNECTED")
                            color: scaleController.scaleNodeConnected ? cSuccess : cDanger
                            font.pixelSize: 16
                            font.bold: true
                        }
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 110
                        color: cFieldStrong
                        border.color: cAccent
                        border.width: 2
                        radius: 8
                        
                        Text {
                            anchors.centerIn: parent
                            text: scaleController.currentWeight.toFixed(1) + " g"
                            color: "#ffffff"
                            font.pixelSize: 64
                            font.bold: true
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8
                        
                        Text { text: "Status:"; color: cSubText; font.pixelSize: 16 }
                        Rectangle {
                            width: 140; height: 35; radius: 6
                            color: {
                                var s = scaleController.monitorStatus;
                                if(s === "NO_SIGNAL") return cDisabled;
                                if(s === "MEASURING") return cWarning;
                                if(s === "PASS") return cSuccess;
                                if(s === "FAIL") return cDanger;
                                return cDisabled;
                            }
                            Text { anchors.centerIn: parent; text: scaleController.monitorStatus; color: "#fff"; font.pixelSize: 16; font.bold: true }
                        }
                        
                        Item { Layout.fillWidth: true }
                        
                        Text { text: "Profile:"; color: cSubText; font.pixelSize: 16 }
                        Text { text: scaleController.activeProfile === "" ? "NOT SELECTED" : scaleController.activeProfile; color: cAccent; font.pixelSize: 18; font.bold: true }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 50
                        Layout.maximumHeight: 50
                        spacing: 15
                        
                        MotionButton {
                            id: tareBtn
                            opacity: down ? 0.8 : 1.0
                            text: "TARE"
                            Layout.fillWidth: true; Layout.preferredHeight: 50; Layout.maximumHeight: 50
                            font.pixelSize: 16; font.bold: true
                            onClicked: scaleController.tare()
                            background: Rectangle {
                                radius: 6
                                gradient: Gradient {
                                    orientation: Gradient.Horizontal
                                    GradientStop { position: 0.0; color: tareBtn.down ? cActionPressStart : (tareBtn.hovered ? cActionHoverStart : cActionStart) }
                                    GradientStop { position: 1.0; color: tareBtn.down ? cActionPressEnd : (tareBtn.hovered ? cActionHoverEnd : cActionEnd) }
                                }
                            }
                        }
                        MotionButton {
                            id: resetTareBtn
                            opacity: down ? 0.8 : 1.0
                            text: "RESET TARE"
                            Layout.fillWidth: true; Layout.preferredHeight: 50; Layout.maximumHeight: 50
                            font.pixelSize: 16; font.bold: true
                            onClicked: scaleController.resetTare()
                            background: Rectangle {
                                radius: 6
                                gradient: Gradient {
                                    orientation: Gradient.Horizontal
                                    GradientStop { position: 0.0; color: resetTareBtn.down ? cPausePressStart : (resetTareBtn.hovered ? cPauseHoverStart : cPauseStart) }
                                    GradientStop { position: 1.0; color: resetTareBtn.down ? cPausePressEnd : (resetTareBtn.hovered ? cPauseHoverEnd : cPauseEnd) }
                                }
                            }
                        }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: cBorder }
                    
                    // ── MOVED: CALIBRATE SCALE ──
                    RowLayout {
                        Layout.fillWidth: true
                        Text { text: "CALIBRATE SCALE"; color: cAccent; font.pixelSize: 18; font.bold: true }
                        Item { Layout.fillWidth: true }
                        Text { text: "Status: " + scaleController.calStatus; color: cWarning; font.pixelSize: 14; font.bold: true }
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 70
                        color: cCard
                        border.color: cBorder
                        radius: 6
                        RowLayout {
                            anchors.fill: parent; anchors.margins: 10
                            ColumnLayout {
                                Text { text: "STEP 1 — Empty Scale"; color: "#ecc45a"; font.pixelSize: 14; font.bold: true }
                                Text { text: "Ensure NOTHING is on the scale."; color: "#c7dcef"; font.pixelSize: 12 }
                            }
                            Item { Layout.fillWidth: true }
                            MotionButton {
                                id: setZeroBtn
                                opacity: down ? 0.8 : 1.0
                                text: "SET ZERO"
                                Layout.preferredWidth: 120; Layout.preferredHeight: 35
                                font.pixelSize: 14; font.bold: true
                                onClicked: scaleController.startCalibration()
                                background: Rectangle {
                                    radius: 4
                                    gradient: Gradient {
                                        orientation: Gradient.Horizontal
                                        GradientStop { position: 0.0; color: setZeroBtn.down ? cActionPressStart : (setZeroBtn.hovered ? cActionHoverStart : cActionStart) }
                                        GradientStop { position: 1.0; color: setZeroBtn.down ? cActionPressEnd : (setZeroBtn.hovered ? cActionHoverEnd : cActionEnd) }
                                    }
                                }
                                contentItem: Text {
                                    text: setZeroBtn.text; font: setZeroBtn.font
                                    color: "#ffffff"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                                }
                                enabled: scaleController.calStatus === "IDLE" || scaleController.calStatus === "ERROR"
                            }
                        }
                    }



                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 130
                        color: cCard
                        border.color: calActive ? cSuccess : cBorder
                        radius: 6
                        ColumnLayout {
                            anchors.fill: parent; anchors.margins: 10; spacing: 4
                            RowLayout {
                                Layout.fillWidth: true
                                ColumnLayout {
                                    spacing: 2
                                    Text {
                                        text: {
                                            if (scaleController.calStatus === "WAITING_WEIGHT") return "STEP 2 — Place Standard Weight (Point 1/4)";
                                            if (scaleController.calStatus === "CONTINUE_CAL_2/5") return "STEP 3 — Place Next Weight (Point 2/4)";
                                            if (scaleController.calStatus === "CONTINUE_CAL_3/5") return "STEP 4 — Place Next Weight (Point 3/4)";
                                            if (scaleController.calStatus === "CONTINUE_CAL_4/5") return "STEP 5 — Place Final Weight (Point 4/4)";
                                            return "STEP 2 — Place Standard Weight";
                                        }
                                        color: "#ecc45a"
                                        font.pixelSize: 13; font.bold: true
                                    }
                                    Text {
                                        text: {
                                            if (scaleController.calStatus === "WAITING_WEIGHT") return "Suggested: 100g";
                                            if (scaleController.calStatus === "CONTINUE_CAL_2/5") return "Suggested: 250g";
                                            if (scaleController.calStatus === "CONTINUE_CAL_3/5") return "Suggested: 500g";
                                            if (scaleController.calStatus === "CONTINUE_CAL_4/5") return "Suggested: 1000g";
                                            return "Enter known weight below";
                                        }
                                        color: cSubText; font.pixelSize: 11
                                    }
                                }
                                Item { Layout.fillWidth: true }
                                // Progress dots
                                Row {
                                    spacing: 5
                                    Repeater {
                                        model: 4
                                        Rectangle {
                                            width: 10; height: 10; radius: 5
                                            color: {
                                                var n = index + 1;
                                                if (scaleController.calStatus === "DONE") return cSuccess;
                                                if (scaleController.calStatus === "CONTINUE_CAL_2/5" && n <= 1) return cSuccess;
                                                if (scaleController.calStatus === "CONTINUE_CAL_3/5" && n <= 2) return cSuccess;
                                                if (scaleController.calStatus === "CONTINUE_CAL_4/5" && n <= 3) return cSuccess;
                                                var activeIndex = 0;
                                                var s = scaleController.calStatus;
                                                if (s === "WAITING_WEIGHT") activeIndex = 1;
                                                else if (s === "CONTINUE_CAL_2/5") activeIndex = 2;
                                                else if (s === "CONTINUE_CAL_3/5") activeIndex = 3;
                                                else if (s === "CONTINUE_CAL_4/5") activeIndex = 4;
                                                
                                                if ((s === "WAITING_WEIGHT" || s.startsWith("CONTINUE_CAL")) && n === activeIndex) return cWarning;
                                                return "#14263c";
                                            }
                                        }
                                    }
                                }
                            }
                            RowLayout {
                                Layout.fillWidth: true; spacing: 6
                                TextField {
                                    id: tfCalW
                                    placeholderText: "Enter value (g)"; placeholderTextColor: cSubText
                                    Layout.fillWidth: true; Layout.preferredHeight: 45
                                    font.pixelSize: 24; font.bold: true; color: cWarning
                                    horizontalAlignment: TextInput.AlignHCenter
                                    verticalAlignment: TextInput.AlignVCenter
                                    validator: DoubleValidator{}
                                    text: scaleController.lastKnownCalWeight.toString()
                                    enabled: calActive
                                    readOnly: true
                                    background: Rectangle { color: cField; radius: 6; border.color: cAccent; border.width: 1 }
                                    MotionMouseArea {
                                        anchors.fill: parent
                                        onClicked: {
                                            inkTab.numpadTarget = tfCalW;
                                            numpadPopup.currentValue = tfCalW.text;
                                            numpadPopup.open();
                                        }
                                    }
                                }
                                MotionButton {
                                    id: applyStep2Btn
                                    opacity: down ? 0.8 : 1.0
                                    text: scaleController.calStatus === "CONTINUE_CAL_4/5" ? "FINISH" : "APPLY"
                                    Layout.preferredWidth: 120; Layout.preferredHeight: 45
                                    font.pixelSize: 18; font.bold: true
                                    onClicked: {
                                        scaleController.setLastKnownCalWeight(parseFloat(tfCalW.text))
                                        scaleController.setKnownCalibration(scaleController.lastKnownCalWeight)
                                    }
                                    background: Rectangle {
                                        radius: 6
                                        gradient: Gradient {
                                            orientation: Gradient.Horizontal
                                            GradientStop { position: 0.0; color: calActive ? (applyStep2Btn.down ? cActionPressStart : (applyStep2Btn.hovered ? cActionHoverStart : cActionStart)) : cDisabled }
                                            GradientStop { position: 1.0; color: calActive ? (applyStep2Btn.down ? cActionPressEnd : (applyStep2Btn.hovered ? cActionHoverEnd : cActionEnd)) : cDisabled }
                                        }
                                    }
                                    contentItem: Text {
                                        text: applyStep2Btn.text; font: applyStep2Btn.font
                                        color: "#ffffff"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                                    }
                                    enabled: calActive
                                }
                            }
                        }
                    }
                    Item { Layout.fillHeight: true } // SPACER TO PUSH BATCH DOWN
 
                    Rectangle { Layout.fillWidth: true; height: 2; color: cBorder }
 
                    Text { text: "PRODUCTION RESULTS"; color: cAccent; font.pixelSize: 18; font.bold: true; font.letterSpacing: 2; Layout.alignment: Qt.AlignHCenter }
 
                    // ── STAT CARDS ROW ──
                    RowLayout {
                        Layout.fillWidth: true; spacing: 8
                        // TOTAL card
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 100; radius: 8
                            color: cCard; border.color: cAccent; border.width: 1
                            ColumnLayout {
                                anchors.centerIn: parent; spacing: 4
                                Text { text: "TOTAL"; color: cSubText; font.pixelSize: 14; font.bold: true; font.letterSpacing: 1.5; Layout.alignment: Qt.AlignHCenter }
                                Text { text: scaleController.totalBatch.toString(); color: "#ffffff"; font.pixelSize: 40; font.bold: true; font.family: "monospace"; Layout.alignment: Qt.AlignHCenter }
                                Text { text: "batch"; color: "#bfe0f5"; font.pixelSize: 12; Layout.alignment: Qt.AlignHCenter }
                            }
                        }
                        // PASS card
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 100; radius: 8
                            color: Qt.rgba(0.13, 0.77, 0.37, 0.15); border.color: cSuccess; border.width: 1
                            ColumnLayout {
                                anchors.centerIn: parent; spacing: 4
                                Text { text: "✓ PASS"; color: "#3ed0b4"; font.pixelSize: 14; font.bold: true; font.letterSpacing: 1.5; Layout.alignment: Qt.AlignHCenter }
                                Text { text: scaleController.passBatch.toString(); color: cSuccess; font.pixelSize: 40; font.bold: true; font.family: "monospace"; Layout.alignment: Qt.AlignHCenter }
                                Text { text: "batch"; color: "#1f9e86"; font.pixelSize: 12; Layout.alignment: Qt.AlignHCenter }
                            }
                        }
                        // FAIL card
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 100; radius: 8
                            color: Qt.rgba(0.94, 0.27, 0.27, 0.15); border.color: cDanger; border.width: 1
                            ColumnLayout {
                                anchors.centerIn: parent; spacing: 4
                                Text { text: "✗ FAIL"; color: "#f5a394"; font.pixelSize: 14; font.bold: true; font.letterSpacing: 1.5; Layout.alignment: Qt.AlignHCenter }
                                Text { text: scaleController.failBatch.toString(); color: cDanger; font.pixelSize: 40; font.bold: true; font.family: "monospace"; Layout.alignment: Qt.AlignHCenter }
                                Text { text: "batch"; color: "#b53527"; font.pixelSize: 12; Layout.alignment: Qt.AlignHCenter }
                            }
                        }
                    }
 
                    // ── FAIL STREAK + RESET ──
                    RowLayout {
                        Layout.fillWidth: true; spacing: 8
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 48; radius: 6
                            color: scaleController.consecFails >= 3 ? Qt.rgba(0.94, 0.27, 0.27, 0.15) : cCard
                            border.color: scaleController.consecFails >= 3 ? cDanger : cBorder; border.width: 1
                            RowLayout {
                                anchors.centerIn: parent; spacing: 8
                                Text { text: "⚠ CONSECUTIVE FAILS:"; color: cSubText; font.pixelSize: 16; font.bold: true; font.letterSpacing: 1 }
                                Text { text: scaleController.consecFails.toString(); color: scaleController.consecFails >= 3 ? cDanger : cWarning; font.pixelSize: 28; font.bold: true; font.family: "monospace" }
                            }
                        }
                        MotionButton {
                            id: resetBatchBtn
                            opacity: down ? 0.8 : 1.0
                            text: "RESET BATCH"
                            Layout.preferredWidth: 130; Layout.preferredHeight: 48
                            font.pixelSize: 16; font.bold: true
                            onClicked: scaleController.resetBatch()
                            background: Rectangle {
                                radius: 6
                                gradient: Gradient {
                                    orientation: Gradient.Horizontal
                                    GradientStop { position: 0.0; color: resetBatchBtn.down ? cActionPressStart : (resetBatchBtn.hovered ? cActionHoverStart : cActionStart) }
                                    GradientStop { position: 1.0; color: resetBatchBtn.down ? cActionPressEnd : (resetBatchBtn.hovered ? cActionHoverEnd : cActionEnd) }
                                }
                            }
                            contentItem: Text {
                                text: parent.text; font: parent.font
                                color: "#ffffff"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                            }
                        }
                    }
                }
            }

            // ════════ RIGHT PANEL: Select Profile & Create New ════════
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredWidth: 600
                Layout.fillHeight: true
                color: cPanel
                border.color: cBorder
                border.width: 1
                radius: 8

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 20
                    spacing: 15

                    // Title
                    Text { text: "INK PROFILES"; color: "#67d0ff"; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5; Layout.alignment: Qt.AlignHCenter }
                    
                    // ── MỤC CHỌN MỰC TƯƠNG TỰ 'SELECT MODE' ──
                    Rectangle { Layout.fillWidth: true; height: 1; color: cBorder; opacity: 0.5 }
                    
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 20
                        
                        // Cột 1: Chọn Mực
                        ColumnLayout {
                            Layout.fillWidth: true
                            Text { text: "SELECT INK PROFILE"; color: "#67d0ff"; font.pixelSize: 13; font.bold: true; font.letterSpacing: 1 }
                            Rectangle {
                                Layout.fillWidth: true; Layout.preferredHeight: 36; radius: 4
                                color: cField; border.color: cAccent; border.width: 1
                                ComboBox {
                                    id: inkSelector
                                    onCurrentIndexChanged: {
                                        if (currentIndex >= 0 && mapInkNameInput) {
                                            var selectedInk = inkModel.get(currentIndex);
                                            mapInkNameInput.text = selectedInk.name || "";
                                            mapLotPiInput.text = selectedInk.lot_pi || "";
                                            mapLotCiInput.text = selectedInk.lot_ci || "";
                                        }
                                    }
                                    anchors.fill: parent; anchors.margins: 1
                                    model: inkModel
                                    textRole: "name"
                                    font.pixelSize: 14; font.bold: true
                                    background: Rectangle { color: "transparent" }
                                    indicator: Image {
                                        x: inkSelector.width - width - 10
                                        y: (inkSelector.height - height) / 2
                                        width: 24; height: 24
                                        source: "icons/folder_search.svg"
                                        sourceSize.width: 96
                                        sourceSize.height: 96
                                        fillMode: Image.PreserveAspectFit
                                        smooth: true
                                        mipmap: true
                                        antialiasing: true
                                    }
                                    contentItem: Text { text: inkSelector.currentIndex >= 0 ? inkSelector.displayText : "-- Select Ink --"; font: inkSelector.font; color: cAccent; verticalAlignment: Text.AlignVCenter; horizontalAlignment: Text.AlignHCenter }
                                    popup: Popup {
                                        y: inkSelector.height; width: inkSelector.width; implicitHeight: contentItem.implicitHeight + 36; padding: 0
                                        background: Rectangle { color: cField; border.color: cAccent; border.width: 1; radius: 4 }
                                        contentItem: Column {
                                            width: parent.width
                                            // Table Header
                                            Rectangle {
                                                width: parent.width; height: 30; color: cCard
                                                Row {
                                                    anchors.fill: parent; spacing: 0
                                                    Item { width: parent.width * 0.10; height: parent.height; Text { anchors.centerIn: parent; text: "No"; color: cAccent; font.pixelSize: 12; font.bold: true } }
                                                    Rectangle { width: 1; height: parent.height; color: cBorder }
                                                    Item { width: parent.width * 0.24 - 1; height: parent.height; Text { anchors.centerIn: parent; text: "ID INK"; color: cAccent; font.pixelSize: 12; font.bold: true } }
                                                    Rectangle { width: 1; height: parent.height; color: cBorder }
                                                    Item { width: parent.width * 0.18 - 1; height: parent.height; Text { anchors.centerIn: parent; text: "LOT PI"; color: cAccent; font.pixelSize: 12; font.bold: true } }
                                                    Rectangle { width: 1; height: parent.height; color: cBorder }
                                                    Item { width: parent.width * 0.18 - 1; height: parent.height; Text { anchors.centerIn: parent; text: "LOT CI"; color: cAccent; font.pixelSize: 12; font.bold: true } }
                                                    Rectangle { width: 1; height: parent.height; color: cBorder }
                                                    Item { width: parent.width * 0.18 - 1; height: parent.height; Text { anchors.centerIn: parent; text: "DENSITY"; color: cAccent; font.pixelSize: 12; font.bold: true } }
                                                    Rectangle { width: 1; height: parent.height; color: cBorder }
                                                    Item { width: parent.width * 0.12 - 1; height: parent.height; Text { anchors.centerIn: parent; text: "DEL"; color: cDanger; font.pixelSize: 11; font.bold: true } }
                                                }
                                            }
                                            Rectangle { width: parent.width; height: 2; color: cAccent }
                                            // Table Rows
                                            ListView {
                                                width: parent.width; implicitHeight: contentHeight; clip: true
                                                model: inkSelector.delegateModel
                                            }
                                        }
                                    }
                                    delegate: ItemDelegate {
                                        width: inkSelector.width; height: 34
                                        contentItem: Item {
                                            anchors.fill: parent
                                            Row {
                                                anchors.fill: parent; spacing: 0
                                                Item { width: parent.width * 0.10; height: parent.height; Text { anchors.centerIn: parent; text: (index+1).toString(); color: inkSelector.highlightedIndex === index ? cSelectedText : cSubText; font.pixelSize: 13; font.bold: true; font.family: "monospace" } }
                                                Rectangle { width: 1; height: parent.height; color: cBorder }
                                                Item { width: parent.width * 0.24 - 1; height: parent.height; Text { anchors.left: parent.left; anchors.leftMargin: 6; anchors.verticalCenter: parent.verticalCenter; text: model.name; color: inkSelector.highlightedIndex === index ? cSelectedText : cAccent; font.pixelSize: 13; font.bold: true; font.family: "monospace"; elide: Text.ElideRight; width: parent.width - 10 } }
                                                Rectangle { width: 1; height: parent.height; color: cBorder }
                                                Item { width: parent.width * 0.18 - 1; height: parent.height; Text { anchors.left: parent.left; anchors.leftMargin: 6; anchors.verticalCenter: parent.verticalCenter; text: model.lot_pi || "--"; color: inkSelector.highlightedIndex === index ? cSelectedText : cAccent; font.pixelSize: 12; font.bold: true; font.family: "monospace"; elide: Text.ElideRight; width: parent.width - 10 } }
                                                Rectangle { width: 1; height: parent.height; color: cBorder }
                                                Item { width: parent.width * 0.18 - 1; height: parent.height; Text { anchors.left: parent.left; anchors.leftMargin: 6; anchors.verticalCenter: parent.verticalCenter; text: model.lot_ci || "--"; color: inkSelector.highlightedIndex === index ? cSelectedText : cAccent; font.pixelSize: 12; font.bold: true; font.family: "monospace"; elide: Text.ElideRight; width: parent.width - 10 } }
                                                Rectangle { width: 1; height: parent.height; color: cBorder }
                                                Item { width: parent.width * 0.18 - 1; height: parent.height; Text { anchors.centerIn: parent; text: model.density.toFixed(2) + " g"; color: inkSelector.highlightedIndex === index ? cSelectedText : cAccent; font.pixelSize: 12; font.bold: true; font.family: "monospace" } }
                                                Rectangle { width: 1; height: parent.height; color: cBorder }
                                                Item {
                                                    width: parent.width * 0.12 - 1; height: parent.height
                                                    Rectangle {
                                                        anchors.centerIn: parent; width: 22; height: 22; radius: 4; color: delInkMA.pressed ? "#b53527" : cDanger
                                                        Text { anchors.centerIn: parent; text: "✕"; font.bold: true; font.pixelSize: 12; color: "#fff" }
                                                        MotionMouseArea { id: delInkMA; anchors.fill: parent; onClicked: { scaleController.deleteInkProfile(model.name); } }
                                                    }
                                                }
                                            }
                                            Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: 1; color: cBorder }
                                        }
                                        background: Rectangle { color: inkSelector.highlightedIndex === index ? cAccent : cField }
                                    }
                                }
                            }
                        }

                        // Cột 2: Chọn Vỏ
                        ColumnLayout {
                            Layout.fillWidth: true
                            Text { text: "SELECT CARTRIDGE TYPE"; color: cAccent; font.pixelSize: 13; font.bold: true; font.letterSpacing: 1 }
                            Rectangle {
                                Layout.fillWidth: true; Layout.preferredHeight: 36; radius: 4
                                color: cField; border.color: cAccent; border.width: 1
                                ComboBox {
                                    id: cartSelector
                                    anchors.fill: parent; anchors.margins: 1
                                    model: cartModel
                                    textRole: "name"
                                    font.pixelSize: 14; font.bold: true
                                    background: Rectangle { color: "transparent" }
                                    indicator: Image {
                                        x: cartSelector.width - width - 10
                                        y: (cartSelector.height - height) / 2
                                        width: 24; height: 24
                                        source: "icons/folder_search.svg"
                                        sourceSize.width: 96
                                        sourceSize.height: 96
                                        fillMode: Image.PreserveAspectFit
                                        smooth: true
                                        mipmap: true
                                        antialiasing: true
                                    }
                                    contentItem: Text { text: cartSelector.currentIndex >= 0 ? cartSelector.displayText : "-- Select Cartridge --"; font: cartSelector.font; color: cAccent; verticalAlignment: Text.AlignVCenter; horizontalAlignment: Text.AlignHCenter }
                                    popup: Popup {
                                        y: cartSelector.height; width: cartSelector.width; implicitHeight: contentItem.implicitHeight + 36; padding: 0
                                        background: Rectangle { color: cField; border.color: cAccent; border.width: 1; radius: 4 }
                                        contentItem: Column {
                                            width: parent.width
                                            Rectangle {
                                                width: parent.width; height: 30; color: cCard
                                                Row {
                                                    anchors.fill: parent; spacing: 0
                                                    Item { width: parent.width * 0.12; height: parent.height; Text { anchors.centerIn: parent; text: "No"; color: cAccent; font.pixelSize: 12; font.bold: true } }
                                                    Rectangle { width: 1; height: parent.height; color: cBorder }
                                                    Item { width: parent.width * 0.48 - 1; height: parent.height; Text { anchors.centerIn: parent; text: "CART NAME"; color: cAccent; font.pixelSize: 12; font.bold: true } }
                                                    Rectangle { width: 1; height: parent.height; color: cBorder }
                                                    Item { width: parent.width * 0.28 - 1; height: parent.height; Text { anchors.centerIn: parent; text: "CART WEIGHT"; color: cAccent; font.pixelSize: 12; font.bold: true } }
                                                    Rectangle { width: 1; height: parent.height; color: cBorder }
                                                    Item { width: parent.width * 0.12 - 1; height: parent.height; Text { anchors.centerIn: parent; text: "DEL"; color: cDanger; font.pixelSize: 11; font.bold: true } }
                                                }
                                            }
                                            Rectangle { width: parent.width; height: 2; color: cAccent }
                                            ListView {
                                                width: parent.width; implicitHeight: contentHeight; clip: true
                                                model: cartSelector.delegateModel
                                            }
                                        }
                                    }
                                    delegate: ItemDelegate {
                                        width: cartSelector.width; height: 34
                                        contentItem: Item {
                                            anchors.fill: parent
                                            Row {
                                                anchors.fill: parent; spacing: 0
                                                Item { width: parent.width * 0.12; height: parent.height; Text { anchors.centerIn: parent; text: (index+1).toString(); color: cartSelector.highlightedIndex === index ? cSelectedText : cSubText; font.pixelSize: 13; font.bold: true; font.family: "monospace" } }
                                                Rectangle { width: 1; height: parent.height; color: cBorder }
                                                Item { width: parent.width * 0.48 - 1; height: parent.height; Text { anchors.left: parent.left; anchors.leftMargin: 6; anchors.verticalCenter: parent.verticalCenter; text: model.name; color: cartSelector.highlightedIndex === index ? cSelectedText : cAccent; font.pixelSize: 13; font.bold: true; font.family: "monospace"; elide: Text.ElideRight; width: parent.width - 10 } }
                                                Rectangle { width: 1; height: parent.height; color: cBorder }
                                                Item { width: parent.width * 0.28 - 1; height: parent.height; Text { anchors.centerIn: parent; text: model.density.toFixed(2) + " g"; color: cartSelector.highlightedIndex === index ? cSelectedText : cAccent; font.pixelSize: 13; font.bold: true; font.family: "monospace" } }
                                                Rectangle { width: 1; height: parent.height; color: cBorder }
                                                Item {
                                                    width: parent.width * 0.12 - 1; height: parent.height
                                                    Rectangle {
                                                        anchors.centerIn: parent; width: 22; height: 22; radius: 4; color: delCartMA.pressed ? "#b53527" : cDanger
                                                        Text { anchors.centerIn: parent; text: "✕"; font.bold: true; font.pixelSize: 12; color: "#fff" }
                                                        MotionMouseArea { id: delCartMA; anchors.fill: parent; onClicked: { scaleController.deleteCartProfile(model.name); } }
                                                    }
                                                }
                                            }
                                            Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: 1; color: cBorder }
                                        }
                                        background: Rectangle { color: cartSelector.highlightedIndex === index ? cAccent : cField }
                                    }
                                }
                            }
                        }
                    }

                    // --- DETAILS AREA ---
                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 250
                        color: cCard; border.color: cBorder; border.width: 1; radius: 6
                        
                        GridLayout {
                            anchors.fill: parent; anchors.margins: 15
                            columns: 3; rowSpacing: 15; columnSpacing: 30
 
                            // Row 1
                            RowLayout {
                                Text { text: "ID INK:"; color: cSubText; font.pixelSize: 14; font.bold: true }
                                Text { text: inkSelector.currentIndex >= 0 ? inkModel.get(inkSelector.currentIndex).name : "--"; color: cAccent; font.pixelSize: 18; font.bold: true; font.family: "monospace" }
                            }
                            RowLayout {
                                Text { text: "DENSITY:"; color: cSubText; font.pixelSize: 14; font.bold: true }
                                Text { text: inkSelector.currentIndex >= 0 ? inkModel.get(inkSelector.currentIndex).density.toFixed(2) + " g" : "--"; color: cAccent; font.pixelSize: 18; font.bold: true; font.family: "monospace" }
                            }
                            RowLayout {
                                Text { text: "RELATIVE ERROR (g):"; color: cAccent; font.pixelSize: 14; font.bold: true }
                                Rectangle {
                                    width: 70; height: 35; color: cField; border.color: cAccent; border.width: 1; radius: 4
                                    TextInput {
                                        id: relativeErrorInput
                                        anchors.fill: parent; anchors.margins: 2; color: cAccent; font.pixelSize: 16; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; font.family: "monospace"
                                        text: "1.0"
                                        readOnly: true
                                        MotionMouseArea { anchors.fill: parent; onClicked: { inkTab.numpadTarget = relativeErrorInput; numpadPopup.currentValue = relativeErrorInput.text; numpadPopup.open() } }
                                    }
                                }
                            }
 
                            // Row 2
                            RowLayout {
                                Text { text: "CARTRIDGE TYPE:"; color: cSubText; font.pixelSize: 14; font.bold: true }
                                Text { text: cartSelector.currentIndex >= 0 ? cartModel.get(cartSelector.currentIndex).name : "--"; color: cAccent; font.pixelSize: 18; font.bold: true; font.family: "monospace" }
                            }
                            RowLayout {
                                Text { text: "CART WEIGHT:"; color: cSubText; font.pixelSize: 14; font.bold: true }
                                Text { text: cartSelector.currentIndex >= 0 ? cartModel.get(cartSelector.currentIndex).density.toFixed(2) + " g" : "--"; color: cAccent; font.pixelSize: 18; font.bold: true; font.family: "monospace" }
                            }
                            RowLayout {
                                Text { text: "ML FILL (ml):"; color: cAccent; font.pixelSize: 14; font.bold: true }
                                Rectangle {
                                    width: 70; height: 35; color: cField; border.color: cAccent; border.width: 1; radius: 4
                                    TextInput {
                                        id: inkCapacityInput
                                        anchors.fill: parent; anchors.margins: 2; color: cAccent; font.pixelSize: 16; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; font.family: "monospace"
                                        text: scaleController.inkCapacity.toString()
                                        readOnly: true
                                        MotionMouseArea { anchors.fill: parent; onClicked: { inkTab.numpadTarget = inkCapacityInput; numpadPopup.currentValue = inkCapacityInput.text; numpadPopup.open() } }
                                        Connections {
                                            target: scaleController
                                            function onInkCapacityChanged() {
                                                if (inkTab.numpadTarget !== inkCapacityInput) {
                                                    inkCapacityInput.text = scaleController.inkCapacity.toString();
                                                }
                                            }
                                        }
                                        onTextChanged: {
                                            var testVal = parseFloat(text.replace(",", ".")) || 0.0;
                                            if (testVal > 70.0) {
                                                text = "70.0";
                                            }
                                        }
                                    }
                                }
                            }
 
 
                            // Row 3: Current ML Fill
                            RowLayout {
                                Text { text: "CURRENT ML FILL:"; color: cSubText; font.pixelSize: 14; font.bold: true }
                                Rectangle {
                                    width: 80; height: 35; color: cField; border.color: cAccent; border.width: 1; radius: 4
                                    Text {
                                        anchors.centerIn: parent; text: scaleController.currentMlFill.toFixed(1) + " ml"; color: cSuccess; font.pixelSize: 16; font.bold: true; font.family: "monospace"
                                    }
                                }
                            }
                            RowLayout {
                                Text { text: "ID INK:"; color: cSubText; font.pixelSize: 14; font.bold: true }
                                Text { text: mapInkNameInput.text !== "" ? mapInkNameInput.text : "--"; color: cAccent; font.pixelSize: 16; font.bold: true; font.family: "monospace" }
                            }
                            RowLayout {
                                Text { text: "LOT PI/CI:"; color: cSubText; font.pixelSize: 14; font.bold: true }
                                Text { text: (mapLotPiInput.text !== "" ? mapLotPiInput.text : "--") + " / " + (mapLotCiInput.text !== "" ? mapLotCiInput.text : "--"); color: cAccent; font.pixelSize: 16; font.bold: true; font.family: "monospace" }
                            }
 
                            // Row 4: CONFIRM BTN & CLEAR SELECTION
                            RowLayout {
                                Layout.columnSpan: 3; Layout.fillWidth: true; spacing: 15
                                Item { Layout.fillWidth: true }
                                MotionButton {
                                    id: applyTargetBtn
                                    opacity: down ? 0.8 : 1.0
                                    text: "APPLY TARGET (RUN)"
                                    Layout.preferredHeight: 50; Layout.preferredWidth: 240
                                    background: Rectangle {
                                        radius: 5
                                        gradient: Gradient {
                                            orientation: Gradient.Horizontal
                                            GradientStop { position: 0.0; color: applyTargetBtn.down ? cActionPressStart : (applyTargetBtn.hovered ? cActionHoverStart : cActionStart) }
                                            GradientStop { position: 1.0; color: applyTargetBtn.down ? cActionPressEnd : (applyTargetBtn.hovered ? cActionHoverEnd : cActionEnd) }
                                        }
                                    }
                                    contentItem: Text { text: parent.text; font.pixelSize: 16; font.bold: true; color: "#ffffff"; horizontalAlignment: Qt.AlignHCenter; verticalAlignment: Qt.AlignVCenter }
                                    onClicked: {
                                        if (inkSelector.currentIndex >= 0 && cartSelector.currentIndex >= 0) {
                                            var inkN = inkModel.get(inkSelector.currentIndex).name;
                                            var inkD = inkModel.get(inkSelector.currentIndex).density;
                                            var cartN = cartModel.get(cartSelector.currentIndex).name;
                                            var cartD = cartModel.get(cartSelector.currentIndex).density;
                                            var relE = parseFloat(relativeErrorInput.text.replace(",", ".")) || 0.0;
                                            var mlCap = parseFloat(inkCapacityInput.text.replace(",", ".")) || 0.0;
                                            if (mlCap > 70.0) mlCap = 70.0;
                                            scaleController.confirmTarget(inkN, inkD, cartN, cartD, relE, mlCap);
                                        }
                                    }
                                }
                                MotionButton {
                                    id: clearSelectionBtn
                                    opacity: down ? 0.8 : 1.0
                                    text: "CLEAR SELECTION"
                                    Layout.preferredHeight: 50; Layout.preferredWidth: 160
                                    background: Rectangle {
                                        border.color: cPauseBorder
                                        border.width: 1
                                        radius: 5
                                        gradient: Gradient {
                                            orientation: Gradient.Horizontal
                                            GradientStop { position: 0.0; color: clearSelectionBtn.down ? cPausePressStart : (clearSelectionBtn.hovered ? cPauseHoverStart : cPauseStart) }
                                            GradientStop { position: 1.0; color: clearSelectionBtn.down ? cPausePressEnd : (clearSelectionBtn.hovered ? cPauseHoverEnd : cPauseEnd) }
                                        }
                                    }
                                    contentItem: Item {
                                        Row {
                                            anchors.centerIn: parent
                                            spacing: 7
                                            Image {
                                                source: "icons/brush_cleaning_white.svg"
                                                width: 24; height: 24
                                                sourceSize.width: 96
                                                sourceSize.height: 96
                                                fillMode: Image.PreserveAspectFit
                                                smooth: true
                                                mipmap: true
                                                antialiasing: true
                                                anchors.verticalCenter: parent.verticalCenter
                                            }
                                            Text {
                                                text: "CLEAR SELECTION"
                                                font.pixelSize: 13
                                                font.bold: true
                                                color: "#ffffff"
                                                anchors.verticalCenter: parent.verticalCenter
                                            }
                                        }
                                    }
                                    onClicked: {
                                        inkSelector.currentIndex = -1;
                                        cartSelector.currentIndex = -1;
                                        mapInkNameInput.text = "";
                                        mapLotPiInput.text = "";
                                        mapLotCiInput.text = "";
                                    }
                                }
                            }
                        }
                    }
 
                    // --- ROW 3: TARGET CALCULATIONS ---
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 15
                        
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 65; color: cCard; border.color: cBorder; border.width: 1; radius: 6
                            Column { anchors.centerIn: parent; spacing: 2
                                Text { text: "TOTAL BATCH (g)"; color: cSubText; font.pixelSize: 13; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                Text { text: scaleController.totalBatchWeight.toFixed(2); color: cAccent; font.pixelSize: 22; font.bold: true; font.family: "monospace"; anchors.horizontalCenter: parent.horizontalCenter }
                            }
                        }
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 65; color: cCard; border.color: cWarning; border.width: 1; radius: 6
                            Row { anchors.centerIn: parent; spacing: 8
                                Image {
                                    source: "icons/weight_tilde_yellow.svg"
                                    width: 28; height: 28
                                    sourceSize.width: 96
                                    sourceSize.height: 96
                                    fillMode: Image.PreserveAspectFit
                                    smooth: true
                                    mipmap: true
                                    antialiasing: true
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                                Column { spacing: 2; anchors.verticalCenter: parent.verticalCenter
                                Text { text: "MIN WEIGHT (g)"; color: cWarning; font.pixelSize: 13; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                Text { text: scaleController.minWeight.toFixed(2); color: cWarning; font.pixelSize: 22; font.bold: true; font.family: "monospace"; anchors.horizontalCenter: parent.horizontalCenter }
                                }
                            }
                        }
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 65; color: cCard; border.color: cSuccess; border.width: 1; radius: 6
                            Row { anchors.centerIn: parent; spacing: 8
                                Image {
                                    source: "icons/weight_tilde.svg"
                                    width: 28; height: 28
                                    sourceSize.width: 96
                                    sourceSize.height: 96
                                    fillMode: Image.PreserveAspectFit
                                    smooth: true
                                    mipmap: true
                                    antialiasing: true
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                                Column { spacing: 2; anchors.verticalCenter: parent.verticalCenter
                                Text { text: "MAX WEIGHT (g)"; color: cSuccess; font.pixelSize: 13; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                Text { text: scaleController.maxWeight.toFixed(2); color: cSuccess; font.pixelSize: 22; font.bold: true; font.family: "monospace"; anchors.horizontalCenter: parent.horizontalCenter }
                                }
                            }
                        }
                    }
 
                    // --- ROW 4: CREATE PROFILE ---
                    Rectangle { Layout.fillWidth: true; height: 1; color: cBorder }
                    Text { text: "CREATE NEW PROFILE"; color: cAccent; font.pixelSize: 20; font.bold: true; font.letterSpacing: 1.5 }
 
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 20
 
                        // Create Ink
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 280; color: cCard; border.color: cBorder; border.width: 1; radius: 6
                            GridLayout {
                                anchors.fill: parent; anchors.margins: 15; columns: 3; columnSpacing: 15; rowSpacing: 12
                                
                                // Row 1 labels (Profile identity)
                                Text { text: "ID INK"; color: cSubText; font.pixelSize: 13; font.bold: true; Layout.fillWidth: true; Layout.preferredWidth: 160 }
                                Text { text: "LOT PI"; color: cSubText; font.pixelSize: 13; font.bold: true; Layout.fillWidth: true; Layout.preferredWidth: 200 }
                                Text { text: "LOT CI"; color: cSubText; font.pixelSize: 13; font.bold: true; Layout.fillWidth: false; Layout.preferredWidth: 110 }
 
                                // Row 1 inputs (Profile identity)
                                Rectangle {
                                    Layout.fillWidth: true; Layout.preferredWidth: 160; Layout.preferredHeight: 40; color: cField; border.color: cAccent; border.width: 1; radius: 4
                                    TextInput {
                                        id: mapInkNameInput
                                        anchors.fill: parent; anchors.margins: 4
                                        color: "#ffffff"; font.pixelSize: 18; font.bold: true
                                        horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                                        clip: true
                                    }
                                    Text {
                                        anchors.centerIn: parent
                                        text: "--ID Ink--"
                                        color: cDisabled; font.pixelSize: 14; font.italic: true
                                        visible: mapInkNameInput.text.length === 0 && !mapInkNameInput.activeFocus
                                    }
                                }
                                Rectangle {
                                    Layout.fillWidth: true; Layout.preferredWidth: 200; Layout.preferredHeight: 40; color: cField; border.color: cAccent; border.width: 1; radius: 4
                                    TextInput {
                                        id: mapLotPiInput
                                        anchors.fill: parent; anchors.margins: 4
                                        color: "#ffffff"; font.pixelSize: 18; font.bold: true
                                        horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                                        clip: true
                                    }
                                    Text {
                                        anchors.centerIn: parent
                                        text: "--Lot PI--"
                                        color: cDisabled; font.pixelSize: 14; font.italic: true
                                        visible: mapLotPiInput.text.length === 0 && !mapLotPiInput.activeFocus
                                    }
                                }
                                Rectangle {
                                    Layout.fillWidth: false; Layout.preferredWidth: 110; Layout.preferredHeight: 40; color: cField; border.color: cAccent; border.width: 1; radius: 4
                                    TextInput {
                                        id: mapLotCiInput
                                        anchors.fill: parent; anchors.margins: 4
                                        color: "#ffffff"; font.pixelSize: 18; font.bold: true
                                        horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                                        clip: true
                                    }
                                    Text {
                                        anchors.centerIn: parent
                                        text: "--Lot CI--"
                                        color: cDisabled; font.pixelSize: 14; font.italic: true
                                        visible: mapLotCiInput.text.length === 0 && !mapLotCiInput.activeFocus
                                    }
                                }

                                Text { text: "DENSITY (g)"; color: cSubText; font.pixelSize: 14; font.bold: true; Layout.columnSpan: 3 }
                                Rectangle {
                                    Layout.fillWidth: true; Layout.preferredHeight: 40; Layout.columnSpan: 3; color: cField; border.color: cAccent; border.width: 1; radius: 4
                                    TextInput { id: newInkDensity; anchors.fill: parent; anchors.margins: 4; color: cAccent; font.pixelSize: 18; font.bold:true; font.family: "monospace"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; readOnly: true; text: "0.0"; MotionMouseArea { anchors.fill: parent; onClicked: { inkTab.numpadTarget = newInkDensity; numpadPopup.currentValue = newInkDensity.text; numpadPopup.open() } } }
                                }
 
                                // Row 3 Buttons: Save Profile & Apply Batch Info
                                RowLayout {
                                    Layout.fillWidth: true; Layout.columnSpan: 3; spacing: 15
                                    MotionButton {
                                        id: saveInkBtn
                                        opacity: down ? 0.8 : 1.0
                                        text: "SAVE INK PROFILE"
                                        Layout.preferredHeight: 45; Layout.fillWidth: true
                                        background: Rectangle {
                                            radius: 6
                                            gradient: Gradient {
                                                orientation: Gradient.Horizontal
                                                GradientStop { position: 0.0; color: saveInkBtn.down ? cResumePressStart : (saveInkBtn.hovered ? cResumeHoverStart : cResumeStart) }
                                                GradientStop { position: 1.0; color: saveInkBtn.down ? cResumePressEnd : (saveInkBtn.hovered ? cResumeHoverEnd : cResumeEnd) }
                                            }
                                        }
                                        contentItem: Row {
                                            spacing: 8
                                            anchors.centerIn: parent
                                            Image {
                                                source: "icons/download.svg"
                                                width: 20; height: 20
                                                fillMode: Image.PreserveAspectFit
                                                smooth: true
                                                anchors.verticalCenter: parent.verticalCenter
                                            }
                                            Text {
                                                text: saveInkBtn.text
                                                color: "#ffffff"
                                                font.pixelSize: 15
                                                font.bold: true
                                                anchors.verticalCenter: parent.verticalCenter
                                            }
                                        }
                                        onClicked: {
                                            var d = parseFloat(newInkDensity.text.replace(",", "."));
                                            if (d > 0 && mapInkNameInput.text.trim() !== "") {
                                                scaleController.createInkProfileWithBatch(
                                                    mapInkNameInput.text.trim(),
                                                    d,
                                                    mapLotPiInput.text.trim(),
                                                    mapLotCiInput.text.trim()
                                                );
                                                newInkDensity.text = "0.0";
                                            }
                                        }
                                    }
                                    MotionButton {
                                        id: applyBatchBtn
                                        opacity: down ? 0.8 : 1.0
                                        text: "APPLY BATCH INFO"
                                        Layout.preferredHeight: 45; Layout.fillWidth: true
                                        background: Rectangle {
                                            radius: 6
                                            gradient: Gradient {
                                                orientation: Gradient.Horizontal
                                                GradientStop { position: 0.0; color: applyBatchBtn.down ? cActionPressStart : (applyBatchBtn.hovered ? cActionHoverStart : cActionStart) }
                                                GradientStop { position: 1.0; color: applyBatchBtn.down ? cActionPressEnd : (applyBatchBtn.hovered ? cActionHoverEnd : cActionEnd) }
                                            }
                                        }
                                        contentItem: Text { text: parent.text; color: "#ffffff"; font.pixelSize: 15; font.bold: true; horizontalAlignment: Qt.AlignHCenter; verticalAlignment: Qt.AlignVCenter }
                                        onClicked: {
                                            var payload = {
                                                "action": "set_fields",
                                                "code": mapInkNameInput.text.trim(),
                                                "lot_pi": mapLotPiInput.text.trim(),
                                                "lot_ci": mapLotCiInput.text.trim(),
                                                "operator": "QML"
                                            };
                                            hpController.publishString("ink_batch_code", JSON.stringify(payload));
                                        }
                                    }
                                }
                            }
                        }
 
                        // Create Cart
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 280; color: cCard; border.color: cBorder; border.width: 1; radius: 6
                            GridLayout {
                                anchors.fill: parent; anchors.margins: 15; columns: 2; columnSpacing: 15; rowSpacing: 15
                                Text { text: "CART NAME"; color: cSubText; font.pixelSize: 14; font.bold: true; Layout.alignment: Qt.AlignHCenter }
                                Text { text: "CART WEIGHT (g)"; color: cSubText; font.pixelSize: 14; font.bold: true; Layout.alignment: Qt.AlignHCenter }
                                
                                Rectangle {
                                    Layout.fillWidth: true; Layout.preferredHeight: 40; color: cField; border.color: cAccent; border.width: 1; radius: 4
                                    TextInput { id: newCartName; anchors.fill: parent; anchors.margins: 4; color: "#ffffff"; font.pixelSize: 18; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; clip: true }
                                    Text { anchors.centerIn: parent; text: "--Type Cart Name--"; color: cDisabled; font.pixelSize: 15; font.italic: true; visible: newCartName.text.length === 0 && !newCartName.activeFocus }
                                }
                                Rectangle {
                                    Layout.fillWidth: true; Layout.preferredHeight: 40; color: cField; border.color: cAccent; border.width: 1; radius: 4
                                    TextInput { id: newCartDensity; anchors.fill: parent; anchors.margins: 4; color: cAccent; font.pixelSize: 18; font.bold:true; font.family: "monospace"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; readOnly: true; text: "0.0"; MotionMouseArea { anchors.fill: parent; onClicked: { inkTab.numpadTarget = newCartDensity; numpadPopup.currentValue = newCartDensity.text; numpadPopup.open() } } }
                                }
                                
                                Item { Layout.fillHeight: true; Layout.columnSpan: 2 }
                                
                                MotionButton {
                                    id: saveCartBtn
                                    opacity: down ? 0.8 : 1.0
                                    text: "SAVE CART PROFILE"
                                    Layout.preferredHeight: 45; Layout.fillWidth: true; Layout.columnSpan: 2
                                    background: Rectangle {
                                        radius: 6
                                        gradient: Gradient {
                                            orientation: Gradient.Horizontal
                                            GradientStop { position: 0.0; color: saveCartBtn.down ? cResumePressStart : (saveCartBtn.hovered ? cResumeHoverStart : cResumeStart) }
                                            GradientStop { position: 1.0; color: saveCartBtn.down ? cResumePressEnd : (saveCartBtn.hovered ? cResumeHoverEnd : cResumeEnd) }
                                        }
                                    }
                                    contentItem: Row {
                                        spacing: 8
                                        anchors.centerIn: parent
                                        Image {
                                            source: "icons/download.svg"
                                            width: 20; height: 20
                                            fillMode: Image.PreserveAspectFit
                                            smooth: true
                                            anchors.verticalCenter: parent.verticalCenter
                                        }
                                        Text {
                                            text: saveCartBtn.text
                                            color: "#ffffff"
                                            font.pixelSize: 16
                                            font.bold: true
                                            anchors.verticalCenter: parent.verticalCenter
                                        }
                                    }
                                    onClicked: {
                                        var d = parseFloat(newCartDensity.text.replace(",", "."));
                                        if (d > 0 && newCartName.text.trim() !== "") {
                                            scaleController.createCartProfile(newCartName.text.trim(), d);
                                            newCartName.text = ""; newCartDensity.text = "0.0";
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }    // ── Persistent banner: hiện khi operator chọn NO ở zero-drift popup ──
    // Set bởi scaleController.dismissZeroDrift(); clear khi tare() đc gọi.
    Rectangle {
        id: zeroDriftBanner
        visible: scaleController.zeroDriftPending
        anchors.top: parent.top
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.topMargin: 6
        width: Math.min(parent.width - 40, 720); height: 38
        radius: 6
        color: cAccent
        border.color: cAccent; border.width: 1
        z: 200
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 12; anchors.rightMargin: 8
            spacing: 10
            Text {
                text: "⚠  TEMPO NOT YET TARED — press TARE to clear drift"
                color: "#0c1726"; font.pixelSize: 16; font.bold: true
                Layout.fillWidth: true
                elide: Text.ElideRight
            }
            MotionButton {
                id: driftTareBtn
                text: "TARE"
                Layout.preferredWidth: 80; Layout.preferredHeight: 28
                font.pixelSize: 13; font.bold: true
                background: Rectangle { color: cField; border.color: cAccent; border.width: 1; radius: 4 }
                contentItem: Text {
                    text: parent.text; color: cAccent
                    font: parent.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                onClicked: scaleController.tare()
            }
        }
    }
 
    // Popups
    Popup {
        id: overloadPopup
        parent: Overlay.overlay
        anchors.centerIn: parent
        width: 500; height: 200
        modal: true
        closePolicy: Popup.NoAutoClose
        background: Rectangle { color: cCard; border.color: cDanger; border.width: 2; radius: 10 }
        ColumnLayout {
            anchors.fill: parent; anchors.margins: 20
            Text { text: "WARNING: OVERLOAD!"; color: cDanger; font.pixelSize: 26; font.bold: true; Layout.alignment: Qt.AlignHCenter }
            Text { text: "Scale load exceeds maximum limit. Check immediately."; color: cSubText; font.pixelSize: 18; Layout.alignment: Qt.AlignHCenter }
            Item { Layout.fillHeight: true }
            MotionButton {
                id: ackOverloadBtn
                opacity: down ? 0.8 : 1.0
                text: "ACKNOWLEDGE"
                Layout.alignment: Qt.AlignHCenter
                font.pixelSize: 18; font.bold: true
                onClicked: { scaleController.ackOverload(); overloadPopup.close() }
                background: Rectangle {
                    radius: 6
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: ackOverloadBtn.down ? cDanger : (ackOverloadBtn.hovered ? "#f0735c" : cDanger) }
                        GradientStop { position: 1.0; color: ackOverloadBtn.down ? "#b53527" : (ackOverloadBtn.hovered ? "#d2473a" : "#b53527") }
                    }
                }
                contentItem: Text { text: parent.text; color: "#ffffff"; font: parent.font; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
            }
        }
    }
 
    Popup {
        id: zeroDriftPopup
        parent: Overlay.overlay
        anchors.centerIn: parent
        width: 500; height: 200
        modal: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        background: Rectangle { color: cCard; border.color: cAccent; border.width: 2; radius: 10 }
        ColumnLayout {
            anchors.fill: parent; anchors.margins: 20
            Text { text: "Zero Drift Warning"; color: cAccent; font.pixelSize: 26; font.bold: true; Layout.alignment: Qt.AlignHCenter }
            Text { text: "Loadcell zero drift detected. Re-tare recommended."; color: cSubText; font.pixelSize: 18; Layout.alignment: Qt.AlignHCenter }
            Item { Layout.fillHeight: true }
            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                spacing: 20
                MotionButton {
                    id: zeroDriftTareBtn
                    opacity: down ? 0.8 : 1.0
                    text: "TARE NOW"
                    Layout.preferredWidth: 150
                    Layout.preferredHeight: 45
                    font.pixelSize: 18; font.bold: true
                    onClicked: { scaleController.tare(); zeroDriftPopup.close() }
                    background: Rectangle {
                        radius: 6
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0; color: zeroDriftTareBtn.down ? cActionPressStart : (zeroDriftTareBtn.hovered ? cActionHoverStart : cActionStart) }
                            GradientStop { position: 1.0; color: zeroDriftTareBtn.down ? cActionPressEnd : (zeroDriftTareBtn.hovered ? cActionHoverEnd : cActionEnd) }
                        }
                    }
                    contentItem: Text { text: parent.text; color: "#ffffff"; font: parent.font; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
                MotionButton {
                    opacity: down ? 0.8 : 1.0
                    text: "NO"
                    Layout.preferredWidth: 150
                    Layout.preferredHeight: 45
                    font.pixelSize: 18; font.bold: true
                    onClicked: { scaleController.dismissZeroDrift(); zeroDriftPopup.close() }
                    background: Rectangle { color: Qt.rgba(0.94, 0.27, 0.27, 0.15); border.color: cDanger; border.width: 1; radius: 6 }
                    contentItem: Text { text: parent.text; color: cDanger; font: parent.font; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
            }
        }
    }
 
    Connections {
        target: scaleController
        function onOverloadAlarm() { overloadPopup.open() }
        function onZeroDriftAlarm() { zeroDriftPopup.open() }
    }
 
    // ══════════ VIRTUAL NUMERIC KEYPAD ══════════
    Popup {
        id: numpadPopup
        anchors.centerIn: parent
        width: 320; height: 420
        modal: true
        closePolicy: Popup.CloseOnEscape
        property string currentValue: "0.0"
        property bool isNewEntry: true
        onOpened: isNewEntry = true
        background: Rectangle { color: cCard; radius: 12; border.color: cAccent; border.width: 2 }
        ColumnLayout {
            anchors.fill: parent; anchors.margins: 15; spacing: 8
            Text { text: "ENTER VALUE (g)"; color: cAccent; font.pixelSize: 16; font.bold: true; Layout.alignment: Qt.AlignHCenter }
            Rectangle {
                Layout.fillWidth: true; Layout.preferredHeight: 50; radius: 6
                color: cField; border.color: cAccent; border.width: 1
                Text {
                    id: numpadDisplay
                    anchors.centerIn: parent
                    text: numpadPopup.currentValue
                    color: cAccent; font.pixelSize: 28; font.bold: true; font.family: "monospace"
                }
            }
            GridLayout {
                columns: 3; rowSpacing: 6; columnSpacing: 6
                Layout.fillWidth: true; Layout.fillHeight: true
                Repeater {
                    model: ["7","8","9","4","5","6","1","2","3",".","0","⌫"]
                    Rectangle {
                        Layout.fillWidth: true; Layout.fillHeight: true; radius: 6
                        color: numBtnMA.pressed ? cAccent : cField
                        border.color: cBorder; border.width: 1
                        Text {
                            anchors.centerIn: parent
                            text: modelData; color: "#fff"; font.pixelSize: 22; font.bold: true
                        }
                        MotionMouseArea {
                            id: numBtnMA; anchors.fill: parent
                            onClicked: {
                                if (modelData === "⌫") {
                                    numpadPopup.isNewEntry = false;
                                    numpadPopup.currentValue = numpadPopup.currentValue.length > 1 ? numpadPopup.currentValue.slice(0, -1) : "0";
                                } else {
                                    var nextVal = numpadPopup.currentValue;
                                    if (numpadPopup.isNewEntry) {
                                        nextVal = (modelData === ".") ? "0." : modelData;
                                        numpadPopup.isNewEntry = false;
                                    } else {
                                        if (modelData === ".") {
                                            if (nextVal.indexOf(".") < 0) {
                                                nextVal += ".";
                                            }
                                        } else {
                                            if (nextVal === "0" || nextVal === "0.0") {
                                                nextVal = modelData;
                                            } else {
                                                nextVal += modelData;
                                            }
                                        }
                                    }
                                    if (nextVal.length > 4) {
                                        nextVal = nextVal.slice(0, 4);
                                    }
                                    numpadPopup.currentValue = nextVal;
                                }
                            }
                        }
                    }
                }
            }
            RowLayout {
                Layout.fillWidth: true; spacing: 10
                MotionButton {
                    opacity: down ? 0.8 : 1.0
                    text: "CANCEL"; Layout.fillWidth: true; Layout.preferredHeight: 44
                    font.pixelSize: 14; font.bold: true
                    onClicked: numpadPopup.close()
                    background: Rectangle { radius: 6; color: Qt.rgba(0.94, 0.27, 0.27, 0.15); border.color: cDanger }
                    contentItem: Text { text: parent.text; font: parent.font; color: cDanger; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
                MotionButton {
                    id: numpadOkBtn
                    opacity: down ? 0.8 : 1.0
                    text: "OK"; Layout.fillWidth: true; Layout.preferredHeight: 44
                    font.pixelSize: 14; font.bold: true
                    onClicked: {
                        if (inkTab.numpadTarget) {
                            inkTab.numpadTarget.text = numpadPopup.currentValue;
                        }
                        numpadPopup.close();
                    }
                    background: Rectangle {
                        radius: 6
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.0; color: numpadOkBtn.down ? cActionPressStart : (numpadOkBtn.hovered ? cActionHoverStart : cActionStart) }
                            GradientStop { position: 1.0; color: numpadOkBtn.down ? cActionPressEnd : (numpadOkBtn.hovered ? cActionHoverEnd : cActionEnd) }
                        }
                    }
                    contentItem: Text { text: parent.text; font: parent.font; color: "#ffffff"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
            }
        }
    }
}
