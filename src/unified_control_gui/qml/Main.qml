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
        Component {
            id: inkPage
            InkPage {}
        }
    }

    // ────────────────────────────────────────────────────────────
    // GLOBAL POPUP — feed_chamber timeout resume choice
    // Robot_logic_node phát systemStatus = "WAIT_RESUME_CHOICE" khi
    // LOAD_CHAMBER_FROM_BUFFER timeout 300s + đã drained SCALE qua
    // PLACE. Operator chọn 1 trong 2 cách resume hoặc dừng hệ thống.
    // Popup hiện global (mọi page) để operator không miss.
    // ────────────────────────────────────────────────────────────
    Connections {
        target: robotController
        function onSystemStatusChanged() {
            if (robotController.systemStatus === "WAIT_RESUME_CHOICE") {
                confirmEmptyBufferPopup.close()
                resumeChoicePopup.open()
            } else {
                resumeChoicePopup.close()
                confirmEmptyBufferPopup.close()
            }
        }
    }

    Popup {
        id: resumeChoicePopup
        parent: Overlay.overlay
        anchors.centerIn: parent
        modal: true
        closePolicy: Popup.NoAutoClose
        width: 620; height: 340
        background: Rectangle {
            color: "#081e29"
            border.color: "#f59e0b"
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
                color: "#f59e0b"
                font.pixelSize: 26
                font.bold: true
            }
            Text {
                Layout.fillWidth: true
                text: "feed_chamber timeout 300s — SCALE đã drained qua PLACE.\nChọn cách resume chu trình:"
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
                    background: Rectangle { color: "#4f6cff"; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#ffffff"
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
                    text: "🔂  INIT LOAD\nCHAMBER DIRECT"
                    font.pixelSize: 15; font.bold: true
                    background: Rectangle { color: "#f59e0b"; radius: 6 }
                    contentItem: Text {
                        text: parent.text; color: "#1a1a1a"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: confirmEmptyBufferPopup.open()
                }
            }

            Text {
                Layout.alignment: Qt.AlignHCenter
                text: "Hoặc dùng STOP để dừng hệ thống."
                color: "#8888aa"
                font.pixelSize: 13
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
                text: "⚠  XÁC NHẬN BUFFER RỖNG"
                color: "#ef4444"
                font.pixelSize: 24
                font.bold: true
            }
            Text {
                Layout.fillWidth: true
                text: "INIT LOAD CHAMBER DIRECT sẽ chạy lại như khởi động mới:\n" +
                      "  INIT_LOAD → INIT_REFILL_BUFFER → cycle.\n\n" +
                      "Bạn ĐÃ lấy hết cartridge khỏi BUFFER thủ công chưa?"
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
                    text: "✓  Buffer rỗng — CONFIRM"
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
                    text: "✗  Hủy / Quay lại"
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
}
