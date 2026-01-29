import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls.Material 2.15
import QtQuick.Window 2.15
import QtCharts 2.15

ApplicationWindow {
    id: mainWindow 
    visible: true
    width: 1920
    height: 1080
    title: "ROS2 - CAMERA VIEWER"
    color: "#0d1117"
    visibility: Window.FullScreen
    property string currentTime: Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm")

    Shortcut {
        sequence: "F11"
        onActivated: {
            if (mainWindow.visibility === Window.FullScreen)
                mainWindow.visibility = Window.Windowed
            else
                mainWindow.visibility = Window.FullScreen
        }
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        // Title row
        Item {
            id: itemTitleRow
            width: parent.width
            height: 80

            property string currentTime: Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")

            Timer {
                interval: 1000
                running: true
                repeat: true
                onTriggered: {
                    currentTime = Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
                }
            }

            Rectangle {
                anchors.fill: parent
                color: "transparent"

                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 10
                    Item {
                        Layout.preferredWidth: 60
                    }
                    

                    
                    Item {
                        Layout.fillWidth: true

                        Text {
                            id: titleText
                            anchors.centerIn: parent
                            text: "ROS2 - ROBOT CONTROL SYSTEM"
                            font.pixelSize: 24
                            font.bold: true
                            color: "#6cf"
                        }
                    }

                    Button {
                        Layout.preferredWidth: 50
                        Layout.preferredHeight: 50

                        onClicked: {
                            var comp = Qt.createComponent("frm_settings.qml");
                            if (comp.status === Component.Ready) {
                                var win = comp.createObject(mainWindow);
                                if (win) {
                                    win.x = mainWindow.x + (mainWindow.width - win.width) / 2;
                                    win.y = mainWindow.y + (mainWindow.height - win.height) / 2;
                                    win.show();
                                } else {
                                    console.log("createObject failed");
                                }
                            } else {
                                console.log("Failed to load settings:", comp.errorString());
                            }
                        }

                        background: Rectangle {
                            radius: 6
                            color: "transparent"
                            border.color: "#134357"
                            border.width: 2
                        }
                        
                        contentItem: Image {
                            source: "qrc:/icons/qml/icons/settings.svg"
                            width: 24
                            height: 24
                            fillMode: Image.PreserveAspectFit
                            smooth: true
                        }
                    }
                    
                    Button {
                        Layout.preferredWidth: 50
                        Layout.preferredHeight: 50
                        onClicked: Qt.quit()

                        background: Rectangle {
                            radius: 6
                            color: "transparent"
                            border.color: "#134357"
                            border.width: 2
                        }

                        contentItem: Image {
                            source: "qrc:/icons/qml/icons/power_settings.svg"
                            width: 24
                            height: 24
                            fillMode: Image.PreserveAspectFit
                            smooth: true
                        }
                    }
                }
            }
        }


        // Main content
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 10
            
            // Middle Left: Camera Area (Flexible height, but prioritized)
            Rectangle{
                color: "#081e29"
                Layout.fillHeight: true
                Layout.fillWidth: true
                Layout.preferredWidth: 3.5
                border.color: "#134357"
                radius: 6


                GridLayout {
                    id: camGrid
                    columns: 2
                    rowSpacing: 10
                    columnSpacing: 10
                    anchors.fill: parent
                    anchors.margins: 10
                    
                    Repeater {
                        model: camNode.cameraList

                        delegate: CameraView {
                            cameraName: modelData.name
                            topic: modelData.topic
                            providerId: modelData.providerId
                            
                            // Let GridLayout handle sizing, but hint aspect ratio could be enforced inside CameraView
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                        }
                    }
                }
            }

            // Middle Right: Status Panel (Vertical)
            Rectangle {
                Layout.fillHeight: true
                Layout.preferredWidth: 350
                Layout.minimumWidth: 300
                color: "#081e29"
                border.color: "#134357"
                radius: 6

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 15

                    Text {
                        text: "SYSTEM MONITOR"
                        color: "#5cf4f1"
                        font.bold: true
                        font.pixelSize: 22
                        Layout.alignment: Qt.AlignHCenter
                    }
                    
                    Rectangle { Layout.fillWidth: true; height: 1; color: "#134357" }

                    // Status Grid
                    GridLayout {
                        Layout.fillWidth: true
                        columns: 2
                        rowSpacing: 10
                        columnSpacing: 10
                        
                        Text { text: "Status:"; color: "#94a3b8"; font.pixelSize: 18 }
                        Text { text: robotController.systemStatus; color: "#10b981"; font.bold: true; font.pixelSize: 18 }
                        
                        Text { text: "Mode:"; color: "#94a3b8"; font.pixelSize: 18 }
                        Text { 
                            text: "MANUAL" // Placeholder, should bind to actual mode if available
                            color: "#6366f1"; font.bold: true; font.pixelSize: 18 
                        }

                        Text { text: "Operation Time:"; color: "#94a3b8"; font.pixelSize: 18 }
                        Text { text: robotController.systemUptime; color: "#f59e0b"; font.bold: true; font.pixelSize: 18 }

                        Text { text: "Tray Count:"; color: "#94a3b8"; font.pixelSize: 18 }
                        Text { text: robotController.trayCount; color: "#8b5cf6"; font.bold: true; font.pixelSize: 18 }

                        Text { text: "Row:"; color: "#94a3b8"; font.pixelSize: 18 }
                        Text { text: robotController.selectedRow > 0 ? robotController.selectedRow : "-"; color: "#5cf4f1"; font.bold: true; font.pixelSize: 18 }

                        Text { text: "Slot:"; color: "#94a3b8"; font.pixelSize: 18 }
                        Text { text: robotController.selectedSlot > 0 ? robotController.selectedSlot : "-"; color: "#5cf4f1"; font.bold: true; font.pixelSize: 18 }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#134357" }
                    
                    // Error Display
                    Rectangle {
                        Layout.fillWidth: true
                        height: errorText.visible ? 60 : 0
                        color: "#ef444420"
                        border.color: "#ef4444"
                        radius: 6
                        visible: robotController.errorMessage !== ""
                        
                        Text {
                            id: errorText
                            anchors.fill: parent
                            anchors.margins: 5
                            text: robotController.errorMessage
                            color: "#ef4444"
                            font.pixelSize: 16
                            wrapMode: Text.WordWrap
                        }
                    }

                    Item { Layout.fillHeight: true } // Spacer
                }
            }
        }

        Item {
            height: 40
            Layout.fillWidth: true

            Rectangle {
                anchors.fill: parent
                color: "#0d2538"
                border.color: "#134357"

                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    Text {
                        text: "© 2025 RYNAN TECHNOLOGIES"
                        color: "#6cf"
                        font.pixelSize: 16
                        Layout.alignment: Qt.AlignVCenter
                    }

                    Item { Layout.fillWidth: true }

                    RowLayout {
                        spacing: 6

                        Image {
                            source: "qrc:/icons/qml/icons/app_badging.svg"
                            width: 24
                            height: 24
                            fillMode: Image.PreserveAspectFit
                            smooth: true
                            Layout.preferredWidth: 24
                            Layout.preferredHeight: 24
                            Layout.alignment: Qt.AlignVCenter
                        }

                        Text {
                            text: "Status: Running"
                            color: "#00ff99"
                            font.pixelSize: 16
                            Layout.alignment: Qt.AlignVCenter
                        }
                    }

                    Rectangle{
                        width: 2
                        Layout.fillHeight: true
                        color:"#134357"
                    }

                    RowLayout {
                        spacing: 6
                        Layout.alignment: Qt.AlignVCenter

                        Image {
                            source: "qrc:/icons/qml/icons/schedule.svg"
                            fillMode: Image.PreserveAspectFit
                            smooth: true
                            Layout.preferredWidth: 24
                            Layout.preferredHeight: 24
                            Layout.alignment: Qt.AlignVCenter
                        }

                        Text {
                            text: currentTime
                            font.pixelSize: 16
                            color: "#6cf"
                            Layout.alignment: Qt.AlignVCenter
                        }
                    }
                }
            }
        }
    }
}
