import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15

ApplicationWindow {
    id: previewWindow
    visible: true
    width: 450
    height: 900
    title: "Robot Control Panel Preview"
    color: "#0d1117"

    Rectangle {
        anchors.fill: parent
        color: "#081e29"
        border.color: "#134357"
        radius: 6

        ScrollView {
            anchors.fill: parent
            anchors.margins: 10
            clip: true
            
            ColumnLayout {
                width: parent.width
                spacing: 15

                // Header
                Text {
                    text: "🤖 ROBOT CONTROL"
                    color: "#5cf4f1"
                    font.bold: true
                    font.pixelSize: 18
                    Layout.alignment: Qt.AlignHCenter
                }

                Rectangle {
                    height: 1
                    Layout.fillWidth: true
                    color: "#134357"
                }

                // Placeholder for future functions
                Item {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                }

                // Info
                Text {
                    text: "Empty template - Add your functions here"
                    color: "#94a3b8"
                    font.pixelSize: 11
                    font.italic: true
                    Layout.alignment: Qt.AlignHCenter
                    wrapMode: Text.WordWrap
                    Layout.fillWidth: true
                    horizontalAlignment: Text.AlignHCenter
                }
            }
        }
    }
}
