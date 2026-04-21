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
}
