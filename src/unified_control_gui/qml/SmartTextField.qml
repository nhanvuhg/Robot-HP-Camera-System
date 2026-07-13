import QtQuick 2.15
import QtQuick.Controls 2.15

TextField {
    id: control

    property Item focusHost: null
    property color focusBorderColor: "#67d0ff"
    property int focusBorderRadius: 4

    selectByMouse: false
    activeFocusOnTab: true
    selectionColor: focusBorderColor
    selectedTextColor: "#06101d"
    inputMethodHints: validator ? Qt.ImhFormattedNumbersOnly : Qt.ImhNone

    onActiveFocusChanged: {
        if (activeFocus) {
            if (focusHost && focusHost.registerDataInput)
                focusHost.registerDataInput(control)
        } else {
            deselect()
            if (focusHost && focusHost.unregisterDataInput)
                focusHost.unregisterDataInput(control)
        }
    }

    Keys.onEscapePressed: {
        if (focusHost && focusHost.dismissDataInput)
            focusHost.dismissDataInput()
    }

    Component.onDestruction: {
        if (focusHost && focusHost.unregisterDataInput)
            focusHost.unregisterDataInput(control)
    }

    Rectangle {
        anchors.fill: parent
        radius: control.focusBorderRadius
        color: "transparent"
        border.color: control.focusBorderColor
        border.width: control.activeFocus ? 1 : 0
        visible: control.activeFocus
        z: 1
    }

    MouseArea {
        anchors.fill: parent
        z: 2
        cursorShape: Qt.IBeamCursor
        onPressed: {
            if (!control.activeFocus) {
                control.forceActiveFocus(Qt.MouseFocusReason)
                control.selectAll()
            } else {
                control.deselect()
                control.cursorPosition = control.positionAt(mouse.x, mouse.y)
            }
            Qt.inputMethod.show()
            mouse.accepted = true
        }
    }
}
