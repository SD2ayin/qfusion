import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Controls.Material 2.12

MouseArea {
    id: root
    height: 36

    hoverEnabled: true

    property bool selected
    property bool detailed
    property string text

    Label {
        id: label
        text: root.text
        anchors.right: undefined
        anchors.horizontalCenter: parent.horizontalCenter

        color: (root.containsMouse || root.selected) ? Material.accent : Material.foreground
        font.weight: Font.Bold
        font.pointSize: 13
        font.letterSpacing: (root.containsMouse || root.selected) ? 1.75 : 1.25
        font.capitalization: Font.AllUppercase

        Behavior on font.letterSpacing { NumberAnimation { duration: 67 } }

        transitions: Transition {
            AnchorAnimation {
                duration: 200
                easing.type: Easing.OutBack
            }
        }

        state: "initial"
        states: [
            State {
                name: "detailed"
                when: detailed
                AnchorChanges {
                    target: label
                    anchors.right: root.right
                    anchors.horizontalCenter: undefined
                }
            },
            State {
                name: "initial"
                when: !detailed
                AnchorChanges {
                    target: label
                    anchors.right: undefined
                    anchors.horizontalCenter: root.horizontalCenter
                }
            }
        ]
    }
}