import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Controls.Material 2.12
import QtQuick.Controls.Material.impl 2.12
import QtQuick.Layouts 1.12
import net.warsow 2.6

Item {
    id: root

    // Wrap the loader in an item to avoid providing inconsistent implicitHeight in the hidden state

    implicitWidth: 720
    implicitHeight: 192 - 32
    width: implicitWidth
    height: implicitHeight

    readonly property color defaultMaterialAccent: Material.accent

    Loader {
        anchors.fill: parent
        active: wsw.isShowingChatPopup || wsw.isShowingTeamChatPopup
        sourceComponent: chatComponent
    }

    Component {
        id: chatComponent
        Item {
            id: contentFrame
            width: root.width
            height: root.height
            layer.enabled: true
            layer.effect: ElevationEffect { elevation: 64 }

            Rectangle {
                anchors.fill: parent
                color: wsw.isShowingTeamChatPopup ? Material.accent : "black"
                radius: 5
                opacity: 0.7
            }

            TextField {
                id: input
                Material.accent: wsw.isShowingTeamChatPopup ? "white" : defaultMaterialAccent

                anchors.top: parent.top
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.topMargin: 8
                anchors.leftMargin: 12
                anchors.rightMargin: 12

                font.weight: Font.Medium
                font.pointSize: 12

                Component.onCompleted: forceActiveFocus()

                onEditingFinished: {
                    if (wsw.isShowingTeamChatPopup) {
                        teamChatProxy.sendMessage(text)
                    } else {
                        chatProxy.sendMessage(text)
                    }
                    text = ""
                }
            }

            ListView {
                model: wsw.isShowingTeamChatPopup ? teamChatProxy.getCompactModel() : chatProxy.getCompactModel()
                verticalLayoutDirection: ListView.TopToBottom
                spacing: 4
                clip: true
                displayMarginBeginning: 0
                displayMarginEnd: 0
                anchors.top: input.bottom
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.bottom: parent.bottom
                anchors.topMargin: 0
                anchors.bottomMargin: 0
                anchors.leftMargin: 12
                anchors.rightMargin: 12

                delegate: RowLayout {
                    width: parent.width
                    Label {
                        Layout.alignment: Qt.AlignTop
                        Layout.rightMargin: 2
                        font.pointSize: 12
                        font.weight: Font.Black
                        font.letterSpacing: 0.5
                        textFormat: Text.StyledText
                        style: Text.Raised
                        text: model.name + ':'
                    }
                    Label {
                        Layout.fillWidth: true
                        wrapMode: Text.WordWrap
                        font.pointSize: 12
                        font.weight: Font.Medium
                        font.letterSpacing: 0.5
                        textFormat: Text.StyledText
                        clip: true
                        text: model.text
                    }
                }
            }
        }
    }

    Keys.onPressed: {
        if (wsw.isShowingChatPopup || wsw.isShowingTeamChatPopup) {
            if (event.key == Qt.Key_Escape || event.key == Qt.Key_Back) {
                wsw.closeChatPopup()
            }
        }
    }
}