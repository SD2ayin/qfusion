import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Controls.Material 2.12
import QtQuick.Controls.Material.impl 2.12
import QtQuick.Layouts 1.12
import net.warsow 2.6

Item {
    id: root
    width: implicitWidth
    height: implicitHeight
    
    property var commonDataModel
    property var povDataModel

    // TODO: Make the native code track the longest string length and give a hint?
    implicitWidth: 256 + 32 + 32 + (root.commonDataModel.hasLocations ? 128 + 32 : 0)
    implicitHeight: list.height + 32

    readonly property color evenRowColor: Qt.rgba(0.0, 0.0, 0.0, 0.7)
    readonly property color oddRowColor: Qt.rgba(0.03, 0.03, 0.03, 0.7)

    Connections {
        target: Hud.ui
        onDisplayedHudItemsRetrievalRequested: {
            if (list.count) {
                Hud.ui.supplyDisplayedHudItemAndMargin(list, 64.0)
            }
        }
    }

    ListView {
        id: list
        anchors.centerIn: parent
        height: contentHeight
        width: root.width - 32
        model: root.povDataModel.getTeamListModel()
        layer.enabled: count
        layer.effect: ElevationEffect { elevation: 64 }
        Component.onDestruction: Hud.destroyLayer(layer)
        delegate: Rectangle {
            id: listDelegate
            width: list.width
            implicitHeight: 40
            color: index % 2 ? oddRowColor : evenRowColor
            Component.onDestruction: {
                Hud.ui.ensureObjectDestruction(model)
                Hud.ui.ensureObjectDestruction(listDelegate)
            }

            RowLayout {
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.leftMargin: 12
                anchors.rightMargin: 12
                anchors.verticalCenter: parent.verticalCenter
                spacing: 8

                Image {
                    Layout.preferredWidth: 16
                    Layout.preferredHeight: 16
                    smooth: true
                    mipmap: true
                    source: weaponIconPath
                }
                Label {
                    Layout.preferredWidth: 32
                    horizontalAlignment: Qt.AlignRight
                    font.family: Hud.ui.numbersFontFamily
                    font.weight: Font.ExtraBold
                    font.pointSize: 12
                    font.letterSpacing: 0.5
                    style: Text.Raised
                    textFormat: Text.PlainText
                    text: health
                }
                Label {
                    font.family: Hud.ui.numbersFontFamily
                    font.weight: Font.ExtraBold
                    font.pointSize: 12
                    style: Text.Raised
                    textFormat: Text.PlainText
                    text: "|"
                }
                Label {
                    Layout.preferredWidth: 32
                    horizontalAlignment: Qt.AlignLeft
                    font.family: Hud.ui.numbersFontFamily
                    font.weight: Font.ExtraBold
                    font.pointSize: 12
                    font.letterSpacing: 0.5
                    style: Text.Raised
                    textFormat: Text.PlainText
                    text: armor
                }
                Column {
                    Layout.preferredWidth: 32
                    spacing: 4
                    Rectangle {
                        width: 32
                        height: 5
                        color: Qt.rgba(1.0, 1.0, 1.0, 0.15)
                        Rectangle {
                            width: parent.width * 0.01 * Math.min(Math.max(0, health), 100)
                            height: parent.height
                            anchors.left: parent.left
                            anchors.verticalCenter: parent.verticalCenter
                            color: health > 100 ? "deeppink" : "green"
                        }
                    }
                    Rectangle {
                        width: 32
                        height: 5
                        color: Qt.rgba(1.0, 1.0, 1.0, 0.15)
                        Rectangle {
                            width: parent.width * 0.01 * Math.min(Math.max(0, armor), 100)
                            height: parent.height
                            anchors.left: parent.left
                            anchors.verticalCenter: parent.verticalCenter
                            color: "white"
                        }
                    }
                }
                Label {
                    Layout.fillWidth: true
                    Layout.preferredWidth: 1.00
                    horizontalAlignment: Qt.AlignHCenter
                    font.weight: Font.ExtraBold
                    font.pointSize: 12
                    font.letterSpacing: 1
                    style: Text.Raised
                    textFormat: Text.StyledText
                    text: nickname
                    elide: Text.ElideMiddle
                }
                Label {
                    visible: root.commonDataModel.hasLocations
                    Layout.fillWidth: true
                    Layout.preferredWidth: 0.75
                    horizontalAlignment: Qt.AlignRight
                    font.weight: Font.ExtraBold
                    font.pointSize: 12
                    font.letterSpacing: 1
                    style: Text.Raised
                    elide: Text.ElideRight
                    textFormat: Text.StyledText
                    text: location
                    opacity: 0.7
                }
            }
        }
    }
}