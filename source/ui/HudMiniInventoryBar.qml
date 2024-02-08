import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Controls.Material 2.12
import QtQuick.Controls.Material.impl 2.12
import QtQuick.Layouts 1.12
import net.warsow 2.6

Item {
    id: root
    // Consider the extra space to the left/right to be equal to the spacing
    implicitWidth: cardWidth * povDataModel.getInventoryModel().numInventoryItems +
                       layout.spacing * (povDataModel.getInventoryModel().numInventoryItems + 1)
    implicitHeight: cardHeight + 32 * actualScale
    width: implicitWidth
    height: implicitHeight

    property var povDataModel
    property real miniviewScale

    readonly property real actualScale: Math.max(12, 32 * miniviewScale) / 32.0

    readonly property real cardWidth: 45 * actualScale
    readonly property real cardHeight: 58 * actualScale
    readonly property real cardRadius: 1 + 8 * actualScale

    RowLayout {
        id: layout
        anchors.centerIn: parent
        spacing: 4 * miniviewScale < 1.0 ? 1.0 : 12 * miniviewScale

        Repeater {
            model: root.povDataModel.getInventoryModel()
            delegate: Item {
                id: delegateItem
                implicitWidth: visible ? cardWidth : 0
                implicitHeight: visible ? cardHeight : 0
                Layout.preferredWidth: implicitWidth
                Layout.preferredHeight: implicitHeight
                Layout.alignment: Qt.AlignVCenter
                visible: model.displayed

                Behavior on implicitWidth {
                    SmoothedAnimation { duration: 100 }
                }
                Behavior on implicitHeight {
                    SmoothedAnimation { duration: 100 }
                }

                Component.onDestruction: {
                    Hud.ui.ensureObjectDestruction(model)
                    Hud.ui.ensureObjectDestruction(delegateItem)
                }

                Rectangle {
                    id: frame
                    anchors.centerIn: parent
                    width: delegateItem.width
                    height: cardHeight
                    radius: cardRadius
                    color: "black"
                    opacity: 0.7
                }

                Rectangle {
                    anchors.centerIn: parent
                    width: delegateItem.visible ? delegateItem.width + 2 : 0
                    height: cardHeight + 2 * border.width
                    radius: cardRadius
                    color: "transparent"
                    border.width: Math.max(1.5, 4 * actualScale)
                    border.color: Qt.lighter(model.color, 1.1)
                    visible: active
                }

                Image {
                    id: icon
                    visible: delegateItem.width + 2 > icon.width
                    anchors.centerIn: parent
                    width: 32 * actualScale
                    height: 32 * actualScale
                    source: model.hasWeapon ? model.iconPath : (model.iconPath + "?grayscale=true")
                    smooth: true
                    mipmap: true
                }
            }
        }
    }
}