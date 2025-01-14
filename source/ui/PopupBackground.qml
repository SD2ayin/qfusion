import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Controls.Material 2.12
import QtQuick.Controls.Material.impl 2.12
import net.warsow 2.6

Rectangle {
    id: root
    Rectangle {
        width: parent.width
        height: 3
        anchors.top: parent.top
        color: Material.accentColor
    }

    implicitWidth: wsw.desiredPopupWidth
    implicitHeight: wsw.desiredPopupHeight
    color: Material.backgroundColor
    radius: 3
    layer.enabled: root.enabled
    layer.effect: ElevationEffect { elevation: 64 }
}