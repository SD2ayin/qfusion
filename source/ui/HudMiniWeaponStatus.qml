import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Controls.Material 2.12
import QtQuick.Controls.Material.impl 2.12
import net.warsow 2.6

Item {
    id: root
    width: implicitWidth
    height: implicitHeight
    implicitWidth: back.width + 16 * actualScale
    implicitHeight: back.height + 16 * actualScale

    property var povDataModel
    property real miniviewScale

    readonly property var actualScale: textMetrics.boundingRect.height / 80

    TextMetrics {
        id: textMetrics
        font.family: Hud.ui.numbersFontFamily
        font.pointSize: Math.max(32 * miniviewScale, 11)
        text: "0"
    }

    Rectangle {
        id: back
        anchors.centerIn: parent
        radius: 12 * actualScale
        width: 128 * actualScale
        height: 172 * actualScale
        color: "black"
        opacity: 0.7
    }

    Image {
        id: icon
        anchors.centerIn: parent
        width: 56 * actualScale
        height: 56 * actualScale
        smooth: true
        mipmap: true
        source: povDataModel.activeWeaponIcon
    }

    readonly property real strongAmmo: povDataModel.activeWeaponStrongAmmo
    readonly property real weakAmmo: povDataModel.activeWeaponWeakAmmo

    readonly property string infinity: "\u221E"

    Label {
        anchors.bottom: icon.top
        anchors.bottomMargin: (strongAmmo >= 0 ? -6 : -16) * actualScale
        anchors.horizontalCenter: parent.horizontalCenter
        font.family: Hud.ui.numbersFontFamily
        font.weight: Font.Black
        font.letterSpacing: 1.25
        font.pointSize: (strongAmmo >= 0 ? 32 : 40) * actualScale
        style: Text.Raised
        textFormat: Text.PlainText
        text: strongAmmo > 0 ? strongAmmo : (strongAmmo < 0 ? infinity : '-')
        opacity: strongAmmo ? 1.0 : 0.5
    }

    Label {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: icon.bottom
        anchors.topMargin: (weakAmmo >= 0 ? -4 : -24) * actualScale
        font.family: Hud.ui.numbersFontFamily
        font.weight: Font.Black
        font.letterSpacing: 1.25
        font.pointSize: (weakAmmo >= 0 ? 32 : 40) * actualScale
        style: Text.Raised
        textFormat: Text.PlainText
        text: weakAmmo > 0 ? weakAmmo : (weakAmmo < 0 ? infinity : '-')
        opacity: weakAmmo ? 1.0 : 0.5
    }
}