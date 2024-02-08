import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Controls.Material 2.12
import QtQuick.Controls.Material.impl 2.12
import net.warsow 2.6

Item {
    id: root
    width: implicitWidth
    height: implicitHeight
    implicitWidth: back.width + 2
    implicitHeight: back.height + 2

    property string iconPath
    property string text
    property color color
    property real value
    property real frac
    property real miniviewScale

    // The actual scale is limited by the label
    readonly property real actualScale: Math.max(72 * miniviewScale, valueLabel.implicitHeight + 2) / 72.0
    readonly property real commonMargin: 3 + 5 * actualScale

    Rectangle {
        id: back
        anchors.centerIn: parent
        width: valueLabel.width + image.width + 3 * commonMargin
        height: 72 * actualScale
        radius: 1
        color: "black"
        opacity: 0.7
    }

    Image {
        id: image
        anchors.left: back.left
        anchors.verticalCenter: back.verticalCenter
        anchors.leftMargin: commonMargin
        width: 40 * actualScale
        height: 40 * actualScale
        smooth: true
        mipmap: true
        source: iconPath
    }

    Label {
        id: valueLabel
        width: 96 * actualScale
        anchors.right: back.right
        anchors.rightMargin: commonMargin
        anchors.verticalCenter: back.verticalCenter
        horizontalAlignment: Qt.AlignRight
        font.family: Hud.ui.numbersFontFamily
        // This condition limits the mininal height of the entire HUD element
        font.pointSize: Math.max(28 * miniviewScale, 10)
        font.weight: Font.Black
        font.letterSpacing: 1.25
        textFormat: Text.PlainText
        text: root.value
        style: Text.Raised
    }
}