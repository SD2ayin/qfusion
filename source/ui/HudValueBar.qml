import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Controls.Material 2.12
import QtQuick.Controls.Material.impl 2.12
import net.warsow 2.6

Item {
    id: root
    width: implicitWidth
    height: implicitHeight
    implicitWidth: 256 + 64 + 32
    implicitHeight: 108

    property string iconPath
    property string text
    property color color
    property real value
    property real frac

    Rectangle {
        id: back
        anchors.centerIn: parent
        width: 256 + 64
        height: 72
        radius: 6
        color: "black"
        opacity: 0.7

        layer.enabled: true
        layer.effect: ElevationEffect { elevation: 16 }
    }

   Image {
        id: image
        anchors.left: back.left
        anchors.top: back.top
        anchors.margins: 8
        width: 24
        height: 24
        smooth: true
        mipmap: true
        source: iconPath
   }

    Item {
        id: bar
        width: back.width - valueLabel.width - 3 * 8
        height: 24
        anchors.left: back.left
        anchors.bottom: back.bottom
        anchors.margins: 8
        // Prevent the animated value bar to leave the desired bounds
        clip: true

        Rectangle {
            anchors.fill: parent
            color: Qt.tint("black", Qt.rgba(root.color.r, root.color.g, root.color.b, 0.2))
            radius: 4
        }

        Rectangle {
            width: Math.max(parent.width * root.frac - 2, 0)
            height: parent.height - 2
            anchors.left: parent.left
            anchors.verticalCenter: parent.verticalCenter
            radius: 4
            color: root.color

            Behavior on width {
                NumberAnimation {
                    duration: 125
                    easing.type: Easing.OutElastic
                }
            }
        }
    }

    Label {
        id: textLabel
        anchors.left: image.right
        anchors.leftMargin: 8
        anchors.verticalCenter: image.verticalCenter
        font.family: wsw.headingFontFamily
        font.weight: Font.Black
        font.letterSpacing: 1.25
        font.pointSize: 13
        textFormat: Text.PlainText
        text: root.text
    }

    Label {
        id: valueLabel
        width: 80
        anchors.right: back.right
        anchors.rightMargin: 8
        anchors.verticalCenter: parent.verticalCenter
        horizontalAlignment: Qt.AlignRight
        font.family: wsw.numbersFontFamily
        font.pointSize: 28
        font.weight: Font.Black
        font.letterSpacing: 1.25
        textFormat: Text.PlainText
        text: root.value
        style: Text.Raised
    }
}