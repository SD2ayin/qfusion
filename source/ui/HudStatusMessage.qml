import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Controls.Material 2.12
import net.warsow 2.6

Item {
    id: root
    implicitWidth: stackView.currentItem ? stackView.currentItem.implicitWidth + 96 : 0
    implicitHeight: stackView.height

    property var povDataModel
    property bool isMiniview

    Connections {
        target: root.povDataModel
        onStatusMessageChanged: {
            stackView.clear(StackView.Immediate)
            stackView.push(component, {"message": statusMessage}, StackView.PushTransition)
            timer.start()
        }
    }

    Timer {
        id: timer
        interval: 3000
        onTriggered: stackView.clear(StackView.PopTransition)
    }

    StackView {
        id: stackView
        width: rootItem.width
        height: 96
        anchors.centerIn: parent

        pushEnter: Transition {
            NumberAnimation {
                property: "transformXScale"
                from: 0.0; to: 1.0
                easing.type: Easing.InOutElastic
                easing.amplitude: 5.0
                duration: 333
            }
            NumberAnimation {
                property: "transformYScale"
                from: 0.0; to: 1.0
                easing.type: Easing.InOutElastic
                easing.amplitude: 5.0
                duration: 333
            }
        }

        popExit: Transition {
            NumberAnimation {
                property: "transformXScale"
                from: 1.0; to: 0.0
                easing.type: Easing.InCubic
                duration: 64
            }
            NumberAnimation {
                property: "transformYScale"
                from: 1.0; to: 0.0
                easing.type: Easing.InCubic
                duration: 64
            }
        }
    }

    Component {
        id: component
        Label {
            id: instantiatedItem
            width: implicitWidth
            height: implicitHeight
            // We must report individual items and not the stack view, otherwise we lose items in transitions
            Connections {
                target: Hud.ui
                enabled: !root.isMiniview
                onDisplayedHudItemsRetrievalRequested: Hud.ui.supplyDisplayedHudItemAndMargin(instantiatedItem, 4.0)
            }
            property string message
            property real transformXScale
            property real transformYScale
            transform: Scale {
                origin.x: 0.5 * width
                origin.y: 0.5 * height
                xScale: transformXScale
                yScale: transformYScale
            }
            horizontalAlignment: Qt.AlignHCenter
            verticalAlignment: Qt.AlignVCenter
            font.weight: Font.Bold
            font.pointSize: 20
            font.letterSpacing: 2
            font.wordSpacing: 3
            font.capitalization: Font.SmallCaps
            textFormat: Text.StyledText
            style: Text.Raised
            text: message
        }
    }
}