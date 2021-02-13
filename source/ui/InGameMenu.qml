import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Controls.Material 2.12
import QtQuick.Controls.Material.impl 2.12
import QtQuick.Layouts 1.12
import net.warsow 2.6

Rectangle {
    visible: wsw.isShowingInGameMenu
    anchors.fill: parent
    color: "#D8AA5500"

    readonly property real heightFrac: (Math.min(1080, rootItem.height - 720)) / (1080 - 720)

    Rectangle {
        width: parent.width
        height: tabBar.implicitHeight
        anchors.top: parent.top
        anchors.horizontalCenter: parent.horizontalCenter
        color: Material.backgroundColor

        TabBar {
            id: tabBar
            visible: stackView.depth < 2
            width: mainPane.width
            anchors.top: parent.top
            anchors.horizontalCenter: parent.horizontalCenter
            background: null

            Behavior on opacity {
                NumberAnimation { duration: 66 }
            }

            TabButton {
                text: "General"
                readonly property var component: generalComponent
            }
            TabButton {
                text: "Chat"
                readonly property var component: chatComponent
            }
            TabButton {
                text: "Callvotes"
                readonly property var component: callvotesComponent
            }

            onCurrentItemChanged: stackView.replace(currentItem.component)
        }
    }

    Rectangle {
        id: mainPane
        focus: true
        width: 480 + 120 * heightFrac
        height: 560 + 210 * heightFrac
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        color: Material.backgroundColor
        radius: 3

        layer.enabled: parent.enabled
        layer.effect: ElevationEffect { elevation: 64 }

        StackView {
            id: stackView
            anchors.fill: parent
            anchors.margins: 16
            initialItem: generalComponent
            clip: true
        }
    }

    Component {
        id: generalComponent
        InGameGeneralPage {}
    }

    Component {
        id: chatComponent
        InGameChatPage {}
    }

    Component {
        id: callvotesComponent
        InGameCallvotesPage {}
    }

    onVisibleChanged: {
        if (visible) {
            stackView.forceActiveFocus()
        }
    }

    Keys.onPressed: {
        if (!visible) {
            return
        }

        let currentItem = stackView.currentItem
        if (currentItem && currentItem.hasOwnProperty("handleKeyEvent")) {
            let handler = currentItem.handleKeyEvent
            if (handler && handler(event)) {
                return
            }
        }

        if (event.key !== Qt.Key_Escape) {
            return
        }

        event.accepted = true
        if (tabBar.currentIndex) {
            tabBar.currentIndex = 0
            return
        }

        if (stackView.depth === 1) {
            wsw.returnFromInGameMenu()
            mainMenu.forceActiveFocus()
            return
        }

        let handler = stackView.currentItem.handleKeyBack
        if (handler && handler()) {
            return
        }

        // .pop() API quirks
        stackView.pop(stackView.get(stackView.depth - 2))
    }
}