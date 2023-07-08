pragma Singleton
import QtQuick 2.12
import net.warsow 2.6

QtObject {
    readonly property var ui: __ui
    readonly property var dataModel: __hudDataModel
    readonly property var actionRequestsModel: __actionRequestsModel
    readonly property var chatProxy: __chatProxy
    readonly property var teamChatProxy: __teamChatProxy
}