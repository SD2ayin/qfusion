pragma Singleton
import QtQuick 2.12
import net.warsow 2.6

QtObject {
    readonly property var ui: __ui
    readonly property var commonDataModel: __hudCommonDataModel
    readonly property var povDataModel: __hudPovDataModel
    readonly property var actionRequestsModel: __actionRequestsModel
    readonly property var chatProxy: __chatProxy
    readonly property var teamChatProxy: __teamChatProxy

    function destroyLayer(layer) {
        if (layer) {
            layer.enabled = false
            layer.effect  = null
            ui.ensureObjectDestruction(layer)
        }
    }
}