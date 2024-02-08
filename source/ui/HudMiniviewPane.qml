import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Controls.Material 2.12
import net.warsow 2.6

Item {
    id: root

    property var commonDataModel
    property int paneNumber
    property var miniviewAllocator

    property var oldMiniviews: []
    property var oldMiniviewIndices: []

    width: implicitWidth
    height: implicitHeight

    implicitWidth: 120
    implicitHeight: 240

    Rectangle {
        anchors.fill: parent
        color: "transparent"
        border.color: "magenta"
        border.width: 1
    }

    Component.onCompleted: {
        console.assert(miniviewAllocator)
        applyMiniviewLayoutPass1()
        applyMiniviewLayoutPass2()
    }

    // Do nothing on destruction, we don't destroy miview huds except when shutting down

    Connections {
        target: Hud.ui
        onDisplayedHudItemsRetrievalRequested: {
            if (oldMiniviews.length) {
                Hud.ui.supplyDisplayedHudItemAndMargin(root, 4.0)
            }
        }
        onHudMiniviewPanesRetrievalRequested: {
            if (parent) {
                console.assert(parent instanceof HudLayoutItem)
                // Don't let occlusion affect the reported number of panes.
                // Otherwise it's possible to indroduce flicker loops
                // due to moving all views to another pane which becomes occluded next frame.
                if (parent.shouldBeVisibleIfNotOccluded()) {
                    Hud.ui.supplyHudMiniviewPane(paneNumber)
                }
            }
        }
        onHudControlledMiniviewItemsRetrievalRequested: {
            if (root.visible) {
                for (let i = 0; i < oldMiniviews.length; ++i) {
                    Hud.ui.supplyHudControlledMiniviewItemAndModelIndex(oldMiniviews[i], oldMiniviewIndices[i])
                }
            }
        }
    }

    Connections {
        target: commonDataModel
        onMiniviewLayoutChangedPass1: applyMiniviewLayoutPass1()
        onMiniviewLayoutChangedPass2: applyMiniviewLayoutPass2()
    }

    function applyMiniviewLayoutPass1() {
        const miniviewIndices = commonDataModel.getMiniviewIndicesForPane(paneNumber)
        for (let i = 0; i < oldMiniviews.length;) {
            const oldMiniviewIndex = oldMiniviewIndices[i]
            if (!miniviewIndices.includes(oldMiniviewIndex)) {
                const view = oldMiniviews[i]
                oldMiniviewIndices.splice(i, 1)
                oldMiniviews.splice(i, 1)
                miniviewAllocator.recycle(view, oldMiniviewIndex)
            } else {
                ++i
            }
        }
    }

    function applyMiniviewLayoutPass2() {
        const miniviewIndices = commonDataModel.getMiniviewIndicesForPane(paneNumber)
        for (const miniviewIndex of miniviewIndices) {
            if (!oldMiniviewIndices.includes(miniviewIndex)) {
                const view = miniviewAllocator.take(miniviewIndex)
                oldMiniviewIndices.push(miniviewIndex)
                oldMiniviews.push(view)
            }
        }

        console.assert(miniviewIndices.length === oldMiniviewIndices.length)
        console.assert(miniviewIndices.length === oldMiniviews.length)

        // Recalculate bounds
        const itemWidth   = 320
        const itemHeight  = 240
        root.implicitWidth  = itemWidth
        root.implicitHeight = itemHeight * miniviewIndices.length

        let accumHeight = 0
        for (const view of oldMiniviews) {
            view.x      = 0
            view.y      = accumHeight
            view.width  = itemWidth
            view.height = itemHeight
            view.parent = root
            accumHeight += itemHeight
        }
    }
}