import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Controls.Material 2.12
import QtQuick.Controls.Material.impl 2.12
import QtQuick.Layouts 1.12
import net.warsow 2.6

Rectangle {
    id: root

    color: wsw.colorWithAlpha(Material.background, wsw.fullscreenOverlayOpacity)

    readonly property real baseCellWidth: 64
    readonly property real clanCellWidth: 96

    readonly property real tableWidth: 600

    // A column layout could have been more apporpriate but it lacks hoirzontal center offset properties

    Loader {
        id: teamTablesLoader
        anchors.top: parent.top
        anchors.topMargin: 0.25 * root.height
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.horizontalCenterOffset: 0.5 * root.baseCellWidth
        sourceComponent: {
            if (!hudDataModel.hasTwoTeams) {
                playersDisplayComponent
            } else if (scoreboard.layout === Scoreboard.Mixed) {
                mixedTeamsDisplayComponent
            } else if (scoreboard.layout === Scoreboard.SideBySide && root.width > 2 * root.tableWidth) {
                sideBySideTeamsDisplayComponent
            } else {
                columnWiseTeamsDisplayComponent
            }
        }
    }

    Item {
        id: hudOccluder
        anchors.top: teamTablesLoader.top
        anchors.horizontalCenter: teamTablesLoader.horizontalCenter
        width: Math.max(teamTablesLoader.item.width, specsPane.width)
        height: teamTablesLoader.item.height + column.anchors.topMargin + column.height

        Component.onCompleted: wsw.registerHudOccluder(hudOccluder)
        Component.onDestruction: wsw.unregisterHudOccluder(hudOccluder)
        onWidthChanged: wsw.updateHudOccluder(hudOccluder)
        onHeightChanged: wsw.updateHudOccluder(hudOccluder)
        onXChanged: wsw.updateHudOccluder(hudOccluder)
        onYChanged: wsw.updateHudOccluder(hudOccluder)
    }

    Column {
        id: column
        anchors.top: teamTablesLoader.bottom
        anchors.topMargin: 48
        anchors.horizontalCenter: parent.horizontalCenter
        spacing: 32

        ScoreboardSpecsPane {
            playersPerRow: 3
            playersInFirstRow: 2
            width: root.tableWidth - root.baseCellWidth
            height: implicitHeight
            model: scoreboard.challengersModel
            title: "Challengers"
        }

        ScoreboardSpecsPane {
            id: specsPane
            playersPerRow: 3
            playersInFirstRow: 3
            width: root.tableWidth - root.baseCellWidth
            height: implicitHeight
            model: scoreboard.specsModel
            title: "Spectators"
        }

        ScoreboardSpecsPane {
            playersPerRow: 3
            playersInFirstRow: 3
            visible: scoreboard.hasChasers && scoreboard.chasersModel.length
            width: root.tableWidth - root.baseCellWidth
            height: implicitHeight
            model: scoreboard.chasersModel
            title: "Chasers"
        }

        ScoreboardStatsPane {
            id: statsPane
            width: root.tableWidth - root.baseCellWidth
            height: implicitHeight
        }
    }

    Component {
        id: playersDisplayComponent

        ScoreboardTeamPane {
            width: root.tableWidth
            model: scoreboardPlayersModel
            baseColor: Qt.lighter(Material.background)
            baseCellWidth: root.baseCellWidth
            clanCellWidth: root.clanCellWidth
        }
    }

    Component {
        id: sideBySideTeamsDisplayComponent

        Row {
            ScoreboardTeamPane {
                width: root.tableWidth
                model: scoreboardAlphaModel
                baseColor: Qt.darker(hudDataModel.alphaColor)
                baseCellWidth: root.baseCellWidth
                clanCellWidth: root.clanCellWidth
            }
            ScoreboardTeamPane {
                width: root.tableWidth
                model: scoreboardBetaModel
                baseColor: Qt.darker(hudDataModel.betaColor)
                baseCellWidth: root.baseCellWidth
                clanCellWidth: root.clanCellWidth
            }
        }
    }

    Component {
        id: columnWiseTeamsDisplayComponent

        Column {
            spacing: 32
            ScoreboardTeamPane {
                width: root.tableWidth
                model: scoreboardAlphaModel
                baseColor: Qt.darker(hudDataModel.alphaColor)
                baseCellWidth: root.baseCellWidth
                clanCellWidth: root.clanCellWidth
            }
            ScoreboardTeamPane {
                width: root.tableWidth
                model: scoreboardBetaModel
                displayHeader: false
                baseColor: Qt.darker(hudDataModel.betaColor)
                baseCellWidth: root.baseCellWidth
                clanCellWidth: root.clanCellWidth
            }
        }
    }

    Component {
        id: mixedTeamsDisplayComponent

        ScoreboardTeamPane {
            width: root.tableWidth
            model: scoreboardMixedModel
            mixedTeamsMode: true
            baseAlphaColor: Qt.darker(hudDataModel.alphaColor)
            baseBetaColor: Qt.darker(hudDataModel.betaColor)
            baseCellWidth: root.baseCellWidth
            clanCellWidth: root.clanCellWidth
        }
    }
}