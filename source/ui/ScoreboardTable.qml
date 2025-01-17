import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Controls.Material 2.12
import QtQuick.Layouts 1.12
import net.warsow 2.6

TableView {
    id: tableView
    columnSpacing: 0
    rowSpacing: 0
    reuseItems: false
    interactive: false
    contentHeight: rowHeight * rows
    implicitHeight: rowHeight * rows

    property color baseColor

    property color baseAlphaColor: "red"
    property color baseBetaColor: "green"

    property bool mixedTeamsMode: false

    property real baseCellWidth
    property real clanCellWidth

    readonly property real rowHeight: 36

    function getCellColor(row, column, tableStyle) {
        let c = baseColor
        if (mixedTeamsMode) {
            c = scoreboard.isMixedListRowAlpha(row) ? baseAlphaColor : baseBetaColor
        }
        switch (tableStyle) {
            case Scoreboard.Checkerboard: {
                if (row % 2) {
                    return column % 2 ? Qt.darker(c, 1.1) : c
                }
                return column % 2 ? c : Qt.lighter(c, 1.2)
            }
            case Scoreboard.RowStripes: {
                return row % 2 ? Qt.darker(c, 1.1) : Qt.lighter(c, 1.1)
            }
            case Scoreboard.ColumnStripes: {
                return column % 2 ? Qt.darker(c, 1.1) : Qt.lighter(c, 1.1)
            }
            case Scoreboard.Flat: {
                return c
            }
        }
    }

    delegate: Item {
        readonly property int kind: scoreboard.getColumnKind(column)
        readonly property bool isColumnTextual: (kind === Scoreboard.Nickname) || (kind === Scoreboard.Clan)
        readonly property bool isColumnStatusOne: kind === Scoreboard.Status
        readonly property bool isDisplayingGlyph: (kind === Scoreboard.Glyph) || (isColumnStatusOne && value >= 32)
        readonly property bool shouldBeDisplayedAsIcon: (kind === Scoreboard.Icon) || (isColumnStatusOne && value < 32)
        readonly property real valueOpacity: isGhosting ? 0.5 : 1.0

        // Table width can be zero while loading via Loader
        implicitWidth: kind === Scoreboard.Nickname ?
                       (tableView.width ?
                           tableView.width - clanCellWidth - (tableView.columns - 2) * baseCellWidth :
                           baseCellWidth) :
                       (kind === Scoreboard.Clan ? clanCellWidth : baseCellWidth)

        implicitHeight: rowHeight
        onImplicitHeightChanged: forceLayoutTimer.start()
        onHeightChanged: forceLayoutTimer.start()

        Rectangle {
            anchors.fill: parent
            visible: !isColumnStatusOne
            opacity: 0.7
            color: isColumnStatusOne ? "transparent" : getCellColor(row, column, scoreboard.tableStyle)
        }

        Label {
            visible: !shouldBeDisplayedAsIcon
            opacity: isColumnStatusOne ? 1.0 : valueOpacity
            anchors.fill: parent
            verticalAlignment: Qt.AlignVCenter
            horizontalAlignment: isColumnTextual ? Qt.AlignLeft : Qt.AlignHCenter
            padding: 4
            text: value
            textFormat: Text.StyledText
            font.family: ((kind !== Scoreboard.Glyph && kind !== Scoreboard.Status) || value < 256) ?
                wsw.regularFontFamily : wsw.symbolsFontFamily
            font.weight: Font.Bold
            font.pointSize: 12
            font.letterSpacing: 1
            font.strikeout: isGhosting && isColumnTextual
            style: Text.Raised
        }

        Loader {
            active: value && shouldBeDisplayedAsIcon
            anchors.centerIn: parent
            width: 20
            height: 20

            sourceComponent: Image {
                opacity: valueOpacity
                mipmap: true
                width: 20
                height: 20
                source: scoreboard.getImageAssetPath(value)
            }
        }
    }

    onColumnsChanged: forceLayoutTimer.start()
    onRowsChanged: forceLayoutTimer.start()

    Timer {
        id: forceLayoutTimer
        interval: 1
        onTriggered: tableView.forceLayout()
    }
}