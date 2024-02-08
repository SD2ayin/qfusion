import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Controls.Material 2.12
import net.warsow 2.6

Item {
    id: hudField

    property real alphaNameWidth
    property real betaNameWidth

    property var layoutModel
    property var commonDataModel
    property var povDataModel

    property var miniviewAllocator

    // Miniviews can't allocate miniviews
    readonly property bool isMiniview: !miniviewAllocator
    readonly property real scale: isMiniview ? Math.min(hudField.width / rootItem.width, hudField.height / rootItem.height) : 1.0

    property bool arrangementReset
    property int stateIndex

    readonly property string healthIconPath:
        hudField.povDataModel.health > 100 ? "image://wsw/gfx/hud/icons/health/100" :
                                             "image://wsw/gfx/hud/icons/health/50"

    readonly property string armorIconPath:
        hudField.povDataModel.armor >= 125 ? "image://wsw/gfx/hud/icons/armor/ra" :
            (hudField.povDataModel.armor >= 75 ? "image://wsw/gfx/hud/icons/armor/ya" :
                                                 "image://wsw/gfx/hud/icons/armor/ga")

    readonly property color healthColor:
        hudField.povDataModel.health > 100 ? "deeppink" :
            (hudField.povDataModel.health >= 50 ? "white" : "orangered")

    readonly property color armorColor:
        hudField.povDataModel.armor >= 125 ? "red" : (hudField.povDataModel.armor >= 75 ? "gold" : "green")

    readonly property real healthFrac: 0.01 * Math.min(100.0, Math.max(0, hudField.povDataModel.health))
    readonly property real armorFrac: 0.01 * Math.min(100.0, hudField.povDataModel.armor)

    Connections {
        target: layoutModel
        onArrangingItemsEnabled: arrangementReset = false
        onArrangingItemsDisabled: arrangementReset = true
    }

    Repeater {
        id: repeater
        model: hudField.layoutModel

        property int numInstantiatedItems: 0

        delegate: HudLayoutItem {
            id: itemLoader
            width: item ? item.implicitWidth : 0
            height: item ? item.implicitHeight : 0

            readonly property int individualMask: model.individualMask

            states: [
                State {
                    name: "arranged"
                    when: repeater.numInstantiatedItems === repeater.count && !hudField.arrangementReset
                    AnchorChanges {
                        target: itemLoader
                        anchors.top: getQmlAnchor(HudLayoutModel.Top)
                        anchors.bottom: getQmlAnchor(HudLayoutModel.Bottom)
                        anchors.left: getQmlAnchor(HudLayoutModel.Left)
                        anchors.right: getQmlAnchor(HudLayoutModel.Right)
                        anchors.horizontalCenter: getQmlAnchor(HudLayoutModel.HCenter)
                        anchors.verticalCenter: getQmlAnchor(HudLayoutModel.VCenter)
                    }
                },
                // We can't just disable "arranged" state, we have to force transition to the "reset" state with all anchors reset
                State {
                    name: "reset"
                    when: repeater.numInstantiatedItems === repeater.count && hudField.arrangementReset
                    AnchorChanges {
                        target: itemLoader
                        anchors.top: undefined
                        anchors.bottom: undefined
                        anchors.left: undefined
                        anchors.right: undefined
                        anchors.horizontalCenter: undefined
                        anchors.verticalCenter: undefined
                    }
                }
            ]

            Connections {
                target: Hud.ui
                enabled: !hudField.isMiniview
                onHudOccludersChanged: itemLoader.updateItemVisibility()
            }

            Connections {
                target: hudField.commonDataModel
                onHasTwoTeamsChanged: itemLoader.updateItemVisibility()
                onIsInPostmatchStateChanged: itemLoader.updateItemVisibility()
                onActiveItemsMaskChanged: itemLoader.updateItemVisibility()
            }

            Connections {
                target: hudField.povDataModel
                onHasActivePovChanged: itemLoader.updateItemVisibility()
                onIsPovAliveChanged: itemLoader.updateItemVisibility()
            }

            Connections {
                target: item
                onWidthChanged: itemLoader.updateItemVisibility()
                onHeightChanged: itemLoader.updateItemVisibility()
                onXChanged: itemLoader.updateItemVisibility()
                onYChanged: itemLoader.updateItemVisibility()
            }

            onLoaded: itemLoader.updateItemVisibility()
            // onStateChanged gets called too early so it turned to be useless it this regard.
            // Intercepting anchor changes produces desired results.
            anchors.onLeftChanged: itemLoader.updateItemVisibility()
            anchors.onHorizontalCenterChanged: itemLoader.updateItemVisibility()
            anchors.onRightChanged: itemLoader.updateItemVisibility()
            anchors.onTopChanged: itemLoader.updateItemVisibility()
            anchors.onVerticalCenterChanged: itemLoader.updateItemVisibility()
            anchors.onBottomChanged: itemLoader.updateItemVisibility()

            Component.onCompleted: {
                updateItemVisibility()
                repeater.numInstantiatedItems++;
            }

            Component.onDestruction: {
                repeater.numInstantiatedItems--;
                Hud.ui.ensureObjectDestruction(model)
                Hud.ui.ensureObjectDestruction(itemLoader)
            }

            function shouldBeVisibleIfNotOccluded() {
                if (item) {
                    // Hack for items which non-present in the actual layout but are added
                    // to prevent reinstantiation upon file change which leads to leaks
                    if (isHidden) {
                        return false
                    }
                    if (itemLoader.individualMask && !(itemLoader.individualMask & hudField.commonDataModel.activeItemsMask)) {
                        return false
                    }
                    if (!hudField.commonDataModel.hasTwoTeams && (flags & HudLayoutModel.TeamBasedOnly)) {
                        return false
                    }
                    if (!hudField.povDataModel.hasActivePov && (flags & HudLayoutModel.PovOnly)) {
                        return false
                    }
                    if (!hudField.povDataModel.isPovAlive && (flags & HudLayoutModel.AlivePovOnly)) {
                        return false
                    }
                    if (hudField.commonDataModel.isInPostmatchState && !(flags & HudLayoutModel.AllowPostmatch)){
                        return false
                    }
                    return true
                }
                return false
            }

            function updateItemVisibility() {
                if (shouldBeVisibleIfNotOccluded()) {
                    // Put the expensive test last
                    item.visible = !Hud.ui.isHudItemOccluded(item)
                } else {
                    item.visible = false
                }
            }

            sourceComponent: {
                if (kind === HudLayoutModel.HealthBar) {
                    hudField.isMiniview ? miniHealthBarComponent : healthBarComponent
                } else if (kind === HudLayoutModel.ArmorBar) {
                    hudField.isMiniview? miniArmorBarComponent : armorBarComponent
                } else if (kind === HudLayoutModel.InventoryBar) {
                    hudField.isMiniview ? miniInventoryBarComponent : inventoryBarComponent
                } else if (kind === HudLayoutModel.WeaponStatus) {
                    hudField.isMiniview ? miniWeaponStatusComponent : weaponStatusComponent
                } else if (kind === HudLayoutModel.MatchTime) {
                    matchTimeComponent
                } else if (kind === HudLayoutModel.AlphaScore) {
                    alphaScoreComponent
                } else if (kind === HudLayoutModel.BetaScore) {
                    betaScoreComponent
                } else if (kind === HudLayoutModel.Chat) {
                    chatComponent
                } else if (kind === HudLayoutModel.TeamInfo) {
                    teamInfoComponent
                } else if (kind === HudLayoutModel.FragsFeed) {
                    fragsFeedComponent
                } else if (kind === HudLayoutModel.MessageFeed) {
                    messageFeedComponent
                } else if (kind === HudLayoutModel.AwardsArea) {
                    awardsAreaComponent
                } else if (kind === HudLayoutModel.StatusMessage) {
                    statusMessageComponent
                } else if (kind === HudLayoutModel.ObjectiveStatus) {
                    objectiveStatusComponent
                } else if (kind === HudLayoutModel.MiniviewPane1) {
                    miniviewPane1Component
                } else if (kind === HudLayoutModel.MiniviewPane2) {
                    miniviewPane2Component
                } else {
                    undefined
                }
            }

            Component {
                id: healthBarComponent
                HudValueBar {
                    text: "HEALTH"
                    value: hudField.povDataModel.health
                    frac: hudField.healthFrac
                    color: hudField.healthColor
                    iconPath: hudField.healthIconPath
                }
            }

            Component {
                id: armorBarComponent
                HudValueBar {
                    text: "ARMOR"
                    value: hudField.povDataModel.armor
                    frac: hudField.armorFrac
                    color: hudField.armorColor
                    iconPath: hudField.armorIconPath
                }
            }

            Component {
                id: miniHealthBarComponent
                HudMiniValueBar {
                    miniviewScale: hudField.width / rootItem.width
                    value: hudField.povDataModel.health
                    frac: hudField.healthFrac
                    color: hudField.healthColor
                    iconPath: hudField.healthIconPath
                }
            }

            Component {
                id: miniArmorBarComponent
                HudMiniValueBar {
                    miniviewScale: hudField.width / rootItem.width
                    value: hudField.povDataModel.armor
                    frac: hudField.armorFrac
                    color: hudField.armorColor
                    iconPath: hudField.armorIconPath
                }
            }

            Component {
                id: inventoryBarComponent
                HudInventoryBar {
                    povDataModel: hudField.povDataModel
                }
            }

            Component {
                id: miniInventoryBarComponent
                HudMiniInventoryBar {
                    povDataModel: hudField.povDataModel
                    miniviewScale: hudField.width / rootItem.width
                }
            }

            Component {
                id: weaponStatusComponent
                HudWeaponStatus {
                    povDataModel: hudField.povDataModel
                }
            }

            Component {
                id: miniWeaponStatusComponent
                HudMiniWeaponStatus {
                    povDataModel: hudField.povDataModel
                    miniviewScale: hudField.width / rootItem.width
                }
            }

            Component {
                id: matchTimeComponent
                HudMatchTime {
                    commonDataModel: hudField.commonDataModel
                }
            }

            Component {
                id: alphaScoreComponent
                HudTeamScore {
                    visible: commonDataModel.hasTwoTeams
                    leftAligned: true
                    color: hudField.commonDataModel.alphaColor
                    name: hudField.commonDataModel.alphaName
                    clan: hudField.commonDataModel.alphaClan
                    score: hudField.commonDataModel.alphaScore
                    playersStatus: hudField.commonDataModel.alphaPlayersStatus
                    siblingNameWidth: hudField.betaNameWidth
                    onNameWidthChanged: hudField.alphaNameWidth = nameWidth
                }
            }

            Component {
                id: betaScoreComponent
                HudTeamScore {
                    visible: commonDataModel.hasTwoTeams
                    leftAligned: false
                    color: hudField.commonDataModel.betaColor
                    name: hudField.commonDataModel.betaName
                    clan: hudField.commonDataModel.betaClan
                    score: hudField.commonDataModel.betaScore
                    playersStatus: hudField.commonDataModel.betaPlayersStatus
                    siblingNameWidth: hudField.alphaNameWidth
                    onNameWidthChanged: hudField.betaNameWidth = nameWidth
                }
            }

            Component {
                id: chatComponent
                HudChat {}
            }

            Component {
                id: teamInfoComponent
                HudTeamInfo {
                    commonDataModel: hudField.commonDataModel
                    povDataModel: hudField.povDataModel
                }
            }

            Component {
                id: fragsFeedComponent
                HudFragsFeed {
                    commonDataModel: hudField.commonDataModel
                }
            }

            Component {
                id: messageFeedComponent
                HudMessageFeed {
                    povDataModel: hudField.povDataModel
                }
            }

            Component {
                id: awardsAreaComponent
                HudAwardsArea {
                    povDataModel: hudField.povDataModel
                    isMiniview: hudField.isMiniview
                }
            }

            Component {
                id: statusMessageComponent
                HudStatusMessage {
                    povDataModel: hudField.povDataModel
                    isMiniview: hudField.isMiniview
                }
            }

            Component {
                id: objectiveStatusComponent
                HudObjectiveStatus {
                    commonDataModel: hudField.commonDataModel
                }
            }

            Component {
                id: miniviewPane1Component
                HudMiniviewPane {
                    commonDataModel: hudField.commonDataModel
                    miniviewAllocator: hudField.miniviewAllocator
                    paneNumber: 1
                }
            }

            Component {
                id: miniviewPane2Component
                HudMiniviewPane {
                    commonDataModel: hudField.commonDataModel
                    miniviewAllocator: hudField.miniviewAllocator
                    paneNumber: 2
                }
            }

            function getQmlAnchor(anchorBit) {
                const anchorItem = anchorItemIndex > 0 ? repeater.itemAt(anchorItemIndex - 1) : hudField
                return getQmlAnchorOfItem(selfAnchors, anchorItemAnchors, anchorBit, anchorItem)
            }
        }
    }

}