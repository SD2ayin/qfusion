import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Controls.Material 2.12
import QtQuick.Controls.Material.impl 2.12
import QtQuick.Layouts 1.12
import net.warsow 2.6

Item {
	id: root
	height: 40

    property bool highlighted: false
	property string text
	property bool leaningRight: false
	property real expansionFrac: 0.0
	property bool enableSlidingOvershoot: true

	signal clicked()

	function toggleExpandedState() {
		if (state == "centered") {
			state = leaningRight ? "pinnedToLeft" : "pinnedToRight"
		} else {
			state = "centered"
		}
	}

	readonly property var transformMatrix: UI.ui.makeSkewXMatrix(root.height, 20.0)

	readonly property color foregroundColor: Qt.lighter(Material.backgroundColor, 1.5)
	readonly property color trailDecayColor: UI.ui.colorWithAlpha(Material.backgroundColor, 0.0)
	readonly property color highlightedColor: Material.accentColor

	property real bodyWidth: mouseArea.containsMouse ? UI.ui.mainMenuButtonWidthDp + 12 : UI.ui.mainMenuButtonWidthDp
	property real bodyHeight: mouseArea.containsMouse ? root.height + 2 : root.height
	property color bodyColor: highlighted || mouseArea.containsMouse ? highlightedColor : foregroundColor

	Behavior on bodyWidth { SmoothedAnimation { duration: 333 } }
	Behavior on bodyHeight { SmoothedAnimation { duration: 333 } }
	Behavior on bodyColor { ColorAnimation { duration: 50 } }

    readonly property real baseTrailElementWidth: 20
    readonly property real trailSpacing: 4
    property real trailElementWidth: mouseArea.containsMouse ? baseTrailElementWidth + 1 : baseTrailElementWidth
	readonly property int trailElementsCount: Math.floor(UI.ui.mainMenuButtonTrailWidthDp / (baseTrailElementWidth + root.trailSpacing))

    Behavior on trailElementWidth {
        NumberAnimation {
            duration: 500
            easing.type: Easing.OutElastic
            easing.amplitude: 2.0
        }
    }

	states: [
		State {
			name: "centered"
			AnchorChanges {
				target: body
				anchors.horizontalCenter: root.horizontalCenter
				anchors.verticalCenter: root.verticalCenter
				anchors.left: undefined
				anchors.right: undefined
			}
		},
		State {
			name: "pinnedToLeft"
			AnchorChanges {
				target: body
				anchors.horizontalCenter: undefined
				anchors.verticalCenter: root.verticalCenter
				anchors.left: root.left
				anchors.right: undefined
			}
		},
		State {
			name: "pinnedToRight"
			AnchorChanges {
				target: body
				anchors.horizontalCenter: undefined
				anchors.verticalCenter: root.verticalCenter
				anchors.left: undefined
				anchors.right: root.right
			}
		}
	]

	transitions: [
	    Transition {
	        to: "centered"
		    AnchorAnimation {
			    duration: 333
			    easing.type: Easing.OutBack
		    }
	    },
	    Transition {
	        from: "centered"
	        AnchorAnimation {
	            duration: 333
	            easing.type: enableSlidingOvershoot ? Easing.OutBack : Easing.OutCubic
	        }
	    }
	]

	state: "centered"

	Loader {
		active: !leaningRight
		anchors.right: body.left
		anchors.rightMargin: trailSpacing
	    anchors.verticalCenter: body.verticalCenter
		sourceComponent: leftTrailComponent
	}

	Component {
		id: leftTrailComponent
		MainMenuButtonTrail {
            spacing: root.trailSpacing
            elementsCount: root.trailElementsCount
            elementWidth: trailElementWidth
            elementHeight: root.bodyHeight
			leftColor: root.bodyColor
			rightColor: root.trailDecayColor
			transformMatrix: root.transformMatrix
		}
	}

    // The trail does not have a shadow but this is sufficent
    Item {
        z: -1
        id: shadowCaster
        anchors.centerIn: body
        opacity: 0.5
        Item {
            anchors.centerIn: parent
            width: body.width
            height: body.height
		    transform: Matrix4x4 { matrix: root.transformMatrix }
            layer.enabled: true
            layer.effect: ElevationEffect { elevation: 16 }
        }
    }

	Rectangle {
		id: body
		width: bodyWidth
		height: bodyHeight
		radius: 3
		color: bodyColor

		transform: Matrix4x4 { matrix: root.transformMatrix }

		onXChanged: {
			const halfContainerWidth = parent.width / 2
			const halfThisWidth = width / 2
			const slidingDistance = halfContainerWidth - halfThisWidth
			let frac = 0.0
			if (root.leaningRight) {
				frac = Math.abs(x - halfContainerWidth + halfThisWidth) / slidingDistance
			} else {
				frac = Math.abs(x - halfContainerWidth + halfThisWidth) / slidingDistance
			}
			root.expansionFrac = Math.min(1.0, Math.max(frac, 0.0))
		}

		Label {
			anchors.left: root.leaningRight ? parent.left: undefined
			anchors.right: root.leaningRight ? undefined: parent.right
			anchors.verticalCenter: parent.verticalCenter
			anchors.leftMargin: 12
			anchors.rightMargin: 12
			font.family: UI.ui.headingFontFamily
			font.pointSize: 12
			font.letterSpacing: mouseArea.containsMouse ? 1.75 : 1.25
			text: root.text
			font.weight: Font.ExtraBold
			font.capitalization: Font.AllUppercase
			Behavior on font.letterSpacing { SmoothedAnimation { duration: 333 } }
		}

		MouseArea {
			id: mouseArea
			anchors.fill: parent
			hoverEnabled: true
			onClicked: {
			    let frac = root.expansionFrac
			    // Suppress clicked() signal in an immediate state
			    if (Math.abs(frac) < 0.001 || Math.abs(frac - 1.0) < 0.001) {
			        root.clicked()
			    }
			}
		}
	}

	Loader {
		active: leaningRight
		anchors.left: body.right
		anchors.leftMargin: trailSpacing
		anchors.verticalCenter: body.verticalCenter
		sourceComponent: rightTrailComponent
	}

	Component {
		id: rightTrailComponent
		MainMenuButtonTrail {
			spacing: root.trailSpacing
			elementsCount: root.trailElementsCount
			elementWidth: trailElementWidth
			elementHeight: bodyHeight
			leftColor: root.trailDecayColor
			rightColor: root.bodyColor
			transformMatrix: root.transformMatrix
		}
	}
}
