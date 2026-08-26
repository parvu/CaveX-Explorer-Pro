import QtQuick 2.9

Rectangle {
  id: panel
  width: ManualControl.mode === "turn" ? 100 : 150
  height: 96
  color: "#66000000"
  radius: 6

  property bool manualOn: false

  Component {
    id: ctrlButton
    Rectangle {
      property alias label: t.text
      property var onPress: function() {}
      width: 40
      height: 26
      radius: 4
      color: area.pressed ? "#aa4488ff" : "#553a3a3a"
      border.color: "#f0f0f0"
      border.width: 1
      Text {
        id: t
        anchors.centerIn: parent
        font.pointSize: 8
        color: "#f0f0f0"
      }
      MouseArea {
        id: area
        anchors.fill: parent
        onClicked: onPress()
      }
    }
  }

  // D-pad: left/fwd/right/rev around a center stop. Unchanged from the
  // ROV version -- left/right already read naturally as "turn" for a
  // tracked vehicle with no strafe capability, no relabeling needed.
  Grid {
    visible: ManualControl.mode !== "turn"
    anchors.centerIn: parent
    columns: 3
    rows: 3
    spacing: 2
    Item { width: 40; height: 26 }
    Loader { sourceComponent: ctrlButton; onLoaded: { item.label = "fwd"; item.onPress = function() { ManualControl.SendCommand("forward") } } }
    Item { width: 40; height: 26 }
    Loader { sourceComponent: ctrlButton; onLoaded: { item.label = "left"; item.onPress = function() { ManualControl.SendCommand("left") } } }
    Loader { sourceComponent: ctrlButton; onLoaded: { item.label = "stop"; item.onPress = function() { ManualControl.SendCommand("stop") } } }
    Loader { sourceComponent: ctrlButton; onLoaded: { item.label = "right"; item.onPress = function() { ManualControl.SendCommand("right") } } }
    Item { width: 40; height: 26 }
    Loader { sourceComponent: ctrlButton; onLoaded: { item.label = "rev"; item.onPress = function() { ManualControl.SendCommand("backward") } } }
    Item { width: 40; height: 26 }
  }

  // Turn (left/right pivot) + Manual toggle -- replaces the ROV
  // version's depth (up/down) pair, which has no ground-vehicle
  // equivalent (real request: "replace up/down with turn left/turn
  // right").
  Column {
    visible: ManualControl.mode === "turn"
    anchors.centerIn: parent
    spacing: 4

    Loader { sourceComponent: ctrlButton; onLoaded: { item.label = "turn L"; item.onPress = function() { ManualControl.SendCommand("turn_left") } } }
    Loader { sourceComponent: ctrlButton; onLoaded: { item.label = "turn R"; item.onPress = function() { ManualControl.SendCommand("turn_right") } } }

    Rectangle {
      width: 40
      height: 24
      radius: 4
      color: panel.manualOn ? "#aa22aa44" : "#553a3a3a"
      border.color: "#f0f0f0"
      border.width: 1
      Text {
        anchors.centerIn: parent
        text: panel.manualOn ? "ON" : "man"
        font.pointSize: 8
        color: "#f0f0f0"
      }
      MouseArea {
        anchors.fill: parent
        onClicked: {
          panel.manualOn = !panel.manualOn
          ManualControl.SendCommand(panel.manualOn ? "manual_on" : "manual_off")
        }
      }
    }
  }
}
