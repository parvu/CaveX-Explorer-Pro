import QtQuick 2.9

Rectangle {
  id: panel
  width: 210
  height: 130
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

  Row {
    anchors.top: parent.top
    anchors.horizontalCenter: parent.horizontalCenter
    anchors.topMargin: 6
    spacing: 14

    // D-pad: left/top/right/down around a center stop.
    Grid {
      columns: 3
      rows: 3
      spacing: 2
      Item { width: 40; height: 26 }
      Loader { sourceComponent: ctrlButton; onLoaded: { item.label = "top"; item.onPress = function() { ManualControl.SendCommand("forward") } } }
      Item { width: 40; height: 26 }
      Loader { sourceComponent: ctrlButton; onLoaded: { item.label = "left"; item.onPress = function() { ManualControl.SendCommand("left") } } }
      Loader { sourceComponent: ctrlButton; onLoaded: { item.label = "stop"; item.onPress = function() { ManualControl.SendCommand("stop") } } }
      Loader { sourceComponent: ctrlButton; onLoaded: { item.label = "right"; item.onPress = function() { ManualControl.SendCommand("right") } } }
      Item { width: 40; height: 26 }
      Loader { sourceComponent: ctrlButton; onLoaded: { item.label = "down"; item.onPress = function() { ManualControl.SendCommand("backward") } } }
      Item { width: 40; height: 26 }
    }

    // Depth: separate up/down pair.
    Column {
      spacing: 2
      anchors.verticalCenter: parent.verticalCenter
      Loader { sourceComponent: ctrlButton; onLoaded: { item.label = "up"; item.onPress = function() { ManualControl.SendCommand("up") } } }
      Loader { sourceComponent: ctrlButton; onLoaded: { item.label = "down"; item.onPress = function() { ManualControl.SendCommand("down") } } }
    }
  }

  Rectangle {
    id: manualToggle
    anchors.bottom: parent.bottom
    anchors.horizontalCenter: parent.horizontalCenter
    anchors.bottomMargin: 6
    width: 90
    height: 24
    radius: 4
    color: panel.manualOn ? "#aa22aa44" : "#553a3a3a"
    border.color: "#f0f0f0"
    border.width: 1
    Text {
      anchors.centerIn: parent
      text: panel.manualOn ? "Manual: ON" : "Manual"
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
