import QtQuick 2.9

Rectangle {
  id: actionCard
  width: 150
  height: 34
  color: "#66000000"
  radius: 6

  Component {
    id: ctrlButton
    Rectangle {
      property alias label: t.text
      property var onPress: function() {}
      width: 48
      height: 22
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
    anchors.centerIn: parent
    spacing: 6

    Text {
      text: ActionButtons.label
      font.pointSize: 8
      color: "#f0f0f0"
      anchors.verticalCenter: parent.verticalCenter
    }

    Loader {
      sourceComponent: ctrlButton
      onLoaded: {
        item.label = ActionButtons.button1Label
        item.onPress = function() { ActionButtons.SendButton1() }
      }
    }
    Loader {
      sourceComponent: ctrlButton
      onLoaded: {
        item.label = ActionButtons.button2Label
        item.onPress = function() { ActionButtons.SendButton2() }
      }
    }
  }
}
