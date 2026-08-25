import QtQuick 2.9

Rectangle {
  id: infoLabelCard
  width: labelText.implicitWidth + input.implicitWidth + unitText.implicitWidth + 32
  height: 30
  color: "#66000000"
  radius: 6

  // Keeps the input field showing the last value received from the
  // publisher until the user actually starts typing -- otherwise a
  // publisher update while the user is mid-edit would overwrite their
  // keystrokes.
  Connections {
    target: InfoLabel
    function onTextChanged() {
      if (!input.activeFocus) {
        input.text = InfoLabel.text
      }
    }
  }
  Component.onCompleted: input.text = InfoLabel.text

  Row {
    anchors.centerIn: parent
    spacing: 4

    Text {
      id: labelText
      text: InfoLabel.label
      font.pointSize: 8
      color: "#f0f0f0"
      anchors.verticalCenter: parent.verticalCenter
    }

    Rectangle {
      width: Math.max(input.implicitWidth + 8, 36)
      height: 20
      radius: 3
      color: "#33ffffff"
      border.color: input.activeFocus ? "#4488ff" : "#f0f0f0"
      border.width: 1
      anchors.verticalCenter: parent.verticalCenter

      TextInput {
        id: input
        anchors.fill: parent
        anchors.margins: 3
        font.pointSize: 8
        color: "#f0f0f0"
        selectionColor: "#4488ff"
        validator: DoubleValidator {}
        onAccepted: {
          InfoLabel.SendValue(input.text)
          focus = false
        }
      }
    }

    Text {
      id: unitText
      text: InfoLabel.unit
      font.pointSize: 8
      color: "#f0f0f0"
      anchors.verticalCenter: parent.verticalCenter
    }
  }
}
