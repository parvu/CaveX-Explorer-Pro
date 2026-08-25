import QtQuick 2.9

Rectangle {
  id: infoLabelCard
  // Small, transparent, borderless -- deliberately not a title-barred
  // gz-gui "card" chrome, just the constant's text in a soft frame.
  width: labelText.implicitWidth + 24
  height: labelText.implicitHeight + 16
  color: "#66000000"
  radius: 6

  Text {
    id: labelText
    anchors.centerIn: parent
    text: InfoLabel.text
    font.pointSize: 9
    color: "#f0f0f0"
  }
}
