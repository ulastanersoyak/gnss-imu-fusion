import QtQuick 2.9
import QtQuick.Layouts 1.3

Rectangle {
  anchors.fill: parent
  implicitWidth: overlayText.implicitWidth + 24
  implicitHeight: overlayText.implicitHeight + 20
  Layout.minimumWidth: implicitWidth
  Layout.minimumHeight: implicitHeight
  color: "#d0202428"
  radius: 6

  Text {
    id: overlayText
    anchors.left: parent.left
    anchors.top: parent.top
    anchors.margins: 12
    text: AccuracyOverlay.text
    color: "#f0f0f0"
    font.family: "monospace"
    font.pixelSize: 14
    textFormat: Text.StyledText
  }
}
