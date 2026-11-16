import QtQuick
import QtQuick.Controls.Basic

Button {
    id: control
    property string tone: "neutral"
    readonly property color accent: tone === "primary" ? "#2d7cdb"
        : tone === "confirm" ? "#38ae77" : tone === "danger" ? "#d8494d" : "#292a30"
    font.pixelSize: 12
    font.weight: Font.Medium
    implicitHeight: 38
    leftPadding: 10
    rightPadding: 10
    contentItem: Label {
        text: control.text
        color: control.enabled ? "#f0f0f2" : "#777981"
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font: control.font
        elide: Text.ElideRight
    }
    background: Rectangle {
        radius: 2
        color: !control.enabled ? "#25262b" : control.down ? Qt.darker(control.accent,1.2)
            : control.hovered ? Qt.lighter(control.accent,1.12) : control.accent
        border.color: control.activeFocus ? "#79a9e8" : control.tone === "neutral" || !control.enabled ? "#393a42" : control.accent
        border.width: 1
    }
}
