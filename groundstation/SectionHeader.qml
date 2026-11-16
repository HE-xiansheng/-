import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

ToolButton {
    id: control
    objectName: "sectionToggle"
    property string title: ""
    property bool collapsed: false
    implicitHeight: 36
    padding: 0
    contentItem: RowLayout {
        spacing: 6
        Label {
            Layout.preferredWidth: 14
            text: control.collapsed ? "▸" : "▾"
            color: "#b8bac4"
            font.pixelSize: 13
            horizontalAlignment: Text.AlignHCenter
        }
        Label {
            Layout.fillWidth: true
            text: control.title
            color: "#dadbe0"
            font.pixelSize: 13
            font.weight: Font.DemiBold
            elide: Text.ElideRight
        }
    }
    background: Rectangle {
        radius: 4
        color: control.down ? "#35363e" : control.hovered ? "#303138" : "#292a30"
    }
}
