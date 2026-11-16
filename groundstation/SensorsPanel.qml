import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Rectangle {
    id: panel
    property var monitor
    property bool collapsed: true
    readonly property var sensors: monitor ? monitor.sensors : ({})
    implicitHeight: collapsed ? 52 : content.implicitHeight + 16
    color: "#f5202125"
    border.color: "#383940"
    radius: 6
    function row(key) { return sensors[key] || {} }
    function live(key) { return monitor && monitor.fresh && row(key).ok }
    function number(value, digits) { return typeof value === "number" && isFinite(value) ? value.toFixed(digits) : "—" }
    function vector(key, name) {
        const values = row(key).values || {}
        const v = values[name]
        return live(key) && v && v.length === 3 ? v.map(x => number(x, 3)).join("   ") : "—   —   —"
    }
    ColumnLayout {
        id: content
        anchors { left: parent.left; right: parent.right; top: parent.top; margins: 8 }
        spacing: 8
        SectionHeader {
            Layout.fillWidth: true
            title: qsTr("传感器数据")
            collapsed: panel.collapsed
            onClicked: panel.collapsed = !panel.collapsed
        }
        ColumnLayout {
            visible: !panel.collapsed
            Layout.fillWidth: true
            spacing: 8
            Repeater {
                model: [{key:"fc_imu", title:"飞控 IMU", unit:"m/s²"}, {key:"imu", title:"雷达 IMU", unit:"驱动原值"}]
                delegate: ColumnLayout {
                    required property var modelData
                    Layout.fillWidth: true
                    spacing: 3
                    Label {
                        Layout.fillWidth: true
                        text: modelData.title + " · " + (panel.live(modelData.key) ? panel.number(panel.row(modelData.key).hz, 1) + " Hz" : "未采样 / 过期")
                        color: panel.live(modelData.key) ? "#3cbb83" : "#93959f"
                    }
                    Label { text: "加速度 XYZ (" + modelData.unit + ")"; color:"#93959f"; font.pixelSize:11 }
                    Label { text: panel.vector(modelData.key,"acceleration"); font.family:"monospace"; color:"#dadbe0"; font.pixelSize:12 }
                    Label { text: "角速度 XYZ (rad/s)"; color:"#93959f"; font.pixelSize:11 }
                    Label { text: panel.vector(modelData.key,"angular_velocity"); font.family:"monospace"; color:"#dadbe0"; font.pixelSize:12 }
                }
            }
            Label {
                Layout.fillWidth: true
                text: "激光雷达 · " + (panel.live("lidar") ? panel.number(panel.row("lidar").hz,1) + " Hz  ·  " + panel.row("lidar").values.point_count + " 点/帧" : "未采样 / 过期")
                color: panel.live("lidar") ? "#3cbb83" : "#93959f"
                wrapMode: Text.Wrap
            }
            Label {
                text: "气压  " + (panel.live("pressure") ? panel.number(panel.row("pressure").values.pressure_pa / 100,2) + " hPa" : "—")
                color:"#dadbe0"
            }

        }
    }
}
