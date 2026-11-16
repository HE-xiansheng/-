import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Rectangle {
    id: panel
    required property var monitor
    property bool collapsed: false
    readonly property var fallback: [
        {label:"MID360点云"}, {label:"雷达IMU"}, {label:"FAST-LIO定位"},
        {label:"规划点云"}, {label:"外部Pose转发"}, {label:"飞控里程计"},
        {label:"飞控连接/模式"}, {label:"着地状态"}, {label:"EKF估计"},
        {label:"遥控输入"}, {label:"电池"}
    ]
    height: collapsed ? 52 : 334
    radius: 6
    color: "#f5202125"
    border.color: "#383940"
    clip: true

    // 面板区域的点击/滚轮不会穿透为地图选点。
    MouseArea {
        anchors.fill: parent
        acceptedButtons: Qt.AllButtons
        onWheel: function(wheel) { wheel.accepted = true }
    }

    Column {
        anchors.fill: parent
        anchors.margins: 8
        spacing: 4
        SectionHeader {
            width:parent.width
            title:qsTr("ROS话题栏")
            collapsed:panel.collapsed
            onClicked:panel.collapsed=!panel.collapsed
            ToolTip.visible:headerHover.hovered
            ToolTip.text:panel.monitor.status+"\n"+panel.monitor.sourceHost
            HoverHandler {id:headerHover}
        }

        Flickable {
            id: table
            visible: !panel.collapsed
            width: parent.width
            height: 242
            contentWidth: Math.max(width, 540)
            contentHeight: 242
            boundsBehavior: Flickable.StopAtBounds
            clip: true
            ScrollBar.horizontal: ScrollBar { policy: table.contentWidth > table.width ? ScrollBar.AsNeeded : ScrollBar.AlwaysOff }

            Column {
                width: table.contentWidth
                Row {
                    height: 22
                    spacing: 4
                    Repeater {
                        model: [{t:"ROS话题 / 数据",w:120}, {t:"采样状态",w:90}, {t:"Hz",w:42}, {t:"龄期(s)",w:58}, {t:"数值 / 来源",w:table.contentWidth-326}]
                        delegate: Label {
                            required property var modelData
                            width: modelData.w
                            height: 22
                            text: modelData.t
                            font.pixelSize: 11
                            font.weight: Font.DemiBold
                            color: "#b8bac4"
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                }
                Repeater {
                    model: panel.monitor.rows.length ? panel.monitor.rows : panel.fallback
                    delegate: Item {
                        id: topicRow
                        required property var modelData
                        required property int index
                        readonly property color rowColor: !panel.monitor.fresh ? "#93959f" : modelData.ok ? "#3cbb83" : "#e27073"
                        width: table.contentWidth
                        height: 20
                        Rectangle { anchors.fill: parent; color: rowMouse.containsMouse ? "#35363e" : index % 2 ? "#25262b" : "transparent" }
                        Row {
                            anchors.fill: parent
                            spacing: 4
                            Label { width:120; height:20; text:topicRow.modelData.label; color:"#d2d3da"; font.pixelSize:11; verticalAlignment:Text.AlignVCenter; textFormat:Text.PlainText }
                            Label { width:90; height:20; text:panel.monitor.fresh ? topicRow.modelData.state : panel.monitor.rows.length ? "监视过期" : "未读取"; color:topicRow.rowColor; font.pixelSize:11; verticalAlignment:Text.AlignVCenter; elide:Text.ElideRight; textFormat:Text.PlainText }
                            Label { font.family:"monospace"; width:42; height:20; text:panel.monitor.fresh ? topicRow.modelData.hz : "—"; color:topicRow.rowColor; font.pixelSize:11; verticalAlignment:Text.AlignVCenter; textFormat:Text.PlainText }
                            Label { font.family:"monospace"; width:58; height:20; text:panel.monitor.fresh ? topicRow.modelData.age : "—"; color:topicRow.rowColor; font.pixelSize:11; verticalAlignment:Text.AlignVCenter; textFormat:Text.PlainText }
                            Label { width:table.contentWidth-326; height:20; text:panel.monitor.fresh ? topicRow.modelData.detail+"  "+topicRow.modelData.publishers : "—"; color:topicRow.rowColor; font.pixelSize:11; verticalAlignment:Text.AlignVCenter; elide:Text.ElideRight; textFormat:Text.PlainText }
                        }
                        MouseArea {
                            id: rowMouse
                            anchors.fill: parent
                            hoverEnabled: true
                            onClicked: panel.monitor.selectRow(topicRow.index)
                        }
                        ToolTip.visible: rowMouse.containsMouse && !!topicRow.modelData.topic
                        ToolTip.delay: 450
                        ToolTip.text: (topicRow.modelData.topic || "尚未采样")+"\n"+(topicRow.modelData.publishers || "无发布者")+"\n"+(topicRow.modelData.detail || "")
                    }
                }
            }
        }
        Label {
            visible: !panel.collapsed
            width: parent.width
            height: 32
            text: panel.monitor.fresh ? panel.monitor.detail : panel.monitor.status
            color: "#93959f"
            font.pixelSize: 10
            wrapMode: Text.WrapAnywhere
            maximumLineCount: 2
            elide: Text.ElideRight
            textFormat: Text.PlainText
        }

    }
}
