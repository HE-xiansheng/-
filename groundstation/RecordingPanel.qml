import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Rectangle {
    id:panel
    property bool collapsed: true
    signal readRecordsRequested()
    required property var recorder
    readonly property real elapsed:recorder.elapsedSeconds
    implicitHeight:content.implicitHeight+24
    radius:6;color:"#202125";border.color:"#383940"
    ColumnLayout {
        id:content;anchors.fill:parent;anchors.margins:12;spacing:8
        SectionHeader {
            Layout.fillWidth: true
            title: qsTr("飞行数据录制")
            collapsed: panel.collapsed
            onClicked: panel.collapsed = !panel.collapsed
        }
        ColumnLayout {
            Layout.fillWidth: true
            visible: !panel.collapsed
            spacing: 8

        CheckBox {
            objectName:"recordingAuto";text:qsTr("自动录制（OFFBOARD → 落地锁桨）")
            checked:panel.recorder.autoEnabled;enabled:panel.recorder.fresh&&!panel.recorder.busy
            onClicked:panel.recorder.setAuto(checked)
        }
        Label {Layout.fillWidth:true;visible:text.length>0;text:panel.recorder.autoText;color:panel.recorder.autoReady ? "#3cbb83" : "#dbad48";font.pixelSize:11;wrapMode:Text.Wrap}
        RowLayout {
            Layout.fillWidth:true
            StudioButton {objectName:"recordingStart";Layout.fillWidth:true;text:qsTr("开始录制");enabled:panel.recorder.canStart;onClicked:panel.recorder.start()}
            StudioButton {objectName:"recordingStop";Layout.fillWidth:true;text:qsTr("停止录制");enabled:panel.recorder.canStop;onClicked:panel.recorder.stop()}
        }
        Label {
            Layout.fillWidth:true;text:panel.recorder.phaseText
            color:panel.recorder.recording ? "#e05257" : panel.recorder.fresh ? "#b8bac4" : "#dbad48"
            font.pixelSize:12;wrapMode:Text.Wrap;textFormat:Text.PlainText
        }
        RowLayout {
            Label {text:qsTr("录制时长");color:"#a1a2ad"}
            Label {
                Layout.fillWidth:true;horizontalAlignment:Text.AlignRight
                text:panel.formatTime(panel.elapsed);font.pixelSize:18;font.family:"monospace"
                color:panel.recorder.recording ? "#e05257" : "#d2d3da"
            }
        }
        Label {
            Layout.fillWidth:true;text:qsTr("已录 %1 MB · 保存于机载 flight-records").arg(panel.recorder.bagMegabytes.toFixed(1))
            color:"#93959f";font.pixelSize:11;wrapMode:Text.Wrap
            ToolTip.visible:pathHover.hovered;ToolTip.text:panel.recorder.outputDirectory
            HoverHandler {id:pathHover}
        }
        StudioButton {
            objectName:"readOnboardFlightRecords"
            Layout.fillWidth:true;text:qsTr("读取机载飞行数据")
            onClicked:panel.readRecordsRequested()
        }
        Label {
            Layout.fillWidth:true;visible:text.length>0;text:panel.recorder.message
            color:"#a1a2ad";font.pixelSize:11;wrapMode:Text.Wrap;textFormat:Text.PlainText
        }
        }
    }
    function formatTime(seconds) {
        const total=Math.max(0,Math.floor(seconds))
        const h=Math.floor(total/3600),m=Math.floor(total/60)%60,s=total%60
        return (h<10 ? "0" : "")+h+":"+(m<10 ? "0" : "")+m+":"+(s<10 ? "0" : "")+s
    }
}
