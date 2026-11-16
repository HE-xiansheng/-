import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Rectangle {
    id: panel
    property bool collapsed: false
    property var taskController: null
    function syncTaskVideo() {
        if (!taskController || !taskController.fresh) return
        const state = taskController.state
        const running = !!state.running
        videoController.setGimbalAruco(running && state.task === "aruco")
        videoController.setTrackingMode(running && state.task === "tracking")
    }
    onTaskControllerChanged: syncTaskVideo()
    Connections { target: panel.taskController; function onChanged() { panel.syncTaskVideo() } }
    implicitHeight: sectionHeader.implicitHeight + 16
    color: "#f5202125"
    border.color: "#383940"
    radius: 6
    clip: true
    MouseArea { anchors.fill: parent; acceptedButtons: Qt.AllButtons; onWheel: function(wheel) { wheel.accepted = true } }
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8
        spacing: 5
        SectionHeader {
            id: sectionHeader
            Layout.fillWidth: true
            title: qsTr("视频流拉取")
            collapsed: panel.collapsed
            onClicked: panel.collapsed = !panel.collapsed
        }
        // 收起仅隐藏显示内容，保留现有视频连接与任务画面状态。
        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            visible: !panel.collapsed
            spacing: 5
            RowLayout {
                Layout.fillWidth: true
                Item { Layout.fillWidth: true }
                StudioButton { objectName: "toggleStereoVideo"; tone: videoController.stereoEnabled ? "confirm" : "neutral"; text: videoController.stereoEnabled ? qsTr("关闭双目") : qsTr("打开双目"); onClicked: videoController.setStreamEnabled(0, !videoController.stereoEnabled) }
                StudioButton { objectName: "toggleGimbalVideo"; tone: videoController.gimbalEnabled ? "confirm" : "neutral"; text: videoController.gimbalEnabled ? qsTr("关闭云台") : qsTr("打开云台"); onClicked: videoController.setStreamEnabled(1, !videoController.gimbalEnabled) }
            }
            Label { visible: videoController.gimbalAruco || videoController.trackingMode; text: videoController.gimbalAruco ? qsTr("已切换到 ArUco 识别画面") : qsTr("已切换到人物跟踪识别画面"); color: "#93959f"; font.pixelSize: 11 }
            Label { Layout.fillWidth: true; text: videoController.status; color: "#93959f"; elide: Text.ElideRight; font.pixelSize: 11 }
            Repeater {
                model: 2
                delegate: Rectangle {
                    required property int index
                    readonly property string videoName: index === 0 ? (videoController.trackingMode ? qsTr("双目 · 人物跟踪") : (videoController.personMode ? qsTr("人物识别") : qsTr("双目"))) : (videoController.gimbalAruco ? qsTr("云台 · ArUco 识别") : qsTr("云台"))
                    readonly property bool streamEnabled: index === 0 ? videoController.stereoEnabled : videoController.gimbalEnabled
                    readonly property bool fresh: index === 0 ? videoController.stereoFresh : videoController.gimbalFresh
                    readonly property bool switchFrame: index === 0 && videoController.stereoSwitchFrameVisible
                    readonly property int revision: index === 0 ? videoController.stereoRevision : videoController.gimbalRevision
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.minimumHeight: 50
                    color: "#101013"
                    border.color: "#383940"
                    Label { z: 2; anchors.left: parent.left; anchors.top: parent.top; anchors.margins: 6; text: parent.switchFrame ? (videoController.stereoImagePersonMode ? qsTr("人物识别 · 上一帧") : qsTr("双目原画 · 上一帧")) : parent.videoName; color: parent.fresh ? "#3cbb83" : "#dbad48"; font.pixelSize: 12 }
                    Image { anchors.fill: parent; anchors.margins: 3; visible: parent.fresh || parent.switchFrame; fillMode: Image.PreserveAspectFit; cache: false; source: (parent.fresh || parent.switchFrame) ? "image://livevideo/" + (index === 0 ? "stereo/" : "gimbal/") + parent.revision : "" }
                    Label { z: 2; anchors.centerIn: parent; visible: parent.switchFrame; text: qsTr("切换中 · 画面暂停"); color: "#dbad48"; padding: 6; background: Rectangle { color: "#d9000000"; radius: 4 } }
                    Label { anchors.centerIn: parent; visible: !parent.fresh && !parent.switchFrame; text: !parent.streamEnabled ? qsTr("未开启") : (index === 0 && videoController.trackingMode ? qsTr("等待人物跟踪识别画面") : (index === 0 && videoController.personMode ? qsTr("等待人物识别画面") : (index === 1 && videoController.gimbalAruco ? qsTr("等待 ArUco 识别画面") : qsTr("等待视频帧 / 来源不可用")))); color: "#93959f" }
                }
            }
        }
    }
}
