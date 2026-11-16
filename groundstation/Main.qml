// Copyright (C) 2026
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Dialogs
import QtQuick.Layouts
import QtQuick3D
import QtQuick3D.Helpers
import CustomGeometryExample

ApplicationWindow {
    id: window
    width: 1320
    height: 780
    visible: true
    title: qsTr("科研飞行平台-地面站")
    color: "#111114"
    palette.windowText: "#dadbe0"
    palette.text: "#dadbe0"
    palette.buttonText: "#eeeef1"
    palette.base: "#202125"
    palette.button: "#292a30"
    palette.highlight: "#2d7cdb"
    palette.highlightedText: "#ffffff"
    palette.placeholderText: "#777983"

    readonly property bool plannerMode: (startupPreflightActive || onboardConnection.authenticated) && !mobileUi
    property var planning: ({})
    property string pathSession: ""
    property var actualHistory: []
    readonly property bool plannedPathVisible: plannerMode && !plannerController.previewSuppressed && flightMonitor.fresh && !!planning.path_valid
    readonly property bool candidatePathVisible: plannerMode && !plannerController.previewSuppressed && flightMonitor.fresh && !!planning.candidate_path_valid
    readonly property var plannerState: planning.status || ({})
    readonly property bool mobileUi: Qt.platform.os === "android"
    readonly property bool mobileLandscape: mobileUi && width > height
    readonly property real panelWidth: 390
    readonly property real fixedSceneScale: 45
    // 点云点、选点容差、航点手柄分别控制：点云保持细腻，航点只作为可拖动的交互标记。
    readonly property real routeHandleScale: 0.08
    readonly property real fixedPointSize: 2.0
    readonly property real fixedPickRadius: 20
    readonly property real routeHandleHitRadiusPx: 18
    readonly property real routeSegmentHitRadiusPx: 16
    readonly property real cameraDragThreshold: 6
    property bool loadAllStaticPoints: false
    property int staticPointLimit: 500000
    property int planningAltitudeCentimeters: 80
    property int obstacleClearanceCentimeters: 45
    property int obstacleVerticalClearanceCentimeters: 45
    property bool ceilingClipEnabled: true
    // 当前场地顶部膨胀层下沿约Z=2.0m，默认裁到1.8m以去除顶部遮挡。
    property int ceilingHeightCentimeters: 180
    property bool showFloorGrid: true
    property bool vividPointColors: true
    property bool showAxisHelper: true
    property int lastLiveFrameCount: 0
    property double lastLiveFrameTime: 0
    property bool liveCloudStale: false
    readonly property bool livePoseVisible: poseReceiver.active && poseReceiver.hasPose
    readonly property bool bagPoseVisible: !pointReceiver.active && staticPointLoader.hasPlaybackPose
    readonly property bool vehiclePoseVisible: livePoseVisible || bagPoseVisible
    readonly property var vehiclePosePositionMeters: livePoseVisible
                                                ? poseReceiver.positionMeters
                                                : staticPointLoader.playbackPosePositionMeters
    readonly property real vehiclePoseYawDegrees: livePoseVisible
                                               ? poseReceiver.yawDegrees
                                               : staticPointLoader.playbackPoseYawDegrees
    property int dragWaypointIndex: -1
    property real dragPlaneY: 0
    property bool routeChangedByDrag: false
    property string dragMode: "none"
    property bool controlsPanelOpen: !mobileUi
    property bool touchInteractionConsumed: false
    property bool pinchGestureActive: false
    property real lastPinchScale: 1
    property real pressX: 0
    property real pressY: 0
    property real lastMouseX: 0
    property real lastMouseY: 0
    property bool mouseMovedBeyondClick: false
    property var pendingPointHit: null
    property real cameraPitch: -25
    property real cameraYaw: 0
    property vector3d cameraFocus: Qt.vector3d(0,0,0)
    property bool lvxPlaybackPlaying: false
    property string operatorMessage: plannerMode ? "" : mobileUi
                                     ? qsTr("启动点云后，长按空旷区域按航点高度生成航线。")
                                     : qsTr("启动雷达数据后，左键短按空旷区域按航点高度生成航线。")
    readonly property string hudPointText: pointCloudGeometry.pointCount > 0
                                           ? pointReceiver.active
                                             ? (window.liveCloudStale ? qsTr("点云断流（末帧） %1 | %2 点") : qsTr("实时点云 %1 | %2 点"))
                                                   .arg(pointCloudGeometry.lastSequence)
                                                   .arg(pointCloudGeometry.visiblePointCount)
                                             : qsTr("静态模型 | %1 点").arg(pointCloudGeometry.visiblePointCount)
                                           : qsTr("等待点云 / 上传模型")
    readonly property string hudRouteText: plannerMode ? (plannerController.sessionInvalid ? qsTr("请重新启动预览") : plannerController.stopPending ? qsTr("停止处理中") : plannerController.stopComplete ? qsTr("避障已停止") : plannerController.previewSuppressed ? qsTr("停止结果未确认") : plannedPathVisible ? (plannerState.planning_mode==="measured_preview" ? qsTr("局部预览 #%1") : qsTr("执行规划 #%1")).arg(planning.trajectory_id) : qsTr("等待有效规划")) : routeGeometry.valid ? qsTr("航线就绪") : qsTr("未规划")
    readonly property string hudPoseText: livePoseVisible
        ? qsTr("位置 X:%1  Y:%2  Z:%3 m  |  定位前向:%4°")
            .arg(poseReceiver.positionMeters.x.toFixed(3))
            .arg(poseReceiver.positionMeters.y.toFixed(3))
            .arg(poseReceiver.positionMeters.z.toFixed(3))
            .arg(poseReceiver.yawDegrees.toFixed(1))
        : poseReceiver.active ? qsTr("等待新鲜位姿 / 已过期") : qsTr("位姿显示未启用")

    PointCloudGeometry {
        id: pointCloudGeometry
        rainbowStyle: window.vividPointColors
        ceilingClipEnabled: window.ceilingClipEnabled
        ceilingHeight: window.ceilingHeightCentimeters / 100
        sceneScale: window.fixedSceneScale
    }

    PointCloudGeometry {
        id: pointCloudPreviewGeometry
        rainbowStyle: window.vividPointColors
        ceilingClipEnabled: window.ceilingClipEnabled
        ceilingHeight: window.ceilingHeightCentimeters / 100
        sceneScale: window.fixedSceneScale
    }

    RouteGeometry {
        id: routeGeometry
        sceneScale: pointCloudGeometry.sceneScale
    }

    UdpPointCloudReceiver {
        id: pointReceiver
        realtimeAccumulation: window.plannerMode
        persistentMapping: window.plannerMode
        geometry: pointCloudGeometry
        listenPort: pointPortSpin.value
        active: true
    }

    UdpPoseReceiver {
        id: poseReceiver
        listenPort: posePortSpin.value
        active: startupPoseActive
    }

    PreflightReceiver {
        id: flightMonitor
        expectedHost:onboardConnection.host
        listenPort: startupPreflightPort
        active: startupPreflightActive
    }

    ConnectionController {
        id: onboardConnection
        onEndpointChanging: {
            videoController.stop()
            missionController.disconnectMonitor()
            pointReceiver.clearDisplay();occupancyReceiver.clearDisplay()
            nativePath.clearRoute();candidatePath.clearRoute();actualPath.clearRoute()
            window.actualHistory=[]
            plannerPanel.clearReview();plannerController.invalidateSession()
            pointReceiver.active=true
            poseReceiver.active=true;poseReceiver.restart()
            flightMonitor.active=false
        }
        onChanged: {
            videoController.setHost(host)
            flightMonitor.active=startupPreflightActive || authenticated
            if (authenticated) missionController.watch()
        }
    }
    readonly property bool onboardFlightActive: flightMonitor.fresh && (
        (!!window.planning.execution && !!window.planning.execution.armed)
        || flightMonitor.rows.some(function(row) { return row.key === "fc" && row.detail && row.detail.indexOf(" / 已解锁") >= 0 }))
    PlannerController { id: plannerController; host:onboardConnection.host; externalControlActive: missionController.busy || !!missionController.state.running }
    MissionController { id: missionController; host: onboardConnection.host }
    RecordingController {id:recordingController;host:plannerController.host}
    FlightRecordController {id:flightRecordController;host:plannerController.host}
    FlightRecordWindow {id:flightRecordWindow;backend:flightRecordController}
    LocalizationController {
        id:localizationController;host:plannerController.host
        onSessionReset:{
            pointReceiver.clearDisplay();occupancyReceiver.clearDisplay()
            nativePath.clearRoute();candidatePath.clearRoute();actualPath.clearRoute();window.actualHistory=[]
            plannerPanel.clearReview();plannerController.invalidateSession()
        }
    }
    PointCloudGeometry {
        id: occupancyGeometry
        sceneScale:window.fixedSceneScale
        // 膨胀栅格与点云共用顶部显示裁剪，不改机载占据地图。
        ceilingClipEnabled: window.ceilingClipEnabled
        ceilingHeight: window.ceilingHeightCentimeters / 100
        voxelSize:window.planning.grid_resolution || 0
    }
    UdpPointCloudReceiver {
        id: occupancyReceiver
        geometry:occupancyGeometry
        listenPort:9004
        active:window.plannerMode
    }
    RouteGeometry { id:missionRoute; sceneScale:window.fixedSceneScale; waypoints:missionControls.routePositions() }
    RouteGeometry { id:nativePath;sceneScale:window.fixedSceneScale }
    RouteGeometry { id:candidatePath;sceneScale:window.fixedSceneScale }
    RouteGeometry { id:actualPath;sceneScale:window.fixedSceneScale }
    Connections {
        target:plannerController
        function onChanged(){
            if(plannerController.previewSuppressed){
                nativePath.clearRoute();candidatePath.clearRoute()
                plannerPanel.pickGoal=false
            }
        }
    }
    Connections {
        target:flightMonitor
        function onRowsChanged(){localizationController.observe(flightMonitor.localization,flightMonitor.rows,flightMonitor.fresh);window.updatePlannerDisplay();recordingController.observe(flightMonitor.recording,flightMonitor.fresh)}
        function onStatusChanged(){
            plannerController.observe(window.planning,flightMonitor.fresh)
            recordingController.observe(flightMonitor.recording,flightMonitor.fresh)
            localizationController.observe(flightMonitor.localization,flightMonitor.rows,flightMonitor.fresh)
            if(!flightMonitor.fresh)plannerPanel.pickGoal=false
        }
    }
    Connections {
        target:poseReceiver
        function onPoseChanged(){
            if(!window.plannerMode)return
            if(!poseReceiver.hasPose){window.actualHistory=[];actualPath.clearRoute();return}
            const p=poseReceiver.positionMeters
            const points=window.actualHistory
            const last=points.length ? points[points.length-1] : null
            if(last && Math.sqrt(Math.pow(p.x-last.x,2)+Math.pow(p.y-last.y,2)+Math.pow(p.z-last.z,2))<.02)return
            points.push(Qt.vector3d(p.x,p.y,p.z))
            if(points.length>2000)points.shift()
            window.actualHistory=points;actualPath.waypoints=points
        }
    }

    Connections {
        target: pointReceiver
        function onStatsChanged() {
            if (pointReceiver.framesReceived !== window.lastLiveFrameCount) {
                window.lastLiveFrameCount = pointReceiver.framesReceived
                window.lastLiveFrameTime = Date.now()
                window.liveCloudStale = false
            }
        }
    }

    Timer {
        interval: 500
        repeat: true
        running: pointReceiver.active
        onTriggered: window.liveCloudStale = window.lastLiveFrameTime > 0 && Date.now() - window.lastLiveFrameTime > 2000
    }

    RouteUdpSender {
        id: routeSender
        targetHost: routeHostField.text
        targetPort: routePortSpin.value
        uploadProtocol: routeProtocolBox.protocolValue
    }

    PointCloudFileLoader {
        id: staticPointLoader
        geometry: pointCloudGeometry
        previewGeometry: pointCloudPreviewGeometry
        maxPoints: window.loadAllStaticPoints ? 0 : window.staticPointLimit
    }

    FileDialog {
        id: staticPointDialog
        title: qsTr("选择静态点云模型")
        nameFilters: [
            qsTr("点云模型 (*.lvx2 *.bag *.db3 *.pcd *.ply *.xyz *.txt *.csv)"),
            qsTr("所有文件 (*)")
        ]
        onAccepted: {
            console.info("用户选择静态点云模型：" + selectedFile)
            window.loadStaticPointCloud(selectedFile)
        }
    }

    Connections {
        target: staticPointLoader

        function onLoadFinished(success) {
            console.info("静态点云加载结束，success=" + success
                         + "，loadedFile=" + staticPointLoader.loadedFile
                         + "，pointCount=" + pointCloudGeometry.pointCount)
            window.lvxPlaybackPlaying = false
            if (success) {
                routeGeometry.clearRoute()
                window.operatorMessage = qsTr("静态点云模型已加载。左键短按空旷区域生成航点，确认后点击上传。")
            } else {
                window.operatorMessage = qsTr("静态点云模型加载失败，请查看左侧状态信息。")
            }
        }
    }

    Timer {
        id: lvxPlaybackTimer
        interval: 50
        repeat: true
        running: window.lvxPlaybackPlaying && staticPointLoader.playbackAvailable
        onTriggered: {
            const nextPosition = staticPointLoader.playbackPositionMs + interval
            if (nextPosition >= staticPointLoader.playbackDurationMs) {
                staticPointLoader.playbackPositionMs = staticPointLoader.playbackDurationMs
                window.lvxPlaybackPlaying = false
            } else {
                staticPointLoader.playbackPositionMs = nextPosition
            }
        }
    }

    Timer {
        id: staticPreviewRestoreTimer
        interval: 1200
        repeat: false
        onTriggered: staticPointLoader.setInteractivePreviewEnabled(false)
    }

    Drawer {
            id: sidePanel
            objectName: "desktopFunctionDrawer"
            width: Math.min(window.panelWidth, window.width - 40)
            height: window.height
            edge: Qt.LeftEdge
            modal: false
            dim: false
            interactive: !window.mobileUi
            closePolicy: Popup.NoAutoClose
            Component.onCompleted: if (!window.mobileUi) open()
            padding: 16
            background: Rectangle {
                color: "#202125"
            }

            contentItem: ScrollView {
                id: sideScroll
                clip: true
                ScrollBar.horizontal.policy: ScrollBar.AlwaysOff

                ColumnLayout {
                    width: sideScroll.availableWidth
                    spacing: 12

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8
                        BrandBadge {
                            Layout.preferredWidth:46
                            Layout.preferredHeight:46
                        }
                        Label {
                            Layout.fillWidth: true
                            text: qsTr("科研飞行平台-地面站")
                            color: "#eeeef1"
                            font.pixelSize: 16
                            font.weight: Font.DemiBold
                            wrapMode: Text.WordWrap
                        }
                        ToolButton { objectName: "closeFunctionDrawer"; text: "‹"; onClicked: sidePanel.close() }
                    }

                    Label {
                        visible:text.length>0
                        Layout.fillWidth: true
                        text: window.operatorMessage
                        color: "#b2b3bc"
                        wrapMode: Text.WordWrap
                        font.pixelSize: 12
                    }

                ConnectionPanel {
                    Layout.fillWidth:true
                    connection:onboardConnection
                    cloudPort:pointPortSpin.value
                    posePort:posePortSpin.value
                    preflightPort:startupPreflightPort
                    canApply:!window.onboardFlightActive && !plannerController.busy && !plannerController.stopPending && !missionController.busy
                        && !localizationController.busy && !recordingController.busy
                }

                LocalizationPanel {
                    visible:window.plannerMode
                    Layout.fillWidth:true
                    controller:localizationController
                    otherBusy:plannerController.busy||plannerController.stopPending
                }

                PlannerPanel {
                    id:plannerPanel
                    enabled:!localizationController.busy
                    visible:window.plannerMode
                    Layout.fillWidth:true
                    controller:plannerController
                    monitor:flightMonitor
                    planning:window.planning
                    poseFresh:window.livePoseVisible
                    currentPoseZ:poseReceiver.positionMeters.z
                    onPickRequested:{
                        missionControls.picking=false
                        // 俯视当前等高平面，消除倾斜视角将点击投影到远处的歧义。
                        window.cameraFocus=window.metersToScene(poseReceiver.positionMeters)
                        window.cameraPitch=-89.9;window.cameraYaw=0
                    }
                }

                MissionControls {
                    id: missionControls
                    onPickRequested: {
                        plannerPanel.pickGoal=false
                        window.cameraFocus=window.metersToScene(poseReceiver.positionMeters)
                        window.cameraPitch=-89.9;window.cameraYaw=0
                    }
                    controller: missionController
                    waypointVisible: window.plannerMode
                }

                RecordingPanel {
                    visible:window.plannerMode
                    Layout.fillWidth:true
                    recorder:recordingController
                    onReadRecordsRequested:flightRecordWindow.openRecords()
                }

                SectionBox {
                    title: qsTr("高级设置")
                    ColumnLayout {
                        anchors.fill: parent
                SectionBox {
                    Layout.fillWidth: true
                    title: qsTr("通信")

                    ColumnLayout {
                        anchors.fill: parent
                        spacing: 8

                        RowLayout {
                            Layout.fillWidth: true
                            Label {
                                Layout.fillWidth: true
                                text: qsTr("点云端口")
                            }
                            SpinBox {
                                id: pointPortSpin
                                from: 1
                                to: 65535
                                value: startupPointCloudPort
                                editable: true
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Label {
                                Layout.fillWidth: true
                                text: qsTr("位姿端口")
                            }
                            SpinBox {
                                id: posePortSpin
                                from: 1
                                to: 65535
                                value: startupPosePort
                                editable: true
                            }
                        }

                        RowLayout {
                            visible: !window.plannerMode
                            Layout.fillWidth: true
                            Label {
                                Layout.fillWidth: true
                                text: qsTr("航线目标")
                            }
                            TextField {
                                id: routeHostField
                                Layout.preferredWidth: 124
                                text: "127.0.0.1"
                                selectByMouse: true
                            }
                        }

                        RowLayout {
                            visible: !window.plannerMode
                            Layout.fillWidth: true
                            Label {
                                Layout.fillWidth: true
                                text: qsTr("航线端口")
                            }
                            SpinBox {
                                id: routePortSpin
                                from: 1
                                to: 65535
                                value: 56001
                                editable: true
                            }
                        }

                        RowLayout {
                            visible: !window.plannerMode
                            Layout.fillWidth: true
                            Label {
                                Layout.fillWidth: true
                                text: qsTr("航线协议")
                            }
                            ComboBox {
                                id: routeProtocolBox
                                Layout.preferredWidth: 170
                                model: [
                                    qsTr("本机 RTE1 完整航线"),
                                    qsTr("香橙派 GOL1 逐点")
                                ]
                                readonly property string protocolValue: currentIndex === 1 ? "gol1" : "rte1"
                            }
                        }

                        ActionButton {
                            Layout.fillWidth: true
                            visible: !window.plannerMode
                            text: qsTr("套用香橙派 UDP")
                            onClicked: {
                                pointPortSpin.value = 9000
                                routeHostField.text = "192.168.1.196"
                                routePortSpin.value = 9002
                                routeProtocolBox.currentIndex = 1
                                pointReceiver.active = true
                                pointReceiver.restart()
                                poseReceiver.active = true
                                poseReceiver.restart()
                                window.operatorMessage = qsTr("已切换到香橙派 UDP：点云 9000，位姿 9001，航线 GOL1 -> 192.168.1.196:9002。")
                            }
                        }

                        ActionButton {
                            Layout.fillWidth: true
                            text: pointReceiver.active ? qsTr("重启点云监听") : qsTr("启动点云监听")
                            onClicked: {
                                pointReceiver.active = true
                                pointReceiver.restart()
                            }
                        }

                        ActionButton {
                            Layout.fillWidth: true
                            text: poseReceiver.active ? qsTr("重启位姿监听") : qsTr("启动位姿监听")
                            onClicked: {
                                poseReceiver.active = true
                                poseReceiver.restart()
                            }
                        }





                        Label {
                            Layout.fillWidth: true
                            visible: !window.plannerMode
                            text: routeSender.status
                            color: "#facc15"
                            wrapMode: Text.WordWrap
                        }
                    }
                }

                    }
                }

                SectionBox {
                    Layout.fillWidth: true
                    title: qsTr("点云与视图")

                    ColumnLayout {
                        anchors.fill: parent
                        spacing: 8

                        InfoRow {
                            label: qsTr("显示点数")
                            value: pointCloudGeometry.visiblePointCount.toString()
                        }
                        InfoRow {
                            label: qsTr("单帧点数")
                            value: pointReceiver.latestFramePointCount.toString()
                        }
                        InfoRow {
                            label: qsTr("已收帧")
                            value: pointReceiver.framesReceived.toString()
                        }
                        InfoRow {
                            label: qsTr("UDP 包")
                            value: pointReceiver.packetsReceived.toString()
                        }
                        InfoRow {
                            label: qsTr("丢帧")
                            value: pointReceiver.droppedFrames.toString()
                        }
                        InfoRow {
                            visible: !window.plannerMode
                            label: qsTr("模拟点率")
                            value: qsTr("200k 点/秒")
                        }
                        StyledCheckBox {
                            Layout.fillWidth: true
                            text: qsTr("隐藏高处点云")
                            checked: window.ceilingClipEnabled
                            onClicked: window.ceilingClipEnabled = checked
                        }
                        RowLayout {
                            Layout.fillWidth: true
                            enabled: window.ceilingClipEnabled
                            Label { Layout.fillWidth: true; text: qsTr("显示高度上限 (m)") }
                            SpinBox {
                                Layout.preferredWidth: 118
                                from: -1000; to: 10000; stepSize: 10; editable: true
                                value: window.ceilingHeightCentimeters
                                textFromValue: function(value) { return (value / 100).toFixed(2) }
                                valueFromText: function(text) {
                                    const n = Number(text)
                                    return isFinite(n) ? Math.max(from, Math.min(to, Math.round(n * 100))) : value
                                }
                                onValueModified: window.ceilingHeightCentimeters = value
                            }
                        }
                        Label {
                            Layout.fillWidth: true
                            text: qsTr("Z 高度相对点云坐标原点；只隐藏画面")
                            color: "#93959f"; font.pixelSize: 11; wrapMode: Text.WordWrap
                        }
                        StyledCheckBox {
                            Layout.fillWidth: true
                            text: qsTr("显示地面参考网格")
                            checked: window.showFloorGrid
                            onClicked: window.showFloorGrid = checked
                        }
                        StyledCheckBox {
                            Layout.fillWidth: true
                            text: qsTr("鲜艳彩虹高度色")
                            checked: window.vividPointColors
                            onClicked: window.vividPointColors = checked
                        }
                        StyledCheckBox {
                            Layout.fillWidth: true
                            text: qsTr("显示 XYZ 轴")
                            checked: window.showAxisHelper
                            onClicked: window.showAxisHelper = checked
                        }
                        ActionButton {
                            Layout.fillWidth:true
                            text:qsTr("复位视角")
                            onClicked:window.resetCamera()
                        }
                        StyledCheckBox {
                            Layout.fillWidth: true
                            text: qsTr("累计建图显示")
                            checked: pointReceiver.realtimeAccumulation
                            onClicked: pointReceiver.realtimeAccumulation = checked
                        }
                        ActionButton {
                            Layout.fillWidth: true
                            visible: pointReceiver.realtimeAccumulation
                            text: qsTr("清空累计点云")
                            onClicked: pointReceiver.clearDisplay()
                        }
                        RowLayout {
                            Layout.fillWidth: true
                            visible: !pointReceiver.persistentMapping
                            enabled: pointReceiver.realtimeAccumulation

                            Label {
                                Layout.fillWidth: true
                                text: qsTr("累计帧数")
                            }
                            SpinBox {
                                Layout.preferredWidth: 118
                                from: 1
                                to: 30
                                stepSize: 1
                                editable: true
                                value: pointReceiver.accumulationFrameWindow
                                onValueModified: pointReceiver.accumulationFrameWindow = value
                            }
                        }
                        RowLayout {
                            Layout.fillWidth: true
                            enabled: pointReceiver.realtimeAccumulation

                            Label {
                                Layout.fillWidth: true
                                text: qsTr("累计上限")
                            }
                            SpinBox {
                                id: accumulationPointLimitSpin
                                Layout.preferredWidth: 142
                                from: 10000
                                to: 2000000
                                stepSize: 10000
                                editable: true
                                value: pointReceiver.accumulationPointLimit
                                textFromValue: function(value) {
                                    return qsTr("%1 万").arg(Math.round(value / 10000))
                                }
                                valueFromText: function(text) {
                                    const value = Number(String(text).replace(/[^\d.]/g, ""))
                                    if (isNaN(value))
                                        return accumulationPointLimitSpin.value
                                    const pointValue = value >= 1000 ? Math.round(value) : Math.round(value * 10000)
                                    return Math.max(accumulationPointLimitSpin.from,
                                                    Math.min(accumulationPointLimitSpin.to, pointValue))
                                }
                                onValueModified: pointReceiver.accumulationPointLimit = value
                            }
                        }
                    }
                }

                SectionBox {
                    Layout.fillWidth: true
                    title: qsTr("静态点云模型")
                    visible: !window.plannerMode

                    ColumnLayout {
                        anchors.fill: parent
                        spacing: 8

                        InfoRow {
                            label: qsTr("显示模式")
                            value: staticPointLoader.loading
                                   ? qsTr("加载中")
                                   : pointReceiver.active ? qsTr("实时 UDP") : qsTr("静态模型")
                        }

                        InfoRow {
                            visible: !pointReceiver.active && !staticPointLoader.loading
                            label: qsTr("渲染状态")
                            value: staticPointLoader.interactivePreviewEnabled
                                   ? qsTr("交互预览")
                                   : staticPointLoader.showingPlaybackFrame ? qsTr("时间窗") : qsTr("完整地图")
                        }

                        StyledCheckBox {
                            Layout.fillWidth: true
                            enabled: !staticPointLoader.loading
                            text: qsTr("加载全部点")
                            checked: window.loadAllStaticPoints
                            onClicked: window.loadAllStaticPoints = checked
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            enabled: !window.loadAllStaticPoints && !staticPointLoader.loading

                            Label {
                                Layout.fillWidth: true
                                text: qsTr("显示上限")
                            }
                            SpinBox {
                                id: staticPointLimitSpin
                                Layout.preferredWidth: 142
                                from: 100000
                                to: 20000000
                                stepSize: 100000
                                editable: true
                                value: window.staticPointLimit
                                onValueModified: window.staticPointLimit = value
                            }
                        }

                        Label {
                            Layout.fillWidth: true
                            visible: window.loadAllStaticPoints
                            text: qsTr("全量点云会占用更多内存，点选和旋转可能变慢。")
                            color: "#dbad48"
                            wrapMode: Text.WordWrap
                        }

                        ActionButton {
                            Layout.fillWidth: true
                            enabled: !staticPointLoader.loading
                            text: staticPointLoader.loading
                                  ? qsTr("正在加载 %1%").arg(staticPointLoader.progress)
                                  : qsTr("上传点云模型")
                            onClicked: staticPointDialog.open()
                        }

                        ActionButton {
                            Layout.fillWidth: true
                            enabled: !pointReceiver.active && !staticPointLoader.loading
                            text: qsTr("恢复实时点云")
                            onClicked: {
                                window.lvxPlaybackPlaying = false
                                pointReceiver.active = true
                                pointReceiver.restart()
                                routeGeometry.clearRoute()
                                window.operatorMessage = qsTr("已恢复实时 UDP 点云。等待模拟器或真实雷达适配器发送数据。")
                            }
                        }

                        Label {
                            Layout.fillWidth: true
                            text: staticPointLoader.status
                            color: pointReceiver.active ? "#93959f" : "#3cbb83"
                            wrapMode: Text.WordWrap
                        }

                        ColumnLayout {
                            Layout.fillWidth: true
                            visible: staticPointLoader.playbackAvailable
                            spacing: 8

                            InfoRow {
                                label: qsTr("录制时间")
                                value: window.formatPlaybackTime(staticPointLoader.playbackPositionMs)
                                       + qsTr(" / ")
                                       + window.formatPlaybackTime(staticPointLoader.playbackDurationMs)
                            }

                            InfoRow {
                                visible: staticPointLoader.playbackPoseAvailable
                                label: qsTr("录制位姿")
                                value: qsTr("%1 条").arg(staticPointLoader.playbackPoseCount)
                            }

                            Slider {
                                id: lvxPlaybackSlider
                                Layout.fillWidth: true
                                from: 0
                                to: Math.max(1, staticPointLoader.playbackDurationMs)
                                value: staticPointLoader.playbackPositionMs
                                enabled: staticPointLoader.playbackAvailable && !staticPointLoader.loading
                                onMoved: staticPointLoader.playbackPositionMs = Math.round(value)
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                ActionButton {
                                    Layout.fillWidth: true
                                    enabled: staticPointLoader.playbackAvailable && !staticPointLoader.loading
                                    text: window.lvxPlaybackPlaying ? qsTr("暂停") : qsTr("播放")
                                    onClicked: {
                                        if (staticPointLoader.playbackPositionMs >= staticPointLoader.playbackDurationMs)
                                            staticPointLoader.playbackPositionMs = 0
                                        window.lvxPlaybackPlaying = !window.lvxPlaybackPlaying
                                    }
                                }
                                ActionButton {
                                    Layout.fillWidth: true
                                    enabled: staticPointLoader.playbackAvailable && !staticPointLoader.loading
                                    text: qsTr("累计地图")
                                    onClicked: {
                                        window.lvxPlaybackPlaying = false
                                        staticPointLoader.showAccumulatedPointCloud()
                                    }
                                }
                            }

                            Label {
                                Layout.fillWidth: true
                                text: staticPointLoader.showingPlaybackFrame
                                      ? qsTr("当前显示录制时间窗。")
                                      : qsTr("当前显示累计点云地图。")
                                color: "#b8bac4"
                                wrapMode: Text.WordWrap
                            }

                            Label {
                                Layout.fillWidth: true
                                visible: staticPointLoader.playbackPoseAvailable
                                text: staticPointLoader.playbackPoseText
                                color: "#facc15"
                                wrapMode: Text.WordWrap
                            }
                        }
                    }
                }

                SectionBox {
                    Layout.fillWidth: true
                    title: qsTr("航点规划")
                    visible: !window.plannerMode

                    ColumnLayout {
                        anchors.fill: parent
                        spacing: 8

                        InfoRow {
                            label: qsTr("航点数")
                            value: routeGeometry.waypointCount.toString()
                        }
                        InfoRow {
                            label: qsTr("已发送")
                            value: routeSender.routesSent.toString()
                        }

                        Label {
                            Layout.fillWidth: true
                            text: routeGeometry.valid
                                  ? qsTr("拖动黄色航点或航线线段避开障碍；可单独发送任意航点。")
                                  : qsTr("请短按空旷区域选择起点和终点。")
                            color: "#bdbec6"
                            wrapMode: Text.WordWrap
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Label {
                                Layout.fillWidth: true
                                text: qsTr("航点高度")
                            }
                            SpinBox {
                                Layout.preferredWidth: 124
                                from: -100
                                to: 500
                                stepSize: 10
                                editable: true
                                value: window.planningAltitudeCentimeters
                                textFromValue: function(value) {
                                    return qsTr("%1 m").arg((value / 100).toFixed(2))
                                }
                                valueFromText: function(text) {
                                    const value = Number(String(text).replace(/[^\d.-]/g, ""))
                                    if (isNaN(value))
                                        return window.planningAltitudeCentimeters
                                    return Math.round(value * 100)
                                }
                                onValueModified: window.planningAltitudeCentimeters = value
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Label {
                                Layout.fillWidth: true
                                text: qsTr("安全半径")
                            }
                            SpinBox {
                                Layout.preferredWidth: 124
                                from: 10
                                to: 300
                                stepSize: 5
                                editable: true
                                value: window.obstacleClearanceCentimeters
                                textFromValue: function(value) {
                                    return qsTr("%1 m").arg((value / 100).toFixed(2))
                                }
                                valueFromText: function(text) {
                                    const value = Number(String(text).replace(/[^\d.-]/g, ""))
                                    if (isNaN(value))
                                        return window.obstacleClearanceCentimeters
                                    return Math.round(value * 100)
                                }
                                onValueModified: window.obstacleClearanceCentimeters = value
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            ActionButton {
                                Layout.fillWidth: true
                                text: qsTr("清空航线")
                                onClicked: {
                                    routeGeometry.clearRoute()
                                    window.operatorMessage = qsTr("航线已清空，请重新选择两个点。")
                                }
                            }
                            ActionButton {
                                Layout.fillWidth: true
                                enabled: routeGeometry.valid
                                text: qsTr("上传航线")
                                onClicked: {
                                    const conflict = window.routeObstacleConflict()
                                    if (conflict.blocked) {
                                        console.warn("航线上传被避障校验拦截，segment="
                                                     + conflict.segmentIndex
                                                     + "，distance=" + conflict.distance)
                                        window.operatorMessage = qsTr("第 %1-%2 段航线距离障碍物过近，请拖动线段插入避障航点。")
                                                .arg(conflict.segmentIndex + 1)
                                                .arg(conflict.segmentIndex + 2)
                                        return
                                    }

                                    const sent = routeSender.sendRoute(routeGeometry)
                                    console.info("用户点击上传航线，航点数=" + routeGeometry.waypointCount
                                                 + "，result=" + sent)
                                    window.operatorMessage = sent
                                            ? qsTr("航线已上传。后续拖动调整不会自动上传，请确认后再次点击上传航线。")
                                            : qsTr("航线上传失败，请检查航线端口和日志文件。")
                                }
                            }
                        }

                        ScrollView {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 120
                            clip: true

                            ListView {
                                model: routeGeometry.waypoints
                                delegate: RowLayout {
                                    required property int index
                                    required property var modelData

                                    width: ListView.view.width
                                    spacing: 8

                                    Label {
                                        Layout.fillWidth: true
                                        color: "#d2d3da"
                                        font.family: "monospace"
                                        font.pixelSize: 11
                                        text: qsTr("%1  X:%2  Y:%3  Z:%4")
                                              .arg(index + 1)
                                              .arg(Number(modelData.x).toFixed(2))
                                              .arg(Number(modelData.y).toFixed(2))
                                              .arg(Number(modelData.z).toFixed(2))
                                    }

                                    ActionButton {
                                        Layout.preferredWidth: 58
                                        Layout.preferredHeight: 28
                                        Layout.minimumHeight: 28
                                        text: qsTr("发送")
                                        onClicked: {
                                            const check = window.waypointObstacleConflict(modelData)
                                            if (check.blocked) {
                                                console.warn("单航点发送被避障校验拦截，index="
                                                             + index
                                                             + "，distance=" + check.distance)
                                                window.operatorMessage = qsTr("第 %1 个航点距离障碍物过近，请先拖动避开。")
                                                        .arg(index + 1)
                                                return
                                            }

                                            const sent = routeSender.sendSingleWaypoint(modelData)
                                            window.operatorMessage = sent
                                                    ? qsTr("已上传第 %1 个单航点。").arg(index + 1)
                                                    : qsTr("单航点上传失败，请检查目标端口和日志。")
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                Item {
                    Layout.fillHeight: true
                }

                Label {
                    Layout.fillWidth: true
                    visible:!window.plannerMode
                    text: window.mobileUi
                          ? qsTr("单指拖动：旋转视角    双指捏合：缩放    长按空旷区域：选点    拖航点/线段：避障调整")
                          : qsTr("左键短按：选点    左键拖空白：旋转    中键拖动：平移    滚轮：缩放")
                    color: "#93959f"
                    wrapMode: Text.WordWrap
                    font.pixelSize: 11
                }
            }
            }
        }

    Button {
        objectName: "openFunctionDrawer"
        visible: !window.mobileUi && !sidePanel.visible
        anchors.left: parent.left
        anchors.verticalCenter: parent.verticalCenter
        z: 100
        text: qsTr("功能 ›")
        onClicked: sidePanel.open()
    }

    RowLayout {
        anchors.fill: parent
        spacing: 0

        Item {
            id: scenePane
            Layout.fillWidth: true
            Layout.fillHeight: true

            View3D {
                id: v3d
                anchors.fill: parent
                camera: cameraNode

                environment: SceneEnvironment {
                    backgroundMode: SceneEnvironment.Color
                    clearColor: "#111114"
                    antialiasingMode: SceneEnvironment.MSAA
                    antialiasingQuality: SceneEnvironment.High
                }

                DirectionalLight {
                    eulerRotation: Qt.vector3d(-45, 25, 0)
                    brightness: 1.2
                    color: "#ffffff"
                    ambientColor: "#4b5563"
                }

                Node {
                    id: cameraOrigin
                    position:window.cameraFocus
                    eulerRotation: Qt.vector3d(window.cameraPitch, window.cameraYaw, 0)

                    PerspectiveCamera {
                        id: cameraNode
                        z: 920
                        clipNear: 1
                        clipFar: 10000
                    }
                }

                Model {
                    id: floorGrid
                    visible: window.showFloorGrid
                    eulerRotation.x: -90
                    y: 0
                    geometry: GridGeometry {
                        horizontalLines: 28
                        verticalLines: 28
                        horizontalStep: pointCloudGeometry.sceneScale
                        verticalStep: pointCloudGeometry.sceneScale
                    }
                    materials: [
                        PrincipledMaterial {
                            lighting: PrincipledMaterial.NoLighting
                            baseColor: "#2f4658"
                            lineWidth: 1
                        }
                    ]
                }

                AxisHelper {
                    id: axisHelper
                    visible: window.showAxisHelper
                    enableAxisLines: true
                    enableXYGrid: false
                    enableXZGrid: false
                    enableYZGrid: false
                    scale: Qt.vector3d(0.55, 0.55, 0.55)
                }

                Model {
                    id: pointCloudModel
                    visible: !staticPointLoader.interactivePreviewEnabled
                    geometry: pointCloudGeometry
                    materials: [
                        PrincipledMaterial {
                            lighting: PrincipledMaterial.NoLighting
                            cullMode: PrincipledMaterial.NoCulling
                            vertexColorsEnabled: true
                            pointSize: window.fixedPointSize
                        }
                    ]
                }

                Model {
                    id: pointCloudPreviewModel
                    visible: staticPointLoader.interactivePreviewEnabled
                    geometry: pointCloudPreviewGeometry
                    materials: [
                        PrincipledMaterial {
                            lighting: PrincipledMaterial.NoLighting
                            cullMode: PrincipledMaterial.NoCulling
                            vertexColorsEnabled: true
                            pointSize: window.fixedPointSize
                        }
                    ]
                }

                Model {
                    visible: window.plannerMode && missionRoute.valid
                    geometry: missionRoute
                    materials: PrincipledMaterial { lighting:PrincipledMaterial.NoLighting; baseColor:"#facc15"; lineWidth:4 }
                }
                Repeater3D {
                    model: window.plannerMode ? missionRoute.sceneWaypoints : []
                    delegate: Model {
                        required property var modelData
                        required property int index
                        position: modelData; source:"#Sphere"; scale:Qt.vector3d(.09,.09,.09)
                        materials: PrincipledMaterial { lighting:PrincipledMaterial.NoLighting; baseColor:"#facc15" }
                    }
                }
                Model {
                    id: routeModel
                    visible: !window.plannerMode && routeGeometry.valid
                    geometry: routeGeometry
                    materials: [
                        PrincipledMaterial {
                            lighting: PrincipledMaterial.NoLighting
                            cullMode: PrincipledMaterial.NoCulling
                            baseColor: "#facc15"
                            lineWidth: 4
                        }
                    ]
                }

                Model {
                    visible:window.plannerMode && plannerPanel.showGrid && flightMonitor.fresh && !!window.planning.grid_display_valid && window.planning.grid_count>0
                    geometry:occupancyGeometry
                    materials:PrincipledMaterial {lighting:PrincipledMaterial.NoLighting;baseColor:"#f97316";opacity:.25;cullMode:PrincipledMaterial.NoCulling}
                }
                Model {
                    visible:window.plannedPathVisible
                    geometry:nativePath
                    materials:PrincipledMaterial {lighting:PrincipledMaterial.NoLighting;baseColor:window.plannerState.planning_mode==="upstream_fsm" ? "#fb923c" : "#4ade80";lineWidth:4}
                }
                Model {
                    visible:window.candidatePathVisible
                    geometry:candidatePath
                    materials:PrincipledMaterial {lighting:PrincipledMaterial.NoLighting;baseColor:"#4ade80";lineWidth:4}
                }
                Model {
                    visible:window.plannerMode && poseReceiver.hasPose && actualPath.valid
                    geometry:actualPath
                    materials:PrincipledMaterial {lighting:PrincipledMaterial.NoLighting;baseColor:"#60a5fa";lineWidth:3}
                }
                Model {
                    visible:window.plannerMode && !plannerController.previewSuppressed && flightMonitor.fresh && !plannerPanel.hasCandidateTarget && !!window.plannerState.has_goal && !!window.plannerState.goal
                    position:window.plannerState.goal ? window.metersToScene(Qt.vector3d(window.plannerState.goal[0],window.plannerState.goal[1],window.plannerState.goal[2])) : Qt.vector3d(0,0,0)
                    source:"#Sphere"
                    scale:Qt.vector3d(.13,.13,.13)
                    materials:PrincipledMaterial {lighting:PrincipledMaterial.NoLighting;baseColor:window.plannerState.phase==="ACTIVE" ? "#4ade80" : "#ef4444"}
                }
                Model {
                    visible:window.plannerMode && !plannerController.previewSuppressed && flightMonitor.fresh && plannerPanel.hasCandidateTarget
                    position:plannerPanel.candidate.goal ? window.metersToScene(Qt.vector3d(plannerPanel.candidate.goal[0],plannerPanel.candidate.goal[1],plannerPanel.candidate.goal[2])) : Qt.vector3d(0,0,0)
                    source:"#Sphere"
                    scale:Qt.vector3d(.13,.13,.13)
                    materials:PrincipledMaterial {lighting:PrincipledMaterial.NoLighting;baseColor:window.candidatePathVisible ? "#4ade80" : plannerPanel.candidate.phase==="READY" || plannerPanel.candidate.phase==="PLANNING" ? "#fbbf24" : "#ef4444"}
                }

                Node {
                    id: vehiclePoseNode
                    visible: window.vehiclePoseVisible
                    position: window.metersToScene(window.vehiclePosePositionMeters)
                    eulerRotation: Qt.vector3d(0, window.vehiclePoseYawDegrees, 0)

                    Model {
                        source: "#Sphere"
                        scale: Qt.vector3d(0.10, 0.10, 0.10)
                        materials: [
                            PrincipledMaterial {
                                lighting: PrincipledMaterial.NoLighting
                                baseColor: "#22d3ee"
                            }
                        ]
                    }

                    Model {
                        source: "#Cone"
                        position: Qt.vector3d(0, 0, -16)
                        eulerRotation.x: -90
                        scale: Qt.vector3d(0.08, 0.18, 0.08)
                        materials: [
                            PrincipledMaterial {
                                lighting: PrincipledMaterial.NoLighting
                                baseColor: "#facc15"
                            }
                        ]
                    }

                    Model {
                        source: "#Cube"
                        position: Qt.vector3d(0, 0, 3)
                        scale: Qt.vector3d(0.28, 0.018, 0.045)
                        materials: [
                            PrincipledMaterial {
                                lighting: PrincipledMaterial.NoLighting
                                baseColor: "#38bdf8"
                            }
                        ]
                    }
                }

                Model {
                    id: vehicleGroundDot
                    visible: window.vehiclePoseVisible
                    source: "#Cylinder"
                    position: window.metersToScene(Qt.vector3d(window.vehiclePosePositionMeters.x,
                                                               window.vehiclePosePositionMeters.y,
                                                               0))
                    scale: Qt.vector3d(0.13, 0.012, 0.13)
                    materials: [
                        PrincipledMaterial {
                            lighting: PrincipledMaterial.NoLighting
                            baseColor: "#facc15"
                        }
                    ]
                }

                Repeater3D {
                    id: handleRepeater
                    model: window.plannerMode ? [] : routeGeometry.sceneWaypoints
                    delegate: Model {
                        required property int index
                        required property var modelData
                        property int routeHandleIndex: index

                        objectName: "routeHandle" + index
                        source: "#Sphere"
                        pickable: true
                        position: modelData
                        scale: Qt.vector3d(window.routeHandleScale,
                                           window.routeHandleScale,
                                           window.routeHandleScale)
                        materials: [
                            PrincipledMaterial {
                                lighting: PrincipledMaterial.NoLighting
                                baseColor: index === 0 ? "#38bdf8"
                                          : index === routeGeometry.waypointCount - 1 ? "#f472b6"
                                          : "#facc15"
                            }
                        ]
                    }
                }

            }

            Rectangle {
                id: topHud
                z: 20
                anchors.left: parent.left
                anchors.top: parent.top
                anchors.leftMargin: window.mobileUi ? 84 : sidePanel.position * sidePanel.width + 16
                anchors.topMargin: window.mobileUi ? 14 : 16
                width: Math.min(parent.width - (window.mobileUi ? 100 : 32), window.mobileUi ? 420 : Math.min(620, Math.max(200, parent.width - sidePanel.position * sidePanel.width - (preflightPanel.visible ? preflightPanel.width + 48 : 32))))
                height: window.mobileUi ? 46 : 52
                radius: 6
                color: "#ed202125"
                border.color: "#383940"

                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 12

                    Label {
                        Layout.fillWidth: true
                        text: window.hudPointText
                        color: "#eeeef1"
                        elide: Text.ElideRight
                        wrapMode: Text.NoWrap
                    }

                    Label {
                        Layout.preferredWidth: window.mobileUi ? 56 : implicitWidth
                        text: window.hudRouteText
                        color: routeGeometry.valid ? "#3cbb83" : "#dbad48"
                        font.weight: Font.DemiBold
                        horizontalAlignment: Text.AlignRight
                        elide: Text.ElideRight
                    }
                }
            }

            Rectangle {
                z: 20
                anchors.left: topHud.left
                anchors.top: topHud.bottom
                anchors.topMargin: 6
                width: topHud.width
                height: 40
                radius: 6
                color: "#ed202125"
                border.color: "#383940"
                Column {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 4
                    Label {
                        width: parent.width
                        text: window.hudPoseText
                        color: window.livePoseVisible ? "#3cbb83" : "#dbad48"
                        elide: Text.ElideRight
                    }

                }
            }

            Label {
                visible: window.plannerMode && plannerPanel.showGrid && flightMonitor.fresh
                         && !!window.planning.grid_display_valid && !window.planning.grid_valid
                anchors.right: rightPanels.left
                anchors.top: parent.top
                anchors.margins: 16
                text: qsTr("栅格延迟")
                color: "#dbad48"
                padding: 6
                background: Rectangle { color: "#e0202125"; radius: 4 }
            }

            ScrollView {
                id: rightPanels
                z: 30
                visible: !window.mobileUi
                anchors { right: parent.right; top: parent.top; bottom: parent.bottom; margins: 16 }
                width: Math.min(560, Math.max(300, parent.width * 0.30))
                clip: true
                contentWidth: availableWidth
                ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
                Column {
                    width: rightPanels.availableWidth
                    spacing: 12
                    PreflightPanel {
                        id: preflightPanel
                        visible: startupPreflightActive
                        monitor: flightMonitor
                        width: parent.width
                    }
                    VideoPanel {
                        id: videoMonitorPanel
                        taskController: missionController
                        width: parent.width
                        height: collapsed ? implicitHeight : Math.max(240, Math.min(650, rightPanels.availableHeight - (preflightPanel.visible ? preflightPanel.height + 12 : 0) - sensorsPanel.height - 12))
                    }
                    SensorsPanel {
                        id: sensorsPanel
                        monitor: flightMonitor
                        width: parent.width
                    }
                }
            }

            Button {
                id: panelToggleButton
                z: 21
                visible: window.mobileUi
                anchors.left: parent.left
                anchors.top: parent.top
                anchors.leftMargin: 14
                anchors.topMargin: 14
                width: 58
                height: 46
                text: qsTr("操作")
                onClicked: mobileControlDrawer.open()
            }

            MouseArea {
                id: editMouseArea
                z: 1
                anchors.fill: parent
                acceptedButtons: Qt.LeftButton | Qt.RightButton | Qt.MiddleButton
                hoverEnabled: true
                // Android 双指缩放需要让 PinchHandler 从单指 MouseArea 中接管触摸点；
                // Windows 桌面仍保持鼠标拖动不被其它控件抢走。
                preventStealing: !window.mobileUi
                cursorShape: window.dragMode === "route" || window.dragMode === "pan" ? Qt.ClosedHandCursor : Qt.ArrowCursor

                onWheel: function(wheel) {
                    window.beginStaticInteractionPreview()
                    window.zoomCamera(wheel.angleDelta.y)
                    window.endStaticInteractionPreview()
                    wheel.accepted = true
                }

                onPressed: function(mouse) {
                    forceActiveFocus()
                    window.pressX = mouse.x
                    window.pressY = mouse.y
                    window.lastMouseX = mouse.x
                    window.lastMouseY = mouse.y
                    window.mouseMovedBeyondClick = false
                    window.pendingPointHit = null
                    window.routeChangedByDrag = false

                    if (window.mobileUi && window.touchInteractionConsumed) {
                        mouse.accepted = true
                        return
                    }

                    if (mouse.button === Qt.MiddleButton) {
                        window.dragMode = "pan"
                        window.beginStaticInteractionPreview()
                        mouse.accepted = true
                        return
                    }
                    if (mouse.button === Qt.RightButton) {
                        window.dragMode = "camera"
                        window.beginStaticInteractionPreview()
                        mouse.accepted = true
                        return
                    }

                    if (window.plannerMode) {
                        window.dragMode=missionControls.picking || (plannerPanel.pickGoal && plannerController.canPreview) ? "pendingPoint" : "camera"
                        mouse.accepted=true
                        return
                    }

                    const handleIndex = window.nearestRouteHandle(mouse.x, mouse.y)
                    if (handleIndex >= 0) {
                        window.dragMode = "route"
                        window.dragWaypointIndex = handleIndex
                        window.dragPlaneY = routeGeometry.sceneWaypoint(handleIndex).y
                        window.beginStaticInteractionPreview()
                        window.operatorMessage = qsTr("正在拖动第 %1 个航点。").arg(handleIndex + 1)
                        mouse.accepted = true
                        return
                    }

                    const segment = window.nearestRouteSegment(mouse.x, mouse.y)
                    if (segment.index >= 0) {
                        const scenePoint = window.scenePointOnHeightPlane(mouse.x, mouse.y, segment.planeY)
                        if (scenePoint !== null) {
                            routeGeometry.insertWaypoint(segment.index + 1, routeGeometry.fromScenePosition(scenePoint))
                            window.dragMode = "route"
                            window.dragWaypointIndex = segment.index + 1
                            window.dragPlaneY = segment.planeY
                            window.routeChangedByDrag = true
                            window.beginStaticInteractionPreview()
                            window.operatorMessage = qsTr("已插入避障航点，拖动到合适位置后松开。")
                        }
                        mouse.accepted = true
                        return
                    }

                    window.dragMode = "pendingPoint"
                    if (routeGeometry.waypointCount >= 2) {
                        window.operatorMessage = qsTr("航线已生成。可拖动航点/线段调整，或拖动空白处旋转视角。")
                        mouse.accepted = true
                        return
                    }

                    window.pendingPointHit = window.plannedWaypointFromScreen(mouse.x, mouse.y)
                    mouse.accepted = true
                }

                onPositionChanged: function(mouse) {
                    if (window.mobileUi && (window.touchInteractionConsumed || window.pinchGestureActive)) {
                        window.lastMouseX = mouse.x
                        window.lastMouseY = mouse.y
                        mouse.accepted = true
                        return
                    }

                    const dx = mouse.x - window.lastMouseX
                    const dy = mouse.y - window.lastMouseY
                    const totalDx = mouse.x - window.pressX
                    const totalDy = mouse.y - window.pressY
                    if (Math.sqrt(totalDx * totalDx + totalDy * totalDy) > window.cameraDragThreshold)
                        window.mouseMovedBeyondClick = true

                    if (window.dragMode === "pan") {
                        window.panCamera(dx,dy)
                        window.lastMouseX=mouse.x
                        window.lastMouseY=mouse.y
                        mouse.accepted=true
                        return
                    }

                    if (window.dragMode === "pendingPoint" && window.mouseMovedBeyondClick) {
                        window.dragMode = "camera"
                        window.pendingPointHit = null
                        window.beginStaticInteractionPreview()
                    }

                    if (window.dragMode === "camera") {
                        window.rotateCamera(dx, dy)
                        window.lastMouseX = mouse.x
                        window.lastMouseY = mouse.y
                        mouse.accepted = true
                        return
                    }

                    if (window.dragMode !== "route" || window.dragWaypointIndex < 0) {
                        window.lastMouseX = mouse.x
                        window.lastMouseY = mouse.y
                        return
                    }

                    const scenePoint = window.scenePointOnHeightPlane(mouse.x, mouse.y, window.dragPlaneY)
                    if (scenePoint === null) {
                        window.lastMouseX = mouse.x
                        window.lastMouseY = mouse.y
                        return
                    }

                    // 拖拽只在当前高度平面内移动，避免不小心把航点拖到楼层外。
                    routeGeometry.setWaypoint(window.dragWaypointIndex, routeGeometry.fromScenePosition(scenePoint))
                    window.routeChangedByDrag = true
                    window.lastMouseX = mouse.x
                    window.lastMouseY = mouse.y
                    mouse.accepted = true
                }

                onReleased: function(mouse) {
                    if (window.touchInteractionConsumed) {
                        window.dragMode = "none"
                        window.dragWaypointIndex = -1
                        window.routeChangedByDrag = false
                        window.pendingPointHit = null
                        window.touchInteractionConsumed = false
                        window.endStaticInteractionPreview()
                        mouse.accepted = true
                        return
                    }

                    if (window.dragMode === "pendingPoint" && !window.mouseMovedBeyondClick) {
                        if(window.plannerMode && missionControls.picking)window.selectMissionWaypoint(mouse.x,mouse.y)
                        else if(window.plannerMode)window.selectPlannerGoal(mouse.x,mouse.y)
                        else window.selectPointFromPendingHit()
                    } else if (window.dragMode === "route" && window.dragWaypointIndex >= 0
                               && window.routeChangedByDrag) {
                        const conflict = window.routeObstacleConflict()
                        window.operatorMessage = conflict.blocked
                                ? qsTr("第 %1-%2 段航线距离障碍物过近，请继续拖动避开。")
                                      .arg(conflict.segmentIndex + 1)
                                      .arg(conflict.segmentIndex + 2)
                                : qsTr("航线已调整。确认无碰撞后点击上传航线。")
                    }
                    window.dragMode = "none"
                    window.dragWaypointIndex = -1
                    window.routeChangedByDrag = false
                    window.pendingPointHit = null
                    window.endStaticInteractionPreview()
                    mouse.accepted = true
                }

                onCanceled: {
                    window.dragMode = "none"
                    window.dragWaypointIndex = -1
                    window.routeChangedByDrag = false
                    window.pendingPointHit = null
                    window.pinchGestureActive = false
                    window.touchInteractionConsumed = false
                    window.endStaticInteractionPreview()
                }
            }

            TapHandler {
                id: touchWaypointTap
                enabled: window.mobileUi
                acceptedDevices: PointerDevice.TouchScreen
                gesturePolicy: TapHandler.WithinBounds
                onLongPressed: {
                    if (window.pinchGestureActive)
                        return

                    window.touchInteractionConsumed = true
                    window.selectWaypointAtScreen(point.position.x, point.position.y)
                }
            }

            PinchHandler {
                id: touchZoomPinch
                enabled: window.mobileUi
                target: null
                acceptedDevices: PointerDevice.TouchScreen | PointerDevice.TouchPad
                minimumPointCount: 2
                maximumPointCount: 2
                grabPermissions: PointerHandler.CanTakeOverFromItems
                                 | PointerHandler.CanTakeOverFromHandlersOfDifferentType
                                 | PointerHandler.ApprovesTakeOverByAnything

                onActiveChanged: {
                    if (active) {
                        window.touchInteractionConsumed = true
                        window.pinchGestureActive = true
                        window.lastPinchScale = 1
                        window.pendingPointHit = null
                        window.dragMode = "pinch"
                        window.beginStaticInteractionPreview()
                    } else {
                        window.pinchGestureActive = false
                        window.dragMode = "none"
                        window.lastPinchScale = 1
                        window.mouseMovedBeyondClick = false
                        window.endStaticInteractionPreview()
                    }
                }

                onActiveScaleChanged: {
                    if (!active)
                        return

                    const deltaScale = activeScale / Math.max(0.001, window.lastPinchScale)
                    if (deltaScale > 0 && deltaScale < 20)
                        window.zoomCameraByFactor(1 / deltaScale)
                    window.lastPinchScale = activeScale
                }
            }
        }
    }

    Drawer {
        id: mobileControlDrawer
        enabled: window.mobileUi
        edge: Qt.BottomEdge
        width: window.width
        height: Math.min(window.height * (window.mobileLandscape ? 0.92 : 0.82),
                         window.mobileLandscape ? 460 : 560)
        modal: true
        dim: true
        interactive: false
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        onOpened: window.controlsPanelOpen = true
        onClosed: window.controlsPanelOpen = false

        background: Rectangle {
            color: "#202125"
            border.color: "#383940"
        }

        contentItem: ColumnLayout {
            spacing: 0

            ScrollView {
                id: mobileDrawerScroll
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                leftPadding: 14
                rightPadding: 14
                topPadding: 8
                bottomPadding: 14
                contentWidth: availableWidth
                ScrollBar.horizontal.policy: ScrollBar.AlwaysOff

                ColumnLayout {
                    width: mobileDrawerScroll.availableWidth
                spacing: 12

                RowLayout {
                    Layout.fillWidth: true
                    Layout.topMargin: 8

                    Label {
                        Layout.fillWidth: true
                        text: qsTr("地面站操作")
                        color: "#eeeef1"
                        font.pixelSize: 18
                        font.weight: Font.DemiBold
                    }

                    ActionButton {
                        Layout.preferredWidth: 76
                        text: qsTr("关闭")
                        onClicked: mobileControlDrawer.close()
                    }
                }

                Label {
                    Layout.fillWidth: true
                    text: window.operatorMessage
                    color: "#b2b3bc"
                    wrapMode: Text.WordWrap
                    maximumLineCount: 3
                    elide: Text.ElideRight
                }

                GridLayout {
                    id: mobileSectionGrid
                    Layout.fillWidth: true
                    columns: window.mobileLandscape ? 2 : 1
                    columnSpacing: 12
                    rowSpacing: 12

                SectionBox {
                    Layout.fillWidth: true
                    Layout.preferredWidth: window.mobileLandscape
                                           ? Math.max(260, (mobileDrawerScroll.availableWidth
                                                            - mobileSectionGrid.columnSpacing) / 2)
                                           : mobileDrawerScroll.availableWidth
                    title: qsTr("快速操作")

                    ColumnLayout {
                        anchors.fill: parent
                        spacing: 10

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 10

                            ActionButton {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 44
                                enabled: !staticPointLoader.loading
                                text: qsTr("上传模型")
                                onClicked: {
                                    mobileControlDrawer.close()
                                    staticPointDialog.open()
                                }
                            }

                            ActionButton {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 44
                                text: pointReceiver.active ? qsTr("重启监听") : qsTr("启动监听")
                                onClicked: {
                                    pointReceiver.active = true
                                    pointReceiver.restart()
                                }
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 10

                            ActionButton {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 44
                                text: qsTr("香橙派 UDP")
                                onClicked: {
                                    pointPortSpin.value = 9000
                                    routeHostField.text = "192.168.1.196"
                                    routePortSpin.value = 9002
                                    routeProtocolBox.currentIndex = 1
                                    pointReceiver.active = true
                                    pointReceiver.restart()
                                    poseReceiver.active = true
                                    poseReceiver.restart()
                                    window.operatorMessage = qsTr("已切换到香橙派 UDP。")
                                }
                            }

                            ActionButton {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 44
                                enabled: !staticPointLoader.loading
                                text: qsTr("实时点云")
                                onClicked: {
                                    window.lvxPlaybackPlaying = false
                                    pointReceiver.active = true
                                    pointReceiver.restart()
                                    routeGeometry.clearRoute()
                                    window.operatorMessage = qsTr("已恢复实时 UDP 点云。")
                                }
                            }
                        }
                    }
                }

                SectionBox {
                    Layout.fillWidth: true
                    Layout.preferredWidth: window.mobileLandscape
                                           ? Math.max(260, (mobileDrawerScroll.availableWidth
                                                            - mobileSectionGrid.columnSpacing) / 2)
                                           : mobileDrawerScroll.availableWidth
                    title: qsTr("UDP")

                    ColumnLayout {
                        anchors.fill: parent
                        spacing: 10

                        RowLayout {
                            Layout.fillWidth: true
                            Label {
                                Layout.fillWidth: true
                                text: qsTr("点云端口")
                                color: "#bdbec6"
                            }
                            SpinBox {
                                Layout.preferredWidth: 136
                                from: 1
                                to: 65535
                                value: pointPortSpin.value
                                editable: true
                                onValueModified: pointPortSpin.value = value
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Label {
                                Layout.fillWidth: true
                                text: qsTr("航线目标")
                                color: "#bdbec6"
                            }
                            TextField {
                                Layout.preferredWidth: 170
                                text: routeHostField.text
                                selectByMouse: true
                                onEditingFinished: routeHostField.text = text
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Label {
                                Layout.fillWidth: true
                                text: qsTr("航线端口")
                                color: "#bdbec6"
                            }
                            SpinBox {
                                Layout.preferredWidth: 136
                                from: 1
                                to: 65535
                                value: routePortSpin.value
                                editable: true
                                onValueModified: routePortSpin.value = value
                            }
                        }

                        ComboBox {
                            Layout.fillWidth: true
                            model: routeProtocolBox.model
                            currentIndex: routeProtocolBox.currentIndex
                            onActivated: routeProtocolBox.currentIndex = currentIndex
                        }
                    }
                }

                SectionBox {
                    Layout.fillWidth: true
                    Layout.preferredWidth: window.mobileLandscape
                                           ? Math.max(260, (mobileDrawerScroll.availableWidth
                                                            - mobileSectionGrid.columnSpacing) / 2)
                                           : mobileDrawerScroll.availableWidth
                    title: qsTr("显示与规划")

                    ColumnLayout {
                        anchors.fill: parent
                        spacing: 10

                        StyledCheckBox {
                            Layout.fillWidth: true
                            text: qsTr("显示 XYZ 轴")
                            checked: window.showAxisHelper
                            onClicked: window.showAxisHelper = checked
                        }

                        StyledCheckBox {
                            Layout.fillWidth: true
                            enabled: !staticPointLoader.loading
                            text: qsTr("加载全部点")
                            checked: window.loadAllStaticPoints
                            onClicked: window.loadAllStaticPoints = checked
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Label {
                                Layout.fillWidth: true
                                text: qsTr("航点高度")
                                color: "#bdbec6"
                            }
                            SpinBox {
                                Layout.preferredWidth: 136
                                from: 30
                                to: 300
                                stepSize: 5
                                value: window.planningAltitudeCentimeters
                                textFromValue: function(value) { return qsTr("%1 cm").arg(value) }
                                valueFromText: function(text) {
                                    const value = Number(String(text).replace(/[^\d.]/g, ""))
                                    return isNaN(value) ? window.planningAltitudeCentimeters : Math.round(value)
                                }
                                onValueModified: window.planningAltitudeCentimeters = value
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Label {
                                Layout.fillWidth: true
                                text: qsTr("安全半径")
                                color: "#bdbec6"
                            }
                            SpinBox {
                                Layout.preferredWidth: 136
                                from: 10
                                to: 200
                                stepSize: 5
                                value: window.obstacleClearanceCentimeters
                                textFromValue: function(value) { return qsTr("%1 cm").arg(value) }
                                valueFromText: function(text) {
                                    const value = Number(String(text).replace(/[^\d.]/g, ""))
                                    return isNaN(value) ? window.obstacleClearanceCentimeters : Math.round(value)
                                }
                                onValueModified: window.obstacleClearanceCentimeters = value
                            }
                        }
                    }
                }

                SectionBox {
                    Layout.fillWidth: true
                    Layout.preferredWidth: window.mobileLandscape
                                           ? Math.max(260, (mobileDrawerScroll.availableWidth
                                                            - mobileSectionGrid.columnSpacing) / 2)
                                           : mobileDrawerScroll.availableWidth
                    title: qsTr("航线")

                    ColumnLayout {
                        anchors.fill: parent
                        spacing: 10

                        InfoRow {
                            label: qsTr("航点数")
                            value: routeGeometry.waypointCount.toString()
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 10

                            ActionButton {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 44
                                text: qsTr("清空航线")
                                onClicked: {
                                    routeGeometry.clearRoute()
                                    window.operatorMessage = qsTr("航线已清空。长按空旷区域重新选点。")
                                }
                            }

                            ActionButton {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 44
                                enabled: routeGeometry.valid
                                text: qsTr("上传航线")
                                onClicked: {
                                    const conflict = window.routeObstacleConflict()
                                    if (conflict.blocked) {
                                        window.operatorMessage = qsTr("第 %1-%2 段航线距离障碍物过近，请先调整。")
                                                .arg(conflict.segmentIndex + 1)
                                                .arg(conflict.segmentIndex + 2)
                                        return
                                    }

                                    const sent = routeSender.sendRoute(routeGeometry)
                                    window.operatorMessage = sent
                                            ? qsTr("航线已上传。")
                                            : qsTr("航线上传失败，请检查端口和日志。")
                                }
                            }
                        }

                        Label {
                            Layout.fillWidth: true
                            visible: routeGeometry.waypointCount > 0
                            text: qsTr("单航点发送")
                            color: "#b8bac4"
                            font.pixelSize: 12
                            font.weight: Font.DemiBold
                        }

                        ColumnLayout {
                            Layout.fillWidth: true
                            visible: routeGeometry.waypointCount > 0
                            spacing: 6

                            Repeater {
                                model: routeGeometry.waypoints

                                delegate: RowLayout {
                                    required property int index
                                    required property var modelData

                                    Layout.fillWidth: true
                                    spacing: 8

                                    Label {
                                        Layout.fillWidth: true
                                        color: "#d2d3da"
                                        font.family: "monospace"
                                        font.pixelSize: 11
                                        elide: Text.ElideRight
                                        text: qsTr("%1  X:%2  Y:%3  Z:%4")
                                              .arg(index + 1)
                                              .arg(Number(modelData.x).toFixed(2))
                                              .arg(Number(modelData.y).toFixed(2))
                                              .arg(Number(modelData.z).toFixed(2))
                                    }

                                    ActionButton {
                                        Layout.preferredWidth: 58
                                        Layout.preferredHeight: 30
                                        Layout.minimumHeight: 30
                                        text: qsTr("发送")
                                        onClicked: {
                                            const check = window.waypointObstacleConflict(modelData)
                                            if (check.blocked) {
                                                console.warn("移动端单航点发送被避障校验拦截，index="
                                                             + index
                                                             + "，distance=" + check.distance)
                                                window.operatorMessage = qsTr("第 %1 个航点距离障碍物过近，请先拖动避开。")
                                                        .arg(index + 1)
                                                return
                                            }

                                            const sent = routeSender.sendSingleWaypoint(modelData)
                                            window.operatorMessage = sent
                                                    ? qsTr("已上传第 %1 个单航点。").arg(index + 1)
                                                    : qsTr("单航点上传失败，请检查目标端口和日志。")
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                }
            }
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 64
                color: "#202125"
                border.color: "#383940"

                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 10

                    Label {
                        Layout.fillWidth: true
                        text: routeGeometry.valid
                              ? qsTr("航线可上传，航点数 %1").arg(routeGeometry.waypointCount)
                              : qsTr("长按空旷区域添加航点")
                        color: "#b2b3bc"
                        elide: Text.ElideRight
                    }

                    ActionButton {
                        Layout.preferredWidth: 112
                        Layout.preferredHeight: 42
                        text: qsTr("关闭面板")
                        onClicked: mobileControlDrawer.close()
                    }
                }
            }
        }
    }

    component InfoRow: RowLayout {
        property string label
        property string value

        Layout.fillWidth: true
        Label {
            Layout.fillWidth: true
            text: label
            color: "#bdbec6"
        }
        Label {
            text: value
            color: "#eeeef1"
            font.family: "monospace"
        }
    }

    component SectionBox: GroupBox {
        id: section
        property bool collapsed: true
        Layout.fillWidth: true
        implicitHeight: collapsed ? 60 : implicitContentHeight + topPadding + bottomPadding
        contentItem.visible: !collapsed
        padding: 12
        topPadding: 56
        label: SectionHeader {
            x: 12
            y: 12
            width: Math.max(0, section.width - 24)
            height: 36
            title: section.title
            collapsed: section.collapsed
            onClicked: section.collapsed = !section.collapsed
        }
        background: Rectangle {
            radius: 6
            color: "#202125"
            border.color: "#383940"
        }
    }

    component ActionButton: Button {
        id: control

        Layout.minimumHeight: 38
        font.pixelSize: 12
        font.weight: Font.Medium

        contentItem: Label {
            text: control.text
            color: control.enabled ? "#eeeef1" : "#777983"
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            elide: Text.ElideRight
        }

        background: Rectangle {
            radius: 7
            color: !control.enabled ? "#25262b"
                  : control.down ? "#2d7cdb"
                  : control.hovered ? "#35363e" : "#292a30"
            border.color: !control.enabled ? "#34353b"
                         : control.hovered || control.down ? "#38bdf8" : "#393a42"
        }
    }

    component StyledCheckBox: CheckBox {
        id: control

        Layout.minimumHeight: 30

        indicator: Rectangle {
            implicitWidth: 18
            implicitHeight: 18
            x: 0
            y: (control.height - height) / 2
            radius: 4
            color: control.checked ? "#2d7cdb" : "#202125"
            border.color: control.checked ? "#79a9e8" : "#50515b"

            Rectangle {
                anchors.centerIn: parent
                width: 9
                height: 9
                radius: 2
                visible: control.checked
                color: "#eeeef1"
            }
        }

        contentItem: Label {
            text: control.text
            leftPadding: control.indicator.width + 10
            color: control.enabled ? "#dadbe0" : "#777983"
            verticalAlignment: Text.AlignVCenter
        }
    }

    Component.onCompleted: {
        if (window.plannerMode) missionController.watch()
        if (autoLoadPointCloudUrl.toString().length > 0)
            loadStaticPointCloud(autoLoadPointCloudUrl)
    }

    function loadStaticPointCloud(fileUrl) {
        lvxPlaybackPlaying = false
        console.info("请求加载静态点云模型：" + fileUrl
                     + "，loadAll=" + loadAllStaticPoints
                     + "，limit=" + staticPointLoader.maxPoints)
        if (staticPointLoader.loadFromUrl(fileUrl)) {
            pointReceiver.active = false
            routeGeometry.clearRoute()
            operatorMessage = qsTr("正在后台加载静态点云模型，界面仍可拖动查看。")
        } else {
            console.warn("静态点云加载请求被拒绝：" + fileUrl)
        }
    }

    function beginStaticInteractionPreview() {
        if (pointReceiver.active || staticPointLoader.loading)
            return

        staticPreviewRestoreTimer.stop()
        staticPointLoader.setInteractivePreviewEnabled(true)
    }

    function endStaticInteractionPreview() {
        if (pointReceiver.active || staticPointLoader.loading)
            return

        staticPreviewRestoreTimer.restart()
    }

    function formatPlaybackTime(ms) {
        const safeMs = Math.max(0, Math.round(ms))
        const minutes = Math.floor(safeMs / 60000)
        const seconds = Math.floor((safeMs % 60000) / 1000)
        const centiseconds = Math.floor((safeMs % 1000) / 10)
        const secondText = seconds < 10 ? "0" + seconds : seconds.toString()
        const centisecondText = centiseconds < 10 ? "0" + centiseconds : centiseconds.toString()
        return minutes.toString() + ":" + secondText + "." + centisecondText
    }

    function rotateCamera(deltaX, deltaY) {
        cameraYaw -= deltaX * 0.22
        cameraPitch = Math.max(-82, Math.min(28, cameraPitch - deltaY * 0.18))
    }

    function panCamera(deltaX,deltaY) {
        if(!isFinite(deltaX) || !isFinite(deltaY) || v3d.width<=0 || v3d.height<=0)return
        // 在观察中心的屏幕平面上求位移，只移动相机支点，地图/目标坐标不变。
        // 复用当前投影，自动适应旋转角度、缩放和窗口尺寸。
        const cx=v3d.width/2
        const cy=v3d.height/2
        const before=v3d.mapTo3DScene(Qt.vector3d(cx,cy,cameraNode.z))
        const after=v3d.mapTo3DScene(Qt.vector3d(cx+deltaX,cy+deltaY,cameraNode.z))
        const dx=before.x-after.x
        const dy=before.y-after.y
        const dz=before.z-after.z
        if(!isFinite(dx) || !isFinite(dy) || !isFinite(dz))return
        cameraFocus=Qt.vector3d(cameraFocus.x+dx,cameraFocus.y+dy,cameraFocus.z+dz)
    }

    function resetCamera() {
        cameraFocus=Qt.vector3d(0,0,0)
        cameraPitch=-25
        cameraYaw=0
        cameraNode.z=920
    }

    function zoomCamera(angleDeltaY) {
        const factor = angleDeltaY > 0 ? 0.88 : 1.14
        zoomCameraByFactor(factor)
    }

    function zoomCameraByFactor(factor) {
        cameraNode.z = Math.max(180, Math.min(2600, cameraNode.z * factor))
    }

    function metersToScene(pointMeters) {
        return Qt.vector3d(-pointMeters.y * window.fixedSceneScale,
                           pointMeters.z * window.fixedSceneScale,
                           -pointMeters.x * window.fixedSceneScale)
    }

    function waypointObstacleConflict(waypoint) {
        if (pointCloudGeometry.pointCount <= 0)
            return { "blocked": false, "distance": 0 }

        return pointCloudGeometry.nearestObstacleAround(
                    waypoint,
                    window.obstacleClearanceCentimeters / 100,
                    window.obstacleVerticalClearanceCentimeters / 100)
    }

    function routeObstacleConflict() {
        if (pointCloudGeometry.pointCount <= 0 || routeGeometry.waypointCount < 2)
            return { "blocked": false, "segmentIndex": -1, "distance": 0 }

        const waypoints = routeGeometry.waypoints
        for (let i = 0; i < waypoints.length - 1; ++i) {
            const check = pointCloudGeometry.nearestObstacleOnSegment(
                        waypoints[i],
                        waypoints[i + 1],
                        window.obstacleClearanceCentimeters / 100,
                        window.obstacleVerticalClearanceCentimeters / 100)
            if (check.blocked) {
                return {
                    "blocked": true,
                    "segmentIndex": i,
                    "distance": check.distance,
                    "routePosition": check.routePosition,
                    "obstaclePosition": check.position
                }
            }
        }

        return { "blocked": false, "segmentIndex": -1, "distance": 0 }
    }

    function updatePlannerDisplay() {
        planning=flightMonitor.planning
        plannerController.observe(planning,flightMonitor.fresh)
        const s=planning.status || ({})
        if(s.session_id && s.session_id!==pathSession){
            pathSession=s.session_id;actualHistory=[];actualPath.clearRoute()
        }
        const points=[]
        if(!plannerController.previewSuppressed && planning.path_valid)for(const p of planning.path)points.push(Qt.vector3d(p[0],p[1],p[2]))
        nativePath.waypoints=points
        const candidates=[]
        if(!plannerController.previewSuppressed && planning.candidate_path_valid)for(const p of planning.candidate_path)candidates.push(Qt.vector3d(p[0],p[1],p[2]))
        candidatePath.waypoints=candidates
    }

    function selectMissionWaypoint(x,y) {
        if(!missionControls.picking || missionControls.routeLocked)return
        const scene=scenePointOnHeightPlane(x,y,missionControls.pickHeight*fixedSceneScale)
        if(scene===null){operatorMessage=qsTr("无法投影到航点高度平面，请调整视角。");return}
        missionControls.addMapWaypoint(routeGeometry.fromScenePosition(scene))
    }

    function selectPlannerGoal(x,y) {
        if(!plannerPanel.pickGoal || !plannerController.canPreview)return
        const selection=plannerPanel.resolveGoalHeight()
        if(!selection.valid){operatorMessage=selection.message;return}
        const height=selection.height
        const scene=scenePointOnHeightPlane(x,y,height*fixedSceneScale)
        if(scene===null){operatorMessage=qsTr("无法投影到目标高度平面，请调整视角。");return}
        const p=routeGeometry.fromScenePosition(scene)
        // 只发精确候选点；原生地图负责占据/膨胀与规划检查，前端不挪目标。
        plannerPanel.pickGoal=false
        console.info("目标选点",JSON.stringify({pixel:[x,y],viewport:[v3d.width,v3d.height],cameraYaw:cameraYaw,cameraPitch:cameraPitch,cameraFocus:[cameraFocus.x,cameraFocus.y,cameraFocus.z],pose:[poseReceiver.positionMeters.x,poseReceiver.positionMeters.y,poseReceiver.positionMeters.z],goal:[p.x,p.y,height],frame:"world/camera_init",height_reference:"fastlio_session_origin"}))
        plannerController.preview(Qt.vector3d(p.x,p.y,height))
        operatorMessage=""
    }

    function plannedWaypointFromScreen(x, y) {
        if (pointCloudGeometry.pointCount <= 0) {
            return {
                "valid": false,
                "message": qsTr("暂无点云障碍地图，无法校验航点安全。")
            }
        }

        const heightMeters = window.planningAltitudeCentimeters / 100
        const scenePoint = window.scenePointOnHeightPlane(x, y, heightMeters * window.fixedSceneScale)
        if (scenePoint === null) {
            return {
                "valid": false,
                "message": qsTr("当前视角无法投影到航点高度平面，请调整视角后再选。")
            }
        }

        const rawPoint = routeGeometry.fromScenePosition(scenePoint)
        const waypoint = Qt.vector3d(rawPoint.x, rawPoint.y, heightMeters)
        const check = pointCloudGeometry.nearestObstacleAround(
                    waypoint,
                    window.obstacleClearanceCentimeters / 100,
                    window.obstacleVerticalClearanceCentimeters / 100)
        if (check.blocked) {
            return {
                "valid": false,
                "blocked": true,
                "position": waypoint,
                "distance": check.distance,
                "message": qsTr("该航点距离障碍物过近，请选择更空旷的位置。")
            }
        }

        return {
            "valid": true,
            "position": waypoint,
            "distance": check.distance
        }
    }

    function selectPointFromPendingHit() {
        if (routeGeometry.waypointCount >= 2) {
            operatorMessage = qsTr("航线已生成。拖动航点或线段调整，清空后可重新选点。")
            return
        }

        if (pendingPointHit === null || !pendingPointHit.valid) {
            console.warn("用户短按自由空间选点失败，当前点数=" + pointCloudGeometry.pointCount)
            operatorMessage = pendingPointHit !== null && pendingPointHit.message
                    ? pendingPointHit.message
                    : qsTr("未生成安全航点，请调整视角后重新点击。")
            return
        }

        routeGeometry.addWaypoint(pendingPointHit.position)
        console.info("用户选中自由空间航点 #" + routeGeometry.waypointCount
                     + "：X=" + Number(pendingPointHit.position.x).toFixed(3)
                     + " Y=" + Number(pendingPointHit.position.y).toFixed(3)
                     + " Z=" + Number(pendingPointHit.position.z).toFixed(3))
        if (routeGeometry.waypointCount === 1) {
            operatorMessage = qsTr("起点已选中，请继续点击终点。")
        } else if (routeGeometry.waypointCount === 2) {
            operatorMessage = qsTr("航线已生成。请拖动航点或线段避障，可上传整条航线或单个航点。")
        }
    }

    function selectWaypointAtScreen(x, y) {
        if (routeGeometry.waypointCount >= 2) {
            operatorMessage = qsTr("航线已生成。拖动航点或线段调整，清空后可重新选点。")
            return
        }

        pendingPointHit = plannedWaypointFromScreen(x, y)
        selectPointFromPendingHit()
        pendingPointHit = null
    }

    function viewRay(x, y) {
        const nearPoint = v3d.mapTo3DScene(Qt.vector3d(x, y, 0))
        const farPoint = v3d.mapTo3DScene(Qt.vector3d(x, y, cameraNode.clipFar))
        const direction = Qt.vector3d(farPoint.x - nearPoint.x,
                                      farPoint.y - nearPoint.y,
                                      farPoint.z - nearPoint.z).normalized()
        return { "origin": nearPoint, "direction": direction }
    }

    function scenePointOnHeightPlane(x, y, heightSceneY) {
        const ray = viewRay(x, y)
        if (Math.abs(ray.direction.y) < 0.0001)
            return null

        const t = (heightSceneY - ray.origin.y) / ray.direction.y
        if (t < 0)
            return null

        return Qt.vector3d(ray.origin.x + ray.direction.x * t,
                           heightSceneY,
                           ray.origin.z + ray.direction.z * t)
    }

    function nearestRouteHandle(x, y) {
        const waypoints = routeGeometry.sceneWaypoints
        let bestIndex = -1
        let bestDistance = window.routeHandleHitRadiusPx
        for (let i = 0; i < waypoints.length; ++i) {
            const screenPoint = v3d.mapFrom3DScene(waypoints[i])
            const dx = screenPoint.x - x
            const dy = screenPoint.y - y
            const distance = Math.sqrt(dx * dx + dy * dy)
            if (distance < bestDistance) {
                bestDistance = distance
                bestIndex = i
            }
        }
        return bestIndex
    }

    function nearestRouteSegment(x, y) {
        const waypoints = routeGeometry.sceneWaypoints
        let best = { "index": -1, "distance": window.routeSegmentHitRadiusPx, "planeY": 0 }
        if (waypoints.length < 2)
            return best

        for (let i = 0; i < waypoints.length - 1; ++i) {
            const a = v3d.mapFrom3DScene(waypoints[i])
            const b = v3d.mapFrom3DScene(waypoints[i + 1])
            const distance = screenDistanceToSegment(x, y, a.x, a.y, b.x, b.y)
            if (distance < best.distance) {
                best.index = i
                best.distance = distance
                best.planeY = (waypoints[i].y + waypoints[i + 1].y) * 0.5
            }
        }
        return best
    }

    function screenDistanceToSegment(px, py, ax, ay, bx, by) {
        const abx = bx - ax
        const aby = by - ay
        const apx = px - ax
        const apy = py - ay
        const ab2 = abx * abx + aby * aby
        const t = ab2 <= 0.0001 ? 0 : Math.max(0, Math.min(1, (apx * abx + apy * aby) / ab2))
        const cx = ax + abx * t
        const cy = ay + aby * t
        const dx = px - cx
        const dy = py - cy
        return Math.sqrt(dx * dx + dy * dy)
    }
}
