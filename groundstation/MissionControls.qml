import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

ColumnLayout {
    id: panel
    required property var controller
    property bool waypointVisible: true
    property bool picking: false
    property real pickHeight: 1.0
    signal pickRequested()
    readonly property bool routeLocked: panel.controller.busy || !!panel.controller.state.running
    onRouteLockedChanged: if(routeLocked) picking=false
    function routePositions() {
        const result=[]
        for(const p of panel.controller.waypoints) result.push(Qt.vector3d(p.x,p.y,p.z))
        return result
    }
    function addMapWaypoint(p) { panel.controller.addWaypoint(p.x,p.y,pickHeight) }

    Layout.fillWidth: true
    spacing: 12
    readonly property var arucoTask: panel.controller.state.aruco || ({})
    readonly property var trackingTask: panel.controller.state.tracking || ({})
    readonly property bool taskStopping: !!panel.controller.state.stop_pending
    function taskRunning(kind) { return panel.controller.fresh && panel.controller.state.running && panel.controller.state.task === kind }
    function taskCanStop(kind) { return !panel.controller.busy && !!panel.controller.state.running && panel.controller.state.task === kind && !taskStopping }
    function arucoOffset(axis) { const o = arucoTask.offset || ({}); return panel.controller.fresh && arucoTask.valid && o[axis] !== undefined ? Number(o[axis]).toFixed(3) + " m" : "—" }
    function arucoReason(reason) {
        const names = {
            rc_switch_out: qsTr("遥控已拨出 OFFBOARD"), rc_channel_missing: qsTr("遥控通道缺失"),
            rc_pwm_invalid: qsTr("遥控 PWM 无效"), rc_rssi_zero: qsTr("遥控 RSSI 为零"),
            rc_stamp_missing: qsTr("遥控时间戳缺失"), rc_stamp_future: qsTr("遥控时间戳超前"),
            rc_stamp_stale: qsTr("遥控源数据过期"), rc_receive_timeout: qsTr("遥控接收超时"),
            marker_source_stale: qsTr("识别源数据过期"), marker_pose_invalid: qsTr("识别位姿无效"),
            detector_stamp_unavailable: qsTr("识别节点缺少同帧时间信息"), detector_pose_not_synchronized: qsTr("识别数据不同步"),
            marker_unavailable: qsTr("等待识别结果"), no_configured_id: qsTr("未识别到配置的标记"),
            resolution_mismatch: qsTr("图像分辨率与标定不符"), selected_pose_valid: qsTr("已识别目标"),
            accepted: qsTr("控制器已接受目标"), source_stamp_stale: qsTr("控制目标源数据过期"),
            pose_image_not_synchronized: qsTr("图像与定位未同步"), target_jump_rejected: qsTr("目标跳变被拒绝"),
            manual_mode_requires_airborne_and_armed: qsTr("触发被拒绝：需先人工起飞")
        }
        return names[reason] ? names[reason] + " · " + reason : (reason || "—")
    }
    function arucoDetectionStatus() {
        if (!panel.controller.fresh) return qsTr("识别监视已过期")
        const text=arucoReason(arucoTask.reason || (arucoTask.detector || {}).reason)
        return text + (arucoTask.valid && !arucoTask.source_stamp_known ? qsTr(" · 采集时间未知") : "")
    }
    function taskStatus(kind) {
        if (!panel.controller.fresh) return qsTr("任务监视未连接 / 已过期")
        if (panel.controller.state.task !== kind) return qsTr("未准备")
        if (!panel.controller.state.running) return panel.controller.state.message || qsTr("任务已退出")
        if (taskStopping) return qsTr("停止处理中；请遥控接管")
        const data = kind === "waypoint" ? (panel.controller.state.waypoint || ({})) : kind === "aruco" ? (arucoTask.landing || ({})) : ({})
        if (kind === "aruco" && arucoTask.landing_fresh === false) return qsTr("降落控制状态未收到 / 已过期")
        if (kind === "aruco" && data.start_reject_reason) return (data.phase || "IDLE") + " · " + arucoReason(data.start_reject_reason)
        const phase = data.phase || ""
        return phase.length ? phase + (data.wp ? " · " + data.wp : "") + (data.reason ? " · " + (kind === "aruco" ? arucoReason(data.reason) : data.reason) : "") : qsTr("任务进程运行中，等待遥控操作")
    }
                SectionBox {
                    objectName: "waypointPlanningSection"
                    title: qsTr("航点规划")
                    visible: panel.waypointVisible
                    onCollapsedChanged: if (!collapsed) panel.controller.watch()
                    ColumnLayout {
                        anchors.fill: parent
                        spacing: 8
                        Label { Layout.fillWidth: true; text: qsTr("逐点设置高度和拍照；每点停留 2 秒，拍照朝下 90°。"); wrapMode: Text.WordWrap; color: "#93959f"; font.pixelSize: 11 }
                        RowLayout {
                            Layout.fillWidth: true
                            Label { text: qsTr("选点高度(m)"); color: "#93959f" }
                            TextField {
                                objectName: "mapPickHeight"
                                Layout.fillWidth: true; text: "1.00"; enabled: !panel.routeLocked; selectByMouse: true
                                onTextEdited: { const z=Number(text); if(text.trim().length && isFinite(z))panel.pickHeight=z }
                                onEditingFinished: { const z=Number(text); if(text.trim().length && isFinite(z))panel.pickHeight=z; else text=panel.pickHeight.toFixed(2) }
                            }
                        }
                        RowLayout {
                            Layout.fillWidth: true
                            StudioButton {
                                objectName: "pickMissionWaypoints"; Layout.fillWidth: true; tone: "primary"
                                text: panel.picking ? qsTr("完成地图选点") : qsTr("地图添加航点")
                                enabled: !panel.routeLocked
                                onClicked: { panel.picking=!panel.picking; if(panel.picking)panel.pickRequested() }
                            }
                            StudioButton { text: qsTr("清空"); enabled: !panel.routeLocked && panel.controller.waypoints.length>0; onClicked: panel.controller.clearWaypoints() }
                        }
                        Label { visible: panel.picking; Layout.fillWidth: true; text: qsTr("点击地图按顺序添加；拖动可调整视角。"); wrapMode: Text.WordWrap; color: "#dbad48" }
                        Repeater {
                            model: panel.controller.waypoints
                            delegate: ColumnLayout {
                                id: row
                                required property int index
                                required property var modelData
                                Layout.fillWidth: true; spacing: 3
                                Label { text: qsTr("航点 %1 · X %2 / Y %3 m").arg(row.index+1).arg(Number(row.modelData.x).toFixed(2)).arg(Number(row.modelData.y).toFixed(2)); color: "#dadbe0"; font.pixelSize: 11 }
                                RowLayout {
                                    Layout.fillWidth: true
                                    Label { text: qsTr("高度"); color: "#93959f" }
                                    TextField {
                                        objectName: "waypointHeight_"+row.index
                                        Layout.fillWidth: true; text: Number(row.modelData.z).toFixed(2); selectByMouse: true; enabled: !panel.routeLocked
                                        onEditingFinished: { const z=Number(text); if(text.trim().length && isFinite(z))panel.controller.editWaypoint(row.index,z,!!row.modelData.photo); else text=Number(row.modelData.z).toFixed(2) }
                                    }
                                    CheckBox { objectName: "waypointPhoto_"+row.index; text: qsTr("拍照"); checked: !!row.modelData.photo; enabled: !panel.routeLocked; onToggled: panel.controller.editWaypoint(row.index,row.modelData.z,checked) }
                                    StudioButton { text: "×"; enabled: !panel.routeLocked; onClicked: panel.controller.removeWaypoint(row.index) }
                                }
                            }
                        }
                        RowLayout {
                            Layout.fillWidth: true
                            StudioButton { objectName: "prepareWaypointTask"; Layout.fillWidth: true; tone: "primary"; text: qsTr("航点任务执行"); enabled: !panel.routeLocked && panel.controller.waypoints.length>0; onClicked: { panel.forceActiveFocus(); panel.picking=false; panel.controller.executeMapMission() } }
                            StudioButton { objectName: "stopWaypointTask"; text: qsTr("停止"); tone: "danger"; enabled: panel.taskCanStop("waypoint"); onClicked: panel.controller.stopTask() }
                        }
                        Label { Layout.fillWidth: true; text: panel.taskStatus("waypoint"); wrapMode: Text.WordWrap; color: panel.taskRunning("waypoint") ? "#3cbb83" : "#93959f" }
                        Label { Layout.fillWidth: true; text: qsTr("地面锁桨 → 执行按钮准备 → 遥控切 OFFBOARD 后自动解锁起飞至第一个航点高度，再执行航线；最后航点完成后返回起飞点并自动降落。无 Ego 避障，高度相对定位原点。"); wrapMode: Text.WordWrap; font.pixelSize: 11; color: "#dbad48" }
                    }
                }

                SectionBox {
                    objectName: "visualControlSection"
                    title: qsTr("视觉控制")
                    onCollapsedChanged: if (!collapsed) panel.controller.watch()
                    ColumnLayout {
                        anchors.fill: parent
                        spacing: 12
                        SectionBox {
                            objectName: "arucoLandingSection"
                            title: qsTr("ArUco 码识别降落")
                            onCollapsedChanged: if (!collapsed) panel.controller.watch()
                            ColumnLayout {
                                anchors.fill: parent
                                spacing: 8
                                InfoRow { objectName: "arucoOffsetX"; label: qsTr("X 偏移量"); value: panel.arucoOffset("x") }
                                InfoRow { objectName: "arucoOffsetY"; label: qsTr("Y 偏移量"); value: panel.arucoOffset("y") }
                                InfoRow { objectName: "arucoMarkerId"; label: qsTr("ID"); value: panel.controller.fresh && panel.arucoTask.valid ? String(panel.arucoTask.id) : "—" }
                                Label { objectName: "arucoDetectionStatus"; Layout.fillWidth: true; text: panel.arucoDetectionStatus(); wrapMode: Text.WordWrap; color: "#93959f"; font.pixelSize: 11 }
                                Label { objectName: "arucoControlTargetStatus"; Layout.fillWidth: true; text: panel.controller.fresh && panel.arucoTask.landing_fresh ? qsTr("控制目标：%1").arg(panel.arucoReason((panel.arucoTask.landing || {}).marker_reason)) : qsTr("控制目标状态未知"); wrapMode: Text.WordWrap; color: "#93959f"; font.pixelSize: 11 }
                                RowLayout {
                                    Layout.fillWidth: true
                                    StudioButton { objectName: "prepareArucoTask"; Layout.fillWidth: true; tone: "primary"; text: qsTr("准备精准降落"); enabled: !panel.controller.busy && !panel.taskStopping; onClicked: panel.controller.prepare("aruco") }
                                    StudioButton { objectName: "stopArucoTask"; text: qsTr("停止"); tone: "danger"; enabled: panel.taskCanStop("aruco"); onClicked: panel.controller.stopTask() }
                                }
                                Label { objectName: "arucoTaskStatus"; Layout.fillWidth: true; text: panel.taskStatus("aruco"); wrapMode: Text.WordWrap; color: (panel.arucoTask.landing || {}).phase === "CANCELLED" ? "#dbad48" : panel.taskRunning("aruco") ? "#3cbb83" : "#93959f" }
                                Label { objectName: "arucoRcDetail"; Layout.fillWidth: true; visible: text.length > 0; text: panel.controller.fresh && panel.arucoTask.landing_fresh ? ((panel.arucoTask.landing || {}).rc_cancel_detail || "") : ""; wrapMode: Text.WrapAnywhere; color: "#dbad48"; font.pixelSize: 11 }
                                Label { Layout.fillWidth: true; text: qsTr("POSCTL 手动飞至目标上方，准备后云台朝下；遥控切 OFFBOARD 才开始降落。"); wrapMode: Text.WordWrap; font.pixelSize: 11; color: "#93959f" }
                            }
                        }
                        SectionBox {
                            objectName: "personTrackingSection"
                            title: qsTr("人物跟踪")
                            onCollapsedChanged: if (!collapsed) panel.controller.watch()
                            ColumnLayout {
                                anchors.fill: parent
                                spacing: 8
                                InfoRow { label: qsTr("目标 ID"); value: panel.controller.fresh && panel.trackingTask.id !== undefined ? String(panel.trackingTask.id) : "—" }
                                InfoRow { label: qsTr("目标距离"); value: panel.controller.fresh && panel.trackingTask.distance_valid > 0.5 ? Number(panel.trackingTask.distance_m).toFixed(2) + " m" : "—" }
                                RowLayout {
                                    Layout.fillWidth: true
                                    StudioButton { objectName: "prepareTrackingTask"; Layout.fillWidth: true; tone: "primary"; text: qsTr("准备跟踪控制"); enabled: !panel.controller.busy && !panel.taskStopping; onClicked: panel.controller.prepare("tracking") }
                                    StudioButton { objectName: "stopTrackingTask"; text: qsTr("停止"); tone: "danger"; enabled: panel.taskCanStop("tracking"); onClicked: panel.controller.stopTask() }
                                }
                                Label { Layout.fillWidth: true; text: panel.taskStatus("tracking"); wrapMode: Text.WordWrap; color: panel.taskRunning("tracking") ? "#3cbb83" : "#93959f" }
                                Label { Layout.fillWidth: true; text: qsTr("手动起飞并切 OFFBOARD；SWA 中档跟随、上档保护、下档下降。"); wrapMode: Text.WordWrap; font.pixelSize: 11; color: "#93959f" }
                            }
                        }
                    }
                }
                Label {
                    Layout.fillWidth: true
                    visible: panel.controller.message.length > 0
                    text: panel.controller.message
                    wrapMode: Text.WordWrap
                    color: "#dbad48"
                    font.pixelSize: 12
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
            x: 12; y: 12; width: Math.max(0, section.width - 24); height: 36
            title: section.title; collapsed: section.collapsed
            onClicked: section.collapsed = !section.collapsed
        }
        background: Rectangle { radius: 6; color: "#202125"; border.color: "#383940" }
    }
    component InfoRow: RowLayout {
        property string label: ""
        property string value: ""
        Layout.fillWidth: true
        Label { text: parent.label; color: "#93959f"; Layout.fillWidth: true }
        Label { text: parent.value; color: "#dadbe0" }
    }
}
