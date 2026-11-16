import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Rectangle {
    id: panel
    property bool collapsed: true
    required property var controller
    required property var monitor
    required property var planning
    signal pickRequested()
    property bool showGrid: false
    property bool pickGoal: false
    property bool poseFresh: false
    property real currentPoseZ: 0
    readonly property bool heightReady: isFinite(currentPoseZ)
    readonly property var state: planning.status || ({})
    readonly property var execution: planning.execution || ({})
    readonly property bool inFlight: state.planning_mode === "upstream_fsm"
    readonly property var candidate: state.candidate || ({})
    readonly property bool hasCurrentGoal: !!state.has_goal && !!state.goal
        && !controller.previewSuppressed && !controller.sessionInvalid
    readonly property bool hasCandidateTarget: inFlight && !controller.previewSuppressed && !controller.sessionInvalid
        && candidate.id > 0 && candidate.phase !== "NONE" && candidate.phase !== "COMMITTED"
        && candidate.reason !== "INVALID_GOAL" && !!candidate.goal && candidate.goal.length === 3
    readonly property bool goalSelected: monitor.fresh && (hasCandidateTarget ? candidate.phase === "READY"
        : hasCurrentGoal && (state.phase === "ACTIVE" || state.phase === "WAIT_TARGET")
          && (state.goal_validation === "FREE_IN_LOCAL_MAP" || state.goal_validation === "OUTSIDE_LOCAL_MAP"))

    implicitHeight: content.implicitHeight+24
    color: "#202125"
    radius: 6
    border.color: "#383940"
    property string observedToken: ""
    onHeightReadyChanged: {
        if (!heightReady) pickGoal=false
    }
    onStateChanged: {
        const token=panel.inFlight ? (candidate.confirm_token || "") : (state.confirm_token || "")
        observedToken=token
        if (!controller.canPreview) pickGoal=false
    }
    Connections {
        target:typeof panel.controller.toastRequested === "function" ? panel.controller : null
        function onChanged() {
            if(panel.controller.starting){
                toastLabel.text=qsTr("正在启动静态预览…")
                toastTimer.stop();operationToast.open()
            }else if(operationToast.visible && !toastTimer.running && !panel.controller.busy){
                if(panel.controller.message.length){toastLabel.text=panel.controller.message;toastTimer.restart()}
                else operationToast.close()
            }
        }
        function onToastRequested(message) {
            toastLabel.text=message
            operationToast.open()
            toastTimer.restart()
        }
    }
    Popup {
        id:operationToast
        objectName:"plannerOperationToast"
        parent:Overlay.overlay
        width:Math.min(360,parent ? parent.width-32 : 360)
        x:parent ? (parent.width-width)/2 : 0
        y:24
        padding:16
        modal:false
        focus:false
        closePolicy:Popup.NoAutoClose
        background:Rectangle {color:"#ed102c29";radius:8;border.color:"#34d399"}
        contentItem:Label {
            id:toastLabel
            color:"#d1fae5"
            font.pixelSize:15
            horizontalAlignment:Text.AlignHCenter
            wrapMode:Text.Wrap
            textFormat:Text.PlainText
        }
        onClosed:toastTimer.stop()
    }
    Timer {id:toastTimer;interval:3000;onTriggered:operationToast.close()}
    ColumnLayout {
        id: content
        anchors.fill: parent
        anchors.margins: 12
        spacing: 8
        SectionHeader {
            Layout.fillWidth: true
            title: qsTr("Ego-Planner 避障")
            collapsed: panel.collapsed
            onClicked: panel.collapsed = !panel.collapsed
        }
        ColumnLayout {
            Layout.fillWidth: true
            visible: !panel.collapsed
            spacing: 8

        RowLayout {
            Layout.fillWidth: true
            StudioButton { tone:"primary"; text:qsTr("启动避障 / 预览"); enabled:!panel.controller.busy && !panel.controller.stopPending; onClicked:panel.controller.start() }
            StudioButton { tone:"danger"; text:panel.controller.stopPending ? qsTr("停止处理中") : qsTr("停止避障"); enabled:!panel.controller.busy && !panel.controller.stopPending; onClicked:panel.controller.stop() }
        }
        CheckBox { text:qsTr("显示原生膨胀栅格（局部前向）"); checked:panel.showGrid; onToggled:panel.showGrid=checked }
        Label {
            objectName:"gridNotice"
            visible:panel.showGrid && (!panel.monitor.fresh || !panel.planning.grid_valid || panel.planning.grid_count===0)
            Layout.fillWidth:true; wrapMode:Text.Wrap; font.pixelSize:11; color:"#a1a2ad"
            text:!panel.monitor.fresh ? qsTr("监视数据过期，栅格状态未知")
                : panel.planning.grid_reason==="WAIT_PLANNER" ? qsTr("等待避障预览启动")
                : panel.planning.grid_reason==="PLANNER_INPUT_STALE" ? qsTr("定位或点云过期，暂停显示栅格")
                : panel.planning.grid_reason==="GRID_STALE" ? qsTr("栅格显示数据已过期，正在等待更新")
                : panel.planning.grid_reason==="INVALID_GRID" ? qsTr("栅格分辨率无效")
                : panel.planning.grid_reason==="EMPTY_GRID" ? qsTr("当前前向栅格为空")
                : qsTr("等待当前规划会话的栅格")
        }
        StudioButton {
            tone:"primary"
            Layout.fillWidth:true
            text:panel.pickGoal ? qsTr("请在地图短按选取 XY（仅预览）") : qsTr("地图选点 / 更新预览")
            checkable:true;checked:panel.pickGoal;enabled:panel.controller.canPreview && panel.heightReady
            onClicked:{panel.pickGoal=checked;if(checked)panel.pickRequested()}
        }
        Label {
            objectName:"goalStatusNotice"
            Layout.fillWidth:true;wrapMode:Text.Wrap;font.pixelSize:12
            color:panel.goalSelected ? "#3cbb83" : "#e05257"
            text:panel.goalNotice()
            textFormat:Text.PlainText
        }
        Label {
            objectName:"goalArrivalNotice"
            Layout.fillWidth:true;wrapMode:Text.Wrap;font.pixelSize:11;color:"#b8bac4"
            text:!panel.monitor.fresh ? "" : panel.hasCandidateTarget
                ? (panel.candidate.phase==="READY" ? qsTr("新目标规划中") : panel.candidateText())
                : panel.state.measured_arrived ? qsTr("已到达当前目标")
                : panel.state.distance_m>=0 ? qsTr("实测距目标 %1 m").arg(Number(panel.state.distance_m).toFixed(2)) : ""
        }
        Label {
            visible:text.length>0 && !panel.controller.starting
            Layout.fillWidth:true;wrapMode:Text.WrapAnywhere;font.pixelSize:11;color:"#a1a2ad";maximumLineCount:7;elide:Text.ElideRight
            text:panel.controller.message;textFormat:Text.PlainText
        }

        }
    }
    // 与地图投影和候选消息共用一次高度采样，不做厘米取整或自动更新已选目标。
    function clearReview() {pickGoal=false}
    function resolveGoalHeight() {
        if (!isFinite(currentPoseZ))
            return {valid:false,message:qsTr("当前定位高度缺失或过期，未发送目标。")}
        return {valid:true,height:currentPoseZ}
    }
    function goalNotice() {
        if (panel.hasCandidateTarget) {
            if (!panel.monitor.fresh) return qsTr("目标状态已过期")
            return qsTr("新目标 X:%1  Y:%2  Z:%3 m")
                .arg(Number(panel.candidate.goal[0]).toFixed(2))
                .arg(Number(panel.candidate.goal[1]).toFixed(2))
                .arg(Number(panel.candidate.goal[2]).toFixed(2))
        }
        if (!panel.hasCurrentGoal) return qsTr("尚无目标")
        if (!panel.monitor.fresh) return qsTr("目标状态已过期")
        if (panel.goalSelected)
            return qsTr("目标已选定  X:%1  Y:%2  Z:%3 m")
                .arg(Number(panel.state.goal[0]).toFixed(2))
                .arg(Number(panel.state.goal[1]).toFixed(2))
                .arg(Number(panel.state.goal[2]).toFixed(2))
                + (panel.state.goal_validation === "OUTSIDE_LOCAL_MAP" ? qsTr("（终点未验证）") : "")
        if (panel.state.goal_validation && panel.state.goal_validation !== "FREE_IN_LOCAL_MAP")
            return panel.reason(panel.state.goal_validation)
        if (panel.state.phase === "PLANNING_FAILED") return qsTr("规划失败，请重新选点")
        if (panel.state.phase === "START_IN_COLLISION") return qsTr("当前位置位于障碍区域")
        if (panel.state.phase === "GOAL_ACCEPTED") return qsTr("目标已接收，正在规划")
        return qsTr("目标暂不可用，请重新预览")
    }
    function reason(value) {
        const labels={OCCUPIED:"目标位于障碍物内",INFLATED:"目标位于膨胀区内",VIRTUAL_WALL:"目标超出原高度边界",MAP_NOT_READY:"规划地图未就绪",OUTSIDE_LOCAL_MAP:"远目标：沿途局部避障，终点尚未验证",FREE_IN_LOCAL_MAP:"局部地图内自由",NO_GOAL:"等待目标",WAIT_FLIGHT_OR_INPUT:"飞行状态或定位、点云尚未就绪",PREVIEW_START_CHANGED:"机体或参考轨迹已变化，请重新选点",SWARM_TRAJECTORY_CHANGED:"其他飞行器轨迹已变化，请重新选点",PREVIEW_IN_COLLISION:"预览轨迹与最新膨胀栅格相交，请重新选点",PREVIEW_OUTSIDE_LOCAL_MAP:"预览轨迹超出当前规划地图",PREVIEW_CHECK_TIMEOUT:"地图校验耗时过长，请重新选点",START_IN_COLLISION:"轨迹起点位于膨胀区",PLANNING_FAILED:"候选目标规划失败",PREVIEW_ERROR:"候选预览计算失败",INVALID_GOAL:"目标坐标或坐标系无效",GOAL_TOO_CLOSE:"目标距当前参考位置过近",CONFIRM_TOKEN_MISMATCH:"候选目标确认已失效",FLIGHT_MODE_CHANGED:"飞行模式已变化，请重新选点",ACTIVE_GOAL_CHANGED:"当前执行目标已变化，请重新选点",PREVIEW_BUSY:"候选预览计算中，请稍候",REFERENCE_MISSING:"缺少有效参考轨迹",HOLD_REFERENCE_MISMATCH:"当前位置偏离悬停参考，暂不能切换",MAP_SNAPSHOT_FAILED:"无法获取原生地图快照",PREVIEW_WORKER_FAILED:"候选预览进程失败，当前目标未变",PREVIEW_WORKER_TIMEOUT:"候选预览超时，当前目标未变",GLOBAL_PLAN_FAILED:"新目标全局规划失败",INVALID_PREVIEW:"候选预览数据无效"}
        return labels[value] || value || "等待验证"
    }
    function candidateText() {
        const c=panel.candidate
        if(c.reason)return panel.reason(c.reason)
        if(c.phase==="PLANNING")return qsTr("正在计算候选预览，继续执行当前目标")
        if(c.phase==="READY")return panel.planning.candidate_path_valid ? qsTr("目标已交给原生规划器") : qsTr("等待规划轨迹")
        if(c.phase==="COMMITTED")return qsTr("新目标已确认，原规划器正在执行")
        if(c.available)return qsTr("可以选下一个目标")
        return qsTr("等待飞行状态和规划数据就绪")
    }
}
