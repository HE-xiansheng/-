import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import QtQuick.Dialogs
import QtQuick3D
import CustomGeometryExample

Window {
    id:viewer
    required property var backend
    title:qsTr("飞行记录 · 三维点云与轨迹")
    width:1180;height:780;minimumWidth:860;minimumHeight:600
    color:"#151619"
    property string recordingName:""
    property real orbitYaw:0
    property real orbitPitch:-35
    property real cameraDistance:900
    property vector3d focusPoint:Qt.vector3d(0,0,0)
    property bool playing:false
    property string loadError:""
    onClosing:{playing=false;backend.cancel()}
    function openRecords(){show();raise();requestActivate();if(!backend.busy)backend.refresh()}
    function fitMap(){
        const a=historyCloud.boundsMinMeters,b=historyCloud.boundsMaxMeters
        focusPoint=historyCloud.toScenePosition(Qt.vector3d((a.x+b.x)/2,(a.y+b.y)/2,(a.z+b.z)/2))
        cameraDistance=Math.max(180,Math.sqrt(Math.pow(b.x-a.x,2)+Math.pow(b.y-a.y,2)+Math.pow(b.z-a.z,2))*45*1.3)
        orbitYaw=0;orbitPitch=-35
    }
    function focusFlight(){
        const points=historyLoader.flightTrajectory
        if(points.length<2){fitMap();return}
        let lo=Qt.vector3d(points[0].x,points[0].y,points[0].z),hi=Qt.vector3d(lo.x,lo.y,lo.z)
        for(const p of points){lo=Qt.vector3d(Math.min(lo.x,p.x),Math.min(lo.y,p.y),Math.min(lo.z,p.z));hi=Qt.vector3d(Math.max(hi.x,p.x),Math.max(hi.y,p.y),Math.max(hi.z,p.z))}
        focusPoint=historyCloud.toScenePosition(Qt.vector3d((lo.x+hi.x)/2,(lo.y+hi.y)/2,(lo.z+hi.z)/2))
        cameraDistance=Math.max(420,Math.sqrt(Math.pow(hi.x-lo.x,2)+Math.pow(hi.y-lo.y,2)+Math.pow(hi.z-lo.z,2))*45*1.8)
        orbitYaw=0;orbitPitch=-45
    }
    function parse(files,name){
        if(!files.length)return
        playing=false;loadError=""
        const file=files[0]
        recordingName=name+" / "+decodeURIComponent(String(file).split("/").pop())
        historyLoader.loadRecordingFiles([file])
    }
    Connections {
        target:viewer.backend
        function onRecordingReady(files,name){viewer.parse(files,name)}
    }
    PointCloudGeometry {id:historyCloud;objectName:"flightRecordCloud";sceneScale:45;rainbowStyle:true;ceilingClipEnabled:hideTop.checked;ceilingHeight:1.8}
    RouteGeometry {id:historyPath;objectName:"flightRecordPath";sceneScale:45;waypoints:historyLoader.playbackTrajectory}
    PointCloudFileLoader {
        id:historyLoader;objectName:"flightRecordLoader";geometry:historyCloud;maxPoints:400000;accumulatedPlayback:true
        onLoadFinished:function(success){if(success){playbackPositionMs=playbackDurationMs;viewer.focusFlight()}else viewer.loadError=status}
    }
    FileDialog {
        id:localBags;title:qsTr("选择一个飞行分包")
        fileMode:FileDialog.OpenFile;nameFilters:[qsTr("ROS1录包 (*.bag)")]
        onAccepted:viewer.parse([selectedFile],qsTr("本地飞行记录"))
    }
    Timer {
        interval:100;repeat:true;running:viewer.playing&&viewer.visible
        onTriggered:{
            historyLoader.playbackPositionMs=Math.min(historyLoader.playbackDurationMs,historyLoader.playbackPositionMs+100)
            if(historyLoader.playbackPositionMs>=historyLoader.playbackDurationMs)viewer.playing=false
        }
    }
    component RecordSelect:ComboBox {
        id:control;textRole:"name"
        contentItem:Text {text:control.displayText;color:control.enabled?"#dadbe0":"#93959f";leftPadding:12;rightPadding:30;verticalAlignment:Text.AlignVCenter;elide:Text.ElideRight}
        background:Rectangle {color:"#26282e";radius:5;border.color:"#42454e"}
        delegate:ItemDelegate {
            required property var modelData
            required property int index
            width:control.width;highlighted:control.highlightedIndex===index
            contentItem:Text {text:modelData.name;color:"#dadbe0";elide:Text.ElideRight}
            background:Rectangle {color:parent.highlighted?"#353b48":"#26282e"}
        }
        popup:Popup {
            y:control.height+4;width:control.width;padding:4
            background:Rectangle {color:"#26282e";radius:5;border.color:"#42454e"}
            contentItem:ListView {clip:true;implicitHeight:Math.min(contentHeight,280);model:control.popup.visible?control.delegateModel:null;currentIndex:control.highlightedIndex;ScrollIndicator.vertical:ScrollIndicator {}}
        }
    }
    ColumnLayout {
        anchors.fill:parent;anchors.margins:16;spacing:10
        RowLayout {
            Layout.fillWidth:true
            Label {text:qsTr("飞行记录");font.pixelSize:20;font.weight:Font.DemiBold;color:"#eeeef1"}
            Label {text:viewer.recordingName;color:"#a1a2ad";Layout.fillWidth:true;elide:Text.ElideMiddle}
            StudioButton {text:qsTr("打开本地录包");enabled:!historyLoader.loading&&!backend.busy;onClicked:localBags.open()}
        }
        RowLayout {
            Layout.fillWidth:true
            RecordSelect {
                id:recordList;objectName:"flightRecordList";Layout.fillWidth:true
                model:backend.records;enabled:!backend.busy&&!historyLoader.loading
                displayText:count&&currentIndex>=0?currentText:qsTr("选择飞行记录")
                onCurrentIndexChanged:partList.currentIndex=0
            }
            RecordSelect {
                id:partList;objectName:"flightRecordPartList";Layout.preferredWidth:180
                model:recordList.currentIndex>=0&&backend.records[recordList.currentIndex]?backend.records[recordList.currentIndex].files:[]
                enabled:!backend.busy&&!historyLoader.loading
                displayText:count&&currentIndex>=0?currentText:qsTr("选择分包")
            }
            StudioButton {objectName:"refreshFlightRecords";text:qsTr("刷新列表");enabled:!backend.busy&&!historyLoader.loading;onClicked:backend.refresh()}
            StudioButton {objectName:"readFlightRecord";text:qsTr("读取并绘图");tone:"primary";enabled:recordList.currentIndex>=0&&partList.currentIndex>=0&&!backend.busy&&!historyLoader.loading;onClicked:backend.readRecordPart(recordList.currentIndex,partList.currentIndex)}
            StudioButton {text:qsTr("取消读取");visible:backend.busy;onClicked:backend.cancel()}
        }
        Label {
            Layout.fillWidth:true
            visible:backend.busy||backend.hasError||viewer.loadError.length>0||(!historyLoader.loadedFile.length&&backend.message.length>0)
            text:viewer.loadError||backend.message;color:backend.hasError||viewer.loadError.length?"#e05257":"#a1a2ad";wrapMode:Text.Wrap;textFormat:Text.PlainText
        }
        ProgressBar {Layout.fillWidth:true;visible:backend.busy||historyLoader.loading;value:historyLoader.loading?historyLoader.progress/100:backend.progress}
        Rectangle {
            Layout.fillWidth:true;Layout.fillHeight:true;color:"#0d0e11";radius:8;border.color:"#383940";clip:true
            View3D {
                id:historyView;anchors.fill:parent
                environment:SceneEnvironment {backgroundMode:SceneEnvironment.Color;clearColor:"#0d0e11";antialiasingMode:SceneEnvironment.MSAA;antialiasingQuality:SceneEnvironment.High}
                Node {
                    position:viewer.focusPoint
                    eulerRotation.y:viewer.orbitYaw
                    Node {
                        eulerRotation.x:viewer.orbitPitch
                        PerspectiveCamera {id:historyCamera;z:viewer.cameraDistance;clipNear:1;clipFar:1000000}
                    }
                }
                camera:historyCamera
                Model {geometry:historyCloud;materials:PrincipledMaterial {lighting:PrincipledMaterial.NoLighting;vertexColorsEnabled:true;pointSize:2}}
                Model {geometry:historyPath;visible:historyPath.valid;materials:PrincipledMaterial {lighting:PrincipledMaterial.NoLighting;baseColor:"#ffb454";lineWidth:4}}
                Model {
                    visible:historyLoader.hasPlaybackPose
                    position:historyCloud.toScenePosition(historyLoader.playbackPosePositionMeters)
                    source:"#Sphere";scale:Qt.vector3d(.055,.055,.055)
                    materials:PrincipledMaterial {lighting:PrincipledMaterial.NoLighting;baseColor:"#4dd8d0"}
                }
                MouseArea {
                    anchors.fill:parent;acceptedButtons:Qt.LeftButton|Qt.MiddleButton
                    property real lastX:0;property real lastY:0
                    onPressed:function(m){lastX=m.x;lastY=m.y}
                    onPositionChanged:function(m){
                        const dx=m.x-lastX,dy=m.y-lastY
                        if(pressedButtons&Qt.LeftButton){viewer.orbitYaw-=dx*.35;viewer.orbitPitch=Math.max(-89,Math.min(89,viewer.orbitPitch-dy*.35))}
                        else if(pressedButtons&Qt.MiddleButton){const s=viewer.cameraDistance/historyView.height;const a=viewer.orbitYaw*Math.PI/180;viewer.focusPoint=Qt.vector3d(viewer.focusPoint.x-dx*s*Math.cos(a),viewer.focusPoint.y+dy*s,viewer.focusPoint.z+dx*s*Math.sin(a))}
                        lastX=m.x;lastY=m.y
                    }
                    onWheel:function(w){viewer.cameraDistance=Math.max(20,Math.min(100000,viewer.cameraDistance*Math.pow(.999,w.angleDelta.y)));w.accepted=true}
                }
            }
            Label {anchors.centerIn:parent;visible:historyCloud.pointCount===0;text:historyLoader.loading?qsTr("正在解析点云与轨迹…"):qsTr("选择一个分包，读取后查看点云与轨迹");color:"#93959f"}
            RowLayout {
                anchors.top:parent.top;anchors.left:parent.left;anchors.right:parent.right;anchors.margins:12
                Item {Layout.fillWidth:true}
                CheckBox {
                    id:hideTop;checked:true;text:qsTr("隐藏顶部")
                    contentItem:Text {text:hideTop.text;color:"#b8bac4";leftPadding:24;verticalAlignment:Text.AlignVCenter}
                    indicator:Rectangle {x:0;y:(parent.height-height)/2;width:18;height:18;radius:3;color:hideTop.checked?"#3475bd":"#26282e";border.color:"#5f6b7b";Text {anchors.centerIn:parent;text:hideTop.checked?"✓":"";color:"#f0f3f7";font.pixelSize:14}}
                }
                StudioButton {text:qsTr("聚焦轨迹");onClicked:viewer.focusFlight()}
                StudioButton {text:qsTr("完整点云");onClicked:viewer.fitMap()}
            }
        }
        RowLayout {
            Layout.fillWidth:true
            StudioButton {text:viewer.playing?qsTr("暂停"):qsTr("播放");enabled:historyLoader.playbackAvailable&&!historyLoader.loading;onClicked:{if(historyLoader.playbackPositionMs>=historyLoader.playbackDurationMs)historyLoader.playbackPositionMs=0;viewer.playing=!viewer.playing}}
            Slider {
                id:playbackSlider;objectName:"flightRecordTimeline";Layout.fillWidth:true;from:0;to:Math.max(1,historyLoader.playbackDurationMs);value:historyLoader.playbackPositionMs;enabled:historyLoader.playbackAvailable&&!historyLoader.loading
                onMoved:{viewer.playing=false;historyLoader.playbackPositionMs=value}
                background:Rectangle {x:playbackSlider.leftPadding;y:(playbackSlider.height-height)/2;width:playbackSlider.availableWidth;height:4;radius:2;color:"#3c3f48";Rectangle {width:playbackSlider.visualPosition*parent.width;height:parent.height;radius:2;color:"#d69a54"}}
                handle:Rectangle {x:playbackSlider.leftPadding+playbackSlider.visualPosition*(playbackSlider.availableWidth-width);y:(playbackSlider.height-height)/2;implicitWidth:14;implicitHeight:14;radius:7;color:playbackSlider.pressed?"#f9cb8e":"#d9dde6"}
            }
            Label {text:(historyLoader.playbackPositionMs/1000).toFixed(1)+" / "+(historyLoader.playbackDurationMs/1000).toFixed(1)+" s";color:"#b8bac4"}
        }
        Label {Layout.fillWidth:true;visible:historyLoader.loadedFile.length>0&&historyLoader.playbackPoseCount===0;text:qsTr("本记录未包含匹配的定位轨迹，仅显示已录制点云。");color:"#dbad48";font.pixelSize:11}
    }
}
