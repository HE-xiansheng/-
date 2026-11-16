import QtQuick
import QtQuick.Controls.Basic

Window {
    id:startup
    objectName:"groundStationStartup"
    title:qsTr("科研飞行平台-地面站 · 正在启动")
    width:510;height:350
    x:(Screen.width-width)/2;y:(Screen.height-height)/2
    flags:Qt.SplashScreen|Qt.FramelessWindowHint|Qt.WindowStaysOnTopHint
    color:"transparent"
    visible:true
    property string message:qsTr("正在加载地面站…")
    property bool ready:false
    property bool failed:false
    onReadyChanged:if(ready&&!failed)fadeOut.start()
    NumberAnimation {id:fadeOut;target:startup;property:"opacity";to:0;duration:180;onFinished:startup.close()}
    Rectangle {
        anchors.fill:parent;anchors.margins:8;radius:22;color:"#202329";border.color:"#414955"
        Rectangle {x:1;y:1;width:parent.width-2;height:3;radius:2;gradient:Gradient {orientation:Gradient.Horizontal;GradientStop {position:0;color:"#3a80d9"} GradientStop {position:1;color:"#50cd9e"}}}
        Column {
            anchors.centerIn:parent;spacing:12
            Image {
                anchors.horizontalCenter:parent.horizontalCenter
                width:140;height:140;source:"assets/icons/research-flight-gcs-v1.png";fillMode:Image.PreserveAspectFit;mipmap:true
            }
            Text {anchors.horizontalCenter:parent.horizontalCenter;text:qsTr("科研飞行平台-地面站");font.pixelSize:21;font.weight:Font.DemiBold;color:"#eef0f4"}
            Row {
                anchors.horizontalCenter:parent.horizontalCenter;spacing:9
                Item {
                    width:18;height:18;visible:!startup.failed
                    Item {
                        anchors.fill:parent
                        Repeater {
                            model:10
                            Rectangle {
                                required property int index
                                x:8+7*Math.cos(index*Math.PI/5);y:8+7*Math.sin(index*Math.PI/5)
                                width:3;height:3;radius:1.5;color:"#56c9d8";opacity:.2+.8*index/9
                            }
                        }
                        RotationAnimator on rotation {from:0;to:360;duration:1000;loops:Animation.Infinite;running:startup.visible&&!startup.failed}
                    }
                }
                Text {text:startup.message;color:startup.failed?"#e87878":"#aab4c1";font.pixelSize:13;verticalAlignment:Text.AlignVCenter;height:18}
            }
            Button {anchors.horizontalCenter:parent.horizontalCenter;visible:startup.failed;text:qsTr("关闭");onClicked:Qt.quit()}
        }
    }
}
