import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Rectangle {
    id: panel
    required property var connection
    property bool canApply: true
    property int cloudPort: 9000
    property int posePort: 9001
    property int preflightPort: 9003
    property bool collapsed: true
    implicitHeight: content.implicitHeight + 24
    radius: 6
    color: "#202125"
    border.color: "#383940"
    ColumnLayout {
        id: content
        anchors.fill: parent
        anchors.margins: 12
        spacing: 8
        SectionHeader {
            Layout.fillWidth: true
            title: qsTr("机载通信")
            collapsed: panel.collapsed
            onClicked: panel.collapsed = !panel.collapsed
        }
        ColumnLayout {
            visible: !panel.collapsed
            Layout.fillWidth: true
            spacing: 8
            RowLayout {
                Label { text: qsTr("机载 IP"); Layout.preferredWidth: 56 }
                TextField { id: address; objectName:"onboardHost"; Layout.fillWidth:true; text:panel.connection.host; enabled:!panel.connection.busy; selectByMouse:true }
            }
            RowLayout {
                Label { text: qsTr("用户"); Layout.preferredWidth: 56 }
                Label { text:"orangepi"; color:"#b2b3bc" }
            }
            RowLayout {
                Label { text: qsTr("密码"); Layout.preferredWidth: 56 }
                TextField { id: password; objectName:"onboardPassword"; Layout.fillWidth:true; echoMode:TextInput.Password; placeholderText:qsTr("留空使用已有 SSH 密钥"); enabled:!panel.connection.busy; selectByMouse:true }
            }
            StudioButton {
                objectName:"onboardConnect"
                Layout.fillWidth:true
                tone:"primary"
                text:panel.connection.busy ? qsTr("正在连接…") : qsTr("应用并连接")
                enabled:panel.canApply && !panel.connection.busy
                onClicked: {
                    panel.connection.connectTo(address.text,password.text,panel.cloudPort,panel.posePort,panel.preflightPort)
                    password.clear()
                }
            }
            Label {
                Layout.fillWidth:true
                text:panel.canApply ? panel.connection.message : qsTr("飞行或操作期间暂不能更换连接")
                color:panel.connection.authenticated ? "#3cbb83" : "#a1a2ad"
                wrapMode:Text.Wrap
                font.pixelSize:11
            }
        }
    }
}
