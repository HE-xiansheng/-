import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Rectangle {
    id:panel
    property bool collapsed: true
    required property var controller
    property bool otherBusy:false
    implicitHeight:content.implicitHeight+24
    radius:6;color:"#202125";border.color:"#383940"
    ColumnLayout {
        id:content;anchors.fill:parent;anchors.margins:12;spacing:8
        SectionHeader {
            Layout.fillWidth: true
            title: qsTr("FAST-LIO 定位与转发")
            collapsed: panel.collapsed
            onClicked: panel.collapsed = !panel.collapsed
        }
        ColumnLayout {
            Layout.fillWidth: true
            visible: !panel.collapsed
            spacing: 8

        RowLayout {
            Layout.fillWidth:true
            StudioButton {objectName:"localizationStart";Layout.fillWidth:true;text:qsTr("启动定位与转发");enabled:panel.controller.fresh&&!panel.controller.busy&&!panel.otherBusy;onClicked:panel.controller.start()}
            StudioButton {objectName:"localizationStop";Layout.fillWidth:true;text:qsTr("停止");enabled:panel.controller.fresh&&!panel.controller.busy&&!panel.otherBusy;onClicked:panel.controller.stop()}
        }
        RowLayout {
            Label {text:qsTr("定位");color:"#a1a2ad";font.pixelSize:12}
            Label {Layout.fillWidth:true;text:panel.controller.lioText;color:panel.controller.lioReady ? "#3cbb83" : "#dbad48";font.pixelSize:12;wrapMode:Text.Wrap}
            Label {text:qsTr("转发");color:"#a1a2ad";font.pixelSize:12}
            Label {Layout.fillWidth:true;text:panel.controller.relayText;color:panel.controller.relayReady ? "#3cbb83" : "#dbad48";font.pixelSize:12;wrapMode:Text.Wrap}
        }
        Label {Layout.fillWidth:true;visible:text.length>0;text:panel.controller.message;color:"#dbad48";font.pixelSize:11;wrapMode:Text.Wrap;textFormat:Text.PlainText}
        }
    }
}
