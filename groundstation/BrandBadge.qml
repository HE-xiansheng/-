import QtQuick
Rectangle {
    id:badge
    implicitWidth:46;implicitHeight:46
    radius:12;color:"#292b30";border.color:"#3b3e45"
    Image {
        id:sourceLogo
        visible:false
        width:36;height:36
        source:"assets/tianyi-logo.jpg"
        // 只取品牌图标，文字由相邻平台名称承担，去掉原白色大留边。
        sourceClipRect:Qt.rect(126,94,146,156)
        fillMode:Image.PreserveAspectFit
        mipmap:true
    }
    ShaderEffect {
        anchors.fill:parent;anchors.margins:7
        property variant source:sourceLogo
        fragmentShader:"qrc:/shaders/brand.frag.qsb"
    }
}
