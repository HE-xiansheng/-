#version 440
layout(location=0) in vec2 qt_TexCoord0;
layout(location=0) out vec4 fragColor;
layout(binding=1) uniform sampler2D source;
layout(std140,binding=0) uniform buf { mat4 qt_Matrix; float qt_Opacity; };
void main() {
    vec4 c=texture(source,qt_TexCoord0);
    // 原图保持不变，仅在绘制时消除白底，保留蓝绿品牌色。
    float mask=1.0-smoothstep(0.82,0.98,min(c.r,min(c.g,c.b)));
    fragColor=vec4(c.rgb*mask,mask)*qt_Opacity;
}
