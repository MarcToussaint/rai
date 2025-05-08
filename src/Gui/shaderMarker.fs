#version 330 core

layout(location = 0) out vec4 color;

in vec4 objColor;
uniform vec4 flatColor;

void main() {
  if(flatColor[3]>0.){
    color.rgb = .75 * flatColor.rgb + .25;
    color.a = 1.;
    return;
  }

  color.rgb = .5 * objColor.rgb + .25;
  color.a = objColor.a;
}
