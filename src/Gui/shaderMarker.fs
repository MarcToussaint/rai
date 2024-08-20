#version 330 core

in vec4 objColor;
layout(location = 0) out vec4 color;

void main() {
  color.rgb = .5 * objColor.rgb + .25;
  color.a = objColor.a;
}
