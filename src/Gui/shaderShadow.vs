#version 330 core

layout(location = 0) in vec3 vertexPosition_M;

uniform mat4 ShadowProjection_W;
uniform mat4 ModelT_WM;

void main() {
  gl_Position =  ShadowProjection_W * ModelT_WM * vec4(vertexPosition_M, 1);
}
