#version 330 core

layout(location = 0) in vec3 vertexPosition_M;
layout(location = 1) in vec4 vertexColor;
layout(location = 2) in vec3 vertexNormal_M;

out vec4 objColor;

uniform mat4 Projection_W;
uniform mat4 ModelT_WM;

void main() {
  gl_Position =  Projection_W * ModelT_WM * vec4(vertexPosition_M, 1);
  objColor = vertexColor;
}

