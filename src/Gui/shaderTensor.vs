#version 330 core

layout(location = 0) in vec3 vertexPosition_M;
layout(location = 1) in vec4 vertexColor;
layout(location = 2) in vec3 vertexNormal_M;

out vec4 objColor;
out vec3 eyePosition_M;
out vec4 objPosition_W;
out vec3 tensorCoord;

uniform vec3 eyePosition_W;
uniform mat4 Projection_W;
uniform mat4 ModelT_WM;

void main() {
  objPosition_W = ModelT_WM * vec4(vertexPosition_M, 1);

  gl_Position =  Projection_W * objPosition_W;

  eyePosition_M = (inverse(ModelT_WM) * vec4(eyePosition_W, 1)).xyz;

  objColor = vertexColor;
  tensorCoord = vertexPosition_M;
}

