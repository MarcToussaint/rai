#version 330 core

layout(location = 0) in vec3 vertexPosition_M;
layout(location = 1) in vec4 vertexColor;
layout(location = 2) in vec3 vertexNormal_M;

out vec4 objColor;
out vec3 objNormal_W;
out vec4 objPosition_W;
out vec4 shadowCoord;

uniform mat4 Projection_W;
uniform mat4 ModelT_WM;
uniform mat4 ShadowProjection_W;

void main() {
  objPosition_W = ModelT_WM * vec4(vertexPosition_M, 1);
  objNormal_W = (ModelT_WM * vec4(vertexNormal_M, 0)).xyz; // Only correct if ModelMatrix does not scale the model ! Use its inverse transpose if not.

  gl_Position = Projection_W * objPosition_W;
  shadowCoord = ShadowProjection_W * objPosition_W;

  objColor = vertexColor;
}

