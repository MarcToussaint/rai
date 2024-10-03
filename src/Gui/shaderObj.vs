#version 330 core

layout(location = 0) in vec3 vertexPosition_M;
layout(location = 1) in vec4 vertexColor;
layout(location = 2) in vec3 vertexNormal_M;

out vec4 objColor;
out vec3 objNormal_C;
out vec3 eyeDirection_C;
out vec3 lightDirection_C;
out vec4 worldCoord;
out vec4 shadowCoord;

uniform mat4 Projection_W;
uniform mat4 ViewT_CW;
uniform mat4 ModelT_WM;
uniform mat4 ShadowProjection_W;

void main() {
  worldCoord = ModelT_WM * vec4(vertexPosition_M, 1);

  gl_Position =  Projection_W * worldCoord;

  shadowCoord = ShadowProjection_W * worldCoord;

  eyeDirection_C = vec3(0, 0, 0) - (ViewT_CW * worldCoord).xyz;
  objNormal_C = (ViewT_CW * ModelT_WM * vec4(vertexNormal_M, 0)).xyz; // Only correct if ModelMatrix does not scale the model ! Use its inverse transpose if not.

  objColor = vertexColor;
}

