#pragma once
#include <string>
//=============================
const std::string objVS = R"(
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

)";
//=============================
const std::string objFS = R"(
#version 330 core

const float AmbientPower = .5f;
const float LightPower = .4f;
const vec3 LightColor = vec3(1, 1., 1.);
const float SpecularPower = .2f;
const vec3 SpecularColor = vec3(1, 1, 1);
const float SpecularNarrow = 20.f;

in vec4 objColor;
in vec3 objNormal_C;
in vec3 eyeDirection_C;
in vec4 worldCoord;
in vec4 shadowCoord;

layout(location = 0) out vec4 color;

uniform vec3 lightDirection_C[2];
uniform int numLights;
uniform int useShadow;
uniform sampler2DShadow shadowMap;
uniform vec4 flatColor;

vec2 poissonDisk[16] = vec2[]( 
   vec2( -0.94201624, -0.39906216 ), 
   vec2( 0.94558609, -0.76890725 ), 
   vec2( -0.094184101, -0.92938870 ), 
   vec2( 0.34495938, 0.29387760 ), 
   vec2( -0.91588581, 0.45771432 ), 
   vec2( -0.81544232, -0.87912464 ), 
   vec2( -0.38277543, 0.27676845 ), 
   vec2( 0.97484398, 0.75648379 ), 
   vec2( 0.44323325, -0.97511554 ), 
   vec2( 0.53742981, -0.47373420 ), 
   vec2( -0.26496911, -0.41893023 ), 
   vec2( 0.79197514, 0.19090188 ), 
   vec2( -0.24188840, 0.99706507 ), 
   vec2( -0.81409955, 0.91437590 ), 
   vec2( 0.19984126, 0.78641367 ), 
   vec2( 0.14383161, -0.14100790 ) 
);

// Returns a random number based on a vec3 and an int.
float random(vec3 seed, int i){
  vec4 seed4 = vec4(seed,i);
  float dot_product = dot(seed4, vec4(12.9898,78.233,45.164,94.673));
  return fract(sin(dot_product) * 43758.5453);
}

void main() {
  if(flatColor[3]>0.){
    color = flatColor;
    return;
  }

  //vec3 objColor = texture( myTextureSampler, UV ).rgb;

  vec3 objRgb = objColor.rgb;

  vec3 rgb = AmbientPower * objRgb;

  vec3 E = normalize(eyeDirection_C);
  vec3 N = normalize(objNormal_C);

  for(int i=0; i<numLights; i++) {
    // Distance to the light
    //float distance = length( lightPosition_W - objPosition_W );

    //diffuse: alignment of object-normal and obj->light
    float cosTheta = clamp(dot(N, lightDirection_C[i]), 0.f, 1.f);

    //specular: alignment of obj->eye and obj->reflectedLight directions
    vec3 reflectedLightDirection_C = reflect(-lightDirection_C[i], N);
    float cosAlpha = clamp(dot(E, reflectedLightDirection_C), 0.f, 1.f);

    float bias = 0.005;
    float visibility = 1.;
    if(i==0 && useShadow>0){
      //visibility = texture(shadowMap, vec3(shadowCoord.xy, (shadowCoord.z-bias)/shadowCoord.w));
      visibility -= 0.5 * (1.0-texture(shadowMap, vec3(shadowCoord.xy, (shadowCoord.z-bias)/shadowCoord.w)));
      for (int j=0;j<4;j++){
        //int index = j;
        //int index = int(16.0*random(gl_FragCoord.xyz, i))%16;
        int index = int(16.0*random(floor(worldCoord.xyz*1000.0), i))%16;
        visibility -= 0.1 * (1.0-texture( shadowMap, vec3(shadowCoord.xy + poissonDisk[index]/2000.0,  (shadowCoord.z-bias)/shadowCoord.w) ));
      }
    }

    rgb = rgb +
          visibility * objRgb * LightColor * LightPower * cosTheta +
          visibility * SpecularColor * SpecularPower * pow(cosAlpha, SpecularNarrow);
  }

  color.rgb = rgb;
  color.a = objColor.a;
  //  color.a = 1.;
}
)";
//=============================
const std::string markerVS = R"(
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

)";
//=============================
const std::string markerFS = R"(
#version 330 core

in vec4 objColor;
layout(location = 0) out vec4 color;

void main() {
  color.rgb = .5 * objColor.rgb + .25;
  color.a = objColor.a;
}
)";
//=============================
const std::string shadowVS = R"(
#version 330 core

layout(location = 0) in vec3 vertexPosition_M;

uniform mat4 ShadowProjection_W;
uniform mat4 ModelT_WM;

void main() {
  gl_Position =  ShadowProjection_W * ModelT_WM * vec4(vertexPosition_M, 1);
}
)";
//=============================
const std::string shadowFS = R"(
#version 330 core

layout(location = 0) out float fragmentdepth;


void main() {
  fragmentdepth = gl_FragCoord.z;
}
)";
//=============================
const std::string textVS = R"(
#version 330 core
layout (location = 0) in vec4 vertex; // <vec2 pos, vec2 tex>
out vec2 TexCoords;

uniform mat4 projection;

void main()
{
    gl_Position = projection * vec4(vertex.xy, 0.0, 1.0);
    TexCoords = vertex.zw;
})";
//=============================
const std::string textFS = R"(
#version 330 core
in vec2 TexCoords;
out vec4 color;

uniform sampler2D text;
uniform vec3 textColor;
uniform int useTexColor;

void main()
{    
  if(useTexColor==1){
    color = texture(text, TexCoords);
  }else{
    vec4 sampled = vec4(1.0, 1.0, 1.0, texture(text, TexCoords).r);
    color = vec4(textColor, 1.0) * sampled;
  }
}
)";
//=============================
