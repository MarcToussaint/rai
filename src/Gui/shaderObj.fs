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
        visibility -= 0.1 * (1.0-texture( shadowMap, vec3(shadowCoord.xy + poissonDisk[index]/700.0,  (shadowCoord.z-bias)/shadowCoord.w) ));
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
