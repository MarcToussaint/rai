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
in vec4 shadowCoord;

layout(location = 0) out vec4 color;

uniform vec3 lightDirection_C[2];
uniform int numLights;
uniform sampler2DShadow shadowMap;

void main() {
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
    if(i==0) visibility = texture(shadowMap, vec3(shadowCoord.xy, (shadowCoord.z-bias)/shadowCoord.w));

    rgb = rgb +
          visibility * objRgb * LightColor * LightPower * cosTheta +
          visibility * SpecularColor * SpecularPower * pow(cosAlpha, SpecularNarrow);
  }

  color.rgb = rgb;
  color.a = objColor.a;
  //  color.a = 1.;
}
