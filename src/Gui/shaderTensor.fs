#version 330 core

const float AmbientPower = .5f;
const float LightPower = .4f;
const vec3 LightColor = vec3(1, 1., 1.);
const float SpecularPower = .2f;
const vec3 SpecularColor = vec3(1, 1, 1);
const float SpecularNarrow = 20.f;

in vec4 objColor;
in vec3 eyePosition_M;
in vec4 objPosition_W;
in vec3 tensorCoord;

layout(location = 0) out vec4 color;

uniform mat4 ModelT_WM;
uniform vec3 lightDirection_C[2];
uniform int numLights;
uniform int useShadow;
uniform sampler2DShadow shadowMap;
uniform vec4 flatColor;
uniform sampler3D tensorTexture;



vec3 calculate_normal(in vec3 p)
{
    const vec3 small_step = vec3(0.001, 0.0, 0.0);

    float gradient_x = texture(tensorTexture, p + small_step.xyy).r - texture(tensorTexture, p - small_step.xyy).r;
    float gradient_y = texture(tensorTexture, p + small_step.yxy).r - texture(tensorTexture, p - small_step.yxy).r;
    float gradient_z = texture(tensorTexture, p + small_step.yyx).r - texture(tensorTexture, p - small_step.yyx).r;

    vec3 normal = vec3(gradient_x, gradient_y, gradient_z);

    return normalize(normal);
}


void main()
{
  color = vec4(1., .5, 0., 1.); //objColor+.5;
  color.rgb = tensorCoord+.5;
color.a = 1.;
//return;

float s = 1.*texture(tensorTexture, tensorCoord+.5).r;
color.rgb = vec3(s,s,s);
//return;

vec3 origin = tensorCoord;
vec3 direction = normalize(origin - eyePosition_M);
vec3 delta = (.01 * 1.415) * direction;
vec3 pos = origin;


//vec3 N = normalize(objNormal_W);

//diffuse: alignment of object-normal and obj->light
//float cosTheta = clamp(dot(N, lightDirection_W[i]), 0.f, 1.f);

//specular: alignment of obj->eye and obj->reflectedLight directions
//vec3 reflectedLightDirection_W = reflect(-lightDirection_W[i], N);
//float cosAlpha = clamp(dot(eyeDirection_W, reflectedLightDirection_W), 0.f, 1.f);


float density = 0.;
vec3 normal = vec3(0,0,0);
for(int i=0;i<=100;i++){
  if(pos.x>-.5 && pos.x<.5 && pos.y>-.5 && pos.y<.5 && pos.z>-.5 && pos.z<.5){
    float d = texture(tensorTexture, pos+.5).r;
    vec3 n = calculate_normal(pos+.5);
    if(d<0.) d=0;
    if(d>1.) d=1.;
    //d = d*d;
    density += d;
    normal += d * n;
    if(density>=1.) break;
  }
  pos += delta;
}

//vec3 eyeDirection_M = normalize(eyePosition_M - tensorCoord);
//normal = normalize(normal);
vec3 lightDirection = normalize(vec3(1.,0,0));
float cosTheta = clamp(abs(dot(normal, lightDirection)), 0.f, 1.f);

float rgb = .3; // + .5*cosTheta;

color = vec4(rgb, rgb, rgb, density);
return;

}


