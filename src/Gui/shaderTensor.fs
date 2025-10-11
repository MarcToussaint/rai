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
in vec3 surfacePoint_M;

layout(location = 0) out vec4 color;

uniform mat4 ModelT_WM;
uniform vec3 ModelScale;
uniform vec3 lightDirection_C[2];
uniform int numLights;
uniform int useShadow;
uniform sampler2DShadow shadowMap;
uniform vec4 flatColor;
uniform sampler3D tensorTexture;

vec3 calculate_normal(in vec3 p){
  const vec3 small_step = vec3(0.001, 0.0, 0.0);	

  float gradient_x = texture(tensorTexture, p + small_step.xyy).r - texture(tensorTexture, p - small_step.xyy).r;
  float gradient_y = texture(tensorTexture, p + small_step.yxy).r - texture(tensorTexture, p - small_step.yxy).r;
  float gradient_z = texture(tensorTexture, p + small_step.yyx).r - texture(tensorTexture, p - small_step.yyx).r;

  vec3 normal = vec3(gradient_x, gradient_y, gradient_z);

  return normalize(normal);
}

void main(){
  //all in model coordinates:
  vec3 step_pos = surfacePoint_M;
  vec3 step_direction = normalize(surfacePoint_M - eyePosition_M);
  vec3 step_delta = .001 * step_direction; //(1.415/steps) * step_direction * ModelScale;

  float weight = .2;
  float density = 0.;
  vec3 normal = vec3(0,0,0);
  for(int i=0;i<=1000;i++){
    vec3 tensor_coord = step_pos/ModelScale;
    if(tensor_coord.x>=-.5 && tensor_coord.x<=.5 && tensor_coord.y>=-.5 && tensor_coord.y<=.5 && tensor_coord.z>=-.5 && tensor_coord.z<=.5){
      float d = texture(tensorTexture, tensor_coord+.5).r;
      vec3 n = calculate_normal(tensor_coord+.5);
      if(d<0.) d=0;
      if(d>1.) d=1.;
      //d = d*d;
      density += weight*d;
      normal += d * n;
      if(density>=1.) break;
    }else{
      if(i>10) break; //might need several steps to be inside
    }
    step_pos += step_delta;
  }

  //vec3 eyeDirection_M = normalize(eyePosition_M - surfacePoint_M);
  //normal = normalize(normal);
  vec3 lightDirection = normalize(vec3(1.,0,0));
  float cosTheta = clamp(abs(dot(normal, lightDirection)), 0.f, 1.f);

  color = flatColor;
  color.a = density;
  //float rgb = 1.; //flatColor.rgb; //objColor.rgb; // + .5*cosTheta;
  //color = vec4(rgb, rgb, rgb, density);
}


