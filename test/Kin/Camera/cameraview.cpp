#include "cameraview.h"

rai::CameraView::CameraView(const rai::KinematicWorld& _K, bool _background)
  : K(_K), background(_background) {

  gl.add(*this);
}

void rai::CameraView::glDraw(OpenGL& gl) {
  //  glStandardLight(NULL);
  //  glEnable(GL_LIGHTING);
  glStandardScene(NULL);
  K.glDraw(gl);

  for(Sensor& sen:sensors){
    glDrawCamera(sen.cam);
    glDrawText(STRING("SENSOR " <<sen.name), 0., 0., 0.);
  }
}

