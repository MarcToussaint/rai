#include "viewer.h"
#include "frame.h"
#include "../Gui/opengl.h"
#include <iomanip>

rai::ConfigurationViewer::~ConfigurationViewer(){
  if(gl) gl.reset();
}

void rai::ConfigurationViewer::ensure_gl() {
  if(!gl) {
    gl = make_shared<OpenGL>("ConfigurationViewer");
    gl->camera.setDefault();
    gl->add(*this);
  }
}

int rai::ConfigurationViewer::update(const char* text, bool nonThreaded){ ensure_gl(); return gl->update(text, nonThreaded); }

int rai::ConfigurationViewer::watch(const char* text){ ensure_gl(); return gl->watch(text); }

void rai::ConfigurationViewer::add(GLDrawer& c) { ensure_gl(); gl->add(c); }

void rai::ConfigurationViewer::resetPressedKey(){ ensure_gl(); gl->pressedkey=0; }

int rai::ConfigurationViewer::update(bool watch) {
  ensure_gl();

  {
    auto _dataLock = gl->dataLock(RAI_HERE);
    gl->text = drawText;
    if(watch) gl->text <<"\n[ENTER]";
  }

  if(watch) {
    gl->watch();
    gl->text = drawText();
  }

  gl->update(nullptr, false);
  return gl->pressedkey;
}

int rai::ConfigurationViewer::setConfiguration(rai::Configuration& _C, const char* text, bool watch){
  ensure_gl();
  if(_C.frames.N!=C.frames.N){
    recopyMeshes(_C);
  }else if(_C.proxies.N){
    auto _dataLock = gl->dataLock(RAI_HERE);
    C.copyProxies(_C.proxies);
  }

  {
    auto _dataLock = gl->dataLock(RAI_HERE);
#if 0
    framePath.resize(_C.frames.N, 7);
    for(uint i=0;i<_C.frames.N;i++) framePath[i] = _C.frames(i)->getPose();
    framePath.reshape(1, _C.frames.N, 7);
#else
    framePath = _C.getFrameState();
    framePath.reshape(1, _C.frames.N, 7);
#endif
    drawTimeSlice=0;
    if(text) drawText = text;
  }

  return update(watch);
}

int rai::ConfigurationViewer::setPath(ConfigurationL& Cs, const char* text, bool watch) {
  CHECK(C.frames.N, "setPath requires that you setConfiguration first");

  uintA frames;
  frames.setStraightPerm(Cs.first()->frames.N);

  arr X(Cs.N, frames.N, 7);
  for(uint t=0; t<X.d0; t++) {
    for(uint i=0; i<X.d1; i++) {
      X(t, i, {}) = Cs(t)->frames(frames(i))->getPose();
    }
  }

  return setPath(X, text, watch);
}

int rai::ConfigurationViewer::setPath(rai::Configuration& _C, const arr& jointPath, const char* text, bool watch, bool full){
  CHECK(C.frames.N, "setPath requires that you setConfiguration first");

  arr X(jointPath.d0, _C.frames.N, 7);
  for(uint t=0; t<X.d0; t++) {
    _C.setJointState(jointPath[t]);
    for(uint i=0; i<X.d1; i++) {
      X(t, i, {}) = _C.frames.elem(i)->getPose();
    }
  }

  return setPath(X, text, watch);
}

int rai::ConfigurationViewer::setPath(const arr& _framePath, const char* text, bool watch, bool full) {
  CHECK(C.frames.N, "setPath requires that you setConfiguration first");

  CHECK_EQ(_framePath.nd, 3, "");
  CHECK_EQ(_framePath.d1, C.frames.N, "");
  CHECK_EQ(_framePath.d2, 7, "");

  {
    auto _dataLock = gl->dataLock(RAI_HERE);
    framePath = _framePath;
    drawFullPath = full;
    drawTimeSlice=-1;
    if(text) drawText = text;
  }

  return update(watch);
}

bool rai::ConfigurationViewer::playVideo(bool watch, double delay, const char* saveVideoPath) {
#ifndef RAI_GL
  return false;
#endif
  const rai::String tag = drawText;

  if(saveVideoPath) {
    rai::system(STRING("mkdir -p " <<saveVideoPath));
    rai::system(STRING("rm -f " <<saveVideoPath <<"*.ppm"));
  }

  for(uint t=0;t<framePath.d0;t++) {
    {
      auto _dataLock = gl->dataLock(RAI_HERE);
      drawTimeSlice=t;
      drawText.clear() <<tag <<" (config:" <<t <<'/' <<framePath.d0 <<")";
    }

    if(delay<0.) {
      update(true);
    } else {
      update(false);
      if(delay) rai::wait(delay / framePath.d0);
    }

    {
      auto _dataLock = gl->dataLock(RAI_HERE);
      if(saveVideoPath) write_ppm(gl->captureImage, STRING(saveVideoPath<<std::setw(4)<<std::setfill('0')<<t<<".ppm"));
    }
  }
  drawText = tag;
  if(watch) {
    int key = update(true);
    return !(key==27 || key=='q');
  }
  return false;
}

void rai::ConfigurationViewer::savePng(const char* saveVideoPath){
  write_ppm(gl->captureImage, STRING(saveVideoPath<<std::setw(4)<<std::setfill('0')<<(pngCount++)<<".ppm"));
}

rai::Camera& rai::ConfigurationViewer::displayCamera() {
  ensure_gl();
  return gl->camera;
}

byteA rai::ConfigurationViewer::getScreenshot(){
  ensure_gl();
  return gl->captureImage;
}

void rai::ConfigurationViewer::recopyMeshes(rai::Configuration& _C){
  ensure_gl();

  {
    auto _dataLock = gl->dataLock(RAI_HERE);
    C.copy(_C, false);
    //deep copy meshes!
    for(rai::Frame* f:C.frames) if(f->shape) {
      ptr<Mesh> org = f->shape->_mesh;
      f->shape->_mesh = make_shared<Mesh> (*org.get());
    }
  }
}

void rai::ConfigurationViewer::glDraw(OpenGL& gl) {
#ifdef RAI_GL
  glStandardScene(NULL, gl);

  if(!framePath.N){
    gl.text <<"\nConfigurationViewer: NOTHING TO DRAW";
    return;
  }

  glPushMatrix();

  rai::Transformation T;

  //draw frame paths
  if(drawFrameLines){
    glColor(0., 0., 0.,.2);
    glLoadIdentity();
    for(uint i=0; i<framePath.d1; i++) {
      glBegin(GL_LINE_STRIP);
      for(uint t=0; t<framePath.d0; t++) {
        T.set(&framePath(t, i, 0));
        //          glTransform(pose);
        glVertex3d(T.pos.x, T.pos.y, T.pos.z);
      }
      glEnd();
    }
  }

  if(drawTimeSlice>=0){
    uint t=drawTimeSlice;
    CHECK_LE(t+1, framePath.d0, "");
    CHECK_EQ(framePath.d1, C.frames.N, "");
    CHECK_EQ(framePath.d2, 7, "");

    C.setFrameState(framePath[t]);
    C.glDraw_sub(gl, 0);
  }else{
    if(drawFullPath){
      CHECK_EQ(framePath.d1, C.frames.N, "");
      CHECK_EQ(framePath.d2, 7, "");
      for(uint t=0;t<framePath.d0;t++){
        C.setFrameState(framePath[t]);
        C.glDraw_sub(gl, 1); //opaque
      }
      for(uint t=0;t<framePath.d0;t++){
        C.setFrameState(framePath[t]);
        C.glDraw_sub(gl, 2); //transparent
      }
    }else{
      NIY;
    }
  }

  glPopMatrix();
#else
  NICO
#endif
}

