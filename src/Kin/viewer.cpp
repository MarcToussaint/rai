/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "viewer.h"
#include "frame.h"
#include "../Core/thread.h"
#include "../Gui/opengl.h"
#include <iomanip>
#include <Kin/forceExchange.h>

rai::ConfigurationViewer::~ConfigurationViewer() { close_gl(); }

OpenGL& rai::ConfigurationViewer::ensure_gl() {
  if(!gl) {
    gl = make_shared<OpenGL>("ConfigurationViewer");
    gl->camera.setDefault();
    gl->add(*this);
  }
  return *gl;
}

void rai::ConfigurationViewer::close_gl() {
  if(gl) gl.reset();
}

void rai::ConfigurationViewer::recopyMeshes(const rai::Configuration& C) {
  ensure_gl().dataLock.lock(RAI_HERE);
  CHECK(!objs.N, "");
  addLight({5.,5.,5.}, {0.,0.,1.});
  addLight({-5.,0.,5.}, {0.,0.,1.});
  { // floor
    rai::Mesh m;
    m.setQuad();
    m.scale(5., 5., 0.);
    m.C = {.4, .45, .5};
    add().mesh(m, 0);
  }

  frame2objID.resize(C.frames.N) = -1;
  for(rai::Frame* f:C.frames) if(f->shape) {
    shared_ptr<Mesh> mesh = f->shape->_mesh;
    if(mesh && mesh->V.N){
      frame2objID(f->ID) = objs.N;
      add().mesh(*mesh, f->ensure_X());
    }
  }
  for(rai::Frame* f:C.frames) if(f->shape && f->shape->type()==ST_marker) {
    frame2objID(f->ID) = objs.N;
    addAxes(f->shape->size(-1), f->ensure_X());
  }
  ensure_gl().dataLock.unlock();
}

void rai::ConfigurationViewer::updateConfiguration(const rai::Configuration& C) {
  bool copyMeshes = false;
  if(!objs.N) copyMeshes = true;

//  if(newC.frames.N!=C.frames.N){
//    copyMeshes = true;
//  }else {
//    for(uint i=0; i<C.frames.N; i++) {
//      rai::Shape* s = newC.frames.elem(i)->shape;
//      rai::Shape* r = C.frames.elem(i)->shape;
//      if((!s) != (!r)) { copyMeshes=true; break; }
//      if(!s) continue;
//      if(s->_type != r->_type) { copyMeshes=true; break; }
//      if(s->size != r->size) { copyMeshes=true; break; }
//      if(s->_mesh && r->_mesh && (s->_mesh.get() != r->_mesh.get())) { copyMeshes=true; break; }
//      if(s->_mesh->version != r->_mesh->version) { copyMeshes=true; break; }
//    }
//  }
  if(copyMeshes) recopyMeshes(C);

  ensure_gl();

  {
    auto _dataLock = gl->dataLock(RAI_HERE);
    for(uint i=0; i<C.frames.N; i++) {
      rai::Frame* f = C.frames.elem(i);
      RenderObject* obj = objs.elem(frame2objID(f->ID)).get();
      //shape pose
      if(f->shape) obj->X = f->ensure_X();
      //forces
      if(f->forces.N){
        NIY;
//        CHECK_EQ(fnew->forces.N, fold->forces.N, "");
//        for(uint j=0;j<fnew->forces.N;j++){
//          fold->forces.elem(j)->copy(*fnew->forces.elem(j));
//        }
      }
    }
  }

  if(C.proxies.N) {
    auto _dataLock = gl->dataLock(RAI_HERE);
    NIY;
//    C.copyProxies(newC.proxies);
  }
}

void rai::ConfigurationViewer::setCamera(rai::Frame* camF) {
  ensure_gl();
  rai::Camera& cam = gl->camera;
  {
    auto _dataLock = gl->dataLock(RAI_HERE);
    if(camF) {
      cam.X = camF->ensure_X();

      rai::Node* at=0;
      if((at=camF->ats->getNode("focalLength"))) cam.setFocalLength(at->as<double>());
      if((at=camF->ats->getNode("orthoAbsHeight"))) cam.setHeightAbs(at->as<double>());
      if((at=camF->ats->getNode("zRange"))) { arr z=at->as<arr>(); cam.setZRange(z(0), z(1)); }
      if((at=camF->ats->getNode("width"))) gl->width=at->as<double>();
      if((at=camF->ats->getNode("height"))) gl->height=at->as<double>();
      //    cam.setWHRatio((double)gl->width/gl->height);
    } else {
      gl->camera.setDefault();
    }
  }
  gl->resize(gl->width, gl->height);
}

int rai::ConfigurationViewer::_update(const char* text, bool nonThreaded) { ensure_gl(); return gl->update(text, nonThreaded); }

int rai::ConfigurationViewer::_watch(const char* text) { ensure_gl(); return gl->watch(text); }

void rai::ConfigurationViewer::_add(GLDrawer& c) { ensure_gl(); gl->add(c); }

void rai::ConfigurationViewer::_resetPressedKey() { ensure_gl(); gl->pressedkey=0; }

int rai::ConfigurationViewer::update(bool watch) {
  ensure_gl();

  int ret=0;
  if(watch) {
    gl->raiseWindow();
    ret = gl->watch();
  } else {
    ret = gl->update(nullptr, false);
  }

  return ret;
}

void rai::ConfigurationViewer::raiseWindow() {
  ensure_gl();
  gl->raiseWindow();
}

int rai::ConfigurationViewer::setConfiguration(const rai::Configuration& C, const char* _text, bool watch, const FrameL& timeSlices) {
  updateConfiguration(C);

  {
    auto _dataLock = gl->dataLock(RAI_HERE);
    for(rai::Frame* f:C.frames) CHECK(f->_state_X_isGood, "");
    drawSlice=-1;
    if(timeSlices.N){
      slices.resizeAs(timeSlices);
      NIY;
//      for(uint i=0;i<slices.N;i++) slices.elem(i) = C.frames.elem(timeSlices.elem(i)->ID);
    }else{
      slices.clear();
    }
    if(_text) text = _text;
  }

  rai::Frame* camF = C.getFrame("camera_gl", false);
  if(camF) setCamera(camF);


  return update(watch);
}

bool rai::ConfigurationViewer::playVideo(const FrameL& timeSlices, bool watch, double delay, const char* saveVideoPath) {
  if(rai::getDisableGui()) return false;

  if(saveVideoPath) {
    rai::system(STRING("mkdir -p " <<saveVideoPath));
    rai::system(STRING("rm -f " <<saveVideoPath <<"*.png"));
  }

  NIY; //CHECK_GE(C.frames.N, timeSlices.N, "");

  {
    auto _dataLock = gl->dataLock(RAI_HERE);
    slices.resizeAs(timeSlices);
//    for(uint i=0;i<slices.N;i++) slices.elem(i) = C.frames.elem(timeSlices.elem(i)->ID);
    NIY;
    drawSlice = 0;
    abortPlay=false;
    gl->scrollCounter = 0;
  }

  Metronome tic(delay / timeSlices.d0);

  int key=0;
  for(uint t=0; t<slices.d0; t++) {
    if(t && delay>0.) tic.waitForTic(); //rai::wait(delay / F.d0);

    if(abortPlay){ watch=true; break; }

    key = view_slice(t, delay<0.);

    {
      auto _dataLock = gl->dataLock(RAI_HERE);
      if(saveVideoPath) write_png(gl->captureImage, STRING(saveVideoPath<<std::setw(4)<<std::setfill('0')<<t<<".png"));
    }
  }
  key = update(watch);
//  drawText = tag;
//  slices.clear();
  return !(key==27 || key=='q' || !rai::getInteractivity());
}

int rai::ConfigurationViewer::view_slice(uint t, bool watch){
  {
    auto _dataLock = gl->dataLock(RAI_HERE);
    drawSlice = t;
  }

  return update(watch);
}

void rai::ConfigurationViewer::savePng(const char* saveVideoPath) {
  write_png(gl->captureImage, STRING(saveVideoPath<<std::setw(4)<<std::setfill('0')<<(pngCount++)<<".png"), true);
}

rai::Camera& rai::ConfigurationViewer::displayCamera() {
  ensure_gl();
  return gl->camera;
}

byteA rai::ConfigurationViewer::getRgb() {
  ensure_gl();
  byteA image = gl->captureImage;
  flip_image(image);
  return image;
}

floatA rai::ConfigurationViewer::getDepth() {
  ensure_gl();
  floatA depth = gl->captureDepth;
  flip_image(depth);
  for(float& d:depth) {
    if(d==1.f || d==0.f) d=-1.f;
    else d = gl->camera.glConvertToTrueDepth(d);
  }
  return depth;
}

void rai::ConfigurationViewer::glDraw(OpenGL& gl) {
  RenderScene::glDraw(gl);
  return;

#ifdef RAI_GL
  glStandardScene(NULL, gl);

  glPushMatrix();

  rai::Transformation T;

  //draw frame paths
  if(drawFrameLines) {
    glColor(0., 0., 0., .2);
    glLoadIdentity();
  }

  if(slices.N && drawSlice>=0) {
    if(gl.scrollCounter){ drawSlice+=gl.scrollCounter; gl.scrollCounter=0; abortPlay=true; }
    if(drawSlice<0) drawSlice=0;
    if(drawSlice>=(int)slices.d0) drawSlice=slices.d0-1;

    gl.text.clear() <<text <<"\n(slice " <<drawSlice <<'/' <<slices.d0;
    if(phaseFactor>0.) gl.text <<", phase " <<phaseFactor*(double(drawSlice)+phaseOffset);
    gl.text <<")";
    if(drawSlice<(int)sliceTexts.N) gl.text <<"\n" <<sliceTexts(drawSlice);

    NIY; //C.glDraw_frames(gl, slices[drawSlice], 0);
  } else {
    if(gl.scrollCounter && slices.N){
      drawSlice=0;
    }else{
      gl.text.clear() <<text;
      NIY; //C.glDraw_frames(gl, C.frames, 0);
    }
  }

  glPopMatrix();
#else
  NICO
#endif
}

