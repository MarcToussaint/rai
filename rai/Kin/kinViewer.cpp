/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kinViewer.h"
#include "proxy.h"
#include "frame.h"

#include <iomanip>

//===========================================================================

KinViewer::KinViewer(const Var<rai::Configuration>& _kin, double beatIntervalSec, const char* _cameraFrameName)
  : Thread("KinViewer", beatIntervalSec),
    world(this, _kin, (beatIntervalSec<0.)) {
  if(_cameraFrameName && strlen(_cameraFrameName)>0) {
    cameraFrameID =  world.get()->getFrame(_cameraFrameName, true)->ID;
  }
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

KinViewer::~KinViewer() {
  threadClose();
}

void KinViewer::open() {
  gl = new OpenGL(STRING("KinViewer: "<<world.name()));
  gl->add(glStandardScene);
  gl->add(glDrawMeshes, &meshesCopy);
//  gl->add(rai::glDrawProxies, &proxiesCopy);
  gl->camera.setDefault();
  if(cameraFrameID>=0) {
    rai::Frame* frame = world.get()->frames(cameraFrameID);
    double d;
    arr z;
    if(frame->ats.get<double>(d, "focalLength")) gl->camera.setFocalLength(d);
    if(frame->ats.get<arr>(z, "zrange")) gl->camera.setZRange(z(0), z(1));
    uint w=0, h=0;
    if(frame->ats.get<double>(d, "width")) w = (uint)d;
    if(frame->ats.get<double>(d, "height")) h = (uint)d;
    if(w && h) {
      gl->resize(w, h);
      gl->camera.setWHRatio((double)w/h);
    }
  }
}

void KinViewer::close() {
  proxiesCopy.clear();
  delete gl;
}

void KinViewer::step() {
  //-- get transforms, or all shapes if their number changed, and proxies
  rai::Array<rai::Transformation> X;
  world.readAccess();
  if(true || world->frames.N!=meshesCopy.N) { //need to copy meshes
    uint n=world->frames.N;
    auto _dataLock = gl->dataLock(RAI_HERE);
    meshesCopy.resize(n);
    for(uint i=0; i<n; i++) {
      if(world->frames.elem(i)->shape) meshesCopy.elem(i) = world->frames.elem(i)->shape->mesh();
      else meshesCopy.elem(i).clear();
    }
  }
  X.resize(world->frames.N);
  for(rai::Frame* f:world().frames) X(f->ID) = f->ensure_X();

  {
    auto _dataLock = gl->dataLock(RAI_HERE);

//  proxiesCopy.resize(world->proxies.N);
//  for(uint i=0;i<proxiesCopy.N;i++) proxiesCopy(i).copy(NoWorld, world->proxies(i));
//  proxiesCopy = world->proxies;

    if(cameraFrameID>=0) {
      gl->camera.X = world->frames(cameraFrameID)->ensure_X();
    }
  }
  world.deAccess();

  //-- set transforms to mesh display
  {
    auto _dataLock = gl->dataLock(RAI_HERE);
    CHECK_EQ(X.N, meshesCopy.N, "");
    for(uint i=0; i<X.N; i++) meshesCopy(i).glX = X(i);
  }

  gl->update(nullptr, true); //nullptr, false, false, true);
}

//===========================================================================

void KinPathViewer::setConfigurations(const ConfigurationL& cs) {
  configurations.writeAccess();
  listResize(configurations(), cs.N);
  for(uint i=0; i<cs.N; i++) configurations()(i)->copy(*cs(i), true);
  configurations.deAccess();
}

void KinPathViewer::clear() {
  listDelete(configurations.set()());
  text.clear();
}

KinPathViewer::KinPathViewer(const Var<ConfigurationL>& _configurations, double beatIntervalSec, int tprefix)
  : Thread(STRING("KinPathViewer_"<<_configurations.name()), beatIntervalSec),
    configurations(this, _configurations, (beatIntervalSec<0.)),
    t(0), tprefix(tprefix), writeToFiles(false) {
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

KinPathViewer::~KinPathViewer() {
  threadClose();
  clear();
}

void KinPathViewer::open() {
  gl = new OpenGL(STRING("KinPathViewer: "<<configurations.name()));
  gl->add(glStandardScene);
  gl->add(copy);
  gl->camera.setDefault();
}

void KinPathViewer::close() {
  delete gl;
}

void KinPathViewer::step() {
  uint T, tt;
  {
    auto _dataLock = gl->dataLock(RAI_HERE);
    configurations.readAccess();
    T=configurations().N;
    if(t>=T*1.1) t=0;
    tt=t;
    if(tt>=T) tt=T-1;
    if(T) copy.copy(*configurations()(tt), true);
    configurations.deAccess();
    copy.checkConsistency();
  }
  if(T) {
    copy.orsDrawMarkers=false;
    gl->update(STRING("(time " <<tprefix+int(tt) <<'/' <<tprefix+int(T) <<")\n" <<text).p, true); //, false, false, true);
    if(writeToFiles) write_ppm(gl->captureImage, STRING("vid/"<<std::setw(4)<<std::setfill('0')<<tprefix+int(tt)<<".ppm"));
  }
  t++;
}

//===========================================================================

void renderConfigurations(const ConfigurationL& cs, const char* filePrefix, int tprefix, int w, int h, rai::Camera* camera) {
  rai::Configuration copy;
  copy.orsDrawMarkers=false;
  rai::system(STRING("mkdir -p " <<filePrefix));
  rai::system(STRING("rm -f " <<filePrefix <<"*.ppm"));
  OpenGL gl("RenderConfiguration", w, h, true);
  gl.add(glStandardScene, 0);
  gl.add(copy);
  if(camera) {
    gl.camera = *camera;
  } else {
    gl.camera.setDefault();
    gl.camera.focus(.5, 0., .7);
  }
  for(uint t=0; t<cs.N; t++) {
    copy.copy(*cs(t), true);
    gl.update(STRING(" (time " <<tprefix+int(t) <<'/' <<tprefix+int(cs.N) <<')').p, true);
    write_ppm(gl.captureImage, STRING(filePrefix<<std::setw(4)<<std::setfill('0')<<t<<".ppm"));
  }
}

//===========================================================================

//KinPoseViewer::KinPoseViewer(const char* modelVarName, const StringA& poseVarNames, double beatIntervalSec)
//  : Thread(STRING("KinPoseViewer_"<<poseVarNames), beatIntervalSec),
//    model(this, modelVarName, false),
//    gl(STRING("KinPoseViewer: " <<poseVarNames)) {
//  for(const String& varname: poseVarNames) {
//    poses.append(new Var<arr>(this, varname, (beatIntervalSec<0.)));   //listen only when beatInterval=1.
//    copies.append(new rai::Configuration());
//  }
//  copy = model.get();
//  computeMeshNormals(copy.frames);
//  for(rai::Configuration *w: copies) w->copy(copy, true);
//  if(beatIntervalSec>=0.) threadLoop();
//}

KinPoseViewer::KinPoseViewer(Var<rai::Configuration>& _kin, const Var<arr>& _frameState, double beatIntervalSec)
  : Thread("KinPoseViewer", beatIntervalSec),
    model(this, _kin, (beatIntervalSec<0.)),
    frameState(this, _frameState, (beatIntervalSec<0.)) {
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

KinPoseViewer::~KinPoseViewer() {
  threadClose();
}

void KinPoseViewer::open() {
  gl.add(*this);
  gl.camera.setDefault();
}

void KinPoseViewer::step() {
  {
    auto modelGet = model.get();
    if(true || modelGet->frames.N!=meshesCopy.N) { //need to copy meshes
      uint n=modelGet->frames.N;
      auto _dataLock = gl.dataLock(RAI_HERE);
      meshesCopy.resize(n);
      for(uint i=0; i<n; i++) {
        rai::Frame* f = modelGet->frames.elem(i);
        if(f->shape) meshesCopy.elem(i) = f->shape->mesh();
        else meshesCopy.elem(i).clear();
      }
    }
  }

  gl.text.clear() <<"step: " <<frameCount <<"\n[temporal profile is not displayed accuratly (tau path ignored)]";
  gl.text <<" path-T: " <<frameState.get()->d0 <<" frames: " <<meshesCopy.N;
  gl.update(nullptr, true); //nullptr, false, false, true);
}

void KinPoseViewer::close() {
  gl.clear();
}

void KinPoseViewer::glDraw(OpenGL& gl) {
#if 1 //def RAI_GL
  arr X = frameState.get();

  if(!meshesCopy.N) return;
  if(!X.N) return;

  glStandardScene(nullptr, gl);

  CHECK_EQ(X.nd, 3, "");
  uint n=meshesCopy.N;
  if(X.d1<n) n=X.d1;

  for(uint i=0; i<n; i++) {
    if(meshesCopy.elem(i).V.N) {
      if(frameCount >= X.d0) frameCount = 0;
      rai::Transformation T;
      T.set(&X.operator()(frameCount, i, 0));
      glTransform(T);
      meshesCopy.elem(i).glDraw(gl);
    }
  }
  frameCount++;
#endif
}

//===========================================================================

ComputeCameraView::ComputeCameraView(const Var<rai::Configuration>& _modelWorld, double beatIntervalSec)
  : Thread("ComputeCameraView", beatIntervalSec),
    modelWorld(this, _modelWorld, (beatIntervalSec<.0)),
    cameraView(this), //"cameraView"),
    cameraDepth(this), //"cameraDepth"),
    cameraFrame(this), //"cameraFrame"),
    getDepth(true) {
  if(beatIntervalSec<0.) threadOpen();
  else threadLoop();
}

ComputeCameraView::~ComputeCameraView() {
  threadClose();
}

void ComputeCameraView::open() {
  gl.add(glStandardLight);
  gl.add(copy);
}

void ComputeCameraView::close() {
  gl.clear();
}

void ComputeCameraView::step() {
  copy = modelWorld.get();
  copy.orsDrawJoints = copy.orsDrawMarkers = copy.orsDrawProxies = false;

  rai::Frame* kinectShape = copy.getFrame("endeffKinect");
  if(kinectShape) { //otherwise 'copy' is not up-to-date yet
    {
      auto _dataLock = gl.dataLock(RAI_HERE);
      gl.camera.setKinect();
      gl.camera.X = kinectShape->ensure_X() * gl.camera.X;
    }
    gl.renderInBack(640, 480);
    flip_image(gl.captureImage);
    flip_image(gl.captureDepth);
    cameraView.set() = gl.captureImage;
    if(getDepth) {
      floatA& D = gl.captureDepth;
      uint16A depth_image(D.d0, D.d1);
      for(uint i=0; i<D.N; i++) {
        depth_image.elem(i)
          = (uint16_t)(gl.camera.glConvertToTrueDepth(D.elem(i)) * 1000.);  // conv. from [m] -> [mm]
      }
      cameraDepth.set() = depth_image;
    }
    cameraFrame.set() = kinectShape->ensure_X();
  }
}

