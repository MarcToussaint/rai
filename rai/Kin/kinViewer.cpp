/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kinViewer.h"
#include "proxy.h"
#include "frame.h"

#include <iomanip>

//===========================================================================

KinViewer_old::KinViewer_old(const char* varname, double beatIntervalSec, bool computeCameraView)
  : Thread(STRING("KinViewer_old_"<<varname), beatIntervalSec),
    modelWorld(this, varname, (beatIntervalSec<0.)),
    modelCameraView(this, "modelCameraView"),
    modelDepthView(this, "modelDepthView"),
    computeCameraView(computeCameraView) {
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

KinViewer_old::~KinViewer_old() { threadClose(); }

void KinViewer_old::open() {
  copy.gl(STRING("KinViewer_old: "<<modelWorld.name));
}

void KinViewer_old::step() {
  copy.gl().dataLock.writeLock();
  copy = modelWorld.get();
  copy.gl().dataLock.unlock();
  copy.gl().update(NULL, false, false, true);
  if(computeCameraView) {
    rai::Frame *kinectShape = copy.getFrameByName("endeffKinect");
    if(kinectShape) { //otherwise 'copy' is not up-to-date yet
      copy.gl().dataLock.writeLock();
      rai::Camera cam = copy.gl().camera;
      copy.gl().camera.setKinect();
      copy.gl().camera.X = kinectShape->X * copy.gl().camera.X;
//      openGlLock();
      copy.gl().renderInBack(true, true, 580, 480);
//      copy.glGetMasks(580, 480, true);
//      openGlUnlock();
      modelCameraView.set() = copy.gl().captureImage;
      modelDepthView.set() = copy.gl().captureDepth;
      copy.gl().camera = cam;
      copy.gl().dataLock.unlock();
    }
  }
}

//===========================================================================

KinViewer::KinViewer(const char* world_name, double beatIntervalSec)
  : Thread(STRING("KinViewer_"<<world_name), beatIntervalSec),
    world(this, world_name, (beatIntervalSec<0.)) {
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

KinViewer::~KinViewer() {
  threadClose();
}

void KinViewer::open() {
  gl = new OpenGL(STRING("KinViewer: "<<world.name));
  gl->add(glStandardScene);
  gl->add(glDrawMeshes, &meshesCopy);
//  gl->add(rai::glDrawProxies, &proxiesCopy);
  gl->camera.setDefault();
}

void KinViewer::close() {
  proxiesCopy.clear();
  delete gl;
}

void KinViewer::step() {
  //-- get transforms, or all shapes if their number changed, and proxies
  rai::Array<rai::Transformation> X;
  world.readAccess();
  if(world->frames.N!=meshesCopy.N) { //need to copy meshes
    uint n=world->frames.N;
    gl->dataLock.writeLock();
    meshesCopy.resize(n);
    for(uint i=0; i<n; i++) {
      if(world->frames.elem(i)->shape) meshesCopy.elem(i) = world->frames.elem(i)->shape->mesh();
      else meshesCopy.elem(i).clear();
    }
    gl->dataLock.unlock();
  }
  X.resize(world->frames.N);
  for(rai::Frame *f:world().frames) X(f->ID) = f->X;
  gl->dataLock.writeLock();
//  proxiesCopy = world->proxies;
  gl->dataLock.unlock();
  world.deAccess();
  
  //-- set transforms to mesh display
  gl->dataLock.writeLock();
  CHECK_EQ(X.N, meshesCopy.N, "");
  for(uint i=0; i<X.N; i++) meshesCopy(i).glX = X(i);
  gl->dataLock.unlock();
  
  gl->update(NULL, false, false, true);
}

//===========================================================================

void KinPathViewer::setConfigurations(const WorldL& cs) {
  configurations.writeAccess();
  listResize(configurations(), cs.N);
  for(uint i=0; i<cs.N; i++) configurations()(i)->copy(*cs(i), true);
  configurations.deAccess();
}

void KinPathViewer::clear() {
  listDelete(configurations.set()());
  text.clear();
}

KinPathViewer::KinPathViewer(const char* varname, double beatIntervalSec, int tprefix)
  : Thread(STRING("KinPathViewer_"<<varname), beatIntervalSec),
    configurations(this, varname, (beatIntervalSec<0.)),
    t(0), tprefix(tprefix), writeToFiles(false) {
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

KinPathViewer::~KinPathViewer() {
  threadClose();
  clear();
}

void KinPathViewer::open() {
  copy.gl(STRING("KinPathViewer: "<<configurations.name));
}

void KinPathViewer::step() {
  copy.gl().dataLock.writeLock();
  configurations.readAccess();
  uint T=configurations().N;
  if(t>=T*1.1) t=0;
  uint tt=t;
  if(tt>=T) tt=T-1;
  if(T) copy.copy(*configurations()(tt), true);
  configurations.deAccess();
  copy.checkConsistency();
  copy.gl().dataLock.unlock();
  if(T) {
    copy.orsDrawMarkers=false;
    copy.gl().computeImage=writeToFiles;
    copy.gl().update(STRING("(time " <<tprefix+int(tt) <<'/' <<tprefix+int(T) <<")\n" <<text).p, false, false, true);
    if(writeToFiles) write_ppm(copy.gl().captureImage,STRING("vid/z."<<std::setw(3)<<std::setfill('0')<<tprefix+int(tt)<<".ppm"));
  }
  t++;
}

//===========================================================================

void renderConfigurations(const WorldL& cs, const char* filePrefix, int tprefix, int w, int h, rai::Camera *camera) {
  rai::KinematicWorld copy;
  copy.orsDrawMarkers=false;
  rai::system(STRING("mkdir -p " <<filePrefix));
  rai::system(STRING("rm -f " <<filePrefix <<"*.ppm"));
  for(uint t=0; t<cs.N; t++) {
    copy.copy(*cs(t), true);
#if 0 //render on screen
    copy.gl().resize(w,h);
    copy.gl().doCaptureImage=true;
    copy.gl().update(STRING(" (time " <<tprefix+int(t) <<'/' <<tprefix+int(cs.N) <<')').p, false, false, true);
#else
    if(camera) {
      copy.gl().camera = *camera;
    } else {
      copy.gl().camera.setDefault();
      copy.gl().camera.focus(.5, 0., .7);
    }
    copy.gl().text.clear() <<"time " <<tprefix+int(t) <<'/' <<tprefix+int(cs.N);
    copy.gl().renderInBack(true, false, w, h);
#endif
    write_ppm(copy.gl().captureImage, STRING(filePrefix<<std::setw(3)<<std::setfill('0')<<t<<".ppm"));
  }
}

//===========================================================================

KinPoseViewer::KinPoseViewer(const char* modelVarName, const StringA& poseVarNames, double beatIntervalSec)
  : Thread(STRING("KinPoseViewer_"<<poseVarNames), beatIntervalSec),
    modelWorld(this, modelVarName, false),
    gl(STRING("KinPoseViewer: " <<poseVarNames)) {
  for(const String& varname: poseVarNames) {
    poses.append(new Var<arr>(this, varname, (beatIntervalSec<0.)));   //listen only when beatInterval=1.
    copies.append(new rai::KinematicWorld());
  }
  copy = modelWorld.get();
  computeMeshNormals(copy.frames);
  for(rai::KinematicWorld *w: copies) w->copy(copy, true);
  if(beatIntervalSec>=0.) threadLoop();
}

KinPoseViewer::~KinPoseViewer() {
  threadClose();
  listDelete(copies);
  listDelete(poses);
}

void KinPoseViewer::recopyKinematics(const rai::KinematicWorld& world) {
  stepMutex.lock();
  if(!!world) copy=world;
  else copy = modelWorld.get();
  computeMeshNormals(copy.frames);
  for(rai::KinematicWorld *w: copies) w->copy(copy, true);
  stepMutex.unlock();
}

void KinPoseViewer::open() {
  gl.add(glStandardScene, 0);
  gl.camera.setDefault();
  
  for(uint i=0; i<copies.N; i++) gl.add(*copies(i));
  //  gl.camera.focus(0.6, -0.1, 0.65);
  //  gl.width = 1280;
  //  gl.height = 960;
}

void KinPoseViewer::step() {
//  copies.first()->proxies = modelWorld.get()->proxies;
//  cout <<copy.proxies.N <<endl;
  gl.dataLock.writeLock();
  for(uint i=0; i<copies.N; i++) {
    arr q=poses(i)->get();
    if(q.N==copies(i)->getJointStateDimension())
      copies(i)->setJointState(q);
  }
  gl.dataLock.unlock();
  gl.update(NULL, false, false, true);
}

void KinPoseViewer::close() {
  gl.clear();
}

//===========================================================================

ComputeCameraView::ComputeCameraView(double beatIntervalSec, const char* modelWorld_name)
  : Thread("ComputeCameraView", beatIntervalSec),
    modelWorld(this, modelWorld_name, (beatIntervalSec<.0)),
    cameraView(this, "kinect_rgb"), //"cameraView"),
    cameraDepth(this, "kinect_depth"), //"cameraDepth"),
    cameraFrame(this, "kinect_frame"), //"cameraFrame"),
    getDepth(true) {
  if(beatIntervalSec<0.) threadOpen();
  else threadLoop();
}

ComputeCameraView::~ComputeCameraView() {
  threadClose();
}

void ComputeCameraView::open() {
  gl.add(glStandardLight);
  gl.addDrawer(&copy);
}

void ComputeCameraView::close() {
  gl.clear();
}

void ComputeCameraView::step() {
  copy = modelWorld.get();
  copy.orsDrawJoints = copy.orsDrawMarkers = copy.orsDrawProxies = false;
  
  rai::Frame *kinectShape = copy.getFrameByName("endeffKinect");
  if(kinectShape) { //otherwise 'copy' is not up-to-date yet
    gl.dataLock.writeLock();
    gl.camera.setKinect();
    gl.camera.X = kinectShape->X * gl.camera.X;
    gl.dataLock.unlock();
    gl.renderInBack(true, getDepth, 640, 480);
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
    cameraFrame.set() = kinectShape->X;
  }
}

