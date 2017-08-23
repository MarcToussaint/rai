/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "kinViewer.h"
#include "proxy.h"
#include "frame.h"

#include <iomanip>

//===========================================================================

OrsViewer_old::OrsViewer_old(const char* varname, double beatIntervalSec, bool computeCameraView)
  : Thread(STRING("OrsViewer_old_"<<varname), beatIntervalSec),
    modelWorld(this, varname, (beatIntervalSec<0.)),
    modelCameraView(this, "modelCameraView"),
    modelDepthView(this, "modelDepthView"),
    computeCameraView(computeCameraView){
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

OrsViewer_old::~OrsViewer_old(){ threadClose(); }

void OrsViewer_old::open(){
  copy.gl(STRING("OrsViewer_old: "<<modelWorld.name));
}

void OrsViewer_old::step(){
  copy.gl().dataLock.writeLock();
  copy = modelWorld.get();
  copy.gl().dataLock.unlock();
  copy.gl().update(NULL, false, false, true);
  if(computeCameraView){
    mlr::Frame *kinectShape = copy.getFrameByName("endeffKinect");
    if(kinectShape){ //otherwise 'copy' is not up-to-date yet
      copy.gl().dataLock.writeLock();
      mlr::Camera cam = copy.gl().camera;
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

OrsViewer::OrsViewer(const char* world_name, double beatIntervalSec)
  : Thread(STRING("OrsViewer_"<<world_name), beatIntervalSec),
    world(this, world_name, (beatIntervalSec<0.)){
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

OrsViewer::~OrsViewer(){
  threadClose();
}

void OrsViewer::open(){
  gl = new OpenGL(STRING("OrsViewer: "<<world.name));
  gl->add(glStandardScene);
  gl->add(glDrawMeshes, &meshesCopy);
  gl->add(mlr::glDrawProxies, &proxiesCopy);
  gl->camera.setDefault();
}

void OrsViewer::close(){
  listDelete(proxiesCopy);
  delete gl;
}

void OrsViewer::step(){
  //-- get transforms, or all shapes if their number changed, and proxies
  mlr::Array<mlr::Transformation> X;
  world.readAccess();
  if(world->frames.N!=meshesCopy.N){ //need to copy meshes
    uint n=world->frames.N;
    gl->dataLock.writeLock();
    meshesCopy.resize(n);
    for(uint i=0;i<n;i++){
      if(world->frames.elem(i)->shape) meshesCopy.elem(i) = world->frames.elem(i)->shape->mesh;
      else meshesCopy.elem(i).clear();
    }
    gl->dataLock.unlock();
  }
  X.resize(world->frames.N);
  for(mlr::Frame *f:world().frames) X(f->ID) = f->X;
  gl->dataLock.writeLock();
  listCopy(proxiesCopy, world->proxies);
  gl->dataLock.unlock();
  world.deAccess();

  //-- set transforms to mesh display
  gl->dataLock.writeLock();
  CHECK_EQ(X.N, meshesCopy.N, "");
  for(uint i=0;i<X.N;i++) meshesCopy(i).glX = X(i);
  gl->dataLock.unlock();

  gl->update(NULL, false, false, true);
}

//===========================================================================

void OrsPathViewer::setConfigurations(const WorldL& cs){
  configurations.writeAccess();
  listResize(configurations(), cs.N);
  for(uint i=0;i<cs.N;i++) configurations()(i)->copy(*cs(i), true);
  configurations.deAccess();
}

void OrsPathViewer::clear(){
  listDelete(configurations.set()());
}

OrsPathViewer::OrsPathViewer(const char* varname, double beatIntervalSec, int tprefix)
  : Thread(STRING("OrsPathViewer_"<<varname), beatIntervalSec),
    configurations(this, varname, (beatIntervalSec<0.)),
    t(0), tprefix(tprefix), writeToFiles(false){
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

OrsPathViewer::~OrsPathViewer(){
  threadClose();
  clear();
}

void OrsPathViewer::open(){
  copy.gl(STRING("OrsPathViewer: "<<configurations.name));
}

void OrsPathViewer::step(){
  copy.gl().dataLock.writeLock();
  configurations.readAccess();
  uint T=configurations().N;
  if(t>=T*1.1) t=0;
  uint tt=t;
  if(tt>=T) tt=T-1;
  if(T) copy.copy(*configurations()(tt), true);
  configurations.deAccess();
  copy.gl().dataLock.unlock();
  if(T){
    copy.gl().captureImg=writeToFiles;
    copy.gl().update(STRING(" (time " <<tprefix+int(tt) <<'/' <<tprefix+int(T) <<')').p, false, false, true);
    if(writeToFiles) write_ppm(copy.gl().captureImage,STRING("vid/z.path."<<std::setw(3)<<std::setfill('0')<<tprefix+int(tt)<<".ppm"));
  }
  t++;
}

//===========================================================================

void renderConfigurations(const WorldL& cs, const char* filePrefix, int tprefix, int w, int h, mlr::Camera *camera){
  mlr::KinematicWorld copy;
  copy.orsDrawMarkers=false;
  for(uint t=0;t<cs.N;t++){
    copy.copy(*cs(t), true);
#if 0 //render on screen
    copy.gl().resize(w,h);
    copy.gl().captureImg=true;
    copy.gl().update(STRING(" (time " <<tprefix+int(t) <<'/' <<tprefix+int(cs.N) <<')').p, false, false, true);
#else
    if(camera){
        copy.gl().camera = *camera;
    }else{
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

OrsPoseViewer::OrsPoseViewer(const char* modelVarName, const StringA& poseVarNames, double beatIntervalSec)
  : Thread(STRING("OrsPoseViewer_"<<poseVarNames), beatIntervalSec),
    modelWorld(this, modelVarName, false),
    gl(STRING("OrsPoseViewer: " <<poseVarNames)){
  for(const String& varname: poseVarNames){
    poses.append( new Access<arr>(this, varname, (beatIntervalSec<0.)) ); //listen only when beatInterval=1.
    copies.append( new mlr::KinematicWorld() );
  }
  copy = modelWorld.get();
  computeMeshNormals(copy.frames);
  for(mlr::KinematicWorld *w: copies) w->copy(copy, true);
  if(beatIntervalSec>=0.) threadLoop();
}

OrsPoseViewer::~OrsPoseViewer(){
  threadClose();
  listDelete(copies);
  listDelete(poses);
}

void OrsPoseViewer::recopyKinematics(const mlr::KinematicWorld& world){
  stepMutex.lock();
  if(&world) copy=world;
  else copy = modelWorld.get();
  computeMeshNormals(copy.frames);
  for(mlr::KinematicWorld *w: copies) w->copy(copy, true);
  stepMutex.unlock();
}

void OrsPoseViewer::open() {
  gl.add(glStandardScene, 0);
  gl.camera.setDefault();

  for(uint i=0;i<copies.N;i++) gl.add(*copies(i));
  //  gl.camera.focus(0.6, -0.1, 0.65);
  //  gl.width = 1280;
  //  gl.height = 960;
}

void OrsPoseViewer::step(){
  listCopy(copies.first()->proxies, modelWorld.get()->proxies);
//  cout <<copy.proxies.N <<endl;
  gl.dataLock.writeLock();
  for(uint i=0;i<copies.N;i++){
    arr q=poses(i)->get();
    if(q.N==copies(i)->getJointStateDimension())
      copies(i)->setJointState(q);
  }
  gl.dataLock.unlock();
  gl.update(NULL, false, false, true);
}

void OrsPoseViewer::close(){
  gl.clear();
}

//===========================================================================

ComputeCameraView::ComputeCameraView(double beatIntervalSec, const char* modelWorld_name)
  : Thread("ComputeCameraView", beatIntervalSec),
    modelWorld(this, modelWorld_name, (beatIntervalSec<.0)),
    cameraView(this, "kinect_rgb"), //"cameraView"),
    cameraDepth(this, "kinect_depth"), //"cameraDepth"),
    cameraFrame(this, "kinect_frame"), //"cameraFrame"),
    getDepth(true){
  if(beatIntervalSec<0.) threadOpen();
  else threadLoop();
}

ComputeCameraView::~ComputeCameraView(){
  threadClose();
}

void ComputeCameraView::open(){
  gl.add(glStandardLight);
  gl.addDrawer(&copy);
}

void ComputeCameraView::close(){
  gl.clear();
}

void ComputeCameraView::step(){
  copy = modelWorld.get();
  copy.orsDrawJoints = copy.orsDrawMarkers = copy.orsDrawProxies = false;

  mlr::Frame *kinectShape = copy.getFrameByName("endeffKinect");
  if(kinectShape){ //otherwise 'copy' is not up-to-date yet
    gl.dataLock.writeLock();
    gl.camera.setKinect();
    gl.camera.X = kinectShape->X * gl.camera.X;
    gl.dataLock.unlock();
    gl.renderInBack(true, getDepth, 640, 480);
    flip_image(gl.captureImage);
    flip_image(gl.captureDepth);
    cameraView.set() = gl.captureImage;
    if(getDepth){
      floatA& D = gl.captureDepth;
      uint16A depth_image(D.d0, D.d1);
      for(uint i=0;i<D.N;i++){
        depth_image.elem(i)
            = (uint16_t) (gl.camera.glConvertToTrueDepth(D.elem(i)) * 1000.); // conv. from [m] -> [mm]
      }
      cameraDepth.set() = depth_image;
    }
    cameraFrame.set() = kinectShape->X;
  }
}


