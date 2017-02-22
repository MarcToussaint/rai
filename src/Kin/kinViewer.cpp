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

#include <iomanip>

//===========================================================================

OrsViewer::OrsViewer(const char* varname, double beatIntervalSec, bool computeCameraView)
  : Thread("OrsViewer", beatIntervalSec),
    modelWorld(this, varname, (beatIntervalSec<0.)),
    modelCameraView(this, "modelCameraView"),
    modelDepthView(this, "modelDepthView"),
    computeCameraView(computeCameraView){
  if(beatIntervalSec>=0.) threadLoop();
}

OrsViewer::~OrsViewer(){ threadClose(); }

void OrsViewer::open(){
  copy.gl(modelWorld.name);
}

void OrsViewer::step(){
  copy.gl().lock.writeLock();
  copy = modelWorld.get();
  copy.gl().lock.unlock();
  copy.gl().update(NULL, false, false, true);
  if(computeCameraView){
    mlr::Shape *kinectShape = copy.getShapeByName("endeffKinect");
    if(kinectShape){ //otherwise 'copy' is not up-to-date yet
      copy.gl().lock.writeLock();
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
      copy.gl().lock.unlock();
    }
  }
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
  : Thread("OrsPathViewer", beatIntervalSec),
    configurations(this, varname, (beatIntervalSec<0.)),
    t(0), tprefix(tprefix), writeToFiles(false){}

OrsPathViewer::~OrsPathViewer(){ threadClose(); clear(); }

void OrsPathViewer::open(){
  copy.gl(configurations.name);
}

void OrsPathViewer::step(){
  copy.gl().lock.writeLock();
  configurations.readAccess();
  uint T=configurations().N;
  if(t>=T) t=0;
  if(T) copy.copy(*configurations()(t), true);
  configurations.deAccess();
  copy.gl().lock.unlock();
  if(T){
    copy.gl().captureImg=writeToFiles;
    copy.gl().update(STRING(" (time " <<tprefix+int(t) <<'/' <<tprefix+int(T) <<')').p, false, false, true);
    if(writeToFiles) write_ppm(copy.gl().captureImage,STRING("vid/z.path."<<std::setw(3)<<std::setfill('0')<<tprefix+int(t)<<".ppm"));
  }
  t++;
}

//===========================================================================

void changeColor(void*){  orsDrawColors=false; glColor(.5, 1., .5, .7); }
void changeColor2(void*){  orsDrawColors=true; orsDrawAlpha=1.; }

OrsPoseViewer::OrsPoseViewer(const char* modelVarName, const StringA& poseVarNames, double beatIntervalSec)
  : Thread("OrsPoseViewer", beatIntervalSec),
    modelWorld(this, modelVarName, false),
    gl(STRING("OrsPoseViewer:" <<poseVarNames)){
  for(const String& varname: poseVarNames){
    poses.append( new Access_typed<arr>(this, varname, (beatIntervalSec<0.)) ); //listen only when beatInterval=1.
    copies.append( new mlr::KinematicWorld() );
  }
  copy = modelWorld.get();
  computeMeshNormals(copy.shapes);
  for(mlr::KinematicWorld *w: copies) w->copy(copy, true);
  if(beatIntervalSec>=0.) threadLoop();
}

OrsPoseViewer::~OrsPoseViewer(){
  threadClose();
  listDelete(copies);
  listDelete(poses);
}

void OrsPoseViewer::recopyKinematics(const mlr::KinematicWorld& world){
//  copy = world;
//  gl.lock.writeLock();
  stepMutex.lock();
  if(&world) copy=world;
  else copy = modelWorld.get();
  computeMeshNormals(copy.shapes);
  for(mlr::KinematicWorld *w: copies) w->copy(copy, true);
  stepMutex.unlock();
//  gl.lock.unlock();
}

void OrsPoseViewer::open() {
  gl.add(glStandardScene, 0);
  gl.camera.setDefault();

  gl.add(changeColor2);
  for(uint i=0;i<copies.N;i++){
    gl.add(*copies(i));
    gl.add(changeColor);
  }
  gl.add(changeColor2);
  //  gl.camera.focus(0.6, -0.1, 0.65);
  //  gl.width = 1280;
  //  gl.height = 960;
}

void OrsPoseViewer::step(){
  listCopy(copies.first()->proxies, modelWorld.get()->proxies);
//  cout <<copy.proxies.N <<endl;
  gl.lock.writeLock();
  for(uint i=0;i<copies.N;i++){
    arr q=poses(i)->get();
    if(q.N==copies(i)->getJointStateDimension())
      copies(i)->setJointState(q);
  }
  gl.lock.unlock();
  gl.update(NULL, false, false, true);
}

void OrsPoseViewer::close(){
  gl.clear();
}

//===========================================================================

ComputeCameraView::ComputeCameraView(uint skipFrames)
  : Thread("ComputeCameraView"),
    modelWorld(this, "modelWorld", true),
    cameraView(this, "cameraView"),
    skipFrames(skipFrames), frame(0){}

ComputeCameraView::~ComputeCameraView(){
  modelWorld.stopListening();
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
  if(!frame--){
    copy = modelWorld.get();
//    modelWorld.readAccess();

    mlr::Shape *kinectShape = copy.getShapeByName("endeffKinect");
    if(kinectShape){ //otherwise 'copy' is not up-to-date yet
      gl.lock.writeLock();
      gl.camera.setKinect();
      gl.camera.X = kinectShape->X * gl.camera.X;
      gl.lock.unlock();
      gl.renderInBack(true, true, 580, 480);
      cameraView.set() = gl.captureImage;
//      depthView.set() = gl.captureDepth;
    }
//    modelWorld.deAccess();

    frame=skipFrames;
  }
}


