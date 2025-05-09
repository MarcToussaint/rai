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
#include "../Kin/dof_forceExchange.h"
#include "../Kin/dof_direction.h"
#include "../Kin/proxy.h"

double shadowHeight = 3.; //5.;
//arr floorColor = ones(3);
arr floorColor = arr{.4, .45, .5};

rai::ConfigurationViewer::~ConfigurationViewer() {
  close_gl();
}

OpenGL& rai::ConfigurationViewer::ensure_gl() {
  if(!gl) {
    gl = make_shared<OpenGL>("ConfigurationViewer", 600, 500);
//    gl->reportEvents=true;
    if(opt.backgroundColor.N) gl->clearColor = convert<float>(opt.backgroundColor);
    gl->camera.setDefault();
    gl->add(this);
  }
  return *gl;
}

void rai::ConfigurationViewer::close_gl() {
  if(gl){
    gl->remove(this);
    gl.reset();
  }
}

void rai::ConfigurationViewer::recopyMeshes(const FrameL& frames) {
  {
    if(gl && gl->window) gl->beginContext();
    clear();
    if(gl && gl->window) gl->endContext();
  }

  auto lock = dataLock(RAI_HERE);

  addStandardScene();

  uint maxID = 0;
  if(frames.N){
    maxID = frames(-1)->ID;
    for(rai::Frame* f:frames) if(f->ID>maxID) maxID = f->ID;
  }

  frame2itemID.resize(maxID+1) = -1;
  for(rai::Frame* f:frames) if(f->shape) {
    shared_ptr<Mesh> mesh = f->shape->_mesh;
    if(mesh && mesh->V.N){
      //find mimic?
      rai::Frame *f_sharedMesh=0;
      for(rai::Frame* fm:frames) if(fm->shape && fm->shape->_mesh.get()==mesh.get()){
        if(fm!=f) f_sharedMesh=fm;
        break;
      }
//      f_mimic=0;
      frame2itemID(f->ID) = items.N;
      if(f_sharedMesh){
        std::shared_ptr<RenderItem>& o_mimic = items(frame2itemID(f_sharedMesh->ID));
        addShared(o_mimic, f->ensure_X(), o_mimic->type);
      }else if(f->shape->type()==ST_pointCloud){
        add(f->ensure_X(), _marker).pointCloud(mesh->V, mesh->C);
        items(-1)->asset->version = mesh->version;
      }else if(f->shape->type()==ST_lines){
        if(!mesh->isArrayFormatted) mesh->makeLinesArrayFormatted();
        add(f->ensure_X(), _marker).lines(mesh->V, mesh->C);
        items(-1)->asset->version = mesh->version;
      }else if(mesh->T.d1==3){
        add(f->ensure_X(), _solid).mesh(*mesh);
      }else if(mesh->T.d1==2){
        add(f->ensure_X(), _marker).lines(mesh->V, mesh->C);
      }else{
        NIY
      }
      items(-1)->flatColor = id2color_b(f->ID);
    }
    shared_ptr<SDF> sdf = f->shape->_sdf;
    if(sdf){
      if(f->shape->type()==ST_tensor){
        auto tensor = std::dynamic_pointer_cast<TensorShape>(sdf);
        add(f->ensure_X(), _tensor).tensor(tensor->gridData, f->shape->size);
        items(-1)->scale = f->shape->size;
      }else{
        NIY;
      }
      items(-1)->flatColor = id2color_b(f->ID);
    }
  }
  for(rai::Frame* f:frames) if(f->shape && f->shape->type()==ST_marker) {
    frame2itemID(f->ID) = items.N;
    double s = .1;
    if(f->shape->size.N) s = f->shape->size(-1);
    addAxes(s, f->ensure_X());
  }
}

rai::ConfigurationViewer& rai::ConfigurationViewer::updateConfiguration(const rai::Configuration& C, const FrameL& timeSlices, bool forceCopyMeshes) {
  bool copyMeshes = false;
  if(!items.N) copyMeshes = true;

  FrameL frames;
  if(timeSlices.nd==2) frames=timeSlices[0];
  else if(timeSlices.nd==1) frames=timeSlices;
  else frames = C.frames;
  frames.reshape(-1);

  //-- update meshes (checks if needed)
  if(frame2itemID.N!=C.frames.N){
    copyMeshes = true;
  } else {
    for(rai::Frame *f : C.frames) {
      int o = frame2itemID(f->ID);
      if(f->shape && f->shape->_mesh && f->shape->_mesh->V.N && o==-1){ copyMeshes=true; break; }
      if(o==-1) continue;
      rai::Shape* s = f->shape;
      if(!s || !s->_mesh){ copyMeshes=true; break; }
      if((int)items.N<=o){ copyMeshes=true; break; }
      if(s->_mesh->V.N && items(o)->asset->version != s->_mesh->version) { copyMeshes=true; break; }
    }
  }
  if(copyMeshes || forceCopyMeshes) recopyMeshes(frames);

  //-- update poses
  {
    auto lock = dataLock(RAI_HERE);
    for(rai::Frame* f : frames) {
      int objID = frame2itemID(f->ID);
      if(objID!=-1){
        items(objID)->X = f->ensure_X();
      }
    }
  }

  //-- update motion
  if(timeSlices.nd==2){
    auto lock = dataLock(RAI_HERE);
    drawSlice=-1;
    motion.resize(timeSlices.d0, items.N, 7).setZero();
    for(uint t=0;t<timeSlices.d0;t++) for(uint i=0;i<timeSlices.d1;i++){
      int o = frame2itemID(timeSlices(0,i)->ID);
      if(o!=-1){
        motion(t, o, {}) = timeSlices(t,i)->ensure_X().getArr7d();
      }
    }
  }else{
    motion.clear();
  }

  {
    auto lock = dataLock(RAI_HERE);
    distMarkers.pos.clear();
    distMarkers.color.clear();
    distMarkers.slices.clear();
  }

  //-- update proxies
  if(C.proxies.N) {
    auto lock = dataLock(RAI_HERE);
    for(const rai::Proxy& p: C.proxies){
      if(p.d<.05){
        int s=-1;
        if(timeSlices.N) s = p.a->ID/timeSlices.d1;
        addDistMarker(p.posA.getArr(), p.posB.getArr(), s, .1);
      }
    }
  }

  //-- update forces
  if(true){
    auto lock = dataLock(RAI_HERE);
    for(rai::Frame* fr: C.frames) for(ForceExchangeDof* f:fr->forces) if(f->sign(fr)>0.){

      arr _poa, _torque, _force;
      f->kinPOA(_poa, NoArr);
      f->kinForce(_force, NoArr);
      f->kinTorque(_torque, NoArr);

      int s=-1;
      if(timeSlices.N) s = fr->ID/timeSlices.d1;
      addDistMarker(_poa, _poa+.1*_force, s, .025, {1.,0.,1.});
      if(f->type==rai::FXT_wrench){
        addDistMarker(_poa, _poa+.1*_torque, s, .025, {1.,1.,0.});
      }
    }
  }

  //-- update directions
  {
    auto lock = dataLock(RAI_HERE);
    for(rai::Frame* f:frames) if(f->dirDof){
        arr p = f->getPosition();
        arr v = (f->get_X().rot * f->dirDof->vec).getArr();
        // LOG(0) <<*f->dirDof <<' ' <<v <<' ' <<p;
        int s=-1;
        if(timeSlices.N) s = f->ID/timeSlices.d1;
        addDistMarker(p, p+.2*v, s, .025);
      }
  }

  //-- update camera
  rai::Frame* camF = C.getFrame("camera_gl", false);
  if(camF) setCamera(camF);

  return *this;
}

void rai::ConfigurationViewer::setMotion(const uintA& frameIDs, const arr& _motion){
  CHECK_EQ(_motion.d1, frameIDs.N, "");
  CHECK_EQ(_motion.d2, 7, "");
  auto lock = dataLock(RAI_HERE);
  drawSlice=-1;
  //initialize with constant motion with current pose
  motion.resize(_motion.d0, items.N, 7).setZero();
  for(uint t=0;t<motion.d0;t++) for(uint o=0;o<motion.d1;o++){
    motion(t,o,{}) = items(o)->X.getArr7d();
  }
  //overwrite for indicated frames
  for(uint t=0;t<motion.d0;t++) for(uint i=0;i<frameIDs.N;i++){
    int o = frame2itemID(frameIDs(i));
    motion(t,o,{}) = _motion(t,i,{});
  }
}

void rai::ConfigurationViewer::setMotion(Configuration& C, const arr& path){
  CHECK_EQ(path.nd, 2, "");
  auto lock = dataLock(RAI_HERE);
  drawSlice=-1;
  motion.resize(path.d0, items.N, 7).setZero();
  for(uint t=0;t<path.d0;t++){
    C.setJointState(path[t]);
    for(rai::Frame* f:C.frames){
      int o = frame2itemID(f->ID);
      if(o!=-1){
        motion(t, o, {}) = f->ensure_X().getArr7d();
      }
    }
  }
}

void rai::ConfigurationViewer::setCamera(rai::Frame* camF) {
  ensure_gl();
  rai::Camera& cam = gl->camera;
  {
    auto lock = gl->dataLock(RAI_HERE);
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

void rai::ConfigurationViewer::_resetPressedKey() {
  ensure_gl();
  gl->pressedkey=0;
}

void rai::ConfigurationViewer::raiseWindow() {
  ensure_gl();
  gl->raiseWindow();
}

int rai::ConfigurationViewer::view(bool watch, const char* _text) {
  if(_text) text = _text;
  if(watch && (!text.N || text(-1)!=']')) text <<"\n[press key to continue]";

  ensure_gl();
  if(watch) gl->raiseWindow();
  return gl->update(watch, nonThreaded);
}

int rai::ConfigurationViewer::view_play(bool watch, double delay, str saveVideoPath) {
  if(rai::getDisableGui()) return false;

  if(saveVideoPath) {
    if(saveVideoPath(-1)=='/') rai::system(STRING("mkdir -p " <<saveVideoPath));
    rai::system(STRING("rm -f " <<saveVideoPath <<"*.png"));
  }

  CHECK(motion.nd==3, "");

  {
    auto lock = gl->dataLock(RAI_HERE);
    drawSlice = 0;
    abortPlay=false;
    gl->scrollCounter = 0;
  }

  Metronome tic(delay / motion.d0);

  int key=0;
  bool _nonThreaded = nonThreaded;
  nonThreaded = true;
  for(uint t=0; t<motion.d0; t++) {
    if(t && delay>0.) tic.waitForTic(); //rai::wait(delay / F.d0);

    if(abortPlay){ watch=true; break; }

    key = view_slice(t, delay<0.);

    if(saveVideoPath) savePng(saveVideoPath);
  }
  nonThreaded = _nonThreaded;
  key = view(watch);
//  drawText = tag;
  return key;
}

int rai::ConfigurationViewer::view_slice(uint t, bool watch){
  {
    auto lock = gl->dataLock(RAI_HERE);
    drawSlice = t;
  }

  return view(watch);
}

void rai::ConfigurationViewer::savePng(str saveVideoPath, int count) {
  nonThreaded=true;
  if(saveVideoPath && saveVideoPath(-1)=='/'){
    if(!FileToken(saveVideoPath).exists()){
      rai::system(STRING("mkdir -p " <<saveVideoPath));
    }
  }

  auto lock = gl->dataLock(RAI_HERE);
  if(count>=0) pngCount=count;
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
  //if(gl.drawOptions.drawVisualsOnly) renderUntil=_shadow; else renderUntil=_all;
  if(!motion.N){
    RenderData::setText(text);
    RenderData::slice=-1;
    RenderData::glDraw(gl);
  }else{
    if(gl.scrollCounter){ drawSlice-=gl.scrollCounter; gl.scrollCounter=0; abortPlay=true; }
    if(drawSlice<-1) drawSlice=-1;
    if(drawSlice>=(int)motion.d0) drawSlice=motion.d0-1;

    if(drawSlice>=0) {
      str s;
      s <<text <<"\n(slice " <<drawSlice <<'/' <<motion.d0;
      if(phaseFactor>0.) s <<", phase " <<phaseFactor*(double(drawSlice)+phaseOffset);
      s <<")";
      if(drawSlice<(int)sliceTexts.N){
//        cout <<"- ConfigurationViewer -\n" <<s <<"\n" <<sliceTexts(drawSlice) <<endl;
        s <<"\n" <<sliceTexts(drawSlice);
      }
      //C.glDraw_frames(gl, slices[drawSlice], 0);
      RenderData::setText(s);

      CHECK_LE(motion.d1, items.N, "");
      for(uint i=0;i<motion.d1;i++) items(i)->X.set(motion(drawSlice, i, {}));
      RenderData::slice=drawSlice;
      RenderData::glDraw(gl);
    }else{
      RenderData::setText(STRING(text <<"\n(motion T:" <<motion.d0 <<", use SHIFT-scroll or SHIFT-RIGHT/LEFT to browse)"));
      RenderData::slice=-1;
      for(uint t=0;t<motion.d0;t++){
        if(motion.d1>items.N) LOG(-1) <<"motion.d1>items.N" <<motion.d1 <<' ' <<items.N; //CHECK_LE(motion.d1, items.N, "");
        for(uint i=0;i<motion.d1 && items.N;i++) items(i)->X.set(motion(t, i, {}));
        RenderData::glDraw(gl);
        //C.glDraw_frames(gl, C.frames, 0);
      }
    }
  }
}

//===========================================================================

rai::ConfigurationViewerThread::ConfigurationViewerThread(const Var<rai::Configuration>& _config, double beatIntervalSec)
  : Thread("ConfigurationViewerThread", beatIntervalSec),
    config(this, _config, (beatIntervalSec<0.)) {
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

rai::ConfigurationViewerThread::~ConfigurationViewerThread() {
  threadClose();
}

void rai::ConfigurationViewerThread::open() {
  viewer = make_shared<ConfigurationViewer>();
  {
    auto C = config.get();
    for(Frame* f:C->frames) f->ensure_X();
    viewer->updateConfiguration(C).view(false);
  }
}

void rai::ConfigurationViewerThread::close() {
  viewer.reset();
}

void rai::ConfigurationViewerThread::step() {
  auto C = config.get();
  viewer->updateConfiguration(C);
}

