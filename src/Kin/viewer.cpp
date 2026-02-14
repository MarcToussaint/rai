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

namespace rai{

ConfigurationViewer::~ConfigurationViewer() {
  close_gl();
}

OpenGL& ConfigurationViewer::ensure_gl() {
  if(!gl) {
    gl = make_shared<OpenGL>("ConfigurationViewer", 600, 500);
//    gl->reportEvents=true;
    if(opt.backgroundColor.N) gl->clearColor = convert<float>(opt.backgroundColor);
    gl->camera.setDefault();
    gl->add(this);
  }
  return *gl;
}

void ConfigurationViewer::close_gl() {
  if(gl){
    gl->remove(this);
    gl.reset();
  }
}

void ConfigurationViewer::recopyMeshes(const FrameL& frames) {
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
    for(Frame* f:frames) if(f->ID>maxID) maxID = f->ID;
  }

  frame2itemID.resize(maxID+1) = -1;
  for(Frame* f:frames) if(f->shape) {
    shared_ptr<Mesh> mesh = f->shape->_mesh;
    if(mesh && mesh->V.N){
      //find mimic?
      Frame *f_sharedMesh=0;
      for(Frame* fm:frames) if(fm->shape && fm->shape->_mesh.get()==mesh.get()){
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
      }else if(f->shape->type()==ST_lines){
        if(!mesh->isArrayFormatted) mesh->makeLinesArrayFormatted();
        add(f->ensure_X(), _marker).lines(mesh->V, mesh->C);
      }else if(mesh->T.d1==3){
        add(f->ensure_X(), _solid).mesh(*mesh);
      }else if(mesh->T.d1==2){
        add(f->ensure_X(), _marker).lines(mesh->V, mesh->C);
      }else{
        NIY
      }
      items(-1)->asset->version = f->shape->version;
      items(-1)->flatColor = id2color_b(f->ID);
    }
    shared_ptr<SDF> sdf = f->shape->_sdf;
    if(sdf){
      frame2itemID(f->ID) = items.N;
      if(f->shape->type()==ST_tensor){
        auto tensor = std::dynamic_pointer_cast<TensorShape>(sdf);
        add(f->ensure_X(), _tensor).tensor(tensor->gridData, f->shape->size);
        items(-1)->scale = f->shape->size;
        items(-1)->asset->version = f->shape->version;
        items(-1)->flatColor = rai::convert<byte>(255.*tensor->color);
      }else{
        NIY;
      }
      // items(-1)->flatColor = id2color_b(f->ID);
    }
  }
  for(Frame* f:frames) if(f->shape && f->shape->type()==ST_marker) {
    frame2itemID(f->ID) = items.N;
    double s = .1;
    if(f->shape->size.N) s = f->shape->size(-1);
    addAxes(s, f->ensure_X());
    items(frame2itemID(f->ID))->asset->version = f->shape->version;
  }
}

ConfigurationViewer& ConfigurationViewer::updateConfiguration(const Configuration& C, const FrameL& timeSlices, bool forceCopyMeshes) {
  bool copyMeshes = false;
  if(!items.N) copyMeshes = true;

  FrameL frames;
  if(timeSlices.nd==2) frames=timeSlices[0];
  else if(timeSlices.nd==1) frames=timeSlices;
  else frames = C.frames;
  frames.reshape(-1);

  //-- check if we need to update meshes
  for(Frame *f : C.frames) {
    if(f->ID>=frame2itemID.N){ copyMeshes=true; break; }
    if(!f->shape) continue;
    int o = frame2itemID(f->ID);
    if(o==-1 && f->shape && f->shape->_mesh && f->shape->_mesh->V.N){ copyMeshes=true; break; }
    if(o==-1 && f->shape && (f->shape->_type==ST_marker || f->shape->_type==ST_tensor)){ copyMeshes=true; break; }
    if(o==-1) continue;
    if((int)items.N<=o){ copyMeshes=true; break; }
    if(items(o)->asset->version != f->shape->version) {
      // LOG(1) <<"frame " <<f->name <<" changed it's shape version (" <<f->shape->type() <<"): " <<items(o)->asset->version <<" -> " <<f->shape->version;
      copyMeshes=true; break;
    }
    // else LOG(1) <<"frame " <<f->name <<" has up-to-date shape version (" <<f->shape->type() <<"): " <<" -> " <<f->shape->version;
  }
  if(copyMeshes || forceCopyMeshes) recopyMeshes(frames);

  //-- update poses
  {
    auto lock = dataLock(RAI_HERE);
    for(Frame* f : frames) {
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
    for(const Proxy& p: C.proxies){
      int s=-1;
      if(timeSlices.N) s = p.A/timeSlices.d1;
      if(!p.collision){
        addDistMarker(C.frames(p.A)->getPosition(), C.frames(p.B)->getPosition(), s, .1);
      }else if(p.d<.05){
        addDistMarker(p.posA.getArr(), p.posB.getArr(), s, .1);
      }
    }
  }

  //-- update forces
  if(true){
    auto lock = dataLock(RAI_HERE);
    for(Frame* fr: C.frames) for(ForceExchangeDof* f:fr->forces) if(f->sign(fr)>0.){

      arr _poa, _torque, _force;
      f->kinPOA(_poa, NoArr);
      f->kinForce(_force, NoArr);
      f->kinTorque(_torque, NoArr);

      int s=-1;
      if(timeSlices.N) s = fr->ID/timeSlices.d1;
      addDistMarker(_poa, _poa+.1*_force, s, .025, {1.,0.,1.});
      if(f->type==FXT_wrench){
        addDistMarker(_poa, _poa+.1*_torque, s, .025, {1.,1.,0.});
      }
    }
  }

  //-- update directions
  {
    auto lock = dataLock(RAI_HERE);
    for(Frame* f:frames) if(f->dirDof){
        arr p = f->getPosition();
        arr v = (f->get_X().rot * f->dirDof->vec).getArr();
        // LOG(0) <<*f->dirDof <<' ' <<v <<' ' <<p;
        int s=-1;
        if(timeSlices.N) s = f->ID/timeSlices.d1;
        addDistMarker(p, p+.2*v, s, .025);
      }
  }

  //-- update camera
  if(!gl){
    Frame* camF = C.getFrame("camera_init", false);
    if(camF) setCamera(camF);
  }
  rai::Frame* camF = C.getFrame("camera_track", false);
  if(camF) setCamera(camF);

  return *this;
}

void ConfigurationViewer::setMotion(const uintA& frameIDs, const arr& _motion){
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

void ConfigurationViewer::setMotion(Configuration& C, const arr& path){
  CHECK_EQ(path.nd, 2, "");
  auto lock = dataLock(RAI_HERE);
  drawSlice=-1;
  motion.resize(path.d0, items.N, 7).setZero();
  for(uint t=0;t<path.d0;t++){
    C.setJointState(path[t]);
    for(Frame* f:C.frames){
      int o = frame2itemID(f->ID);
      if(o!=-1){
        motion(t, o, {}) = f->ensure_X().getArr7d();
      }
    }
  }
}

void ConfigurationViewer::setCamera(Frame* camFrame) {
  ensure_gl();
  Camera& cam = gl->camera;
  uint W=gl->width, H=gl->height;
  {
    auto lock = gl->dataLock(RAI_HERE);

    if(camFrame) {
      cam.X = camFrame->ensure_X();
      cam.alignFocus();

      if(camFrame->ats){
        Node* at=0;
        if((at=camFrame->ats->getNode("focalLength"))) cam.setFocalLength(at->asFlex<double>());
        if((at=camFrame->ats->getNode("orthoAbsHeight"))) cam.setHeightAbs(at->asFlex<double>());
        if((at=camFrame->ats->getNode("zRange"))) { arr z=at->as<arr>(); cam.setZRange(z(0), z(1)); }
        if((at=camFrame->ats->getNode("width"))) W=at->asFlex<double>();
        if((at=camFrame->ats->getNode("height"))) H=at->asFlex<double>();
        //    cam.setWHRatio((double)gl->width/gl->height);
      }
    } else {
      gl->camera.setDefault();
    }
  }
  if(W!=gl->width || H!=gl->height) gl->resize(W, H);
}

void ConfigurationViewer::focus(const arr& position, double heightAbs){
  rai::Camera& cam = displayCamera();
  arr pos = position;
  cam.focus(pos, true);
  double dist = heightAbs * cam.focalLength;
  pos -= dist * cam.X.rot.getZ().getArr();
  cam.setPosition(pos);
}

void ConfigurationViewer::setCameraPose(const arr& pose){
  displayCamera().X.set(pose);
}

arr ConfigurationViewer::getCameraPose(){
  return displayCamera().X.getArr7d();
}

void ConfigurationViewer::_resetPressedKey() {
  ensure_gl();
  gl->pressedkey=0;
}

int ConfigurationViewer::setQuad(int id, const byteA& rgb, float x, float y, float h){
  auto lock = dataLock(RAI_HERE);
  return RenderData::setQuad(id, rgb, x, y, h);
}

void ConfigurationViewer::raiseWindow() {
  ensure_gl();
  gl->raiseWindow();
}

void ConfigurationViewer::setWindow(const char* window_title, int w, int h){
  ensure_gl().setTitle(window_title);
  ensure_gl().resize(w, h);
}

int ConfigurationViewer::view(bool watch, const char* _text, bool offscreen) {
  if(_text) text = _text;
  if(watch && (!text.N || text(-1)!=']')) text <<"\n[press key to continue]";

  ensure_gl();
  if(offscreen){ gl->renderInBack(); return 0; }
  if(watch) gl->raiseWindow();
  return gl->update(watch, nonThreaded);
}

int ConfigurationViewer::view_play(bool watch, double delay, str saveVideoPath) {
  if(getDisableGui()) return false;

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
    if(t && delay>0.) tic.waitForTic(); //wait(delay / F.d0);

    if(abortPlay){ watch=true; break; }

    key = view_slice(t, delay<0.);

    if(saveVideoPath) savePng(saveVideoPath);
  }
  nonThreaded = _nonThreaded;
  key = view(watch);
//  drawText = tag;
  return key;
}

int ConfigurationViewer::view_slice(uint t, bool watch){
  {
    auto lock = gl->dataLock(RAI_HERE);
    drawSlice = t;
  }

  return view(watch);
}

void ConfigurationViewer::savePng(str saveVideoPath, int count) {
  nonThreaded=true;
  if(saveVideoPath && saveVideoPath(-1)=='/'){
    if(!FileToken(saveVideoPath).exists()){
      rai::system(STRING("mkdir -p " <<saveVideoPath));
    }
  }

  auto lock = gl->dataLock(RAI_HERE);
  if(count>=0) pngCount=count;
  write_png(gl->captureImage, STRING(saveVideoPath<<std::setw(6)<<std::setfill('0')<<(pngCount++)<<".png"), true);
}

Camera& ConfigurationViewer::displayCamera() {
  ensure_gl();
  return gl->camera;
}

byteA ConfigurationViewer::getRgb(bool _nonThreaded) {
  ensure_gl().needsUpdate.waitForStatusEq(0);

  // if(_nonThreaded && !nonThreaded){
  //   gl->update(false, true);
  //   nonThreaded=true;
  // }
  byteA image;
  {
    auto lock = gl->dataLock(RAI_HERE);
    image = gl->captureImage;
  }
  flip_image(image);
  return image;
}

floatA ConfigurationViewer::getDepth(bool _nonThreaded) {
  ensure_gl().needsUpdate.waitForStatusEq(0);
  // if(_nonThreaded && !nonThreaded){
  //   gl->update(false, true);
  //   nonThreaded=true;
  // }
  floatA depth;
  {
    auto lock = gl->dataLock(RAI_HERE);
    depth = gl->captureDepth;
  }
  flip_image(depth);
  for(float& d:depth) {
    if(d==1.f || d==0.f) d=-1.f;
    else d = gl->camera.glConvertToTrueDepth(d);
  }
  return depth;
}

void ConfigurationViewer::glDraw(OpenGL& gl) {
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
      RenderData::setText(0);
      RenderData::slice=-1;
      for(uint t=0;t<motion.d0;t++){
        if(motion.d1>items.N) LOG(-1) <<"motion.d1>items.N" <<motion.d1 <<' ' <<items.N; //CHECK_LE(motion.d1, items.N, "");
        for(uint i=0;i<motion.d1 && items.N;i++) items(i)->X.set(motion(t, i, {}));
        if(t==motion.d0-1) RenderData::setText(STRING(text <<"\n(motion T:" <<motion.d0 <<", use SHIFT-scroll or SHIFT-RIGHT/LEFT to browse)"));
        RenderData::glDraw(gl);
        //C.glDraw_frames(gl, C.frames, 0);
      }
    }
  }
}

//===========================================================================

struct ViewerEventHandler : OpenGL::GLClickCall, OpenGL::GLKeyCall, OpenGL::GLScrollCall, OpenGL::GLHoverCall {
  Mutex mux;
  // Configuration& C;
  bool blockDefaultHandler;
  arr cursor;
  StringA events;

  ViewerEventHandler(bool blockDefaultHandler) : blockDefaultHandler(blockDefaultHandler) {
  }

  virtual bool clickCallback(OpenGL& gl, int button, int buttonIsDown){
    str e;
    if(buttonIsDown) e <<"click down "; else e <<"click up ";
    if(button==0) e <<"left";
    else if(button==1) e <<"middle";
    else if(button==2) e <<"right";
    else e <<button;
    auto lock = mux(RAI_HERE);
    events.append(e);
    return !blockDefaultHandler;
  }

  virtual bool keyCallback(OpenGL& gl, int key, int mods, bool _keyIsDown){
    str e;
    if(key=='%'){
      e <<"mod";
      if(mods&1) e<<" shift";
      if(mods&2) e<<" ctrl";
      if(mods&4) e<<" alt";
    }else{
      e <<"key ";
      if(_keyIsDown) e <<"down "; else e <<"up ";
      if((key>='a' && key<='z') || (key>='A' && key<='Z')) e <<(char)key;
      else if(key==27) e <<"esc";
      else if(key==13) e <<"enter";
      else if(key==32) e <<"space";
      else if(key==8) e <<"backspace";
      else if(key==9) e <<"tab";
      else e <<key;
    }
    auto lock = mux(RAI_HERE);
    events.append(e);
    return !blockDefaultHandler;
  }

  virtual bool scrollCallback(OpenGL& gl, int direction){
    str e;
    e <<"scroll ";
    if(direction>0) e <<"up";
    else if(direction<0) e <<"down";
    auto lock = mux(RAI_HERE);
    events.append(e);
    return !blockDefaultHandler;
  }

  virtual bool hoverCallback(OpenGL& gl){
    arr normal;
    arr x = gl.get3dMousePos(normal);
    if(!x.N) return false; //outside window

    auto lock = mux(RAI_HERE);
    cursor = (x, normal);
    return !blockDefaultHandler;
  }
};

void ConfigurationViewer::setupEventHandler(bool blockDefaultHandler){
  eventHandler = make_shared<ViewerEventHandler>(blockDefaultHandler);
  ensure_gl().addHoverCall(eventHandler.get());
  ensure_gl().addKeyCall(eventHandler.get());
  ensure_gl().scrollCalls.append(eventHandler.get());
  ensure_gl().clickCalls.append(eventHandler.get());
}

arr ConfigurationViewer::getEventCursor(){
  arr cursor;
  {
    auto lock = eventHandler->mux(RAI_HERE);
    cursor = eventHandler->cursor;
  }
  return cursor;
}

uint ConfigurationViewer::getEventCursorObject(){
  uint id = gl->get3dMouseObjID();
  return id;
}

StringA ConfigurationViewer::getEvents(){
  StringA events;
  {
    auto lock = eventHandler->mux(RAI_HERE);
    if(eventHandler->events.N){ events = eventHandler->events; eventHandler->events.resize(0); }
  }
  return events;
}

Mutex& ConfigurationViewer::getEventsMutex(){
  return eventHandler->mux;
}




//===========================================================================

ConfigurationViewerThread::ConfigurationViewerThread(const Var<Configuration>& _config, double beatIntervalSec)
  : Thread("ConfigurationViewerThread", beatIntervalSec),
    config(this, _config, (beatIntervalSec<0.)) {
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

ConfigurationViewerThread::~ConfigurationViewerThread() {
  threadClose();
}

void ConfigurationViewerThread::open() {
  viewer = make_shared<ConfigurationViewer>();
  {
    auto C = config.get();
    for(Frame* f:C->frames) f->ensure_X();
    viewer->updateConfiguration(C).view(false);
  }
}

void ConfigurationViewerThread::close() {
  viewer.reset();
}

void ConfigurationViewerThread::step() {
  auto C = config.get();
  viewer->updateConfiguration(C);
}

} //namespace
