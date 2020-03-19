/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "simulation.h"
#include "frame.h"
#include "proxy.h"
#include "kin_bullet.h"
#include "kin_physx.h"
#include "F_geometrics.h"
#include "switch.h"
#include "../Gui/opengl.h"

namespace rai {

struct Simulation_self {
  arr qdot;
  std::shared_ptr<struct Simulation_DisplayThread> display;
  std::shared_ptr<CameraView> cameraview;
  std::shared_ptr<BulletInterface> bullet;
  std::shared_ptr<PhysXInterface> physx;

  void updateDisplayData(double _time, const arr& _frameState, const ProxyA& _proxies);
  void updateDisplayData(const byteA& _image, const floatA& _depth);
};

struct SimulationState {
  arr frameState;
  arr frameVels;

  SimulationState(const arr& _frameState, const arr& _frameVels) : frameState(_frameState), frameVels(_frameVels) {}
};

Simulation::Simulation(Configuration& _C, Simulation::SimulatorEngine _engine, bool _display)
  : self(make_unique<Simulation_self>()),
    C(_C),
    engine(_engine),
    display(_display) {
  if(engine==_physx){
    self->physx = make_shared<PhysXInterface>(C, true);
  } else if(engine==_bullet){
    self->bullet = make_shared<BulletInterface>(C, true);
  } else if(engine==_kinematic){
    //nothing
  } else NIY;
  if(display) self->display = make_shared<Simulation_DisplayThread>(C);
}

Simulation::~Simulation(){
}

void Simulation::step(const arr& u_control, double tau, ControlMode u_mode) {
  time += tau;
  //perform control using C
  if(u_mode==_none){
  } else if(u_mode==_position){
    C.setJointState(u_control);
  } else if(u_mode==_velocity){
    arr q = C.getJointState();
    q += tau * u_control;
    C.setJointState(q);
  } else NIY;
  //call the physics ending
  if(engine==_physx) {
    self->physx->pushKinematicStates(C.frames);
    self->physx->step(tau);
    self->physx->pullDynamicStates(C.frames);
  }else if(engine==_bullet){
    self->bullet->pushKinematicStates(C.frames);
    self->bullet->step(tau);
    self->bullet->pullDynamicStates(C.frames);
  } else if(engine==_kinematic){
  } else NIY;
  if(display) self->updateDisplayData(time, C.getFrameState(), C.proxies);
}

void Simulation::openGripper(const char* gripperFrameName, double width, double speed){
  rai::Frame *g = C.getFrameByName(gripperFrameName);
  if(!g) LOG(-1) <<"you passed me a non-existing gripper name!";

  rai::Frame *obj = g->children(-1);
  CHECK_EQ(obj->joint->type, rai::JT_rigid, "");

  C.attach(C.frames(0), obj);
  if(engine==_physx) {
    self->physx->changeObjectType(obj, rai::BT_dynamic);
  }else{
    NIY;
  }
}

void Simulation::closeGripper(const char* gripperFrameName, double width, double speed, double force){
  rai::Frame *g = C.getFrameByName(gripperFrameName);
  if(!g) LOG(-1) <<"you passed me a non-existing gripper name!";

  //requirement: two of the children of need to be the finger geometries
  rai::Frame *fing1 = g->children(0);
  rai::Frame *fing2 = g->children(1);
  rai::Frame *obj = 0;

  C.stepSwift();
  rai::Proxy *p1=0, *p2=0;
  for(rai::Proxy& p:C.proxies){
    if(p.a == fing1){ p1 = &p; obj = p.b; }
    if(p.b == fing1){ p1 = &p; obj = p.a; }
    if(p.a == fing2){ p2 = &p; obj = p.b; }
    if(p.b == fing2){ p2 = &p; obj = p.a; }
  }
  if(!p1 || !p2){
    LOG(-1) <<"fingers are not close enough";
  }

  F_GraspOppose oppose(fing1->ID, fing2->ID, obj->ID);

  arr y;
  oppose.__phi(y, NoArr, C);

  if(sumOfSqr(y) < 0.1){ //good enough...
//    rai::KinematicSwitch sw(rai::SW_joint, rai::JT_rigid, gripperFrameName, obj->name, C, SWInit_copy);
//    sw.apply(C);
    obj = obj->getUpwardLink();
    C.attach(gripperFrameName, obj->name);
    if(engine==_physx) {
      self->physx->changeObjectType(obj, rai::BT_kinematic);
    }else{
      NIY;
    }
  }

}

ptr<SimulationState> Simulation::getState() {
  arr qdot;
  if(engine==_physx) {
    self->physx->pullDynamicStates(C.frames, qdot);
  }else if(engine==_bullet){
    self->bullet->pullDynamicStates(C.frames, qdot);
  }else NIY;
  return make_shared<SimulationState>(C.getFrameState(), qdot);
}

void Simulation::setState(const arr& frameState, const arr& frameVelocities){
  C.setFrameState(frameState);
  if(engine==_physx) {
    self->physx->pushFullState(C.frames, frameVelocities);
  }else if(engine==_bullet){
    self->bullet->pushFullState(C.frames, frameVelocities);
  }else NIY;
}

void Simulation::resetToPreviousState(const ptr<SimulationState>& state) {
  setState(state->frameState, state->frameVels);
}

void Simulation::pushConfigurationToSimulator(const arr& frameVelocities) {
  if(engine==_physx) {
    self->physx->pushFullState(C.frames, frameVelocities);
  }else if(engine==_bullet){
    self->bullet->pushFullState(C.frames, frameVelocities);
  }else NIY;
}

const arr& Simulation::qdot() {
  return self->qdot;
}

CameraView& Simulation::cameraview() {
  if(!self->cameraview) {
    self->cameraview = make_shared<CameraView>(C, true, false);
  }
  return *self->cameraview;
}

void Simulation::getImageAndDepth(byteA& image, floatA& depth) {
  cameraview().updateConfiguration(C);
  cameraview().computeImageAndDepth(image, depth);
  if(display) self->updateDisplayData(image, depth);
}

//===========================================================================

struct Simulation_DisplayThread : Thread, GLDrawer {
  Configuration Ccopy;
  OpenGL gl;
  //data
  Mutex mux;
  double time;
  byteA image;
  floatA depth;
  byteA segmentation;

  Simulation_DisplayThread(const Configuration& C)
    : Thread("Sim_DisplayThread", .05),
      Ccopy(C),
      gl("Simulation Display") {
    gl.add(*this);
    gl.camera.setDefault();
    threadLoop();
  }

  ~Simulation_DisplayThread() {
    threadClose();
  }

  void step() {
    mux.lock(RAI_HERE);
    gl.dataLock.lock(RAI_HERE);
    double t = time;
    gl.dataLock.unlock();
    mux.unlock();

    gl.update(STRING("t:" <<t), true);
  }

  void glDraw(OpenGL& gl) {
#ifdef RAI_GL
    glStandardScene(nullptr, gl);
    Ccopy.glDraw(gl);

    if(image.N && depth.N) {
      static byteA dep;
      resizeAs(dep, depth);
      float x;
      for(uint i=0; i<dep.N; i++) {
        x = 10.f * depth.elem(i);
        dep.elem(i) = (x<0.)?0:((x>255.)?255:x);
      }
      float scale = .3*float(gl.width)/image.d1;

      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      glOrtho(0, 1., 1., 0., -1., 1.); //only affects the offset - the rest is done with raster zooms
      glDisable(GL_DEPTH_TEST);
      glRasterImage(.3, .05, image, scale);
      glRasterImage(.65, .05, dep, scale);
    }
#else
    NICO
#endif
  }
};

void Simulation_self::updateDisplayData(double _time, const arr& _frameState, const ProxyA& _proxies) {
  CHECK(display, "");
  display->mux.lock(RAI_HERE);
  display->time = _time;
  display->Ccopy.setFrameState(_frameState);
  display->Ccopy.copyProxies(_proxies);
  display->mux.unlock();
}

void Simulation_self::updateDisplayData(const byteA& _image, const floatA& _depth) {
  CHECK(display, "");
  display->mux.lock(RAI_HERE);
  display->image = _image;
  display->depth= _depth;
  display->mux.unlock();
}

} //namespace rai
