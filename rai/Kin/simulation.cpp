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
#include "F_PairCollision.h"
#include "../Gui/opengl.h"

namespace rai {

//===========================================================================

struct Simulation_self {
  arr qdot;
  arr frameVelocities;
  std::shared_ptr<struct Simulation_DisplayThread> display;
  std::shared_ptr<CameraView> cameraview;
  std::shared_ptr<BulletInterface> bullet;
  std::shared_ptr<PhysXInterface> physx;

  void updateDisplayData(double _time, const arr& _frameState, const ProxyA& _proxies);
  void updateDisplayData(const byteA& _image, const floatA& _depth);
};

//===========================================================================

struct SimulationState {
  arr frameState;
  arr frameVels;

  SimulationState(const arr& _frameState, const arr& _frameVels) : frameState(_frameState), frameVels(_frameVels) {}
};

//===========================================================================

struct SimulationImp {
  enum When { _undefined, _beforeControl, _beforePhysics, _afterPhysics, _afterImages };

  Simulation::ImpType type;
  When when = _undefined;
  bool killMe = false;

  //-- imps overload these methods to modify/perturb something
  virtual void modControl(Simulation& S, arr& u_control, double& tau, Simulation::ControlMode u_mode) {}
  virtual void modConfiguration(Simulation& S) {}
  virtual void modImages(Simulation& S, byteA& image, floatA& depth) {}
};

//===========================================================================

struct Imp_CloseGripper : SimulationImp {
  Frame *gripper, *fing1, *fing2, *obj;
  F_PairCollision coll1;
  F_PairCollision coll2;
  arr q;

  Imp_CloseGripper(Frame* _gripper, Frame* _fing1, Frame* _fing2, Frame* _obj);
  virtual void modConfiguration(Simulation& S);
};

//===========================================================================

struct Imp_OpenGripper : SimulationImp {
  Frame *gripper, *fing1, *fing2;
  arr q;

  Imp_OpenGripper(Frame* _gripper, Frame* _fing1, Frame* _fing2);
  virtual void modConfiguration(Simulation& S);
};

//===========================================================================

struct Imp_ObjectImpulses : SimulationImp {
  Frame *obj;
  uint count=0;

  Imp_ObjectImpulses(Frame* _obj) : obj(_obj) { CHECK(obj, "");  when = _beforePhysics;  }
  virtual void modConfiguration(Simulation& S);
};

//===========================================================================

struct Imp_BlockJoints : SimulationImp {
  FrameL joints;
  arr qBlocked;

  Imp_BlockJoints(const FrameL& _joints, Simulation& S);
  virtual void modConfiguration(Simulation& S);
};

//===========================================================================

Simulation::Simulation(Configuration& _C, Simulation::SimulatorEngine _engine, int _verbose)
  : self(make_unique<Simulation_self>()),
    C(_C),
    time(0.),
    engine(_engine),
    verbose(_verbose) {
  if(engine==_physx){
    self->physx = make_shared<PhysXInterface>(C, true);
  } else if(engine==_bullet){
    self->bullet = make_shared<BulletInterface>(C, true);
  } else if(engine==_kinematic){
    //nothing
  } else NIY;
  if(verbose>0) self->display = make_shared<Simulation_DisplayThread>(C);
}

Simulation::~Simulation(){
}

void Simulation::step(const arr& u_control, double tau, ControlMode u_mode) {
  //-- kill done imps
  for(uint i=imps.N;i--;){
    if(imps.elem(i)->killMe) imps.remove(i);
  }

  arr ucontrol = u_control; //a copy to allow for perturbations

  //-- imps before control
  for(ptr<SimulationImp>& imp : imps) if(imp->when==SimulationImp::_beforeControl){
    imp->modControl(*this, ucontrol, tau, u_mode);
  }

  //-- perform control using C
  time += tau;
  if(u_mode==_none){
  } else if(u_mode==_position){
    C.setJointState(ucontrol);
  } else if(u_mode==_velocity){
    arr q = C.getJointState();
    q += tau * ucontrol;
    C.setJointState(q);
  } else NIY;

  //-- imps before physics
  for(ptr<SimulationImp>& imp : imps) if(imp->when==SimulationImp::_beforePhysics){
    imp->modConfiguration(*this);
  }

  //-- call the physics ending
  if(engine==_physx) {
    self->physx->pushKinematicStates(C.frames);
    self->physx->step(tau);
    self->physx->pullDynamicStates(C.frames, self->frameVelocities);
  }else if(engine==_bullet){
    self->bullet->pushKinematicStates(C.frames);
    self->bullet->step(tau);
    self->bullet->pullDynamicStates(C.frames, self->frameVelocities);
  } else if(engine==_kinematic){
  } else NIY;

  //-- imps after physics
  for(ptr<SimulationImp>& imp : imps) if(imp->when==SimulationImp::_afterPhysics){
    imp->modConfiguration(*this);
  }

  if(verbose>0) self->updateDisplayData(time, C.getFrameState(), C.proxies);
}

void Simulation::openGripper(const char* gripperFrameName, double width, double speed){
  rai::Frame *gripper = C.getFrameByName(gripperFrameName);
  if(!gripper){
    LOG(-1) <<"you passed me a non-existing gripper name!";
    return;
  }

  //remove gripper from grasps list
  for(uint i=grasps.N;i--;){
    if(grasps.elem(i)==gripper) grasps.remove(i);
  }

  //check if an object is attached
  rai::Frame *obj = gripper->children(-1);
  if(!obj || !obj->joint || obj->joint->type != rai::JT_rigid){
    LOG(-1) <<"gripper '" <<gripper->name <<"' does not hold an object";
    obj=0;
  }

  //reattach object to world frame, and make it physical
  if(obj){
    C.attach(C.frames(0), obj);
    if(engine==_physx) {
      self->physx->changeObjectType(obj, rai::BT_dynamic);
    }else{
      NIY;
    }
  }

  //requirement: two of the children of need to be the finger geometries
  rai::Frame *fing1 = gripper->children(0); while(!fing1->shape && fing1->children.N) fing1 = fing1->children(0);
  rai::Frame *fing2 = gripper->children(1); while(!fing2->shape && fing2->children.N) fing2 = fing2->children(0);

  imps.append(make_shared<Imp_OpenGripper>(gripper, fing1, fing2));
}

void Simulation::closeGripper(const char* gripperFrameName, double width, double speed, double force){
  rai::Frame *gripper = C.getFrameByName(gripperFrameName);
  if(!gripper) LOG(-1) <<"you passed me a non-existing gripper name!";

  //-- first, find the object that is between the fingers

  //requirement: two of the children of need to be the finger geometries
  rai::Frame *fing1 = gripper->children(0); while(!fing1->shape && fing1->children.N) fing1 = fing1->children(0);
  rai::Frame *fing2 = gripper->children(1); while(!fing2->shape && fing2->children.N) fing2 = fing2->children(0);

  //collect objects close to fing1 and fing2
  C.stepSwift();
  FrameL fing1close;
  FrameL fing2close;
  for(rai::Proxy& p:C.proxies){
    if(p.a == fing1) fing1close.setAppend( p.b );
    if(p.b == fing1) fing1close.setAppend( p.a );
    if(p.a == fing2) fing2close.setAppend( p.b );
    if(p.b == fing2) fing2close.setAppend( p.a );
  }

  //intersect
  FrameL objs = setSection(fing1close, fing2close);
//  cout <<"initiating ";
//  listWrite(objs);
//  cout <<endl;
  if(!objs.N){
    LOG(-1) <<"fingers are not close to objects";
    return;
  }

  rai::Frame *obj = objs.elem(0);

  if(objs.N>1){
    arr center = .5*(fing1->getPosition()+fing2->getPosition());
    double d = length(center - obj->getPosition());
    for(uint i=1;i<objs.N;i++){
      double d2 = length(center - objs(i)->getPosition());
      if(d2<d){ obj = objs(i); d = d2; }
    }
  }

  LOG(1) <<"initiating grasp of object " <<obj->name <<" (if this is not what you expect, did you setContact(1) for the object you want to grasp?)";


#if 1
  imps.append(make_shared<Imp_CloseGripper>(gripper, fing1, fing2, obj));
#else

  //-- actually close gripper until both distances are < .001
  F_PairCollision coll1(fing1->ID, obj->ID, coll1._negScalar, false);
  auto d1 = coll1.eval(C);

  F_PairCollision coll2(fing2->ID, obj->ID, coll1._negScalar, false);
  auto d2 = coll2.eval(C);

  cout <<"d1: " <<d1.y <<"d2: " <<d2.y <<endl;

  arr q = fing1->joint->calc_q_from_Q(fing1->get_Q());

  for(;;){
    q(0) -= .0001;
    fing1->joint->calc_Q_from_q(q, 0);
    fing2->joint->calc_Q_from_q(q, 0);
    step({}, .01, _none);
    auto d1 = coll1.eval(C);
    auto d2 = coll2.eval(C);
    cout <<q <<" d1: " <<d1.y <<"d2: " <<d2.y <<endl;
    if(-d1.y(0)<1e-3 && -d2.y(0)<1e-3) break; //close enough!
    if(q(0)<-.1) return;
//    rai::wait(.01);
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
#endif

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

void Simulation::restoreState(const ptr<SimulationState>& state) {
  setState(state->frameState, state->frameVels);
}

const arr& Simulation::get_qDot() {
  return self->qdot;
}

double Simulation::getGripperWidth(const char* gripperFrameName){
  rai::Frame *gripper = C.getFrameByName(gripperFrameName);
  if(!gripper){
    LOG(-1) <<"you passed me a non-existing gripper name!";
    return -1.;
  }

  //requirement: two of the children of need to be the finger geometries
  rai::Frame *fing1 = gripper->children(0); while(!fing1->shape && fing1->children.N) fing1 = fing1->children(0);
  rai::Frame *fing2 = gripper->children(1); while(!fing2->shape && fing2->children.N) fing2 = fing2->children(0);

  return fing1->joint->calc_q_from_Q(fing1->get_Q()).scalar();
}

bool Simulation::getGripperIsGrasping(const char* gripperFrameName){
  for(Frame *g:grasps) if(g->name==gripperFrameName) return true;
  return false;
}

CameraView& Simulation::cameraview() {
  if(!self->cameraview) {
    self->cameraview = make_shared<CameraView>(C, true, false);
  }
  return *self->cameraview;
}

void Simulation::addImp(Simulation::ImpType type, const StringA& frames, const arr& parameters){
  if(type==_objectImpulses){
    CHECK_EQ(frames.N, 1, "");
    rai::Frame *obj = C.getFrameByName(frames(0));
    imps.append(make_shared<Imp_ObjectImpulses>(obj));
  } else if(type==_blockJoints){
    FrameL F = C.getFramesByNames(frames);
    auto block = make_shared<Imp_BlockJoints>(F, *this);
    imps.append(block);
  } else {
    NIY;
  }
}

void Simulation::getImageAndDepth(byteA& image, floatA& depth) {
  cameraview().updateConfiguration(C);
  cameraview().renderMode = CameraView::visuals;
  cameraview().computeImageAndDepth(image, depth);

  //-- imps after images
  for(ptr<SimulationImp>& imp : imps) if(imp->when==SimulationImp::_afterImages){
    imp->modImages(*this, image, depth);
  }

  if(verbose>0) self->updateDisplayData(image, depth);
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
    gl.clear();
    threadClose(.5);
  }

  void step() {
    mux.lock(RAI_HERE);
    double t = time;
    mux.unlock();

    gl.update(STRING("t:" <<t), true);
  }

  void glDraw(OpenGL& gl) {
#ifdef RAI_GL
    mux.lock(RAI_HERE);
    glStandardScene(nullptr, gl);
    Ccopy.glDraw(gl);

    if(image.N && depth.N) {
      static byteA dep;
      resizeAs(dep, depth);
      float x;
      for(uint i=0; i<dep.N; i++) {
        x = 100.f * depth.elem(i); //this means that the RGB values are cm distance (up to 255cm distance)
        dep.elem(i) = (x<0.)?0:((x>255.)?255:x);
      }
      float scale = .3*float(gl.width)/image.d1;
      float top = 1. - scale*float(image.d0)/gl.height;

      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      glOrtho(0, 1., 1., 0., -1., 1.); //only affects the offset - the rest is done with raster zooms
      glDisable(GL_DEPTH_TEST);
      glRasterImage(.0, top, image, scale);
      glRasterImage(.7, top, dep, scale);
    }
    mux.unlock();
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

//===========================================================================

Imp_CloseGripper::Imp_CloseGripper(Frame* _gripper, Frame* _fing1, Frame* _fing2, Frame* _obj)
  : gripper(_gripper), fing1(_fing1), fing2(_fing2), obj(_obj),
    coll1(fing1->ID, obj->ID, coll1._negScalar, false),
    coll2(fing2->ID, obj->ID, coll1._negScalar, false) {
  when = _beforePhysics;
  type = Simulation::_closeGripper;


  //    auto d1 = coll1.eval(C);

  //    auto d2 = coll2.eval(C);

  //    cout <<"d1: " <<d1.y <<"d2: " <<d2.y <<endl;

  q = fing1->joint->calc_q_from_Q(fing1->get_Q());

}

void Imp_CloseGripper::modConfiguration(Simulation& S) {
  if(killMe) return;

  CHECK_EQ(&S.C, &fing1->C, "");
  CHECK_EQ(&S.C, &fing2->C, "");
  CHECK_EQ(&S.C, &obj->C, "");

  //-- actually close gripper until both distances are < .001
  q.scalar() -= .0001;
  fing1->joint->calc_Q_from_q(q, 0);
  fing2->joint->calc_Q_from_q(q, 0);
  //      step({}, .01, _none);
  auto d1 = coll1.eval(S.C);
  auto d2 = coll2.eval(S.C);
//  cout <<q <<" d1: " <<d1.y <<"d2: " <<d2.y <<endl;
  if(q(0)<-.1){ //stop grasp by joint limits -> unsuccessful
    killMe = true;
  }
  if(-d1.y(0)<1e-3 && -d2.y(0)<1e-3){ //stop grasp by contact
    //evaluate stability
    F_GraspOppose oppose(fing1->ID, fing2->ID, obj->ID);
    arr y;
    oppose.__phi(y, NoArr, S.C);

    if(sumOfSqr(y) < 0.1){ //good enough -> success!
      // kinematically attach object to gripper
      obj = obj->getUpwardLink();
      S.C.attach(gripper, obj);

      // tell engine that object is now kinematic, not dynamic
      if(S.engine==S._physx) {
        S.self->physx->changeObjectType(obj, BT_kinematic);
      }else{
        NIY;
      }

      //allows the user to know that gripper grasps something
      S.grasps.append(gripper);
    }else{ //unsuccessful
    }
    killMe = true;
  }
}

//===========================================================================

Imp_OpenGripper::Imp_OpenGripper(Frame* _gripper, Frame* _fing1, Frame* _fing2)
  : gripper(_gripper), fing1(_fing1), fing2(_fing2) {
  when = _beforePhysics;
  type = Simulation::_openGripper;

  q = fing1->joint->calc_q_from_Q(fing1->get_Q());
}

void Imp_OpenGripper::modConfiguration(Simulation& S) {
  if(killMe) return;

  CHECK_EQ(&S.C, &gripper->C, "");
  CHECK_EQ(&S.C, &fing1->C, "");
  CHECK_EQ(&S.C, &fing2->C, "");

  //-- actually open gripper until limit
  q.scalar() += .0001;
  fing1->joint->calc_Q_from_q(q, 0);
  fing2->joint->calc_Q_from_q(q, 0);
  if(q.scalar() > fing1->joint->limits(1)){ //stop opening
    killMe = true;
  }
}

//===========================================================================

void Imp_ObjectImpulses::modConfiguration(Simulation& S){
  count ++;
  if(count<100) return;

  count=0;

  arr vel = randn(3);
  if(vel(2)<0.) vel(2)=0.;
  vel(0) *= .1;
  vel(1) *= .1;

  std::shared_ptr<SimulationState> state = S.getState();

  state->frameVels(obj->ID, 0, {}) = vel;

  S.restoreState(state);
}

//===========================================================================

Imp_BlockJoints::Imp_BlockJoints(const FrameL& _joints, Simulation& S)
  : joints(_joints) {
  when = _beforePhysics;
  qBlocked.resize(joints.N);
  arr q = S.C.getJointState();
  for(uint i=0;i<joints.N;i++){
    rai::Joint *j = joints(i)->joint;
    CHECK(j, "");
    qBlocked(i) = q(j->qIndex);
  }
}

void Imp_BlockJoints::modConfiguration(Simulation& S){
  CHECK_EQ(joints.N, qBlocked.N, "");
  arr q = S.C.getJointState();
  for(uint i=0;i<joints.N;i++){
    rai::Joint *j = joints(i)->joint;
    CHECK(j, "");
    q(j->qIndex) = qBlocked(i);
  }
  S.C.setJointState(q);
}

} //namespace rai
