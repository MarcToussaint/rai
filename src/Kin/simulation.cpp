/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "simulation.h"
#include "frame.h"
#include "proxy.h"
#include "kin_bullet.h"
#include "kin_physx.h"
#include "F_geometrics.h"
#include "F_collisions.h"
#include "../Gui/opengl.h"
#include "../Algo/SplineCtrlFeed.h"

#include <iomanip>
//#define BACK_BRIDGE

template<> const char* rai::Enum<rai::Simulation::Engine>::names []= {
  "none", "physx", "bullet", "kinematic", nullptr
};

namespace rai {

//===========================================================================

struct Simulation_self {
  arr qDot;
  arr frameVelocities;
  std::shared_ptr<struct Simulation_DisplayThread> display;
  std::shared_ptr<CameraView> cameraview;
  std::shared_ptr<BulletInterface> bullet;
  std::shared_ptr<PhysXInterface> physx;
#ifdef BACK_BRIDGE
  std::shared_ptr<BulletBridge> bulletBridge;
  rai::Configuration bridgeC;
#endif
  std::shared_ptr<OpenGL> glDebug;

  BSplineCtrlReference ref;

  void updateDisplayData(double _time, const Configuration& _C);
  void updateDisplayData(const byteA& _image, const floatA& _depth);
};

//===========================================================================

struct SimulationImp {
  enum When { _undefined, _beforeControl, _beforePhysics, _afterPhysics, _afterImages };

  Simulation::ImpType type;
  When when = _undefined;
  bool killMe = false;

  //-- imps overload these methods to modify/perturb something
  virtual void modControl(Simulation& S, arr& u_control, double& tau, Simulation::ControlMode u_mode) {}
  virtual void modConfiguration(Simulation& S, double tau) {}
  virtual void modImages(Simulation& S, byteA& image, floatA& depth) {}
};

//===========================================================================

struct Imp_CloseGripper : SimulationImp {
  Frame* gripper, *fing1, *fing2, *obj, *finger1, *finger2;
  Joint* joint;
  Vector axis;
  arr limits;
  std::unique_ptr<F_PairCollision> coll1;
  std::unique_ptr<F_PairCollision> coll2;
  double q;
  double speed;

  Imp_CloseGripper(Frame* _gripper, Joint* _joint, Frame* _fing1, Frame* _fing2, Frame* _obj, double _speed);
  virtual void modConfiguration(Simulation& S, double tau);
};

//===========================================================================

struct Imp_GripperMove : SimulationImp {
  Frame* gripper, *fing1, *fing2;
  Joint* joint;
  Vector axis;
  double q;
  double speed;
  double stop;

  Imp_GripperMove(Frame* _gripper, Joint* _joint, Frame* _fing1, Frame* _fing2, double _speed, double _stop=-1.);
  virtual void modConfiguration(Simulation& S, double tau);
};

//===========================================================================

struct Imp_ObjectImpulses : SimulationImp {
  Frame* obj;
  double timeToImpulse=0.;

  Imp_ObjectImpulses(Frame* _obj) : obj(_obj) { CHECK(obj, "");  when = _beforePhysics;  }
  virtual void modConfiguration(Simulation& S, double tau);
};

//===========================================================================

struct Imp_BlockJoints : SimulationImp {
  FrameL joints;
  arr qBlocked;

  Imp_BlockJoints(const FrameL& _joints, Simulation& S);
  virtual void modConfiguration(Simulation& S, double tau);
};
//===========================================================================

struct Imp_NoPenetrations : SimulationImp {
  Imp_NoPenetrations() {when = _beforePhysics;};
  virtual void modConfiguration(Simulation& S, double tau);
};

//===========================================================================

Simulation::Simulation(Configuration& _C, Engine _engine, int _verbose)
  : self(make_unique<Simulation_self>()),
    C(_C),
    time(0.),
    engine(_engine),
    verbose(_verbose) {
  C.ensure_q();
  if(engine==_physx) {
    self->physx = make_shared<PhysXInterface>(C, verbose-1);
  } else if(engine==_bullet) {
    self->bullet = make_shared<BulletInterface>(C, rai::Bullet_Options().set_verbose(verbose-1));
#ifdef BACK_BRIDGE
    self->bulletBridge = make_shared<BulletBridge>(self->bullet->getDynamicsWorld());
    self->bulletBridge->getConfiguration(self->bridgeC);
    self->bulletBridge->pullPoses(self->bridgeC, true);
#endif
  } else if(engine==_kinematic) {
    //nothing
  } else NIY;
  self->ref.initialize(C.getJointState(), NoArr, time);
  if(verbose>0) self->display = make_shared<Simulation_DisplayThread>(C, STRING(" ["<<rai::Enum<Engine>(engine)<<"]"));
}

Simulation::~Simulation() {
  if(verbose>0) {
    LOG(0) <<"shutting down Simulation";
  }
}

void Simulation::step(const arr& u_control, double tau, ControlMode u_mode) {
  //-- kill done imps
  for(uint i=imps.N; i--;) {
    if(imps.elem(i)->killMe) imps.remove(i);
  }

  arr ucontrol = u_control; //a copy to allow for perturbations
  if(!ucontrol.N && u_mode==_position && teleopCallbacks) {
    ucontrol = teleopCallbacks->q_ref;
  }

  //-- imps before control
  for(shared_ptr<SimulationImp>& imp : imps) if(imp->when==SimulationImp::_beforeControl) {
      imp->modControl(*this, ucontrol, tau, u_mode);
    }

  //-- define a reference depending on control mode (size zero means no reference)
  arr q_ref, qDot_ref;
  time += tau;
  if(u_mode==_none) {
  } else if(u_mode==_position) {
    q_ref = ucontrol;
  } else if(u_mode==_velocity) {
    arr q = C.getJointState();
    self->qDot = ucontrol;
    q += tau * self->qDot;
    q_ref = q;
    qDot_ref = ucontrol;
  } else if(u_mode==_posVel) {
    ucontrol.reshape(2, -1);
    q_ref = ucontrol[0];
    qDot_ref = ucontrol[1];
  } else if(u_mode==_acceleration) {
    arr q = C.getJointState();
    if(!self->qDot.N) self->qDot = zeros(q.N);
    q += .5 * tau * self->qDot;
    self->qDot += tau * ucontrol;
    q += .5 * tau * self->qDot;
    q_ref = q;
    qDot_ref = self->qDot;
  } else if(u_mode==_spline) {
    arr q = C.getJointState();
    if(!self->qDot.N) self->qDot = zeros(q.N);
    self->ref.getReference(q_ref, qDot_ref, NoArr, q, self->qDot, time);
  } else NIY;

  //-- imps before physics
  for(shared_ptr<SimulationImp>& imp : imps) if(imp->when==SimulationImp::_beforePhysics) {
      imp->modConfiguration(*this, tau);
    }

  //-- call the physics engine
  if(engine==_physx) {
    if(self->physx->opt().jointedBodies || self->physx->opt().multiBody) {
      self->physx->pushFrameStates(C, NoArr, true); //kinematicOnly (usually none anyway)
      if(q_ref.N) {
        C.setJointState(q_ref);
        self->physx->pushMotorStates(C); //qDot_ref, motor control
      }
    } else {
      if(q_ref.N) C.setJointState(q_ref); //kinematic control
      if(qDot_ref.N) self->qDot = qDot_ref;
      self->physx->pushFrameStates(C, NoArr, true); //kinematicOnly
    }
    self->physx->step(tau);
    self->physx->pullDynamicStates(C, self->frameVelocities);
    self->physx->pullMotorStates(C, self->qDot);  //why not also pull the motor states?
  } else if(engine==_bullet) {
    self->bullet->pushKinematicStates(C);
    if(self->bullet->opt().multiBody) {
      if(ucontrol.nd!=2) LOG(1) <<"stepping motorized bullet without ctrl reference";
      else self->bullet->setMotorQ(ucontrol[0], ucontrol[1]); //C.getJointState(), self->qDot);
    }
    self->bullet->step(tau);
    self->bullet->pullDynamicStates(C); //, self->frameVelocities);
#ifdef BACK_BRIDGE
    self->bulletBridge->pullPoses(self->bridgeC, true);
    self->bridgeC.view(false, "bullet bridge");
#endif
  } else if(engine==_kinematic) {
    if(q_ref.N) {
      C.setJointState(q_ref);
    }
  } else NIY;

  //-- imps after physics
  for(shared_ptr<SimulationImp>& imp : imps) if(imp->when==SimulationImp::_afterPhysics) {
      imp->modConfiguration(*this, tau);
    }

  C.ensure_q();

  if(verbose>0) self->updateDisplayData(time, C); //does not update with freq >20hz - see method

  if(engine==_physx && verbose>3) {
    if(!self->glDebug) {
      self->glDebug = make_shared<OpenGL>("physx sim DEBUG", 500, 300);
      self->glDebug->camera.setDefault();
      NIY; //self->glDebug->add(*self->physx);
    }
    self->glDebug->update();
  }
}

void Simulation::setSplineRef(const arr& _x, const arr& _times, bool append) {
  arr path = _x;
  if(_x.nd==1) path.reshape(1, _x.N);

  arr times = _times;
  if(times.N==1 && path.d0>1) {
    double t = times.elem();
    times.setGrid(1, t/(path.d0), t, path.d0-1);
  }
  CHECK_EQ(path.d0, times.N, "need times for each control point");

  if(append) self->ref.append(path, times, time);
  else self->ref.overwriteSmooth(path, times, time);
}

void Simulation::resetSplineRef() {
  self->ref.initialize(C.getJointState(), NoArr, time);
}

bool getFingersForGripper(rai::Frame*& gripper, rai::Joint*& joint, rai::Frame*& fing1, rai::Frame*& fing2, rai::Configuration& C, const char* gripperFrameName) {
  gripper = C.getFrame(gripperFrameName);
  joint=0;
  if(!gripper) {
    LOG(-1) <<"you passed me a non-existing gripper name!";
    gripper=fing1=fing2=0;
    joint=0;
    return false;
  }
  gripper = gripper->getUpwardLink();
  //browse all children of the gripper and find by name
  FrameL F;
  gripper->getSubtree(F);
  for(rai::Frame* f:F) {
    if(f->name.endsWith("finger1")) fing1=f;
    if(f->name.endsWith("finger2")) fing2=f;
    if(f->joint && f->joint->dim==1 && !f->joint->active && !f->joint->mimic) joint = f->joint;
  }
  fing1 = fing1->parent;
  fing2 = fing2->parent;
//  fing1 = fing1->getUpwardLink();
//  fing2 = fing2->getUpwardLink();
//  CHECK(fing1->joint, "");
//  CHECK(fing2->joint, "");
//  CHECK(!fing1->joint->active || !fing1->joint->dim, ""); //grippers need to be rigid joints! (to not be part of the dynamic/control system)
//  CHECK(!fing2->joint->active || !fing2->joint->dim, "");

  //requirement: two of the children of need to be the finger geometries
//  fing1 = gripper->children(0); while(!fing1->shape && fing1->children.N) fing1 = fing1->children(0);
//  fing2 = gripper->children(1); while(!fing2->shape && fing2->children.N) fing2 = fing2->children(0);
  return true;
}

void Simulation::moveGripper(const char* gripperFrameName, double width, double speed) {
  rai::Frame* gripper, *fing1, *fing2;
  rai::Joint* joint;
  getFingersForGripper(gripper, joint, fing1, fing2, C, gripperFrameName);
  if(!gripper) return;

  //remove gripper from grasps list
  for(uint i=grasps.N; i--;) {
    if(grasps.elem(i)==gripper) grasps.remove(i);
  }

  //check if an object is attached
  rai::Frame* obj = gripper->children(-1);
  if(!obj || !obj->joint || obj->joint->type != rai::JT_rigid) obj=0;

  //reattach object to world frame, and make it physical
  if(obj) {
    if(verbose>1) LOG(1) <<"initiating opening gripper " <<gripper->name <<" and releasing obj " <<obj->name <<" width:" <<width <<" speed:" <<speed;
    detach(obj);
  } else {
    if(verbose>1) LOG(1) <<"initiating opening gripper " <<gripper->name <<" (without releasing obj)" <<" width:" <<width <<" speed:" <<speed;
  }

  C.ensure_q();
  imps.append(make_shared<Imp_GripperMove>(gripper, joint, fing1, fing2, speed, width));
}

void Simulation::closeGripper(const char* gripperFrameName, double width, double speed, double force) {
  rai::Frame* gripper, *fing1, *fing2;
  rai::Joint* joint;
  getFingersForGripper(gripper, joint, fing1, fing2, C, gripperFrameName);
  if(!gripper) return;

  rai::Frame* finger1 = fing1, *finger2=fing2;
  while(!finger1->shape || finger1->shape->type()!=ST_capsule) finger1=finger1->children.last();
  while(!finger2->shape || finger2->shape->type()!=ST_capsule) finger2=finger2->children.last();

  CHECK(finger1->shape && finger1->shape->cont, "");
  CHECK(finger2->shape && finger2->shape->cont, "");

  //collect objects close to fing1 and fing2
  C.stepFcl();
  FrameL fing1close;
  FrameL fing2close;
  for(rai::Proxy& p:C.proxies) {
    if(!p.collision) p.calc_coll();
    if(p.d<.2) {
      if(p.a == finger1) fing1close.setAppend(p.b);
      if(p.b == finger1) fing1close.setAppend(p.a);
      if(p.a == finger2) fing2close.setAppend(p.b);
      if(p.b == finger2) fing2close.setAppend(p.a);
      //LOG(0) <<"near objects: " <<p;
    }
  }

  //intersect
  FrameL objs = rai::setSection(fing1close, fing2close);
//  cout <<"initiating ";
//  listWrite(objs);
//  cout <<endl;

  rai::Frame* obj = 0;
  if(objs.N) obj = objs.elem(0);

  //choose from multiple object candidates
  if(objs.N>1) {
    arr center = .5*(finger1->getPosition()+finger2->getPosition());
    double d = length(center - obj->getPosition());
    for(uint i=1; i<objs.N; i++) {
      double d2 = length(center - objs(i)->getPosition());
      if(d2<d) { obj = objs(i); d = d2; }
    }
  }

  if(verbose>1) {
    if(obj) {
      LOG(1) <<"initiating grasp of object " <<obj->name <<" (auto detected; if this is not what you expect, did you setContact(1) for the object you want to grasp?)";
    } else {
      LOG(1) <<"closing gripper without near object (if this is not what you expect, did you setContact(1) for the object you want to grasp?)";
    }
  }

  imps.append(make_shared<Imp_CloseGripper>(gripper, joint, fing1, fing2, obj, speed));
}

void Simulation::closeGripperGrasp(const char* gripperFrameName, const char* objectName, double width, double speed, double force) {
  rai::Frame* gripper, *fing1, *fing2;
  rai::Joint* joint;
  getFingersForGripper(gripper, joint, fing1, fing2, C, gripperFrameName);
  if(!gripper) return;

  rai::Frame* finger1 = fing1, *finger2=fing2;
  while(!finger1->shape || finger1->shape->type()!=ST_capsule) finger1=finger1->children.last();
  while(!finger2->shape || finger2->shape->type()!=ST_capsule) finger2=finger2->children.last();

  rai::Frame* obj = 0;
  if(objectName) obj = C.getFrame(objectName);
  if(verbose>1) LOG(1) <<"initiating grasp of object " <<(obj?obj->name:"--nil--") <<" (prefixed)";

  imps.append(make_shared<Imp_CloseGripper>(gripper, joint, fing1, fing2, obj, speed));
}

bool Simulation::gripperIsDone(const char* gripperFrameName) {
  rai::Frame* gripper = C.getFrame(gripperFrameName);
  if(!gripper) {
    LOG(-1) <<"you passed me a non-existing gripper name!";
    return true;
  }
  gripper = gripper->getUpwardLink();
  for(auto& imp: imps) {
    if(std::dynamic_pointer_cast<Imp_GripperMove>(imp) && std::dynamic_pointer_cast<Imp_GripperMove>(imp)->gripper==gripper) return false;
    if(std::dynamic_pointer_cast<Imp_CloseGripper>(imp) && std::dynamic_pointer_cast<Imp_CloseGripper>(imp)->gripper==gripper) return false;
  }
  return true;
}

void Simulation::getState(arr& frameState, arr& q, arr& frameVelocities, arr& qDot) {
  if(engine==_physx) {
    self->physx->pullDynamicStates(C, frameVelocities);
    self->physx->pullMotorStates(C, qDot);
  } else if(engine==_bullet) {
    self->bullet->pullDynamicStates(C, frameVelocities);
    if(!!q) NIY;
  } else NIY;
  frameState = C.getFrameState();
  q = C.getJointState();
}

void Simulation::setState(const arr& frameState, const arr& q, const arr& frameVelocities, const arr& qDot) {
  C.setFrameState(frameState);
  if(!!q && q.N) C.setJointState(q);
  pushConfigurationToSimulator(frameVelocities, qDot);
  if(engine==_physx) {
    self->physx->step(1e-3);
    pushConfigurationToSimulator(frameVelocities, qDot);
  }
}

void Simulation::pushConfigurationToSimulator(const arr& frameVelocities, const arr& qDot) {
  if(engine==_physx) {
    self->physx->pushFrameStates(C, frameVelocities);
    self->physx->pushMotorStates(C, true, qDot);
  } else if(engine==_bullet) {
    self->bullet->pushFullState(C, frameVelocities);
  } else NIY;
  if(verbose>0) self->updateDisplayData(time, C);
}

void Simulation::registerNewObjectWithEngine(Frame* f) {
  CHECK_EQ(&f->C, &C, "can't register frame that is not part of the simulated configuration");
  if(engine==_physx) {
    self->physx->postAddObject(f);
  } else if(engine==_bullet) {
    NIY;
  } else NIY;
}

const arr& Simulation::get_qDot() {
  return self->qDot;
}

double Simulation::getTimeToSplineEnd() {
  return self->ref.getEndTime()-time;
}

double Simulation::getGripperWidth(const char* gripperFrameName) {
  rai::Frame* gripper, *fing1, *fing2;
  rai::Joint* joint;
  getFingersForGripper(gripper, joint, fing1, fing2, C, gripperFrameName);
  if(!gripper) return -1.;
  if(joint) return joint->get_q();
  return fing1->get_Q().pos.sum();
}

bool Simulation::getGripperIsGrasping(const char* gripperFrameName) {
  rai::Frame* gripper, *fing1, *fing2;
  rai::Joint* joint;
  getFingersForGripper(gripper, joint, fing1, fing2, C, gripperFrameName);
  for(Frame* g:grasps) if(g==gripper) return true;
  return false;
}

bool Simulation::getGripperIsClose(const char* gripperFrameName) {
  rai::Frame* gripper, *fing1, *fing2;
  rai::Joint* joint;
  getFingersForGripper(gripper, joint, fing1, fing2, C, gripperFrameName);
  if(!gripper) return -1.;
  double speed=.1;
  if(joint->frame->parent->name.contains("robotiq")) speed=-1.;
  if(joint) {
    double q = joint->get_q();
    if((speed>0. && q<joint->limits(0)+.01)
        || (speed<0. && q>joint->limits(1)-.01)) return true;
  } else {
    NIY;
  }
  return false;
}

bool Simulation::getGripperIsOpen(const char* gripperFrameName) {
  rai::Frame* gripper, *fing1, *fing2;
  rai::Joint* joint;
  getFingersForGripper(gripper, joint, fing1, fing2, C, gripperFrameName);
  if(!gripper) return false;
  double speed=-1.;
  if(joint->frame->parent->name.contains("robotiq")) speed=1.;
  if(joint) {
    double q = joint->get_q();
    if((speed>0. && q<joint->limits(0)+.001)
        || (speed<0. && q>joint->limits(1)-.001)) return true;
  } else {
    NIY;
  }
  return false;
}

CameraView& Simulation::cameraview() {
  if(!self->cameraview) self->cameraview = make_shared<CameraView>(C, true);
  return *self->cameraview;
}

void Simulation::addImp(Simulation::ImpType type, const StringA& frames, const arr& parameters) {
  if(type==_objectImpulses) {
    CHECK_EQ(frames.N, 1, "");
    rai::Frame* obj = C.getFrame(frames(0));
    imps.append(make_shared<Imp_ObjectImpulses>(obj));
  } else if(type==_blockJoints) {
    FrameL F = C.getFrames(frames);
    auto block = make_shared<Imp_BlockJoints>(F, *this);
    imps.append(block);
  } else if(type==_noPenetrations) {
    imps.append(make_shared<Imp_NoPenetrations>());
  } else {
    NIY;
  }
}

void Simulation::getImageAndDepth(byteA& image, floatA& depth) {
  cameraview().updateConfiguration(C);
  cameraview().renderMode = CameraView::visuals;
  cameraview().computeImageAndDepth(image, depth);

  //-- imps after images
  for(shared_ptr<SimulationImp>& imp : imps) if(imp->when==SimulationImp::_afterImages) {
      imp->modImages(*this, image, depth);
    }

  if(verbose>0) self->updateDisplayData(image, depth);
}

//===========================================================================

void glRasterImage(float x, float y, byteA& img, float zoom) {
  glRasterPos3f(x, y, 0.); //(int)(y+zoom*img.d0)); (the latter was necessary for other pixel/raster coordinates)
  glPixelZoom(zoom, -zoom);
  if(img.d1%4) {  //necessary: extend the image to have width mod 4
    uint P=img.d2;
    if(!P) P=1;
    uint add=4-(img.d1%4);
    img.reshape(img.d0, img.d1*P);
    img.insColumns(-1, add*P);
    if(P>1) img.reshape(img.d0, img.d1/P, P);
  }

  switch(img.d2) {
    case 0:
    case 1:  glDrawPixels(img.d1, img.d0, GL_LUMINANCE, GL_UNSIGNED_BYTE, img.p);        break;
    case 2:  glDrawPixels(img.d1, img.d0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, img.p);  break;
    case 3:  glDrawPixels(img.d1, img.d0, GL_RGB, GL_UNSIGNED_BYTE, img.p);              break;
    case 4:  glDrawPixels(img.d1, img.d0, GL_RGBA, GL_UNSIGNED_BYTE, img.p);             break;
    default: HALT("no image format");
  };
}

struct Simulation_DisplayThread : Thread, ConfigurationViewer {
  //data
  Mutex mux;
  double time=0.;
  byteA image;
  floatA depth;
  byteA depthImage;
  byteA segmentation;
  byteA screenshot;
  uint pngCount=0;
  uint drawCount=0;
  str text;

  Simulation_DisplayThread(const rai::Configuration& C, const char* _text) : Thread("Sim_DisplayThread", .05), text(_text) {
    updateConfiguration(C);
    if(rai::getParameter<bool>("sim/displayVisualsOnly", true)) renderUntil=_shadow;
    ensure_gl().setTitle("Simulation");
    ensure_gl().update(0, true); //waits for first draw
    threadLoop();
  }

  ~Simulation_DisplayThread() {
    threadClose(.5);
  }

  void step() {
    str txt;
    txt <<"Kin/Simulation" <<text <<" - time:" <<time <<"\n[Simulation_DisplayThread; disable with Simulation.verbose=0]";
    view(false, txt);
    //write_png(gl->captureImage, STRING("z.vid/"<<std::setw(4)<<std::setfill('0')<<(pngCount++)<<".png"));
    //if(!(step_count%10)) cout <<"display thread load:" <<timer.report() <<endl;
  }

  void close() {
    close_gl();
  }

  void glDraw(OpenGL& gl) {
    drawCount++;
#ifdef RAI_GL
    mux.lock(RAI_HERE);

    if(image.N && depth.N) {
      resizeAs(depthImage, image);
      float x;
      for(uint i=0; i<depth.N; i++) {
        x = 100.f * depth.p[i]; //this means that the RGB values are cm distance (up to 255cm distance)
        if(x<0.f) x=0.f;
        if(x>255.f) x=255.f;
        for(uint j=0;j<3;j++)
          depthImage.p[3*i+j] = x;
      }

      if(!quads.N){
        float w = .3*float(gl.width);
        addQuad(image, 10, 10, w, -1);
        addQuad(depthImage, gl.width-w-10, 10, w, -1);
      }else{
        quads(0)->img = image;
        quads(1)->img = depthImage;
      }
    }

    ConfigurationViewer::glDraw(gl);

    screenshot.resize(gl.height, gl.width, 3);
    glReadPixels(0, 0, gl.width, gl.height, GL_RGB, GL_UNSIGNED_BYTE, screenshot.p);

    mux.unlock();
#else
    NICO
#endif
  }
};

void Simulation_self::updateDisplayData(double _time, const rai::Configuration& _C) {
  CHECK(display, "");
  if(!display->drawCount) return; //don't update when not even drawn once.. to save compute
  if(_time - display->time <.025) return; //don't update with > 50Hz
  display->mux.lock(RAI_HERE);
  display->time = _time;
  display->drawCount = 0;
  display->mux.unlock();

  display->updateConfiguration(_C);
#if 0
  bool copyMeshes = false;
  if(_C.frames.N!=display->Ccopy.frames.N) copyMeshes = true;
  else {
    for(uint i=0; i<_C.frames.N; i++) {
      rai::Shape* s = _C.frames.elem(i)->shape;
      rai::Shape* r = display->Ccopy.frames.elem(i)->shape;
      if((!s) != (!r)) { copyMeshes=true; break; }
      if(!s) continue;
      if(s->_type != r->_type) { copyMeshes=true; break; }
      if(s->size != r->size) { copyMeshes=true; break; }
      if(s->_mesh && r->_mesh && (s->_mesh->V.N != r->_mesh->V.N)) { copyMeshes=true; break; }
    }
  }
  if(copyMeshes) {
    display->Ccopy.copy(_C, false);
    //deep copy meshes!
    for(rai::Frame* f:display->Ccopy.frames) if(f->shape) {
        if(f->shape->_mesh) {
          f->shape->_mesh = make_shared<Mesh> (*f->shape->_mesh.get());
          f->shape->glListId = 0;
        }
      }
    LOG(0) <<"simulation frames changed: #frames: " <<display->Ccopy.frames.N <<" last: " <<display->Ccopy.frames(-1)->name;
  }
#if 0
  display->Ccopy.setFrameState(_C.getFrameState());
#else
  for(uint i=0; i<_C.frames.N; i++) {
    rai::Frame* f = _C.frames.elem(i);
    if(f->shape) display->Ccopy.frames.elem(i)->set_X() = f->ensure_X();
  }
#endif

  display->Ccopy.copyProxies(_C.proxies);
#endif
}

void Simulation_self::updateDisplayData(const byteA& _image, const floatA& _depth) {
  CHECK(display, "");
  display->mux.lock(RAI_HERE);
  display->image = _image;
  display->depth= _depth;
  display->mux.unlock();
}

byteA Simulation::getScreenshot() {
  if(!self->display) return byteA();
  byteA ret;
  self->display->mux.lock(RAI_HERE);
  ret = self->display->screenshot;
  self->display->mux.unlock();
  return ret;
}

//===========================================================================

Imp_CloseGripper::Imp_CloseGripper(Frame* _gripper, Joint* _joint,  Frame* _fing1, Frame* _fing2, Frame* _obj, double _speed)
  : gripper(_gripper), fing1(_fing1), fing2(_fing2), obj(_obj), finger1(fing1), finger2(fing2), joint(_joint), speed(_speed) {
  when = _beforePhysics;
  type = Simulation::_closeGripper;

  while(!finger1->shape || finger1->shape->type()!=ST_capsule) finger1=finger1->children.last();
  while(!finger2->shape || finger2->shape->type()!=ST_capsule) finger2=finger2->children.last();

  if(obj) {
    coll1 = make_unique<F_PairCollision>(F_PairCollision::_negScalar, false);
    coll1->setFrameIDs({finger1->ID, obj->ID});
    coll2 = make_unique<F_PairCollision>(F_PairCollision::_negScalar, false);
    coll2->setFrameIDs({finger2->ID, obj->ID});
  }

  if(joint->frame->parent->name.contains("robotiq")) speed *= 1.;
  else speed *= -.1;

//  CHECK(!fing1->joint->active || !fing1->joint->dim, "");
  if(joint) {
    limits = joint->limits;
    q = joint->get_q();
  } else {
    limits = fing1->ats->get<arr>("limits");
    axis = fing1->get_Q().pos;
    q = axis.sum();
    if(q) axis /= q; else axis = Vector_x;
  }
}

void Simulation::attach(Frame* gripper, Frame* obj) {
  obj = obj->getUpwardLink();
  gripper = gripper->getUpwardLink();
  Joint* j = C.attach(gripper, obj);
#if 0
  obj->inertia->type = BT_kinematic;

  // tell engine that object is now kinematic, not dynamic
  if(engine==_physx) {
    self->physx->changeObjectType(obj, BT_kinematic);
  } else if(engine==_bullet) {
    self->bullet->changeObjectType(obj, BT_kinematic);
  } else if(engine==_kinematic) {
  } else NIY;
#else
  if(engine==_physx) {
    self->physx->addJoint(j);
  } else if(engine==_bullet) {
    NIY;
  } else if(engine==_kinematic) {
  } else NIY;
#endif
}

void Simulation::detach(rai::Frame* obj) {
  obj = obj->getUpwardLink();
#if 0
  obj->unLink();
  obj->inertia->type = BT_dynamic;
  if(engine==_physx) {
    self->physx->changeObjectType(obj, rai::BT_dynamic);
  } else if(engine==_bullet) {
    self->bullet->changeObjectType(obj, rai::BT_dynamic);
  } else if(engine==_kinematic) {
  } else NIY;
#else
  if(engine==_physx) {
    self->physx->removeJoint(obj->joint);
  }
  obj->unLink();
#endif
}

void Imp_CloseGripper::modConfiguration(Simulation& S, double tau) {
  if(killMe) return;

  CHECK_EQ(&S.C, &fing1->C, "");
  CHECK_EQ(&S.C, &fing2->C, "");
  if(obj) {
    CHECK_EQ(&S.C, &obj->C, "");
  }

  //-- actually close gripper until both distances are < .001
  q += speed*tau;
  //cout <<"q: " <<q <<endl;
  if(joint) {
    S.C.setDofState({q}, {joint});
  } else {
    fing1->set_Q()->pos =  q*axis;
    fing2->set_Q()->pos = -q*axis;
  }

  if((speed>0. && q>limits(1))
      || (speed<0. && q < limits(0))) { //stop grasp by joint limits -> unsuccessful
    if(S.verbose>1) {
      LOG(1) <<"terminating closing gripper (limit) - nothing grasped";
    }
    killMe = true;
  } else if(obj) {
    double d1 = -coll1->eval(coll1->getFrames(S.C)).scalar();
    double d2 = -coll2->eval(coll2->getFrames(S.C)).scalar();
    //cout <<q <<" d1: " <<d1 <<" d2: " <<d2 <<endl;
    if(d1<1e-3 && d2<1e-3) { //stop when both close < 1mm
      //evaluate stability
      F_GraspOppose oppose;
      arr y = oppose.eval({finger1, finger2, obj});

      if(sumOfSqr(y) < 0.1) { //good enough -> success!
#if 1
        if(obj) S.attach(gripper, obj);
#endif

        //allows the user to know that gripper grasps something
        S.grasps.append(gripper);

        if(S.verbose>1) LOG(1) <<"terminating grasp of object " <<obj->name <<" - SUCCESS (distances d1:" <<d1 <<" d2:" <<d2 <<" oppose:" <<y.noJ() <<")";
      } else { //unsuccessful
        if(S.verbose>1) LOG(1) <<"terminating grasp of object " <<obj->name <<" - FAILURE (distances d1:" <<d1 <<" d2:" <<d2 <<" oppose:" <<y.noJ() <<")";
      }
      killMe = true;
    }
  }
}

//===========================================================================

Imp_GripperMove::Imp_GripperMove(Frame* _gripper, Joint* _joint, Frame* _fing1, Frame* _fing2, double _speed, double _stop)
  : gripper(_gripper), fing1(_fing1), fing2(_fing2), joint(_joint), speed(_speed), stop(_stop) {
  when = _beforePhysics;
  type = Simulation::_moveGripper;

  if(joint->frame->parent->name.contains("robotiq")) speed *= -1.;
  else speed *= .1;

  arr limits;
//  CHECK(!fing1->joint->active || !fing1->joint->dim, "");
  if(joint) {
    limits = joint->limits;
    q = joint->get_q();
  } else {
    limits = fing1->ats->get<arr>("limits");
    axis = fing1->get_Q().pos;
    q = axis.sum();
    if(q) axis /= q; else axis = Vector_x;
  }

  rai::clip(stop, limits(0), limits(1));

  if((stop>q && speed<0.) || (stop<q && speed>0.)) speed *= -1.;
}

void Imp_GripperMove::modConfiguration(Simulation& S, double tau) {
  if(killMe) return;

  CHECK_EQ(&S.C, &gripper->C, "");
  CHECK_EQ(&S.C, &fing1->C, "");
  CHECK_EQ(&S.C, &fing2->C, "");

  //-- actually open gripper until limit
  q += speed*tau;
  if(joint) {
    S.C.setDofState({q}, {joint});
  } else {
    fing1->set_Q()->pos =  q*axis;
    fing2->set_Q()->pos = -q*axis;
  }

  if((speed>0 && q>stop)
      || (speed<0 && q<stop)) { //stop opening
    if(S.verbose>1) LOG(1) <<"terminating opening gripper " <<gripper->name <<" at width " <<q;
    killMe = true;
  }
}

//===========================================================================

void Imp_ObjectImpulses::modConfiguration(Simulation& S, double tau) {
  timeToImpulse -= tau;
  if(timeToImpulse>0.) return;

  timeToImpulse = 1.;

  arr vel = randn(3);
  if(vel(2)<0.) vel(2)=0.;
  vel(0) *= .1;
  vel(1) *= .1;

  arr X, V;
  S.getState(X, V);

  V(obj->ID, 0, {}) = vel;

  S.setState(X, V);
}

//===========================================================================

Imp_BlockJoints::Imp_BlockJoints(const FrameL& _joints, Simulation& S)
  : joints(_joints) {
  when = _beforePhysics;
  qBlocked.resize(joints.N);
  arr q = S.C.getJointState();
  for(uint i=0; i<joints.N; i++) {
    rai::Joint* j = joints(i)->joint;
    CHECK(j, "");
    qBlocked(i) = q(j->qIndex);
  }
}

void Imp_BlockJoints::modConfiguration(Simulation& S, double tau) {
  CHECK_EQ(joints.N, qBlocked.N, "");
  arr q = S.C.getJointState();
  for(uint i=0; i<joints.N; i++) {
    rai::Joint* j = joints(i)->joint;
    CHECK(j, "");
    q(j->qIndex) = qBlocked(i);
  }
  S.C.setJointState(q);
}

void Imp_NoPenetrations::modConfiguration(Simulation& S, double tau) {

  uintA dynamicFrames;
  for(rai::Frame* f: S.C.getLinks()) {
    if(f->inertia)
      if(f->inertia->type == rai::BT_dynamic) {
        FrameL parts = {f};
        f->getRigidSubFrames(parts);
        for(rai::Frame* p: parts) dynamicFrames.append(p->ID);
//            cout << f->name.p << endl;
      }
  }

  for(uint t=0; t<100; t++) {

    arr y, J;
    S.C.kinematicsZero(y, J, 1);

    // Check penetrations between robot vs. static objects
    S.C.stepFcl();
    for(rai::Proxy& p: S.C.proxies) {
      if(!(dynamicFrames.contains(p.a->ID) || dynamicFrames.contains(p.b->ID))) {
        if(p.d > p.a->shape->radius() + p.b->shape->radius() + .01) continue;
        if(!p.collision) p.calc_coll();
        if(p.collision->getDistance()>0.) continue;

        arr Jp1, Jp2;
        p.a->C.jacobian_pos(Jp1, p.a, p.collision->p1);
        p.b->C.jacobian_pos(Jp2, p.b, p.collision->p2);

        arr y_dist, J_dist;
        p.collision->kinDistance(y_dist, J_dist, Jp1, Jp2);

        if(y_dist.scalar()>0.) continue;
        y -= y_dist.scalar();
        J -= J_dist;
      }
    }

    // Resolve penetration
    arr q = S.C.getJointState();
//    q -= 0.3*pseudoInverse(J, NoArr, 1e-2) * y;
//    q -= 0.3*inverse((~J)*J+1e-2*eye(q.d0)) * (~J) * y;

    arr vel = (~J) * y;
    q -= 0.3*vel; //the above two cause an unknown error (only) in rai-python... why?
    S.C.setJointState(q);
    if(length(vel) < 1e-3) return;
  }

}

//===========================================================================

uint& Simulation::pngCount() {
  return self->display->pngCount;
}

Mutex& Simulation::displayMutex() {
  return self->display->mux;
}

std::shared_ptr<PhysXInterface> Simulation::hidden_physx() {
  return self->physx;
}

OpenGL& Simulation::hidden_gl() {
  return self->display->ensure_gl();
}

void Simulation::loadTeleopCallbacks() {
  CHECK(!teleopCallbacks, "");
  teleopCallbacks = make_shared<TeleopCallbacks>(C);
  self->display->gl->addClickCall(teleopCallbacks.get());
  self->display->gl->addKeyCall(teleopCallbacks.get());
  self->display->gl->addHoverCall(teleopCallbacks.get());
}

bool TeleopCallbacks::hasNewMarker() {
  if(markerWasSet) { markerWasSet=false; return true; }
  return false;
}

bool TeleopCallbacks::clickCallback(OpenGL& gl) {
  LOG(0) <<"click";
  if(gl.modifiersCtrl() && gl.mouseIsDown) {
    LOG(0) <<"creating marker " <<nMarkers;
    arr normal;
    arr x = gl.get3dMousePos(normal);
    uint id = gl.get3dMouseObjID();
    if(id>=C.frames.N) return true;
    rai::Frame* f = C.frames(id);
    rai::Frame* m = marker;
    if(!m) m = C.addFrame(STRING("m" <<nMarkers <<"_" <<f->name));
    else if(m->parent) m->unLink();
    m->setParent(f);
    m->setShape(rai::ST_marker, {.1});
    rai::Transformation X=0;
    X.pos = x;
    X.rot.setDiff(Vector_z, normal);
    m->setPose(X);
    nMarkers++;
    markerWasSet=true;
  }
  return true;
}

bool TeleopCallbacks::keyCallback(OpenGL& gl) {
  if(gl.pressedkey=='q') stop=true;
  return true;
}

bool TeleopCallbacks::hoverCallback(OpenGL& gl) {
  grab = gl.modifiersShift();
  if(!mouseDepth) mouseDepth = gl.captureDepth(gl.mouseposy, gl.mouseposx);
  if(mouseDepth<.01 || mouseDepth==1.) {
    mouseDepth=0.;
    grab=false;
  }
  if(grab) {
    arr x = {double(gl.mouseposx), double(gl.mouseposy), mouseDepth};
    gl.camera.unproject_fromPixelsAndGLDepth(x, gl.width, gl.height);

    if(oldx.N) {
      arr del = x-oldx;
      q_ref(0) += del(0);
      q_ref(1) += del(1);
      q_ref(2) += del(2);
    }
    oldx=x;
  } else {
    oldx.clear();
    mouseDepth=0.;
  }
  return true;
}

} //namespace rai
