/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "simulation.h"
#include "splineRunner.h"
#include "../Algo/spline.h"
#include "../Kin/kin_swift.h"
#include "../Kin/proxy.h"

arr computeNextFeasibleConfiguration(rai::Configuration& K, arr q_ref, StringA& jointsInLimit, StringA& collisionPairs);

struct Sensor {
  rai::String name;
  rai::Camera cam;
  uint width=640, height=480;
  byteA backgroundImage;
  Sensor() {}
};

struct Simulation_self {
  rai::Array<Sensor> sensors;

  Mutex threadLock;

  rai::Configuration K_compute;
  OpenGL gl;

  uintA currentlyUsedJoints; //the joints that the spline refers to
  SplineRunner spline;
  double dt; // time stepping interval
  uint stepCount=0; // number of simulation steps
};

Simulation::Simulation(const rai::Configuration& _K, double dt)
  : K(_K) {
  self = new Simulation_self;
  setUsedRobotJoints(K.getJointIDs());
  self->dt = dt;

  self->gl.title = "Simulation";
  self->gl.camera.setDefault();
  self->gl.add(glStandardScene);
  self->gl.add(K);
//  self->gl.update();

  self->K_compute = K;
//  self->K_compute.swift().deactivate(K["table1"], K["iiwa_link_0_0"]);
//  self->K_compute.swift().deactivate(K["table1"], K["stick"]);
//  self->K_compute.swift().deactivate(K["table1"], K["stickTip"]);
}

Simulation::~Simulation() {
  delete self;
}

void Simulation::stepKin() {
  auto lock = self->threadLock(RAI_HERE);

  //read out the new reference
  arr q_ref = self->spline.run(self->dt);

  if(q_ref.N) {
//    self->K_ref.setJointState(q_ref); //for display only
//    timeToGo.set() = conv_double2Float64(maxPhase-phase);

    //compute feasible q_next (collision & limits)
    StringA jointsInLimit, collisionPairs;
//    arr q_next = computeNextFeasibleConfiguration(K, q_ref, jointsInLimit, collisionPairs);
    //    set the simulation's joint state
    //    K.setJointState(q_next);

//    setJointStateSafe(q_ref, jointsInLimit, collisionPairs);
    setJointState(self->currentlyUsedJoints, q_ref);
//    if(jointsInLimit.N)
//      cout <<"LIMITS:" <<jointsInLimit <<endl;
    if(collisionPairs.N)  cout <<"COLLISIONS:" <<collisionPairs <<endl;

    //display
//    K_disp.setJointState(K.q);
  }

  if(!(self->stepCount%10))
    self->gl.update(STRING("step=" <<self->stepCount <<" phase=" <<self->spline.phase <<" timeToGo=" <<self->spline.timeToGo() <<" #ref=" <<self->spline.refSpline.points.d0));

  self->stepCount++;

}

void Simulation::setJointState(const uintA& joints, const arr& q_ref) {
  auto lock = self->threadLock(RAI_HERE);

  K.setJointState(q_ref, joints);
}

void Simulation::setJointStateSafe(arr q_ref, StringA& jointsInLimit, StringA& collisionPairs) {
  auto lock = self->threadLock(RAI_HERE);

  arr q = q_ref;
  arr q0 = K.getJointState(self->currentlyUsedJoints);

  self->K_compute.setJointState(q0, self->currentlyUsedJoints);
  rai::Configuration& KK = self->K_compute;

  jointsInLimit.clear();
  collisionPairs.clear();

  //-- first check limits -> box constraints -> clip
  for(rai::Joint* j:KK.activeJoints) {
    bool active=false;
    if(j->limits.N) {
      for(uint d=0; d<j->dim; d++) {
        if(q(j->qIndex+d) < j->limits(0)) {
          q(j->qIndex+d) = j->limits(0);
          active=true;
        }
        if(q(j->qIndex+d) > j->limits(1)) {
          q(j->qIndex+d) = j->limits(1);
          active=true;
        }
      }
    }
    if(active) jointsInLimit.append(j->frame->name);
  }
  q_ref = q;

  //-- optimize s.t. collisions
  KK.setJointState(q, self->currentlyUsedJoints);
  KK.stepSwift();

//  KK.kinematicsPenetrations(y);

  arr y;
  double margin = .03;
  KK.kinematicsPenetration(y, NoArr, margin);

  for(rai::Proxy& p:KK.proxies) if(p.d<margin) {
      collisionPairs.append({p.a->name, p.b->name});
    }
  collisionPairs.reshape(2, collisionPairs.N/2);

  if(y.scalar()>.9 || collisionPairs.N) {
    //back to old config
    q = q0;

    //simple IK to find config closest to q_ref in nullspace of contact
    arr y, J, JJ, invJ, I=eye(KK.q.N);
    KK.setJointState(q, self->currentlyUsedJoints);
    KK.stepSwift();
    KK.kinematicsPenetration(y, J, margin);

    uint k;
    for(k=0; k<10; k++) {
      JJ = J*~J;
      for(uint i=0; i<JJ.d0; i++) JJ(i, i) += 1e-5;
      invJ = ~J * inverse_SymPosDef(JJ);

      arr qstep_nullspace = q_ref - q;
      if(length(qstep_nullspace)>.1) qstep_nullspace = .1/length(qstep_nullspace) * qstep_nullspace;

      arr qstep = invJ * (ARR(.9) - y); //.9 is the target for the proxy cost (1. is contact, 0. is margin))
      qstep += (I - invJ*J) * qstep_nullspace;

      if(absMax(qstep)<1e-6) break;

      q += qstep;

      KK.setJointState(q, self->currentlyUsedJoints);
      KK.stepSwift();
      KK.kinematicsPenetration(y, J, margin);
    }

    if(y.scalar()>1.) {
      //failed -> back to old config
      q=q0;
      KK.setJointState(q, self->currentlyUsedJoints);
      KK.stepSwift();
      KK.kinematicsPenetration(y, J, margin);
    }

    if(y.scalar()>1.) {
      LOG(-1) <<"what's going on??? y=" <<y;
    }
  }

  K.setJointState(q, self->currentlyUsedJoints);
}

void Simulation::setUsedRobotJoints(const uintA& joints) {
  auto lock = self->threadLock(RAI_HERE);

  if(self->currentlyUsedJoints!=joints) {
    if(self->spline.refPoints.N) {
      LOG(-1) <<"you changed the robot joints before the spline was done -- killing spline execution";
      self->spline.stop();
    }
    self->currentlyUsedJoints=joints;
  }
}

void Simulation::exec(const arr& x, const arr& t, bool append) {
  auto lock = self->threadLock(RAI_HERE);

  if(x.d1 != self->currentlyUsedJoints.N) {
    LOG(-1) <<"you're sending me a motion reference of wrong dimension!"
            <<"\n  I'm ignoring this"
            <<"\n  my dimension=" <<self->currentlyUsedJoints.N <<"  your message=" <<x.d1
            <<"\n  my joints=" <<self->currentlyUsedJoints;
    return;
  }

  self->spline.set(x, t, K.getJointState(self->currentlyUsedJoints), append);
}

void Simulation::exec(const StringA& command) {
  auto lock = self->threadLock(RAI_HERE);

  LOG(0) <<"CMD = " <<command <<endl;
  if(command(0)=="attach") {
    rai::Frame* a = K.getFrame(command(1));
    rai::Frame* b = K.getFrame(command(2));
    b = b->getUpwardLink();

    if(b->parent) b->unLink();
    b->linkFrom(a, true);
    (new rai::Joint(*b)) -> type=rai::JT_rigid;
    K.ensure_q();
  }
}

void Simulation::stop(bool hard) {
  auto lock = self->threadLock(RAI_HERE);

  self->spline.stop();
}

double Simulation::getTimeToGo() {
  auto lock = self->threadLock(RAI_HERE);
  return self->spline.timeToGo();
}

arr Simulation::getJointState() {
  auto lock = self->threadLock(RAI_HERE);
  return K.getJointState(self->currentlyUsedJoints);
}

arr Simulation::getFrameState() {
  auto lock = self->threadLock(RAI_HERE);
  return K.getFrameState();
}

arr Simulation::getObjectPoses(const StringA& objects) {
  auto lock = self->threadLock(RAI_HERE);

  FrameL objs;
  if(objects.N) {
    for(const rai::String& s:objects) objs.append(K[s]);
  } else { //non specified... go through the list and pick 'percets'
    for(rai::Frame* a:K.frames) {
      if(a->ats["percept"]) objs.append(a);
    }
  }

  arr X(objs.N, 7);
  for(uint i=0; i<objs.N; i++) X[i] = objs.elem(i)->ensure_X().getArr7d();
  return X;
}

StringA Simulation::getJointNames() {
  auto lock = self->threadLock(RAI_HERE);

  return K.getJointNames();
}

StringA Simulation::getObjectNames() {
  auto lock = self->threadLock(RAI_HERE);

  StringA objs;
  for(rai::Frame* a:K.frames) {
    if(a->ats["percept"]) objs.append(a->name);
  }
  return objs;
}

void Simulation::glDraw(OpenGL& gl) {
  glStandardLight(nullptr, gl);
  //  glEnable(GL_LIGHTING);

  for(Sensor& sen:self->sensors) {
    glDrawCamera(sen.cam);
    glDrawText(STRING("SENSOR " <<sen.name), 0., 0., 0.);
  }
}

//=============================================================================

#if 0
arr computeNextFeasibleConfiguration(rai::Configuration& K, arr q_ref, StringA& jointsInLimit, StringA& collisionPairs) {
  arr q = q_ref;
  arr q0 = K.getJointState();

  jointsInLimit.clear();
  collisionPairs.clear();

  //-- first check limits -> box constraints -> clip
  for(rai::Joint* j:K.fwdActiveJoints) {
    bool active=false;
    if(j->limits.N) {
      for(uint d=0; d<j->dim; d++) {
        if(q(j->qIndex+d) < j->limits(0)) {
          q(j->qIndex+d) = j->limits(0);
          active=true;
        }
        if(q(j->qIndex+d) > j->limits(1)) {
          q(j->qIndex+d) = j->limits(1);
          active=true;
        }
      }
    }
    if(active) jointsInLimit.append(j->frame.name);
  }
  q_ref = q;

  //-- optimize s.t. collisions
  K.setJointState(q);
//  K.stepSwift();

  arr y;
  K.kinematicsPenetrations(y);
  for(uint i=0; i<y.N; i++) if(y(i)>0.) {
      collisionPairs.append({K.proxies(i).a->name, K.proxies(i).b->name});
    }
  collisionPairs.reshape(2, collisionPairs.N/2);

  double margin = .03;
  K.kinematicsProxyCost(y, NoArr, margin);
  if(y.scalar()>.9 || collisionPairs.N) {
    //back to old config
    q = q0;

    //simple IK to find config closest to q_ref in nullspace of contact
    arr y, J, JJ, invJ, I=eye(K.q.N);
    K.setJointState(q);
    K.stepSwift();
    K.kinematicsProxyCost(y, J, margin);

    uint k;
    for(k=0; k<10; k++) {
      JJ = J*~J;
      for(uint i=0; i<JJ.d0; i++) JJ(i, i) += 1e-5;
      invJ = ~J * inverse_SymPosDef(JJ);

      arr qstep_nullspace = q_ref - q;
      if(length(qstep_nullspace)>.1) qstep_nullspace = .1/length(qstep_nullspace) * qstep_nullspace;

      arr qstep = invJ * (ARR(.9) - y); //.9 is the target for the proxy cost (1. is contact, 0. is margin))
      qstep += (I - invJ*J) * qstep_nullspace;

      if(absMax(qstep)<1e-6) break;

      q += qstep;

      K.setJointState(q);
      K.stepSwift();
      K.kinematicsProxyCost(y, J, margin);
    }

    if(y.scalar()>1.) {
      //failed -> back to old config
      q=q0;
      K.setJointState(q);
      K.stepSwift();
      K.kinematicsProxyCost(y, J, margin);
    }

    if(y.scalar()>1.) {
      LOG(-1) <<"what's going on??? y=" <<y;
    }
  }

  return q;
}
#endif

