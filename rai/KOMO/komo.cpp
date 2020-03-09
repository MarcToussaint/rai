/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "komo.h"
#include "komo-ext.h"

#include "../Algo/spline.h"
#include "../Gui/opengl.h"

#include "../Kin/frame.h"
#include "../Kin/switch.h"
#include "../Kin/forceExchange.h"
#include "../Kin/kin_swift.h"
#include "../Kin/kin_physx.h"
#include "../Kin/F_qFeatures.h"
#include "../Kin/TM_default.h"
#include "../Kin/F_pose.h"
#include "../Kin/TM_proxy.h"
#include "../Kin/F_PairCollision.h"
#include "../Kin/F_geometrics.h"
#include "../Kin/F_operators.h"
#include "../Kin/F_contacts.h"
#include "../Kin/F_dynamics.h"
#include "../Kin/TM_time.h"
#include "../Kin/TM_angVel.h"

#include "../Optim/optimization.h"
#include "../Optim/convert.h"
#include "../Optim/primalDual.h"
#include "../Optim/GraphOptim.h"

#include <iomanip>

#ifdef RAI_GL
#  include <GL/gl.h>
#endif

//#ifndef RAI_SWIFT
//#  define FCLmode
//#endif

using namespace rai;

//===========================================================================

double shapeSize(const Configuration& K, const char* name, uint i=2);

Shape* getShape(const Configuration& K, const char* name) {
  Frame* f = K.getFrameByName(name);
  Shape* s = f->shape;
  if(!s) {
    for(Frame* b:f->children) if(b->name==name && b->shape) { s=b->shape; break; }
  }
  return s;
}

KOMO::KOMO() : useSwift(true), verbose(1), komo_problem(*this), dense_problem(*this), graph_problem(*this) {
  verbose = getParameter<int>("KOMO/verbose", 1);
}

KOMO::KOMO(const Configuration& C, bool _useSwift)
  : KOMO() {
  setModel(C, _useSwift);
  world.optimizeTree();
  world.ensure_q();
}

KOMO::~KOMO() {
  gl.reset();
  if(opt) delete opt;
  if(logFile) delete logFile;
  objectives.clear();
//  listDelete(flags);
  listDelete(switches);
  listDelete(configurations);
}

void KOMO::setModel(const Configuration& C, bool _useSwift) {
  if(&C!=&world) world.copy(C, _useSwift);
  useSwift = _useSwift;
  if(useSwift){
#ifndef FCLmode
    world.swift();
#else
    world.fcl();
#endif
  }
  world.ensure_q();
}

void KOMO_ext::useJointGroups(const StringA& groupNames, bool OnlyTheseOrNotThese) {
  world.selectJointsByGroup(groupNames, OnlyTheseOrNotThese, false);

  world.reset_q();
  world.optimizeTree();
  world.getJointState();

//  world.meldFixedJoints();
//  world.removeUselessBodies();

//  FILE("z.komo.model") <<world;
}

void KOMO::setTiming(double _phases, uint _stepsPerPhase, double durationPerPhase, uint _k_order) {
  double maxPhase = _phases;
  stepsPerPhase = _stepsPerPhase;
  if(stepsPerPhase>=0) {
    T = ceil(stepsPerPhase*maxPhase);
    CHECK(T, "using T=0 to indicate inverse kinematics is deprecated.");
    tau = durationPerPhase/double(stepsPerPhase);
  }
//    setTiming(stepsPerPhase*maxPhase, durationPerPhase*maxPhase);
  k_order = _k_order;
}

void KOMO::setPairedTimes() {
  CHECK_EQ(k_order, 1, "NIY");
  for(uint s=1; s<T; s+=2) {
    configurations(s)  ->setTimes(1.98*tau); //(tau*(.98+int(s+1)-int(k_order)));
    configurations(s+1)->setTimes(0.02*tau); //(tau*(int(s)-int(k_order)));
  }
}

void KOMO::activateCollisions(const char* s1, const char* s2) {
  if(!useSwift) return;
  Frame* sh1 = world.getFrameByName(s1);
  Frame* sh2 = world.getFrameByName(s2);
  if(sh1 && sh2) world.swift().activate(sh1, sh2);
}

void KOMO::deactivateCollisions(const char* s1, const char* s2) {
  if(!useSwift) return;
  Frame* sh1 = world.getFrameByName(s1);
  Frame* sh2 = world.getFrameByName(s2);
  if(sh1 && sh2) world.swift().deactivate(sh1, sh2);
  else LOG(-1) <<"not found:" <<s1 <<' ' <<s2;
}

void KOMO::setTimeOptimization() {
  world.addTauJoint();
  ptr<Objective> o = addObjective({}, make_shared<TM_Time>(), OT_sos, {1e2}, {}, 1); //smooth time evolution
#if 1 //break the constraint at phase switches:
  CHECK(o->configs.nd==1 && o->configs.N==T, "");
  CHECK_GE(stepsPerPhase, 10, "NIY")
  for(uint t=2; t<o->configs.N; t+=stepsPerPhase) o->configs(t)=0;
#endif

  addObjective({}, make_shared<TM_Time>(), OT_sos, {1e-1}, {tau}); //prior on timing
  addObjective({}, make_shared<TM_Time>(), OT_ineq, {-1e1}, {.9*tau}); //lower bound on timing
}

//===========================================================================
//
// task specs
//

void KOMO::clearObjectives() {
  objectives.clear(); //listDelete(objectives);
  listDelete(switches);
}

ptr<Objective> KOMO::addObjective(const arr& times,
                              const ptr<Feature>& f, ObjectiveType type,
                              const arr& scale, const arr& target, int order,
                              int deltaFromStep, int deltaToStep) {
  if(!!scale) f->scale = scale;
  if(!!target) f->target = target;
  if(order>=0) f->order = order;

  CHECK_GE(k_order, f->order, "task requires larger k-order: " <<f->shortTag(world));
  ptr<Objective> task = make_shared<Objective>(f, type);
  task->name = f->shortTag(world);
  objectives.append(task);
  task->setCostSpecs(times, stepsPerPhase, T, deltaFromStep, deltaToStep, denseOptimization || sparseOptimization);
  if(denseOptimization || sparseOptimization) CHECK_EQ(task->configs.nd, 2, "");
  return task;
}

ptr<Objective> KOMO::addObjective(const arr& times, const FeatureSymbol& feat, const StringA& frames,
                                  ObjectiveType type, const arr& scale, const arr& target, int order,
                                  int deltaFromStep, int deltaToStep) {
  return addObjective(times, symbols2feature(feat, frames, world), type, scale, target, order, deltaFromStep, deltaToStep);
}

//void KOMO::addFlag(double time, Flag *fl, int deltaStep) {
//  if(time<0.) time=0.;
//  fl->stepOfApplication = conv_time2step(time, stepsPerPhase) + deltaStep;
//  flags.append(fl);
//}

void KOMO::addSwitch(double time, bool before, KinematicSwitch* sw) {
  sw->setTimeOfApplication(time, before, stepsPerPhase, T);
  switches.append(sw);
}

void KOMO::addSwitch(double time, bool before, rai::JointType type, SwitchInitializationType init,
                     const char* ref1, const char* ref2,
                     const rai::Transformation& jFrom, const rai::Transformation& jTo) {
  KinematicSwitch* sw = new KinematicSwitch(SW_joint, type, ref1, ref2, world, init, 0, jFrom, jTo);
  addSwitch(time, before, sw);
}

void KOMO::addSwitch_mode(SkeletonSymbol prevMode, SkeletonSymbol newMode, double time, double endTime, const char* prevFrom, const char* from, const char* to) {
  //-- creating a stable kinematic linking
  if(newMode==SY_stable || newMode==SY_stableOn) {
    // create the kinematic switch
    if(newMode==SY_stable) {
      addSwitch(time, true, JT_free, SWInit_copy, from, to);
    } else { //SY_stableOn
      Transformation rel = 0;
      rel.pos.set(0, 0, .5*(shapeSize(world, from) + shapeSize(world, to)));
      addSwitch(time, true, JT_transXYPhi, SWInit_copy, from, to, rel);
    }

    // ensure the DOF is constant throughout its existance
    if((endTime<0. && stepsPerPhase*time<T) || stepsPerPhase*endTime>stepsPerPhase*time+1) {
      addObjective({time, endTime}, make_shared<F_qZeroVel>(world, to), OT_eq, {1e1}, NoArr, 1, +1, -1);
      //      addObjective({time, endTime}, FS_poseRel, {from, to}, OT_eq, {1e1}, NoArr, 1, +1, -1);
    }

    //-- no relative jump at end
    //    if(endTime>0.) addObjective({endTime, endTime}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);

    if(prevMode==SY_initial || prevMode==SY_stable || prevMode==SY_stableOn) {
      //-- no acceleration at start: +0 INCLUDES (x-2, x-1, x0)
      //        if(k_order>1) addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);

      if(k_order>1) {
#if 1   //no jump: zero pose velocity
        if(prevFrom) addObjective({time}, FS_poseRel, {prevFrom, to}, OT_eq, {1e2}, NoArr, 1, 0, 0);
        else addObjective({time}, FS_pose, {to}, OT_eq, {1e0}, NoArr, 1, +0, +1);
#elif 1 //no jump: zero pose acceleration
        addObjective({time}, FS_pose, {to}, OT_eq, {1e2}, NoArr, 2, +0, +1);
#elif 1 //no jump: zero linang velocity
        if(prevFrom) addObjective({time}, make_shared<TM_LinAngVel>(world, to), OT_eq, {1e2}, NoArr, 2, +0, +1);
        else addObjective({time}, make_shared<TM_LinAngVel>(world, to), OT_eq, {1e1}, NoArr, 1, +0, +1);
#endif
      } else {
        addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);
      }
    } else {
      //-- no acceleration at start: +1 EXCLUDES (x-2, x-1, x0), ASSUMPTION: this is a placement that can excert impact
      if(k_order>1) addObjective({time}, make_shared<TM_LinAngVel>(world, to), OT_eq, {1e2}, NoArr, 2, +1, +1);
      else addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);
    }
  }

  if(newMode==SY_dynamic) {
    addSwitch(time, true, JT_free, SWInit_copy, from, to);
    //new contacts don't exist in step [-1], so we rather impose only zero acceleration at [-2,-1,0]
    addObjective({time}, FS_pose, {to}, OT_eq, {1e0}, NoArr, k_order, +0, +0);
    //... and physics starting from [-1,0,+1], ... until [-3,-2,-1]
    addObjective({time, endTime}, make_shared<F_NewtonEuler>(world, to), OT_eq, {1e0}, NoArr, k_order, +1, -1);
  }

  if(newMode==SY_dynamicTrans) {
    addSwitch(time, true, JT_trans3, SWInit_copy, from, to);
#if 0
    addObjective(time, endTime, new TM_Gravity2(world, to), OT_eq, NoArr, 3e1, k_order, +1, -1);
#else
    addObjective({time, endTime}, make_shared<F_NewtonEuler>(world, to, true), OT_eq, {3e1}, NoArr, k_order, +0, -1);
#endif
  }

  if(newMode==SY_dynamicOn) {
    Transformation rel = 0;
    rel.pos.set(0, 0, .5*(shapeSize(world, from) + shapeSize(world, to)));
    addSwitch(time, true, JT_transXYPhi, SWInit_copy, from, to, rel);
    if(k_order>=2) addObjective({time, endTime}, make_shared<F_Pose>(world, to), OT_eq, {3e1}, NoArr, k_order, +0, -1);
    //  else addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);
  }

  if(newMode==SY_quasiStatic) {
    addSwitch(time, true, JT_free, SWInit_copy, from, to);
    addObjective({time, endTime}, make_shared<F_NewtonEuler_DampedVelocities>(world, to), OT_eq, {1e1}, NoArr, 1, +0, -1);
  }

  if(newMode==SY_quasiStaticOn) {
    Transformation rel = 0;
    rel.pos.set(0, 0, .5*(shapeSize(world, from) + shapeSize(world, to)));
    addSwitch(time, true, JT_transXYPhi, SWInit_copy, from, to, rel);
#if 0
    addObjective({time, endTime}, make_shared<F_NewtonEuler_DampedVelocities>(world, to, 0., false), OT_eq, {1e2}, NoArr, 1, +0, -1);
#else
    //eq for 3DOFs only
    ptr<Objective> o = addObjective({time, endTime}, make_shared<F_NewtonEuler_DampedVelocities>(world, to, 0., false), OT_eq, {1e2}, NoArr, 1, +0, -1);
    o->map->scale=1e2 * arr({3,6}, {
      1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1
    });
    //sos penalty of other forces
    o = addObjective({time, endTime}, make_shared<F_NewtonEuler_DampedVelocities>(world, to, 0., false), OT_sos, {1e2}, NoArr, 1, +0, -1);
    o->map->scale=1e1 * arr({3,6}, {
      0, 0, 1, 0, 0, 0,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0
    });
#endif
//    addObjective({time, endTime}, make_shared<F_pushed>(world, to), OT_eq, {1e0}, NoArr, 1, +0, -1);

    //-- no acceleration at start: +1 EXCLUDES (x-2, x-1, x0), ASSUMPTION: this is a placement that can excert impact
    if(k_order>1) addObjective({time}, make_shared<TM_LinAngVel>(world, to), OT_eq, {1e2}, NoArr, 2, +0, +1);
    else addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);

  }
}

void KOMO::addSwitch_stable(double time, double endTime, const char* from, const char* to) {
#if 1
  addSwitch_mode(SY_none, SY_stable, time, endTime, NULL, from, to);
#else
  addSwitch(time, true, JT_free, SWInit_zero, from, to);
  //-- DOF-is-constant constraint
  if(endTime<0. || stepsPerPhase*endTime>stepsPerPhase*time+1)
    addObjective({time, endTime}, make_shared<F_qZeroVel>(world, to), OT_eq, {3e1}, NoArr, 1, +1, -1);
  //-- no relative jump at end
  if(endTime>0.) addObjective({endTime, endTime}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);
  //-- no object acceleration at start: +0 include (x-2, x-1, x0), which enforces a SMOOTH pickup
  if(k_order>1) addObjective({time}, make_shared<TM_LinAngVel>(world, to), OT_eq, {1e2}, NoArr, 2, +0, +1);
  else addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);
#endif
}

void KOMO::addSwitch_stableOn(double time, double endTime, const char* from, const char* to) {
#if 1
  addSwitch_mode(SY_none, SY_stableOn, time, endTime, NULL, from, to);
#else
  Transformation rel = 0;
  rel.pos.set(0, 0, .5*(shapeSize(world, from) + shapeSize(world, to)));
  addSwitch(time, true, JT_transXYPhi, SWInit_zero, from, to, rel);
  //-- DOF-is-constant constraint
  if(endTime<0. || stepsPerPhase*endTime>stepsPerPhase*time+1)
    addObjective({time, endTime}, make_shared<F_qZeroVel>(world, to), OT_eq, {3e1}, NoArr, 1, +1, -1);
  //-- no relative jump at end
  if(endTime>0.) addObjective({endTime, endTime}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);
  //-- no acceleration at start: +1 EXCLUDES (x-2, x-1, x0), ASSUMPTION: this is a placement that can excert impact
  if(k_order>1) addObjective({time}, make_shared<TM_LinAngVel>(world, to), OT_eq, {1e2}, NoArr, 2, +1, +1);
//  else addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);
#endif
}

void KOMO::addSwitch_dynamic(double time, double endTime, const char* from, const char* to, bool dampedVelocity) {
  addSwitch(time, true, JT_free, SWInit_copy, from, to);
  if(!dampedVelocity)
    addObjective({time, endTime}, make_shared<F_NewtonEuler>(world, to), OT_eq, {1e0}, NoArr, 2, +0, -1);
  else
    addObjective({time, endTime}, make_shared<F_NewtonEuler_DampedVelocities>(world, to), OT_eq, {1e2}, NoArr, 1, +0, -1);
//  addObjective({time}, make_shared<TM_LinAngVel>(world, to), OT_eq, {1e2}, NoArr, 2); //this should be implicit in the NE equations!
}

void KOMO::addSwitch_dynamicTrans(double time, double endTime, const char* from, const char* to) {
  addSwitch(time, true, JT_trans3, SWInit_copy, from, to);
#if 0
  addObjective(time, endTime, new TM_Gravity2(world, to), OT_eq, NoArr, 3e1, k_order, +1, -1);
#else
  addObjective({time, endTime}, make_shared<F_NewtonEuler>(world, to, true), OT_eq, {3e1}, NoArr, k_order, +0, -1);
#endif
}

void KOMO::addSwitch_dynamicOn(double time, double endTime, const char* from, const char* to) {
  Transformation rel = 0;
  rel.pos.set(0, 0, .5*(shapeSize(world, from) + shapeSize(world, to)));
  addSwitch(time, true, JT_transXYPhi, SWInit_zero, from, to, rel);
  if(k_order>=2) addObjective({time, endTime}, make_shared<F_Pose>(world, to), OT_eq, {3e1}, NoArr, k_order, +0, -1);
//  else addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);
}

void KOMO::addSwitch_dynamicOnNewton(double time, double endTime, const char* from, const char* to) {
  Transformation rel = 0;
  rel.pos.set(0, 0, .5*(shapeSize(world, from) + shapeSize(world, to)));
  addSwitch(time, true, JT_transXYPhi, SWInit_zero, from, to, rel);
  if(k_order>=2) addObjective({time, endTime}, make_shared<F_NewtonEuler>(world, to), OT_eq, {1e0}, NoArr, k_order, +0, -1);
}

void KOMO::addSwitch_magic(double time, double endTime, const char* from, const char* to, double sqrAccCost, double sqrVelCost) {
  addSwitch(time, true, JT_free, SWInit_copy, from, to);
  if(sqrVelCost>0. && k_order>=1) {
    addObjective({time, endTime}, make_shared<TM_LinAngVel>(world, to), OT_sos, {sqrVelCost}, NoArr, 1);
  }
  if(sqrAccCost>0. && k_order>=2) {
    addObjective({time, endTime}, make_shared<TM_LinAngVel>(world, to), OT_sos, {sqrAccCost}, NoArr, 2);
  }
}

void KOMO::addSwitch_magicTrans(double time, double endTime, const char* from, const char* to, double sqrAccCost) {
  addSwitch(time, true, JT_transZ, SWInit_copy, from, to);
  if(sqrAccCost>0.) {
    addObjective({time, endTime}, make_shared<TM_LinAngVel>(world, to), OT_eq, {sqrAccCost}, NoArr, 2);
  }
}

void KOMO::addSwitch_on(double time, const char* from, const char* to, bool copyInitialization) {
  Transformation rel = 0;
  rel.pos.set(0, 0, .5*(shapeSize(world, from) + shapeSize(world, to)));
  addSwitch(time, true, JT_transXYPhi, (copyInitialization?SWInit_copy:SWInit_zero), from, to, rel);
}

void KOMO::addContact_slide(double startTime, double endTime, const char* from, const char* to) {
  addSwitch(startTime, true, new rai::KinematicSwitch(rai::SW_addContact, rai::JT_none, from, to, world));
  if(endTime>0.) addSwitch(endTime, false, new rai::KinematicSwitch(rai::SW_delContact, rai::JT_none, from, to, world));

  //constraints
  addObjective({startTime, endTime}, make_shared<TM_Contact_ForceIsNormal>(world, from, to), OT_eq, {1e2});
  addObjective({startTime, endTime}, make_shared<TM_Contact_ForceIsPositive>(world, from, to), OT_ineq, {1e2});
  addObjective({startTime, endTime}, make_shared<TM_Contact_POAisInIntersection_InEq>(world, from, to), OT_ineq, {1e1});
  addObjective({startTime, endTime}, make_shared<TM_PairCollision>(world, from, to, TM_PairCollision::_negScalar, false), OT_eq, {1e1});

  //regularization
  addObjective({startTime, endTime}, make_shared<F_LinearForce>(world, from, to), OT_sos, {1e-2}, NoArr, 2, +2, 0);
  addObjective({startTime, endTime}, make_shared<F_LinearForce>(world, from, to), OT_sos, {1e-2});
  addObjective({startTime, endTime}, make_shared<TM_Contact_POA>(world, from, to), OT_sos, {1e-2}, NoArr, 2, +2, +0);
  addObjective({startTime, endTime}, make_shared<TM_Contact_POAzeroRelVel>(world, from, to), OT_sos, {1e-1}, NoArr, 1, +1, +0);
}

void KOMO::addContact_stick(double startTime, double endTime, const char* from, const char* to) {
  addSwitch(startTime, true, new rai::KinematicSwitch(rai::SW_addContact, rai::JT_none, from, to, world));
  if(endTime>0.) addSwitch(endTime, false, new rai::KinematicSwitch(rai::SW_delContact, rai::JT_none, from, to, world));

  //constraints
  addObjective({startTime, endTime}, make_shared<TM_Contact_ForceIsPositive>(world, from, to), OT_ineq, {1e1});
  addObjective({startTime, endTime}, make_shared<TM_Contact_POAisInIntersection_InEq>(world, from, to), OT_ineq, {1e1});
  addObjective({startTime, endTime}, make_shared<TM_PairCollision>(world, from, to, TM_PairCollision::_negScalar, false), OT_eq, {1e1});
  addObjective({startTime, endTime}, make_shared<TM_Contact_POAzeroRelVel>(world, from, to), OT_eq, {1e0}, NoArr, 1, +1, +1);

  //regularization
//  addObjective({startTime, endTime}, make_shared<TM_Contact_Force>(world, from, to), OT_sos, {1e-2}, NoArr, 2, +2, 0);
  addObjective({startTime, endTime}, make_shared<F_LinearForce>(world, from, to), OT_sos, {1e-4});
  addObjective({startTime, endTime}, make_shared<TM_Contact_POA>(world, from, to), OT_sos, {1e-2}, NoArr, 2, +2, +0);
  addObjective({startTime, endTime}, make_shared<TM_Contact_POA>(world, from, to), OT_sos, {1e-2}, NoArr, 1, +1, +0);
}

void KOMO::addContact_ComplementarySlide(double startTime, double endTime, const char* from, const char* to) {
  addSwitch(startTime, true, new rai::KinematicSwitch(rai::SW_addContact, rai::JT_none, from, to, world));
  if(endTime>0.) addSwitch(endTime, false, new rai::KinematicSwitch(rai::SW_delContact, rai::JT_none, from, to, world));

  //constraints
  addObjective({startTime, endTime}, make_shared<TM_Contact_ForceIsNormal>(world, from, to), OT_eq, {1e2});
  addObjective({startTime, endTime}, make_shared<TM_Contact_ForceIsComplementary>(world, from, to), OT_eq, {1e2});
  addObjective({startTime, endTime}, make_shared<TM_Contact_NormalVelIsComplementary>(world, from, to, 0., 0.), OT_eq, {1e2}, NoArr, 1, +1);
  addObjective({startTime, endTime}, make_shared<TM_PairCollision>(world, from, to, TM_PairCollision::_negScalar, false), OT_ineq, {1e1});

  //regularization
  addObjective({startTime, endTime}, make_shared<F_LinearForce>(world, from, to), OT_sos, {1e-4});
  addObjective({startTime, endTime}, make_shared<TM_Contact_POA>(world, from, to), OT_sos, {1e-2}, NoArr, 2, +3, +0);
  addObjective({startTime, endTime}, make_shared<TM_Contact_POA>(world, from, to), OT_sos, {1e-2}, NoArr, 1, +1, +0);
//  addObjective({startTime, endTime}, make_shared<TM_Contact_POAzeroRelVel>(world, from, to), OT_sos, {1e-1}, NoArr, 1, +1, +0);
}

void KOMO::addContact_staticPush(double startTime, double endTime, const char* from, const char* to) {
  HALT("OBSOLETE");
  addSwitch(startTime, true, new rai::KinematicSwitch(rai::SW_addContact, rai::JT_none, from, to, world));
  if(endTime>0.) addSwitch(endTime, false, new rai::KinematicSwitch(rai::SW_delContact, rai::JT_none, from, to, world));

  addObjective({startTime, endTime}, make_shared<TM_Contact_ForceIsNormal>(world, from, to), OT_sos, {1e1});
  addObjective({startTime, endTime}, make_shared<TM_Contact_ForceIsPositive>(world, from, to), OT_ineq, {1e2});
  addObjective({startTime, endTime}, make_shared<TM_Contact_POAisInIntersection_InEq>(world, from, to), OT_ineq, {1e1});
  addObjective({startTime, endTime}, make_shared<TM_Contact_POAmovesContinuously>(world, from, to), OT_sos, {1e0}, NoArr, 1, +1, +0);
  addObjective({startTime, endTime}, make_shared<F_LinearForce>(world, from, to), OT_sos, {1e-1});
  addObjective({startTime, endTime}, make_shared<TM_Contact_POAzeroRelVel>(world, from, to), OT_sos, {1e-1}, NoArr, 1, +1, +0);
  //  addObjective({startTime, endTime}, make_shared<TM_Contact_POAzeroRelVel>(world, from, to), OT_eq, {1e1}, NoArr, 1, +1, +0);
  //  addObjective({time}, make_shared<F_pushed>(world, to), OT_eq, {1e1}, NoArr, 1, +1, +0);
}

void KOMO::addContact_noFriction(double startTime, double endTime, const char* from, const char* to) {
  HALT("OBSOLETE");
  addSwitch(startTime, true, new rai::KinematicSwitch(rai::SW_addContact, rai::JT_none, from, to, world));
  if(endTime>0.) addSwitch(endTime, false, new rai::KinematicSwitch(rai::SW_delContact, rai::JT_none, from, to, world));

  addObjective({startTime, endTime}, make_shared<TM_Contact_ForceIsNormal>(world, from, to), OT_eq, {3e1});
  addObjective({startTime, endTime}, make_shared<TM_Contact_ForceIsPositive>(world, from, to), OT_ineq, {1e1});
  addObjective({startTime, endTime}, make_shared<TM_Contact_POAisInIntersection_InEq>(world, from, to), OT_ineq, {1e1});
  addObjective({startTime, endTime}, make_shared<TM_Contact_POAmovesContinuously>(world, from, to), OT_sos, {1e0}, NoArr, 1, +1, +0);
  addObjective({startTime, endTime}, make_shared<F_LinearForce>(world, from, to), OT_sos, {1e-4});
  addObjective({startTime, endTime}, make_shared<TM_PairCollision>(world, from, to, TM_PairCollision::_negScalar, false), OT_eq, {1e1});
}

void KOMO::addContact_elasticBounce(double time, const char* from, const char* to, double elasticity, double stickiness) {
  addSwitch(time, true,  new rai::KinematicSwitch(rai::SW_addContact, rai::JT_none, from, to, world));
  addSwitch(time, false, new rai::KinematicSwitch(rai::SW_delContact, rai::JT_none, from, to, world));

  if(stickiness<=0.) addObjective({time}, make_shared<TM_Contact_ForceIsNormal>(world, from, to), OT_eq, {1e2});
  addObjective({time}, make_shared<TM_Contact_ForceIsPositive>(world, from, to), OT_ineq, {1e1});
  addObjective({time}, make_shared<TM_Contact_POAisInIntersection_InEq>(world, from, to), OT_ineq, {1e1});
  addObjective({time}, make_shared<F_LinearForce>(world, from, to), OT_sos, {1e-4});
  addObjective({time}, make_shared<TM_PairCollision>(world, from, to, TM_PairCollision::_negScalar, false), OT_eq, {1e1});

  if(!elasticity && stickiness>=1.) {
    addObjective({time}, make_shared<TM_Contact_POAzeroRelVel>(world, from, to), OT_eq, {1e1}, NoArr, 2, +1, +1);
  } else {
    addObjective({time}, make_shared<TM_Contact_ElasticVel>(world, from, to, elasticity, stickiness), OT_eq, {1e1}, NoArr, 2, +1, +1);
  }
}

void KOMO::setKS_slider(double time, double endTime, bool before, const char* obj, const char* slider, const char* table) {
  //disconnect object from grasp ref
//  setKinematicSwitch(time, before, "delete", nullptr, obj);

  //the two slider objects
  String slidera = STRING(slider <<'a');
  String sliderb = STRING(slider <<'b');

  Transformation rel = 0;
  rel.addRelativeTranslation(0., 0., .5*(shapeSize(world, obj) + shapeSize(world, table)));

//  setKinematicSwitch(time, true, "transXYPhiZero", table, slidera, rel);
//  setKinematicSwitch(time, true, "hingeZZero", sliderb, obj);
  addSwitch(time, true, JT_transXYPhi, SWInit_zero, table, slidera, rel);
  addSwitch(time, true, JT_hingeZ, SWInit_zero, sliderb, obj);

  addObjective({time, endTime}, make_shared<F_qZeroVel>(world, slidera), OT_eq, {3e1}, NoArr, 1, +1, +0);
  addObjective({time, endTime}, make_shared<F_qZeroVel>(world, obj), OT_eq, {3e1}, NoArr, 1, +1, -1);
  addObjective({time}, make_shared<TM_LinAngVel>(world, obj), OT_eq, {1e2}, NoArr, 1);

//  setKinematicSwitch(time, before, "sliderMechanism", table, obj, rel );

//  if(!actuated)
//    setKinematicSwitch(time, before, "hingeZZero", slider, obj, rel );
//  else
  //    setKinematicSwitch(time, before, "transXActuated", slider, obj, rel );
}

void KOMO::setHoming(double startTime, double endTime, double prec, const char* keyword) {
  uintA bodies;
  Joint* j;
  for(Frame* f:world.frames) if((j=f->joint) && j->qDim()>0 && (!keyword || f->ats[keyword])) bodies.append(f->ID);
//  cout <<"HOMING: "; for(uint i:bodies) cout <<' ' <<world.frames(i)->name;  cout <<endl;
  addObjective({startTime, endTime}, make_shared<F_qItself>(bodies, true), OT_sos, {prec}, NoArr); //world.q, prec);
}

//void KOMO::setSquaredQAccelerations(double startTime, double endTime, double prec) {
//  CHECK_GE(k_order, 2,"");
//  addObjective({startTime, endTime}, make_shared<TM_Transition>(world), OT_sos, {}, NoArrprec);
//}

auto getQFramesAndScale(const rai::Configuration& C){
  struct Return{ uintA frames; arr scale; } R;
  for(rai::Frame* f : C.frames){
    if(f->joint && f->joint->active && f->joint->dim>0 && f->joint->H>0. && f->joint->type!=JT_tau) {
      CHECK(!f->joint->mimic, "");
      R.frames.append(TUP(f->ID, f->parent->ID));
      R.scale.append(f->joint->H, f->joint->dim);
    }
  }
  R.frames.reshape(-1, 2);
  //  cout <<scale <<endl <<world.getHmetric() <<endl;
  return R;
}

void KOMO::add_qControlObjective(const arr& times, uint order, double scale, const arr& target, int deltaFromStep, int deltaToStep){
  auto F = getQFramesAndScale(world);
  scale *= sqrt(tau);

  CHECK_GE(k_order, order, "");
  ptr<Objective> o = addObjective(times, make_shared<F_qItself>(F.frames), OT_sos, scale*F.scale, target, order, deltaFromStep, deltaToStep);
}

void KOMO::setSquaredQAccVelHoming(double startTime, double endTime, double accPrec, double velPrec, double homingPrec, int deltaFromStep, int deltaToStep) {
  auto F = getQFramesAndScale(world);
  F.scale *= sqrt(tau);

  if(accPrec) {
    addObjective({startTime, endTime}, make_shared<F_qItself>(F.frames), OT_sos, accPrec*F.scale, NoArr, 2, deltaFromStep, deltaToStep);
  }
  if(velPrec) {
    addObjective({startTime, endTime}, make_shared<F_qItself>(F.frames), OT_sos, velPrec*F.scale, NoArr, 1, deltaFromStep, deltaToStep);
  }
  if(homingPrec) {
    addObjective({startTime, endTime}, make_shared<F_qItself>(F.frames, true), OT_sos, {homingPrec*sqrt(tau)}, NoArr, 0, deltaFromStep, deltaToStep);
  }
}

//void KOMO::setSquaredQVelocities(double startTime, double endTime, double prec) {
//  auto *map = new TM_Transition(world);
//  map->velCoeff = 1.;
//  map->accCoeff = 0.;
//  addObjective(startTime, endTime, map, OT_sos, NoArr, prec, 1);
//}

//void KOMO::setFixEffectiveJoints(double startTime, double endTime, double prec) {
////  setTask(startTime, endTime, new TM_Transition(world, true), OT_eq, NoArr, prec, 1); //NOTE: order=1!!
//  addObjective({startTime, endTime}, make_shared<TM_FlagConstraints>(), OT_eq, {}, NoArrprec, k_order);
//  addObjective({startTime, endTime}, make_shared<TM_FlagCosts>(), OT_sos, {1.}, NoArr, k_order);
//}

//void KOMO::setFixSwitchedObjects(double startTime, double endTime, double prec) {
//  addObjective({startTime, endTime}, make_shared<TM_FixSwichedObjects>(), OT_eq, {}, NoArrprec, k_order);
//}

void KOMO::setSquaredQuaternionNorms(double startTime, double endTime, double prec) {
  addObjective({startTime, endTime}, make_shared<F_qQuaternionNorms>(), OT_eq, {prec}, NoArr);
}

void KOMO::setHoldStill(double startTime, double endTime, const char* shape, double prec) {
  Frame* s = world.getFrameByName(shape);
  addObjective({startTime, endTime}, make_shared<F_qItself>(TUP(s->ID)), OT_sos, {prec}, NoArr, 1);
}

void KOMO_ext::setPosition(double startTime, double endTime, const char* shape, const char* shapeRel, ObjectiveType type, const arr& target, double prec) {
  addObjective({startTime, endTime}, make_shared<TM_Default>(TMT_posDiff, world, shape, NoVector, shapeRel, NoVector), type, {prec}, target);
}

void KOMO_ext::setOrientation(double startTime, double endTime, const char* shape, const char* shapeRel, ObjectiveType type, const arr& target, double prec) {
//  setTask(startTime, endTime, new TM_Align(world, shape, shapeRel), type, target, prec);
  addObjective({startTime, endTime}, make_shared<TM_Default>(TMT_quatDiff, world, shape, NoVector, shapeRel, NoVector), type, {prec}, target);
}

void KOMO_ext::setVelocity(double startTime, double endTime, const char* shape, const char* shapeRel, ObjectiveType type, const arr& target, double prec) {
  addObjective({startTime, endTime}, make_shared<TM_Default>(TMT_posDiff, world, shape, NoVector, shapeRel, NoVector), type, {prec}, target, 1);
}

void KOMO_ext::setLastTaskToBeVelocity() {
  objectives.last()->map->order = 1; //set to be velocity!
}

void KOMO_ext::setImpact(double time, const char* a, const char* b) {
  add_touch(time, time, a, b);
  HALT("obsolete");
//  add_impulse(time, a, b);
}

void KOMO_ext::setOverTheEdge(double time, const char* object, const char* from, double margin) {
  double negMargin = margin + .5*shapeSize(world, object, 0); //how much outside the bounding box?
  addObjective({time, time+.5},
               make_shared<F_Max>(make_shared<TM_AboveBox>(world, object, from, -negMargin), true), //this is the max selection -- only one of the four numbers need to be outside the BB
               OT_ineq, {3e0}); //NOTE: usually this is an inequality constraint <0; here we say this should be zero for a negative margin (->outside support)
}

void KOMO_ext::setInertialMotion(double startTime, double endTime, const char* object, const char* base, double g, double c) {
  addSwitch(startTime, true, JT_trans3, SWInit_zero, base, object);
//  setFlag(time, new Flag(FT_gravityAcc, world[object]->ID, 0, true),+1); //why +1: the kinematic switch triggers 'FixSwitchedObjects' to enforce acc 0 for time slide +0
//  setFlag(startTime, new Flag(FL_noQControlCosts, world[object]->ID, 0, true), +2);
//  setFlag(endTime, new Flag(FL_noQControlCosts, world[object]->ID, 0, true, false), +1);
  if(k_order>=2) {
    NIY;
//    setTask(startTime, endTime, new TM_InertialMotion(world, object, g, c), OT_sos, {}, 1e1, 2);
  }
}

/// a standard pick up: lower-attached-lift; centered, from top
void KOMO_ext::setGrasp(double time, double endTime, const char* endeffRef, const char* object, int verbose, double weightFromTop, double timeToLift) {
  if(verbose>0) cout <<"KOMO_setGrasp t=" <<time <<" endeff=" <<endeffRef <<" obj=" <<object <<endl;
  //  String& endeffRef = world.getFrameByName(graspRef)->body->inLinks.first()->from->shapes.first()->name;

  //-- position the hand & graspRef
  //hand upright
  //  setTask(time, time, new TM_Default(TMT_vec, world, endeffRef, Vector_z), OT_sos, {0.,0.,1.}, weightFromTop);

  //hand center at object center (could be replaced by touch)
//  setTask(time, time, new TM_Default(TMT_posDiff, world, endeffRef, NoVector, object, NoVector), OT_eq, NoArr, 3e1);

  //hand grip axis orthogonal to object length axis
//  setTask(time, time, new TM_Default(TMT_vecAlign, world, endeffRef, Vector_x, object, Vector_x), OT_sos, NoArr, 3e0);
  //hand grip axis orthogonal to object length axis
//  setTask(time, time, new TM_Default(TMT_vecAlign, world, endeffRef, Vector_y, object, Vector_x), OT_sos, {-1.}, 3e0);

  //hand touches object
//  Shape *endeffShape = world.getFrameByName(endeffRef)->body->shapes.first();
//  setTask(time, time, new TM_GJK(endeffShape, world.getFrameByName(object), false), OT_eq, NoArr, 3e1);

  //disconnect object from table
//  setKinematicSwitch(time, true, "delete", nullptr, object);
  //connect graspRef with object
#if 0
  setKinematicSwitch(time, true, new KinematicSwitch(SW_effJoint, JT_quatBall, endeffRef, object, world));
  setKinematicSwitch(time, true, new KinematicSwitch(SW_insertEffJoint, JT_trans3, nullptr, object, world));
  setTask(time, time, new TM_InsideBox(world, endeffRef, NoVector, object), OT_ineq, NoArr, 1e1);
#else
//  addSwitch(time, true, new KinematicSwitch(SW_effJoint, JT_free, endeffRef, object, world));
  addSwitch_stable(time, endTime, endeffRef, object);
  addObjective({time}, make_shared<TM_InsideBox>(world, endeffRef, NoVector, object), OT_ineq, {1e1});
//  setTouch(time, time, endeffRef, object);
#endif

//  if(stepsPerPhase>2 && timeToLift>0.){ //velocities down and up
//    setTask(time-timeToLift, time-2.*timeToLift/3, new TM_Default(TMT_pos, world, endeffRef), OT_sos, {0.,0.,-.1}, 1e0, 1); //move down
//    setTask(time-timeToLift/3,  time+timeToLift/3, new TM_Default(TMT_pos, world, endeffRef), OT_sos, {0.,0.,0.}, 3e0, 1); //move down
//    setTask(time+2.*timeToLift/3, time+timeToLift, new TM_Default(TMT_pos, world, endeffRef), OT_sos, {0.,0.,.1}, 1e0, 1); // move up
//  }

//  setFlag(time, new Flag(FL_clear, world[object]->ID, 0, true));
//  setFlag(time, new Flag(FL_zeroQVel, world[object]->ID, 0, true));
//  setFlag(time, new Flag(FL_kinematic, world[object]->getUpwardLink()->ID, 0, true));
}

/// a standard pick up: lower-attached-lift; centered, from top
void KOMO_ext::setGraspStick(double time, const char* endeffRef, const char* object, int verbose, double weightFromTop, double timeToLift) {
  if(verbose>0) cout <<"KOMO_setGraspStick t=" <<time <<" endeff=" <<endeffRef <<" obj=" <<object <<endl;

  //disconnect object from table
//  setKinematicSwitch(time, true, "delete", nullptr, object);

  //connect graspRef with object
  addSwitch(time, true, JT_quatBall, SWInit_zero, endeffRef, object);
  HALT("deprecated"); //addSwitch(time, true, "insert_transX", nullptr, object);
//  setTask(time, time,
//          new TM_LinTrans(
//              new TM_Default(TMT_posDiff, world, endeffRef, NoVector, object, NoVector),
//              arr(2,3,{0,1,0,0,0,1}), {}),
//          OT_eq, NoArr, 3e1);
  addObjective({time}, make_shared<TM_InsideBox>(world, endeffRef, NoVector, object), OT_ineq, {1e1});

  if(stepsPerPhase>2) { //velocities down and up
    addObjective({time-timeToLift, time}, make_shared<TM_Default>(TMT_pos, world, endeffRef), OT_sos, {3e0}, {0., 0., -.1}, 1); //move down
    addObjective({time, time+timeToLift}, make_shared<TM_Default>(TMT_pos, world, object), OT_sos, {3e0}, {0., 0., .1}, 1); // move up
  }
}

/// standard place on a table
void KOMO_ext::setPlace(double time, const char* endeff, const char* object, const char* placeRef, int verbose) {
  if(verbose>0) cout <<"KOMO_setPlace t=" <<time <<" obj=" <<object <<" place=" <<placeRef <<endl;

//  if(stepsPerPhase>2){ //velocities down and up
//    if(endeff){
//      setTask(time-.15, time-.10, new TM_Default(TMT_pos, world, endeff), OT_sos, {0.,0.,-.1}, 3e0, 1); //move down
//      setTask(time-.05, time+.05, new TM_Default(TMT_pos, world, endeff), OT_sos, {0.,0.,0. }, 1e1, 1); //hold still
//      setTask(time+.10, time+.15, new TM_Default(TMT_pos, world, endeff), OT_sos, {0.,0.,+.1}, 3e0, 1); //move up
//    }else{
////      setTask(time-.15, time, new TM_Default(TMT_pos, world, object), OT_sos, {0.,0.,-.1}, 3e0, 1); //move down
//    }
//  }

  //place upright
//  setTask(time-.02, time, new TM_Default(TMT_vec, world, object, Vector_z), OT_sos, {0.,0.,1.}, 1e1);

  //place inside box support
//  setTask(time, time, new TM_StaticStability(world, placeRef, .01), OT_ineq);
  addObjective({time}, make_shared<TM_AboveBox>(world, object, placeRef), OT_ineq, {1e1});

  //connect object to placeRef
#if 0
  Transformation rel = 0;
  rel.pos.set(0, 0, .5*(shapeSize(world, object) + shapeSize(world, placeRef)));
//  setKinematicSwitch(time, true, "transXYPhiZero", placeRef, object, rel );
  addSwitch(time, true, new KinematicSwitch(SW_effJoint, JT_transXYPhi, placeRef, object, world, SWInit_zero, 0, rel));

  addFlag(time, new Flag(FL_clear, world[object]->ID, 0, true));
  addFlag(time, new Flag(FL_zeroQVel, world[object]->ID, 0, true));
#else
  addSwitch_stableOn(time, -1., placeRef, object);
#endif
}

/// place with a specific relative pose -> no effective DOFs!
void KOMO_ext::setPlaceFixed(double time, const char* endeff, const char* object, const char* placeRef, const Transformation& relPose, int verbose) {
  if(verbose>0) cout <<"KOMO_setPlace t=" <<time <<" endeff=" <<endeff <<" obj=" <<object <<" place=" <<placeRef <<endl;

  //connect object to table
  addSwitch(time, true, JT_rigid, SWInit_zero, placeRef, object, relPose);

  if(stepsPerPhase>2) { //velocities down and up
    if(endeff) {
      addObjective({time-.15, time-.10}, make_shared<TM_Default>(TMT_pos, world, endeff), OT_sos, {3e0}, {0., 0., -.1}, 1); //move down
      addObjective({time-.05, time+.05}, make_shared<TM_Default>(TMT_pos, world, endeff), OT_sos, {1e1}, {0., 0., 0.}, 1); //hold still
      addObjective({time+.10, time+.15}, make_shared<TM_Default>(TMT_pos, world, endeff), OT_sos, {3e0}, {0., 0., +.1}, 1); //move up
    }
  }
}

/// switch attachemend (-> ball eDOF)
void KOMO_ext::setHandover(double time, const char* oldHolder, const char* object, const char* newHolder, int verbose) {
#if 1
  setGrasp(time, -1., newHolder, object, verbose, -1., -1.);
#else
  if(verbose>0) cout <<"KOMO_setHandover t=" <<time <<" oldHolder=" <<oldHolder <<" obj=" <<object <<" newHolder=" <<newHolder <<endl;

  //hand center at object center (could be replaced by touch)

  //disconnect object from table
//  setKinematicSwitch(time, true, "delete", oldHolder, object);
  //connect graspRef with object
#if 0
  setKinematicSwitch(time, true, "ballZero", newHolder, object); //why does this sometimes lead to worse motions?
#else
  setKinematicSwitch(time, true, "freeZero", newHolder, object);
  setTask(time, time, new TM_Default(TMT_posDiff, world, newHolder, NoVector, object, NoVector), OT_eq, NoArr, 3e1);
#endif

  if(stepsPerPhase>2) { //velocities: no motion
    setTask(time-.15, time+.15, new TM_Default(TMT_pos, world, object), OT_sos, {0., 0., 0.}, 3e0, 1); // no motion
  }
#endif
}

void KOMO::setPush(double startTime, double endTime, const char* stick, const char* object, const char* table, int verbose) {
  if(verbose>0) cout <<"KOMO_setPush t=" <<startTime <<" stick=" <<stick <<" object=" <<object <<" table=" <<table <<endl;

#if 1
  //stick normal alignes with slider direction
  addObjective({startTime, endTime}, make_shared<TM_Default>(TMT_vecAlign, world, stick, -Vector_y, "slider1b", Vector_x), OT_sos, {1e1}, {1.});
  //stick horizontal is orthogonal to world vertical
//  setTask(startTime, endTime, new TM_Default(TMT_vecAlign, world, stick, Vector_x, nullptr, Vector_z), OT_sos, {0.}, 1e1);
  add_touch(startTime, endTime, stick, table);

  double dist = .05; //.5*shapeSize(world, object, 0)+.01;
  addObjective({startTime, endTime}, make_shared<TM_InsideBox>(world, "slider1b", Vector(dist, .0, .0), stick), OT_ineq);
//  setTask(startTime, endTime, new TM_Default(TMT_posDiff, world, stick, NoVector, "slider1b", {dist, .0, .0}), OT_sos, {}, 1e1);
#else
  setTouch(startTime, endTime, stick, object);
#endif

  setKS_slider(startTime, endTime, true, object, "slider1", table);

  addObjective({startTime, endTime-.1}, make_shared<TM_AboveBox>(world, object, table), OT_ineq, {1e1});

#if 0
  //connect object to placeRef
  Transformation rel = 0;
  rel.pos.set(0, 0, .5*(shapeSize(world, object) + shapeSize(world, table)));
  addSwitch(endTime, true, "transXYPhiZero", table, object, rel);
//  auto *o = addObjective({startTime, endTime}, make_shared<TM_ZeroQVel>(world, object), OT_eq, {3e1}, NoArr, 1, +1);
//  o->prec(-1)=o->prec(-2)=0.;
#endif

  if(stepsPerPhase>2) { //velocities down and up
    addObjective({startTime-.3, startTime-.1}, make_shared<TM_Default>(TMT_pos, world, stick), OT_sos, {3e0}, {0., 0., -.1}, 1); //move down
    addObjective({startTime-.05, startTime-.0}, make_shared<TM_Default>(TMT_pos, world, stick), OT_sos, {3e0}, {0., 0., 0}, 1); //hold still
    addObjective({endTime+.0, endTime+.05}, make_shared<TM_Default>(TMT_pos, world, stick), OT_sos, {3e0}, {0., 0., 0}, 1); //hold still
    addObjective({endTime+.1, endTime+.3}, make_shared<TM_Default>(TMT_pos, world, stick), OT_sos, {3e0}, {0., 0., .1}, 1); // move up
  }
}

void KOMO::setGraspSlide(double time, const char* endeff, const char* object, const char* placeRef, int verbose) {

  double startTime = time;
  double endTime = time+1.;

  if(verbose>0) cout <<"KOMO_setSlide t=" <<startTime <<" endeff=" <<endeff <<" obj=" <<object <<endl;

  //-- grasp part
  //hand upright
//  setTask(startTime, startTime, new TM_Default(TMT_vec, world, endeff, Vector_z), OT_sos, {0.,0.,1.}, 1e-2);

  //disconnect object from table
//  setKinematicSwitch(startTime, true, "delete", placeRef, object);
  //connect graspRef with object
//  setKinematicSwitch(startTime, true, "ballZero", endeff, object);
//  setKinematicSwitch(time, true, "insert_trans3", nullptr, object);
//  setTask(time, time, new TM_InsideBox(world, endeff, NoVector, object), OT_ineq, NoArr, 1e1);

//  addSwitch_stable(startTime, endTime+1., endeff, object);
  addSwitch(startTime, true, JT_free, SWInit_zero, endeff, object);
  addObjective({time, endTime}, make_shared<F_qZeroVel>(world, object), OT_eq, {3e1}, NoArr, 1, +1, -1);
  if(k_order>1) addObjective({time}, make_shared<TM_LinAngVel>(world, object), OT_eq, {1e2}, NoArr, 2, 0);
  else addObjective({time}, make_shared<TM_NoJumpFromParent>(world, object), OT_eq, {1e2}, NoArr, 1, 0, 0);

  add_touch(startTime, startTime, endeff, object);

  //-- place part
  //place inside box support
//  setTask(endTime, endTime, new TM_AboveBox(world, object, placeRef), OT_ineq, NoArr, 1e1);
  add_aboveBox(endTime, endTime, object, placeRef);

  //disconnect object from grasp ref
//  setKinematicSwitch(endTime, true, "delete", endeff, object);

  //connect object to table
//  Transformation rel = 0;
//  rel.pos.set(0,0, h);
//  setKinematicSwitch(endTime, true, "transXYPhiZero", placeRef, object, rel );

  //-- slide constraints!
  //keep height of object above table
//  double h = .5*(shapeSize(world, object) + shapeSize(world, placeRef));
  HALT("TODO: fix syntax:")
//  addObjective(startTime, endTime,
//          make_shared<TM_Default>(TMT_posDiff, world, object, NoVector, placeRef),
//          OT_sos, ARR(h), ~ARR(0,0,1e1));
  //keep object vertial
  addObjective({startTime, endTime},
               make_shared<TM_Default>(TMT_vecDiff, world, object, Vector_z, placeRef, Vector_z), OT_sos, {1e1});

//  if(stepsPerPhase>2){ //velocities down and up
//    setTask(startTime-.15, startTime, new TM_Default(TMT_pos, world, endeff), OT_sos, {0.,0.,-.1}, 3e0, 1); //move down
//    setTask(endTime, endTime+.15, new TM_Default(TMT_pos, world, endeff), OT_sos, {0.,0.,.1}, 3e0, 1); // move up
//  }
}

void KOMO_ext::setSlideAlong(double time, const char* stick, const char* object, const char* wall, int verbose) {
  if(verbose>0) cout <<"KOMO_setSlideAlong t=" <<time <<" obj=" <<object<<" wall=" <<wall <<endl;

  double endTime = time+1.;

  //stick normal alignes with slider direction
  addObjective({time, time+1.}, make_shared<TM_Default>(TMT_vecAlign, world, stick, -Vector_y, object, Vector_x), OT_sos, {1e0}, {1.});
  //stick horizontal is orthogonal to world vertical
  addObjective({time, time+1.}, make_shared<TM_Default>(TMT_vecAlign, world, stick, Vector_x, nullptr, Vector_z), OT_sos, {1e1}, {0.});

  double dist = .5*shapeSize(world, object, 0)+.01;
  addObjective({time, time+1.}, make_shared<TM_InsideBox>(world, object, Vector(dist, .0, .0), stick), OT_ineq);

  add_touch(time, time+1., stick, wall);

  //    //disconnect object from table
  //    setKinematicSwitch(time, true, "delete", nullptr, object);
  //    //connect graspRef with object
  //    setKinematicSwitch(startTime, true, "ballZero", endeff, object);

  Transformation rel = 0;
  rel.rot.setDeg(-90, {1, 0, 0});
  rel.pos.set(0, -.5*(shapeSize(world, wall, 1) - shapeSize(world, object)), +.5*(shapeSize(world, wall, 2) + shapeSize(world, object, 1)));
  addSwitch(time, true, JT_transX, SWInit_zero, wall, object);
  HALT("deprecated")//addSwitch(time, true, new KinematicSwitch(SW_insertEffJoint, JT_transZ, nullptr, object, world, SWInit_zero, 0, rel));
  //    setKinematicSwitch(time, true, "insert_trans3", nullptr, object);
  //    setTask(time, time, new TM_InsideBox(world, endeff, NoVector, object), OT_ineq, NoArr, 1e1);

  if(stepsPerPhase>2) { //velocities down and up
    addObjective({endTime+.0, endTime+.05}, make_shared<TM_Default>(TMT_pos, world, stick), OT_sos, {3e0}, {0., 0., 0}, 1); //hold still
    addObjective({endTime+.1, endTime+.3}, make_shared<TM_Default>(TMT_pos, world, stick), OT_sos, {3e0}, {0., 0., .05}, 1); // move up
  }
}

void KOMO_ext::setDropEdgeFixed(double time, const char* object, const char* to, const Transformation& relFrom, const Transformation& relTo, int verbose) {

  //disconnect object from anything
//  setKinematicSwitch(time, true, "delete", nullptr, object);

  //connect to world with lift
//  setKinematicSwitch(time, true, "JT_trans3", "world", object);

  addSwitch(time, true, JT_hingeX, SWInit_zero, to, object, relFrom, relTo);
//  setKinematicSwitch(time, true, new KinematicSwitch(insertActuated, JT_transZ, nullptr, object, world, 0));
//  setKinematicSwitch(time, true, new KinematicSwitch(SW_insertEffJoint, JT_trans3, nullptr, object, world, 0));

//  if(stepsPerPhase>2){ //velocities down and up
//    setTask(time, time+.2, new TM_Default(TMT_pos, world, object), OT_sos, {0.,0.,-.1}, 3e0, 1); // move down
//  }
}

void KOMO_ext::setAttach(double time, const char* endeff, const char* object1, const char* object2, Transformation& rel, int verbose) {
  if(verbose>0) cout <<"KOMO_setAttach t=" <<time <<" endeff=" <<endeff <<" obj1=" <<object1 <<" obj2=" <<object2 <<endl;

  //hand center at object center (could be replaced by touch)
//  setTask(time, time, new TM_Default(TMT_pos, world, object2, NoVector, object1, NoVector), OT_sos, rel.pos.getArr(), 3e1);
//  setTask(time, time, new TM_Default(TMT_quatDiff, world, object2, NoVector, object1, NoVector), OT_sos, conv_quat2arr(rel.rot), 3e1);

//  setTask(time, time, new TM_Default(TMT_vecAlign, world, newHolder, Vector_y, object, Vector_x), OT_sos, {-1.}, 3e0);

  //disconnect object from grasp ref
//  setKinematicSwitch(time, true, "delete", endeff, object2);

//  Transformation rel = 0;
//  rel.addRelativeTranslation( 0., 0., .5*(shapeSize(world.getFrameByName(object)) + shapeSize(world.getFrameByName(placeRef))));
  addSwitch(time, true, JT_rigid, SWInit_zero, object1, object2, rel);

}

void KOMO::setSlow(double startTime, double endTime, double prec, bool hardConstrained) {
  if(stepsPerPhase>2) { //otherwise: no velocities
#if 1
    uintA selectedBodies;
    for(rai::Frame* f:world.frames) if(f->joint && f->joint->dim>0 && f->joint->dim<7 && f->joint->type!=rai::JT_tau && f->joint->active && f->joint->H>0.) {
        selectedBodies.append(TUP(f->ID, f->parent->ID));
      }
    selectedBodies.reshape(selectedBodies.N/2, 2);
    ptr<Feature> feat = make_shared<F_qItself>(selectedBodies);
#else
    Feature* map = new TM_qItself;
#endif
    if(!hardConstrained) addObjective({startTime, endTime}, feat, OT_sos, {prec}, NoArr, 1);
    else addObjective({startTime, endTime}, feat, OT_eq, {prec}, NoArr, 1);
  }
  //#    _MinSumOfSqr_qItself_vel(MinSumOfSqr qItself){ order=1 time=[0.98 1] scale=3e0 } #slow down
}

void KOMO::setSlowAround(double time, double delta, double prec, bool hardConstrained) {
  setSlow(time-delta, time+delta, prec, hardConstrained);
}

void KOMO_ext::setFine_grasp(double time, const char* endeff, const char* object, double above, double gripSize, const char* gripper, const char* gripper2) {
  double t1=-.25; //time when gripper is positined above
  double t2=-.1;  //time when gripper is lowered
  double t3=-.05; //time when gripper is closed

  //position above
  addObjective({time+t1, 1.}, make_shared<TM_Default>(TMT_vec, world, endeff, Vector_z), OT_sos, {1e0}, {0., 0., 1.});
  addObjective({time+t1, t1}, make_shared<TM_Default>(TMT_posDiff, world, endeff, NoVector, object, NoVector), OT_sos, {3e1}, {0., 0., above+.1});
  addObjective({time+t1, 1.}, make_shared<TM_Default>(TMT_vecAlign, world, endeff, Vector_x, object, Vector_y), OT_sos, {3e0});
  addObjective({time+t1, 1.}, make_shared<TM_Default>(TMT_vecAlign, world, endeff, Vector_x, object, Vector_z), OT_sos, {3e0});
  //open gripper
  if(gripper)  addObjective({time+t1, .85}, make_shared<F_qItself>(F_qItself::byJointNames, StringA({gripper}), world), OT_sos, {gripSize + .05});
  if(gripper2) addObjective({time+t1, .85}, make_shared<F_qItself>(F_qItself::byJointNames, StringA({gripper2}), world), OT_sos, {::asin((gripSize + .05)/(2.*.10))});
  //lower
  addObjective({time+t2, 1.}, make_shared<TM_Default>(TMT_posDiff, world, endeff, NoVector, object, NoVector), OT_sos, {3e1}, {0., 0., above});
  //close gripper
  if(gripper)  addObjective({time+t3, 1.}, make_shared<F_qItself>(F_qItself::byJointNames, StringA({gripper}), world), OT_sos, {gripSize});
  if(gripper2) addObjective({time+t3, 1.}, make_shared<F_qItself>(F_qItself::byJointNames, StringA({gripper2}), world), OT_sos, {::asin((gripSize)/(2.*.10))});
  setSlowAround(time, .05, 3e1);
}

/// translate a list of facts (typically facts in a FOL state) to LGP tasks
/*
void KOMO_ext::setAbstractTask(double phase, const Graph& facts, int verbose) {
//  CHECK_LE(phase, maxPhase,"");
//  listWrite(facts, cout,"\n");  cout <<endl;
  for(Node *n:facts) {
    if(!n->parents.N) continue;
    StringL symbols;
    for(Node *p:n->parents) symbols.append(&p->keys.last());
    double time=1.; //n->get<double>(); //komo tag needs to be double valued!
    if(n->keys.N==1 && n->keys.scalar() == "komo") {
      if(*symbols(0)=="grasp")                      setGrasp(phase+time, *symbols(1), *symbols(2), verbose);
      else if(*symbols(0)=="push")                  setPush(phase+time, phase+time+1., *symbols(1), *symbols(2), *symbols(3), verbose); //TODO: the +1. assumes pushes always have duration 1
      else if(*symbols(0)=="place" && symbols.N==3) setPlace(phase+time, nullptr, *symbols(1), *symbols(2), verbose);
      else if(*symbols(0)=="place" && symbols.N==4) setPlace(phase+time, *symbols(1), *symbols(2), *symbols(3), verbose);
      else if(*symbols(0)=="graspSlide")            setGraspSlide(phase+time, *symbols(1), *symbols(2), *symbols(3), verbose);
      else if(*symbols(0)=="handover")              setHandover(phase+time, *symbols(1), *symbols(2), *symbols(3), verbose);

      //elementary
      else if(*symbols(0)=="flagClear")  {} //           setFlag(phase+time, new Flag(FL_clear, world[*symbols(1)]->ID, 0, true));
      else if(*symbols(0)=="touch")                 add_touch(phase+time, phase+time, *symbols(1), *symbols(2));
      else if(*symbols(0)=="lift")                  setLiftDownUp(phase+time, *symbols(1));
      else if(*symbols(0)=="impulse")               add_impulse(phase+time, *symbols(1), *symbols(2));
      else if(*symbols(0)=="inside")                add_insideBox(phase+time, phase+time, *symbols(1), *symbols(2));
      else if(*symbols(0)=="above")                 add_aboveBox(phase+time, phase+time+.9, *symbols(1), *symbols(2));
      else if(*symbols(0)=="stable")                addSwitch_stable(phase+time, phase+time+1., *symbols(1), *symbols(2));
      else if(*symbols(0)=="stableOn")              addSwitch_stableOn(phase+time, phase+time+1., *symbols(1), *symbols(2));
      else if(*symbols(0)=="dynamic")               addSwitch_dynamic(phase+time, phase+time+1., "base", *symbols(1));
      else if(*symbols(0)=="dynamicTrans")          addSwitch_dynamicTrans(phase+time, phase+time+1., "base", *symbols(1));
      else if(*symbols(0)=="dynamicOn")             addSwitch_dynamicOn(phase+time, phase+time+1., *symbols(1), *symbols(2));

      else if(*symbols(0)=="notAbove") {
        double margin = .05;
        double negMargin = margin + .5*shapeSize(world, *symbols(1), 0); //how much outside the bounding box?
        addObjective(phase+time, phase+time+.9,
                new TM_Max(new TM_AboveBox(world, *symbols(1),*symbols(2), -negMargin), true), //this is the max selection -- only one of the four numbers need to be outside the BB
                OT_ineq, {}, 3e0); //NOTE: usually this is an inequality constraint <0; here we say this should be zero for a negative margin (->outside support)
      } else if(*symbols(0)=="fricSlide") {
        Transformation rel = 0;
        rel.pos.set(0,0, .5*(shapeSize(world, *symbols(1)) + shapeSize(world, *symbols(2))));
        addSwitch(phase+time, false, JT_transXYPhi, SWInit_zero, *symbols(2), *symbols(1), rel);
        addFlag(phase+time, new Flag(FL_clear, world[*symbols(1)]->ID, 0, true), +1);
        addFlag(phase+time, new Flag(FL_impulseExchange, world[*symbols(1)]->ID), 0);
        addFlag(phase+time, new Flag(FL_xPosVelCosts, world[*symbols(1)]->ID, 0, true), +1);
      } else if(*symbols(0)=="dynVert")               {
        addSwitch(phase+time, true, JT_transZ, SWInit_zero, *symbols(2), *symbols(1));
        HALT("deprecated")//addSwitch(phase+time, true, new KinematicSwitch(SW_insertEffJoint, JT_transXY, nullptr, *symbols(1), world));
        addFlag(phase+time, new Flag(FL_clear, world[*symbols(1)]->ID, 0, true), +1);
//        setFlag(phase+time, new Flag(FL_zeroAcc, world[*symbols(1)]->ID, 0, true), +1);
        addFlag(phase+time, new Flag(FL_xPosAccCosts, world[*symbols(1)]->ID, 0, true), +1); //why +1: the kinematic switch triggers 'FixSwitchedObjects' to enforce acc 0 for time slide +0

      } else HALT("UNKNOWN komo symbol: '" <<*symbols(0) <<"'");
    } else if(n->keys.N && n->keys.last().startsWith("komo")) {
      if(n->keys.last()=="komoSlideAlong") setSlideAlong(phase+time, *symbols(0), *symbols(1), *symbols(2), verbose);
      else if(n->keys.last()=="komoDrop") {
        if(symbols.N==2) setDrop(phase+time, *symbols(0), nullptr, *symbols(1), verbose);
        else setDrop(phase+time, *symbols(0), *symbols(1), *symbols(2), verbose);
      } else if(n->keys.last()=="komoThrow") {
//        setInertialMotion(phase+time, phase+time+1., *symbols(0), "base", -.1, 0.);
        addSwitch(phase+time, true, JT_trans3, SWInit_zero, "base", *symbols(0));
        addFlag(phase+time, new Flag(FL_gravityAcc, world[*symbols(0)]->ID, 0, true), +1); //why +1: the kinematic switch triggers 'FixSwitchedObjects' to enforce acc 0 for time slide +0
      } else if(n->keys.last()=="komoHit") {
        setImpact(phase+time, *symbols(0), *symbols(1));
        if(symbols.N==2) {
          addSwitch(phase+time, true, JT_trans3, SWInit_zero, "base", *symbols(1));
          addFlag(phase+time, new Flag(FL_gravityAcc, world[*symbols(1)]->ID, 0, true), +1); //why +1: the kinematic switch triggers 'FixSwitchedObjects' to enforce acc 0 for time slide +0
        } else if(symbols.N==3) {
          //const char* bat = *symbols(0);
          const char* object = *symbols(1);
          const char* placeRef = *symbols(2);
          Transformation rel = 0;
          rel.pos.set(0,0, .5*(shapeSize(world, object) + shapeSize(world, placeRef)));

          addSwitch(phase+time, true, JT_transXYPhi, SWInit_zero, placeRef, object, rel);
          addFlag(phase+time, new Flag(FL_clear, world[object]->ID, 0, true));
          addFlag(phase+time, new Flag(FL_zeroAcc, world[object]->ID, 0, true));

//          setKinematicSwitch(phase+time, false, new KinematicSwitch(SW_actJoint, JT_transXYPhi, placeRef, bat, world, 0, rel));
//          setFlag(phase+time, new Flag(FL_clear, world[bat]->ID, 0, true), +1);
//          setFlag(phase+time, new Flag(FL_xPosVelCosts, world[bat]->ID, 0, true), +1);
        } else NIY;
      } else if(n->keys.last()=="komoAttach") {
        Node *attachableSymbol = facts["attachable"];
        CHECK(attachableSymbol!=nullptr,"");
        Node *attachableFact = facts.getEdge({attachableSymbol, n->parents(1), n->parents(2)});
        Transformation rel = attachableFact->get<Transformation>();
        setAttach(phase+time, *symbols(0), *symbols(1), *symbols(2), rel, verbose);
      } else HALT("UNKNOWN komo TAG: '" <<n->keys.last() <<"'");
    }
  }
}
*/

void KOMO::setSkeleton(const Skeleton& S, bool ignoreSwitches) {
  //-- add objectives for mode switches
  intA switches = getSwitchesFromSkeleton(S);
  if(!ignoreSwitches) {
    for(uint i=0; i<switches.d0; i++) {
      int j = switches(i, 0);
      int k = switches(i, 1);
      const char* newFrom=world.frames.first()->name;
      if(S(k).frames.N==2) newFrom = S(k).frames(0);
      if(j<0)
        addSwitch_mode(SY_initial, S(k).symbol, S(k).phase0, S(k).phase1+1., nullptr, newFrom, S(k).frames.last());
      else
        addSwitch_mode(S(j).symbol, S(k).symbol, S(k).phase0, S(k).phase1+1., S(j).frames(0), newFrom, S(k).frames.last());
    }
  }
  //-- add objectives for rest
  for(const SkeletonEntry& s:S) {
    switch(s.symbol) {
      case SY_none:       HALT("should not be here");  break;
      case SY_initial: case SY_identical: case SY_noCollision:    break;
      case SY_touch:      add_touch(s.phase0, s.phase1, s.frames(0), s.frames(1));  break;
      case SY_above:      add_aboveBox(s.phase0, s.phase1, s.frames(0), s.frames(1));  break;
      case SY_inside:     add_insideBox(s.phase0, s.phase1, s.frames(0), s.frames(1));  break;
//      case SY_inside:     addObjective({s.phase0, s.phase1}, make_shared<TM_InsideLine>(world, s.frames(0), s.frames(1)), OT_ineq, {1e1});  break;
      case SY_oppose:     addObjective({s.phase0, s.phase1}, FS_oppose, s.frames, OT_eq, {1e1});  break;
      case SY_impulse:    HALT("obsolete"); /*add_impulse(s.phase0, s.frames(0), s.frames(1));*/  break;

      case SY_makeFree:   world.makeObjectsFree(s.frames);  break;
      case SY_stableRelPose: addObjective({s.phase0, s.phase1+1.}, FS_poseRel, s.frames, OT_eq, {1e2}, {}, 1);  break;
      case SY_stablePose:  addObjective({s.phase0, s.phase1+1.}, FS_pose, s.frames, OT_eq, {1e2}, {}, 1);  break;
      case SY_poseEq: addObjective({s.phase0, s.phase1}, FS_poseDiff, s.frames, OT_eq, {1e2});  break;

      case SY_liftDownUp: setLiftDownUp(s.phase0, s.frames(0), .4);  break;
      case SY_break:      addObjective({s.phase0, s.phase1}, make_shared<TM_NoJumpFromParent>(world, s.frames(0)), OT_eq, {1e2}, NoArr, 1, 0, 0);  break;

      case SY_contact:    addContact_slide(s.phase0, s.phase1, s.frames(0), s.frames(1));  break;
      case SY_contactStick:    addContact_stick(s.phase0, s.phase1, s.frames(0), s.frames(1));  break;
      case SY_contactComplementary: addContact_ComplementarySlide(s.phase0, s.phase1, s.frames(0), s.frames(1));  break;
      case SY_bounce:     addContact_elasticBounce(s.phase0, s.frames(0), s.frames(1), .9);  break;
      //case SY_contactComplementary:     addContact_Complementary(s.phase0, s.phase1, s.frames(0), s.frames(1));  break;

      case SY_dampMotion: {
        double sqrAccCost=1e-2, sqrVelCost=1e-2;
        if(sqrVelCost>0. && k_order>=1) {
          addObjective({s.phase0, s.phase1}, make_shared<TM_LinAngVel>(world, s.frames(0)), OT_sos, {sqrVelCost}, NoArr, 1);
        }
        if(sqrAccCost>0. && k_order>=2) {
          addObjective({s.phase0, s.phase1}, make_shared<TM_LinAngVel>(world, s.frames(0)), OT_sos, {sqrAccCost}, NoArr, 2);
        }
      } break;
      case SY_alignByInt: {
        addObjective({s.phase0, s.phase1}, FS_scalarProductXX, s.frames, OT_sos);  break;
        cout <<"THE INTEGER IS: " <<s.frames(2) <<endl;
      } break;

      case SY_push:       setPush(s.phase0, s.phase1+1., s.frames(0), s.frames(1), s.frames(2), verbose);  break;//TODO: the +1. assumes pushes always have duration 1
      case SY_graspSlide: setGraspSlide(s.phase0, s.frames(0), s.frames(1), s.frames(2), verbose);  break;
      //    else case SY_handover)              setHandover(s.phase0, s.frames(0), s.frames(1), s.frames(2), verbose);
      //    else LOG(-2) <<"UNKNOWN PREDICATE!: " <<s;

      //switches are handled above now
      case SY_stable:      //if(!ignoreSwitches) addSwitch_stable(s.phase0, s.phase1+1., s.frames(0), s.frames(1));  break;
      case SY_stableOn:    //if(!ignoreSwitches) addSwitch_stableOn(s.phase0, s.phase1+1., s.frames(0), s.frames(1));  break;
      case SY_dynamic:     //if(!ignoreSwitches) addSwitch_dynamic(s.phase0, s.phase1+1., "base", s.frames(0));  break;
      case SY_dynamicOn:   //if(!ignoreSwitches) addSwitch_dynamicOn(s.phase0, s.phase1+1., s.frames(0), s.frames(1));  break;
      case SY_dynamicTrans:   //if(!ignoreSwitches) addSwitch_dynamicTrans(s.phase0, s.phase1+1., "base", s.frames(0));  break;
      case SY_quasiStatic:
      case SY_quasiStaticOn:
      case SY_free:
        break;
      case SY_magic:      addSwitch_magic(s.phase0, s.phase1, world.frames.first()->name, s.frames(0), 0., 0.);  break;
      case SY_magicTrans: addSwitch_magicTrans(s.phase0, s.phase1, world.frames.first()->name, s.frames(0), 0.);  break;
    }
  }
}

void KOMO_ext::setAlign(double startTime, double endTime, const char* shape, const arr& whichAxis, const char* shapeRel, const arr& whichAxisRel, ObjectiveType type, const arr& target, double prec) {
#if 0
  String map;
  map <<"map=vecAlign ref1="<<shape;
  if(whichAxis) map <<" vec1=[" <<whichAxis <<']';
  if(shapeRel) map <<" ref2=" <<shapeRel <<" vec2=" <<;
  if(whichAxisRel) map <<" vec2=[" <<whichAxisRel <<']';
  setTask(startTime, endTime, map, type, target, prec);
#else
  addObjective({startTime, endTime}, make_shared<TM_Default>(TMT_vecAlign, world, shape, Vector(whichAxis), shapeRel, Vector(whichAxisRel)), type, {prec}, target);
#endif

}

void KOMO::add_touch(double startTime, double endTime, const char* shape1, const char* shape2, ObjectiveType type, const arr& target, double prec) {
  addObjective({startTime, endTime}, make_shared<TM_PairCollision>(world, shape1, shape2, TM_PairCollision::_negScalar, false), type, {prec}, target);
}

void KOMO::add_aboveBox(double startTime, double endTime, const char* shape1, const char* shape2, double prec) {
  addObjective({startTime, endTime}, make_shared<TM_AboveBox>(world, shape1, shape2), OT_ineq, {prec}, NoArr);
}

void KOMO::add_insideBox(double startTime, double endTime, const char* shape1, const char* shape2, double prec) {
  addObjective({startTime, endTime}, make_shared<TM_InsideBox>(world, shape1, NoVector, shape2), OT_ineq, {prec}, NoArr);
}

//void KOMO::add_impulse(double time, const char* shape1, const char* shape2, ObjectiveType type, double prec) {
////    setTask(time, time, new TM_ImpulsExchange(world, a, b), OT_sos, {}, 3e1, 2, +1); //+1 deltaStep indicates moved 1 time slot backward (to cover switch)
//  if(k_order>=2) {
//    addObjective({time}, make_shared<TM_ImpulsExchange>(world, shape1, shape2), type, {}, prec, 2, +1, +1); //+1 deltaStep indicates moved 1 time slot backward (to cover switch)
//    addFlag(time, new Flag(FL_impulseExchange, world[shape1]->ID), +0);
//    addFlag(time, new Flag(FL_impulseExchange, world[shape2]->ID), +0);
//  }
//}

void KOMO::add_stable(double time, const char* shape1, const char* shape2, ObjectiveType type, double prec) {
  addObjective({time}, make_shared<TM_Default>(TMT_pose, world, shape1, NoVector, shape2), type, {prec}, NoArr, 1, 0);
}

void KOMO_ext::setAlignedStacking(double time, const char* object, ObjectiveType type, double prec) {
  HALT("obsolete");
//  addObjective({time}, make_shared<TM_AlignStacking>(world, object), type, NoArr, prec);
}

void KOMO::add_collision(bool hardConstraint, double margin, double prec) {
  if(hardConstraint) { //interpreted as hard constraint (default)
    addObjective({}, make_shared<TM_Proxy>(TMT_allP, uintA(), margin), OT_eq, {prec}, NoArr);
  } else { //cost term
    addObjective({}, make_shared<TM_Proxy>(TMT_allP, uintA(), margin), OT_sos, {prec}, NoArr);
  }
}

void KOMO::add_jointLimits(bool hardConstraint, double margin, double prec) {
  if(hardConstraint) { //interpreted as hard constraint (default)
    addObjective({}, make_shared<F_qLimits>(), OT_ineq, {-prec}, NoArr);
  } else { //cost term
    NIY;
//    setTask(0., -1., new TM_Proxy(TMT_allP, {}, margin), OT_sos, NoArr, prec);
  }
}

void KOMO::setLiftDownUp(double time, const char* endeff, double timeToLift) {
  if(stepsPerPhase>2 && timeToLift>0.) { //velocities down and up
    addObjective({time-timeToLift, time-.5*timeToLift}, make_shared<TM_Default>(TMT_pos, world, endeff), OT_sos, {1e0}, {0., 0., -.2}, 1); //move down
//    addObjective({time-timeToLift/3,  time+timeToLift/3}, make_shared<TM_Default>(TMT_pos, world, endeff), OT_sos, {3e0}, {0.,0.,0.}, 1); //move down
    addObjective({time+.5*timeToLift, time+timeToLift}, make_shared<TM_Default>(TMT_pos, world, endeff), OT_sos, {1e0}, {0., 0., .2}, 1); // move up
  }
}

//===========================================================================
//
// config
//

void KOMO_ext::setConfigFromFile() {
  Configuration C(getParameter<String>("KOMO/modelfile"));
//  K.optimizeTree();
  setModel(
    C,
    getParameter<bool>("KOMO/useSwift", true)
  );
  setTiming(
    getParameter<uint>("KOMO/phases"),
    getParameter<uint>("KOMO/stepsPerPhase"),
    getParameter<double>("KOMO/durationPerPhase", 5.),
    getParameter<uint>("KOMO/k_order", 2)
  );
}

void KOMO::setIKOpt() {
  denseOptimization=true;
  stepsPerPhase = 1;
  T = 1;
  tau = 1.;
  k_order = 1;
//  setSquaredQVelocities(0.,-1.,1e-1);
  setSquaredQAccVelHoming(0., -1., 0., 1e-1, 1e-2);
  setSquaredQuaternionNorms();
}

void KOMO::setDiscreteOpt(uint k) {
  denseOptimization=true;
  stepsPerPhase = 1;
  T = k;
  tau = 1.;
  k_order = 1;
  setSquaredQuaternionNorms();
}

void KOMO::setPoseOpt() {
  denseOptimization=true;
  setTiming(1., 2, 5., 1);
  setSquaredQuaternionNorms();
}

void KOMO::setSequenceOpt(double _phases) {
  setTiming(_phases, 2, 5., 1);
  setSquaredQuaternionNorms();
}

void KOMO::setPathOpt(double _phases, uint stepsPerPhase, double timePerPhase) {
  setTiming(_phases, stepsPerPhase, timePerPhase, 2);
  setSquaredQuaternionNorms();
}

void setTasks(KOMO_ext& MP,
              Frame& endeff,
              Frame& target,
              byte whichAxesToAlign,
              uint iterate,
              int timeSteps,
              double duration) {

#if 1
  HALT("deprecated");
#else
  //-- parameters
  double posPrec = getParameter<double>("KOMO/moveTo/precision", 3e1);
  double colPrec = getParameter<double>("KOMO/moveTo/collisionPrecision", -1e0);
  double margin = getParameter<double>("KOMO/moveTo/collisionMargin", .1);
  double zeroVelPrec = getParameter<double>("KOMO/moveTo/finalVelocityZeroPrecision", 3e0);
  double alignPrec = getParameter<double>("KOMO/moveTo/alignPrecision", 3e1);

  //-- set up the KOMO
  target.shape->cont=false; //turn off contact penalization with the target

//  MP.world.swift().initActivations(MP.world);
  //MP.world.watch(false);

  MP.setTiming(1., getParameter<uint>("timeSteps", 50), getParameter<double>("duration", 5.));
  if(timeSteps>=0) MP.setTiming(1., timeSteps, duration);
  if(timeSteps==0) MP.k_order=1;

  Task* t;

  t = MP.addTask("transitions", new TM_Transition(MP.world), OT_sos);
  if(timeSteps!=0) {
    t->map->order=2; //make this an acceleration task!
  } else {
    t->map->order=1; //make this a velocity task!
  }
  t->setCostSpecs(0, MP.T-1, {0.}, 1e0);

  if(timeSteps!=0) {
    t = MP.addTask("final_vel", new TM_qItself(), OT_sos);
    t->map->order=1; //make this a velocity task!
    t->setCostSpecs(MP.T-4, MP.T-1, {0.}, zeroVelPrec);
  }

  if(colPrec<0) { //interpreted as hard constraint (default)
    t = MP.addTask("collisionConstraints", new CollisionConstraint(margin), OT_ineq);
    t->setCostSpecs(0, MP.T-1, {0.}, 1.);
  } else { //cost term
    t = MP.addTask("collision", new TM_Proxy(TMT_allP, {}, margin), OT_sos);
    t->setCostSpecs(0, MP.T-1, {0.}, colPrec);
  }

  t = MP.addTask("endeff_pos", new TM_Default(TMT_pos, endeff.ID, NoVector, target.ID, NoVector), OT_sos);
  t->setCostSpecs(MP.T-1, MP.T-1, {0.}, posPrec);

  for(uint i=0; i<3; i++) if(whichAxesToAlign&(1<<i)) {
      Vector axis;
      axis.setZero();
      axis(i)=1.;
      t = MP.addTask(STRING("endeff_align_"<<i),
                     new TM_Default(TMT_vecAlign, endeff.ID, axis, target.ID, axis),
                     OT_sos);
      t->setCostSpecs(MP.T-1, MP.T-1, {1.}, alignPrec);
    }
#endif
}

void KOMO_ext::setMoveTo(Configuration& world, Frame& endeff, Frame& target, byte whichAxesToAlign) {
//  if(MP) delete MP;
//  MP = new KOMO(world);
  setModel(world);
  this->world.checkConsistency();

  setTasks(*this, endeff, target, whichAxesToAlign, 1, -1, -1.);
  reset();
}

void KOMO::setSpline(uint splineT) {
  Spline S;
  S.setUniformNonperiodicBasis(T-1, splineT, 2);
  uint n=dim_x(0);
  splineB = zeros(S.basis.d0*n, S.basis.d1*n);
  for(uint i=0; i<S.basis.d0; i++) for(uint j=0; j<S.basis.d1; j++)
      splineB.setMatrixBlock(S.basis(i, j)*eye(n, n), i*n, j*n);
  z = pseudoInverse(splineB) * x;
}

void KOMO::reset(double initNoise) {
  if(!configurations.N) setupConfigurations();
  x = getPath_decisionVariable();
  dual.clear();
  featureValues.clear();
  featureJacobians.clear();
  featureTypes.clear();
  komo_problem.clear();
  dense_problem.clear();
  if(initNoise>0.)
    rndGauss(x, initNoise, true); //don't initialize at a singular config
  if(splineB.N) {
    z = pseudoInverse(splineB) * x;
  }
}

void KOMO::setInitialConfigurations(const arr& q){
  if(!configurations.N) setupConfigurations();
  for(uint s=0; s<k_order; s++){
    configurations(s)->setJointState(q);
  }
}

void KOMO::setConfiguration(int t, const arr& q){
  if(!configurations.N) setupConfigurations();
  if(t<0) CHECK_LE(-t, (int)k_order,"");
  configurations(t+k_order)->setJointState(q);
}

void KOMO::initWithConstant(const arr& q) {
  for(uint t=0; t<T; t++) {
    configurations(k_order+t)->setJointState(q);
  }

  reset(0.);
}

void KOMO::initWithWaypoints(const arrA& waypoints, uint waypointStepsPerPhase, bool sineProfile) {
  //compute in which steps (configuration time slices) the waypoints are imposed
  uintA steps(waypoints.N);
  for(uint i=0; i<steps.N; i++) {
    steps(i) = conv_time2step(conv_step2time(i, waypointStepsPerPhase), stepsPerPhase);
  }

  //first set the path piece-wise CONSTANT at waypoints and the subsequent steps (each waypoint may have different dimension!...)
  for(uint i=0; i<steps.N; i++) {
    uint Tstop=T;
    if(i+1<steps.N && steps(i+1)<T) Tstop=steps(i+1);
    for(uint t=steps(i); t<Tstop; t++){
      configurations(k_order+t)->setJointState(waypoints(i));
    }  
  }

  //then interpolate w.r.t. non-switching frames within the intervals
#if 1
  for(uint i=0; i<steps.N; i++) {
    uint i1=steps(i);
    uint i0=0; if(i) i0 = steps(i-1);
    //motion profile
    if(i1<T) {
      for(uint j=i0+1; j<=i1; j++) {
        uintA nonSwitched = getNonSwitchedFrames({configurations(k_order+j), configurations(k_order+i1)});
        arr q0 = configurations(k_order+j)->getJointState(nonSwitched);
        arr q1 = configurations(k_order+i1)->getJointState(nonSwitched);
        arr q;
        double phase = double(j-i0)/double(i1-i0);
        if(sineProfile) {
          q = q0 + (.5*(1.-cos(RAI_PI*phase))) * (q1-q0);
        } else {
          q = q0 + phase * (q1-q0);
        }
        configurations(k_order+j)->setJointState(q, nonSwitched);
      }
    }/*else{
      for(uint j=i0+1;j<T;j++){
        configurations(k_order+j)->setJointState(q0, nonSwitched);
      }
    }*/
  }
#endif

  reset(0.);
}

void KOMO::run(const OptOptions options) {
  Configuration::setJointStateCount=0;
  double timeZero = rai::realTime();
  CHECK(T, "");
  if(logFile)(*logFile) <<"KOMO_run_log: [" <<endl;
  if(opt) delete opt;
  if(denseOptimization) {
    CHECK(!splineB.N, "NIY");
    OptConstrained _opt(x, dual, dense_problem, rai::MAX(verbose-2, 0), options);
//    OptPrimalDual _opt(x, dual, dense_problem, rai::MAX(verbose-2, 0));
    _opt.logFile = logFile;
    _opt.run();
    timeNewton += _opt.newton.timeNewton;
  } else if(sparseOptimization) {
    CHECK(!splineB.N, "NIY");
#if 1
//    ModGraphProblem selG(graph_problem);
//    Conv_Graph_ConstrainedProblem C(selG);
    Conv_Graph_ConstrainedProblem C(graph_problem, logFile);
    OptConstrained _opt(x, dual, C, rai::MAX(verbose-2, 0), options, logFile);
    if(bound_up.N && bound_lo.N){
      _opt.newton.bound_lo = bound_lo;
      _opt.newton.bound_up = bound_up;
    }
//    OptPrimalDual _opt(x, dual, C, rai::MAX(verbose-2, 0), options);
    _opt.run();
    {
//      testing primal dual:
//      rai::wait();
//      OptPrimalDual _opt(x, dual, C, rai::MAX(verbose-2, 0));
//      _opt.run();
    }

    timeNewton += _opt.newton.timeNewton;
#else
    BacktrackingGraphOptimization BGO(graph_problem);
    BGO.evaluate(x);
    BGO.run();
#endif
  } else if(!splineB.N) { //DEFAULT CASE
    Convert C(komo_problem);
    opt = new OptConstrained(x, dual, C, rai::MAX(verbose-2, 0), options);
    if(bound_up.N && bound_lo.N){
      opt->newton.bound_lo = bound_lo;
      opt->newton.bound_up = bound_up;
    }
    opt->logFile = logFile;
    opt->run();
  } else {
    arr a, b, c, d, e;
    Conv_KOMO_ConstrainedProblem P0(komo_problem);
    Conv_linearlyReparameterize_ConstrainedProblem P(P0, splineB);
    opt = new OptConstrained(z, dual, P, rai::MAX(verbose-2, 0));
    opt->logFile = logFile;
    opt->run();
  }
  runTime = rai::realTime() - timeZero;
  if(logFile)(*logFile) <<"\n] #end of KOMO_run_log" <<endl;
  if(verbose>0) {
    cout <<"** optimization time=" <<runTime
         <<" (kin:" <<timeKinematics <<" coll:" <<timeCollisions <<" feat:" <<timeFeatures <<" newton: " <<timeNewton <<")"
         <<" setJointStateCount=" <<Configuration::setJointStateCount <<endl;
  }
  if(verbose>0) cout <<getReport(verbose>1) <<endl;
}

void KOMO::run_sub(const uintA& X, const uintA& Y) {
  Configuration::setJointStateCount=0;
  double timeZero = timerStart();
  if(opt) delete opt;

  {
    GraphProblem_Structure Gstruct(graph_problem);
    //evaluate once with full parameters to adopt initialization
    {
      uintA X;
      X.setStraightPerm(Gstruct.V.N);
      SubGraphProblem G_X(Gstruct, X, {});
      G_X.phi(NoArr, NoArrA, NoArrA, x);
    }

    SubGraphProblem G_XY(Gstruct, X, Y);
    G_XY.optim(rai::MAX(verbose-2, 0));
    sos = G_XY.sos; eq = G_XY.eq; ineq = G_XY.ineq;
  }

  runTime = timerRead(true, timeZero);
  if(verbose>0) {
    cout <<"** optimization time=" <<runTime
         <<" (kin:" <<timeKinematics <<" coll:" <<timeCollisions <<" feat:" <<timeFeatures <<" newton: " <<timeNewton <<")"
         <<" setJointStateCount=" <<Configuration::setJointStateCount <<endl;
  }
  if(verbose>0) cout <<getReport(verbose>1) <<endl;
}

void KOMO::optimize(bool initialize, double initNoise) {
  if(initialize) reset(initNoise);
  CHECK_EQ(configurations.N, T+k_order, "");

  if(verbose>0) reportProblem();

  run();
}

void KOMO_ext::getPhysicsReference(uint subSteps, int display) {
  x.resize(T, world.getJointStateDimension());
  PhysXInterface& px = world.physx();
  px.pushFullState(world.frames);
  for(uint t=0; t<T; t++) {
    for(uint s=0; s<subSteps; s++) {
      px.step(tau/subSteps);
      if(display) px.watch((display<0), STRING("t="<<t<<";"<<s));
    }
    x[t] = world.q;
//      K.calc_fwdPropagateFrames();
//    K.watch();
  }
//  K.watch(true);
  world.setJointState(x[0]);
  if(configurations.N) {
    for(uint s=0; s<k_order; s++) {
      configurations(s)->setJointState(x[0]);
    }
  }
}

void KOMO_ext::playInPhysics(uint subSteps, bool display) {
  arr vels;
  PhysXInterface& px = world.physx();
  for(uint t=0; t<T; t++) {
    NIY; //get the velocity from consequtive frames?
    px.pushFullState(configurations(k_order+t)->frames, NoArr, true);
    for(uint s=0; s<subSteps; s++) {
      if(display) px.watch(false, STRING("t="<<t<<";"<<s));
      world.physx().step(tau/subSteps);
    }
    px.pullDynamicStates(configurations(k_order+t)->frames, vels);
  }
  //  for(uint i=0;i<vels.d0;i++) if(i<world.frames.N) cout <<world.frames(i)->name <<" v=" <<vels[i] <<endl;
}

void KOMO::reportProblem(std::ostream& os) {
  os <<"KOMO Problem:" <<endl;
  os <<"  x-dim:" <<x.N <<"  dual-dim:" <<dual.N <<endl;
  os <<"  T:" <<T <<" k:" <<k_order <<" phases:" <<double(T)/stepsPerPhase <<" stepsPerPhase:" <<stepsPerPhase <<" tau:" <<tau <<endl;
  os <<"  #configurations:" <<configurations.N <<" q-dims: ";
  uintA dims(configurations.N);
  for(uint i=0; i<configurations.N; i++) dims(i)=configurations(i)->q.N;
  writeConsecutiveConstant(os, dims);
  os <<endl;

  if(configurations.N) {
    arr times = getPath_times();
    if(times.N>10) times.resizeCopy(10);
    os <<"    times:" <<times <<endl;
  }

  os <<"  usingSwift:" <<useSwift <<endl;
  for(ptr<Objective>& t:objectives) os <<"    " <<*t <<endl;
  for(KinematicSwitch* sw:switches) {
    os <<"    ";
    if(sw->timeOfApplication+k_order >= configurations.N) {
//      LOG(-1) <<"switch time " <<sw->timeOfApplication <<" is beyond time horizon " <<T;
      sw->write(os, nullptr);
    } else {
      sw->write(os, configurations(sw->timeOfApplication+k_order));
    }
    os <<endl;
  }
//  for(Flag* fl:flags) {
//    os <<"    ";
//    if(fl->stepOfApplication+k_order >= configurations.N) {
//      LOG(-1) <<"flag time " <<fl->stepOfApplication <<" is beyond time horizon " <<T;
//      fl->write(os, nullptr);
//    } else {
//      fl->write(os, configurations(fl->stepOfApplication+k_order));
//    }
//    os <<endl;
//  }
}

void KOMO::checkGradients() {
  CHECK(T, "");
  if(!splineB.N) {
#if 0
    checkJacobianCP(Convert(komo_problem), x, 1e-4);
#else
    double tolerance=1e-4;

    ptr<ConstrainedProblem> CP;

    if(denseOptimization) {
      CP = make_shared<Conv_KOMO_DenseProblem>(*this);
    } else if(sparseOptimization) {
      CP = make_shared<Conv_Graph_ConstrainedProblem>(graph_problem);
    } else { //DEFAULT CASE
      CP = make_shared<Conv_KOMO_ConstrainedProblem>(komo_problem);
    }


    VectorFunction F = [CP](arr& phi, arr& J, const arr& x) {
      return CP->phi(phi, J, NoArr, NoObjectiveTypeA, x);
    };
//    checkJacobian(F, x, tolerance);
    arr J;
    arr JJ=finiteDifferenceJacobian(F, x, J);
    bool succ=true;
    double mmd=0.;
    for(uint i=0; i<J.d0; i++) {
      uint j;
      double md=maxDiff(J[i], JJ[i], &j);
      if(md>mmd) mmd=md;
      if(md>tolerance && md>fabs(J(i, j))*tolerance) {
        if(!denseOptimization) {
          LOG(-1) <<"FAILURE in line " <<i <<" t=" <</*CP_komo.featureTimes(i) <<*/' ' <<komo_problem.featureNames(i) <<" -- max diff=" <<md <<" |"<<J(i, j)<<'-'<<JJ(i, j)<<"| (stored in files z.J_*)";
        } else {
          LOG(-1) <<"FAILURE in line " <<i <<" t=" <</*CP_komo.featureTimes(i) <<' ' <<komo_problem.featureNames(i) <<*/" -- max diff=" <<md <<" |"<<J(i, j)<<'-'<<JJ(i, j)<<"| (stored in files z.J_*)";
        }
        J[i] >>FILE("z.J_analytical");
        JJ[i] >>FILE("z.J_empirical");
        //cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
        //HALT("");
//        return;
        rai::wait();
        succ=false;
      }
    }
    if(succ) cout <<"jacobianCheck -- SUCCESS (max diff error=" <<mmd <<")" <<endl;
#endif
  } else {
    Conv_KOMO_ConstrainedProblem P0(komo_problem);
    Conv_linearlyReparameterize_ConstrainedProblem P1(P0, splineB);
    checkJacobianCP(P1, z, 1e-4);
  }
}

void KOMO::plotTrajectory() {
  ofstream fil("z.trajectories");
  StringA jointNames = world.getJointNames();
  //first line: legend
  for(auto s:jointNames) fil <<s <<' ';
  fil <<endl;

  x.reshape(T, world.getJointStateDimension());
  x.write(fil, nullptr, nullptr, "  ");
  fil.close();

  ofstream fil2("z.trajectories.plt");
  fil2 <<"set key autotitle columnheader" <<endl;
  fil2 <<"set title 'trajectories'" <<endl;
  fil2 <<"set term qt 2" <<endl;
  fil2 <<"plot 'z.trajectories' \\" <<endl;
  for(uint i=1; i<=jointNames.N; i++) fil2 <<(i>1?"  ,''":"     ") <<" u (($0+1)/" <<stepsPerPhase <<"):"<<i<<" w l lw 3 lc " <<i <<" lt " <<1-((i/10)%2) <<" \\" <<endl;
//  if(dual.N) for(uint i=0;i<objectives.N;i++) fil <<"  ,'' u (($0+1)/" <<stepsPerPhase <<"):"<<1+objectives.N+i<<" w l \\" <<endl;
  fil2 <<endl;
  fil2.close();

  gnuplot("load 'z.trajectories.plt'");
}

void KOMO::plotPhaseTrajectory() {
  ofstream fil("z.phase");
  //first line: legend
  fil <<"phase" <<endl;

  arr X = getPath_times();

  X.reshape(T, 1);
  X.write(fil, nullptr, nullptr, "  ");
  fil.close();

  ofstream fil2("z.phase.plt");
  fil2 <<"set key autotitle columnheader" <<endl;
  fil2 <<"set title 'phase'" <<endl;
  fil2 <<"set term qt 2" <<endl;
  fil2 <<"plot 'z.phase' u (($0+1)/" <<stepsPerPhase <<"):1 w l lw 3 lc 1 lt 1" <<endl;
  fil2 <<endl;
  fil2.close();

  gnuplot("load 'z.phase.plt'");
}

struct DrawPaths : GLDrawer {
  arr& X;
  DrawPaths(arr& X): X(X) {}
  void glDraw(OpenGL& gl) {
#ifdef RAI_GL
    glColor(0., 0., 0.);
    for(uint i=0; i<X.d1; i++) {
      glBegin(GL_LINES);
      for(uint t=0; t<X.d0; t++) {
        rai::Transformation pose;
        pose.set(&X(t, i, 0));
//          glTransform(pose);
        glVertex3d(pose.pos.x, pose.pos.y, pose.pos.z);
      }
      glEnd();
    }
#endif
  }
};

bool KOMO::displayTrajectory(double delay, bool watch, bool overlayPaths, const char* saveVideoPath, const char* addText) {
  const char* tag = "KOMO planned trajectory";
  rai::String timetag;
  if(!gl) {
    gl = make_shared<OpenGL>("KOMO display");
    gl->camera.setDefault();
  }

  if(saveVideoPath) {
    rai::system(STRING("mkdir -p " <<saveVideoPath));
    rai::system(STRING("rm -f " <<saveVideoPath <<"*.ppm"));
  }

  uintA allFrames;
  allFrames.setStraightPerm(configurations.first()->frames.N);
  arr X = getPath_frames(allFrames);
  DrawPaths drawX(X);

  for(int t=-(int)k_order; t<(int)T; t++) {
    rai::Configuration& K = *configurations(t+k_order);
    timetag.clear() <<tag <<" (config:" <<t <<'/' <<T <<"  s:" <<conv_step2time(t, stepsPerPhase) <<" tau:" <<K.frames.first()->tau <<')';
    if(addText) timetag <<addText;
//    K.reportProxies();
    K.orsDrawProxies=false;
    gl->clear();
    gl->add(glStandardScene, 0);
    gl->add(K);
    if(overlayPaths) gl->add(drawX);
    if(delay<0.) {
      if(delay<-10.) FILE("z.graph") <<K;
      gl->watch(timetag.p);
    } else {
      gl->update(timetag.p);
      if(delay) rai::wait(delay * K.frames.first()->tau);
    }
    if(saveVideoPath) write_ppm(gl->captureImage, STRING(saveVideoPath<<std::setw(4)<<std::setfill('0')<<t<<".ppm"));
  }
  if(watch) {
    int key = gl->watch(timetag.p);
    return !(key==27 || key=='q');
  }
  gl->clear();
  return false;
}

bool KOMO::displayPath(bool watch, bool full) {
  uintA allFrames;
  allFrames.setStraightPerm(configurations.first()->frames.N);
  arr X = getPath_frames(allFrames);
  CHECK_EQ(X.nd, 3, "");
  CHECK_EQ(X.d2, 7, "");

  DrawPaths drawX(X);

  if(!gl) {
    gl = make_shared<OpenGL>("KOMO display");
    gl->camera.setDefault();
  }
  gl->clear();
  gl->add(glStandardScene, 0);
  if(!full) {
    gl->addDrawer(configurations.last());
    gl->add(drawX);
  } else {
    for(uint t=0; t<T; t++) gl->addDrawer(configurations(k_order+t));
  }
  if(watch) {
    int key = gl->watch();
//    gl.reset();
    gl->clear();
    return !(key==27 || key=='q');
  }
  gl->update(nullptr, true);
//  gl.reset();
  gl->clear();
  return true;
}

Camera& KOMO::displayCamera() {
  if(!gl) {
    gl = make_shared<OpenGL>("KOMO display");
    gl->camera.setDefault();
  }
  return gl->camera;
}

void KOMO::selectJointsBySubtrees(const StringA& roots, const arr& times, bool notThose){
  if(!configurations.N) setupConfigurations();

  world.selectJointsBySubtrees(roots, notThose);

  if(!times.N){
    for(Configuration* C:configurations){
      C->selectJointsBySubtrees(roots, notThose);
      C->ensure_q();
      C->checkConsistency();
    }
  }else{
    int tfrom = conv_time2step(times(0), stepsPerPhase);
    int tto   = conv_time2step(times(1), stepsPerPhase);
    for(uint s=0;s<configurations.N;s++){
      Configuration* C = configurations(s);
      int t = int(s) - k_order;
      if(t<=tfrom || t>tto) C->selectJointsBySubtrees({});
      else C->selectJointsBySubtrees(roots, notThose);
      C->ensure_q();
      C->checkConsistency();
    }
  }
}

//===========================================================================

void KOMO::setupConfigurations(const arr& q_init, const StringA& q_initJoints) {

  //IMPORTANT: The configurations need to include the k prefix configurations!
  //Therefore configurations(0) is for time=-k and configurations(k+t) is for time=t
  CHECK(configurations.N != k_order+T, "why setup again?");

  int xIndexCount=0;
  if(!configurations.N){ //add the initial configuration (with index -k_order )
    computeMeshNormals(world.frames, true);
    computeMeshGraphs(world.frames, true);

    rai::Configuration *C = configurations.append(new Configuration());
    C->copy(world, true);
    C->setTimes(tau);
    for(KinematicSwitch* sw:switches) { //apply potential switches
      if(sw->timeOfApplication+(int)k_order<=0) {
        sw->apply(*C);
      }
    }
    if(useSwift) {
#ifndef FCLmode
      C->stepSwift();
#else
      C->stepFcl();
#endif
    }
    C->ensure_q();
    C->checkConsistency();

    xIndexCount = -k_order*C->getJointStateDimension();
    C->xIndex = xIndexCount;
    xIndexCount += C->getJointStateDimension();
  }

  while(configurations.N<k_order+T) { //add further configurations
    uint s = configurations.N;
    rai::Configuration *C = configurations.append(new Configuration());
    C->copy(*configurations(s-1), true);
    CHECK_EQ(configurations(s), configurations.last(), "");
    C->setTimes(tau);
    if(!!q_init && s>k_order) C->setJointState(q_init, q_initJoints);
    for(KinematicSwitch* sw:switches) { //apply potential switches
      if(sw->timeOfApplication+k_order==s) {
        sw->apply(*C);
      }
    }
    if(useSwift) {
#ifndef FCLmode
      C->stepSwift();
#else
      C->stepFcl();
#endif
    }
    C->ensure_q();
    C->checkConsistency();

    C->xIndex = xIndexCount;
    xIndexCount += C->getJointStateDimension();
  }
}

void KOMO::retrospectAddSwitches(rai::Array<KinematicSwitch*>& _switches){
  for(KinematicSwitch* sw:_switches) {
    uint s = sw->timeOfApplication+k_order;
    rai::Configuration *C = configurations.elem(s);
    rai::Frame *f = sw->apply(*C);
    s++;
    //apply the same switch on all following configurations!
    for(;s<k_order+T;s++){
      rai::Configuration *C1 = configurations.elem(s);
      rai::Frame *f1 = sw->apply(*C1);
      if(f && f1){
        f1->set_Q() = f->get_Q(); //copy the relative pose (switch joint initialization) from the first application
      }
    }
  }
}

//===========================================================================

void KOMO::setupRepresentations() {
  NIY;

#if 0
  setupConfigurations();


  CHECK(!objs.N,"why setup again?");

  uintA Cdims = getKtupleDim(configurations);
  Cdims.prepend(0);

//  uint M=0;
  for(ptr<Objective>& ob:objectives) {
    CHECK_EQ(ob->configs.nd, 2, "in sparse mode, vars need to be tuples of variables");
    for(uint l=0;l<ob->configs.d0;l++) {
      ConfigurationL Ctuple = configurations.sub(convert<uint,int>(ob->configs[l]+(int)k_order));
      uintA cdim = getKtupleDim(Ctuple);
      cdim.prepend(0);

      intA S;
      //query the task map and check dimensionalities of returns
      ob->map->signature(S, Ctuple);
      for(int k:ob->configs[l]) if(k>=0){
        for(int& i:S){
          if(i>=(int)cdim(k) && i<(int)cdim(k+1)) i = i - cdim(k) + Cdims(ob->configs(l,k));
        }
      }
      ptr<GroundedObjective> o = make_shared<GroundedObjective>();
      o->Ctuple = Ctuple;
      o->signature = S;
      o->dim = ob->map->__dim_phi(Ctuple);
      objs.append(o);
    }
  }
#endif
}

void KOMO::setBounds(){
  if(!configurations.N) setupConfigurations();
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");

  arr x = getPath_decisionVariable();
  bound_lo.resize(x.N);
  bound_up.resize(x.N);

  uintA configs;
  configs.setStraightPerm(T); //by default, we loop straight through all configurations

  //-- set the configurations' states
  uint x_count=0;
  arr limits;
  for(uint t:configs) {
    int s = t+k_order;
    uint x_dim = dim_x(t);
    if(x_dim) {
      limits = ~configurations(s)->getLimits();
      bound_lo.setVectorBlock(limits[0], x_count);
      bound_up.setVectorBlock(limits[1], x_count);
      x_count += x_dim;
    }
  }
  CHECK_EQ(x_count, x.N, "");
}

void KOMO::checkBounds(const arr& x){
  CHECK_EQ(x.N, bound_lo.N, "");
  CHECK_EQ(x.N, bound_up.N, "");

  for(uint i=0;i<x.N;i++) if(bound_up.elem(i)>bound_lo.elem(i)){
    if(x.elem(i)<bound_lo.elem(i)) cout <<"lower bound violation: x_" <<i <<"=" <<x.elem(i) <<" lo_" <<i <<"=" <<bound_lo.elem(i) <<endl;
    if(x.elem(i)>bound_up.elem(i)) cout <<"lower upper violation: x_" <<i <<"=" <<x.elem(i) <<" up_" <<i <<"=" <<bound_up.elem(i) <<endl;
  }

}

//===========================================================================

void KOMO::set_x(const arr& x, const uintA& selectedConfigurationsOnly) {
  if(!configurations.N) setupConfigurations();
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");

  uintA configs;
  if(!!selectedConfigurationsOnly) { //we only set some configurations: those listed in selectedConfigurationsOnly
    configs = selectedConfigurationsOnly;
  } else {
    configs.setStraightPerm(T); //by default, we loop straight through all configurations
  }

  //-- set the configurations' states
  uint x_count=0;
  for(uint t:configs) {
    uint s = t+k_order;
    uint x_dim = dim_x(t);
    if(x_dim) {
      rai::timerRead(true);
      if(x.nd==1)  configurations(s)->setJointState(x({x_count, x_count+x_dim-1}));
      else         configurations(s)->setJointState(x[t]);
      timeKinematics += rai::timerRead(true);
      if(useSwift) {
#ifndef FCLmode
        configurations(s)->stepSwift();
#else
        configurations(s)->stepFcl();
#endif
      }
      timeCollisions += rai::timerRead(true);
      x_count += x_dim;
    }
//    configurations(s)->checkConsistency();
  }
  CHECK_EQ(x_count, x.N, "");

  if(animateOptimization>0) {
    if(animateOptimization>1){
      if(animateOptimization>2){
        reportProxies();
        cout <<getReport(true) <<endl;
      }
      displayPath(true);
    }else{
      displayPath(false);
    }
//    komo.plotPhaseTrajectory();
//    rai::wait();
  }

//  reportProxies();
}

#if 0
void KOMO::setState(const arr& x, const uintA& selectedVariablesOnly){
  if(!configurations.N) setupConfigurations();
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");

  if(!!selectedVariablesOnly) { //we only set some configurations: those listed in selectedConfigurationsOnly
    CHECK(selectedVariablesOnly.isSorted(), "");
    uint selCount=0;
    uint vCount=0;
    for(uint t=0;t<T;t++){
      uint s = t+k_order;
      uint n = configurations(s)->vars_getNum();
      for(uint i=0;i<n;i++){
        if(vCount == selectedVariablesOnly(selCount)){
          configurations(s)->vars_activate(i);
          selCount++;
          if(selCount == selectedVariablesOnly.N) break;
        } else {
          configurations(s)->vars_deactivate(i);
        }
        vCount++;
      }
      if(selCount == selectedVariablesOnly.N) break;
    }
  } else {
    for(uint t=0;t<T;t++){
      uint s = t+k_order;
      uint n = configurations(s)->vars_getNum();
      for(uint i=0;i<n;i++){
        configurations(s)->vars_activate(i);
      }
    }
  }

  //-- set the configurations' states
  uint x_count=0;
  for(uint t=0;t<T;t++) {
    uint s = t+k_order;
    uint x_dim = dim_x(t);
    if(x_dim) {
      rai::timerRead(true);
      if(x.nd==1)  configurations(s)->setJointState(x({x_count, x_count+x_dim-1}));
      else         configurations(s)->setJointState(x[t]);
      timeKinematics += rai::timerRead(true);
      if(useSwift) {
#ifndef FCLmode
        configurations(s)->stepSwift();
#else
        configurations(s)->stepFcl();
#endif
        //configurations(s)->proxiesToContacts(1.1);
      }
      timeCollisions += rai::timerRead(true);
      x_count += x_dim;
    }
//    configurations(s)->checkConsistency();
  }
  CHECK_EQ(x_count, x.N, "");

  if(animateOptimization>0) {
    if(animateOptimization>1){
//      if(animateOptimization>2)
//        cout <<getReport(true) <<endl;
      displayPath(true);
    }else{
      displayPath(false);
    }
//    komo.plotPhaseTrajectory();
//    rai::wait();
  }
}
#endif

void KOMO::reportProxies(std::ostream& os, double belowMargin) {
  int s=0;
  for(auto& K:configurations) {
    os <<" **** KOMO PROXY REPORT t=" <<s-(int)k_order <<endl;
    if(K->_state_proxies_isGood){
      K->reportProxies(os, belowMargin);
    }else{
      os <<"  [not evaluated]" <<endl;
    }
    s++;
  }
}

rai::Graph KOMO::getContacts() {
  rai::Graph G;
  int s=0;
  for(auto& K:configurations) {
    for(rai::ForceExchange* con:K->forces) {
      Graph& g = G.newSubgraph();
      g.newNode<int>({"at"}, {}, s-(int)k_order);
      g.newNode<rai::String>({"from"}, {}, con->a.name);
      g.newNode<rai::String>({"to"}, {}, con->b.name);
      g.newNode<arr>({"force"}, {}, con->force);
      g.newNode<arr>({"poa"}, {}, con->position);
    }
    s++;
  }
  return G;
}

struct EffJointInfo {
  rai::Joint* j;
  rai::Transformation Q=0;
  int t, t_start=0, t_end=0;
  double accum=0.;
  EffJointInfo(rai::Joint* j, uint t): j(j), t(t) {}
  void write(ostream& os) const {
    os <<"EffInfo " <<j->frame->parent->name <<"->" <<j->frame->name <<" \t" <<j->type <<" \tt=" <<t_start <<':' <<t_end <<" \tQ=" <<Q;
  }
};
stdOutPipe(EffJointInfo)
bool operator==(const EffJointInfo&, const EffJointInfo&) { return false; }

rai::Array<rai::Transformation> KOMO::reportEffectiveJoints(std::ostream& os) {
  os <<"**** KOMO EFFECTIVE JOINTS" <<endl;
  Graph G;
  std::map<rai::Joint*, Node*> map;
  for(uint s=k_order+1; s<T+k_order; s++) {
    JointL matches = getMatchingJoints({configurations(s-1), configurations(s)}, true);
    for(uint i=0; i<matches.d0; i++) {
      JointL match = matches[i];
      auto* n = new Node_typed<EffJointInfo>(G, {match(1)->frame->name}, {}, EffJointInfo(match(1), s-k_order));
      map[match(1)] = n;
      if(map.find(match(0))==map.end()) map[match(0)] = new Node_typed<EffJointInfo>(G, {match(0)->frame->name}, {}, EffJointInfo(match(0), s-k_order-1));
      Node* other=map[match(0)];
      n->addParent(other);
    }
  }

//  for(uint t=0;t<T+k_order;t++){
//    rai::Configuration *K = configurations(t);
//    for(rai::Frame *f:K->frames){
//      if(f->joint && f->joint->constrainToZeroVel)
//        os <<" t=" <<t-k_order <<'\t' <<f->name <<" \t" <<f->joint->type <<" \tq=" <<f->joint->getQ() <<" \tQ=" <<f->Q <<endl;
//    }
//  }

//  G.displayDot();

  for(Node* n:G) {
    if(!n->parents.N) { //a root node -> accumulate all info
      EffJointInfo& info = n->get<EffJointInfo>();
      info.t_start = info.t_end = info.t;
      info.Q = info.j->frame->get_Q();
      info.accum += 1.;
      Node* c=n;
      for(;;) {
        if(!c->children.N) break;
        c = c->children.scalar();
        EffJointInfo& cinfo = c->get<EffJointInfo>();
        if(info.t_end<cinfo.t) info.t_end=cinfo.t;
        info.Q.rot.add(cinfo.j->frame->get_Q().rot);
        info.Q.pos += cinfo.j->frame->get_Q().pos;
        info.accum += 1.;
//        cout <<" t=" <<cinfo.t <<'\t' <<c->keys <<" \t" <<cinfo.j->type <<" \tq=" <<cinfo.j->getQ() <<" \tQ=" <<cinfo.j->frame->Q <<endl;
      }
      info.Q.pos /= info.accum;
      info.Q.rot.normalize();
      cout <<info <<endl;
    }
  }

  //-- align this with the switches and return the transforms
  uint s=0;
  rai::Array<rai::Transformation> Qs(switches.N);
  for(Node* n:G) {
    if(!n->parents.N) {
      EffJointInfo& info = n->get<EffJointInfo>();
#ifndef RAI_NOCHECK
      rai::KinematicSwitch* sw = switches(s);
      CHECK_EQ(info.t_start, sw->timeOfApplication, "");
      CHECK_EQ(info.j->type, sw->jointType, "");
//      CHECK_EQ(info.j->frame->parent->ID, sw->fromId, "");
//      CHECK_EQ(info.j->frame->ID, sw->toId, "");
#endif

      Qs(s) = info.Q;

      s++;
    }
  }

  cout <<Qs <<endl;

  return Qs;
}

rai::Graph KOMO::getReport(bool gnuplt, int reportFeatures, std::ostream& featuresOs) {
  bool wasRun = featureValues.N!=0;

  //-- collect all task costs and constraints
  StringA name; name.resize(objectives.N);
  arr err=zeros(T, objectives.N);
  arr dualSolution; if(dual.N) dualSolution=zeros(T, objectives.N);
  arr taskC=zeros(objectives.N);
  arr taskG=zeros(objectives.N);
  arr taskH=zeros(objectives.N);
  arr taskF=zeros(objectives.N);
  uint M=0;
  if(!denseOptimization && !sparseOptimization) {
    for(uint t=0; t<T; t++) {
      for(uint i=0; i<objectives.N; i++) {
        ptr<Objective> task = objectives(i);
        if(task->isActive(t)) {
          uint d=0;
          if(wasRun) {
            d=task->map->__dim_phi(configurations({t, t+k_order}));
            for(uint j=0; j<d; j++) CHECK_EQ(featureTypes(M+j), task->type, "");
            if(d) {
              if(task->type==OT_sos) {
                for(uint j=0; j<d; j++) err(t, i) += sqr(featureValues(M+j)); //sumOfSqr(phi.sub(M,M+d-1));
                if(dual.N) dualSolution(t, i) = dual(M);
                taskC(i) += err(t, i);
              }
              if(task->type==OT_ineq) {
                for(uint j=0; j<d; j++) err(t, i) += MAX(0., featureValues(M+j));
                if(dual.N) dualSolution(t, i) = dual(M);
                taskG(i) += err(t, i);
              }
              if(task->type==OT_eq) {
                for(uint j=0; j<d; j++) err(t, i) += fabs(featureValues(M+j));
                if(dual.N) dualSolution(t, i) = dual(M);
                taskH(i) += err(t, i);
              }
              if(task->type==OT_f) {
                for(uint j=0; j<d; j++) err(t, i) += featureValues(M+j);
                if(dual.N) dualSolution(t, i) = dual(M);
                taskF(i) += err(t, i);
              }
              M += d;
            }
          }
          if(reportFeatures==1) {
            featuresOs <<std::setw(4) <<t <<' ' <<std::setw(2) <<i <<' ' <<std::setw(2) <<d
                       <<' ' <<std::setw(40) <<task->name
                       <<" k=" <<task->map->order <<" ot=" <<task->type <<" prec=" <<std::setw(4) <<task->map->scale;
            if(task->map->target.N<5) featuresOs <<" y*=[" <<task->map->target <<']'; else featuresOs<<"y*=[..]";
            featuresOs <<" y^2=" <<err(t, i) <<endl;
          }
        }
      }
    }
  } else { //featureDense=true
    for(uint i=0; i<objectives.N; i++) {
      ptr<Objective> ob = objectives.elem(i);
      for(uint l=0; l<ob->configs.d0; l++) {
        ConfigurationL Ktuple = configurations.sub(convert<uint, int>(ob->configs[l]+(int)k_order));
        uint d=0;
        uint time=ob->configs(l, -1);
        if(wasRun) {
          d=ob->map->__dim_phi(Ktuple);
          for(uint j=0; j<d; j++) CHECK_EQ(featureTypes(M+j), ob->type, "");
          if(d) {
            if(ob->type==OT_sos) {
              for(uint j=0; j<d; j++) err(time, i) += sqr(featureValues(M+j));
              taskC(i) += err(time, i);
            }
            if(ob->type==OT_ineq) {
              for(uint j=0; j<d; j++) err(time, i) += MAX(0., featureValues(M+j));
              taskG(i) += err(time, i);
            }
            if(ob->type==OT_eq) {
              for(uint j=0; j<d; j++) err(time, i) += fabs(featureValues(M+j));
              taskH(i) += err(time, i);
            }
            if(ob->type==OT_eq) {
              for(uint j=0; j<d; j++) err(time, i) += featureValues(M+j);
              taskF(i) += err(time, i);
            }
            M += d;
          }
        }
        if(reportFeatures==1) {
          featuresOs <<std::setw(4) <<time <<' ' <<std::setw(2) <<i <<' ' <<std::setw(2) <<d <<ob->configs[l]
                     <<' ' <<std::setw(40) <<ob->name
                     <<" k=" <<ob->map->order <<" ot=" <<ob->type <<" prec=" <<std::setw(4) <<ob->map->scale;
          if(ob->map->target.N<5) featuresOs <<" y*=[" <<ob->map->target <<']'; else featuresOs<<"y*=[..]";
          featuresOs <<" y^2=" <<err(time, i) <<endl;
        }
      }
    }
  }
  CHECK_EQ(M, featureValues.N, "");

  //-- generate a report graph
  rai::Graph report;
  double totalC=0., totalG=0., totalH=0., totalF=0.;
  for(uint i=0; i<objectives.N; i++) {
    ptr<Objective> c = objectives(i);
    Graph& g = report.newSubgraph({c->name}, {});
    g.newNode<double>({"order"}, {}, c->map->order);
    g.newNode<String>({"type"}, {}, STRING(c->type.name()));
    if(taskC(i)) g.newNode<double>("sos", {}, taskC(i));
    if(taskG(i)) g.newNode<double>("ineq", {}, taskG(i));
    if(taskH(i)) g.newNode<double>("eq", {}, taskH(i));
    if(taskF(i)) g.newNode<double>("f", {}, taskF(i));
    totalC += taskC(i);
    totalG += taskG(i);
    totalH += taskH(i);
    totalF += taskF(i);
  }
  report.newNode<double>("sos", {}, totalC);
  report.newNode<double>("ineq", {}, totalG);
  report.newNode<double>("eq", {}, totalH);
  report.newNode<double>("f", {}, totalF);

  if(gnuplt) {
    //-- write a nice gnuplot file
    ofstream fil("z.costReport");
    //first line: legend
    for(auto c:objectives) fil <<c->name <<' ';
    for(auto c:objectives) if(c->type==OT_ineq && dualSolution.N) fil <<c->name <<"_dual ";
    fil <<endl;

    //rest: just the matrix
    if(true) { // && !dualSolution.N) {
      err.write(fil, nullptr, nullptr, "  ");
    } else {
      dualSolution.reshape(T, dualSolution.N/(T));
      catCol(err, dualSolution).write(fil, nullptr, nullptr, "  ");
    }
    fil.close();

    ofstream fil2("z.costReport.plt");
    fil2 <<"set key autotitle columnheader" <<endl;
    fil2 <<"set title 'costReport ( plotting sqrt(costs) )'" <<endl;
    fil2 <<"plot 'z.costReport' \\" <<endl;
    for(uint i=1; i<=objectives.N; i++) fil2 <<(i>1?"  ,''":"     ") <<" u (($0+1)/" <<stepsPerPhase <<"):"<<i<<" w l lw 3 lc " <<i <<" lt " <<1-((i/10)%2) <<" \\" <<endl;
    if(dualSolution.N) for(uint i=0; i<objectives.N; i++) fil2 <<"  ,'' u (($0+1)/" <<stepsPerPhase <<"):"<<1+objectives.N+i<<" w l \\" <<endl;
    fil2 <<endl;
    fil2.close();

    if(gnuplt) {
//      cout <<"KOMO Report\n" <<report <<endl;
      gnuplot("load 'z.costReport.plt'");
    }
  }

  return report;
}

/// output the defined problem as a generic graph, that can also be displayed, saved and loaded
rai::Graph KOMO::getProblemGraph(bool includeValues, bool includeSolution) {
  rai::Graph K;
  //header
#if 1
  Graph& g = K.newSubgraph({"KOMO_specs"});
  g.newNode<uint>({"x_dim"}, {}, x.N);
  g.newNode<uint>({"T"}, {}, T);
  g.newNode<uint>({"k_order"}, {}, k_order);
  g.newNode<double>({"tau"}, {}, tau);
  //  uintA dims(configurations.N);
  //  for(uint i=0; i<configurations.N; i++) dims(i)=configurations(i)->q.N;
  //  g.newNode<uintA>({"q_dims"}, {}, dims);
  //  arr times(configurations.N);
  //  for(uint i=0; i<configurations.N; i++) times(i)=configurations(i)->frames.first()->time;
  //  g.newNode<double>({"times"}, {}, times);
  g.newNode<bool>({"useSwift"}, {}, useSwift);
#endif

  if(includeSolution) {
    //full configuration paths
    g.newNode<arr>({"X"}, {}, getPath_frames());
    g.newNode<arrA>({"x"}, {}, getPath_q());
    g.newNode<arr>({"dual"}, {}, dual);
  }

  //objectives
  for(ptr<Objective>& ob : objectives) {
    Graph& g = K.newSubgraph({ob->name});
    g.newNode<double>({"order"}, {}, ob->map->order);
    g.newNode<String>({"type"}, {}, STRING(ob->type));
    g.newNode<String>({"feature"}, {}, STRING(ob->name));
    if(ob->configs.N) g.newNode<intA>({"vars"}, {}, ob->configs);
//    g.copy(task->map->getSpec(world), true);
    if(includeValues) {
      arr y, Jy;
      arrA V, J;
      if(!denseOptimization && !sparseOptimization) {
        for(uint t=0; t<ob->configs.N && t<T; t++) if(ob->isActive(t)) {
            ob->map->__phi(y, Jy, configurations({t, t+k_order}));
            if(isSpecial(Jy)) Jy = unpack(Jy);

            V.append(y);
            J.append(Jy);
          }
      } else {
        for(uint l=0; l<ob->configs.d0; l++) {
          ConfigurationL Ktuple = configurations.sub(convert<uint, int>(ob->configs[l]+(int)k_order));
          ob->map->__phi(y, Jy, Ktuple);
          if(isSpecial(Jy)) Jy = unpack(Jy);

          V.append(y);
          J.append(Jy);
        }
      }
      g.newNode<arrA>({"y"}, {}, V);
      g.newNode<arrA>({"J"}, {}, J);

      arr Vflat = cat(V);

      if(ob->type==OT_sos) {
        g.newNode<double>({"sos_sumOfSqr"}, {}, sumOfSqr(Vflat));
      } else if(ob->type==OT_eq) {
        g.newNode<double>({"eq_sumOfAbs"}, {}, sumOfAbs(Vflat));
      } else if(ob->type==OT_ineq) {
        double c=0.;
        for(double& v:Vflat) if(v>0) c+=v;
        g.newNode<double>({"inEq_sumOfPos"}, {}, c);
      }
    }
  }

  return K;
}

double KOMO::getConstraintViolations() {
  Graph R = getReport(false);
  return R.get<double>("ineq_sumOfPos") + R.get<double>("eq_sumOfAbs");
}

double KOMO::getCosts() {
  Graph R = getReport(false);
  return R.get<double>("sos_sumOfSqr");
}

void KOMO::Conv_KOMO_KOMOProblem::getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes) {
  CHECK_EQ(komo.configurations.N, komo.k_order+komo.T, "configurations are not setup yet: use komo.reset()");
  if(!!variableDimensions) {
    variableDimensions.resize(komo.T);
    for(uint t=0; t<komo.T; t++) variableDimensions(t) = komo.configurations(t+komo.k_order)->getJointStateDimension();
  }

  if(!!featureTimes) featureTimes.clear();
  if(!!featureTypes) featureTypes.clear();
  featureNames.clear();
  uint M=0;
  phiIndex.resize(komo.T, komo.objectives.N); phiIndex.setZero();
  phiDim.resize(komo.T, komo.objectives.N);   phiDim.setZero();
  for(uint t=0; t<komo.T; t++) {
    for(uint i=0; i<komo.objectives.N; i++) {
      ptr<Objective> task = komo.objectives.elem(i);
      if(task->isActive(t)) {
        uint m = task->map->__dim_phi(komo.configurations({t, t+komo.k_order})); //dimensionality of this task

        if(!!featureTimes) featureTimes.append(t, m); //consts<uint>(t, m));
        if(!!featureTypes) featureTypes.append(task->type, m); //consts<ObjectiveType>(task->type, m));
        for(uint j=0; j<m; j++)  featureNames.append(STRING(task->name <<'_'<<j));

        //store indexing phi <-> tasks
        phiIndex(t, i) = M;
        phiDim(t, i) = m;
        M += m;
      }
    }
  }
  dimPhi = M;
  CHECK_EQ(M, sum(phiDim), "");
}

bool WARN_FIRST_TIME=true;

void KOMO::Conv_KOMO_KOMOProblem::phi(arr& phi, arrA& J, arrA& H, uintA& featureTimes, ObjectiveTypeA& tt, const arr& x) {
  const uintA prevPhiIndex=phiIndex, prevPhiDim=phiDim;

  //-- set the trajectory
  komo.set_x(x);

  CHECK(dimPhi, "getStructure must be called first");
  //  getStructure(NoUintA, featureTimes, tt);
  //  if(WARN_FIRST_TIME){ LOG(-1)<<"calling inefficient getStructure"; WARN_FIRST_TIME=false; }
  phi.resize(dimPhi);
  if(!!tt) tt.resize(dimPhi);
  if(!!J) J.resize(dimPhi);

  arr y, Jy;
  uint M=0;
  for(uint t=0; t<komo.T; t++) {
    //build the Ktuple with order given by map
    ConfigurationL Ktuple = komo.configurations({t, t+komo.k_order});
    uintA Ktuple_dim = getKtupleDim(Ktuple);

    for(uint i=0; i<komo.objectives.N; i++) {
      ptr<Objective> task = komo.objectives.elem(i);
      if(task->isActive(t)) {
        //query the task map and check dimensionalities of returns
        task->map->__phi(y, (!!J?Jy:NoArr), Ktuple);
//        uint m = task->map->__dim_phi(Ktuple);
//        CHECK_EQ(m,y.N,"");
        if(!!J) CHECK_EQ(y.N, Jy.d0, "");
        if(!!J) CHECK_EQ(Jy.nd, 2, "");
        if(!!J) CHECK_EQ(Jy.d1, Ktuple_dim.last(), "");
        if(!y.N) continue;
        if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

        //write into phi and J
        phi.setVectorBlock(y, M);
        if(!!J) {
          if(isSpecial(Jy)) Jy = unpack(Jy);
//          Jy *= task->prec(t);
          if(t<komo.k_order) Jy.delColumns(0, Ktuple_dim(komo.k_order-t-1)); //delete the columns that correspond to the prefix!!
//          if(t<komo.k_order) Jy.delColumns(0,(komo.k_order-t)*komo.configurations(0)->q.N); //delete the columns that correspond to the prefix!!
          for(uint i=0; i<y.N; i++) J(M+i) = Jy[i]; //copy it to J(M+i); which is the Jacobian of the M+i'th feature w.r.t. its variables
        }
        if(!!tt) for(uint i=0; i<y.N; i++) tt(M+i) = task->type;

//        //store indexing phi <-> tasks
//        phiIndex(t, i) = M;
//        phiDim(t, i) = y.N;

        //counter for features phi
        M += y.N;
      }
    }
  }

  CHECK_EQ(M, dimPhi, "");
//  if(!!lambda) CHECK_EQ(prevLambda, lambda, ""); //this ASSERT only holds is none of the tasks is variable dim!
  komo.featureValues = phi;
  if(!!J) komo.featureJacobians = J;
  if(!!tt) komo.featureTypes = tt;
}

void KOMO::Conv_KOMO_DenseProblem::phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x) {
  //-- set the trajectory
  komo.set_x(x);

  if(!dimPhi) getDimPhi();
//  CHECK(dimPhi,"getStructure must be called first");
//  getStructure(NoUintA, featureTimes, tt);
//  if(WARN_FIRST_TIME){ LOG(-1)<<"calling inefficient getStructure"; WARN_FIRST_TIME=false; }
  phi.resize(dimPhi);
  if(!!tt) tt.resize(dimPhi);
  if(!!J){
    bool SPARSE_JACOBIANS = true;
    if(!SPARSE_JACOBIANS) {
      J.resize(dimPhi, x.N).setZero();
    } else {
      J.sparse().resize(dimPhi, x.N, 0);
    }
  }

  uintA x_index = getKtupleDim(komo.configurations({komo.k_order, -1}));
  x_index.prepend(0);

  arr y, Jy;
  uint M=0;
  for(uint i=0; i<komo.objectives.N; i++) {
    ptr<Objective> ob = komo.objectives.elem(i);
    for(uint l=0; l<ob->configs.d0; l++) {
      ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      uintA kdim = getKtupleDim(Ktuple);
      kdim.prepend(0);

      //query the task map and check dimensionalities of returns
      ob->map->__phi(y, (!!J?Jy:NoArr), Ktuple);
      if(!!J) CHECK_EQ(y.N, Jy.d0, "");
      if(!!J) CHECK_EQ(Jy.nd, 2, "");
      if(!!J) CHECK_EQ(Jy.d1, kdim.last(), "");
      if(!y.N) continue;
      if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

      //write into phi and J
      phi.setVectorBlock(y, M);

      if(!!J) {
        if(isSpecial(Jy) && ob->configs.d1!=1) Jy = unpack(Jy); //
        if(!isSpecial(Jy)){
          for(uint j=0; j<ob->configs.d1; j++) {
            if(ob->configs(l, j)>=0) {
              J.setMatrixBlock(Jy.sub(0, -1, kdim(j), kdim(j+1)-1), M, x_index(ob->configs(l, j)));
            }
          }
        }else{
          uint j=0;
          if(ob->configs(l, j)>=0) {
            Jy.sparse().reshape(J.d0, J.d1);
            Jy.sparse().colShift(M);
            Jy.sparse().rowShift(x_index(ob->configs(l, j)));
            J += Jy;
          }
        }
      }

      if(!!tt) for(uint i=0; i<y.N; i++) tt(M+i) = ob->type;

      //counter for features phi
      M += y.N;
    }
  }

  CHECK_EQ(M, dimPhi, "");
  komo.featureValues = phi;
  if(!!J) komo.featureJacobians.resize(1).scalar() = J;
  if(!!tt) komo.featureTypes = tt;

  if(quadraticPotentialLinear.N){
      tt.append(OT_f);
      phi.append( (~x * quadraticPotentialHessian * x).scalar() + scalarProduct(quadraticPotentialLinear, x));
      J.append(quadraticPotentialLinear);
      H = quadraticPotentialHessian;
  }
}

void KOMO::Conv_KOMO_DenseProblem::getDimPhi() {
  CHECK_EQ(komo.configurations.N, komo.k_order+komo.T, "configurations are not setup yet: use komo.reset()");
  uint M=0;
  for(uint i=0; i<komo.objectives.N; i++) {
    ptr<Objective> ob = komo.objectives.elem(i);
    for(uint l=0; l<ob->configs.d0; l++) {
      ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      M += ob->map->__dim_phi(Ktuple); //dimensionality of this task
    }
  }
  dimPhi = M;
}

void KOMO::Conv_KOMO_GraphProblem::getStructure(uintA& variableDimensions, intAA& featureVariables, ObjectiveTypeA& featureTypes) {
  CHECK_EQ(komo.configurations.N, komo.k_order+komo.T, "configurations are not setup yet: use komo.reset()");
  if(!!variableDimensions) {
#if 1
    variableDimensions.resize(komo.T);
    for(uint t=0; t<komo.T; t++) variableDimensions(t) = komo.configurations(t+komo.k_order)->getJointStateDimension();
#else
    variableDimensions.clear();
    for(uint t=0; t<komo.T; t++){
      uint n=komo.configurations(t+komo.k_order)->vars_getNum();
      for(uint i=0;i<n;i++){
        variableDimensions.append() = komo.configurations(t+komo.k_order)->vars_getDim(i);
      }
    }
#endif
  }

  if(!!featureVariables) featureVariables.clear();
  if(!!featureTypes) featureTypes.clear();
  uint M=0;
  for(ptr<Objective>& ob:komo.objectives) {
    CHECK_EQ(ob->configs.nd, 2, "in sparse mode, vars need to be tuples of variables");
    for(uint l=0; l<ob->configs.d0; l++) {
      ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      uint m = ob->map->__dim_phi(Ktuple); //dimensionality of this task
      if(!!featureVariables) featureVariables.append(ob->configs[l], m);
      if(!!featureTypes) featureTypes.append(ob->type, m);
      M += m;
    }
  }

  if(!!featureTypes) komo.featureTypes = featureTypes;

  dimPhi = M;
}

void KOMO::Conv_KOMO_GraphProblem::getSemantics(StringA& varNames, StringA& phiNames) {
#if 1
  varNames.resize(komo.T);
  for(uint t=0; t<komo.T; t++) varNames(t) <<"config_" <<t;
#else
  varNames.clear();
  for(uint t=0; t<komo.T; t++){
    uint n=komo.configurations(t+komo.k_order)->vars_getNum();
    for(uint i=0;i<n;i++){
      varNames.append( STRING("config_" <<t <<"_var_" <<i));
    }
  }
#endif

  phiNames.clear();
  uint M=0;
  for(ptr<Objective>& ob:komo.objectives) {
    CHECK_EQ(ob->configs.nd, 2, "in sparse mode, vars need to be tuples of variables");
    for(uint l=0; l<ob->configs.d0; l++) {
      ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      uint m = ob->map->__dim_phi(Ktuple); //dimensionality of this task
      phiNames.append(ob->name, m);
      M += m;
    }
  }
}

void KOMO::Conv_KOMO_GraphProblem::phi(arr& phi, arrA& J, arrA& H, const arr& x) {
  //-- set the trajectory
  komo.set_x(x);

//  if(!dimPhi) getStructure();
  CHECK(dimPhi, "getStructure must be called first");
  phi.resize(dimPhi);
  if(!!J) J.resize(dimPhi);

  rai::timerStart();
  arr y, Jy;
//  Jy.sparse();
  uint M=0;
  for(ptr<Objective>& ob:komo.objectives) {
    CHECK_EQ(ob->configs.nd, 2, "in sparse mode, vars need to be tuples of variables");
    for(uint l=0; l<ob->configs.d0; l++) {
      ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      uintA kdim = getKtupleDim(Ktuple);
      kdim.prepend(0);

      //query the task map and check dimensionalities of returns
      ob->map->__phi(y, (!!J?Jy:NoArr), Ktuple);
      if(!!J) CHECK_EQ(y.N, Jy.d0, "");
      if(!!J) CHECK_EQ(Jy.nd, 2, "");
      if(!!J) CHECK_EQ(Jy.d1, kdim.last(), "");
      if(!y.N) continue;
      if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

//      if(!!Jy) CHECK(isSparseMatrix(J), "");

      //write into phi and J
      phi.setVectorBlock(y, M);

      if(!!J) {
        for(uint j=ob->configs.d1; j--;) {
          if(ob->configs(l, j)<0) {
            if(isSpecial(Jy)) Jy = unpack(Jy);
            Jy.delColumns(kdim(j), kdim(j+1)-kdim(j)); //delete the columns that correspond to the prefix!!
          }
        }
        if(!isSparseMatrix(Jy)) {
          for(uint i=0; i<y.N; i++) J(M+i) = Jy[i];
        } else {
          Jy.sparse().setupRowsCols();
          for(uint i=0; i<y.N; i++) J(M+i) = Jy.sparse().getSparseRow(i);
        }
      }

      //counter for features phi
      M += y.N;
    }
  }
  komo.timeFeatures += rai::timerRead(true);

  CHECK_EQ(M, dimPhi, "");
  komo.featureValues = phi;
}

void KOMO::Conv_KOMO_GraphProblem::setPartialX(const uintA& whichX, const arr& x) {
  komo.set_x(x, whichX);
}

void KOMO::Conv_KOMO_GraphProblem::getPartialPhi(arr& phi, arrA& J, arrA& H, const uintA& whichPhi) {
  //NON EFFICIENT

  {
    //copy and past from full phi!
    CHECK(dimPhi, "getStructure must be called first");
    if(!!phi) phi.resize(dimPhi);
    if(!!J) J.resize(dimPhi);

//    uintA x_index = getKtupleDim(komo.configurations({komo.k_order,-1}));
//    x_index.prepend(0);

    arr y, Jy;
    uint M=0;
    for(ptr<Objective>& ob:komo.objectives) {
      CHECK_EQ(ob->configs.nd, 2, "in sparse mode, vars need to be tuples of variables");
      for(uint l=0; l<ob->configs.d0; l++) {
        ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
        uintA kdim = getKtupleDim(Ktuple);
        kdim.prepend(0);

        //query the task map and check dimensionalities of returns
        ob->map->__phi(y, (!!J?Jy:NoArr), Ktuple);
        if(!!J && isSpecial(Jy)) Jy = unpack(Jy);

        if(!!J) CHECK_EQ(y.N, Jy.d0, "");
        if(!!J) CHECK_EQ(Jy.nd, 2, "");
        if(!!J) CHECK_EQ(Jy.d1, kdim.last(), "");
        if(!y.N) continue;
        if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

        //write into phi and J
        if(!!phi) phi.setVectorBlock(y, M);

        if(!!J) {
          for(uint j=ob->configs.d1; j--;) {
            if(ob->configs(l, j)<0) {
              Jy.delColumns(kdim(j), kdim(j+1)-kdim(j)); //delete the columns that correspond to the prefix!!
            }
          }
          for(uint i=0; i<y.N; i++) J(M+i) = Jy[i];
        }

        //counter for features phi
        M += y.N;
      }
    }

    CHECK_EQ(M, dimPhi, "");
    //  if(!!lambda) CHECK_EQ(prevLambda, lambda, ""); //this ASSERT only holds is none of the tasks is variable dim!
    if(!!phi) komo.featureValues = phi;
  }

  //now subselect features
  if(!!phi) phi = phi.sub(whichPhi);
  if(!!J) J = J.sub(whichPhi);
}

void KOMO::Conv_KOMO_MathematicalProgram::createIndices(){
  if(!komo.configurations.N) komo.setupConfigurations();
  CHECK_EQ(komo.configurations.N, komo.k_order+komo.T, "configurations are not setup yet");

  //count variables
  uint numVars=0;
  uint xDim=0;
  for(uint t=0; t<komo.T; t++){
    int s = t+komo.k_order;
    komo.configurations(s)->ensure_indexedJoints();
    numVars += komo.configurations(s)->activeJoints.N;
    numVars += komo.configurations(s)->forces.N;
    xDim += komo.configurations(s)->getJointStateDimension();
  }

  //create variable index
  xIndex2VarId.resize(xDim);
  variableIndex.resize(numVars);
  uint var=0;
  uint idx=0;
  for(uint t=0; t<komo.T; t++){
    int s = t+komo.k_order;
    for(rai::Joint *j:komo.configurations(s)->activeJoints){
      CHECK_EQ(idx, j->qIndex + j->frame->C.xIndex, "mismatch index counting");
      VariableIndexEntry& V = variableIndex(var);
      V.joint = j;
      V.dim = j->qDim();
      V.xIndex = idx;
      for(uint i=0;i<j->qDim();i++) xIndex2VarId(idx++) = var;
      var++;
    }
    for(rai::ForceExchange *c:komo.configurations(s)->forces){
      CHECK_EQ(idx, c->qIndex + c->a.C.xIndex, "mismatch index counting");
      VariableIndexEntry& V = variableIndex(var);
      V.force = c;
      V.dim = c->qDim();
      V.xIndex = idx;
      for(uint i=0;i<c->qDim();i++) xIndex2VarId(idx++) = var;
      var++;
    }
  }
  CHECK_EQ(var, numVars, "");
  CHECK_EQ(idx, xDim, "");

  //count features
  uint F=0;
  for(ptr<Objective>& ob:komo.objectives) {
    CHECK_EQ(ob->configs.nd, 2, "in sparse mode, vars need to be tuples of variables");
    F += ob->configs.d0;
  }

  featureIndex.resize(F);

  //create feature index
  uint f=0;
  uint fDim = 0;
  for(ptr<Objective>& ob:komo.objectives) {
    for(uint l=0; l<ob->configs.d0; l++) {
      FeatureIndexEntry& F = featureIndex(f);
      F.ob = ob;
      F.Ctuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      F.dim = ob->map->__dim_phi(F.Ctuple); //dimensionality of this task
      fDim += F.dim;
      f++;
    }
  }

  featuresDim = fDim;
}

uint KOMO::Conv_KOMO_MathematicalProgram::getDimension(){
  if(!variableIndex.N) createIndices();

  return xIndex2VarId.N;
}

void KOMO::Conv_KOMO_MathematicalProgram::getBounds(arr& bounds_lo, arr& bounds_up){
  if(!komo.configurations.N) komo.setupConfigurations();
  CHECK_EQ(komo.configurations.N, komo.k_order+komo.T, "configurations are not setup yet");

  uint n = getDimension();
  bounds_lo.resize(n);
  bounds_up.resize(n);

  uint x_count=0;
  arr limits;
  for(uint t=0; t<komo.T; t++){
    int s = t+komo.k_order;
    uint x_dim = komo.configurations(s)->getJointStateDimension();
    if(x_dim) {
      limits = ~komo.configurations(s)->getLimits();
      bounds_lo.setVectorBlock(limits[0], x_count);
      bounds_up.setVectorBlock(limits[1], x_count);
      x_count += x_dim;
    }
  }
}

void KOMO::Conv_KOMO_MathematicalProgram::getFeatureTypes(ObjectiveTypeA& featureTypes){
  if(!featureIndex.N) createIndices();

  featureTypes.clear();
  for(uint f=0;f<featureIndex.N;f++){
    featureTypes.append( consts<ObjectiveType>(featureIndex(f).ob->type, featureIndex(f).dim) );
  }
}

bool KOMO::Conv_KOMO_MathematicalProgram::isStructured(){
  return true;
}

void KOMO::Conv_KOMO_MathematicalProgram::getStructure(uintA& variableDimensions, uintA& featureDimensions, intAA& featureVariables){
  if(!variableIndex.N) createIndices();

  variableDimensions.resize(variableIndex.N);
  for(uint i=0;i<variableIndex.N;i++){
    variableDimensions(i) = variableIndex(i).dim;
  }

  featureDimensions.resize(featureIndex.N);
  featureVariables.resize(featureIndex.N);
  arr y,J;
  for(uint f=0;f<featureIndex.N;f++){
    FeatureIndexEntry& F = featureIndex(f);
    featureDimensions(f) = F.dim;
    if(!F.dim) continue;

    F.ob->map->__phi(y, J, F.Ctuple);

    CHECK(isSparseMatrix(J), "");
    F.varIds.clear();
    intA& elems = J.sparse().elems;
    for(uint i=0;i<elems.d0;i++){
      uint idx = elems(i,1);
      F.varIds.setAppendInSorted(xIndex2VarId(idx));
    }
    featureVariables(f) = F.varIds;
  }
}

void KOMO::Conv_KOMO_MathematicalProgram::evaluate(arr& phi, arr& J, arr& H, const arr& x){
  //-- set the decision variable
#if 0 //the following should be equivalent, althought they work quite differently
  komo.set_x(x);
#else
  for(uint i=0;i<variableIndex.N;i++){
    VariableIndexEntry& V = variableIndex(i);
    if(!V.dim) continue;
    setSingleVariable(i, x({V.xIndex, V.xIndex+V.dim-1}));
  }
#endif

  //-- query the features
  phi.resize(featuresDim);
  if(!!J){
    if(!komo.world.useSparseJacobians) {
      J.resize(featuresDim, x.N).setZero();
    } else {
      J.sparse().resize(featuresDim, x.N, 0);
    }
  }

  arr y, Jy;
  uint M=0;
  for(uint f=0;f<featureIndex.N;f++){
    FeatureIndexEntry& F = featureIndex(f);
    if(!F.dim) continue;

#if 1 //simpler and more direct
    F.ob->map->__phi(y, (!!J?Jy:NoArr), F.Ctuple);
    CHECK_EQ(y.N, F.dim, "");
    if(!!J) CHECK_EQ(y.N, Jy.d0, "");
    if(!!J) CHECK_EQ(Jy.nd, 2, "");
    if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

    //write into phi and J
    phi.setVectorBlock(y, M);

    if(!!J) {
      if(!isSpecial(Jy)){
        uintA kdim = getKtupleDim(F.Ctuple);
        kdim.prepend(0);
        for(uint j=0;j<F.Ctuple.N;j++){
          if(F.Ctuple(j)->xIndex>=0){
            J.setMatrixBlock(Jy.sub(0, -1, kdim(j), kdim(j+1)-1), M, F.Ctuple(j)->xIndex);
          }
        }
      }else{
        //          uint j=0;
        Jy.sparse().reshape(J.d0, J.d1);
        Jy.sparse().colShift(M);
        //          Jy.sparse().rowShift(F.Ctuple(j)->xIndex);   //sparse matrix should be properly shifted already!
        J += Jy;
      }
    }
#else //calling the single features, then converting back to sparse... just for checking..
    evaluateSingleFeature(f, y, (!!J?Jy:NoArr), NoArr);
    phi.setVectorBlock(y, M);
    if(!!J) {
      CHECK(!isSpecial(Jy), "");
      uint xCount = 0;
      for(uint v=0;v<F.varIds.N;v++){
        VariableIndexEntry& V = variableIndex(F.varIds(v));
        if(!V.dim) continue;
        if(V.xIndex<0) continue;
        for(uint j=0;j<V.dim;j++){
          for(uint i=0;i<Jy.d0;i++){
            J.elem(M+i, V.xIndex+j) += Jy(i, xCount+j);
          }
        }
        xCount += V.dim;
      }
      CHECK_EQ(xCount, Jy.d1, "");
    }
#endif

    //counter for features phi
    M += F.dim;
  }

  CHECK_EQ(M, featuresDim, "");
  komo.featureValues = phi;
  if(!!J) komo.featureJacobians.resize(1).scalar() = J;
}

void KOMO::Conv_KOMO_MathematicalProgram::setSingleVariable(uint var_id, const arr& x){
  VariableIndexEntry& V = variableIndex(var_id);
  CHECK_EQ(V.dim, x.N, "");
  if(V.joint){
    V.joint->calc_Q_from_q(x, 0);
  }
  if(V.force){
    V.force->calc_F_from_q(x, 0);
  }
}

void KOMO::Conv_KOMO_MathematicalProgram::evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H){
  FeatureIndexEntry& F = featureIndex(feat_id);

  if(!J){
    F.ob->map->__phi(phi, NoArr, F.Ctuple);
    return;
  }

  arr Jsparse;
  F.ob->map->__phi(phi, Jsparse, F.Ctuple);
  CHECK_EQ(phi.N, F.dim, "");
  CHECK_EQ(phi.N, Jsparse.d0, "");
  CHECK_EQ(Jsparse.nd, 2, "");
  if(absMax(phi)>1e10) RAI_MSG("WARNING phi=" <<phi);

  auto S = Jsparse.sparse();

  uint n=0;
  for(uint v:F.varIds) n += variableIndex(v).dim;
  J.resize(phi.N, n).setZero();

  for(uint k=0; k<Jsparse.N; k++){
    uint i = S.elems(k,0);
    uint j = S.elems(k,1);
    double x = Jsparse.elem(k);
    uint var = xIndex2VarId(j);
    VariableIndexEntry& V = variableIndex(var);
    uint var_j = j - V.xIndex;
    CHECK(var_j < V.dim, "");
    if(V.dim == J.d1){
      J(i, var_j) += x;
    }else{
      bool good=false;
      uint offset=0;
      for(uint v:F.varIds){
        if(v==var){
          J(i, offset+var_j) += x;
          good=true;
          break;
        }
        offset += variableIndex(v).dim;
      }
      CHECK(good, "Jacobian is non-zero on variable " <<var <<", but indices say that feature depends on " <<F.varIds);
    }
  }
}

void KOMO::Conv_KOMO_MathematicalProgram::reportFeatures(){
  arr y,J;
  for(uint f=0;f<featureIndex.N;f++){
    FeatureIndexEntry& F = featureIndex(f);
    cout <<f <<' ' <<F.dim <<' ' <<F.ob->map->shortTag(*F.Ctuple.last()) <<endl;
    evaluateSingleFeature(f, y, J, NoArr);
    cout <<"y:" <<y <<endl;
    cout <<"J:" <<J <<endl;
  }
}

void KOMO::TimeSliceProblem::getDimPhi() {
  CHECK_EQ(komo.configurations.N, komo.k_order+komo.T, "configurations are not setup yet: use komo.reset()");
  uint M=0;
  for(uint i=0; i<komo.objectives.N; i++) {
    ptr<Objective> ob = komo.objectives.elem(i);
    CHECK_EQ(ob->configs.nd, 2, "only in sparse mode!");
    if(ob->configs.d1!=1) continue; //ONLY USE order 0 objectives!!!!!
    for(uint l=0; l<ob->configs.d0; l++) {
      if(ob->configs(l,0)!=slice) continue; //ONLY USE objectives for this slice
      ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      M += ob->map->__dim_phi(Ktuple); //dimensionality of this task
    }
  }
  dimPhi = M;
}

void KOMO::TimeSliceProblem::phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x){
  komo.set_x(x, TUP(slice));

  if(!dimPhi) getDimPhi();

  phi.resize(dimPhi);
  if(!!tt) tt.resize(dimPhi);
  if(!!J) J.resize(dimPhi, x.N).setZero();

  uintA x_index = getKtupleDim(komo.configurations({komo.k_order, -1}));
  x_index.prepend(0);

  arr y, Jy;
  uint M=0;
  for(uint i=0; i<komo.objectives.N; i++) {
    ptr<Objective> ob = komo.objectives.elem(i);
    CHECK_EQ(ob->configs.nd, 2, "only in sparse mode!");
    if(ob->configs.d1!=1) continue; //ONLY USE order 0 objectives!!!!!
    for(uint l=0; l<ob->configs.d0; l++) {
      if(ob->configs(l,0)!=slice) continue; //ONLY USE objectives for this slice
      ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      uintA kdim = getKtupleDim(Ktuple);
      kdim.prepend(0);

      //query the task map and check dimensionalities of returns
      ob->map->__phi(y, (!!J?Jy:NoArr), Ktuple);
      if(!!J) CHECK_EQ(y.N, Jy.d0, "");
      if(!!J) CHECK_EQ(Jy.nd, 2, "");
      if(!!J) CHECK_EQ(Jy.d1, kdim.last(), "");
      if(!y.N) continue;
      if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

      //write into phi and J
      phi.setVectorBlock(y, M);

      if(!!J) {
        if(isSpecial(Jy)) Jy = unpack(Jy);
        J.setMatrixBlock(Jy, M, 0);
      }

      if(!!tt) for(uint i=0; i<y.N; i++) tt(M+i) = ob->type;

      //counter for features phi
      M += y.N;
    }
  }

  CHECK_EQ(M, dimPhi, "");
  komo.featureValues = phi;
  if(!!J) komo.featureJacobians.resize(1).scalar() = J;
  if(!!tt) komo.featureTypes = tt;
}

rai::Configuration& KOMO::getConfiguration(double phase) {
  uint s = k_order + conv_time2step(phase, stepsPerPhase);
  return *configurations(s);
}

Configuration&KOMO::getConfiguration_t(int t){
  if(t<0) CHECK_LE(-t, (int)k_order,"");
  return *configurations(t+k_order);
}

arr KOMO::getJointState(double phase) {
  uint s = k_order + conv_time2step(phase, stepsPerPhase);
  return configurations(s)->getJointState();
}

arr KOMO::getFrameState(double phase) {
  uint s = k_order + conv_time2step(phase, stepsPerPhase);
  return configurations(s)->getFrameState();
}

arr KOMO::getPath_decisionVariable() {
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");
  arr x;
  for(uint t=0; t<T; t++) x.append(configurations(t+k_order)->getJointState());
  return x;
}

arr KOMO::getPath(const StringA& joints) {
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");
  uint n = joints.N;
  if(!n) n = world.getJointStateDimension();
  arr X(T, n);
  for(uint t=0; t<T; t++) {
    if(joints.N)
      X[t] = configurations(t+k_order)->getJointState(joints);
    else
      X[t] = configurations(t+k_order)->getJointState();
  }
  return X;
}

arr KOMO::getPath(const uintA& joints) {
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");
  arr X = configurations(k_order)->getJointState(joints);
  X.resizeCopy(T, X.N);
  for(uint t=1; t<T; t++) {
    X[t] = configurations(k_order+t)->getJointState(joints);
  }
  return X;
}

arr KOMO::getPath_frames(const StringA& frame) {
  uintA _frames;
  if(frame.N) for(const rai::String& f:frame) _frames.append(world.getFrameByName(f)->ID);
  else _frames.setStraightPerm(world.frames.N);
  return getPath_frames(_frames);
}

arr KOMO::getPath_frames(const uintA& frames) {
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");
  arr X(T, frames.N, 7);
  for(uint t=0; t<T; t++) {
    for(uint i=0; i<frames.N; i++) {
      X(t, i, {}) = configurations(t+k_order)->frames(frames(i))->ensure_X().getArr7d();
    }
  }
  return X;
}

arrA KOMO::getPath_q() {
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");
  arrA q(T);
  for(uint t=0; t<T; t++) {
    q(t) = configurations(t+k_order)->getJointState();
  }
  return q;
}

arr KOMO::getPath_tau() {
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");
  arr X(T);
  for(uint t=0; t<T; t++) {
    X(t) = configurations(t+k_order)->frames.first()->tau;
  }
  return X;
}

arr KOMO::getPath_times() {
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");
  arr X(T);
  double time=0.;
  for(uint t=0; t<T; t++) {
    time += configurations(t+k_order)->frames.first()->tau;
    X(t) = time;
  }
  return X;
}

arr KOMO::getPath_energies() {
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");
  F_Energy E;
  E.order=1;
  arr X(T), y;
  for(uint t=0; t<T; t++) {
    E.phi(y, NoArr, {configurations(t+k_order-1), configurations(t+k_order)});
    X(t) = y.scalar();
  }
  return X;
}

arr KOMO::getActiveConstraintJacobian() {
  uint n=0;
  for(uint i=0; i<dual.N; i++) if(dual.elem(i)>0.) n++;

  arr J(n, x.N);

  n=0;
  for(uint i=0; i<dual.N; i++) {
    if(dual.elem(i)>0.) {
      J[n] = featureJacobians.scalar()[i];
      n++;
    }
  }
  CHECK_EQ(n, J.d0, "");

  return J;
}

template<> const char* rai::Enum<SkeletonSymbol>::names []= {
  "touch",
  "above",
  "inside",
  "oppose",

  "impulse",
  "initial",
  "free",

  "poseEq",
  "stableRelPose",
  "stablePose",

  "stable",
  "stableOn",
  "dynamic",
  "dynamicOn",
  "dynamicTrans",
  "quasiStatic",
  "quasiStaticOn",
  "liftDownUp",
  "break",

  "contact",
  "contactStick",
  "contactComplementary",
  "bounce",

  "magic",
  "magicTrans",

  "push",
  "graspSlide",

  "dampMotion",

  "noCollision",
  "identical",

  "alignByInt",

  "makeFree",
  nullptr
};

intA getSwitchesFromSkeleton(const Skeleton& S) {
  rai::Array<SkeletonSymbol> modes = { SY_free, SY_stable, SY_stableOn, SY_dynamic, SY_dynamicOn, SY_dynamicTrans, SY_quasiStatic, SY_quasiStaticOn };

  intA ret;
  for(int i=0; i<(int)S.N; i++) {
    if(modes.contains(S.elem(i).symbol)) {
      int j=i-1;
      for(; j>=0; j--) {
        if(modes.contains(S.elem(j).symbol) && S.elem(j).frames.last()==S.elem(i).frames.last()) {
          break;
        }
      }
      ret.append({j, i});
    }
  }
  ret.reshape(ret.N/2, 2);

  return ret;
}

void writeSkeleton(ostream& os, const Skeleton& S, const intA& switches) {
  os <<"SKELETON:" <<endl;
  for(auto& s:S) os <<"  " <<s <<endl;
  if(switches.N) {
    os <<"SWITCHES:" <<endl;
    for(uint i=0; i<switches.d0; i++) {
      int j = switches(i, 0);
      if(j<0)
        os <<"  START  -->  " <<S(switches(i, 1)) <<endl;
      else
        os <<"  " <<S(j) <<"  -->  " <<S(switches(i, 1)) <<endl;
    }
  }
}


