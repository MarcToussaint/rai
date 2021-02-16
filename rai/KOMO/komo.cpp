/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "komo.h"
#include "komo-ext.h"

#include "../Algo/spline.h"
#include "../Gui/opengl.h"
#include "../Geo/fclInterface.h"

#include "../Kin/frame.h"
#include "../Kin/switch.h"
#include "../Kin/proxy.h"
#include "../Kin/forceExchange.h"
#include "../Kin/kin_swift.h"
#include "../Kin/kin_physx.h"
#include "../Kin/F_qFeatures.h"
#include "../Kin/TM_default.h"
#include "../Kin/F_pose.h"
#include "../Kin/F_collisions.h"
#include "../Kin/F_geometrics.h"
#include "../Kin/F_operators.h"
#include "../Kin/F_forces.h"
#include "../Kin/viewer.h"

#include "../Optim/optimization.h"
#include "../Optim/primalDual.h"
#include "../Optim/GraphOptim.h"
#include "../Optim/opt-nlopt.h"
#include "../Optim/opt-ipopt.h"
#include "../Optim/opt-ceres.h"

#include "../Core/util.ipp"

#include <iomanip>

#ifdef RAI_GL
#  include <GL/gl.h>
#endif

//#ifndef RAI_SWIFT
//#  define FCLmode
//#endif

#define KOMO_PATH_CONFIG

using namespace rai;

//===========================================================================

template<> const char* rai::Enum<rai::KOMOsolver>::names []= {
  "dense", "sparse", "banded", "sparseFactored", "NLopt", "Ipopt", "Ceres", nullptr
};

rai::Array<SkeletonSymbol> skeletonModes = { SY_free, SY_stable, SY_stableOn, SY_dynamic, SY_dynamicOn, SY_dynamicTrans, SY_quasiStatic, SY_quasiStaticOn, SY_magicTrans };

//===========================================================================

double shapeSize(const Configuration& K, const char* name, uint i=2);

struct getQFramesAndScale_Return { uintA frames; arr scale; };
getQFramesAndScale_Return getCtrlFramesAndScale(const rai::Configuration& C);

Shape* getShape(const Configuration& K, const char* name) {
  Frame* f = K.getFrame(name);
  Shape* s = f->shape;
  if(!s) {
    for(Frame* b:f->children) if(b->name==name && b->shape) { s=b->shape; break; }
  }
  return s;
}

KOMO::KOMO() : computeCollisions(true), verbose(1) {
  verbose = getParameter<double>("KOMO/verbose", 1);
  animateOptimization = getParameter<double>("KOMO/animateOptimization", 0);
  solver = getParameter<rai::Enum<rai::KOMOsolver>>("KOMO/solver", KS_sparse);
}

KOMO::~KOMO() {
  gl.reset();
  if(logFile) delete logFile;
  objs.clear();
  objectives.clear();
  listDelete(switches);
}

void KOMO::setModel(const Configuration& C, bool _computeCollisions) {
  if(&C!=&world) world.copy(C, _computeCollisions);
  computeCollisions = _computeCollisions;
  if(computeCollisions) {
#ifndef FCLmode
    world.swift();
#else
    world.fcl();
#endif
  }
  world.ensure_q();
}

void KOMO_ext::useJointGroups(const StringA& groupNames, bool notThese) {
  world.selectJointsByGroup(groupNames, notThese);

  world.optimizeTree();
  world.getJointState();

//  world.meldFixedJoints();
//  world.removeUselessBodies();

//  FILE("z.komo.model") <<world;
}

void KOMO::setTiming(double _phases, uint _stepsPerPhase, double durationPerPhase, uint _k_order) {
  stepsPerPhase = _stepsPerPhase;
  T = ceil(stepsPerPhase*_phases);
  tau = durationPerPhase/double(stepsPerPhase);
  k_order = _k_order;
}

void KOMO::activateCollisions(const char* s1, const char* s2) {
  if(!computeCollisions) return;
  Frame* sh1 = world.getFrame(s1);
  Frame* sh2 = world.getFrame(s2);
  if(sh1 && sh2) world.swift()->activate(sh1, sh2);
}

void KOMO::deactivateCollisions(const char* s1, const char* s2) {
  if(!computeCollisions) return;
  Frame* sh1 = world.getFrame(s1);
  Frame* sh2 = world.getFrame(s2);
  if(sh1 && sh2) world.swift()->deactivate(sh1, sh2);
  else LOG(-1) <<"not found:" <<s1 <<' ' <<s2;
}

void KOMO::addTimeOptimization() {
  world.addTauJoint();
  rai::Frame *timeF = world.frames.first();
#if 0 //break the constraint at phase switches: EMPIRICAL EQUIVALENT TO BELOW (passive_ballBounce TEST)
  ptr<Objective> o = addObjective({}, make_shared<F_qTime>(), {timeF->name}, OT_sos, {1e2}, {}, 1); //smooth time evolution
  if(o->configs.nd==1) { //for KOMO optimization
    CHECK(o->configs.nd==1 && o->configs.N==T, "");
    CHECK_GE(stepsPerPhase, 10, "NIY");
    for(uint t=2; t<o->configs.N; t+=stepsPerPhase) o->configs(t)=0;
  } else { //for sparse optimization
    CHECK(o->configs.nd==2 && o->configs.N==2*T, "");
    CHECK_GE(stepsPerPhase, 10, "NIY");
    for(uint t=o->configs.d0; t--;) if(o->configs(t, 0)%stepsPerPhase==0) o->configs.delRows(t);
  }
#else
  addObjective({0.}, make_shared<F_qTime>(), {timeF->name}, OT_sos, {1e2}, {}, 1, 0, +1); //smooth time evolution
  for(uint t=0; t<T/stepsPerPhase; t++){
    addObjective({double(t), double(t+1)}, make_shared<F_qTime>(), {timeF->name}, OT_sos, {1e2}, {}, 1, +3, +1); //smooth time evolution
  }
#endif

  addObjective({}, make_shared<F_qTime>(), {timeF->name}, OT_sos, {1e-1}, {tau}); //prior on timing
  addObjective({}, make_shared<F_qTime>(), {timeF->name}, OT_ineq, {-1e1}, {.9*tau}); //lower bound on timing
}

//===========================================================================
//
// task specs
//

void KOMO::clearObjectives() {
  objectives.clear(); //listDelete(objectives);
  objs.clear();
  listDelete(switches);
  reset();
}

ptr<Objective> KOMO::addObjective(const arr& times,
                                  const ptr<Feature>& f, const StringA& frames,
                                  ObjectiveType type, const arr& scale, const arr& target, int order,
                                  int deltaFromStep, int deltaToStep) {
#ifdef KOMO_PATH_CONFIG
  if(!timeSlices.N) setupConfigurations2();
#endif

  if(!!frames && frames.N){
    if(frames.N==1 && frames.scalar()=="ALL") f->frameIDs = framesToIndices(world.frames); //important! this means that, if no explicit selection of frames was made, all frames (of a time slice) are referred to
    else f->frameIDs = world.getFrameIDs(frames);
  }
  if(!!scale) f->scale = scale;
  if(!!target) f->target = target;
  if(order>=0) f->order = order;

  CHECK_GE(k_order, f->order, "task requires larger k-order: " <<f->shortTag(world));
  std::shared_ptr<Objective> task = make_shared<Objective>(f, type);
  task->name = f->shortTag(world);
  objectives.append(task);
//  task->setCostSpecs(times, stepsPerPhase, T, deltaFromStep, deltaToStep, true); //solver!=rai::KS_banded);
  task->configs = conv_times2tuples(times, f->order, stepsPerPhase, T, deltaFromStep, deltaToStep);
//  if(solver!=rai::KS_banded)
  CHECK_EQ(task->configs.nd, 2, "");
#ifdef KOMO_PATH_CONFIG
  for(uint c=0;c<task->configs.d0;c++){
    shared_ptr<GroundedObjective> o = objs.append( make_shared<GroundedObjective>(f, type) );
    o->configs = task->configs[c];
    o->objId = objectives.N-1;
    o->frames.resize(task->configs.d1, o->feat->frameIDs.N);
    for(uint i=0;i<task->configs.d1;i++){
      int s = task->configs(c,i) + k_order;
      for(uint j=0;j<o->feat->frameIDs.N;j++){
        uint fID = o->feat->frameIDs.elem(j);
        o->frames(i,j) = timeSlices(s, fID);
      }
    }
    if(o->feat->frameIDs.nd==2){
      o->frames.reshape(task->configs.d1, o->feat->frameIDs.d0, o->feat->frameIDs.d1);
    }
  }
#endif
  return task;
}

ptr<Objective> KOMO::addObjective(const arr& times, const FeatureSymbol& feat, const StringA& frames,
                                  ObjectiveType type, const arr& scale, const arr& target, int order,
                                  int deltaFromStep, int deltaToStep) {
  return addObjective(times, symbols2feature(feat, frames, world),
                      NoStringA, type, scale, target, order, deltaFromStep, deltaToStep);
}

//void KOMO::addFlag(double time, Flag *fl, int deltaStep) {
//  if(time<0.) time=0.;
//  fl->stepOfApplication = conv_time2step(time, stepsPerPhase) + deltaStep;
//  flags.append(fl);
//}

void KOMO::addSwitch(const arr& times, bool before, KinematicSwitch* sw) {
  sw->setTimeOfApplication(times, before, stepsPerPhase, T);
  switches.append(sw);
}

KinematicSwitch* KOMO::addSwitch(const arr& times, bool before,
                     rai::JointType type, SwitchInitializationType init,
                     const char* ref1, const char* ref2,
                     const rai::Transformation& jFrom, const rai::Transformation& jTo) {
  KinematicSwitch* sw = new KinematicSwitch(SW_joint, type, ref1, ref2, world, init, 0, jFrom, jTo);
  addSwitch(times, before, sw);
  return sw;
}

void KOMO::addSwitch_mode2(const arr& times, SkeletonSymbol newMode, const StringA& frames, bool firstSwitch) {
  //-- creating a stable kinematic linking
  if(newMode==SY_stable || newMode==SY_stableOn){
    if(newMode==SY_stable) {
      auto sw = addSwitch(times, true, JT_free, SWInit_copy, frames(0), frames(1));
      sw->isStable = true;
    } else { //SY_stableOn
      Transformation rel = 0;
      rel.pos.set(0, 0, .5*(shapeSize(world, frames(0)) + shapeSize(world, frames(1))));
      auto sw = addSwitch(times, true, JT_transXYPhi, SWInit_copy, frames(0), frames(1), rel);
      sw->isStable = true;
    }

    // ensure the DOF is constant throughout its existance
    if((times(1)<0. && stepsPerPhase*times(0)<T) || stepsPerPhase*times(1)>stepsPerPhase*times(0)+1) {
      addObjective({times(0), times(1)}, make_shared<F_qZeroVel>(), {frames(-1)}, OT_eq, {1e1}, NoArr, 1, +1, -1);
    }

    //-- no jump at start
    if(firstSwitch){
      //NOTE: when frames(0) is picking up a kinematic chain (e.g., where frames(1) is a handB of a walker),
      //  then we actually need to impose the no-jump constrained on the root of the kinematic chain!
      //  To prevent special case for the skeleton specifer, we use this ugly code to determine the root of
      //  the kinematic chain for frames(1) -- when frames(1) is a normal object, this should be just frames(1) itself
      rai::Frame *toBePicked = world[frames(1)];
      rai::Frame *rootOfPicked = toBePicked->getUpwardLink(NoTransformation, true);
      if(stepsPerPhase>3){
        addObjective({times(0)}, FS_pose, {rootOfPicked->name}, OT_eq, {1e2}, NoArr, 1, 0, +1);
      }else{
        addObjective({times(0)}, FS_pose, {rootOfPicked->name}, OT_eq, {1e2}, NoArr, 1, 0, 0);
      }
    }

    //-- no jump at end
    if(times(1)>=0){
      addObjective({times(1)}, FS_poseRel, {frames(1), frames(0)}, OT_eq, {1e2}, NoArr, 1, 0, 0);
      if(k_order>1) addObjective({times(1)}, make_shared<F_LinAngVel>(), {frames(-1)}, OT_eq, {1e0}, NoArr, 2, +1, +1); //no acceleration of the object
    }

  } else if(newMode==SY_dynamic) {
    CHECK_EQ(frames.N, 1, "");
    addSwitch(times, true, JT_free, SWInit_copy, world.frames.first()->name, frames(-1));
    //new contacts don't exist in step [-1], so we rather impose only zero acceleration at [-2,-1,0]
    if(firstSwitch){
      if(stepsPerPhase>3){
        addObjective({times(0)}, FS_pose, {frames(-1)}, OT_eq, {1e2}, NoArr, 1, 0, +1); //overlaps with Newton-Euler -> requires forces!
      }else{
        addObjective({times(0)}, FS_pose, {frames(-1)}, OT_eq, {1e2}, NoArr, 1, 0, 0);
      }
    }
    if(k_order>1){
      //... and physics starting from [-1,0,+1], ... until [-2,-1,-0]
      addObjective(times, make_shared<F_NewtonEuler>(), {frames(-1)}, OT_eq, {1e2}, NoArr, 2, +1, 0);
    }
  } else if(newMode==SY_quasiStaticOn) {
    CHECK_EQ(frames.N, 2, "");
    Transformation rel = 0;
    rel.pos.set(0, 0, .5*(shapeSize(world, frames(0)) + shapeSize(world, frames(1))));
    addSwitch(times, true, JT_transXYPhi, SWInit_copy, frames(0), frames(1), rel);
    //-- no jump at start
    if(firstSwitch){
      addObjective({times(0)}, FS_pose, {frames(-1)}, OT_eq, {1e2}, NoArr, 1, 0, +1);
    }
#if 0
    addObjective(times, make_shared<F_NewtonEuler_DampedVelocities>(0., false), {frames(-1)}, OT_eq, {1e2}, NoArr, 1, +1, 0);
#else
    //eq for 3DOFs only
    ptr<Objective> o = addObjective(times, make_shared<F_NewtonEuler_DampedVelocities>(false), {frames(-1)}, OT_eq, {1e2}, NoArr, 1, +1, 0);
    o->feat->scale=1e2 * arr({3, 6}, {
      1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1
    });
    //sos penalty of other forces
    o = addObjective(times, make_shared<F_NewtonEuler_DampedVelocities>(false), {frames(-1)}, OT_sos, {1e0}, NoArr, 1, +1, 0);
    o->feat->scale=1e0 * arr({3, 6}, {
      0, 0, 1, 0, 0, 0,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0
    });
#endif
  } else NIY;
}


void KOMO::addSwitch_mode(SkeletonSymbol prevMode, SkeletonSymbol newMode, double time, double endTime, const char* prevFrom, const char* from, const char* to) {
  //-- creating a stable kinematic linking
  if(newMode==SY_stable || newMode==SY_stableOn) {
    // create the kinematic switch
    if(newMode==SY_stable) {
      addSwitch({time}, true, JT_free, SWInit_copy, from, to);
    } else { //SY_stableOn
      Transformation rel = 0;
      rel.pos.set(0, 0, .5*(shapeSize(world, from) + shapeSize(world, to)));
      addSwitch({time}, true, JT_transXYPhi, SWInit_copy, from, to, rel);
    }

    // ensure the DOF is constant throughout its existance
    if((endTime<0. && stepsPerPhase*time<T) || stepsPerPhase*endTime>stepsPerPhase*time+1) {
      addObjective({time, endTime}, make_shared<F_qZeroVel>(), {to}, OT_eq, {1e1}, NoArr, 1, +1, -1);
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
//        addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);
        if(prevFrom) addObjective({time}, FS_poseRel, {to, prevFrom}, OT_eq, {1e2}, NoArr, 1, 0, 0);
        else addObjective({time}, FS_pose, {to}, OT_eq, {1e2}, NoArr, 1, 0, 0);
      }
    } else {
      //-- no acceleration at start: +1 EXCLUDES (x-2, x-1, x0), ASSUMPTION: this is a placement that can excert impact
      if(k_order>1) addObjective({time}, make_shared<F_LinAngVel>(), {to}, OT_eq, {1e2}, NoArr, 2, +1, +1);
//      else addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);
      else{
        if(prevFrom) addObjective({time}, FS_poseRel, {to, prevFrom}, OT_eq, {1e2}, NoArr, 1, 0, 0);
        else addObjective({time}, FS_pose, {to}, OT_eq, {1e2}, NoArr, 1, 0, 0);
//        addObjective({time}, FS_pose, {to}, OT_eq, {1e2}, NoArr, 1, 0, 0);
      }
    }
  }

  if(newMode==SY_dynamic) {
    addSwitch({time}, true, JT_free, SWInit_copy, from, to);
    //new contacts don't exist in step [-1], so we rather impose only zero acceleration at [-2,-1,0]
    addObjective({time}, FS_pose, {to}, OT_eq, {1e0}, NoArr, k_order, +0, +0);
    //... and physics starting from [-1,0,+1], ... until [-3,-2,-1]
    addObjective({time, endTime}, make_shared<F_NewtonEuler>(), {to}, OT_eq, {1e0}, NoArr, k_order, +1, -1);
  }

  if(newMode==SY_dynamicTrans) {
    HALT("deprecated")
//    addSwitch({time}, true, JT_trans3, SWInit_copy, from, to);
//    addObjective({time, endTime}, make_shared<F_NewtonEuler>(true), {to}, OT_eq, {3e1}, NoArr, k_order, +0, -1);
  }

  if(newMode==SY_dynamicOn) {
    Transformation rel = 0;
    rel.pos.set(0, 0, .5*(shapeSize(world, from) + shapeSize(world, to)));
    addSwitch({time}, true, JT_transXYPhi, SWInit_copy, from, to, rel);
    if(k_order>=2) addObjective({time, endTime}, FS_pose, {to}, OT_eq, {3e1}, NoArr, k_order, +0, -1);
    //  else addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);
  }

  if(newMode==SY_quasiStatic) {
    addSwitch({time}, true, JT_free, SWInit_copy, from, to);
    addObjective({time, endTime}, make_shared<F_NewtonEuler_DampedVelocities>(), {to}, OT_eq, {1e1}, NoArr, 1, +0, -1);
  }

  if(newMode==SY_quasiStaticOn) {
    Transformation rel = 0;
    rel.pos.set(0, 0, .5*(shapeSize(world, from) + shapeSize(world, to)));
    addSwitch({time}, true, JT_transXYPhi, SWInit_copy, from, to, rel);
#if 0
    addObjective({time, endTime}, make_shared<F_NewtonEuler_DampedVelocities>(world, to, 0., false), OT_eq, {1e2}, NoArr, 1, +0, -1);
#else
    //eq for 3DOFs only
    ptr<Objective> o = addObjective({time, endTime}, make_shared<F_NewtonEuler_DampedVelocities>(false), {to}, OT_eq, {1e2}, NoArr, 1, +0, -1);
    o->feat->scale=1e2 * arr({3, 6}, {
      1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1
    });
    //sos penalty of other forces
    o = addObjective({time, endTime}, make_shared<F_NewtonEuler_DampedVelocities>(false), {to}, OT_sos, {1e2}, NoArr, 1, +0, -1);
    o->feat->scale=1e1 * arr({3, 6}, {
      0, 0, 1, 0, 0, 0,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0
    });
#endif
//    addObjective({time, endTime}, make_shared<F_pushed>(world, to), OT_eq, {1e0}, NoArr, 1, +0, -1);

    //-- no acceleration at start: +1 EXCLUDES (x-2, x-1, x0), ASSUMPTION: this is a placement that can excert impact
    if(k_order>1) addObjective({time}, make_shared<F_LinAngVel>(), {to}, OT_eq, {1e2}, NoArr, 2, +0, +1);
//    else addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);
    else addObjective({time}, FS_pose, {to}, OT_eq, {1e2}, NoArr, 1, 0, 0);

  }

  if(newMode==SY_magicTrans){
    addSwitch({time}, true, JT_transZ, SWInit_copy, from, to);
    double sqrAccCost=0.;
    if(sqrAccCost>0.) {
      addObjective({time, endTime}, make_shared<F_LinAngVel>(), {to}, OT_sos, {sqrAccCost}, NoArr, 2);
    }
  }
}

void KOMO::addSwitch_stable(double time, double endTime, const char* prevFrom, const char* from, const char* to, bool firstSwitch) {
#if 0
  addSwitch_mode(SY_none, SY_stable, time, endTime, prevFrom, from, to);
#else
  addSwitch_mode2({time, endTime}, SY_stable, {from, to}, firstSwitch);
#endif
}

void KOMO::addSwitch_stableOn(double time, double endTime, const char* from, const char* to) {
#if 1
  addSwitch_mode(SY_none, SY_stableOn, time, endTime, NULL, from, to);
#else
  Transformation rel = 0;
  rel.pos.set(0, 0, .5*(shapeSize(world, from) + shapeSize(world, to)));
  addSwitch({time}, true, JT_transXYPhi, SWInit_zero, from, to, rel);
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
  addSwitch({time}, true, JT_free, SWInit_copy, from, to);
  if(!dampedVelocity)
    addObjective({time, endTime}, make_shared<F_NewtonEuler>(), {to}, OT_eq, {1e0}, NoArr, 2, +0, -1);
  else
    addObjective({time, endTime}, make_shared<F_NewtonEuler_DampedVelocities>(), {to}, OT_eq, {1e2}, NoArr, 1, +0, -1);
//  addObjective({time}, make_shared<TM_LinAngVel>(world, to), OT_eq, {1e2}, NoArr, 2); //this should be implicit in the NE equations!
}

void KOMO::addSwitch_dynamicTrans(double time, double endTime, const char* from, const char* to) {
  HALT("deprecated")
//  addSwitch({time}, true, JT_trans3, SWInit_copy, from, to);
//  addObjective({time, endTime}, make_shared<F_NewtonEuler>(true), {to}, OT_eq, {3e1}, NoArr, k_order, +0, -1);
}

void KOMO::addSwitch_dynamicOn(double time, double endTime, const char* from, const char* to) {
  Transformation rel = 0;
  rel.pos.set(0, 0, .5*(shapeSize(world, from) + shapeSize(world, to)));
  addSwitch({time}, true, JT_transXYPhi, SWInit_zero, from, to, rel);
  if(k_order>=2) addObjective({time, endTime}, FS_pose, {to}, OT_eq, {3e1}, NoArr, k_order, +0, -1);
//  else addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);
}

void KOMO::addSwitch_dynamicOnNewton(double time, double endTime, const char* from, const char* to) {
  Transformation rel = 0;
  rel.pos.set(0, 0, .5*(shapeSize(world, from) + shapeSize(world, to)));
  addSwitch({time}, true, JT_transXYPhi, SWInit_zero, from, to, rel);
  if(k_order>=2) addObjective({time, endTime}, make_shared<F_NewtonEuler>(), {to}, OT_eq, {1e0}, NoArr, k_order, +0, -1);
}

void KOMO::addSwitch_magic(double time, double endTime, const char* from, const char* to, double sqrAccCost, double sqrVelCost) {
  addSwitch({time}, true, JT_free, SWInit_copy, from, to);
  if(sqrVelCost>0. && k_order>=1) {
    addObjective({time, endTime}, make_shared<F_LinAngVel>(), {to}, OT_sos, {sqrVelCost}, NoArr, 1);
  }
  if(sqrAccCost>0. && k_order>=2) {
    addObjective({time, endTime}, make_shared<F_LinAngVel>(), {to}, OT_sos, {sqrAccCost}, NoArr, 2);
  }
}

void KOMO::addSwitch_magicTrans(double time, double endTime, const char* from, const char* to, double sqrAccCost) {
  addSwitch({time}, true, JT_transZ, SWInit_copy, from, to);
  if(sqrAccCost>0.) {
    addObjective({time, endTime}, make_shared<F_LinAngVel>(), {to}, OT_eq, {sqrAccCost}, NoArr, 2);
  }
}

void KOMO::addSwitch_on(double time, const char* from, const char* to, bool copyInitialization) {
  Transformation rel = 0;
  rel.pos.set(0, 0, .5*(shapeSize(world, from) + shapeSize(world, to)));
  addSwitch({time}, true, JT_transXYPhi, (copyInitialization?SWInit_copy:SWInit_zero), from, to, rel);
}

void KOMO::addContact_slide(double startTime, double endTime, const char* from, const char* to) {
  addSwitch({startTime}, true, new rai::KinematicSwitch(rai::SW_addContact, rai::JT_none, from, to, world));
  if(endTime>0.) addSwitch({endTime}, false, new rai::KinematicSwitch(rai::SW_delContact, rai::JT_none, from, to, world));

  //constraints
#if 1 //new, based on functionals
  addObjective({startTime, endTime}, make_shared<F_fex_POASurfaceDistance>(rai::_left), {from, to}, OT_eq, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_POASurfaceDistance>(rai::_right), {from, to}, OT_eq, {1e1});
//  addObjective({startTime, endTime}, make_shared<F_fex_POASurfaceNormalsOppose>(), {from, to}, OT_eq, {1e0});
#else //old, based on PairCollision
  addObjective({startTime, endTime}, make_shared<F_fex_POAisInIntersection_InEq>(), {from, to}, OT_ineq, {1e1});
#endif
  addObjective({startTime, endTime}, FS_pairCollision_negScalar, {from, to}, OT_eq, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_ForceIsNormal>(), {from, to}, OT_eq, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_ForceIsPositive>(), {from, to}, OT_ineq, {1e2});

  //regularization
  addObjective({startTime, endTime}, make_shared<F_fex_Force>(), {from, to}, OT_sos, {1e-2}, NoArr, k_order, +2, 0);
  addObjective({startTime, endTime}, make_shared<F_fex_Force>(), {from, to}, OT_sos, {1e-2});
  addObjective({startTime, endTime}, make_shared<F_fex_POA>(), {from, to}, OT_sos, {1e-2}, NoArr, k_order, +2, +0);
  addObjective({startTime, endTime}, make_shared<F_fex_POAzeroRelVel>(), {from, to}, OT_sos, {1e-1}, NoArr, 1, +1, +0);
}

void KOMO::addContact_stick(double startTime, double endTime, const char* from, const char* to) {
  addSwitch({startTime}, true, new rai::KinematicSwitch(rai::SW_addContact, rai::JT_none, from, to, world));
  if(endTime>0.) addSwitch({endTime}, false, new rai::KinematicSwitch(rai::SW_delContact, rai::JT_none, from, to, world));

  //constraints
#if 1 //new, based on functionals
  addObjective({startTime, endTime}, make_shared<F_fex_POASurfaceDistance>(rai::_left), {from, to}, OT_eq, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_POASurfaceDistance>(rai::_right), {from, to}, OT_eq, {1e1});
#else
  addObjective({startTime, endTime}, make_shared<F_fex_POAContactDistances>(), {from, to}, OT_ineq, {1e1});
#endif
  addObjective({startTime, endTime}, FS_pairCollision_negScalar, {from, to}, OT_eq, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_ForceIsPositive>(), {from, to}, OT_ineq, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_POAzeroRelVel>(), {from, to}, OT_eq, {1e0}, NoArr, 1, +1, +1);

  //regularization
//  addObjective({startTime, endTime}, make_shared<TM_Contact_Force>(world, from, to), OT_sos, {1e-2}, NoArr, 2, +2, 0);
  addObjective({startTime, endTime}, make_shared<F_fex_Force>(), {from, to}, OT_sos, {1e-4});
  addObjective({startTime, endTime}, make_shared<F_fex_POA>(), {from, to}, OT_sos, {1e-2}, NoArr, 2, +2, +0);
  addObjective({startTime, endTime}, make_shared<F_fex_POA>(), {from, to}, OT_sos, {1e-2}, NoArr, 1, +1, +0);
}

void KOMO::addContact_ComplementarySlide(double startTime, double endTime, const char* from, const char* to) {
  addSwitch({startTime}, true, new rai::KinematicSwitch(rai::SW_addContact, rai::JT_none, from, to, world));
  if(endTime>0.) addSwitch({endTime}, false, new rai::KinematicSwitch(rai::SW_delContact, rai::JT_none, from, to, world));

  //constraints
  addObjective({startTime, endTime}, make_shared<F_fex_ForceIsNormal>(), {from, to}, OT_eq, {1e2});
  addObjective({startTime, endTime}, make_shared<F_fex_ForceIsComplementary>(), {from, to}, OT_eq, {1e2});
  addObjective({startTime, endTime}, make_shared<F_fex_NormalVelIsComplementary>(0., 0.), {from, to}, OT_eq, {1e2}, NoArr, 1, +1);
  addObjective({startTime, endTime}, FS_pairCollision_negScalar, {from, to}, OT_ineq, {1e1});

  //regularization
  addObjective({startTime, endTime}, make_shared<F_fex_Force>(), {from, to}, OT_sos, {1e-4});
  addObjective({startTime, endTime}, make_shared<F_fex_POA>(), {from, to}, OT_sos, {1e-2}, NoArr, 2, +3, +0);
  addObjective({startTime, endTime}, make_shared<F_fex_POA>(), {from, to}, OT_sos, {1e-2}, NoArr, 1, +1, +0);
//  addObjective({startTime, endTime}, make_shared<TM_Contact_POAzeroRelVel>(world, from, to), OT_sos, {1e-1}, NoArr, 1, +1, +0);
}

void KOMO::addContact_staticPush(double startTime, double endTime, const char* from, const char* to) {
  HALT("OBSOLETE");
  addSwitch({startTime}, true, new rai::KinematicSwitch(rai::SW_addContact, rai::JT_none, from, to, world));
  if(endTime>0.) addSwitch({endTime}, false, new rai::KinematicSwitch(rai::SW_delContact, rai::JT_none, from, to, world));

  addObjective({startTime, endTime}, make_shared<F_fex_ForceIsNormal>(), {from, to}, OT_sos, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_ForceIsPositive>(), {from, to}, OT_ineq, {1e2});
  addObjective({startTime, endTime}, make_shared<F_fex_POAContactDistances>(), {from, to}, OT_ineq, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_POAmovesContinuously>(), {from, to}, OT_sos, {1e0}, NoArr, 1, +1, +0);
  addObjective({startTime, endTime}, make_shared<F_fex_Force>(), {from, to}, OT_sos, {1e-1});
  addObjective({startTime, endTime}, make_shared<F_fex_POAzeroRelVel>(), {from, to}, OT_sos, {1e-1}, NoArr, 1, +1, +0);
  //  addObjective({startTime, endTime}, make_shared<TM_Contact_POAzeroRelVel>(world, from, to), OT_eq, {1e1}, NoArr, 1, +1, +0);
  //  addObjective({time}, make_shared<F_pushed>(world, to), OT_eq, {1e1}, NoArr, 1, +1, +0);
}

void KOMO::addContact_noFriction(double startTime, double endTime, const char* from, const char* to) {
  HALT("OBSOLETE");
  addSwitch({startTime}, true, new rai::KinematicSwitch(rai::SW_addContact, rai::JT_none, from, to, world));
  if(endTime>0.) addSwitch({endTime}, false, new rai::KinematicSwitch(rai::SW_delContact, rai::JT_none, from, to, world));

  addObjective({startTime, endTime}, make_shared<F_fex_ForceIsNormal>(), {from, to}, OT_eq, {3e1});
  addObjective({startTime, endTime}, make_shared<F_fex_ForceIsPositive>(), {from, to}, OT_ineq, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_POAContactDistances>(), {from, to}, OT_ineq, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_POAmovesContinuously>(), {from, to}, OT_sos, {1e0}, NoArr, 1, +1, +0);
  addObjective({startTime, endTime}, make_shared<F_fex_Force>(), {from, to}, OT_sos, {1e-4});
  addObjective({startTime, endTime}, FS_pairCollision_negScalar, {from, to}, OT_eq, {1e1});
}

void KOMO::addContact_elasticBounce(double time, const char* from, const char* to, double elasticity, double stickiness) {
  addSwitch({time}, true,  new rai::KinematicSwitch(rai::SW_addContact, rai::JT_none, from, to, world));
  addSwitch({time}, false, new rai::KinematicSwitch(rai::SW_delContact, rai::JT_none, from, to, world));

  //constraints
#if 1 //new, based on functionals
  addObjective({time}, make_shared<F_fex_POASurfaceDistance>(rai::_left), {from, to}, OT_eq, {1e1});
  addObjective({time}, make_shared<F_fex_POASurfaceDistance>(rai::_right), {from, to}, OT_eq, {1e1});
#else
  addObjective({time}, make_shared<F_fex_POAContactDistances>(), {from, to}, OT_ineq, {1e1});
#endif
  addObjective({time}, FS_pairCollision_negScalar, {from, to}, OT_eq, {1e1});
  if(stickiness<=0.) addObjective({time}, make_shared<F_fex_ForceIsNormal>(), {from, to}, OT_eq, {1e1});
  addObjective({time}, make_shared<F_fex_ForceIsPositive>(), {from, to}, OT_ineq, {1e1});
  if(!elasticity && stickiness>=1.) {
    addObjective({time}, make_shared<F_fex_POAzeroRelVel>(), {from, to}, OT_eq, {1e1}, NoArr, 2, +1, +1);
  } else {
    addObjective({time}, make_shared<F_fex_ElasticVel>(elasticity, stickiness), {from, to}, OT_eq, {1e1}, NoArr, 2, +1, +1);
  }

  //regularization
  addObjective({time}, make_shared<F_fex_Force>(), {from, to}, OT_sos, {1e-4});
}

void KOMO_ext::setKS_slider(double time, double endTime, bool before, const char* obj, const char* slider, const char* table) {
  //disconnect object from grasp ref
//  setKinematicSwitch(time, before, "delete", nullptr, obj);

  //the two slider objects
  String slidera = STRING(slider <<'a');
  String sliderb = STRING(slider <<'b');

  Transformation rel = 0;
  rel.addRelativeTranslation(0., 0., .5*(shapeSize(world, obj) + shapeSize(world, table)));

//  setKinematicSwitch(time, true, "transXYPhiZero", table, slidera, rel);
//  setKinematicSwitch(time, true, "hingeZZero", sliderb, obj);
  addSwitch({time}, true, JT_transXYPhi, SWInit_zero, table, slidera, rel);
  addSwitch({time}, true, JT_hingeZ, SWInit_zero, sliderb, obj);

  addObjective({time, endTime}, make_shared<F_qZeroVel>(), {slidera}, OT_eq, {3e1}, NoArr, 1, +1, +0);
  addObjective({time, endTime}, make_shared<F_qZeroVel>(), {obj}, OT_eq, {3e1}, NoArr, 1, +1, -1);
  addObjective({time}, make_shared<F_LinAngVel>(), {obj}, OT_eq, {1e2}, NoArr, 1);

//  setKinematicSwitch(time, before, "sliderMechanism", table, obj, rel );

//  if(!actuated)
//    setKinematicSwitch(time, before, "hingeZZero", slider, obj, rel );
//  else
  //    setKinematicSwitch(time, before, "transXActuated", slider, obj, rel );
}

void KOMO_ext::setHoming(double startTime, double endTime, double prec, const char* keyword) {
  uintA bodies;
  Joint* j;
  for(Frame* f:world.frames) if((j=f->joint) && j->qDim()>0 && (!keyword || f->ats[keyword])) bodies.append(f->ID);
//  cout <<"HOMING: "; for(uint i:bodies) cout <<' ' <<world.frames(i)->name;  cout <<endl;
  addObjective({startTime, endTime}, make_shared<F_qItself>(bodies, true), {}, OT_sos, {prec}, NoArr); //world.q, prec);
}

//void KOMO_ext::setSquaredQAccelerations(double startTime, double endTime, double prec) {
//  CHECK_GE(k_order, 2,"");
//  addObjective({startTime, endTime}, make_shared<TM_Transition>(world), OT_sos, {}, NoArrprec);
//}

ptr<Objective> KOMO::add_qControlObjective(const arr& times, uint order, double scale, const arr& target, int deltaFromStep, int deltaToStep) {
  auto F = getCtrlFramesAndScale(world);
  scale *= sqrt(tau);

  CHECK_GE(k_order, order, "");
  ptr<Objective> o = addObjective(times, make_shared<F_qItself>(F.frames, (order==0)), {}, OT_sos, scale*F.scale, target, order, deltaFromStep, deltaToStep);
  return o;
}

void KOMO_ext::setSquaredQAccVelHoming(double startTime, double endTime, double accPrec, double velPrec, double homingPrec, int deltaFromStep, int deltaToStep) {
  auto F = getCtrlFramesAndScale(world);
  F.scale *= sqrt(tau);

  if(accPrec) {
    addObjective({startTime, endTime}, make_shared<F_qItself>(F.frames), {}, OT_sos, accPrec*F.scale, NoArr, 2, deltaFromStep, deltaToStep);
  }
  if(velPrec) {
    addObjective({startTime, endTime}, make_shared<F_qItself>(F.frames), {}, OT_sos, velPrec*F.scale, NoArr, 1, deltaFromStep, deltaToStep);
  }
  if(homingPrec) {
    addObjective({startTime, endTime}, make_shared<F_qItself>(F.frames, true), {}, OT_sos, {homingPrec*sqrt(tau)}, NoArr, 0, deltaFromStep, deltaToStep);
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

void KOMO::addSquaredQuaternionNorms(const arr& times, double scale) {
  addObjective(times, make_shared<F_qQuaternionNorms>(), {"ALL"}, OT_eq, {scale}, NoArr);
}

void KOMO_ext::setHoldStill(double startTime, double endTime, const char* shape, double prec) {
  Frame* s = world.getFrame(shape);
  addObjective({startTime, endTime}, make_shared<F_qItself>(TUP(s->ID)), {}, OT_sos, {prec}, NoArr, 1);
}

void KOMO_ext::setPosition(double startTime, double endTime, const char* shape, const char* shapeRel, ObjectiveType type, const arr& target, double prec) {
  addObjective({startTime, endTime}, make_shared<F_PositionDiff>(), {shape, shapeRel}, type, {prec}, target);
}

void KOMO_ext::setOrientation(double startTime, double endTime, const char* shape, const char* shapeRel, ObjectiveType type, const arr& target, double prec) {
//  setTask(startTime, endTime, new TM_Align(world, shape, shapeRel), type, target, prec);
  addObjective({startTime, endTime}, make_shared<F_QuaternionDiff>(), {shape, shapeRel}, type, {prec}, target);
}

void KOMO_ext::setVelocity(double startTime, double endTime, const char* shape, const char* shapeRel, ObjectiveType type, const arr& target, double prec) {
  addObjective({startTime, endTime}, make_shared<F_PositionDiff>(), {shape, shapeRel}, type, {prec}, target, 1);
}

void KOMO_ext::setLastTaskToBeVelocity() {
  objectives.last()->feat->order = 1; //set to be velocity!
}

void KOMO_ext::setImpact(double time, const char* a, const char* b) {
  add_touch(time, time, a, b);
  HALT("obsolete");
//  add_impulse(time, a, b);
}

void KOMO_ext::setOverTheEdge(double time, const char* object, const char* from, double margin) {
//  double negMargin = margin + .5*shapeSize(world, object, 0); //how much outside the bounding box?
  NIY;
//  addObjective({time, time+.5},
//               make_shared<F_Max>(make_shared<TM_AboveBox>(world, object, from, -negMargin), true), //this is the max selection -- only one of the four numbers need to be outside the BB
//               OT_ineq, {3e0}); //NOTE: usually this is an inequality constraint <0; here we say this should be zero for a negative margin (->outside support)
}

void KOMO_ext::setInertialMotion(double startTime, double endTime, const char* object, const char* base, double g, double c) {
  addSwitch({startTime}, true, JT_trans3, SWInit_zero, base, object);
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
//  addSwitch({time}, true, new KinematicSwitch(SW_effJoint, JT_free, endeffRef, object, world));
  addSwitch_stable(time, endTime, 0, endeffRef, object);
  addObjective({time}, make_shared<F_InsideBox>(), {endeffRef, object}, OT_ineq, {1e1});
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
  addSwitch({time}, true, JT_quatBall, SWInit_zero, endeffRef, object);
  HALT("deprecated"); //addSwitch({time}, true, "insert_transX", nullptr, object);
//  setTask(time, time,
//          new TM_LinTrans(
//              new TM_Default(TMT_posDiff, world, endeffRef, NoVector, object, NoVector),
//              arr(2,3,{0,1,0,0,0,1}), {}),
//          OT_eq, NoArr, 3e1);
  addObjective({time}, make_shared<F_InsideBox>(), {endeffRef, object}, OT_ineq, {1e1});

  if(stepsPerPhase>2) { //velocities down and up
    addObjective({time-timeToLift, time}, make_shared<F_Position>(), {endeffRef}, OT_sos, {3e0}, {0., 0., -.1}, 1); //move down
    addObjective({time, time+timeToLift}, make_shared<F_Position>(), {object}, OT_sos, {3e0}, {0., 0., .1}, 1); // move up
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
  addObjective({time}, FS_aboveBox, {object, placeRef}, OT_ineq, {1e1});

  //connect object to placeRef
#if 0
  Transformation rel = 0;
  rel.pos.set(0, 0, .5*(shapeSize(world, object) + shapeSize(world, placeRef)));
//  setKinematicSwitch(time, true, "transXYPhiZero", placeRef, object, rel );
  addSwitch({time}, true, new KinematicSwitch(SW_effJoint, JT_transXYPhi, placeRef, object, world, SWInit_zero, 0, rel));

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
  addSwitch({time}, true, JT_rigid, SWInit_zero, placeRef, object, relPose);

  if(stepsPerPhase>2) { //velocities down and up
    if(endeff) {
      addObjective({time-.15, time-.10}, make_shared<F_Position>(), {endeff}, OT_sos, {3e0}, {0., 0., -.1}, 1); //move down
      addObjective({time-.05, time+.05}, make_shared<F_Position>(), {endeff}, OT_sos, {1e1}, {0., 0., 0.}, 1); //hold still
      addObjective({time+.10, time+.15}, make_shared<F_Position>(), {endeff}, OT_sos, {3e0}, {0., 0., +.1}, 1); //move up
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

void KOMO_ext::setPush(double startTime, double endTime, const char* stick, const char* object, const char* table, int verbose) {
  if(verbose>0) cout <<"KOMO_setPush t=" <<startTime <<" stick=" <<stick <<" object=" <<object <<" table=" <<table <<endl;

#if 1
  //stick normal alignes with slider direction
  addObjective({startTime, endTime}, make_shared<F_ScalarProduct>(-Vector_y, Vector_x), {stick, "slider1b"}, OT_sos, {1e1}, {1.});
  //stick horizontal is orthogonal to world vertical
//  setTask(startTime, endTime, new TM_Default(TMT_vecAlign, world, stick, Vector_x, nullptr, Vector_z), OT_sos, {0.}, 1e1);
  add_touch(startTime, endTime, stick, table);

  double dist = .05; //.5*shapeSize(world, object, 0)+.01;
  addObjective({startTime, endTime}, make_shared<F_InsideBox>(), {"slider1b", stick}, OT_ineq);
  HALT("ivec = Vector(dist, .0, .0) is missing");
  Vector(dist, .0, .0),
//  setTask(startTime, endTime, new TM_Default(TMT_posDiff, world, stick, NoVector, "slider1b", {dist, .0, .0}), OT_sos, {}, 1e1);
#else
  setTouch(startTime, endTime, stick, object);
#endif

  setKS_slider(startTime, endTime, true, object, "slider1", table);

  addObjective({startTime, endTime-.1}, FS_aboveBox, {object, table}, OT_ineq, {1e1});

#if 0
  //connect object to placeRef
  Transformation rel = 0;
  rel.pos.set(0, 0, .5*(shapeSize(world, object) + shapeSize(world, table)));
  addSwitch({endTime}, true, "transXYPhiZero", table, object, rel);
//  auto *o = addObjective({startTime, endTime}, make_shared<TM_ZeroQVel>(world, object), OT_eq, {3e1}, NoArr, 1, +1);
//  o->prec(-1)=o->prec(-2)=0.;
#endif

  if(stepsPerPhase>2) { //velocities down and up
    addObjective({startTime-.3, startTime-.1}, make_shared<F_Position>(), {stick}, OT_sos, {3e0}, {0., 0., -.1}, 1); //move down
    addObjective({startTime-.05, startTime-.0}, make_shared<F_Position>(), {stick}, OT_sos, {3e0}, {0., 0., 0}, 1); //hold still
    addObjective({endTime+.0, endTime+.05}, make_shared<F_Position>(), {stick}, OT_sos, {3e0}, {0., 0., 0}, 1); //hold still
    addObjective({endTime+.1, endTime+.3}, make_shared<F_Position>(), {stick}, OT_sos, {3e0}, {0., 0., .1}, 1); // move up
  }
}

void KOMO_ext::setGraspSlide(double time, const char* endeff, const char* object, const char* placeRef, int verbose) {

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
  addSwitch({startTime}, true, JT_free, SWInit_zero, endeff, object);
  addObjective({time, endTime}, make_shared<F_qZeroVel>(), {object}, OT_eq, {3e1}, NoArr, 1, +1, -1);
  if(k_order>1) addObjective({time}, make_shared<F_LinAngVel>(), {object}, OT_eq, {1e2}, NoArr, 2, 0);
  else addObjective({time}, make_shared<F_NoJumpFromParent_OBSOLETE>(), {object}, OT_eq, {1e2}, NoArr, 1, 0, 0);

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
               make_shared<F_VectorDiff>(Vector_z, Vector_z), {object, placeRef}, OT_sos, {1e1});

//  if(stepsPerPhase>2){ //velocities down and up
//    setTask(startTime-.15, startTime, new TM_Default(TMT_pos, world, endeff), OT_sos, {0.,0.,-.1}, 3e0, 1); //move down
//    setTask(endTime, endTime+.15, new TM_Default(TMT_pos, world, endeff), OT_sos, {0.,0.,.1}, 3e0, 1); // move up
//  }
}

void KOMO_ext::setSlideAlong(double time, const char* stick, const char* object, const char* wall, int verbose) {
  if(verbose>0) cout <<"KOMO_setSlideAlong t=" <<time <<" obj=" <<object<<" wall=" <<wall <<endl;

  double endTime = time+1.;

  //stick normal alignes with slider direction
  addObjective({time, time+1.}, make_shared<F_ScalarProduct>(-Vector_y, Vector_x), {stick, object}, OT_sos, {1e0}, {1.});
  //stick horizontal is orthogonal to world vertical
  addObjective({time, time+1.}, make_shared<F_ScalarProduct>(Vector_x, Vector_z), {stick}, OT_sos, {1e1}, {0.});

//  double dist = .5*shapeSize(world, object, 0)+.01;
  addObjective({time, time+1.}, make_shared<F_InsideBox>(), {object, stick}, OT_ineq);
  HALT("ivec = Vector(dist, .0, .0), is missing");

  add_touch(time, time+1., stick, wall);

  //    //disconnect object from table
  //    setKinematicSwitch(time, true, "delete", nullptr, object);
  //    //connect graspRef with object
  //    setKinematicSwitch(startTime, true, "ballZero", endeff, object);

  Transformation rel = 0;
  rel.rot.setDeg(-90, {1, 0, 0});
  rel.pos.set(0, -.5*(shapeSize(world, wall, 1) - shapeSize(world, object)), +.5*(shapeSize(world, wall, 2) + shapeSize(world, object, 1)));
  addSwitch({time}, true, JT_transX, SWInit_zero, wall, object);
  HALT("deprecated")//addSwitch({time}, true, new KinematicSwitch(SW_insertEffJoint, JT_transZ, nullptr, object, world, SWInit_zero, 0, rel));
  //    setKinematicSwitch(time, true, "insert_trans3", nullptr, object);
  //    setTask(time, time, new TM_InsideBox(world, endeff, NoVector, object), OT_ineq, NoArr, 1e1);

  if(stepsPerPhase>2) { //velocities down and up
    addObjective({endTime+.0, endTime+.05}, make_shared<F_Position>(), {stick}, OT_sos, {3e0}, {0., 0., 0}, 1); //hold still
    addObjective({endTime+.1, endTime+.3}, make_shared<F_Position>(), {stick}, OT_sos, {3e0}, {0., 0., .05}, 1); // move up
  }
}

void KOMO_ext::setDropEdgeFixed(double time, const char* object, const char* to, const Transformation& relFrom, const Transformation& relTo, int verbose) {

  //disconnect object from anything
//  setKinematicSwitch(time, true, "delete", nullptr, object);

  //connect to world with lift
//  setKinematicSwitch(time, true, "JT_trans3", "world", object);

  addSwitch({time}, true, JT_hingeX, SWInit_zero, to, object, relFrom, relTo);
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
  addSwitch({time}, true, JT_rigid, SWInit_zero, object1, object2, rel);

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
    if(!hardConstrained) addObjective({startTime, endTime}, feat, {}, OT_sos, {prec}, NoArr, 1);
    else addObjective({startTime, endTime}, feat, {}, OT_eq, {prec}, NoArr, 1);
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
  addObjective({time+t1, 1.}, make_shared<F_Vector>(Vector_z), {endeff}, OT_sos, {1e0}, {0., 0., 1.});
  addObjective({time+t1, t1}, make_shared<F_PositionDiff>(), {endeff, object}, OT_sos, {3e1}, {0., 0., above+.1});
  addObjective({time+t1, 1.}, make_shared<F_ScalarProduct>(Vector_x, Vector_y), {endeff, object}, OT_sos, {3e0});
  addObjective({time+t1, 1.}, make_shared<F_ScalarProduct>(Vector_x, Vector_z), {endeff, object}, OT_sos, {3e0});
  //open gripper
  if(gripper)  addObjective({time+t1, .85}, make_shared<F_qItself>(F_qItself::byJointNames, StringA({gripper}), world), {}, OT_sos, {gripSize + .05});
  if(gripper2) addObjective({time+t1, .85}, make_shared<F_qItself>(F_qItself::byJointNames, StringA({gripper2}), world), {}, OT_sos, {::asin((gripSize + .05)/(2.*.10))});
  //lower
  addObjective({time+t2, 1.}, make_shared<F_PositionDiff>(), {endeff, object}, OT_sos, {3e1}, {0., 0., above});
  //close gripper
  if(gripper)  addObjective({time+t3, 1.}, make_shared<F_qItself>(F_qItself::byJointNames, StringA({gripper}), world), {}, OT_sos, {gripSize});
  if(gripper2) addObjective({time+t3, 1.}, make_shared<F_qItself>(F_qItself::byJointNames, StringA({gripper2}), world), {}, OT_sos, {::asin((gripSize)/(2.*.10))});
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

void KOMO::setSkeleton(const Skeleton& S) {
  //-- add objectives for mode switches
  intA switches = getSwitchesFromSkeleton(S, world);
  for(uint i=0; i<switches.d0; i++) {
    int j = switches(i, 0);
    int k = switches(i, 1);
    addSwitch_mode2({S(k).phase0, S(k).phase1}, S(k).symbol, S(k).frames, j<0);
  }
  //-- add objectives for rest
  for(const SkeletonEntry& s:S) {
    switch(s.symbol) {
      case SY_none:       HALT("should not be here");  break;
      case SY_end: break; //explicit redundant symbol, e.g. to mark the end of a skeleton
      case SY_initial: case SY_identical: case SY_noCollision:    break;
      case SY_touch:      addObjective({s.phase0, s.phase1}, FS_distance, {s.frames(0), s.frames(1)}, OT_eq, {1e2});  break;
      case SY_above:      addObjective({s.phase0, s.phase1}, FS_aboveBox, {s.frames(0), s.frames(1)}, OT_ineq, {1e1});  break;
      case SY_inside:     addObjective({s.phase0, s.phase1}, FS_insideBox, {s.frames(0), s.frames(1)}, OT_ineq, {1e1});  break;
//      case SY_inside:     addObjective({s.phase0, s.phase1}, make_shared<TM_InsideLine>(world, s.frames(0), s.frames(1)), OT_ineq, {1e1});  break;
      case SY_oppose:     addObjective({s.phase0, s.phase1}, FS_oppose, s.frames, OT_eq, {1e1});  break;
      case SY_impulse:    HALT("obsolete"); /*add_impulse(s.phase0, s.frames(0), s.frames(1));*/  break;

      case SY_topBoxGrasp: {
        addObjective({s.phase0}, FS_positionDiff, s.frames, OT_eq, {1e2});
        addObjective({s.phase0}, FS_scalarProductXX, s.frames, OT_eq, {1e2}, {0.});
        addObjective({s.phase0}, FS_vectorZ, {s.frames(0)}, OT_eq, {1e2}, {0., 0., 1.});
        //slow - down - up
        if(k_order>=2){
          addObjective({s.phase0}, FS_qItself, {}, OT_eq, {}, {}, 1);
          addObjective({s.phase0-.1,s.phase0+.1}, FS_position, {s.frames(0)}, OT_eq, {}, {0.,0.,.1}, 2);
        }
        break;
      }
      case SY_topBoxPlace: {
        addObjective({s.phase0}, FS_positionDiff, {s.frames(1), s.frames(2)}, OT_eq, {1e2}, {0,0,.08}); //arr({1,3},{0,0,1e2})
        addObjective({s.phase0}, FS_vectorZ, {s.frames(0)}, OT_eq, {1e2}, {0., 0., 1.});
        //slow - down - up
        if(k_order>=2){
          addObjective({s.phase0}, FS_qItself, {}, OT_eq, {}, {}, 1);
          addObjective({s.phase0-.1,s.phase0+.1}, FS_position, {s.frames(0)}, OT_eq, {}, {0.,0.,.1}, 2);
        }
        break;
      }

      case SY_touchBoxNormalX: {
        rai::Frame* box = world.getFrame(s.frames(1));
        CHECK(box, "");
        CHECK(box->shape && box->shape->type()==rai::ST_ssBox, "");
        double boxSize = shapeSize(world, s.frames(1), 0);
        addObjective({s.phase0}, FS_positionDiff, {s.frames(0), s.frames(1)}, OT_eq, {{1,3},{1e2,.0,.0}}, {.5*boxSize,0.,0.}); //arr({1,3},{0,0,1e2})
        addObjective({s.phase0}, FS_scalarProductXZ, {s.frames(1), s.frames(0)}, OT_eq, {1e2}, {1.});
//        addObjective({s.phase0}, FS_scalarProductYZ, {s.frames(1), s.frames(0)}, OT_eq, {1e2});
        break;
      }
      case SY_touchBoxNormalZ: {
        rai::Frame* box = world.getFrame(s.frames(1));
        CHECK(box, "");
        CHECK(box->shape && box->shape->type()==rai::ST_ssBox, "");
        double boxSize = shapeSize(world, s.frames(1), 2);
        addObjective({s.phase0}, FS_positionDiff, {s.frames(0), s.frames(1)}, OT_eq, {{1,3},{0.,0.,1e2}}, {0,0,.5*boxSize}); //arr({1,3},{0,0,1e2})
        addObjective({s.phase0}, FS_scalarProductZZ, {s.frames(1), s.frames(0)}, OT_eq, {1e2}, {1.});
//        addObjective({s.phase0}, FS_vectorZDiff, {s.frames(0), s.frames(1)}, OT_eq, {1e2});
        break;
      }

      case SY_makeFree:   world.makeObjectsFree(s.frames);  break;
      case SY_stableRelPose: addObjective({s.phase0, s.phase1+1.}, FS_poseRel, s.frames, OT_eq, {1e2}, {}, 1);  break;
      case SY_stablePose:  addObjective({s.phase0, s.phase1+1.}, FS_pose, s.frames, OT_eq, {1e2}, {}, 1);  break;
      case SY_poseEq: addObjective({s.phase0, s.phase1}, FS_poseDiff, s.frames, OT_eq, {1e2});  break;

      case SY_downUp:{
        if(k_order>=2){
          addObjective({s.phase0,s.phase1}, FS_position, {s.frames(0)}, OT_eq, {}, {0.,0.,.1}, 2, +1,+1);
        }
        break;
      }

      case SY_break:      addObjective({s.phase0, s.phase1}, make_shared<F_NoJumpFromParent_OBSOLETE>(), {s.frames(0)}, OT_eq, {1e2}, NoArr, 1, 0, 0);  break;

      case SY_contact:    addContact_slide(s.phase0, s.phase1, s.frames(0), s.frames(1));  break;
      case SY_contactStick:    addContact_stick(s.phase0, s.phase1, s.frames(0), s.frames(1));  break;
      case SY_contactComplementary: addContact_ComplementarySlide(s.phase0, s.phase1, s.frames(0), s.frames(1));  break;
      case SY_bounce:     addContact_elasticBounce(s.phase0, s.frames(0), s.frames(1), .9);  break;
      //case SY_contactComplementary:     addContact_Complementary(s.phase0, s.phase1, s.frames(0), s.frames(1));  break;

      case SY_dampMotion: {
        double sqrAccCost=1e-2, sqrVelCost=1e-2;
        if(sqrVelCost>0. && k_order>=1) {
          addObjective({s.phase0, s.phase1}, make_shared<F_LinAngVel>(), {s.frames(0)}, OT_sos, {sqrVelCost}, NoArr, 1);
        }
        if(sqrAccCost>0. && k_order>=2) {
          addObjective({s.phase0, s.phase1}, make_shared<F_LinAngVel>(), {s.frames(0)}, OT_sos, {sqrAccCost}, NoArr, 2);
        }
      } break;
      case SY_alignByInt: {
        addObjective({s.phase0, s.phase1}, FS_scalarProductXX, s.frames, OT_sos);  break;
        cout <<"THE INTEGER IS: " <<s.frames(2) <<endl;
      } break;

      case SY_push:       HALT("retired"); //setPush(s.phase0, s.phase1+1., s.frames(0), s.frames(1), s.frames(2), verbose);  break;//TODO: the +1. assumes pushes always have duration 1
      case SY_graspSlide: HALT("retired"); //setGraspSlide(s.phase0, s.frames(0), s.frames(1), s.frames(2), verbose);  break;
      //    else case SY_handover)              setHandover(s.phase0, s.frames(0), s.frames(1), s.frames(2), verbose);
      //    else LOG(-2) <<"UNKNOWN PREDICATE!: " <<s;

      case SY_forceBalance: {
        addObjective({s.phase0, s.phase1}, make_shared<F_TotalForce>(), {s.frames(0)}, OT_eq, {1e2});
      } break;

      //switches are handled above now
      case SY_stable:      //if(!ignoreSwitches) addSwitch_stable(s.phase0, s.phase1+1., s.frames(0), s.frames(1));  break;
      case SY_stableOn:    //if(!ignoreSwitches) addSwitch_stableOn(s.phase0, s.phase1+1., s.frames(0), s.frames(1));  break;
      case SY_dynamic:     //if(!ignoreSwitches) addSwitch_dynamic(s.phase0, s.phase1+1., "base", s.frames(0));  break;
      case SY_dynamicOn:   //if(!ignoreSwitches) addSwitch_dynamicOn(s.phase0, s.phase1+1., s.frames(0), s.frames(1));  break;
      case SY_dynamicTrans:   //if(!ignoreSwitches) addSwitch_dynamicTrans(s.phase0, s.phase1+1., "base", s.frames(0));  break;
      case SY_quasiStatic:
      case SY_quasiStaticOn:
      case SY_magicTrans: //addSwitch_magicTrans(s.phase0, s.phase1, world.frames.first()->name, s.frames(0), 0.);  break;
      case SY_free:
        break;
      case SY_magic:      addSwitch_magic(s.phase0, s.phase1, world.frames.first()->name, s.frames(0), 0., 0.);  break;
      default: HALT("undefined symbol: " <<s.symbol);
    }
  }
}

void KOMO::setSkeleton(const Skeleton& S, rai::ArgWord sequenceOrPath){
  if(sequenceOrPath==rai::_sequence){
    solver = rai::KS_dense;
  }else{
    solver = rai::KS_sparse;
  }

  double maxPhase = getMaxPhaseFromSkeleton(S);
  if(sequenceOrPath==rai::_sequence){
    setTiming(maxPhase, 1, 2., 1);
    add_qControlObjective({}, 1, 1e-1);
  }else{
    setTiming(maxPhase, 30, 5., 2);
    add_qControlObjective({}, 2, 1e0);
  }
  addSquaredQuaternionNorms();

  setSkeleton(S);
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
  addObjective({startTime, endTime}, make_shared<F_ScalarProduct>(Vector(whichAxis), Vector(whichAxisRel)), {shape, shapeRel}, type, {prec}, target);
#endif

}

void KOMO_ext::add_touch(double startTime, double endTime, const char* shape1, const char* shape2, ObjectiveType type, const arr& target, double prec) {
  addObjective({startTime, endTime}, FS_pairCollision_negScalar, {shape1, shape2}, type, {prec}, target);
}

void KOMO_ext::add_aboveBox(double startTime, double endTime, const char* shape1, const char* shape2, double prec) {
  addObjective({startTime, endTime}, FS_aboveBox, {shape1, shape2}, OT_ineq, {prec}, NoArr);
}

void KOMO_ext::add_insideBox(double startTime, double endTime, const char* shape1, const char* shape2, double prec) {
  addObjective({startTime, endTime}, make_shared<F_InsideBox>(), {shape1, shape2}, OT_ineq, {prec}, NoArr);
}

//void KOMO::add_impulse(double time, const char* shape1, const char* shape2, ObjectiveType type, double prec) {
////    setTask(time, time, new TM_ImpulsExchange(world, a, b), OT_sos, {}, 3e1, 2, +1); //+1 deltaStep indicates moved 1 time slot backward (to cover switch)
//  if(k_order>=2) {
//    addObjective({time}, make_shared<TM_ImpulsExchange>(world, shape1, shape2), type, {}, prec, 2, +1, +1); //+1 deltaStep indicates moved 1 time slot backward (to cover switch)
//    addFlag(time, new Flag(FL_impulseExchange, world[shape1]->ID), +0);
//    addFlag(time, new Flag(FL_impulseExchange, world[shape2]->ID), +0);
//  }
//}

void KOMO_ext::add_stable(double time, const char* shape1, const char* shape2, ObjectiveType type, double prec) {
  addObjective({time}, make_shared<F_PoseRel>(), {shape1, shape2}, type, {prec}, NoArr, 1, 0);
}

void KOMO_ext::setAlignedStacking(double time, const char* object, ObjectiveType type, double prec) {
  HALT("obsolete");
//  addObjective({time}, make_shared<TM_AlignStacking>(world, object), type, NoArr, prec);
}

void KOMO::add_collision(bool hardConstraint, double margin, double prec) {
  if(hardConstraint) { //interpreted as hard constraint (default)
    addObjective({}, make_shared<F_AccumulatedCollisions>(margin), {"ALL"}, OT_eq, {prec}, NoArr);
  } else { //cost term
    addObjective({}, make_shared<F_AccumulatedCollisions>(margin), {"ALL"}, OT_sos, {prec}, NoArr);
  }
}

void KOMO::add_jointLimits(bool hardConstraint, double margin, double prec) {
  if(hardConstraint) { //interpreted as hard constraint (default)
    addObjective({}, make_shared<F_qLimits>(), {}, OT_ineq, {-prec}, NoArr);
  } else { //cost term
    NIY;
//    setTask(0., -1., new TM_Proxy(TMT_allP, {}, margin), OT_sos, NoArr, prec);
  }
}

void KOMO::setLiftDownUp(double time, const char* endeff, double timeToLift) {
  if(stepsPerPhase>2 && timeToLift>0.) { //velocities down and up
    addObjective({time-timeToLift, time-.5*timeToLift}, make_shared<F_Position>(), {endeff}, OT_sos, {1e0}, {0., 0., -.2}, 1); //move down
//    addObjective({time-timeToLift/3,  time+timeToLift/3}, make_shared<TM_Default>(TMT_pos, world, endeff), OT_sos, {3e0}, {0.,0.,0.}, 1); //move down
    addObjective({time+.5*timeToLift, time+timeToLift}, make_shared<F_Position>(), {endeff}, OT_sos, {1e0}, {0., 0., .2}, 1); // move up
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
  solver = rai::KS_dense;
//  stepsPerPhase = 1;
//  T = 1;
//  tau = 1.;
//  k_order = 1;
  setTiming(1., 1, 1., 1);
//  setSquaredQVelocities(0.,-1.,1e-1);
  add_qControlObjective({}, 1, 1e-1);
//  setSquaredQAccVelHoming(0., -1., 0., 1e-1, 1e-2);
  addSquaredQuaternionNorms();
}

void KOMO::setDiscreteOpt(uint k) {
  solver = rai::KS_sparse;
//  stepsPerPhase = 1;
//  T = k;
//  tau = 1.;
//  k_order = 1;
  setTiming(k, 1, 1., 1);
  addSquaredQuaternionNorms();
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
    t->feat->order=2; //make this an acceleration task!
  } else {
    t->feat->order=1; //make this a velocity task!
  }
  t->setCostSpecs(0, MP.T-1, {0.}, 1e0);

  if(timeSteps!=0) {
    t = MP.addTask("final_vel", new TM_qItself(), OT_sos);
    t->feat->order=1; //make this a velocity task!
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

void KOMO::setConfiguration(int t, const arr& q) {
#ifdef KOMO_PATH_CONFIG
  FrameL F;
  for(auto* f:timeSlices[k_order+t]) if(f->joint) F.append(f);
  pathConfig.setJointState(q, F, false);
#else
  if(!configurations.N) setupConfigurations();
  if(t<0) CHECK_LE(-t, (int)k_order, "");
  configurations(t+k_order)->setJointState(q);
#endif
}

void KOMO::setConfiguration_X(int t, const arr& X) {
  pathConfig.setFrameState(X, timeSlices[k_order+t]);
}

arr KOMO::getConfiguration_q(int t) {
#ifdef KOMO_PATH_CONFIG
  FrameL F;
  for(auto* f:timeSlices[k_order+t]) if(f->joint) F.append(f);
  return pathConfig.getJointState(F, false);
#else
  if(!configurations.N) setupConfigurations();
  if(t<0) CHECK_LE(-t, (int)k_order, "");
  return configurations(t+k_order)->getJointState();
#endif
}

arr KOMO::getFrameState(int t) {
#ifdef KOMO_PATH_CONFIG
  return pathConfig.getFrameState(timeSlices[k_order+t]);
#else
  NIY;
#endif
}

arr KOMO::getPath_q(int t) {
#ifdef KOMO_PATH_CONFIG
  uintA F = jointsToIndices(world.activeJoints);
  F += timeSlices(k_order+t,0)->ID;
  return pathConfig.getJointState(F);
//  return pathConfig.getJointState(timeSlices[k_order+t]);
#else
  NIY;
#endif
}

arr KOMO::getPath(uintA joints, const bool activesOnly){
  if(!joints.N) joints = jointsToIndices( world.activeJoints );
  arr q = pathConfig.getJointState(joints+timeSlices(0+k_order,0)->ID, activesOnly);
  q.resizeCopy(T, q.N);
  for(uint t=1; t<T; t++) {
    q[t] = pathConfig.getJointState(joints+timeSlices(t+k_order,0)->ID, activesOnly);
  }
  return q;
}

arr KOMO::getPath_frames(const uintA& frames) {
#ifdef KOMO_PATH_CONFIG
  CHECK(!frames.N, "NIY");
  arr X(T, timeSlices.d1, 7);
  for(uint t=0; t<T; t++) {
    X[t] = pathConfig.getFrameState(timeSlices[k_order+t]);
  }
#else
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");
  arr X(T, frames.N, 7);
  for(uint t=0; t<T; t++) {
    for(uint i=0; i<frames.N; i++) {
      X(t, i, {}) = configurations(t+k_order)->frames(frames(i))->ensure_X().getArr7d();
    }
  }
#endif
  return X;
}

arrA KOMO::getPath_q() {
  arrA q(T);
  for(uint t=0; t<T; t++) {
#ifdef KOMO_PATH_CONFIG
    FrameL F;
    for(auto* f:timeSlices[k_order+t]) if(f->joint && f->joint->active) F.append(f);
    q(t) = pathConfig.getJointState(F, true);
#else
    CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");
    q(t) = configurations(t+k_order)->getJointState();
#endif
  }
  return q;
}

arr KOMO::getPath_tau() {
  bool hasTau = world.hasTauJoint();
  if(!hasTau) return consts<double>(tau, T);
  arr X(T);
  for(uint t=0; t<T; t++) {
    rai::Frame *first = timeSlices(t+k_order,0);
    CHECK(first->joint && first->joint->type==JT_tau, "");
    X(t) = first->tau;
  }
  return X;
}

arr KOMO::getPath_times() {
  arr tau = getPath_tau();
  return integral(tau);
}

arr KOMO::getPath_energies() {
  F_Energy E;
  E.order=1;
  arr X(T), y;
  for(uint t=0; t<T; t++) {
    NIY//    E.phi2(y, NoArr, cat(configurations(t+k_order-1)->frames, configurations(t+k_order)->frames).reshape(2,-1));
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

void KOMO::initWithConstant(const arr& q) {
  for(uint t=0; t<T; t++) {
    setConfiguration(t, q);
  }

  run_prepare(0.);
}

void KOMO::initWithWaypoints(const arrA& waypoints, uint waypointStepsPerPhase, bool sineProfile) {
  //compute in which steps (configuration time slices) the waypoints are imposed
  uintA steps(waypoints.N);
  for(uint i=0; i<steps.N; i++) {
    steps(i) = conv_time2step(conv_step2time(i, waypointStepsPerPhase), stepsPerPhase);
  }

//  view(true, STRING("before"));

  //first set the path piece-wise CONSTANT at waypoints and the subsequent steps (each waypoint may have different dimension!...)
#if 1 //depends on sw->isStable -> mimic !!
  for(uint i=0; i<steps.N; i++) {
    uint Tstop=T;
    if(i+1<steps.N && steps(i+1)<T) Tstop=steps(i+1);
    for(uint t=steps(i); t<Tstop; t++) {
      setConfiguration(t, waypoints(i));
    }
  }
#else
  for(uint i=0; i<steps.N; i++) {
    if(steps(i)<T) setConfiguration(steps(i), waypoints(i));
  }
#endif

//  view(true, STRING("after"));

  //then interpolate w.r.t. non-switching frames within the intervals
#if 1
  for(uint i=0; i<steps.N; i++) {
    uint i1=steps(i);
    uint i0=0; if(i) i0 = steps(i-1);
    //motion profile
    if(i1-1<T) {
#ifdef KOMO_PATH_CONFIG
      uintA nonSwitched = getNonSwitchedFrames(timeSlices[k_order+i0], timeSlices[k_order+i1]);
      arr q0 = pathConfig.getJointState(timeSlices[k_order+i0].sub(nonSwitched), false);
      arr q1 = pathConfig.getJointState(timeSlices[k_order+i1].sub(nonSwitched), false);
#else
      uintA nonSwitched = getNonSwitchedFrames({configurations(k_order+j), configurations(k_order+i1)});
      arr q0 = configurations(k_order+j)->getJointState(nonSwitched);
      arr q1 = configurations(k_order+i1)->getJointState(nonSwitched);
#endif
      for(uint j=i0+1; j<i1; j++) {
        arr q;
        double phase = double(j-i0)/double(i1-i0);
        if(sineProfile) {
          q = q0 + (.5*(1.-cos(RAI_PI*phase))) * (q1-q0);
        } else {
          q = q0 + phase * (q1-q0);
        }
#ifdef KOMO_PATH_CONFIG
        pathConfig.setJointState(q, timeSlices[k_order+j].sub(nonSwitched), false);
#else
        configurations(k_order+j)->setJointState(q, nonSwitched);
#endif
//        view(true, STRING("interpolating: step:" <<i <<" t: " <<j));
      }
    }/*else{
      for(uint j=i0+1;j<T;j++){
        configurations(k_order+j)->setJointState(q0, nonSwitched);
      }
    }*/
  }
#endif

  run_prepare(0.);
}

void KOMO::reset() {
  dual.clear();
  featureValues.clear();
  featureJacobians.clear();
  featureTypes.clear();
}

void KOMO::optimize(double addInitializationNoise, const OptOptions options) {
  run_prepare(addInitializationNoise);

  if(verbose>0) reportProblem();

  run(options);
}

void KOMO::run_prepare(double addInitializationNoise) {
  //ensure the configurations are setup
#ifndef KOMO_PATH_CONFIG
  if(!configurations.N) setupConfigurations();
  CHECK_EQ(configurations.N, k_order+T, "");
  for(uint s=0; s<k_order; s++) configurations(s)->ensure_q(); //also the prefix!
#else
  if(!timeSlices.nd) setupConfigurations2();
  if(!switchesWereApplied) retrospectApplySwitches2();
#endif

  //ensure the decision variable is in sync from the configurations
  x = pathConfig.getJointState();

  //add noise
  if(addInitializationNoise>0.) {
    rndGauss(x, addInitializationNoise, true); //don't initialize at a singular config
  }
}

void KOMO::run(OptOptions options) {
  Configuration::setJointStateCount=0;
  if(verbose>0) {
    cout <<"** KOMO::run solver:"
        <<rai::Enum<KOMOsolver>(solver)
       <<" collisions:" <<computeCollisions
      <<" x-dim:" <<x.N
      <<" T:" <<T <<" k:" <<k_order <<" phases:" <<double(T)/stepsPerPhase <<" stepsPerPhase:" <<stepsPerPhase <<" tau:" <<tau;
    #ifdef KOMO_PATH_CONFIG
    cout <<"  #timeSlices:" <<timeSlices.d0 <<" #totalDOFs:" <<pathConfig.getJointStateDimension() <<" #frames:" <<pathConfig.frames.N;
    #else
    cout <<" #config:" <<configurations.N <<" q-dims: ";
    uintA dims(configurations.N);
    for(uint i=0; i<configurations.N; i++) dims(i)=configurations(i)->q.N;
    writeConsecutiveConstant(cout, dims);
    #endif
    cout <<endl;
  }

  options.verbose = rai::MAX(verbose-2, 0);
  double timeZero = rai::realTime();
  CHECK(T, "");
  if(logFile)(*logFile) <<"KOMO_run_log: [" <<endl;

  if(solver==rai::KS_none) {
    HALT("you need to choose a KOMO solver");

  } else if(solver==rai::KS_dense || solver==rai::KS_sparse) {
    Conv_KOMO_SparseNonfactored P(*this, solver==rai::KS_sparse);
    OptConstrained _opt(x, dual, P, options, logFile);
    _opt.run();
    timeNewton += _opt.newton.timeNewton;

  } else if(solver==rai::KS_sparseFactored) {
    Conv_KOMO_SparseNonfactored P(*this, true);
    OptConstrained _opt(x, dual, P, options, logFile);
    _opt.run();
    timeNewton += _opt.newton.timeNewton;

  } else if(solver==rai::KS_banded) {
    pathConfig.jacMode = rai::Configuration::JM_rowShifted;
    Conv_KOMO_FactoredNLP P(*this);
    Conv_FactoredNLP_BandedNLP C(P, 0);
    C.maxBandSize = (k_order+1)*max(C.variableDimensions);
    OptConstrained opt(x, dual, C, options, logFile);
    opt.run();

  } else if(solver==rai::KS_NLopt) {
    Conv_KOMO_SparseNonfactored P(*this, false);
    NLoptInterface nlopt(P);
    x = nlopt.solve();
    set_x2(x);

  } else if(solver==rai::KS_Ipopt) {
    Conv_KOMO_SparseNonfactored P(*this, false);
    IpoptInterface ipopt(P);
    x = ipopt.solve();
    set_x2(x);

  } else if(solver==rai::KS_Ceres) {
    Conv_KOMO_SparseNonfactored P(*this, false);
//    Conv_KOMO_FactoredNLP P(*this);
    LagrangianProblem L(P, options);
    Conv_MathematicalProgram_TrivialFactoreded P2(L);
    CeresInterface ceres(P2);
    x = ceres.solve();
    set_x2(x);

  } else NIY;

  runTime = rai::realTime() - timeZero;
  if(logFile)(*logFile) <<"\n] #end of KOMO_run_log" <<endl;
  if(verbose>0) {
    cout <<"** optimization time=" <<runTime
         <<" (kin:" <<timeKinematics <<" coll:" <<timeCollisions <<" feat:" <<timeFeatures <<" newton: " <<timeNewton <<")"
         <<" setPathConfigCount: " <<set_xCount <<" setJointStateCount=" <<Configuration::setJointStateCount <<endl;
  }
  if(verbose>0) cout <<getReport(verbose>1) <<endl;
}

void KOMO::reportProblem(std::ostream& os) {
  os <<"KOMO Problem:" <<endl;
  os <<"  x-dim:" <<x.N <<"  dual-dim:" <<dual.N <<endl;
  os <<"  T:" <<T <<" k:" <<k_order <<" phases:" <<double(T)/stepsPerPhase <<" stepsPerPhase:" <<stepsPerPhase <<" tau:" <<tau <<endl;
  os <<"  #timeSlices:" <<timeSlices.d0 <<" #totalDOFs:" <<pathConfig.getJointStateDimension() <<" #frames:" <<pathConfig.frames.N;
  os <<"  #pathQueries:" <<pathConfig.setJointStateCount;
  os <<endl;

  arr times = getPath_times();
  if(times.N>10) times.resizeCopy(10);
  os <<"    times:" <<times <<endl;

  os <<"  computeCollisions:" <<computeCollisions <<endl;
  for(ptr<Objective>& t:objectives) os <<"    " <<*t <<endl;
  for(KinematicSwitch* sw:switches) {
    os <<"    ";
    if(sw->timeOfApplication+k_order >= timeSlices.d0) {
//      LOG(-1) <<"switch time " <<sw->timeOfApplication <<" is beyond time horizon " <<T;
      sw->write(os, {});
    } else {
      sw->write(os, timeSlices[sw->timeOfApplication+k_order]);
    }
    os <<endl;
  }

  if(verbose>6){
    os <<"  INITIAL STATE" <<endl;
#ifdef KOMO_PATH_CONFIG
    for(rai::Frame* f:pathConfig.frames){
#else
    for(auto *C:configurations) for(rai::Frame* f:C->frames){
#endif
      if(f->joint && f->joint->dim) os <<"    " <<f->name <<" [" <<f->joint->type <<"] : " <<f->joint->calc_q_from_Q(f->get_Q()) /*<<" - " <<pathConfig.q.elem(f->joint->qIndex)*/ <<endl;
      for(auto *ex:f->forces) os <<"    " <<f->name <<" [force " <<ex->a.name <<'-' <<ex->b.name <<"] : " <<ex->force /*<<' ' <<ex->torque*/ <<' ' <<ex->poa <<endl;
    }
  }
}

void KOMO::checkGradients() {
  CHECK(T, "");
#if 0
  checkJacobianCP(Convert(komo_problem), x, 1e-4);
#else
  double tolerance=1e-4;

  ptr<MathematicalProgram_Factored> SP;
  ptr<MathematicalProgram> CP;

  if(solver==rai::KS_none) {
    NIY;
  } else if(solver==rai::KS_banded) {
    SP = make_shared<Conv_KOMO_FactoredNLP>(*this);
    auto BP = make_shared<Conv_FactoredNLP_BandedNLP>(*SP, 0);
    BP->maxBandSize = (k_order+1)*max(BP->variableDimensions);
    CP = BP;
  } else if(solver==rai::KS_sparseFactored) {
    SP = make_shared<Conv_KOMO_FactoredNLP>(*this);
    CP = make_shared<Conv_FactoredNLP_BandedNLP>(*SP, 0, true);
  } else {
    CP = make_shared<Conv_KOMO_SparseNonfactored>(*this, solver==rai::KS_sparse);
  }

  VectorFunction F = [CP](arr& phi, arr& J, const arr& x) {
    return CP->evaluate(phi, J, x);
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
      if(featureNames.N) {
        LOG(-1) <<"FAILURE in line " <<i <<" t=" <</*CP_komo.featureTimes(i) <<*/' ' <<featureNames(i) <<" -- max diff=" <<md <<" |"<<J(i, j)<<'-'<<JJ(i, j)<<"| (stored in files z.J_*)";
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
}

int KOMO::view(bool pause, const char* txt){ pathConfig.gl()->recopyMeshes(pathConfig); return pathConfig.watch(pause, txt); }

int KOMO::view_play(bool pause, double delay, const char* saveVideoPath){ pathConfig.gl()->recopyMeshes(pathConfig); return pathConfig.gl()->playVideo(timeSlices.d0, timeSlices.d1, pause, delay*tau*T, saveVideoPath); }

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


//===========================================================================

void KOMO::retrospectApplySwitches2() {
#ifdef KOMO_PATH_CONFIG
  for(KinematicSwitch* sw:switches) {
    int s = sw->timeOfApplication+(int)k_order;
    if(s<0) s=0;
    int sEnd = int(k_order+T);
//    if(sw->timeOfTermination>=0)  sEnd = sw->timeOfTermination+(int)k_order;
    CHECK(sEnd>s, "");
    rai::Frame *f0=0;
    for(; s<sEnd; s++) { //apply switch on all configurations!
      rai::Frame* f = sw->apply(timeSlices[s]());
      if(!f0){
        f0=f;
      } else {
        if(sw->symbol==SW_addContact){
          rai::ForceExchange* ex0 = f0->forces.last();
          rai::ForceExchange* ex1 = f->forces.last();
          ex1->poa = ex0->poa;
        }else{
          f->set_Q() = f0->get_Q(); //copy the relative pose (switch joint initialization) from the first application
//          /*CRUCIAL CHANGE!*/ if(sw->isStable)  f->joint->setMimic(f0->joint);
        }
      }
    }
  }
  switchesWereApplied=true;
#endif
}

void KOMO::retrospectChangeJointType(int startStep, int endStep, uint frameID, JointType newJointType) {
  uint s = startStep+k_order;
  //apply the same switch on all following configurations!
  for(; s<endStep+k_order; s++) {
    rai::Frame* f = timeSlices(s, frameID);
    f->setJoint(newJointType);
  }
}

void KOMO::selectJointsBySubtrees(const StringA& roots, const arr& times, bool notThose){
  uintA rootIds = world.getFrameIDs(roots);

  world.selectJointsBySubtrees(world.getFrames(rootIds), notThose);

  FrameL allRoots;

  if(!times.N) {
    for(uint s=0;s<timeSlices.d0;s++) allRoots.append( pathConfig.getFrames(rootIds+s*timeSlices.d1) );
  } else {
    int tfrom = conv_time2step(times(0), stepsPerPhase);
    int tto   = conv_time2step(times(1), stepsPerPhase);
    for(int t=tfrom;t<=tto;t++) allRoots.append( pathConfig.getFrames(rootIds+(t+k_order)*timeSlices.d1) );
  }
  pathConfig.selectJointsBySubtrees(allRoots, notThose);
  pathConfig.ensure_q();
  pathConfig.checkConsistency();

}


//===========================================================================

void KOMO::setupConfigurations2() {
  //IMPORTANT: The configurations need to include the k prefix configurations!
  //Therefore configurations(0) is for time=-k and configurations(k+t) is for time=t
  CHECK(timeSlices.d0 != k_order+T, "why setup again?");
  CHECK(!pathConfig.frames.N, "why setup again?");

  computeMeshNormals(world.frames, true);
  computeMeshGraphs(world.frames, true);

  rai::Configuration C;
  C.copy(world, true);
  C.setTaus(tau);

  if(computeCollisions) {
    CHECK(!fcl, "");
    CHECK(!swift, "");
#ifndef FCLmode
    swift = C.swift();
#else
    fcl = C.fcl();
#endif
  }

  for(uint s=0;s<k_order+T;s++) {
//    for(KinematicSwitch* sw:switches) { //apply potential switches
//      if(sw->timeOfApplication+(int)k_order==(int)s)  sw->apply(C.frames);
//    }

//    uint nBefore = pathConfig.frames.N;
    pathConfig.addCopies(C.frames, C.forces);
//    timeSlices[s] = pathConfig.frames({nBefore, -1});

  }
  timeSlices = pathConfig.frames;

  //select only the non-prefix joints as active, and only joints that have previously been active
  FrameL activeJoints;
  for(uint t=0; t<T; ++t){
    for(auto* f:timeSlices[t + k_order]) {
      if(f->joint && f->joint->active){
        activeJoints.append(f);
      }
    }
  }
  pathConfig.selectJoints(activeJoints); 

  pathConfig.ensure_q();
  pathConfig.checkConsistency();
}

void KOMO::setBounds() {
#ifndef KOMO_PATH_CONFIG
  if(!configurations.N) setupConfigurations();
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");

  arr x = pathConfig.getJointState();
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
#else
  arr limits = ~pathConfig.getLimits();
  bound_lo = limits[0];
  bound_up = limits[1];
#endif
}

void KOMO::checkBounds(const arr& x) {
  CHECK_EQ(x.N, bound_lo.N, "");
  CHECK_EQ(x.N, bound_up.N, "");

  for(uint i=0; i<x.N; i++) if(bound_up.elem(i)>bound_lo.elem(i)) {
      if(x.elem(i)<bound_lo.elem(i)) cout <<"lower bound violation: x_" <<i <<"=" <<x.elem(i) <<" lo_" <<i <<"=" <<bound_lo.elem(i) <<endl;
      if(x.elem(i)>bound_up.elem(i)) cout <<"lower upper violation: x_" <<i <<"=" <<x.elem(i) <<" up_" <<i <<"=" <<bound_up.elem(i) <<endl;
    }

}

//===========================================================================

void reportAfterPhiComputation(KOMO& komo) {
  if(komo.verbose>6 || komo.animateOptimization>2) {
    //  komo.reportProxies();
    cout <<komo.getReport(true) <<endl;
  }
  if(komo.animateOptimization>0) {
    komo.view(komo.animateOptimization>1, "optAnim");
    //  komo.plotPhaseTrajectory();
    //  rai::wait();
    //  reportProxies();
  }
}


void KOMO::set_x2(const arr& x, const uintA& selectedConfigurationsOnly) {
  CHECK_EQ(timeSlices.d0, k_order+T, "configurations are not setup yet");

  rai::timerRead(true);

  if(!selectedConfigurationsOnly.N){
    pathConfig.setJointState(x);
  }else{
    pathConfig.setJointState(x, timeSlices.sub(selectedConfigurationsOnly+k_order));
    HALT("this is untested...");
  }

  timeKinematics += rai::timerRead(true);
  if(computeCollisions) {
    pathConfig.proxies.clear();
    arr X;
    uintA collisionPairs;
    for(uint s=k_order;s<timeSlices.d0;s++){
      X = pathConfig.getFrameState(timeSlices[s]);
#ifndef FCLmode
      collisionPairs = swift->step(X);
#else
      fcl->step(X);
      collisionPairs = fcl->collisions;
#endif
      collisionPairs += timeSlices.d1 * s; //fcl returns frame IDs related to 'world' -> map them into frameIDs within that time slice
      pathConfig.addProxies(collisionPairs);
    }
  }
  timeCollisions += rai::timerRead(true);
}

#if 0
void KOMO::setState(const arr& x, const uintA& selectedVariablesOnly) {
  if(!configurations.N) setupConfigurations();
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");

  if(!!selectedVariablesOnly) { //we only set some configurations: those listed in selectedConfigurationsOnly
    CHECK(selectedVariablesOnly.isSorted(), "");
    uint selCount=0;
    uint vCount=0;
    for(uint t=0; t<T; t++) {
      uint s = t+k_order;
      uint n = configurations(s)->vars_getNum();
      for(uint i=0; i<n; i++) {
        if(vCount == selectedVariablesOnly(selCount)) {
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
    for(uint t=0; t<T; t++) {
      uint s = t+k_order;
      uint n = configurations(s)->vars_getNum();
      for(uint i=0; i<n; i++) {
        configurations(s)->vars_activate(i);
      }
    }
  }

  //-- set the configurations' states
  uint x_count=0;
  for(uint t=0; t<T; t++) {
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
    if(animateOptimization>1) {
//      if(animateOptimization>2)
//        cout <<getReport(true) <<endl;
      displayPath(true);
    } else {
      displayPath(false);
    }
//    komo.plotPhaseTrajectory();
//    rai::wait();
  }
}
#endif


rai::Graph KOMO::getReport(bool gnuplt, int reportFeatures, std::ostream& featuresOs) {
  //-- collect all task costs and constraints
  StringA name; name.resize(objectives.N);
  arr err=zeros(T, objectives.N);
  arr dualSolution; if(dual.N) dualSolution=zeros(T, objectives.N);
  arr taskC=zeros(objectives.N);
  arr taskG=zeros(objectives.N);
  arr taskH=zeros(objectives.N);
  arr taskF=zeros(objectives.N);
  uint M=0;
#ifndef KOMO_PATH_CONFIG
  for(uint i=0; i<objectives.N; i++) {
    ptr<Objective> ob = objectives.elem(i);
    for(uint l=0; l<ob->configs.d0; l++) {
      ConfigurationL Ktuple = configurations.sub(convert<uint, int>(ob->configs[l]+(int)k_order));
      uint d=ob->feat->__dim_phi(Ktuple);
      uint time=ob->configs(l, -1);
      //        if(wasRun) {
#else
  for(ptr<GroundedObjective>& ob:objs) {
    uint d = ob->feat->__dim_phi2(ob->frames);
    int i = ob->objId;
    uint time = ob->configs.last();
#endif
    //          for(uint j=0; j<d; j++) CHECK_EQ(featureTypes(M+j), ob->type, "");
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
      if(ob->type==OT_f) {
        for(uint j=0; j<d; j++) err(time, i) += featureValues(M+j);
        taskF(i) += err(time, i);
      }
      M += d;
    }
    //        }
    if(reportFeatures==1) {
      featuresOs <<std::setw(4) <<time <<' ' <<std::setw(2) <<i <<' ' <<std::setw(2) <<d <<ob->configs
                <<' ' <<std::setw(40) <<typeid(*ob->feat).name()
               <<" k=" <<ob->feat->order <<" ot=" <<ob->type <<" prec=" <<std::setw(4) <<ob->feat->scale;
      if(ob->feat->target.N<5) featuresOs <<" y*=[" <<ob->feat->target <<']'; else featuresOs<<"y*=[..]";
      featuresOs <<" y^2=" <<err(time, i) <<endl;
    }
#ifndef KOMO_PATH_CONFIG
  }
#endif
  }
CHECK_EQ(M, featureValues.N, "");

  //-- generate a report graph
  rai::Graph report;
  double totalC=0., totalG=0., totalH=0., totalF=0.;
  for(uint i=0; i<objectives.N; i++) {
    ptr<Objective> c = objectives(i);
    Graph& g = report.newSubgraph({c->name}, {});
    g.newNode<double>({"order"}, {}, c->feat->order);
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
  g.newNode<bool>({"computeCollisions"}, {}, computeCollisions);
#endif

  if(includeSolution) {
    //full configuration paths
    g.newNode<arr>({"X"}, {}, getPath_frames());
    g.newNode<arrA>({"x"}, {}, getPath_q());
    g.newNode<arr>({"dual"}, {}, dual);
  }

  //objectives
#ifndef KOMO_PATH_CONFIG
  for(ptr<Objective>& ob : objectives) {
#else
  for(ptr<GroundedObjective>& ob:objs) {
#endif


    Graph& g = K.newSubgraph({ob->feat->shortTag(pathConfig)});
    g.newNode<double>({"order"}, {}, ob->feat->order);
    g.newNode<String>({"type"}, {}, STRING(ob->type));
    g.newNode<String>({"feature"}, {}, ob->feat->shortTag(pathConfig));
    if(ob->configs.N) g.newNode<intA>({"vars"}, {}, ob->configs);
//    g.copy(task->feat->getSpec(world), true);
    if(includeValues) {
      arr y, Jy;
      arrA V, J;
      for(uint l=0; l<ob->configs.d0; l++) {
//        ConfigurationL Ktuple = configurations.sub(convert<uint, int>(ob->configs[l]+(int)k_order));
//        ob->feat->__phi(y, Jy, Ktuple);
        ob->feat->__phi2(y, Jy, ob->frames);
        if(isSpecial(Jy)) Jy = unpack(Jy);

        V.append(y);
        J.append(Jy);
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


void KOMO::Conv_KOMO_SparseNonfactored::evaluate(arr& phi, arr& J, const arr& x) {
  //-- set the trajectory
  komo.set_x2(x);
  if(sparse){
    komo.pathConfig.jacMode = rai::Configuration::JM_sparse;
  }else {
    komo.pathConfig.jacMode = rai::Configuration::JM_dense;
  }

  if(!dimPhi) getDimPhi();

  phi.resize(dimPhi);
  if(!!J) {
    if(sparse) {
      J.sparse().resize(dimPhi, x.N, 0);
    } else {
      J.resize(dimPhi, x.N).setZero();
    }
  }

  uint M=0;
#ifdef KOMO_PATH_CONFIG
  for(ptr<GroundedObjective>& ob : komo.objs) {
#else
  uintA x_index = getKtupleDim(komo.configurations({komo.k_order, -1}));
  x_index.prepend(0);

  for(ptr<Objective>& ob : komo.objectives) {
    for(uint l=0; l<ob->configs.d0; l++) {
      intA configs = ob->configs[l];
      ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(configs+(int)komo.k_order));
      uintA kdim = getKtupleDim(Ktuple);
      kdim.prepend(0);
#endif

      arr y, Jy;
      //query the task map and check dimensionalities of returns
#ifdef KOMO_PATH_CONFIG
      ob->feat->__phi2(y, Jy, ob->frames);
#else
      ob->feat->__phi(y, Jy, Ktuple);
#endif
      if(!!J) CHECK_EQ(y.N, Jy.d0, "");
      if(!y.N) continue;
      if(!!J) CHECK_EQ(Jy.nd, 2, "");
#ifdef KOMO_PATH_CONFIG
      if(!!J) CHECK_EQ(Jy.d1, komo.pathConfig.getJointStateDimension(), "");
//      uint d = ob->feat->__dim_phi2(ob->frames);
//      if(d!=y.N){
//        d  = ob->feat->__dim_phi2(ob->frames);
//        ob->feat->__phi2(y, Jy, ob->frames);
//      }
//      CHECK_EQ(d, y.N, "");
#else
      if(!!J) CHECK_EQ(Jy.d1, kdim.last(), "");
#endif
      if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

      //write into phi and J
      phi.setVectorBlock(y, M);

      if(!!J) {
#ifdef KOMO_PATH_CONFIG
        if(sparse){
          Jy.sparse().reshape(J.d0, J.d1);
          Jy.sparse().colShift(M);
          J += Jy;
        }else{
          J.setMatrixBlock(Jy, M, 0);
        }
#else
        if(isSpecial(Jy) && configs.N!=1) Jy = unpack(Jy); //
        if(!isSpecial(Jy)) {
          for(uint j=0; j<configs.N; j++) {
            if(configs(j)>=0) {
              J.setMatrixBlock(Jy.sub(0, -1, kdim(j), kdim(j+1)-1), M, x_index(configs(j)));
            }
          }
        } else {
          uint j=0;
          if(configs(j)>=0) {
            Jy.sparse().reshape(J.d0, J.d1);
            Jy.sparse().colShift(M);
            Jy.sparse().rowShift(x_index(configs(j)));
            J += Jy;
          }
        }
#endif
      }

      //counter for features phi
      M += y.N;
#ifndef KOMO_PATH_CONFIG
    }
#endif
  }

  CHECK_EQ(M, dimPhi, "");
  komo.featureValues = phi;
  if(!!J) komo.featureJacobians.resize(1).scalar() = J;

  reportAfterPhiComputation(komo);

  if(quadraticPotentialLinear.N) {
    phi.append((~x * quadraticPotentialHessian * x).scalar() + scalarProduct(quadraticPotentialLinear, x));
    J.append(quadraticPotentialLinear);
  }
}

void KOMO::Conv_KOMO_SparseNonfactored::getFHessian(arr& H, const arr& x) {
  if(quadraticPotentialLinear.N) {
    H = quadraticPotentialHessian;
  } else {
    H.clear();
  }
}

void KOMO::Conv_KOMO_SparseNonfactored::report(std::ostream& os, int verbose) {
  komo.reportProblem(os);
  if(verbose>1) os <<komo.getReport(verbose>2);
  if(verbose>3) komo.view(true, "Conv_KOMO_SparseNonfactored - report");
  if(verbose>4) komo.view_play(true);
  if(verbose>5){
    rai::system("mkdir -p z.vid");
    komo.view_play(false, .1, "z.vid/");
    if(verbose>3) komo.view(true, "Conv_KOMO_SparseNonfactored - video saved in z.vid/");
  }
}

void KOMO::Conv_KOMO_SparseNonfactored::getDimPhi() {
#ifndef KOMO_PATH_CONFIG
  CHECK_EQ(komo.configurations.N, komo.k_order+komo.T, "configurations are not setup yet: use komo.reset()");
//  komo.pathConfig.jacMode = rai::Configuration::JM_noArr;
//  for(rai::Configuration* C:komo.configurations) C->jacMode = rai::Configuration::JM_noArr;
  uint M=0;
  for(uint i=0; i<komo.objectives.N; i++) {
    ptr<Objective> ob = komo.objectives.elem(i);
    for(uint l=0; l<ob->configs.d0; l++) {
      ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      M += ob->feat->__dim_phi(Ktuple); //dimensionality of this task
    }
  }
#else
  uint M=0;
  for(ptr<GroundedObjective>& ob : komo.objs) {
    M += ob->feat->__dim_phi2(ob->frames);
  }
#endif
  dimPhi = M;
}

void KOMO::Conv_KOMO_SparseNonfactored::getFeatureTypes(ObjectiveTypeA& ft) {
  if(!dimPhi) getDimPhi();
  ft.resize(dimPhi);
  komo.featureNames.clear();
  uint M=0;
  for(ptr<GroundedObjective>& ob : komo.objs) {
    uint m = ob->feat->__dim_phi2(ob->frames);
    for(uint i=0; i<m; i++) ft(M+i) = ob->type;
    for(uint j=0; j<m; j++) komo.featureNames.append(ob->feat->shortTag(komo.pathConfig));
    M += m;
  }
  if(quadraticPotentialLinear.N) {
    ft.append(OT_f);
  }
  komo.featureTypes = ft;
}

void KOMO::Conv_KOMO_SparseNonfactored::getBounds(arr& bounds_lo, arr& bounds_up) {
  if(!komo.bound_lo.N) komo.setBounds();
  bounds_lo = komo.bound_lo;
  bounds_up = komo.bound_up;
}

arr KOMO::Conv_KOMO_SparseNonfactored::getInitializationSample(const arr& previousOptima) {
  komo.run_prepare(.01);
  return komo.x;
}

KOMO::Conv_KOMO_FactoredNLP::Conv_KOMO_FactoredNLP(KOMO& _komo) : komo(_komo) {
  //count variables
  uint xDim = getDimension();

  //create variable index
  xIndex2VarId.resize(xDim);
  variableIndex.resize(komo.T);
  uint count=0;
  for(uint t=0; t<komo.T; t++) {
    VariableIndexEntry& V = variableIndex(t);
    V.t = t;
    V.dim = komo.getConfiguration_q(t).N; //.configurations(t+komo.k_order)->getJointStateDimension();
    V.xIndex = count;
    for(uint i=0; i<V.dim; i++) xIndex2VarId(count++) = t;
  }

  //count features
  uint F=0;
  for(ptr<Objective>& ob:komo.objectives) if(ob->configs.N) {
      CHECK_EQ(ob->configs.nd, 2, "in sparse mode, vars need to be tuples of variables");
      F += ob->configs.d0;
    }
  featureIndex.resize(F);

  //create feature index
  uint f=0;
  uint fDim = 0;
#ifdef KOMO_PATH_CONFIG
  for(ptr<GroundedObjective>& ob:komo.objs) {
    FeatureIndexEntry& F = featureIndex(f);
    F.ob2 = ob;
//    F.Ctuple = komo.configurations.sub(convert<uint, int>(ob->configs+(int)komo.k_order));
//    F.t = l;
    F.varIds = ob->configs;
    F.dim = ob->feat->__dim_phi2(ob->frames); //dimensionality of this task
    F.phiIndex = fDim;
    fDim += F.dim;
    f++;
  }
#else
  for(ptr<Objective>& ob:komo.objectives) {
    for(uint l=0; l<ob->configs.d0; l++) {
      FeatureIndexEntry& F = featureIndex(f);
      F.ob = ob;
      F.Ctuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      F.t = l;
      F.varIds = ob->configs[l];
      F.dim = ob->feat->__dim_phi(F.Ctuple); //dimensionality of this task
      F.phiIndex = fDim;
      fDim += F.dim;
      f++;
    }
  }
#endif
  CHECK_EQ(f, featureIndex.N, "");

  featuresDim = fDim;
}

uint KOMO::Conv_KOMO_FactoredNLP::getDimension() { return komo.pathConfig.getJointStateDimension(); }

void KOMO::Conv_KOMO_FactoredNLP::getFeatureTypes(ObjectiveTypeA& featureTypes) {
  if(!!featureTypes) featureTypes.clear();
  featureTypes.resize(featuresDim);
  komo.featureNames.resize(featuresDim);
  uint M=0;
  for(ptr<GroundedObjective>& ob : komo.objs) {
    uint m = ob->feat->__dim_phi2(ob->frames);
    for(uint i=0; i<m; i++) featureTypes(M+i) = ob->type;
    for(uint i=0; i<m; i++) komo.featureNames(M+i) = "TODO";
    M += m;
  }

  if(!!featureTypes) komo.featureTypes = featureTypes;
}

void KOMO::Conv_KOMO_FactoredNLP::getBounds(arr& bounds_lo, arr& bounds_up) {
  if(!komo.bound_lo.N) komo.setBounds();
  bounds_lo = komo.bound_lo;
  bounds_up = komo.bound_up;
}

arr KOMO::Conv_KOMO_FactoredNLP::getInitializationSample(const arr& previousOptima) {
  komo.run_prepare(.01);
  return komo.x;
}

void KOMO::Conv_KOMO_FactoredNLP::getFactorization(uintA& variableDimensions, uintA& featureDimensions, intAA& featureVariables) {
  variableDimensions.resize(variableIndex.N);
  variableDimensions.resize(variableIndex.N);
  for(uint i=0; i<variableIndex.N; i++) variableDimensions(i) = variableIndex(i).dim;

  featureDimensions.resize(featureIndex.N);
  featureVariables.resize(featureIndex.N);
  for(uint f=0; f<featureIndex.N; f++) {
    FeatureIndexEntry& F = featureIndex(f);
    featureDimensions(f) = F.dim;
    featureVariables(f) = F.varIds;
  }
}


void KOMO::Conv_KOMO_FactoredNLP::setAllVariables(const arr& x) {
  komo.set_x2(x);
}

void KOMO::Conv_KOMO_FactoredNLP::setSingleVariable(uint var_id, const arr& x) {
  komo.set_x2(x, {var_id});
}

void KOMO::Conv_KOMO_FactoredNLP::report(){
  reportAfterPhiComputation(komo);
}

void KOMO::Conv_KOMO_FactoredNLP::evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H) {
#if 1
  if(!komo.featureValues.N) {
    FeatureIndexEntry& Flast = featureIndex.last();
    komo.featureValues.resize(Flast.phiIndex+Flast.dim).setZero();
  }

  FeatureIndexEntry& F = featureIndex(feat_id);

  arr y,Jy;
#ifdef KOMO_PATH_CONFIG
  F.ob2->feat->__phi2(phi, Jy, F.ob2->frames);
#else
  F.ob->feat->__phi(y, Jy, F.Ctuple);
  phi = y;
#endif
  CHECK_EQ(phi.N, F.dim, "");

  komo.featureValues.setVectorBlock(phi, F.phiIndex);

  if(!J) return;

  CHECK_EQ(phi.N, Jy.d0, "");
  CHECK_EQ(Jy.nd, 2, "");
  if(absMax(phi)>1e10) RAI_MSG("WARNING phi=" <<phi);

  if(isSparseMatrix(Jy)) {
    auto& S = Jy.sparse();

    uint n=0;
    for(uint v:F.varIds) n += variableIndex(v).dim;
    J.resize(phi.N, n).setZero();

    for(uint k=0; k<Jy.N; k++) {
      uint i = S.elems(k, 0);
      uint j = S.elems(k, 1);
      double x = Jy.elem(k);
      uint var = xIndex2VarId(j);
      VariableIndexEntry& V = variableIndex(var);
      uint var_j = j - V.xIndex;
      CHECK(var_j < V.dim, "");
      if(V.dim == J.d1) {
        J(i, var_j) += x;
      } else {
        bool good=false;
        uint offset=0;
        for(uint v:F.varIds) {
          if(v==var) {
            J(i, offset+var_j) += x;
            good=true;
            break;
          }
          offset += variableIndex(v).dim;
        }
        CHECK(good, "Jacobian is non-zero on variable " <<var <<", but indices say that feature depends on " <<F.varIds);
      }
    }
  } else if(isRowShifted(Jy)){
    J = Jy;
  } else {
#ifdef KOMO_PATH_CONFIG
    HALT("??");
#else
    intA vars = F.ob->configs[F.t];
    uintA kdim = getKtupleDim(F.Ctuple).prepend(0);
    for(uint j=vars.N; j--;) {
      if(vars(j)<0) {
        Jy.delColumns(kdim(j), kdim(j+1)-kdim(j)); //delete the columns that correspond to the prefix!!
      }
    }
    J = Jy;
#endif
  }

#else
  //count to the feat_id;

  uint count=0;
  for(ptr<Objective>& ob:komo.objectives) {
    for(uint l=0; l<ob->configs.d0; l++) {
      if(count==feat_id) { //this is the feature we want!
        ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
        uintA kdim = getKtupleDim(Ktuple);
        kdim.prepend(0);

        //query the task map and check dimensionalities of returns
        arr Jy;
        ob->feat->__phi(phi, Jy, Ktuple);
        if(!!J && isSpecial(Jy)) Jy = unpack(Jy);

        if(!!J) CHECK_EQ(phi.N, Jy.d0, "");
        if(!!J) CHECK_EQ(Jy.nd, 2, "");
        if(!!J) CHECK_EQ(Jy.d1, kdim.last(), "");
        if(!phi.N) continue;
        if(absMax(phi)>1e10) RAI_MSG("WARNING y=" <<phi);

        if(!!J) {
          for(uint j=ob->configs.d1; j--;) {
            if(ob->configs(l, j)<0) {
              Jy.delColumns(kdim(j), kdim(j+1)-kdim(j)); //delete the columns that correspond to the prefix!!
            }
          }
          J = Jy;
        }

        if(!!H) NIY;

        if(komo.featureValues.N!=featDimIntegral.last()) {
          komo.featureValues.resize(featDimIntegral.last());
        }
        komo.featureValues.setVectorBlock(phi, featDimIntegral(feat_id));

        return;
      }
      count++;
    }
  }
#endif
}

#if 0 //the factored NLPs are NIY for pathConfig!
KOMO::Conv_KOMO_FineStructuredProblem::Conv_KOMO_FineStructuredProblem(KOMO& _komo) : komo(_komo) {
  if(!komo.configurations.N) komo.setupConfigurations();
  CHECK_EQ(komo.configurations.N, komo.k_order+komo.T, "configurations are not setup yet");

  //count variables
  uint numVars=0;
  uint xDim=0;
  for(uint t=0; t<komo.T; t++) {
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
  for(uint t=0; t<komo.T; t++) {
    int s = t+komo.k_order;
    for(rai::Joint* j:komo.configurations(s)->activeJoints) {
//      CHECK_EQ(idx, j->qIndex + j->frame->C.xIndex, "mismatch index counting");
      VariableIndexEntry& V = variableIndex(var);
      V.joint = j;
      V.dim = j->qDim();
      V.xIndex = idx;
      for(uint i=0; i<j->qDim(); i++) xIndex2VarId(idx++) = var;
      var++;
    }
    for(rai::ForceExchange* c:komo.configurations(s)->forces) {
//      CHECK_EQ(idx, c->qIndex + c->a.C.xIndex, "mismatch index counting");
      VariableIndexEntry& V = variableIndex(var);
      V.force = c;
      V.dim = c->qDim();
      V.xIndex = idx;
      for(uint i=0; i<c->qDim(); i++) xIndex2VarId(idx++) = var;
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
      F.dim = ob->feat->__dim_phi(F.Ctuple); //dimensionality of this task
      fDim += F.dim;
      f++;
    }
  }

  featuresDim = fDim;
}

uint KOMO::Conv_KOMO_FineStructuredProblem::getDimension() {
  return xIndex2VarId.N;
}

void KOMO::Conv_KOMO_FineStructuredProblem::getBounds(arr& bounds_lo, arr& bounds_up) {
  if(!komo.configurations.N) komo.setupConfigurations();
  CHECK_EQ(komo.configurations.N, komo.k_order+komo.T, "configurations are not setup yet");

  uint n = getDimension();
  bounds_lo.resize(n);
  bounds_up.resize(n);

  uint x_count=0;
  arr limits;
  for(uint t=0; t<komo.T; t++) {
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

void KOMO::Conv_KOMO_FineStructuredProblem::getFeatureTypes(ObjectiveTypeA& featureTypes) {
  featureTypes.clear();
  for(uint f=0; f<featureIndex.N; f++) {
    featureTypes.append(consts<ObjectiveType>(featureIndex(f).ob->type, featureIndex(f).dim));
  }
}

void KOMO::Conv_KOMO_FineStructuredProblem::getFactorization(uintA& variableDimensions, uintA& featureDimensions, intAA& featureVariables) {
  variableDimensions.resize(variableIndex.N);
  for(uint i=0; i<variableIndex.N; i++) {
    variableDimensions(i) = variableIndex(i).dim;
  }

  featureDimensions.resize(featureIndex.N);
  featureVariables.resize(featureIndex.N);
  arr y, J;
  for(uint f=0; f<featureIndex.N; f++) {
    FeatureIndexEntry& F = featureIndex(f);
    featureDimensions(f) = F.dim;
    if(!F.dim) continue;

    F.ob->feat->__phi(y, J, F.Ctuple);

    if(isSparseMatrix(J)) {
      F.varIds.clear();
      intA& elems = J.sparse().elems;
      for(uint i=0; i<elems.d0; i++) {
        uint idx = elems(i, 1);
        F.varIds.setAppendInSorted(xIndex2VarId(idx));
      }
    } else {
      intA vars = F.ob->configs[F.t];
      for(int& i : vars) if(i>=0) F.varIds.append(i);
    }
    featureVariables(f) = F.varIds;
  }
}

/*void KOMO::Conv_KOMO_MathematicalProgram::evaluate(arr& phi, arr& J, const arr& x){
  HALT("should not be used!");
  //-- set the decision variable
#if 0 //the following should be equivalent, althought they work quite differently
  komo.set_x2(x);
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
    F.ob->feat->__phi(y, Jy, F.Ctuple);
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
    evaluateSingleFeature(f, y, Jy, NoArr);
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

  reportAfterPhiComputation(komo);
}*/

void KOMO::Conv_KOMO_FineStructuredProblem::setSingleVariable(uint var_id, const arr& x) {
  VariableIndexEntry& V = variableIndex(var_id);
  CHECK_EQ(V.dim, x.N, "");
  if(V.joint) {
    V.joint->calc_Q_from_q(x, 0);
  }
  if(V.force) {
    V.force->calc_F_from_q(x, 0);
  }
}

void KOMO::Conv_KOMO_FineStructuredProblem::evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H) {
  FeatureIndexEntry& F = featureIndex(feat_id);

  if(!J) {
    F.ob->feat->__phi(phi, NoArr, F.Ctuple);
    return;
  }

  arr Jy;
  F.ob->feat->__phi(phi, Jy, F.Ctuple);
  CHECK_EQ(phi.N, F.dim, "");
  CHECK_EQ(phi.N, Jy.d0, "");
  CHECK_EQ(Jy.nd, 2, "");
  if(absMax(phi)>1e10) RAI_MSG("WARNING phi=" <<phi);

  if(!isSparseMatrix(Jy)) {
    intA vars = F.ob->configs[F.t];
    uintA kdim = getKtupleDim(F.Ctuple).prepend(0);
    for(uint j=vars.N; j--;) {
      if(vars(j)<0) {
        Jy.delColumns(kdim(j), kdim(j+1)-kdim(j)); //delete the columns that correspond to the prefix!!
      }
    }
    J = Jy;
  } else {
    auto S = Jy.sparse();

    uint n=0;
    for(uint v:F.varIds) n += variableIndex(v).dim;
    J.resize(phi.N, n).setZero();

    for(uint k=0; k<Jy.N; k++) {
      uint i = S.elems(k, 0);
      uint j = S.elems(k, 1);
      double x = Jy.elem(k);
      uint var = xIndex2VarId(j);
      VariableIndexEntry& V = variableIndex(var);
      uint var_j = j - V.xIndex;
      CHECK(var_j < V.dim, "");
      if(V.dim == J.d1) {
        J(i, var_j) += x;
      } else {
        bool good=false;
        uint offset=0;
        for(uint v:F.varIds) {
          if(v==var) {
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
}

void KOMO::Conv_KOMO_FineStructuredProblem::reportFeatures() {
  arr y, J;
  for(uint f=0; f<featureIndex.N; f++) {
    FeatureIndexEntry& F = featureIndex(f);
    cout <<f <<' ' <<F.dim <<' ' <<F.ob->feat->shortTag(*F.Ctuple.last()) <<endl;
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
      if(ob->configs(l, 0)!=slice) continue; //ONLY USE objectives for this slice
      ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      M += ob->feat->__dim_phi(Ktuple); //dimensionality of this task
    }
  }
  dimPhi = M;
}

void KOMO::TimeSliceProblem::getFeatureTypes(ObjectiveTypeA& ft) {
  if(!dimPhi) getDimPhi();
  ft.resize(dimPhi);

  uint M=0;
  for(uint i=0; i<komo.objectives.N; i++) {
    ptr<Objective> ob = komo.objectives.elem(i);
    CHECK_EQ(ob->configs.nd, 2, "only in sparse mode!");
    if(ob->configs.d1!=1) continue; //ONLY USE order 0 objectives!!!!!
    for(uint l=0; l<ob->configs.d0; l++) {
      if(ob->configs(l, 0)!=slice) continue; //ONLY USE objectives for this slice
      ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      uint m = ob->feat->__dim_phi(Ktuple);
      if(!!ft) for(uint i=0; i<m; i++) ft(M+i) = ob->type;
      M += m;
    }
  }
  komo.featureTypes = ft;
}

void KOMO::TimeSliceProblem::evaluate(arr& phi, arr& J, const arr& x) {
  komo.set_x2(x, TUP(slice));

  if(!dimPhi) getDimPhi();

  phi.resize(dimPhi);
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
      if(ob->configs(l, 0)!=slice) continue; //ONLY USE objectives for this slice
      ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      uintA kdim = getKtupleDim(Ktuple);
      kdim.prepend(0);

      //query the task map and check dimensionalities of returns
      ob->feat->__phi(y, Jy, Ktuple);
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

      //counter for features phi
      M += y.N;
    }
  }

  CHECK_EQ(M, dimPhi, "");
  komo.featureValues = phi;
  if(!!J) komo.featureJacobians.resize(1).scalar() = J;

  reportAfterPhiComputation(komo);
}

#endif


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
  "downUp",
  "break",

  "contact",
  "contactStick",
  "contactComplementary",
  "bounce",

  "magic",
  "magicTrans",

  "topBoxGrasp",
  "topBoxPlace",

  "push",
  "graspSlide",

  "dampMotion",

  "noCollision",
  "identical",

  "alignByInt",

  "makeFree",
  nullptr
};

intA getSwitchesFromSkeleton(const Skeleton& S, const rai::Configuration& world) {
  intA ret;
  for(int i=0; i<(int)S.N; i++) {
    if(skeletonModes.contains(S.elem(i).symbol)) { //S(i) is about a switch
      int j=i-1;
      rai::Frame *toBeSwitched = world[S.elem(i).frames(-1)];
      rai::Frame *rootOfSwitch = toBeSwitched->getUpwardLink(NoTransformation, true);
      rai::Frame *childOfSwitch = toBeSwitched->getDownwardLink(true);
      for(; j>=0; j--) {
        if(skeletonModes.contains(S.elem(j).symbol)){ //S(j) is about a switch
          const rai::String& prevSwitched = S.elem(j).frames.last();
          if(prevSwitched==toBeSwitched->name
             || prevSwitched==rootOfSwitch->name
             || prevSwitched==childOfSwitch->name)
            break;
        }
      }
      //j=-1 if not previously switched, otherwise the index of the previous switch
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

double getMaxPhaseFromSkeleton(const Skeleton& S){
  double maxPhase=0;
  for(const SkeletonEntry& s:S){
    if(s.phase0>maxPhase) maxPhase=s.phase0;
    if(s.phase1>maxPhase) maxPhase=s.phase1;
  }
  return maxPhase;
}
