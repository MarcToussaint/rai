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
//#define KOMO_MIMIC_STABLE

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
  if(logFile) delete logFile;
  objs.clear();
  objectives.clear();
  listDelete(switches);
}

void KOMO::setModel(const Configuration& C, bool _computeCollisions) {
  orgJointIndices = C.getJointIDs();
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

void KOMO::setTiming(double _phases, uint _stepsPerPhase, double durationPerPhase, uint _k_order) {
  stepsPerPhase = _stepsPerPhase;
  T = ceil(stepsPerPhase*_phases);
  tau = durationPerPhase/double(stepsPerPhase);
  k_order = _k_order;
}

void KOMO::addTimeOptimization() {
  world.addTauJoint();
  rai::Frame *timeF = world.frames.first();
#if 0 //break the constraint at phase switches: EMPIRICALLY EQUIVALENT TO BELOW (passive_ballBounce TEST)
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
  //-- we need the path configuration to ground the objectives
  if(!timeSlices.N) setupPathConfig();

  //-- if arguments are given, modify the feature's frames, scaling and order
  if(!!frames && frames.N){
    if(frames.N==1 && frames.scalar()=="ALL") f->frameIDs = framesToIndices(world.frames); //important! this means that, if no explicit selection of frames was made, all frames (of a time slice) are referred to
    else f->frameIDs = world.getFrameIDs(frames);
  }
  if(!!scale) f->scale = scale;
  if(!!target) f->target = target;
  if(order>=0) f->order = order;

  //-- create a (non-grounded) objective
  CHECK_GE(k_order, f->order, "task requires larger k-order: " <<f->shortTag(world));
  std::shared_ptr<Objective> task = objectives.append( make_shared<Objective>(f, type, f->shortTag(world), times) );

  //-- create the grounded objectives
  intA timeSlices = conv_times2tuples(times, f->order, stepsPerPhase, T, deltaFromStep, deltaToStep);
  CHECK_EQ(timeSlices .nd, 2, "");
  for(uint c=0;c<timeSlices .d0;c++){
    shared_ptr<GroundedObjective> o = objs.append( make_shared<GroundedObjective>(f, type) );
    o->timeSlices = timeSlices[c];
    o->objId = objectives.N-1;
    o->frames.resize(timeSlices .d1, o->feat->frameIDs.N);
    for(uint i=0;i<timeSlices .d1;i++){
      int s = timeSlices (c,i) + k_order;
      for(uint j=0;j<o->feat->frameIDs.N;j++){
        uint fID = o->feat->frameIDs.elem(j);
        o->frames(i,j) = this->timeSlices(s, fID);
      }
    }
    if(o->feat->frameIDs.nd==2){
      o->frames.reshape(timeSlices.d1, o->feat->frameIDs.d0, o->feat->frameIDs.d1);
    }
  }
  return task;
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

void KOMO::addModeSwitch(const arr& times, SkeletonSymbol newMode, const StringA& frames, bool firstSwitch) {
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
      addObjective({times(0), times(1)}, make_shared<F_qZeroVel>(), {frames(1)}, OT_eq, {1e1}, NoArr, 1, +1, -1);
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
      if(k_order>1) addObjective({times(1)}, make_shared<F_LinAngVel>(), {frames(1)}, OT_eq, {1e0}, NoArr, 2, +1, +1); //no acceleration of the object
    }

  } else if(newMode==SY_dynamic) {
    if(frames.N==1){
      addSwitch(times, true, JT_free, SWInit_copy, world.frames.first()->name, frames(-1));
    }else{
      addSwitch(times, true, JT_free, SWInit_copy, frames(0), frames(1));
    }
    //new contacts don't exist in step [-1], so we rather impose only zero acceleration at [-2,-1,0]
    if(firstSwitch){
      if(stepsPerPhase>3){
        addObjective({times(0)}, FS_pose, {frames(1)}, OT_eq, {1e2}, NoArr, 1, 0, +1); //overlaps with Newton-Euler -> requires forces!
      }else{
        addObjective({times(0)}, FS_pose, {frames(1)}, OT_eq, {1e2}, NoArr, 1, 0, 0);
      }
    }
    if(k_order>1){
      //... and physics starting from [-1,0,+1], ... until [-2,-1,-0]
      addObjective(times, make_shared<F_NewtonEuler>(), {frames(1)}, OT_eq, {1e2}, NoArr, 2, +1, 0);
    }
  } else if(newMode==SY_dynamicOn) {
    CHECK_EQ(frames.N, 2, "");
    Transformation rel = 0;
    rel.pos.set(0, 0, .5*(shapeSize(world, frames(0)) + shapeSize(world, frames(1))));

    addSwitch(times, true, JT_transXYPhi, SWInit_copy, frames(0), frames(1), rel);
    //new contacts don't exist in step [-1], so we rather impose only zero acceleration at [-2,-1,0]
    if(firstSwitch){
      if(stepsPerPhase>3){
        addObjective({times(0)}, FS_pose, {frames(1)}, OT_eq, {1e2}, NoArr, 1, 0, +1); //overlaps with Newton-Euler -> requires forces!
      }else{
        addObjective({times(0)}, FS_pose, {frames(1)}, OT_eq, {1e2}, NoArr, 1, 0, 0);
      }
    }

    if(k_order>1){
        addObjective(times, make_shared<F_NewtonEuler>(false), {frames(1)}, OT_eq, {1e2}, NoArr, 2, +1, 0);
    }
  } else if(newMode==SY_quasiStaticOn) {
    CHECK_EQ(frames.N, 2, "");
    Transformation rel = 0;
    rel.pos.set(0, 0, .5*(shapeSize(world, frames(0)) + shapeSize(world, frames(1))));
    addSwitch(times, true, JT_transXYPhi, SWInit_copy, frames(0), frames(1), rel);
    //-- no jump at start
    if(firstSwitch){
      addObjective({times(0)}, FS_pose, {frames(1)}, OT_eq, {1e2}, NoArr, 1, 0, +1);
    }
#if 0
    addObjective(times, make_shared<F_NewtonEuler_DampedVelocities>(0., false), {frames(1)}, OT_eq, {1e2}, NoArr, 1, +1, 0);
#else
    //eq for 3DOFs only
    ptr<Objective> o = addObjective(times, make_shared<F_NewtonEuler_DampedVelocities>(false), {frames(1)}, OT_eq, {1e2}, NoArr, 1, +1, 0);
    o->feat->scale=1e2 * arr({3, 6}, {
      1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1
    });
    //sos penalty of other forces
    o = addObjective(times, make_shared<F_NewtonEuler_DampedVelocities>(false), {frames(1)}, OT_sos, {1e0}, NoArr, 1, +1, 0);
    o->feat->scale=1e0 * arr({3, 6}, {
      0, 0, 1, 0, 0, 0,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0
    });
#endif
  } else NIY;
}

void KOMO::addSwitch_stable(double time, double endTime, const char* prevFrom, const char* from, const char* to, bool firstSwitch) {
  addModeSwitch({time, endTime}, SY_stable, {from, to}, firstSwitch);
}

void KOMO::addSwitch_stableOn(double time, double endTime, const char* prevFrom, const char* from, const char* to, bool firstSwitch) {
  addModeSwitch({time, endTime}, SY_stableOn, {from, to}, firstSwitch);
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

ptr<Objective> KOMO::add_qControlObjective(const arr& times, uint order, double scale, const arr& target, int deltaFromStep, int deltaToStep) {
  auto F = getCtrlFramesAndScale(world);
//  scale *= sqrt(tau);

  CHECK_GE(k_order, order, "");
  ptr<Objective> o = addObjective(times, make_shared<F_qItself>(F.frames, (order==0)), {}, OT_sos, scale*F.scale, target, order, deltaFromStep, deltaToStep);
  o->feat->timeIntegral=1;
  return o;
}

void KOMO::addSquaredQuaternionNorms(const arr& times, double scale) {
  addObjective(times, make_shared<F_qQuaternionNorms>(), {"ALL"}, OT_eq, {scale}, NoArr);
}

void KOMO::setSlow(double startTime, double endTime, double prec, bool hardConstrained) {
  if(stepsPerPhase>2) { //otherwise: no velocities
    uintA selectedBodies;
    for(rai::Frame* f:world.frames) if(f->joint && f->joint->dim>0 && f->joint->dim<7 && f->joint->type!=rai::JT_tau && f->joint->active && f->joint->H>0.) {
        selectedBodies.append(TUP(f->ID, f->parent->ID));
      }
    selectedBodies.reshape(selectedBodies.N/2, 2);
    ptr<Feature> feat = make_shared<F_qItself>(selectedBodies);
    if(!hardConstrained) addObjective({startTime, endTime}, feat, {}, OT_sos, {prec}, NoArr, 1);
    else addObjective({startTime, endTime}, feat, {}, OT_eq, {prec}, NoArr, 1);
  }
  //#    _MinSumOfSqr_qItself_vel(MinSumOfSqr qItself){ order=1 time=[0.98 1] scale=3e0 } #slow down
}

void KOMO::setSlowAround(double time, double delta, double prec, bool hardConstrained) {
  setSlow(time-delta, time+delta, prec, hardConstrained);
}

void KOMO::setSkeleton(const Skeleton& S) {
  //-- add objectives for mode switches
  intA switches = getSwitchesFromSkeleton(S, world);
  for(uint i=0; i<switches.d0; i++) {
    int j = switches(i, 0);
    int k = switches(i, 1);
    addModeSwitch({S(k).phase0, S(k).phase1}, S(k).symbol, S(k).frames, j<0);
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
//        rai::Frame* box = world.getFrame(s.frames(1));
//        CHECK(box, "");
//        CHECK(box->shape && box->shape->type()==rai::ST_ssBox, "");
        double boxSize = shapeSize(world, s.frames(1), 0);
        addObjective({s.phase0}, FS_positionDiff, {s.frames(0), s.frames(1)}, OT_eq, {{1,3},{1e2,.0,.0}}, {.5*boxSize,0.,0.}); //arr({1,3},{0,0,1e2})
        addObjective({s.phase0}, FS_scalarProductXZ, {s.frames(1), s.frames(0)}, OT_eq, {1e2}, {1.});
//        addObjective({s.phase0}, FS_scalarProductYZ, {s.frames(1), s.frames(0)}, OT_eq, {1e2});
        break;
      }
      case SY_touchBoxNormalY: {
        //rai::Frame* box = world.getFrame(s.frames(1));
//        CHECK(box, "");
//        CHECK(box->shape && box->shape->type()==rai::ST_ssBox, "");
        double boxSize = shapeSize(world, s.frames(1), 1);
        addObjective({s.phase0}, FS_positionDiff, {s.frames(0), s.frames(1)}, OT_eq, {{1,3},{1e2,.0,.0}}, {.5*boxSize,0.,0.}); //arr({1,3},{0,0,1e2})
        addObjective({s.phase0}, FS_scalarProductYZ, {s.frames(1), s.frames(0)}, OT_eq, {1e2}, {1.});
//        addObjective({s.phase0}, FS_scalarProductYZ, {s.frames(1), s.frames(0)}, OT_eq, {1e2});
        break;
      }
      case SY_touchBoxNormalZ: {
        rai::Frame* box = world.getFrame(s.frames(1));
        CHECK(box, "");
        CHECK(box->shape, "");
        double boxSize = 0.;
        if(box->shape->type()==rai::ST_ssBox){
          boxSize = shapeSize(world, s.frames(1), 2);
        }else if(box->shape->type()==rai::ST_cylinder){
          boxSize = shapeSize(world, s.frames(1), 1);
        }else HALT("");
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
//  if(sequenceOrPath==rai::_sequence){
//    solver = rai::KS_dense;
//  }else{
//    solver = rai::KS_sparse;
//  }

  double maxPhase = getMaxPhaseFromSkeleton(S);
  if(sequenceOrPath==rai::_sequence){
    setTiming(maxPhase, 1, 2., 1);
    add_qControlObjective({}, 1, 1e-1);
  }else{
    setTiming(maxPhase, 30, 2., 2);
    add_qControlObjective({}, 2, 1e-1);
  }
  addSquaredQuaternionNorms();

  setSkeleton(S);
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

void KOMO::setIKOpt() {
  solver = rai::KS_dense;
  setTiming(1., 1, 1., 1);
  add_qControlObjective({}, 1, 1e-1);
  addSquaredQuaternionNorms();
}

void KOMO::setConfiguration_qAll(int t, const arr& q) {
  pathConfig.setJointState(q, pathConfig.getJointsSlice(timeSlices[k_order+t], false));
}

arr KOMO::getConfiguration_qAll(int t) {
  return pathConfig.getJointState(pathConfig.getJointsSlice(timeSlices[k_order+t], false));
}

void KOMO::setConfiguration_X(int t, const arr& X) {
  pathConfig.setFrameState(X, timeSlices[k_order+t]);
}

arr KOMO::getConfiguration_X(int t) {
  return pathConfig.getFrameState(timeSlices[k_order+t]);
}

arr KOMO::getConfiguration_qOrg(int t) {
  return pathConfig.getJointState(orgJointIndices + timeSlices(k_order+t,0)->ID);
}

void KOMO::setConfiguration_qOrg(int t, const arr& q) {
  return pathConfig.setJointState(q, orgJointIndices + timeSlices(k_order+t,0)->ID);
}

arr KOMO::getPath_qOrg(){
  arr q = getConfiguration_qOrg(0);
  q.resizeCopy(T, q.N);
  for(uint t=1; t<T; t++) {
    q[t] = getConfiguration_qOrg(t);
  }
  return q;
}

arr KOMO::getPath_X() {
  arr X(T, timeSlices.d1, 7);
  for(uint t=0; t<T; t++) {
    X[t] = getConfiguration_X(t);
  }
  return X;
}

arrA KOMO::getPath_qAll() {
  arrA q(T);
  for(uint t=0; t<T; t++) q(t) = getConfiguration_qAll(t);
  return q;
}

arr KOMO::getPath_tau() {
  arr X(T);
  for(uint t=0; t<T; t++) {
    pathConfig.kinematicsTau(X(t), NoArr, timeSlices(t+k_order,0));
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
    setConfiguration_qAll(t, q);
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
#ifndef KOMO_MIMIC_STABLE //depends on sw->isStable -> mimic !!
  for(uint i=0; i<steps.N; i++) {
    uint Tstop=T;
    if(i+1<steps.N && steps(i+1)<T) Tstop=steps(i+1);
    for(uint t=steps(i); t<Tstop; t++) {
      setConfiguration_qAll(t, waypoints(i));
    }
  }
#else
  for(uint i=0; i<steps.N; i++) {
    if(steps(i)<T) setConfiguration_qAll(steps(i), waypoints(i));
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
      uintA nonSwitched = getNonSwitchedFrames(timeSlices[k_order+i0], timeSlices[k_order+i1]);
      arr q0 = pathConfig.getJointState(timeSlices[k_order+i0].sub(nonSwitched));
      arr q1 = pathConfig.getJointState(timeSlices[k_order+i1].sub(nonSwitched));
      for(uint j=i0+1; j<i1; j++) {
        arr q;
        double phase = double(j-i0)/double(i1-i0);
        if(sineProfile) {
          q = q0 + (.5*(1.-cos(RAI_PI*phase))) * (q1-q0);
        } else {
          q = q0 + phase * (q1-q0);
        }
        pathConfig.setJointState(q, timeSlices[k_order+j].sub(nonSwitched));
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

//default - transcription as sparse, but non-factored NLP
struct Conv_KOMO_SparseNonfactored : MathematicalProgram {
  KOMO& komo;
  bool sparse;
  uint dimPhi=0;

  arr quadraticPotentialLinear, quadraticPotentialHessian;

  Conv_KOMO_SparseNonfactored(KOMO& _komo, bool sparse=true) : komo(_komo), sparse(sparse) {}
  void clear() { dimPhi=0; }

  void getDimPhi();

  virtual uint getDimension() { return komo.pathConfig.getJointStateDimension(); }
  virtual void getFeatureTypes(ObjectiveTypeA& ft);
  virtual void getBounds(arr& bounds_lo, arr& bounds_up) { komo.getBounds(bounds_lo, bounds_up); }
  virtual arr getInitializationSample(const arr& previousOptima= {});
  virtual void evaluate(arr& phi, arr& J, const arr& x);
  virtual void getFHessian(arr& H, const arr& x);

  virtual void report(ostream& os, int verbose);
};

//this treats EACH BRANCH and dof as its own variable
struct Conv_KOMO_FineStructuredProblem : MathematicalProgram_Factored {
  KOMO& komo;
  //each variable refers to a SET OF dofs (e.g., a set of joints)
  struct VariableIndexEntry { DofL dofs; uint dim; };
  rai::Array<VariableIndexEntry> __variableIndex;
  uintA subVars;

  //features are one-to-one with gounded KOMO features, but with additional info on varIds
  struct FeatureIndexEntry { ptr<GroundedObjective> ob; uint dim; uintA varIds; };
  rai::Array<FeatureIndexEntry> __featureIndex;
  uintA subFeats;

  VariableIndexEntry& variableIndex(uint var_id){ if(subVars.N) return __variableIndex(subVars(var_id)); else return __variableIndex(var_id); }
  FeatureIndexEntry& featureIndex(uint feat_id){ if(subFeats.N) return __featureIndex(subFeats(feat_id)); else return __featureIndex(feat_id); }

  Conv_KOMO_FineStructuredProblem(KOMO& _komo);

  virtual void subSelect(const uintA& activeVariables, const uintA& conditionalVariables);

  virtual uint getNumVariables() { if(subVars.N) return subVars.N; return __variableIndex.N; }
  virtual uint getNumFeatures() { if(subFeats.N) return subFeats.N; return __featureIndex.N; }
//  virtual uint getVariableDim(uint var_id) { return variableIndex(var_id).dim; }
//  virtual uint getFactorDim(uint feat_id) { return featureIndex(feat_id).dim; }
//  virtual uintA getVariableFactors(uint var_id) { NIY; } //return variableIndex(var_id).featIds; }
//  virtual uintA getFactorVariables(uint feat_id) { return featureIndex(feat_id).varIds; }
  virtual rai::String getVariableName(uint var_id);

  ///-- signature/structure of the mathematical problem
  virtual uint getDimension(){ return komo.pathConfig.getJointStateDimension(); }
  virtual void getFeatureTypes(ObjectiveTypeA& featureTypes);
  virtual void getBounds(arr& bounds_lo, arr& bounds_up){ komo.getBounds(bounds_lo, bounds_up); }
//    virtual arr getInitializationSample();
  virtual arr getInitializationSample(const arr& previousOptima) {
    komo.run_prepare(.01);
    return komo.x;
  }
  virtual void getFactorization(uintA& variableDimensions, uintA& featureDimensions, uintAA& featureVariables);

  ///--- evaluation
  virtual void setSingleVariable(uint var_id, const arr& x); //set a single variable block
  virtual void evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H); //get a single feature block

  void report(ostream& os, int verbose);
};

//this treats each time slice as its own variable
struct Conv_KOMO_FactoredNLP : MathematicalProgram_Factored {
  KOMO& komo;

  struct VariableIndexEntry { uint t; uint dim; uint xIndex; };
  rai::Array<VariableIndexEntry> variableIndex;

  struct FeatureIndexEntry { shared_ptr<Objective> ob; shared_ptr<GroundedObjective> ob2; uint t; uintA varIds; uint dim; uint phiIndex; };
  rai::Array<FeatureIndexEntry> featureIndex;

  uintA xIndex2VarId;
  uint featuresDim;

  Conv_KOMO_FactoredNLP(KOMO& _komo);

  virtual uint getDimension();
  virtual void getFeatureTypes(ObjectiveTypeA& featureTypes);
  virtual void getBounds(arr& bounds_lo, arr& bounds_up) {  komo.getBounds(bounds_lo, bounds_up);  }
  virtual arr getInitializationSample(const arr& previousOptima= {});
  virtual void getFactorization(uintA& variableDimensions, uintA& featureDimensions, uintAA& featureVariables);

  virtual void setAllVariables(const arr& x);
  virtual void setSingleVariable(uint var_id, const arr& x); //set a single variable block
  virtual void evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H); //get a single feature block
  virtual void report();
};


struct Conv_KOMO_TimeSliceProblem : MathematicalProgram {
  KOMO& komo;
  int slice;
  uint dimPhi=0;

  Conv_KOMO_TimeSliceProblem(KOMO& _komo, int _slice) : komo(_komo), slice(_slice) {}

  void getDimPhi();

  virtual uint getDimension() { return komo.pathConfig.getJointStateDimension(); }
  virtual void getFeatureTypes(ObjectiveTypeA& ft);
  virtual void evaluate(arr& phi, arr& J, const arr& x);
};

void KOMO::optimize(double addInitializationNoise, const OptOptions options) {
  run_prepare(addInitializationNoise);

  if(verbose>0) reportProblem();

  run(options);
}

void KOMO::run_prepare(double addInitializationNoise) {
  //ensure the configurations are setup
  if(!timeSlices.nd) setupPathConfig();
  if(!switchesWereApplied) retrospectApplySwitches();

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
    cout <<"  #timeSlices:" <<timeSlices.d0 <<" #totalDOFs:" <<pathConfig.getJointStateDimension() <<" #frames:" <<pathConfig.frames.N;
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
    set_x(x);

  } else if(solver==rai::KS_Ipopt) {
    Conv_KOMO_SparseNonfactored P(*this, false);
    IpoptInterface ipopt(P);
    x = ipopt.solve();
    set_x(x);

  } else if(solver==rai::KS_Ceres) {
    Conv_KOMO_SparseNonfactored P(*this, false);
//    Conv_KOMO_FactoredNLP P(*this);
    LagrangianProblem L(P, options);
    Conv_MathematicalProgram_TrivialFactoreded P2(L);
    CeresInterface ceres(P2);
    x = ceres.solve();
    set_x(x);

  } else NIY;

  timeTotal = rai::realTime() - timeZero;
  if(logFile)(*logFile) <<"\n] #end of KOMO_run_log" <<endl;
  if(verbose>0) {
    cout <<"** optimization time=" <<timeTotal
         <<" (kin:" <<timeKinematics <<" coll:" <<timeCollisions <<" feat:" <<timeFeatures <<" newton: " <<timeNewton <<")"
         <<" setJointStateCount=" <<Configuration::setJointStateCount <<endl;
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
  for(ptr<Objective>& t:objectives){
    os <<"    " <<*t;
    int fromStep, toStep;
    conv_times2steps(fromStep, toStep, t->times, stepsPerPhase, T, +0, +0);
    os <<"  timeSlices: [" <<fromStep <<".." <<toStep <<"]" <<endl;
  }
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
    for(rai::Frame* f:pathConfig.frames){
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

int KOMO::view_play(bool pause, double delay, const char* saveVideoPath){ view(false, 0); return pathConfig.gl()->playVideo(timeSlices.d0, timeSlices.d1, pause, delay*tau*T, saveVideoPath); }

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

//===========================================================================

void KOMO::retrospectApplySwitches() {
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
#ifdef KOMO_MIMIC_STABLE
          /*CRUCIAL CHANGE!*/
          if(sw->isStable) f->joint->setMimic(f0->joint);
#endif
        }
      }
    }
  }
  switchesWereApplied=true;
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

void KOMO::setupPathConfig() {
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

void KOMO::getBounds(arr& bounds_lo, arr& bounds_up) {
  arr limits = ~pathConfig.getLimits();
  bounds_lo = limits[0];
  bounds_up = limits[1];
}

void KOMO::checkBounds(const arr& x) {
  DEPR; //should not be necessary, I think!, or move to kin!

  arr bound_lo, bound_up;
  getBounds(bound_lo, bound_up);
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


void KOMO::set_x(const arr& x, const uintA& selectedConfigurationsOnly) {
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

shared_ptr<MathematicalProgram> KOMO::nlp_SparseNonFactored(){
  return make_shared<Conv_KOMO_SparseNonfactored>(*this, solver==rai::KS_sparse);
}

shared_ptr<MathematicalProgram_Factored> KOMO::nlp_Factored(){
  return make_shared<Conv_KOMO_FineStructuredProblem>(*this);
}

Camera& KOMO::displayCamera(){ DEPR; return pathConfig.gl()->displayCamera(); }

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
  for(ptr<GroundedObjective>& ob:objs) {
    uint d = ob->feat->dim(ob->frames);
    int i = ob->objId;
    uint time = ob->timeSlices.last();
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
      featuresOs <<std::setw(4) <<time <<' ' <<std::setw(2) <<i <<' ' <<std::setw(2) <<d <<ob->timeSlices
                <<' ' <<std::setw(40) <<typeid(*ob->feat).name()
               <<" k=" <<ob->feat->order <<" ot=" <<ob->type <<" prec=" <<std::setw(4) <<ob->feat->scale;
      if(ob->feat->target.N<5) featuresOs <<" y*=[" <<ob->feat->target <<']'; else featuresOs<<"y*=[..]";
      featuresOs <<" y^2=" <<err(time, i) <<endl;
    }
  }
CHECK_EQ(M, featureValues.N, "");

  //-- generate a report graph
  rai::Graph report;
  double totalC=0., totalG=0., totalH=0., totalF=0.;
  for(uint i=0; i<objectives.N; i++) {
    ptr<Objective> c = objectives(i);
    Graph& g = report.newSubgraph({c->name}, {});
    g.newNode<double>({"order"}, {}, c->feat->order);
    g.newNode<String>({"type"}, {}, Enum<ObjectiveType>(c->type).name());
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
    g.newNode<arr>({"X"}, {}, getPath_X());
    g.newNode<arrA>({"x"}, {}, getPath_qAll());
    g.newNode<arr>({"dual"}, {}, dual);
  }

  //objectives
  for(ptr<GroundedObjective>& ob:objs) {

    Graph& g = K.newSubgraph({ob->feat->shortTag(pathConfig)});
    g.newNode<double>({"order"}, {}, ob->feat->order);
    g.newNode<String>({"type"}, {}, STRING(ob->type));
    g.newNode<String>({"feature"}, {}, ob->feat->shortTag(pathConfig));
    if(ob->timeSlices.N) g.newNode<intA>({"vars"}, {}, ob->timeSlices);
//    g.copy(task->feat->getSpec(world), true);
    if(includeValues) {
      arr y, Jy;
      arrA V, J;
      for(uint l=0; l<ob->timeSlices.d0; l++) {
//        ConfigurationL Ktuple = configurations.sub(convert<uint, int>(ob->timeSlices[l]+(int)k_order));
//        ob->feat->eval(y, Jy, Ktuple);
        ob->feat->eval(y, Jy, ob->frames);
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

void Conv_KOMO_SparseNonfactored::evaluate(arr& phi, arr& J, const arr& x) {
  //-- set the trajectory
  komo.set_x(x);
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
  for(ptr<GroundedObjective>& ob : komo.objs) {
      arr y, Jy;
      //query the task map and check dimensionalities of returns
      ob->feat->eval(y, Jy, ob->frames);
//      cout <<"EVAL '" <<ob->name() <<"' phi:" <<y <<endl <<Jy <<endl<<endl;
      if(!!J) CHECK_EQ(y.N, Jy.d0, "");
      if(!y.N) continue;
      if(!!J) CHECK_EQ(Jy.nd, 2, "");
      if(!!J) CHECK_EQ(Jy.d1, komo.pathConfig.getJointStateDimension(), "");
//      uint d = ob->feat->dim(ob->frames);
//      if(d!=y.N){
//        d  = ob->feat->dim(ob->frames);
//        ob->feat->eval(y, Jy, ob->frames);
//      }
//      CHECK_EQ(d, y.N, "");
      if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

      //write into phi and J
      phi.setVectorBlock(y, M);

      if(!!J) {
        if(sparse){
          Jy.sparse().reshape(J.d0, J.d1);
          Jy.sparse().colShift(M);
          J += Jy;
        }else{
          J.setMatrixBlock(Jy, M, 0);
        }
      }

      //counter for features phi
      M += y.N;
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

void Conv_KOMO_SparseNonfactored::getFHessian(arr& H, const arr& x) {
  if(quadraticPotentialLinear.N) {
    H = quadraticPotentialHessian;
  } else {
    H.clear();
  }
}

void Conv_KOMO_SparseNonfactored::report(std::ostream& os, int verbose) {
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

void Conv_KOMO_SparseNonfactored::getDimPhi() {
  uint M=0;
  for(ptr<GroundedObjective>& ob : komo.objs) {
    M += ob->feat->dim(ob->frames);
  }
  dimPhi = M;
}

void Conv_KOMO_SparseNonfactored::getFeatureTypes(ObjectiveTypeA& ft) {
  if(!dimPhi) getDimPhi();
  ft.resize(dimPhi);
  komo.featureNames.clear();
  uint M=0;
  for(ptr<GroundedObjective>& ob : komo.objs) {
    uint m = ob->feat->dim(ob->frames);
    for(uint i=0; i<m; i++) ft(M+i) = ob->type;
    for(uint j=0; j<m; j++) komo.featureNames.append(ob->feat->shortTag(komo.pathConfig));
    M += m;
  }
  if(quadraticPotentialLinear.N) {
    ft.append(OT_f);
  }
  komo.featureTypes = ft;
}

void Conv_KOMO_FineStructuredProblem::getFeatureTypes(ObjectiveTypeA& featureTypes) {
  featureTypes.clear();
  for(uint f=0; f<getNumFeatures(); f++) {
    featureTypes.append(consts<ObjectiveType>(featureIndex(f).ob->type, featureIndex(f).dim));
  }
}

arr Conv_KOMO_SparseNonfactored::getInitializationSample(const arr& previousOptima) {
  komo.run_prepare(.01);
  return komo.x;
}

Conv_KOMO_FactoredNLP::Conv_KOMO_FactoredNLP(KOMO& _komo) : komo(_komo) {
  //count variables
  uint xDim = getDimension();

  //create variable index
  xIndex2VarId.resize(xDim);
  variableIndex.resize(komo.T);
  uint count=0;
  for(uint t=0; t<komo.T; t++) {
    VariableIndexEntry& V = variableIndex(t);
    V.t = t;
    V.dim = komo.getConfiguration_qAll(t).N; //.configurations(t+komo.k_order)->getJointStateDimension();
    V.xIndex = count;
    for(uint i=0; i<V.dim; i++) xIndex2VarId(count++) = t;
  }

  //count features
  uint F=0;
  NIY;
//  for(ptr<Objective>& ob:komo.objectives) if(ob->timeSlices.N) {
//      CHECK_EQ(ob->timeSlices.nd, 2, "in sparse mode, vars need to be tuples of variables");
//      F += ob->timeSlices.d0;
//    }
  featureIndex.resize(F);

  //create feature index
  uint f=0;
  uint fDim = 0;
  for(ptr<GroundedObjective>& ob:komo.objs) {
    FeatureIndexEntry& F = featureIndex(f);
    F.ob2 = ob;
//    F.Ctuple = komo.configurations.sub(convert<uint, int>(ob->timeSlices+(int)komo.k_order));
//    F.t = l;
    copy(F.varIds, ob->timeSlices);
    F.dim = ob->feat->dim(ob->frames); //dimensionality of this task
    F.phiIndex = fDim;
    fDim += F.dim;
    f++;
  }
  CHECK_EQ(f, featureIndex.N, "");

  featuresDim = fDim;
}

uint Conv_KOMO_FactoredNLP::getDimension() { return komo.pathConfig.getJointStateDimension(); }

void Conv_KOMO_FactoredNLP::getFeatureTypes(ObjectiveTypeA& featureTypes) {
  if(!!featureTypes) featureTypes.clear();
  featureTypes.resize(featuresDim);
  komo.featureNames.resize(featuresDim);
  uint M=0;
  for(ptr<GroundedObjective>& ob : komo.objs) {
    uint m = ob->feat->dim(ob->frames);
    for(uint i=0; i<m; i++) featureTypes(M+i) = ob->type;
    for(uint i=0; i<m; i++) komo.featureNames(M+i) = "TODO";
    M += m;
  }

  if(!!featureTypes) komo.featureTypes = featureTypes;
}

arr Conv_KOMO_FactoredNLP::getInitializationSample(const arr& previousOptima) {
  komo.run_prepare(.01);
  return komo.x;
}

void Conv_KOMO_FactoredNLP::getFactorization(uintA& variableDimensions, uintA& featureDimensions, uintAA& featureVariables) {
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

void Conv_KOMO_FactoredNLP::setAllVariables(const arr& x) {
  komo.set_x(x);
}

void Conv_KOMO_FactoredNLP::setSingleVariable(uint var_id, const arr& x) {
  komo.set_x(x, {var_id});
}

void Conv_KOMO_FactoredNLP::report(){
  reportAfterPhiComputation(komo);
}

void Conv_KOMO_FactoredNLP::evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H) {
#if 1
  if(!komo.featureValues.N) {
    FeatureIndexEntry& Flast = featureIndex.last();
    komo.featureValues.resize(Flast.phiIndex+Flast.dim).setZero();
  }

  FeatureIndexEntry& F = featureIndex(feat_id);

  arr y,Jy;
  F.ob2->feat->eval(phi, Jy, F.ob2->frames);
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
        ob->feat->eval(phi, Jy, Ktuple);
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

Conv_KOMO_FineStructuredProblem::Conv_KOMO_FineStructuredProblem(KOMO& _komo) : komo(_komo) {
  komo.run_prepare(0.);
  komo.pathConfig.jacMode = rai::Configuration::JM_sparse;

  //create variable index
  uint xDim=0;
  uint varId=0;
  FrameL roots = komo.pathConfig.getRoots();
  JointL activeJoints;
  //each frame varies only with a single variable (THAT'S A LIMITING ASSUMPTION)
  uintA frameID2VarId;
  frameID2VarId.resize(komo.pathConfig.frames.N) = UINT_MAX;
  for(Frame *f:roots) {
    if(f->ID < komo.timeSlices(komo.k_order,0)->ID) continue; //ignore prefixes!
    DofL varDofs;
    FrameL branch;
    f->getSubtree(branch);
    uint varDim=0;
    for(Frame* b:branch){
      frameID2VarId(b->ID) = varId; //all frames in the branch depend on this variable
      if(b->joint && b->joint->active && b->joint->type!=JT_rigid){ //and we collect all branch active dofs into this variable
        activeJoints.append(b->joint);
        if(!b->joint->mimic && b->joint->dim){ //this is different to Configuration::calc_indexActiveJoints
          //           cout <<b->name <<", ";
          varDofs.append(b->joint);
          varDim += b->joint->dim;
        }
      }
    }
//    cout <<"--" <<endl;
    CHECK_EQ(varId, getNumVariables(), "");
    __variableIndex.append( VariableIndexEntry{ varDofs, varDim } );
    xDim += varDim;
    varId++;
  }
  CHECK_EQ(xDim, komo.pathConfig.getJointStateDimension(), "");

  //ensure that komo.pathConfig uses the same indexing -- that its activeJoint set is indexed exactly as consecutive variables
  komo.pathConfig.setActiveJoints(activeJoints);

  //create feature index
  uint featId=0;
  for(ptr<GroundedObjective>& ob:komo.objs) {
    uint featDim = ob->feat->dim(ob->frames);
    uintA featVars;
    for(rai::Frame *f:ob->frames){ //which frames does the objective depend on?
      uint varId = frameID2VarId(f->ID); //which variable parameterized that frame
      if(varId!=UINT_MAX) featVars.setAppendInSorted(varId);
    }
    __featureIndex.append( FeatureIndexEntry{ ob, featDim, featVars } );
    featId++;
  }
}

void Conv_KOMO_FineStructuredProblem::subSelect(const uintA& activeVariables, const uintA& conditionalVariables){
  uintA allVars;
  for(uint i:activeVariables) allVars.setAppendInSorted(i);
  for(uint i:conditionalVariables) allVars.setAppendInSorted(i);

  JointL activeJoints;
  for(uint v:activeVariables){
    VariableIndexEntry& V = __variableIndex(v);
    for(Dof *d:V.dofs) activeJoints.append(dynamic_cast<Joint*>(d));
  }
  subVars = activeVariables;

  //ensure that komo.pathConfig uses the same indexing -- that its activeJoint set is indexed exactly as consecutive variables
  komo.pathConfig.setActiveJoints(activeJoints);
  komo.run_prepare(0.);

  subFeats.clear();
  for(uint f=0;f<__featureIndex.N;f++){
    FeatureIndexEntry& F = __featureIndex(f);
    bool active=true;
    for(int j:F.varIds){
      if(!allVars.containsInSorted(j)) { //only objectives that link only to X \cup Y
        active=false;
      }
    }
    if(active) subFeats.append(f);
  }
}

String Conv_KOMO_FineStructuredProblem::getVariableName(uint var_id){
  Dof* d = variableIndex(var_id).dofs.first();
  Joint *j = dynamic_cast<Joint*>(d);
  CHECK(j, "");
  return STRING(j->frame->name <<'.' <<j->frame->ID);
}

void Conv_KOMO_FineStructuredProblem::getFactorization(uintA& variableDimensions, uintA& featureDimensions, uintAA& featureVariables) {
  variableDimensions.resize(getNumVariables());
  for(uint i=0; i<getNumVariables(); i++) {
    variableDimensions(i) = variableIndex(i).dim;
  }

  featureDimensions.resize(getNumFeatures());
  featureVariables.resize(getNumFeatures());
  arr y, J;

  for(uint f=0; f<getNumFeatures(); f++) {
    FeatureIndexEntry& F = featureIndex(f);
    featureDimensions(f) = F.dim;
    if(!F.dim) continue;
    featureVariables(f) = F.varIds;
  }
}

void Conv_KOMO_FineStructuredProblem::setSingleVariable(uint var_id, const arr& x) {
  //komo.pathConfig.ensure_q();

  VariableIndexEntry& v = variableIndex(var_id);
  CHECK_EQ(v.dim, x.N, "");
//  uint dofIdx = v.xIndex;
  uint xIdx=0;
  for(Dof *dof:v.dofs){
    dof->setDofs(x, xIdx);
    //      if(!j->mimic){
    if(dof->active){
      for(uint ii=0; ii<dof->dim; ii++) komo.pathConfig.q.elem(dof->qIndex+ii) = x(xIdx+ii);
    }else{
      HALT("shoudn't be here..?");
      for(uint ii=0; ii<dof->dim; ii++) komo.pathConfig.qInactive.elem(dof->qIndex+ii) = x(xIdx+ii);
    }
    xIdx += dof->dim;
  }
  CHECK_EQ(xIdx, v.dim, "");

  komo.pathConfig.proxies.clear();
  komo.pathConfig._state_q_isGood=true;
  komo.pathConfig._state_proxies_isGood=false;
}

void Conv_KOMO_FineStructuredProblem::evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H) {
  FeatureIndexEntry& F = featureIndex(feat_id);

  F.ob->feat->eval(phi, J, F.ob->frames);
  //  cout <<"EVAL '" <<F.ob->name() <<"' phi:" <<phi <<endl;
}

void Conv_KOMO_FineStructuredProblem::report(std::ostream& os, int verbose) {
  komo.reportProblem(os);
  komo.pathConfig.ensure_q();

  if(verbose>1){
    for(uint v=0; v<getNumVariables(); v++) {
      VariableIndexEntry& V = variableIndex(v);
      os <<"Variable " <<v <<" dim:" <<V.dim <<" dofs:{ ";
      for(Dof* d:V.dofs){
        Joint *j = dynamic_cast<Joint*>(d);
        CHECK(j, "");
        os <<j->frame->name <<'.' <<j->frame->ID <<' ';
      }
      os <<'}' <<endl;
    }


    arr y, J;
    for(uint f=0; f<getNumFeatures(); f++) {
      FeatureIndexEntry& F = featureIndex(f);
      os <<"Feature " <<f <<" dim:" <<F.dim <<" vars:( ";
      for(uint& i:F.varIds) os <<i <<' ';
      os <<") desc:" <<F.ob->feat->shortTag(komo.pathConfig);
      evaluateSingleFeature(f, y, J, NoArr);
      os <<" y:" <<y <<endl;
//      os <<"J:" <<J <<endl;
    }
  }

  if(verbose>3) komo.view(true, "Conv_KOMO_FineStructuredProblem - report");
  if(verbose>4) komo.view_play(true);
  if(verbose>5){
    rai::system("mkdir -p z.vid");
    komo.view_play(false, .1, "z.vid/");
    if(verbose>3) komo.view(true, "Conv_KOMO_SparseNonfactored - video saved in z.vid/");
  }
}

#if 0
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
      ob->feat->eval(y, Jy, Ktuple);
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
  "forceBalance",

  "touchBoxNormalX",
  "touchBoxNormalY",
  "touchBoxNormalZ",

  "end",

  nullptr
};

intA getSwitchesFromSkeleton(const Skeleton& S, const rai::Configuration& world) {
  intA ret;
  for(int i=0; i<(int)S.N; i++) {
    if(skeletonModes.contains(S.elem(i).symbol)) { //S(i) is about a switch
      int j=i-1;
      rai::Frame *toBeSwitched = world[S.elem(i).frames(1)];
      rai::Frame *rootOfSwitch = toBeSwitched->getUpwardLink(NoTransformation, true);
      rai::Frame *childOfSwitch = toBeSwitched->getDownwardLink(true);
      for(; j>=0; j--) {
        if(skeletonModes.contains(S.elem(j).symbol)){ //S(j) is about a switch
          const rai::String& prevSwitched = S.elem(j).frames(1);
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

Skeleton readSkeleton(std::istream& is){
  rai::Graph G(is);
  Skeleton S;
  for(rai::Node* n:G){
    cout <<"ENTRY: " << *n <<endl;
    rai::Graph& entry = n->graph();
    arr& when = entry.elem(0)->get<arr>();
    CHECK(when.N<=2, "Skeleton error entry " <<n->index <<" time interval: interval needs no, 1, or 2 elements");
    if(when.N==0) when={0.,-1.};
    if(when.N==1) when={when.scalar(),when.scalar()};
    rai::Enum<SkeletonSymbol> symbol;
    try{
      symbol = entry.elem(1)->key;
    } catch(std::runtime_error& err) {
      LOG(-1) <<"Skeleton error line " <<n->index <<" symbol: " <<err.what() <<endl;
    }
    StringA frames;
    try{
      if(entry.elem(2)->isOfType<arr>()){
        CHECK(!entry.elem(2)->get<arr>().N, "");
      }else{
        frames = entry.elem(2)->get<StringA>();
      }
    } catch(std::runtime_error& err) {
      LOG(-1) <<"Skeleton error line " <<n->index <<" frames: " <<err.what() <<endl;
    }
    S.append(SkeletonEntry(when(0), when(1), symbol, frames));
  }
  return S;
}

Skeleton readSkeleton2(std::istream& is){
  //-- first get a PRE-skeleton
  rai::Graph G(is);
  Skeleton S;
  double phase0=1.;
  for(rai::Node* step:G){
    rai::Graph& stepG = step->graph();
    for(rai::Node *lit:stepG){
      StringA frames;
      try{
        frames = lit->get<StringA>();
      } catch(std::runtime_error& err) {
        LOG(-1) <<"Skeleton error step" <<phase0 <<" literal: " <<*lit <<" err: " <<err.what() <<endl;
      }

      rai::Enum<SkeletonSymbol> symbol;
      rai::String& symb = frames.first();
      double phase1 = phase0;
      if(symb(-1)=='_'){
        phase1=-1.;
        symb.resize(symb.N-1, true);
      }
      try{
        symbol = symb;
      } catch(std::runtime_error& err) {
        LOG(-1) <<"Skeleton error line " <<phase0 <<" literal: " <<*lit <<" err: " <<err.what() <<endl;
      }

      S.append(SkeletonEntry(phase0, phase1, symbol, frames({1,-1})));
    }
    phase0 += 1.;
  }

  cout <<"PRE_skeleton: " <<endl;
  writeSkeleton(cout, S);

  //-- fill in the missing phase1!
  for(uint i=0;i<S.N;i++){
    SkeletonEntry& si = S(i);
    if(si.phase1==-1 && si.frames.N){
      for(uint j=i+1;j<S.N;j++){
        SkeletonEntry& sj = S(j);
        if(sj.phase1==-1. && sj.frames.N && sj.frames.last()==si.frames.last()){ //this is also a mode symbol (due to phase1==-1.)
          si.phase1 = sj.phase0;
          break;
        }
      }
    }
  }

  cout <<"TIMED_skeleton: " <<endl;
  writeSkeleton(cout, S);

  return S;
}
