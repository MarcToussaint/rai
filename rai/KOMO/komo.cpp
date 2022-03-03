/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "komo.h"
#include "komo-ext.h"

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
#include "../Optim/opt-nlopt.h"
#include "../Optim/opt-ipopt.h"
#include "../Optim/opt-ceres.h"

#include "../Core/util.ipp"

#include "pathTools.h"

#include <iomanip>

#ifdef RAI_GL
#  include <GL/gl.h>
#endif

#define RAI_USE_FUNCTIONALS

using namespace rai;

//===========================================================================

template<> const char* rai::Enum<rai::KOMOsolver>::names []= {
  "dense", "sparse", "banded", "sparseFactored", "NLopt", "Ipopt", "Ceres", nullptr
};


//===========================================================================

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

KOMO::KOMO() : computeCollisions(true) {
  solver = getParameter<rai::Enum<rai::KOMOsolver>>("KOMO/solver", KS_sparse);
}

KOMO::~KOMO() {
  if(logFile) delete logFile;
  objs.clear();
  objectives.clear();
  switches.clear();
}

void KOMO::setModel(const Configuration& C, bool _computeCollisions) {
  orgJointIndices = C.getJointIDs();
  if(&C!=&world) world.copy(C, _computeCollisions);
  computeCollisions = _computeCollisions;
  if(computeCollisions) {
    if(!opt.useFCL) world.swift();
    else world.fcl();
  }
  world.ensure_q();
}

void KOMO::setTiming(double _phases, uint _stepsPerPhase, double durationPerPhase, uint _k_order) {
  stepsPerPhase = _stepsPerPhase;
  T = ceil(stepsPerPhase*_phases);
  tau = durationPerPhase/double(stepsPerPhase);
  k_order = _k_order;
}

void KOMO::clone(const KOMO& komo, bool deepCopyFeatures){
  clearObjectives();
  opt = komo.opt;
  setModel(komo.world, komo.computeCollisions);
  //setTiming:
  stepsPerPhase = komo.stepsPerPhase;
  T = komo.T;
  tau = komo.tau;
  k_order = komo.k_order;

  if(komo.fcl) fcl=komo.fcl;
  if(komo.swift) swift=komo.swift;

  //directly copy pathConfig instead of recreating it (including switches)
  pathConfig.copy(komo.pathConfig, false);
  timeSlices = pathConfig.getFrames(framesToIndices(komo.timeSlices));

  //copy running objectives
  for(const ptr<Objective>& o:komo.objectives){
    std::shared_ptr<Feature> f = o->feat;
    if(deepCopyFeatures) f = f->deepCopy();
    objectives.append(make_shared<Objective>(f, o->type, o->name, o->times));
  }

  //copy grounded objectives
  for(const ptr<GroundedObjective>& o:komo.objs){
    std::shared_ptr<Feature> f = o->feat;
    if(deepCopyFeatures) f = f->deepCopy();
    auto ocopy = objs.append(make_shared<GroundedObjective>(f, o->type, o->timeSlices));
    ocopy->frames = pathConfig.getFrames(framesToIndices(o->frames));
  }
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
  switches.clear();
  reset();
}

void KOMO::_addObjective(const std::shared_ptr<Objective>& ob, const intA& timeSlices){
  objectives.append(ob);

  CHECK_EQ(timeSlices.nd, 2, "");
  CHECK_EQ(timeSlices.d1, ob->feat->order+1, "");
  for(uint c=0;c<timeSlices.d0;c++){
    shared_ptr<GroundedObjective> o = objs.append( make_shared<GroundedObjective>(ob->feat, ob->type, timeSlices[c]) );
    o->objId = objectives.N-1;
    o->frames.resize(timeSlices.d1, o->feat->frameIDs.N);
    for(uint i=0;i<timeSlices.d1;i++){
      int s = timeSlices(c,i) + k_order;
      for(uint j=0;j<o->feat->frameIDs.N;j++){
        uint fID = o->feat->frameIDs.elem(j);
        o->frames(i,j) = this->timeSlices(s, fID);
      }
    }
    if(o->feat->frameIDs.nd==2){
      o->frames.reshape(timeSlices.d1, o->feat->frameIDs.d0, o->feat->frameIDs.d1);
    }
  }
}

ptr<Objective> KOMO::addObjective(const arr& times,
                                  const ptr<Feature>& f, const StringA& frames,
                                  ObjectiveType type, const arr& scale, const arr& target, int order,
                                  int deltaFromStep, int deltaToStep) {
  //-- we need the path configuration to ground the objectives
  if(!timeSlices.N) setupPathConfig();

  //-- if arguments are given, modify the feature's frames, scaling and order
  f->setup(world, frames, scale, target, order);

  //-- determine when exactly it is active (list of tuples of given order
  intA timeSlices = conv_times2tuples(times, f->order, stepsPerPhase, T, deltaFromStep, deltaToStep);

  //-- create a (non-grounded) objective
  CHECK_GE(k_order, f->order, "task requires larger k-order: " <<f->shortTag(world));
  std::shared_ptr<Objective> task = make_shared<Objective>(f, type, f->shortTag(world), times);

  //-- create the grounded objectives
  _addObjective(task, timeSlices);
  return task;
}



//void KOMO::addFlag(double time, Flag *fl, int deltaStep) {
//  if(time<0.) time=0.;
//  fl->stepOfApplication = conv_time2step(time, stepsPerPhase) + deltaStep;
//  flags.append(fl);
//}

void KOMO::addSwitch(const arr& times, bool before, const ptr<KinematicSwitch>& sw) {
  sw->setTimeOfApplication(times, before, stepsPerPhase, T);
  applySwitch(*sw); //apply immediately
  switches.append(sw); //only to report, not apply in retrospect
}

ptr<KinematicSwitch> KOMO::addSwitch(const arr& times, bool before, bool stable,
                     rai::JointType type, SwitchInitializationType init,
                     const char* ref1, const char* ref2,
                     const rai::Transformation& jFrom, const rai::Transformation& jTo) {
  auto sw = make_shared<KinematicSwitch>(SW_joint, type, ref1, ref2, world, init, 0, jFrom, jTo);
  sw->isStable = stable;
  addSwitch(times, before, sw);
  return sw;
}

void KOMO::addModeSwitch(const arr& times, SkeletonSymbol newMode, const StringA& frames, bool firstSwitch) {
  //-- creating a stable kinematic linking
  if(newMode==SY_stable || newMode==SY_stableOn || newMode==SY_stableYPhi || newMode==SY_stableZero){
    if(newMode==SY_stable) {
      auto sw = addSwitch(times, true, true, JT_free, SWInit_copy, frames(0), frames(1));
    } else if(newMode==SY_stableZero) {
      auto sw = addSwitch(times, true, true, JT_rigid, SWInit_zero, frames(0), frames(1));
    } else if(newMode==SY_stableOn) {
      Transformation rel = 0;
      rel.pos.set(0, 0, .5*(shapeSize(world, frames(0)) + shapeSize(world, frames(1))));
      auto sw = addSwitch(times, true, true, JT_transXYPhi, SWInit_copy, frames(0), frames(1), rel);
    } else if(newMode==SY_stableYPhi) {
      Transformation rel = 0;
//      rel.pos.set(0, 0, -.5*(shapeSize(world, frames(0)) + shapeSize(world, frames(1))));
      auto sw = addSwitch(times, true, true, JT_transY, SWInit_copy, frames(0), frames(1), rel);
    } else NIY;

    if(!opt.mimicStable && newMode!=SY_stableZero){
      // ensure the DOF is constant throughout its existance
      if((times(1)<0. && stepsPerPhase*times(0)<T) || stepsPerPhase*times(1)>stepsPerPhase*times(0)+1) {
        addObjective({times(0), times(1)}, make_shared<F_qZeroVel>(), {frames(1)}, OT_eq, {1e1}, NoArr, 1, +1, -1);
      }
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
      addSwitch(times, true, false, JT_free, SWInit_copy, world.frames.first()->name, frames(-1));
    }else{
      addSwitch(times, true, false, JT_free, SWInit_copy, frames(0), frames(1));
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

    addSwitch(times, true, false, JT_transXYPhi, SWInit_copy, frames(0), frames(1), rel);
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
//    addSwitch(times, true, JT_transXYPhi, SWInit_copy, frames(0), frames(1), rel);
    addSwitch(times, true, make_shared<KinematicSwitch>(SW_joint, JT_transXYPhi, frames(0), frames(1), world, SWInit_copy, 0, rel, NoTransformation));

    //-- no jump at start
    if(firstSwitch){
      if(stepsPerPhase>3){
        addObjective({times(0)}, FS_pose, {frames(1)}, OT_eq, {1e2}, NoArr, 1, 0, +1); //overlaps with Newton-Euler -> requires forces!
      }else{
        addObjective({times(0)}, FS_pose, {frames(1)}, OT_eq, {1e2}, NoArr, 1, 0, 0);
      }
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

void KOMO::addContact_slide(double startTime, double endTime, const char* from, const char* to) {
  addSwitch({startTime}, true, make_shared<rai::KinematicSwitch>(rai::SW_addContact, rai::JT_none, from, to, world));
  if(endTime>0.) addSwitch({endTime}, false, make_shared<rai::KinematicSwitch>(rai::SW_delContact, rai::JT_none, from, to, world));

  //constraints
#ifdef RAI_USE_FUNCTIONALS //new, based on functionals
  addObjective({startTime, endTime}, make_shared<F_fex_POASurfaceDistance>(rai::_left), {from, to}, OT_eq, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_POASurfaceDistance>(rai::_right), {from, to}, OT_eq, {1e1});
#else //old, based on PairCollision
  addObjective({startTime, endTime}, make_shared<F_fex_POAContactDistances>(), {from, to}, OT_ineq, {1e1});
#endif
  addObjective({startTime, endTime}, FS_pairCollision_negScalar, {from, to}, OT_eq, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_ForceIsNormal>(), {from, to}, OT_eq, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_ForceIsPositive>(), {from, to}, OT_ineq, {1e2});

  //regularization
  addObjective({startTime, endTime}, make_shared<F_fex_Force>(), {from, to}, OT_sos, {1e-1}, NoArr, k_order, +2, 0);
  addObjective({startTime, endTime}, make_shared<F_fex_Force>(), {from, to}, OT_sos, {1e-2});
  addObjective({startTime, endTime}, make_shared<F_fex_POA>(), {from, to}, OT_sos, {1e-2}, NoArr, k_order, +2, +0);
  addObjective({startTime, endTime}, make_shared<F_fex_POAzeroRelVel>(), {from, to}, OT_sos, {1e0}, NoArr, 1, +1, +0);
}

void KOMO::addContact_stick(double startTime, double endTime, const char* from, const char* to) {
  addSwitch({startTime}, true, make_shared<rai::KinematicSwitch>(rai::SW_addContact, rai::JT_none, from, to, world));
  if(endTime>0.) addSwitch({endTime}, false, make_shared<rai::KinematicSwitch>(rai::SW_delContact, rai::JT_none, from, to, world));

  //constraints
#ifdef RAI_USE_FUNCTIONALS //new, based on functionals
  addObjective({startTime, endTime}, make_shared<F_fex_POASurfaceDistance>(rai::_left), {from, to}, OT_eq, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_POASurfaceDistance>(rai::_right), {from, to}, OT_eq, {1e1});
#else
  addObjective({startTime, endTime}, make_shared<F_fex_POAContactDistances>(), {from, to}, OT_ineq, {1e1});
#endif
  addObjective({startTime, endTime}, FS_pairCollision_negScalar, {from, to}, OT_eq, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_ForceIsPositive>(), {from, to}, OT_ineq, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_POAzeroRelVel>(), {from, to}, OT_eq, {1e0}, NoArr, 1, +1, 0);

  //regularization
  addObjective({startTime, endTime}, make_shared<F_fex_Force>(), {from, to}, OT_sos, {1e-2}, NoArr, k_order, +2, 0);
  addObjective({startTime, endTime}, make_shared<F_fex_Force>(), {from, to}, OT_sos, {1e-4});
  addObjective({startTime, endTime}, make_shared<F_fex_POA>(), {from, to}, OT_sos, {1e-2}, NoArr, k_order, +2, +0);
}

void KOMO::addContact_ComplementarySlide(double startTime, double endTime, const char* from, const char* to) {
  addSwitch({startTime}, true, make_shared<rai::KinematicSwitch>(rai::SW_addContact, rai::JT_none, from, to, world));
  if(endTime>0.) addSwitch({endTime}, false, make_shared<rai::KinematicSwitch>(rai::SW_delContact, rai::JT_none, from, to, world));

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

void KOMO::addContact_elasticBounce(double time, const char* from, const char* to, double elasticity, double stickiness) {
  addSwitch({time}, true,  make_shared<rai::KinematicSwitch>(rai::SW_addContact, rai::JT_none, from, to, world));
  addSwitch({time}, false, make_shared<rai::KinematicSwitch>(rai::SW_delContact, rai::JT_none, from, to, world));

  //constraints
#ifdef RAI_USE_FUNCTIONALS //new, based on functionals
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
  //F.scale *= sqrt(tau); NO!! The Feature::finiteDifference does this automatically, depending on the timeIntegral flag!

  CHECK_GE(k_order, order, "");
  ptr<Objective> o = addObjective(times, make_shared<F_qItself>(F.frames, (order==0)), {}, OT_sos, scale*F.scale, target, order, deltaFromStep, deltaToStep);
  o->feat->timeIntegral=1;
  return o;
}

void KOMO::addQuaternionNorms(const arr& times, double scale, bool hard) {
  addObjective(times, make_shared<F_qQuaternionNorms>(), {"ALL"}, (hard?OT_eq:OT_sos), {scale}, NoArr);
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

void KOMO::add_collision(bool hardConstraint, double margin, double prec) {
  if(hardConstraint) { //interpreted as hard constraint (default)
    addObjective({}, make_shared<F_AccumulatedCollisions>(margin), {"ALL"}, OT_eq, {prec}, NoArr);
  } else { //cost term
    addObjective({}, make_shared<F_AccumulatedCollisions>(margin), {"ALL"}, OT_sos, {prec}, NoArr);
  }
}

void KOMO::add_jointLimits(bool hardConstraint, double margin, double prec) {
  if(hardConstraint) { //interpreted as hard constraint (default)
    addObjective({}, make_shared<F_qLimits>(), {"ALL"}, OT_ineq, {prec}, {-margin});
  } else { //cost term
    NIY;
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
  addQuaternionNorms();
}

void KOMO::setConfiguration_qAll(int t, const arr& q) {
  pathConfig.setDofState(q, pathConfig.getDofs(timeSlices[k_order+t], false));
}

arr KOMO::getConfiguration_qAll(int t) {
  return pathConfig.getDofState(pathConfig.getDofs(timeSlices[k_order+t], false));
}

void KOMO::setConfiguration_X(int t, const arr& X) {
  pathConfig.setFrameState(X, timeSlices[k_order+t]);
}

arr KOMO::getConfiguration_X(int t) {
  return pathConfig.getFrameState(timeSlices[k_order+t]);
}

arr KOMO::getConfiguration_qOrg(int t) {
  return pathConfig.getDofState(pathConfig.getDofs(pathConfig.getFrames(orgJointIndices + timeSlices(k_order+t,0)->ID), false));
}

void KOMO::setConfiguration_qOrg(int t, const arr& q) {
  pathConfig.setDofState(q, pathConfig.getDofs(pathConfig.getFrames(orgJointIndices + timeSlices(k_order+t,0)->ID), false));
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
    setConfiguration_qOrg(t, q);
  }
  run_prepare(0.);
}

void setQByPairs(rai::Configuration& C, const FrameL& F, arr q){
  uint m=0;
  FrameL sel;
  for(uint i=0; i<F.N; i+=2) {
    rai::Frame* a = F.elem(i+0);
    rai::Frame* b = F.elem(i+1);
    if(a->parent==b) { sel.append(a); m += a->joint->dim; }
    else if(b->parent==a) { sel.append(b); for(uint k=0; k<b->joint->dim; k++) q.elem(m++) *= -1; }
    else HALT("a and b are not linked");
  }
  CHECK_EQ(m, q.N, "");
  C.setJointState(q, sel);
}

void getDofsAndSignFromFramePairs(DofL& dofs, arr& signs, const FrameL& F){
  signs.clear();
  dofs.resize(F.N/2);
  for(uint i=0; i<F.N/2; i++) {
    rai::Frame* a = F.elem(2*i+0);
    rai::Frame* b = F.elem(2*i+1);
    if(a->parent==b) { dofs(i)=a->joint; signs.append(ones(a->joint->dim)); }
    else if(b->parent==a) { dofs(i)=b->joint; signs.append(-ones(b->joint->dim)); }
    else HALT("a and b are not linked");
  }
}

uintA KOMO::initWithWaypoints_pieceWiseConstant(const arrA& waypoints, uint waypointStepsPerPhase, int verbose) {

  //compute in which steps (configuration time slices) the waypoints are imposed
  uintA steps(waypoints.N);
  for(uint i=0; i<steps.N; i++) {
    steps(i) = conv_time2step(conv_step2time(i, waypointStepsPerPhase), stepsPerPhase);
  }

  if(verbose>0){
    view(true, STRING("initWithWaypoints - before"));
  }

  //first set the path piece-wise CONSTANT at waypoints and the subsequent steps (each waypoint may have different dimension!...)
  if(!opt.mimicStable){ //depends on sw->isStable -> mimic !!
    for(uint i=0; i<steps.N; i++) {
      uint Tstop=T;
      if(i+1<steps.N && steps(i+1)<T) Tstop=steps(i+1);
      for(uint t=steps(i); t<Tstop; t++) {
        if(waypoints(i).nd==1){
          try{
            setConfiguration_qAll(t, waypoints(i));
          }catch(...){}
        }else{
          setConfiguration_X(t, waypoints(i));
        }
      }
    }
  }else{
    for(uint i=0; i<steps.N; i++) {
      if(steps(i)<T) setConfiguration_qAll(steps(i), waypoints(i));
    }
  }

  return steps;
}

void KOMO::initWithWaypoints(const arrA& waypoints, uint waypointStepsPerPhase, int verbose) {

  uintA steps = initWithWaypoints_pieceWiseConstant(waypoints, waypointStepsPerPhase, verbose);

  if(verbose>0){
    view(true, STRING("initWithWaypoints - after keyframes->constant"));
  }

  //then interpolate w.r.t. non-switching frames within the intervals
#if 1
  auto F = getCtrlFramesAndScale(world);
  //F.frames.reshape(1,-1,2); F_qItself qfeat;
  arr signs;
  DofL dofs;
  for(uint i=0; i<steps.N; i++) {
    uint t0=0; if(i) t0 = steps(i-1);
    uint t1=steps(i);

    //interpolate the controlled DOFs, using qItself to also use the pair-wise joint indexing for flipped joints (walker case)
#if 0
    if(t1-1<T) {
      uintA nonSwitched = getNonSwitchedFrames(timeSlices[k_order+t0], timeSlices[k_order+t1]);
      arr q0 = pathConfig.getJointState(timeSlices[k_order+t0].sub(nonSwitched));
      arr q1 = pathConfig.getJointState(timeSlices[k_order+t1].sub(nonSwitched));
      for(uint t=t0+1; t<t1; t++) {
        double phase = double(t-t0)/double(t1-t0);
        arr q = q0 + (.5*(1.-cos(RAI_PI*phase))) * (q1-q0); //p = p0 + phase * (p1-p0);
        pathConfig.setJointState(q, timeSlices[k_order+t].sub(nonSwitched));
        //view(true, STRING("interpolating: step:" <<i <<" t: " <<j));
      }
    }
#else
    if(t1-1<T) {
      getDofsAndSignFromFramePairs(dofs, signs, pathConfig.getFrames(F.frames + timeSlices(k_order+t0,0)->ID));
      arr q0 = signs%pathConfig.getDofState(dofs);
      getDofsAndSignFromFramePairs(dofs, signs, pathConfig.getFrames(F.frames + timeSlices(k_order+t1,0)->ID));
      arr q1 = signs%pathConfig.getDofState(dofs);
      //TODO: check if all dofs are rotational joints!
      makeMod2Pi(q0, q1);
      pathConfig.setDofState(signs%q1, dofs);
//      arr q0 = qfeat.eval(pathConfig.getFrames(F.frames + timeSlices(k_order+t0,0)->ID));  q0.J_reset();
//      arr q1 = qfeat.eval(pathConfig.getFrames(F.frames + timeSlices(k_order+t1,0)->ID));  q1.J_reset();
      for(uint t=t0+1; t<t1; t++) {
        double phase = double(t-t0)/double(t1-t0);
        arr q = q0 + (.5*(1.-cos(RAI_PI*phase))) * (q1-q0); //p = p0 + phase * (p1-p0);
        getDofsAndSignFromFramePairs(dofs, signs, pathConfig.getFrames(F.frames + timeSlices(k_order+t,0)->ID));
        pathConfig.setDofState(signs%q, dofs);
//        setQByPairs(pathConfig, pathConfig.getFrames(F.frames + timeSlices(k_order+t,0)->ID), q);
        //view(true, STRING("interpolating: step:" <<i <<" t: " <<t));
      }
    }
#endif

    //motion profile for switched object positions
#if 0 //that doesn't work for walking!!
    if(t1-1<T) {
      uintA switched = getSwitchedFrames(timeSlices[k_order+t0], timeSlices[k_order+t1]);
      for(uint k:switched){
        if(timeSlices(k_order+t0,k)->joint && timeSlices(k_order+t0,k)->joint->isStable) continue; //don't interpolate stable joints
        arr p0 = timeSlices(k_order+t0,k)->getPosition();
        arr p1 = timeSlices(k_order+t1,k)->getPosition();
        for(uint t=t0+1; t<t1; t++) {
          double phase = double(t-t0)/double(t1-t0);
          arr p = p0 + (.5*(1.-cos(RAI_PI*phase))) * (p1-p0); //p = p0 + phase * (p1-p0);
          timeSlices(k_order+t, k)->setPosition(p);
          //view(true, STRING("interpolating: step:" <<i <<" t: " <<j));
        }
      }
    }
#endif
  }
#endif

  if(verbose>0){
    view(true, STRING("initWithWaypoints - done"));
  }

  run_prepare(0.);
}

void KOMO::straightenCtrlFrames_mod2Pi(){
  auto F = getCtrlFramesAndScale(world);
  arr signs;
  DofL dofs;
  for(uint t=0;t<T-1;t++) {
    getDofsAndSignFromFramePairs(dofs, signs, pathConfig.getFrames(F.frames + timeSlices(k_order+t,0)->ID));
    arr q0 = signs%pathConfig.getDofState(dofs);
    getDofsAndSignFromFramePairs(dofs, signs, pathConfig.getFrames(F.frames + timeSlices(k_order+t+1,0)->ID));
    arr q1 = signs%pathConfig.getDofState(dofs);
    //TODO: check if all dofs are rotational joints!
    makeMod2Pi(q0, q1);
    pathConfig.setDofState(signs%q1, dofs);
  }
}


void KOMO::addWaypointsInterpolationObjectives(const arrA& waypoints, uint waypointStepsPerPhase) {

  uintA steps = initWithWaypoints_pieceWiseConstant(waypoints, waypointStepsPerPhase);

  for(uint k=0; k<steps.N; k++) {
    uint t0=0; if(k) t0 = steps(k-1);
    uint t1=steps(k);

    for(uint i=0;i<timeSlices.d1;i++){
      rai::Transformation A = timeSlices(k_order+t0, i)->ensure_X();
      rai::Transformation B = timeSlices(k_order+t1, i)->ensure_X();
      rai::Transformation X;
      if(A==B){
      }else{
        for(uint t=t0; t<=t1; t++) {
          double phase = double(t-t0)/double(t1-t0);
          X.setInterpolate(phase, A, B);
          {
            std::shared_ptr<Feature> feat = make_shared<F_Position>();
            feat->setFrameIDs({i}) .setTarget(X.pos.getArr());
            ptr<struct Objective>  tmp = addObjective({0.}, feat, {}, OT_sos, NoArr, NoArr, -1, t, t);
          }
          {
            std::shared_ptr<Feature> feat = make_shared<F_Quaternion>();
            feat->setFrameIDs({i}) .setTarget(X.rot.getArr4d());
            ptr<struct Objective>  tmp = addObjective({0.}, feat, {}, OT_sos, NoArr, NoArr, -1, t, t);
          }
        }
      }
    }
  }
}

void KOMO::updateRootObjects(const Configuration& C){
  //-- frame state of roots only, if objects moved:
  FrameL _roots = C.getRoots();
  {//also add rigid children of roots
    FrameL F;
    for(auto f:_roots) f->getRigidSubFrames(F, true);
    _roots.append(F);
  }
  uintA roots = framesToIndices(_roots);
  arr X0 = C.getFrameState(roots);
  //set t=0..T to new frame state:
  for(uint t=0; t<T; t++) pathConfig.setFrameState(X0, roots+timeSlices(k_order+t,0)->ID);
  //shift the frame states within the prefix (t=-1 becomes equal to t=0, which is new state)
  for(int t=-k_order; t<0; t++){
    arr Xt = pathConfig.getFrameState(roots+timeSlices(k_order+t+1,0)->ID);
    pathConfig.setFrameState(Xt, roots+timeSlices(k_order+t,0)->ID);
  }
}

void KOMO::updateAndShiftPrefix(const Configuration& C){
  //-- joint state
  //set t=0 to new joint state:
  setConfiguration_qOrg(0, C.getJointState());
  //shift the joint state within prefix (t=-1 becomes equal to t=0, which is new state)
  for(int t=-k_order; t<0; t++) setConfiguration_qOrg(t, getConfiguration_qOrg(t+1));

  updateRootObjects(C);
}

void KOMO::reset() {
  dual.clear();
  featureValues.clear();
  featureJacobians.clear();
  featureTypes.clear();
  timeTotal=timeCollisions=timeKinematics=timeNewton=timeFeatures=0.;
}

//default - transcription as sparse, but non-factored NLP
struct Conv_KOMO_SparseNonfactored : MathematicalProgram {
  KOMO& komo;
  bool sparse;

  arr quadraticPotentialLinear, quadraticPotentialHessian;

  Conv_KOMO_SparseNonfactored(KOMO& _komo, bool sparse=true);

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
  struct FeatureIndexEntry { shared_ptr<GroundedObjective> ob; uint dim; uintA varIds; };
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
//    virtual arr getInitializationSample();
  virtual arr getInitializationSample(const arr& previousOptima) {
    komo.run_prepare(.01);
    return komo.x;
  }

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

  virtual arr getInitializationSample(const arr& previousOptima= {});

  virtual void setAllVariables(const arr& x);
  virtual void setSingleVariable(uint var_id, const arr& x); //set a single variable block
  virtual void evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H); //get a single feature block
  virtual void report();
};


struct Conv_KOMO_TimeSliceProblem : MathematicalProgram {
  KOMO& komo;
  int slice;

  Conv_KOMO_TimeSliceProblem(KOMO& _komo, int _slice) : komo(_komo), slice(_slice) {
    dimension = komo.pathConfig.getJointStateDimension();
  }

  void getDimPhi();

  virtual void evaluate(arr& phi, arr& J, const arr& x);
};

void KOMO::optimize(double addInitializationNoise, const OptOptions options) {
  run_prepare(addInitializationNoise);

  if(opt.verbose>1) reportProblem();

  run(options);
}

void KOMO::run_prepare(double addInitializationNoise) {
  //ensure the configurations are setup
  if(!timeSlices.nd) setupPathConfig();
//  if(!switchesWereApplied) retrospectApplySwitches(); //should be applied immediately!

  //ensure the decision variable is in sync from the configurations
  x = pathConfig.getJointState();

  //add noise
  if(addInitializationNoise>0.) {
    rndGauss(x, addInitializationNoise, true); //don't initialize at a singular config
    arr lo, up;
    getBounds(lo, up);
    boundClip(x, lo, up);
  }
}

void KOMO::run(OptOptions options) {
  Configuration::setJointStateCount=0;
  if(opt.verbose>0) {
    cout <<"** KOMO::run solver:"
        <<rai::Enum<KOMOsolver>(solver)
       <<" collisions:" <<computeCollisions
      <<" x-dim:" <<x.N
      <<" T:" <<T <<" k:" <<k_order <<" phases:" <<double(T)/stepsPerPhase <<" stepsPerPhase:" <<stepsPerPhase <<" tau:" <<tau;
    cout <<"  #timeSlices:" <<timeSlices.d0 <<" #totalDOFs:" <<pathConfig.getJointStateDimension() <<" #frames:" <<pathConfig.frames.N;
    cout <<endl;
  }

  options.verbose = rai::MAX(opt.verbose-2, 0);
  timeTotal -= rai::cpuTime();
  CHECK(T, "");
  if(logFile)(*logFile) <<"KOMO_run_log: [" <<endl;

  if(solver==rai::KS_none) {
    HALT("you need to choose a KOMO solver");

  } else if(solver==rai::KS_dense || solver==rai::KS_sparse) {
    Conv_KOMO_SparseNonfactored P(*this, solver==rai::KS_sparse);
    OptConstrained _opt(x, dual, P.ptr(), options, logFile);
    _opt.run();
    timeNewton += _opt.newton.timeNewton;

  } else if(solver==rai::KS_sparseFactored) {
    Conv_KOMO_SparseNonfactored P(*this, true);
    OptConstrained _opt(x, dual, P.ptr(), options, logFile);
    _opt.run();
    timeNewton += _opt.newton.timeNewton;

  } else if(solver==rai::KS_banded) {
    pathConfig.jacMode = rai::Configuration::JM_rowShifted;
    auto P = make_shared<Conv_KOMO_FactoredNLP>(*this);
    Conv_FactoredNLP_BandedNLP C(P, 0);
    C.maxBandSize = (k_order+1)*max(P->variableDimensions);
    OptConstrained opt(x, dual, C.ptr(), options, logFile);
    opt.run();

  } else if(solver==rai::KS_NLopt) {
    Conv_KOMO_SparseNonfactored P(*this, false);
    NLoptInterface nlopt(P.ptr());
    x = nlopt.solve();
    set_x(x);

  } else if(solver==rai::KS_Ipopt) {
    Conv_KOMO_SparseNonfactored P(*this, false);
    IpoptInterface ipopt(P.ptr());
    x = ipopt.solve();
    set_x(x);

  } else if(solver==rai::KS_Ceres) {
    Conv_KOMO_SparseNonfactored P(*this, false);
//    Conv_KOMO_FactoredNLP P(*this);
    LagrangianProblem L(P.ptr(), options);
    auto P2 = make_shared<Conv_MathematicalProgram_TrivialFactoreded>(L.ptr());
    CeresInterface ceres(P2);
    x = ceres.solve();
    set_x(x);

  } else NIY;

  timeTotal += rai::cpuTime();

  if(logFile)(*logFile) <<"\n] #end of KOMO_run_log" <<endl;
  if(opt.verbose>0) {
    cout <<"** optimization time:" <<timeTotal
         <<" (kin:" <<timeKinematics <<" coll:" <<timeCollisions <<" feat:" <<timeFeatures <<" newton: " <<timeNewton <<")"
         <<" setJointStateCount:" <<Configuration::setJointStateCount
        <<"\n   sos:" <<sos <<" ineq:" <<ineq <<" eq:" <<eq <<endl;
  }
  if(opt.verbose>1) cout <<getReport(opt.verbose>2) <<endl;
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
  for(shared_ptr<Objective>& t:objectives){
    os <<"    " <<*t;
    int fromStep, toStep;
    conv_times2steps(fromStep, toStep, t->times, stepsPerPhase, T, +0, +0);
    os <<"  timeSlices: [" <<fromStep <<".." <<toStep <<"]" <<endl;
  }
  for(std::shared_ptr<KinematicSwitch>& sw:switches) {
    os <<"    ";
    if(sw->timeOfApplication+k_order >= timeSlices.d0) {
//      LOG(-1) <<"switch time " <<sw->timeOfApplication <<" is beyond time horizon " <<T;
      sw->write(os, {});
    } else {
      sw->write(os, timeSlices[sw->timeOfApplication+k_order]);
    }
    os <<endl;
  }

  if(opt.verbose>6){
    os <<"  INITIAL STATE" <<endl;
    for(rai::Frame* f:pathConfig.frames){
      if(f->joint && f->joint->dim) os <<"    " <<f->name <<" [" <<f->joint->type <<"] : " <<f->joint->calcDofsFromConfig() /*<<" - " <<pathConfig.q.elem(f->joint->qIndex)*/ <<endl;
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

  shared_ptr<MathematicalProgram_Factored> SP;
  shared_ptr<MathematicalProgram> CP;

  if(solver==rai::KS_none) {
    NIY;
  } else if(solver==rai::KS_banded) {
    SP = make_shared<Conv_KOMO_FactoredNLP>(*this);
    auto BP = make_shared<Conv_FactoredNLP_BandedNLP>(SP, 0);
    BP->maxBandSize = (k_order+1)*max(SP->variableDimensions);
    CP = BP;
  } else if(solver==rai::KS_sparseFactored) {
    SP = make_shared<Conv_KOMO_FactoredNLP>(*this);
    CP = make_shared<Conv_FactoredNLP_BandedNLP>(SP, 0, true);
  } else {
    CP = make_shared<Conv_KOMO_SparseNonfactored>(*this, solver==rai::KS_sparse);
  }

  VectorFunction F = [CP](const arr& x) -> arr{
    arr phi, J;
    CP->evaluate(phi, J, x);
    phi.J() = J;
    return phi;
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

  fil <<getPath_qOrg().modRaw() <<endl;
  fil.close();

  ofstream fil2("z.trajectories.plt");
  fil2 <<"set key autotitle columnheader" <<endl;
  fil2 <<"set title 'trajectories'" <<endl;
//  fil2 <<"set term qt 2" <<endl;
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

void KOMO::applySwitch(const KinematicSwitch& sw) {
#if 0 //for debugging
    cout <<"APPLYING SWITCH:\n" <<*sw <<endl;
    cout <<world.frames(sw.fromId)->name <<"->" <<world.frames(sw.toId)->name <<endl;
    sw.apply(world.frames);
    listWriteNames( world.frames(sw.toId)->getPathToRoot(), cout );
    listWriteNames( world.frames(sw.toId)->children, cout );
#endif
    int s = sw.timeOfApplication+(int)k_order;
    if(s<0) s=0;
    int sEnd = int(k_order+T);
//    if(sw.timeOfTermination>=0)  sEnd = sw.timeOfTermination+(int)k_order;
    CHECK(s<sEnd, "s:" <<s <<" sEnd:" <<sEnd);
    rai::Frame *f0=0;
    for(; s<sEnd; s++) { //apply switch on all configurations!
      rai::Frame* f = sw.apply(timeSlices[s]());
      if(!f0){
        f0=f;
      } else {
        if(sw.symbol==SW_addContact){
          rai::ForceExchange* ex0 = f0->forces.last();
          rai::ForceExchange* ex1 = f->forces.last();
          ex1->poa = ex0->poa;
        }else{
          f->set_Q() = f0->get_Q(); //copy the relative pose (switch joint initialization) from the first application
          if(opt.mimicStable){
            /*CRUCIAL CHANGE!*/
            if(sw.isStable) f->joint->setMimic(f0->joint);
          }
        }
      }
    }
}

void KOMO::retrospectApplySwitches() {
  DEPR; //switches are applied immediately on addSwitch
  for(std::shared_ptr<KinematicSwitch>& sw:switches) applySwitch(*sw);
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

  //computeMeshNormals(world.frames, true);
  //computeMeshGraphs(world.frames, true);

  rai::Configuration C;
  C.copy(world, true);
  C.setTaus(tau);

  if(computeCollisions) {
    CHECK(!fcl, "");
    CHECK(!swift, "");
    if(!opt.useFCL) swift = C.swift();
    else fcl = C.fcl();
  }

  for(uint s=0;s<k_order+T;s++) {
//    for(KinematicSwitch* sw:switches) { //apply potential switches
//      if(sw.timeOfApplication+(int)k_order==(int)s)  sw.apply(C.frames);
//    }

//    uint nBefore = pathConfig.frames.N;
    pathConfig.addCopies(C.frames, C.dofs);
//    timeSlices[s] = pathConfig.frames({nBefore, -1});

  }
  timeSlices = pathConfig.frames;

  //deactivate prefix dofs
  pathConfig.calc_indexedActiveJoints();
  uint firstID = timeSlices(k_order, 0)->ID;
  for(Dof* dof:pathConfig.activeDofs){
    if(dof->frame->ID < firstID){
      if(!dof->mimicers.N) dof->active=false;
      else{
        bool act=false;
        for(Dof* m:dof->mimicers) if(m->active){ act=true; break; }
        if(!act) dof->active=false;
      }
    }
  }
  pathConfig.calc_indexedActiveJoints();
//  for(uint t=0; t<T; ++t){
//    for(auto* f:timeSlices[t + k_order]) {
//      if(f->joint && f->joint->active){
//        activeJoints.append(f);
//      }
//    }
//  }
//  pathConfig.selectJoints(activeJoints);

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
  boundCheck(x, bound_lo, bound_up);
}

//===========================================================================

void reportAfterPhiComputation(KOMO& komo) {
  if(komo.opt.verbose>6 || komo.opt.animateOptimization>2) {
    //  komo.reportProxies();
    cout <<komo.getReport(true) <<endl;
  }
  if(komo.opt.animateOptimization>0) {
    komo.view(komo.opt.animateOptimization>1, "optAnim");
    if(komo.opt.animateOptimization>2){
      komo.view_play(komo.opt.animateOptimization>3);
    }
    //  komo.plotPhaseTrajectory();
    //  rai::wait();
    //  reportProxies();
  }
}


void KOMO::set_x(const arr& x, const uintA& selectedConfigurationsOnly) {
  CHECK_EQ(timeSlices.d0, k_order+T, "configurations are not setup yet");

  timeKinematics -= rai::cpuTime();

  if(!selectedConfigurationsOnly.N){
    pathConfig.setJointState(x);
  }else{
    pathConfig.setJointState(x, timeSlices.sub(selectedConfigurationsOnly+k_order));
    HALT("this is untested...");
  }

  timeKinematics += rai::cpuTime();

  if(computeCollisions) {
    timeCollisions -= rai::cpuTime();
    pathConfig.proxies.clear();
    arr X;
    uintA collisionPairs;
    for(uint s=k_order;s<timeSlices.d0;s++){
      X = pathConfig.getFrameState(timeSlices[s]);
      if(!opt.useFCL){
        collisionPairs = swift->step(X);
      }else{
        fcl->step(X);
        collisionPairs = fcl->collisions;
      }
      collisionPairs += timeSlices.d1 * s; //fcl returns frame IDs related to 'world' -> map them into frameIDs within that time slice
      pathConfig.addProxies(collisionPairs);
    }
    pathConfig._state_proxies_isGood=true;
    timeCollisions += rai::cpuTime();
  }
}

shared_ptr<MathematicalProgram> KOMO::mp_SparseNonFactored(){
  return make_shared<Conv_KOMO_SparseNonfactored>(*this, solver==rai::KS_sparse);
}

shared_ptr<MathematicalProgram_Factored> KOMO::mp_Factored(){
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
  for(shared_ptr<GroundedObjective>& ob:objs) {
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
  for(shared_ptr<GroundedObjective>& ob:objs) {

    Graph& g = K.newSubgraph({ob->feat->shortTag(pathConfig)});
    g.newNode<double>({"order"}, {}, ob->feat->order);
    g.newNode<String>({"type"}, {}, STRING(ob->type));
    g.newNode<String>({"feature"}, {}, ob->feat->shortTag(pathConfig));
    if(ob->timeSlices.N) g.newNode<intA>({"vars"}, {}, ob->timeSlices);
//    g.copy(task->feat->getSpec(world), true);
    if(includeValues) {
      arr y;
      arrA V, J;
      for(uint l=0; l<ob->timeSlices.d0; l++) {
//        ConfigurationL Ktuple = configurations.sub(convert<uint, int>(ob->timeSlices[l]+(int)k_order));
//        ob->feat->eval(y, Jy, Ktuple);
        y = ob->feat->eval(ob->frames);
        if(isSpecial(y.J())) y.J() = unpack(y.J());

        V.append(y);
        J.append(y.J());
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
  return R.get<double>("ineq") + R.get<double>("eq");
}

double KOMO::getCosts() {
  Graph R = getReport(false);
  return R.get<double>("sos");
}

StringA KOMO::getCollisionPairs(double belowMargin){
  //similar to Configuration::getTotalPenetration
  uint nFrames = world.frames.N;
  CHECK_EQ(nFrames, timeSlices.d1, "");
  intAA collisions(nFrames);

  for(const Proxy& p:pathConfig.proxies) {
    //early check: if proxy is way out of collision, don't bother computing it precise
    if(p.d > p.a->shape->radius()+p.b->shape->radius()+.01+belowMargin) continue;
    //exact computation
    if(!p.collision)((Proxy*)&p)->calc_coll();
    double d = p.collision->getDistance();
    if(d<belowMargin){
//      cout <<"KOMO collision pair: " <<p.a->name <<"--" <<p.b->name <<" : " <<p.d <<endl;
      uint i=p.a->ID % nFrames;
      uint j=p.b->ID % nFrames;
      if(j<i){ int a=i; i=j; j=a; }
      collisions(i).setAppendInSorted(j);
    }
  }

  StringA cols;
  for(uint i=0;i<collisions.N;i++){
    for(int j:collisions(i)){
      cols.append(world.frames.elem(i)->name);
      cols.append(world.frames.elem(j)->name);
    }
  }
  cols.reshape(-1, 2);
//  cout <<"KOMO collision pairs: " <<cols;
  return cols;
}

void Conv_KOMO_SparseNonfactored::evaluate(arr& phi, arr& J, const arr& x) {
  //-- set the trajectory
  komo.set_x(x);
  if(sparse){
    komo.pathConfig.jacMode = rai::Configuration::JM_sparse;
  }else {
    komo.pathConfig.jacMode = rai::Configuration::JM_dense;
  }

  phi.resize(featureTypes.N);
  if(!!J) {
    if(sparse) {
      J.sparse().resize(phi.N, x.N, 0);
    } else {
      J.resize(phi.N, x.N).setZero();
    }
  }

  komo.sos=komo.ineq=komo.eq=0.;

  komo.timeFeatures -= rai::cpuTime();

  uint M=0;
  for(shared_ptr<GroundedObjective>& ob : komo.objs) {
      //query the task map and check dimensionalities of returns
      arr y = ob->feat->eval(ob->frames);
//      cout <<"EVAL '" <<ob->name() <<"' phi:" <<y <<endl <<y.J() <<endl<<endl;
      if(!y.N) continue;
      checkNan(y);
      if(!!J){
        CHECK(y.jac, "Jacobian needed but missing");
        CHECK_EQ(y.J().nd, 2, "");
        CHECK_EQ(y.J().d0, y.N, "");
        CHECK_EQ(y.J().d1, komo.pathConfig.getJointStateDimension(), "");
      }
//      uint d = ob->feat->dim(ob->frames);
//      if(d!=y.N){
//        d  = ob->feat->dim(ob->frames);
//        ob->feat->eval(y, y.J(), ob->frames);
//      }
//      CHECK_EQ(d, y.N, "");
      if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

      //write into phi and J
      arr yJ = y.J_reset();
      phi.setVectorBlock(y, M);

      double scale = (ob->feat->scale.N?absMax(ob->feat->scale):1.);
      CHECK_GE(scale, 1e-4, "");

      if(ob->type==OT_sos) komo.sos+=sumOfSqr(y); // / max(ob->feat->scale);
      else if(ob->type==OT_ineq) komo.ineq += sumOfPos(y) / scale;
      else if(ob->type==OT_eq) komo.eq += sumOfAbs(y) / scale;

      if(!!J) {
        if(sparse){
          yJ.sparse().reshape(J.d0, J.d1);
          yJ.sparse().colShift(M);
          J += yJ;
        }else{
          J.setMatrixBlock(yJ, M, 0);
        }
      }

      //counter for features phi
      M += y.N;
  }

  komo.timeFeatures += rai::cpuTime();

  CHECK_EQ(M, phi.N, "");
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
  if(verbose>1) os <<komo.getReport(verbose>3);
  if(verbose>2) komo.view(verbose>3, "Conv_KOMO_SparseNonfactored - report");
  if(verbose>4) komo.view_play(false);
  if(verbose>5) while(komo.view_play(true));
  if(verbose>6){
    rai::system("mkdir -p z.vid");
    komo.view_play(false, .1, "z.vid/");
  }
}

Conv_KOMO_SparseNonfactored::Conv_KOMO_SparseNonfactored(KOMO& _komo, bool sparse) : komo(_komo), sparse(sparse) {
  dimension = komo.pathConfig.getJointStateDimension();

  komo.getBounds(bounds_lo, bounds_up);

  //-- feature types
  uint M=0;
  for(shared_ptr<GroundedObjective>& ob : komo.objs) M += ob->feat->dim(ob->frames);

  featureTypes.resize(M);
  komo.featureNames.clear();
  M=0;
  for(shared_ptr<GroundedObjective>& ob : komo.objs) {
    uint m = ob->feat->dim(ob->frames);
    for(uint i=0; i<m; i++) featureTypes(M+i) = ob->type;
    for(uint j=0; j<m; j++) komo.featureNames.append(ob->feat->shortTag(komo.pathConfig));
    M += m;
  }
  if(quadraticPotentialLinear.N) {
    featureTypes.append(OT_f);
  }
  komo.featureTypes = featureTypes;
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
  //  for(shared_ptr<Objective>& ob:komo.objectives) if(ob->timeSlices.N) {
  //      CHECK_EQ(ob->timeSlices.nd, 2, "in sparse mode, vars need to be tuples of variables");
//      F += ob->timeSlices.d0;
//    }
  featureIndex.resize(F);

  //create feature index
  uint f=0;
  uint fDim = 0;
  for(shared_ptr<GroundedObjective>& ob:komo.objs) {
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

  //define signature and factorization
  dimension = komo.pathConfig.getJointStateDimension();

  featureTypes.resize(featuresDim);
  komo.featureNames.resize(featuresDim);
  uint M=0;
  for(shared_ptr<GroundedObjective>& ob : komo.objs) {
    uint m = ob->feat->dim(ob->frames);
    for(uint i=0; i<m; i++) featureTypes(M+i) = ob->type;
    for(uint i=0; i<m; i++) komo.featureNames(M+i) = "TODO";
    M += m;
  }

  komo.featureTypes = featureTypes;

  komo.getBounds(bounds_lo, bounds_up);

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

arr Conv_KOMO_FactoredNLP::getInitializationSample(const arr& previousOptima) {
  komo.run_prepare(.01);
  return komo.x;
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

  phi = F.ob2->feat->eval(F.ob2->frames);
  CHECK_EQ(phi.N, F.dim, "");

  komo.featureValues.setVectorBlock(phi, F.phiIndex);

  if(!J) return;

  CHECK_EQ(phi.N, phi.J().d0, "");
  CHECK_EQ(phi.J().nd, 2, "");
  if(absMax(phi)>1e10) RAI_MSG("WARNING phi=" <<phi);

  if(isSparseMatrix(phi.J())) {
    auto& S = phi.J().sparse();

    uint n=0;
    for(uint v:F.varIds) n += variableIndex(v).dim;
    J.resize(phi.N, n).setZero();

    for(uint k=0; k<phi.J().N; k++) {
      uint i = S.elems(k, 0);
      uint j = S.elems(k, 1);
      double x = phi.J().elem(k);
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
  } else if(isRowShifted(phi.J())){
    J = phi.J();
  } else {
    HALT("??");
#if 0 //ndef KOMO_PATH_CONFIG
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
  for(shared_ptr<Objective>& ob:komo.objectives) {
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
  DofL activeJoints;
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
  for(shared_ptr<GroundedObjective>& ob:komo.objs) {
    uint featDim = ob->feat->dim(ob->frames);
    uintA featVars;
    for(rai::Frame *f:ob->frames){ //which frames does the objective depend on?
      uint varId = frameID2VarId(f->ID); //which variable parameterized that frame
      if(varId!=UINT_MAX) featVars.setAppendInSorted(varId);
    }
    __featureIndex.append( FeatureIndexEntry{ ob, featDim, featVars } );
    featId++;
  }

  //define signature
  dimension = komo.pathConfig.getJointStateDimension();
  komo.getBounds(bounds_lo, bounds_up);
  featureTypes.clear();
  for(uint f=0; f<getNumFeatures(); f++) {
    featureTypes.append(consts<ObjectiveType>(featureIndex(f).ob->type, featureIndex(f).dim));
  }

  //define factorization
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

void Conv_KOMO_FineStructuredProblem::subSelect(const uintA& activeVariables, const uintA& conditionalVariables){
  uintA allVars;
  for(uint i:activeVariables) allVars.setAppendInSorted(i);
  for(uint i:conditionalVariables) allVars.setAppendInSorted(i);

  DofL activeJoints;
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

  phi = F.ob->feat->eval(F.ob->frames);
  J = phi.J();
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


