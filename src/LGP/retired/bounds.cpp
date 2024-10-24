/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "bounds.h"
//#include "../KOMO/switch.h"
#include "../Kin/F_qFeatures.h"
#include "../Kin/F_forces.h"
#include "../Kin/F_pose.h"

double conv_step2time(int step, uint stepsPerPhase);

template<> const char* rai::Enum<BoundType>::names []= {
  "symbolic",
  "pose",
  "seq",
  "path",
  "seqPath",
  "seqVelPath",
  "poseFromSub",
  "max",
  nullptr
};

namespace rai {
Array<SkeletonSymbol> modes = { SY_stable, SY_stableOn, SY_dynamic, SY_dynamicOn, SY_dynamicTrans, };
}

shared_ptr<KOMO_based_bound> skeleton2Bound(shared_ptr<KOMO>& komo, BoundType boundType, const rai::Skeleton& S,
    const rai::Configuration& startKinematics,
    bool collisions, const arrA& waypoints) {

  if(boundType==BD_pose)
    return make_shared<PoseBound>(komo, S, startKinematics, collisions);

  if(boundType==BD_seq)
    return make_shared<SeqBound>(komo, S, startKinematics, collisions);

  if(boundType==BD_path)
    return make_shared<PathBound>(komo, S, startKinematics, collisions);

  if(boundType==BD_seqPath)
    return make_shared<SeqPathBound>(komo, S, startKinematics, collisions, waypoints);

  HALT("should not be here!");

  return shared_ptr<KOMO_based_bound>();
}

//===========================================================================

PoseBound::PoseBound(shared_ptr<KOMO>& komo,
                     const rai::Skeleton& S, const rai::Configuration& startKinematics,
                     bool collisions)
  : KOMO_based_bound(komo) {

  double maxPhase = S.getMaxPhase();
  komo->clearObjectives();

  //-- prepare the komo problem
  double optHorizon=maxPhase;
  if(optHorizon<1.) optHorizon=maxPhase=1.;
  if(optHorizon>2.) optHorizon=2.;

  //-- remove non-switches
  rai::Skeleton finalS;
  for(const rai::SkeletonEntry& s:S.S) {
    if(rai::modes.contains(s.symbol)
        || s.phase0>=maxPhase) {
      rai::SkeletonEntry& fs = finalS.S.append(s);
      fs.phase0 -= maxPhase-optHorizon;
      fs.phase1 -= maxPhase-optHorizon;
      if(fs.phase0<0.) fs.phase0=0.;
      if(fs.phase1<0.) fs.phase1=0.;
    }
  }
#if 0
  //-- grep only the latest entries in the skeleton
  Skeleton finalS;
  for(const SkeletonEntry& s:S) if(s.phase0>=maxPhase) {
      finalS.append(s);
      finalS.last().phase0 -= maxPhase-1.;
      finalS.last().phase1 -= maxPhase-1.;
    }
#endif

  if(komo->verbose>1) {
    cout <<"POSE skeleton:" <<endl;
    finalS.write(cout, finalS.getSwitches(startKinematics));
  }

  komo->setConfig(startKinematics, collisions);
  komo->setTiming(optHorizon, 1, 10., 1);

  komo->addQuaternionNorms();
#if 0
  komo->setHoming(0., -1., 1e-2);
  komo->setSquaredQVelocities(1., -1., 1e-1); //IMPORTANT: do not penalize transitions of from prefix to x_{0} -> x_{0} is 'loose'
#else
  komo->addControlObjective({}, 1, 1e-2);
  komo->addControlObjective({}, 0, 1e-2);
#endif

  finalS.addObjectives(*komo);

  //-- deactivate all velocity objectives except for transition
  for(shared_ptr<Objective>& o:komo->objectives) {
    if(o->feat->order>0
        && !std::dynamic_pointer_cast<F_qItself>(o->feat)
        && !std::dynamic_pointer_cast<F_Pose>(o->feat)
        && !std::dynamic_pointer_cast<F_PoseRel>(o->feat)) {
      o->times= {1e6};
    }
  }
  for(shared_ptr<GroundedObjective>& o:komo->objs) {
    if(o->feat->order>0
        && !std::dynamic_pointer_cast<F_qItself>(o->feat)
        && !std::dynamic_pointer_cast<F_Pose>(o->feat)
        && !std::dynamic_pointer_cast<F_PoseRel>(o->feat)) {
      o->feat.reset();
    }
  }
  for(uint i=komo->objs.N; i--;) if(!komo->objs(i)->feat) {
      komo->objs.remove(i);
    }

  if(collisions) komo->add_collision(false);

  komo->run_prepare(.01);
  //      komo->setPairedTimes();
}

SeqBound::SeqBound(shared_ptr<KOMO>& komo,
                   const rai::Skeleton& S, const rai::Configuration& startKinematics,
                   bool collisions)
  : KOMO_based_bound(komo) {

  double maxPhase = S.getMaxPhase();
  komo->clearObjectives();

  komo->setConfig(startKinematics, collisions);
  komo->setTiming(maxPhase+1., 1, 5., 1);
//  komo->solver=rai::KS_sparse; //sparseOptimization = true;
  komo->animateOptimization = 0;

  komo->addQuaternionNorms();
#if 0
  komo->setHoming(0., -1., 1e-2);
  komo->setSquaredQVelocities(0., -1., 1e-2);
#else
  komo->addControlObjective({}, 1, 1e-2);
  komo->addControlObjective({}, 0, 1e-2);
#endif
  S.addObjectives(*komo);

  if(collisions) komo->add_collision(true);

  komo->run_prepare(.01);
//      komo->setPairedTimes();
  //      cout <<komo->getPath_times() <<endl;

}

PathBound::PathBound(shared_ptr<KOMO>& komo,
                     const rai::Skeleton& S, const rai::Configuration& startKinematics,
                     bool collisions)
  : KOMO_based_bound(komo) {

  double maxPhase = S.getMaxPhase();
  komo->clearObjectives();

  komo->setConfig(startKinematics, collisions);
  uint stepsPerPhase = rai::getParameter<uint>("LGP/stepsPerPhase", 10);
  uint pathOrder = rai::getParameter<uint>("LGP/pathOrder", 2);
  komo->setTiming(maxPhase+.5, stepsPerPhase, 10., pathOrder);
  komo->animateOptimization = 0;

  komo->addQuaternionNorms();
#if 0
  komo->setHoming(0., -1., 1e-2);
  if(pathOrder==1) komo->setSquaredQVelocities();
  else komo->setSquaredQAccelerations();
#else
  komo->addControlObjective({}, 2, 1.);
  komo->addControlObjective({}, 0, 1e-2);
#endif

  S.addObjectives(*komo);

  if(collisions) komo->add_collision(true, 0., 1e1);

  komo->run_prepare(.01);
  //      cout <<komo->getPath_times() <<endl;

}

SeqPathBound::SeqPathBound(shared_ptr<KOMO>& komo,
                           const rai::Skeleton& S, const rai::Configuration& startKinematics,
                           bool collisions, const arrA& waypoints)
  : KOMO_based_bound(komo) {

  double maxPhase = S.getMaxPhase();
  komo->clearObjectives();

  komo->setConfig(startKinematics, collisions);
  uint stepsPerPhase = rai::getParameter<uint>("LGP/stepsPerPhase", 10);
  uint pathOrder = rai::getParameter<uint>("LGP/pathOrder", 2);
  komo->setTiming(maxPhase+.5, stepsPerPhase, 10., pathOrder);
  komo->animateOptimization = 0;

  komo->addQuaternionNorms();
#if 0
  komo->setHoming(0., -1., 1e-2);
  if(pathOrder==1) komo->setSquaredQVelocities();
  else komo->setSquaredQAccelerations();
#else
  komo->addControlObjective({}, 2, 1.);
  komo->addControlObjective({}, 0, 1e-2);
#endif

  uint T = floor(maxPhase+.5);
  uint waypointsStepsPerPhase = waypoints.N/(T+1);
  CHECK_EQ(waypoints.N, waypointsStepsPerPhase * (T+1), "waypoint steps not clear");
#if 0 //impose waypoint costs?
  for(uint i=0; i<waypoints.N-1; i++) {
    komo->addObjective(arr{conv_step2time(i, waypointsStepsPerPhase)), FS_qItself, {}, OT_sos, {1e-1}, waypoints(i)};
  }
#endif

  S.addObjectives(*komo);
  //delete all added objectives! -> only keep switches
  //      uint O = komo->objectives.N;
  //      for(uint i=O; i<komo->objectives.N; i++) delete komo->objectives(i);
  //      komo->objectives.resizeCopy(O);

  if(collisions) komo->add_collision(true, 0., 1e1);

  komo->run_prepare(.01);
  komo->initWithWaypoints(waypoints, waypointsStepsPerPhase, true);
  //      cout <<komo->getPath_times() <<endl;

}

SeqVelPathBound::SeqVelPathBound(shared_ptr<KOMO>& komo,
                                 const rai::Skeleton& S, const rai::Configuration& startKinematics,
                                 bool collisions, const arrA& waypoints)
  : KOMO_based_bound(komo) {

  double maxPhase = S.getMaxPhase();
  komo->clearObjectives();

  komo->setConfig(startKinematics, collisions);
  uint stepsPerPhase = rai::getParameter<uint>("LGP/stepsPerPhase", 10);
  komo->setTiming(maxPhase+.5, stepsPerPhase, 10., 1);

  komo->addControlObjective({}, 1, 1.);
  komo->addControlObjective({}, 0, 1e-2);
  komo->addQuaternionNorms();

  CHECK_EQ(waypoints.N-1, floor(maxPhase+.5), "");
  for(uint i=0; i<waypoints.N-1; i++) {
    komo->addObjective(arr{double(i+1)), FS_qItself, {}, OT_sos, {1e-1}, waypoints(i)};
//        komo->addObjective(arr{double(i+1)), FS_qItself, {}, OT_eq, {1e0}, waypoints(i)};
  }
//      uint O = komo->objectives.N;

  S.setKOMO(*komo);
  //delete all added objectives! -> only keep switches
//      for(uint i=O; i<komo->objectives.N; i++) delete komo->objectives(i);
//      komo->objectives.resizeCopy(O);

  if(collisions) komo->add_collision(true, 0, 1e1);

  komo->run_prepare(.01);
  komo->initWithWaypoints(waypoints, false);
  //      cout <<komo->getPath_times() <<endl;

}

