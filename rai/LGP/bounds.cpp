/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "bounds.h"
//#include "../Kin/switch.h"
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

rai::Array<SkeletonSymbol> modes = { SY_stable, SY_stableOn, SY_dynamic, SY_dynamicOn, SY_dynamicTrans, };

ptr<ComputeObject> skeleton2Bound(ptr<KOMO>& komo, BoundType boundType, const Skeleton& S,
                                  const rai::Configuration& startKinematics,
                                  bool collisions, const arrA& waypoints) {

  if(boundType==BD_pose)
    return make_shared<PoseBound>(komo, S, startKinematics, collisions);

  if(boundType==BD_seq || boundType==BD_poseFromSeq)
    return make_shared<SeqBound>(komo, S, startKinematics, collisions);

  if(boundType==BD_path)
    return make_shared<PathBound>(komo, S, startKinematics, collisions);

  if(boundType==BD_seqPath)
    return make_shared<SeqPathBound>(komo, S, startKinematics, collisions, waypoints);

  if(boundType==BD_seqVelPath)
    return make_shared<SeqVelPathBound>(komo, S, startKinematics, collisions, waypoints);

  HALT("should not be here!");

  return ptr<ComputeObject>();
}

//===========================================================================

double getMaxPhase(const Skeleton& S) {
  double maxPhase=0;
  for(const SkeletonEntry& s:S) {
    if(s.phase0>maxPhase) maxPhase=s.phase0;
    if(s.phase1>maxPhase) maxPhase=s.phase1;
  }
  return maxPhase;
}

PoseBound::PoseBound(ptr<KOMO>& komo,
                     const Skeleton& S, const rai::Configuration& startKinematics,
                     bool collisions)
  : KOMO_based_bound(komo) {

  double maxPhase = getMaxPhase(S);
  komo->clearObjectives();

  //-- prepare the komo problem
  double optHorizon=maxPhase;
  if(optHorizon<1.) optHorizon=maxPhase=1.;
  if(optHorizon>2.) optHorizon=2.;

  //-- remove non-switches
  Skeleton finalS;
  for(const SkeletonEntry& s:S) {
    if(modes.contains(s.symbol)
        || s.phase0>=maxPhase) {
      SkeletonEntry& fs = finalS.append(s);
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
    writeSkeleton(cout, finalS, getSwitchesFromSkeleton(finalS, komo->world));
  }

  komo->setModel(startKinematics, collisions);
  komo->setTiming(optHorizon, 1, 10., 1);

  komo->addSquaredQuaternionNorms();
#if 0
  komo->setHoming(0., -1., 1e-2);
  komo->setSquaredQVelocities(1., -1., 1e-1); //IMPORTANT: do not penalize transitions of from prefix to x_{0} -> x_{0} is 'loose'
#else
  komo->add_qControlObjective({}, 1, 1e-2);
  komo->add_qControlObjective({}, 0, 1e-2);
#endif

  komo->setSkeleton(finalS);

  //-- deactivate all velocity objectives except for transition
  //      for(Objective *o:komo->objectives){
  //        if((std::dynamic_pointer_cast<TM_ZeroQVel>(o->feat)
  //           || std::dynamic_pointer_cast<TM_Default>(o->feat))
  //           && o->feat->order==1){
  //          o->vars.clear();
  //        }
  //      }
  for(ptr<Objective>& o:komo->objectives) {
    if(!std::dynamic_pointer_cast<F_qItself>(o->feat)
//        && !std::dynamic_pointer_cast<TM_NoJumpFromParent>(o->feat)
       && !std::dynamic_pointer_cast<F_Pose>(o->feat)
       && !std::dynamic_pointer_cast<F_PoseRel>(o->feat)
       && o->feat->order>0) {
      o->configs.clear();
    }
  }
  for(ptr<GroundedObjective>& o:komo->objs) {
    if(!std::dynamic_pointer_cast<F_qItself>(o->feat)
       && !std::dynamic_pointer_cast<F_Pose>(o->feat)
       && !std::dynamic_pointer_cast<F_PoseRel>(o->feat)
       && o->feat->order>0) {
      o->feat.reset();
    }
  }
  for(uint i=komo->objs.N;i--;) if(!komo->objs(i)->feat){
    komo->objs.remove(i);
  }


  if(collisions) komo->add_collision(false);

  komo->run_prepare(.01);
  //      komo->setPairedTimes();
}

SeqBound::SeqBound(ptr<KOMO>& komo,
                   const Skeleton& S, const rai::Configuration& startKinematics,
                   bool collisions)
  : KOMO_based_bound(komo) {

  double maxPhase = getMaxPhase(S);
  komo->clearObjectives();

  komo->setModel(startKinematics, collisions);
  komo->setTiming(maxPhase+1., 1, 5., 1);
//  komo->solver=rai::KS_sparse; //sparseOptimization = true;
  komo->animateOptimization = 0;

  komo->addSquaredQuaternionNorms();
#if 0
  komo->setHoming(0., -1., 1e-2);
  komo->setSquaredQVelocities(0., -1., 1e-2);
#else
  komo->add_qControlObjective({}, 1, 1e-2);
  komo->add_qControlObjective({}, 0, 1e-2);
#endif
  komo->setSkeleton(S);

  if(collisions) komo->add_collision(true);

  komo->run_prepare(.01);
//      komo->setPairedTimes();
  //      cout <<komo->getPath_times() <<endl;

}

PathBound::PathBound(ptr<KOMO>& komo,
                     const Skeleton& S, const rai::Configuration& startKinematics,
                     bool collisions)
  : KOMO_based_bound(komo) {

  double maxPhase = getMaxPhase(S);
  komo->clearObjectives();

  komo->setModel(startKinematics, collisions);
  uint stepsPerPhase = rai::getParameter<uint>("LGP/stepsPerPhase", 10);
  uint pathOrder = rai::getParameter<uint>("LGP/pathOrder", 2);
  komo->setTiming(maxPhase+.5, stepsPerPhase, 10., pathOrder);
  komo->animateOptimization = 0;

  komo->addSquaredQuaternionNorms();
#if 0
  komo->setHoming(0., -1., 1e-2);
  if(pathOrder==1) komo->setSquaredQVelocities();
  else komo->setSquaredQAccelerations();
#else
  komo->add_qControlObjective({}, 2, 1.);
  komo->add_qControlObjective({}, 0, 1e-2);
#endif

  komo->setSkeleton(S);

  if(collisions) komo->add_collision(true, 0., 1e1);

  komo->run_prepare(.01);
  //      cout <<komo->getPath_times() <<endl;

}

SeqPathBound::SeqPathBound(ptr<KOMO>& komo,
                           const Skeleton& S, const rai::Configuration& startKinematics,
                           bool collisions, const arrA& waypoints)
  : KOMO_based_bound(komo) {

  double maxPhase = getMaxPhase(S);
  komo->clearObjectives();

  komo->setModel(startKinematics, collisions);
  uint stepsPerPhase = rai::getParameter<uint>("LGP/stepsPerPhase", 10);
  uint pathOrder = rai::getParameter<uint>("LGP/pathOrder", 2);
  komo->setTiming(maxPhase+.5, stepsPerPhase, 10., pathOrder);
  komo->animateOptimization = 0;

  komo->addSquaredQuaternionNorms();
#if 0
  komo->setHoming(0., -1., 1e-2);
  if(pathOrder==1) komo->setSquaredQVelocities();
  else komo->setSquaredQAccelerations();
#else
  komo->add_qControlObjective({}, 2, 1.);
  komo->add_qControlObjective({}, 0, 1e-2);
#endif

  uint T = floor(maxPhase+.5);
  uint waypointsStepsPerPhase = waypoints.N/(T+1);
  CHECK_EQ(waypoints.N, waypointsStepsPerPhase * (T+1), "waypoint steps not clear");
#if 0 //impose waypoint costs?
  for(uint i=0; i<waypoints.N-1; i++) {
    komo->addObjective(ARR(conv_step2time(i, waypointsStepsPerPhase)), FS_qItself, {}, OT_sos, {1e-1}, waypoints(i));
  }
#endif

  komo->setSkeleton(S);
  //delete all added objectives! -> only keep switches
  //      uint O = komo->objectives.N;
  //      for(uint i=O; i<komo->objectives.N; i++) delete komo->objectives(i);
  //      komo->objectives.resizeCopy(O);

  if(collisions) komo->add_collision(true, 0., 1e1);

  komo->run_prepare(.01);
  komo->initWithWaypoints(waypoints, waypointsStepsPerPhase);
  //      cout <<komo->getPath_times() <<endl;

}

SeqVelPathBound::SeqVelPathBound(ptr<KOMO>& komo,
                                 const Skeleton& S, const rai::Configuration& startKinematics,
                                 bool collisions, const arrA& waypoints)
  : KOMO_based_bound(komo) {

  double maxPhase = getMaxPhase(S);
  komo->clearObjectives();

  komo->setModel(startKinematics, collisions);
  uint stepsPerPhase = rai::getParameter<uint>("LGP/stepsPerPhase", 10);
  komo->setTiming(maxPhase+.5, stepsPerPhase, 10., 1);

  komo->add_qControlObjective({}, 1, 1.);
  komo->add_qControlObjective({}, 0, 1e-2);
  komo->addSquaredQuaternionNorms();

  CHECK_EQ(waypoints.N-1, floor(maxPhase+.5), "");
  for(uint i=0; i<waypoints.N-1; i++) {
    komo->addObjective(ARR(double(i+1)), FS_qItself, {}, OT_sos, {1e-1}, waypoints(i));
//        komo->addObjective(ARR(double(i+1)), FS_qItself, {}, OT_eq, {1e0}, waypoints(i));
  }
//      uint O = komo->objectives.N;

  komo->setSkeleton(S);
  //delete all added objectives! -> only keep switches
//      for(uint i=O; i<komo->objectives.N; i++) delete komo->objectives(i);
//      komo->objectives.resizeCopy(O);

  if(collisions) komo->add_collision(true, 0, 1e1);

  komo->run_prepare(.01);
  komo->initWithWaypoints(waypoints, false);
  //      cout <<komo->getPath_times() <<endl;

}

