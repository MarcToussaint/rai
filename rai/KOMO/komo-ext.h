/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "komo.h"

//===========================================================================

// IK problems
void addBoxGrasp(KOMO& komo, const char* object, const char* endeff, int axis=1);
void chooseBoxGrasp(rai::Configuration& K, const char* endeff, const char* object);
void findOpposingGrasp(rai::Configuration& K, const char* fingerL, const char* fingerR, const char* object);

// motion interpolation
void addMotionTo(KOMO& komo, const arr& target_q, const StringA& target_joints, const char* endeff, double up, double down);

//===========================================================================

struct KOMO_ext : KOMO {

  void setConfigFromFile();
//  void setPathOpt(double _phases, uint stepsPerPhase=20, double timePerPhase=5.){ setTiming(_phases, stepsPerPhase, timePerPhase, 2); }
  void setSquaredQAccVelHoming(double startTime=0., double endTime=-1., double accPrec=1., double velPrec=0., double homingPrec=1e-2, int deltaFromStep=0, int deltaToStep=0);
//  void setSquaredQAccelerations(double startTime=0., double endTime=-1., double prec=1.);

  //should be done by set model
  void useJointGroups(const StringA& groupNames, bool notThese=false);

  //-- tasks mid-level
  void setHoming(double startTime=0., double endTime=-1., double prec=1e-1, const char* keyword="robot");
  void setHoldStill(double startTime, double endTime, const char* shape, double prec=1e1);
  void setPosition(double startTime, double endTime, const char* shape, const char* shapeRel=nullptr, ObjectiveType type=OT_sos, const arr& target=NoArr, double prec=1e2);
  void setOrientation(double startTime, double endTime, const char* shape, const char* shapeRel, ObjectiveType type=OT_sos, const arr& target=NoArr, double prec=1e2);
  void setVelocity(double startTime, double endTime, const char* shape, const char* shapeRel=nullptr, ObjectiveType type=OT_sos, const arr& target=NoArr, double prec=1e2);
  void setAlign(double startTime, double endTime, const char* shape,  const arr& whichAxis=ARR(1., 0., 0.), const char* shapeRel=nullptr, const arr& whichAxisRel=ARR(1., 0., 0.), ObjectiveType type=OT_sos, const arr& target=ARR(1.), double prec=1e2);
  void setAlignedStacking(double time, const char* object, ObjectiveType type=OT_sos, double prec=1e2);
  void setLastTaskToBeVelocity();

  //-- core objective symbols of skeletons
  void add_touch(double startTime, double endTime, const char* shape1, const char* shape2, ObjectiveType type=OT_eq, const arr& target=NoArr, double prec=1e2);
  void add_aboveBox(double startTime, double endTime, const char* shape1, const char* shape2, double prec=1e1);
  void add_insideBox(double startTime, double endTime, const char* shape1, const char* shape2, double prec=1e1);
//  void add_impulse(double time, const char* shape1, const char* shape2, ObjectiveType type=OT_eq, double prec=1e1);
  void add_stable(double time,  const char* shape1, const char* shape2, ObjectiveType type=OT_eq, double prec=1e1);

  //dinos... can't get rid of them yet
  void setGraspSlide(double time, const char* stick, const char* object, const char* placeRef, int verbose=0);
  void setPush(double startTime, double endTime, const char* stick, const char* object, const char* table, int verbose=0);
  void setKS_slider(double time, double endTime, bool before, const char* obj, const char* slider, const char* table);

  //===========================================================================
  //
  // high-level 'script elements' to define tasks: typically adding multiple tasks to realize some kind of 'action'
  //

  //-- dynamic
  void setImpact(double time, const char* a, const char* b);
  void setOverTheEdge(double time, const char* object, const char* from, double margin=.0);
  void setInertialMotion(double startTime, double endTime, const char* object, const char* base, double g=-9.81, double c=0.);
  void setFreeGravity(double time, const char* object, const char* base="base");
  //-- tasks (cost/constraint terms) high-level (rough, for LGP)
  void setPlaceFixed(double time, const char* endeffRef, const char* object, const char* placeRef, const rai::Transformation& relPose, int verbose=0);
  void setSlideAlong(double time, const char* strick,  const char* object, const char* wall, int verbose=0);
  void setDrop(double time, const char* object, const char* from, const char* to, int verbose=0);
  void setDropEdgeFixed(double time, const char* object, const char* to, const rai::Transformation& relFrom, const rai::Transformation& relTo, int verbose=0);
  void setAttach(double time, const char* endeff, const char* object1, const char* object2, rai::Transformation& rel, int verbose=0);

  void setGrasp(double time, double endTime, const char* endeffRef, const char* object, int verbose=0, double weightFromTop=3e0, double timeToLift=.15);
  void setPlace(double time, const char* endeff, const char* object, const char* placeRef, int verbose=0);
  void setHandover(double time, const char* endeffRef, const char* object, const char* prevHolder, int verbose=0);

  //DEPRECATED
  void setGraspStick(double time, const char* endeffRef, const char* object, int verbose=0, double weightFromTop=1e1, double timeToLift=.15);
  void setFine_grasp(double time, const char* endeff, const char* object, double above, double gripSize=.05, const char* gripper=nullptr, const char* gripper2=nullptr);
  void setTowersAlign();
  void setMoveTo(rai::Configuration& world, //in initial state
                 rai::Frame& endeff,         //endeffector to be moved
                 rai::Frame& target,         //target shape
                 byte whichAxesToAlign=0);   //bit coded options to align axes

  //===========================================================================
  //
  // optimizing, getting results, and verbosity
  //

  void getPhysicsReference(uint subSteps=10, int display=0);
  void playInPhysics(uint subSteps=10, bool display=false);
  PhysXInterface& physx() { return world.physx(); }

  //===========================================================================
  //
  // internal (kind of private); old interface of 'KOMO'; kept for compatibility
  //

  //-- (not much in use..) specs gives as logic expressions in a Graph (or config file)
  KOMO_ext() : KOMO() {}
  KOMO_ext(const rai::Configuration& K) : KOMO_ext() { setModel(K); } //for compatibility only

};

//===========================================================================

inline arr finalPoseTo(rai::Configuration& world,
                       rai::Frame& endeff,
                       rai::Frame& target,
                       byte whichAxesToAlign=0,
                       uint iterate=1) {
  KOMO_ext komo(world);
  komo.setTiming(1., 1);
  komo.setMoveTo(world, endeff, target, whichAxesToAlign);
  komo.run();
  return komo.x;
}

//===========================================================================

inline arr getVelocities(const arr& q, double tau) {
  arr v;
  v.resizeAs(q);
  v.setZero();
  for(uint t=1; t<q.d0-1; t++) {
    v[t] = (q[t+1]-q[t-1])/(2.*tau);
  }
  return v;
}
