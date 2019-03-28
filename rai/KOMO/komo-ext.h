#pragma once

#include "komo.h"

//===========================================================================

// IK problems
void addBoxGrasp(KOMO& komo, const char* object, const char* endeff, int axis=1);
void chooseBoxGrasp(rai::KinematicWorld& K, const char* endeff, const char* object);

// motion interpolation
void addMotionTo(KOMO& komo, const arr &target_q, const StringA& target_joints, const char* endeff, double up, double down);

//===========================================================================

struct KOMO_ext : KOMO{

  //should be done by set model
  void useJointGroups(const StringA& groupNames, bool OnlyTheseOrNotThese=true);


  //-- tasks mid-level
  void setPosition(double startTime, double endTime, const char* shape, const char* shapeRel=NULL, ObjectiveType type=OT_sos, const arr& target=NoArr, double prec=1e2);
  void setOrientation(double startTime, double endTime, const char* shape, const char* shapeRel, ObjectiveType type=OT_sos, const arr& target=NoArr, double prec=1e2);
  void setVelocity(double startTime, double endTime, const char* shape, const char* shapeRel=NULL, ObjectiveType type=OT_sos, const arr& target=NoArr, double prec=1e2);
  void setAlign(double startTime, double endTime, const char* shape,  const arr& whichAxis=ARR(1.,0.,0.), const char* shapeRel=NULL, const arr& whichAxisRel=ARR(1.,0.,0.), ObjectiveType type=OT_sos, const arr& target=ARR(1.), double prec=1e2);
  void setAlignedStacking(double time, const char* object, ObjectiveType type=OT_sos, double prec=1e2);
  void setLastTaskToBeVelocity();

  //===========================================================================
  //
  // high-level 'script elements' to define tasks: typically adding multiple tasks to realize some kind of 'action'
  //

  void setAbstractTask(double phase, const Graph& facts, int verbose=0);
  //-- dynamic
  void setImpact(double time, const char* a, const char* b);
  void setOverTheEdge(double time, const char* object, const char* from, double margin=.0);
  void setInertialMotion(double startTime, double endTime, const char *object, const char *base, double g=-9.81, double c=0.);
  void setFreeGravity(double time, const char* object, const char* base="base");
  //-- tasks (cost/constraint terms) high-level (rough, for LGP)
  void setPlaceFixed(double time, const char* endeffRef, const char* object, const char* placeRef, const rai::Transformation& relPose, int verbose=0);
  void setSlideAlong(double time, const char *strick,  const char* object, const char* wall, int verbose=0);
  void setDrop(double time, const char* object, const char* from, const char* to, int verbose=0);
  void setDropEdgeFixed(double time, const char* object, const char* to, const rai::Transformation& relFrom, const rai::Transformation& relTo, int verbose=0);
  void setAttach(double time, const char* endeff, const char* object1, const char* object2, rai::Transformation& rel, int verbose=0);

  void setGrasp(double time, const char* endeffRef, const char* object, int verbose=0, double weightFromTop=3e0, double timeToLift=.15);
  void setPlace(double time, const char *endeff, const char* object, const char* placeRef, int verbose=0);
  void setHandover(double time, const char* endeffRef, const char* object, const char* prevHolder, int verbose=0);


  //DEPRECATED
  void setGraspStick(double time, const char* endeffRef, const char* object, int verbose=0, double weightFromTop=1e1, double timeToLift=.15);
  void setFine_grasp(double time, const char* endeff, const char* object, double above, double gripSize=.05, const char* gripper=NULL, const char* gripper2=NULL);
  void setTowersAlign();
  void setMoveTo(rai::KinematicWorld& world, //in initial state
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
  KOMO_ext(const rai::KinematicWorld& K) : KOMO_ext() { setModel(K); } //for compatibility only

};

//===========================================================================

inline arr finalPoseTo(rai::KinematicWorld& world,
                       rai::Frame& endeff,
                       rai::Frame& target,
                       byte whichAxesToAlign=0,
                       uint iterate=1) {
  KOMO_ext komo(world);
  komo.setTiming(1.,1);
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
