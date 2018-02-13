/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#pragma once
#include <Kin/kin.h>
#include <Optim/optimization.h>
#include <Optim/constrained.h>
#include <Optim/KOMO_Problem.h>
#include "task.h"
#include <Kin/flag.h>

//===========================================================================

struct KOMO{

  //-- the problem definition
  double maxPhase;             ///< number of phases (roughly: actions), but phase is continuous valued
  uint stepsPerPhase;          ///< time slices per phase
  uint T;                      ///< total number of time steps
  double tau;                  ///< real time duration of single step (used when evaluating task space velocities/accelerations)
  uint k_order;                ///< the (Markov) order of the KOMO problem (default 2)
  mlr::Array<Task*> tasks;     ///< list of tasks
  mlr::Array<mlr::Flag*> flags;     ///< list of flaggings that are applied to the frames/joints in the configurations and modify tasks
  mlr::Array<mlr::KinematicSwitch*> switches;  ///< list of kinematic switches along the motion

  //-- internals
  mlr::KinematicWorld world;   ///< original world; which is the blueprint for all time-slice worlds (almost const: only makeConvexHulls modifies it)
  WorldL configurations;       ///< copies for each time slice; including kinematic switches; only these are optimized
  bool useSwift;               ///< whether swift (collisions/proxies) is evaluated whenever new configurations are set (needed if tasks read proxy list)

  //-- optimizer
  OptConstrained *opt;         ///< optimizer; created in run()
  arr x, dual;                 ///< the primal and dual solution
  arr z, splineB;              ///< when a spline representation is used: z are the nodes; splineB the B-spline matrix; x = splineB * z

  //-- verbosity only: buffers of all feature values computed on last set_x
  arrA featureValues;           ///< storage of all features in all time slices
  mlr::Array<ObjectiveTypeA> featureTypes;  ///< storage of all feature-types in all time slices
  arr dualSolution;             ///< the dual solution computed during constrained optimization
  struct OpenGL *gl;            ///< internal only: used in 'displayTrajectory'
  int verbose;                  ///< verbosity level
  double runTime=0.;            ///< just measure run time
  ofstream *fil=NULL;

  KOMO();
  ~KOMO();

  //-- setup the problem
  void setModel(const mlr::KinematicWorld& K,
                bool _useSwift=true,  //disabling swift: no collisions, much faster
                bool meldFixedJoints=false, bool makeConvexHulls=false, bool computeOptimalSSBoxes=false, bool activateAllContacts=false);
  void useJointGroups(const StringA& groupNames, bool OnlyTheseOrNotThese=true);
  void setTiming(double _phases=1., uint _stepsPerPhase=10, double durationPerPhase=5., uint _k_order=2);
  void setPairedTimes();
  void activateCollisions(const char* s1, const char* s2);
  void deactivateCollisions(const char* s1, const char* s2);

  //-- higher-level setup defaults
  void setConfigFromFile();
  void setIKOpt();
  void setPoseOpt();
  void setSequenceOpt(double _phases);
  void setPathOpt(double _phases, uint stepsPerPhase=20, double timePerPhase=5.);

  //===========================================================================
  //
  // lowest level way to define tasks: basic methods to add any single task or switch
  //

  /** THESE ARE THE TWO MOST IMPORTANT METHODS TO DEFINE A PROBLEM
   * they allow the user to add a cost task, or a kinematic switch in the problem definition
   * Typically, the user does not call them directly, but uses the many methods below
   * Think of all of the below as examples for how to set arbirary tasks/switches yourself */
  struct Task* setTask(double startTime, double endTime, TaskMap* map, ObjectiveType type=OT_sumOfSqr, const arr& target=NoArr, double prec=1e2, uint order=0, int deltaStep=0);
  void setFlag(double time, mlr::Flag* fl, int deltaStep=0);
  void setKinematicSwitch(double time, bool before, mlr::KinematicSwitch* sw);
  void setKinematicSwitch(double time, bool before, const char *type, const char* ref1, const char* ref2, const mlr::Transformation& jFrom=NoTransformation);

  //===========================================================================
  //
  // mid-level ways to define tasks: typically adding one specific task
  //

  //-- tasks (transitions) mid-level
  void setHoming(double startTime=-1., double endTime=-1., double prec=1e-1, const char *keyword="robot");
  void setSquaredQAccelerations(double startTime=-1., double endTime=-1., double prec=1.);
  void setSquaredQVelocities(double startTime=-1., double endTime=-1., double prec=1.);
  void setFixEffectiveJoints(double startTime=-1., double endTime=-1., double prec=1e3);
  void setFixSwitchedObjects(double startTime=-1., double endTime=-1., double prec=1e3);
  void setSquaredQuaternionNorms(double startTime=-1., double endTime=-1., double prec=1e1);

  //-- tasks (tasks) mid-level
  void setHoldStill(double startTime, double endTime, const char* shape, double prec=1e2);
  void setPosition(double startTime, double endTime, const char* shape, const char* shapeRel=NULL, ObjectiveType type=OT_sumOfSqr, const arr& target=NoArr, double prec=1e2);
  void setOrientation(double startTime, double endTime, const char* shape, const char* shapeRel, ObjectiveType type=OT_sumOfSqr, const arr& target=NoArr, double prec=1e2);
  void setVelocity(double startTime, double endTime, const char* shape, const char* shapeRel=NULL, ObjectiveType type=OT_sumOfSqr, const arr& target=NoArr, double prec=1e2);
  void setAlign(double startTime, double endTime, const char* shape,  const arr& whichAxis=ARR(1.,0.,0.), const char* shapeRel=NULL, const arr& whichAxisRel=ARR(1.,0.,0.), ObjectiveType type=OT_sumOfSqr, const arr& target=ARR(1.), double prec=1e2);
  void setTouch(double startTime, double endTime, const char* shape1, const char* shape2, ObjectiveType type=OT_eq, const arr& target=NoArr, double prec=1e2);
  void setAlignedStacking(double time, const char* object, ObjectiveType type=OT_sumOfSqr, double prec=1e2);
  void setLastTaskToBeVelocity();
  void setCollisions(bool hardConstraint, double margin=.05, double prec=1.);
  void setLimits(bool hardConstraint, double margin=.05, double prec=1.);
  void setSlow(double startTime, double endTime, double prec=1e2, bool hardConstrained=false);
  void setSlowAround(double time, double delta, double prec=1e2, bool hardConstrained=false);

  //-- kinematic switches mid-level
  void setKS_placeOn(double time, bool before, const char* obj, const char* table, bool actuated=false);
  void setKS_slider(double time, bool before, const char* obj, const char* slider, const char* table);

  //===========================================================================
  //
  // high-level 'script elements' to define tasks: typically adding multiple tasks to realize some kind of 'action'
  //

  //-- dynamic
  void setImpact(double time, const char* a, const char* b);
  void setOverTheEdge(double time, const char* object, const char* from, double margin=.0);
  void setInertialMotion(double startTime, double endTime, const char *object, const char *base, double g=-9.81, double c=0.);
  void setFreeGravity(double time, const char* object, const char* base="base");

  //-- tasks (cost/constraint terms) high-level (rough, for LGP)
  void setGrasp(double time, const char* endeffRef, const char* object, int verbose=0, double weightFromTop=1e1, double timeToLift=.15);
  void setPlace(double time, const char *endeff, const char* object, const char* placeRef, int verbose=0);
  void setPlaceFixed(double time, const char* endeffRef, const char* object, const char* placeRef, const mlr::Transformation& relPose, int verbose=0);
  void setHandover(double time, const char* endeffRef, const char* object, const char* prevHolder, int verbose=0);
  void setPush(double startTime, double endTime, const char* stick, const char* object, const char* table, int verbose=0);
  void setGraspSlide(double time, const char* stick, const char* object, const char* placeRef, int verbose=0);
  void setSlideAlong(double time, const char *strick,  const char* object, const char* wall, int verbose=0);
  void setDrop(double time, const char* object, const char* from, const char* to, int verbose=0);
  void setDropEdgeFixed(double time, const char* object, const char* to, const mlr::Transformation& relFrom, const mlr::Transformation& relTo, int verbose=0);

  //-- tasks - logic level (used within LGP)
  void setAbstractTask(double phase, const Graph& facts, int verbose=0);

  //DEPRECATED
  void setGraspStick(double time, const char* endeffRef, const char* object, int verbose=0, double weightFromTop=1e1, double timeToLift=.15);
  void setGraspSlide(double startTime, double endTime, const char* endeffRef, const char* object, const char* placeRef, int verbose=0, double weightFromTop=1e1);
  void setAttach(double time, const char* endeff, const char* object1, const char* object2, mlr::Transformation& rel, int verbose=0);
  void setFine_grasp(double time, const char* endeff, const char* object, double above, double gripSize=.05, const char* gripper=NULL, const char* gripper2=NULL);
  void setTowersAlign();
  void setMoveTo(mlr::KinematicWorld& world, //in initial state
                 mlr::Frame& endeff,         //endeffector to be moved
                 mlr::Frame& target,         //target shape
                 byte whichAxesToAlign=0);   //bit coded options to align axes

  //===========================================================================
  //
  // optimizing and verbosity
  //

  //-- optimization macros
  void setSpline(uint splineT);   ///< optimize B-spline nodes instead of the path; splineT specifies the time steps per node
  void reset(double initNoise=.01);      ///< reset the optimizer (initializes x to a default path)
  void run();                     ///< run the optimization (using OptConstrained -- its parameters are read from the cfg file)
  void getPhysicsReference(uint subSteps=10, int display=0);
  void playInPhysics(uint subSteps=10, bool display=false);
  arr getPath(const StringA& joints);
  void reportProblem(ostream &os=std::cout);
  Graph getReport(bool gnuplt=false, int reportFeatures=0, ostream& featuresOs=std::cout); ///< return a 'dictionary' summarizing the optimization results (optional: gnuplot task costs; output detailed cost features per time slice)
  void reportProxies(ostream& os=std::cout); ///< report the proxies (collisions) for each time slice
  void checkGradients();          ///< checks all gradients numerically
  void plotTrajectory();
  bool displayTrajectory(double delay=0.01, bool watch=true, const char* saveVideoPrefix=NULL); ///< display the trajectory; use "vid/z." as vid prefix
  mlr::Camera& displayCamera();   ///< access to the display camera to change the view
  PhysXInterface& physx(){ return world.physx(); }


  //===========================================================================
  //
  // internal (kind of private); old interface of 'KOMO'; kept for compatibility
  //

  //-- (not much in use..) specs gives as logic expressions in a Graph (or config file)
  KOMO(const mlr::KinematicWorld& K) : KOMO() { setModel(K); } //for compatibility only
//  KOMO(const Graph& specs);
//  void init(const Graph& specs);
//  void setFact(const char* fact);
//  bool parseTask(const Node *n, int stepsPerPhase=-1);           ///< read a single task from a node-spec
  void clearTasks();
  Task* addTask(const char* name, TaskMap *map, const ObjectiveType& termType); ///< manually add a task
  void setupConfigurations();   ///< this creates the @configurations@, that is, copies the original world T times (after setTiming!) perhaps modified by KINEMATIC SWITCHES and FLAGS
  arr getInitialization();      ///< this reads out the initial state trajectory after 'setupConfigurations'
  void set_x(const arr& x);            ///< set the state trajectory of all configurations
  uint dim_x(uint t) { return configurations(t+k_order)->getJointStateDimension(); }

  struct Conv_MotionProblem_KOMO_Problem : KOMO_Problem {
    KOMO& komo;
    uint dimPhi;
    arr prevLambda;
    uintA phiIndex, phiDim;

    Conv_MotionProblem_KOMO_Problem(KOMO& _komo) : komo(_komo){}

    virtual uint get_k(){ return komo.k_order; }
    virtual void getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes);
    virtual void phi(arr& phi, arrA& J, arrA& H, uintA& featureTimes, ObjectiveTypeA& tt, const arr& x, arr& lambda);
  } komo_problem;

};

//===========================================================================

inline arr finalPoseTo(mlr::KinematicWorld& world,
                       mlr::Frame& endeff,
                       mlr::Frame& target,
                       byte whichAxesToAlign=0,
                       uint iterate=1){
  KOMO komo(world);
  komo.setTiming(1.,1);
  komo.setMoveTo(world, endeff, target, whichAxesToAlign);
  komo.run();
  return komo.x;
}

//===========================================================================

inline arr getVelocities(const arr& q, double tau){
  arr v;
  v.resizeAs(q);
  v.setZero();
  for(uint t=1;t<q.d0-1;t++){
    v[t] = (q[t+1]-q[t-1])/(2.*tau);
  }
  return v;
}
