/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once
#include <Kin/kin.h>
#include <Optim/optimization.h>
#include <Optim/constrained.h>
#include <Optim/KOMO_Problem.h>
#include "task.h"
#include <Kin/flag.h>

//===========================================================================

struct SkeletonEntry {
  StringA symbols;
  double phase0=-1.;
  double phase1=-1.;
  SkeletonEntry() {}
  SkeletonEntry(StringA symbols,double phase0,double phase1):symbols(symbols), phase0(phase0), phase1(phase1) {}
  void write(ostream& os) const { symbols.write(os," ",NULL,"()"); os <<" from " <<phase0 <<" to " <<phase1; }
};
stdOutPipe(SkeletonEntry)
typedef rai::Array<SkeletonEntry> Skeleton;

//===========================================================================

struct KOMO {

  //-- the problem definition
  double maxPhase;             ///< number of phases (roughly: actions), but phase is continuous valued
  uint stepsPerPhase;          ///< time slices per phase
  uint T;                      ///< total number of time steps
  double tau;                  ///< real time duration of single step (used when evaluating task space velocities/accelerations)
  uint k_order;                ///< the (Markov) order of the KOMO problem (default 2)
  rai::Array<Task*> tasks;     ///< list of tasks
  rai::Array<rai::Flag*> flags;     ///< list of flaggings that are applied to the frames/joints in the configurations and modify tasks
  rai::Array<rai::KinematicSwitch*> switches;  ///< list of kinematic switches along the motion
  
  //-- internals
  rai::KinematicWorld world;   ///< original world; which is the blueprint for all time-slice worlds (almost const: only makeConvexHulls modifies it)
  WorldL configurations;       ///< copies for each time slice; including kinematic switches; only these are optimized
  bool useSwift;               ///< whether swift (collisions/proxies) is evaluated whenever new configurations are set (needed if tasks read proxy list)
  
  //-- optimizer
  OptConstrained *opt;         ///< optimizer; created in run()
  arr x, dual;                 ///< the primal and dual solution
  arr z, splineB;              ///< when a spline representation is used: z are the nodes; splineB the B-spline matrix; x = splineB * z
  
  //-- verbosity only: buffers of all feature values computed on last set_x
  arr featureValues;           ///< storage of all features in all time slices
  ObjectiveTypeA featureTypes; ///< storage of all feature-types in all time slices
  bool featureDense;
  arr dualSolution;            ///< the dual solution computed during constrained optimization
  struct OpenGL *gl=NULL;      ///< internal only: used in 'displayTrajectory'
  int verbose;                 ///< verbosity level
  int animateOptimization=0;   ///< display the current path for each evaluation during optimization
  double runTime=0.;           ///< just measure run time
  ofstream *fil=NULL;
  
  KOMO();
  ~KOMO();
  
  //-- setup the problem
  void setModel(const rai::KinematicWorld& K,
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
  struct Task* setTask(double startTime, double endTime, TaskMap* map, ObjectiveType type=OT_sos, const arr& target=NoArr, double prec=1e2, uint order=0, int deltaStep=0);
  void setFlag(double time, rai::Flag* fl, int deltaStep=0);
  void setKinematicSwitch(double time, bool before, rai::KinematicSwitch* sw);
  void setKinematicSwitch(double time, bool before, const char *type, const char* ref1, const char* ref2, const rai::Transformation& jFrom=NoTransformation);
  
  //===========================================================================
  //
  // mid-level ways to define tasks: typically adding one specific task
  //
  
  //-- tasks mid-level
  void setHoming(double startTime=-1., double endTime=-1., double prec=1e-1, const char *keyword="robot");
  void setSquaredQAccelerations(double startTime=-1., double endTime=-1., double prec=1.);
  void setSquaredQVelocities(double startTime=-1., double endTime=-1., double prec=1.);
  void setFixEffectiveJoints(double startTime=-1., double endTime=-1., double prec=1e3);
  void setFixSwitchedObjects(double startTime=-1., double endTime=-1., double prec=1e3);
  void setSquaredQuaternionNorms(double startTime=-1., double endTime=-1., double prec=1e1);
  
  //-- tasks mid-level
  void setHoldStill(double startTime, double endTime, const char* shape, double prec=1e2);
  void setPosition(double startTime, double endTime, const char* shape, const char* shapeRel=NULL, ObjectiveType type=OT_sos, const arr& target=NoArr, double prec=1e2);
  void setOrientation(double startTime, double endTime, const char* shape, const char* shapeRel, ObjectiveType type=OT_sos, const arr& target=NoArr, double prec=1e2);
  void setVelocity(double startTime, double endTime, const char* shape, const char* shapeRel=NULL, ObjectiveType type=OT_sos, const arr& target=NoArr, double prec=1e2);
  void setAlign(double startTime, double endTime, const char* shape,  const arr& whichAxis=ARR(1.,0.,0.), const char* shapeRel=NULL, const arr& whichAxisRel=ARR(1.,0.,0.), ObjectiveType type=OT_sos, const arr& target=ARR(1.), double prec=1e2);
  void setAlignedStacking(double time, const char* object, ObjectiveType type=OT_sos, double prec=1e2);
  void setLastTaskToBeVelocity();
  void setCollisions(bool hardConstraint, double margin=.05, double prec=1.);
  void setLimits(bool hardConstraint, double margin=.05, double prec=1.);
  void setLiftDownUp(double time, const char *endeff, double timeToLift=.15);
  void setSlow(double startTime, double endTime, double prec=1e2, bool hardConstrained=false);
  void setSlowAround(double time, double delta, double prec=1e2, bool hardConstrained=false);
  
  //-- core task symbols of skeletons
  void core_setTouch(double startTime, double endTime, const char* shape1, const char* shape2, ObjectiveType type=OT_eq, const arr& target=NoArr, double prec=1e2);
  void core_setAbove(double startTime, double endTime, const char* shape1, const char* shape2, double prec=1e2);
  void core_setInside(double startTime, double endTime, const char* shape1, const char* shape2, double prec=1e2);
  void core_setImpulse(double time, const char* shape1, const char* shape2, ObjectiveType type=OT_eq, double prec=1e2);
  
  //-- core kinematic switch symbols of skeletons
  void core_setKSstable(double time, const char *from, const char *to);
  void core_setKSstableOn(double time, const char* from, const char* to);
  void core_setKSdynamic(double time, const char *from, const char *to);
  void core_setKSdynamicOn(double time, const char *from, const char* to);
  
  void setContact(double startTime, double endTime, const char *from, const char* to);
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
  void setPlaceFixed(double time, const char* endeffRef, const char* object, const char* placeRef, const rai::Transformation& relPose, int verbose=0);
  void setHandover(double time, const char* endeffRef, const char* object, const char* prevHolder, int verbose=0);
  void setPush(double startTime, double endTime, const char* stick, const char* object, const char* table, int verbose=0);
  void setGraspSlide(double time, const char* stick, const char* object, const char* placeRef, int verbose=0);
  void setSlideAlong(double time, const char *strick,  const char* object, const char* wall, int verbose=0);
  void setDrop(double time, const char* object, const char* from, const char* to, int verbose=0);
  void setDropEdgeFixed(double time, const char* object, const char* to, const rai::Transformation& relFrom, const rai::Transformation& relTo, int verbose=0);
  
  //-- tasks - logic level (used within LGP)
  void setAbstractTask(double phase, const Graph& facts, int verbose=0);
  void setSkeleton(const Skeleton& S);
  
  //DEPRECATED
  void setGraspStick(double time, const char* endeffRef, const char* object, int verbose=0, double weightFromTop=1e1, double timeToLift=.15);
  void setAttach(double time, const char* endeff, const char* object1, const char* object2, rai::Transformation& rel, int verbose=0);
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
  
  //-- optimization macros
  void setSpline(uint splineT);   ///< optimize B-spline nodes instead of the path; splineT specifies the time steps per node
  void reset(double initNoise=.01);      ///< reset the optimizer (initializes x to a default path)
  void run(bool dense=false);            ///< run the optimization (using OptConstrained -- its parameters are read from the cfg file)
  void getPhysicsReference(uint subSteps=10, int display=0);
  void playInPhysics(uint subSteps=10, bool display=false);
  rai::KinematicWorld& getConfiguration(double phase);
  arr getPath(const StringA& joints);
  arr getPath_frames(const uintA &frames);
  arr getPath_times();
  arr getPath_energies();
  void reportProblem(ostream &os=std::cout);
  Graph getReport(bool gnuplt=false, int reportFeatures=0, ostream& featuresOs=std::cout); ///< return a 'dictionary' summarizing the optimization results (optional: gnuplot task costs; output detailed cost features per time slice)
  void reportProxies(ostream& os=std::cout); ///< report the proxies (collisions) for each time slice
  void reportContacts(ostream& os=std::cout); ///< report the contacts
  rai::Array<rai::Transformation> reportEffectiveJoints(ostream& os=std::cout);
  void checkGradients(bool dense=false);          ///< checks all gradients numerically
  void plotTrajectory();
  void plotPhaseTrajectory();
  bool displayTrajectory(double delay=1., bool watch=true, bool overlayPaths=true, const char* saveVideoPrefix=NULL); ///< display the trajectory; use "vid/z." as vid prefix
  bool displayPath(bool watch=true); ///< display the trajectory; use "vid/z." as vid prefix
  rai::Camera& displayCamera();   ///< access to the display camera to change the view
  PhysXInterface& physx() { return world.physx(); }
  
  //===========================================================================
  //
  // internal (kind of private); old interface of 'KOMO'; kept for compatibility
  //
  
  //-- (not much in use..) specs gives as logic expressions in a Graph (or config file)
  KOMO(const rai::KinematicWorld& K) : KOMO() { setModel(K); } //for compatibility only
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
    StringA featureNames;
    
    Conv_MotionProblem_KOMO_Problem(KOMO& _komo) : komo(_komo) {}
    
    virtual uint get_k() { return komo.k_order; }
    virtual void getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes);
    virtual void phi(arr& phi, arrA& J, arrA& H, uintA& featureTimes, ObjectiveTypeA& tt, const arr& x, arr& lambda);
  } komo_problem;

  struct Conv_MotionProblem_DenseProblem : ConstrainedProblem {
    KOMO& komo;
    uint dimPhi;

    Conv_MotionProblem_DenseProblem(KOMO& _komo) : komo(_komo) {}

    void getStructure(uintA& variableDimensions, intAA& featureTimes, ObjectiveTypeA& featureTypes);
    virtual void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x, arr& lambda);

  } dense_problem;
};

//===========================================================================

inline arr finalPoseTo(rai::KinematicWorld& world,
                       rai::Frame& endeff,
                       rai::Frame& target,
                       byte whichAxesToAlign=0,
                       uint iterate=1) {
  KOMO komo(world);
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
