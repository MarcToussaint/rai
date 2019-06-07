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
#include "objective.h"
#include <Kin/flag.h>
#include <Kin/featureSymbols.h>

//===========================================================================

enum SkeletonSymbol{
  SY_none=-1,
  SY_touch,
  SY_above,
  SY_inside,
  SY_impulse,
  SY_initial,
  SY_stable,
  SY_stableOn,
  SY_dynamic,
  SY_dynamicOn,
  SY_dynamicTrans,
  SY_liftDownUp,
  SY_break,

  SY_contact,
  SY_bounce,

  SY_magic,
  SY_magicTrans,

  SY_push,
  SY_graspSlide,

  SY_noCollision,
  SY_identical,

  SY_alignByInt,

  SY_makeFree,
  SY_stableRelPose,
  SY_stablePose,
};


struct SkeletonEntry {
  double phase0=-1.;
  double phase1=-1.;
  rai::Enum<SkeletonSymbol> symbol;
  StringA frames; //strings referring to things
  SkeletonEntry() {}
  SkeletonEntry(double phase0, double phase1, SkeletonSymbol symbol, StringA frames) : phase0(phase0), phase1(phase1), symbol(symbol), frames(frames){}
  void write(ostream& os) const { os <<symbol <<' '; frames.write(os," ",NULL,"()"); os <<" from " <<phase0 <<" to " <<phase1; }
};
stdOutPipe(SkeletonEntry)
typedef rai::Array<SkeletonEntry> Skeleton;
intA getSwitchesFromSkeleton(const Skeleton& S);
void writeSkeleton(std::ostream& os, const Skeleton& S, const intA& switches={});

//===========================================================================

struct KOMO : NonCopyable {

  //-- the problem definition
  uint stepsPerPhase=0;        ///< time slices per phase
  uint T=0;                    ///< total number of time steps
  double tau=0.;               ///< real time duration of single step (used when evaluating task space velocities/accelerations)
  uint k_order=0;              ///< the (Markov) order of the KOMO problem (default 2)
  rai::Array<Objective*> objectives;     ///< list of tasks
  rai::Array<rai::Flag*> flags;     ///< list of flaggings that are applied to the frames/joints in the configurations and modify tasks
  rai::Array<rai::KinematicSwitch*> switches;  ///< list of kinematic switches along the motion
  
  //-- internals
  rai::KinematicWorld world;   ///< original world; which is the blueprint for all time-slice worlds (almost const: only makeConvexHulls modifies it)
  WorldL configurations;       ///< copies for each time slice; including kinematic switches; only these are optimized
  bool useSwift;               ///< whether swift (collisions/proxies) is evaluated whenever new configurations are set (needed if tasks read proxy list)
  bool useSwitches;            ///< if true, switches change kinematic topology; if false, switches only impose relative pose constraints

  //-- optimizer
  bool denseOptimization=false;///< calls optimization with a dense (instead of banded) representation
  bool sparseOptimization=false;///< calls optimization with a sparse (instead of banded) representation
  OptConstrained *opt=0;       ///< optimizer; created in run()
  arr x, dual;                 ///< the primal and dual solution
  arr z, splineB;              ///< when a spline representation is used: z are the nodes; splineB the B-spline matrix; x = splineB * z
  //return values
  double sos, eq, ineq;

  //-- verbosity only: buffers of all feature values computed on last set_x
  arr featureValues;           ///< storage of all features in all time slices
  ObjectiveTypeA featureTypes; ///< storage of all feature-types in all time slices
  bool featureDense;
//  arr dualSolution;            ///< the dual solution computed during constrained optimization
  ptr<struct OpenGL> gl;              ///< internal only: used in 'displayTrajectory'
  int verbose;                 ///< verbosity level
  int animateOptimization=0;   ///< display the current path for each evaluation during optimization
  double runTime=0.;           ///< measured run time
  double timeCollisions=0., timeKinematics=0., timeNewton=0., timeFeatures=0.;
  ofstream *logFile=0;
  
  KOMO();
  KOMO(const rai::KinematicWorld& K, bool _useSwift=true);
  ~KOMO();
  
  //-- setup the problem
  void setModel(const rai::KinematicWorld& K, bool _useSwift=true);
  void setTiming(double _phases=1., uint _stepsPerPhase=10, double durationPerPhase=5., uint _k_order=2);
  void setPairedTimes();
  void activateCollisions(const char* s1, const char* s2);
  void deactivateCollisions(const char* s1, const char* s2);
  void setTimeOptimization();
  
  //-- higher-level setup defaults
  void setConfigFromFile();
  void setIKOpt();
  void setDiscreteOpt(uint k);
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
  struct Objective* addObjective(double startTime, double endTime, const ptr<Feature>& map, ObjectiveType type=OT_sos, const arr& target=NoArr, double scale=-1., int order=-1, int deltaFromStep=0, int deltaToStep=0);
  struct Objective* addObjective(double startTime, double endTime, Feature* map, ObjectiveType type=OT_sos, const arr& target=NoArr, double scale=-1., int order=-1, int deltaFromStep=0, int deltaToStep=0);
  struct Objective* addObjective(const arr& times, ObjectiveType type, const FeatureSymbol& feat, const StringA& frames={}, const arr& scale=NoArr, const arr& target=NoArr, int order=-1);

  void addSwitch(double time, bool before, rai::KinematicSwitch* sw);
  void addSwitch(double time, bool before, const char *type, const char* ref1, const char* ref2, const rai::Transformation& jFrom=NoTransformation);
  void addFlag(double time, rai::Flag* fl, int deltaStep=0);
  void addContact_slide(double startTime, double endTime, const char *from, const char* to);
  void addContact_stick(double startTime, double endTime, const char *from, const char* to);
  void addContact_elasticBounce(double time, const char *from, const char* to, double elasticity=.8, double stickiness=0.);
  void addContact_noFriction(double startTime, double endTime, const char *from, const char* to);
  void addContact_Complementary(double startTime, double endTime, const char *from, const char* to);
  //  void addContact_Relaxed(double startTime, double endTime, const char *from, const char* to);
  void addContact_staticPush(double startTime, double endTime, const char *from, const char* to);



  //===========================================================================
  //
  // mid-level ways to define tasks: typically adding one specific task
  //

  //-- tasks mid-level
  void setSquaredQAccelerations(double startTime=0., double endTime=-1., double prec=1.);
  void setSquaredQAccVelHoming(double startTime=0., double endTime=-1., double accPrec=1., double velPrec=0., double homingPrec=1e-2);
  void setSquaredQVelocities(double startTime=0., double endTime=-1., double prec=1.);
  void setFixEffectiveJoints(double startTime=0., double endTime=-1., double prec=3e1);
  void setFixSwitchedObjects(double startTime=0., double endTime=-1., double prec=3e1);
  void setSquaredQuaternionNorms(double startTime=0., double endTime=-1., double prec=3e0);

  void setHoming(double startTime=0., double endTime=-1., double prec=1e-1, const char *keyword="robot");
  void setHoldStill(double startTime, double endTime, const char* shape, double prec=1e1);

  void add_collision(bool hardConstraint, double margin=.0, double prec=1e1);
  void add_jointLimits(bool hardConstraint, double margin=.05, double prec=1.);
  void setLiftDownUp(double time, const char *endeff, double timeToLift=.15);
  void setSlow(double startTime, double endTime, double prec=1e1, bool hardConstrained=false);
  void setSlowAround(double time, double delta, double prec=1e1, bool hardConstrained=false);
  
  //-- core task symbols of skeletons
  void add_touch(double startTime, double endTime, const char* shape1, const char* shape2, ObjectiveType type=OT_eq, const arr& target=NoArr, double prec=1e2);
  void add_aboveBox(double startTime, double endTime, const char* shape1, const char* shape2, double prec=1e1);
  void add_insideBox(double startTime, double endTime, const char* shape1, const char* shape2, double prec=1e1);
  void add_impulse(double time, const char* shape1, const char* shape2, ObjectiveType type=OT_eq, double prec=1e1);
  void add_stable(double time,  const char* shape1, const char* shape2, ObjectiveType type=OT_eq, double prec=1e1);
  
  //-- core kinematic switch symbols of skeletons
  void addSwitch_mode(SkeletonSymbol prevMode, SkeletonSymbol newMode,
                      double time, double endTime,
                      const char *prevFrom, const char *newFrom, const char *obj);
  void addSwitch_stable(double time, double endTime, const char *from, const char *to);
  void addSwitch_stableOn(double time, double endTime, const char* from, const char* to);
  void addSwitch_dynamic(double time, double endTime, const char *from, const char *to);
  void addSwitch_dynamicOn(double time, double endTime, const char *from, const char* to);
  void addSwitch_dynamicOnNewton(double time, double endTime, const char *from, const char* to);
  void addSwitch_dynamicTrans(double time, double endTime, const char *from, const char *to);
  void addSwitch_magic(double time, double endTime, const char* from, const char* to, double sqrAccCost);
  void addSwitch_magicTrans(double time, double endTime, const char* from, const char* to, double sqrAccCost);
  void addSwitch_on(double time, const char *from, const char* to);


  
  //-- tasks - logic level (used within LGP)
  void setSkeleton(const Skeleton& S, bool ignoreSwitches=false);
  
  //dinos... can't get rid of them yet
  void setGraspSlide(double time, const char* stick, const char* object, const char* placeRef, int verbose=0);
  void setPush(double startTime, double endTime, const char* stick, const char* object, const char* table, int verbose=0);
  void setKS_slider(double time, double endTime, bool before, const char* obj, const char* slider, const char* table);

  //macros for pick-and-place in CGO -- should perhaps not be here.. KOMOext?
  void add_StableRelativePose(const std::vector<int>& confs, const char* gripper, const char* object){
    for(uint i=1;i<confs.size();i++)
      addObjective(ARR(confs[0], confs[i]), OT_eq, FS_poseRel, {gripper, object});
    world.makeObjectsFree({object});
  }
  void add_StablePose(const std::vector<int>& confs, const char* object){
    for(uint i=1;i<confs.size();i++)
      addObjective(ARR(confs[0], confs[i]), OT_eq, FS_pose, {object});
    world.makeObjectsFree({object});
  }
  void add_grasp(int conf, const char* gripper, const char* object){
    addObjective(ARR(conf), OT_eq, FS_distance, {gripper, object});
  }
  void add_place(int conf, const char* object, const char* table){
    addObjective(ARR(conf), OT_ineq, FS_aboveBox, {table, object});
    addObjective(ARR(conf), OT_eq, FS_standingAbove, {table, object});
    addObjective(ARR(conf), OT_sos, FS_vectorZ, {object}, {}, {0.,0.,1.});
  }
  void add_resting(int conf1, int conf2, const char* object){
    addObjective(ARR(conf1, conf2), OT_eq, FS_pose, {object});
  }
  void add_restingRelative(int conf1, int conf2, const char* object, const char* tableOrGripper){
    addObjective(ARR(conf1, conf2), OT_eq, FS_poseRel, {tableOrGripper, object});
  }


  //===========================================================================
  //
  // optimizing, getting results, and verbosity
  //
  
  //-- optimization macros
  void setSpline(uint splineT);      ///< optimize B-spline nodes instead of the path; splineT specifies the time steps per node
  void reset(double initNoise=.01);  ///< reset the optimizer (initializes x to a default path)
  void initWithConstant(const arr& q);
  void initWithWaypoints(const arrA& waypoints, uint waypointStepsPerPhase=1, bool sineProfile=true);
  void run();                        ///< run the optimization (using OptConstrained -- its parameters are read from the cfg file)
  void run_sub(const uintA& X, const uintA& Y);
  void optimize(bool initialize=true);

  rai::KinematicWorld& getConfiguration(double phase);
  arr getJointState(double phase);
  arr getFrameState(double phase);
  arr getPath_decisionVariable();
  arr getPath(const uintA& joints);
  arr getPath(const StringA& joints={});
  arr getPath_frames(const uintA &frames);
  arr getPath_frames(const StringA &frame={});
  arrA getPath_q();
  arr getPath_tau();
  arr getPath_times();
  arr getPath_energies();

  void reportProblem(ostream &os=std::cout);
  Graph getReport(bool gnuplt=false, int reportFeatures=0, ostream& featuresOs=std::cout); ///< return a 'dictionary' summarizing the optimization results (optional: gnuplot task costs; output detailed cost features per time slice)
  Graph getProblemGraph(bool includeValues);
  double getConstraintViolations();
  double getCosts();
  void reportProxies(ostream& os=std::cout, double belowMargin=.1); ///< report the proxies (collisions) for each time slice
  Graph getContacts(); ///< report the contacts
  rai::Array<rai::Transformation> reportEffectiveJoints(ostream& os=std::cout);

  void checkGradients(bool dense=false);          ///< checks all gradients numerically

  void plotTrajectory();
  void plotPhaseTrajectory();
  bool displayTrajectory(double delay=1., bool watch=true, bool overlayPaths=true, const char* saveVideoPrefix=NULL); ///< display the trajectory; use "vid/z." as vid prefix
  bool displayPath(bool watch=true, bool full=true); ///< display the trajectory; use "vid/z." as vid prefix
  rai::Camera& displayCamera();   ///< access to the display camera to change the view
  
  //===========================================================================
  //
  // internal (kind of private); old interface of 'KOMO'; kept for compatibility
  //
  
  //-- (not much in use..) specs gives as logic expressions in a Graph (or config file)
  void clearObjectives();
//  Task* addTask(const char* name, Feature *map, const ObjectiveType& termType); ///< manually add a task
  void setupConfigurations();   ///< this creates the @configurations@, that is, copies the original world T times (after setTiming!) perhaps modified by KINEMATIC SWITCHES and FLAGS
//  arr getInitialization();      ///< this reads out the initial state trajectory after 'setupConfigurations'
  void set_x(const arr& x, const uintA& selectedConfigurationsOnly=NoUintA);            ///< set the state trajectory of all configurations
  uint dim_x(uint t) { return configurations(t+k_order)->getJointStateDimension(); }

  struct Conv_MotionProblem_KOMO_Problem : KOMO_Problem {
    KOMO& komo;
    uint dimPhi;
    arr prevLambda;
    uintA phiIndex, phiDim;
    StringA featureNames;
    
    Conv_MotionProblem_KOMO_Problem(KOMO& _komo) : komo(_komo) {}
    void clear(){ dimPhi=0; prevLambda.clear(); phiIndex.clear(); phiDim.clear(); featureNames.clear(); }

    virtual uint get_k() { return komo.k_order; }
    virtual void getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes);
    virtual void phi(arr& phi, arrA& J, arrA& H, uintA& featureTimes, ObjectiveTypeA& tt, const arr& x, arr& lambda);
  } komo_problem;

  struct Conv_MotionProblem_DenseProblem : ConstrainedProblem {
    KOMO& komo;
    uint dimPhi=0;

    Conv_MotionProblem_DenseProblem(KOMO& _komo) : komo(_komo) {}
    void clear(){ dimPhi=0; }

    void getDimPhi();

    virtual void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x, arr& lambda);
  } dense_problem;

  struct Conv_MotionProblem_GraphProblem : GraphProblem {
    KOMO& komo;
    uint dimPhi=0;

    Conv_MotionProblem_GraphProblem(KOMO& _komo) : komo(_komo) {}
    void clear(){ dimPhi=0; }

    virtual void getStructure(uintA& variableDimensions, intAA& featureVariables, ObjectiveTypeA& featureTypes);
    virtual void phi(arr& phi, arrA& J, arrA& H, const arr& x);

    virtual void setPartialX(const uintA& whichX, const arr& x);
    virtual void getPartialPhi(arr& phi, arrA& J, arrA& H, const uintA& whichPhi);
    virtual void getSemantics(StringA& varNames, StringA& phiNames);
  } graph_problem;
};

