/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
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
#include <Kin/switch.h>
#include <Kin/featureSymbols.h>

//===========================================================================

enum SkeletonSymbol {
  SY_none=-1,

  //geometric:
  SY_touch,
  SY_above,
  SY_inside,
  SY_oppose,

  SY_impulse, //old
  SY_initial,
  SY_free, //old

  //pose constraints:
  SY_poseEq,
  SY_stableRelPose,
  SY_stablePose,

  //mode switches:
  SY_stable,
  SY_stableOn,
  SY_dynamic,
  SY_dynamicOn,
  SY_dynamicTrans,
  SY_quasiStatic,
  SY_quasiStaticOn,
  SY_liftDownUp, //old
  SY_break,

  //interactions:
  SY_contact,
  SY_contactStick,
  SY_contactComplementary,
  SY_bounce,

  //mode switches:
  SY_magic,
  SY_magicTrans,

  SY_push,  //old
  SY_graspSlide, //old

  SY_dampMotion,

  SY_noCollision, //old
  SY_identical,

  SY_alignByInt,

  SY_makeFree,

};

struct SkeletonEntry {
  double phase0=-1.;
  double phase1=-1.;
  rai::Enum<SkeletonSymbol> symbol;
  StringA frames; //strings referring to things
  SkeletonEntry() {}
  SkeletonEntry(double phase0, double phase1, SkeletonSymbol symbol, StringA frames) : phase0(phase0), phase1(phase1), symbol(symbol), frames(frames) {}
  void write(ostream& os) const { os <<symbol <<' '; frames.write(os, " ", nullptr, "()"); os <<" from " <<phase0 <<" to " <<phase1; }
};
stdOutPipe(SkeletonEntry)

typedef rai::Array<SkeletonEntry> Skeleton;

intA getSwitchesFromSkeleton(const Skeleton& S);
void writeSkeleton(std::ostream& os, const Skeleton& S, const intA& switches= {});

//===========================================================================

struct KOMO : NonCopyable {

  //-- the problem definition
  uint stepsPerPhase=0;        ///< time slices per phase
  uint T=0;                    ///< total number of time steps
  double tau=0.;               ///< real time duration of single step (used when evaluating task space velocities/accelerations)
  uint k_order=0;              ///< the (Markov) order of the KOMO problem (default 2)
  rai::Array<Objective*> objectives;     ///< list of tasks
  rai::Array<rai::KinematicSwitch*> switches;  ///< list of kinematic switches along the motion

  //-- internals
  rai::Configuration world;   ///< original world; which is the blueprint for all time-slice worlds (almost const: only makeConvexHulls modifies it)
  ConfigurationL configurations;       ///< copies for each time slice; including kinematic switches; only these are optimized
  bool useSwift;               ///< whether swift (collisions/proxies) is evaluated whenever new configurations are set (needed if tasks read proxy list)

  //-- experimental!
  rai::Configuration pathConfig;
  rai::Array<ptr<GroundedObjective>> objs;

  //-- optimizer
  bool denseOptimization=false;///< calls optimization with a dense (instead of banded) representation
  bool sparseOptimization=true;///< calls optimization with a sparse (instead of banded) representation
  OptConstrained *opt=0;       ///< optimizer; created in run()
  arr x, dual;                 ///< the primal and dual solution
  arr z, splineB;              ///< when a spline representation is used: z are the nodes; splineB the B-spline matrix; x = splineB * z
  //return values
  double sos, eq, ineq;

  //-- verbosity only: buffers of all feature values computed on last set_x
  arr featureValues;           ///< storage of all features in all time slices
  arrA featureJacobians;           ///< storage of all features in all time slices
  ObjectiveTypeA featureTypes; ///< storage of all feature-types in all time slices
  ptr<struct OpenGL> gl;              ///< internal only: used in 'displayTrajectory'
  int verbose;                 ///< verbosity level
  int animateOptimization=0;   ///< display the current path for each evaluation during optimization
  double runTime=0.;           ///< measured run time
  double timeCollisions=0., timeKinematics=0., timeNewton=0., timeFeatures=0.;
  ofstream* logFile=0;

  KOMO();
  KOMO(const rai::Configuration& C, bool _useSwift=true);
  ~KOMO();

  //-- setup the problem
  void setModel(const rai::Configuration& C, bool _useSwift=true);
  void setTiming(double _phases=1., uint _stepsPerPhase=10, double durationPerPhase=5., uint _k_order=2);
  void setPairedTimes();
  void activateCollisions(const char* s1, const char* s2);
  void deactivateCollisions(const char* s1, const char* s2);
  void setTimeOptimization();

  //-- higher-level default setups
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
  struct Objective* addObjective(const arr& times, const ptr<Feature>& f,
                                 ObjectiveType type, const arr& scale=NoArr, const arr& target=NoArr, int order=-1, int deltaFromStep=0, int deltaToStep=0);
  struct Objective* addObjective(const arr& times, const FeatureSymbol& feat, const StringA& frames,
                                 ObjectiveType type, const arr& scale=NoArr, const arr& target=NoArr, int order=-1, int deltaFromStep=0, int deltaToStep=0);

  void addSwitch(double time, bool before, rai::KinematicSwitch* sw);
  void addSwitch(double time, bool before, rai::JointType type, rai::SwitchInitializationType init,
                 const char* ref1, const char* ref2,
                 const rai::Transformation& jFrom=NoTransformation, const rai::Transformation& jTo=NoTransformation);
  void addContact_slide(double startTime, double endTime, const char* from, const char* to);
  void addContact_stick(double startTime, double endTime, const char* from, const char* to);
  void addContact_elasticBounce(double time, const char* from, const char* to, double elasticity=.8, double stickiness=0.);
  void addContact_noFriction(double startTime, double endTime, const char* from, const char* to);
  void addContact_ComplementarySlide(double startTime, double endTime, const char* from, const char* to);
  //  void addContact_Relaxed(double startTime, double endTime, const char *from, const char* to);
  void addContact_staticPush(double startTime, double endTime, const char* from, const char* to);

  //===========================================================================
  //
  // mid-level ways to define tasks: typically adding one specific task
  //

  //-- tasks mid-level
//  void setSquaredQAccelerations(double startTime=0., double endTime=-1., double prec=1.);
  void setSquaredQAccVelHoming(double startTime=0., double endTime=-1., double accPrec=1., double velPrec=0., double homingPrec=1e-2);
//  void setSquaredQVelocities(double startTime=0., double endTime=-1., double prec=1.);
//  void setFixEffectiveJoints(double startTime=0., double endTime=-1., double prec=3e1);
//  void setFixSwitchedObjects(double startTime=0., double endTime=-1., double prec=3e1);
  void setSquaredQuaternionNorms(double startTime=0., double endTime=-1., double prec=3e0);

  void setHoming(double startTime=0., double endTime=-1., double prec=1e-1, const char* keyword="robot");
  void setHoldStill(double startTime, double endTime, const char* shape, double prec=1e1);

  void add_collision(bool hardConstraint, double margin=.0, double prec=1e1);
  void add_jointLimits(bool hardConstraint, double margin=.05, double prec=1.);
  void setLiftDownUp(double time, const char* endeff, double timeToLift=.15);
  void setSlow(double startTime, double endTime, double prec=1e1, bool hardConstrained=false);
  void setSlowAround(double time, double delta, double prec=1e1, bool hardConstrained=false);

  //-- core task symbols of skeletons
  void add_touch(double startTime, double endTime, const char* shape1, const char* shape2, ObjectiveType type=OT_eq, const arr& target=NoArr, double prec=1e2);
  void add_aboveBox(double startTime, double endTime, const char* shape1, const char* shape2, double prec=1e1);
  void add_insideBox(double startTime, double endTime, const char* shape1, const char* shape2, double prec=1e1);
//  void add_impulse(double time, const char* shape1, const char* shape2, ObjectiveType type=OT_eq, double prec=1e1);
  void add_stable(double time,  const char* shape1, const char* shape2, ObjectiveType type=OT_eq, double prec=1e1);

  //-- core kinematic switch symbols of skeletons
  void addSwitch_mode(SkeletonSymbol prevMode, SkeletonSymbol newMode,
                      double time, double endTime,
                      const char* prevFrom, const char* newFrom, const char* obj);
  void addSwitch_stable(double time, double endTime, const char* from, const char* to);
  void addSwitch_stableOn(double time, double endTime, const char* from, const char* to);
  void addSwitch_dynamic(double time, double endTime, const char* from, const char* to, bool dampedVelocity=false);
  void addSwitch_dynamicOn(double time, double endTime, const char* from, const char* to);
  void addSwitch_dynamicOnNewton(double time, double endTime, const char* from, const char* to);
  void addSwitch_dynamicTrans(double time, double endTime, const char* from, const char* to);
  void addSwitch_magic(double time, double endTime, const char* from, const char* to, double sqrAccCost, double sqrVelCost);
  void addSwitch_magicTrans(double time, double endTime, const char* from, const char* to, double sqrAccCost);
  void addSwitch_on(double time, const char* from, const char* to, bool copyInitialization=false);


  //-- tasks - logic level (used within LGP)
  void setSkeleton(const Skeleton& S, bool ignoreSwitches=false);

  //dinos... can't get rid of them yet
  void setGraspSlide(double time, const char* stick, const char* object, const char* placeRef, int verbose=0);
  void setPush(double startTime, double endTime, const char* stick, const char* object, const char* table, int verbose=0);
  void setKS_slider(double time, double endTime, bool before, const char* obj, const char* slider, const char* table);

  //macros for pick-and-place in CGO -- should perhaps not be here.. KOMOext?
  void add_StableRelativePose(const std::vector<int>& confs, const char* gripper, const char* object) {
    for(uint i=1; i<confs.size(); i++)
      addObjective(ARR(confs[0], confs[i]), FS_poseRel, {gripper, object}, OT_eq);
    world.makeObjectsFree({object});
  }
  void add_StablePose(const std::vector<int>& confs, const char* object) {
    for(uint i=1; i<confs.size(); i++)
      addObjective(ARR(confs[0], confs[i]), FS_pose, {object}, OT_eq);
    world.makeObjectsFree({object});
  }
  void add_grasp(int conf, const char* gripper, const char* object) {
    addObjective(ARR(conf), FS_distance, {gripper, object}, OT_eq);
  }
  void add_place(int conf, const char* object, const char* table) {
    addObjective(ARR(conf), FS_aboveBox, {table, object}, OT_ineq);
    addObjective(ARR(conf), FS_standingAbove, {table, object}, OT_eq);
    addObjective(ARR(conf), FS_vectorZ, {object}, OT_sos, {}, {0., 0., 1.});
  }
  void add_resting(int conf1, int conf2, const char* object) {
    addObjective(ARR(conf1, conf2), FS_pose, {object}, OT_eq);
  }
  void add_restingRelative(int conf1, int conf2, const char* object, const char* tableOrGripper) {
    addObjective(ARR(conf1, conf2), FS_poseRel, {tableOrGripper, object}, OT_eq);
  }

  //===========================================================================
  //
  // optimizing, getting results, and verbosity
  //

  //-- optimization macros
  void setSpline(uint splineT);      ///< optimize B-spline nodes instead of the path; splineT specifies the time steps per node
  void reset(double initNoise=.01);  ///< reset the optimizer (initializes x to a default path)
  void setConfiguration(int t, const arr& q);
  void setInitialConfigurations(const arr& q);
  void initWithConstant(const arr& q);
  void initWithWaypoints(const arrA& waypoints, uint waypointStepsPerPhase=1, bool sineProfile=true);
  void run(const OptOptions options=NOOPT);                        ///< run the optimization (using OptConstrained -- its parameters are read from the cfg file)
  void run_sub(const uintA& X, const uintA& Y);
  void optimize(bool initNoise=true);

  rai::Configuration& getConfiguration(double phase);
  arr getJointState(double phase);
  arr getFrameState(double phase);
  arr getPath_decisionVariable();
  arr getPath(const uintA& joints);
  arr getPath(const StringA& joints= {});
  arr getPath_frames(const uintA& frames);
  arr getPath_frames(const StringA& frame= {});
  arrA getPath_q();
  arr getPath_tau();
  arr getPath_times();
  arr getPath_energies();

  arr getActiveConstraintJacobian();

  void reportProblem(ostream& os=std::cout);
  Graph getReport(bool gnuplt=false, int reportFeatures=0, ostream& featuresOs=std::cout); ///< return a 'dictionary' summarizing the optimization results (optional: gnuplot task costs; output detailed cost features per time slice)
  Graph getProblemGraph(bool includeValues, bool includeSolution=true);
  double getConstraintViolations();
  double getCosts();
  void reportProxies(ostream& os=std::cout, double belowMargin=.1); ///< report the proxies (collisions) for each time slice
  Graph getContacts(); ///< report the contacts
  rai::Array<rai::Transformation> reportEffectiveJoints(ostream& os=std::cout);

  void checkGradients(bool dense=false);          ///< checks all gradients numerically

  void plotTrajectory();
  void plotPhaseTrajectory();
  bool displayTrajectory(double delay=1., bool watch=true, bool overlayPaths=true, const char* saveVideoPath=nullptr, const char* addText=nullptr); ///< display the trajectory; use "vid/z." as vid prefix
  bool displayPath(bool watch=true, bool full=true); ///< display the trajectory; use "vid/z." as vid prefix
  rai::Camera& displayCamera();   ///< access to the display camera to change the view

  //===========================================================================
  //
  // internal (kind of private)
  //

  void selectJointsBySubtrees(const StringA& roots, const arr& times={});
  void clearObjectives();
  void setupConfigurations();   ///< this creates the @configurations@, that is, copies the original world T times (after setTiming!) perhaps modified by KINEMATIC SWITCHES and FLAGS
  void setupRepresentations();
  void set_x(const arr& x, const uintA& selectedConfigurationsOnly=NoUintA);            ///< set the state trajectory of all configurations
//  void setState(const arr& x, const uintA& selectedVariablesOnly=NoUintA);            ///< set the state trajectory of all configurations
  uint dim_x(uint t) { return configurations(t+k_order)->getJointStateDimension(); }

  struct Conv_KOMO_KOMOProblem : KOMO_Problem {
    KOMO& komo;
    uint dimPhi;
    uintA phiIndex, phiDim;
    StringA featureNames;
    
    Conv_KOMO_KOMOProblem(KOMO& _komo) : komo(_komo) {}
    void clear(){ dimPhi=0; phiIndex.clear(); phiDim.clear(); featureNames.clear(); }

    virtual uint get_k() { return komo.k_order; }
    virtual void getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes);
    virtual void phi(arr& phi, arrA& J, arrA& H, uintA& featureTimes, ObjectiveTypeA& tt, const arr& x);
  } komo_problem;

  struct Conv_KOMO_DenseProblem : ConstrainedProblem {
    KOMO& komo;
    uint dimPhi=0;

    arr quadraticPotentialLinear, quadraticPotentialHessian;

    Conv_KOMO_DenseProblem(KOMO& _komo) : komo(_komo) {}
    void clear(){ dimPhi=0; }

    void getDimPhi();

    virtual void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x);
  } dense_problem;

  struct Conv_KOMO_GraphProblem : GraphProblem {
    KOMO& komo;
    uint dimPhi=0;

    Conv_KOMO_GraphProblem(KOMO& _komo) : komo(_komo) {}
    void clear(){ dimPhi=0; }

    virtual void getStructure(uintA& variableDimensions, intAA& featureVariables, ObjectiveTypeA& featureTypes);
    virtual void phi(arr& phi, arrA& J, arrA& H, const arr& x);

    virtual void setPartialX(const uintA& whichX, const arr& x);
    virtual void getPartialPhi(arr& phi, arrA& J, arrA& H, const uintA& whichPhi);
    virtual void getSemantics(StringA& varNames, StringA& phiNames);
  } graph_problem;

  struct TimeSliceProblem : ConstrainedProblem {
    KOMO& komo;
    int slice;
    uint dimPhi=0;

    TimeSliceProblem(KOMO& _komo, int _slice) : komo(_komo), slice(_slice) {}

    void getDimPhi();
    virtual void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x);
  };

};

