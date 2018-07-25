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

struct KOMO : NonCopyable {

  //-- the problem definition
  double maxPhase;             ///< number of phases (roughly: actions), but phase is continuous valued
  uint stepsPerPhase;          ///< time slices per phase
  uint T;                      ///< total number of time steps
  double tau;                  ///< real time duration of single step (used when evaluating task space velocities/accelerations)
  uint k_order;                ///< the (Markov) order of the KOMO problem (default 2)
  rai::Array<Objective*> objectives;     ///< list of tasks
  rai::Array<rai::Flag*> flags;     ///< list of flaggings that are applied to the frames/joints in the configurations and modify tasks
  rai::Array<rai::KinematicSwitch*> switches;  ///< list of kinematic switches along the motion
  
  //-- internals
  rai::KinematicWorld world;   ///< original world; which is the blueprint for all time-slice worlds (almost const: only makeConvexHulls modifies it)
  WorldL configurations;       ///< copies for each time slice; including kinematic switches; only these are optimized
  bool useSwift;               ///< whether swift (collisions/proxies) is evaluated whenever new configurations are set (needed if tasks read proxy list)
  
  //-- optimizer
  OptConstrained *opt=0;       ///< optimizer; created in run()
  arr x, dual;                 ///< the primal and dual solution
  arr z, splineB;              ///< when a spline representation is used: z are the nodes; splineB the B-spline matrix; x = splineB * z
  
  //-- verbosity only: buffers of all feature values computed on last set_x
  arr featureValues;           ///< storage of all features in all time slices
  ObjectiveTypeA featureTypes; ///< storage of all feature-types in all time slices
  bool featureDense;
//  arr dualSolution;            ///< the dual solution computed during constrained optimization
  struct OpenGL *gl=NULL;      ///< internal only: used in 'displayTrajectory'
  int verbose;                 ///< verbosity level
  int animateOptimization=0;   ///< display the current path for each evaluation during optimization
  double runTime=0.;           ///< measured run time
  ofstream *fil=NULL;
  
  KOMO();
  ~KOMO();
  
  //-- setup the problem
  void setModel(const rai::KinematicWorld& K,
                bool _useSwift=true,  //disabling swift: no collisions, much faster
                bool meldFixedJoints=false, bool makeConvexHulls=false, bool computeOptimalSSBoxes=false, bool activateAllContacts=false);
  void setTiming(double _phases=1., uint _stepsPerPhase=10, double durationPerPhase=5., uint _k_order=2);
  void setPairedTimes();
  void activateCollisions(const char* s1, const char* s2);
  void deactivateCollisions(const char* s1, const char* s2);
  void setTimeOptimization();
  
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
  struct Objective* addObjective(double startTime, double endTime, Feature* map, ObjectiveType type=OT_sos, const arr& target=NoArr, double prec=1e1, int order=-1, int deltaStep=0);
  struct Objective* addObjective(double startTime, double endTime, ObjectiveType type, const FeatureSymbol& feat, const StringA& frames, double scale=1e1, const arr& target=NoArr, int order=-1);
  void addSwitch(double time, bool before, rai::KinematicSwitch* sw);
  void addSwitch(double time, bool before, const char *type, const char* ref1, const char* ref2, const rai::Transformation& jFrom=NoTransformation);
  void addFlag(double time, rai::Flag* fl, int deltaStep=0);
  void addContact(double startTime, double endTime, const char *from, const char* to);
  void addContact_Complementary(double startTime, double endTime, const char *from, const char* to);
  void addContact_Relaxed(double startTime, double endTime, const char *from, const char* to);

  //===========================================================================
  //
  // mid-level ways to define tasks: typically adding one specific task
  //

  //-- tasks mid-level
  void setSquaredQAccelerations(double startTime=0., double endTime=-1., double prec=1.);
  void setSquaredQVelocities(double startTime=0., double endTime=-1., double prec=1.);
  void setFixEffectiveJoints(double startTime=0., double endTime=-1., double prec=3e1);
  void setFixSwitchedObjects(double startTime=0., double endTime=-1., double prec=3e1);
  void setSquaredQuaternionNorms(double startTime=0., double endTime=-1., double prec=3e0);

  void setHoming(double startTime=0., double endTime=-1., double prec=1e-1, const char *keyword="robot");
  void setHoldStill(double startTime, double endTime, const char* shape, double prec=1e1);

  void add_collision(bool hardConstraint, double margin=.0, double prec=1.);
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
  void addSwitch_stable(double time, double endTime, const char *from, const char *to);
  void addSwitch_stableOn(double time, double endTime, const char* from, const char* to);
  void addSwitch_dynamic(double time, double endTime, const char *from, const char *to);
  void addSwitch_dynamicOn(double time, double endTime, const char *from, const char* to);
  
  //-- tasks - logic level (used within LGP)
  void setSkeleton(const Skeleton& S);
  
  //dinos... can't get rid of them yet
  void setGraspSlide(double time, const char* stick, const char* object, const char* placeRef, int verbose=0);
  void setPush(double startTime, double endTime, const char* stick, const char* object, const char* table, int verbose=0);
  void setKS_slider(double time, double endTime, bool before, const char* obj, const char* slider, const char* table);


  //===========================================================================
  //
  // optimizing, getting results, and verbosity
  //
  
  //-- optimization macros
  void setSpline(uint splineT);   ///< optimize B-spline nodes instead of the path; splineT specifies the time steps per node
  void reset(double initNoise=.01);      ///< reset the optimizer (initializes x to a default path)
  void run(bool dense=false);            ///< run the optimization (using OptConstrained -- its parameters are read from the cfg file)

  rai::KinematicWorld& getConfiguration(double phase);
  arr getPath_decisionVariable();
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
  
  //===========================================================================
  //
  // internal (kind of private); old interface of 'KOMO'; kept for compatibility
  //
  
  //-- (not much in use..) specs gives as logic expressions in a Graph (or config file)
  void clearTasks();
//  Task* addTask(const char* name, Feature *map, const ObjectiveType& termType); ///< manually add a task
  void setupConfigurations();   ///< this creates the @configurations@, that is, copies the original world T times (after setTiming!) perhaps modified by KINEMATIC SWITCHES and FLAGS
//  arr getInitialization();      ///< this reads out the initial state trajectory after 'setupConfigurations'
  void set_x(const arr& x);            ///< set the state trajectory of all configurations
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

    void getStructure(uintA& variableDimensions, intAA& featureTimes, ObjectiveTypeA& featureTypes);
    virtual void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x, arr& lambda);
  } dense_problem;
};

