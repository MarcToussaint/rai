/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "objective.h"
#include "../Kin/kin.h"
#include "../Optim/optimization.h"
#include "../Optim/constrained.h"
#include "../Optim/KOMO_Problem.h"
#include "../Optim/MathematicalProgram.h"
#include "../Kin/switch.h"
#include "../Kin/featureSymbols.h"

//===========================================================================

namespace rai {
  struct FclInterface;
}

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
  SY_downUp, //old
  SY_break,

  //interactions:
  SY_contact,
  SY_contactStick,
  SY_contactComplementary,
  SY_bounce,

  //mode switches:
  SY_magic,
  SY_magicTrans,

  //grasps/placements:
  SY_topBoxGrasp,
  SY_topBoxPlace,

  SY_push,  //old
  SY_graspSlide, //old

  SY_dampMotion,

  SY_noCollision, //old
  SY_identical,

  SY_alignByInt,

  SY_makeFree,
  SY_forceBalance,

  SY_touchBoxNormalX,
  SY_touchBoxNormalY,
  SY_touchBoxNormalZ,

  SY_end,

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

intA getSwitchesFromSkeleton(const Skeleton& S, const rai::Configuration& world);
double getMaxPhaseFromSkeleton(const Skeleton& S);
void writeSkeleton(std::ostream& os, const Skeleton& S, const intA& switches= {});
Skeleton readSkeleton(std::istream& is);
Skeleton readSkeleton2(std::istream& is);

//===========================================================================

namespace rai {
enum KOMOsolver { KS_none=-1, KS_dense=0, KS_sparse, KS_banded, KS_sparseFactored, KS_NLopt, KS_Ipopt, KS_Ceres };
}

//===========================================================================

struct KOMO : NonCopyable {

  //-- the problem definition
  uint stepsPerPhase=0;        ///< time slices per phase
  uint T=0;                    ///< total number of time steps
  double tau=0.;               ///< real time duration of single step (used when evaluating feature space velocities/accelerations)
  uint k_order=0;              ///< the (Markov) order of the KOMO problem (default 2)
  rai::Array<ptr<Objective>> objectives;    ///< list of objectives
  rai::Array<ptr<GroundedObjective>> objs;
  rai::Array<rai::KinematicSwitch*> switches;  ///< list of kinematic switches along the motion

  //-- internals
  rai::Configuration world;       ///< original configuration; which is the blueprint for all time-slice worlds (almost const: only makeConvexHulls modifies it)
  rai::Configuration pathConfig;  ///< configuration containing full path (T+k_order copies of world, with switches applied)
  uintA orgJointIndices;          ///< set of joint IDs (IDs of frames with dofs) of the original world
  FrameL timeSlices;              ///< the original timeSlices of the pathConfig (when switches add frames, pathConfig.frames might differ from timeSlices - otherwise not)
  bool computeCollisions;         ///< whether swift or fcl (collisions/proxies) is evaluated whenever new configurations are set (needed if features read proxy list)
  shared_ptr<rai::FclInterface> fcl;
  shared_ptr<SwiftInterface> swift;
  bool switchesWereApplied = false; //TODO: apply them directly? Would only work when no frames were added?

  //-- optimizer
  rai::KOMOsolver solver=rai::KS_sparse;
  arr x, dual;                 ///< the primal and dual solution

  //-- verbosity only: buffers of all feature values computed on last set_x
  double sos, eq, ineq;
  arr featureValues;           ///< storage of all features in all time slices
  arrA featureJacobians;       ///< storage of all features in all time slices
  ObjectiveTypeA featureTypes; ///< storage of all feature-types in all time slices
  StringA featureNames;
  int verbose;                 ///< verbosity level
  int animateOptimization=0;   ///< display the current path for each evaluation during optimization
  double timeTotal=0.;           ///< measured run time
  double timeCollisions=0., timeKinematics=0., timeNewton=0., timeFeatures=0.;
  ofstream* logFile=0;

  KOMO();
  ~KOMO();

  //-- setup the problem
  void setModel(const rai::Configuration& C, bool _computeCollisions=true);
  void setTiming(double _phases=1., uint _stepsPerPhase=30, double durationPerPhase=5., uint _k_order=2);

  //-- higher-level default setups
  void setIKOpt(); ///< setTiming(1., 1, 1., 1); and velocity objective

  //===========================================================================
  //
  // lowest level way to define objectives: basic methods to add any single objective or switch
  //

  /** THESE ARE THE TWO MOST IMPORTANT METHODS TO DEFINE A PROBLEM
   * they allow the user to add an objective, or a kinematic switch in the problem definition
   * Typically, the user does not call them directly, but uses the many methods below
   * Think of all of the below as examples for how to set arbirary objectives/switches yourself */
  ptr<struct Objective> addObjective(const arr& times, const ptr<Feature>& f, const StringA& frames,
                                     ObjectiveType type, const arr& scale=NoArr, const arr& target=NoArr, int order=-1, int deltaFromStep=0, int deltaToStep=0);
  ptr<struct Objective> addObjective(const arr& times, const FeatureSymbol& feat, const StringA& frames,
                                     ObjectiveType type, const arr& scale=NoArr, const arr& target=NoArr, int order=-1, int deltaFromStep=0, int deltaToStep=0);
  void clearObjectives(); ///< clear all objective

  void addContact_slide(double startTime, double endTime, const char* from, const char* to);
  void addContact_stick(double startTime, double endTime, const char* from, const char* to);
  void addContact_elasticBounce(double time, const char* from, const char* to, double elasticity=.8, double stickiness=0.);
  void addContact_noFriction(double startTime, double endTime, const char* from, const char* to);
  void addContact_ComplementarySlide(double startTime, double endTime, const char* from, const char* to);
  //  void addContact_Relaxed(double startTime, double endTime, const char *from, const char* to);
  void addContact_staticPush(double startTime, double endTime, const char* from, const char* to);

  void getBounds(arr& bounds_lo, arr& bounds_up);       ///< define the bounds (passed to the constrained optimization) based on the limit definitions of all DOFs

  //===========================================================================
  //
  // mid-level ways to define objectives: typically adding one specific objective
  //

  ptr<struct Objective> add_qControlObjective(const arr& times, uint order, double scale=1., const arr& target=NoArr, int deltaFromStep=0, int deltaToStep=0);
  void addSquaredQuaternionNorms(const arr& times=NoArr, double scale=3e0);

  void add_collision(bool hardConstraint, double margin=.0, double prec=1e1);
  void add_jointLimits(bool hardConstraint, double margin=.05, double prec=1.);
  void setLiftDownUp(double time, const char* endeff, double timeToLift=.15);
  void setSlow(double startTime, double endTime, double prec=1e1, bool hardConstrained=false);
  void setSlowAround(double time, double delta, double prec=1e1, bool hardConstrained=false);

  //-- core kinematic switch symbols of skeletons
protected:
  //low-level add dof switches
  void addSwitch(const arr& times, bool before, rai::KinematicSwitch* sw);
  rai::KinematicSwitch* addSwitch(const arr& times, bool before, rai::JointType type, rai::SwitchInitializationType init,
                 const char* ref1, const char* ref2,
                 const rai::Transformation& jFrom=NoTransformation, const rai::Transformation& jTo=NoTransformation);
public:
  //add a mode switch: both, the low-level dof switches and corresponding constraints of consistency
  void addModeSwitch(const arr& times, SkeletonSymbol newMode, const StringA& frames, bool firstSwitch);

  //1-liner specializations of setModeSwitch:
  void addSwitch_stable(double time, double endTime, const char* prevFrom, const char* from, const char* to, bool firstSwitch=true);
  void addSwitch_stableOn(double time, double endTime, const char* prevFrom, const char* from, const char* to, bool firstSwitch);
  void addSwitch_dynamic(double time, double endTime, const char* from, const char* to, bool dampedVelocity=false);
  void addSwitch_dynamicOn(double time, double endTime, const char* from, const char* to);
  void addSwitch_dynamicOnNewton(double time, double endTime, const char* from, const char* to);
  void addSwitch_dynamicTrans(double time, double endTime, const char* from, const char* to);
  void addSwitch_magic(double time, double endTime, const char* from, const char* to, double sqrAccCost, double sqrVelCost);
  void addSwitch_magicTrans(double time, double endTime, const char* from, const char* to, double sqrAccCost);

  //-- objectives - logic level (used within LGP)
  void setSkeleton(const Skeleton& S);
  void setSkeleton(const Skeleton& S, rai::ArgWord sequenceOrPath);

  //advanced:
  void setPairedTimes();
  void addTimeOptimization();

  //===========================================================================
  //
  // optimizing, getting results, and verbosity
  //

  //-- setting individual time slices
  void setConfiguration_qAll(int t, const arr& q); ///< t<0 allows to set the prefix configurations; while 0 <= t < T allows to set all other initial configurations
  void setConfiguration_qOrg(int t, const arr& q); ///< set only those DOFs that were defined in the original world (excluding extra DOFs from switches)
  void setConfiguration_X(int t, const arr& X); ///< t<0 allows to set the prefix configurations; while 0 <= t < T allows to set all other initial configurations
  void initWithConstant(const arr& q); ///< set all configurations EXCEPT the prefix to a particular state
  void initWithWaypoints(const arrA& waypoints, uint waypointStepsPerPhase=1, bool sineProfile=true); ///< set all configurations (EXCEPT prefix) to interpolate given waypoints
  void updateAndShiftPrefix(const rai::Configuration& C){
    //-- joint state
    //set t=0 to new joint state:
    setConfiguration_qAll(0, C.getJointState());
    //shift the joint state within prefix (t=-1 becomes equal to t=0, which is new state)
    for(int t=-k_order; t<0; t++) setConfiguration_qOrg(t, getConfiguration_qOrg(t+1));

    //-- frame state of roots only, if objects moved:
    uintA roots = framesToIndices(C.getRoots());
    arr X0 = C.getFrameState(roots);
    //set t=0..T to new frame state:
    for(uint t=0; t<T; t++) pathConfig.setFrameState(X0, roots+timeSlices(k_order+t,0)->ID);
    //shift the frame states within the prefix (t=-1 becomes equal to t=0, which is new state)
    for(int t=-k_order; t<0; t++){
      arr Xt = pathConfig.getFrameState(roots+timeSlices(k_order+t+1,0)->ID);
      pathConfig.setFrameState(Xt, roots+timeSlices(k_order+t,0)->ID);
    }
  }


  //-- optimization
  void optimize(double addInitializationNoise=.01, const OptOptions options=NOOPT);  ///< run the solver (same as run_prepare(); run(); )
  void reset();                                      ///< reset the dual variables and feature value buffers (always needed when adding/changing objectives before continuing an optimization)

  //advanced
  void run_prepare(double addInitializationNoise);   ///< ensure the configurations are setup, decision variable is initialized, and noise added (if >0)
  void run(OptOptions options=NOOPT);          ///< run the solver iterations (configurations and decision variable needs to be setup before)
  void setSpline(uint splineT);      ///< optimize B-spline nodes instead of the path; splineT specifies the time steps per node

  //-- reading results
  arr getConfiguration_qAll(int t);  ///< get all DOFs
  arr getConfiguration_qOrg(int t);  ///< get only those DOFs that were defined in the original world (excluding extra DOFs from switches)
  arr getConfiguration_X(int t);     ///< get frame path for selected frames

  arrA getPath_qAll();                            ///< get the DOFs (of potentially varying dimensionality) for each configuration
  arr getPath_qOrg();      ///< get joint path, optionally for selected joints
  arr getPath_X();     ///< get frame path, optionally for selected frames

  arr getPath_tau();
  arr getPath_times();
  arr getPath_energies();

  arr getActiveConstraintJacobian();

  void reportProblem(ostream& os=std::cout);
  rai::Graph getReport(bool gnuplt=false, int reportFeatures=0, ostream& featuresOs=std::cout); ///< return a 'dictionary' summarizing the optimization results (optional: gnuplot objective costs; output detailed cost features per time slice)
  rai::Graph getProblemGraph(bool includeValues, bool includeSolution=true);
  double getConstraintViolations();
  double getCosts();
  void reportProxies(ostream& os=std::cout, double belowMargin=.1); ///< report the proxies (collisions) for each time slice

  void checkGradients();          ///< checks all gradients numerically

  int view(bool pause=false, const char* txt=nullptr);
  int view_play(bool pause=false, double delay=.2, const char* saveVideoPath=nullptr);

  void plotTrajectory();
  void plotPhaseTrajectory();

  //===========================================================================
  //
  // internal (kind of private)
  //

  void selectJointsBySubtrees(const StringA& roots, const arr& times= {}, bool notThose=false);
  void setupConfigurations();
  void checkBounds(const arr& x);
  void retrospectApplySwitches();
  void retrospectChangeJointType(int startStep, int endStep, uint frameID, rai::JointType newJointType);
  void set_x(const arr& x, const uintA& selectedConfigurationsOnly=NoUintA);            ///< set the state trajectory of all configurations


  //===========================================================================
  //
  // MathematicalProgram transcriptions
  //

  shared_ptr<MathematicalProgram> nlp_SparseNonFactored();
  shared_ptr<MathematicalProgram_Factored> nlp_Factored();


  //===========================================================================
  //
  // deprecated
  //

  bool displayTrajectory(double delay=1., bool watch=true, bool overlayPaths=true, const char* saveVideoPath=nullptr, const char* addText=nullptr){
    DEPR; return view_play(watch, delay, saveVideoPath);  }
  bool displayPath(const char* txt, bool watch=true, bool full=true){
    DEPR; return view(watch, txt); }
  rai::Camera& displayCamera();

  void add_StableRelativePose(const std::vector<int>& confs, const char* gripper, const char* object) {
    DEPR;
    for(uint i=1; i<confs.size(); i++)
      addObjective(ARR(confs[0], confs[i]), FS_poseRel, {gripper, object}, OT_eq);
    world.makeObjectsFree({object});
  }
  void add_StablePose(const std::vector<int>& confs, const char* object) {
    DEPR;
    for(uint i=1; i<confs.size(); i++)
      addObjective(ARR(confs[0], confs[i]), FS_pose, {object}, OT_eq);
    world.makeObjectsFree({object});
  }
  void add_grasp(int conf, const char* gripper, const char* object) {
    DEPR;
    addObjective(ARR(conf), FS_distance, {gripper, object}, OT_eq);
  }
  void add_place(int conf, const char* object, const char* table) {
    DEPR;
    addObjective(ARR(conf), FS_aboveBox, {table, object}, OT_ineq);
    addObjective(ARR(conf), FS_standingAbove, {table, object}, OT_eq);
    addObjective(ARR(conf), FS_vectorZ, {object}, OT_sos, {}, {0., 0., 1.});
  }
  void add_resting(int conf1, int conf2, const char* object) {
    DEPR;
    addObjective(ARR(conf1, conf2), FS_pose, {object}, OT_eq);
  }
  void add_restingRelative(int conf1, int conf2, const char* object, const char* tableOrGripper) {
    DEPR;
    addObjective(ARR(conf1, conf2), FS_poseRel, {tableOrGripper, object}, OT_eq);
  }
  void activateCollisions(const char* s1, const char* s2){ DEPR; HALT("see komo-21-03-06"); }
  void deactivateCollisions(const char* s1, const char* s2);
  arr getFrameStateX(int t){ DEPR; return getConfiguration_X(t); }
  arr getPath_qAll(int t){ DEPR; return getConfiguration_qOrg(t); }
  arr getConfiguration_q(int t) { DEPR; return getConfiguration_qAll(t); }
  arr getPath_qOrg(uintA joints, const bool activesOnly){ DEPR; return getPath_qOrg(); }
};

