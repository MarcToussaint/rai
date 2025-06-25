/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "objective.h"
#include "../Kin/kin.h"
#include "../Optim/options.h"

//===========================================================================

namespace rai {
struct FclInterface;
enum SwitchInitializationType : int;
enum SkeletonSymbol : int;
enum JointType : int;
enum ForceExchangeType : int;
}
typedef rai::Array<ObjectiveType> ObjectiveTypeA;
struct SolverReturn;
struct NLP;
struct NLP_Factored;

//===========================================================================

namespace rai {
struct KOMO_Options {
  RAI_PARAM("KOMO/", int, verbose, 1)
  RAI_PARAM("KOMO/", int, animateOptimization, 0)
  RAI_PARAM("KOMO/", bool, mimicStable, true)
  RAI_PARAM("KOMO/", double, sampleRate_stable, .0)
  RAI_PARAM("KOMO/", bool, sparse, true)
};
}//namespace

struct KOMO : rai::NonCopyable {

  //-- the problem definition
  uint stepsPerPhase=0;        ///< time slices per phase
  uint T=0;                    ///< total number of time steps
  double tau=0.;               ///< real time duration of single step (used when evaluating feature space velocities/accelerations)
  uint k_order=0;              ///< the (Markov) order of the KOMO problem
  ObjectiveL objectives;       ///< list of running objectives (each for a running interval of indexed time slices)
  rai::Array<shared_ptr<GroundedObjective>> objs;  ///< list of grounded objective (each for only a single tuple of frames (not running intervals))
  rai::Array<shared_ptr<rai::KinematicSwitch>> switches;  ///< list of kinematic switches along the motion -- only as record: they are applied immediately at addSwitch

  //-- internals
  rai::Configuration world;       ///< original configuration; which is the blueprint for all time-slice worlds (almost const: only makeConvexHulls modifies it)
  rai::Configuration pathConfig;  ///< configuration containing full path (T+k_order copies of world, with switches applied)
  uintA orgJointIndices;          ///< set of joint IDs (IDs of frames with dofs) of the original world
  FrameL timeSlices;              ///< the original timeSlices of the pathConfig (when switches add frames, pathConfig.frames might differ from timeSlices - otherwise not)
  bool computeCollisions=true;    ///< whether swift or fcl (collisions/proxies) is evaluated whenever new configurations are set (needed if features read proxy list)
  shared_ptr<rai::FclInterface> fcl;

  //-- optimizer
  arr x, dual;                    ///< the primal and dual solution

  //-- options
  rai::KOMO_Options opt;

  //-- verbosity only: buffers of all feature values computed on last set_x
  arr featureValues;           ///< storage of all features in all time slices
  arrA featureJacobians;       ///< storage of all features in all time slices
  ObjectiveTypeA featureTypes; ///< storage of all feature-types in all time slices
  StringA featureNames;
  double timeTotal=0., timeCollisions=0., timeKinematics=0., timeNewton=0., timeFeatures=0.; ///< measured run times
  uint evalCount=0;

  KOMO();
  KOMO(const rai::Configuration& C, double _phases, uint _stepsPerPhase, uint _k_order, bool _enableCollisions=true);
  ~KOMO();

  //-- setup the problem
  void setConfig(const rai::Configuration& C, bool _computeCollisions=true);
  void setTiming(double _phases, uint _stepsPerPhase, double durationPerPhase=5., uint _k_order=2);

  void clone(const KOMO& komo, bool deepCopyFeatures=true);

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
  shared_ptr<struct Objective> addObjective(const arr& times, const shared_ptr<Feature>& f, const StringA& frames,
      ObjectiveType type, const arr& scale=NoArr, const arr& target=NoArr, int order=-1, int deltaFromStep=0, int deltaToStep=0);
  shared_ptr<struct Objective> addObjective(const arr& times, const FeatureSymbol& feat, const StringA& frames,
      ObjectiveType type, const arr& scale=NoArr, const arr& target=NoArr, int order=-1, int deltaFromStep=0, int deltaToStep=0) {
    return addObjective(times, symbols2feature(feat, frames, world),
                        {}, type, scale, target, order, deltaFromStep, deltaToStep);
  }
  void clearObjectives(); ///< clear all objective
  void removeObjective(const Objective* ob);
  void copyObjectives(KOMO& komoB, bool deepCopyFeatures=true);

  void addContact_slide(double startTime, double endTime, const char* from, const char* to);
  void addContact_stick(double startTime, double endTime, const char* from, const char* to, double frictionCone_mu=.8);
  rai::Frame* addContact_WithPoaFrame(double time, str obj, str from, double frictionCone_mu, double init_objMass, double init_POAdist=.1);
  void addContact_elasticBounce(double time, const char* from, const char* to, double elasticity=.8, double stickiness=0.);
  void addContact_ComplementarySlide(double startTime, double endTime, const char* from, const char* to);
  //  void addContact_Relaxed(double startTime, double endTime, const char *from, const char* to);

  arr getBounds() { return pathConfig.getJointLimits(); }       ///< define the bounds (passed to the constrained optimization) based on the limit definitions of all DOFs

  //===========================================================================
  //
  // mid-level ways to define objectives: typically adding one specific objective
  //

  shared_ptr<struct Objective> addControlObjective(const arr& times, uint order, double scale=1., const arr& target=NoArr, int deltaFromStep=0, int deltaToStep=0);
  void addQuaternionNorms(const arr& times=NoArr, double scale=3e0, bool hard=true);

  void add_collision(bool hardConstraint=true, double margin=.0, double prec=1e1);
  void add_jointLimits(bool hardConstraint=true, double margin=.0, double prec=1.);
  void setLiftDownUp(double time, const char* endeff, double timeToLift=.15);
  void setSlow(double startTime, double endTime, double prec=1e1, bool hardConstrained=false);
  void setSlowAround(double time, double delta, double prec=1e1, bool hardConstrained=false);

  //-- core kinematic switch symbols of skeletons
//protected:
  //low-level add dof switches
  rai::Frame* addSwitch(double time, bool before, const shared_ptr<rai::KinematicSwitch>& sw);
  rai::Frame* addSwitch(double time, bool before, bool stable, rai::JointType type, rai::SwitchInitializationType init,
                        const char* ref1, const char* ref2,
                        const rai::Transformation& jFrom=NoTransformation);
 public:
  //add a mode switch: both, the low-level dof switches and corresponding constraints of consistency
  void addModeSwitch(const arr& times, rai::SkeletonSymbol newMode, const StringA& frames, bool firstSwitch);
  void addRigidSwitch(double time, const StringA& frames, bool noJumpStart=true);

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
  void initOrg();
  void initRandom(int verbose=0);
  void initWithConstant(const arr& q); ///< set all configurations EXCEPT the prefix to a particular state
  void initWithPath_qOrg(const arr& q);
  uintA initWithWaypoints(const arrA& waypoints, uint waypointStepsPerPhase=1, bool interpolate=false, double qHomeInterpolate=-1., int verbose=-1); ///< set all configurations (EXCEPT prefix) to interpolate given waypoints
  void initPhaseWithDofsPath(uint t_phase, const uintA& dofIDs, const arr& path_org, bool autoResamplePath=false);
  void addWaypointsInterpolationObjectives(const arrA& waypoints, uint waypointStepsPerPhase=1);
  void straightenCtrlFrames_mod2Pi();
  void updateRootObjects(const rai::Configuration& C);
  void updateAndShiftPrefix(const rai::Configuration& C);

  //-- calling a solver
  std::shared_ptr<SolverReturn> solve(double addInitializationNoise=.01, int splineKnots=-1, const rai::OptOptions& options=DEFAULT_OPTIONS);  ///< run the solver (same as run_prepare(); run(); )
  void reset();                          ///< reset the dual variables and feature value buffers (always needed when adding/changing objectives before continuing an optimization)

  //advanced
  void run_prepare(double addInitializationNoise);   ///< ensure the configurations are setup, decision variable is initialized, and noise added (if >0)

  //-- reading results
  arr getConfiguration_qAll(int t);  ///< get all DOFs
  arr getConfiguration_qOrg(int t);  ///< get only those DOFs that were defined in the original world (excluding extra DOFs from switches)
  arr getConfiguration_X(int t);     ///< get frame path for selected frames
  arr getConfiguration_dofs(uint t, const uintA& dofIndices);
  void getConfiguration_full(rai::Configuration& C, int t, int verbose);

  arrA getPath_qAll();                            ///< get the DOFs (of potentially varying dimensionality) for each configuration
  arr getPath_qOrg();      ///< get joint path, optionally for selected joints
  arr getPath(uintA dofIndices={});
  arr getPath_X();     ///< get frame path, optionally for selected frames

  arr getPath_tau();
  arr getPath_times();
  arr getPath_energies();

  rai::Graph report(bool specs=false, bool listObjectives=true, bool plotOverTime=false);

  arr info_objectiveErrorTraces();
  StringA info_objectiveNames();
  str info_sliceErrors(uint t, const arr& errorTraces);
  str info_sliceCollisions(uint t, double belowMargin=.01);
  arr info_errorTotals(const arr& errorTraces);

  StringA getCollisionPairs(double belowMargin=.01); ///< report the proxies (collisions) for each time slice

  void checkGradients();          ///< checks all gradients numerically

  int view(bool pause=false, const char* txt=nullptr);
  int view_play(bool pause=false, const char* txt=0, double delay=.2, const char* saveVideoPath=nullptr);
  int view_slice(uint t, bool pause=false);
  void view_close();
  std::shared_ptr<rai::ConfigurationViewer> get_viewer();
  void set_viewer(const std::shared_ptr<rai::ConfigurationViewer>& _viewer);


  void plotTrajectory();
  void plotPhaseTrajectory();

  void getSubProblem(uint phase, rai::Configuration& C, arr& q0, arr& q1);

  //===========================================================================
  //
  // internal (kind of private)
  //

  rai::Frame* addFrameDof(const char* name, const char* parent, rai::JointType jointType, bool stable, const char* originFrameName=0, rai::Frame* originFrame=0, const rai::Transformation& relOrigin=0);
  void addForceExchangeDofs(const arr& times, const char* onto, const char* from,  rai::ForceExchangeType _type, const arr& initPoa={}, const arr& initForce={});
  void set_x(const arr& x, const uintA& selectedConfigurationsOnly= {});           ///< set the state trajectory of all configurations
private:
  void selectJointsBySubtrees(const StringA& roots, const arr& times= {}, bool notThose=false);
  void setupPathConfig();
//  void addStableFrame(rai::SkeletonSymbol newMode, const char* parent, const char* name, const char* toShape);
  rai::Frame* applySwitch(const rai::KinematicSwitch& sw);
public:

  //===========================================================================
  //
  // NLP transcriptions
  //

  std::shared_ptr<NLP> nlp();
  std::shared_ptr<NLP> nlp_spline(uint numCtrlPoints=10, uint degree=3);
  std::shared_ptr<NLP> nlp_sub(const rai::Array<GroundedObjective*>& objs, const DofL& dofs);
  std::shared_ptr<NLP_Factored> nlp_FactoredTime();
  std::shared_ptr<NLP_Factored> nlp_FactoredParts();

//private:
  void _addObjective(const std::shared_ptr<Objective>& ob, const intA& timeSlices);
};

int conv_time2step(double time, uint stepsPerPhase);
double conv_step2time(int step, uint stepsPerPhase);
void conv_times2steps(int& fromStep, int& toStep, const arr& times, int stepsPerPhase, uint T,
                      int deltaFromStep, int deltaToStep);
intA conv_times2tuples(const arr& times, uint order, int stepsPerPhase, uint T,
                       int deltaFromStep, int deltaToStep);
