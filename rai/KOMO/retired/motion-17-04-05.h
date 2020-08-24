/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Kin/kin.h"
#include "../Optim/optimization.h"
#include "../Optim/KOMO_Problem.h"
#include "../Kin/feature.h"

//===========================================================================
//
/// A k-order sumOfSqr feature, inequality or equality constraint,
/// optionally rescaled using 'target' and 'prec'
//

struct Objective {
  Feature* map;
  const ObjectiveType type;  ///< element of {sumOfSqr, inequality, equality}
  rai::String name;
  arr target, prec;     ///< optional linear, time-dependent, rescaling (with semantics of target & precision)

  Objective(Feature* m, const ObjectiveType& type) : map(m), type(type) {}
  ~Objective() { if(map) delete map; map=nullptr; }

  void setCostSpecs(int fromTime, int toTime,
                    const arr& _target=ARR(0.),
                    double _prec=1.);
  void setCostSpecs(double fromTime, double toTime, int stepsPerPhase, uint T,
                    const arr& _target,
                    double _prec);
  bool isActive(uint t) { return (prec.N>t && prec(t)); }
  void write(std::ostream& os) const {
    os <<"TASK '" <<name <<"'"
       <<" type=" <<type
       <<" target=" <<target
       <<" prec=" <<prec;
  }

  static Objective* newTask(const Node* specs, const rai::Configuration& world, int stepsPerPhase, uint T); ///< create a new Task from specs
};
stdOutPipe(Task)

//===========================================================================
//
/// This class allows you to DESCRIBE a motion planning problem, nothing more
//

struct KOMO {
  rai::Configuration& world;  ///< the original world, which also defines the 'start conditions'
  ConfigurationL configurations;       ///< copies for each time slice; including kinematic switches; only these are optimized
  bool useSwift;

  rai::Array<Objective*> tasks; ///< task cost descriptions
  rai::Array<rai::KinematicSwitch*> switches;  ///< kinematic switches along the motion

  //-- trajectory length and tau
  uint T;       ///< number of time steps
  double tau;   ///< duration of single step
  uint k_order; ///< determine the order of the KOMO problem (default 2)

  //-- for reporting only: buffers of all feature values computed on last set_x
  arrA featureValues;                  ///< storage of all features in all time slices
  rai::Array<ObjectiveTypeA> featureTypes;  ///< storage of all feature-types in all time slices
  arr dualSolution;                    ///< the dual solution computed during constrained optimization

  struct OpenGL* gl; //internal only: used in 'displayTrajectory'

  KOMO(rai::Configuration& originalWorld, bool useSwift=true);
  ~KOMO();

  KOMO& operator=(const KOMO& other);

  /// setting the numer of time steps and total duration in seconds
  void setTiming(uint steps, double duration);

  /// core method to add tasks
  Objective* addTask(const char* name, Feature* map, const ObjectiveType& termType); ///< manually add a task

  //-- setting costs in a task space via specs
  void parseTasks(const Graph& specs, int stepsPerPhase=-1);     ///< read all tasks from a graph
  bool parseTask(const Node* n, int stepsPerPhase=-1);           ///< read a single task from a node-spec

  /// ``fix'' a certain time slice to configuration x (especitally final time slices). fix means that all joints become rigid and q zero-dimensional in that time slice
  void set_fixConfiguration(const arr& x, uint t);

  //-- initialization
  void setupConfigurations();   ///< this creates the @configurations@, that is, copies the original world T times (after setTiming!) perhaps modified by KINEMATIC SWITCHES
  arr getInitialization();      ///< this reads out the initial state trajectory after 'setupConfigurations'

  //-- methods accessed by the optimizers
  void set_x(const arr& x);            ///< set the state trajectory of all configurations
  uint dim_x(uint t) { return configurations(t+k_order)->getJointStateDimension(); }

  //-- info on the costs
  Graph getReport(bool gnuplt=true, int reportFeatures=0); ///< return summary of costs and constraint violations for all features
  void costReport(bool gnuplt=true) { cout <<getReport(gnuplt) <<endl; } //old convention
  void reportFeatures(bool brief=false, ostream& os=std::cout); ///< list all tasks, switches, feature values, for all times t
  void reportProxies(ostream& os=std::cout); ///< list all tasks, switches, feature values, for all times t

  //-- helpers
  bool displayTrajectory(int steps, const char* tag, double delay=0.);

  /// inverse kinematics problem (which is the special case T=0) returned as a @ConstrainedProblem@
  /// as input to optimizers
  struct Conv_MotionProblem_InvKinProblem : ConstrainedProblem {
    KOMO& MP;
    Conv_MotionProblem_InvKinProblem(KOMO& P) : MP(P) {}

    void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x) {
      MP.inverseKinematics(phi, J, H, tt, x);
    };
  } invKin_problem;
  void inverseKinematics(arr& y, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x);

  struct Conv_KOMO_KOMOProblem_toBeRetired : KOMO_Problem {
    KOMO& MP;
    uint dimPhi;

    Conv_KOMO_KOMOProblem_toBeRetired(KOMO& P) : MP(P) {}

    virtual uint get_k() { return MP.k_order; }
    virtual void getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes);
    virtual void phi(arr& phi, arrA& J, arrA& H, ObjectiveTypeA& tt, const arr& x);
  } komo_problem;
};

//===========================================================================
//
// basic helpers (TODO: move to a different place)
//

arr getH_rate_diag(const rai::Configuration& world);
void sineProfile(arr& q, const arr& q0, const arr& qT, uint T);
arr reversePath(const arr& q);
void getVel(arr& v, const arr& q, double tau);
void getAcc(arr& a, const arr& q, double tau);

