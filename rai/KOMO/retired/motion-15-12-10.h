/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef _MT_motion_h
#define _MT_motion_h

#include "../Kin/kin.h"
#include "../Optim/optimization.h"

/* Notes
  -- transition models: kinematic, non-holonomic (vel = B u), pseudo dynamic, non-hol dynamic (acc = B u), real dynamic
  -- transition costs: vel^2 *tau, acc^2 * tau, u^2 * tau
  */

//===========================================================================
//
// defines only a map (task space), not yet the costs in this space
//

struct Feature {
  ObjectiveType type; // element of {cost_feature, inequality, equality} MAYBE: move this to Task?
  uint order;       ///< 0=position, 1=vel, etc
  virtual void phi(arr& y, arr& J, const rai::Configuration& G, int t=-1) = 0; ///< this needs to be overloaded
  virtual void phi(arr& y, arr& J, const ConfigurationL& G, double tau, int t=-1); ///< if not overloaded this computes the generic pos/vel/acc depending on order
  virtual uint dim_phi(const rai::Configuration& G) = 0; //the dimensionality of $y$

  VectorFunction vf(rai::Configuration& G) {
    return [this, &G](arr& y, arr& J, const arr& x) -> void {
      G.setJointState(x);
      phi(y, J, G, -1);
    };
  }

  Feature():type(OT_sos), order(0) {}
  virtual ~Feature() {};
};

//===========================================================================
//
/// A k-order cost_feature, inequality or equality constraint,
/// optionally rescaled using 'target' and 'prec'
//

struct Objective {
  Feature& map;
  rai::String name;
  bool active;
  arr target, prec;  ///< optional linear, potentially time-dependent, rescaling (with semantics of target & precision)

  uint dim_phi(const rai::Configuration& G, uint t) {
    if(!active || prec.N<=t || !prec(t)) return 0; return map.dim_phi(G);
  }

  Objective(Feature* m):map(*m), active(true) {} //TODO: require type here!!

  void setCostSpecs(uint fromTime, uint toTime,
                    const arr& _target=ARR(0.),
                    double _prec=1.);
};

Objective* newTask(const Node* specs, const rai::Configuration& world, uint Tinterval, uint Tzero=0);

//===========================================================================
//
// a motion problem description
//

/// This class allows you to DESCRIBE a motion planning problem, nothing more
struct KOMO {
  //engines to compute things
  rai::Configuration& world;  ///< the original world
  ConfigurationL configurations;       ///< copies for each time slice; including kinematic switches
  bool useSwift;

  //******* the following three sections are parameters that define the problem

  //-- task cost descriptions
  rai::Array<Objective*> tasks;

  //-- kinematic switches along the motion
  rai::Array<rai::KinematicSwitch*> switches;

  //-- trajectory length and tau
  uint T; ///< number of time steps
  double tau; ///< duration of single step
  uint k_order; ///< determine the order of the KOMO problem (default 2)

  //-- start constraints
  arr x0, v0; ///< fixed start state and velocity [[TODO: remove this and replace by prefix only (redundant...)]]
  arr prefix; ///< a set of states PRECEEDING x[0] (having 'negative' time indices) and which influence the control cost on x[0]. NOTE: x[0] is subject to optimization. DEFAULT: constantly equals x0
  arr postfix; ///< fixing the set of statex x[T-k]...x[T] //TODO: remove?
  //TODO: add methods to properly set the prefix given x0,v0?

  //-- stationary parameters
  arr z0; ///< an initialization of the stationary parameters of the motion problem

  //-- return values of an optimizer
  arrA phiMatrix;
  arr dualMatrix;
  rai::Array<ObjectiveTypeA> ttMatrix;

  KOMO(rai::Configuration& _world, bool useSwift=true);

  KOMO& operator=(const KOMO& other);

  //-- setting time aspects
  void setTiming(uint timeSteps, double duration);

  //-- setting costs in a task space
  bool parseTask(const Node* n, int Tinterval=-1, uint Tzero=0);
  void parseTasks(const Graph& specs, int Tinterval=-1, uint Tzero=0);
  Objective* addTask(const char* name, Feature* map);
  //TODO: the following are deprecated; use Task::setCostSpecs instead
//  enum TaskCostInterpolationType { constant, finalOnly, final_restConst, early_restConst, final_restLinInterpolated };
//  void setInterpolatingCosts(Task *c,
//                             TaskCostInterpolationType inType,
//                             const arr& y_finalTarget, double y_finalPrec, const arr& y_midTarget=NoArr, double y_midPrec=-1., double earlyFraction=-1.);

  //-- cost infos
  bool getPhi(arr& phi, arr& J, ObjectiveTypeA& tt, uint t, const ConfigurationL& G, double tau); ///< the general task vector and its Jacobian
  uint dim_phi(const rai::Configuration& G, uint t);
  uint dim_g(const rai::Configuration& G, uint t);
  uint dim_h(const rai::Configuration& G, uint t);
  StringA getPhiNames(const rai::Configuration& G, uint t);
  void reportFeatures(bool brief=false);
  void costReport(bool gnuplt=true); ///< also computes the costMatrix
  Graph getReport();

  void setState(const arr& x, const arr& v=NoArr);
  void activateAllTaskCosts(bool activate=true);

  //-- helpers
  arr getH_rate_diag();
  arr getInitialization();
  void setupConfigurations();
  void temporallyAlignKinematicSwitchesInConfiguration(uint t);
  void displayTrajectory(int steps, const char* tag, double delay=0.);

  //-- inverse Kinematics
  void inverseKinematics(arr& y, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x);

  ConstrainedProblem InvKinProblem() {
    return [this](arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x) -> void {
      this->inverseKinematics(phi, J, H, tt, x);
    };
  }

//  KOrderMarkovFunction PathProblem(){
//    NIY;
//  }
};

//===========================================================================
//
// transforming a motion problem description into an optimization problem
//

struct MotionProblemFunction:KOrderMarkovFunction {
  KOMO& MP;

  MotionProblemFunction(KOMO& _P):MP(_P) {}

  uint dim_g_h() { uint d=0; for(uint t=0; t<=MP.T; t++) d += dim_g(t) + dim_h(t); return d; }

  //KOrderMarkovFunction definitions
  virtual void phi_t(arr& phi, arr& J, ObjectiveTypeA& tt, uint t, const arr& x_bar);
  //functions to get the parameters $T$, $k$ and $n$ of the $k$-order Markov Process
  virtual uint get_T() { return MP.T; }
  virtual uint get_k() { return MP.k_order; }
  virtual uint dim_x() { return MP.x0.N; }
  virtual uint dim_z() { return MP.z0.N; }
  virtual uint dim_phi(uint t) { return MP.dim_phi(MP.world, t); } //transitions plus costs (latter include constraints)
  virtual uint dim_g(uint t) { return MP.dim_g(MP.world, t); }
  virtual uint dim_h(uint t) { return MP.dim_h(MP.world, t); }
  virtual StringA getPhiNames(uint t);
  virtual arr get_prefix(); //the history states x(-k),..,x(-1)
  virtual arr get_postfix();
};

//===========================================================================
//
// transforming a motion problem description into an end-pose optimization problem only
//

struct MotionProblem_EndPoseFunction {
  KOMO& MP;

  MotionProblem_EndPoseFunction(KOMO& _MP);

  //VectorFunction definitions
  void Phi(arr& phi, arr& J, ObjectiveTypeA& tt, const arr& x);
  virtual void fv(arr& phi, arr& J, const arr& x);
};

//===========================================================================
//
// basic helpers
//

void sineProfile(arr& q, const arr& q0, const arr& qT, uint T);
arr reversePath(const arr& q);
void getVel(arr& v, const arr& q, double tau);
void getAcc(arr& a, const arr& q, double tau);

#endif
