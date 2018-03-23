/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Kin/taskMaps.h>
#include <Algo/spline.h>

/**
 * @file
 * With the feedback control we can define motions for operation space control.
 *
 * We simply define a set of motions via CtrlTasks/ConstraintForceTask and run
 * them.
 */


struct CtrlTask;
typedef rai::Array<CtrlTask*> CtrlTaskL;
enum CT_Status { CT_init=-1, CT_running, CT_conv, CT_done, CT_stalled };

//===========================================================================

/// a motion profile is a non-feedback(!) way to generate a task space reference path
/// [perhaps an adaptive phase, or Peter's adaptation to object motions, could be a modest way to incorporate feedback in the future]
struct MotionProfile{
  virtual ~MotionProfile(){}
  virtual CT_Status update(arr& yRef, arr& ydotRef, double tau,const arr& y, const arr& ydot) = 0;
  virtual void resetState() = 0;
  virtual bool isDone() = 0;
};

//===========================================================================

struct MotionProfile_Const : MotionProfile{
  arr y_target;
  bool flipTargetSignOnNegScalarProduct;
  MotionProfile_Const(const arr& y_target, bool flip=false) : y_target(y_target), flipTargetSignOnNegScalarProduct(flip) {}
  virtual CT_Status update(arr& yRef, arr& ydotRef, double tau,const arr& y, const arr& ydot);
  virtual void resetState(){}
  virtual bool isDone(){ return false; }
};

//===========================================================================

struct MotionProfile_Sine : MotionProfile{
  arr y_init, y_target, y_err;
  double t, T;
  MotionProfile_Sine(const arr& y_target, double duration) : y_target(y_target), t(0.), T(duration){}
  virtual CT_Status update(arr& yRef, arr& ydotRef, double tau,const arr& y, const arr& ydot);
  virtual void resetState(){ y_init.clear(); t=0.; }
  virtual bool isDone();
};

//===========================================================================

struct MotionProfile_PD: MotionProfile{
  arr y_ref, v_ref;
  arr y_target, v_target;
  double kp, kd;
  double maxVel, maxAcc;
  bool flipTargetSignOnNegScalarProduct;
  bool makeTargetModulo2PI;
  double tolerance;
  MotionProfile_PD();
  MotionProfile_PD(const arr& _y_target, double decayTime, double dampingRatio, double maxVel=0., double maxAcc=0.);
  MotionProfile_PD(const Graph& params);

  void setTarget(const arr& ytarget, const arr& vtarget=NoArr);
  void setGains(double _kp, double _kd);
  void setGainsAsNatural(double decayTime, double dampingRatio); ///< the decayTime is the to decay to 10% of the initial offset/error

  virtual CT_Status update(arr& yRef, arr& ydotRef, double tau,const arr& y, const arr& ydot);
  virtual void resetState(){ y_ref.clear(); v_ref.clear(); }

  arr getDesiredAcceleration();
  void getDesiredLinAccLaw(arr& Kp_y, arr& Kd_y, arr& a0_y);

  double error();
  bool isConverged(double tolerance);
  virtual bool isDone(){ return isConverged(tolerance); }
};

//===========================================================================

struct MotionProfile_Path: MotionProfile{
  rai::Spline spline;
  double executionTime;
  double phase;
  MotionProfile_Path(const arr& path, double executionTime);
  virtual CT_Status update(arr& yRef, arr& ydotRef, double tau,const arr& y, const arr& ydot);
  virtual void resetState(){ NIY }
  virtual bool isDone(){ return phase>=1.; }
};

//===========================================================================

/** In the given task space, a task can represent: 1) a pos/vel ctrl task
 *  and/or 2) a compliance and/or 3) a force limit control */
struct CtrlTask{
  TaskMap *map;      ///< this defines the task space
  rai::String name;  ///< just for easier reporting
  bool active;       ///< also non-active tasks are updates (states evaluated), but don't enter the TaskControlMethods
  CT_Status status;
  rai::Array<std::function<void(CtrlTask*,int)> > callbacks;

  //-- this is always kept up-to-date (in update)
  arr y, v, J_y;     ///< update() will evaluate these for a given kinematic configuration

  //-- pos/vel ctrl task
  MotionProfile *ref;  ///< non-NULL iff this is a pos/vel task
  arr y_ref, v_ref;    ///< update() will define compute these references (reference=NOW, target=FUTURE)
  arr prec;            ///< Cholesky(!) of C, not C itself: sumOfSqr(prec*(y-y_ref)) is the error, and prec*J the Jacobian
  uint hierarchy;      ///< hierarchy level in hiearchycal inverse kinematics: higher = higher priority

  //-- compliance task
  arr complianceDirection;             ///< non-empty iff this is a compliance task; defines the task space compliance coefficients

  //-- if this is a force ctrl task
  arr f_ref;           ///< non-empty iff this is a force limit control task; defines the box limits (abs value in all dimensions)
  double f_alpha, f_gamma; ///< TODO

  CtrlTask(const char* name, TaskMap* map);
  CtrlTask(const char* name, TaskMap* map, double decayTime, double dampingRatio, double maxVel, double maxAcc);
  CtrlTask(const char* name, TaskMap* map, const Graph& params);
  ~CtrlTask();

  CT_Status update(double tau, const rai::KinematicWorld& world);
  void resetState(){ if(ref) ref->resetState(); status=CT_init; }

  arr getPrec();
  void getForceControlCoeffs(arr& f_des, arr& u_bias, arr& K_I, arr& J_ft_inv, const rai::KinematicWorld& world);

  MotionProfile_PD& PD();
  void setRef(MotionProfile *_ref);
  void setTarget(const arr& y_target);

  void reportState(ostream& os);
};

//===========================================================================

void getForceControlCoeffs(arr& f_des, arr& u_bias, arr& KfL, arr& J_ft, const rai::KinematicWorld& world);
void fwdSimulateControlLaw(arr &Kp, arr &Kd, arr &u0, rai::KinematicWorld& world);

//===========================================================================

/// implements a number of basic equations given a set of control tasks
struct TaskControlMethods {
  rai::Array<CtrlTask*> tasks;
  arr Hmetric;           ///< defines the metric in q-space (or qddot-space)
  CtrlTask qNullCostRef; ///< defines the 'desired behavior' in qddot-space (regularization of operational space control)
  boolA lockJoints;

  TaskControlMethods(const rai::KinematicWorld& world);

  CtrlTask* addPDTask(const char* name, double decayTime, double dampingRatio, TaskMap *map);

  void updateCtrlTasks(double tau, const rai::KinematicWorld& world);
  void resetCtrlTasksState();

  void lockJointGroup(const char *groupname, rai::KinematicWorld& world, bool lockThem=true);

  double getIKCosts(const arr& q=NoArr, const arr& q0=NoArr, arr& g=NoArr, arr& H=NoArr);
  arr inverseKinematics(arr& qdot, const arr& nullRef=NoArr, double* cost=NULL);
  arr inverseKinematics_hierarchical();
  arr getComplianceProjection();
  arr operationalSpaceControl();
  arr calcOptimalControlProjected(arr &Kp, arr &Kd, arr &u0, const arr& q, const arr& qdot, const arr& M, const arr& F); ///< returns the linearized control law
  arr getDesiredLinAccLaw(arr &Kp, arr &Kd, arr &u0, const arr& q, const arr& qdot); ///< returns the linearized control law
  arr getDesiredConstraintForces(); ///< J^T lambda^*
  void calcForceControl(arr& K_ft, arr& J_ft_inv, arr& fRef, double& gamma, const rai::KinematicWorld& world); ///< returns the force controller coefficients
  void reportCurrentState();
};

//===========================================================================
