/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Kin/taskMaps.h>
#include <Algo/spline.h>
#include <Core/thread.h>

/**
 * @file
 * With the feedback control we can define motions for operation space control.
 *
 * We simply define a set of motions via CtrlTasks/ConstraintForceTask and run
 * them.
 */

struct CtrlTask;
typedef rai::Array<CtrlTask*> CtrlTaskL;

//===========================================================================

/// a motion profile is a non-feedback(!) way to generate a task space reference path
/// [perhaps an adaptive phase, or Peter's adaptation to object motions, could be a modest way to incorporate feedback in the future]
struct MotionProfile {
  virtual ~MotionProfile() {}
  virtual ActStatus update(arr& yRef, arr& ydotRef, double tau,const arr& y, const arr& ydot) = 0;
  virtual void setTarget(const arr& ytarget, const arr& vtarget=NoArr) = 0;
  virtual void setTimeScale(double d) = 0;
  virtual void resetState() = 0;
  virtual bool isDone() = 0;
};

//===========================================================================

struct MotionProfile_Const : MotionProfile {
  arr y_target;
  bool flipTargetSignOnNegScalarProduct;
  MotionProfile_Const(const arr& y_target, bool flip=false) : y_target(y_target), flipTargetSignOnNegScalarProduct(flip) {}
  virtual ActStatus update(arr& yRef, arr& ydotRef, double tau,const arr& y, const arr& ydot);
  virtual void setTarget(const arr& ytarget, const arr& vtarget=NoArr){ y_target = ytarget; }
  virtual void setTimeScale(double d) {}
  virtual void resetState() {}
  virtual bool isDone() { return false; }
};

//===========================================================================

struct MotionProfile_ConstVel : MotionProfile {
  arr v_target;
  MotionProfile_ConstVel(const arr& _v_target) : v_target(_v_target) {}
  virtual ActStatus update(arr& yRef, arr& ydotRef, double tau, const arr& y, const arr& ydot);
  virtual void setTarget(const arr& ytarget, const arr& vtarget=NoArr){ CHECK(!!vtarget, ""); v_target = vtarget; }
  virtual void setTimeScale(double d) {}
  virtual void resetState() {}
  virtual bool isDone() { return false; }
};

//===========================================================================

struct MotionProfile_Sine : MotionProfile {
  arr y_start, y_target, y_err;
  double t, T;
  MotionProfile_Sine(const arr& y_target, double duration) : y_target(y_target), t(0.), T(duration) {}
  virtual ActStatus update(arr& yRef, arr& ydotRef, double tau,const arr& y, const arr& ydot);
  virtual void setTarget(const arr& ytarget, const arr& vtarget=NoArr);
  virtual void setTimeScale(double d) { T=d; }
  virtual void resetState() { y_start.clear(); y_err.clear(); t=0.; }
  virtual bool isDone();
};

//===========================================================================

struct MotionProfile_Bang : MotionProfile {
  arr y_target;           ///< position target of this motion generator
  double maxVel;          ///< parameters
  double tolerance;
  MotionProfile_Bang();
  MotionProfile_Bang(const arr& _y_target, double _maxVel);

  virtual void setTarget(const arr& ytarget, const arr& vtarget=NoArr);
  virtual void setTimeScale(double d) { HALT("doesn't make sense"); }
  virtual ActStatus update(arr& yRef, arr& ydotRef, double tau,const arr& y, const arr& ydot);
  virtual void resetState() {}
  virtual bool isDone() { return false; }
};

//===========================================================================

struct MotionProfile_PD : MotionProfile {
  arr y_ref, v_ref;
  arr y_target, v_target;
  double kp, kd;
  double maxVel, maxAcc;
  bool flipTargetSignOnNegScalarProduct;
  bool makeTargetModulo2PI;
  double tolerance;
  MotionProfile_PD();
  MotionProfile_PD(const arr& _y_target, double decayTime, double dampingRatio, double maxVel=-1., double maxAcc=-1.);
  MotionProfile_PD(const Graph& params);
  
  virtual void setTarget(const arr& ytarget, const arr& vtarget=NoArr);
  virtual void setTimeScale(double d) { setGainsAsNatural(d, .9); }
  virtual ActStatus update(arr& yRef, arr& ydotRef, double tau,const arr& y, const arr& ydot);
  virtual void resetState() { y_ref.clear(); v_ref.clear(); }

  void setGains(double _kp, double _kd);
  void setGainsAsNatural(double decayTime, double dampingRatio); ///< the decayTime is the to decay to 10% of the initial offset/error
  
  
  arr getDesiredAcceleration();
  void getDesiredLinAccLaw(arr& Kp_y, arr& Kd_y, arr& a0_y);
  
  double error();
  bool isConverged(double tolerance);
  virtual bool isDone() { return isConverged(tolerance); }
};

//===========================================================================

struct MotionProfile_Path: MotionProfile {
  rai::Spline spline;
  double executionTime;
  double phase;
  MotionProfile_Path(const arr& path, double executionTime);
  virtual ActStatus update(arr& yRef, arr& ydotRef, double tau,const arr& y, const arr& ydot);
  virtual void setTarget(const arr& ytarget, const arr& vtarget=NoArr){ HALT("can't directly set target of a path"); }
  virtual void setTimeScale(double d){ executionTime = d; }
  virtual void resetState() { NIY }
  virtual bool isDone() { return phase>=1.; }
};

//===========================================================================

/** In the given task space, a task can represent: 1) a pos/vel ctrl task
 *  and/or 2) a compliance and/or 3) a force limit control */
struct CtrlTask {
  //-- general information
  rai::String name;  ///< just for easier reporting
  bool active;       ///< also non-active tasks are updated (states evaluated), but don't enter the TaskControlMethods
  Var<ActStatus> status;

  Var<CtrlTaskL>* ctrlTasks=0; ///< if non-zero, auto-removes itself from this list on destruction

  //-- defines the feature map
  ptr<Feature> map;        ///< this defines the task space
  
  //-- feature values -- these are always kept up-to-date (in update)
  arr y, v, J_y;           ///< update() will evaluate these for a given kinematic configuration

  //-- if motion task: defines the reference in task space
  ptr<MotionProfile> ref;  ///< non-NULL iff this is a pos/vel task
  arr y_ref, v_ref;        ///< update() will define compute these references (reference=NOW, target=FUTURE)
  double scale;            ///< additional scaling (precision) for each task (redundant with map->scale! use latter for non-isotropic!)
  uint hierarchy;          ///< hierarchy level in hiearchycal inverse kinematics: higher = higher priority
  
  //-- if compliance task:
  arr compliance;          ///< non-empty iff this is a compliance task; values in [0,1] for each dimension of the Jacobian
  
  //-- if force task:
  arr f_ref;               ///< non-empty iff this is a force limit control task; defines the box limits (abs value in all dimensions)
  double f_alpha, f_gamma; ///< TODO
  
  CtrlTask(const char* name, const ptr<Feature>& _map);
  CtrlTask(const char* name, const ptr<Feature>& _map, const ptr<MotionProfile>& _ref);
  CtrlTask(const char* name, const ptr<Feature>& _map, double maxVel);
  CtrlTask(const char* name, const ptr<Feature>& _map, double decayTime, double dampingRatio, double maxVel=-1., double maxAcc=-1.);
  CtrlTask(const char* name, const ptr<Feature>& _map, const Graph& params);
  ~CtrlTask();
  
  ActStatus update(double tau, const rai::KinematicWorld& world);
  void resetState() { if(ref) ref->resetState(); status.set()=AS_init; }
  
  arr getPrec();
  void getForceControlCoeffs(arr& f_des, arr& u_bias, arr& K_I, arr& J_ft_inv, const rai::KinematicWorld& world);
  
  MotionProfile_PD& PD();
  void setRef(ptr<MotionProfile> _ref);
  void setTarget(const arr& y_target);
  void setTimeScale(double d){ CHECK(ref,""); ref->setTimeScale(d); ref->resetState(); }
  
  void reportState(ostream& os);
};

//===========================================================================

void getForceControlCoeffs(arr& f_des, arr& u_bias, arr& KfL, arr& J_ft, const rai::KinematicWorld& world);
void fwdSimulateControlLaw(arr &Kp, arr &Kd, arr &u0, rai::KinematicWorld& world);

//===========================================================================

/// implements a number of basic equations given a set of control tasks
struct TaskControlMethods {
  arr Hmetric;           ///< defines the metric in q-space (or qddot-space)
  boolA lockJoints;
  
  TaskControlMethods(const arr& _Hmetric);
  
  CtrlTask* addPDTask(CtrlTaskL& tasks, const char* name, double decayTime, double dampingRatio, ptr<Feature> map);

//  void updateCtrlTasks(double tau, const rai::KinematicWorld& world);
//  void resetCtrlTasksState();

  void lockJointGroup(const char *groupname, rai::KinematicWorld& world, bool lockThem=true);
  
  double getIKCosts(CtrlTaskL& tasks, const arr& q=NoArr, const arr& q0=NoArr, arr& g=NoArr, arr& H=NoArr);
  arr inverseKinematics(CtrlTaskL& tasks, arr& qdot, const arr& P_compliance, const arr& nullRef=NoArr, double* cost=NULL);
  arr inverseKinematics_hierarchical(CtrlTaskL& tasks);
  arr getComplianceProjection(CtrlTaskL& tasks);
  arr operationalSpaceControl(CtrlTaskL& tasks);
  arr calcOptimalControlProjected(CtrlTaskL& tasks, arr &Kp, arr &Kd, arr &u0, const arr& q, const arr& qdot, const arr& M, const arr& F); ///< returns the linearized control law
  arr getDesiredLinAccLaw(CtrlTaskL& tasks, arr &Kp, arr &Kd, arr &u0, const arr& q, const arr& qdot); ///< returns the linearized control law
  arr getDesiredConstraintForces(CtrlTaskL& tasks); ///< J^T lambda^*
  void calcForceControl(CtrlTaskL& tasks, arr& K_ft, arr& J_ft_inv, arr& fRef, double& gamma, const rai::KinematicWorld& world); ///< returns the force controller coefficients
  void reportCurrentState(CtrlTaskL& tasks);
};

//===========================================================================

void naturalGains(double& Kp, double& Kd, double decayTime, double dampingRatio);
