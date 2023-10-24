/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Kin/taskMaps.h"

struct CtrlObjective;
typedef rai::Array<CtrlObjective*> CtrlObjectiveL;

//===========================================================================
/**
 * A CtrlObjective defines a motion in operational space.
 */
struct CtrlObjective { //TODO: rename/refactor to become LinearAccelerationLaw (LAW) in task spaces
  Feature& map;
  rai::String name;
  bool active;
  arr prec; ///< compliance matrix $C$

  /// @{ @name Parameters that define the linear acceleration control law
  arr y_ref; ///< position reference
  arr v_ref; ///< velocity reference
  arr Kp; ///< proportional gain
  arr Kd; ///< derivative gain
  /// @}

  /// @{ @name Parameters that define velocity, acceleration and force limits
  double maxVel, maxAcc;
  arr f_ref;
  double f_alpha, f_gamma;

  /// Option for metric (difference) in task space: flip sign if scalar product is negative (for quaternion targets)
  bool flipTargetSignOnNegScalarProduct;
  bool makeTargetModulo2PI;

  /// @{ @name The actual state when LAST getDesiredAcceleration was called
  arr y, v;
  /// @}

  CtrlObjective(const char* name, Feature* map);
  CtrlObjective(const char* name, Feature* map, double decayTime, double dampingRatio, double maxVel, double maxAcc);
  CtrlObjective(const char* name, Feature* map, const Graph& params);

  void set(const Graph& params);
  void setTarget(const arr& yref, const arr& vref=NoArr);
  void setTargetToCurrent();
  void setGains(const arr& _Kp, const arr& _Kd);
  void setGains(double Kp, double Kd);
  void setGainsAsNatural(double decayTime, double dampingRatio); ///< the decayTime is the to decay to 10% of the initial offset/error
  void setC(const arr& C);

  arr get_y_ref();
  arr get_ydot_ref();
  arr getPrec();

  arr getDesiredAcceleration();
  void getDesiredLinAccLaw(arr& Kp_y, arr& Kd_y, arr& a0);
  void getForceControlCoeffs(arr& f_des, arr& u_bias, arr& KfL, arr& J_ft, const rai::Configuration& world);

  double error();
  bool isConverged(double tolerance=1e-2);
  void reportState(ostream& os);
};

//===========================================================================

struct ConstraintForceTask {
  Feature& map;
  rai::String name;
  bool active;

  double desiredForce;
  CtrlObjective desiredApproach;

  ConstraintForceTask(Feature* m):map(*m), active(true), desiredForce(0.), desiredApproach("desiredApproach", m) {}

  void updateConstraintControl(const arr& g, const double& lambda_desired);
};

//===========================================================================

/**
 * TaskControlMethods contains all individual motions/CtrlObjectives.
 */
struct TaskControlMethods {
  rai::Configuration& world;
  rai::Array<CtrlObjective*> tasks;
  rai::Array<ConstraintForceTask*> forceTasks;
  CtrlObjective qNullCostRef;
  boolA lockJoints;
  bool useSwift;

  TaskControlMethods(rai::Configuration& _world, bool _useSwift=true);

  /// @{ @name adding tasks
  CtrlObjective* addPDTask(const char* name, double decayTime, double dampingRatio, Feature* map);
  CtrlObjective* addPDTask(const char* name,
                           double decayTime, double dampingRatio,
                           TM_DefaultType type,
                           const char* iShapeName=nullptr, const rai::Vector& ivec=NoVector,
                           const char* jShapeName=nullptr, const rai::Vector& jvec=NoVector);
  ConstraintForceTask* addConstraintForceTask(const char* name, Feature* map);
  /// @}

  void lockJointGroup(const char* groupname, bool lockThem=true);

  void getTaskCoeffs(arr& yddot_des, arr& J); ///< the general (`big') task vector and its Jacobian
  arr getDesiredConstraintForces(); ///< J^T lambda^*
  arr operationalSpaceControl();
  arr calcOptimalControlProjected(arr& Kp, arr& Kd, arr& u0, const arr& M, const arr& F); ///< returns the linearized control law
  arr getDesiredLinAccLaw(arr& Kp, arr& Kd, arr& u0); ///< returns the linearized control law
  void calcForceControl(arr& K_ft, arr& J_ft_inv, arr& fRef, double& gamma); ///< returns the force controller coefficients
  void updateConstraintControllers();
  void reportCurrentState();

  void fwdSimulateControlLaw(arr& Kp, arr& Kd, arr& u0);

  void setState(const arr& q, const arr& qdot);
};
