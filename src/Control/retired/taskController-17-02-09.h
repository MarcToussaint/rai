/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */
#pragma once

#include <Motion/taskMaps.h>

/**
 * @file
 * With the feedback control we can define motions for operation space control.
 *
 * We simply define a set of motions via CtrlTasks/ConstraintForceTask and run
 * them.
 */


struct CtrlTask;
typedef mlr::Array<CtrlTask*> CtrlTaskL;


//===========================================================================
/**
 * A CtrlTask defines a motion in operational space.
 */
struct CtrlTask{ //TODO: rename/refactor to become LinearAccelerationLaw (LAW) in task spaces
  TaskMap& map;
  mlr::String name;
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

  CtrlTask(const char* name, TaskMap* map);
  CtrlTask(const char* name, TaskMap* map, double decayTime, double dampingRatio, double maxVel, double maxAcc);
  CtrlTask(const char* name, TaskMap* map, const Graph& params);

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
  void getForceControlCoeffs(arr& f_des, arr& u_bias, arr& KfL, arr& J_ft, const mlr::KinematicWorld& world);

  double error();
  bool isConverged(double tolerance=1e-2);
  void reportState(ostream& os);
};

//===========================================================================

struct ConstraintForceTask{
  TaskMap& map;
  mlr::String name;
  bool active;

  double desiredForce;
  CtrlTask desiredApproach;

  ConstraintForceTask(TaskMap* m):map(*m), active(true), desiredForce(0.), desiredApproach("desiredApproach", m){}

  void updateConstraintControl(const arr& g, const double& lambda_desired);
};

//===========================================================================

/**
 * TaskControlMethods contains all individual motions/CtrlTasks.
 */
struct TaskControlMethods {
  mlr::KinematicWorld& world;
  mlr::Array<CtrlTask*> tasks;
  mlr::Array<ConstraintForceTask*> forceTasks;
  CtrlTask qNullCostRef;
  boolA lockJoints;
  bool useSwift;

  TaskControlMethods(mlr::KinematicWorld& _world, bool _useSwift=true);

  /// @{ @name adding tasks
  CtrlTask* addPDTask(const char* name, double decayTime, double dampingRatio, TaskMap *map);
  CtrlTask* addPDTask(const char* name,
                    double decayTime, double dampingRatio,
                    TaskMap_DefaultType type,
                    const char* iShapeName=NULL, const mlr::Vector& ivec=NoVector,
                    const char* jShapeName=NULL, const mlr::Vector& jvec=NoVector);
  ConstraintForceTask* addConstraintForceTask(const char* name, TaskMap *map);
  /// @}

  void lockJointGroup(const char *groupname, bool lockThem=true);

  void getTaskCoeffs(arr& yddot_des, arr& J); ///< the general (`big') task vector and its Jacobian
  arr getDesiredConstraintForces(); ///< J^T lambda^*
  arr operationalSpaceControl();
  arr calcOptimalControlProjected(arr &Kp, arr &Kd, arr &u0, const arr& M, const arr& F); ///< returns the linearized control law
  arr getDesiredLinAccLaw(arr &Kp, arr &Kd, arr &u0); ///< returns the linearized control law
  void calcForceControl(arr& K_ft, arr& J_ft_inv, arr& fRef, double& gamma); ///< returns the force controller coefficients
  void updateConstraintControllers();
  void reportCurrentState();

  void fwdSimulateControlLaw(arr &Kp, arr &Kd, arr &u0);

  void setState(const arr& q, const arr& qdot);
};

//===========================================================================
