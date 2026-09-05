/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "CtrlObjective.h"
#include "../Optim/NLP.h"

//===========================================================================

void getForceControlCoeffs(arr& f_des, arr& u_bias, arr& KfL, arr& J_ft, const rai::Configuration& world);
void fwdSimulateControlLaw(arr& Kp, arr& Kd, arr& u0, rai::Configuration& world);

//===========================================================================

/// implements a number of basic equations given a set of control tasks
struct TaskControlMethods {
  arr Hmetric;           ///< defines the metric in q-space (or qddot-space)
  boolA lockJoints;

  TaskControlMethods(const arr& _Hmetric);

  CtrlObjective* addPDTask(CtrlObjectiveL& tasks, const char* name, double decayTime, double dampingRatio, shared_ptr<Feature> map);

//  void updateCtrlObjectives(double tau, const rai::Configuration& world);
//  void resetCtrlObjectivesState();

  void lockJointGroup(const char* groupname, rai::Configuration& world, bool lockThem=true);

  double getIKCosts(CtrlObjectiveL& tasks, const arr& q=NoArr, const arr& q0=NoArr, arr& g=NoArr, arr& H=NoArr);
  arr inverseKinematics(const rai::Configuration& pathConfig, CtrlObjectiveL& tasks, arr& qdot, const arr& P_compliance, const arr& nullRef=NoArr, double* cost=nullptr);
  arr inverseKinematics_hierarchical(CtrlObjectiveL& tasks);
  arr getComplianceProjection(CtrlObjectiveL& tasks);
  arr operationalSpaceControl(CtrlObjectiveL& tasks);
  arr calcOptimalControlProjected(CtrlObjectiveL& tasks, arr& Kp, arr& Kd, arr& u0, const arr& q, const arr& qdot, const arr& M, const arr& F); ///< returns the linearized control law
  arr getDesiredLinAccLaw(CtrlObjectiveL& tasks, arr& Kp, arr& Kd, arr& u0, const arr& q, const arr& qdot); ///< returns the linearized control law
  arr getDesiredConstraintForces(CtrlObjectiveL& tasks); ///< J^T lambda^*
  void calcForceControl(CtrlObjectiveL& tasks, arr& K_ft, arr& J_ft_inv, arr& fRef, double& gamma, const rai::Configuration& world); ///< returns the force controller coefficients
  void reportCurrentState(CtrlObjectiveL& tasks);
};

//===========================================================================

struct CtrlProblem_NLP : NLP {
  CtrlSolver& CP;
  ConfigurationL Ctuple;
  uint dimPhi=0;
  arr store_phi;
  arr store_J;

  CtrlProblem_NLP(CtrlSolver& _CP);

  virtual uint getDimension();
  virtual void getBounds(arr& bounds_lo, arr& bounds_up);
  virtual void getFeatureTypes(ObjectiveTypeA& featureTypes);

  virtual void getNames(StringA& variableNames, StringA& featureNames);

  virtual arr getInitializationSample();

  virtual void evaluate(arr& phi, arr& J, const arr& x);
};

//===========================================================================

arr solve_optim(CtrlSolver& CP);
