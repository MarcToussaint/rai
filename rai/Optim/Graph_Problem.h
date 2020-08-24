/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "optimization.h"
#include "MathematicalProgram.h"

struct GraphProblem {
  /// We have 'variableDimensions.N' variables, each with a different dimension 'variableDimensions(i)'.
  /// We have 'featureVariables.N' features, each depends on the tuple/clique 'featureVariables(j)' of variables.
  /// That is, 'featureVariables' is a list of tuples/cliques that defines the hyper graph
  virtual void getStructure(uintA& variableDimensions, intAA& featureVariables, ObjectiveTypeA& featureTypes) = 0;

  /// We require 'x.N == \sum_i variableDimensions(i)'; so x defines the value of all variables
  /// This returns the feature values, types and Jacobians at state x
  /// Only for features of type 'OT_f' also a Hessian is returned (hardly in use)
  /// Jacobians and Hessians are dense! But only w.r.t. the variables the feature depends on!!
  /// (It is the job of the optimizer to translate this to sparse global Jacobians/Hessians)
  virtual void phi(arr& phi, arrA& J, arrA& H, const arr& x) = 0;

  virtual void setPartialX(const uintA& whichX, const arr& x) {NIY}
  virtual void getPartialPhi(arr& phi, arrA& J, arrA& H, const uintA& whichPhi) {NIY}
  virtual void getSemantics(StringA& varNames, StringA& phiNames) {NIY}

  bool checkStructure(const arr& x);                 ///< check if Jacobians and Hessians have right dimensions (=clique size)
//  bool checkJacobian(const arr& x, double tolerance); ///< finite differences check of the returned Jacobian at x
//  bool checkHessian(const arr& x, double tolerance);  ///< finite differences check of the returned Hessian at x
};

/// Reduction to an unstructured constrained problem
struct Conv_Graph_MathematicalProgram : MathematicalProgram {
  GraphProblem& G;
  uintA variableDimensions, varDimIntegral;
  intAA featureVariables;
  ObjectiveTypeA featureTypes;
  arrA J_G, H_G;
  ostream* logFile=0;
  uint queryCount=0;

  Conv_Graph_MathematicalProgram(GraphProblem& _G, ostream* _log=0);

  uint getDimension();
  void getFeatureTypes(ObjectiveTypeA& ft);
  void evaluate(arr& phi, arr& J, const arr& x);

  void reportProblem(ostream& os);
};

struct ModGraphProblem : GraphProblem {
  GraphProblem& G;
  uintA subselectFeatures;

  ModGraphProblem(GraphProblem& G) : G(G) {}
  virtual void getStructure(uintA& variableDimensions, intAA& featureVariables, ObjectiveTypeA& featureTypes);
  virtual void getSemantics(StringA& varNames, StringA& phiNames);
  virtual void phi(arr& phi, arrA& J, arrA& H, const arr& x);
};
