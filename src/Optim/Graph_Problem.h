/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#pragma once

#include <Core/array.h>
#include "optimization.h"

struct GraphProblem {
  /// We have 'variableDimensions.N' variables, each with a different dimension 'variableDimensions(i)'.
  /// We have 'featureVariables.N' features, each depends on the tuple/clique 'featureVariables(j)' of variables.
  /// That is, 'featureVariables' is a list of tuples/cliques that defines the hyper graph
  virtual void getStructure(uintA& variableDimensions, uintAA& featureVariables, ObjectiveTypeA& featureTypes) = 0;

  /// We require 'x.N == \sum_i variableDimensions(i)'; so x defines the value of all variables
  /// This returns the feature values, types and Jacobians at state x
  /// Only for features of type 'OT_f' also a Hessian is returned
  /// Jacobians and Hessians are dense! But only w.r.t. the variables the feature depends on!!
  /// (It is the job of the optimizer to translate this to sparse global Jacobians/Hessians)
  virtual void phi(arr& phi, arrA& J, arrA& H, const arr& x) = 0;

//  bool checkStructure(const arr& x);                 ///< check if Jacobians and Hessians have right dimensions (=clique size)
//  bool checkJacobian(const arr& x, double tolerance); ///< finite differences check of the returned Jacobian at x
//  bool checkHessian(const arr& x, double tolerance);  ///< finite differences check of the returned Hessian at x
};

/// Reduction to an unstructured constrained problem
struct Conv_Graph_ConstrainedProblem:ConstrainedProblem{
  GraphProblem& G;
  uintA variableDimensions, varDimIntegral;
  uintAA featureVariables;
  ObjectiveTypeA featureTypes;
  arrA J_G, H_G;

  Conv_Graph_ConstrainedProblem(GraphProblem& _G);
  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x);
};

