/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "NLP.h"
#include "options.h"

//==============================================================================
//
// LagrangianProblem
//
// we define an unconstraint optimization problem from a constrained one
// that can include lagrange terms, penalties, log barriers, and augmented lagrangian terms
//

struct LagrangianProblem : ScalarFunction, NLP {
  shared_ptr<NLP> P;

  //-- parameters of the unconstrained (Lagrangian) scalar function
  double muLB;       ///< log barrier mu
  double mu;         ///< penalty parameter for inequalities g and equalities h
  arr lambda;        ///< lagrange multipliers for inequalities g and equalities h
  bool useLB;        ///< interpret ALL ineq as LG instead of penalty

  //-- buffers to avoid re-evaluating points
  arr x;               ///< point where P was last evaluated
  arr phi_x, J_x, H_x; ///< features at x

  ostream* logFile=nullptr;  ///< file for logging

  LagrangianProblem(const shared_ptr<NLP>& P, const rai::OptOptions& opt=DEFAULT_OPTIONS, arr& lambdaInit=NoArr);

  virtual void evaluate(arr& phi, arr& J, const arr& x);       //evaluate all features and (optionally) their Jacobians for state x
  virtual void getFHessian(arr& H, const arr& x);              //the Hessian of the sum of all f-features (or Hessian in addition to the Gauss-Newton Hessian of all other features)
  virtual arr  getInitializationSample(const arr& previousOptima= {}) { return P->getInitializationSample(previousOptima); }

  double lagrangian(arr& dL, arr& HL, const arr& x); ///< CORE METHOD: the unconstrained scalar function F

  arr get_totalFeatures();

  double get_cost_f();            ///< info on the terms from last call
  double get_cost_sos();            ///< info on the terms from last call
  double get_costs();            ///< info on the terms from last call
  double get_sumOfGviolations(); ///< info on the terms from last call
  double get_sumOfHviolations(); ///< info on the terms from last call
  uint get_dimOfType(const ObjectiveType& ot); ///< info on the terms from last call

  rai::Graph reportGradients(const StringA& featureNames);
  void reportMatrix(std::ostream& os);

  void aulaUpdate(const rai::OptOptions& opt, bool anyTimeVariant, double lambdaStepsize=1., double* L_x=nullptr, arr& dL_x=NoArr, arr& HL_x=NoArr);
  void autoUpdate(const rai::OptOptions& opt, double* L_x=nullptr, arr& dL_x=NoArr, arr& HL_x=NoArr);

  //private: used gpenalty function
  double gpenalty(double g);
  double gpenalty_d(double g);
  double gpenalty_dd(double g);

  double hpenalty(double h);
  double hpenalty_d(double h);
  double hpenalty_dd(double h);
};

