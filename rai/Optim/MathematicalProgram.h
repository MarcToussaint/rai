/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"

//===========================================================================

/// symbols to declare of which type an objective feature is
enum ObjectiveType { OT_none=0, OT_f, OT_sos, OT_ineq, OT_eq };
typedef rai::Array<ObjectiveType> ObjectiveTypeA;
extern ObjectiveTypeA& NoObjectiveTypeA;

arr summarizeErrors(const arr& phi, const ObjectiveTypeA& tt);

//===========================================================================

/** The MathematicalProgram abstraction provides a solver access to all it needs. To declare a MP problem, the user needs to
 *  overload the virtuals.
 *
 *  The core method to overload is 'evaluate', which returns a feature vector $phi$ and optionally its Jacobian $J$.
 *  For each entry of
 *  this feature vector $featureTypes(i)$ determins whether this is an inequality constraint, an equality constraint,
 *  a sumOfSqr or "direct-f" cost feature. The latter two define the objective function as
 *  $f(x) = \sum_{i \in F} \phi_i(x) + \sum_{i \in SOS} \phi_i(x)^2$, where the summations go over
 *  features of type 'f', and features of type 'sos', respectively. The direct-f feature type is special:
 *  if either they are linear, or another method 'getFHessian' needs to return the full Hessian of the sum of all f-features.
 *
 *  Importantly: the Jacobian may be sparse! This allows to implicitly represent structured NLP (in contrast to explicit structure, see below)
 */
struct MathematicalProgram : NonCopyable {
  virtual ~MathematicalProgram() {}

  //-- essential methods that need overload
  virtual void getFeatureTypes(ObjectiveTypeA& featureTypes) = 0;  //the number and type of all features (sos, ineq, eq, or f)
  virtual void evaluate(arr& phi, arr& J, const arr& x) = 0;       //evaluate all features and (optionally) their Jacobians for state x

  //-- optional signature and semantics
  virtual uint getDimension() { return 0; }                 //the dimensionality of the full decision variable
  virtual void getBounds(arr& bounds_lo, arr& bounds_up) { bounds_lo.clear(); bounds_up.clear(); } //lower/upper bounds for the decision variable (may be {})
  virtual void getNames(StringA& variableNames, StringA& featureNames) { variableNames.clear(); featureNames.clear(); } //the names of each variable/feature block (or element if unstructured)
  virtual arr  getInitializationSample(const arr& previousOptima= {}); //get an initialization (for MC sampling/restarts) [default: initialize random within bounds]

  //-- optional evaluation of Hessian of all scalar objectives
  virtual void getFHessian(arr& H, const arr& x) { H.clear(); } //the Hessian of the sum of all f-features (or Hessian in addition to the Gauss-Newton Hessian of all other features)

  //-- optional: return some info on the problem and the last evaluation, potentially with display
  virtual void report(ostream& os, int verbose){ os <<"NLP of type '" <<niceTypeidName(typeid(*this)) <<"' -- no reporting implemented"; }
};

//===========================================================================

struct MathematicalProgram_Factored : MathematicalProgram {
  //-- structure of the mathematical problem
  virtual void getFactorization(uintA& variableDimensions, //the size of each variable block
                                uintA& featureDimensions,  //the size of each feature block
                                intAA& featureVariables    //which variables the j-th feature block depends on
                               ) = 0;

  //-- structured (local) setting variable and evaluate feature
  virtual void setAllVariables(const arr& x){ NIY; } //set all variables at once
  virtual void setSingleVariable(uint var_id, const arr& x) = 0; //set a single variable block
  virtual void evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H) = 0; //get a single feature block

  //-- unstructured (batch) evaluation
  virtual void evaluate(arr& phi, arr& J, const arr& x); //default implementation: use setSingleVariable and evaluateSingleFeature

  virtual void report(){}
};

//===========================================================================
// TRIVIAL only header

struct MathematicalProgram_Traced : MathematicalProgram {
  MathematicalProgram& P;
  ObjectiveTypeA featureTypes;
  arr xTrace, costTrace, phiTrace, JTrace;
  bool trace_x=true;
  bool trace_costs=true;
  bool trace_phi=false;
  bool trace_J=false;

  MathematicalProgram_Traced(MathematicalProgram& P) : P(P) {}

  void setTracing(bool trace_x, bool trace_costs, bool trace_phi, bool trace_J){ NIY }

  virtual void evaluate(arr& phi, arr& J, const arr& x);

  //trivial
  virtual void getFeatureTypes(ObjectiveTypeA& _featureTypes) { P.getFeatureTypes(_featureTypes); featureTypes = _featureTypes; }
  virtual uint getDimension() { return P.getDimension(); }
  virtual void getBounds(arr& bounds_lo, arr& bounds_up) { P.getBounds(bounds_lo, bounds_up); }
  virtual void getNames(StringA& variableNames, StringA& featureNames) { P.getNames(variableNames, featureNames); }
  virtual arr  getInitializationSample(const arr& previousOptima= {}) { return P.getInitializationSample(previousOptima); }
  virtual void getFHessian(arr& H, const arr& x) { P.getFHessian(H, x); }
};

//===========================================================================
// TRIVIAL only header

struct Conv_MathematicalProgram_TrivialFactoreded : MathematicalProgram_Factored {
  MathematicalProgram& P;
  arr x_buffer;

  Conv_MathematicalProgram_TrivialFactoreded(MathematicalProgram& P) : P(P) {}

  virtual void getFeatureTypes(ObjectiveTypeA& featureTypes) { P.getFeatureTypes(featureTypes); }
  virtual uint getDimension() { return P.getDimension(); }
  virtual void getBounds(arr& bounds_lo, arr& bounds_up) { P.getBounds(bounds_lo, bounds_up); }
  virtual arr  getInitializationSample(const arr& previousOptima= {}) { return P.getInitializationSample(previousOptima); }

  virtual void getFactorization(uintA& variableDimensions, uintA& featureDimensions, intAA& featureVariables) {
    variableDimensions = { getDimension() };
    ObjectiveTypeA featureTypes;
    getFeatureTypes(featureTypes);
    featureDimensions = { featureTypes.N };
    featureVariables = { intA({0}) };
  }
  virtual void setSingleVariable(uint var_id, const arr& x) { x_buffer = x; }
  virtual void evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H) {  P.evaluate(phi, J, x_buffer);   if(!!H) NIY;  }
};

//===========================================================================

struct Conv_FactoredNLP_BandedNLP : MathematicalProgram {
  MathematicalProgram_Factored& P;
  uint maxBandSize;
  bool sparseNotBanded;
  uintA variableDimensions, varDimIntegral, featureDimensions, featDimIntegral;
  intAA featureVariables;
  //buffers
  arrA J_i;

  Conv_FactoredNLP_BandedNLP(MathematicalProgram_Factored& P, uint _maxBandSize, bool _sparseNotBanded=false);

  // trivial
  virtual uint getDimension() { return P.getDimension(); }
  virtual void getFeatureTypes(ObjectiveTypeA& featureTypes) { P.getFeatureTypes(featureTypes); }
  virtual void getBounds(arr& bounds_lo, arr& bounds_up) { P.getBounds(bounds_lo, bounds_up); }
  virtual void getNames(StringA& variableNames, StringA& featureNames) { P.getNames(variableNames, featureNames); }
  virtual arr  getInitializationSample(const arr& previousOptima= {}) { return P.getInitializationSample(previousOptima); }
  virtual void getFHessian(arr& H, const arr& x) { P.getFHessian(H, x); }

  virtual void evaluate(arr& phi, arr& J, const arr& x);
};

//===========================================================================

