/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"

//===========================================================================

/// symbols to declare of which type an objective feature is
enum ObjectiveType : int { OT_f=0, OT_sos, OT_ineq, OT_eq, OT_ineqB, OT_ineqP, OT_none };
typedef rai::Array<ObjectiveType> ObjectiveTypeA;
extern ObjectiveTypeA& NoObjectiveTypeA;

//===========================================================================

/** The NLP abstraction provides a solver access to all it needs. To declare a NLP problem, the user needs to
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
struct NLP : rai::NonCopyable {
  //-- problem signature: needs to be defined in the constructor or a derived class
  uint dimension=0;
  ObjectiveTypeA featureTypes;
  arr bounds;

  virtual ~NLP() {}

  void copySignature(const NLP& P) {
    dimension = P.dimension;
    bounds = P.bounds;
    featureTypes = P.featureTypes;
  };

  //-- virtual methods that define the problem

  // essential method that needs overload
  virtual void evaluate(arr& phi, arr& J, const arr& x) = 0;       //evaluate all features and (optionally) their Jacobians for state x

  // optional initialization method
  virtual arr  getInitializationSample(); //get an initialization (for MC sampling/restarts) [default: initialize random within bounds]

  // optional evaluation of Hessian of all scalar objectives
  virtual void getFHessian(arr& H, const arr& x) { H.clear(); } //the Hessian of the sum of all f-features (or Hessian in addition to the Gauss-Newton Hessian of all other features)

  // optional: return some info on the problem and the last evaluation, potentially with display
  virtual void report(ostream& os, int verbose, const char* msg=0);

  //-- utilities
  shared_ptr<NLP> ptr() { return shared_ptr<NLP>(this, [](NLP*) {}); }
  double eval_scalar(arr& g, arr& H, const arr& x);
  bool checkJacobian(const arr& x, double tolerance, const StringA& featureNames= {});
  bool checkHessian(const arr& x, double tolerance);
  bool checkBounds(bool strictlyLarger);
  arr getUniformSample() { return bounds[0] + rand(dimension) % (bounds[1] - bounds[0]); }
  rai::String reportSignature();
  uint get_numOfType(const ObjectiveType& ot) {
    uint d=0;
    for(uint i=0; i<featureTypes.N; i++) if(featureTypes(i)==ot) d++;
    return d;
  }
  arr summarizeErrors(const arr& phi);
};

//===========================================================================

struct NLP_Factored : NLP {
  //-- problem factorization: needs to be defined in the constructor or a derived class
  uintA variableDimensions; //the size of each variable block
  uintA featureDimensions;  //the size of each feature block
  uintAA featureVariables;  //which variables the j-th feature block depends on

  //-- structured (local) setting variable and evaluate feature
  virtual void setAllVariables(const arr& x) { NIY; } //set all variables at once
  virtual void setSingleVariable(uint var_id, const arr& x) = 0; //set a single variable block
  virtual void evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H) = 0; //get a single feature block

  virtual arr  getSingleVariableInitSample(uint var_id) { return {}; } //get an initialization (for MC sampling/restarts) [default: initialize random within bounds]
  virtual void randomizeSingleVariable(uint var_id) { return; }

  //-- unstructured (batch) evaluation
  virtual void evaluate(arr& phi, arr& J, const arr& x); //default implementation: loop through setSingleVariable and evaluateSingleFeature

  virtual void subSelect(const uintA& activeVariables, const uintA& conditionalVariables) { NIY }
  virtual uint numTotalVariables() { NIY; return 0; }

  virtual rai::String getVariableName(uint var_id);
};

//===========================================================================
// TRIVIAL only header

struct NLP_Traced : NLP {
  shared_ptr<NLP> P;
  uint evals=0;
  arr xTrace, costTrace, phiTrace, JTrace;
  bool trace_x=true;
  bool trace_costs=true;
  bool trace_phi=false;
  bool trace_J=false;

  NLP_Traced(const shared_ptr<NLP>& P) : P(P) {
    copySignature(*P);
  }

  void setTracing(bool _trace_x, bool _trace_costs, bool _trace_phi, bool _trace_J) {
    trace_x=_trace_x; trace_costs=_trace_costs, trace_phi=_trace_phi, trace_J=_trace_J;
  }
  void clear() {
    evals=0;
    xTrace.clear();
    costTrace.clear();
    phiTrace.clear();
    JTrace.clear();
  }

  virtual void evaluate(arr& phi, arr& J, const arr& x);

  //trivial
  virtual arr  getInitializationSample() { return P->getInitializationSample(); }
  virtual void getFHessian(arr& H, const arr& x) { P->getFHessian(H, x); }
  virtual void report(ostream& os, int verbose, const char* msg=0){ P->report(os, verbose, msg); }
};

//===========================================================================

struct NLP_Viewer {
  shared_ptr<NLP> P;
  shared_ptr<NLP_Traced> T;

  NLP_Viewer(const shared_ptr<NLP>& P, const shared_ptr<NLP_Traced>& T= {}) : P(P), T(T) {}

  void display(double mu=1e3, double muLB=-1.);
  void plotCostTrace();
};

//===========================================================================

struct RegularizedNLP : NLP {
  NLP& P;
  arr x_mean;
  double mu;

  RegularizedNLP(NLP& _P, double _mu=1.);

  void setRegularization(const arr& _x_mean, double x_var);
  void evaluate(arr& phi, arr& J, const arr& x);
};

//===========================================================================

struct SolverReturn {
  arr x;
  arr dual;
  uint evals=0;
  double time=0.;
  bool feasible=false;
  double sos=-1., f=-1., ineq=-1., eq=-1.;
  bool done=false;
  void write(ostream& os) const;
};
stdOutPipe(SolverReturn)

//===========================================================================
// TRIVIAL only header

struct Conv_NLP_TrivialFactoreded : NLP_Factored {
  shared_ptr<NLP> P;
  arr x_buffer;

  Conv_NLP_TrivialFactoreded(const shared_ptr<NLP>& P) : P(P) {
    copySignature(*P);
    variableDimensions = { dimension };
    featureDimensions = { featureTypes.N };
    featureVariables = { uintA({0}) };
  }

  virtual arr  getInitializationSample() { return P->getInitializationSample(); }

  virtual void setSingleVariable(uint var_id, const arr& x) { x_buffer = x; }
  virtual void evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H) {  P->evaluate(phi, J, x_buffer);   if(!!H) NIY;  }
};

//===========================================================================

struct Conv_FactoredNLP_BandedNLP : NLP {
  shared_ptr<NLP_Factored> P;
  uint maxBandSize;
  bool sparseNotBanded;
  uintA varDimIntegral, featDimIntegral;
  //buffers
  arrA J_i;

  Conv_FactoredNLP_BandedNLP(const shared_ptr<NLP_Factored>& P, uint _maxBandSize, bool _sparseNotBanded=false);

  // trivial
  virtual arr  getInitializationSample() { return P->getInitializationSample(); }
  virtual void getFHessian(arr& H, const arr& x) { P->getFHessian(H, x); }

  virtual void evaluate(arr& phi, arr& J, const arr& x);
};

//===========================================================================

