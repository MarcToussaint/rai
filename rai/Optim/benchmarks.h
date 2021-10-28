/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "optimization.h"
#include "MathematicalProgram.h"

extern ScalarFunction RosenbrockFunction();
extern ScalarFunction RastriginFunction();
extern ScalarFunction SquareFunction();
extern ScalarFunction SumFunction();
extern ScalarFunction HoleFunction();
extern ScalarFunction ChoiceFunction();

std::shared_ptr<MathematicalProgram> getBenchmarkFromCfg();

//===========================================================================

struct ScalarUnconstrainedProgram : MathematicalProgram {
  double forsythAlpha = -1.;
  shared_ptr<ScalarFunction> S;
  uint dim;
  ScalarUnconstrainedProgram() {}
  ScalarUnconstrainedProgram(const shared_ptr<ScalarFunction>& S, uint dim) : S(S), dim(dim) {}
  uint getDimension(){ return dim; }
  void getFeatureTypes(ObjectiveTypeA& ot) { ot = {OT_f}; }
  void evaluate(arr& phi, arr& J, const arr& x) {
    double y = f(J, NoArr, x);
    if(forsythAlpha>0.){
      CHECK_GE(y, 0., "Forsyth wrapping only makes sense for positive (sqr-like) functions");
      y = y / (forsythAlpha+y);
    }
    phi = {y};
    if(!!J){
      J.reshape(1, x.N);
      if(forsythAlpha>0.) J *= y;
    }
  }
  void getFHessian(arr& H, const arr& x) {
    f(NoArr, H, x);
  }
  virtual double f(arr& g, arr& H, const arr& x){
    CHECK(S, "no scalar function given in the constructor");
    return (*S)(g, H, x);
  }
};

//===========================================================================

struct MP_TrivialSquareFunction : MathematicalProgram {
  uint dim;
  double lo, hi;

  MP_TrivialSquareFunction(uint dim=10, double lo=-1., double hi=1.) : dim(dim), lo(lo), hi(hi) {}

  virtual uint getDimension() { return dim; }
  virtual void getBounds(arr& bounds_lo, arr& bounds_up) { //lower/upper bounds for the decision variable (may be {})
    bounds_lo = consts<double>(lo, dim);
    bounds_up = consts<double>(hi, dim);
  }
  virtual void getFeatureTypes(ObjectiveTypeA& featureTypes) {
    featureTypes = consts<ObjectiveType>(OT_sos, dim);
  }

  void evaluate(arr& phi, arr& J, const arr& x) {
    phi = x;
    if(!!J) J.setId(x.N);
  }
};

//===========================================================================

struct MP_RandomLP : MathematicalProgram {
  arr randomG;

  MP_RandomLP(uint dim) {
    randomG.resize(5*dim+5, dim+1);
    rndGauss(randomG, 1.);
    for(uint i=0; i<randomG.d0; i++) {
      if(randomG(i, 0)>0.) randomG(i, 0) *= -1.; //ensure (0,0) is feasible
      randomG(i, 0) -= .2;
    }
  }

  virtual uint getDimension(){ return randomG.d1-1; }

  virtual void getFeatureTypes(ObjectiveTypeA& tt) {
    tt = { OT_f };
    tt.append(consts(OT_ineq, randomG.d0));
  }

  virtual void evaluate(arr &phi, arr &J, const arr &x) {
    phi = {sum(x)};
    if(!!J) J = ones(1, x.N);

    phi.append(randomG * cat({1.}, x));
    if(!!J) J.append(randomG.sub(0, -1, 1, -1));
  }

};

//===========================================================================

struct ChoiceConstraintFunction : MathematicalProgram {
  enum WhichConstraint { none=0, wedge2D=1, halfcircle2D, randomLinear, circleLine2D, boundConstrained, boundConstrainedIneq } which;
  uint n;
  arr randomG;
  ChoiceConstraintFunction();

  uint getDimension();

  void getFeatureTypes(ObjectiveTypeA& tt);

  void getBounds(arr& bounds_lo, arr& bounds_hi);

  void evaluate(arr& phi, arr& J, const arr& x);
  void getFHessian(arr& H, const arr& x);
//  virtual uint dim_g(){
//    if(which==randomLinear) return ;
//    if(which==wedge2D) return n;
//    if(which==circleLine2D) return 1;
//    return 2;
//  }
//  virtual uint dim_h(){
//    if(which==circleLine2D) return 1;
//    return 0;
//  }
};

//===========================================================================

struct SimpleConstraintFunction : MathematicalProgram {
  SimpleConstraintFunction() {
  }
  virtual void getFeatureTypes(ObjectiveTypeA& tt) { tt = { OT_sos, OT_sos, OT_ineq, OT_ineq }; }
  virtual void evaluate(arr& phi, arr& J, const arr& _x) {
    CHECK_EQ(_x.N, 2, "");
    phi.resize(4);
    if(!!J) { J.resize(4, 2); J.setZero(); }

    //simple squared potential, displaced by 1
    arr x(_x);
    x(0) -= 1.;
    phi({0, 1}) = x;
    if(!!J) J.setMatrixBlock(eye(2), 0, 0);
    x(0) += 1.;

    phi(2) = .25-sumOfSqr(x);  if(!!J) J[2]() = -2.*x; //OUTSIDE the circle
    phi(3) = x(0);             if(!!J) J(3, 0) = 1.;
  }
};

//===========================================================================

struct MP_RastriginSOS : MathematicalProgram {
  double a;
  double condition;
  MP_RastriginSOS() {
    a = rai::getParameter<double>("Rastrigin/a");
    condition = rai::getParameter<double>("benchmark/condition");
  }
  virtual uint getDimension(){ return 2; }
  virtual void getFeatureTypes(ObjectiveTypeA &featureTypes){ featureTypes = consts<ObjectiveType>(OT_sos, 4); }
  virtual void evaluate(arr &phi, arr &J, const arr &x) {
    CHECK_EQ(x.N, 2, "");
    phi.resize(4);
    phi(0) = sin(a*x(0));
    phi(1) = sin(a*condition*x(1));
    phi(2) = 2.*x(0);
    phi(3) = 2.*condition*x(1);
    if(!!J) {
      J.resize(4, 2);
      J.setZero();
      J(0, 0) = cos(a*x(0))*a;
      J(1, 1) = cos(a*condition*x(1))*a*condition;
      J(2, 0) = 2.;
      J(3, 1) = 2.*condition;
    }
  }
};

//===========================================================================

/// $f(x) = x^T C x$ where C has eigen values ranging from 1 to 'condition'
struct MP_RandomSquared : MathematicalProgram {
  arr M; /// $C = M^T M $
  uint n;  /// dimensionality of $x$

  MP_RandomSquared(uint n, double condition=100.);

  virtual uint getDimension(){ return n; }
  virtual void getFeatureTypes(ObjectiveTypeA &featureTypes){ featureTypes = consts<ObjectiveType>(OT_sos, n); }
  virtual void evaluate(arr &phi, arr &J, const arr &x){ phi=M*x; if(!!J) J=M; }
};

//===========================================================================

struct MP_Wedge : MathematicalProgram {
  virtual uint getDimension(){ return 2; }
  virtual void getFeatureTypes(ObjectiveTypeA& tt) {
    tt = { OT_f };
    tt.append(consts(OT_ineq, 2));
  }

  virtual void evaluate(arr &phi, arr &J, const arr &x) {
    phi = {sum(x)};
    if(!!J) J = ones(1, x.N);

    for(uint i=0; i<x.N; i++) { phi.append(-sum(x)+1.5*x(i)-.2); }
    if(!!J) { arr Jg(x.N, x.N); Jg=-1.; for(uint i=0; i<x.N; i++) Jg(i, i) = +.5; J.append(Jg); }
  }
};

//===========================================================================

struct MP_HalfCircle : MathematicalProgram {
  virtual uint getDimension(){ return 2; }
  virtual void getFeatureTypes(ObjectiveTypeA& tt) {
    tt = { OT_f };
    tt.append(consts(OT_ineq, 2));
  }

  virtual void evaluate(arr &phi, arr &J, const arr &x) {
    phi = {sum(x)};
    if(!!J) J = ones(1, x.N);

    phi.append(sumOfSqr(x)-.25);  if(!!J) J.append(2.*x);       //feasible=IN circle of radius .5
    phi.append(-x(0)-.2);         if(!!J) { J.append(zeros(x.N)); J.elem(-x.N) = -1.; }      //feasible=right of -.2
  }
};

//===========================================================================

struct MP_CircleLine : MathematicalProgram {
  virtual uint getDimension(){ return 2; }
  virtual void getFeatureTypes(ObjectiveTypeA& tt) {
    tt = { OT_f };
    tt.append(OT_ineq);
    tt.append(OT_eq);
  }

  virtual void evaluate(arr &phi, arr &J, const arr &x) {
    phi = {sum(x)};
    if(!!J) J = ones(1, x.N);

    phi.append(sumOfSqr(x)-.25);  if(!!J) J.append(2.*x);       //feasible=IN circle of radius .5
    phi.append(x(0));             if(!!J) { J.append(zeros(x.N)); J.elem(-x.N) = 1.; }
  }
};

