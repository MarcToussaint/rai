/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "NLP.h"
#include "../Core/util.h"

extern ScalarFunction RosenbrockFunction();
extern ScalarFunction RastriginFunction();
extern ScalarFunction SquareFunction();
extern ScalarFunction SumFunction();
extern ScalarFunction HoleFunction();
extern ScalarFunction ChoiceFunction();

std::shared_ptr<NLP> getBenchmarkFromCfg();

//===========================================================================

struct ScalarUnconstrainedProgram : NLP {
  double forsythAlpha = -1.;
  shared_ptr<ScalarFunction> S;
  ScalarUnconstrainedProgram() { featureTypes = {OT_f}; }
  ScalarUnconstrainedProgram(const shared_ptr<ScalarFunction>& S, uint dim) : S(S) {
    dimension=dim;
    featureTypes = {OT_f};
  }
  void evaluate(arr& phi, arr& J, const arr& x) {
    double y = f(J, NoArr, x);
    if(forsythAlpha>0.) {
      CHECK_GE(y, 0., "Forsyth wrapping only makes sense for positive (sqr-like) functions");
      y = y / (forsythAlpha+y);
    }
    phi = {y};
    if(!!J) {
      J.reshape(1, x.N);
      if(forsythAlpha>0.) J *= y;
    }
  }
  void getFHessian(arr& H, const arr& x) {
    f(NoArr, H, x);
  }
  virtual double f(arr& g, arr& H, const arr& x) {
    CHECK(S, "no scalar function given in the constructor");
    return (*S)(g, H, x);
  }
};

//===========================================================================

struct NLP_TrivialSquareFunction : NLP {
  double lo, hi;

  NLP_TrivialSquareFunction(uint dim=10, double lo=-1., double up=1.) {
    dimension = dim;
    featureTypes = rai::consts<ObjectiveType>(OT_sos, dimension);
    bounds.resize(2, dimension);
    bounds[0] = lo;
    bounds[1] = up;
  }

  void evaluate(arr& phi, arr& J, const arr& x) {
    phi = x;
    if(!!J) J.setId(x.N);
  }
};

//===========================================================================

struct NLP_RandomLP : NLP {
  arr randomG;

  NLP_RandomLP(uint dim=2);

  virtual void evaluate(arr& phi, arr& J, const arr& x);

};

//===========================================================================

struct BoxNLP : NLP {
  BoxNLP();
  void evaluate(arr& phi, arr& J, const arr& x);
};

//===========================================================================

struct ModesNLP : NLP {
  arr cen;
  arr radii;

  ModesNLP();
  void evaluate(arr& phi, arr& J, const arr& x);
};

//===========================================================================

struct SimpleConstraintFunction : NLP {
  SimpleConstraintFunction() {
    featureTypes = { OT_sos, OT_sos, OT_ineq, OT_ineq };
  }
  virtual void evaluate(arr& phi, arr& J, const arr& _x) {
    CHECK_EQ(_x.N, 2, "");
    phi.resize(4);
    if(!!J) { J.resize(4, 2).setZero(); }

    //simple squared potential, displaced by 1
    arr x(_x);
    x(0) -= 1.;
    phi({0, 1+1}) = x;
    if(!!J) J.setMatrixBlock(eye(2), 0, 0);
    x(0) += 1.;

    phi(2) = .25-sumOfSqr(x);  if(!!J) J[2] = -2.*x; //OUTSIDE the circle
    phi(3) = x(0);             if(!!J) J(3, 0) = 1.;
  }
};

//===========================================================================

struct NLP_RastriginSOS : NLP {
  double a;
  double condition;
  NLP_RastriginSOS();
  virtual void evaluate(arr& phi, arr& J, const arr& x);
};

//===========================================================================

/// $f(x) = x^T C x$ where C has eigen values ranging from 1 to 'condition'
struct NLP_Squared : NLP {
  arr C; /// $A = C^T C $
  uint n;  /// dimensionality of $x$

  NLP_Squared(uint n=2, double condition=100., bool random=true);

  virtual void evaluate(arr& phi, arr& J, const arr& x) { phi=C*x; if(!!J) J=C; }
//  virtual arr getInitializationSample(){ return ones(n); }
};

//===========================================================================

struct NLP_Wedge : NLP {
  NLP_Wedge() {
    dimension=2;
    featureTypes = { OT_f };
    featureTypes.append(rai::consts(OT_ineq, 2));
  }

  virtual void evaluate(arr& phi, arr& J, const arr& x) {
    phi = {sum(x)};
    if(!!J) J = ones(1, x.N);

    for(uint i=0; i<x.N; i++) { phi.append(-sum(x)+1.5*x(i)-.2); }
    if(!!J) { arr Jg(x.N, x.N); Jg=-1.; for(uint i=0; i<x.N; i++) Jg(i, i) = +.5; J.append(Jg); }
  }
};

//===========================================================================

struct NLP_HalfCircle : NLP {
  NLP_HalfCircle() {
    dimension=2;
    featureTypes = { OT_f };
    featureTypes.append(rai::consts(OT_ineq, 2));
  }

  virtual void evaluate(arr& phi, arr& J, const arr& x) {
    phi = {sum(x)};
    if(!!J) J = ones(1, x.N);

    phi.append(sumOfSqr(x)-.25);  if(!!J) J.append(2.*x);       //feasible=IN circle of radius .5
    phi.append(-x(0)-.2);         if(!!J) { J.append(zeros(x.N)); J.elem(-x.N) = -1.; }      //feasible=right of -.2
  }
};

//===========================================================================

struct NLP_CircleLine : NLP {
  NLP_CircleLine() {
    dimension=2;
    featureTypes = { OT_f };
    featureTypes.append(OT_ineq);
    featureTypes.append(OT_eq);
  }

  virtual void evaluate(arr& phi, arr& J, const arr& x) {
    phi = {sum(x)};
    if(!!J) J = ones(1, x.N);

    phi.append(sumOfSqr(x)-.25);  if(!!J) J.append(2.*x);       //feasible=IN circle of radius .5
    phi.append(x(0));             if(!!J) { J.append(zeros(x.N)); J.elem(-x.N) = 1.; }
//    phi.append(0.);             if(!!J) { J.append(zeros(x.N)); }
  }
};

//===========================================================================

struct ChoiceConstraintFunction : NLP {
  enum WhichConstraint { none=0, wedge2D=1, halfcircle2D, randomLinear, circleLine2D, boundConstrained, boundConstrainedIneq } which;
  uint n;
  arr randomG;
  ChoiceConstraintFunction();

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

