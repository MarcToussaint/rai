/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "optimization.h"
#include "MathematicalProgram.h"
#include "KOMO_Problem.h"

extern ScalarFunction RosenbrockFunction();
extern ScalarFunction RastriginFunction();
extern ScalarFunction SquareFunction();
extern ScalarFunction SumFunction();
extern ScalarFunction HoleFunction();
extern ScalarFunction ChoiceFunction();

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

struct RandomLPFunction : MathematicalProgram {
  arr randomG;

  RandomLPFunction() {}

  void generateG(uint xN) {
    randomG.resize(5*xN+5, xN+1);
    rndGauss(randomG, 1.);
    for(uint i=0; i<randomG.d0; i++) {
      if(randomG(i, 0)>0.) randomG(i, 0)*=-1.; //ensure (0,0) is feasible
      randomG(i, 0) -= .2;
    }
  }

  uint dim_x() { return rnd(10)+10; }

  virtual void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x) {
    if(!randomG.N) generateG(x.N);
    CHECK_EQ(randomG.d1, x.N+1, "you changed dimensionality!");

    phi.clear();
    if(!!tt) tt.clear();
    if(!!J) J.clear();

    phi.append() = sum(x);
    if(!!tt) tt.append(OT_f);
    if(!!J) J.append(ones(1, x.N));
    if(!!H) H = zeros(x.N, x.N);

    phi.append(randomG * cat({1.}, x));
    if(!!tt) tt.append(consts(OT_ineq, randomG.d0));
    if(!!J) J.append(randomG.sub(0, -1, 1, -1));
    if(!!J) J.reshape(J.N/x.N, x.N);
  }
};

//===========================================================================

struct ChoiceConstraintFunction : MathematicalProgram {
  enum WhichConstraint { wedge2D=1, halfcircle2D, randomLinear, circleLine2D, boundConstrained, boundConstrainedIneq } which;
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

struct SinusesFunction:VectorFunction {
  double a;
  double condition;
  SinusesFunction() {
    a = rai::getParameter<double>("SinusesFunction_a");
    condition = rai::getParameter<double>("condition");
    NIY
  }
  virtual void fv(arr& phi, arr& J, const arr& x) {
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
struct SquaredCost : VectorFunction {
  arr M; /// $C = M^T M $
  uint n;  /// dimensionality of $x$

  SquaredCost(uint n, double condition=100.);
  void initRandom(uint n, double condition=100.);

  void fv(arr& y, arr& J, const arr& x);
};

//===========================================================================

/// Same as SquaredCost but $x_i \gets atan(x_i)$ before evaluating the squared cost
struct NonlinearlyWarpedSquaredCost : VectorFunction {
  uint n;  /// dimensionality of $x$
  SquaredCost sq;

  NonlinearlyWarpedSquaredCost(uint n, double condition=100.);
  void initRandom(uint n, double condition=100.);

  void fv(arr& y, arr& J, const arr& x);
};

//===========================================================================

struct ParticleAroundWalls2 : KOMO_Problem {
  //options of the problem
  uint T, k, n;
  bool useKernel;
  arr x;

  ParticleAroundWalls2():
    T(rai::getParameter<uint>("opt/ParticleAroundWalls/T", 1000)),
    k(rai::getParameter<uint>("opt/ParticleAroundWalls/k", 2)),
    n(rai::getParameter<uint>("opt/ParticleAroundWalls/n", 3)),
    useKernel(false) {}

  //implementations of the kOrderMarkov virtuals
  uint get_T() { return T; }
  uint get_k() { return k; }
  void getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes);

  void phi(arr& phi, arrA& J, arrA& H, uintA& featureTimes, ObjectiveTypeA& tt, const arr& x);
};
