/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "benchmarks.h"
#include "lagrangian.h"

#include <math.h>

//===========================================================================

enum BenchmarkSymbol {
  BS_none=0,
  BS_Rosenbrock,
  BS_Rastrigin,
  BS_RastriginSOS,
  BS_Square,
  BS_RandomSquared,
  BS_Sum,
  BS_RandomLP,
  BS_SimpleConstrained,
  BS_Wedge,
  BS_HalfCircle,
  BS_CircleLine,
};

template<> const char* rai::Enum<BenchmarkSymbol>::names []= {
  "none",
  "Rosenbrock",
  "Rastrigin",
  "RastriginSOS",
  "Square",
  "RandomSquared",
  "Sum",
  "RandomLP",
  "SimpleConstrained",
  "Wedge",
  "HalfCircle",
  "CircleLine",
  0
};

//===========================================================================

double _RosenbrockFunction(arr& g, arr& H, const arr& x) {
  double f=0.;
  for(uint i=1; i<x.N; i++) f += rai::sqr(x(i)-rai::sqr(x(i-1))) + .01*rai::sqr(1-x(i-1));
//  f = ::log(1.+f);
  if(!!g) {
    g.resize(x.N).setZero();
    for(uint i=1; i<x.N; i++) {
      g(i) += 2.*(x(i)-rai::sqr(x(i-1)));
      g(i-1) += 2.*(x(i)-rai::sqr(x(i-1)))*(-2.*x(i-1));
      g(i-1) -= .01*2.*(1-x(i-1));
    }
  }
  if(!!H) {
    H.resize(x.N, x.N).setZero();
    for(uint i=1; i<x.N; i++) {
      //g(i) += 2.*(x(i)-rai::sqr(x(i-1)));
      H(i, i) += 2.;
      H(i, i-1) += -4.*x(i-1);

      //g(i-1) += 2.*(x(i)-rai::sqr(x(i-1)))*(-2.*x(i-1));
      H(i-1, i) += -4.*x(i-1);
      H(i-1, i-1) += -4.*x(i-1)*(-2.*x(i-1)) - 4.*(x(i)-rai::sqr(x(i-1)));

      //g(i-1) += .01*2.*(1-x(i-1))*(-10.);
      H(i-1, i-1) += .01*2.;
    }
  }
  return f;
}

struct NLP_Rosenbrock : ScalarUnconstrainedProgram {
  NLP_Rosenbrock(uint dim) { dimension=dim; }
  virtual double f(arr& g, arr& H, const arr& x) { return _RosenbrockFunction(g, H, x); }
};

//===========================================================================

double _RastriginFunction(arr& g, arr& H, const arr& x) {
  double A=.5, f=A*x.N;
  for(uint i=0; i<x.N; i++) f += x(i)*x(i) - A*::cos(10.*x(i));
  if(!!g) {
    g.resize(x.N);
    for(uint i=0; i<x.N; i++) g(i) = 2*x(i) + 10.*A*::sin(10.*x(i));
  }
  if(!!H) {
    H.resize(x.N, x.N).setZero();
    for(uint i=0; i<x.N; i++) H(i, i) = 2 + 100.*A*::cos(10.*x(i));
  }
  return f;
}

struct NLP_Rastrigin : ScalarUnconstrainedProgram {
  NLP_Rastrigin(uint dim) { dimension=dim; }
  virtual double f(arr& g, arr& H, const arr& x) { return _RastriginFunction(g, H, x); }
};

//===========================================================================

double _SquareFunction(arr& g, arr& H, const arr& x) {
  if(!!g) g=2.*x;
  if(!!H) H.setDiag(2., x.N);
  return sumOfSqr(x);
}

ScalarFunction SquareFunction() { return _SquareFunction; }

//===========================================================================

double _SumFunction(arr& g, arr& H, const arr& x) {
  if(!!g) { g.resize(x.N); g=1.; }
  if(!!H) { H.resize(x.N, x.N); H.setZero(); }
  return sum(x);
}

ScalarFunction SumFunction() { return _SumFunction; }

//===========================================================================

double _HoleFunction(arr& g, arr& H, const arr& x) {
  double f=exp(-sumOfSqr(x));
  if(!!g) g=2.*f*x;
  if(!!H) { H.setDiag(2.*f, x.N); H -= 4.*f*(x^x); }
  f = 1.-f;
  return f;
}

ScalarFunction HoleFunction() { return _HoleFunction; }

//===========================================================================

struct _ChoiceFunction : ScalarFunction {
  enum Which { none=0, sum, square, hole, rosenbrock, rastrigin } which;
  arr condition;
  _ChoiceFunction():which(none) {
    ScalarFunction::operator=(
      [this](arr& g, arr& H, const arr& x) -> double { return this->fs(g, H, x); }
    );
  }

  double fs(arr& g, arr& H, const arr& x) {
    //initialize on first call
    if(which==none) {
      which = (Which) rai::getParameter<double>("fctChoice");
    }
    arr C = eye(x.N);
    double cond = rai::getParameter<double>("condition");
    if(cond>1.) {
      if(condition.N!=x.N) {
        condition.resize(x.N);
        double curv = rai::getParameter<double>("curvature");
        if(x.N>1) {
          for(uint i=0; i<x.N; i++) condition(i) = curv*pow(cond, 0.5*i/(x.N-1));
        } else {
          condition = curv;
        }
      }

      C = diag(condition);
      C(0, 1) = C(0, 0);
      C(1, 0) = -C(1, 1);
    }
    arr y = C * x;
    double f;
    switch(which) {
      case sum: f = _SumFunction(g, H, y); break;
      case square: f = _SquareFunction(g, H, y); break;
      case hole: f = _HoleFunction(g, H, y); break;
      case rosenbrock: f = _RosenbrockFunction(g, H, y); break;
      case rastrigin: f = _RastriginFunction(g, H, y); break;
      default: NIY;
    }
    if(!!g) g = ~C * g;
    if(!!H) H = ~C * H * C;
    return f;
  }

  //  ScalarFunction get_f(){
  //    return [this](arr& g, arr& H, const arr& x) -> double { return this->fs(g, H, x); };
  //  }

} choice;

ScalarFunction ChoiceFunction() { return (ScalarFunction&)choice; }

//===========================================================================

void generateConditionedRandomProjection(arr& M, uint n, double condition) {
}

//===========================================================================

NLP_Squared::NLP_Squared(uint _n, double condition, bool random) : n(_n) {
  dimension = n;
  featureTypes = rai::consts<ObjectiveType>(OT_sos, n);

  //let C be a ortho-normal matrix (=random rotation matrix)
  C.resize(n, n);

  if(random) {
    rndUniform(C, -1., 1., false);
    //orthogonalize
    for(uint i=0; i<n; i++) {
      for(uint j=0; j<i; j++) C[i] -= scalarProduct(C[i], C[j])*C[j];
      C[i] /= length(C[i]);
    }
    //we condition each column of M with powers of the condition
    for(uint i=0; i<n; i++) C[i] *= pow(condition, double(i) / (2.*double(n - 1)));

  } else {
    arr cond(n);
    if(n>1) {
      for(uint i=0; i<n; i++) cond(i) = pow(condition, 0.5*i/(n-1));
    } else {
      cond = 1.;
    }

    C = diag(cond);
//    C(0,1) = C(0,0);
//    C(1,0) = -C(1,1);
  }
}

//===========================================================================

ChoiceConstraintFunction::ChoiceConstraintFunction() {
  which = (WhichConstraint) rai::getParameter<double>("constraintChoice");
  n = rai::getParameter<uint>("dim", 2);

  dimension = n;

  bounds.resize(2,n);
  bounds[0] = -2.;
  bounds[1] = +2.;
  if(which==boundConstrained) {
    bounds(0,0) = +0.5;
    //    bounds_lo(1) = +0.51;
  }

  ObjectiveTypeA& tt = featureTypes;
  tt.clear();
  tt.append(OT_f);
  switch(which) {
    case none:
      break;
    case wedge2D:
      tt.append(rai::consts(OT_ineq, n));
      break;
    case halfcircle2D:
      tt.append(OT_ineq);
      tt.append(OT_ineq);
      break;
    case circleLine2D:
      tt.append(OT_ineq);
      tt.append(OT_eq);
      break;
    case randomLinear:
      tt.append(rai::consts(OT_ineq, 5*n+5));
      break;
    case boundConstrained:
      break;
    case boundConstrainedIneq:
      tt.append(OT_ineq);
      break;
    default: HALT("not taken care of");
  }
}

void ChoiceConstraintFunction::evaluate(arr& phi, arr& J, const arr& x) {
  CHECK_EQ(x.N, n, "");
  phi.clear();  if(!!J) J.clear();

  phi.append(ChoiceFunction()(J, NoArr, x));

  switch(which) {
    case none: HALT("should not be here")
    case wedge2D:
      for(uint i=0; i<x.N; i++) { phi.append(-sum(x)+1.5*x(i)-.2); }
      if(!!J) { arr Jg(x.N, x.N); Jg=-1.; for(uint i=0; i<x.N; i++) Jg(i, i) = +.5; J.append(Jg); }
      break;
    case halfcircle2D:
      phi.append(sumOfSqr(x)-.25);  if(!!J) J.append(2.*x);       //feasible=IN circle of radius .5
      phi.append(-x(0)-.2);         if(!!J) { J.append(zeros(x.N)); J.elem(-x.N) = -1.; }      //feasible=right of -.2
      break;
    case circleLine2D:
      phi.append(sumOfSqr(x)-.25);  if(!!J) J.append(2.*x);       //feasible=IN circle of radius .5
      phi.append(x(0));             if(!!J) { J.append(zeros(x.N)); J.elem(-x.N) = 1.; }
      break;
    case randomLinear: {
      if(!randomG.N) {
        randomG.resize(5*x.N+5, x.N+1);
        rndGauss(randomG, 1.);
        for(uint i=0; i<randomG.d0; i++) {
          if(randomG(i, 0)>0.) randomG(i, 0)*=-1.; //ensure (0,0) is feasible
          randomG(i, 0) -= .2;
        }
      }
      CHECK_EQ(randomG.d1, x.N+1, "you changed dimensionality");
      phi.append(randomG * (arr{1.}, x));
      if(!!J) J.append(randomG.sub(0, -1, 1, -1));
    } break;
    case boundConstrained: {
//      phi.append(1. - x(0));
//      if(!!J) { J.append( eyeVec(x.N, 0) ); }
    } break;
    case boundConstrainedIneq: {
      phi.append(0.5 - x(0));
      if(!!J) { J.append(-eyeVec(x.N, 0)); }
    } break;
  }

  if(!!J) J.reshape(J.N/x.N, x.N);
}

void ChoiceConstraintFunction::getFHessian(arr& H, const arr& x) {
  ChoiceFunction()(NoArr, H, x);
}

std::shared_ptr<NLP> getBenchmarkFromCfg() {
  rai::Enum<BenchmarkSymbol> bs(rai::getParameter<rai::String>("benchmark"));
  uint dim = rai::getParameter<uint>("benchmark/dim", 2);
  double forsyth = rai::getParameter<double>("benchmark/forsyth", -1.);
  double condition = rai::getParameter<double>("benchmark/condition", 10.);

  //-- unconstrained problems

  {
    std::shared_ptr<ScalarUnconstrainedProgram> nlp;

    if(bs==BS_Rosenbrock) nlp = make_shared<NLP_Rosenbrock>(dim);
    else if(bs==BS_Rastrigin) nlp = make_shared<NLP_Rastrigin>(dim);
    else if(forsyth>0.) {
      shared_ptr<NLP> org;
      if(bs==BS_Square) org = make_shared<NLP_Squared>(dim, condition, false);
      else if(bs==BS_RandomSquared) org = make_shared<NLP_Squared>(dim, condition, true);
      else if(bs==BS_RastriginSOS) org = make_shared<NLP_RastriginSOS>();
      if(org) {
        auto lag = make_shared<LagrangianProblem>(org); //convert to scalar
        nlp = make_shared<ScalarUnconstrainedProgram>(lag, dim);
      }
    }

    if(nlp) {
      nlp->bounds = rai::getParameter<arr>("benchmark/bounds", {});
      nlp->bounds.reshape(2,-1);
      if(forsyth>0.) nlp->forsythAlpha = forsyth;
      return nlp;
    }
  }

  //-- constrained problems

  std::shared_ptr<NLP> nlp;

  if(bs==BS_RandomLP) nlp = make_shared<NLP_RandomLP>(dim);
  else if(bs==BS_Square) nlp = make_shared<NLP_Squared>(dim, condition, false);
  else if(bs==BS_RandomSquared) nlp = make_shared<NLP_Squared>(dim, condition, true);
  else if(bs==BS_RastriginSOS) nlp = make_shared<NLP_RastriginSOS>();
  else if(bs==BS_Wedge) nlp = make_shared<NLP_Wedge>();
  else if(bs==BS_HalfCircle) nlp = make_shared<NLP_HalfCircle>();
  else if(bs==BS_CircleLine) nlp = make_shared<NLP_CircleLine>();
  else HALT("can't interpret benchmark symbol: " <<bs);

  nlp->bounds = rai::getParameter<arr>("benchmark/bounds", {});
  if(nlp->bounds.N){
    nlp->bounds.reshape(2, nlp->dimension);
  }

  return nlp;
}

void NLP_RastriginSOS::evaluate(arr& phi, arr& J, const arr& x) {
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
