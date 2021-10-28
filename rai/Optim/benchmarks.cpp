/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "benchmarks.h"
#include "lagrangian.h"
//#include "functions.h"

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
  for(uint i=1; i<x.N; i++) f += rai::sqr(x(i)-rai::sqr(x(i-1))) + .01*rai::sqr(1-10.*x(i-1));
//  f = ::log(1.+f);
  if(!!g) {
    g.resize(x.N).setZero();
    for(uint i=1; i<x.N; i++) {
      g(i) += 2.*(x(i)-rai::sqr(x(i-1)));
      g(i-1) += 2.*(x(i)-rai::sqr(x(i-1)))*(-2.*x(i-1));
      g(i-1) += .01*2.*(1-10.*x(i-1))*(-10.);
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

      //g(i-1) += .01*2.*(1-10.*x(i-1))*(-10.);
      H(i-1, i-1) += .01*2.*(-10.)*(-10.);
    }
  }
  return f;
};

struct MP_Rosenbrock : ScalarUnconstrainedProgram {
  MP_Rosenbrock(uint dim) { dimension=dim; }
  virtual double f(arr &g, arr &H, const arr &x){ return _RosenbrockFunction(g, H, x); }
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

struct MP_Rastrigin : ScalarUnconstrainedProgram {
  MP_Rastrigin(uint dim){ dimension=dim; }
  virtual uint getDimension(){ return dimension; }
  virtual double f(arr &g, arr &H, const arr &x){ return _RastriginFunction(g, H, x); }
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
    if(condition.N!=x.N) {
      condition.resize(x.N);
      double cond = rai::getParameter<double>("condition");
      double curv = rai::getParameter<double>("curvature");
      if(x.N>1) {
        for(uint i=0; i<x.N; i++) condition(i) = curv*pow(cond, 0.5*i/(x.N-1));
      } else {
        condition = curv;
      }
    }

    arr C = diag(condition);
    C(0,1) = C(0,0);
    C(1,0) = -C(1,1);
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
    if(!!g) g = ~C*g; //elem-wise product
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

MP_RandomSquared::MP_RandomSquared(uint _n, double condition) : n(_n) {
  dimension = n;
  featureTypes = consts<ObjectiveType>(OT_sos, n);

  //let M be a ortho-normal matrix (=random rotation matrix)
  M.resize(n, n);
  rndUniform(M, -1., 1., false);
  //orthogonalize
  for(uint i=0; i<n; i++) {
    for(uint j=0; j<i; j++) M[i]()-=scalarProduct(M[i], M[j])*M[j];
    M[i]()/=length(M[i]);
  }
  //we condition each column of M with powers of the condition
  for(uint i=0; i<n; i++) M[i]() *= pow(condition, double(i) / (2.*double(n - 1)));
}

//===========================================================================

ChoiceConstraintFunction::ChoiceConstraintFunction() {
  which = (WhichConstraint) rai::getParameter<double>("constraintChoice");
  n = rai::getParameter<uint>("dim", 2);
}

uint ChoiceConstraintFunction::getDimension() {
  return n;
}

void ChoiceConstraintFunction::getFeatureTypes(ObjectiveTypeA& tt) {
  tt.clear();
  tt.append(OT_f);
  switch(which) {
    case none:
      break;
    case wedge2D:
      tt.append(consts(OT_ineq, n));
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
      tt.append(consts(OT_ineq, 5*n+5));
      break;
    case boundConstrained:
      break;
    case boundConstrainedIneq:
      tt.append(OT_ineq);
      break;
    default: HALT("not taken care of");
  }
}

void ChoiceConstraintFunction::getBounds(arr& bounds_lo, arr& bounds_hi) {
  bounds_lo.resize(n) = -2.;
  bounds_hi.resize(n) = +2.;
  if(which==boundConstrained){
    bounds_lo(0) = +0.5;
//    bounds_lo(1) = +0.51;
  }
}

void ChoiceConstraintFunction::evaluate(arr& phi, arr& J, const arr& x) {
  CHECK_EQ(x.N, n, "");
  phi.clear();  if(!!J) J.clear();

  phi.append(ChoiceFunction()(J, NoArr, x));

  switch(which) {
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
      phi.append(randomG * cat({1.}, x));
      if(!!J) J.append(randomG.sub(0, -1, 1, -1));
    } break;
    case boundConstrained: {
//      phi.append(1. - x(0));
//      if(!!J) { J.append( eyeVec(x.N, 0) ); }
    } break;
    case boundConstrainedIneq: {
      phi.append(0.5 - x(0));
      if(!!J) { J.append( -eyeVec(x.N, 0) ); }
    } break;
  }

  if(!!J) J.reshape(J.N/x.N, x.N);
}

void ChoiceConstraintFunction::getFHessian(arr& H, const arr& x) {
  ChoiceFunction()(NoArr, H, x);
}



std::shared_ptr<MathematicalProgram> getBenchmarkFromCfg(){
  rai::Enum<BenchmarkSymbol> bs (rai::getParameter<rai::String>("benchmark"));
  uint dim = rai::getParameter<uint>("benchmark/dim", 2);
  double forsyth = rai::getParameter<double>("benchmark/forsyth", -1.);

  //-- unconstrained problems

  {
    std::shared_ptr<ScalarUnconstrainedProgram> mp;

    if(bs==BS_Rosenbrock) mp = make_shared<MP_Rosenbrock>(dim);
    else if(bs==BS_Rastrigin) mp = make_shared<MP_Rastrigin>(dim);
    else if(forsyth>0.){
      shared_ptr<MathematicalProgram> org;
      if(bs==BS_RandomSquared) org = make_shared<MP_RandomSquared>(dim);
      else if(bs==BS_RastriginSOS) org = make_shared<MP_RastriginSOS>();
      if(org){
        auto lag = make_shared<LagrangianProblem>(org); //convert to scalar
        mp = make_shared<ScalarUnconstrainedProgram>(lag, dim);
      }
    }

    if(mp){
      if(forsyth>0.) mp->forsythAlpha = forsyth;
      return mp;
    }
  }

  //-- constrained problems

  std::shared_ptr<MathematicalProgram> mp;

  if(bs==BS_RandomLP) mp = make_shared<MP_RandomLP>(dim);
  else if(bs==BS_RandomSquared) mp = make_shared<MP_RandomSquared>(dim);
  else if(bs==BS_RastriginSOS) mp = make_shared<MP_RastriginSOS>();
  else if(bs==BS_Wedge) mp = make_shared<MP_Wedge>();
  else if(bs==BS_HalfCircle) mp = make_shared<MP_HalfCircle>();
  else if(bs==BS_CircleLine) mp = make_shared<MP_CircleLine>();
  else HALT("can't interpret benchmark symbol: " <<bs);

  return mp;
}
