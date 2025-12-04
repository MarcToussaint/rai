/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "testProblems_Opt.h"
#include "lagrangian.h"
#include "utils.h"
#include "../Algo/ann.h"

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

double NLP_Rosenbrock::f(arr& g, arr& H, const arr& x) {
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

//===========================================================================

double NLP_Rastrigin::f(arr& g, arr& H, const arr& x) {
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

//===========================================================================

struct _SquareFunction : ScalarFunction {
  double f(arr& g, arr& H, const arr& x) {
  if(!!g) g=2.*x;
  if(!!H) H.setDiag(2., x.N);
  return sumOfSqr(x);
  }
};

//===========================================================================

struct _SumFunction : ScalarFunction {
  double f(arr& g, arr& H, const arr& x) {
    if(!!g) { g.resize(x.N); g=1.; }
    if(!!H) { H.resize(x.N, x.N); H.setZero(); }
    return sum(x);
  }
};

//===========================================================================

struct _HoleFunction : ScalarFunction {
  double f(arr& g, arr& H, const arr& x) {
    double f=exp(-sumOfSqr(x));
    if(!!g) g=2.*f*x;
    if(!!H) { H.setDiag(2.*f, x.N); H -= 4.*f*(x^x); }
    f = 1.-f;
    return f;
  }
};

//===========================================================================

struct _ChoiceFunction : ScalarFunction {
  enum Which { none=0, sum, square, hole, rosenbrock, rastrigin } which;
  arr condition;
  _ChoiceFunction():which(none) {}

  double f(arr& g, arr& H, const arr& x) {
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
      case sum: f = _SumFunction().f(g, H, y); break;
      case square: f = _SquareFunction().f(g, H, y); break;
      case hole: f = _HoleFunction().f(g, H, y); break;
      case rosenbrock: f = NLP_Rosenbrock(y.N).f(g, H, y); break;
      case rastrigin: f = NLP_Rastrigin(y.N).f(g, H, y); break;
      default: NIY;
    }
    if(!!g) g = ~C * g;
    if(!!H) H = ~C * H * C;
    return f;
  }

  //  ScalarFunction get_f(){
  //    return [this](arr& g, arr& H, const arr& x) -> double { return this->fs(g, H, x); };
  //  }

};

//===========================================================================

void generateConditionedRandomProjection(arr& M, uint n, double condition) {
}

//===========================================================================

NLP_Squared::NLP_Squared(uint dim, double condition, bool random) {
  dimension = dim;
  bounds = (consts(-2., dim), consts(2., dim)).reshape(2,dim);
  featureTypes = rai::consts<ObjectiveType>(OT_sos, dim);

  x0 = zeros(dim);

  //let C be a ortho-normal matrix (=random rotation matrix)
  C.resize(dim, dim);

  if(random) {
    rndUniform(C, -1., 1., false);
    //orthogonalize
    for(uint i=0; i<dim; i++) {
      for(uint j=0; j<i; j++) C[i] -= scalarProduct(C[i], C[j])*C[j];
      C[i] /= length(C[i]);
    }
    //we condition each column of M with powers of the condition
    for(uint i=1; i<dim; i++) C[i] *= pow(condition, double(i) / (2.*double(dim - 1)));

    x0 = .5 * rand({dim}, bounds);

  } else {
    arr cond(dim);
    if(dim>1) {
      for(uint i=1; i<dim; i++) cond(i) = pow(condition, 0.5*i/(dim-1));
    } else {
      cond = 1.;
    }

    C = diag(cond);
//    C(0,1) = C(0,0);
//    C(1,0) = -C(1,1);
  }
}

//===========================================================================

NLP_Rugged::NLP_Rugged(uint dim, bool sos, uint num_points, int num_features){
  dimension = dim;
  featureTypes = rai::consts((sos?OT_sos:OT_f), num_features);
  bounds = (consts(-1., dimension), consts(1., dimension)). reshape(2,-1);

  pts = rand({num_points, dimension}, bounds);
  Phi = randn({num_points, num_features});
  ann = make_shared<ANN>();
  ann->setX(pts);
}

void NLP_Rugged::evaluate(arr& phi, arr& J, const arr& x){
  arr sqrDists;
  uintA idx;
  ann->getkNN(sqrDists, idx, x, 2);

  arr phi0 = Phi[idx(0)];
  arr phi1 = Phi[idx(1)];
  arr d = ::sqrt(sqrDists);
  d /= sum(d)+1e-10;

  phi = d(1) * phi0 + d(0) * phi1;
}

//===========================================================================

BoxNLP::BoxNLP(){
  dimension = rai::getParameter<uint>("problem/dim", 2);
  featureTypes.resize(2*dimension);
  featureTypes = OT_ineq;
  bounds.resize(2, dimension);
  bounds[0] = -2.;
  bounds[1] = +2.;
  if(rai::getParameter<bool>("problem/costs", false)){
    featureTypes.append(rai::consts<ObjectiveType>(OT_sos,dimension));
  }
}

void BoxNLP::evaluate(arr& phi, arr& J, const arr& x){
  phi.resize(2*dimension);
  phi({0,dimension}) = -(x + 1.);
  phi({dimension,0}) = x - 1.;

  J.resize(phi.N, x.N).setZero();
  for(uint i=0;i<dimension;i++){
    J(i,i) = -1.;
    J(dimension+i,i) = 1.;
  }

  if(featureTypes.N>2*dimension){
    arr d = x;
    //    if(d(0)>0.) d(0) -= 1.; else d(0) += 1.;
    d -= ones(d.N);
    double w = 2.;
    phi.append(w*d);
    J.append(w*eye(dimension));
  }
}

//===========================================================================

ModesNLP::ModesNLP(){
  dimension = rai::getParameter<uint>("problem/dim", 2);
  uint k = rai::getParameter<uint>("problem/modes", 5);
  cen = randn(k, dimension);
  cen.reshape(k, dimension);
  radii = .2 +10*rand(k);

#if 1
  k = 1+(uint(1)<<dimension);
  cen = randn(k, dimension);
  radii = consts(.1, k);
  cen[-1] = 0.;
  radii(-1) = .5;
  for(uint i=0;i<k-1;i++){
    for(uint d=0;d<dimension;d++){
      cen(i,d) = (i&(1<<d))? -1.: 1.;
    }
  }
#endif

  cen.reshape(-1, dimension);
  featureTypes.resize(1);
  featureTypes = OT_ineq;
  bounds.resize(2,dimension);
  bounds[0] = -1.2;
  bounds[1] = +1.2;
}

void ModesNLP::evaluate(arr& phi, arr& J, const arr& x){
  arr _x = x;
  _x.J_setId();

  uint k = cen.d0;
  arrA y(k);
  arr yval(k);
  for(uint i=0;i<k;i++){
    arr d = cen[i]-_x;
    double s = 1./(radii(i)*radii(i));
    y(i) = s * ~d*d - 1.;
    yval(i) = y(i).scalar();
  }

  phi = y(argmin(yval));
  J = phi.J_reset();
}

//===========================================================================

ChoiceConstraintFunction::ChoiceConstraintFunction() {
  which = (WhichConstraint) rai::getParameter<double>("constraintChoice");
  n = rai::getParameter<uint>("dim", 2);

  f_uc = _ChoiceFunction();

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

  phi.append(f_uc(J, NoArr, x));

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
      if(!!J) J.append(randomG.sub({0,0},{ 1,0}));
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
  f_uc(NoArr, H, x);
}

std::shared_ptr<NLP> getBenchmarkFromCfg() {
  rai::Enum<BenchmarkSymbol> bs(rai::getParameter<rai::String>("benchmark"));
  uint dim = rai::getParameter<uint>("benchmark/dim", 2);
  // double forsyth = rai::getParameter<double>("benchmark/forsyth", -1.);
  double condition = rai::getParameter<double>("benchmark/condition", 10.);

  //-- unconstrained problems

  {
    shared_ptr<NLP> nlp;

    if(bs==BS_Rosenbrock) nlp = make_shared<NLP_Rosenbrock>(dim);
    else if(bs==BS_Rastrigin) nlp = make_shared<NLP_Rastrigin>(dim);
    else if(bs==BS_Square) nlp = make_shared<NLP_Squared>(dim, condition, false);
    else if(bs==BS_RandomSquared) nlp = make_shared<NLP_Squared>(dim, condition, true);
    else if(bs==BS_RastriginSOS) nlp = make_shared<NLP_RastriginSOS>();

    if(nlp) {
      nlp->bounds = rai::getParameter<arr>("benchmark/bounds", {});
      nlp->bounds.reshape(2,-1);
      // if(forsyth>0.) F->forsythAlpha = forsyth;
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

  arr B = rai::getParameter<arr>("benchmark/bounds", {});
  if(B.N){
    if(B.nd==1 && B.N==2){
      B = repmat(B, 1, nlp->dimension);
    }
    nlp->bounds = B.reshape(2, nlp->dimension);
  }

  return nlp;
}

NLP_RastriginSOS::NLP_RastriginSOS() {
  a = rai::getParameter<double>("Rastrigin/a", 4.);
  condition = rai::getParameter<double>("benchmark/condition", 20.);

  dimension=2;
  featureTypes = rai::consts<ObjectiveType>(OT_sos, 4);

  cout <<bounds <<endl;
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

NLP_RandomLP::NLP_RandomLP(uint dim) {
  dimension = dim;

  bounds.resize(2, dimension);
  bounds[0]=-2.;
  bounds[1]=+2.;

  randomG.resize(5*dim+5, dim+1);
  rndGauss(randomG, 1.);
  for(uint i=0; i<randomG.d0; i++) {
    if(randomG(i, 0)>0.) randomG(i, 0) *= -1.; //ensure (0,0) is feasible
    randomG(i, 0) -= .2;
  }

  featureTypes = { OT_f };
  featureTypes.append(rai::consts(OT_ineq, randomG.d0));
}

void NLP_RandomLP::evaluate(arr& phi, arr& J, const arr& x) {
  phi = {sum(x)};
  if(!!J) J = ones(1, x.N);

  phi.append(randomG * (arr{1.}, x));
  if(!!J) J.append(randomG.sub({0,0},{ 1,0}));
}

