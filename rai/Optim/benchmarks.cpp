/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "benchmarks.h"
//#include "functions.h"

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

ScalarFunction RosenbrockFunction() { return _RosenbrockFunction; }

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

ScalarFunction RastriginFunction() { return _RastriginFunction; }

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
  uint i, j;
  //let M be a ortho-normal matrix (=random rotation matrix)
  M.resize(n, n);
  rndUniform(M, -1., 1., false);
  //orthogonalize
  for(i=0; i<n; i++) {
    for(j=0; j<i; j++) M[i]()-=scalarProduct(M[i], M[j])*M[j];
    M[i]()/=length(M[i]);
  }
  //we condition each column of M with powers of the condition
  for(i=0; i<n; i++) M[i]() *= pow(condition, double(i) / (2.*double(n - 1)));
}

//===========================================================================

SquaredCost::SquaredCost(uint _n, double condition) {
  initRandom(_n, condition);
}

void SquaredCost::initRandom(uint _n, double condition) {
  n=_n;
  generateConditionedRandomProjection(M, n, condition);
  //the metric is equal M^T*M
  //C=~M*M;
  //arr U,d,V;    svd(U, d, V, C);    cout <<U <<d <<V <<M <<C <<endl;
}

void SquaredCost::fv(arr& y, arr& J, const arr& x) {
  CHECK_EQ(x.N, n, "");
  y = M*x;
  if(!!J) J=M;
}

//===========================================================================

NonlinearlyWarpedSquaredCost::NonlinearlyWarpedSquaredCost(uint _n, double condition):sq(_n, condition) {
  n=_n;
}

void NonlinearlyWarpedSquaredCost::initRandom(uint _n, double condition) {
  n=_n;
  sq.initRandom(n, condition);
}

void NonlinearlyWarpedSquaredCost::fv(arr& y, arr& J, const arr& x) {
  CHECK_EQ(x.N, n, "");
  arr xx=atan(x);
  y=sq.M*xx;
  if(!!J) {
    arr gg(xx.N);
    for(uint i=0; i<gg.N; i++) gg(i) = 1./(1.+x(i)*x(i));
    J = sq.M*diag(gg);
  }
}

//===========================================================================

void ParticleAroundWalls2::getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes) {
  variableDimensions = consts<uint>(n, T);

  if(!!featureTimes) featureTimes.clear();
  if(!!featureTypes) featureTypes.clear();
  for(uint t=0; t<T; t++) {
    if(!!featureTimes) featureTimes.append(consts<uint>(t, n));
    if(!!featureTypes) featureTypes.append(consts(OT_sos, n));
    if(t==T/4 || t==T/2 || t==3*T/4 || t==T-1) {
      if(!!featureTimes) featureTimes.append(consts<uint>(t, n));
      if(!!featureTypes) featureTypes.append(consts(OT_ineq, n));
    }
  }
}

void ParticleAroundWalls2::phi(arr& phi, arrA& J, arrA& H, uintA& featureTimes, ObjectiveTypeA& tt, const arr& x) {

  uint M=x.N + 4*3;
  phi.resize(M);
  if(!!J) J.resize(M);
  if(!!tt) tt.resize(M);

  uint m=0;
  for(uint t=0; t<T; t++) {
    //-- construct x_bar
    arr x_bar;
    if(t>=k) {
      x_bar.referToRange(x, t-k, t);
    } else { //x_bar includes the prefix
      x_bar.resize(k+1, n);
      for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i<0)? zeros(n) : x[i];
    }

    //-- assert some dimensions
    CHECK_EQ(x_bar.d0, k+1, "");
    CHECK_EQ(x_bar.d1, n, "");

    //-- transition costs
    for(uint i=0; i<n; i++) {
      if(k==1) {
        phi(m) = x_bar(1, i)-x_bar(0, i); //penalize velocity
        if(!!J) { J(m).resize(k+1, n).setZero(); J(m)(1, i) = 1.;  J(m)(0, i) = -1.; }
      }
      if(k==2) {
        phi(m) = x_bar(2, i)-2.*x_bar(1, i)+x_bar(0, i); //penalize acceleration
        if(!!J) { J(m).resize(k+1, n).setZero(); J(m)(2, i) = 1.;  J(m)(1, i) = -2.;  J(m)(0, i) = 1.; }
      }
      if(k==3) {
        phi(m) = x_bar(3, i)-3.*x_bar(2, i)+3.*x_bar(1, i)-x_bar(0, i); //penalize jerk
        if(!!J) { J(m).resize(k+1, n).setZero(); J(m)(3, i) = 1.;  J(m)(2, i) = -3.;  J(m)(1, i) = +3.;  J(m)(0, i) = -1.; }
      }
      if(!!J && t<k) J(m) = J(m).sub(k-t, -1, 0, -1); //cut the prefix Jacobians
      if(!!tt) tt(m) = OT_sos;
      m++;
    }

    //-- wall constraints
    if(t==T/4 || t==T/2 || t==3*T/4 || t==T-1) {
      for(uint i=0; i<n; i++) { //add barrier costs to each dimension
        if(t==T/4) {
          phi(m) = (i+1.-x_bar(k, i)); //``greater than i+1''
          if(!!J) { J(m).resize(k+1, n).setZero(); J(m)(k, i) = -1.; }
        }
        if(t==T/2) {
          phi(m) = (x_bar(k, i)+i+1.); //``lower than -i-1''
          if(!!J) { J(m).resize(k+1, n).setZero(); J(m)(k, i) = +1.; }
        }
        if(t==3*T/4) {
          phi(m) = (i+1.-x_bar(k, i)); //``greater than i+1''
          if(!!J) { J(m).resize(k+1, n).setZero(); J(m)(k, i) = -1.; }
        }
        if(t==T-1) {
          phi(m) = (x_bar(k, i)+i+1.); //``lower than -i-1''
          if(!!J) { J(m).resize(k+1, n).setZero(); J(m)(k, i) = +1.; }
        }
        if(!!tt) tt(m) = OT_ineq;
        m++;
      }
    }
  }
  CHECK_EQ(m, M, "");
}

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

