/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


/// @file
/// @ingroup group_Optim
/// @addtogroup group_Optim
/// @{

#ifndef MT_optimization_benchmarks_h
#define MT_optimization_benchmarks_h

#include "optimization.h"

//===========================================================================

struct RosenbrockFunction:ScalarFunction {
  virtual double fs(arr& g, arr& H, const arr& x) {
    if(&H) NIY;
    double f=0.;
    for(uint i=1; i<x.N; i++) f += MT::sqr(x(i)-MT::sqr(x(i-1))) + .01*MT::sqr(1-x(i-1));
    if(&g) NIY;
    return f;
  }
};

//===========================================================================

struct RastriginFunction:ScalarFunction {
  virtual double fs(arr& g, arr& H, const arr& x) {
    if(&H) NIY;
    double A=.5, f=A*x.N;
    for(uint i=0; i<x.N; i++) f += x(i)*x(i) - A*::cos(10.*x(i));
    if(&g) {
      g.resize(x.N);
      for(uint i=0; i<x.N; i++) g(i) = 2*x(i) + 10.*A*::sin(10.*x(i));
    }
    if(&H) {
      H.resize(x.N,x.N);  H.setZero();
      for(uint i=0; i<x.N; i++) H(i,i) = 2 + 100.*A*::cos(10.*x(i));
    }
    return f;
  }
};

//===========================================================================

struct SquareFunction:ScalarFunction {
  virtual double fs(arr& g, arr& H, const arr& x) {
    if(&g) g=2.*x;
    if(&H) H.setDiag(2., x.N);
    return sumOfSqr(x);
  }
};

//===========================================================================

struct HoleFunction:ScalarFunction {
  virtual double fs(arr& g, arr& H, const arr& x) {
    double f=exp(-sumOfSqr(x));
    if(&g) g=2.*f*x;
    if(&H) { H.setDiag(2.*f, x.N); H -= 4.*f*(x^x); }
    f = 1.-f;
    return f;
  }
};

//===========================================================================

struct ChoiceFunction:ScalarFunction {
  enum Which { none=0, sum, square, hole, rosenbrock, rastrigin } which;
  double condition;
  ChoiceFunction() {
    which = (Which) MT::getParameter<int>("fctChoice");
    condition = MT::getParameter<double>("condition");
  }
  virtual double fs(arr& g, arr& H, const arr& x) {
    double f;
    arr c(x.N), y(x);
    for(uint i=0; i<c.N; i++) c(i) = pow(condition,(double)i/(x.N-1));
    y *= c; //elem-wise product
    switch(which) {
      case sum: {
        f = ::sum(x);
        if(&g) { g.resize(x.N); g=1.; }
        if(&H) { H.resize(x.N,x.N); H.setZero(); }
      } break;
      case square: f = SquareFunction().fs(g, H, y); break;
      case hole: f = HoleFunction().fs(g, H, y); break;
      default: NIY;
    }
    if(&g) g*=c; //elem-wise product
    if(&H) { arr C; C.setDiag(c); H = C*H*C; }
    return f;
  }
};

//===========================================================================

struct ChoiceConstraintFunction:ConstrainedProblem {
  enum WhichConstraint { wedge2D=1, halfcircle2D, randomLinear } which;
  uint n;
  arr randomG;
  ChoiceFunction f;
  ChoiceConstraintFunction() {
    which = (WhichConstraint) MT::getParameter<int>("constraintChoice");
    n = MT::getParameter<uint>("dim", 2);
  }
  virtual double fc(arr& df, arr& Hf, arr& g, arr& Jg, const arr& x) {
    CHECK(x.N==n,"");
    double fx =  f.fs(df, Hf, x);

    if(&g) g.resize(dim_g());
    if(&Jg) { Jg.resize(g.N, x.N); Jg.setZero(); }
    switch(which) {
      case wedge2D:
        if(&g)  for(uint i=0;i<g.N;i++) g(i) = -sum(x)+1.5*x(i)-.1;
        if(&Jg){ Jg=-1.; for(uint i=0;i<g.N;i++) Jg(i,i) = +.5; }
        break;
      case halfcircle2D:
        if(&g) g(0) = sumOfSqr(x)-.25;     if(&Jg) Jg[0]() = 2.*x; //feasible=IN circle of radius .5
        if(&g) g(1) = -x(0)-.2;            if(&Jg) Jg(1,0) = -1.; //feasible=right of -.2
        break;
      case randomLinear:{
        uint n=x.N;
        if(!randomG.N){
          randomG.resize(dim_g(),n+1);
          rndGauss(randomG, 1.);
          for(uint i=0;i<randomG.d0;i++){
            if(randomG(i,n)>0.) randomG(i,n)*=-1.; //ensure (0,0) is feasible
            randomG(i,n) -= .2;
          }
        }
        if(&g) g = randomG * cat(x,ARRAY(1.));
        if(&Jg) Jg = randomG.sub(0,-1,0,-2);
      } break;
    }

    return fx;
  }
  virtual uint dim_x(){
    return n;
  }
  virtual uint dim_g(){
    if(which==randomLinear) return 5*n+2;
    if(which==wedge2D) return n;
    return 2;
  }
};

//===========================================================================

struct SimpleConstraintFunction:ConstrainedProblem {
  SimpleConstraintFunction() {
  }
  virtual double fc(arr& df, arr& Hf, arr& g, arr& Jg, const arr& x) {
    //simple squared potential, displaced by 1
    x(0) -= 1.;
    double f=sumOfSqr(x)-1.;
    if(&df) df=2.*x;
    if(&Hf) Hf.setDiag(2., x.N);
    x(0) += 1.;

    g.resize(2);
    if(&Jg) { Jg.resize(g.N, x.N); Jg.setZero(); }
    g(0) = .25-sumOfSqr(x);  if(&Jg) Jg[0]() = -2.*x; //OUTSIDE the circle
    g(1) = x(0);             if(&Jg) Jg(1,0) = 1.;

    return f;
  }
  virtual uint dim_x(){ return 2; }
  virtual uint dim_g(){ return 2; }
};

//===========================================================================

struct SinusesFunction:VectorFunction {
  double a;
  double condition;
  SinusesFunction() {
    a = MT::getParameter<double>("SinusesFunction_a");
    condition = MT::getParameter<double>("condition");
  }
  virtual void fv(arr& phi, arr& J, const arr& x) {
    CHECK(x.N==2,"");
    phi.resize(4);
    phi(0) = sin(a*x(0));
    phi(1) = sin(a*condition*x(1));
    phi(2) = 2.*x(0);
    phi(3) = 2.*condition*x(1);
    if(&J) {
      J.resize(4,2);
      J.setZero();
      J(0,0) = cos(a*x(0))*a;
      J(1,1) = cos(a*condition*x(1))*a*condition;
      J(2,0) = 2.;
      J(3,1) = 2.*condition;
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
  
  void fv(arr& y, arr& J,const arr& x);
};

//===========================================================================

/// Same as SquaredCost but $x_i \gets atan(x_i)$ before evaluating the squared cost
struct NonlinearlyWarpedSquaredCost : VectorFunction {
  uint n;  /// dimensionality of $x$
  SquaredCost sq;
  
  NonlinearlyWarpedSquaredCost(uint n, double condition=100.);
  void initRandom(uint n, double condition=100.);
  
  void fv(arr& y, arr& J,const arr& x);
};

//===========================================================================

struct ParticleAroundWalls:KOrderMarkovFunction {
  uint k;
  bool useKernel, constrained;
  void phi_t(arr& phi, arr& J, uint t, const arr& x_bar);

  uint get_T(){ return 1000; }
  uint get_k(){ return k; }
  uint dim_x(){ return 3; }
  uint dim_phi(uint t);
  uint dim_g(uint t);

  bool isConstrained(){ return constrained; }
  bool hasKernel(){ return useKernel; }
  double kernel(uint t0, uint t1){
    //if(t0==t1) return 1e3;
    return 1e0*::exp(-.001*MT::sqr((double)t0-t1));
  }
};

//===========================================================================

#endif
/// @}
