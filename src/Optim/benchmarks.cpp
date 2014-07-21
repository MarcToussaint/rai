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

#include "benchmarks.h"
//#include "functions.h"

//===========================================================================

struct _RosenbrockFunction:ScalarFunction {
  virtual double fs(arr& g, arr& H, const arr& x) {
    double f=0.;
    for(uint i=1; i<x.N; i++) f += MT::sqr(x(i)-MT::sqr(x(i-1))) + .01*MT::sqr(1-10.*x(i-1));
    f = ::log(1.+f);
    if(&g) NIY;
    if(&H) NIY;
    return f;
  }
} rosen;
ScalarFunction& RosenbrockFunction=rosen;

//===========================================================================

struct _RastriginFunction:ScalarFunction {
  virtual double fs(arr& g, arr& H, const arr& x) {
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
} rastrigin;
ScalarFunction& RastriginFunction=rastrigin;

//===========================================================================

struct _SquareFunction:ScalarFunction {
  virtual double fs(arr& g, arr& H, const arr& x) {
    if(&g) g=2.*x;
    if(&H) H.setDiag(2., x.N);
    return sumOfSqr(x);
  }
} square;
ScalarFunction& SquareFunction=square;

//===========================================================================

struct _SumFunction:ScalarFunction {
  virtual double fs(arr& g, arr& H, const arr& x) {
    if(&g) { g.resize(x.N); g=1.; }
    if(&H) { H.resize(x.N,x.N); H.setZero(); }
    return sum(x);
  }
} sumf;
ScalarFunction& SumFunction=sumf;

//===========================================================================

struct _HoleFunction:ScalarFunction {
  virtual double fs(arr& g, arr& H, const arr& x) {
    double f=exp(-sumOfSqr(x));
    if(&g) g=2.*f*x;
    if(&H) { H.setDiag(2.*f, x.N); H -= 4.*f*(x^x); }
    f = 1.-f;
    return f;
  }
} hole;
ScalarFunction& HoleFunction=hole;

//===========================================================================

struct _ChoiceFunction:ScalarFunction {
  enum Which { none=0, sum, square, hole, rosenbrock, rastrigin } which;
  arr condition;
  virtual double fs(arr& g, arr& H, const arr& x) {
    //initialize on first call
    if(which==none){
      which = (Which) MT::getParameter<int>("fctChoice");
    }
    if(!condition.N!=x.N){
      condition.resize(x.N);
      double cond = MT::getParameter<double>("condition");
      for(uint i=0; i<x.N; i++) condition(i) = pow(cond,0.5*i/(x.N-1));
    }

    arr y = x;
    y *= condition; //elem-wise product
    double f;
    switch(which) {
      case sum: f = SumFunction.fs(g, H, y); break;
      case square: f = SquareFunction.fs(g, H, y); break;
      case hole: f = HoleFunction.fs(g, H, y); break;
      case rosenbrock: f = RosenbrockFunction.fs(g, H, y); break;
      case rastrigin: f = RastriginFunction.fs(g, H, y); break;
      default: NIY;
    }
    if(&g) g *= condition; //elem-wise product
    if(&H) H = condition%H%condition;
    return f;
  }
} choice;
ScalarFunction& ChoiceFunction=choice;

//===========================================================================

void generateConditionedRandomProjection(arr& M, uint n, double condition) {
  uint i,j;
  //let M be a ortho-normal matrix (=random rotation matrix)
  M.resize(n,n);
  rndUniform(M,-1.,1.,false);
  //orthogonalize
  for(i=0; i<n; i++) {
    for(j=0; j<i; j++) M[i]()-=scalarProduct(M[i],M[j])*M[j];
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

void SquaredCost::fv(arr& y, arr& J,const arr& x) {
  CHECK(x.N==n,"");
  y = M*x;
  if(&J) J=M;
}

//===========================================================================

NonlinearlyWarpedSquaredCost::NonlinearlyWarpedSquaredCost(uint _n, double condition):sq(_n,condition) {
  n=_n;
}

void NonlinearlyWarpedSquaredCost::initRandom(uint _n, double condition) {
  n=_n;
  sq.initRandom(n,condition);
}

void NonlinearlyWarpedSquaredCost::fv(arr& y, arr& J,const arr& x) {
  CHECK(x.N==n,"");
  arr xx=atan(x);
  y=sq.M*xx;
  if(&J) {
    arr gg(xx.N);
    for(uint i=0; i<gg.N; i++) gg(i) = 1./(1.+x(i)*x(i));
    J = sq.M*diag(gg);
  }
}

//===========================================================================

uint ParticleAroundWalls::dim_phi(uint t){
  uint T=get_T();
  if(t==T/2 || t==T/4 || t==3*T/4 || t==T) return 2*dim_x();
  return dim_x();
}

uint ParticleAroundWalls::dim_g(uint t){
  if(!hardConstrained) return 0;
  uint T=get_T();
  if(t==T/2 || t==T/4 || t==3*T/4 || t==T) return dim_x();
  return 0;
}

void ParticleAroundWalls::phi_t(arr& phi, arr& J, uint t, const arr& x_bar){
  uint T=get_T(), n=dim_x(), k=get_k();

  //assert some dimensions
  CHECK(x_bar.d0==k+1,"");
  CHECK(x_bar.d1==n,"");
  CHECK(t<=T,"");

  //-- transition costs: append to phi
  if(k==1)  phi = x_bar[1]-x_bar[0]; //penalize velocity
  if(k==2)  phi = x_bar[2]-2.*x_bar[1]+x_bar[0]; //penalize acceleration
  if(k==3)  phi = x_bar[3]-3.*x_bar[2]+3.*x_bar[1]-x_bar[0]; //penalize jerk

  //-- walls: append to phi
  //Note: here we append to phi ONLY in certain time slices: the dimensionality of phi may very with time slices; see dim_phi(uint t)
  double eps=.1, power=2.;
  if(!hardConstrained){
    //-- wall costs
    for(uint i=0;i<n;i++){ //add barrier costs to each dimension
      if(t==T/4)   phi.append(MT::ineqConstraintCost(i+1.-x_bar(k,i), eps, power));  //middle factor: ``greater than i''
      if(t==T/2)   phi.append(MT::ineqConstraintCost(x_bar(k,i)+i+1., eps, power));  //last factor: ``lower than -i''
      if(t==3*T/4) phi.append(MT::ineqConstraintCost(i+1.-x_bar(k,i), eps, power));  //middle factor: ``greater than i''
      if(t==T)     phi.append(MT::ineqConstraintCost(x_bar(k,i)+i+1., eps, power));  //last factor: ``lower than -i''
    }
  }else{
    //-- wall constraints
    for(uint i=0;i<n;i++){ //add barrier costs to each dimension
      if(t==T/4)   phi.append((i+1.-x_bar(k,i)));  //middle factor: ``greater than i''
      if(t==T/2)   phi.append((x_bar(k,i)+i+1.));  //last factor: ``lower than -i''
      if(t==3*T/4) phi.append((i+1.-x_bar(k,i)));  //middle factor: ``greater than i''
      if(t==T)     phi.append((x_bar(k,i)+i+1.));  //last factor: ``lower than -i''
    }
  }

  uint m=phi.N;
  CHECK(m==dim_phi(t),"");

  if(&J){ //we also need to return the Jacobian
    J.resize(m,k+1,n).setZero();

    //-- transition costs
    for(uint i=0;i<n;i++){
      if(k==1){ J(i,1,i) = 1.;  J(i,0,i) = -1.; }
      if(k==2){ J(i,2,i) = 1.;  J(i,1,i) = -2.;  J(i,0,i) = 1.; }
      if(k==3){ J(i,3,i) = 1.;  J(i,2,i) = -3.;  J(i,1,i) = +3.;  J(i,0,i) = -1.; }
    }

    //-- walls
    if(!hardConstrained){
      for(uint i=0;i<n;i++){
        if(t==T/4)   J(n+i,k,i) = -MT::d_ineqConstraintCost(i+1.-x_bar(k,i), eps, power);
        if(t==T/2)   J(n+i,k,i) =  MT::d_ineqConstraintCost(x_bar(k,i)+i+1., eps, power);
        if(t==3*T/4) J(n+i,k,i) = -MT::d_ineqConstraintCost(i+1.-x_bar(k,i), eps, power);
        if(t==T)     J(n+i,k,i) =  MT::d_ineqConstraintCost(x_bar(k,i)+i+1., eps, power);
      }
    }else{
      for(uint i=0;i<n;i++){
        if(t==T/4)   J(n+i,k,i) = -1.;
        if(t==T/2)   J(n+i,k,i) = +1.;
        if(t==3*T/4) J(n+i,k,i) = -1.;
        if(t==T)     J(n+i,k,i) = +1.;
      }
    }
  }
}

