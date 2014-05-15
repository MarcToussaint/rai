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
  if(!constrained) return 0;
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

  //-- transition costs
  if(k==1)  phi = x_bar[1]-x_bar[0]; //penalize velocity
  if(k==2)  phi = x_bar[2]-2.*x_bar[1]+x_bar[0]; //penalize acceleration
  if(k==3)  phi = x_bar[3]-3.*x_bar[2]+3.*x_bar[1]-x_bar[0]; //penalize jerk

  double eps=.1, power=2.;
  if(!constrained){
    //-- wall costs
    for(uint i=0;i<n;i++){ //add barrier costs to each dimension
      if(t==T/4)   phi.append(MT::barrier(i+1.-x_bar(k,i), eps, power));  //middle factor: ``greater than i''
      if(t==T/2)   phi.append(MT::barrier(x_bar(k,i)+i+1., eps, power));  //last factor: ``lower than -i''
      if(t==3*T/4) phi.append(MT::barrier(i+1.-x_bar(k,i), eps, power));  //middle factor: ``greater than i''
      if(t==T)     phi.append(MT::barrier(x_bar(k,i)+i+1., eps, power));  //last factor: ``lower than -i''
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

    //curvature
    for(uint i=0;i<n;i++){
      if(k==1){ J(i,1,i) = 1.;  J(i,0,i) = -1.; }
      if(k==2){ J(i,2,i) = 1.;  J(i,1,i) = -2.;  J(i,0,i) = 1.; }
      if(k==3){ J(i,3,i) = 1.;  J(i,2,i) = -3.;  J(i,1,i) = +3.;  J(i,0,i) = -1.; }
    }

    if(!constrained){
      for(uint i=0;i<n;i++){
        if(t==T/4)   J(n+i,k,i) = -MT::d_barrier(i+1.-x_bar(k,i), eps, power);
        if(t==T/2)   J(n+i,k,i) =  MT::d_barrier(x_bar(k,i)+i+1., eps, power);
        if(t==3*T/4) J(n+i,k,i) = -MT::d_barrier(i+1.-x_bar(k,i), eps, power);
        if(t==T)     J(n+i,k,i) =  MT::d_barrier(x_bar(k,i)+i+1., eps, power);
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

