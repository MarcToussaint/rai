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

#include "spline.h"

//fwd declarations
void plot(bool wait=true, const char* txt=0);
void plotClear();
void plotFunction(const arr& f, double x0=0., double x1=0.);


//==============================================================================
//
// Spline
//


namespace MT{

void Spline::plotBasis() {
  plotClear();
  arr b_sum(basis.d0);
  tensorMarginal(b_sum, basis_trans, TUP(1));
  plotFunction(b_sum, -1, 1);
  for(uint i=0; i<points.d0; i++) plotFunction(basis_trans[i], -1, 1);
  plot();
}

void Spline::setBasis(uint T, uint K) {
  CHECK(times.N-1==K+1+degree, "wrong number of time knots");
  basis.resize(T+1, K+1);
  for(uint t=0; t<=T; t++) basis[t] = getCoeffs((double)t/T, K);
  transpose(basis_trans, basis);
}

// returns the spline coefficients: it is guaranteed that if t1 (or t2) is summed over a grid of stepsizes 1, this sums to 1
double Spline::getCoeff(double t, double t2, uint der) const{
  double dt=t-t2;
  if(der == 0) {
    if(degree==0){
      if(dt<-.5 || dt>=.5) return 0.;
      return 1.;
    }
    if(degree==1){
      if(dt<-1. || dt>=1.) return 0.;
      if(dt<0) return 1+dt;
      return 1.-dt;
    }
    if(degree==2){
      dt += 1.5;
      if(dt<=0. || dt>=3.) return 0.;
      double dt2=dt*dt;
      if(dt<=1.) return 0.5*dt2;
      if(dt<=2.) return 0.5*(-2.*dt2 + 6.*dt - 3.);
      if(dt<=3.) return 0.5*(    dt2 - 6.*dt + 9.);
      // from http://en.wikipedia.org/wiki/IrwinE2%80%93Hall_distribution#Special_cases
    }
    if(degree==3){
      dt += 2.;
      if(dt<=0. || dt>=4.) return 0.;
      double dt2=dt*dt;
      double dt3=dt2*dt;
      if(dt<=1.) return (dt3)/6.;
      if(dt<=2.) return (-3.*dt3 + 12.*dt2 - 12.*dt +  4.)/6.;
      if(dt<=3.) return ( 3.*dt3 - 24.*dt2 + 60.*dt - 44.)/6.;
      if(dt<=4.) return (   -dt3 + 12.*dt2 - 48.*dt + 64.)/6.;
      // from http://en.wikipedia.org/wiki/IrwinE2%80%93Hall_distribution#Special_cases
    }
  }else if(der == 1){
    if(degree==0) return 0.;
    if(degree==1){
      if(dt<-1. || dt>=1.) return 0.;
      if(dt<0) return 1.;
      return -1.;
    }
    if(degree==2){
      dt += 1.5;
      if(dt<=0. || dt>=3.) return 0.;
      if(dt<=1.) return dt;
      if(dt<=2.) return (-2.*dt + 3.);
      if(dt<=3.) return (    dt - 3.);
    }
    if(degree==3){
      dt += 2.;
      if(dt<=0. || dt>=4.) return 0.;
      double dt2=dt*dt;
      if(dt<=1.) return (3.*dt2)/6.;
      if(dt<=2.) return (-9.*dt2 + 24.*dt - 12.)/6.;
      if(dt<=3.) return ( 9.*dt2 - 48.*dt + 60.)/6.;
      if(dt<=4.) return (-3.*dt2 + 24.*dt - 48.)/6.;
      // from http://en.wikipedia.org/wiki/IrwinE2%80%93Hall_distribution#Special_cases
    }
  }
  HALT("nigher derivates or degrees not yet done");
  return 0.;
}

arr Spline::getCoeffs(double time, uint K, uint der) const {
  arr b(K+1), b_0(K+1), db(K+1), db_0(K+1), ddb(K+1), ddb_0(K+1);
  for(uint p=0; p<=degree; p++) {
    b_0=b; b.setZero();
    db_0=db; db.setZero();
    ddb_0=ddb; ddb.setZero();
    for(uint k=0; k<=K; k++) {
      if(!p) {
        if(times(k)<=time && time<times(k+1)) b(k)=1.;
        if(k==K && time>=times(k+1)) b(k)=1.;
      } else {
        double xnom = time - times(k);
        double xden = times(k+p) - times(k);
        double x = DIV(xnom, xden, true);
        b(k) = x * b_0(k);
        db(k) = DIV(1., xden, true) * b_0(k) + x * db_0(k);
        ddb(k) = DIV(2., xden, true) * db_0(k) + x * ddb_0(k);
        if(k<K) {
          double ynom = times(k+p+1) - time;
          double yden = times(k+p+1) - times(k+1);
          double y = DIV(ynom, yden, true);
          b(k) += y * b_0(k+1);
          db(k) += DIV(-1., yden, true) * b_0(k+1) + y * db_0(k+1);
          ddb(k) += DIV(-2., yden, true) * db_0(k+1) + y * ddb_0(k+1);
        }
      }
    }
  }
  switch(der) {
    case 0:
      return b;
    case 1:
      return db;
    case 2:
      return ddb;
  }
  HALT("Derivate of order " << der << " not yet implemented.");
}

void Spline::setBasisAndTimeGradient(uint T, uint K) {
  uint i, j, t, p, m=times.N-1;
  double time, x, xx, y, yy;
  CHECK(m==K+1+degree, "wrong number of time knots");
  arr b(K+1, T+1), b_0(K+1, T+1), dbt(m+1, K+1, T+1), dbt_0(m+1, K+1, T+1);
  for(p=0; p<=degree; p++) {
    if(p>0) { b_0=b; dbt_0=dbt; }
    for(i=0; i<=K; i++) for(t=0; t<=T; t++) {
        time = (double)t/(double)T;
        if(!p) {
          b(i, t) = 0.;
          if(times(i)<=time && time<times(i+1)) b(i, t)=1.;
          if(t==T && i==K && time==times(i+1)) b(i, t)=1.;
          for(j=0; j<=m; j++) dbt(j, i, t)=0.;
        } else {
          xx=times(i+p)-times(i);
          x=DIV(time-times(i), xx, true);
          if(i<K) {
            yy=times(i+p+1)-times(i+1);
            y=DIV(times(i+p+1)-time, yy, true);
          } else {
            yy=1.;
            y=0.;
          }
          b(i, t) = x * b_0(i, t);
          if(i<K) b(i, t) += y * b_0(i+1, t);
          for(j=0; j<=m; j++) {
            dbt(j, i, t) = x * dbt_0(j, i, t);
            if(i<K) dbt(j, i, t) += y * dbt_0(j, i+1, t);
            if(j==i)            dbt(j, i, t) += DIV((x-1), xx, true) * b_0(i, t);
            if(j==i+p)          dbt(j, i, t) -= DIV(x , xx, true) * b_0(i, t);
            if(i<K && j==i+1)   dbt(j, i, t) += DIV(y , yy, true) * b_0(i+1, t);
            if(i<K && j==i+p+1) dbt(j, i, t) -= DIV((y-1), yy, true) * b_0(i+1, t);
          }
        }
      }
  }
  basis_trans=b;
  transpose(basis, b);
  basis_timeGradient=dbt;
}

void Spline::setUniformNonperiodicBasis(uint T, uint K, uint _degree) {
  degree=_degree;
  uint i, m;
  m=K+1+degree;
  times.resize(m+1);
  for(i=0; i<=m; i++) {
    if(i<=degree) times(i)=.0;
    else if(i>=m-degree) times(i)=1.;
    else times(i) = double(i-degree)/double(m-1-degree);
  }
  setBasis(T, K);
//  setBasisAndTimeGradient();
}

arr Spline::eval(double t, bool velocities) const {
#if 0
  return (~getBasis(t-.1, velocities) * points).reshape(points.d1);
#else
  uint N=points.d0-1;
  t*=N;
  int K=floor(t);
  arr x(points.d1);
  x.setZero();
  for(int k=K-1-(int)degree/2; k<=K+1+(int)degree/2; k++){
    double a = getCoeff(t, (double)k, velocities);
    if(!a) continue;
    uint k_ref = (k>=0)?k:0;
    if(k_ref>N) k_ref=N;
    x+=a*points[k_ref];
  }
  if(velocities) x *= (double)N;
  return x;
#endif
}

arr Spline::eval(uint t) const { return (~basis[t]*points).reshape(points.d1); };

arr Spline::eval() const { return basis*points; };

arr Spline::smooth(double lambda) const {
  CHECK(lambda >= 0, "Lambda must be non-negative");
  uint T = basis.d0 - 1;
  uint K = basis.d1 - 1;
  arr ddbasis(T+1, K+1);
  for(uint t=0; t<=T; t++)
    ddbasis[t] = getCoeffs((double)t/K, K, 2);
  
  arr A = ~ddbasis * ddbasis / (double)T;
  return inverse(eye(K+1) + lambda*A)*points;
}

void Spline::partial(arr& grad_points, const arr& grad_path) const {
  CHECK(grad_path.d0==points.d0 && grad_path.d1==points.d1, "");
  grad_points = basis_trans * grad_path;
}

void Spline::partial(arr& dCdx, arr& dCdt, const arr& dCdf, bool constrain) const {
  uint K=points.d0-1, T=basis.d0-1;
  CHECK(dCdf.d0==T+1 && dCdf.d1==points.d1, "");
  CHECK(basis_timeGradient.N, "");
  uint n=dCdf.d1, m=K+1+degree, j;
  dCdx = basis_trans * dCdf;
  arr X;
  X.referTo(points);
  X.reshape((K+1)*n);
  arr B;
  B.referTo(basis_timeGradient);
  B.reshape((m+1)*(K+1), T+1);
  arr Z = B * dCdf; Z.reshape(m+1, (K+1)*n);
  dCdt = Z*X;
  if(constrain) {
    for(j=0; j<=degree; j++) dCdt(j)=0.;
    for(j=m-degree; j<=m; j++) dCdt(j)=0.;
  }
  dCdt(0)=dCdt(m)=0.;
}

//==============================================================================

arr Path::getPosition(double t) const{
  return Spline::eval(t);
}

arr Path::getVelocity(double t) const{
  return Spline::eval(t, true);
}

void Path::transform_CurrentBecomes_EndFixed(const arr& current, double t){
  arr delta = current - eval(t);
  for(uint i=0;i<points.d0;i++){
    double ti = double(i)/double(points.d0-1);
    double a = (1.-ti)/(1.-t);
    points[i]() += a*delta;
  }
}

void Path::transform_CurrentFixed_EndBecomes(const arr& end, double t){
  arr delta = end - eval(1.);
  for(uint i=0;i<points.d0;i++){
    double ti = double(i)/double(points.d0-1);
    double a = (ti-t)/(1.-t);
    points[i]() += a*delta;
  }
}

void Path::transform_CurrentBecomes_AllFollow(const arr& current, double t){
  arr delta = current - eval(t);
  for(uint i=0;i<points.d0;i++) points[i]() += delta;
}

} //namespace MT
