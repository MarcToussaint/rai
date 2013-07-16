/*  ---------------------------------------------------------------------
    Copyright 2013 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de

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

#include "algos.h"

namespace MT{

void rk4(arr& x1, const arr& x0,
             void (*df)(arr& xd, const arr& x),
             double dt) {
  uint n=x0.N;
  arr k1(n), k2(n), k3(n), k4(n);
  
  df(k1, x0);
  df(k2, x0+(double).5*dt*k1);
  df(k3, x0+(double).5*dt*k2);
  df(k4, x0+   dt*k3);
  
  x1 = x0;
  x1 += (dt/(double)6.)*(k1 + (double)2.*k2 + (double)2.*k3 + k4);
}

void (*global_ddf)(arr& xdd, const arr& x, const arr& v);
void (*global_sf)(arr& s,  const arr& x, const arr& v);
void rk_df(arr& xd, const arr& x) {
  uint n=x.N/2;
  arr X; X.referTo(x);
  X.reshape(2, n);
  arr a;
  global_ddf(a, X[0], X[1]);
  xd.resize(x.N);
  xd.setVectorBlock(X[1], 0);
  xd.setVectorBlock(a, n);
}
void rk_sf(arr& s, const arr& x) {
  uint n=x.N/2;
  arr X; X.referTo(x);
  X.reshape(2, n);
  global_sf(s, X[0], X[1]);
}

void rk4dd(arr& x1, arr& v1, const arr& x0, const arr& v0,
               void (*ddf)(arr& xdd, const arr& x, const arr& v),
               double dt) {
               
  global_ddf = ddf;
  
  uint n=x0.N;
  
  arr X(2, n), Y(2*n);
  X[0]=x0;
  X[1]=v0;
  X.reshape(2*n);
  
  rk4(Y, X, rk_df, dt);
  
  Y.reshape(2, n);
  x1=Y[0];
  v1=Y[1];
}


bool rk4_switch(arr& x1, arr& s1, const arr& x0, const arr& s0,
                    void (*df)(arr& xd, const arr& x),
                    void (*sf)(arr& s, const arr& x),
                    double& dt, double tol) {
  uint i, sn;
  arr sa=s0, sb, sm, xa=x0, xb, xm; //states at times a, m, t
  rk4(xb, x0, df, dt);
  sf(sb, xb);
  //CHECK(sa.N==sb.N, "inconsistent state indicators");
  bool change=false;
  sn=sa.N<sb.N?sa.N:sb.N;
  for(i=0; i<sn; i++) if(s0(i)*sb(i)<0.) {
      change=true;
      break;
    }
  if(!change) { x1=xb; s1=sb; return false; } //no problems: no switch
  
  //we have a switch - so we must find it precisely!
  double a=0., b=dt; //time interval [a, b]
  double m, min_m;   //where to cut the interval (determined by linear interpolation)
  
  cout <<"entering zero-crossing detection loop" <<endl;
  for(; fabs(b-a)>tol;) {
    //compute new m
    min_m=m=b;
    sn=sa.N<sb.N?sa.N:sb.N;
    for(i=0; i<sn; i++) if(sa(i)*sb(i)<0.) {
        m = b - sb(i) * (b-a)/(sb(i)-sa(i));
        if(m<min_m) min_m=m;
      }
    min_m=m;
    if(m-a<.1*tol) m+=.1*tol; //really close already
    if(b-m<.1*tol) m-=.1*tol; //really close already
    rk4(xm, x0, df, m);
    sf(sm, xm);
    change=false;
    sn=s0.N<sm.N?s0.N:sm.N;
    for(i=0; i<sn; i++) if(s0(i)*sm(i)<0.) { change=true; break; }
    
    //cout <<"a=" <<a <<" b=" <<b <<" m=" <<m <<" sa=" <<sa <<" sb=" <<sb <<" sm=" <<sm <<endl;
    cout <<" sm=" <<sm <<endl;
    if(!change) {
      a=m;
      sa=sm;
      xa=xm;
    } else {
      b=m;
      sb=sm;
      xb=xm;
    }
  }
  
  //take right limit of time interval
  dt=b;
  x1=xb;
  s1=sb;
  cout <<"DONE" <<endl <<"dt=" <<dt <<" s1=" <<s1 <<endl;
  return true;
}

bool rk4dd_switch(arr& x1, arr& v1, arr& s1, const arr& x0, const arr& v0, const arr& s0,
                      void (*ddf)(arr& xdd, const arr& x, const arr& v),
                      void (*sf)(arr& s, const arr& x, const arr& v),
                      double& dt, double tol) {
                      
  global_ddf = ddf;
  global_sf  = sf;
  
  uint n=x0.N;
  
  arr X(2, n), Y(2*n);
  X[0]=x0;
  X[1]=v0;
  X.reshape(2*n);
  
  bool change=rk4_switch(Y, s1, X, s0, rk_df, rk_sf, dt, tol);
  
  Y.reshape(2, n);
  x1=Y[0];
  v1=Y[1];
  return change;
}


//==============================================================================
//
// Spline
//

void Spline::plotBasis() {
#ifdef MT_plot_h
  plotClear();
  arr b_sum(T+1);
  tensorMarginal(b_sum, basis_trans, TUP(1));
  plotFunction(b_sum, -1, 1);
  for(uint i=0; i<=K; i++) plotFunction(basis_trans[i], -1, 1);
  plot();
#else
  NIY;
#endif
}

void Spline::setBasis() {
  uint i, t, p;
  double time, x, y;
  CHECK(times.N-1==K+1+degree, "wrong number of time knots");
  arr b(K+1, T+1), b_0(K+1, T+1);
  for(p=0; p<=degree; p++) {
    if(p>0) b_0=b;
    for(i=0; i<=K; i++) for(t=0; t<=T; t++) {
        time = (double)t/(double)T;
        if(!p) {
          b(i, t) = 0.;
          if(times(i)<=time && time<times(i+1)) b(i, t)=1.;
          if(t==T && i==K && time==times(i+1)) b(i, t)=1.;
        } else {
          x=DIV(time-times(i), times(i+p)-times(i), true);
          b(i, t) = x * b_0(i, t);
          if(i<K) {
            y=DIV(times(i+p+1)-time, times(i+p+1)-times(i+1), true);
            b(i, t) += y * b_0(i+1, t);
          }
        }
      }
  }
  basis_trans=b;
  transpose(basis, b);
}

void Spline::setBasisAndTimeGradient() {
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

void Spline::setUniformNonperiodicBasis(uint _T, uint _K, uint _degree) {
  T=_T; K=_K; degree=_degree;
  uint i, m;
  m=K+1+degree;
  times.resize(m+1);
  for(i=0; i<=m; i++) {
    if(i<=degree) times(i)=.0;
    else if(i>=m-degree) times(i)=1.;
    else times(i) = double(i-degree)/double(m-2*degree);
  }
  //setBasis(T, K, degree);
  setBasisAndTimeGradient();
}

void Spline::eval(arr& f_t, uint t) const { f_t = basis[t]*points; };

void Spline::eval(arr& f) const { f = basis*points; };

void Spline::partial(arr& dCdx, const arr& dCdf) const {
  CHECK(dCdf.d0==T+1 && dCdf.d1==points.d1, "");
  dCdx = basis_trans * dCdf;
}

void Spline::partial(arr& dCdx, arr& dCdt, const arr& dCdf, bool constrain) const {
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

} //end namespace
