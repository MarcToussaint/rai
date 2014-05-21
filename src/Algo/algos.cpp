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


#include "algos.h"


namespace MT{

void rk4(arr& x1, const arr& x0,
         VectorFunction& f,
         double dt) {
  arr k1,k2,k3,k4;
  f.fv(k1, NoArr, x0);
  f.fv(k2, NoArr, x0 + 0.5*dt*k1);
  f.fv(k3, NoArr, x0 + 0.5*dt*k2);
  f.fv(k4, NoArr, x0 +     dt*k3);
  
  if(&x1!=&x0) x1 = x0;
  x1 += (dt/6.)*(k1 + 2.*k2 + 2.*k3 + k4);
}

void rk4_2ndOrder(arr& x, const arr& x0, VectorFunction& f, double dt){
  CHECK(x0.nd==2 && x0.d0==2,"need a 2-times-n array   rk4_2ndOrder input");
  struct F2:VectorFunction{
    VectorFunction& f;
    F2(VectorFunction& _f):f(_f){}
    void fv(arr& y, arr& J, const arr& x){
      CHECK(x.nd==2 && x.d0==2,"");
      y.resizeAs(x);
      y[0]=x[1];
      f.fv(y[1](), NoArr, x);
    }

  } f2(f);
  rk4(x, x0, f2, dt);
}

#if 0
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
#endif


} //end namespace
