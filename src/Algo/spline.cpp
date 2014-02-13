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
  arr b_sum(T+1);
  tensorMarginal(b_sum, basis_trans, TUP(1));
  plotFunction(b_sum, -1, 1);
  for(uint i=0; i<=K; i++) plotFunction(basis_trans[i], -1, 1);
  plot();
}

void Spline::setBasis() {
  CHECK(times.N-1==K+1+degree, "wrong number of time knots");
  basis.resize(T+1, K+1);
  for(uint t=0; t<=T; t++) basis[t] = getBasis((double)t/T);
  transpose(basis_trans, basis);
}

arr Spline::getBasis(double time) const {
  arr b(K+1), b_0(K+1);
  for(uint p=0; p<=degree; p++) {
    if(p>0) b_0=b;
    for(uint k=0; k<=K; k++) {
      if(!p) {
        b(k) = 0.;
        if(times(k)<=time && time<times(k+1)) b(k)=1.;
        if(k==K && time>=times(k+1)) b(k)=1.;
      } else {
        double x=DIV(time-times(k), times(k+p)-times(k), true);
        b(k) = x * b_0(k);
        if(k<K) {
          double y=DIV(times(k+p+1)-time, times(k+p+1)-times(k+1), true);
          b(k) += y * b_0(k+1);
        }
      }
    }
  }
  return b;
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
  setBasis();
//  setBasisAndTimeGradient();
}

arr Spline::eval(double t) const { return (~getBasis(t)*points).reshape(points.d1); }

arr Spline::eval(uint t) const { return (~basis[t]*points).reshape(points.d1); };

arr Spline::eval() const { return basis*points; };

void Spline::partial(arr& grad_points, const arr& grad_path) const {
  CHECK(grad_path.d0==T+1 && grad_path.d1==points.d1, "");
  grad_points = basis_trans * grad_path;
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

} //namespace MT
