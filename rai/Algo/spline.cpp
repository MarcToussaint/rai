/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "spline.h"
#include "../Plot/plot.h"

//==============================================================================
//
// Spline
//

namespace rai {

Spline::Spline(uint degree) : degree(degree) {}

Spline::Spline(uint T, const arr& X, uint degree) : points(X) {
  CHECK_EQ(points.nd, 2, "");
  setUniformNonperiodicBasis(T, points.d0, degree);
}

void Spline::clear() {
  points.clear();
  times.clear();
  basis.clear();
  basis_trans.clear();
  basis_timeGradient.clear();
}

void Spline::plotBasis(PlotModule& plt) {
  plt.Clear();
  arr b_sum(basis.d0);
  tensorMarginal(b_sum, basis_trans, TUP(1u));
  plt.Function(b_sum, -1, 1);
  for(uint i=0; i<points.d0; i++) plt.Function(basis_trans[i], -1, 1);
  plt.update();
}

arr Spline::getCoeffs(double t, uint K, uint derivative) const {
  arr b(K+1), b_0(K+1), db(K+1), db_0(K+1), ddb(K+1), ddb_0(K+1);
  for(uint p=0; p<=degree; p++) {
    b_0=b; b.setZero();
    if(derivative>0) { db_0=db; db.setZero(); }
    if(derivative>1) { ddb_0=ddb; ddb.setZero(); }
    for(uint k=0; k<=K; k++) {
      if(!p) {
        if(!k && t<times(0)) b(k)=1.;
        else if(k==K && t>=times(k)) b(k)=1.;
        else if(times(k)<=t && t<times(k+1)) b(k)=1.;
      } else {
        if(k+p<times.N) {
          double xnom = t - times(k);
          double xden = times(k+p) - times(k);
          double x = DIV(xnom, xden, true);
          b(k) = x * b_0(k);
          if(derivative>0) db(k) = DIV(1., xden, true) * b_0(k) + x * db_0(k);
          if(derivative>1) ddb(k) = DIV(2., xden, true) * db_0(k) + x * ddb_0(k);
        }
        if(k<K && k+p+1<times.N) {
          double ynom = times(k+p+1) - t;
          double yden = times(k+p+1) - times(k+1);
          double y = DIV(ynom, yden, true);
          b(k) += y * b_0(k+1);
          if(derivative>0) db(k) += DIV(-1., yden, true) * b_0(k+1) + y * db_0(k+1);
          if(derivative>1) ddb(k) += DIV(-2., yden, true) * db_0(k+1) + y * ddb_0(k+1);
        }
      }
    }
    if(t<times(0) || t>=times.last()) break;
  }
  switch(derivative) {
    case 0:
      return b;
    case 1:
      return db;
    case 2:
      return ddb;
  }
  HALT("Derivate of order " << derivative << " not yet implemented.");
}

void Spline::setBasis(uint T, uint K) {
//  CHECK_EQ(times.N-1,K+1+degree, "wrong number of time knots");
  basis.resize(T+1, K+1);
  for(uint t=0; t<=T; t++) basis[t] = getCoeffs((double)t/T, K);
  transpose(basis_trans, basis);
}

void Spline::setBasisAndTimeGradient(uint T, uint K) {
  uint i, j, t, p, m=times.N-1;
  double time, x, xx, y, yy;
  CHECK_EQ(m, K+1+degree, "wrong number of time knots");
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
            if(j==i+p)          dbt(j, i, t) -= DIV(x, xx, true) * b_0(i, t);
            if(i<K && j==i+1)   dbt(j, i, t) += DIV(y, yy, true) * b_0(i+1, t);
            if(i<K && j==i+p+1) dbt(j, i, t) -= DIV((y-1), yy, true) * b_0(i+1, t);
          }
        }
      }
  }
  basis_trans=b;
  transpose(basis, b);
  basis_timeGradient=dbt;
}

void Spline::setUniformNonperiodicBasis() {
  setUniformNonperiodicBasis(0, points.d0, degree);
}

void Spline::set(uint _degree, const arr& x, const arr& t) {
  CHECK_EQ(x.d0, t.N, "");
  degree = _degree;

  points = x;
  for(uint i=0; i<degree/2; i++) {
    points.prepend(x[0]);
    points.append(x[x.d0-1]);
  }

  uint m=t.N+2*degree;
  times.resize(m+1);
  for(uint i=0; i<=m; i++) {
    if(i<=degree) times(i)=.0;
    else if(i>=m-degree) times(i)=t.last();
    else if((degree%2)) {
      times(i) = t(i-degree);
    } else {
      times(i) = .5*(t(i-degree-1)+t(i-degree));
    }
  }
}

void Spline::setUniformNonperiodicBasis(uint T, uint nPoints, uint _degree) {
  degree=_degree;
  uint i, m;
  uint K=nPoints - 2*(degree/2) - 1;
  m=K+1+2*degree;
  times.resize(m+1);
  for(i=0; i<=m; i++) {
    if(i<=degree) times(i)=.0;
    else if(i>=m-degree) times(i)=1.;
    else if((degree%2)) {
      times(i) = double(i-degree)/double(K);
    } else {
      times(i) = double(double(i)-.5-degree)/double(K);
    }
  }
  if(T) setBasis(T, nPoints-1);
//  setBasisAndTimeGradient();
}

arr Spline::eval(double t, uint derivative) const {
  uint K = points.d0-1;
  arr coeffs = getCoeffs(t, K, derivative);
//  if(!derivative){
//      cout <<"t: " <<t <<" dot: " <<derivative <<" a: " <<coeffs <<endl;
//      cout <<"t: " <<t <<" dot: " <<derivative <<" a: " <<rai::getCoeffs(t, times({2,4}), 0) <<endl;
//  }
  return (~coeffs * points).reshape(points.d1);
}

arr Spline::eval(uint t) const { return (~basis[t]*points).reshape(points.d1); }

arr Spline::eval() const { return basis*points; }

arr Spline::smooth(double lambda) const {
  CHECK_GE(lambda,  0, "Lambda must be non-negative");
  uint T = basis.d0 - 1;
  uint K = basis.d1 - 1;
  arr ddbasis(T+1, K+1);
  for(uint t=0; t<=T; t++)
    ddbasis[t] = getCoeffs((double)t/K, K, 2);

  arr A = ~ddbasis * ddbasis / (double)T;
  return basis*inverse(eye(K+1) + lambda*A)*points;
}

void Spline::partial(arr& grad_points, const arr& grad_path) const {
  CHECK_EQ(grad_path.d1, points.d1, "");
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

arr Path::getPosition(double t) const {
  return Spline::eval(t);
}

arr Path::getVelocity(double t) const {
  return Spline::eval(t, 1);
}

void Path::transform_CurrentBecomes_EndFixed(const arr& current, double t) {
  arr delta = current - eval(t);
  for(uint i=0; i<points.d0; i++) {
    double ti = double(i)/double(points.d0-1);
    double a = (1.-ti)/(1.-t);
    points[i]() += a*delta;
  }
}

void Path::transform_CurrentFixed_EndBecomes(const arr& end, double t) {
  arr delta = end - eval(1.);
  for(uint i=0; i<points.d0; i++) {
    double ti = double(i)/double(points.d0-1);
    double a = (ti-t)/(1.-t);
    points[i]() += a*delta;
  }
}

void Path::transform_CurrentBecomes_AllFollow(const arr& current, double t) {
  arr delta = current - eval(t);
  for(uint i=0; i<points.d0; i++) points[i]() += delta;
}

} //namespace rai
