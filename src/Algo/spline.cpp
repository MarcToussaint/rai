/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "spline.h"
#include "../Core/util.h"

#include <math.h>

//==============================================================================
//
// Spline
//

namespace rai {

void BSpline::clear() {
  ctrlPoints.clear();
  knotTimes.clear();
}

arr BSpline::getCoeffs(double t, uint K, uint derivative) const {
  arr b(K+1), b_0(K+1), db(K+1), db_0(K+1), ddb(K+1), ddb_0(K+1);
  for(uint p=0; p<=degree; p++) {
    b_0=b; b.setZero();
    if(derivative>0) { db_0=db; db.setZero(); }
    if(derivative>1) { ddb_0=ddb; ddb.setZero(); }
    for(uint k=0; k<=K; k++) {
      if(!p) {
        if(!k && t<knotTimes(0)) b(k)=1.;
        else if(k==K && t>=knotTimes(k)) b(k)=1.;
        else if(knotTimes(k)<=t && t<knotTimes(k+1)) b(k)=1.;
      } else {
        if(k+p<knotTimes.N) {
          double xnom = t - knotTimes(k);
          double xden = knotTimes(k+p) - knotTimes(k);
          double x = DIV(xnom, xden, true);
          b(k) = x * b_0(k);
          if(derivative>0) db(k) = DIV(1., xden, true) * b_0(k) + x * db_0(k);
          if(derivative>1) ddb(k) = DIV(2., xden, true) * db_0(k) + x * ddb_0(k);
        }
        if(k<K && k+p+1<knotTimes.N) {
          double ynom = knotTimes(k+p+1) - t;
          double yden = knotTimes(k+p+1) - knotTimes(k+1);
          double y = DIV(ynom, yden, true);
          b(k) += y * b_0(k+1);
          if(derivative>0) db(k) += DIV(-1., yden, true) * b_0(k+1) + y * db_0(k+1);
          if(derivative>1) ddb(k) += DIV(-2., yden, true) * db_0(k+1) + y * ddb_0(k+1);
        }
      }
    }
    if(t<knotTimes(0) || t>=knotTimes.last()) break;
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

#define ZDIV(x,y) (y?x/y:0.)

arr BSplineCore::get(double t, uint degree, arr& J) {
  B.resize(knots.N-degree, degree+1).setZero();
//  CHECK_LE(knots(0), t, "");
//  CHECK_LE(t, knots(-1), "");

  if(!!J) J.resize(B.d0, B.d1, knots.N).setZero();

  //initialize rank 0
  for(uint k=0; k<B.d0; k++) {
    if(knots(k)<=t && t<knots(k+1)) B(k, 0)=1.;
  }

  //recursion
  for(uint p=1; p<=degree; p++) {
    for(uint i=0; i<B.d0; i++) {
      if(i+p<knots.N) {
        double xnom = t - knots(i);
        double xden = knots(i+p) - knots(i);
        if(xnom!=0. && xden!=0.) {
          double x = DIV(xnom, xden, true);
          B(i, p) = x * B(i, p-1);
          if(!!J) {
            J(i, p, i) += (-1./xnom + 1./xden) * x * B(i, p-1);
            J(i, p, i+p) += (-1./xden) * x * B(i, p-1);
//            J(i,p,{}) += x * J(i, p-1, {});
            for(double* a=&J(i, p, 0), *b=&J(i, p-1, 0), *stop=a+J.d2; a!=stop; a++, b++) *a += x * (*b);
          }
        }
      }
      if(i+1<knots.N && i+p+1<knots.N && i+1<B.d0) {
        double ynom = knots(i+p+1) - t;
        double yden = knots(i+p+1) - knots(i+1);
        if(ynom!=0. && yden!=0.) {
          double y = DIV(ynom, yden, true);
          B(i, p) += y * B(i+1, p-1);
          if(!!J) {
            J(i, p, i+1) += (1./yden) * y * B(i+1, p-1);
            J(i, p, i+p+1) += (1./ynom - 1./yden) * y * B(i+1, p-1);
//            J(i,p,{}) += y * J(i+1, p-1, {});
            for(double* a=&J(i, p, 0), *b=&J(i+1, p-1, 0), *stop=a+J.d2; a!=stop; a++, b++) *a += y * (*b);
          }
        }
      }
    }
  }
  return B;
}

void BSpline::getCoeffs2(arr& b, arr& db, arr& ddb, double t, uint degree, double* knotTimes, uint knotN, uint knotTimesN, uint derivatives) {
  CHECK_EQ(knotN+degree+1, knotTimesN, "");

  b.resize(knotN).setZero();
  if(derivatives>0) db.resize(knotN).setZero();
  if(derivatives>1) ddb.resize(knotN).setZero();

  arr b_prev, db_prev, ddb_prev;
  for(uint p=0; p<=degree; p++) {
    b_prev=b; b.setZero();
    if(derivatives>0) { db_prev=db; db.setZero(); }
    if(derivatives>1) { ddb_prev=ddb; ddb.setZero(); }
    for(uint k=0; k<knotN; k++) {
      if(!p) {
        if(!k && t<knotTimes[0]) b.elem(k)=1.;
        else if(k==knotN-1 && t>=knotTimes[k]) b.elem(k)=1.;
        else if(knotTimes[k]<=t && t<knotTimes[k+1]) b.elem(k)=1.;
      } else {
        if(k+p<knotTimesN) {
          double xnom = t - knotTimes[k];
          double xden = knotTimes[k+p] - knotTimes[k];
          double x = ZDIV(xnom, xden);
          b.elem(k) = x * b_prev.elem(k);
          if(derivatives>0) db.elem(k) = ZDIV(1., xden) * b_prev.elem(k) + x * db_prev.elem(k);
          if(derivatives>1) ddb.elem(k) = ZDIV(2., xden) * db_prev.elem(k) + x * ddb_prev.elem(k);
        }
        if(k<knotN-1 && k+p+1<knotTimesN) {
          double ynom = knotTimes[k+p+1] - t;
          double yden = knotTimes[k+p+1] - knotTimes[k+1];
          double y = ZDIV(ynom, yden);
          b.elem(k) += y * b_prev.elem(k+1);
          if(derivatives>0) db.elem(k) += ZDIV(-1., yden) * b_prev.elem(k+1) + y * db_prev.elem(k+1);
          if(derivatives>1) ddb.elem(k) += ZDIV(-2., yden) * db_prev.elem(k+1) + y * ddb_prev.elem(k+1);
        }
      }
    }
    if(t<knotTimes[0] || t>=knotTimes[knotTimesN-1]) break;
  }
}

void BSpline::eval(arr& x, arr& xDot, arr& xDDot, double t) const {
#if 0 //computing coeffs for ALL knot points (most zero...)
//  uint K = knotPoints.d0-1;
  arr coeffs = getCoeffs(t, knotPoints.d0-1, derivative);
//  arr coeffs = getCoeffs2(t, degree, knotTimes.p, knotPoints.d0, knotTimes.N, derivative);
  return (~coeffs * knotPoints).reshape(knotPoints.d1);
#else //pick out only the LOCAL knot points

  //find the first knotTime >t
  int offset = knotTimes.rankInSorted(t, rai::lowerEqual<double>, true);
  offset -= degree+1;
  if(offset<0) offset=0;

  uint knotN = degree+1;
  uint knotTimesN = knotN + 1+degree;
  if(offset+knotTimesN>knotTimes.N) offset = knotTimes.N - knotTimesN;

  //get coeffs
  arr b, db, ddb;
  uint derivative=0;
  if(!!xDot) derivative=1;
  if(!!xDDot) derivative=2;
  getCoeffs2(b, db, ddb, t, degree, knotTimes.p+offset, knotN, knotTimesN, derivative);

  //linear combination
  uint n = ctrlPoints.d1;
  if(!!x) x.resize(n).setZero();
  if(!!xDot) xDot.resize(n).setZero();
  if(!!xDDot) xDDot.resize(n).setZero();
  for(uint j=0; j<b.N; j++) {
    if(!!x) for(uint k=0; k<n; k++) x.elem(k) += b.elem(j)*ctrlPoints(offset+j, k);
    if(!!xDot) for(uint k=0; k<n; k++) xDot.elem(k) += db.elem(j)*ctrlPoints(offset+j, k);
    if(!!xDDot) for(uint k=0; k<n; k++) xDDot.elem(k) += ddb.elem(j)*ctrlPoints(offset+j, k);
  }
#endif
}

arr BSpline::eval(double t, uint derivative) const {
  arr x;
  if(derivative==0) eval(x, NoArr, NoArr, t);
  else if(derivative==1) eval(NoArr, x, NoArr, t);
  else if(derivative==2) eval(NoArr, NoArr, x, t);
  else NIY;
  return x;
}

arr BSpline::eval2(double t, uint derivative, arr& Jpoints, arr& Jtimes) const {
  if(!!Jpoints) Jpoints.resize(ctrlPoints.d1, ctrlPoints.d0, ctrlPoints.d1).setZero();
  if(!!Jtimes) Jtimes.resize(ctrlPoints.d1, knotTimes.N).setZero();

  if(t<knotTimes(0)) return ctrlPoints[0];
  if(t>=knotTimes(-1)) {
    if(!!Jpoints) for(uint i=0; i<ctrlPoints.d1; i++) Jpoints(i, -1, i) = 1.;
    return ctrlPoints[-1];
  }

  int center = knotTimes.rankInSorted(t, rai::lowerEqual<double>, true);
  center -= 1;
  int lo = center - degree;  if(lo<0) lo=0;
  int up = center + degree;  if(up>(int)knotTimes.N-1) up=knotTimes.N-1;

  arr Jtmp, Jtmp2;

  BSplineCore core;
  core.knots.referToRange(knotTimes, lo, up);
  core.get(t, degree, (!Jtimes?NoArr:Jtmp));

  arr x(ctrlPoints.d1);
  x.setZero();
  for(uint j=0; j<core.B.d0; j++) {
    double b = core.B(j, degree);
    if(lo+j>=ctrlPoints.d0) { CHECK_ZERO(b, 1e-4, ""); continue; }
    x += b * ctrlPoints[lo+j];
    if(!!Jpoints) {
      for(uint i=0; i<ctrlPoints.d1; i++) Jpoints(i, lo+j, i) = b;
    }
    if(!!Jtimes) {
      Jtmp2.resize(ctrlPoints.d1, knotTimes.N).setZero();
      Jtmp2.setMatrixBlock(ctrlPoints[lo+j] * ~Jtmp(j, degree, {}), 0, lo);
      Jtimes += Jtmp2;
    }
  }
  return x;
}

arr BSpline::eval(const arr& ts) {
  arr f(ts.N, ctrlPoints.d1);
  for(uint i=0; i<ts.N; i++) f[i] = eval(ts(i));
  return f;
}

arr BSpline::jac_point(double t, uint derivative) const {
  NIY;
  return {};
}

BSpline& BSpline::set(uint _degree, const arr& _points, const arr& _times, const arr& startVel, const arr& endVel) {
  CHECK_EQ(_times.nd, 1, "");
  CHECK_EQ(_points.nd, 2, "");
  CHECK_EQ(_points.d0, _times.N, "");

  degree = _degree;

  //knot points with head and tail
  ctrlPoints = _points;
  for(uint i=0; i<degree/2; i++) {
    ctrlPoints.prepend(_points[0]);
    ctrlPoints.append(_points[-1]);
  }

  //knot times with head and tail
  uint m=ctrlPoints.d0+degree;
  knotTimes.resize(m+1);
  for(uint i=0; i<=m; i++) {
    if(i<=degree) knotTimes(i)=_times.first();
    else if(i>=m-degree) knotTimes(i)=_times.last();
    else if((degree%2)) {
      knotTimes(i) = _times(i-degree);
    } else {
      knotTimes(i) = .5*(_times(i-degree-1)+_times(i-degree));
    }
  }

  //can also tune startVel and endVel for degree 2
  if(!!startVel) setDoubleKnotVel(-1, startVel);
  if(!!endVel) setDoubleKnotVel(_points.d0-1, endVel);

  CHECK_EQ(ctrlPoints.d0, knotTimes.N-degree-1, "");

  return *this;
}

BSpline& BSpline::set_vel(uint degree, const arr& _points, const arr& velocities, const arr& _times) {
  arr pts = repmat(_points, 1, 2).reshape(-1, _points.d1);
  arr tms = repmat(_times, 1, 2).reshape(-1);
  set(degree, pts, tms);
  if(velocities.N) {
    for(uint t=0; t<velocities.d0; t++) {
      setDoubleKnotVel(2*t, velocities[t]);
    }
  }
  return *this;
}

BSpline& BSpline::setUniform(uint _degree, uint steps) {
  arr t = ::range(0., 1., steps);
  arr x = t;
  set(_degree, x.reshape(-1, 1), t);
  return *this;
}

arr BSpline::getGridBasis(uint T) {
  arr basis(T+1, ctrlPoints.d0);
  arr db, ddb;
  for(uint t=0; t<=T; t++) {
    getCoeffs2(basis[t].noconst(), db, ddb, double(t)/double(T), degree, knotTimes.p, ctrlPoints.d0, knotTimes.N, 0);
  }
  return basis;
}

void BSpline::append(const arr& _points, const arr& _times, bool inside) {
  CHECK_EQ(_points.nd, 2, "");
  CHECK_EQ(_points.d0, _times.N, "");

  CHECK_GE(_times.first(), 0., "append needs to be in relative time, always with _times.first()>=0.");
  if(!_times.first()) {
    CHECK_LE(maxDiff(ctrlPoints[-1], _points[0]), 1e-10, "when appending with _times.first()=0., the first point needs to be identical to the previous last, making this a double knot");
  }

  //remember end time
  double Tend = knotTimes.last();

  //remove tails
  if(inside) {
    ctrlPoints.resizeCopy(ctrlPoints.d0-degree/2, ctrlPoints.d1);
    knotTimes.resizeCopy(knotTimes.N-1-2*(degree/2));
  } else {
    knotTimes.resizeCopy(knotTimes.N-1-(degree/2));
  }

  //append things:
  ctrlPoints.append(_points);
  knotTimes.append(_times+Tend);
  if(!(degree%2)) {
    arr tmp = knotTimes;
    for(uint i=knotTimes.N-1; i>=knotTimes.N-_times.N; i--) {
//      times(i) = .5*(times(i-1)+times(i));
      knotTimes(i) = .5*(tmp(i-1) + tmp(i));
    }
  }

  //append tails;
  for(uint i=0; i<degree/2; i++) ctrlPoints.append(_points[-1]);
  knotTimes.append(_times(-1)+Tend, 1+2*(degree/2)); //multiple

  CHECK_EQ(ctrlPoints.d0, knotTimes.N-degree-1, "");
}

arr BSpline::getPoints() {
  arr pts = ctrlPoints;
  //remove tail and head
  pts.delRows(0, degree/2);
  pts.resizeCopy(pts.d0-degree/2, pts.d1);
  return pts;
}

void BSpline::setPoints(const arr& pts) {
  CHECK_EQ(pts.d1, ctrlPoints.d1, "");
  CHECK_EQ(pts.d0+2*(degree/2), ctrlPoints.d0, "");
  ctrlPoints = pts;
  for(uint i=0; i<degree/2; i++) {
    ctrlPoints.prepend(pts[0]);
    ctrlPoints.append(pts[-1]);
  }
}

void BSpline::doubleKnot(uint t) {
  ctrlPoints.insRows(t + degree/2);
  ctrlPoints[t+degree/2] = ctrlPoints[t+degree/2-1];

  knotTimes.insert(t+degree+1, knotTimes(t+degree));
}

void BSpline::setDoubleKnotVel(int t, const arr& vel) {
//  CHECK_EQ(degree, 2, "setDoubleKnot only implemented for degree=2");
  arr a=ctrlPoints[t+degree/2];
  arr b=ctrlPoints[t+degree/2+1];
  CHECK(maxDiff(a, b)<1e-10, "this is not a double knot!");
  if(degree==2) {
    a -= vel/degree*(knotTimes(t+degree+1)-knotTimes(t+degree));
    b += vel/degree*(knotTimes(t+degree+2)-knotTimes(t+degree+1));
  } else if(degree==3) {
    a -= vel/degree*(knotTimes(t+degree)-knotTimes(t+degree-1));
    b += vel/degree*(knotTimes(t+degree+2)-knotTimes(t+degree+1));
  } else NIY;
}

//==============================================================================

arr Path::getPosition(double t) const {
  return BSpline::eval(t);
}

arr Path::getVelocity(double t) const {
  return BSpline::eval(t, 1);
}

void Path::transform_CurrentBecomes_EndFixed(const arr& current, double t) {
  arr delta = current - eval(t);
  for(uint i=0; i<ctrlPoints.d0; i++) {
    double ti = double(i)/double(ctrlPoints.d0-1);
    double a = (1.-ti)/(1.-t);
    ctrlPoints[i] += a*delta;
  }
}

void Path::transform_CurrentFixed_EndBecomes(const arr& end, double t) {
  arr delta = end - eval(1.);
  for(uint i=0; i<ctrlPoints.d0; i++) {
    double ti = double(i)/double(ctrlPoints.d0-1);
    double a = (ti-t)/(1.-t);
    ctrlPoints[i] += a*delta;
  }
}

void Path::transform_CurrentBecomes_AllFollow(const arr& current, double t) {
  arr delta = current - eval(t);
  for(uint i=0; i<ctrlPoints.d0; i++) ctrlPoints[i] += delta;
}

//==============================================================================

void CubicPiece::set(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau) {
  double tau2 = tau*tau, tau3 = tau*tau2;
  d = x0;
  c = v0;
  b = 1./tau2 * (3.*(x1-x0) - tau*(v1+2.*v0));
  a = 1./tau3 * (-2.*(x1-x0) + tau*(v1+v0));
}

void CubicPiece::eval(arr& x, arr& xDot, arr& xDDot, double t) const {
  double t2=t*t, t3=t*t2;
  if(!!x) {
    x = d;
    x += t*c;
    x += t2*b;
    x += t3*a;
  }
  if(!!xDot) {
    xDot = c;
    xDot += (2.*t)*b;
    xDot += (3.*t2)*a;
  }
  if(!!xDDot) {
    xDDot = 2.*b;
    xDDot += (6.*t)*a;
  }
}

arr CubicPiece::eval(double t, uint diff) {
  if(diff==3) {
    return 6.*a;
  } else {
    NIY;
  }
  return {};
}

void CubicSpline::set(const arr& pts, const arr& vels, const arr& _times) {
  CHECK_GE(_times.N, 2, "need at least 2 knots");
  times = _times;
  uint K=pts.d0-1;
  pieces.resize(K);
  for(uint k=0; k<K; k++) {
    pieces(k).set(pts[k], vels[k], pts[k+1], vels[k+1], times(k+1)-times(k));
  }
}

void CubicSpline::append(const arr& pts, const arr& vels, const arr& _times) {
  CHECK_GE(_times(0), 1e-6, "for appending, first time needs to be greater zero");

  //current end state
  arr x, xDot;
  pieces(-1).eval(x, xDot, NoArr, times(-1)-times(-2));

  //new times
  double Tend = times.last();
  times.append(_times+Tend);

  uint K0=pieces.N;
  uint K=pts.d0;
  pieces.resizeCopy(K0+K);
  pieces(K0+0).set(x, xDot, pts[0], vels[0], _times(0));
  for(uint k=1; k<K; k++) {
    pieces(K0+k).set(pts[k-1], vels[k-1], pts[k], vels[k], _times(k)-_times(k-1));
  }
}

uint CubicSpline::getPiece(double t) const {
  CHECK_GE(times.N, 2, "spline is empty");
  if(t<times(0)) return 0;
  if(t>times(-1)) return pieces.N-1;
  uint k = times.rankInSorted(t, rai::lowerEqual<double>, false);
  if(k) k--;
  if(k>=pieces.N-1) k=pieces.N-1;
  return k;
}

void CubicSpline::eval(arr& x, arr& xDot, arr& xDDot, double t) const {
  CHECK_GE(times.N, 2, "spline is empty");
  if(t<times(0)) {
    pieces(0).eval(x, xDot, xDDot, 0.);
    if(!!xDDot) {
      CHECK_ZERO(absMax(xDot), 1e-6, "don't query a cubic spline at neg time for non-zero start velocity");
      xDDot.setZero();
    }
    return;
  }
  if(t>times(-1)) {
    pieces(-1).eval(x, xDot, xDDot, times(-1)-times(-2));
    if(!!xDDot) {
      CHECK_ZERO(absMax(xDot), 1e-6, "don't query a cubic spline after final time for non-zero final velocity");
      xDDot.setZero();
    }
    return;
  }

  uint k = times.rankInSorted(t, rai::lowerEqual<double>, false);
  if(k<times.N) { CHECK_LE(t, times(k), ""); }
  else { CHECK_GE(t, times.last(), ""); }
  if(k) k--;
  if(k>=pieces.N-1) k=pieces.N-1;
  //    CHECK_GE(t, times(k), "");
  //    cout <<"t: " <<t <<" k: " <<k <<' ' <<times <<endl;

  pieces(k).eval(x, xDot, xDDot, t-times(k));
}

arr CubicSpline::eval(double t, uint diff) const {
  arr ret;
  if(diff==0) eval(ret, NoArr, NoArr, t);
  else if(diff==1) eval(NoArr, ret, NoArr, t);
  else if(diff==2) eval(NoArr, NoArr, ret, t);
  else { uint k=getPiece(t); ret = pieces(k).eval(t-times(k), diff); }
  return ret;
}

arr CubicSpline::eval(const arr& T, uint diff) const {
  arr x(T.N, pieces.first().d.N);
  for(uint i=0; i<T.N; i++) x[i] = eval(T(i), diff);
  return x;
}

arr CubicSplineLeapCost(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau, const arr& tauJ) {
  arr D = (x1-x0) - (.5*tau)*(v0+v1);
  if(tauJ.N) {
    if(!D.jac) D.J().sparse().resize(v0.N, tauJ.d1, 0);
    D.J() -= .5 * (v0+v1).noJ() * tauJ;
  }

  arr V = v1 - v0;

  double s12 = sqrt(12.);

  arr tilD = (s12 * pow(tau, -1.5)) * D;
  if(tauJ.N) tilD.J() += (s12 * (-1.5) * pow(tau, -2.5)) * D.noJ() * tauJ;

  arr tilV = pow(tau, -0.5) * V;
  if(tauJ.N) {
    if(!tilV.jac) tilV.J().sparse().resize(v0.N, tauJ.d1, 0);
    tilV.J() += ((-0.5) * pow(tau, -1.5)) * V.noJ() * tauJ;
  }

  arr y;
  y.setBlockVector(tilD, tilV);
  return y;
}

arr CubicSplineMaxJer(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau, const arr& tauJ) {
  //jerk is 6a
  double tau2 = tau*tau, tau3 = tau*tau2, tau4 = tau2*tau2;
  arr a6 = 6./tau3 * (-2.*(x1-x0) + tau*(v1+v0));
  if(tauJ.N) {
    a6.J() += (36./tau4 * (x1-x0)) * tauJ;
    a6.J() += (-12./tau3 * (v1+v0)) * tauJ;
  }

  uint n=x0.N;
  arr y(2*n);
  if(a6.jac) y.J().sparse().resize(y.N, a6.jac->d1, 0);
  y.setVectorBlock(a6, 0*n);
  y.setVectorBlock(-a6, 1*n);
  return y;
}

arr CubicSplineAcc0(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau, const arr& tauJ) {
  //acceleration is 6a t + 2b
  double tau2 = tau*tau, tau3 = tau*tau2;
  arr b2 = 2./tau2 * (3.*(x1-x0) - tau*(v1+2.*v0));
  if(tauJ.N) {
    b2.J() += -12./tau3 * (x1-x0) * tauJ;
    b2.J() -= -2./tau2 * (v1+2.*v0) * tauJ;
  }
  return b2;
}

arr CubicSplineAcc1(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau, const arr& tauJ) {
  //acceleration is 6a t + 2b
  double tau2 = tau*tau, tau3 = tau*tau2;
  //  arr d = x0;
  //  arr c = v0;
  arr b2 = 2./tau2 * (3.*(x1-x0) - tau*(v1+2.*v0));
  if(tauJ.N) {
    b2.J() += -12./tau3 * (x1-x0) * tauJ;
    b2.J() -= -2./tau2 * (v1+2.*v0) * tauJ;
  }
  arr a6_tau = 6./tau2 * (-2.*(x1-x0) + tau*(v1+v0));
  if(tauJ.N) {
    a6_tau.J() -= -24./tau3 * (x1-x0) * tauJ;
    a6_tau.J() += -6./tau2 * (v1+v0) * tauJ;
  }
  return a6_tau + b2;
}

arr CubicSplineMaxAcc(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau, const arr& tauJ) {
  //acceleration is 6a t + 2b
  double tau2 = tau*tau, tau3 = tau*tau2;
  //  arr d = x0;
  //  arr c = v0;
  arr b2 = 2./tau2 * (3.*(x1-x0) - tau*(v1+2.*v0));
  if(tauJ.N) {
    b2.J() += -12./tau3 * (x1.noJ()-x0.noJ()) * tauJ;
    b2.J() -= -2./tau2 * (v1.noJ()+2.*v0.noJ()) * tauJ;
  }
  arr a6_tau = 6./tau2 * (-2.*(x1-x0) + tau*(v1+v0));
  if(tauJ.N) {
    a6_tau.J() -= -24./tau3 * (x1.noJ()-x0.noJ()) * tauJ;
    a6_tau.J() += -6./tau2 * (v1.noJ()+v0.noJ()) * tauJ;
  }

  uint d=x0.N;
  arr y(4*d);
  if(b2.jac) y.J().sparse().resize(y.N, b2.jac->d1, 0);
  y.setVectorBlock(b2, 0*d);
  y.setVectorBlock(-b2, 1*d);
  y.setVectorBlock(b2 + a6_tau, 2*d);
  y.setVectorBlock(-b2 - a6_tau, 3*d);
  return y;
}

arr CubicSplineMaxVel(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau, const arr& tauJ) {
  //acc is 6a t + 2b; with root at t=-b/3a
  //velocity is 3a t^2 + 2b t + c; at root is -b^2 / 3a + c

#if 1
  double tau2 = tau*tau;
  //  arr d = x0;
  arr c = v0;
  arr b = (3.*(x1-x0) - tau*(v1+2.*v0));
  if(tauJ.N) {
    b.J() -= (v1+2.*v0) * tauJ;
  }
  arr a = (-2.*(x1-x0) + tau*(v1+v0));
  if(tauJ.N) {
    a.J() += (v1+v0) * tauJ;
  }
  arr t=-tau*b.noJ()/(3.*a.noJ());
  //indicators for each dimension
  arr iv0=zeros(t.N), iv1=zeros(t.N), ivm=zeros(t.N);
  for(uint i=0; i<t.N; i++) {
    if(t(i)<=0) iv0(i)=1.;
    else if(t(i)>=tau) iv1(i)=1.;
    else ivm(i)=1.;
  }
  arr vmax = c + (1./tau) * (b + 3./4.*a);
  if(tauJ.N) {
    vmax.J() -= (1./tau2) * (b + 3./4.*a) * tauJ;
  }

//  vmax = iv0%v0;
//  vmax += iv1%v1;
//  vmax += ((1./(3.*tau)) * ((ivm%b%b)/a) + c);
//  if(tauJ.N){
//    vmax.J() -= (1./(3.*tau2)) * ((ivm%b%b)/a) * tauJ;
//  }
#endif

  uint n=x0.N;
  arr y(4*n);
  y.setZero();
  if(v0.jac) y.J().sparse().resize(y.N, v0.jac->d1, 0);
  else if(vmax.jac) y.J().sparse().resize(y.N, vmax.jac->d1, 0);
  y.setVectorBlock(v0, 0*n);
  y.setVectorBlock(-v0, 1*n);
  y.setVectorBlock(vmax, 2*n);
  y.setVectorBlock(-vmax, 3*n);
  return y;
}

void CubicSplinePosVelAcc(arr& pos, arr& vel, arr& acc, double trel, const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau, const arr& tauJ) {
  //position at time t:
  // a t^3 + b t^2 + c t + d

  CHECK_GE(trel, 0., "");
  CHECK_LE(trel, 1., "");

  double tau2 = tau*tau;
  double tau3 = tau2*tau;
  arr d = x0;
  arr c = v0;
#if 0
  arr b = 1./tau2 * (3.*(x1-x0) - tau*(v1+2.*v0));
  if(tauJ.N) {
    b.J() += -6./tau3 * (x1.noJ()-x0.noJ()) * tauJ;
    b.J() -= -1./tau2 * (v1.noJ()+2.*v0.noJ()) * tauJ;
  }
  arr a = 1./tau3 * (-2.*(x1-x0) + tau*(v1+v0));
  if(tauJ.N) {
    a.J() -= -6./tau4 * (x1.noJ()-x0.noJ()) * tauJ;
    a.J() += -2./tau3 * (v1.noJ()+v0.noJ()) * tauJ;
  }
#endif
  arr c_tau = tau*c;
  if(tauJ.N) { if(c_tau.jac) c_tau.J() += (c) * tauJ; else c_tau.J() = c*tauJ; }

  arr b_tau2 = (3.*(x1-x0) - tau*(v1+2.*v0));
  if(tauJ.N) b_tau2.J() -= (v1.noJ()+2.*v0.noJ()) * tauJ;

  arr b_tau1 = 1./tau * b_tau2;
  if(tauJ.N) b_tau1.J() += (-1./tau2) * b_tau2.noJ() * tauJ;

  arr b_tau0 = 1./tau2 * b_tau2;
  if(tauJ.N) b_tau0.J() += (-2./tau3) * b_tau2.noJ() * tauJ;

  arr a_tau3 = (-2.*(x1-x0) + tau*(v1+v0));
  if(tauJ.N) a_tau3.J() += (v1+v0) * tauJ;

  arr a_tau2 = 1./tau * a_tau3;
  if(tauJ.N) a_tau2.J() += (-1./tau2) * a_tau3.noJ() * tauJ;

  arr a_tau1 = 1./tau2 * a_tau3;
  if(tauJ.N) a_tau1.J() += (-2./tau3) * a_tau3.noJ() * tauJ;

  //arr a_tau0 = 1./tau3 * a_tau3;
  //if(tauJ.N) a_tau0.J() += (-3./tau4) * a_tau3.noJ() * tauJ;

  if(!!pos) pos = (trel*trel*trel)*a_tau3 + (trel*trel)*b_tau2 + trel*c_tau + d;
  if(!!vel) vel = (3.*trel*trel)*a_tau2 + (2.*trel)*b_tau1 + c;
  if(!!acc) acc = (6.*trel)*a_tau1 + (2.)*b_tau0;
#if 0
  if(!!pos) pos = (t*t*t)*a + (t*t)*b + t*c + d;
  if(!!vel) vel = (3.*t*t)*a + (2.*t)*b + c;
  if(!!acc) acc = (6.*t)*a + (2.)*b;
#endif
}

} //namespace rai
