/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"

namespace rai {

//==============================================================================

struct BSplineCore {
  arr knots;
  arr B, Bdot, Bddot;
  arr JBtimes;

  void setUniformKnots(uint degree, uint nPointsWODuplicates);
  arr get(double t, uint degree, uint derivatives=0, bool calc_JBtimes=false);
  arr getBmatrix(uint T, uint degree);
  void setKnots(uint degree, const arr& times);
};

/// a B-spline
struct BSpline {
  uint degree;
  arr ctrlPoints, knotTimes; ///< the points and times with (non-intuitive) head and tail added depending on degree

  //-- methods to define the points and times
  BSpline& set(uint degree, const arr& _points, const arr& _times, const arr& startVel=NoArr, const arr& endVel=NoArr);
  BSpline& set_vel(uint degree, const arr& _points, const arr& velocities, const arr& _times);
  BSpline& setUniform(uint _degree, uint steps);
  arr getGridBasis(uint T);

  void append(const arr& _points, const arr& _times, bool inside);
  void clear();

  //
  arr getPoints();
  void setPoints(const arr& pts);

  //experimental
  void doubleKnot(uint t);
  void setDoubleKnotVel(int t, const arr& vel);

  /// core method to evaluate spline
  // void eval(arr& x, arr& xDot, arr& xDDot, double t) const;
  arr eval(double t, uint derivative=0) const;
  void eval2(arr& x, arr& xDot, arr& xDDot, double t, arr& Jpoints=NoArr, arr& Jtimes=NoArr) const;
  arr eval(const arr& ts);

  // arr jac_point(double t, uint derivative=0) const;

  /// for t \in [0,1] the coefficients are the weighting of the points: f(t) = coeffs(t)^T * points
  // arr getCoeffs(double t, uint K, uint derivative=0) const;

  double begin() const { return knotTimes.first(); }
  double end() const { return knotTimes.last(); }

//  arr getGridBasis(uint derivative=0){ HALT("see retired/spline-21-04-01.cpp"); }

  // static void getCoeffs2(arr& c0, arr& c1, arr& c2, double t, uint degree, double* knots, uint nCtrls, uint nKnots, uint derivatives=0);
};

//==============================================================================

arr BSpline_path2ctrlPoints(const arr& path, uint numCtrlPoints, uint degree=2, bool flatEnds=true);

//==============================================================================

struct CubicPiece {
  arr a, b, c, d;
  void set(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau);
  void eval(arr& x, arr& xDot, arr& xDDot, double t) const;
  arr eval(double t, uint diff);
  void write(ostream& os) const { os <<"a:" <<a <<"b:" <<b <<"c:" <<c <<"d:" <<d; }
};
stdOutPipe(CubicPiece)

struct CubicSplineCtor { arr pts, vels, times; };

struct CubicSpline {
  rai::Array<CubicPiece> pieces;
  arr times;

  void set(const arr& pts, const arr& vels, const arr& _times);
  void append(const arr& pts, const arr& vels, const arr& _times);

  uint getPiece(double t) const;
  void eval(arr& x, arr& xDot, arr& xDDot, double t) const;
  arr eval(double t, uint diff=0) const;
  arr eval(const arr& T, uint diff=0) const;

  double begin() const { return times.first(); }
  double end() const { return times.last(); }
};

//==============================================================================

arr CubicSplineLeapCost(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau, const arr& tauJ= {});
arr CubicSplineMaxJer(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau, const arr& tauJ= {});
arr CubicSplineMaxAcc(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau, const arr& tauJ= {});
arr CubicSplineMaxVel(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau, const arr& tauJ= {});
arr CubicSplineAcc0(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau, const arr& tauJ= {});
arr CubicSplineAcc1(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau, const arr& tauJ= {});
void CubicSplinePosVelAcc(arr& pos, arr& vel, arr& acc, double trel, const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau, const arr& tauJ);

} //namespace rai
