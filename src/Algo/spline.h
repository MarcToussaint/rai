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

/// a B-spline
struct BSpline {
  uint degree;
  arr knots;
  arr ctrlPoints;
  //return values of calcB
  arr B, Bdot, Bddot;
  arr JBtimes;

  //-- core Bspline methods concerning knots and coefficients
  void setKnots(uint _degree, const arr& times);
  void calcB(double t, uint derivatives=0, bool calc_JBtimes=false);
  arr getBmatrix(const arr& sampleTimes, bool startDuplicates=false, bool endDuplicates=false);

  //-- methods concerning ctrl points
  void setCtrlPoints(const arr& points, bool addStartDuplicates=true, bool addEndDuplicates=true, const arr& setStartVel=NoArr, const arr& setEndVel=NoArr);

  //-- convenience user functions
  BSpline& set(uint _degree, const arr& points, const arr& times, const arr& startVel=NoArr, const arr& endVel=NoArr);
  void overwriteSmooth(const arr& points, const arr& times_rel, double time_cut);
  void append(const arr& points, const arr& times_rel, bool inside);
  void clear();
  arr& getKnots(){ return knots; }
  arr& getCtrlPoints(){ return ctrlPoints; }
  arr getPoints();

  //experimental
  void doubleKnot(uint t);
  void setDoubleKnotVel(int t, const arr& vel);

  /// core method to evaluate spline
  void eval3(arr& x, arr& xDot, arr& xDDot, double t, arr& Jpoints=NoArr, arr& Jtimes=NoArr) const;
  arr eval(double t, uint derivative=0) const;
  arr eval(const arr& sampleTimes, uint derivative=0) const;

  // arr jac_point(double t, uint derivative=0) const;

  /// for t \in [0,1] the coefficients are the weighting of the points: f(t) = coeffs(t)^T * points
  // arr getCoeffs(double t, uint K, uint derivative=0) const;

  double begin() const { return knots.elem(0); }
  double end() const { return knots.elem(-1); }
};

//==============================================================================

arr BSpline_path2ctrlPoints(const arr& path, uint numCtrlPoints, uint degree=2, bool startDuplicates=true, bool endDuplicates=true);

//==============================================================================

struct CubicPiece {
  arr a, b, c, d;
  void set(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau);
  void eval(arr& x, arr& xDot, arr& xDDot, double t) const;
  arr eval(double t, uint diff);
  void write(ostream& os) const { os <<"a:" <<a <<" b:" <<b <<" c:" <<c <<" d:" <<d; }
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
