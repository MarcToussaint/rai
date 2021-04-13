/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"

namespace rai {

/// a spline
struct Spline {
  uint degree;
  arr points, times; ///< the points and times as provided by the user
  arr knotPoints, knotTimes; ///< the points and times with (non-intuitive) head and tail added depending on degree

  //-- methods to define the points and times
  Spline& set(uint degree, const arr& _points, const arr& _times, const arr& startVel=NoArr, const arr& endVel=NoArr);
  void append(const arr& _points, const arr& _times);
  void clear();

  //experimental
  void doubleKnot(uint t);
  void setDoubleKnotVel(int t, const arr& vel);


  /// core method to evaluate spline
  void eval(arr& x, arr& xDot, arr& xDDot, double t) const;
  arr eval(double t, uint derivative=0) const{
    arr x;
    if(derivative==0) eval(x, NoArr, NoArr, t);
    else if(derivative==1) eval(NoArr, x, NoArr, t);
    else if(derivative==2) eval(NoArr, NoArr, x, t);
    else NIY;
    return x;
  }
  arr eval(const arr& ts){
    arr f(ts.N, points.d1);
    for(uint i=0;i<ts.N;i++) f[i] = eval(ts(i));
    return f;
  }

  /// for t \in [0,1] the coefficients are the weighting of the points: f(t) = coeffs(t)^T * points
  arr getCoeffs(double t, uint K, uint derivative=0) const;


  double begin() const { return knotTimes.first(); }
  double end() const { return knotTimes.last(); }

  arr getGridBasis(uint derivative=0){ HALT("see retired/spline-21-04-01.cpp"); }

  static void getCoeffs2(arr& c0, arr& c1, arr& c2, double t, uint degree, double* knotTimes, uint knotN, uint knotTimesN, uint derivatives=0);

};

} //namespace rai

//==============================================================================

namespace rai {

/// a wrapper around a spline with methods specific to online path adaptation
struct Path : Spline {
  Path(arr& X, uint degree=3) { set(3, X, grid(1, 0., 1., X.d0-1)); }

  arr getPosition(double t) const;
  arr getVelocity(double t) const;

  /// use this when your endeffector moved differently than expected, but the goal remains fixed
  void transform_CurrentBecomes_EndFixed(const arr& current, double t);
  /// use this when your sensors say that the goal moved, but the endeffector remains fixed
  void transform_CurrentFixed_EndBecomes(const arr& end, double t);
  /// use this when sensor say that the whole task space has to be recalibrated, including current and end
  void transform_CurrentBecomes_AllFollow(const arr& current, double t);
};

} //namespace rai
