/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#ifndef Algo_spline_h
#define Algo_spline_h

#include <Core/array.h>

namespace mlr {

/// a spline
struct Spline {
  uint degree;
  arr points; ///< the reference points
  arr times;  ///< what times (in [0,1]) the reference points refer to (usually uniform)
  arr basis, basis_trans, basis_timeGradient;

  /// for T>0 this is directly constructing basis functions over a (fine) grid of resolution T
  Spline(uint degree=2);
  Spline(uint T, const arr& X, uint degree=2);

  /// for t \in [0,1] the coefficients are the weighting of the points: f(t) = coeffs(t)^T * points
  arr getCoeffs(double t, uint K, uint derivative=0) const;

  /// returns f(t) using getCoeffs for any t \in [0,1]
  arr eval(double t, uint derivative=0) const;

  //-- the rest are all matrix methods, using the basis mastix for fixed grid of size T

  /// methods to construct a basis matrix mapping from the K points to a (fine) grid of resolution T
  void setBasis(uint T, uint K); ///< requires that degree and times has been set; computes basis, basis_trans
  void setBasisAndTimeGradient(uint T, uint K); ///< as above, but computes also gradient w.r.t. times
  void setUniformNonperiodicBasis(uint T, uint K, uint degree); ///< sets the times uniformly, then computes basis
  void setUniformNonperiodicBasis();

  /// returns f(t/T) at one of the precomputed grid points of the basis matrix
  arr eval(uint t) const;
  /// returns the full f(:) at all grid points -> (T+1, points.d1)-matrix
  arr eval() const;
  arr smooth(double lambda) const;

  /// gradient w.r.t. the points (trivial: mapping is linear)
  void partial(arr& grad_points, const arr& grad_path) const;
  /// gradient w.r.t. the timings of the point
  void partial(arr& dCdx, arr& dCdt, const arr& dCdf, bool clip=true) const;

  void plotBasis();
};

} //namespace mlr

//==============================================================================

namespace mlr {

/// a wrapper around a spline with methods specific to online path adaptation
struct Path : Spline {
  Path(arr& X, uint degree=3):Spline(0,X,degree){}

  arr getPosition(double t) const;
  arr getVelocity(double t) const;

  /// use this when your endeffector moved differently than expected, but the goal remains fixed
  void transform_CurrentBecomes_EndFixed(const arr& current, double t);
  /// use this when your sensors say that the goal moved, but the endeffector remains fixed
  void transform_CurrentFixed_EndBecomes(const arr& end, double t);
  /// use this when sensor say that the whole task space has to be recalibrated, including current and end
  void transform_CurrentBecomes_AllFollow(const arr& current, double t);
};

} //namespace mlr

#endif
