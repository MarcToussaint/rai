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



#ifndef Algo_spline_h
#define Algo_spline_h

#include <Core/array.h>

namespace MT {

/// a spline
struct Spline {
  uint degree;
  arr points; ///< the reference points
  arr times;  ///< what times (in [0,1]) the reference points refer to (usually uniform)
  arr basis, basis_trans, basis_timeGradient;

  /// for T>0 this is directly constructing basis functions over a (fine) grid of resolution T
  Spline(uint T, arr& X, uint degree=2){ setUniformNonperiodicBasis(T, X.d0-1, degree); points=X; }

  /// two methods to get the coefficients -- the first analytic, the second buggy and recursive
  double getCoeff(double t, double t2, uint der=0) const;
  arr getCoeffs(double time, uint K, uint der=0) const;

  /// core method to evaluate the spline at an arbitrary point
  arr eval(double t, bool velocities=false) const;

  //-- the rest are all matrix methods:

  /// methods to construct a basis matrix mapping from the K points to a (fine) grid of resolution T
  void setBasis(uint T, uint K);
  void setBasisAndTimeGradient(uint T, uint K);
  void setUniformNonperiodicBasis(uint T, uint K, uint degree);

  arr eval(uint t) const;
  arr eval() const;
  arr smooth(double lambda) const;

  /// gradient w.r.t. the points (trivial: mapping is linear)
  void partial(arr& grad_points, const arr& grad_path) const;
  /// gradient w.r.t. the timings of the point
  void partial(arr& dCdx, arr& dCdt, const arr& dCdf, bool clip=true) const;

  void plotBasis();
};

} //namespace MT

//==============================================================================

namespace MT {

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

} //namespace MT

#endif
