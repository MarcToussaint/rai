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


#ifndef Algo_spline_h
#define Algo_spline_h

#include <Core/array.h>

namespace MT {

/// a spline
struct Spline {
  uint T, K, degree;
  arr points; ///< the reference points
  arr times;  ///< what times (in [0,1]) the reference points refer to (usually uniform)
//  arr weights;
  arr basis, basis_trans, basis_timeGradient;

  Spline(uint T, arr& X, uint degree=2){ setUniformNonperiodicBasis(T, X.d0-1, degree); points=X; }

  arr getBasis(double time) const;
  void setBasis();
  void setBasisAndTimeGradient();
  void setUniformNonperiodicBasis(uint T, uint K, uint degree);

  arr eval(double t) const;
  arr eval(uint t) const;
  arr eval() const;

  void partial(arr& grad_points, const arr& grad_path) const;
  void partial(arr& dCdx, arr& dCdt, const arr& dCdf, bool constrain=true) const;

  void plotBasis();
};

} //namespace MT

#endif
