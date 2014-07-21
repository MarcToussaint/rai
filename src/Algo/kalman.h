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

#include <Core/array.h>

struct Kalman{
  arr b_mean, b_var;
  void initialize(const arr& _b_mean, const arr& _b_var){ b_mean=_b_mean, b_var=_b_var; }
  void stepPredict(const arr& A, const arr& a, const arr& Q);
  void stepObserve(const arr& y, const arr& C, const arr& c, const arr& W);
  void step(const arr& A, const arr& a, const arr& Q, const arr& y, const arr& C, const arr& c, const arr& W);
};

