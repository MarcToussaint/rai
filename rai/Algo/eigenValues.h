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


/** This implements power methods to efficiently compute the extreme
 *  (maximal and minimal) eigenvalue and eigenVector for a matrix.
 *  This allows for a very efficient incremental update (e.g., when
 *  the statistics change). (mt) */

#pragma once

#include <Core/array.h>

struct ExtremeEigenValues{
  arr A;
  arr x_hi, x_lo;
  double lambda_hi, lambda_lo;

  void computeExact(); ///< computes exact extreme eigenvalues/-vectors using lapack

  void initPowerMethodRandom(); ///< initializes the power method with random eigenvectors
  void stepPowerMethod(uint k=1); ///< step the power method (k-times)
};
