/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

/** This implements power methods to efficiently compute the extreme
 *  (maximal and minimal) eigenvalue and eigenVector for a matrix.
 *  This allows for a very efficient incremental update (e.g., when
 *  the statistics change). (mt) */

#pragma once

#include "../Core/array.h"

struct ExtremeEigenValues {
  arr A;
  arr x_hi, x_lo;
  double lambda_hi, lambda_lo;

  void computeExact(); ///< computes exact extreme eigenvalues/-vectors using lapack

  void initPowerMethodRandom(); ///< initializes the power method with random eigenvectors
  void stepPowerMethod(uint k=1); ///< step the power method (k-times)
};
