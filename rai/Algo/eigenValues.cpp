/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "eigenValues.h"

void ExtremeEigenValues::computeExact() {
  arr lambda, x;
  lapack_EigenDecomp(A, lambda, x);
  lambda_lo = lambda(0);      if(lambda_lo>1e-10) x_lo = x[0];
  lambda_hi = lambda.last();  if(lambda_hi>1e-10) x_hi = x[x.d0-1];
}

void ExtremeEigenValues::initPowerMethodRandom() {
  x_hi = 2.*rand(A.d0)-1.;  x_hi/=length(x_hi);
  x_lo = 2.*rand(A.d0)-1.;  x_lo/=length(x_lo);
}

void ExtremeEigenValues::stepPowerMethod(uint k) {
  for(uint i=0; i<k; i++) {
    x_hi = A*x_hi;
    lambda_hi=length(x_hi);
    x_hi /= lambda_hi;
    
    x_lo = (lambda_hi*eye(A.d0) - A) * x_lo;
    lambda_lo=length(x_lo);
    x_lo /= lambda_lo;
    lambda_lo = lambda_hi - lambda_lo;
  }
}
