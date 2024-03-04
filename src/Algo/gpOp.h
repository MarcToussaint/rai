/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"
#include "../Core/util.h"

#include <math.h>

struct GaussianProcessKernel;

struct GaussianProcessOptimized {
  arr X, Y;   ///< data

  arr L; ///< cholesky factor of K
  arr GinvY;

  double m; ///< const bias of the GP

  double obsVar;

  GaussianProcessKernel* kernel;

  GaussianProcessOptimized();
  GaussianProcessOptimized(const GaussianProcessOptimized& copy);
  ~GaussianProcessOptimized();

  void setKernel(GaussianProcessKernel* k);

  void appendObsRecompute(const arr& x, const double& y);

  void recompute();

  void clearData();

  double evaluate(const arr& x);
  double evaluateVariance(const arr& x);
  void evaluate(const arr& x, double& y, double& sig);
  void evaluate(const arr& x, double& y, bool calcY, double& sig, bool calcSig);

  arr gradient(const arr& x);
  arr gradientVariance(const arr& x);
  arr hessian(const arr& x);
};

struct GaussianProcessKernel {
  virtual double k(const arr& x, const arr& xPrime) = 0;
  virtual arr dk_dx(const arr& x, const arr& xPrime) = 0;
  virtual arr d2k_dx2(const arr& x, const arr& xPrime) = 0;

  arr kappa(const arr& x, const arr& X) {
    arr kap;
    kap.resize(X.d0);
    for(uint i = 0; i < X.d0; i++) {
      kap(i) = k(x, X[i]);
    }
    return kap;
  }

  arr dKappa(const arr& x, const arr& X) {
    arr dKap;
    dKap.resize(X.d0, X.d1);
    for(uint i = 0; i < X.d0; i++) {
      dKap[i] = dk_dx(x, X[i]);
    }
    return dKap;
  }

  virtual ~GaussianProcessKernel() {}
  virtual GaussianProcessKernel* clone() = 0;
};

struct GaussianProcessGaussKernel : GaussianProcessKernel {
  double priorVar;
  double l;

  GaussianProcessGaussKernel(double priorStdD, double l)
    : priorVar(priorStdD*priorStdD)
    , l(l*l) {}

  double k(const arr& x, const arr& xPrime) {
    if(&x == &xPrime) {
      return priorVar;
    }
    return priorVar*exp(-0.5*sqrDistance(x, xPrime)/l);
  }

  arr dk_dx(const arr& x, const arr& xPrime) {
    if(&x == &xPrime) return zeros(x.d0);
    return k(x, xPrime) * (xPrime - x) / l;
  }

  arr d2k_dx2(const arr& x, const arr& xPrime) {
    if(&x == &xPrime) {
      return priorVar/l*eye(x.d0);
    }
    double co = k(x, xPrime)/l;
    arr diff = (x-xPrime);
    arr hessian;
    op_outerProduct(hessian, diff, diff);
    hessian = hessian*co/l;
    for(uint i = 0; i < hessian.d0; i++) {
      hessian(i, i) -= co;
    }
    return hessian;
  }

  GaussianProcessGaussKernel* clone() {
    return new GaussianProcessGaussKernel(*this);
  }
};

struct GaussianProcessNegativeDistanceKernel : GaussianProcessKernel {

  double k(const arr& x, const arr& xPrime) {
    if(&x == &xPrime) {
      return 0.0;
    }
    return -sqrDistance(x, xPrime);
  }

  arr dk_dx(const arr& x, const arr& xPrime) {
    if(&x == &xPrime) return zeros(x.d0);
    return -2.0*(x-xPrime);
  }

  arr d2k_dx2(const arr& x, const arr& xPrime) {
    return -2.0*eye(x.d0);
  }

  GaussianProcessNegativeDistanceKernel* clone() {
    return new GaussianProcessNegativeDistanceKernel(*this);
  }
};

struct GaussianProcessInverseMultiQuadricKernel : GaussianProcessKernel {
  double c;

  GaussianProcessInverseMultiQuadricKernel(double c) : c(c) {}

  double k(const arr& x, const arr& xPrime) {
    return 1.0/sqrt(sqrDistance(x, xPrime)+c*c);
  }

  arr dk_dx(const arr& x, const arr& xPrime) {
    if(&x == &xPrime) return zeros(x.d0);
    return -(x-xPrime)/pow(sqrDistance(x, xPrime)+c*c, 1.5);
  }

  arr d2k_dx2(const arr& x, const arr& xPrime) {
    double co = k(x, xPrime);
    double co3 = co*co*co;
    arr diff = (x-xPrime);
    arr hessian;
    op_outerProduct(hessian, diff, diff);
    hessian = 3.0*co3*co*co*hessian;
    for(uint i = 0; i < hessian.d0; i++) {
      hessian(i, i) -= co3;
    }
    return hessian;
  }

  GaussianProcessInverseMultiQuadricKernel* clone() {
    return new GaussianProcessInverseMultiQuadricKernel(*this);
  }
};
