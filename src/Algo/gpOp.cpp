/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "gpOp.h"

GaussianProcessOptimized::GaussianProcessOptimized() :
  m(0.) {}

GaussianProcessOptimized::GaussianProcessOptimized(const GaussianProcessOptimized& copy) {
  X = copy.X;
  Y = copy.Y;
  L = copy.L;
  GinvY = copy.GinvY;
  m = copy.m;
  obsVar = copy.obsVar;
  kernel = copy.kernel->clone();
}

GaussianProcessOptimized::~GaussianProcessOptimized() {
  delete kernel;
}

void GaussianProcessOptimized::setKernel(GaussianProcessKernel* k) {
  kernel = k->clone();
  delete k;
}

void GaussianProcessOptimized::appendObsRecompute(const arr& x, const double& y) {
  if(X.d0) {
    X.append(~x);
    Y.append(y);
    double k = kernel->k(x, x) + obsVar;
    arr kappa = zeros(Y.N-1);
    for(uint i = 0; i < kappa.d0; i++) {
      kappa(i) = kernel->k(X[i], x);
    }
    arr l = lapack_Ainv_b_triangular(L, kappa);
    double lDiag = sqrt(k-sumOfSqr(l));

    arr temp;
    temp.resize(L.d0+1, L.d1+1);
    temp.setMatrixBlock(L, 0, 0);
    temp.setMatrixBlock(l, 0, L.d1);
    temp.setMatrixBlock(zeros(1, L.d1), L.d0, 0);
    temp(L.d0, L.d1) = lDiag;
    L = temp;
    GinvY = lapack_Ainv_b_symPosDef_givenCholesky(L, Y-m);
  } else {
    X.clear();
    Y.clear();
    X.append(~x);
    Y.append(y);
    recompute();
  }
}

void GaussianProcessOptimized::recompute() {
  if(X.d0) {
    arr G;
    G.resize(X.d0, X.d0);
    for(uint i = 0; i < G.d0; i++) {
      for(uint j = i; j < G.d1; j++) {
        G(i, j) = kernel->k(X[i], X[j]); //fill only the upper triangle, because the lapack routine does not access the other one at all.
      }
      G(i, i) += obsVar;
    }
    lapack_cholesky(L, G);
    GinvY = lapack_Ainv_b_symPosDef_givenCholesky(L, Y-m);
  }
}

void GaussianProcessOptimized::clearData() {
  X.clear();
  Y.clear();
  L.clear();
  GinvY.clear();
}

double GaussianProcessOptimized::evaluate(const arr& x) {
  double y, nonsense;
  evaluate(x, y, true, nonsense, false);
  return y;
}

double GaussianProcessOptimized::evaluateVariance(const arr& x) {
  double sig, nonsense;
  evaluate(x, nonsense, false, sig, true);
  return sig;
}

void GaussianProcessOptimized::evaluate(const arr& x, double& y, double& sig) {
  evaluate(x, y, true, sig, true);
}

void GaussianProcessOptimized::evaluate(const arr& x, double& y, bool calcY, double& sig, bool calcSig) {
  arr kappa;
  kappa = kernel->kappa(x, X);
  /*kappa.resize(X.d0);
  for(uint i = 0; i < X.d0; i++) {
    kappa(i) = kernel->k(X[i], x);
  }*/
  if(calcY) {
    y = scalarProduct(kappa, GinvY) + m;
  }
  if(calcSig) {
    arr v = lapack_Ainv_b_triangular(L, kappa);
    sig = sqrt(kernel->k(x, x) - scalarProduct(v, v));
  }
}

arr GaussianProcessOptimized::gradient(const arr& x) {
  arr grad = zeros(x.d0);
  //kernel->dk_dx(x, X[0]) * GinvY(0);
  for(uint i = 0; i < X.d0; i++) {
    grad += kernel->dk_dx(x, X[i])*GinvY(i);
  }
  return grad;
}

arr GaussianProcessOptimized::gradientVariance(const arr& x) {
  arr kappa;
  kappa = kernel->kappa(x, X);
  /*kappa.resize(X.d0);
  for(uint i = 0; i < X.d0; i++) {
    kappa(i) = kernel->k(x, X[i]);
  }*/

  arr dKappa;
  dKappa = kernel->dKappa(x, X);
  /*
  dKappa.resize(X.d0, X.d1);
  for(uint i = 0; i < X.d0; i++) {
    dKappa[i] = kernel->dk_dx(x, X[i]);
  }*/
  //this can be optimized to n^2 complexity, if kappa*L^-1 is solved first (need different lapack/blas routine). Lapack offers no such routine, but blas does (not directly, needs two calls).
  //return kernel->dk_dx(x,x) - 2.0*(~kappa*lapack_Ainv_b_symPosDef_givenCholesky(L, dKappa)).reshapeFlat();

  //this is the optimized, n^2 version, was quite easy :-)
  return kernel->dk_dx(x, x) - 2.0*(~lapack_Ainv_b_symPosDef_givenCholesky(L, kappa)*dKappa).reshapeFlat();
}

arr GaussianProcessOptimized::hessian(const arr& x) {
  arr hessian = zeros(x.d0, x.d0);
  //arr hessian = kernel->d2k_dx2(x, X[0]) * GinvY(0);
  for(uint i = 0; i < X.d0; i++) {
    hessian += kernel->d2k_dx2(x, X[i]) * GinvY(i);
  }
  return hessian;
}
