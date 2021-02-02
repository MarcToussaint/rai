/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "gaussianProcess.h"

#include "../Plot/plot.h"

void plotBelief(GaussianProcess& gp, double lo, double hi, bool pause) {
  arr X, Y, Z, S;
  uint dim;
  //there should be at least 1 observation to guess the dimensionality from
  dim = gp.X.d1 ? gp.X.d1 : gp.dX.d1;
  CHECK(dim > 0, "still no data here. I have no clue about dimensionality!?!");

  X.setGrid(dim, lo, hi, 100);
  gp.evaluate(X, Y, S);
  plot()->Clear();
  switch(dim) {
    case 1:
      plot()->FunctionPrecision(X, Y, Y+S, Y-S);
      //plot()->Function(X, Y);
      //plot()->Function(X, Y+S);
      //plot()->Function(X, Y-S);
      plot()->Points(gp.X, gp.Y);
      plot()->Points(gp.dX, gp.dY);
      break;
    case 2:
      //plot()->Function(X, Y);
      //plot()->Function(X, Y+S);
      //plot()->Function(X, Y-S);
      plot()->Points(gp.X, gp.Y);
      plot()->Points(gp.dX, gp.dY);
      break;
    default :
      HALT("Space is either 0- or higher than 3-dimensional. Tell me how to plot that!")
      break;
  }
  plot()->update(pause);
}

void plotKernel1D(GaussianProcess& gp, double lo, double hi, bool pause) {
  arr X, K, KD1, KD2;
  X.setGrid(1, lo, hi, 600);
  K.resize(X.d0);
  KD1.resize(X.d0);
  KD2.resize(X.d0);
  arr null=ARR(0.);
  for(uint i=0; i<X.d0; i++) {
    K(i) = gp.cov(gp.kernelP, null, X[i]);
    KD1(i) = gp.covF_D(0, gp.kernelP, null, X[i]);
    KD2(i) = gp.covDD_F(0, 0, gp.kernelP, X[i], null);
  }
  plot()->Clear();
  plot()->Function(X, K);
  plot()->Function(X, KD1);
  plot()->Function(X, KD2);
  plot()->update(pause);
}

void plotKernel2D(GaussianProcess& gp, double lo, double hi, bool pause) {
  arr X, K, KD1, KD2;
  X.setGrid(2, lo, hi, 1000);
  K.resize(X.d0, X.d1);
  KD1.resize(X.d0, X.d1);
  KD2.resize(X.d0, X.d1);
  arr null=ARR(0.);
  for(uint i=0; i<X.d0; i++) {
    for(uint j=0; j<X.d1; j++) {
      K(i, j) = gp.cov(gp.kernelP, null, X[i]);
      KD1(i, j) = gp.covF_D(0, gp.kernelP, null, X[i]);
      KD2(i, j) = gp.covDD_F(0, 0, gp.kernelP, X[i], null);
    }
  }
  plot()->Clear();
  plot()->Surface(K);
  plot()->Surface(KD1);
  plot()->Surface(KD2);
  plot()->update(pause);
}
