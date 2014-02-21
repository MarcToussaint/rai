#include "gaussianProcess.h"
#include <Gui/plot.h>

void plotBelief(GaussianProcess& gp, double lo, double hi, bool pause){
  arr X, Y, Z, S;
  uint dim;
  //there should be at least 1 observation to guess the dimensionality from
  dim = gp.X.d1 ? gp.X.d1 : gp.dX.d1;
  CHECK(dim > 0, "still no data here. I have no clue about dimensionality!?!");
  
  X.setGrid(dim, lo, hi, 10000);
  gp.evaluate(X, Y, S);
  plotClear();
  switch(dim){
    case 1:
      plotFunctionPrecision(X, Y, Y+S, Y-S);
      //plotFunction(X, Y);
      //plotFunction(X, Y+S);
      //plotFunction(X, Y-S);
      plotPoints(gp.X, gp.Y);
      plotPoints(gp.dX, gp.dY);
      break;
    case 2:
      plotFunction(X, Y);
      plotFunction(X, Y+S);
      plotFunction(X, Y-S);
      plotPoints(gp.X, gp.Y);
      plotPoints(gp.dX, gp.dY);
      break;
    default :
      HALT("Space is either 0- or higher than 3-dimensional. Tell me how to plot that!")
      break;
  }
  plot(pause);
}

void plotKernel1D(GaussianProcess& gp, double lo, double hi, bool pause){
  arr X, K, KD1, KD2;
  X.setGrid(1, lo, hi, 600);
  K.resize(X.d0);
  KD1.resize(X.d0);
  KD2.resize(X.d0);
  arr null=ARR(0);
  for(uint i=0; i<X.d0; i++){
    K(i) = gp.cov(gp.kernelP, null, X[i]);
    KD1(i) = gp.covF_D(0, gp.kernelP, null, X[i]);
    KD2(i) = gp.covDD_F(0, 0, gp.kernelP, X[i], null);
  }
  plotClear();
  plotFunction(X, K);
  plotFunction(X, KD1);
  plotFunction(X, KD2);
  plot(pause);
}

void plotKernel2D(GaussianProcess& gp, double lo, double hi, bool pause){
  arr X, K, KD1, KD2;
  X.setGrid(2, lo, hi, 1000);
  K.resize(X.d0, X.d1);
  KD1.resize(X.d0, X.d1);
  KD2.resize(X.d0, X.d1);
  arr null=ARR(0);
  for(uint i=0; i<X.d0; i++){
    for(uint j=0; j<X.d1; j++){
      K(i, j) = gp.cov(gp.kernelP, null, X[i]);
      KD1(i, j) = gp.covF_D(0, gp.kernelP, null, X[i]);
      KD2(i, j) = gp.covDD_F(0, 0, gp.kernelP, X[i], null);
    }
  }
  plotClear();
  plotSurface(K);
  plotSurface(KD1);
  plotSurface(KD2);
  plot(pause);
}
