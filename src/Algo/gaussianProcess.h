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

//===========================================================================
//
/// @name Gaussian Process code
//

struct GaussianProcess {
  arr X, Y;   ///< data
  arr dX, dY; ///< derivative data
  uintA dI;  ///< derivative data (derivative indexes)
  arr Ginv, GinvY, ig2;  ///< inverse gram matrix (and a second buffer for push/pop)

  //--prior function
  double mu; ///< const bias of the GP
  double(*mu_func)(const arr& x, const void* param);  ///< prior of the GP (variable bias)
  void* priorP;

  double obsVar;

  //-- new covariance function naming
  // kernelF  = cov = covF_F
  // dkernelF = dcov
  // kernelD1 = covF_D
  // kernelD2 = covD_D
  // kernelD3 = covDD_D
  /* covariance between two function values */
  double(*cov)(void* P, const arr& a, const arr& b);
  double covF_F(void* P, const arr& a, const arr& b) { return cov(P, a, b); }
  /* gradient of covariance. other words \forall i covD_F(i, P, a, a) */
  void (*dcov)(arr& grad, void* P, const arr& a, const arr& b);
  /* covariance between derivative and function values */
  double(*covF_D)(uint e, void* P, const arr& a, const arr& b);
  double   covD_F(uint e, void* P, const arr& a, const arr& b) {return covF_D(e, P, b, a);}
  /* covariance between two derivatives */
  double(*covD_D)(uint e, uint l, void* P, const arr& a, const arr& b);
  /* covariance between 2nd derivative and function values */
  double(*covDD_F)(uint e, uint l, void* P, const arr& a, const arr& b);
  /* covariance between 2nd derivative and 1st derivative */
  double(*covDD_D)(uint e, uint l, uint s, void* P, const arr& a, const arr& b);

  void* kernelP;                      ///< pointer to parameters (a struct or so) passed to the kernel function

  GaussianProcess();

  GaussianProcess(const GaussianProcess& f) {
    X=f.X; Y=f.Y; dX=f.dX; dY=f.dY; dI=f.dI;
    Ginv=f.Ginv; GinvY=f.GinvY; ig2=f.ig2;
    mu=f.mu; mu_func=f.mu_func; priorP=f.priorP;
    cov=f.cov; dcov=f.dcov; covF_D=f.covF_D;
    covD_D=f.covD_D; covDD_F=f.covDD_F; covDD_D=f.covDD_D;
    kernelP=f.kernelP; obsVar=f.obsVar;
  }

  void clear() { X.clear(); Y.clear(); dX.clear(); dY.clear(); dI.clear(); Ginv.clear(); GinvY.clear(); ig2.clear(); }

  void copyFrom(GaussianProcess& f) {
    X=f.X; Y=f.Y; dX=f.dX; dY=f.dY; dI=f.dI;
    Ginv=f.Ginv; GinvY=f.GinvY; ig2=f.ig2;
    mu=f.mu; mu_func=f.mu_func; priorP=f.priorP;
    cov=f.cov; dcov=f.dcov; covF_D=f.covF_D;
    covD_D=f.covD_D; covDD_F=f.covDD_F; covDD_D=f.covDD_D;
    kernelP=f.kernelP; obsVar=f.obsVar;
  }

  /** set an arbitrary covariance function,
      P is a pointer to parameters (a struct) that is passed
      everytime when the _cov is called */
  void setKernel(double(*_cov)(void* P, const arr& x, const arr& y), void* _kernelP) {
    kernelP=_kernelP;
    cov=_cov;
    dcov=nullptr;
  }
  void setKernel(double(*_cov)(void* P, const arr& x, const arr& y), void (*_dcov)(arr& grad, void* P, const arr& x, const arr& y), void* _kernelP) {
    kernelP=_kernelP;
    cov=_cov;
    dcov=_dcov;
  }
  /** set a covariance function,
   * gradient of the cov,
   * cov of function value and
   * derivative,
   * cov of two derivatives.
   * cov function parameters (SDV rather than variance)
   */
  void setKernel(
    double(*_cov)(void*, const arr&, const arr&),
    void (*_dcov)(arr&, void*, const arr&, const arr&),
    double(*_covF_D)(uint, void*, const arr&, const arr&),
    double(*_covD_D)(uint, uint, void*, const arr&, const arr&),
    double(*_covDD_F)(uint, uint, void*, const arr&, const arr&),
    double(*_covDD_D)(uint, uint, uint, void*, const arr&, const arr&),
    void* _kernelP) {
    kernelP=_kernelP;
    cov=_cov;
    dcov=_dcov;
    covF_D=_covF_D;
    covD_D=_covD_D;
    covDD_F=_covDD_F;
    covDD_D=_covDD_D;
  }
  void setGaussKernelGP(void* _kernelP, double(*_mu)(const arr&, const void*), void*);
  void setGaussKernelGP(void* _kernelP, double _mu);

  void recompute(const arr& X, const arr& Y);             ///< calculates the inv Gram matrix for the given data
  void recompute();                                      ///< recalculates the inv Gram matrix for the current data
  void appendObservation(const arr& x, double y);     ///< add a new datum to the data and updates the inv Gram matrix
  void appendDerivativeObservation(const arr& x, double dy, uint i);
  void appendGradientObservation(const arr& x, const arr& dydx);

  void evaluate(const arr& x, double& y, double& sig, bool calcSig = true);   ///< evaluate the GP at some point - returns y and sig (=standard deviation)
  void evaluate(const arr& X, arr& Y, arr& S);   ///< evaluate the GP at some array of points - returns all y's and sig's
  double log_likelihood();
  double max_var(); // the variance when no data present
  void gradient(arr& grad, const arr& x);           ///< evaluate the gradient dy/dx of the mean at some point
  void hessianPos(arr& hess, const arr& x);            ///< evaluate the hessian dy/dx1dx2 of the mean at some point
  void gradientV(arr& grad, const arr& x); ///< variance gradient
  void k_star(const arr& x, arr& k);
  void dk_star(const arr& x, arr& k);

  void push(const arr& x, double y) { ig2=Ginv; appendObservation(x, y); recompute(); }
  void pop() { Ginv=ig2; X.resizeCopy(X.d0-1, X.d1); Y.resizeCopy(Y.N-1); }
};

#define KRONEKER(a, b)   ( ((a)==(b)) ? 1 : 0 )

//===========================================================================
//
/// @name standard Gaussian covariance function
//

struct GaussKernelParams {
  double priorVar, widthVar, derivVar;
  GaussKernelParams() { priorVar=.01; widthVar=.04; derivVar=.01; }
  GaussKernelParams(double _priorSDV, double _widthSDV, double _derivSDV) { priorVar=_priorSDV*_priorSDV; widthVar=_widthSDV*_widthSDV; derivVar=_derivSDV*_derivSDV;}
};

/// you can also pass a double[3] as parameters
/* covariance between functionvalues at \vec a and \vec b */
inline double GaussKernel(void* P, const arr& a, const arr& b) {
  GaussKernelParams& K = *((GaussKernelParams*)P);
  if((&a==&b) || operator==(a, b))
    return K.priorVar;
  double d;
  if(a.N!=1) d=sqrDistance(a, b); else { d=b(0)-a(0); d=d*d; }
  return K.priorVar*::exp(-.5 * d/K.widthVar);
}

/** @brief return gradient of the covariance function, i.e.
  for i \in {vector dimensions}: \dfdx{k(a, b)}{x_i}  w.r.t.
  In other words: for all components invoke covD_F(i, P, a, b)
  you can also pass a double[3] as parameters */
inline void dGaussKernel(arr& grad, void* P, const arr& a, const arr& b) {
  GaussKernelParams& K = *((GaussKernelParams*)P);
  if(&a==&b) { grad.resizeAs(a); grad.setZero(); return; }
  double gauss=GaussKernel(P, a, b), gamma=1./K.widthVar;
  grad = gamma * (b-a) * gauss; // SD: Note the (b - a) swap cancles the leading minus
  //RAI_MSG("gamma=" <<gamma <<"; b-a" <<b -a <<"; gauss=" <<gauss<<"; grad=" <<grad);
}

/** @brief covariance between derivative at point a and function value at
 * point b
  you can also pass a double[3] as parameters */
inline double GaussKernelF_D(uint e, void* P, const arr& a, const arr& b) {
  GaussKernelParams& K = *((GaussKernelParams*)P);
  if(&a==&b) { HALT("this shouldn't happen, I think"); }
  double gauss=GaussKernel(P, a, b), gamma=1./K.widthVar;
  double de=a(e)-b(e);
  return gamma * de * gauss;
}

/** @brief covariance between derivatives at points \vec a and \vec b
  you can also pass a double[3] as parameters */
inline double GaussKernelD_D(uint e, uint l, void* P, const arr& a, const arr& b) {
  GaussKernelParams& K = *((GaussKernelParams*)P);
  if(&a==&b) return K.priorVar/K.widthVar + K.derivVar;
  double gauss=GaussKernel(P, a, b), gamma=1./K.widthVar;
  double de=a(e)-b(e), dl=a(l)-b(l);
  return gamma *(KRONEKER(e, l) - gamma*de*dl) * gauss;
}

/** @brief covariance between 2nd derivative at \vec a and fun value at \vec b
  you can also pass a double[3] as parameters */
inline double GaussKernelDD_F(uint e, uint l, void* P, const arr& a, const arr& b) {
  return - GaussKernelD_D(e, l, P, a, b);
}

/** @brief covariance between second derivative at point \vec a and  1st
 * derivative at \vec b
  you can also pass a double[3] as parameters */
inline double GaussKernelDD_D(uint e, uint l, uint s, void* P, const arr& a, const arr& b) {
  GaussKernelParams& K = *((GaussKernelParams*)P);
  if(&a==&b) return K.priorVar/K.widthVar + K.derivVar;
  double gauss=GaussKernel(P, a, b), gamma=1./K.widthVar;
  arr d=a-b;
  return gamma*gamma*(
           -KRONEKER(l, s)*d(e)
           -KRONEKER(e, l)*d(s)
           -KRONEKER(e, s)*d(l)
           + gamma*d(l)*d(s)*d(e)
         )*gauss;
}

inline double maximizeGP(GaussianProcess& gp, arr& x) {
  NIY;
  /*
     CHECK(gp.Y.N, "");
     if(!x.N){ x.resize(gp.X.d1); x=0.; } //rndGauss(x, 1.); }
     arr dx;
     Rprop rp;
     rp.init(x.N, .1);
     uint t=0;
     do{
     gp.gradient(dx, x);
     dx *= -1.;
     rp.learn(x, dx);
     t++;
     if(t>100){ cout <<" couldn't stop!"; break; }
     }while(!rp.fine());
     double y, s;
     gp.evaluate(x, y, s);
     cout <<"GP maximization: " <<t <<"iterations, x=" <<x <<" y=" <<y <<" s=" <<s <<endl;
     return y;
   */
}

void plotBelief(GaussianProcess& gp, double lo, double hi, bool pause=true);
void plotKernel1D(GaussianProcess& gp, double lo, double hi, bool pause=true);
void plotKernel2D(GaussianProcess& gp, double lo, double hi, bool pause=true);

inline void randomFunction(GaussianProcess& gp, arr& Xbase, bool illustrate, bool fromPosterior=false) {
  double orgObsVar=gp.obsVar;
  gp.obsVar=1e-6;

  arr x;
  double y, sig;
  uint i;
  //gp.setKernel(&stdKernel, GaussKernel);
  if(!fromPosterior) gp.clear();

//  Xbase.randomPermute();
  //gp.X.resize(0, 1); gp.Y.resize(0); //clear current data
  for(i=0; i<Xbase.d0; i++) {
    if(illustrate && i>0 && !(i%10)) { //display
      plotBelief(gp, min(Xbase), max(Xbase));
    }

    x.referToDim(Xbase, i); //get next input point
    gp.evaluate(x, y, sig);      //sample it from the GP itself
    y+=sig*rnd.gauss();        //with standard deviation..
    gp.appendObservation(x, y);
    gp.recompute();
  }

  gp.obsVar=orgObsVar;
}
