/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "array.h"
#include "util.h"

namespace rai {

void normalizeData(arr& X);

//----- spline stuff
void makeSpline(arr& X, arr& P, uint intersteps);
void makeSpline(arr& X, arr& V, arr& P, uint intersteps);
void randomSpline(arr& X, uint dim, uint points, uint intersteps=100, double lo=-1., double hi=1., uint cycles=1);
void randomSpline(arr& X, arr& dX, uint dim, uint points, uint intersteps=100, double lo=-1., double hi=1., uint cycles=1);

//----- gradient optimization
bool checkGradient(double(*f)(arr*, const arr&, void*),  //double valued with optional gradient
                   void* data,
                   const arr& x, double tolerance);
bool checkGradient(void (*f)(arr&, arr*, const arr&, void*), //vector valued with optional gradient
                   void* data,
                   const arr& x, double tolerance);
//obsolete:
/*void checkGradient(void (*f)(arr&, const arr&, void*),
                   void (*df)(arr&, const arr&, void*),
                   void *data,
                   const arr& x, double tolerance);
void checkGradient(double(*f)(const arr&, void*),
                   void (*df)(arr&, const arr&, void*),
                   void *data,
                   const arr& x, double tolerance);*/
int minimize(double(*f)(arr*, const arr&, void*),
             void* data,
             arr& x,
             double* fmin_return,
             int method,
             uint maxIterations,
             double stoppingTolerance,
             bool chkGrad);

//----- LU decomposition
double determinant_LU(const arr& X);
void inverse_LU(arr& Xinv, const arr& X);
void LU_decomposition(arr& L, arr& U, const arr& X);

//----- Runge-Kutta
/// standard Runge-Kutta 4
void rk4(arr& x1, const arr& x0,
         void (*df)(arr& xd, const arr& x),
         double dt);
/// same for second order diff equation
void rk4dd(arr& x1, arr& v1, const arr& x0, const arr& v0,
           void (*ddf)(arr& xdd, const arr& x, const arr& v),
           double dt);
/** RK with discrete event localization (zero-crossing detection): the
    function sf computes some double-valued indicators. If one of
    these indicators crosses zero this is interpreted as a
    discontinuity in the dynamics. The algorithm iteratively tries to
    find the zero-crossing point up to a tolerance tol (measured in
    time). The routine returns false for no-switching and true and the
    executed time step dt in the case of zero-crossing */
bool rk4_switch(arr& x1, arr& s1, const arr& x0, const arr& s0,
                void (*df)(arr& xd, const arr& x),
                void (*sf)(arr& s, const arr& x),
                double& dt, double tol);
/// same for 2nd order DEs
bool rk4dd_switch(arr& x1, arr& v1, arr& s1, const arr& x0, const arr& v0, const arr& s0,
                  void (*ddf)(arr& xdd, const arr& x, const arr& v),
                  void (*sf)(arr& s, const arr& x, const arr& v),
                  double& dt, double tol);

//bandpass filtering
void convolution(arr& y, const arr& x, double(*h)(double), double scale=1.);
void bandpassFilter(arr& y, const arr& x, double loWavelength, double hiWavelength);
void bandpassEnergy(arr& y, const arr& x, double loWavelength, double hiWavelength);

//----- comparing connectivity matrices
double matdistance(intA& fix, intA& fox, uintA& p, bool sub);
double matdistance(intA& A, intA& B, bool sub);
double matannealing(intA& fix, intA& fox, uintA& p, bool sub, double annealingRepetitions, double annealingCooling);
}

//===========================================================================
//
// MonSolver
//

/** a trivial solver for monotonic unimodal 1D functions */
class MonSolver {
 public:
  double min, max;
  int phase;
  MonSolver();
  void init(double& par, double wide=2.);
  void solve(double& par, const double& err);
};

//===========================================================================
//
// Linear Statistics
//

/** collects means and (co-)variances of data, also input-output data.
    This class is the basis for linear regression techniques, like
    Least Squares, Partial Least Squares, but can also be used on its
    own, simply to average or calculate (co-)variances \ingroup
    regression */
class LinearStatistics {
 public:
  double accum;  ///< the accumulation norm (=number of collected data points if not weighted)
  arr meanX, meanY, varX, covXY; //<! these are not normalized or centered bufferes
  arr MeanX; ///< X mean
  arr MeanY; ///< Y mean
  arr VarX;  ///< X variance
  arr CovXY; ///< XY covariance
  double lambda; ///< forgetting rate [default=0]
  bool computed; ///< internal indicator whether recomputation is needed

  LinearStatistics();

  //feed data
  void learn(const arr& x, const arr& y, double weight=1.);
  void learn(const arr& x, double y, double weight);
  void learn(const arr& x);
  void learn(const arr& x, double weight);
  void clear();
  void forget(double lambda=1.);

  //get information
  uint inDim();
  double variance();
  void correlationX(arr& corr, bool clearDiag=false);
  void correlationXY(arr& corr, bool clearDiag=false);
  void mahalanobisMetric(arr& metric);
  void regressor(arr& A);
  void regressor(arr& A, arr& a);
  void predict(const arr& x, arr& y);

  //used internally
  void compute();
  void computeZeroMean();

  //output
  void write(std::ostream& os) const;
};
stdOutPipe(LinearStatistics);

//===========================================================================
//
// Tuple index
//

class TupleIndex:public uintA {
 public:
  uintA tri;
  void init(uint k, uint n);
  uint index(uintA i);
  void checkValid();
};

//===========================================================================
//
// Kalman filter
//

/** a Kalman filter */
class Kalman {
  arr
  A, a, //linear forward transition
  Q, //covariance of forward transition x(t) = A*x(t-1) + \NN(0, Q)
  C, //linear observation matrix
  R; //covariance of observation: y = C*x + \NN(0, R)

  void setTransitions(uint d, double varT, double varO);
  void filter(arr& Y, arr& X, arr& V, arr* Rt=0);
  void smooth(arr& Y, arr& X, arr& V, arr* Vxx=0, arr* Rt=0);
  void EMupdate(arr& Y, arr* Rt=0);
  void fb(arr& y, arr& f, arr& F, arr& g, arr& G, arr& p, arr& P, arr* Rt=0);
};

//===========================================================================
//
// X-Splines
//

class XSpline {
 public:
  double DELTA;     // Distance between each

  arr V; //vertices
  arr W; //weights associated to each vertex

  XSpline();
  ~XSpline();

  void setWeights(double w);
  void type(bool hit, double smooth);
  void referTo(arr& P);
  arr eval(double t);
  void eval(double t, arr& x, arr* v=0);
  void eval(double t, arr& x, arr& v);
};

//===========================================================================
//
// Partial Least Squares (PLS, SIMPLS, de Jong)
//

/** An implementation of Partial Least Squares following the SIMPLS algorithm (de Jong).
    This PLS implementation finds an optimal linear regression (from n to m dimensions)
    by first calculating input projections `with higest correlation to the output'. This
    implementation uses the Singular Value Decomposition routine rai::svd and
    the LinearStatistics. \ingroup regression */
class PartialLeastSquares {
 public:
  LinearStatistics S; //<! the statistics collected with learning

  arr W, Q, B; //the transformation matricies

  arr resErr; //the residual errors

  rai::Parameter<uint> maxProj;

  PartialLeastSquares():maxProj("PLSmaxProjections", 0) { }

  //feed data
  void learn(const arr& x, const arr& y, double weight=1.);
  void learn(const arr& x, double y, double weight=1.);
  void clear();

  //access
  void map(const arr& x, arr& y);
  void map(const arr& x, double& y);
  double map(const arr& x);
  arr projection(uint k);
  uint inDim();
  uint outDim();

  void write(std::ostream& os) const;

  //internally used
  void SIMPLS();
};
