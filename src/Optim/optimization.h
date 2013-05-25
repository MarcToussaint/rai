/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
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

/** @file
 * @ingroup group_optim
 * @addtogroup group_optim
 * @{ */

#ifndef MT_optimization_h
#define MT_optimization_h

#include <Array/array.h>
#include <Array/util.h>

// function evaluation counter (used only for performance meassurements, global for simplicity)
extern uint eval_cost;


//===========================================================================
//
// functions that imply (optimization) problems
//

//-- return types
struct     SqrPotential { arr A, a;          double c; }; ///< return type representing \f$x'A x - 2x'a + c\f$
struct PairSqrPotential { arr A, B, C, a, b; double c; }; ///< return type representing \f$(x,y)' [A C, C' B] (x,y) - 2(x,y)'(a,b) + c\f$
extern arr& NoGrad; ///< reference to NULL! used for optional arguments
extern SqrPotential& NoPot; ///< reference to NULL! used for optional arguments
extern PairSqrPotential& NoPairPot; ///< reference to NULL! used for optional arguments

/// a scalar function \f$f:~x\mapsto y\in\mathbb{R}\f$ with optional gradient and hessian
struct ScalarFunction { virtual double fs(arr& g, arr& H, const arr& x) = 0; };

/// a vector function \f$f:~x\mapsto y\in\mathbb{R}^d\f$ with optional Jacobian
struct VectorFunction { virtual void   fv(arr& y, arr& J, const arr& x) = 0; };

/// a locally quadratic function TODO: replace by scalar with hessian!
struct QuadraticFunction { virtual double fq(SqrPotential& S, const arr& x) = 0; };

/// functions \f$f_i:x_i \mapsto y\f$ and \f$f_{ij}: x_i,x_j \mapsto y\f$ over a chain \f$x_0,..,x_T\f$ of variables
struct VectorChainFunction {
  virtual uint get_T() = 0;
  virtual void fv_i(arr& y, arr& J, uint i, const arr& x_i) = 0;
  virtual void fv_ij(arr& y, arr& Ji, arr& Jj, uint i, uint j, const arr& x_i, const arr& x_j) = 0;
};

struct QuadraticChainFunction {
  virtual uint get_T() = 0;
  virtual double fq_i(SqrPotential& S, uint i, const arr& x_i) = 0;
  virtual double fq_ij(PairSqrPotential& S, uint i, uint j, const arr& x_i, const arr& x_j) = 0;
};

/// functions \f$\phi_t:(x_{t-k},..,x_t) \mapsto y\f$ over a chain \f$x_0,..,x_T\f$ of variables
struct KOrderMarkovFunction { //TODO: rename KOrderChainFunction
  virtual void phi_t(arr& phi, arr& J, uint t, const arr& x_bar) = 0;

  //functions to get the parameters $T$, $k$ and $n$ of the $k$-order Markov Process
  virtual uint get_T() = 0;
  virtual uint get_k() = 0;
  virtual uint get_n() = 0; ///< the dimensionality of \f$x_t\f$
  virtual uint get_m(uint t) = 0; ///< the dimensionality of \f$\phi_t\f$
  virtual arr get_prefix(){ return NoArr; }

  //optional: kernel costs
  virtual bool hasKernel(){ return false; }
  virtual double kernel(uint t0,uint t1){ NIY; } ///< a kernal add additional costs: neg-log-likelihoods of a Gaussian Process
};

/*
struct VectorGraphFunction {
  virtual uintA edges() = 0;
  virtual double fi (arr* grad, uint i, const arr& x_i) = 0;
  virtual double fij(arr* gradi, arr* gradj, uint i, uint j, const arr& x_i, const arr& x_j) = 0;
  double f_total(const arr& X);
};*/


//===========================================================================
//
// converter
//

/// A struct that allows to convert one function type into another, even when given as argument
struct Convert{
  struct sConvert* s;
  Convert(ScalarFunction&);
  Convert(VectorFunction&);
  Convert(QuadraticFunction&);
  Convert(VectorChainFunction&);
  Convert(QuadraticChainFunction&);
  Convert(KOrderMarkovFunction&);
  Convert(struct ControlledSystem&);
  ~Convert();
  operator ScalarFunction&();
  operator VectorFunction&();
  operator VectorChainFunction&();
  operator QuadraticChainFunction&();
  operator KOrderMarkovFunction&();
};


//===========================================================================
//
// checks, evaluation
//

bool checkGradient(ScalarFunction &f, const arr& x, double tolerance);
bool checkHessian(ScalarFunction &f, const arr& x, double tolerance); //TODO! NIY!
bool checkJacobian(VectorFunction &f, const arr& x, double tolerance);
bool checkDirectionalGradient(ScalarFunction &f, const arr& x, const arr& delta, double tolerance);
bool checkDirectionalJacobian(VectorFunction &f, const arr& x, const arr& delta, double tolerance);

//these directly simply evaluate squared potentials at some point
double evaluateSP(const SqrPotential& S, const arr& x);
double evaluatePSP(const PairSqrPotential& S, const arr& x, const arr& y);
double evaluateCSP(const MT::Array<SqrPotential>& fi, const MT::Array<PairSqrPotential>& fij, const arr& x);

//these actually call the functions (->query cost) to evalute them at some point
double evaluateSF(ScalarFunction& f, const arr& x);
double evaluateVF(VectorFunction& f, const arr& x);
double evaluateVCF(VectorChainFunction& f, const arr& x);
double evaluateQCF(QuadraticChainFunction& f, const arr& x);


//===========================================================================
//
// optimization methods
//

struct optOptions {
  double *fmin_return;
  double stopTolerance;
  uint   stopEvals;
  uint   stopIters;
  double useAdaptiveDamping;
  double initStep;
  double minStep;
  double maxStep;
  bool clampInitialState;
  uint verbose;
  optOptions();
};

extern optOptions globalOptOptions;
#define OPT0() (globalOptOptions)
#define OPT1(a) (globalOptOptions.a, globalOptOptions)
#define OPT2(a,b) (globalOptOptions.a, globalOptOptions.b, globalOptOptions)
#define OPT3(a,b,c) (globalOptOptions.a, globalOptOptions.b, globalOptOptions.c, globalOptOptions)
#define OPT4(a,b,c,d) (globalOptOptions.a, globalOptOptions.b, globalOptOptions.c, globalOptOptions.d, globalOptOptions)
#define OPT5(a,b,c,d,e) (globalOptOptions.a, globalOptOptions.b, globalOptOptions.c, globalOptOptions.d, globalOptOptions.e, globalOptOptions)
#define OPT6(a,b,c,d,e,f) (globalOptOptions.a, globalOptOptions.b, globalOptOptions.c, globalOptOptions.d, globalOptOptions.e, globalOptOptions.f, globalOptOptions)

uint optGaussNewton(arr& x, VectorFunction& phi, optOptions opt, arr *addRegularizer=NULL, arr *fx_user=NULL, arr *Jx_user=NULL);
uint optNewton(arr& x, QuadraticFunction& f, optOptions opt, double *fx_user=NULL, SqrPotential *Sx_user=NULL);
uint optRprop(arr& x, ScalarFunction& f, optOptions opt);
uint optGradDescent(arr& x, ScalarFunction& f, optOptions opt);
uint optDynamicProgramming(arr& x, QuadraticChainFunction& f, optOptions opt);
/// preliminary
uint optNodewise(arr& x, VectorChainFunction& f, optOptions opt);
/// preliminary
uint optMinSumGaussNewton(arr& x, QuadraticChainFunction& f, optOptions opt);


//===========================================================================
//
// Rprop
//

/** Rprop, a fast gradient-based minimization */
struct Rprop {
  struct sRprop *s;
  Rprop();
  void init(double initialStepSize=1., double minStepSize=1e-6, double maxStepSize=50.);
  bool step(arr& x, ScalarFunction& f);
  uint loop(arr& x, ScalarFunction& f, double *fmin_return=NULL, double stoppingTolerance=1e-2, double initialStepSize=1., uint maxIterations=1000, uint verbose=0);
};



#ifdef  MT_IMPLEMENTATION
#  include "optimization.cpp"
#endif

#endif

/** @} */
