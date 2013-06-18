/*  ---------------------------------------------------------------------
    Copyright 2013 Marc Toussaint
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

/// @file
/// @ingroup group_Optim
/// @addtogroup group_Optim
/// @{

#ifndef MT_optimization_h
#define MT_optimization_h

#include <Core/array.h>

// function evaluation counter (used only for performance meassurements, global for simplicity)
extern uint eval_cost;


//===========================================================================
//
// functions that imply (optimization) problems
//

//-- return types
struct     SqrPotential { arr A, a;          double c; }; ///< return type representing \f$x'A x - 2x'a + c\f$
struct PairSqrPotential { arr A, B, C, a, b; double c; }; ///< return type representing \f$(x,y)' [A C, C' B] (x,y) - 2(x,y)'(a,b) + c\f$
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
  virtual arr get_prefix() { return NoArr; }
  
  //optional: kernel costs
  virtual bool hasKernel() { return false; }
  virtual double kernel(uint t0,uint t1) { NIY; } ///< a kernal add additional costs: neg-log-likelihoods of a Gaussian Process
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
struct Convert {
  struct sConvert* s;
  Convert(ScalarFunction&);
  Convert(VectorFunction&);
  Convert(QuadraticFunction&);
  Convert(VectorChainFunction&);
  Convert(QuadraticChainFunction&);
  Convert(KOrderMarkovFunction&);
//  Convert(struct ControlledSystem&);
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
bool checkJacobian(VectorFunction &f, const arr& x, double tolerance);
bool checkHessian(ScalarFunction &f, const arr& x, double tolerance);
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

struct OptOptions {
  uint verbose;
  double *fmin_return;
  double stopTolerance;
  uint   stopEvals;
  uint   stopIters;
  double initStep;
  double minStep;
  double maxStep;
  double damping;
  bool useAdaptiveDamping;
  bool clampInitialState;
  OptOptions();
};


uint optGaussNewton(arr& x, VectorFunction& phi, OptOptions opt, arr *addRegularizer=NULL, arr *fx_user=NULL, arr *Jx_user=NULL);
uint optNewton(arr& x, ScalarFunction& f, OptOptions opt, double *f_user=NULL, arr *g_user=NULL, arr *H_user=NULL);
uint optRprop(arr& x, ScalarFunction& f, OptOptions opt);
uint optGradDescent(arr& x, ScalarFunction& f, OptOptions opt);
uint optDynamicProgramming(arr& x, QuadraticChainFunction& f, OptOptions opt);
/// preliminary
uint optNodewise(arr& x, VectorChainFunction& f, OptOptions opt);
/// preliminary
uint optMinSumGaussNewton(arr& x, QuadraticChainFunction& f, OptOptions opt);


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


//===========================================================================
//
// Named Parameters: Macros for the OPT
//

extern OptOptions globalOptOptions;
//#define OPT0() (globalOptOptions)
//#define OPT1(a) (globalOptOptions.a, globalOptOptions)
//#define OPT2(a,b) (globalOptOptions.a, globalOptOptions.b, globalOptOptions)
//#define OPT3(a,b,c) (globalOptOptions.a, globalOptOptions.b, globalOptOptions.c, globalOptOptions)
//#define OPT4(a,b,c,d) (globalOptOptions.a, globalOptOptions.b, globalOptOptions.c, globalOptOptions.d, globalOptOptions)
//#define OPT5(a,b,c,d,e) (globalOptOptions.a, globalOptOptions.b, globalOptOptions.c, globalOptOptions.d, globalOptOptions.e, globalOptOptions)
//#define OPT6(a,b,c,d,e,f) (globalOptOptions.a, globalOptOptions.b, globalOptOptions.c, globalOptOptions.d, globalOptOptions.e, globalOptOptions.f, globalOptOptions)

#define _NUM_ARGS2(X,X64,X63,X62,X61,X60,X59,X58,X57,X56,X55,X54,X53,X52,X51,X50,X49,X48,X47,X46,X45,X44,X43,X42,X41,X40,X39,X38,X37,X36,X35,X34,X33,X32,X31,X30,X29,X28,X27,X26,X25,X24,X23,X22,X21,X20,X19,X18,X17,X16,X15,X14,X13,X12,X11,X10,X9,X8,X7,X6,X5,X4,X3,X2,X1,N,...) N
#define _NUM_ARGS(...) _NUM_ARGS2(0,1, __VA_ARGS__ ,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0)

#define _OPT_1(obj)         obj
#define _OPT_2(obj, assign) obj.assign
#define _OPT_3(obj, assign, ...) obj.assign, _OPT_2(obj,__VA_ARGS__)
#define _OPT_4(obj, assign, ...) obj.assign, _OPT_3(obj,__VA_ARGS__)
#define _OPT_5(obj, assign, ...) obj.assign, _OPT_4(obj,__VA_ARGS__)
#define _OPT_6(obj, assign, ...) obj.assign, _OPT_5(obj,__VA_ARGS__)
#define _OPT_7(obj, assign, ...) obj.assign, _OPT_6(obj,__VA_ARGS__)
#define _OPT_8(obj, assign, ...) obj.assign, _OPT_7(obj,__VA_ARGS__)
#define _OPT_9(obj, assign, ...) obj.assign, _OPT_8(obj,__VA_ARGS__)
#define _OPT_N2(obj, N, ...) _OPT_ ## N(obj, __VA_ARGS__)
#define _OPT_N1(obj, N, ...) _OPT_N2(obj, N, __VA_ARGS__) //this forces that _NUM_ARGS(...) is expanded to a number N
#define OPT(...)     (_OPT_N1(globalOptOptions, _NUM_ARGS(__VA_ARGS__), __VA_ARGS__) , globalOptOptions)

#ifdef  MT_IMPLEMENTATION
#  include "optimization.cpp"
#endif

#endif

/// @} //end group
