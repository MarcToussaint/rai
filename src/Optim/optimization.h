/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
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
// functions that imply optimization problems
//

/** NOTE: Why do I define these virtual function with so many arguments to return f, J, and constraints all at once,
 * instead of having nice individual methods to return those?
 *
 * 1) Because I want these problem definitions (classes) to be STATE-LESS. That is, there must not be a set_x(x); before a get_f();
 *    I really have (bad) experience with non-stateless problem definitions.
 *
 * 2) Because the computation of quantities is expensive and it is usually most efficient to compute all needed quantities together
 *    (Instead of calling get_f(x); and then get_J(x); separately)
 *
 * All these methods allow some returns to be optional: use NoArr
 *
 */

#if 0 //these are already defined in array.h
/// a scalar function \f$f:~x\mapsto y\in\mathbb{R}\f$ with optional gradient and hessian
struct ScalarFunction {
  virtual double fs(arr& g, arr& H, const arr& x) = 0;
  virtual ~ScalarFunction(){}
};

/// a vector function \f$f:~x\mapsto y\in\mathbb{R}^d\f$ with optional Jacobian
struct VectorFunction {
  virtual void fv(arr& y, arr& J, const arr& x) = 0; ///< returning a vector y and (optionally, if !NoArr) Jacobian J for x
  virtual ~VectorFunction(){}
};
#endif

struct ConstrainedProblem {
  /// returns \f$f(x), \nabla f(x), \nabla^2 f(x), g(x), \nabla g(x)\f$ (giving NoArr as argument -> do not return this quantity)
  virtual double fc(arr& df, arr& Hf, arr& g, arr& Jg, const arr& x) = 0;
  virtual uint dim_x() = 0; ///< returns \f$ \dim(x) \f$
  virtual uint dim_g() = 0; ///< returns \f$ \dim(g) \f$

  virtual ~ConstrainedProblem(){}
};

/// functions \f$ \phi_t:(x_{t-k},..,x_t) \mapsto y\in\mathbb{R}^{m_t} \f$ over a chain \f$x_0,..,x_T\f$ of variables
struct KOrderMarkovFunction {
  /// returns $\f\phi(x), \nabla \phi(x)\f$ for a given time step t and a k+1 tuple of states \f$\bar x = (x_{t-k},..,x_t)\f$.
  /// This defines the cost function \f$f_t = \phi_t^\top \phi_t\f$ in the time slice. Optionally, the last dim_g entries of
  ///  \f$\phi\f$ are interpreted as inequality constraint function \f$g(\bar x)\f$ for time slice t
  virtual void phi_t(arr& phi, arr& J, uint t, const arr& x_bar) = 0;
  
  //functions to get the parameters $T$, $k$ and $n$ of the $k$-order Markov Process
  virtual uint get_T() = 0;       ///< horizon (the total x-dimension is (T+1)*n )
  virtual uint get_k() = 0;       ///< the order of dependence: \f$ \phi=\phi(x_{t-k},..,x_t) \f$
  virtual uint dim_x() = 0;       ///< \f$ \dim(x_t) \f$
  virtual uint dim_z(){ return 0; } ///< \f$ \dim(z) \f$
  virtual uint dim_phi(uint t) = 0; ///< \f$ \dim(\phi_t) \f$
  virtual uint dim_g(uint t){ return 0; } ///< number of inequality constraints at the end of \f$ \phi_t \f$
  virtual arr get_prefix(){ arr x(get_k(), dim_x()); x.setZero(); return x; } ///< the augmentation \f$ (x_{t=-k},..,x_{t=-1}) \f$ that makes \f$ \phi_{0,..,k-1} \f$ well-defined
  virtual arr get_postfix(){ return arr(); } ///< by default there is no definite final configuration

  /// optional: we make include kernel costs \f$ \sum_{i,j} k(i,j) x_i^\top x_j \f$ -- PRELIM, see examples/kOrderMarkov
  virtual bool hasKernel() { return false; }
  virtual double kernel(uint t0, uint t1) { NIY; } ///< a kernel adds additional costs: neg-log-likelihoods of a Gaussian Process

  virtual ~KOrderMarkovFunction(){}
};


//===========================================================================
//
// converter
//

/// A struct that allows to convert one function type into another, even when given as argument
struct Convert {
  struct sConvert* s;
  Convert(ScalarFunction&);
  Convert(VectorFunction&);
  Convert(KOrderMarkovFunction&);
  Convert(double(*fs)(arr*, const arr&, void*),void *data);
  Convert(void (*fv)(arr&, arr*, const arr&, void*),void *data);
  ~Convert();
  operator ScalarFunction&();
  operator VectorFunction&();
  operator ConstrainedProblem&();
  operator KOrderMarkovFunction&();
};


//===========================================================================
//
// checks, evaluation
//

bool checkAllGradients(ConstrainedProblem &P, const arr& x, double tolerance);
bool checkDirectionalGradient(ScalarFunction &f, const arr& x, const arr& delta, double tolerance);
bool checkDirectionalJacobian(VectorFunction &f, const arr& x, const arr& delta, double tolerance);


//these actually call the functions (->query cost) to evalute them at some point
double evaluateSF(ScalarFunction& f, const arr& x);
double evaluateVF(VectorFunction& f, const arr& x);


//===========================================================================
//
// optimization methods
//

enum ConstrainedMethodType { noMethod=0, squaredPenalty, augmentedLag, logBarrier, anyTimeAula };

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
  double stepInc, stepDec;
  double dampingInc, dampingDec;
  int nonStrictSteps; //# of non-strict iterations
  bool allowOverstep;
  ConstrainedMethodType constrainedMethod;
  double aulaMuInc;
  OptOptions();
};

extern Singleton<OptOptions> globalOptOptions;

#define NOOPT (globalOptOptions())

// declared separately:
#include "opt-newton.h"
#include "opt-constrained.h"
#include "opt-rprop.h"
uint optGradDescent(arr& x, ScalarFunction& f, OptOptions opt);


//===========================================================================
//
// helpers
//

void displayFunction(ScalarFunction &F, bool wait=true, double lo=-1.2, double hi=1.2);


//===========================================================================
//
// Named Parameters: Macros for the OPT
//


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
#define OPT(...)     (_OPT_N1(globalOptOptions(), _NUM_ARGS(__VA_ARGS__), __VA_ARGS__) , globalOptOptions())

#ifdef  MT_IMPLEMENTATION
#  include "optimization.cpp"
#endif

#endif

/// @} //end group
