/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/array.h>

//===========================================================================
//
// types (mostly abstract classes or lambda expressions) to represent non-linear programs
//

/// A scalar function $y = f(x)$, if @df@ or @Hf@ are not NoArr, the gradient and/or Hessian is returned
typedef std::function<double(arr& df, arr& Hf, const arr& x)> ScalarFunction;

/// A vector function $y = f(x)$, if @J@ is not NoArr, Jacobian is returned
/// This also implies an optimization problem $\hat f(y) = y^T(x) y(x)$ of (iterated)
/// Gauss-Newton type where the Hessian is approximated by J^T J
typedef std::function<void(arr& y, arr& Jy, const arr& x)> VectorFunction;

/// symbols to declare of which type an objective feature is
enum ObjectiveType { OT_none=0, OT_f, OT_sos, OT_ineq, OT_eq };
typedef rai::Array<ObjectiveType> ObjectiveTypeA;
extern ObjectiveTypeA& NoTermTypeA;

/** A ConstrainedProblem returns a feature vector $phi$ and optionally its Jacobian $J$. For each entry of
 *  this feature vector $tt(i)$ determins whether this is an inequality constraint, an equality constraint,
 *  a sumOfSqr or "direct-f" cost feature. The latter two define the objective function as
 *  $f(x) = f_j(x) + \sum_i \phi_i(x)^2$, where the sum only goes over sumOfSqr features, and f_j is a
 *  direct-f feature (tt(i)==OT_f). The direct-f feature is special: there may only exist a single such
 *  feature; and if there exists this feature the returned Hessian $H$ needs to be its hessian.
 *  For the sumOfSqr features no Hessian is returned: we assume the Gauss-Newton approximation.
 */
struct ConstrainedProblem {
  //TODO: add getStructure -> dim_x, tt
  virtual ~ConstrainedProblem() = default;
  virtual void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda) = 0;
};

//===========================================================================
//
// lambda expression interfaces
//

typedef std::function<void(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x)> ConstrainedProblemLambda;

struct Conv_Lambda_ConstrainedProblem : ConstrainedProblem {
  ConstrainedProblemLambda f;
  Conv_Lambda_ConstrainedProblem(const ConstrainedProblemLambda& f): f(f) {}
  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x) { f(phi, J, H, tt, x); }
};

//===========================================================================
//
// checks, evaluation
//

bool checkJacobianCP(ConstrainedProblem& P, const arr& x, double tolerance);
bool checkHessianCP(ConstrainedProblem& P, const arr& x, double tolerance);
bool checkDirectionalGradient(const ScalarFunction &f, const arr& x, const arr& delta, double tolerance);
bool checkDirectionalJacobian(const VectorFunction &f, const arr& x, const arr& delta, double tolerance);

inline arr summarizeErrors(const arr& phi, const ObjectiveTypeA& tt) {
  arr err = zeros(3);
  for(uint i=0; i<phi.N; i++) {
    if(tt(i)==OT_f) err(0) += phi(i);
    if(tt(i)==OT_sos) err(0) += rai::sqr(phi(i));
    if(tt(i)==OT_ineq && phi(i)>0.) err(1) += phi(i);
    if(tt(i)==OT_eq) err(2) += fabs(phi(i));
  }
  return err;
}


//===========================================================================
//
// generic optimization options
//

enum ConstrainedMethodType { noMethod=0, squaredPenalty, augmentedLag, logBarrier, anyTimeAula, squaredPenaltyFixed };

struct OptOptions {
  int verbose;
  double *fmin_return;
  double stopTolerance;
  double stopFTolerance;
  double stopGTolerance;
  uint   stopEvals;
  uint   stopIters;
  uint   stopOuters;
  uint   stopLineSteps;
  uint   stopTinySteps;
  double initStep;
  double minStep;
  double maxStep;
  double damping;
  double stepInc, stepDec;
  double dampingInc, dampingDec;
  double wolfe;
  int nonStrictSteps; //# of non-strict iterations
  bool allowOverstep;
  ConstrainedMethodType constrainedMethod;
  double muInit, muLBInit;
  double aulaMuInc;
  OptOptions();
  void write(std::ostream& os) const;
};
stdOutPipe(OptOptions)

extern Singleton<OptOptions> globalOptOptions;

#define NOOPT (globalOptOptions())

// optimization algorithms declared separately:
#include "newton.h"
#include "gradient.h"
//#include "lagrangian.h"
#include "convert.h"
//uint optGradDescent(arr& x, const ScalarFunction& f, OptOptions opt);

//===========================================================================
//
// helpers
//

void displayFunction(const ScalarFunction &f, bool wait=false, double lo=-1.2, double hi=1.2);

// function evaluation counter (used only for performance meassurements, global for simplicity)
extern uint eval_count;

//===========================================================================
//
// Named Parameters: Macros for the OPT
//

#ifndef _NUM_ARGS
#define _NUM_ARGS2(X,X64,X63,X62,X61,X60,X59,X58,X57,X56,X55,X54,X53,X52,X51,X50,X49,X48,X47,X46,X45,X44,X43,X42,X41,X40,X39,X38,X37,X36,X35,X34,X33,X32,X31,X30,X29,X28,X27,X26,X25,X24,X23,X22,X21,X20,X19,X18,X17,X16,X15,X14,X13,X12,X11,X10,X9,X8,X7,X6,X5,X4,X3,X2,X1,N,...) N
#define _NUM_ARGS(...) _NUM_ARGS2(0,1, __VA_ARGS__ ,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0)
#endif

#define _OPT_1(obj)         obj
#define _OPT_2(obj, assign) obj->assign
#define _OPT_3(obj, assign, ...) obj->assign, _OPT_2(obj,__VA_ARGS__)
#define _OPT_4(obj, assign, ...) obj->assign, _OPT_3(obj,__VA_ARGS__)
#define _OPT_5(obj, assign, ...) obj->assign, _OPT_4(obj,__VA_ARGS__)
#define _OPT_6(obj, assign, ...) obj->assign, _OPT_5(obj,__VA_ARGS__)
#define _OPT_7(obj, assign, ...) obj->assign, _OPT_6(obj,__VA_ARGS__)
#define _OPT_8(obj, assign, ...) obj->assign, _OPT_7(obj,__VA_ARGS__)
#define _OPT_9(obj, assign, ...) obj->assign, _OPT_8(obj,__VA_ARGS__)
#define _OPT_N2(obj, N, ...) _OPT_ ## N(obj, __VA_ARGS__)
#define _OPT_N1(obj, N, ...) _OPT_N2(obj, N, __VA_ARGS__) //this forces that _NUM_ARGS(...) is expanded to a number N
#define OPT(...)     (_OPT_N1(globalOptOptions(), _NUM_ARGS(__VA_ARGS__), __VA_ARGS__) , globalOptOptions())

