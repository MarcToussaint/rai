/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "MathematicalProgram.h"

//===========================================================================
//
// lambda expression interfaces
//

/// A scalar function $y = f(x)$, if @df@ or @Hf@ are not NoArr, the gradient and/or Hessian is returned
typedef std::function<double(arr& df, arr& Hf, const arr& x)> ScalarFunction;

/// A vector function $y = f(x)$, if @J@ is not NoArr, Jacobian is returned
/// This also implies an optimization problem $\hat f(y) = y^T(x) y(x)$ of (iterated)
/// Gauss-Newton type where the Hessian is approximated by J^T J
typedef std::function<void(arr& y, arr& Jy, const arr& x)> VectorFunction;


struct Conv_ScalarProblem_MathematicalProgram : MathematicalProgram {
  ScalarFunction f;
  uint xDim;
  arr bounds_lo, bounds_up;
  Conv_ScalarProblem_MathematicalProgram(const ScalarFunction& f, uint xDim): f(f), xDim(xDim) {}
  uint getDimension() { return xDim; }
  void getBounds(arr& _bounds_lo, arr& _bounds_up) { _bounds_lo=bounds_lo; _bounds_up=bounds_up; }
  void getFeatureTypes(ObjectiveTypeA& ot) { ot = {OT_f}; }
  void evaluate(arr& phi, arr& J, const arr& x) {
    double y = f(J, NoArr, x);
    phi = {y};
    J.reshape(1, x.N);
  }
  void getFHessian(arr& H, const arr& x) {
    f(NoArr, H, x);
  }

  void setBounds(double lo, double up){ bounds_lo.resize(xDim) = lo;  bounds_up.resize(xDim) = up; }
};

struct Conv_MathematicalProgram_ScalarProblem : ScalarFunction {
  MathematicalProgram &P;

  Conv_MathematicalProgram_ScalarProblem(MathematicalProgram &P) : P(P){
    ScalarFunction::operator=([this](arr& g, arr& H, const arr& x) -> double {
      return this->scalar(g, H, x);
    });
  }

  double scalar(arr& g, arr& H, const arr& x){
    ObjectiveTypeA featureTypes;
    arr phi, J;
    P.getFeatureTypes(featureTypes);
    P.evaluate(phi, J, x);

    CHECK_EQ(phi.N, featureTypes.N, "");
    CHECK_EQ(phi.N, J.d0, "");
    CHECK_EQ(x.N, J.d1, "");

    double f=0.;
    for(uint i=0; i<phi.N; i++) {
      if(featureTypes.p[i]==OT_sos) f += rai::sqr(phi.p[i]);
      else if(featureTypes.p[i]==OT_f) f += phi.p[i];
      else HALT("this must be an unconstrained problem!")
    }

    if(!!g) { //gradient
      arr coeff=zeros(phi.N);
      for(uint i=0; i<phi.N; i++) {
        if(featureTypes.p[i]==OT_sos) coeff.p[i] += 2.* phi.p[i];
        else if(featureTypes.p[i]==OT_f) coeff.p[i] += 1.;
      }
      g = comp_At_x(J, coeff);
      g.reshape(x.N);
    }

    if(!!H) { //hessian: Most terms are of the form   "J^T  diag(coeffs)  J"
      arr coeff=zeros(phi.N);
      double hasF=false;
      for(uint i=0; i<phi.N; i++) {
        if(featureTypes.p[i]==OT_sos) coeff.p[i] += 2.;
        else if(featureTypes.p[i]==OT_f) hasF=true;
      }
      arr tmp = J;
      if(!isSparseMatrix(tmp)) {
        for(uint i=0; i<phi.N; i++) tmp[i]() *= sqrt(coeff.p[i]);
      } else {
        arr sqrtCoeff = sqrt(coeff);
        tmp.sparse().rowWiseMult(sqrtCoeff);
      }
      H = comp_At_A(tmp); //Gauss-Newton type!

      if(hasF) { //For f-terms, the Hessian must be given explicitly, and is not \propto J^T J
        arr fH;
        P.getFHessian(fH, x);
        H += fH;
      }

      if(!H.special) H.reshape(x.N, x.N);
    }

    return f;
  }
};


//===========================================================================
//
// checks, evaluation
//

bool checkJacobianCP(MathematicalProgram& P, const arr& x, double tolerance);
bool checkHessianCP(MathematicalProgram& P, const arr& x, double tolerance);
bool checkInBound(MathematicalProgram& P, const arr& x);
void boundClip(MathematicalProgram& P, arr& x);
void boundClip(arr& y, const arr& bound_lo, const arr& bound_up);
bool checkDirectionalGradient(const ScalarFunction& f, const arr& x, const arr& delta, double tolerance);
bool checkDirectionalJacobian(const VectorFunction& f, const arr& x, const arr& delta, double tolerance);


//===========================================================================
//
// accumulative constraints
//

inline void accumulateInequalities(arr& y, arr& J, const arr& yAll, const arr& JAll) {
  y.resize(1).setZero();
  if(!!J) J.resize(1, JAll.d1).setZero();

  for(uint i=0; i<yAll.N; i++) {
    if(yAll.elem(i)>0.) {
      y.scalar() += yAll.elem(i);
      if(!!J && !!JAll) J[0] += JAll[i];
    }
  }
}

//===========================================================================
//
// generic optimization options
//

enum ConstrainedMethodType { noMethod=0, squaredPenalty, augmentedLag, logBarrier, anyTimeAula, squaredPenaltyFixed };

struct OptOptions {
#define arg(type, name) type name; OptOptions& set_##name(type _##name){ name=_##name; return *this; }
  arg(int, verbose)
  arg(double*, fmin_return)
  arg(double, stopTolerance)
  arg(double, stopFTolerance)
  arg(double, stopGTolerance)
  arg(uint,   stopEvals)
  arg(uint,   stopIters)
  arg(uint,   stopOuters)
  arg(uint,   stopLineSteps)
  arg(uint,   stopTinySteps)
  arg(double, initStep)
  arg(double, minStep)
  arg(double, maxStep)
  arg(double, damping)
  arg(double, stepInc)
  arg(double, stepDec)
  arg(double, dampingInc)
  arg(double, dampingDec)
  arg(double, wolfe)
  arg(int, nonStrictSteps) //# of non-strict iterations
  arg(bool, allowOverstep)
  arg(ConstrainedMethodType, constrainedMethod)
  arg(double, muInit)
  arg(double, muLBInit)
  arg(double, aulaMuInc)
#undef arg
  OptOptions();
  void write(std::ostream& os) const;
};
stdOutPipe(OptOptions)

extern Singleton<OptOptions> globalOptOptions;
extern OptOptions *__globalOptOptions;

#define NOOPT (globalOptOptions())

// optimization algorithms declared separately:
#include "newton.h"
#include "gradient.h"
//#include "lagrangian.h"
//#include "convert.h"
//uint optGradDescent(arr& x, const ScalarFunction& f, OptOptions opt);

//===========================================================================
//
// helpers
//

void displayFunction(const ScalarFunction& f, bool wait=false, double lo=-1.2, double hi=1.2);

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
#define OPT(...)     (globalOptOptions(), _OPT_N1(__globalOptOptions, _NUM_ARGS(__VA_ARGS__), __VA_ARGS__) , *__globalOptOptions)

