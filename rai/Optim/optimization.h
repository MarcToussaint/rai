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

// see function types defined in array.h
//ScalarFunction
//VectorFunction
//fct

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
    if(!!J) J.reshape(1, x.N);
  }
  void getFHessian(arr& H, const arr& x) {
    f(NoArr, H, x);
  }
  void setBounds(double lo, double up){ bounds_lo.resize(xDim) = lo;  bounds_up.resize(xDim) = up; }
};

struct Conv_MathematicalProgram_ScalarProblem : ScalarFunction {
  std::shared_ptr<MathematicalProgram> P;

  Conv_MathematicalProgram_ScalarProblem(std::shared_ptr<MathematicalProgram> _P) : P(_P) {
    ScalarFunction::operator=([this](arr& g, arr& H, const arr& x) -> double {
      return this->scalar(g, H, x);
    });
  }

  double scalar(arr& g, arr& H, const arr& x);
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
bool boundCheck(const arr& x, const arr& bound_lo, const arr& bound_up, double eps=1e-3);
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
  RAI_PARAM("opt/", int, verbose, 1)
  RAI_PARAM("opt/", double, stopTolerance, 1e-2)
  RAI_PARAM("opt/", double, stopFTolerance, 1e-1)
  RAI_PARAM("opt/", double, stopGTolerance, -1.)
  RAI_PARAM("opt/", int,    stopEvals, 1000)
  RAI_PARAM("opt/", int,    stopIters, 1000)
  RAI_PARAM("opt/", int,    stopOuters, 1000)
  RAI_PARAM("opt/", int,    stopLineSteps, 10)
  RAI_PARAM("opt/", int,    stopTinySteps, 10)
  RAI_PARAM("opt/", double, initStep, 1.)
  RAI_PARAM("opt/", double, minStep, -1.)
  RAI_PARAM("opt/", double, maxStep, .2)
  RAI_PARAM("opt/", double, damping, 1.)
  RAI_PARAM("opt/", double, stepInc, 1.5)
  RAI_PARAM("opt/", double, stepDec, .5)
  RAI_PARAM("opt/", double, dampingInc, -1.)
  RAI_PARAM("opt/", double, dampingDec, -1.)
  RAI_PARAM("opt/", double, wolfe, .01)
  RAI_PARAM("opt/", int,    nonStrictSteps, 0) //# of non-strict iterations
  RAI_PARAM("opt/", bool,   boundedNewton, true)
  RAI_PARAM("opt/", bool,   allowOverstep, false)
  RAI_PARAM("opt/", double, muInit, 1.)
  RAI_PARAM("opt/", double, aulaMuInc, 5.)
  RAI_PARAM("opt/", double, muLBInit, .1)
  RAI_PARAM("opt/", double, muLBDec, .2)
  RAI_PARAM_ENUM("opt/", ConstrainedMethodType, constrainedMethod, augmentedLag)
//  void write(std::ostream& os) const;
};
//stdOutPipe(OptOptions)

OptOptions& globalOptOptions();
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
