/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "NLP.h"

//===========================================================================
//
// lambda expression interfaces
//

// see function types defined in array.h
//ScalarFunction
//VectorFunction
//fct

struct Conv_ScalarProblem_NLP : NLP {
  ScalarFunction f;
  uint xDim;
  arr bounds_lo, bounds_up;
  Conv_ScalarProblem_NLP(const ScalarFunction& f, uint xDim): f(f), xDim(xDim) {}
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

struct Conv_NLP_ScalarProblem : ScalarFunction {
  std::shared_ptr<NLP> P;

  Conv_NLP_ScalarProblem(std::shared_ptr<NLP> _P) : P(_P) {
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

bool checkJacobianCP(NLP& P, const arr& x, double tolerance);
bool checkHessianCP(NLP& P, const arr& x, double tolerance);
bool checkInBound(NLP& P, const arr& x);
void boundClip(NLP& P, arr& x);
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
