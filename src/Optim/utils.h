/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
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
  void evaluate(arr& phi, arr& J, const arr& x) {
    double y = f(J, NoArr, x);
    phi = {y};
    if(!!J) J.reshape(1, x.N);
  }
  void getFHessian(arr& H, const arr& x) {
    f(NoArr, H, x);
  }
  void setBounds(double lo, double up) { bounds_lo.resize(xDim) = lo;  bounds_up.resize(xDim) = up; }
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

struct Conv_NLP_SlackLeastSquares : NLP {
  std::shared_ptr<NLP> P;
  uintA pick;

  Conv_NLP_SlackLeastSquares(std::shared_ptr<NLP> _P) : P(_P) {
    dimension = P->dimension;
    bounds = P->bounds;

    //pick constraints
    for(uint i=0; i<P->featureTypes.N; i++) {
      ObjectiveType f = P->featureTypes(i);
      if(f==OT_eq || f==OT_ineq) pick.append(i);
    }
    featureTypes.resize(pick.N) = OT_sos;
  }

  virtual void evaluate(arr& phi, arr& J, const arr& x) {
    arr Pphi, PJ;
    P->evaluate(Pphi, PJ, x);
    phi = Pphi.pick(pick);
    J = PJ.pick(pick);
    for(uint i=0; i<pick.N; i++) {
      if(P->featureTypes(pick(i))==OT_ineq) {
        if(phi(i)<0.) { phi(i)=0.; J[i]=0.; } //ReLu for g
      } else if(P->featureTypes(pick(i))==OT_eq) {
        if(phi(i)<0.) { phi(i)*=-1.; J[i]*=-1.; } //make positive
      } else {
        NIY;
      }
    }
  }
};

struct NLP_LinTransformed : NLP {
  std::shared_ptr<NLP> P;
  arr A, b, Ainv;

  NLP_LinTransformed(std::shared_ptr<NLP> _P, const arr& _A, const arr& _b);

  virtual arr getInitializationSample();
  virtual void evaluate(arr& phi, arr& J, const arr& x);
  virtual void report(ostream& os, int verbose, const char* msg=0){ P->report(os, verbose, msg); }
};

//===========================================================================
//
// checks, evaluation
//

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
// helpers
//

void displayFunction(const ScalarFunction& f, bool wait=false, double lo=-1.2, double hi=1.2);

