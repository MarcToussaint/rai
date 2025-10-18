/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "NLP.h"
#include "../Core/util.h"

struct Conv_ScalarFunction2NLP : NLP {
  shared_ptr<ScalarFunction> f;
  Conv_ScalarFunction2NLP(shared_ptr<ScalarFunction> f);
  virtual void evaluate(arr& phi, arr& J, const arr& x);
  virtual void getFHessian(arr& H, const arr& x);
  virtual void report(ostream& os, int verbose, const char *msg=0){
    os <<"ScalarFunction of type '" <<rai::niceTypeidName(typeid(*this)) <<"'";
    if(msg) os <<' ' <<msg;
    os <<" dimension:" <<f->dim;
  }
};

struct NLP_FiniteDifference : NLP {
  std::shared_ptr<NLP> P;
  NLP_FiniteDifference(std::shared_ptr<NLP> _P) : P(_P) { copySignature(*P); }
  virtual void evaluate(arr& phi, arr& J, const arr& x0);
  virtual void report(ostream& os, int verbose, const char *msg=0){ os <<"FiniteDifference version of: "; P->report(os, verbose, msg); }
};

struct Conv_NLP2ScalarProblem : ScalarFunction {
  std::shared_ptr<NLP> P;
  bool useFiniteDifference=false;
  Conv_NLP2ScalarProblem(std::shared_ptr<NLP> _P, bool useFiniteDifference=false) : P(_P), useFiniteDifference(useFiniteDifference) { dim = P->dimension; }
  virtual double f(arr& g, arr& H, const arr& x){
    double f_x;
    if(!useFiniteDifference){
      f_x = P->eval_scalar(g, H, x);
    }else{
      CHECK(!H, "");
      CHECK(!!g, "");
      f_x = P->eval_scalar(NoArr, NoArr, x);
      g = finiteDifference_gradient([this](const arr& x){ return this->P->eval_scalar(NoArr, NoArr, x); }, x, f_x, 1e-6);
    }
    return f_x;
  }
};

struct Conv_NLP_SlackLeastSquares : NLP {
  std::shared_ptr<NLP> P;
  Conv_NLP_SlackLeastSquares(std::shared_ptr<NLP> _P);
  virtual void evaluate(arr& phi, arr& J, const arr& x);
  virtual void report(ostream& os, int verbose, const char *msg=0){ os <<"SlackLeastSquares of: "; P->report(os, verbose, msg); }
private:
  uintA pick;
};

struct NLP_LinTransformed : NLP {
  std::shared_ptr<NLP> P;
  arr A, b, Ainv;

  NLP_LinTransformed(std::shared_ptr<NLP> _P, const arr& _A, const arr& _b);

  virtual arr getInitializationSample();
  virtual void evaluate(arr& phi, arr& J, const arr& x);
  virtual void report(ostream& os, int verbose, const char *msg=0){ os <<"LinTransformed of: "; P->report(os, verbose, msg); }
};

//===========================================================================
//
// checks, evaluation
//

bool checkDirectionalGradient(const ScalarFunction& f, const arr& x, const arr& delta, double tolerance);
bool checkDirectionalJacobian(const VectorFunction& f, const arr& x, const arr& delta, double tolerance);

//===========================================================================
//
// helpers
//

void accumulateInequalities(arr& y, arr& J, const arr& yAll, const arr& JAll);
void displayFunction(ScalarFunction& f, bool wait=false, double lo=-1.2, double hi=1.2);

