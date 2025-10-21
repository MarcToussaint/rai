/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "NLP.h"
#include "../Core/util.h"

struct NLP_FiniteDifference : NLP {
  std::shared_ptr<NLP> P;
  double eps;
  NLP_FiniteDifference(std::shared_ptr<NLP> _P, double eps=1e-6) : P(_P), eps(eps) { copySignature(*P); }
  virtual void evaluate(arr& phi, arr& J, const arr& x0);
  virtual void report(ostream& os, int verbose, const char *msg=0){ os <<"FiniteDifference version of: "; P->report(os, verbose, msg); }
};

struct NLP_SlackLeastSquares : NLP {
  std::shared_ptr<NLP> P;
  NLP_SlackLeastSquares(std::shared_ptr<NLP> _P);
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

bool checkDirectionalGradient(ScalarFunction f, const arr& x, const arr& delta, double tolerance);
bool checkDirectionalJacobian(VectorFunction f, const arr& x, const arr& delta, double tolerance);

//===========================================================================
//
// helpers
//

void accumulateInequalities(arr& y, arr& J, const arr& yAll, const arr& JAll);
void displayFunction(ScalarFunction f, bool wait=false, double lo=-1.2, double hi=1.2);

