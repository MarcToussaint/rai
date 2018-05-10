/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "optimization.h"

//-- basic converters
ScalarFunction     conv_cstylefs2ScalarFunction(double(*fs)(arr*, const arr&, void*),void *data);
VectorFunction     conv_cstylefv2VectorFunction(void (*fv)(arr&, arr*, const arr&, void*),void *data);
ScalarFunction     conv_VectorFunction2ScalarFunction(const VectorFunction& f);
//ConstrainedProblem conv_KOMO2ConstrainedProblem(struct KOMO_Problem& f);

/// this takes a constrained problem over $x$ and re-represents it over $z$ where $x=Bz$

struct Conv_linearlyReparameterize_ConstrainedProblem : ConstrainedProblem {
  ConstrainedProblem& P;
  arr B;
  Conv_linearlyReparameterize_ConstrainedProblem(ConstrainedProblem& P, const arr& B):P(P), B(B) {}
  virtual void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& z, arr& lambda);
};

/// A struct that allows to convert one function type into another, even when given as argument
struct Convert {
  double(*cstyle_fs)(arr*, const arr&, void*);
  void (*cstyle_fv)(arr&, arr*, const arr&, void*);
  void *data;
  ScalarFunction sf;
  VectorFunction vf;
  ConstrainedProblem *cpm;
  
  Convert(const ScalarFunction&);
  Convert(const VectorFunction&);
  Convert(struct KOMO_Problem&);
  Convert(double(*fs)(arr*, const arr&, void*),void *data);
  Convert(void (*fv)(arr&, arr*, const arr&, void*),void *data);
  ~Convert();
  operator ScalarFunction();
  operator VectorFunction();
  operator ConstrainedProblem&();
};

