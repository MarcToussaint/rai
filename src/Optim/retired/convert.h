/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "optimization.h"
#include "NLP.h"

//-- basic converters
ScalarFunction     conv_cstylefs2ScalarFunction(double(*fs)(arr*, const arr&, void*), void* data);
VectorFunction     conv_cstylefv2VectorFunction(void (*fv)(arr&, arr*, const arr&, void*), void* data);
ScalarFunction     conv_VectorFunction2ScalarFunction(const VectorFunction& f);
//NLP conv_KOMO2NLP(struct KOMO_Problem& f);

/// this takes a constrained problem over $x$ and re-represents it over $z$ where $x=Bz$

struct Conv_linearlyReparameterize_NLP : NLP {
  NLP& P;
  arr B;
  Conv_linearlyReparameterize_NLP(NLP& P, const arr& B):P(P), B(B) {}
  ~Conv_linearlyReparameterize_NLP() {}

  virtual void evaluate(arr& phi, arr& J, const arr& z);
};

/// A struct that allows to convert one function type into another, even when given as argument
struct Convert {
  double(*cstyle_fs)(arr*, const arr&, void*);
  void (*cstyle_fv)(arr&, arr*, const arr&, void*);
  void* data;
  ScalarFunction sf;
  VectorFunction vf;
  NLP* cpm;

  Convert(const ScalarFunction&);
  Convert(const VectorFunction&);
  Convert(struct KOMO_Problem&);
  Convert(double(*fs)(arr*, const arr&, void*), void* data);
  Convert(void (*fv)(arr&, arr*, const arr&, void*), void* data);
  ~Convert();
  operator ScalarFunction();
  operator VectorFunction();
  operator NLP& ();
};

