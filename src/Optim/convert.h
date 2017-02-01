/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#pragma once

#include "optimization.h"

//-- basic converters
ScalarFunction     conv_cstylefs2ScalarFunction(double(*fs)(arr*, const arr&, void*),void *data);
VectorFunction     conv_cstylefv2VectorFunction(void (*fv)(arr&, arr*, const arr&, void*),void *data);
ScalarFunction     conv_VectorFunction2ScalarFunction(const VectorFunction& f);
//ConstrainedProblem conv_KOMO2ConstrainedProblem(struct KOMO_Problem& f);

/// this takes a constrained problem over $x$ and re-represents it over $z$ where $x=Bz$

struct Conv_linearlyReparameterize_ConstrainedProblem : ConstrainedProblem{
  ConstrainedProblem& P;
  arr B;
  Conv_linearlyReparameterize_ConstrainedProblem(ConstrainedProblem& P, const arr& B):P(P), B(B){}
  virtual void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& z);
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

