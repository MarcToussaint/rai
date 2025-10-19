/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "NLP.h"

struct NLoptInterface {
  shared_ptr<NLP> P;
  arr x, phi_x, J_x;
  int verbose=1;

  NLoptInterface(const shared_ptr<NLP>& _P) : P(_P) {}

  arr solve(const arr& x_init=NoArr);

 private:
  double f(const arr& _x, arr& _grad);
  double g(const arr& _x, arr& _grad, uint feature);
  double h(const arr& _x, arr& _grad, uint feature);

  static double _f(const std::vector<double>& _x, std::vector<double>& _grad, void* f_data);
  static double _g(const std::vector<double>& _x, std::vector<double>& _grad, void* f_data);
  static double _h(const std::vector<double>& _x, std::vector<double>& _grad, void* f_data);
};
