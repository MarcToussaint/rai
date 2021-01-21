/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "MathematicalProgram.h"

struct NLoptInterface {
  MathematicalProgram& P;
  arr x, phi_x, J_x;
  ObjectiveTypeA featureTypes;
  int verbose=1;

  NLoptInterface(MathematicalProgram& _P) : P(_P) {
    P.getFeatureTypes(featureTypes);
  }

  arr solve(const arr& x_init=NoArr);

 private:
  double f(const arr& _x, arr& _grad);
  double g(const arr& _x, arr& _grad, uint feature);
  double h(const arr& _x, arr& _grad, uint feature);

  static double _f(const std::vector<double>& _x, std::vector<double>& _grad, void* f_data);
  static double _g(const std::vector<double>& _x, std::vector<double>& _grad, void* f_data);
  static double _h(const std::vector<double>& _x, std::vector<double>& _grad, void* f_data);
};
