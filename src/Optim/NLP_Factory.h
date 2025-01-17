/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "NLP.h"

/** The NLP class is an interface for /solvers/: it provides access to anything that a solver
 *  wants to query. The NLP_Factory class is an interface for /users/: It makes it easier to
 *  declare an NLP (as an alternative to overloading the virtuals of NLP). But the semantics
 *  of the setting methods are perfectly analogous to the virtual methods of NLP */
struct NLP_Factory : NLP {
  void* userData;

  std::function<void(arr& y, arr& J, const arr& x, void* _userData)> eval;
  std::function<std::tuple<arr, arr> (const arr&)> eval2;
  std::function<arr(void* _userData)> init;

  NLP_Factory() {}

  void setDimension(uint _dim) { dimension = _dim; }
  void setFeatureTypes(const ObjectiveTypeA& _featureTypes) { featureTypes = _featureTypes; }
  void setBounds(const arr& _bounds_lo, const arr& _bounds_up) { bounds.resize(2,_bounds_lo.N); bounds[0]=_bounds_lo; bounds[1]=_bounds_up; }

  void setEvalCallback1(const std::function<void(arr& y, arr& J, const arr& x, void* _userData)>& _eval, void* _userData=0) { eval = _eval;  userData = _userData; }
  void setEvalCallback2(const std::function<std::tuple<arr, arr> (const arr&)>& _eval) { eval2 = _eval; }
  void setInitCallback(const std::function<arr(void* _userData)>& _init) { init = _init; }

  //-- helpers for analysis
  void checkGradients(const arr& x, double eps = 1e-4);

  //-- overloads of NLP virtuals

  void evaluate(arr& phi, arr& J, const arr& x) {
    if(eval) eval(phi, J, x, userData);
    else if(eval2) {
      auto ret = eval2(x);
      phi=std::get<0>(ret);
      J = std::get<1>(ret);
    } else HALT("no evaluation method set");
  }
};
