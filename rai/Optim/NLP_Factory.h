#pragma once

#include "MathematicalProgram.h"

/** The MathematicalProgram class is an interface for /solvers/: it provides access to anything that a solver
 *  wants to query. The MathematicalProgram_Factory class is an interface for /users/: It makes it easier to
 *  declare an NLP (as an alternative to overloading the virtuals of MathematicalProgram). But the semantics
 *  of the setting methods are perfectly analogous to the virtual methods of MathematicalProgram */
struct NLP_Factory : MathematicalProgram {
  void *userData;
  uint dim;
  ObjectiveTypeA featureTypes;
  arr bounds_lo, bounds_up;

  std::function<void(arr& y, arr& J, const arr& x, void* _userData)> eval;
  std::function<std::tuple<arr, arr> (const arr&)> eval2;
  std::function<arr(const arrL& previousOptima, void* _userData)> init;

  NLP_Factory() {}

  void setDimension(uint _dim) { dim = _dim; }
  void setFeatureTypes(const ObjectiveTypeA& _featureTypes){ featureTypes = _featureTypes; }
  void setBounds(const arr& _bounds_lo, const arr& _bounds_up) { bounds_lo=_bounds_lo; bounds_up=_bounds_up; }

  void setEvalCallback1(const std::function<void(arr& y, arr& J, const arr& x, void* _userData)>& _eval, void *_userData=0){ eval = _eval;  userData = _userData; }
  void setEvalCallback2(const std::function<std::tuple<arr, arr> (const arr&)>& _eval){ eval2 = _eval; }
  void setInitCallback(const std::function<arr(const arrL& previousOptima, void* _userData)>& _init){ init = _init; }

  //-- helpers for analysis
  void checkGradients(const arr& x, double eps = 1e-4);

  //-- overloads of MathematicalProgram virtuals

  void getFeatureTypes(ObjectiveTypeA& _featureTypes){ _featureTypes=featureTypes; }
  void evaluate(arr& phi, arr& J, const arr& x){
    if(eval) eval(phi, J, x, userData);
    else if(eval2){
      auto ret = eval2(x);
      phi=std::get<0>(ret);
      J = std::get<1>(ret);
    } else HALT("no evaluation method set");
  }
  uint getDimension() { return dim; }
  void getBounds(arr& _bounds_lo, arr& _bounds_up) { _bounds_lo=bounds_lo; _bounds_up = bounds_up; }
};
