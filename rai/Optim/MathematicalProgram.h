#pragma once

#include "optimization.h"

//===========================================================================

struct MathematicalProgram {
  ///-- signature of the mathematical problem
  virtual uint getDimension() = 0;                  //the dimensionality of the full decision variable
  virtual void getBounds(arr& bounds_lo, arr& bounds_up){ bounds_lo.clear(); bounds_up.clear(); } //lower/upper bounds for the decision variable (may be {})
  virtual void getFeatureTypes(ObjectiveTypeA& featureTypes) = 0;   //the number and type of all features (sos, ineq, eq, or f)

  /// optional - for more semantic reports
  virtual void getNames(StringA& variableNames, StringA& featureNames){ variableNames.clear(); featureNames.clear(); } //the names of each variable/feature block (or element if unstructured)

  ///-- initialization/MC sampling/restarts
  virtual arr getInitializationSample(const arrL& previousOptima={});

  ///--- evaluation

  //unstructured (batch) interface (where J may/should be sparse! and H optional
  virtual void evaluate(arr& phi, arr& J, arr& H, const arr& x) = 0; //default implementation: if isStructured, gets everything from there
};

//===========================================================================

struct MathematicalProgram_Structured : MathematicalProgram {
  ///-- structure of the mathematical problem
  virtual void getStructure(uintA& variableDimensions, //the size of each variable block
                            uintA& featureDimensions,  //the size of each feature block
                            intAA& featureVariables    //which variables the j-th feature block depends on
                            );

  ///--- evaluation
  //unstructured (batch) interface (where J may/should be sparse! and H optional
  virtual void evaluate(arr& phi, arr& J, arr& H, const arr& x); //default implementation: use setSingleVariable and evaluateSingleFeature

  //structured (local) interface
  virtual void setSingleVariable(uint var_id, const arr& x) = 0; //set a single variable block
  virtual void evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H) = 0; //get a single feature block
};

//===========================================================================
//
// conversion to old interface
//

struct Conv_MathematicalProgram_ConstrainedProblem : ConstrainedProblem {
  ptr<MathematicalProgram> MP;
  ObjectiveTypeA ot;
  Conv_MathematicalProgram_ConstrainedProblem(const ptr<MathematicalProgram>& _MP): MP(_MP) {
    MP->getFeatureTypes(ot);
  }
  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& _ot, const arr& x) {
    MP->evaluate(phi, J, H, x);
    if(!!_ot) _ot = ot;
  }
};
