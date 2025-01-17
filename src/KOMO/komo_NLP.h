/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "komo.h"
#include "../Optim/utils.h"

namespace rai {

//default - transcription as sparse, but non-factored NLP
struct KOMO_NLP : NLP {
  KOMO& komo;

  arr quadraticPotentialLinear, quadraticPotentialHessian;

  KOMO_NLP(KOMO& _komo);

  virtual arr getInitializationSample();
  virtual void evaluate(arr& phi, arr& J, const arr& x);
  virtual void getFHessian(arr& H, const arr& x);

  virtual void report(ostream& os, int verbose, const char* msg=0);
};

struct KOMO_Spline_NLP : NLP {
  std::shared_ptr<KOMO_NLP> fine_nlp;
  std::shared_ptr<NLP_LinTransformed> nlp;
  arr spline_B, spline_b;

  KOMO_Spline_NLP(KOMO& _komo, uint splineT, uint degree);

  virtual arr getInitializationSample();
  virtual void evaluate(arr& phi, arr& J, const arr& x){ nlp->evaluate(phi, J, x); }
  virtual void report(ostream& os, int verbose, const char* msg=0){ fine_nlp->report(os, verbose, msg); }
};

struct KOMO_SubNLP : NLP {
  KOMO& komo;
  rai::Array<GroundedObjective*> objs;
  DofL dofs;
  StringA featureNames;
  uint evalCount=0;

  KOMO_SubNLP(KOMO& _komo, const rai::Array<GroundedObjective*>& _objs, const DofL& _dofs);

  virtual arr getInitializationSample();
  virtual void evaluate(arr& phi, arr& J, const arr& x);
  virtual void getFHessian(arr& H, const arr& x);

  virtual void report(ostream& os, int verbose, const char* msg=0);
};

//this treats EACH PART and force-dof as its own variable
struct Conv_KOMO_FactoredNLP : NLP_Factored {
  KOMO& komo;

  //redundant to NLP_Factored::variableDims -- but sub can SUBSELECT!; in addition: dofs and names
  struct VariableIndexEntry { uint dim; DofL dofs; String name; };
  rai::Array<VariableIndexEntry> __variableIndex;
  uintA subVars;

  //redundant to NLP_Factored::featureDims*  -- but sub can SUBSELECT!
  struct FeatureIndexEntry { uintA vars; shared_ptr<GroundedObjective> ob; };
  rai::Array<FeatureIndexEntry> __featureIndex;
  uintA subFeats;

  virtual uint varsN() { if(subVars.N) return subVars.N; return __variableIndex.N; }
  virtual uint featsN() { if(subVars.N) return subFeats.N; return __featureIndex.N; }
  VariableIndexEntry& vars(uint var_id) { if(subVars.N) return __variableIndex(subVars(var_id)); else return __variableIndex(var_id); }
  FeatureIndexEntry& feats(uint feat_id) { if(subVars.N) return __featureIndex(subFeats(feat_id)); else return __featureIndex(feat_id); }

  Conv_KOMO_FactoredNLP(KOMO& _komo, const rai::Array<DofL>& varDofs);

  virtual void subSelect(const uintA& activeVariables, const uintA& conditionalVariables);
  virtual uint numTotalVariables() { return __variableIndex.N; }

  virtual rai::String getVariableName(uint var_id) { return vars(var_id).name; }

  ///-- signature/structure of the mathematical problem
//    virtual arr getInitializationSample();
  virtual arr getInitializationSample();
  virtual arr  getSingleVariableInitSample(uint var_id);
  virtual void randomizeSingleVariable(uint var_id);

  ///--- evaluation
  virtual void setSingleVariable(uint var_id, const arr& x); //set a single variable block
  virtual void evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H); //get a single feature block

  void evaluate(arr& phi, arr& J, const arr& x);
  void report(ostream& os, int verbose, const char* msg=0);
  void reportDetails(ostream& os, int verbose, const char* msg=0);
};

}//namespace
