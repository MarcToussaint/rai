#pragma once

#include "komo.h"

namespace rai {

//default - transcription as sparse, but non-factored NLP
struct Conv_KOMO_NLP : NLP {
  KOMO& komo;
  bool sparse;

  arr quadraticPotentialLinear, quadraticPotentialHessian;

  Conv_KOMO_NLP(KOMO& _komo, bool sparse=true);

  virtual arr getInitializationSample(const arr& previousOptima= {});
  virtual void evaluate(arr& phi, arr& J, const arr& x);
  virtual void getFHessian(arr& H, const arr& x);

  virtual void report(ostream& os, int verbose);
};

//this treats EACH BRANCH and dof as its own variable
struct Conv_KOMO_FactoredNLP : NLP_Factored {
  KOMO& komo;

  //in addition to the NLP_Factored signature, we store per variable:
  rai::Array<DofL> variableDofs;
  StringA variableNames;

//  //each variable refers to a SET OF dofs (e.g., a set of joints)
//  struct VariableIndexEntry { DofL dofs; uint dim; String name; };
//  rai::Array<VariableIndexEntry> __variableIndex;

//  //features are one-to-one with gounded KOMO features, but with additional info on varIds
//  struct FeatureIndexEntry { shared_ptr<GroundedObjective> ob; uint dim; uintA varIds; };
//  rai::Array<FeatureIndexEntry> __featureIndex;

  //when subSelect, this stores which variables are active
  uintA subVars;
  uintA subFeats;

//  VariableIndexEntry& variableIndex(uint var_id){ if(subVars.N) return __variableIndex(subVars(var_id)); else return __variableIndex(var_id); }
//  FeatureIndexEntry& featureIndex(uint feat_id){ if(subFeats.N) return __featureIndex(subFeats(feat_id)); else return __featureIndex(feat_id); }

  Conv_KOMO_FactoredNLP(KOMO& _komo, const rai::Array<DofL>& varDofs);

  virtual void subSelect(const uintA& activeVariables, const uintA& conditionalVariables);

  virtual uint getNumVariables() { if(subVars.N) return subVars.N; return variableDimensions.N; }
  virtual uint getNumFeatures() { if(subFeats.N) return subFeats.N; return featureDimensions.N; }

  virtual rai::String getVariableName(uint var_id){ if(subVars.N) return variableNames(subVars(var_id)); return variableNames(var_id); }

  ///-- signature/structure of the mathematical problem
//    virtual arr getInitializationSample();
  virtual arr getInitializationSample(const arr& previousOptima) {
    komo.run_prepare(.01);
    return komo.x;
  }

  ///--- evaluation
  virtual void setSingleVariable(uint var_id, const arr& x); //set a single variable block
  virtual void evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H); //get a single feature block

  void report(ostream& os, int verbose);
};

//this treats each time slice as its own variable
struct Conv_KOMO_TimeFactoredNLP : NLP_Factored {
  KOMO& komo;

  struct VariableIndexEntry { uint t; uint dim; uint xIndex; };
  rai::Array<VariableIndexEntry> variableIndex;

  struct FeatureIndexEntry { shared_ptr<Objective> ob; shared_ptr<GroundedObjective> ob2; uint t; uintA varIds; uint dim; uint phiIndex; };
  rai::Array<FeatureIndexEntry> featureIndex;

  uintA xIndex2VarId;
  uint featuresDim;

  Conv_KOMO_TimeFactoredNLP(KOMO& _komo);

  virtual arr getInitializationSample(const arr& previousOptima= {});

  virtual void setAllVariables(const arr& x);
  virtual void setSingleVariable(uint var_id, const arr& x); //set a single variable block
  virtual void evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H); //get a single feature block
  virtual void report();
};


struct Conv_KOMO_TimeSliceProblem : NLP {
  KOMO& komo;
  int slice;

  Conv_KOMO_TimeSliceProblem(KOMO& _komo, int _slice) : komo(_komo), slice(_slice) {
    dimension = komo.pathConfig.getJointStateDimension();
  }

  void getDimPhi();

  virtual void evaluate(arr& phi, arr& J, const arr& x);
};

}//namespace
