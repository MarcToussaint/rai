#pragma once

#include "../Control/timingOpt.h"
#include "../KOMO/komo.h"
#include "../Algo/spline.h"

struct SplinedKOMO : NLP {
  rai::BSpline S;
  KOMO& komo;
  arr x0;
  shared_ptr<NLP> komo_nlp;
  arr a_fine, u_fine;
  rai::Configuration C;
  bool useTorqueLimits;
  uint pieceSubSamples;
  StringA featureNames;

  SplinedKOMO(uint degree, uint numCtrlPoints, KOMO& _komo);

  virtual void evaluate(arr& phi, arr& J, const arr& x);

  virtual arr getInitializationSample(const arr& previousOptima= {});

  virtual void report(ostream& os, int verbose, const char* msg=0){
     os <<komo.report();
  }
};


