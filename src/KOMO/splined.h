/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

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

  virtual arr getInitializationSample();

  virtual void report(ostream& os, int verbose, const char* msg=0) {
    os <<komo.report();
  }
};

