/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "CtrlSet.h"
#include "../Kin/F_LeapCost.h"
#include "../KOMO/komo.h"

struct LeapMPC {
  KOMO komo;

  //for info only:
  arr x1, xT, tau;

  LeapMPC(rai::Configuration& C, double timingScale=1.);

  void reinit(const arr& x, const arr& v);
  void reinit(const rai::Configuration& C);

  void solve();
};
