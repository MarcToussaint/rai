/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "CtrlObjective.h"
#include "CtrlSet.h"

#include "../KOMO/komo.h"

//===========================================================================

struct CtrlSolver : NonCopyable {
  KOMO komo;
  double tau;
  double maxVel=1.;
  double maxAcc=1.;
  rai::Graph optReport;

  rai::Array<shared_ptr<CtrlObjective>> objectives;    ///< list of objectives

  CtrlSolver(rai::Configuration& _C, double _tau, uint k_order=1);
  ~CtrlSolver();

  void set(const CtrlSet& CS);
  void addObjectives(const rai::Array<ptr<CtrlObjective>>& O);
  void delObjectives(const rai::Array<ptr<CtrlObjective>>& O);

  void update(rai::Configuration& C);
  void report(ostream& os=std::cout);
  arr solve();

};
