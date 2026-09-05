/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "CtrlObjective.h"
#include "CtrlSet.h"

#include "../KOMO/komo.h"

//===========================================================================

struct CtrlSolver : rai::NonCopyable {
  KOMO komo;
  double tau;
  double maxVel=1.;
  double maxAcc=1.;
  rai::Graph optReport;

  rai::Array<shared_ptr<CtrlObjective>> objectives;    ///< list of objectives

  CtrlSolver(const rai::Configuration& _C, double _tau, uint k_order=1);
  ~CtrlSolver();

  void set(const CtrlSet& CS);
  void addObjectives(const rai::Array<shared_ptr<CtrlObjective>>& O);
  void delObjectives(const rai::Array<shared_ptr<CtrlObjective>>& O);

  void update(const arr& q_real, const arr& qDot_real, rai::Configuration& C);
  void report(ostream& os=std::cout);
  arr solve();

};
