/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "CtrlObjective.h"

#include "../KOMO/komo.h"

//===========================================================================

struct CtrlProblem : NonCopyable {
  KOMO komo;
  double tau;
  double maxVel=1.;
  double maxAcc=1.;

  CtrlObjectiveL objectives;    ///< list of objectives

  CtrlProblem(rai::Configuration& _C, double _tau, uint k_order=1);
  CtrlObjective* addPDTask(CtrlObjectiveL& tasks, const char* name, double decayTime, double dampingRatio, ptr<Feature> map);

  void addObjectives(const rai::Array<ptr<CtrlObjective>>& O);
  void delObjectives(const rai::Array<ptr<CtrlObjective>>& O);
  std::shared_ptr<CtrlObjective> addObjective(const ptr<Feature>& f, ObjectiveType type);
  std::shared_ptr<CtrlObjective> addObjective(const FeatureSymbol& feat, const StringA& frames,
      ObjectiveType type, const arr& scale=NoArr, const arr& target=NoArr, int order=-1);

  std::shared_ptr<CtrlObjective> add_qControlObjective(uint order, double scale=1., const arr& target=NoArr);

  void update(rai::Configuration& C);
  void report(ostream& os=std::cout);
  arr solve();

};
