#pragma once

#include "CtrlObjective.h"

//===========================================================================

struct CtrlSet {
  rai::Array<std::shared_ptr<CtrlObjective>> objectives;    ///< list of objectives
  std::shared_ptr<CtrlObjective> addObjective(const ptr<Feature>& f, ObjectiveType type, bool transient);
  void report(ostream& os=std::cout);
};
