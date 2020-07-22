#pragma once

#include "CtrlObjective.h"
#include <Kin/feature.h>

//===========================================================================

struct CtrlSet {
  rai::Array<std::shared_ptr<CtrlObjective>> objectives;    ///< list of objectives
  std::shared_ptr<CtrlObjective> addObjective(const ptr<Feature>& f, ObjectiveType type, double transientStep=-1.);
  void report(ostream& os=std::cout);
};

//===========================================================================

bool isFeasible(const CtrlSet& CS, const ConfigurationL& Ctuple, bool initOnly=true, double eqPrecision=1e-4);
