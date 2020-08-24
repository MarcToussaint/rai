/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "CtrlObjective.h"
#include <Kin/feature.h>

//===========================================================================

struct CtrlSet {
  rai::Array<std::shared_ptr<CtrlObjective>> objectives;    ///< list of objectives
  std::shared_ptr<CtrlObjective> addObjective(const ptr<Feature>& f, ObjectiveType type, double transientStep=-1.);
  void report(ostream& os=std::cout) const;
};

//===========================================================================

bool isFeasible(const CtrlSet& CS, const ConfigurationL& Ctuple, bool initOnly=true, double eqPrecision=1e-4);
