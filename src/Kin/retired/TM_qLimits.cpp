/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_qLimits.h"

//===========================================================================

void LimitsConstraint::phi(arr& y, arr& J, const rai::Configuration& G) {
//  if(!limits.N)
  limits = G.getLimits();
  G.kinematicsLimits(y, J, limits, margin);
  y -= .5;
}

