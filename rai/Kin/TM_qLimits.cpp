/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_qLimits.h"

//===========================================================================

void TM_qLimits::phi(arr& y, arr& J, const rai::KinematicWorld& G) {
//  if(!limits.N)
  limits=G.getLimits(); //G might change joint ordering (kinematic switches), need to query limits every time
  G.kinematicsLimitsCost(y, J, limits);
}

//===========================================================================

void LimitsConstraint::phi(arr& y, arr& J, const rai::KinematicWorld& G) {
//  if(!limits.N)
  limits = G.getLimits();
  G.kinematicsLimitsCost(y, J, limits, margin);
  y -= .5;
}

