/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */
#pragma once

#include <Optim/optimization.h>
#include <Kin/kin.h>

//===========================================================================

struct EffectivePoseProblem:ConstrainedProblem{
  rai::KinematicWorld& effKinematics;
  const Graph& KB;
  const Graph& symbolicState_before;
  const Graph& symbolicState_after;
  arr x0;
  int verbose;
  EffectivePoseProblem(rai::KinematicWorld& effKinematics_before,
                       const Graph& KB, const Graph& symbolicState_before, const Graph& symbolicState_after,
                       int verbose);
  void phi(arr& phi, arr& phiJ, arr& H, ObjectiveTypeA& tt, const arr& x);

  double optimize(arr& x);
};

