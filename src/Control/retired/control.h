/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "CtrlObjective.h"
#include "CtrlSet.h"
#include "CtrlSolver.h"
#include "CtrlTargets.h"

//===========================================================================

void naturalGains(double& Kp, double& Kd, double decayTime, double dampingRatio);
void getForceControlCoeffs(arr& f_des, arr& u_bias, arr& K_I, arr& J_ft_inv, const arr& f_ref, double f_alpha, const CtrlObjective& co, const rai::Configuration& world);

