#pragma once

#include "CtrlObjective.h"
#include "CtrlProblem.h"
#include "CtrlTargets.h"
#include "CtrlSolvers.h"

//===========================================================================

void naturalGains(double& Kp, double& Kd, double decayTime, double dampingRatio);
void getForceControlCoeffs(arr& f_des, arr& u_bias, arr& K_I, arr& J_ft_inv, const arr& f_ref, double f_alpha, const CtrlObjective& co, const rai::Configuration& world);

