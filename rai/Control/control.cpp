/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "control.h"

//===========================================================================

void naturalGains(double& Kp, double& Kd, double decayTime, double dampingRatio) {
  CHECK(decayTime>0. && dampingRatio>=0., "this does not define proper gains!");
  double lambda = decayTime*dampingRatio/(-log(.1));
//  double lambda = decayTime/(-log(.1)); //assume the damping ratio always 1. -- just so that setting ratio to zero still gives a reasonable value
  double freq = 1./lambda;
  Kp = freq*freq;
  Kd = 2.*dampingRatio*freq;
}

//===========================================================================

#if 0

void getForceControlCoeffs(arr& f_des, arr& u_bias, arr& K_I, arr& J_ft_inv, const arr& f_ref, double f_alpha, const CtrlObjective& co, const rai::Configuration& world) {
  //-- get necessary Jacobians
  ptr<TM_Default> m = std::dynamic_pointer_cast<TM_Default>(co.feat);
  CHECK(m, "this only works for the default position task feat");
  CHECK_EQ(m->type, TMT_pos, "this only works for the default positioni task feat");
  CHECK_GE(m->i, 0, "this only works for the default position task feat");
  rai::Frame* body = world.frames(m->i);
  rai::Frame* l_ft_sensor = world.getFrameByName("l_ft_sensor");
  arr J_ft, J;
  world.kinematicsPos(NoArr, J,   body, m->ivec);
  world.kinematicsPos_wrtFrame(NoArr, J_ft, body, m->ivec, l_ft_sensor);

  //-- compute the control coefficients
  u_bias = ~J*f_ref;
  f_des = f_ref;
  J_ft_inv = inverse_SymPosDef(J_ft*~J_ft)*J_ft;
  K_I = f_alpha*~J;
}

#else

void getForceControlCoeffs(arr& f_des, arr& u_bias, arr& K_I, arr& J_ft_inv, const arr& f_ref, double f_alpha, const CtrlObjective& co, const rai::Configuration& world) {
  NIY
}

#endif
