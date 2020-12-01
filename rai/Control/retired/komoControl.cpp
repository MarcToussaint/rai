/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "komoControl.h"

void KOMO_Control::setup(const rai::Configuration& C, double tau, double accCosts, double velCosts, bool avoidCollisions) {

  setModel(C, true);
  setTiming(1., 1, tau, 2);

  if(accCosts>0.) add_qControlObjective({}, 2, accCosts);
  if(velCosts>0.) add_qControlObjective({}, 1, velCosts);
  if(avoidCollisions) add_collision(true, 0., 1e0);

  q = C.getJointState();
  setupConfigurations(q);
  verbose=0;
}

void KOMO_Control::setBounds(double maxVel, double maxAcc) {
  arr q_2 = getConfiguration_t(-2).getJointState();
  arr q_1 = getConfiguration_t(-1).getJointState();

  //set bounds to limits:
  KOMO::setBounds();

  //velocity bounds
  bound_lo = elemWiseMax(bound_lo, q_1 - maxVel*tau);
  bound_up = elemWiseMin(bound_up, q_1 + maxVel*tau);

  //acceleration bounds
  bound_lo = elemWiseMax(bound_lo, 2.*q_1 - q_2 - (maxAcc*tau*tau));
  bound_up = elemWiseMin(bound_up, 2.*q_1 - q_2 + (maxAcc*tau*tau));
}

void KOMO_Control::updateConfiguration(const rai::Configuration& C) {
  getConfiguration_t(0).setFrameState(C.getFrameState());
}

void KOMO_Control::step(const arr& real_q) {
  if(!!real_q.N && real_q.N) q = real_q;

  //push configurations
  arr q_1 = getConfiguration_t(-1).getJointState();
  setConfiguration(-2, q_1);
  setConfiguration(-1, q);
  setConfiguration(0, q + (q - q_1));

  //update bounds
//  setBounds(1., 100.);

  OptOptions opt;
  opt.stopTolerance=1e-4;
  opt.nonStrictSteps=-1;
  opt.maxStep = tau; //maxVel*tau;
//  verbose=6;
//  animateOptimization=2;
  optimize(0., opt);

  {
    rai::Graph R = getReport(false);
    sos = R.get<double>("sos");
    eq = R.get<double>("eq");
    ineq = R.get<double>("ineq");
  }

  q = getConfiguration_t(0).getJointState();
}

