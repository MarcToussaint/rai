#include "komoControl.h"


void KOMO_Control::setup(const rai::Configuration& C, double tau, double accCosts, double velCosts, bool avoidCollisions) {

  setModel(C, true);
  setTiming(1., 1, tau, 2);
  add_qControlObjective({}, 2, accCosts);
  add_qControlObjective({}, 1, velCosts);
  if(avoidCollisions) add_collision(true);

  q = C.getJointState();
  setupConfigurations(q);
  verbose=0;
}

void KOMO_Control::step(const arr& real_q){
  if(!!real_q.N && real_q.N) q = real_q;

  arr q_1 = getConfiguration_t(-1).getJointState();
  setConfiguration(-2, q_1);
  setConfiguration(-1, q);
  setConfiguration(0, q + (q - q_1));

  OptOptions opt;
  opt.stopTolerance=1e-4;
  optimize(0., opt);

  {
    rai::Graph R = getReport(false);
    sos = R.get<double>("sos");
    eq = R.get<double>("eq");
    ineq = R.get<double>("ineq");
  }

  q = getConfiguration_t(0).getJointState();
}
