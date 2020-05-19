#include "komoControl.h"


KOMO_Control::KOMO_Control(const rai::Configuration& C, bool avoidCollisions) {
  setModel(C, true);
  setTiming(1., 1, .1, 2);
  add_qControlObjective({}, 2, 1.);
  add_qControlObjective({}, 1, 1e1);
  if(avoidCollisions){
    add_collision(true);
  }
  q = C.getJointState();
  setupConfigurations(q);
  verbose=0;
}

void KOMO_Control::step(){
  arr q_1 = getConfiguration_t(-1).getJointState();
  setConfiguration(-2, q_1);
  setConfiguration(-1, q);
  setConfiguration(0, q + (q - q_1));

  optimize(0.);

  {
    rai::Graph R = getReport(false);
    sos = R.get<double>("sos");
    eq = R.get<double>("eq");
    ineq = R.get<double>("ineq");
  }

  q = getConfiguration_t(0).getJointState();
}
