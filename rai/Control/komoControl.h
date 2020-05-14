#include "../KOMO/komo.h"

struct KOMO_Control : KOMO {
  arr q;
  double sos, eq, ineq;

  KOMO_Control(const rai::Configuration& C, bool avoidCollisions);

  void step();

  arr getStep(){
    return q - getConfiguration_t(-1).getJointState();
  }

};
