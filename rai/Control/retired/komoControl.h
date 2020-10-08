/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "../KOMO/komo.h"

struct KOMO_Control : KOMO {
  arr q;
  double sos, eq, ineq;

  KOMO_Control() {}

  void setup(const rai::Configuration& C, double tau=.01, double accCosts=1., double velCosts=1, bool avoidCollisions=true);

  void setBounds(double maxVel=1., double maxAcc=1.);

  void updateConfiguration(const rai::Configuration& C);

  void step(const arr& real_q);

  arr getStep() {
    return q - getConfiguration_t(-1).getJointState();
  }

};
