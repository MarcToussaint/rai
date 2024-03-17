/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "LeapMPC.h"

LeapMPC::LeapMPC(rai::Configuration& C, double timingScale) {
  komo.setConfig(C, false);
#if 0
  komo.setTiming(2., 1, .1, 2);

  //control costs at short horizon
  komo.addControlObjective({1}, 2, 1e0);

  //leap costs for the leap
  komo.addObjective({2}, make_shared<CubicSplineLeapCost>(C.getCtrlFramesAndScale()), {}, OT_sos, {1.}, NoArr, 2);

  //add a time joint for just the last slice
  {
    rai::Joint* jt = new rai::Joint(*komo.timeSlices(-1, 0), rai::JT_tau);
    jt->H = 0.;
    //timing cost
    komo.addObjective({2}, make_shared<F_qTime>(), {"world"}, OT_f, {timingScale}, {});
    komo.addObjective({2}, make_shared<F_qTime>(), {"world"}, OT_ineq, {-1e1}, {.1}); //lower bound on timing
  }
  cout <<komo.report(true, false) <<endl;

  komo.timeSlices(-1, 0)->setJointState({100.}); //this should be the tau joint!
#else
  komo.setTiming(1., 3, 1., 1);

  //control costs at short horizon
  komo.addControlObjective({}, 1, 1e-1);
  cout <<komo.report(true, false) <<endl;
#endif

  //MISSING: THE TASK COSTS..
}

void LeapMPC::reinit(const arr& x, const arr& v) {
  //set the prefix to init:
  komo.setConfiguration_qOrg(-1, x);
  komo.setConfiguration_qOrg(-2, x - komo.tau*v);
  //initialize x(0) also to current
  komo.setConfiguration_qOrg(0, x);
  //leave the leap configuration as is...
//  komo.timeSlices(-1,0)->setJointState({100.}); //this should be the tau joint!
}

void LeapMPC::reinit(const rai::Configuration& C) {
  //shifts only prefix, not the whole trajectory! (would not make sense for x(H) \gets x(T) )
  komo.updateAndShiftPrefix(C);
}

void LeapMPC::solve() {
  //re-run KOMO
  rai::OptOptions opt;
  opt.stopTolerance = 1e-4;
  opt.stopGTolerance = 1e-4;
  komo.opt.verbose=0;
  komo.optimize(0., opt);
  //komo.checkGradients();

  //store as output result
  x1 = komo.getConfiguration_qOrg(0);
  xT = komo.getConfiguration_qOrg(komo.T-1); //the last one
  tau = komo.getPath_tau();
}
