/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "PathSmoother.h"

#include "../KOMO/komo.h"
#include "../Kin/viewer.h"
#include "../Optim/NLP_Solver.h"

arr ReceedingHorizonPathSmoother::run(int verbose) {
  bool disp = false;
  std::unique_ptr<rai::ConfigurationViewer> V;

  arr smoothed = initialPath;

  const double horizonDuration = totalDuration / smoothed.d0 * horizon;

  // TODO: Check better iteration termination criteria
  //Marc: perhaps have different parameters for the first optimization (conservative), and rest (with low damping)
//  options.stopIters = 10;
//  options.damping = 1e-1;

  // set up KOMO problem for part of the path
  KOMO komo;
  komo.setConfig(P.C, true);
  komo.setTiming(1., horizon, horizonDuration, 2);
  komo.opt.verbose = verbose;

  CHECK_EQ(komo.T, horizon, "");

  if(disp) {
    komo.opt.verbose = 6;
  }

  for(uint i=1; i<=initialPath.d0 - horizon; ++i) {
    if(verbose>1) LOG(0) << "Smoother Iteration " << i;

    komo.add_collision(true);
    komo.addControlObjective({}, 2, 1.);
    //-- set prefix configurations
    if(i<=1) {
      komo.setConfiguration_qOrg(-2, smoothed[0]);
      komo.setConfiguration_qOrg(-1, smoothed[0]);
    } else {
      komo.setConfiguration_qOrg(-2, smoothed[i-2]);
      komo.setConfiguration_qOrg(-1, smoothed[i-1]);
    }
    //-- initialize other configurations with the previous horizon
    for(uint j=0; j<horizon; ++j) {
      komo.setConfiguration_qOrg(j, smoothed[i+j]);
    }
    komo.run_prepare(0.); //mt: only this makes komo actually adopt the set configurations as decision variable komo.x!!! TODO: setConfiguration should komo.x.clear()...!

    // final obj
    komo.addObjective({1}, FS_qItself, {}, OT_eq, {1e1}, smoothed[i+komo.T-1]);

    // for velocity=0 at the end
    komo.addObjective({1}, FS_qItself, {}, OT_eq, {1e1}, NoArr, 1);

    NLP_Solver sol;
    sol.setProblem(komo.nlp());
    sol.opt.set_stopInners(10);
    sol.solve();

    // get results from komo
    for(uint j=0; j<horizon; ++j) {
      smoothed[i+j] = komo.getConfiguration_qOrg(j);
    }

    if(disp) {
      std::cout << komo.report() << std::endl;
      if(!V) V = make_unique<rai::ConfigurationViewer>();
      V->updateConfiguration(komo.pathConfig, komo.timeSlices);
      V->view(true, "smoothing...");
//      rai::wait();
    }

    komo.clearObjectives();
  }

  //copy last configuration, to ensure exact goal
  smoothed[-1] = initialPath[-1];

  return smoothed;
}
