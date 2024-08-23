/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "ShortPathMPC.h"
#include "../Control/timingOpt.h"
#include "../Optim/NLP_Solver.h"

ShortPathMPC::ShortPathMPC(rai::Configuration& C, uint steps, double _defaultTau)
  : defaultTau(_defaultTau) {
  qHome = C.getJointState();

  komo.setConfig(C, false);
  komo.setTiming(1., steps, defaultTau*steps, 2);
  sliceOfConstraint=komo.T-1;

  //control costs at short horizon
  //komo.addControlObjective({}, 1, 1e-1);
//  komo.add_jointLimits();
  komo.addControlObjective({}, 2, .1);
//  komo.addControlObjective({}, 0, 1e0);
  //komo.add_collision();
//  cout <<komo.report(true, false) <<endl;
}

void ShortPathMPC::reinit_taus(double timeToConstraint) {
#if 0
  komo.tau = timeToConstraint / komo.T + .01;
  komo.pathConfig.setTaus(komo.tau);
#else
  sliceOfConstraint = floor(timeToConstraint / defaultTau);
  if(sliceOfConstraint>(int)komo.T-2) sliceOfConstraint=komo.T-2;
  if(sliceOfConstraint<2) sliceOfConstraint = 2;

  //change taus?
//  double tauLast = timeToConstraint - defaultTau * sliceOfConstraint;
//  arr taus(komo.k_order+komo.T);
//  taus = defaultTau;
//  taus(komo.k_order+sliceOfConstraint) = tauLast;
//  LOG(0) <<timeToConstraint <<' ' <<sliceOfConstraint <<' ' <<taus;
//  komo.pathConfig.setTaus(taus);
  LOG(0) <<timeToConstraint <<' ' <<sliceOfConstraint; // <<' ' <<taus;

  komo.objs.popLast();
  komo.objs.popLast();
  shared_ptr<Objective> ob2 = komo.objectives.popLast();
  shared_ptr<Objective> ob1 = komo.objectives.popLast();
  komo._addObjective(ob1, intA{{1, 1}, {sliceOfConstraint}});
  komo._addObjective(ob2, intA{{1, 2}, {sliceOfConstraint, sliceOfConstraint+1}});
#endif

//  cout <<komo.report(true, false) <<endl;  rai::wait();
}

void ShortPathMPC::reinit(const arr& x, const arr& v) {
  x0=x; v0=v;
  //set the prefix to init:
  komo.setConfiguration_qOrg(-1, x);
  komo.setConfiguration_qOrg(-2, x - defaultTau*v);
  //initialize x(0) also to current
  komo.setConfiguration_qOrg(0, x);
  //leave the leap configuration as is...
//  komo.timeSlices(-1,0)->setJointState({100.}); //this should be the tau joint!
}

void ShortPathMPC::reinit(const rai::Configuration& C) {
  //shifts only prefix, not the whole trajectory! (would not make sense for x(H) \gets x(T) )
//  komo.updateAndShiftPrefix(C);
  komo.updateRootObjects(C);
}

std::shared_ptr<SolverReturn> ShortPathMPC::solve(bool alsoVels, int verbose) {
  iters++;

  //re-run KOMO
  rai::OptOptions opt;
  opt.set_stopTolerance(1e-3)
  .set_verbose(0);
  komo.opt.verbose=0;
  komo.timeTotal=0.;
  komo.pathConfig.setJointStateCount=0;
//  komo.initWithConstant(qHome);
  auto ret = komo.optimize(0., -1, opt);
  //komo.checkGradients();

  //is feasible?
  feasible=ret->sos<50. && ret->ineq<.1 && ret->eq<.1;

  if(verbose>0) {
    msg.clear() <<"SHORT it " <<iters <<" feasible: " <<(feasible?" good":" FAIL") <<" -- queries: " <<komo.pathConfig.setJointStateCount <<" time:" <<komo.timeTotal <<"\t sos:" <<ret->sos <<"\t ineq:" <<ret->ineq <<"\t eq:" <<ret->eq <<endl;
    komo.view(false, msg);
  }

  path = komo.getPath_qOrg();
  tau = komo.getPath_tau();
  times = komo.getPath_times();
  vels.clear();

  //store as output result
  if(feasible) {
    if(alsoVels) {
//      arr tangents = getVelocities_centralDifference(path, tau(0));
      TimingProblem timingProblem(path, {}, x0, v0, 1., 1., true, true, {}, tau);
      NLP_Solver solver;
      solver
      .setProblem(timingProblem.ptr())
      .setSolver(NLPS_newton);
      solver.opt
      .set_stopTolerance(1e-4)
      .set_maxStep(1e0)
      .set_damping(1e-2);
      auto ret = solver.solve();
      timingProblem.getVels(vels);
      LOG(1) <<"timing f: " <<ret->f <<' ' <<ret->evals <<'\n' <<vels;
      vels.prepend(v0);
    } else {
      vels.clear();
    }

    path.prepend(x0);
    times.prepend(0.);

  } else {
    cout <<komo.report();
    path.clear();
    times.clear();
//    komo.reset();
//    komo.initWithConstant(qHome);
  }

  return ret;
}

arr ShortPathMPC::getPath() {
  if(!feasible) return arr{};
  return path({0, sliceOfConstraint});
}
