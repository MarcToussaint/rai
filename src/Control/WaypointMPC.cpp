/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "WaypointMPC.h"

#include "../Optim/options.h"
#include "../Optim/NLP.h"

WaypointMPC::WaypointMPC(KOMO& _komo, const arr& _qHome)
  : komo(_komo), qHome(_qHome) {

  if(!qHome.N) qHome=_komo.world.getJointState();

//  komo.reset();
//  komo.initWithConstant(qHome);
  path = komo.getPath_qOrg();
  this->tau = komo.getPath_tau();
}

void WaypointMPC::reinit(const rai::Configuration& C) {
  komo.updateRootObjects(C);
//  komo.setConfiguration_qOrg(-1, C.getJointState());
}

std::shared_ptr<SolverReturn> WaypointMPC::solve(int verbose) {
  steps++;

  //re-run KOMO
  rai::OptOptions opt;
  opt
  .set_verbose(0)
  .set_damping(1e1)
  .set_maxStep(0.1)
  .set_stopTolerance(1e-3)
  .set_stopEvals(200);
  komo.opt.verbose=0;
  komo.timeTotal=0.;
  komo.pathConfig.setJointStateCount=0;
//  cout <<komo.report(true, false) <<endl;
//  komo.initWithConstant(qHome);
//  komo.opt.animateOptimization=2;
  auto ret = komo.optimize(.0, -1, opt);
//  komo.checkGradients();
//  cout <<komo.report() <<endl;

  //is feasible?
  feasible=ret->sos<50. && ret->ineq<.1 && ret->eq<.1;

  path = komo.getPath_qOrg();
  tau = komo.getPath_tau();

  msg.clear() <<"WAY it " <<steps <<" feasible: " <<(feasible?" good":" FAIL") <<" -- queries: " <<komo.pathConfig.setJointStateCount <<" time:" <<komo.timeTotal <<"\t sos:" <<ret->sos <<"\t ineq:" <<ret->ineq <<"\t eq:" <<ret->eq <<endl;
  if(!feasible) msg <<komo.report();

  if(verbose>0) {
    komo.view(false, msg);
  }

  if(!feasible) { // || komo.pathConfig.setJointStateCount>50
    komo.reset();
    komo.initWithConstant(qHome);
  }

  return ret;
}
