#include "WaypointMPC.h"

#include <Optim/options.h>

WaypointMPC::WaypointMPC(KOMO& _komo, const arr& _qHome)
  : komo(_komo), qHome(_qHome){

  if(!qHome.N) qHome=_komo.world.getJointState();

//  komo.reset();
//  komo.initWithConstant(qHome);
  path = komo.getPath_qOrg();
  this->tau = komo.getPath_tau();
}

void WaypointMPC::reinit(const rai::Configuration& C){
  komo.updateRootObjects(C);
//  komo.setConfiguration_qOrg(-1, C.getJointState());
}

void WaypointMPC::solve(int verbose){
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
//  komo.reportProblem();
//  komo.initWithConstant(qHome);
//  komo.opt.animateOptimization=2;
  komo.optimize(.0, opt);
//  komo.checkGradients();
//  cout <<komo.getReport(false) <<endl;

  //is feasible?
  feasible=komo.sos<50. && komo.ineq<.1 && komo.eq<.1;

  path = komo.getPath_qOrg();
  tau = komo.getPath_tau();

  msg.clear() <<"WAY it " <<steps <<" feasible: " <<(feasible?" good":" FAIL") <<" -- queries: " <<komo.pathConfig.setJointStateCount <<" time:" <<komo.timeTotal <<"\t sos:" <<komo.sos <<"\t ineq:" <<komo.ineq <<"\t eq:" <<komo.eq <<endl;
  if(!feasible) msg <<komo.getReport(false);

  if(verbose>0){
    komo.view(false, msg);
  }

  if(!feasible){ // || komo.pathConfig.setJointStateCount>50
    komo.reset();
    komo.initWithConstant(qHome);
  }
}
