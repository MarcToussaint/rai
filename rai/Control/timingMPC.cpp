#include "timingMPC.h"
#include "timingOpt.h"
#include "../Optim/MP_Solver.h"

TimingMPC::TimingMPC(const arr& _flags, double _timeCost, double _ctrlCost)
  : waypoints(_flags),
    timeCost(_timeCost),
    ctrlCost(_ctrlCost){

  tau = 10.*ones(waypoints.d0);

  opt .set_maxStep(1e0)
      .set_stopTolerance(1e-4)
      .set_damping(1e-2);
}

shared_ptr<SolverReturn> TimingMPC::solve(const arr& x0, const arr& v0, int verbose){
  if(!vels.N){
    vels = zeros(waypoints.d0-1, waypoints.d1);
    if(tangents.N) vels=zeros(waypoints.d0-1);
  }

  TimingProblem mp(waypoints({phase, -1}), tangents({phase, -1}),
                   x0, v0, timeCost,
                   vels({phase, -1}), tau({phase, -1}),
                   true,
                   -1., -1., -1., ctrlCost);

  MP_Solver S;
  if(warmstart_dual.N){
//    S.setWarmstart({}, warmstart_dual);
//    opt.muInit = 25;
  }
  S.setOptions(opt)
      .setProblem(mp.ptr())
      .setSolver(MPS_augmentedLag);

  auto ret = S.solve();

  if(verbose>1){
    cout <<*ret <<endl;
    cout <<"## vels:\n" <<mp.v <<endl;
    cout <<"## taus: " <<mp.tau <<endl;
  }

  tau({phase, -1}) = mp.tau;
  vels({phase, -1}) = mp.v;
  warmstart_dual = ret->dual;

  if(verbose>0){
    cout <<"FLAGS phase: " <<phase <<" tau: " <<tau <<endl;
  }
  return ret;
}

arr TimingMPC::getVels() const{
  if(done()) return arr{};
  arr _vels;
  if(!tangents.N){
    _vels = vels({phase, -1}).copy();
  }else{
    _vels = (vels%tangents)({phase, -1}).copy();
  }
  _vels.append(zeros(waypoints.d1));
  _vels.reshape(waypoints.d0 - phase, waypoints.d1);
  return _vels;
}

void TimingMPC::update_progressTime(double gap){
  if(gap < tau(phase)){ //time still within phase
    tau(phase) -= gap; //change initialization of timeOpt
  }else{ //time beyond current phase
    if(phase+1<tau.N){ //if there exists another phase
      tau(phase+1) -= gap-tau(phase); //change initialization of timeOpt
      tau(phase) = 0.; //change initialization of timeOpt
    }else{
      tau = 0.;
    }
    phase++; //increase phase
  }
}

void TimingMPC::update_flags(const arr& _flags){
  waypoints = _flags;
  if(tangents.N){
    for(uint k=1; k<waypoints.d0; k++){
      tangents[k-1] = waypoints[k] - waypoints[k-1];
      op_normalize(tangents[k-1]());
    }
  }
}

void TimingMPC::update_backtrack(){
  CHECK(phase>0, "");
  uint phaseTo = phase-1;
  if(backtrackingTable.N) phaseTo = backtrackingTable(phase);
  update_setPhase(phaseTo);
}

void TimingMPC::update_setPhase(uint phaseTo){
  LOG(0) <<"backtracking " <<phase <<"->" <<phaseTo <<" tau:" <<tau;
  CHECK_LE(phaseTo, phase, "");
  while(phase>phaseTo){
    if(phase<tau.N) tau(phase) = rai::MAX(1., tau(phase));
    phase--;
  }
  tau(phase) = 1.;
}

void TimingMPC::getCubicSpline(rai::CubicSpline& S, const arr& x0, const arr& v0) const{

  arr _pts = getFlags();
  arr _times = getTimes();
  arr _vels = getVels();
  _pts.prepend(x0);
  _vels.prepend(v0);
  _times.prepend(0.);

  if(_times.N>1){
    S.set(_pts, _vels, _times);
  }

  //  //check spline errors
  //  for(uint k=0;k<times.N;k++){
  //    double t=times(k);
  //    cout <<k <<' ' <<t <<' ' <<pts[k] <<' ' <<sumOfSqr(pts[k]-S.eval(t)) <<' ' <<vels[k] <<' ' <<sumOfSqr(vels[k] - S.eval(t+1e-6, 1)) <<' ' <<sumOfSqr(vels[k] - S.eval(t-1e-6, 1)) <<endl;
  //  }
}
