#include "flagHunter.h"
#include "timingOpt.h"
#include "../Optim/MP_Solver.h"

FlagHuntingControl::FlagHuntingControl(const arr& _flags, double _alpha)
  : flags(_flags), alpha(_alpha){

  tau = ones(flags.d0);

  opt .set_maxStep(1e0)
      .set_stopTolerance(1e-4)
      .set_damping(1e-2)
      .set_verbose(rai::getParameter<int>("opt/verbose"));
}

shared_ptr<SolverReturn> FlagHuntingControl::solve(const arr& x0, const arr& v0, int verbose){
  if(!vels.N){
    vels = zeros(flags.d0-1, flags.d1);
    if(tangents.N) vels=zeros(flags.d0-1);
  }

  TimingProblem mp(flags({phase, -1}), tangents({phase, -1}), x0, v0, alpha, vels({phase, -1}), tau({phase, -1}));

  auto ret = MP_Solver()
             .setOptions(opt)
             .setProblem(mp.ptr())
             .setSolver(MPS_newton)
             .solve();

  if(verbose>1){
    cout <<*ret <<endl;
    cout <<"## vels:\n" <<mp.v <<endl;
    cout <<"## taus: " <<mp.tau <<endl;
  }

  tau({phase, -1}) = mp.tau;
  vels({phase, -1}) = mp.v;

  if(verbose>0){
    cout <<"FLAGS phase: " <<phase <<" tau: " <<tau <<endl;
  }
  return ret;
}

arr FlagHuntingControl::getVels() const{
  if(done()) return arr{};
  arr _vels;
  if(!tangents.N){
    _vels = vels({phase, -1}).copy();
  }else{
    _vels = (vels%tangents)({phase, -1}).copy();
  }
  _vels.append(zeros(flags.d1));
  _vels.reshape(flags.d0 - phase, flags.d1);
  return _vels;
}

void FlagHuntingControl::update_progressTime(double gap){
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

void FlagHuntingControl::update_flags(const arr& _flags){
  flags = _flags;
  tangents[-1] = flags[-1] - flags[-0];
  op_normalize(tangents[-1]());
}

void FlagHuntingControl::update_backtrack(){
  LOG(0) <<"backtracking " <<phase <<"->" <<phase-1 <<" tau:" <<tau;
  CHECK(phase>0, "");
  phase--;
  tau(phase) = 1.;
}

void FlagHuntingControl::getCubicSpline(rai::CubicSpline& S, const arr& x0, const arr& v0) const{

  arr _pts = getFlags();
  arr _times = getTimes();
  arr _vels = getVels();
  _pts.prepend(x0);
  _vels.prepend(v0);
  _times.prepend(0.);

  S.set(_pts, _vels, _times);

  //  //check spline errors
  //  for(uint k=0;k<times.N;k++){
  //    double t=times(k);
  //    cout <<k <<' ' <<t <<' ' <<pts[k] <<' ' <<sumOfSqr(pts[k]-S.eval(t)) <<' ' <<vels[k] <<' ' <<sumOfSqr(vels[k] - S.eval(t+1e-6, 1)) <<' ' <<sumOfSqr(vels[k] - S.eval(t-1e-6, 1)) <<endl;
  //  }
}
