#include "flagHunter.h"
#include "timingOpt.h"
#include "../Optim/MP_Solver.h"

FlagHuntingControl::FlagHuntingControl(const arr& _flags, double _alpha)
  : flags(_flags), alpha(_alpha){

  tangents = zeros(flags.d0-1, flags.d1);
  for(uint k=0;k<tangents.d0;k++) tangents(k, 2) = 1.;
  tangents.clear();

  tau = ones(flags.d0);

  vels = zeros(flags.d0-1, flags.d1), tau;
  if(tangents.N) vels=zeros(flags.d0-1);

  opt .set_maxStep(1e0)
      .set_stopTolerance(1e-4)
      .set_damping(1e-2)
      .set_verbose(rai::getParameter<int>("opt/verbose"));
}

shared_ptr<SolverReturn> FlagHuntingControl::solve(const arr& x0, const arr& v0, int verbose){
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
