/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "SecMPC.h"

#include "../Optim/NLP_Solver.h"
#include "../KOMO/pathTools.h"
#include "../Kin/F_qFeatures.h"

#include <iomanip>

//===========================================================================

SecMPC::SecMPC(KOMO& komo, int subSeqStart, int subSeqStop, double timeCost, double ctrlCost,
               bool _setNextWaypointTangent, const StringA& explicitCollisions)
  : waypointMPC(komo),
    timingMPC(waypointMPC.path({subSeqStart, subSeqStop}), timeCost, ctrlCost),
shortMPC(komo.world, 5, .1),
subSeqStart(subSeqStart), subSeqStop(subSeqStop), setNextWaypointTangent(_setNextWaypointTangent) {

  for(uint i=0; i<explicitCollisions.d0; i++) {
    CHECK_EQ(explicitCollisions.d1, 2, "");
    shortMPC.komo.addObjective({}, FS_distance, explicitCollisions[i], OT_ineqP, {1e1}, {-.001});
  }
//  StringA colls = {"l_palm", "l_finger1", "l_finger2", "l_panda_coll7b", "l_panda_coll7", "l_panda_coll6", "l_panda_coll5", "l_panda_coll4", "l_panda_coll3"};
//  if(komo.world["stick"]) colls.append("stick");
//  for(auto& s:colls){
//    shortMPC.komo.addObjective({}, FS_distance, {"obst", s}, OT_ineqP, {5.}, {-.1});
//  }

  if(waypointMPC.qHome.N) {
    for(uint t=0; t<shortMPC.komo.T; t++) {
      shortMPC.komo.addObjective({0.}, FS_qItself, {}, OT_sos, {1.}, waypointMPC.qHome, 0, t+1, t+1);
    }
  }

  if(setNextWaypointTangent) timingMPC.set_updatedWaypoints(timingMPC.waypoints, true);

  if(opt.verbose>0) {
    LOG(0) <<"new SecMPC with following waypoint komo:";
    cout <<waypointMPC.komo.report(true, false) <<endl;
  }
}

//===========================================================================

void SecMPC::updateWaypoints(const rai::Configuration& C) {
  waypointMPC.reinit(C); //adopt all frames in C as prefix (also positions of objects)
  auto ret = waypointMPC.solve(opt.verbose-2);

  if(!waypointMPC.feasible) wayInfeasible++; else wayInfeasible=0;

  msg <<" WAY #" <<waypointMPC.komo.pathConfig.setJointStateCount;
  msg <<' ' <<ret->sos <<'|' <<ret->ineq + ret->eq;
  if(!waypointMPC.feasible) msg <<'!' <<wayInfeasible <<"\n  " <<waypointMPC.msg;
}

//===========================================================================

void SecMPC::updateTiming(const rai::Configuration& C, const ObjectiveL& phi, const arr& q_real) {
  //-- adopt the new path
  timingMPC.set_updatedWaypoints(waypointMPC.path({subSeqStart, subSeqStop}), setNextWaypointTangent);

  //-- progress time (potentially phase)
  if(!timingMPC.done() && ctrlTimeDelta>0.) {
    phaseSwitch = timingMPC.set_progressedTime(ctrlTimeDelta, opt.tauCutoff);
  } else {
    phaseSwitch = false;
  }

  arr tauExpected = timingMPC.tau;

  //-- phase backtracking
  if(timingMPC.done()) {
    if(phi.maxError(C, timingMPC.phase+subSeqStart) > opt.precision) {
      phi.maxError(C, timingMPC.phase+subSeqStart, 1); //verbose
      timingMPC.update_backtrack();
      phaseSwitch = true;
    }
  }
  if(!timingMPC.done()) {
    while(timingMPC.phase>0 && phi.maxError(C, 0.5+timingMPC.phase+subSeqStart) > opt.precision) { //OR while?
      phi.maxError(C, 0.5+timingMPC.phase+subSeqStart, 1); //verbose
      timingMPC.update_backtrack();
      phaseSwitch = true;
    }
  }

  msg <<" \tTIMING";
  //-- re-optimize the timing
  if(!timingMPC.done()) {
    if(timingMPC.tau(timingMPC.phase) > opt.tauCutoff) {
      shared_ptr<SolverReturn> ret;
      double ctrlErr = length(q_real-q_ref_atLastUpdate);
      double thresh = .02;
      //cout <<"err: " <<err <<"  \t" <<flush;
      if(ctrlErr>thresh) {
        //LOG(0) <<"ERROR MODE: " <<ctrlErr <<endl;
        q_refAdapted = q_ref_atLastUpdate + ((ctrlErr-thresh)/ctrlErr) * (q_real-q_ref_atLastUpdate);
        ret = timingMPC.solve(q_refAdapted, qDot_ref_atLastUpdate, opt.verbose-3);
      } else {
        q_refAdapted.clear();
        q_refAdapted = q_ref_atLastUpdate;
        ret = timingMPC.solve(q_ref_atLastUpdate, qDot_ref_atLastUpdate, opt.verbose-3);
      }
      msg <<" #" <<ret->evals;
      //      msg <<" T:" <<ret->time <<" f:" <<ret->f;
    } else {
      msg <<" skip";
    }
  }

  //check if tau progress is stalling
  if(max(timingMPC.tau - tauExpected) > .8*ctrlTimeDelta) tauStalling++;
  else tauStalling=0;

  msg <<" ph:" <<timingMPC.phase <<" tau:" <<timingMPC.tau;
  msg <<timingMPC.tau - tauExpected;

//  msg <<ctrlTime_atLastUpdate + timingMPC.getTimes(); // <<' ' <<F.vels;
  if(phaseSwitch && opt.verbose>0) LOG(0) <<"phase switch to ph: " <<timingMPC.phase;
}

//===========================================================================

void SecMPC::updateShortPath(const rai::Configuration& C) {
  shortMPC.reinit(C); //adopt all frames in C as prefix (also positions of objects)
  shortMPC.reinit(q_refAdapted, qDot_ref_atLastUpdate);
  rai::CubicSpline S;
#if 0
  timingMPC.getCubicSpline(S, q_ref, qDot_ref);
  if(!S.pieces.N) return;
#else
//  timingMPC.getCubicSpline(S, q_ref_atLastUpdate, qDot_ref_atLastUpdate);
  auto sp = getSpline(ctrlTime_atLastUpdate, true);
  if(!sp.pts.N) { shortMPC.feasible=false; return; }
  S.set(sp.pts, sp.vels, sp.times);
#endif
  arr times = shortMPC.komo.getPath_times();
  arr pts = S.eval(times);
  CHECK_EQ(times.N, shortMPC.komo.T, "");
  CHECK_EQ(pts.d0, shortMPC.komo.T, "");
  for(int t=0; t<(int)pts.d0; t++) {
    shortMPC.komo.setConfiguration_qOrg(t, q_refAdapted); //pts[t]);
    std::shared_ptr<GroundedObjective> ob = shortMPC.komo.objs.elem(t - (int)pts.d0);
    ob->feat->setTarget(pts[t]);
//    cout <<off <<' ' <<t <<' ' <<ob->feat->shortTag(C) <<ob->feat->scale <<ob->feat->target <<ob->timeSlices <<endl;
  }
  shortMPC.komo.run_prepare(0.);
  //shortMPC.reinit_taus(times(0));

  auto ret = shortMPC.solve(false, opt.verbose-2);

//  shortMPC.komo.view(false, "SHORT");
//  shortMPC.feasible = true;
//  shortMPC.times = shortMPC.komo.getPath_times(); //grid(1, .0, .5, 10).reshape(-1);
//  shortMPC.times.prepend(0);
//  shortMPC.path = S.eval(shortMPC.times);
//  shortMPC.vels = S.eval(shortMPC.times, 1);

//  cout <<init - shortMPC.path <<endl;
//  cout <<shortMPC.komo.getConfiguration_qOrg(-2) <<endl;
//  cout <<shortMPC.komo.getConfiguration_qOrg(-1) <<endl;
//  cout <<shortMPC.komo.getConfiguration_qOrg(-0) <<endl;
//  cout <<q_ref <<endl <<qDot_ref <<endl;
//  cout <<init <<endl <<shortMPC.path <<endl;
//  rai::wait();
//  cout <<shortMPC.komo.report(true, false) <<endl;

  msg <<" \tPATH #" <<shortMPC.komo.pathConfig.setJointStateCount;
  msg <<' ' <<ret->sos <<'|' <<ret->ineq + ret->eq;
  if(!shortMPC.feasible) msg <<'!' <<wayInfeasible;
}

//===========================================================================

void SecMPC::cycle(const rai::Configuration& C, const arr& q_ref, const arr& qDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime) {
  //-- store ctrl state at start of this cycle
  if(ctrlTime_atLastUpdate>0.) {
    ctrlTimeDelta = ctrlTime - ctrlTime_atLastUpdate;
  }
  ctrlTime_atLastUpdate = ctrlTime;
  q_ref_atLastUpdate = q_ref;
  qDot_ref_atLastUpdate = qDot_ref;

  msg.clear();
  msg <<std::setprecision(3);
  msg <<"SecMPC d:" <<ctrlTimeDelta;

  updateWaypoints(C);
  updateTiming(C, waypointMPC.komo.objectives, q_real);
  updateShortPath(C);
}

rai::CubicSplineCtor SecMPC::getSpline(double realtime, bool prependRef) {
  if(!waypointMPC.feasible) return {};
//  if(timingMPC.done() || !waypointMPC.feasible) return {};
  arr pts = timingMPC.getWaypoints();
  arr vels = timingMPC.getVels();
  arr times = timingMPC.getTimes();
  CHECK_EQ(vels.d0, times.N, "");
  times -= realtime - ctrlTime_atLastUpdate; //ctrlTimeLast=when the timing was optimized; realtime=time now; -> shift spline to stich it at realtime
//  if(times.first()<tauCutoff) return {};
  if(q_refAdapted.N) { //this will overrideHard the spline, as first time is negative;
    pts.prepend(q_refAdapted);
    vels.prepend(qDot_ref_atLastUpdate);
    times.prepend(0. - (realtime - ctrlTime_atLastUpdate));
  } else if(prependRef) {
    pts.prepend(q_ref_atLastUpdate);
    vels.prepend(qDot_ref_atLastUpdate);
    times.prepend(0. - (realtime - ctrlTime_atLastUpdate));
  }
  return {pts, vels, times};
}

rai::CubicSplineCtor SecMPC::getShortPath_debug(double realtime) {
  if(timingMPC.done() || !waypointMPC.feasible) return {};

  rai::CubicSpline S;
//  timingMPC.getCubicSpline(S, q_ref_atLastUpdate, qDot_ref_atLastUpdate);
  auto sp = getSpline(ctrlTime_atLastUpdate, true);
  if(!sp.pts.N) return {};
  S.set(sp.pts, sp.vels, sp.times);

#if 1
  arr times = grid(1, .0, .5, 10).reshape(-1);
  arr pts = S.eval(times);
  arr vels = S.eval(times, 1);
  vels.clear();
#else
  arr times = sp.times, pts=sp.pts, vels=sp.vels;
#endif

  times -= realtime - ctrlTime_atLastUpdate; //ctrlTimeLast=when the timing was optimized; realtime=time now; -> shift spline to stich it at realtime
//  while(times.N && times.first()<tauCutoff){
//    times.remove(0);
//    pts.delRows(0);
//    vels.delRows(0);
//  }
  return {pts, vels, times};
}

rai::CubicSplineCtor SecMPC::getShortPath(double realtime) {
  if(/*timingMPC.done() || */!waypointMPC.feasible || !shortMPC.feasible) { return {}; }
  arr times = shortMPC.times; //komo.getPath_times();
  arr pts = shortMPC.path;
  arr vels = shortMPC.vels;
  if(!pts.N) return {};

  times -= realtime - ctrlTime_atLastUpdate; //ctrlTimeLast=when the timing was optimized; realtime=time now; -> shift spline to stich it at realtime
//  while(times.N && times.first()<tauCutoff){
//    times.remove(0);
//    pts.delRows(0);
//    if(vels.N) vels.delRows(0);
//  }
  return {pts, vels, times};
}

void SecMPC::report(const rai::Configuration& C) {
#if 0
  const ObjectiveL& phi = waypointMPC.komo.objectives;
  msg <<" \tFEA " <<phi.maxError(C, 0.5+timingMPC.phase)
      <<' ' <<phi.maxError(C, 1.+timingMPC.phase)
      <<' ' <<phi.maxError(C, 1.5+timingMPC.phase)
      <<' ' <<phi.maxError(C, 2.+timingMPC.phase);
#endif
  cout <<msg <<endl;
}
