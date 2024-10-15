/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "pathTools.h"
#include "komo.h"
#include "manipTools.h"
#include "../Kin/F_collisions.h"

arr getVelocities_centralDifference(const arr& q, double tau) {
  arr v;
  v.resizeAs(q);
  for(uint t=1; t<q.d0-1; t++) {
    v[t] = (q[t+1]-q[t-1])/(2.*tau);
  }
  v[0] = (q[1] - q[0])/tau;
  v[-1] = (q[-1] - q[-2])/tau;
  return v;
}

arr getAccelerations_centralDifference(const arr& q, double tau) {
  arr a;
  a.resizeAs(q);
  for(uint t=1; t<q.d0-1; t++)  a[t] = (q[t+1] + q[t-1] - 2.*q[t])/(tau*tau);
  a[0] = a[1]/2.;
  a[-1] = a[-2]/2.;
  return a;
}

double getMinDuration(const arr& q, double maxVel, double maxAcc) {
  arr v = getVelocities_centralDifference(q, 1.);
  arr a = getAccelerations_centralDifference(q, 1.);

  CHECK(maxVel>0. || maxAcc>0., "");

  double vscale = maxVel>0. ? maxVel / absMax(v) : 1e10;
  double ascale = maxAcc>0. ? sqrt(maxAcc / absMax(a)) : 1e10;
  double tau = 1./ rai::MIN(vscale, ascale);

  v = getVelocities_centralDifference(q, tau);
  a = getAccelerations_centralDifference(q, tau);
  //cout <<absMax(v) <<' ' <<absMax(a) <<endl;

  return q.d0*tau;
}

arr getSineProfile(const arr& q0, const arr& qT, uint T) {
  arr q(T+1, q0.N);
  for(uint t=0; t<=T; t++) q[t] = q0 + .5 * (1.-cos(RAI_PI*t/T)) * (qT-q0);
  return q;
}

arr reversePath(const arr& q) {
  uint T=q.d0-1;
  arr r(T+1, q.d1);
  for(uint t=0; t<=T; t++) r[T-t] = q[t];
  return r;
}

rai::String validatePath(const rai::Configuration& _C, const arr& q_now, const StringA& joints, const arr& q, const arr& times) {
  rai::Configuration C;
  C.copy(_C, true);

//  arr q0 = K.getJointState();

//  syncModelJointStateWithRealOrSimulation();

//  arr q_now = K.getJointState(joints);

  CHECK_EQ(q_now.N, q.d1, "");

  rai::String txt;
  txt <<"VALIDATE ";

  if(q.d0>1) {
    double startVel, endVel, maxVel=0.;
    startVel = length(q[0]-q_now)/(times(0));
    endVel = length(q[-1]-q[-2])/(times(-1)-times(-2));
    for(uint t=1; t<q.d0; t++) {
      double v = length(q[t]-q[t-1])/(times(t)-times(t-1));
      if(v>maxVel) maxVel = v;
    }
    txt <<"\nv0=" <<startVel <<" vT=" <<endVel <<" vMax=" <<maxVel;
  }
  if(joints.N<=3) {
    txt <<"\n" <<joints;
  }
  txt <<"\n";
  return txt;
}

//  PlanDrawer planDrawer(K, q_now, joints, q, tau);
//  K.gl().remove(K);
//  K.gl().add(planDrawer);
//  for(;;){
//    int key = K.view(true, txt);

//    if(key==13){ //validated
//      K.gl().remove(planDrawer);
//      K.gl().add(K);
//      K.setJointState(q0);
//      K.view(false, "validated");
//      return;
//    }

//    if(key==27){
//      LOG(0) <<"NO VALIDATION - exiting";
//      K.gl().closeWindow();
//      exit(0);
//    }
//  }
//}

std::pair<arr, arr> getStartGoalPath_obsolete(const rai::Configuration& C, const arr& target_q, const StringA& target_joints, const char* endeff, double up, double down) {
  KOMO komo;
  komo.setConfig(C, true);
  komo.setTiming(1., 20, 3.);
  komo.addControlObjective({}, 2, 1.);

  if(endeff) {
    if(up>0.) {
      komo.addObjective({0., up}, FS_position, {endeff}, OT_sos, {1e2}, {0., 0., .05}, 2);
    }
    if(down>0.) {
      komo.addObjective({down, 1.}, FS_position, {endeff}, OT_sos, {1e2}, {0., 0., -.05}, 2);
    }
  }

  komo.addObjective({1., 1.}, FS_qItself, target_joints, OT_eq, {1e1}, target_q);

  komo.setSlow(0., 0., 1e2, true);
  komo.setSlow(1., 1., 1e2, true);

  komo.opt.verbose=1;
  komo.optimize();

  arr path = komo.getPath_qOrg();
  path[path.d0-1] = target_q; //overwrite last config
  arr times = komo.getPath_times();
  cout <<validatePath(C, C.getJointState(), target_joints, path, times) <<endl;
  int key = komo.view(true);
  if(key=='q') {
    cout <<"ABORT!" <<endl;
    return {arr(), arr()};
  }
  return {path, times};
}

//===========================================================================

arr getStartGoalPath(rai::Configuration& C, const arr& qTarget, const arr& qHome, const rai::Array<Avoid>& avoids, StringA endeffectors, bool endeffApproach, bool endeffRetract) {

  arr q0 = C.getJointState();

  //set endeff target helper frames
  if(endeffectors.N) {
    C.setJointState(qTarget);
    for(rai::String endeff:endeffectors) {
      rai::Frame* f = C[STRING(endeff<<"_target")];
      if(!f) {
        f = C.addFrame(STRING(endeff<<"_target"));
        f->setShape(rai::ST_marker, {.5})
        .setColor({1., 1., 0, .5});
      }
      f->set_X() = C[endeff]->ensure_X();
    }
    C.setJointState(q0);
  }

  KOMO komo;
  komo.opt.verbose=0;
//  komo.opt.animateOptimization=4;
  komo.setConfig(C, true);
  komo.setTiming(1., 32, 5., 2);
  komo.addControlObjective({}, 2, 1.);

  // constrain target - either hard endeff target (and soft q), or hard qTarget
  if(endeffectors.N) {
    for(rai::String endeff:endeffectors) {
      komo.addObjective({1.}, FS_poseDiff, {endeff, STRING(endeff<<"_target")}, OT_eq, {1e0}); //uses endeff target helper frames
    }
    komo.addObjective({1.}, FS_qItself, {}, OT_sos, {1e0}, qTarget);
  } else {
    komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, qTarget);
  }

  // final still
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, {}, 1);

  // homing
  if(qHome.N) komo.addObjective({.4, .6}, FS_qItself, {}, OT_sos, {.1}, qHome);

  // generic collisions
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});

  // explicit collision avoidances
  for(const Avoid& a:avoids) {
    komo.addObjective(a.times, FS_distance, a.frames, OT_ineq, {1e1}, {-a.dist});
  }

  // retract: only longitudial velocity, only about-z rotation
  if(endeffRetract) {
    for(rai::String endeff:endeffectors) {
      arr ori = ~C[endeff]->ensure_X().rot.getArr();
      arr yz = ori({1, 2});
      komo.addObjective({0., .2}, FS_position, {endeff}, OT_eq, ori[0].reshape(1, -1)*1e2, {}, 1);
      komo.addObjective({0., .2}, FS_angularVel, {endeff}, OT_eq, yz*1e1, {}, 1);
    }
  }

  // approach: only longitudial velocity, only about-z rotation
  if(endeffApproach) {
    for(rai::String endeff:endeffectors) {
      arr ori = ~C[STRING(endeff<<"_target")]->get_X().rot.getArr();
      arr yz = ori({1, 2});
      komo.addObjective({.8, 1.}, FS_position, {endeff}, OT_eq, ori[0].reshape(1, -1)*1e2, {}, 1);
      komo.addObjective({.8, 1.}, FS_angularVel, {endeff}, OT_eq, yz*1e1, {}, 1);
    }
  }

  //-- run several times with random initialization
  bool feasible=false;
  uint trials=3;
  for(uint trial=0; trial<trials; trial++) {
    //initialize with constant q0 or qTarget
    komo.reset();
    if(trial%2) komo.initWithConstant(qTarget);
    else komo.initWithConstant(q0);

    //optimize
    auto ret = komo.optimize(.01*trial, -1, rai::OptOptions().set_stopTolerance(1e-3)); //trial=0 -> no noise!

    //is feasible?
    feasible=ret->sos<50. && ret->ineq<.1 && ret->eq<.1;

    //if not feasible -> add explicit collision pairs (from proxies presently in komo.pathConfig)
    if(!feasible) {
      //cout <<komo.report();
      //komo.pathConfig.reportProxies();
      StringA collisionPairs = komo.getCollisionPairs(.01);
      if(collisionPairs.N) {
        komo.addObjective({}, FS_distance, collisionPairs, OT_ineq, {1e2}, {-.001});
      }
    }

    cout <<"  path trial " <<trial <<(feasible?" good":" FAIL") <<" -- time:" <<komo.timeTotal <<"\t sos:" <<ret->sos <<"\t ineq:" <<ret->ineq <<"\t eq:" <<ret->eq <<endl;
    if(feasible) break;
  }

  if(!feasible) return {};

  arr path = komo.getPath_qOrg();

  return path;
}

//===========================================================================

arr getStartGoalPath_new(rai::Configuration& C, const arr& qTarget, const arr& qHome, const rai::Array<Avoid>& avoids, StringA endeffectors, bool endeffApproach, bool endeffRetract) {
  auto info = STRING("end to end motion");
  ManipulationModelling M(info);
  M.setup_point_to_point_motion(C, qTarget);
  for(auto& gripper:endeffectors) {
    if(endeffRetract) M.retract({.0, .2}, gripper);
    if(endeffApproach) M.approach({.8, 1.}, gripper);
  }
  for(const Avoid& a:avoids) {
    M.komo->addObjective(a.times, FS_distance, a.frames, OT_ineq, {1e1}, {-a.dist});
  }
  M.solve();
  cout <<"  " <<info <<" -- " <<*M.ret <<endl;
  return M.path;
}

//===========================================================================

void mirrorDuplicate(std::pair<arr, arr>& path) {
  arr& q = path.first;
  arr& t = path.second;

  if(!q.N) return;

  uint T=q.d0-1;
  double D=2.*t.last();

  q.resizeCopy(2*T+1, q.d1);
  t.resizeCopy(2*T+1);
  for(uint i=1; i<=T; i++) {
    q[T+i] = q[T-i];
    t(T+i) = D - t(T-i);
  }
}

arr path_resample(const arr& q, double durationScale) {
  rai::BSpline S = getSpline(q);

  uint T = durationScale * q.d0;
  durationScale = double(T)/double(q.d0);

  arr r(T, q.d1);
  for(uint t=0; t<T; t++) {
    r[t] = S.eval(double(t)/double(T-1));
  }

  return r;
}

arr path_resampleLinear(const arr& q, uint T) {
  arr r(T, q.d1);
  for(uint t=0; t<T-1; t++) {
    double s = double(t)/(T-1)*(q.d0-1);
    uint i0 = floor(s);
    double a = s-i0;
    uint i1=i0+1;
    if(i1>=q.d0) i1=i0;
    r[t] = (1.-a)*q[i0] + a*q[i0+1];
  }
  r[T-1] = q[q.d0-1];
  return r;
}

rai::BSpline getSpline(const arr& q, double duration, uint degree) {
  rai::BSpline S;
  S.set(degree, q, grid(1, 0., duration, q.N-1));
  return S;
}

#if 0 //deprecated
bool checkCollisionsAndLimits(rai::Configuration& C, const FrameL& collisionPairs, const arr& limits, bool solveForFeasible, int verbose) {
  arr B;
  //-- check for limits
  if(limits.N) {
    arr q = C.getJointState();
    B = ~limits;
    bool good = boundCheck(q, B[0], B[1]);
    if(!good) {
      if(solveForFeasible) {
        boundClip(q, B[0], B[1]);
        C.setJointState(q);
      } else {
        THROW("BOUNDS FAILED")
        return false;
      }
    }
  }

  //-- check for collisions!
  if(collisionPairs.N) {
    CHECK_EQ(&collisionPairs.last()->C, &C, "");
    auto coll = F_PairCollision().eval(collisionPairs);
    bool doesCollide=false;
    for(uint i=0; i<coll.N; i++) {
      if(coll.elem(i)>0.) {
        LOG(-1) <<"in collision: " <<collisionPairs(i, 0)->name <<'-' <<collisionPairs(i, 1)->name <<' ' <<coll.elem(i);
        doesCollide=true;
      }
    }
    if(doesCollide) {
      if(solveForFeasible) {
        KOMO komo;
        komo.setConfig(C);
        komo.setTiming(1., 1, 1., 1);
        komo.addControlObjective({}, 1, 1e-1);
        komo.addQuaternionNorms();

        komo.addObjective({}, FS_distance, framesToNames(collisionPairs), OT_ineq, {1e2}, {-.001});

        komo.opt.verbose=0;
        komo.optimize(0., OptOptions().set_verbose(0).set_stopTolerance(1e-3));

        if(komo.ineq>1e-1) {
          LOG(-1) <<"solveForFeasible failed!" <<komo.report();
          if(verbose>1) komo.view(verbose>2, "FAILED!");
          return false;
        } else {
          LOG(0) <<"collisions resolved";
          if(B.N) {
            bool good = boundCheck(komo.x, B[0], B[1]);
            LOG(0) <<"bounds=good after resolution: " <<good;
            if(!good) HALT("this should not be the case! collision resolution should respect bounds!");
          }
          C.setJointState(komo.x);
        }
      } else {
        THROW("COLLIDES!")
        return false;
      }
    }
  }
  return true;
}
#endif

bool PoseTool::checkLimits(const arr& limits, bool solve, bool assert) {
  //get bounds
  arr B;
  if(limits.N) B = limits;
  else B = C.getJointLimits();

  //check
  arr q = C.getJointState();
  CHECK_EQ(B.d0, 2, "");
  CHECK_EQ(B.d1, q.N, "");
  bool good = boundCheck(q, B);
  if(good) return true;

  //without solve
  if(!solve) {
    if(verbose) THROW("BOUNDS FAILED")
      if(assert) HALT("limit check failed");
    return false;
  }

  //solve
  boundClip(q, B);
  C.setJointState(q);
  return true;
}

bool PoseTool::checkCollisions(const FrameL& collisionPairs, bool solve, bool assert) {
  bool good=true;
  if(collisionPairs.N) {
    //use explicitly given collision pairs
    CHECK_EQ(&collisionPairs.last()->C, &C, "");
    auto coll = F_PairCollision().eval(collisionPairs);
    for(uint i=0; i<coll.N; i++) {
      if(coll.elem(i)>0.) {
        if(verbose>1) LOG(-1) <<"in collision: " <<collisionPairs(i, 0)->name <<'-' <<collisionPairs(i, 1)->name <<' ' <<coll.elem(i);
        good=false;
      }
    }
  } else {
    //use broadphase
    C.ensure_proxies();
    double p = C.getTotalPenetration();
    if(verbose>1) C.reportProxies();
    if(p>0.) good=false;
  }

  if(good) return true;

  //without solve
  if(!solve) {
    if(verbose) {
      LOG(-1) <<"collision check failed";
      if(!collisionPairs.N) C.reportProxies();
    }
    if(assert) HALT("collision check failed");
    return false;
  }

  //solve
  KOMO komo;
  komo.setConfig(C);
  komo.setTiming(1., 1, 1., 1);
  komo.addControlObjective({}, 1, 1e-1);
  komo.addQuaternionNorms();

  if(collisionPairs.N) {
    komo.addObjective({}, FS_distance, framesToNames(collisionPairs), OT_ineq, {1e2}, {-.001});
  } else {
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_ineq, {1e2}, {-.001});
  }

  komo.opt.verbose=0;
  auto ret = komo.optimize(0., -1, rai::OptOptions().set_verbose(0).set_stopTolerance(1e-3));

  if(ret->ineq>1e-1) {
    if(verbose) LOG(-1) <<"solveForFeasible failed!" <<komo.report();
    if(verbose>1) komo.view(verbose>2, "collision resolution failed");
    if(assert) HALT("collision resolution failed");
    return false;
  } else {
    if(verbose) LOG(0) <<"collisions resolved";
//          if(B.N){
//            bool good = boundCheck(komo.x, B[0], B[1]);
//            LOG(0) <<"bounds=good after resolution: " <<good;
//            if(!good) HALT("this should not be the case! collision resolution should respect bounds!");
//          }
    C.setJointState(komo.x);
    if(verbose>1) {
      C.ensure_proxies();
      double p = C.getTotalPenetration();
      if(verbose>1) C.reportProxies();
      CHECK(p<=0., "not resolved");
    }
    return true;
  }
  return true;
}

bool PoseTool::checkLimitsAndCollisions(const arr& limits, const FrameL& collisionPairs, bool solve, bool assert) {
  return checkLimits(limits, solve, assert) && checkCollisions(collisionPairs, solve, assert);
}

arr getVel(const arr& x, const arr& tau) {
  arr v;
  v.resizeAs(x).setZero();
  for(uint t=1; t<x.d0; t++) v[t] = (x[t]-x[t-1])/tau(t);
  return v;
}

arr getAcc(const arr& x, const arr& tau) {
  arr a;
  a.resizeAs(x).setZero();
  for(uint t=2; t<x.d0; t++) a[t] = ((x[t]-x[t-1])/tau(t) - (x[t-1]-x[t-2])/tau(t-1)) /(.5*(tau(t)+tau(t-1)));
  return a;
}

arr getJerk(const arr& x, const arr& tau) {
  arr j;
  j.resizeAs(x).setZero();
  for(uint t=3; t<x.d0; t++) {
    j[t] = (
             ((x[t-0]-x[t-1])/tau(t-0) - (x[t-1]-x[t-2])/tau(t-1)) /(.5*(tau(t-0)+tau(t-1)))
             -((x[t-1]-x[t-2])/tau(t-1) - (x[t-2]-x[t-3])/tau(t-2)) /(.5*(tau(t-1)+tau(t-2)))
           ) / tau(t-1);
  }
  return j;
}

void makeMod2Pi(const arr& q0, arr& q1) {
  CHECK_EQ(q0.N, q1.N, "");
  for(uint i=0; i<q0.N; i++) {
    double del = q0.elem(i) - q1.elem(i);
    if(del>RAI_PI) q1.elem(i) += RAI_2PI;
    if(del<-RAI_PI) q1.elem(i) -= RAI_2PI;
  }
}
