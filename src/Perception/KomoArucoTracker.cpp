#include "KomoArucoTracker.h"

#include "aruco.h"

#include <Kin/frame.h>
#include <Kin/F_pose.h>
#include <Kin/F_geometrics.h>
#include <Kin/F_qFeatures.h>
#include <Optim/NLP_Solver.h>

namespace rai {


//===========================================================================


void KomoArucoTracker::reset(bool force_contructor){
  if(!komo || force_contructor){
    komo = make_shared<KOMO>();
    komo->setTiming(1, 1, 1, 0);
    komo->setConfig(CS.C, false);

    //-- select only obj dofs to be optimized
    {
      DofL dofs;
      dofs.append(komo->timeSlices(0, CS.obj->ID)->joint);
      komo->pathConfig.selectJoints(dofs);

      // cout <<"-- selected dofs: " <<endl;
      // for(auto* d: dofs) cout <<d->frame->time <<' ' <<d->frame->name <<endl;
    }
  }else{
    komo->clearObjectives();
    komo->reset();
  }
}

void KomoArucoTracker::addArucoDetected(uint cam_id, uint aruco_id){
  komo->addObjective({1.}, make_shared<F_PositionRel>(), { CS.cams(cam_id)->name, STRING("arc_"<<aruco_id <<'_' <<0) }, OT_ineq, {{1,3},{0., 0., -1e2}});
}

void KomoArucoTracker::addPointView(arr p, uint cam_id, uint aruco_id, uint corner_id){
  komo->addObjective({1.}, make_shared<F_PointView>(p, CS.Fxycxy(cam_id)), { STRING("arc_"<<aruco_id <<'_' <<corner_id), CS.cams(cam_id)->name }, OT_sos, {1e2});
}

void KomoArucoTracker::addMultiPointView(const intA& ids, const arr& pts, uint cam_id, bool undistort_points){
  for(uint i=0;i<ids.d0;i++){
    uint id = ids(i);
    if(CS.obj_aruco_ids.contains(id)){
      for(uint j=0;j<pts.d1;j++){
        arr p = pts(i, j, {});
        if(std::isnan(p(0)) || std::isnan(p(1))) continue;
        if(undistort_points) undistort_point(p, CS.Fxycxy(cam_id), CS.Distortion(cam_id));
        addPointView(p, cam_id, id, j);
      }
    }
  }
}

void KomoArucoTracker::solve(int verbose, double tolerance){
  komo->addQuaternionNorms({}, 1e1, false);

  // cout <<komo->report() <<endl;

  komo->run_prepare(0.);
  if(verbose>1){
    cout <<"== initial parameters (camera, dots): " <<komo->x <<endl;
  }
  if(verbose>0){
    komo->view(true, "before optim");
    // komo->pathConfig.animate();
    komo->opt.animateOptimization = verbose-2;
  }

  NLP_Solver sol;
  sol.setProblem(komo->nlp());
  sol.setInitialization(komo->x.copy());
  sol.opt->set_stopTolerance(tolerance);
  sol.opt->set_verbose(verbose);
  ret = sol.solve();
  if(verbose>0){
    cout <<komo->report(false) <<endl; //reports match per feature..
    cout <<"-- result: " <<*ret <<endl;
    cout <<"== optimized parameters (camera, dots): " <<ret->x <<endl;
    // komo->checkGradients();
  }

  if(ret->sos<10.){
      // filter.update(ret->x);
      filter.q = ret->x;
  }
}

NaiveTrackerFilter::NaiveTrackerFilter(double threshold) : threshold(threshold) {
    threshold = .02;
    alpha = .7;
    beta = .3;
    gamma = .1;
}

void NaiveTrackerFilter::update(const arr& q_measured){
    if(!q.N || good_ratio<1e-2){
        LOG(0) <<"reinitializing";
        q = q_measured;
        qdel.resize(q.N).setZero();
        good_ratio = .5;
        err_filtered = 0.;
        return;
    }
    q += qdel;
    arr res = q_measured - q;
    double err = length(res);
    err_filtered += gamma * (err - err_filtered);
    if(err<threshold){
        q += alpha * res;
        qdel += beta * res;
    }else{
        if(err>10.*threshold){
            LOG(0) <<"huge error: " <<err <<' ' <<res;
        }else{
            q += alpha * res;
            qdel *= (1.-beta);
        }
    }
    // qdel.setZero();
}

KomoArucoTracker_Thread::KomoArucoTracker_Thread(const Array<std::shared_ptr<ArucoThread> >& aruco_threads,
                                                 Var<CtrlStateMsg>& state,
                                                 Configuration& C, const char* obj_name)
    : Thread("aruco_obj_tracker_thread", .025), aruco_threads(aruco_threads), state(state), tracker(C, obj_name) {
    LOG(0) <<"launching aruco obj tracker thread";
    threadLoop();
}

KomoArucoTracker_Thread::~KomoArucoTracker_Thread(){
    LOG(0) <<"DTOR - " <<timer.report();
    threadClose();
}

void KomoArucoTracker_Thread::step() {
    tracker.reset();

    timer.tic(1);

    arr data_times(aruco_threads.N);
    arrA pts(aruco_threads.N);
    Array<ArucoOutput> ao(aruco_threads.N);

    // aruco_threads(-1)->output.waitForNextRevision();
    timer.tic(2);

    for(uint i=0;i<ao.N;i++){ data_times(i) = aruco_threads(i)->output.get().var.data_time; }
    // cout <<"TRACKER: relative data times: " <<data_times-rai::clockTime() <<endl;
    double min_time = rai::min(data_times);
    double delay = min_time - rai::realTime();
    if(delay < -0.1){
        LOG(0) <<"TRACKER WARNING: time delay from sensor is pretty large: " <<delay <<data_times-rai::realTime();
    }

    for(uint i=0;i<ao.N;i++){
      auto get = aruco_threads(i)->filter.get();
      pts(i) = get.data.get_x(min_time);
    }

    uint n=0;
#if 0
    for(uint i=0;i<ao.N;i++){
      auto get = aruco_threads(i)->output.get();
      ao(i) = get();
      // if(!i) cout <<"cam " <<i <<": ids: " <<ao(i).ids <<endl;
    }
#else
    for(uint i=0;i<ao.N;i++){
        arr& P = pts(i);
        if(!P.N) continue;
        ArucoOutput& o = ao(i);
        o.cam_id = i;
        o.ids.clear();
        o.pts.clear();
        P.reshape(50, 8);
        for(uint a=0;a<P.d0;a++){
            if(!std::isnan(P(a,0))){
                o.ids.append(a);
                o.pts.append(P[a]);
            }
        }
        o.pts.reshape(o.ids.N, 4, 2);
        // if(!i) cout <<"cam " <<i <<": ids: " <<o.ids <<endl;
        n += o.ids.N;
    }
#endif
    if(n<10) return;

    for(auto& o:ao) tracker.addMultiPointView(o.ids, o.pts, o.cam_id);

    timer.tic(3);

    tracker.solve(0);

    timer.tic(4);

    obj_pose.set() = tracker.ret->x;
    state.set()->q({tracker.CS.obj->joint->qIndex, tracker.CS.obj->joint->qIndex+7}) = tracker.ret->x;
}

} //namespace
