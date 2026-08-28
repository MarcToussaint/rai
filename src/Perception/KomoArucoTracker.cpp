#include "KomoArucoTracker.h"

#include "aruco.h"

#include <Kin/frame.h>
#include <Kin/F_pose.h>
#include <Kin/F_geometrics.h>
#include <Kin/F_qFeatures.h>
#include <Optim/NLP_Solver.h>

namespace rai {

//from aruco.h:
void undistort_point(arr& p, const arr& fxycxy, const arr& distortion);

//===========================================================================

CalibrationScene::CalibrationScene(Configuration& _C, const char* obj_name)
    : C(_C){

  for(Frame* f:C.frames){
    if(f->name.startsWith("camera_") && f->name(-1)>='0' && f->name(-1)<='9'){
      CHECK_EQ(f->name, STRING("camera_" <<cams.N), "cameras need to be enumerated consecutively");
      cams.append(f);
      Fxycxy.append(f->ats->get<arr>("fxycxy"));
      Distortion.append(f->ats->get<arr>("distortion"));
    }
  }

  arucos.resize(50).setZero();
  for(Frame* f:C.frames.copy()){
    if(f->ats && f->ats->findNode("aruco_id")){
      uint id = f->ats->getFlex<uint>("aruco_id");
      CHECK(!arucos(id), "aruco id " <<id <<" already used by frame " <<arucos(id)->name);
      arucos(id) = f;
      if(!C.getFrame(STRING("arc_" <<id <<"_0"), false)){
        C.addFrame(STRING("arc_" <<id <<"_0"))->setShape(ST_sphere, {.001}).setParent(f).setRelativePosition({-.0175,+.0175,.0});
        C.addFrame(STRING("arc_" <<id <<"_1"))->setShape(ST_sphere, {.001}).setParent(f).setRelativePosition({+.0175,+.0175,.0});
        C.addFrame(STRING("arc_" <<id <<"_2"))->setShape(ST_sphere, {.001}).setParent(f).setRelativePosition({+.0175,-.0175,.0});
        C.addFrame(STRING("arc_" <<id <<"_3"))->setShape(ST_sphere, {.001}).setParent(f).setRelativePosition({-.0175,-.0175,.0});
      }
    }
  }

  if(obj_name){
    obj = C.getFrame(obj_name);
    FrameL sub = obj->getSubtree();
    for(Frame* f:sub){
      if(f->ats && f->ats->findNode("aruco_id")){
        uint id = f->ats->getFlex<uint>("aruco_id");
        obj_aruco_ids.append(id);
      }
    }
  }
}

void CalibrationScene::addCalibDofs_arucos(){
  //add translational calibration joints to all arucos
  for(Frame *ar:arucos) if(ar){
      ar->insertPreLink(0, true, "_calib");
      calibs.append(ar);
      cout <<" -- making stable dof: " <<ar->name <<endl;
      ar->setJoint(JT_transXY);
      ar->joint->isStable = true;
    }
}

void CalibrationScene::addCalibDofs_cameras(){
  //add camera calibration joints
  for(Frame* cam:cams){
    cam->insertPreLink(0, true, "_calib");
    calibs.append(cam);
    cout <<" -- making stable dof: " <<cam->name <<endl;
    cam->setJoint(JT_free);
    cam->joint->isStable = true;
  }
}

void CalibrationScene::addCalibDofs_joints(const uintA& jointIds){
  for(uint i:jointIds){
    Frame *f = C.frames(i);
    Frame *pre = f->insertPreLink(0, false, "_calib");
    calibs.append(pre);
    calibs_joints.append(pre);
    cout <<" -- making stable dof: " <<pre->name <<endl;
    pre->setJoint(JT_hingeZ);
    pre->joint->isStable = true;
  }
}

str CalibrationScene::report(){
  str s;
  s <<"\ncameras: [" <<cams.N <<"]";
  for(uint i=0;i<cams.N;i++) s <<"\n  " <<cams(i)->name <<" Fxycxy: " <<Fxycxy(i) <<" distortion: " <<Distortion(i);
  s <<"\narucos: [" <<arucos.N <<"]";
  for(uint i=0;i<arucos.N;i++) if(arucos(i)){ s <<"\n  " <<arucos(i)->name <<" id: " <<i; }
  if(obj){ s <<"\nobj: " <<obj->name <<" with arucos: " <<obj_aruco_ids <<" and joint: "; if(obj->joint) s<<obj->joint->type; else s <<"none"; }
  cout <<"\ncalibration joints: " <<framesToNames(calibs) <<endl;
  return s;
}

//===========================================================================

void komoCalibrate(CalibrationScene& CS, const intAA& ids, const arrA& pts, const arr& qs, bool calibrate_cams, bool calibrate_arucos, bool calibrate_joints, bool calibrate_objPoses, bool undistort_points, double calib_joint_regularization){

  CS.C.getJointState();
  Frame *obj = CS.C.getFrame("obj");
  uintA jointIds;
  for(auto* d:CS.C.activeDofs({7,14})) jointIds.append(d->frame->ID);


  //-- setup calib frames
  if(calibrate_arucos) CS.addCalibDofs_arucos();
  if(calibrate_cams) CS.addCalibDofs_cameras();
  if(calibrate_joints) CS.addCalibDofs_joints(jointIds);
  cout <<CS.report() <<endl;

  //================ create komo

  //-- find maxT
  CHECK_EQ(ids.d0, pts.d0, "");
  CHECK_EQ(ids.d0, qs.d0, "");

  //-- setup KOMO, one slice for each datapoint
  KOMO komo(CS.C, ids.d0+1, 1, 0, false);

  //-- add objectives for each data point
  for(uint t=0;t<ids.d0;t++){
    for(uint c=0;c<ids.d1;c++){
      CHECK_EQ(ids(t,c).N, pts(t,c).d0, "");
      for(uint i=0;i<ids(t,c).N;i++){
        uint a = ids(t,c)(i);
        if(CS.arucos(a)){
          for(uint j=0;j<4;j++){ //corners
            arr p = pts(t,c)(i,j,{});
            if(undistort_points) undistort_point(p, CS.Fxycxy(c), CS.Distortion(c));
            komo.addObjective({t+1.}, make_shared<F_PointView>(p, CS.Fxycxy(c)), { STRING("arc_"<<a <<"_" <<j), CS.cams(c)->name }, OT_sos, {1e2});
          }
        }
      }
    }

    arr q = qs[t];
    komo.setConfiguration_dofs(t, jointIds, q);
  }

  //-- select dofs to be optimized
  {
    DofL dofs;
    if(calibrate_objPoses){
      for(uint s=0;s<komo.timeSlices.d0;s++){
        Joint * j = komo.timeSlices(s, obj->ID)->joint;
        if(j->active) dofs.append(j);
      }
    }
    for(Frame *f:CS.calibs) dofs.append(komo.timeSlices(0, f->ID)->joint);

    komo.pathConfig.selectJoints(dofs);

    //    dofs = komo.pathConfig.getDofs(komo.pathConfig.frames, true, false, false);
    cout <<"-- selected dofs: " <<endl;
    for(auto* d: dofs) cout <<d->frame->time <<' ' <<d->frame->name <<endl;
  }

  //-- add regularization to calibs
  if(calib_joint_regularization>0.){
    komo.addObjective({1.}, make_shared<F_qItself>(framesToIndices(CS.calibs_joints), false), {}, OT_sos, {calib_joint_regularization});
  }

  komo.addQuaternionNorms({}, 1e1, false);

  // cout <<komo.report() <<endl;

  komo.run_prepare(0.);
  cout <<"== initial parameters (camera, dots): " <<komo.x <<endl;
  komo.view(true, "before optim");
  // komo.pathConfig.animate();
  komo.opt.animateOptimization = 1;

  NLP_Solver sol;
  sol.setProblem(komo.nlp());
  sol.setInitialization(komo.x.copy());
  sol.opt->set_stopTolerance(1e-6);
  sol.opt->set_verbose(4);
  auto ret = sol.solve();
  cout <<komo.report(false) <<endl; //reports match per feature..
  cout <<"-- result: " <<*ret <<endl;
  cout <<"== optimized parameters (camera, dots): " <<ret->x <<endl;


  if(calibrate_arucos){
    auto fil = ofstream("calib_arucos.yml");
    for(auto ar:CS.arucos) if(ar){
        Frame *f = komo.timeSlices(0, ar->ID);
        fil <<"   Edit(" <<ar->name <<"): { aruco_id: " <<ar->ats->getFlex<uint>("aruco_id") <<", Q: " <<f->parent->get_Q() * f->get_Q() <<" } #calib: " <<f->get_Q().diffZero() <<endl;
      }
  }
  if(calibrate_cams){
    auto fil = ofstream("calib_cams.yml");
    for(auto c:CS.cams){
      Frame *f = komo.timeSlices(0, c->ID);
      fil <<"   Edit(" <<c->name <<"): { Q: " <<f->parent->get_Q() * f->get_Q() <<" } #calib: " <<f->get_Q().diffZero() <<endl;
    }
  }
  if(calibrate_joints){
    auto fil = ofstream("calib_joints.yml");
    for(Frame* f_org:CS.calibs_joints){
      Frame *f = komo.timeSlices(0, f_org->ID);
      fil <<"   Edit(" <<f->parent->name <<"): { pose: " <<f->parent->get_Q() * f->get_Q() <<" } #calib: " <<f->joint->get_q()*180./RAI_PI <<"deg" <<endl;
    }
  }
  if(calibrate_objPoses){
    auto fil = ofstream("calib_box.yml");
    for(uint s=0;s<komo.timeSlices.d0;s++){
      Frame* f = komo.timeSlices(s, obj->ID);
      fil <<"   Edit(" <<f->name <<"(obj_base)): { Q: " <<f->get_Q() <<" }" <<endl;
    }
  }

  cout <<"=== written files:" <<endl;
  if(calibrate_cams){ str fn = "calib_cams.yml"; auto fil = ifstream(fn); cout <<"#--- " <<fn <<endl <<str(fil) <<endl; }
  if(calibrate_arucos){ str fn = "calib_arucos.yml"; auto fil = ifstream(fn); cout <<"#--- " <<fn <<endl <<str(fil) <<endl; }
  if(calibrate_joints){ str fn = "calib_joints.yml"; auto fil = ifstream(fn); cout <<"#--- " <<fn <<endl <<str(fil) <<endl; }
  if(calibrate_objPoses){ str fn = "calib_obj.yml"; auto fil = ifstream(fn); cout <<"#--- " <<fn <<endl <<str(fil) <<endl; }

  komo.view(true, "after optim");
}

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
