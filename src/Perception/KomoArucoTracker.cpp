#include "KomoArucoTracker.h"

#include <Kin/frame.h>
#include <Kin/F_pose.h>
#include <Kin/F_geometrics.h>
#include <Kin/F_qFeatures.h>
#include <Optim/NLP_Solver.h>

//from aruco.h:
void undistort_point(arr& p, const arr& fxycxy, const arr& distortion);

//===========================================================================

CalibrationScene::CalibrationScene(rai::Configuration& C, const char* obj_name)
    : C(C){

  for(rai::Frame* f:C.frames){
    if(f->name.startsWith("camera_") && f->name(-1)>='0' && f->name(-1)<='9'){
      CHECK_EQ(f->name, STRING("camera_" <<cams.N), "cameras need to be enumerated consecutively");
      cams.append(f);
      Fxycxy.append(f->ats->get<arr>("fxycxy"));
      Distortion.append(f->ats->get<arr>("distortion"));
    }
  }

  arucos.resize(50).setZero();
  for(rai::Frame* f:C.frames.copy()){
    if(f->ats && f->ats->findNode("aruco_id")){
      uint id = f->ats->getFlex<uint>("aruco_id");
      CHECK(!arucos(id), "aruco id " <<id <<" already used by frame " <<arucos(id)->name);
      arucos(id) = f;
      C.addFrame(STRING("arc_" <<id <<"_0"))->setShape(rai::ST_sphere, {.001}).setParent(f).setRelativePosition({-.0175,+.0175,.0});
      C.addFrame(STRING("arc_" <<id <<"_1"))->setShape(rai::ST_sphere, {.001}).setParent(f).setRelativePosition({+.0175,+.0175,.0});
      C.addFrame(STRING("arc_" <<id <<"_2"))->setShape(rai::ST_sphere, {.001}).setParent(f).setRelativePosition({+.0175,-.0175,.0});
      C.addFrame(STRING("arc_" <<id <<"_3"))->setShape(rai::ST_sphere, {.001}).setParent(f).setRelativePosition({-.0175,-.0175,.0});
    }
  }

  if(obj_name){
    obj = C.getFrame(obj_name);
    FrameL sub = obj->getSubtree();
    for(rai::Frame* f:sub){
      if(f->ats && f->ats->findNode("aruco_id")){
        uint id = f->ats->getFlex<uint>("aruco_id");
        obj_aruco_ids.append(id);
      }
    }
  }
}

void CalibrationScene::addCalibDofs_arucos(){
  //add translational calibration joints to all arucos
  for(rai::Frame *ar:arucos) if(ar){
      ar->insertPreLink(0, true, "_calib");
      calibs.append(ar);
      cout <<" -- making stable dof: " <<ar->name <<endl;
      ar->setJoint(rai::JT_transXY);
      ar->joint->isStable = true;
    }
}

void CalibrationScene::addCalibDofs_cameras(){
  //add camera calibration joints
  for(rai::Frame* cam:cams){
    cam->insertPreLink(0, true, "_calib");
    calibs.append(cam);
    cout <<" -- making stable dof: " <<cam->name <<endl;
    cam->setJoint(rai::JT_free);
    cam->joint->isStable = true;
  }
}

void CalibrationScene::addCalibDofs_joints(const uintA& jointIds){
  for(uint i:jointIds){
    rai::Frame *f = C.frames(i);
    rai::Frame *pre = f->insertPreLink(0, false, "_calib");
    calibs.append(pre);
    calibs_joints.append(pre);
    cout <<" -- making stable dof: " <<pre->name <<endl;
    pre->setJoint(rai::JT_hingeZ);
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
  cout <<"\ncalibration joints: " <<rai::framesToNames(calibs) <<endl;
  return s;
}

//===========================================================================

void komoCalibrate(CalibrationScene& CS, const intAA& ids, const arrA& pts, const arr& qs, bool calibrate_cams, bool calibrate_arucos, bool calibrate_joints, bool calibrate_objPoses, bool undistort_points, double calib_joint_regularization){

  CS.C.getJointState();
  rai::Frame *obj = CS.C.getFrame("obj");
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
        rai::Joint * j = komo.timeSlices(s, obj->ID)->joint;
        if(j->active) dofs.append(j);
      }
    }
    for(rai::Frame *f:CS.calibs) dofs.append(komo.timeSlices(0, f->ID)->joint);

    komo.pathConfig.selectJoints(dofs);

    //    dofs = komo.pathConfig.getDofs(komo.pathConfig.frames, true, false, false);
    cout <<"-- selected dofs: " <<endl;
    for(auto* d: dofs) cout <<d->frame->time <<' ' <<d->frame->name <<endl;
  }

  //-- add regularization to calibs
  if(calib_joint_regularization>0.){
    komo.addObjective({1.}, make_shared<F_qItself>(rai::framesToIndices(CS.calibs_joints), false), {}, OT_sos, {calib_joint_regularization});
  }

  komo.addQuaternionNorms({}, 1e1, false);

  // cout <<komo.report() <<endl;

  komo.run_prepare(0.);
  cout <<"== initial parameters (camera, dots): " <<komo.x <<endl;
  komo.view(true, "before optim");
  // komo.pathConfig.animate();
  komo.opt.animateOptimization = 1;

  rai::NLP_Solver sol;
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
        rai::Frame *f = komo.timeSlices(0, ar->ID);
        fil <<"   Edit(" <<ar->name <<"): { aruco_id: " <<ar->ats->getFlex<uint>("aruco_id") <<", Q: " <<f->parent->get_Q() * f->get_Q() <<" } #calib: " <<f->get_Q().diffZero() <<endl;
      }
  }
  if(calibrate_cams){
    auto fil = ofstream("calib_cams.yml");
    for(auto c:CS.cams){
      rai::Frame *f = komo.timeSlices(0, c->ID);
      fil <<"   Edit(" <<c->name <<"): { Q: " <<f->parent->get_Q() * f->get_Q() <<" } #calib: " <<f->get_Q().diffZero() <<endl;
    }
  }
  if(calibrate_joints){
    auto fil = ofstream("calib_joints.yml");
    for(rai::Frame* f_org:CS.calibs_joints){
      rai::Frame *f = komo.timeSlices(0, f_org->ID);
      fil <<"   Edit(" <<f->parent->name <<"): { pose: " <<f->parent->get_Q() * f->get_Q() <<" } #calib: " <<f->joint->get_q()*180./RAI_PI <<"deg" <<endl;
    }
  }
  if(calibrate_objPoses){
    auto fil = ofstream("calib_box.yml");
    for(uint s=0;s<komo.timeSlices.d0;s++){
      rai::Frame* f = komo.timeSlices(s, obj->ID);
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


void KomoArucoTracker::reset(rai::Configuration& C, bool force_contructor){
  if(!komo || force_contructor){
    komo = make_shared<KOMO>();
    komo->setTiming(1, 1, 1, 0);
    komo->setConfig(C, false);

    //-- select only obj dofs to be optimized
    {
      DofL dofs;
      dofs.append(komo->timeSlices(0, CS.obj->ID)->joint);
      komo->pathConfig.selectJoints(dofs);

      cout <<"-- selected dofs: " <<endl;
      for(auto* d: dofs) cout <<d->frame->time <<' ' <<d->frame->name <<endl;
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

  rai::NLP_Solver sol;
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
}
