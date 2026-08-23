#include "KomoArucoTracker.h"

#include <Kin/frame.h>
#include <Kin/F_pose.h>
#include <Kin/F_geometrics.h>
#include <Optim/NLP_Solver.h>

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

str CalibrationScene::report(){
  str s;
  s <<"\ncameras: [" <<cams.N <<"]";
  for(uint i=0;i<cams.N;i++) s <<"\n  " <<cams(i)->name <<" Fxycxy: " <<Fxycxy(i) <<" distortion: " <<Distortion(i);
  s <<"\narucos: [" <<arucos.N <<"]";
  for(uint i=0;i<arucos.N;i++) if(arucos(i)){ s <<"\n  " <<arucos(i)->name <<" id: " <<i; }
  if(obj){ s <<"\nobj: " <<obj->name <<" with arucos: " <<obj_aruco_ids <<" and joint: "; if(obj->joint) s<<obj->joint->type; else s <<"none"; }
  return s;
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
