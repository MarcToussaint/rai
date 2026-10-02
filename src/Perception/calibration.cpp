#include "calibration.h"

#include <Kin/frame.h>
#include <Kin/F_pose.h>
#include <Kin/F_geometrics.h>
#include <Kin/F_qFeatures.h>
#include <Kin/cameraview.h>
#include <Optim/NLP_Solver.h>
#include <KOMO/komo.h>

namespace rai {

//from aruco.h:

byteAA render_images(Configuration& C, const arr& qs, uint T){
    rai::CalibrationScene CS(C);
    if(!T) T=qs.d0;
    CHECK_LE(T, qs.d0, "");

    rai::CameraView V(C);
    V.RenderData::opt.renderShadow=false;
    V.RenderData::opt.renderText=false;
    V.RenderData::opt.renderPolygonLines=true;

    byteAA rgb(qs.d0, CS.cams.N);
    floatA depth;
    for(uint t=0;t<T;t++){
        for(uint c=0;c<CS.cams.N;c++){
            cout <<"t: " <<t <<" c: " <<c <<endl;
            C.setJointState(qs[t]);

            V.updateConfiguration(C);
            V.setCamera(CS.cams(c));
            V.computeImageAndDepth(rgb(t,c), depth);
        }
    }
    return rgb;
}
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
            ar->insertPreFrame(0, true, "_calib");
            calibs.append(ar);
            cout <<" -- making stable dof: " <<ar->name <<endl;
            ar->setJoint(JT_transXY);
            ar->joint->isStable = true;
        }
}

void CalibrationScene::addCalibDofs_cameras(){
    //add camera calibration joints
    for(Frame* cam:cams){
        cam->insertPreFrame(0, true, "_calib");
        calibs.append(cam);
        cout <<" -- making stable dof: " <<cam->name <<endl;
        cam->setJoint(JT_free);
        cam->joint->isStable = true;
    }
}

void CalibrationScene::addCalibDofs_joints(const uintA& jointIds){
    for(uint i:jointIds){
        Frame *f = C.frames(i);
        Frame *pre = f->insertPreFrame(0, false, "_calib");
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

void komo_calibrate(Configuration& C, const intAA& ids, const arrA& pts, const arr& qs, const uintA& exclude_times, bool calibrate_cams, bool calibrate_arucos, bool calibrate_joints, bool calibrate_objPoses, double calib_joint_regularization, int verbose){

    uintA org_jointIds = framesToIndices( C.getJoints() );
    CalibrationScene CS(C, "obj");

    CS.C.getJointState();
    Frame *obj = CS.C.getFrame("obj");
    uintA jointIds;
    for(auto* d:CS.C.activeDofs({7,14})) jointIds.append(d->frame->ID);

    //-- setup calib frames
    if(calibrate_arucos) CS.addCalibDofs_arucos();
    if(calibrate_cams) CS.addCalibDofs_cameras();
    if(calibrate_joints) CS.addCalibDofs_joints(jointIds);
    cout <<"-- CalibrationScene:\n" <<CS.report() <<endl;

    //================ create komo

    //-- find maxT
    CHECK_EQ(ids.d0, pts.d0, "");
    CHECK_EQ(ids.d0, qs.d0, "");

    //-- setup KOMO, one slice for each datapoint
    KOMO komo(CS.C, ids.d0, 1, 0, false);

    //-- add objectives for each data point
    for(uint t=0;t<ids.d0;t++){
        if(exclude_times.contains(t)) continue;
        for(uint c=0;c<ids.d1;c++){
            CHECK_EQ(ids(t,c).N, pts(t,c).d0, "");
            for(uint i=0;i<ids(t,c).N;i++){
                uint a = ids(t,c)(i);
                if(CS.arucos(a)){
                    for(uint j=0;j<4;j++){ //corners
                        arr p = pts(t,c)(i,j,{});
                        undistort_point(p, CS.Fxycxy(c), CS.Distortion(c));
                        komo.addObjective({t+1.}, make_shared<F_PointView>(p, CS.Fxycxy(c)), { STRING("arc_"<<a <<"_" <<j), CS.cams(c)->name }, OT_sos, {1e2});
                    }
                }
            }
        }

        // komo.setConfiguration_dofs(t, jointIds, qs[t]);
        komo.setConfiguration_dofs(t, org_jointIds, qs[t]);
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
        for(auto* d: dofs) cout <<d->frame->time <<"[" <<d->frame->name <<"]  ";
        cout <<endl;
    }

    //-- add regularization to calibs
    if(calib_joint_regularization>0.){
        komo.addObjective({1.}, make_shared<F_qItself>(framesToIndices(CS.calibs_joints), false), {}, OT_sos, {calib_joint_regularization});
    }

    komo.addQuaternionNorms({}, 1e1, false);

    // cout <<komo.report() <<endl;

    komo.run_prepare(0.);
    cout <<"== initial parameters (camera, dots): " <<komo.x <<endl;
    if(verbose>1) komo.view(true, "before optim");

    // komo.pathConfig.animate();
    // komo.opt.animateOptimization = 1;

    NLP_Solver sol;
    sol.setProblem(komo.nlp());
    sol.setInitialization(komo.x.copy());
    sol.opt->set_stopTolerance(1e-6);
    sol.opt->set_verbose(4);
    auto ret = sol.solve();
    if(verbose>0){
        cout <<komo.report(false) <<endl; //reports match per feature..
    }
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

    if(verbose>1) komo.view(true, "after optim");
}

byteAA undistort_images(Configuration& C, const byteAA& org){
    rai::CalibrationScene CS(C);
    CHECK_EQ(CS.cams.N, org.d1, "");

    byteAA rgb(org.d0, org.d1);
    for(uint t=0;t<org.d0;t++){
        for(uint c=0;c<org.d1;c++){

            rgb(t,c) = undistort_image(org(t,c), CS.Fxycxy(c), CS.Distortion(c));
        }
    }
    return rgb;
}



} //namspace
