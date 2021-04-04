#include <Control/control.h>

#include <Kin/viewer.h>
#include <Kin/F_pose.h>
#include <Kin/F_geometrics.h>
#include <Kin/F_collisions.h>

//===========================================================================

void testMinimal(){
  rai::Configuration C;
  C.addFile("scene.g");
  C.watch(false, "start");

  double tau=.01;

  CtrlSet CS;
  //control costs
  CS.add_qControlObjective(2, 1e-2*sqrt(tau), C);
  CS.add_qControlObjective(1, 1e-1*sqrt(tau), C);

  //position carrot (is transient!)
  auto pos = CS.addObjective(make_feature(FS_poseDiff, {"gripper", "target"}, C, {1e0}), OT_sos, .1);

  //collision constraint
  CS.addObjective(make_feature<F_AccumulatedCollisions>({"ALL"}, C, {1e2}), OT_eq);

  CtrlSolver ctrl(C, tau, 2);

  for(uint t=0;t<1000;t++){
    ctrl.set(CS);
    arr q = C.getJointState();
    ctrl.update(q, {}, C);
    q = ctrl.solve();
    C.setJointState(q);
    C.stepSwift();

    ctrl.report();
    C.watch(false, STRING("t:" <<t));
    rai::wait(.01);
    if(pos->status>AS_running) break;
//    if(CS.isConverged(ctrl.komo.pathConfig)) break;
  }
}

//===========================================================================

void testGrasp(){
  rai::Configuration C;
  C.addFile("pandas.g");
  C.watch(true, "start");

  double tau=.01;

  CtrlSet approach;
  approach.addObjective(make_feature(FS_vectorZDiff, {"object", "R_gripperCenter"}, C, {1e1}), OT_sos, .005);
  approach.addObjective(make_feature(FS_positionRel, {"object", "R_gripperCenter"}, C, {1e1}, {.0, 0., -.15}), OT_sos, .005);
  approach.symbolicCommands.append({"openGripper", "R_gripper"});

  CtrlSet preGrasp;
  //immediate constraint:
  preGrasp.addObjective(make_feature(FS_insideBox, {"object", "R_gripperPregrasp"}, C, {1e0}), OT_ineq, -1);
  //transient:
  preGrasp.addObjective(make_feature(FS_vectorZDiff, {"object", "R_gripperCenter"}, C, {1e1}), OT_sos, .005);
  preGrasp.addObjective(make_feature(FS_positionDiff, {"R_gripperCenter", "object"}, C, {1e1}), OT_sos, .002);
  preGrasp.symbolicCommands.append({"preOpenGripper", "R_gripper"});

  CtrlSet grasp;
  grasp.addObjective(make_feature(FS_vectorZ, {"R_gripperCenter"}, C, {}, {0., 0., 1.}), OT_eq, -1.);
  grasp.addObjective(make_feature(FS_positionDiff, {"R_gripperCenter", "object"}, C, {1e1}), OT_eq, -1.);
  grasp.symbolicCommands.append({"closeGripper", "R_gripper"});

  CtrlSet controls;
  controls.add_qControlObjective(2, 1e-3*sqrt(tau), C);
  controls.add_qControlObjective(1, 1e-1*sqrt(tau), C);
//  auto coll = ctrl.addObjective(FS_accumulatedCollisions, {}, OT_eq, {1e2});

  CtrlSolver ctrl(C, tau, 2);

  for(uint t=0;t<1000;t++){
    rai::String txt;

    if(grasp.isConverged(ctrl.komo.pathConfig)){
      break;
    }else if(grasp.canBeInitiated(ctrl.komo.pathConfig)){
      ctrl.set(controls + grasp);
      txt <<"grasp";
    }else if(preGrasp.canBeInitiated(ctrl.komo.pathConfig)){
      ctrl.set(controls + preGrasp);
      txt <<"preGrasp2";
    }else if(approach.canBeInitiated(ctrl.komo.pathConfig)){
      ctrl.set(controls + approach);
      txt <<"preGrasp";
    }

    arr q = C.getJointState();
    ctrl.update(q, {}, C);
    q = ctrl.solve();
    C.setJointState(q);
    C.stepSwift();

    ctrl.report();
    C.watch(false, STRING(txt <<"t:" <<t));
    rai::wait(.01);
//    if(c_pos->status>AS_running) break;
  }
}

//===========================================================================

void testIneqCarrot(){
  rai::Configuration C;
  C.addFile("pandas.g");
  C.watch(true, "start");

  double tau=.01;

  CtrlSet home;
  home.addObjective(make_feature(FS_insideBox, {"R_gripperCenter", "R_workspace"}, C, {1e1}), OT_ineq, .01);

  CtrlSet reach;
  reach.addObjective(make_feature(FS_positionDiff, {"object", "R_gripperCenter"}, C, {1e1}, {.0, 0., -.05}), OT_eq, .01);

  CtrlSet controls;
//  controls.add_qControlObjective(2, 1e-2*sqrt(tau), C);
  controls.add_qControlObjective(1, 1e-1*sqrt(tau), C);

  CtrlSolver ctrl(C, tau, 2);

  int mode = 0;

  for(uint t=0;t<300;t++){
    if(mode==0){
      ctrl.set(controls + home);
      if(home.isConverged(ctrl.komo.pathConfig)) mode=1;
    }else if(mode==1){
      ctrl.set(controls + reach);
      if(reach.isConverged(ctrl.komo.pathConfig)) mode=0;
    }

    arr q = C.getJointState();
    ctrl.update(q, {}, C);
    q = ctrl.solve();
    C.setJointState(q);

    ctrl.report();
    C.watch(false, STRING("mode: " <<mode <<" t:" <<t));
    rai::wait(.01);
  }
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  testMinimal();
  testGrasp();
  testIneqCarrot();

  return 0;
}

