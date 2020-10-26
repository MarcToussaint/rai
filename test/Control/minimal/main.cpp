#include <Control/control.h>

#include <Kin/viewer.h>

//===========================================================================

void testMinimal(){
  rai::Configuration C;
  C.addFile("scene.g");

  rai::ConfigurationViewer V;
  V.setConfiguration(C, "start", true);

  double tau=.01;

  CtrlProblem ctrl(C, tau, 2);
  //YOU NEED TO KEEP HANDLES (SHARED PTRS) OF THE RETURNED CONTROL OBJECTS -- OTHERWISE THEY'RE REMOVED AGAIN!

  //control costs
  auto c_acc = ctrl.add_qControlObjective(2, 1e-2);
  auto c_vel = ctrl.add_qControlObjective(1, 1e-1);

  //position carrot
  auto c_pos = ctrl.addObjective(FS_positionDiff, {"gripper", "target"}, OT_sos, {1e0});
  c_pos->setRef(make_shared<CtrlTarget_MaxCarrot>(*c_pos, .1));

  //collision constraint
  auto coll = ctrl.addObjective(FS_accumulatedCollisions, {}, OT_eq, {1e2});

  for(uint t=0;t<1000;t++){
    ctrl.update(C);
    arr q = ctrl.solve();
    C.setJointState(q);
    C.stepSwift();

    ctrl.report();
    V.setConfiguration(C, STRING("t:" <<t), false);
    rai::wait(.01);
    if(c_pos->status>AS_running) break;
  }
}

//===========================================================================

void testGrasp(){
  rai::Configuration C;
  C.addFile("pandas.g");

  rai::ConfigurationViewer V;
  V.setConfiguration(C, "start", true);

  double tau=.01;

  CtrlSet preGrasp;
  preGrasp.addObjective(symbols2feature(FS_vectorZ, {"R_gripperCenter"}, C, {1e1}, {0., 0., 1.}), OT_sos, .005);
  preGrasp.addObjective(symbols2feature(FS_positionRel, {"object", "R_gripperCenter"}, C, {1e1}, {.0, 0., -.15}), OT_sos, .002);

  CtrlSet preGrasp2;
  //immediate constraint:
  preGrasp2.addObjective(symbols2feature(FS_insideBox, {"object", "R_gripperPregrasp"}, C, {1e0}), OT_ineq, -1);
  //transient:
  preGrasp2.addObjective(symbols2feature(FS_vectorZ, {"R_gripperCenter"}, C, {1e1}, {0., 0., 1.}), OT_sos, .005);
  preGrasp2.addObjective(symbols2feature(FS_positionDiff, {"R_gripperCenter", "object"}, C, {1e1}), OT_sos, .002);

  CtrlSet grasp;
  grasp.addObjective(symbols2feature(FS_vectorZ, {"R_gripperCenter"}, C, {}, {0., 0., 1.}), OT_eq, -1.);
  grasp.addObjective(symbols2feature(FS_positionDiff, {"R_gripperCenter", "object"}, C, {1e1}), OT_eq, -1.);

  CtrlSet controls;
  controls.add_qControlObjective(2, 1e-2*sqrt(tau), C);
  controls.add_qControlObjective(1, 1e-1*sqrt(tau), C);
//  auto coll = ctrl.addObjective(FS_accumulatedCollisions, {}, OT_eq, {1e2});

  CtrlProblem ctrl(C, tau, 2);

  for(uint t=0;t<1000;t++){
    rai::String txt;

    ctrl.komo.pathConfig.checkConsistency();
    if(grasp.isConverged(ctrl.komo.pathConfig)){
      break;
    }else if(grasp.canBeInitiated(ctrl.komo.pathConfig)){
      ctrl.set(grasp);
      txt <<"grasp";
    }else if(preGrasp2.canBeInitiated(ctrl.komo.pathConfig)){
      ctrl.set(preGrasp2);
      txt <<"preGrasp2";
    }else if(preGrasp.canBeInitiated(ctrl.komo.pathConfig)){
      ctrl.set(controls + preGrasp);
      txt <<"preGrasp";
    }

    ctrl.update(C);
    arr q = ctrl.solve();
    C.setJointState(q);
    C.stepSwift();

    ctrl.report();
    V.setConfiguration(C, STRING(txt <<" t:" <<t), false);
    rai::wait(.01);
//    if(c_pos->status>AS_running) break;
  }
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  testMinimal();
  testGrasp();

  return 0;
}

