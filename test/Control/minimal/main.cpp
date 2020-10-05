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

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  testMinimal();

  return 0;
}

