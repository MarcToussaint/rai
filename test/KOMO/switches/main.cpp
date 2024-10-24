#include <KOMO/komo.h>
#include <string>
#include <map>
#include <Core/graph.h>
#include <KOMO/switch.h>
#include <Kin/frame.h>
#include <Optim/NLP_Solver.h>
#include <KOMO/skeletonSymbol.h>

using namespace std;

//===========================================================================

void testPickAndPlace(uint order){
  rai::Configuration C("model2.g");

//  rai::ConfigurationViewer V;
//  V.setConfiguration(C, "initial model", false);

  KOMO komo;

  komo.setConfig(C, false);
  if(order==2){
    komo.setTiming(2.5, 30, 5., 2);
    komo.addControlObjective({}, 2);
    komo.addControlObjective({}, 0, 1e-1);
  } else if(order==1) {
    komo.setTiming(3., 20, 5., 1);
    komo.addControlObjective({}, 1, 1e0);
    komo.addControlObjective({}, 0, 1e-2);
  } else if(order==0) {
    komo.setTiming(3., 1, 5., 1);
    komo.addControlObjective({}, 0, 1e0);
  }else NIY;
  komo.addQuaternionNorms();

  komo.pathConfig.report();

  //grasp
#if 0
  komo.addStableFrame(rai::SY_stable, "gripper", "gripper_box", "box");
  komo.addRigidSwitch({1., 2.}, {"gripper_box", "box"}, true);
#else
  komo.addModeSwitch({1., 2.}, rai::SY_stable, {"gripper", "box"}, true);
#endif
  komo.addObjective({1.}, FS_positionDiff, {"gripper", "box"}, OT_eq, {1e2});
  komo.addObjective({1.}, FS_scalarProductXX, {"gripper", "box"}, OT_eq, {1e2}, {0.});
  komo.addObjective({1.}, FS_vectorZ, {"gripper"}, OT_eq, {1e2}, {0., 0., 1.});

  //slow - down - up
  if(order>0) komo.addObjective({1.}, FS_qItself, {}, OT_eq, {}, {}, 1);
  if(order>1) komo.addObjective({.9,1.1}, FS_position, {"gripper"}, OT_eq, {}, {0.,0.,.1}, 2);

  //place
#if 0
  komo.addStableFrame(rai::SY_stableOn, "table", "table_box", "box");
  komo.addRigidSwitch({2., -1.}, {"table_box", "box"}, false);
#else
  komo.addModeSwitch({2., -1.}, rai::SY_stableOn, {"table", "box"}, false);
#endif
//  komo.addObjective({2.}, FS_positionDiff, {"table_box", "table"}, OT_eq, {1e2}, {0,0,.08}); //arr({1,3},{0,0,1e2})
//  komo.addObjective({2.}, FS_vectorZ, {"table_box"}, OT_eq, {1e2}, {0., 0., 1.});

  //slow - down - up
  if(order>0) komo.addObjective({2.}, FS_qItself, {}, OT_eq, {}, {}, 1);
  if(order>1) komo.addObjective({1.9,2.1}, FS_position, {"gripper"}, OT_eq, {}, {0.,0.,.1}, 2);

//  komo.add_jointLimits();

  komo.opt.verbose = 4;
//  komo.opt.animateOptimization = 2;

  NLP_Solver sol;
  auto ret = sol.setProblem(komo.nlp()) .solve();
  cout <<*ret <<endl;

  cout <<"REPORT\n" <<komo.report(false, true, true) <<endl;
  cout <<"GRADS\n" <<sol.reportLagrangeGradients(komo.featureNames) <<endl;

  //komo.optimize();
  //komo.checkGradients();

  komo.view(true, "optimized motion");
  while(komo.view_play(true, 0, .2));
}

//===========================================================================

void grasp(double t, KOMO& komo, const char* gripper, const char* box, bool firstTime){
  //grasp
#if 0
  komo.addStableFrame(rai::SY_stable, gripper, STRING(gripper <<'+' <<box), box);
  komo.addRigidSwitch({1., 2.}, {STRING(gripper <<'+' <<box), box}, true);
#else
  komo.addModeSwitch({t, t+1}, rai::SY_stable, {gripper, box}, firstTime);
#endif
//  komo.addObjective({t}, FS_positionDiff, {gripper, box}, OT_eq, {1e2});
  komo.addObjective({t}, FS_insideBox, {gripper, box}, OT_ineq, {1e2});
  komo.addObjective({t}, FS_scalarProductXX, {gripper, box}, OT_eq, {1e2}, {0.});
  komo.addObjective({t}, FS_vectorZ, {gripper}, OT_eq, {1e2}, {0., 0., 1.});

  //slow - down - up
//  if(komo.k_order>0) komo.addObjective({1.}, FS_qItself, {}, OT_eq, {}, {}, 1);
//  if(komo.k_order>1) komo.addObjective({.9,1.1}, FS_position, {gripper}, OT_eq, {}, {0.,0.,.1}, 2);
}

void place(double t, KOMO& komo, const char* gripper, const char* box, const char* table){
#if 0
  komo.addStableFrame(rai::SY_stableOn, table, STRING(table <<'+' <<box), box);
  komo.addRigidSwitch({2., -1.}, {STRING(table <<'+' <<box), box}, false);
#else
  komo.addModeSwitch({t, -1.}, rai::SY_stableOn, {table, box}, false);
#endif
//  komo.addObjective({2.}, FS_positionDiff, {"box", "table"}, OT_eq, {1e2}, {0,0,.08}); //arr({1,3},{0,0,1e2})
//  komo.addObjective({2.}, FS_vectorZ, {"gripper"}, OT_eq, {1e2}, {0., 0., 1.});

  //slow - down - up
//  if(komo.k_order>0) komo.addObjective({2.}, FS_qItself, {}, OT_eq, {}, {}, 1);
//  if(komo.k_order>1) komo.addObjective({1.9,2.1}, FS_position, {gripper}, OT_eq, {}, {0.,0.,.1}, 2);
}

void testHandover(uint order){
  rai::Configuration C("model3.g");

  KOMO komo;
  komo.setConfig(C, false);
  double phases = 7.;
  if(order==2){
    komo.setTiming(phases, 30, 5., 2);
    komo.addControlObjective({}, 2, 1.);
//    komo.addControlObjective({}, 0, 1e-1);
  } else if(order==1) {
    komo.setTiming(phases, 20, 1., 1);
    komo.addControlObjective({}, 1, 1e0);
    komo.addControlObjective({}, 0, 1e-2);
  } else if(order==0) {
    komo.setTiming(phases, 1, 1., 1);
    komo.addControlObjective({}, 0, 1e0);
  }else NIY;
  komo.addQuaternionNorms();

  grasp(1., komo, "r_gripper", "box1", true);
  grasp(2., komo, "l_gripper", "box1", false);
  place(3., komo, "l_gripper", "box1", "tray");
  grasp(4., komo, "r_gripper", "box2", true);
  grasp(5., komo, "l_gripper", "box2", false);
  place(6., komo, "l_gripper", "box2", "tray");

  komo.addObjective({}, FS_negDistance, {"box1", "box2"}, OT_ineq);
  komo.addObjective({}, FS_negDistance, {"r_palm", "l_palm"}, OT_ineq);

  komo.opt.verbose = 4;
  komo.optimize();
  //komo.checkGradients();

  komo.view(true, "optimized motion");
  while(komo.view_play(true, 0, .2));
}

//===========================================================================

void testFloat(uint order){
  rai::Configuration C(rai::getParameter<rai::String>("model"));
  C.view(true);

  C.getFrame("obj")->makeManipJoint(rai::JT_transXYPhi, C["floor"], true);

  KOMO komo;
  komo.setConfig(C, false);

  double phases = 1.;
  if(order==2){
    komo.setTiming(phases, 10, 5., 2);
    komo.addControlObjective({}, 2, 1.);
//    komo.addControlObjective({}, 0, 1e-1);
  } else if(order==1) {
    komo.setTiming(phases, 10, 1., 1);
    komo.addControlObjective({}, 1, 1e0);
    komo.addControlObjective({}, 0, 1e-2);
  } else if(order==0) {
    komo.setTiming(phases, 1, 1., 1);
    komo.addControlObjective({}, 0, 1e0);
  }else NIY;
  komo.addQuaternionNorms();

//  komo.addFlexiSwitch({0., -1.}, false, false, rai::JT_transXYPhi, {"floor", "obj"}, true, true, false);
//  komo.addObjective({0.2, -1.}, FS_qItself, {"obj"}, OT_sos, {1e-1}, {}, 1);

  komo.addObjective({1.}, FS_positionDiff, {"obj", "goal"}, OT_eq);
  if(komo.k_order>1) komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e1}, {}, 1);

  cout <<komo.report(true) <<endl;

  komo.opt.verbose = 4;
  //komo.optimize();
  //komo.checkGradients();

  NLP_Solver sol;
  auto ret = sol.setProblem(komo.nlp()) .solve();
  cout <<*ret <<endl;

//  cout <<"REPORT\n" <<komo.report(false, true) <<endl;
//  cout <<"GRADS\n" <<sol.reportLangrangeGradients(komo.featureNames) <<endl;

  komo.view(true, "optimized motion");
  while(komo.view_play(true, 0, -1.));

}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  testPickAndPlace(2);
  testPickAndPlace(1);
  testPickAndPlace(0);

  testHandover(2);
  testHandover(1);
  testHandover(0);

  testFloat(2);
  testFloat(1);
  testFloat(0);

  return 0;
}

