#include <KOMO/komo.h>
#include <string>
#include <map>
#include <Core/graph.h>
#include <Kin/switch.h>
#include <Kin/viewer.h>

using namespace std;

//===========================================================================

void TEST(Grasp){
  rai::Configuration C("model.g");
//  K.optimizeTree();
  C.checkConsistency();

  rai::ConfigurationViewer V;
  V.setConfiguration(C, "initial model", false);

  KOMO komo;

  komo.setModel(C);
  komo.setTiming(2.5, 10., 5.);
  komo.add_qControlObjective({}, 2, 1.);
  komo.addQuaternionNorms();

  komo.addObjective({1.}, FS_distance, {"endeff", "stickTip"}, OT_eq, {1e2});
//  komo.addObjective({1.}, FS_positionDiff, {"endeff", "stickTip"}, OT_eq, {1e2});
//  komo.addSwitch_stable(1., -1., 0, "endeff", "stickTip");
  komo.addModeSwitch({1., -1.}, rai::SY_stable, {"endeff", "stickTip"}, true);

  komo.add_collision(true);

  komo.addObjective({2.}, FS_distance, {"stick", "redBall"}, OT_eq, {1e2});

  komo.addObjective({2.,-1.}, FS_qItself, {}, OT_eq, {1e1}, {}, 1);

//  komo.animateOptimization = 2;
//  komo.verbose = 8;
  komo.optimize();
//  komo.checkGradients();

  rai::Graph result = komo.getReport(true);

//  for(uint i=0;i<2;i++) if(!komo.displayTrajectory(.1, true)) break;
  V.setPath(komo.getPath_X(), "optimized motion", true);
  for(uint i=0;i<2;i++) V.playVideo(true);
}

//===========================================================================

void testPickAndPlace(bool keyframesOnly){
  rai::Configuration C("model2.g");

//  rai::ConfigurationViewer V;
//  V.setConfiguration(C, "initial model", false);

  KOMO komo;

  komo.setModel(C, false);
  if(!keyframesOnly){
    komo.setTiming(2.5, 30, 5., 2);
    komo.add_qControlObjective({}, 2);
  }else{
    komo.setTiming(3., 1, 5., 1);
    komo.add_qControlObjective({}, 1, 1e-1);
  }
  komo.addQuaternionNorms();

  //grasp
  komo.addModeSwitch({1., 2.}, rai::SY_stable, {"gripper", "box"}, true);
  komo.addObjective({1.}, FS_positionDiff, {"gripper", "box"}, OT_eq, {1e2});
  komo.addObjective({1.}, FS_scalarProductXX, {"gripper", "box"}, OT_eq, {1e2}, {0.});
  komo.addObjective({1.}, FS_vectorZ, {"gripper"}, OT_eq, {1e2}, {0., 0., 1.});

  if(!keyframesOnly){
    //slow - down - up
    komo.addObjective({1.}, FS_qItself, {}, OT_eq, {}, {}, 1);
    komo.addObjective({.9,1.1}, FS_position, {"gripper"}, OT_eq, {}, {0.,0.,.1}, 2);
  }

  //place
  komo.addModeSwitch({2., -1.}, rai::SY_stableOn, {"table", "box"}, false);
//  komo.addObjective({2.}, FS_positionDiff, {"box", "table"}, OT_eq, {1e2}, {0,0,.08}); //arr({1,3},{0,0,1e2})
//  komo.addObjective({2.}, FS_vectorZ, {"gripper"}, OT_eq, {1e2}, {0., 0., 1.});

  if(!keyframesOnly){
    //slow - down - up
    komo.addObjective({2.}, FS_qItself, {}, OT_eq, {}, {}, 1);
    komo.addObjective({1.9,2.1}, FS_position, {"gripper"}, OT_eq, {}, {0.,0.,.1}, 2);
  }

  komo.opt.verbose = 4;
  komo.optimize();
  //komo.checkGradients();

  komo.view(true, "optimized motion");
  for(uint i=0;i<2;i++) komo.view_play(true);
//  V.setPath(komo.getPath_frames(), "optimized motion", true);
//  for(uint i=0;i<2;i++) V.playVideo(true);
}

//===========================================================================

void testPickAndPlace2(bool keyframesOnly){
  rai::Configuration C("model2.g");

//  rai::ConfigurationViewer V;
//  V.setConfiguration(C, "initial model", false);

  KOMO komo;

  komo.setModel(C, false);
  if(!keyframesOnly){
    komo.setTiming(2.5, 30, 5., 2);
    komo.add_qControlObjective({}, 2);
  }else{
    komo.setTiming(3., 1, 5., 1);
    komo.add_qControlObjective({}, 1, 1e-1);
  }
  komo.addQuaternionNorms();

  //grasp
#if 1
  komo.addStableFrame("gripper_box", "gripper", rai::JT_free, "box");
  komo.addModeSwitch({1., 2.}, rai::SY_stableZero, {"gripper_box", "box"}, true);
#else
  komo.addModeSwitch({1., 2.}, rai::SY_stable, {"gripper", "box"}, true);
#endif
  komo.addObjective({1.}, FS_positionDiff, {"gripper", "box"}, OT_eq, {1e2});
  komo.addObjective({1.}, FS_scalarProductXX, {"gripper", "box"}, OT_eq, {1e2}, {0.});
  komo.addObjective({1.}, FS_vectorZ, {"gripper"}, OT_eq, {1e2}, {0., 0., 1.});

  if(!keyframesOnly){
    //slow - down - up
    komo.addObjective({1.}, FS_qItself, {}, OT_eq, {}, {}, 1);
    komo.addObjective({.9,1.1}, FS_position, {"gripper"}, OT_eq, {}, {0.,0.,.1}, 2);
  }

  //place
#if 1
  komo.addStableFrame("table_box", "table", rai::JT_free, "box");
  komo.addModeSwitch({2., -1.}, rai::SY_stableZero, {"table_box", "box"}, true);
#else
  komo.addModeSwitch({2., -1.}, rai::SY_stable, {"table", "box"}, false);
#endif
  komo.addObjective({2.}, FS_positionDiff, {"box", "table"}, OT_eq, {1e2}, {0,0,.08}); //arr({1,3},{0,0,1e2})
  komo.addObjective({2.}, FS_vectorZ, {"gripper"}, OT_eq, {1e2}, {0., 0., 1.});

  if(!keyframesOnly){
    //slow - down - up
    komo.addObjective({2.}, FS_qItself, {}, OT_eq, {}, {}, 1);
    komo.addObjective({1.9,2.1}, FS_position, {"gripper"}, OT_eq, {}, {0.,0.,.1}, 2);
  }

  komo.opt.verbose = 4;
//  komo.opt.animateOptimization = 4;
  komo.optimize();
//  komo.checkGradients();

  komo.view(true, "optimized motion");
  for(uint i=0;i<2;i++) komo.view_play(true);
//  V.setPath(komo.getPath_frames(), "optimized motion", true);
//  for(uint i=0;i<2;i++) V.playVideo(true);
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  testGrasp();
//  testPickAndPlace2(false);
  testPickAndPlace(false);

  return 0;
}

