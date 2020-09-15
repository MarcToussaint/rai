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
  komo.addSquaredQuaternionNorms();

#if 0
  komo.setGrasp(1., "endeff", "stick");
#else
  komo.addObjective({1.}, FS_distance, {"endeff", "stickTip"}, OT_eq, {1e2});
//  komo.addObjective({1.}, FS_positionDiff, {"endeff", "stickTip"}, OT_eq, {1e2});
  komo.addSwitch_stable(1., -1., "endeff", "stickTip");
#endif

  komo.add_collision(true);

  komo.addObjective({2.}, FS_distance, {"stick", "redBall"}, OT_eq, {1e2});

  komo.addObjective({2.,-1.}, FS_qItself, {}, OT_eq, {1e1}, {}, 1);

//  komo.animateOptimization = 2;
//  komo.verbose = 8;
  komo.optimize();
  komo.checkGradients();

  rai::Graph result = komo.getReport(true);

//  for(uint i=0;i<2;i++) if(!komo.displayTrajectory(.1, true)) break;
  V.setPath(komo.getPath_frames(), "optimized motion", true);
  for(uint i=0;i<2;i++) V.playVideo(true);
}

//===========================================================================

int TEST(Pnp){
  rai::Configuration C("model2.g");

  rai::ConfigurationViewer V;
  V.setConfiguration(C, "initial model", false);

  KOMO komo;

  komo.setModel(C, false);
  komo.setTiming(2.5, 30, 5., 2);
  komo.add_qControlObjective({}, 2);
  komo.addSquaredQuaternionNorms();

  //grasp
  komo.addSwitch_stable(1., 2., "gripper", "box");
  komo.addObjective({1.}, FS_positionDiff, {"gripper", "box"}, OT_eq, {1e2});
  komo.addObjective({1.}, FS_scalarProductXX, {"gripper", "box"}, OT_eq, {1e2}, {0.});
  komo.addObjective({1.}, FS_vectorZ, {"gripper"}, OT_eq, {1e2}, {0., 0., 1.});

  //slow - down - up
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {}, {}, 1);
  komo.addObjective({.9,1.1}, FS_position, {"gripper"}, OT_eq, {}, {0.,0.,.1}, 2);

  //place
  komo.addSwitch_stable(2., -1., "table", "box");
  komo.addObjective({2.}, FS_positionDiff, {"box", "table"}, OT_eq, arr({1,3},{0,0,1e2}), {0,0,.1});
  komo.addObjective({2.}, FS_vectorZ, {"gripper"}, OT_eq, {1e2}, {0., 0., 1.});

  //slow - down - up
  komo.addObjective({2.}, FS_qItself, {}, OT_eq, {}, {}, 1);
  komo.addObjective({1.9,2.2}, FS_position, {"gripper"}, OT_eq, {}, {0.,0.,.1}, 2);

  komo.verbose = 4;
  komo.optimize();
//  komo.checkGradients();

  V.setPath(komo.getPath_frames(), "optimized motion", true);
  for(uint i=0;i<2;i++) V.playVideo(true);

  return 0;
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  //  testGrasp();
  testPnp();

  return 0;
}

