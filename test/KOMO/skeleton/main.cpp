#include <KOMO/komo.h>
#include <string>
#include <map>
#include <Core/graph.h>
#include <Kin/switch.h>
#include <Kin/viewer.h>

using namespace std;

//===========================================================================

void TEST(Grasp){
  rai::Configuration C;
  C.addFile("../switches/model.g");

  KOMO komo;

  komo.setModel(C);
  komo.setTiming(2.5, 10., 5.);
  komo.add_qControlObjective({}, 2, 1.);
  komo.addSquaredQuaternionNorms();
  komo.add_collision(true);

  Skeleton S = {
    { 1., 1., SY_touch, {"endeff", "stickTip"} },
    { 1., -1., SY_stable, {"endeff", "stickTip"} },
    { 2., 2., SY_touch, {"stick", "redBall"} }
  };
  komo.setSkeleton(S);
  komo.addObjective({2.,-1.}, FS_qItself, {}, OT_eq, {1e1}, {}, 1);

  komo.optimize();

  komo.getReport(true);
  komo.view(true, "optimized motion");
  for(uint i=0;i<2;i++) komo.view_play(true);
}

//===========================================================================

void testPnP(bool keyframesOnly){
  rai::Configuration C;
  C.addFile("../switches/model2.g");

  KOMO komo;

  komo.setModel(C, false);
  if(!keyframesOnly){
    komo.setTiming(2.5, 30, 5., 2);
    komo.add_qControlObjective({}, 2);
  }else{
    komo.setTiming(3., 1, 5., 1);
    komo.add_qControlObjective({}, 1, 1e-1);
  }
  komo.addSquaredQuaternionNorms();

  //grasp
  Skeleton S = {
    { 1., 1., SY_topBoxGrasp, {"gripper", "box"} },
    { 1., 2., SY_stable, {"gripper", "box"} },
    { 2., 2., SY_topBoxPlace, {"gripper", "box", "table"} },
    { 2., -1., SY_stable, {"table", "box"} },
  };
  komo.setSkeleton(S);

  komo.optimize();

  komo.getReport(true);
  komo.view(true, "optimized motion");
  for(uint i=0;i<2;i++) komo.view_play(true);
}

//===========================================================================

void testPickAndThrow(bool keyframesOnly){
  rai::Configuration C;
  C.addFile("../switches/model2.g");

  KOMO komo;

  komo.setModel(C, false);
  if(!keyframesOnly){
    komo.setTiming(5.5, 30, 2., 2);
    komo.add_qControlObjective({}, 2, 1e-1);
  }else{
    komo.setTiming(5., 1, 2., 1);
    komo.add_qControlObjective({}, 1, 1e-1);
  }
  komo.addSquaredQuaternionNorms();

  //grasp
  Skeleton S = {
    { 1., 1., SY_topBoxGrasp, {"gripper", "box"} },
    { 1., 2., SY_stable, {"gripper", "box"} },
    { 2., 4., SY_dynamic, {"box"} },
    { 4., 4., SY_topBoxGrasp, {"gripper", "box"} },
    { 4., 5., SY_stable, {"gripper", "box"} },
    { 5., 5., SY_topBoxPlace, {"gripper", "box", "table"} },
    { 5., -1., SY_stable, {"table", "box"} },
  };
  komo.setSkeleton(S);

  komo.optimize();

  komo.getReport(true);
  komo.view(true, "optimized motion");
  for(uint i=0;i<2;i++) komo.view_play(true);
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  testGrasp();
//  testPnP(false);
//  testPnP(true);
  testPickAndThrow(false);

  return 0;
}

