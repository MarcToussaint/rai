#include <KOMO/komo.h>
#include <string>
#include <map>
#include <Core/graph.h>
#include <Kin/switch.h>
#include <Kin/viewer.h>

using namespace std;

//===========================================================================

void testPickAndPlace(bool keyframesOnly){
  rai::Configuration C;
//  C.addFile("../switches/model2.g");
  C.addFile("model.g");

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
    { 1., 1., SY_topBoxGrasp, {"gripper", "box2"} },
    { 1., 2., SY_stable, {"gripper", "box2"} },
//    { 2., 2., SY_topBoxPlace, {"gripper", "box2", "table"} },
    { 2., 2., SY_downUp, {"gripper"} },
    { 2., 2., SY_poseEq, {"box2", "target2"} },
    { 2., -1., SY_stable, {"table", "box2"} },
  };
  komo.setSkeleton(S);

  komo.optimize();

  komo.getReport(true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
}

//===========================================================================

void testPickAndPush(bool keyframesOnly){
  rai::Configuration C;
  C.addFile("model.g");

  KOMO komo;

  komo.setModel(C, false);
  if(!keyframesOnly){
    komo.setTiming(3.5, 30, 5., 2);
    komo.add_qControlObjective({}, 2);
  }else{
    komo.setTiming(3., 1, 5., 1);
    komo.add_qControlObjective({}, 1, 1e-1);
  }
  komo.addSquaredQuaternionNorms();

  //grasp
  Skeleton S = {
    {1., 1., SY_inside, {"gripper", "stick"} },
    {1., -1, SY_stable, {"gripper", "stick"} },
    {2., -1., SY_quasiStaticOn, {"table", "box"} },
    {2., -1, SY_contact, {"stick", "box"} },
    {3., 3., SY_poseEq, {"box", "target"} },
  };
  komo.setSkeleton(S);

  komo.optimize();

  komo.getReport(true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
}

//===========================================================================

void testPickAndThrow(bool keyframesOnly){
  rai::Configuration C;
  C.addFile("model.g");

  KOMO komo;

  komo.setModel(C, false);
  if(!keyframesOnly){
    komo.setTiming(4.5, 30, 2., 2);
    komo.add_qControlObjective({}, 2, 1e-1);
  }else{
    komo.setTiming(5., 1, 2., 1);
    komo.add_qControlObjective({}, 1, 1e-1);
  }
  komo.addSquaredQuaternionNorms();

  //grasp
  Skeleton S = {
    { 1., 1., SY_topBoxGrasp, {"gripper", "box2"} },
    { 1., 2., SY_stable, {"gripper", "box2"} },
    { 2., 3., SY_dynamic, {"box2"} },
    { 3., 3., SY_topBoxGrasp, {"gripper", "box2"} },
    { 3., 4., SY_stable, {"gripper", "box2"} },
//    { 5., 5., SY_topBoxPlace, {"gripper", "box2", "table"} },
    { 4., 4., SY_downUp, {"gripper"} },
    { 4., 4., SY_poseEq, {"box2", "target2"} },
    { 4., -1., SY_stable, {"table", "box2"} },
  };
  komo.setSkeleton(S);

  komo.optimize();

  komo.getReport(true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
}

//===========================================================================

void testWalkAndPick(bool keyframesOnly){
  rai::Configuration C;
  C.addFile("model.g");

  KOMO komo;

  komo.setModel(C, false);
  if(!keyframesOnly){
    komo.setTiming(4.5, 30, 5., 2);
    komo.add_qControlObjective({}, 2, 1e0);
  }else{
    komo.setTiming(5., 1, 2., 1);
    komo.add_qControlObjective({}, 1, 1e-1);
  }
  komo.addSquaredQuaternionNorms();

  //grasp
  Skeleton S = {
    { 1., 1., SY_touch, {"table", "handB"} },
    { 1., -1., SY_stable, {"table", "handB"} },

    //grasp box
    { 2., 4., SY_touch, {"handA", "box"} },
    { 2., 3., SY_stable, {"handA", "box"} },
    { 2., 2., SY_downUp, {"handA"} },

    //place box
    { 3., 3., SY_downUp, {"handA"} },
    { 3., 3., SY_poseEq, {"box", "target"} },
    { 3., -1., SY_stable, {"table", "box"} },
  };
  komo.setSkeleton(S);

  komo.optimize();

  komo.getReport(true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
}
//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  testPickAndPlace(false);
  testPickAndPlace(true);
  testPickAndPush(false);
  testPickAndPush(true);
  testPickAndThrow(false);
  testPickAndThrow(true);
  testWalkAndPick(false);
  testWalkAndPick(true);

  return 0;
}

