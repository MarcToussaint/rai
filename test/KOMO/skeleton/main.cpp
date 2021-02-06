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
    { 1.9, 2.1, SY_downUp, {"gripper"} },
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
//    { 4., 4., SY_downUp, {"gripper"} },
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

  Skeleton S = {
    //take a step
    { 1., 1., SY_touch, {"table", "handB"} },
    { 1., -1., SY_stable, {"table", "handB"} },

    //grasp box
    { 2., 2., SY_touch, {"handA", "box"} },
    { 2., 3., SY_stable, {"handA", "box"} },
    { 1.9, 2.1, SY_downUp, {"handA"} },

    //handover
    { 3., 3., SY_touch, {"gripper", "box"} },
    { 3., 4., SY_stable, {"gripper", "box"} },

    //place box
    { 3.9, 4.1, SY_downUp, {"handA"} },
    { 4., 4., SY_poseEq, {"box", "target"} },
    { 4., -1., SY_stable, {"table", "box"} },
  };
  komo.setSkeleton(S);

  komo.optimize();

  komo.getReport(true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
}

//===========================================================================

void testHandover(bool keyframesOnly){
  rai::Configuration C;
  C.addFile("model2.g");

  KOMO komo;

  komo.setModel(C, false);
  if(!keyframesOnly){
    komo.setTiming(3.5, 30, 5., 2);
    komo.add_qControlObjective({}, 2, 1e0);
  }else{
    komo.setTiming(4., 1, 2., 1);
    komo.add_qControlObjective({}, 1, 1e-1);
  }
  komo.addSquaredQuaternionNorms();

  Skeleton S = {
    //grasp
    { 1., 1., SY_touch, {"R_endeff", "stick"} },
    { 1., 2., SY_stable, {"R_endeff", "stick"} },

    //handover
    { 2., 2., SY_touch, {"L_endeff", "stick"} },
    { 2., -1., SY_stable, {"L_endeff", "stick"} },

    //touch something
    { 3., -1., SY_touch, {"stick", "ball"} },
  };
  komo.setSkeleton(S);

  komo.optimize();

  komo.getReport(true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
}

//===========================================================================

void testStackAndBalance(bool keyframesOnly){
  rai::Configuration C;
  C.addFile("model2.g");

  KOMO komo;

  komo.setModel(C, false);
  if(!keyframesOnly){
    komo.setTiming(5.5, 30, 5., 2);
    komo.add_qControlObjective({}, 2, 1e0);
  }else{
    komo.setTiming(6., 1, 2., 1);
    komo.add_qControlObjective({}, 1, 1e-1);
  }
  komo.addSquaredQuaternionNorms();

  Skeleton S = {
    //pick
    { 1., 1., SY_touch, {"R_endeff", "box0"} },
    { 1., 2., SY_stable, {"R_endeff", "box0"} },
    { .9, 1.1, SY_downUp, {"R_endeff"} },

    //place
    { 2., 2., SY_touch, {"table", "box0"} },
    { 2., -1., SY_stable, {"table", "box0"} },
    { 1.9, 2.1, SY_downUp, {"R_endeff"} },

    //pick
    { 1.5, 1.5, SY_touch, {"L_endeff", "box1"} },
    { 1.5, 3., SY_stable, {"L_endeff", "box1"} },
    { 1.4, 1.5, SY_downUp, {"L_endeff"} },

    //place
    { 3., 3., SY_touch, {"box0", "box1"} },
    { 3., -1., SY_stable, {"box0", "box1"} },
    { 2.9, 3.1, SY_downUp, {"L_endeff"} },

    { 3., 4., SY_forceBalance, {"box1"} },
    { 3., 4., SY_contact, {"box0", "box1"} },

    //pick
    { 4., 4., SY_touch, {"R_endeff", "box2"} },
    { 4., 5., SY_stable, {"R_endeff", "box2"} },
    { 3.9, 4.5, SY_downUp, {"R_endeff"} },

    //place
    { 5., 5., SY_touch, {"box1", "box2"} },
    { 5., -1., SY_stable, {"box1", "box2"} },
    { 4.9, 5.1, SY_downUp, {"R_endeff"} },

    { 5., 5., SY_forceBalance, {"box2"} },
    { 5., 5., SY_contact, {"box1", "box2"} },

    { 5., -1., SY_forceBalance, {"box1"} },
    { 5., -1., SY_contact, {"box0", "box1"} },

    //pick
    { 4., 4., SY_touch, {"L_endeff", "box3"} },
    { 4., 5., SY_stable, {"L_endeff", "box3"} },
    { 3.9, 4.5, SY_downUp, {"L_endeff"} },

    //place
    { 5., 5., SY_touch, {"box1", "box3"} },
    { 5., 5., SY_touch, {"box2", "box3"} },
    { 5., -1., SY_stable, {"box1", "box3"} },
    { 4.9, 5.1, SY_downUp, {"L_endeff"} },

    { 5., 5., SY_forceBalance, {"box3"} },
    { 5., 5., SY_contact, {"box1", "box3"} },

  };
  komo.setSkeleton(S);

  komo.optimize();

  komo.getReport(true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
}

//===========================================================================

void testWalking(bool keyframesOnly){
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

  Skeleton S = {
    //take a step
    { 1., 1., SY_touch, {"table", "handB"} },
    { 1., 2., SY_stable, {"table", "handB"} },
    { 0.8, 1., SY_downUp, {"handB"} },
    { 1.0, 1.2, SY_downUp, {"handA"} },

    //take a step
    { 2., 2., SY_touch, {"table", "handA"} },
    { 2., 3., SY_stable, {"table", "handA"} },
    { 1.8, 2., SY_downUp, {"handA"} },
    { 2.0, 2.2, SY_downUp, {"handB"} },

    //take a step
    { 3., 3., SY_touch, {"table", "handB"} },
    { 3., -1., SY_stable, {"table", "handB"} },
    { 2.8, 3., SY_downUp, {"handB"} },
    { 3.0, 3.2, SY_downUp, {"handA"} },

    //touch something
    { 4., 4., SY_touch, {"handA", "stick"} },
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

  rnd.clockSeed();

  testPickAndPlace(false);
//  testPickAndPlace(true);
//  testPickAndPush(false);
//  testPickAndPush(true);
//  testPickAndThrow(false);
//  testPickAndThrow(true);
//  testWalkAndPick(false);
//  testWalkAndPick(true);
//  testWalking(false);
//  testWalking(true);
//  testHandover(false);
//  testHandover(true);
//  testStackAndBalance(false);
//  testStackAndBalance(true);

  return 0;
}

