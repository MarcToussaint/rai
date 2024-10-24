#include <KOMO/komo.h>
#include <KOMO/skeleton.h>
#include <string>
#include <map>
#include <Core/graph.h>
#include <KOMO/switch.h>
#include <Kin/viewer.h>

using namespace std;

//===========================================================================

void testPickAndPlace(rai::ArgWord pathOrSeq){
  rai::Configuration C;
  C.addFile("model.g");

  KOMO komo;

  komo.setConfig(C, false);
  if(pathOrSeq==rai::_path){
    komo.setTiming(2.5, 30, 5., 2);
    komo.addControlObjective({}, 2);
  }else{
    komo.setTiming(3., 1, 5., 1);
    komo.addControlObjective({}, 1, 1e-1);
  }
  komo.addQuaternionNorms();

  //grasp
  rai::Skeleton S = {
//    { 1., 1., rai::SY_topBoxGrasp, {"gripper", "box2"} },
    { 1., 1., rai::SY_touch, {"gripper", "box2"} },
    { 1., 2., rai::SY_stable, {"gripper", "box2"} },
//    { 2., 2., rai::SY_topBoxPlace, {"gripper", "box2", "table"} },
    { .8, 1.2, rai::SY_downUp, {"gripper"} },
    { 1.8, 2.2, rai::SY_downUp, {"gripper"} },
    { 2., 2., rai::SY_poseEq, {"box2", "target2"} },
    { 2., -1., rai::SY_stable, {"table", "box2"} },
  };
  S.addObjectives(komo);

  komo.optimize();

  komo.report(false, false, true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
}

//===========================================================================

void testPickAndPush(rai::ArgWord pathOrSeq){
  rai::Configuration C;
  C.addFile("model.g");

  KOMO komo;

  komo.setConfig(C, false);
  if(pathOrSeq==rai::_path){
    komo.setTiming(3.5, 30, 5., 2);
    komo.addControlObjective({}, 2);
  }else{
    komo.setTiming(3., 1, 5., 1);
    komo.addControlObjective({}, 1, 1e-1);
  }
  komo.addQuaternionNorms();

  //grasp
  rai::Skeleton S = {
    {1., 1., rai::SY_inside, {"gripper", "stick"} },
    {1., -1, rai::SY_stable, {"gripper", "stick"} },
    {2., -1., rai::SY_quasiStaticOn, {"table", "box"} },
    {2., -1, rai::SY_contact, {"stick", "box"} },
    {3., 3., rai::SY_poseEq, {"box", "target"} },
  };
  S.addObjectives(komo);

  komo.optimize();

  komo.report(false, false, true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
}

//===========================================================================

void testPickAndThrow(rai::ArgWord pathOrSeq){
  rai::Configuration C;
  C.addFile("model.g");

  KOMO komo;

  komo.setConfig(C, false);
  if(pathOrSeq==rai::_path){
    komo.setTiming(4.5, 30, 2., 2);
    komo.addControlObjective({}, 2, 1e-1);
  }else{
    komo.setTiming(5., 1, 2., 1);
    komo.addControlObjective({}, 1, 1e-1);
  }
  komo.addQuaternionNorms();

  //grasp
  rai::Skeleton S = {
    { 1., 1., rai::SY_touch, {"gripper", "box2"} },
//    { 1., 1., rai::SY_topBoxGrasp, {"gripper", "box2"} },
    { 1., 2., rai::SY_stable, {"gripper", "box2"} },
    { 2., 3., rai::SY_dynamic, {"world", "box2"} },
    { 3., 3., rai::SY_touch, {"gripper", "box2"} },
    { 3., 4., rai::SY_stable, {"gripper", "box2"} },
//    { 5., 5., rai::SY_topBoxPlace, {"gripper", "box2", "table"} },
//    { 4., 4., rai::SY_downUp, {"gripper"} },
    { 4., 4., rai::SY_poseEq, {"box2", "target2"} },
    { 4., -1., rai::SY_stable, {"table", "box2"} },
  };
  S.addObjectives(komo);

  komo.optimize();

  komo.report(false, false, true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
}

//===========================================================================

void testTouchAndRoll(rai::ArgWord pathOrSeq){
  rai::Configuration C;
  C.addFile("model.g");

  //grasp
  rai::Skeleton S = {
    { 1., 1., rai::SY_touch, {"handB", "ball"} },
    { 1., 1.2, rai::SY_contact, {"handB", "ball"} },
    { 1., -1., rai::SY_dynamicOn, {"table", "ball"} },
    { 2., 2., rai::SY_touch, {"ball", "box"} },
  };

  std::shared_ptr<KOMO> komo;
  if(pathOrSeq==rai::_path) komo = S.getKomo_path(C);
  else komo=S.getKomo_waypoints(C);

  komo->optimize();

  komo->report(false, false, true);
  komo->view(true, "optimized motion");
  while(komo->view_play(true, 0, 1.));
}

//===========================================================================

void testWalkAndPick(rai::ArgWord pathOrSeq){
  rai::Configuration C;
  C.addFile("model.g");

  KOMO komo;

  komo.setConfig(C, false);
  if(pathOrSeq==rai::_path){
    komo.setTiming(4.5, 30, 5., 2);
    komo.addControlObjective({}, 2, 1e0);
  }else{
    komo.setTiming(5., 1, 2., 1);
    komo.addControlObjective({}, 1, 1e-1);
  }
  komo.addQuaternionNorms();

  rai::Skeleton S = {
    //take a step
    { 1., 1., rai::SY_touch, {"table", "handB"} },
    { 1., -1., rai::SY_stable, {"table", "handB"} },

    //grasp box
    { 2., 2., rai::SY_touch, {"handA", "box"} },
    { 2., 3., rai::SY_stable, {"handA", "box"} },
    { 1.9, 2.1, rai::SY_downUp, {"handA"} },

    //handover
    { 3., 3., rai::SY_touch, {"gripper", "box"} },
    { 3., 4., rai::SY_stable, {"gripper", "box"} },

    //place box
    { 3.9, 4.1, rai::SY_downUp, {"handA"} },
    { 4., 4., rai::SY_poseEq, {"box", "target"} },
    { 4., -1., rai::SY_stable, {"table", "box"} },
  };
  S.addObjectives(komo);

  komo.optimize();

  komo.report(false, false, true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
}

//===========================================================================

void testHandover(rai::ArgWord pathOrSeq){
  rai::Configuration C;
  C.addFile("model2.g");

  KOMO komo;

  komo.setConfig(C, false);
  if(pathOrSeq==rai::_path){
    komo.setTiming(3.5, 30, 5., 2);
    komo.addControlObjective({}, 2, 1e0);
  }else{
    komo.setTiming(4., 1, 2., 1);
    komo.addControlObjective({}, 1, 1e-1);
  }
  komo.addQuaternionNorms();

  rai::Skeleton S = {
    //grasp
    { 1., 1., rai::SY_touch, {"R_endeff", "stick"} },
    { 1., 2., rai::SY_stable, {"R_endeff", "stick"} },

    //handover
    { 2., 2., rai::SY_touch, {"L_endeff", "stick"} },
    { 2., -1., rai::SY_stable, {"L_endeff", "stick"} },

    //touch something
    { 3., -1., rai::SY_touch, {"stick", "ball"} },
  };
  S.addObjectives(komo);

  komo.optimize();

  komo.report(false, false, true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
}

//===========================================================================

void testStackAndBalance(rai::ArgWord pathOrSeq){
  rai::Configuration C;
  C.addFile("model2.g");

  KOMO komo;

  komo.setConfig(C, false);
  if(pathOrSeq==rai::_path){
    komo.setTiming(5.5, 30, 5., 2);
    komo.addControlObjective({}, 2, 1e0);
  }else{
    komo.setTiming(6., 1, 2., 1);
    komo.addControlObjective({}, 1, 1e-1);
  }
  komo.addQuaternionNorms();

  rai::Skeleton S = {
    //pick
    { 1., 1., rai::SY_touch, {"R_endeff", "box0"} },
    { 1., 2., rai::SY_stable, {"R_endeff", "box0"} },
    { .9, 1.1, rai::SY_downUp, {"R_endeff"} },

    //place
    { 2., 2., rai::SY_touch, {"table", "box0"} },
    { 2., -1., rai::SY_stable, {"table", "box0"} },
    { 1.9, 2.1, rai::SY_downUp, {"R_endeff"} },

    //pick
    { 1.5, 1.5, rai::SY_touch, {"L_endeff", "box1"} },
    { 1.5, 3., rai::SY_stable, {"L_endeff", "box1"} },
    { 1.4, 1.5, rai::SY_downUp, {"L_endeff"} },

    //place
    { 3., 3., rai::SY_touch, {"box0", "box1"} },
    { 3., -1., rai::SY_stable, {"box0", "box1"} },
    { 2.9, 3.1, rai::SY_downUp, {"L_endeff"} },

    { 3., 4., rai::SY_forceBalance, {"box1"} },
    { 3., 4., rai::SY_contact, {"box0", "box1"} },

    //pick
    { 4., 4., rai::SY_touch, {"R_endeff", "box2"} },
    { 4., 5., rai::SY_stable, {"R_endeff", "box2"} },
    { 3.9, 4.5, rai::SY_downUp, {"R_endeff"} },

    //place
    { 5., 5., rai::SY_touch, {"box1", "box2"} },
    { 5., -1., rai::SY_stable, {"box1", "box2"} },
    { 4.9, 5.1, rai::SY_downUp, {"R_endeff"} },

    { 5., 5., rai::SY_forceBalance, {"box2"} },
    { 5., 5., rai::SY_contact, {"box1", "box2"} },

    { 5., -1., rai::SY_forceBalance, {"box1"} },
    { 5., -1., rai::SY_contact, {"box0", "box1"} },

    //pick
    { 4., 4., rai::SY_touch, {"L_endeff", "box3"} },
    { 4., 5., rai::SY_stable, {"L_endeff", "box3"} },
    { 3.9, 4.5, rai::SY_downUp, {"L_endeff"} },

    //place
    { 5., 5., rai::SY_touch, {"box1", "box3"} },
    { 5., 5., rai::SY_touch, {"box2", "box3"} },
    { 5., -1., rai::SY_stable, {"box1", "box3"} },
    { 4.9, 5.1, rai::SY_downUp, {"L_endeff"} },

    { 5., 5., rai::SY_forceBalance, {"box3"} },
    { 5., 5., rai::SY_contact, {"box1", "box3"} },

  };
  S.addObjectives(komo);

  komo.optimize();

  komo.report(false, false, true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
}

//===========================================================================

void testWalking(rai::ArgWord pathOrSeq){
  rai::Configuration C;
  C.addFile("model.g");

  KOMO komo;

  komo.setConfig(C, false);
  if(pathOrSeq==rai::_path){
    komo.setTiming(4.5, 30, 5., 2);
    komo.addControlObjective({}, 2, 1e0);
  }else{
    komo.setTiming(5., 1, 2., 1);
    komo.addControlObjective({}, 1, 1e-1);
  }
  komo.addQuaternionNorms();

  rai::Skeleton S = {
    //take a step
    { 1., 1., rai::SY_touch, {"table", "handB"} },
    { 1., 2., rai::SY_stable, {"table", "handB"} },
    { 0.8, 1., rai::SY_downUp, {"handB"} },
    { 1.0, 1.2, rai::SY_downUp, {"handA"} },

    //take a step
    { 2., 2., rai::SY_touch, {"table", "handA"} },
    { 2., 3., rai::SY_stable, {"table", "handA"} },
    { 1.8, 2., rai::SY_downUp, {"handA"} },
    { 2.0, 2.2, rai::SY_downUp, {"handB"} },

    //take a step
    { 3., 3., rai::SY_touch, {"table", "handB"} },
    { 3., -1., rai::SY_stable, {"table", "handB"} },
    { 2.8, 3., rai::SY_downUp, {"handB"} },
    { 3.0, 3.2, rai::SY_downUp, {"handA"} },

    //touch something
    { 4., 4., rai::SY_touch, {"handA", "stick"} },
  };
  S.addObjectives(komo);

  komo.optimize();

  komo.report(false, false, true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  rnd.clockSeed();

  testPickAndPlace(rai::_path);
//  testPickAndPlace(rai::_sequence);
  testPickAndPush(rai::_path);
//  testPickAndPush(rai::_sequeqnce);
  testPickAndThrow(rai::_path);
//  testPickAndThrow(rai::_sequence);
  testTouchAndRoll(rai::_path);
//  testTouchAndRoll(rai::_sequence);
  testWalkAndPick(rai::_path);
//  testWalkAndPick(rai::_sequence);
  testWalking(rai::_path);
//  testWalking(rai::_sequence);
  testHandover(rai::_path);
//  testHandover(rai::_sequence);
  testStackAndBalance(rai::_path); //works rarely only - why?
//  testStackAndBalance(rai::_sequence);

  return 0;
}

