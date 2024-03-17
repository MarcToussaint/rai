#include <KOMO/komo.h>
#include <KOMO/skeleton.h>
#include <string>
#include <map>
#include <Core/graph.h>
#include <Kin/switch.h>
#include <Kin/viewer.h>
#include <KOMO/manipTools.h>

using namespace std;

//===========================================================================

void testPickAndPlace(){
  rai::Configuration C;
  C.addFile("scene.g");

  auto gripper = "l_gripper";
  auto palm = "l_palm";
  auto box = "box";
  auto table = "table";
  auto qHome = C.getJointState();

  C[box]->setRelativePosition({-.0,.3-.055,.095});
  C[box]->setRelativeQuaternion({1.,0,0,0});

  for(uint i=0;i<7;i++){
    arr qStart = C.getJointState();

    auto graspDirection = STRING("yz"); //random.choice(['xz', 'yz'])
    auto placeDirection = STRING("z");
    auto place_position = arr{(i%3)*.3-.3, .2};
    auto place_orientation = arr{-double(i%2),double((i+1)%2),0.};
    auto info = STRING("placement " <<i <<": grasp " <<graspDirection <<" place " <<placeDirection <<" place_pos " <<place_position <<" place_ori " <<place_orientation);
    cout <<"===" <<info <<endl;

    ManipulationModelling M1(C, info, {gripper});
    M1.setup_pick_and_place_waypoints(gripper, box);
    M1.grasp_top_box(1., gripper, box, graspDirection);
    M1.place_box(2., box, table, palm, placeDirection);
    M1.target_relative_xy_position(2., box, table, place_position);
    M1.target_x_orientation(2., box, place_orientation);
    M1.solve();
    if(!M1.ret->feasible) continue;

    {
      auto R1 = M1.sub_rrt(1);
      R1->solve(1);
      if(!R1->ret->feasible) continue;
    }

    auto M2 = M1.sub_motion(0);
    M2->retract({.0, .2}, gripper);
    M2->approach({.8, 1.}, gripper);
    M2->solve();
    if(!M2->ret->feasible) continue;

    auto M3 = M1.sub_motion(1);
    M3->no_collision({}, {table, box,
                          box, "obstacle"});
    //M3.bias(.5, qHome, 1e0);
    M3->solve();
    if(!M3->ret->feasible) continue;

    M2->play(C);
    C.attach(gripper, box);
    M3->play(C);
    C.attach(table, box);
  }
}

//===========================================================================

void testPush(){
  rai::Configuration C;
  C.addFile("scene.g");
  C.delFrame("obstacle");
  rai::Joint *j = C["l_panda_finger_joint1"]->joint;
  j->setDofs(arr{.0});

  auto gripper = "l_gripper";
  auto palm = "l_palm";
  auto obj = "box";
  auto table = "table";
  auto qHome = C.getJointState();

  C[obj]->setRelativePosition({-.0,.3-.055,.08});
  C[obj]->setRelativeQuaternion({1.,0,0,0});

  for(uint i=0;i<20;i++){
    arr qStart = C.getJointState();

    str info = STRING("push_" <<i);
    ManipulationModelling manip(C, info, {"l_gripper"});
    manip.setup_pick_and_place_waypoints(gripper, obj, 1e-1);
    manip.straight_push({1.,2.}, obj, gripper, table);
    //random target position
    manip.komo->addObjective({2.}, FS_position, {obj}, OT_eq, 1e1*arr{{2,3}, {1,0,0,0,1,0}}, .4*rand(3) - .2+arr{.0,.3,.0});
    manip.solve();
    if(!manip.ret->feasible) continue;

    auto M1 = manip.sub_motion(0);
    M1->retractPush({.0, .15}, gripper, .03);
    M1->approachPush({.85, 1.}, gripper, .03);
    M1->no_collision({.15,.85}, {obj, "l_finger1",
                                 obj, "l_finger2",
                                 obj, "l_palm"}, .02);
    M1->no_collision({}, {table, "l_finger1",
                          table, "l_finger2"}, .0);
    M1->solve();
    if(!M1->ret->feasible) continue;

    auto M2 = manip.sub_motion(1);
    M2->komo->addObjective({}, FS_positionRel, {gripper, "_push_start"}, OT_eq, 1e1*arr{{2,3},{1,0,0,0,0,1}});
    M2->solve();
    if(!M2->ret->feasible) continue;

    M1->play(C, 1.);
    C.attach(gripper, obj);
    M2->play(C, 1.);
    C.attach(table, obj);

  }
}

//===========================================================================

void testPush2(){
  rai::Configuration C;
  C.addFile("scene.g");
  C.delFrame("obstacle");
  rai::Joint *j = C["l_panda_finger_joint1"]->joint;
  j->setDofs(arr{.0});

  auto gripper = "l_gripper";
  auto palm = "l_palm";
  auto obj = "box";
  auto table = "table";
  auto qHome = C.getJointState();

  C[obj]->setRelativePosition({-.0,.3-.055,.08});
  C[obj]->setRelativeQuaternion({1.,0,0,0});

  for(uint i=0;i<20;i++){
    arr qStart = C.getJointState();

    str info = STRING("push_" <<i);
    ManipulationModelling manip(C, info, {"l_gripper"});
    manip.setup_sequence(2, 1e-1);

    //-- 'mechanisms': 1) a frame with a FREE transIX joint, attached to 'table', initialized at 'obj', to which the obj is attached at time 1
    manip.komo->addFrameDof("obj_trans", table, rai::JT_transXY, false, obj);
    manip.komo->addRigidSwitch(1., {"obj_trans", obj});

//    manip.setup_pick_and_place_waypoints(gripper, obj, 1e-1);
    manip.straight_push({1.,2.}, obj, gripper, table);
    manip.komo->addObjective({2.}, FS_poseRel, {gripper, obj}, OT_eq, {1e1}, {}, 1); //constant relative pose!
    //random target position
    manip.komo->addObjective({2.}, FS_position, {obj}, OT_eq, 1e1*arr{{2,3}, {1,0,0,0,1,0}}, .4*rand(3) - .2+arr{.0,.3,.0});
    manip.solve();
    if(!manip.ret->feasible) continue;

    auto M1 = manip.sub_motion(0);
    M1->retractPush({.0, .15}, gripper, .03);
    M1->approachPush({.85, 1.}, gripper, .03);
    M1->no_collision({.15,.85}, {obj, "l_finger1",
                                 obj, "l_finger2",
                                 obj, "l_palm"}, .02);
    M1->no_collision({}, {table, "l_finger1",
                          table, "l_finger2"}, .0);
    M1->solve();
    if(!M1->ret->feasible) continue;

    auto M2 = manip.sub_motion(1);
    M2->komo->addObjective({}, FS_positionRel, {gripper, "_push_start"}, OT_eq, 1e1*arr{{2,3},{1,0,0,0,0,1}});
    M2->solve();
    if(!M2->ret->feasible) continue;

    M1->play(C, 1.);
    C.attach(gripper, obj);
    M2->play(C, 1.);
    C.attach(table, obj);

  }
}

//===========================================================================

void testPivot(){
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  C.delSubtree("cameraWrist");

  C.addFrame("hinge", "table") ->setJoint(rai::JT_rigid) .setRelativePosition({.3, .2, .15});
  C.addFrame("door", "hinge") ->setRelativePosition({.1, .0, .0}) .setShape(rai::ST_ssBox, {.2, .04, .2, .005}). setContact(1);
  C.addFrame("handle", "door") ->setRelativePosition({.08, -.02, .0}) .setShape(rai::ST_marker, {.1});

  rai::Joint *j = C["l_panda_finger_joint1"]->joint;
  j->setDofs(arr{.0});

//  C.animate();  C.view(true);

  auto gripper = "l_gripper";
  auto palm = "l_palm";
  auto obj = "door";
  auto table = "table";
  auto qHome = C.getJointState();

  for(uint i=0;i<20;i++){
    arr qStart = C.getJointState();

    str info = STRING("pivot");
    ManipulationModelling manip(C, info, {"l_gripper"});
    manip.setup_sequence(3, 1e-2, 1e-1, false);

    //-- 'mechanisms': 1) a frame with a FREE hinge joint, attached to 'table', placed at 'hinge', to which the door is attached at time 1
    manip.komo->addFrameDof("hinge_joint", "table", rai::JT_hingeZ, false, "hinge");
    manip.komo->addRigidSwitch(1., {"hinge_joint", "hinge"});

    //-- 2) a frame with a STABLE transXYPhi joint, attached to 'table', initially placed at 'hinge', to which the door is attached at time 2
    manip.komo->addFrameDof("door_place", "table", rai::JT_transXYPhi, true, "hinge");
    manip.komo->addRigidSwitch(2., {"door_place", "hinge"});

    //gripper -- door pose: touch the door, at hing distance
//    manip.komo->addObjective({1., 2.}, FS_negDistance, {gripper, obj}, OT_eq, {1e1}); //touch
    manip.komo->addObjective({1., 2.}, FS_positionDiff, {gripper, "handle"}, OT_eq, {1e1});
    manip.komo->addObjective({1., 2.}, FS_vectorZRel, {gripper, "door"}, OT_sos, {1e-1}, {0, -1., 0});
    manip.no_collision({1.,2.}, {"l_palm", "door"});
    manip.komo->addObjective({2.}, FS_poseRel, {gripper, "door"}, OT_eq, {1e1}, {}, 1); //constant relative pose!

    //random target pose for the door
    manip.komo->addObjective({2.}, FS_vectorY, {obj}, OT_eq, {1e1}, arr{-1,0,0});

    // just to have a 3 phase, ensuring the the door remains placed
    manip.bias(3., qHome);
    manip.solve();
    if(!manip.ret->feasible) continue;

    auto M1 = manip.sub_motion(0);
    M1->approach({.8, 1.}, gripper, .1);
    M1->no_collision({.15,.85}, {obj, "l_finger1",
                                 obj, "l_finger2",
                                 obj, "l_palm"}, .02);
    M1->solve();
    if(!M1->ret->feasible) continue;

    auto M2 = manip.sub_motion(1);
    M2->komo->addObjective({}, FS_poseRel, {gripper, "door"}, OT_eq, {1e1}, {}, 1); //constant relative pose!
    M2->solve(4);
    if(!M2->ret->feasible) continue;

    auto M3 = manip.sub_motion(2);
    M1->retract({.0, .2}, gripper);
    M1->no_collision({.15,.85}, {obj, "l_finger1",
                                 obj, "l_finger2",
                                 obj, "l_palm"}, .02);
    M3->solve();
    if(!M3->ret->feasible) continue;

    M1->play(C, 1.);
    M2->play(C, 1.);
    M3->play(C, 1.);
  }
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  //  rnd.clockSeed();

//  testPickAndPlace();
//  testPush();
  testPush2();
//  testPivot();

  return 0;
}

