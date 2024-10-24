#include <KOMO/komo.h>
#include <KOMO/skeleton.h>
#include <string>
#include <map>
#include <Core/graph.h>
#include <KOMO/switch.h>
#include <Kin/viewer.h>
#include <KOMO/manipTools.h>

using namespace std;

//===========================================================================

void testPickAndPlace(){
  rai::Configuration C;
  C.addFile("scene.g");

  auto gripper = "l_gripper";
  auto palm = "l_palm";
  auto obj = "box";
  auto table = "table";
  auto qHome = C.getJointState();

  C[obj]->setRelativePosition({-.0,.3-.055,.095});
  C[obj]->setRelativeQuaternion({1.,0,0,0});

  for(uint i=0;i<7;i++){
    arr qStart = C.getJointState();

    auto graspDirection = STRING("yz"); //random.choice(['xz', 'yz'])
    auto placeDirection = STRING("z");
    auto place_position = arr{(i%3)*.3-.3, .2};
    auto place_orientation = arr{-double(i%2),double((i+1)%2),0.};
    auto info = STRING("placement " <<i <<": grasp " <<graspDirection <<" place " <<placeDirection <<" place_pos " <<place_position <<" place_ori " <<place_orientation);
    cout <<"===" <<info <<endl;

    ManipulationModelling seq(info);
//    M1.setup_pick_and_place_waypoints(gripper, box);
    seq.setup_sequence(C, 2.);
    //three totally different ways to model the gripper->object switch:
#if 0
    seq.komo->addModeSwitch({1., -1.}, rai::SY_stable, {gripper, obj}, true); //a temporary free stable joint gripper -> object
#elif 0
    seq.komo->addFrameDof("obj_grasp", gripper, rai::JT_free, true, obj); //a permanent free stable gripper->grasp joint; and a snap grasp->object
    seq.komo->addRigidSwitch(1., {"obj_grasp", obj});
#else
    seq.komo->addFrameDof("obj_grasp", obj, rai::JT_free, true, obj); //a permanent free stable object->grasp joint; and a snap gripper->grasp
    seq.komo->addRigidSwitch(1., {gripper, "obj_grasp"});
#endif
    //seq.grasp_top_box(1., gripper, box, graspDirection);
    seq.grasp_box(1., gripper, obj, palm, "y");
    seq.place_box(2., obj, table, palm, placeDirection);
    seq.target_relative_xy_position(2., obj, table, place_position);
    seq.target_x_orientation(2., obj, place_orientation);
    seq.solve();
    if(!seq.ret->feasible) continue;

    {
      auto rrt1 = seq.sub_rrt(1);
      rrt1->solve(1);
      if(!rrt1->ret->feasible) continue;
    }

    auto move0 = seq.sub_motion(0);
    move0->retract({.0, .2}, gripper);
    move0->approach({.8, 1.}, gripper);
    move0->solve();
    if(!move0->ret->feasible) continue;

    auto move1 = seq.sub_motion(1);
    move1->no_collision({}, {table, obj,
                          obj, "obstacle"});
    //move1.bias(.5, qHome, 1e0);
    move1->solve();
    if(!move1->ret->feasible) continue;

    move0->play(C);
    C.attach(gripper, obj);
    move1->play(C);
    C.attach(table, obj);
  }
}

//===========================================================================

void testPush(){
  rai::Configuration C;
  C.addFile("scene.g");
  C.delFrame("obstacle");

  //close the gripper
  rai::Joint *j = C["l_panda_finger_joint1"]->joint;
  j->setDofs(arr{.01});

  auto gripper = "l_gripper";
//  auto palm = "l_palm";
  auto obj = "box";
  auto table = "table";
//  auto qHome = C.getJointState();

  C[obj]->setRelativePosition({-.0,.3-.055,.08});
  C[obj]->setRelativeQuaternion({1.,0,0,0});

  for(uint i=0;i<20;i++){
    arr qStart = C.getJointState();

    str info = STRING("push_" <<i);
    ManipulationModelling seq(info);
    seq.setup_sequence(C, 2, 1e-1);

#if 0
    seq.komo->addModeSwitch({1., -1.}, rai::SY_stable, {gripper, obj}, true); //a temporary stable free joint gripper->obj
#elif 1
    seq.komo->addFrameDof("obj_grasp", gripper, rai::JT_free, true, obj); //a permanent free stable gripper->grasp joint; and a snap grasp->object
    seq.komo->addRigidSwitch(1., {"obj_grasp", obj});
#else
    seq.komo->addFrameDof("obj_trans", table, rai::JT_transXY, false, obj); //a permanent moving(!) transXY joint table->trans, and a snap trans->obj
    seq.komo->addRigidSwitch(1., {"obj_trans", obj});
#endif

    seq.straight_push({1.,2.}, obj, gripper, table);
    seq.komo->addObjective({2.}, FS_poseRel, {gripper, obj}, OT_eq, {1e1}, {}, 1); //constant relative pose! (redundant for first switch option)
    //random target position
    seq.komo->addObjective({2.}, FS_position, {obj}, OT_eq, 1e1*arr{{2,3}, {1,0,0,0,1,0}}, .4*rand(3) - .2+arr{.0,.3,.0});
    seq.solve(2);
    if(!seq.ret->feasible) continue;

    auto move0 = seq.sub_motion(0);
    move0->retractPush({.0, .15}, gripper, .03);
    move0->approachPush({.85, 1.}, gripper, .03);
    move0->no_collision({.15,.85}, {obj, "l_finger1",
                                 obj, "l_finger2",
                                 obj, "l_palm"}, .02);
    move0->no_collision({}, {table, "l_finger1",
                          table, "l_finger2"}, .0);
    move0->solve(2);
    if(!move0->ret->feasible) continue;

    auto move1 = seq.sub_motion(1);
//    move1->komo->addObjective({}, FS_positionRel, {gripper, "_push_start"}, OT_eq, 1e1*arr{{2,3},{1,0,0,0,0,1}});
//    move1->komo->addObjective({}, FS_negDistance, {gripper, obj}, OT_eq, {1e1}, {-.02});
    move1->komo->addObjective({}, FS_poseRel, {gripper, obj}, OT_eq, {1e1}, {}, 1); //constant relative pose! (redundant for first switch option)
    move1->solve(2);
    if(!move1->ret->feasible) continue;

    move0->play(C, 1.);
    C.attach(gripper, obj);
    move1->play(C, 1.);
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

  auto gripper = "l_gripper";
//  auto palm = "l_palm";
  auto obj = "door";
//  auto table = "table";
  auto qHome = C.getJointState();

  for(uint i=0;i<20;i++){
    str info = STRING("pivot");
    ManipulationModelling seq(info);
    seq.setup_sequence(C, 3, 1e-2, 1e-1, false);

    seq.komo->addFrameDof("hinge_joint", "table", rai::JT_hingeZ, false, "hinge"); //a permanent moving(!) hinge joint table->hinge_joint, and a snap hinge_joint->hinge
    seq.komo->addRigidSwitch(1., {"hinge_joint", "hinge"});

    seq.komo->addFrameDof("placement", "table", rai::JT_transXYPhi, true, "hinge"); //a permanent stable joint table->placement, and a snap placement->hinge
    seq.komo->addRigidSwitch(2., {"placement", "hinge"});

    //geometric constraints: gripper at handle position, z-vector backward, no palm collision
    //seq.komo->addObjective({1., 2.}, FS_negDistance, {gripper, obj}, OT_eq, {1e1}); //touch
    seq.komo->addObjective({1., 2.}, FS_positionDiff, {gripper, "handle"}, OT_eq, {1e1});
    seq.komo->addObjective({1., 2.}, FS_vectorZRel, {gripper, "door"}, OT_sos, {1e-1}, {0, -1., 0});
    seq.no_collision({1.,2.}, {"l_palm", "door"});

    //ready relative pose (we could relax that)
    seq.komo->addObjective({2.}, FS_poseRel, {gripper, "door"}, OT_eq, {1e1}, {}, 1); //constant relative pose!

    //random target pose for the door
    seq.komo->addObjective({2.}, FS_qItself, {"hinge_joint"}, OT_eq, {1e1}, 2.*rand(1)-1.);

    // just to have a 3 phase, ensuring the the door remains placed
    seq.bias(3., qHome);
    seq.solve();
    if(!seq.ret->feasible) continue;

    auto move0 = seq.sub_motion(0);
    move0->approach({.8, 1.}, gripper, .1);
    move0->no_collision({.15,.85}, {obj, "l_finger1",
                                    obj, "l_finger2",
                                    obj, "l_palm"}, .02);
    move0->solve();
    if(!move0->ret->feasible) continue;

    auto move1 = seq.sub_motion(1);
    move1->komo->addObjective({}, FS_poseRel, {gripper, "door"}, OT_eq, {1e1}, {}, 1); //constant relative pose!
    move1->solve();
    if(!move1->ret->feasible) continue;

    auto move2 = seq.sub_motion(2);
    move2->retract({.0, .15}, gripper);
    move2->no_collision({.15,.85}, {obj, "l_finger1",
                                    obj, "l_finger2",
                                    obj, "l_palm"}, .05);
    move2->solve();
    if(!move2->ret->feasible) continue;

    move0->play(C, 1.);
    move1->play(C, 1.);
    move2->play(C, 1.);
  }
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  //  rnd.clockSeed();

  testPickAndPlace();
  testPush();
  testPivot();

  return 0;
}

