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
    M3->no_collision({}, table, box);
    M3->no_collision({}, box, "obstacle");
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

  rai::Joint *j = C["l_panda_finger_joint1"]->joint;
  j->setDofs(arr{.0});

  auto gripper = "l_gripper";
  auto palm = "l_palm";
  auto obj = "box";
  auto table = "table";
  auto qHome = C.getJointState();


  C[obj]->setRelativePosition({-.0,.3-.055,.095});
  C[obj]->setRelativeQuaternion({1.,0,0,0});

  for(uint i=0;i<50;i++){
    arr qStart = C.getJointState();

    str info = STRING("push");
    ManipulationModelling manip(C, info, {"l_gripper"});
    manip.setup_pick_and_place_waypoints(gripper, obj, 1e-1);
    manip.straight_push({1.,2.}, obj, gripper, table);
    //random target position
    manip.komo->addObjective({2.}, FS_position, {obj}, OT_eq, 1e1*arr{{2,3}, {1,0,0,0,1,0}}, .4*rand(3) - .2+arr{.0,.3,.0});
    manip.solve(0);
    if(!manip.ret->feasible) continue;

    auto M1 = manip.sub_motion(0);
    M1->retractPush({.0, .15}, gripper, .03);
    M1->approachPush({.85, 1.}, gripper, .03);
    M1->no_collision({.15,.85}, obj, "l_finger1", .02);
    M1->no_collision({.15,.85}, obj, "l_finger2", .02);
    M1->no_collision({.15,.85}, obj, "l_palm", .02);
    M1->no_collision({}, table, "l_finger1", .0);
    M1->no_collision({}, table, "l_finger2", .0);
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

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  //  rnd.clockSeed();

//  testPickAndPlace();
  testPush();

  return 0;
}

