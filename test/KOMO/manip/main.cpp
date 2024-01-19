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

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  //  rnd.clockSeed();

  testPickAndPlace();

  return 0;
}

