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

  for(uint i=0;i<7;i++){
    arr qStart = C.getJointState();

    str info = STRING("push");
    ManipulationModelling manip(C, info);
    manip.setup_pick_and_place_waypoints(gripper, obj);

    //start frame
    rai::Frame *f = manip.komo->addStableFrame(rai::JT_hingeZ, table, "push", obj);
    f->setShape(rai::ST_marker, {.2});
    f->setColor({1.,0., 1.});
    f->joint->sampleSdv=1.;
    f->joint->setRandom(manip.komo->timeSlices.d1, 2);

    //end frame
    rai::Frame *f2 = manip.komo->addStableFrame(rai::JT_transXYPhi, table, "end", obj);
    f2->setShape(rai::ST_marker, {.2});
    f2->setColor({1.,0., 1.});
    f2->joint->sampleSdv=1.;
    f2->joint->setRandom(manip.komo->timeSlices.d1, 2);

    //couple both frames symmetricaly
    //aligned orientation
    manip.komo->addObjective({1.}, FS_vectorYDiff, {"push", "end"}, OT_eq, {1e1});

    //aligned position
    manip.komo->addObjective({1.}, FS_positionRel, {"end", "push"}, OT_eq, 1e1*arr{{2,3},{1.,0.,0.,0.,0.,1.}});
    manip.komo->addObjective({1.}, FS_positionRel, {"push", "end"}, OT_eq, 1e1*arr{{2,3},{1.,0.,0.,0.,0.,1.}});

    //at least 2cm appart, in positive!!! direction
    manip.komo->addObjective({1.}, FS_positionRel, {"end", "push"}, OT_ineq, -1e2*arr{{1,3},{0.,1.,0.}}, {.0, .02, .0});
    manip.komo->addObjective({1.}, FS_positionRel, {"push", "end"}, OT_ineq, 1e2*arr{{1,3},{0.,1.,0.}}, {.0, -.02, .0});


    manip.komo->addObjective({2.}, FS_positionDiff, {obj, "end"}, OT_eq, {1e1});


    //push orientation


//    arr norm = diff;  norm /= length(norm);
//    arr up = norm + arr{0.,0.,1.};     up /= length(up);
//    arr side = crossProduct(diff, arr{0.,0.,1.});     side /= length(side);

//    arr proj = eye(3) - (norm^norm);
//    arr along = ~norm;


    // touch
    manip.komo->addObjective({1.}, FS_negDistance, {gripper, obj}, OT_eq, {1e1}, {-.02});

    //push gripper pose
    //position
    manip.komo->addObjective({1.}, FS_positionRel, {gripper, "push"}, OT_eq, 1e1*arr{{2,3},{1.,0.,0.,0.,0.,1.}});
    manip.komo->addObjective({1.}, FS_positionRel, {gripper, "push"}, OT_ineq, 1e1*arr{{1,3},{0.,1.,0.}}, {.0,-.02,.0});
    //orientation
    manip.komo->addObjective({1.}, FS_scalarProductYY, {gripper, "push"}, OT_ineq, {-1e1}, {.2});
    manip.komo->addObjective({1.}, FS_scalarProductYZ, {gripper, "push"}, OT_ineq, {-1e1}, {.2});
    manip.komo->addObjective({1.}, FS_vectorXDiff, {gripper, "push"}, OT_eq, {1e1});

    // target: unchanged orientation
    manip.komo->addObjective({2.}, FS_quaternion, {obj}, OT_eq, {1e1}, {}, 1); //qobjPose.rot.getArr4d());

    // target position
    arr diff = .4*rand(3) - .2;
    diff(2) = 0.;
    manip.komo->addObjective({2.}, FS_position, {obj}, OT_eq, {1e1}, diff, 1);


//    manip.target_relative_xy_position(2., obj, table,  + arr{0., .3} );

//    manip.komo->addObjective({time}, FS_positionRel, {obj, table}, OT_ineq, 1e1*arr{{2,3},{1,0,0,0,1,0}}, .5*tableSize-margin);
//    manip.komo->addObjective({time}, FS_positionRel, {obj, table}, OT_ineq, -1e1*arr{{2,3},{1,0,0,0,1,0}}, -.5*tableSize+margin);

    manip.solve(4);
    if(!manip.ret->feasible) continue;

//    manip.play(C);

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

