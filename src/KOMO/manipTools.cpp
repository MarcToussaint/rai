#include "manipTools.h"

#include "../Optim/NLP_Solver.h"

void addBoxPickObjectives(KOMO& komo, double time, rai::ArgWord dir, const char* boxName, const arr& boxSize, const char* gripperName, const char* palmName, const char* tableName, bool pre) {
    arr xLine, yzPlane;
    FeatureSymbol xyScalarProduct=FS_none, xzScalarProduct=FS_none;
    if(dir==rai::_xAxis){
        xLine = arr{{1,3},{1,0,0}};
        yzPlane = arr{{2,3},{0,1,0,0,0,1}};
        xyScalarProduct = FS_scalarProductXY;
        xzScalarProduct = FS_scalarProductXZ;
    } else if(dir==rai::_yAxis){
        xLine = arr{{1,3},{0,1,0}};
        yzPlane = arr{{2,3},{1,0,0,0,0,1}};
        xyScalarProduct = FS_scalarProductXX;
        xzScalarProduct = FS_scalarProductXZ;
    } else if(dir==rai::_zAxis){
        xLine = arr{{1,3},{0,0,1}};
        yzPlane = arr{{2,3},{1,0,0,0,1,0}};
        xyScalarProduct = FS_scalarProductXX;
        xzScalarProduct = FS_scalarProductXY;
    }

    double margin=.02;

    //position: center in inner target plane; X-specific
    if(!pre){
      komo.addObjective({time}, FS_positionRel, {gripperName, boxName}, OT_eq, xLine*1e1, {});
      komo.addObjective({time}, FS_positionRel, {gripperName, boxName}, OT_ineq, yzPlane*1e0, (boxSize/2.-margin));
      komo.addObjective({time}, FS_positionRel, {gripperName, boxName}, OT_ineq, yzPlane*(-1e0), -(boxSize/2.-margin));
    }else{
      komo.addObjective({time, time+1.}, FS_positionRel, {gripperName, boxName}, OT_eq, xLine*1e1, {});
    }

    //orientation: grasp axis orthoginal to target plane; X-specific
    komo.addObjective({time-.2,time}, xyScalarProduct, {gripperName, boxName}, OT_eq, {1e0}, {});
    komo.addObjective({time-.2,time}, xzScalarProduct, {gripperName, boxName}, OT_eq, {1e0}, {});

    //no collision with palm
    if(!pre){
      komo.addObjective({time-.3,time}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.001});
    }else{
      komo.addObjective({time-.3,time}, FS_distance, {palmName, boxName}, OT_eq, {1e1}, {-.07});
    }

    //approach: only longitudial velocity, min distance before and at grasp
    if(komo.k_order>1) komo.addObjective({time-.3,time}, FS_positionRel, {boxName, gripperName}, OT_eq, arr{{2,3}, {1,0,0,0,1,0}}*1e2, {}, 1);
    if(komo.k_order>1) komo.addObjective({time-.5,time-.3}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.1});

    //zero vel
    if(komo.k_order>1) komo.addObjective({time}, FS_qItself, {}, OT_eq, {}, {}, 1);
}

void addBoxPlaceObjectives(KOMO& komo, double time,
                           rai::ArgWord dir, const char* boxName, const arr& boxSize,
                           const char* tableName,
                           const char* gripperName, const char* palmName,
                           double margin, bool pre) {
    double relPos=0.;
    FeatureSymbol zVector = FS_none;
    arr zVectorTarget = {0.,0.,1.};
    if(dir==rai::_xAxis){
        relPos = .5*boxSize(0)+.03;
        zVector = FS_vectorX;
    } else if(dir==rai::_yAxis){
        relPos = .5*boxSize(1)+.03;
        zVector = FS_vectorY;
    } else if(dir==rai::_zAxis){
        relPos = .5*boxSize(2)+.03;
        zVector = FS_vectorZ;
    } else if(dir==rai::_xNegAxis){
        relPos = .5*boxSize(0)+.03;
        zVector = FS_vectorX;
        zVectorTarget *= -1.;
    } else if(dir==rai::_yNegAxis){
        relPos = .5*boxSize(1)+.03;
        zVector = FS_vectorY;
        zVectorTarget *= -1.;
    } else if(dir==rai::_zNegAxis){
        relPos = .5*boxSize(2)+.03;
        zVector = FS_vectorZ;
        zVectorTarget *= -1.;
    }

    //z-position: fixed
    if(!pre){
      komo.addObjective({time}, FS_positionDiff, {boxName, tableName}, OT_eq, 1e1*arr({1,3},{0,0,1}), {.0, .0, relPos});
    }else{
      komo.addObjective({time}, FS_positionDiff, {boxName, tableName}, OT_eq, 1e1*arr({1,3},{0,0,1}), {.0, .0, relPos+.04});
    }

    //xy-position: above table
    if(!pre){
      komo.addObjective({time}, FS_positionDiff, {boxName, tableName}, OT_eq, 1e1*arr({2,3},{1,0,0,0,1,0}));
      //komo.addObjective({time}, FS_aboveBox, {boxName, tableName}, OT_ineq, {3e0}, {margin});
    }else{
      komo.addObjective({time}, FS_positionDiff, {boxName, tableName}, OT_eq, 1e1*arr({2,3},{1,0,0,0,1,0}));
      //komo.addObjective({time, time+1.}, FS_aboveBox, {boxName, tableName}, OT_ineq, {3e0}, {margin});
    }

    //orientation: Y-up
    komo.addObjective({time-.2, time}, zVector, {boxName}, OT_eq, {0.5}, zVectorTarget);

    //retract: only longitudial velocity, min distance after grasp
    if(komo.k_order>1) komo.addObjective({time,time+.3}, FS_positionRel, {boxName, gripperName}, OT_eq, arr{{2,3}, {1,0,0,0,1,0}}*1e2, {}, 1);
    if(komo.k_order>1) komo.addObjective({time+.3,time+.5}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.1});

    //zero vel
    if(komo.k_order>1) komo.addObjective({time}, FS_qItself, {}, OT_eq, {}, {}, 1);
}

//void ManipulationModelling::grasp_cylinder(double time, const char* gripper, const char* obj, const char* palm, double margin){
//  auto size = komo->world[obj]->getSize();

//  komo->addObjective({time}, FS_positionRel, {gripper, obj}, OT_eq, arr({2,3},{1,0,0,0,1,0})*1e1);
//  komo->addObjective({time}, FS_positionRel, {gripper, obj}, OT_ineq, arr({1,3},{0,0,1})*1e1, {0.,0.,.5*size(0)-margin});
//  komo->addObjective({time}, FS_positionRel, {gripper, obj}, OT_ineq, arr({1,3},{0,0,1})*(-1e1), {0.,0.,-.5*size(0)+margin});

//  // orientation: grasp axis orthoginal to target plane X-specific
//  komo->addObjective({time-.2,time}, FS_scalarProductXZ, {gripper, obj}, OT_eq, {1e0});

//  // no collision with palm
//  komo->addObjective({time-.3,time}, FS_negDistance, {palm, obj}, OT_ineq, {1e1}, {-.001});
//}

//void ManipulationModelling::no_collision(const arr& times, const char* obj1, const char* obj2, double margin){
//  komo->addObjective(times, FS_negDistance, {obj1, obj2}, OT_ineq, {1e1}, {-margin});
//}

ManipulationModelling::ManipulationModelling(rai::Configuration& _C, const str& _info, const StringA& _helpers)
  : C(&_C), info(_info), helpers(_helpers) {
  for(auto& frame:helpers){
    auto name = STRING("_" <<frame <<"_end");
    auto f = C->getFrame(name, false);
    if(!f) C->addFrame(name);
    name = STRING("_" <<frame <<"_start");
    f = C->getFrame(name, false);
    if(!f) C->addFrame(name);
  }
}

void ManipulationModelling::setup_inverse_kinematics(double homing_scale, bool accumulated_collisions, bool joint_limits, bool quaternion_norms){
  // setup a 1 phase single step problem
  komo = make_shared<KOMO>(*C, 1., 1, 0, accumulated_collisions);
  komo->addControlObjective({}, 0, homing_scale);
  if (accumulated_collisions){
    komo->addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e0});
  }
  if (joint_limits){
    komo->addObjective({}, FS_jointLimits, {}, OT_ineq, {1e0});
  }
  if (quaternion_norms){
    komo->addQuaternionNorms();
  }
}

void ManipulationModelling::setup_pick_and_place_waypoints(const char* gripper, const char* obj, double homing_scale, double velocity_scale, bool accumulated_collisions, bool joint_limits, bool quaternion_norms){
  // setup a 2 phase pick-and-place problem, with a pick switch at time 1, and a place switch at time 2
  // the place mode switch at the final time two might seem obselete, but this switch also implies the geometric constraints of placeOn
  komo = make_shared<KOMO>(*C, 2., 1, 1, accumulated_collisions);
  komo->addControlObjective({}, 0, homing_scale);
  komo->addControlObjective({}, 1, velocity_scale);
  if (accumulated_collisions){
    komo->addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e0});
  }
  if (joint_limits){
    komo->addObjective({}, FS_jointLimits, {}, OT_ineq, {1e0});
  }
  if (quaternion_norms){
    komo->addQuaternionNorms();
  }

  komo->addModeSwitch({1.,-1.}, rai::SY_stable, {gripper, obj}, true);
}

void ManipulationModelling::setup_point_to_point_motion(const arr& q0, const arr& q1, double homing_scale, double acceleration_scale, bool accumulated_collisions, bool quaternion_norms){
  // setup a 1 phase fine-grained motion problem with 2nd order (acceleration) control costs
  C->setJointState(q1);
  for(auto& frame:helpers){
    auto f = C->getFrame(STRING("_" <<frame <<"_end"), false);
    CHECK(f, "helpers need to be specified");
//    if(!f) f = C->addFrame(STRING("_" <<frame <<"_end"));
    auto f_org = C->getFrame(frame);
    f->setPosition(f_org->getPosition());
    f->setQuaternion(f_org->getQuaternion());
    //f.setShape(ry.ST.marker, {.1});
    //f.setColor({0,0,1});
  }
  C->setJointState(q0);
  for(auto& frame: helpers){
    auto f = C->getFrame(STRING("_" <<frame <<"_start"), false);
    CHECK(f, "helpers need to be specified");
    if(!f) f = C->addFrame(STRING("_" <<frame <<"_start"));
    auto f_org = C->getFrame(frame);
    f->setPosition(f_org->getPosition());
    f->setQuaternion(f_org->getQuaternion());
    //f.setShape(ry.ST.marker, {.1});
    //f.setColor({0,1,0});
  }
  komo = make_shared<KOMO>(*C, 1., 32, 2, false);
  komo->addControlObjective({}, 0, homing_scale);
  komo->addControlObjective({}, 2, acceleration_scale);
  komo->initWithWaypoints({q1}, 1, true, .5, 0);
  if (quaternion_norms){
    komo->addQuaternionNorms();
  }

  // zero vel at end
  komo->addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, {}, 1);

  // end point
  komo->addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, q1);
}

void ManipulationModelling::setup_point_to_point_rrt(const arr& q0, const arr& q1, const StringA& explicitCollisionPairs){
  rrt = make_shared<rai::PathFinder>();
  rrt->setProblem(*C, q0, q1);
  if(explicitCollisionPairs.N) rrt->setExplicitCollisionPairs(explicitCollisionPairs);
}

void ManipulationModelling::add_helper_frame(rai::JointType type, const char* parent, const char* name, const char* initFrame){
  rai::Frame *f = komo->addStableFrame(type, parent, name, initFrame);
  f->setShape(rai::ST_marker, {.2});
  f->setColor({1.,0., 1.});
  f->joint->sampleSdv=1.;
  f->joint->setRandom(komo->timeSlices.d1, 0);
}

void ManipulationModelling::grasp_top_box(double time, const char* gripper, const char* obj, str grasp_direction){
  // grasp a box with a centered top grasp (axes fully aligned);
  rai::Array<FeatureSymbol> align;
  if (grasp_direction == "xz"){
    align = {FS_scalarProductXY, FS_scalarProductXZ, FS_scalarProductYZ};
  } else if(grasp_direction == "yz"){
    align = {FS_scalarProductYY, FS_scalarProductXZ, FS_scalarProductYZ};
  } else if(grasp_direction == "xy"){
    align = {FS_scalarProductXY, FS_scalarProductXZ, FS_scalarProductZZ};
  } else if(grasp_direction == "zy"){
    align = {FS_scalarProductXX, FS_scalarProductXZ, FS_scalarProductZZ};
  } else if(grasp_direction == "yx"){
    align = {FS_scalarProductYY, FS_scalarProductYZ, FS_scalarProductZZ};
  } else if(grasp_direction == "zx"){
    align = {FS_scalarProductYX, FS_scalarProductYZ, FS_scalarProductZZ};
  }else{
    LOG(-2) <<"pickDirection not defined:" <<grasp_direction;
  }

  // position: centered
  komo->addObjective({time}, FS_positionDiff, {gripper, obj}, OT_eq, {1e1});

  // orientation: grasp axis orthoginal to target plane X-specific
  komo->addObjective({time-.2,time}, align(0), {obj, gripper}, OT_eq, {1e0});
  komo->addObjective({time-.2,time}, align(1), {obj, gripper}, OT_eq, {1e0});
  komo->addObjective({time-.2,time}, align(2), {obj, gripper}, OT_eq, {1e0});
}

void ManipulationModelling::grasp_box(double time, const char* gripper, const char* obj, const char* palm, str grasp_direction, double margin){
  // general grasp of a box, squeezing along provided grasp_axis (-> 3
  // possible grasps of a box), where and angle of grasp is decided by
  // inequalities on grasp plan and no-collision of box and palm
  arr xLine, yzPlane;
  rai::Array<FeatureSymbol> align;
  if (grasp_direction == "x"){
    xLine = arr{1,0,0};
    yzPlane = arr{{2,3}, {0,1,0,0,0,1}};
    align = {FS_scalarProductXY, FS_scalarProductXZ};
  } else if(grasp_direction == "y"){
    xLine = arr{0,1,0};
    yzPlane = arr{{2,3}, {1,0,0,0,0,1}};
    align = {FS_scalarProductXX, FS_scalarProductXZ};
  } else if(grasp_direction == "z"){
    xLine = arr{0,0,1};
    yzPlane = arr{{2,3}, {1,0,0,0,1,0}};
    align = {FS_scalarProductXX, FS_scalarProductXY};
  } else{
    LOG(-2) <<"grasp_direction not defined:" <<grasp_direction;
  }

  arr boxSize = C->getFrame(obj)->getSize();  boxSize.resizeCopy(3);
  boxSize.resizeCopy(3);

  // position: center in inner target plane X-specific
  komo->addObjective({time}, FS_positionRel, {gripper, obj}, OT_eq, xLine*1e1);
  komo->addObjective({time}, FS_positionRel, {gripper, obj}, OT_ineq, yzPlane*1e1, .5*boxSize-margin);
  komo->addObjective({time}, FS_positionRel, {gripper, obj}, OT_ineq, yzPlane*(-1e1), -.5*boxSize+margin);

  // orientation: grasp axis orthoginal to target plane X-specific
  komo->addObjective({time-.2,time}, align(0), {gripper, obj}, OT_eq, {1e0});
  komo->addObjective({time-.2,time}, align(1), {gripper, obj}, OT_eq, {1e0});

  // no collision with palm
  komo->addObjective({time-.3,time}, FS_distance, {palm, obj}, OT_ineq, {1e1}, {-.001});
}

void ManipulationModelling::grasp_cylinder(double time, const char* gripper, const char* obj, const char* palm, double margin){
  // general grasp of a cylinder, with squeezing the axis normally,
  // inequality along z-axis for positioning, and no-collision with palm
  arr size = C->getFrame(obj)->getSize();

  // position: center along axis, stay within z-range
  komo->addObjective({time}, FS_positionRel, {gripper, obj}, OT_eq, arr{{2,3}, {1,0,0,0,1,0}}*1e1);
  komo->addObjective({time}, FS_positionRel, {gripper, obj}, OT_ineq, arr{0,0,1}*1e1, arr{0.,0.,.5*size(0)-margin});
  komo->addObjective({time}, FS_positionRel, {gripper, obj}, OT_ineq, arr{0,0,1}*(-1e1), arr{0.,0.,-.5*size(0)+margin});

  // orientation: grasp axis orthoginal to target plane X-specific
  komo->addObjective({time-.2,time}, FS_scalarProductXZ, {gripper, obj}, OT_eq, {1e0});

  // no collision with palm
  komo->addObjective({time-.3,time}, FS_distance, {palm, obj}, OT_ineq, {1e1}, {-.001});
}

void ManipulationModelling::place_box(double time, const char* obj, const char* table, const char* palm, str place_direction, double margin){
  // placement of one box on another
  arr zVectorTarget = arr{0.,0.,1.};
  arr boxSize = C->getFrame(obj)->getSize();  boxSize.resizeCopy(3);
  arr tableSize = C->getFrame(table)->getSize();  tableSize.resizeCopy(3);
  double relPos=0.;
  FeatureSymbol zVector;
  rai::Array<FeatureSymbol> align;
  if (place_direction == "x"){
    relPos = .5*(boxSize(0)+tableSize(2));
    zVector = FS_vectorX;
    align = {FS_scalarProductXX, FS_scalarProductYX};
  } else if(place_direction == "y"){
    relPos = .5*(boxSize(1)+tableSize(2));
    zVector = FS_vectorY;
    align = {FS_scalarProductXY, FS_scalarProductYY};
  } else if(place_direction == "z"){
    relPos = .5*(boxSize(2)+tableSize(2));
    zVector = FS_vectorZ;
    align = {FS_scalarProductXZ, FS_scalarProductYZ};
  } else if(place_direction == "xNeg"){
    relPos = .5*(boxSize(0)+tableSize(2));
    zVector = FS_vectorX;
    zVectorTarget *= -1.;
    align = {FS_scalarProductXX, FS_scalarProductYX};
  } else if(place_direction == "yNeg"){
    relPos = .5*(boxSize(1)+tableSize(2));
    zVector = FS_vectorY;
    zVectorTarget *= -1.;
    align = {FS_scalarProductXY, FS_scalarProductYY};
  } else if(place_direction == "zNeg"){
    relPos = .5*(boxSize(2)+tableSize(2));
    zVector = FS_vectorZ;
    zVectorTarget *= -1.;
    align = {FS_scalarProductXZ, FS_scalarProductYZ};
  } else {
    LOG(-2) <<"place_direction not defined:" <<place_direction;
  }

  // position: above table, inside table
  komo->addObjective({time}, FS_positionDiff, {obj, table}, OT_eq, 1e1*arr{{1,3},{0,0,1}}, arr{.0, .0, relPos});
  komo->addObjective({time}, FS_positionRel, {obj, table}, OT_ineq, 1e1*arr{{2,3},{1,0,0,0,1,0}}, .5*tableSize-margin);
  komo->addObjective({time}, FS_positionRel, {obj, table}, OT_ineq, -1e1*arr{{2,3},{1,0,0,0,1,0}}, -.5*tableSize+margin);

  // orientation: Z-up
  komo->addObjective({time-.2, time}, zVector, {obj}, OT_eq, {0.5}, zVectorTarget);
  komo->addObjective({time-.2,time}, align(0), {table, obj}, OT_eq, {1e0});
  komo->addObjective({time-.2,time}, align(1), {table, obj}, OT_eq, {1e0});

  // no collision with palm
  komo->addObjective({time-.3,time}, FS_distance, {palm, table}, OT_ineq, {1e1}, {-.001});
}

void ManipulationModelling::straight_push(arr times, str obj, str gripper, str table){
  //start & end helper frames
  add_helper_frame(rai::JT_hingeZ, table, "_push_start", obj);
  add_helper_frame(rai::JT_transXYPhi, table, "_push_end", obj);

  //-- couple both frames symmetricaly
  //aligned orientation
  komo->addObjective({times(0)}, FS_vectorYDiff, {"_push_start", "_push_end"}, OT_eq, {1e1});
  //aligned position
  komo->addObjective({times(0)}, FS_positionRel, {"_push_end", "_push_start"}, OT_eq, 1e1*arr{{2,3},{1.,0.,0.,0.,0.,1.}});
  komo->addObjective({times(0)}, FS_positionRel, {"_push_start", "_push_end"}, OT_eq, 1e1*arr{{2,3},{1.,0.,0.,0.,0.,1.}});
  //at least 2cm appart, in positive!!! direction
  komo->addObjective({times(0)}, FS_positionRel, {"_push_end", "_push_start"}, OT_ineq, -1e2*arr{{1,3},{0.,1.,0.}}, {.0, .02, .0});
  komo->addObjective({times(0)}, FS_positionRel, {"_push_start", "_push_end"}, OT_ineq, 1e2*arr{{1,3},{0.,1.,0.}}, {.0, -.02, .0});

  //gripper touch
  komo->addObjective({times(0)}, FS_negDistance, {gripper, obj}, OT_eq, {1e1}, {-.02});
  //gripper start position
  komo->addObjective({times(0)}, FS_positionRel, {gripper, "_push_start"}, OT_eq, 1e1*arr{{2,3},{1.,0.,0.,0.,0.,1.}});
  komo->addObjective({times(0)}, FS_positionRel, {gripper, "_push_start"}, OT_ineq, 1e1*arr{{1,3},{0.,1.,0.}}, {.0,-.02,.0});
  //gripper start orientation
  komo->addObjective({times(0)}, FS_scalarProductYY, {gripper, "_push_start"}, OT_ineq, {-1e1}, {.2});
  komo->addObjective({times(0)}, FS_scalarProductYZ, {gripper, "_push_start"}, OT_ineq, {-1e1}, {.2});
  komo->addObjective({times(0)}, FS_vectorXDiff, {gripper, "_push_start"}, OT_eq, {1e1});

  //obj end position
  komo->addObjective({times(1)}, FS_positionDiff, {obj, "_push_end"}, OT_eq, {1e1});
  //obj end orientation: unchanged
  komo->addObjective({times(1)}, FS_quaternion, {obj}, OT_eq, {1e1}, {}, 1); //qobjPose.rot.getArr4d());
}

void ManipulationModelling::no_collision(const arr& time_interval, const char* obj1, const char* obj2, double margin){
  // inequality on distance between two objects
  komo->addObjective(time_interval, FS_negDistance, {obj1, obj2}, OT_ineq, {1e1}, {-margin});
}

void ManipulationModelling::switch_pick(){
  // a kinematic mode switch, where obj becomes attached to gripper, with freely parameterized but stable (=constant) relative pose
}

void ManipulationModelling::switch_place(){
  // a kinematic mode switch, where obj becomes attached to table, with a 3D parameterized (XYPhi) stable relative pose
  // this requires obj and table to be boxes and assumes default placement alone z-axis
  // more general placements have to be modelled with switch_pick (table picking the object) and additinal user-defined geometric constraints
}

void ManipulationModelling::target_position(){
  // impose a specific 3D target position on some object
}

void ManipulationModelling::target_relative_xy_position(double time, const char* obj, const char* relativeTo, arr pos){
  // impose a specific 3D target position on some object
  if(pos.N==2){
    pos.append(0.);
  }
  komo->addObjective({time}, FS_positionRel, {obj, relativeTo}, OT_eq, 1e1*arr{{2,3},{1,0,0,0,1,0}}, pos);
}

void ManipulationModelling::target_x_orientation(double time, const char* obj, const arr& x_vector){
  komo->addObjective({time}, FS_vectorX, {obj}, OT_eq, {1e1}, x_vector);
}

void ManipulationModelling::bias(double time, arr& qBias, double scale){
  // impose a square potential bias directly in joint space
  komo->addObjective({time}, FS_qItself, {}, OT_sos, {scale}, qBias);
}

void ManipulationModelling::retract(const arr& time_interval, const char* gripper, double dist){
  auto helper = STRING("_" <<gripper <<"_start");
  komo->addObjective(time_interval, FS_positionRel, {gripper, helper}, OT_eq, 1e2 * arr{{1,3},{1,0,0}});
  komo->addObjective(time_interval, FS_quaternionDiff, {gripper, helper}, OT_eq, {1e2});
  komo->addObjective({time_interval(1)}, FS_positionRel, {gripper, helper}, OT_ineq, -1e2 * arr{{1,3},{0,0,1}}, {0., 0., dist});
}

void ManipulationModelling::approach(const arr& time_interval, const char* gripper, double dist){
  auto helper = STRING("_" <<gripper <<"_end");
  komo->addObjective(time_interval, FS_positionRel, {gripper, helper}, OT_eq, 1e2 * arr{{1,3},{1,0,0}});
  komo->addObjective(time_interval, FS_quaternionDiff, {gripper, helper}, OT_eq, {1e2});
  komo->addObjective({time_interval(0)}, FS_positionRel, {gripper, helper}, OT_ineq, -1e2 * arr{{1,3},{0,0,1}}, {0., 0., dist});
}

void ManipulationModelling::retractPush(const arr& time_interval, const char* gripper, double dist){
  auto helper = STRING("_" <<gripper <<"_start");
//  komo->addObjective(time_interval, FS_positionRel, {gripper, helper}, OT_eq, 1e2 * arr{{1,3},{1,0,0}});
//  komo->addObjective(time_interval, FS_quaternionDiff, {gripper, helper}, OT_eq, {1e2});
  komo->addObjective(time_interval, FS_positionRel, {gripper, helper}, OT_eq, 1e2 * arr{{1,3},{1,0,0}});
  komo->addObjective({time_interval(1)}, FS_positionRel, {gripper, helper}, OT_ineq, 1e2 * arr{{1,3},{0,1,0}}, {0., -dist, 0.});
  komo->addObjective({time_interval(1)}, FS_positionRel, {gripper, helper}, OT_ineq, -1e2 * arr{{1,3},{0,0,1}}, {0., 0., dist});
}

void ManipulationModelling::approachPush(const arr& time_interval, const char* gripper, double dist, str _helper){
//  if(!helper.N) helper = STRING("_push_start");
//  komo->addObjective(time_interval, FS_positionRel, {gripper, helper}, OT_eq, 1e2 * arr{{2,3},{1,0,0,0,0,1}});
//  komo->addObjective({time_interval(0)}, FS_positionRel, {gripper, helper}, OT_ineq, 1e2 * arr{{1,3},{0,1,0}}, {0., -dist, 0.});
  auto helper = STRING("_" <<gripper <<"_end");
  komo->addObjective(time_interval, FS_positionRel, {gripper, helper}, OT_eq, 1e2 * arr{{1,3},{1,0,0}});
  komo->addObjective({time_interval(0)}, FS_positionRel, {gripper, helper}, OT_ineq, 1e2 * arr{{1,3},{0,1,0}}, {0., -dist, 0.});
  komo->addObjective({time_interval(0)}, FS_positionRel, {gripper, helper}, OT_ineq, -1e2 * arr{{1,3},{0,0,1}}, {0., 0., dist});
}

arr ManipulationModelling::solve(int verbose){
  if(komo){
    NLP_Solver sol;
    sol.setProblem(komo->nlp());
    sol.opt.set_damping(1e-3). set_verbose(verbose-1). set_stopTolerance(1e-3). set_maxLambda(100.). set_stopEvals(200);
    ret = sol.solve();
    if(ret->feasible){
      path = komo->getPath_qOrg();
    }else{
      path.clear();
    }
    if(!ret->feasible){
      if(verbose>0){
        cout <<"  -- infeasible:" <<info <<"\n     " <<*ret <<endl;
        if(verbose>1){
          cout <<komo->report(false, true) <<endl;
        }
        komo->view(true, STRING("failed: " <<info <<"\n" <<*ret));
        if(verbose>2){
          while(komo->view_play(true, 1.));
        }
      }
    }else{
      if(verbose>0){
        cout <<"  -- feasible:" <<info <<"\n     " <<*ret <<endl;
        if(verbose>2){
          komo->view(true, STRING("success: " <<info <<"\n" <<*ret));
          if(verbose>3){
            while(komo->view_play(true, 1.));
          }
        }
      }
    }

  }else if(rrt){
    rrt->rrtSolver->verbose=verbose;
    ret = rrt->solve();
    if(ret->feasible) path = ret->x;
    else path.clear();
  }else{
    NIY
  }
  return path;
}

std::shared_ptr<ManipulationModelling> ManipulationModelling::sub_motion(uint phase, double homing_scale, double acceleration_scale, bool accumulated_collisions, bool quaternion_norms){
  rai::Configuration C;
  arr q0, q1;
  komo->getSubProblem(phase, C, q0, q1);

  std::shared_ptr<ManipulationModelling> manip = make_shared<ManipulationModelling>(C, STRING("sub_motion"<<phase<<"--"<<info), helpers);
  manip->setup_point_to_point_motion(q0, q1, homing_scale, acceleration_scale, accumulated_collisions, quaternion_norms);
  return manip;
}

std::shared_ptr<ManipulationModelling> ManipulationModelling::sub_rrt(uint phase, const StringA& explicitCollisionPairs){
  rai::Configuration C;
  arr q0, q1;
  komo->getSubProblem(phase, C, q0, q1);

  std::shared_ptr<ManipulationModelling> manip = make_shared<ManipulationModelling>(C, STRING("sub_rrt"<<phase<<"--"<<info), helpers);
  manip->setup_point_to_point_rrt(q0, q1, explicitCollisionPairs);
  return manip;
}
