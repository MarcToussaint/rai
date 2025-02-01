/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "manipTools.h"
#include "skeletonSymbol.h"

#include "../Kin/frame.h"
#include "../Kin/feature.h"
#include "../Kin/F_pose.h"

#include "../Optim/NLP_Solver.h"
#include "../Optim/NLP_Sampler.h"

ManipulationModelling::ManipulationModelling(const str& _info)
    : komo(make_shared<KOMO>()), info(_info) {
}

ManipulationModelling::ManipulationModelling(const std::shared_ptr<KOMO>& _komo, const str& _info)
  : komo(_komo), info(_info) {
}

void ManipulationModelling::setup_inverse_kinematics(rai::Configuration& C, double homing_scale, bool accumulated_collisions, bool joint_limits, bool quaternion_norms) {
  /* setup a 1 phase single step problem */
  k().setTiming(1., 1, 1., 0);
  k().setConfig(C, accumulated_collisions);
  k().addControlObjective({}, 0, homing_scale);
  if(accumulated_collisions) {
    k().addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e0});
  }
  if(joint_limits) {
    k().addObjective({}, FS_jointLimits, {}, OT_ineq, {1e0});
  }
  if(quaternion_norms) {
    k().addQuaternionNorms();
  }
}

void ManipulationModelling::setup_sequence(rai::Configuration& C, uint K, double homing_scale, double velocity_scale, bool accumulated_collisions, bool joint_limits, bool quaternion_norms){
  k().setTiming(double(K), 1, 1., 1);
  k().setConfig(C, accumulated_collisions);
  k().addControlObjective({}, 0, homing_scale);
  k().addControlObjective({}, 1, velocity_scale);
  if(accumulated_collisions) {
    k().addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e0});
  }
  if(joint_limits) {
    k().addObjective({}, FS_jointLimits, {}, OT_ineq, {1e0});
  }
  if(quaternion_norms) {
    k().addQuaternionNorms();
  }
}

void ManipulationModelling::setup_motion(rai::Configuration& C, uint K, uint steps_per_phase, double homing_scale, double acceleration_scale, bool accumulated_collisions, bool joint_limits, bool quaternion_norms){
  k().setTiming(double(K), steps_per_phase, 1., 2);
  k().setConfig(C, accumulated_collisions);
  if(homing_scale>0.) k().addControlObjective({}, 0, homing_scale);
  k().addControlObjective({}, 2, acceleration_scale);
  if(accumulated_collisions) {
    k().addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e0});
  }
  if(joint_limits) {
    k().addObjective({}, FS_jointLimits, {}, OT_ineq, {1e0});
  }
  if(quaternion_norms) {
    k().addQuaternionNorms();
  }

  // zero vel at end
  k().addObjective({double(K)}, FS_qItself, {}, OT_eq, {1e0}, {}, 1);
}

void ManipulationModelling::setup_pick_and_place_waypoints(rai::Configuration& C, const char* gripper, const char* obj, double homing_scale, double velocity_scale, bool accumulated_collisions, bool joint_limits, bool quaternion_norms) {
  /* setup a 2 phase pick-and-place problem, with a pick switch at time 1, and a place switch at time 2
     the place mode switch at the final time two might seem obselete, but this switch also implies the geometric constraints of placeOn */
  CHECK(!komo->T, "komo already previously setup");
  setup_sequence(C, 2, homing_scale, velocity_scale, accumulated_collisions, joint_limits, quaternion_norms);

  k().addModeSwitch({1., -1.}, rai::SY_stable, {gripper, obj}, true);
}

void ManipulationModelling::setup_point_to_point_motion(rai::Configuration& C, const arr& q1, double homing_scale, double acceleration_scale, bool accumulated_collisions, bool joint_limits, bool quaternion_norms) {
  /* setup a 1 phase fine-grained motion problem with 2nd order (acceleration) control costs */
  CHECK(!komo->T, "komo already previously setup");
  setup_motion(C, 1, 32, homing_scale, acceleration_scale, accumulated_collisions, joint_limits, quaternion_norms);

  if(q1.N){
    k().initWithWaypoints({q1}, 1, true, .5, 0);
    k().addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, q1);
  }
}

void ManipulationModelling::add_helper_frame(rai::JointType type, const char* parent, const char* name, const char* initName, rai::Frame* initFrame, double markerSize) {
  rai::Frame* f = k().addFrameDof(name, parent, type, true, initName, initFrame);
  if(markerSize>0.){
    f->setShape(rai::ST_marker, {.1});
    f->setColor({1., 0., 1.});
  }
  if(f->joint){
    f->joint->sampleSdv=1.;
    f->joint->setRandom(k().timeSlices.d1, 0);
  }
}

void ManipulationModelling::grasp_top_box(double time, const char* gripper, const char* obj, str grasp_direction) {
  /* grasp a box with a centered top grasp (axes fully aligned) */
  rai::Array<FeatureSymbol> align;
  if(grasp_direction == "xz") {
    align = {FS_scalarProductXY, FS_scalarProductXZ, FS_scalarProductYZ} ;
  } else if(grasp_direction == "yz") {
    align = {FS_scalarProductYY, FS_scalarProductXZ, FS_scalarProductYZ} ;
  } else if(grasp_direction == "xy") {
    align = {FS_scalarProductXY, FS_scalarProductXZ, FS_scalarProductZZ} ;
  } else if(grasp_direction == "zy") {
    align = {FS_scalarProductXX, FS_scalarProductXZ, FS_scalarProductZZ} ;
  } else if(grasp_direction == "yx") {
    align = {FS_scalarProductYY, FS_scalarProductYZ, FS_scalarProductZZ} ;
  } else if(grasp_direction == "zx") {
    align = {FS_scalarProductYX, FS_scalarProductYZ, FS_scalarProductZZ} ;
  } else {
    LOG(-2) <<"pickDirection not defined:" <<grasp_direction;
  }

  // position: centered
  k().addObjective({time}, FS_positionDiff, {gripper, obj}, OT_eq, {1e1});

  // orientation: grasp axis orthoginal to target plane X-specific
  k().addObjective({time-.2, time}, align(0), {obj, gripper}, OT_eq, {1e0});
  k().addObjective({time-.2, time}, align(1), {obj, gripper}, OT_eq, {1e0});
  k().addObjective({time-.2, time}, align(2), {obj, gripper}, OT_eq, {1e0});
}

void ManipulationModelling::grasp_box(double time, const char* gripper, const char* obj, const char* palm, str grasp_direction, double margin) {
  /* general grasp of a box, squeezing along provided grasp_axis (-> 3
     possible grasps of a box), where and angle of grasp is decided by
     inequalities on grasp plan and no-collision of box and palm */
  arr xLine, yzPlane;
  rai::Array<FeatureSymbol> align;
  if(grasp_direction == "x") {
    xLine = arr{1, 0, 0} ;
    yzPlane = arr{{2, 3}, {0, 1, 0, 0, 0, 1}} ;
    align = {FS_scalarProductXY, FS_scalarProductXZ} ;
  } else if(grasp_direction == "y") {
    xLine = arr{0, 1, 0} ;
    yzPlane = arr{{2, 3}, {1, 0, 0, 0, 0, 1}} ;
    align = {FS_scalarProductXX, FS_scalarProductXZ} ;
  } else if(grasp_direction == "z") {
    xLine = arr{0, 0, 1} ;
    yzPlane = arr{{2, 3}, {1, 0, 0, 0, 1, 0}} ;
    align = {FS_scalarProductXX, FS_scalarProductXY} ;
  } else {
    LOG(-2) <<"grasp_direction not defined:" <<grasp_direction;
  }

  arr boxSize = k().world.getFrame(obj)->getSize();
  boxSize.resizeCopy(3);

  // position: center in inner target plane X-specific
  k().addObjective({time}, FS_positionRel, {gripper, obj}, OT_eq, xLine*1e1);
  k().addObjective({time}, FS_positionRel, {gripper, obj}, OT_ineq, yzPlane*1e1, .5*boxSize-margin);
  k().addObjective({time}, FS_positionRel, {gripper, obj}, OT_ineq, yzPlane*(-1e1), -.5*boxSize+margin);

  // orientation: grasp axis orthoginal to target plane X-specific
  k().addObjective({time-.2, time}, align(0), {gripper, obj}, OT_eq, {1e0});
  k().addObjective({time-.2, time}, align(1), {gripper, obj}, OT_eq, {1e0});

  // no collision with palm
  k().addObjective({time-.3, time}, FS_distance, {palm, obj}, OT_ineq, {1e1}, {-.001});
}

void ManipulationModelling::grasp_cylinder(double time, const char* gripper, const char* obj, const char* palm, double margin) {
  /* general grasp of a cylinder, with squeezing the axis normally,
     inequality along z-axis for positioning, and no-collision with palm */
  arr size = k().world.getFrame(obj)->getSize();

  // position: center along axis, stay within z-range
  k().addObjective({time}, FS_positionRel, {gripper, obj}, OT_eq, arr{{2, 3}, {1, 0, 0, 0, 1, 0}}*1e1);
  k().addObjective({time}, FS_positionRel, {gripper, obj}, OT_ineq, arr{0, 0, 1}*1e1, arr{0., 0., .5*size(0)-margin});
  k().addObjective({time}, FS_positionRel, {gripper, obj}, OT_ineq, arr{0, 0, 1}*(-1e1), arr{0., 0., -.5*size(0)+margin});

  // orientation: grasp axis orthoginal to target plane X-specific
  k().addObjective({time-.2, time}, FS_scalarProductXZ, {gripper, obj}, OT_eq, {1e0});

  // no collision with palm
  k().addObjective({time-.3, time}, FS_distance, {palm, obj}, OT_ineq, {1e1}, {-.001});
}

//void ManipulationModelling::grasp_cylinder(double time, const char* gripper, const char* obj, const char* palm, double margin){
//  auto size = k().world[obj]->getSize();

//  k().addObjective({time}, FS_positionRel, {gripper, obj}, OT_eq, arr({2,3},{1,0,0,0,1,0})*1e1);
//  k().addObjective({time}, FS_positionRel, {gripper, obj}, OT_ineq, arr({1,3},{0,0,1})*1e1, {0.,0.,.5*size(0)-margin});
//  k().addObjective({time}, FS_positionRel, {gripper, obj}, OT_ineq, arr({1,3},{0,0,1})*(-1e1), {0.,0.,-.5*size(0)+margin});

//  // orientation: grasp axis orthoginal to target plane X-specific
//  k().addObjective({time-.2,time}, FS_scalarProductXZ, {gripper, obj}, OT_eq, {1e0});

//  // no collision with palm
//  k().addObjective({time-.3,time}, FS_negDistance, {palm, obj}, OT_ineq, {1e1}, {-.001});
//}

//void ManipulationModelling::no_collision(const arr& times, const char* obj1, const char* obj2, double margin){
//  k().addObjective(times, FS_negDistance, {obj1, obj2}, OT_ineq, {1e1}, {-margin});
//}

void ManipulationModelling::place_box(double time, const char* obj, const char* table, const char* palm, str place_direction, double margin) {
  /* placement of one box on another */
  arr zVectorTarget = arr{0., 0., 1.} ;
  rai::Frame *obj_frame = k().world.getFrame(obj);
  arr boxSize = obj_frame->getSize();
  if(obj_frame->shape->type()==rai::ST_ssBox){
    boxSize.resizeCopy(3);
  }else if(obj_frame->shape->type()==rai::ST_ssCylinder){
    boxSize = {boxSize(1), boxSize(1), boxSize(0)} ;
  }else NIY
  arr tableSize = k().world.getFrame(table)->getSize();  tableSize.resizeCopy(3);
  double relPos=0.;
  FeatureSymbol zVector;
  rai::Array<FeatureSymbol> align;
  if(place_direction == "x") {
    relPos = .5*(boxSize(0)+tableSize(2));
    zVector = FS_vectorX;
    align = {FS_scalarProductXX, FS_scalarProductYX} ;
  } else if(place_direction == "y") {
    relPos = .5*(boxSize(1)+tableSize(2));
    zVector = FS_vectorY;
    align = {FS_scalarProductXY, FS_scalarProductYY} ;
  } else if(place_direction == "z") {
    relPos = .5*(boxSize(2)+tableSize(2));
    zVector = FS_vectorZ;
    align = {FS_scalarProductXZ, FS_scalarProductYZ} ;
  } else if(place_direction == "xNeg") {
    relPos = .5*(boxSize(0)+tableSize(2));
    zVector = FS_vectorX;
    zVectorTarget *= -1.;
    align = {FS_scalarProductXX, FS_scalarProductYX} ;
  } else if(place_direction == "yNeg") {
    relPos = .5*(boxSize(1)+tableSize(2));
    zVector = FS_vectorY;
    zVectorTarget *= -1.;
    align = {FS_scalarProductXY, FS_scalarProductYY} ;
  } else if(place_direction == "zNeg") {
    relPos = .5*(boxSize(2)+tableSize(2));
    zVector = FS_vectorZ;
    zVectorTarget *= -1.;
    align = {FS_scalarProductXZ, FS_scalarProductYZ} ;
  } else {
    LOG(-2) <<"place_direction not defined:" <<place_direction;
  }

  // position: above table, inside table
  k().addObjective({time}, FS_positionDiff, {obj, table}, OT_eq, 1e1*arr{{1, 3}, {0, 0, 1}}, arr{.0, .0, relPos});
  k().addObjective({time}, FS_positionRel, {obj, table}, OT_ineq, 1e1*arr{{2, 3}, {1, 0, 0, 0, 1, 0}}, .5*tableSize-margin);
  k().addObjective({time}, FS_positionRel, {obj, table}, OT_ineq, -1e1*arr{{2, 3}, {1, 0, 0, 0, 1, 0}}, -.5*tableSize+margin);

  // orientation: Z-up
  k().addObjective({time-.2, time}, zVector, {obj}, OT_eq, {0.5}, zVectorTarget);
  k().addObjective({time-.2, time}, align(0), {table, obj}, OT_eq, {1e0});
  k().addObjective({time-.2, time}, align(1), {table, obj}, OT_eq, {1e0});

  // no collision with palm
  if(palm) k().addObjective({time-.3, time}, FS_distance, {palm, table}, OT_ineq, {1e1}, {-.001});
}

struct AlignWithDiff : Feature {
  rai::Vector ref;
  AlignWithDiff(const rai::Vector& _ref=Vector_x) : ref(_ref) { setOrder(1); }
  virtual arr phi(const FrameL& F){
    CHECK_EQ(order, 1, "");
    CHECK_EQ(F.N, 4, "");
    arr v = F_Vector(ref).eval({F(0,0)});
    arr d = F_Position().eval({F(1,1)}) - F_Position().eval({F(0,1)});
    op_normalize(d, 1e-4);
    arr y = d - v; //*(~v*d);
    return y;
  }
  virtual uint dim_phi(const FrameL& F) {
    return 3;
  }
};

void ManipulationModelling::straight_push(arr time_interval, str obj, str gripper, str table) {
  //start & end helper frames
  str helperStart = STRING("_straight_pushStart_" <<gripper <<"_" <<obj <<'_' <<time_interval(0));
  str helperEnd = STRING("_straight_pushEnd_" <<gripper <<"_" <<obj <<'_' <<time_interval(1));
  if(!k().world.getFrame(helperStart, false)){
    // add_helper_frame(rai::JT_hingeZ, table, helperStart, obj, 0, .3);
    rai::Frame* helper_frame = komo->addFrameDof(helperStart, obj, rai::JT_hingeZ, true);
    helper_frame->setAutoLimits();
    helper_frame->joint->sampleUniform=1.;
  }
  // if(!k().world.getFrame(helperEnd, false))
    // add_helper_frame(rai::JT_transXYPhi, table, helperEnd, obj, 0, .3);

#if 0
  //-- couple both frames symmetricaly
  //aligned orientation
  k().addObjective({time_interval(0)}, FS_vectorYDiff, {helperStart, helperEnd}, OT_eq, {1e1});
  //aligned position
  k().addObjective({time_interval(0)}, FS_positionRel, {helperEnd, helperStart}, OT_eq, 1e1*arr{{2, 3}, {1., 0., 0., 0., 0., 1.}});
  k().addObjective({time_interval(0)}, FS_positionRel, {helperStart, helperEnd}, OT_eq, 1e1*arr{{2, 3}, {1., 0., 0., 0., 0., 1.}});
  //at least 2cm appart, in positive!!! direction
  k().addObjective({time_interval(0)}, FS_positionRel, {helperEnd, helperStart}, OT_ineq, -1e2*arr{{1, 3}, {0., 1., 0.}}, {.0, .02, .0});
  k().addObjective({time_interval(0)}, FS_positionRel, {helperStart, helperEnd}, OT_ineq, 1e2*arr{{1, 3}, {0., 1., 0.}}, {.0, -.02, .0});
#else

  //x-axis of A aligns with diff-pos of B AT END TIME! (always backward diff)
  k().addObjective({time_interval(1)}, make_shared<AlignWithDiff>(Vector_y), {helperStart, obj}, OT_eq, {1e0}, {}, 1);
#endif

  //gripper touch
  k().addObjective({time_interval(0)}, FS_negDistance, {gripper, obj}, OT_eq, {1e0}, {-.025});
  // //gripper start position
  k().addObjective({time_interval(0)}, FS_positionRel, {gripper, helperStart}, OT_eq, 1e0*arr{{2, 3}, {1., 0., 0., 0., 0., 1.}});
  k().addObjective({time_interval(0)}, FS_positionRel, {gripper, helperStart}, OT_ineq, 1e0*arr{{1, 3}, {0., 1., 0.}}, {.0, -.02, .0});
  //gripper start orientation
  k().addObjective({time_interval(0)}, FS_scalarProductYY, {gripper, helperStart}, OT_ineq, {-1e0}, {.2});
  k().addObjective({time_interval(0)}, FS_scalarProductYZ, {gripper, helperStart}, OT_ineq, {-1e0}, {.2});
  k().addObjective({time_interval(0)}, FS_vectorXDiff, {gripper, helperStart}, OT_eq, {1e0});

  //obj end position
  // k().addObjective({time_interval(1)}, FS_positionDiff, {obj, helperEnd}, OT_eq, {1e1});
  //obj end orientation: unchanged
  // k().addObjective({time_interval(1)}, FS_quaternion, {obj}, OT_eq, {1e1}, {}, 1); //qobjPose.rot.getArr4d());
  freeze_relativePose({time_interval(1)}, gripper, obj);
}

void ManipulationModelling::no_collision(const arr& time_interval, const StringA& pairs, double margin, double scale) {
  /* inequality on distance between two objects */
  StringA _pairs = pairs.ref();
  _pairs.reshape(-1,2);
  for(uint i=0;i<_pairs.d0;i++){
    k().addObjective(time_interval, FS_negDistance, _pairs[i], OT_ineq, {scale}, {-margin});
  }
}

void ManipulationModelling::freeze_joint(const arr& time_interval, const StringA& joints){
  komo->addObjective(time_interval, FS_qItself, joints, OT_eq, {1e1}, {}, 1);
}

void ManipulationModelling::freeze_relativePose(const arr& time_interval, str to, str from){
  komo->addObjective(time_interval, FS_poseRel, {to, from}, OT_eq, {1e1}, {}, 1);
}

void ManipulationModelling::switch_pick() {
  /* a kinematic mode switch, where obj becomes attached to gripper, with freely parameterized but stable (=constant) relative pose */
}

void ManipulationModelling::switch_place() {
  /* a kinematic mode switch, where obj becomes attached to table, with a 3D parameterized (XYPhi) stable relative pose
     this requires obj and table to be boxes and assumes default placement alone z-axis
     more general placements have to be modelled with switch_pick (table picking the object) and additinal user-defined geometric constraints */
}

void ManipulationModelling::target_position() {
  /* impose a specific 3D target position on some object */
}

void ManipulationModelling::target_relative_xy_position(double time, const char* obj, const char* relativeTo, arr pos) {
  /* impose a specific 3D target position on some object */
  if(pos.N==2) {
    pos.append(0.);
  }
  k().addObjective({time}, FS_positionRel, {obj, relativeTo}, OT_eq, 1e1*arr{{2, 3}, {1, 0, 0, 0, 1, 0}}, pos);
}

void ManipulationModelling::target_x_orientation(double time, const char* obj, const arr& x_vector) {
  k().addObjective({time}, FS_vectorX, {obj}, OT_eq, {1e1}, x_vector);
}

void ManipulationModelling::bias(double time, arr& qBias, double scale) {
  /* impose a square potential bias directly in joint space */
  k().addObjective({time}, FS_qItself, {}, OT_sos, {scale}, qBias);
}

void ManipulationModelling::retract(const arr& time_interval, const char* gripper, double dist) {
  auto helper = STRING("_" <<gripper <<"_retract_" <<time_interval(0));
  int t = conv_time2step(time_interval(0), k().stepsPerPhase);
  rai::Frame *f = k().timeSlices(k().k_order+t, k().world[gripper]->ID);
  add_helper_frame(rai::JT_none, 0, helper, 0, f);
//  k().view(true, helper);

  k().addObjective(time_interval, FS_positionRel, {gripper, helper}, OT_eq, 1e2 * arr{{1, 3}, {1, 0, 0}});
  k().addObjective(time_interval, FS_quaternionDiff, {gripper, helper}, OT_eq, {1e2});
  k().addObjective({time_interval(1)}, FS_positionRel, {gripper, helper}, OT_ineq, -1e2 * arr{{1, 3}, {0, 0, 1}}, {0., 0., dist});
}

void ManipulationModelling::approach(const arr& time_interval, const char* gripper, double dist) {
  auto helper = STRING("_" <<gripper <<"_approach_" <<time_interval(1));
  int t = conv_time2step(time_interval(1), k().stepsPerPhase);
  rai::Frame *f = k().timeSlices(k().k_order+t, k().world[gripper]->ID);
  add_helper_frame(rai::JT_none, 0, helper, 0, f);
//  k().view(true, helper);

  k().addObjective(time_interval, FS_positionRel, {gripper, helper}, OT_eq, 1e2 * arr{{1, 3}, {1, 0, 0}});
  k().addObjective(time_interval, FS_quaternionDiff, {gripper, helper}, OT_eq, {1e2});
  k().addObjective({time_interval(0)}, FS_positionRel, {gripper, helper}, OT_ineq, -1e2 * arr{{1, 3}, {0, 0, 1}}, {0., 0., dist});
}

void ManipulationModelling::retractPush(const arr& time_interval, const char* gripper, double dist) {
  auto helper = STRING("_" <<gripper <<"_retractPush_"  <<time_interval(0));
  int t = conv_time2step(time_interval(0), k().stepsPerPhase);
  rai::Frame *f = k().timeSlices(k().k_order+t, k().world[gripper]->ID);
  add_helper_frame(rai::JT_none, 0, helper, 0, f);
//  k().addObjective(time_interval, FS_positionRel, {gripper, helper}, OT_eq, 1e2 * arr{{1,3},{1,0,0}});
//  k().addObjective(time_interval, FS_quaternionDiff, {gripper, helper}, OT_eq, {1e2});
  k().addObjective(time_interval, FS_positionRel, {gripper, helper}, OT_eq, 1e2 * arr{{1, 3}, {1, 0, 0}});
  k().addObjective({time_interval(1)}, FS_positionRel, {gripper, helper}, OT_ineq, 1e2 * arr{{1, 3}, {0, 1, 0}}, {0., -dist, 0.});
  k().addObjective({time_interval(1)}, FS_positionRel, {gripper, helper}, OT_ineq, -1e2 * arr{{1, 3}, {0, 0, 1}}, {0., 0., dist});
}

void ManipulationModelling::approachPush(const arr& time_interval, const char* gripper, double dist, str _helper) {
//  if(!helper.N) helper = STRING("_push_start");
//  k().addObjective(time_interval, FS_positionRel, {gripper, helper}, OT_eq, 1e2 * arr{{2,3},{1,0,0,0,0,1}});
//  k().addObjective({time_interval(0)}, FS_positionRel, {gripper, helper}, OT_ineq, 1e2 * arr{{1,3},{0,1,0}}, {0., -dist, 0.});
  auto helper = STRING("_" <<gripper <<"_approachPush_" <<time_interval(1));
  int t = conv_time2step(time_interval(1), k().stepsPerPhase);
  rai::Frame *f = k().timeSlices(k().k_order+t, k().world[gripper]->ID);
  add_helper_frame(rai::JT_none, 0, helper, 0, f);
  k().addObjective(time_interval, FS_positionRel, {gripper, helper}, OT_eq, 1e2 * arr{{1, 3}, {1, 0, 0}});
  k().addObjective({time_interval(0)}, FS_positionRel, {gripper, helper}, OT_ineq, 1e2 * arr{{1, 3}, {0, 1, 0}}, {0., -dist, 0.});
  k().addObjective({time_interval(0)}, FS_positionRel, {gripper, helper}, OT_ineq, -1e2 * arr{{1, 3}, {0, 0, 1}}, {0., 0., dist});
}

arr ManipulationModelling::solve(int verbose) {
  if(komo) {
    NLP_Solver sol;
    sol.setProblem(k().nlp());
    sol.opt.set_damping(1e-1). set_verbose(verbose-1). set_stopTolerance(1e-3). set_lambdaMax(100.). set_stopInners(20). set_stopEvals(200);
    ret = sol.solve();
    if(ret->feasible) {
      path = k().getPath_qOrg();
    } else {
      path.clear();
    }
    if(verbose>0) {
      if(!ret->feasible) {
        cout <<"  -- infeasible: " <<info <<"\n     " <<*ret <<endl;
        cout <<k().report(false, true, verbose>1) <<endl;
        cout <<"  --" <<endl;
        if(verbose>1) {
          cout <<sol.reportLagrangeGradients(k().featureNames) <<endl;
        }
        k().view(true, STRING("infeasible: " <<info <<"\n" <<*ret));
        if(verbose>2) {
          k().view_play(true, 0, 1.);
        }
      } else {
        cout <<"  -- feasible: " <<info <<"\n     " <<*ret <<endl;
        if(verbose>2) {
          cout <<sol.reportLagrangeGradients(k().featureNames) <<endl;
          cout <<k().report(false, true, verbose>2) <<endl;
          rai::wait(.5);
          cout <<"  --" <<endl;
          k().view(true, STRING("feasible: " <<info <<"\n" <<*ret));
          if(verbose>3) {
            k().view_play(true, 0, 1.);
          }
        }
      }
    }

  } else {
    NIY
  }
  return path;
}

arr ManipulationModelling::sample(const char* sampleMethod, int verbose) {
  CHECK(komo, "");

  NLP_Sampler sol(k().nlp());
  arr data;
  uintA dataEvals;
  double time = -rai::cpuTime();

//  sol.opt.seedMethod="gauss";
  if(sampleMethod) sol.opt.seedMethod=sampleMethod;
  sol.opt.verbose=verbose;
  sol.opt.downhillMaxSteps=50;
  sol.opt.slackMaxStep=.5;

  sol.run(data, dataEvals);
  time += rai::cpuTime();

  ret = make_shared<SolverReturn>();
  if(data.N){
    ret->x = data.reshape(-1);
    ret->evals = dataEvals.elem();
    ret->feasible = true;
  }else{
    ret->evals = k().evalCount;
    ret->feasible = false;
  }
  ret->time = time;
  ret->done = true;
  {
    arr totals = k().info_errorTotals(k().info_objectiveErrorTraces());
    ret->sos = totals(OT_sos);
    ret->ineq = totals(OT_ineq);
    ret->eq = totals(OT_eq);
    ret->f = totals(OT_f);
  }

  if(ret->feasible) {
    path = k().getPath_qOrg();
  } else {
    path.clear();
  }
  if(!ret->feasible) {
    if(verbose>0) {
      cout <<"  -- infeasible:" <<info <<"\n     " <<*ret <<endl;
      if(verbose>1) {
        cout <<k().report(false, true, verbose>1) <<endl;
        cout <<"  --" <<endl;
      }
      k().view(true, STRING("infeasible: " <<info <<"\n" <<*ret));
      if(verbose>2) {
        k().view_play(true, 0, 1.);
      }
    }
  } else {
    if(verbose>0) {
      cout <<"  -- feasible:" <<info <<"\n     " <<*ret <<endl;
      if(verbose>2) {
        cout <<k().report(false, true, verbose>2) <<endl;
        cout <<"  --" <<endl;
        k().view(true, STRING("feasible: " <<info <<"\n" <<*ret));
        if(verbose>3) {
          k().view_play(true, 0, 1.);
        }
      }
    }
  }

  return path;
}

void ManipulationModelling::debug(bool listObjectives, bool plotOverTime){
  cout <<"  -- DEBUG: " <<info <<endl;
  cout <<"  == solver return: " <<*ret <<endl;
  cout <<"  == all KOMO objectives with increasing errors:\n" <<k().report(false, listObjectives, plotOverTime) <<endl;
//  cout <<"  == objectives sorted by error and Lagrange gradient:\n" <<sol.reportLagrangeGradients(k().featureNames) <<endl;
  cout <<"  == view objective errors over slices in gnuplot" <<endl;
  cout <<"  == scroll through solution in display window using SHIFT-scroll" <<endl;
  k().view(true, STRING("debug: " <<info <<"\n" <<*ret));
}

void ManipulationModelling::play(rai::Configuration& C, double duration) {
  uintA dofIndices = C.getDofIDs();
  for(uint t=0; t<path.d0; t++) {
    C.setFrameState(k().getConfiguration_X(t));
    C.setJointState(k().getConfiguration_dofs(t, dofIndices));
    C.view(false, STRING("step " <<t <<"\n" <<info));
    rai::wait(duration/path.d0);
  }
}

std::shared_ptr<ManipulationModelling> ManipulationModelling::sub_motion(uint phase, bool fixEnd, double homing_scale, double acceleration_scale, bool accumulated_collisions, bool joint_limits, bool quaternion_norms) {
  rai::Configuration C;
  arr q0, q1;
  k().getSubProblem(phase, C, q0, q1);

  if(!fixEnd) q1.clear();

  std::shared_ptr<ManipulationModelling> manip = make_shared<ManipulationModelling>(STRING("sub_motion"<<phase));
  manip->setup_point_to_point_motion(C, q1, homing_scale, acceleration_scale, accumulated_collisions, joint_limits, quaternion_norms);
  return manip;
}

std::shared_ptr<rai::RRT_PathFinder> ManipulationModelling::sub_rrt(uint phase, const StringA& explicitCollisionPairs, const StringA& activeDofs) {
  rai::Configuration C;
  arr q0, q1;
  k().getSubProblem(phase, C, q0, q1);

  if(activeDofs.N){
    DofL orgDofs = C.activeDofs;
    C.selectJointsByName(activeDofs);
    C.setDofState(q1, orgDofs);
    q1 = C.getJointState();
    C.setDofState(q0, orgDofs);
    q0 = C.getJointState();
  }

  std::shared_ptr<rai::RRT_PathFinder> rrt = make_shared<rai::RRT_PathFinder>();
  rrt->setProblem(C);
  rrt->setStartGoal(q0, q1);
  if(explicitCollisionPairs.N) rrt->setExplicitCollisionPairs(explicitCollisionPairs);

  return rrt;
}
