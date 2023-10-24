/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "pr2_heuristics.h"
#include "../Kin/kin.h"

arr pr2_zero_pose() {
  arr q = { 0.1, 0.999998, 0.500003, 0.999998, 1.5, -2, 0, 0.500003, 0, 0 };
  //{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, -1, 0.5, -1, -1.5, -2, 0, -0.5, 0, 0, 1, 0.5, 1, 1.5, -2, 0, 0.5, 0, 0 };
  return q;
}

arr pr2_reasonable_W(const rai::Configuration& world) {
#if 0
  arr W = world.naturalQmetric(5.);
  rai::Joint* j = world.getJointByName("torso_lift_joint");
  if(j) {
    CHECK_EQ(j->type, rai::JT_transX, "");
    W(j->qIndex) *= 10;
  }
  j = world.getJointByName("worldTranslationRotation");
  if(j) {
    CHECK_EQ(j->type, rai::JT_transXYPhi, "");
    W(j->qIndex+0) *= 3;
    W(j->qIndex+1) *= 3;
//    W(j->qIndex+2) *= 10;
  }
  return W;
#else
  return world.getHmetric();
#endif
}

uintA _get_shape_indices(rai::Body* b) {
  uintA idx;
  for(rai::Shape* s : b->shapes) {
    idx.append(s->index);
  }
  return idx;
}

rai::Array<const char*> pr2_left_get_bodynames() {
  return {
    "base_footprint",
    "torso_lift_link",
    "l_shoulder_pan_link",
    "l_shoulder_lift_link",
    "l_upper_arm_roll_link",
    "l_forearm_roll_link",
    "l_elbow_flex_link",
    "l_wrist_flex_link",
    "l_wrist_roll_link",
    "l_gripper_l_finger_link",
    "l_gripper_r_finger_link",
    "l_gripper_l_finger_tip_link",
    "l_gripper_r_finger_tip_link"
  };
}

rai::Array<const char*> pr2_full_get_bodynames() {
  return {
    "base_footprint",
    "fl_caster_rotation_link",
    "fl_caster_l_wheel_link",
    "fl_caster_r_wheel_link",
    "fr_caster_rotation_link",
    "fr_caster_l_wheel_link",
    "fr_caster_r_wheel_link",
    "bl_caster_rotation_link",
    "bl_caster_l_wheel_link",
    "bl_caster_r_wheel_link",
    "br_caster_rotation_link",
    "br_caster_l_wheel_link",
    "br_caster_r_wheel_link",
    "torso_lift_link",
    "head_pan_link",
    "head_tilt_link",
    "laser_tilt_mount_link",
    "r_shoulder_pan_link",
    "r_shoulder_lift_link",
    "r_upper_arm_roll_link",
    "r_forearm_roll_link",
    "r_elbow_flex_link",
    "r_wrist_flex_link",
    "r_wrist_roll_link",
    "r_gripper_l_finger_link",
    "r_gripper_r_finger_link",
    "r_gripper_l_finger_tip_link",
    "r_gripper_r_finger_tip_link",
    "l_shoulder_pan_link",
    "l_shoulder_lift_link",
    "l_upper_arm_roll_link",
    "l_forearm_roll_link",
    "l_elbow_flex_link",
    "l_wrist_flex_link",
    "l_wrist_roll_link",
    "l_gripper_l_finger_link",
    "l_gripper_r_finger_link",
    "l_gripper_l_finger_tip_link",
    "l_gripper_r_finger_tip_link"
  };

}

rai::Array<const char*> pr2_get_joints() {
  return {
    "worldTranslationRotation",
    "torso_lift_joint",
    "head_pan_joint",
    "laser_tilt_mount_joint",
    "r_shoulder_pan_joint",
    "l_shoulder_pan_joint",
    "head_tilt_joint",
    "r_shoulder_lift_joint",
    "l_shoulder_lift_joint",
    "r_upper_arm_roll_joint",
    "l_upper_arm_roll_joint",
    "r_elbow_flex_joint",
    "l_elbow_flex_joint",
    "r_forearm_roll_joint",
    "l_forearm_roll_joint",
    "r_wrist_flex_joint",
    "l_wrist_flex_joint",
    "r_wrist_roll_joint",
    "l_wrist_roll_joint",
    "r_gripper_l_finger_joint",
    "r_gripper_r_finger_joint",
    "l_gripper_l_finger_joint",
    "l_gripper_r_finger_joint",
    "r_gripper_l_finger_tip_joint",
    "r_gripper_r_finger_tip_joint",
    "l_gripper_l_finger_tip_joint",
    "l_gripper_r_finger_tip_joint",
    "r_gripper_joint",
    "l_gripper_joint"};
}

uintA pr2_get_shapes(const rai::Configuration& G) {
  rai::Array<const char*> bodynames = pr2_left_get_bodynames();
  uintA shape_idx;
  for(const char* bodyname: bodynames) {
    shape_idx.append(_get_shape_indices(G.getBodyByName(bodyname)));
  }
  return shape_idx;
}
