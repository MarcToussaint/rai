/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "komo.h"
#include "../PathAlgos/RRT_PathFinder.h"

//===========================================================================

struct ManipulationHelper {
  std::shared_ptr<KOMO> komo;
  str info;

  //solver buffers:
  std::shared_ptr<SolverReturn> ret;
  arr qTarget;
  arr path;

  ManipulationHelper(const str& _info={});
  ManipulationHelper(const std::shared_ptr<KOMO>& _komo, const str& _info = str{});

  KOMO& k() { return *komo; }

  void setup_inverse_kinematics(rai::Configuration& C, double homing_scale=1e-1, bool accumulated_collisions=true, bool joint_limits=true, bool quaternion_norms=true);
  void setup_sequence(rai::Configuration& C, uint K, double homing_scale=1e-2, double velocity_scale=1e-1, bool accumulated_collisions=true, bool joint_limits=true, bool quaternion_norms=true);
  void setup_motion(rai::Configuration& C, uint K, uint steps_per_phase, double homing_scale=0., double acceleration_scale=1e-1, bool accumulated_collisions=true, bool joint_limits=true, bool quaternion_norms=true);
  void setup_pick_and_place_waypoints(rai::Configuration& C, const char* gripper, const char* obj, double homing_scale=1e-2, double velocity_scale=1e-1, bool accumulated_collisions=true, bool joint_limits=true, bool quaternion_norms=true);
  void setup_point_to_point_motion(rai::Configuration& C, const arr& q1, uint steps_per_phase, double homing_scale=1e-2, double acceleration_scale=1e-1, bool accumulated_collisions=true, bool joint_limits=true, bool quaternion_norms=true);
  void setup_point_to_point_rrt(rai::Configuration& C, const arr& q0, const arr& q1, const StringA& explicitCollisionPairs);

  void add_stable_frame(rai::JointType type, const char* parent, const char* name, const char* initName=0, rai::Frame* initFrame=0, double markerSize=-1.);

  void grasp_top_box(double time, const char* gripper, const char* obj, str grasp_direction="xz");
  void grasp_box(double time, const char* gripper, const char* obj, const char* palm, str grasp_direction="x", double margin=.02);
  void grasp_cylinder(double time, const char* gripper, const char* obj, const char* palm, double margin=.02);
  void place_box(double time, const char* obj, const char* table, const char* palm, str place_direction="z", double margin=.02);

  void straight_push(arr times, str obj, str gripper, str table);

  void no_collisions(const arr& time_interval, const StringA& pairs, double margin=.001, double scale=1e1);
  void freeze_joint(const arr& time_interval, const StringA& joints);
  void freeze_relativePose(const arr& time_interval, str to, str from);

  void snap_switch(double time, str parent, str obj);

  void target_position();
  void target_relative_xy_position(double time, const char* obj, const char* relativeTo, arr pos);
  void target_x_orientation(double time, const char* obj, const arr& x_vector);

  void bias(double time, arr& qBias, double scale=1e0);
  void retract(const arr& time_interval, const char* gripper, double dist=.05);
  void approach(const arr& time_interval, const char* gripper, double dist=.05);
  void retractPush(const arr& time_interval, const char* gripper, double dist=.02);
  void approachPush(const arr& time_interval, const char* gripper, double dist=.05, str helper= {});

  std::shared_ptr<SolverReturn> solve(int verbose=1);
  arr sample(const char* sampleMethod=0, int verbose=1);
  void debug(bool listObjectives=true, bool plotOverTime=false);

  std::shared_ptr<ManipulationHelper> sub_motion(uint phase, uint steps_per_phase=50, bool fixEnd=true, double homing_scale=1e-2, double acceleration_scale=1e-1, bool accumulated_collisions=true, bool joint_limits=true, bool quaternion_norms=true, const StringA& activeDofs={});
  std::shared_ptr<rai::RRT_PathFinder> sub_rrt(uint phase, const StringA& explicitCollisionPairs= {}, const StringA& activeDofs={});

  void play(rai::Configuration& C, double duration=1.);

};
