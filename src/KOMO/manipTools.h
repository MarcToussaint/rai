/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "komo.h"

//===========================================================================

void addBoxPickObjectives(KOMO& komo, double time, rai::ArgWord dir,
                          const char* boxName, const arr& boxSize,
                          const char* gripperName, const char* palmName, const char* tableName, bool pre=false);

void addBoxPlaceObjectives(KOMO& komo, double time,
                           rai::ArgWord dir, const char* boxName, const arr& boxSize, const char* tableName,
                           const char* gripperName, const char* palmName, double margin=-.02, bool pre=false);


struct ManipulationModelling{
  rai::Configuration* C;
  str info;
  StringA helpers;
  std::shared_ptr<KOMO> komo;
  std::shared_ptr<SolverReturn> ret;
  arr path;

  ManipulationModelling(rai::Configuration& _C, const str& _info={}, const StringA& helpers={});

  void setup_inverse_kinematics(double homing_scale=1e-1, bool accumulated_collisions=true, bool quaternion_norms=false);

  void setup_pick_and_place_waypoints(const char* gripper, const char* obj, double homing_scale=1e-2, double velocity_scale=1e-1, bool accumulated_collisions=true, bool joint_limits=true, bool quaternion_norms=false);

  void setup_point_to_point_motion(const arr& q0, const arr& q1, double homing_scale=1e-2, double acceleration_scale=1e-1, bool accumulated_collisions=true, bool quaternion_norms=false);

  void grasp_top_box(double time, const char* gripper, const char* obj, str grasp_direction="xz");

  void grasp_box(double time, const char* gripper, const char* obj, const char* palm, str grasp_direction="x", double margin=.02);

  void grasp_cylinder(double time, const char* gripper, const char* obj, const char* palm, double margin=.02);

  void place_box(double time, const char* obj, const char* table, const char* palm, str place_direction="z", double margin=.02);

  void no_collision(const arr& time_interval, const char* obj1, const char* obj2, double margin=.001);

  void switch_pick();
  void switch_place();
  void target_position();
  void target_relative_xy_position(double time, const char* obj, const char* relativeTo, arr pos);
  void target_x_orientation(double time, const char* obj, const arr& x_vector);

  void bias(double time, arr& qBias, double scale=1e0);

  void endeff_forward_motion(const arr& time_interval, const char* gripper, arr& q);

  void retract(const arr& time_interval, const char* gripper, double dist=.05);

  void approach(const arr& time_interval, const char* gripper, double dist=.05);

  arr solve(int verbose=1);

  void play(){
    for(uint t=0;t<path.d0;t++){
      C->setJointState(path[t]);
      C->view(false, STRING("step " <<t <<"\n" <<info));
      rai::wait(1./path.d0);
    }
  }




};
