/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include <Control/taskControl.h>

struct Teleop2Tasks {
  //data of previous loop cycle
  //position
  arr old_pos_rh;
  arr old_pos_lh;
  arr old_effpos_r;
  arr old_effpos_l;

  //bools to deactivate movement on certain axis
  bool move_x = true;
  bool move_y = true;
  bool move_z = true;
  bool rotate = false;

  bool move_lh_x = true;
  bool move_lh_y = true;
  bool move_lh_z = true;

  //used to get only one button press out of one button press
  int old_button = 0;

  bool initialised = false;
  TaskControlMethods& fmc;
  CtrlTask* effPosR, *gripperR, *effOrientationR;
  CtrlTask* effPosL, *gripperL, *effOrientationL;
  CtrlTask* base, *fc;
  Teleop2Tasks(TaskControlMethods& _MP, const rai::Configuration& K);
  CtrlTaskL tasks;
  CtrlTaskL getTasks();
  void updateMovement(floatA& cal_pose, arr& old_pos, arr& old_effPos, CtrlTask* effPos);
  void deactivateTasks();
  void updateTasks(floatA cal_pose_rh, floatA cal_pose_lh, float calibrated_gripper_lh, float calibrated_gripper_rh, arr drive, int button, const rai::Configuration& K);
};

