/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "control.h"

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
  CtrlObjective* effPosR, *gripperR, *effOrientationR;
  CtrlObjective* effPosL, *gripperL, *effOrientationL;
  CtrlObjective* base, *fc;
  Teleop2Tasks(TaskControlMethods& _MP, const rai::Configuration& K);
  CtrlObjectiveL tasks;
  CtrlObjectiveL getTasks();
  void updateMovement(floatA& cal_pose, arr& old_pos, arr& old_effPos, CtrlObjective* effPos);
  void deactivateTasks();
  void updateTasks(floatA cal_pose_rh, floatA cal_pose_lh, float calibrated_gripper_lh, float calibrated_gripper_rh, arr drive, int button, const rai::Configuration& K);
};

