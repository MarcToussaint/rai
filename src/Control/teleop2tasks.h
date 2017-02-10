/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */
#include <Control/taskController.h>

struct Teleop2Tasks{
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
  TaskController& fmc;
  CtrlTask *effPosR, *gripperR, *effOrientationR;
  CtrlTask *effPosL, *gripperL, *effOrientationL;
  CtrlTask *base, *fc;
  Teleop2Tasks(TaskController& _MP, const mlr::KinematicWorld& K);
  mlr::Array<CtrlTask*> getTasks();
  void updateMovement(floatA& cal_pose, arr& old_pos, arr& old_effPos, CtrlTask *effPos);
  void deactivateTasks();
  void updateTasks( floatA cal_pose_rh, floatA cal_pose_lh, float calibrated_gripper_lh, float calibrated_gripper_rh, arr drive, int button, const mlr::KinematicWorld& K);
};

