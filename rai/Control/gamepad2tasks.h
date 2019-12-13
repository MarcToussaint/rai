/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include <Control/taskControl.h>

struct Gamepad2Tasks {
  TaskControlMethods& TC;
  const arr q0;
  rai::String robot;
  CtrlTask* homing, *endeffR, *endeffL, *base, *torso, *head, *headAxes, *limits, *coll,  *gripperL, *gripperR;

  Gamepad2Tasks(TaskControlMethods& _TC, const rai::Configuration& K, const arr& q0);
  rai::Array<CtrlTask*> getTasks();
  bool updateTasks(arr& gamepadState, const rai::Configuration& K);
};

