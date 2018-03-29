/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/thread.h>
#include <Control/taskControl.h>

struct GamepadControlThread : Thread{
  struct Gamepad2Tasks *g2t;
  VAR(arr, gamepadState)
  VAR(CtrlTaskL, ctrlTasks)
  VAR(rai::KinematicWorld, modelWorld)

  struct GamepadInterface *gamepadPoller;
  TaskControlMethods *tc;

  GamepadControlThread();
  ~GamepadControlThread();

  virtual void open();
  virtual void step();
  virtual void close();

};
