/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "taskControl.h"
#include "../Core/thread.h"

struct GamepadControlThread : Thread {
  struct Gamepad2Tasks* g2t;
  VAR(arr, gamepadState)
  VAR(CtrlObjectiveL, ctrlTasks)
  VAR(rai::Configuration, modelWorld)

  struct GamepadInterface* gamepadPoller;
  TaskControlMethods* tc;

  GamepadControlThread();
  ~GamepadControlThread();

  virtual void open();
  virtual void step();
  virtual void close();

};
