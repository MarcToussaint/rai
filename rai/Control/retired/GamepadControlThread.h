#pragma once

#include <Core/thread.h>
#include <Control/taskControl.h>

struct GamepadControlThread : Thread{
  struct Gamepad2Tasks *g2t;
  VAR(arr, gamepadState)
  VAR(CtrlTaskL, ctrlTasks)
  VAR(mlr::KinematicWorld, modelWorld)

  struct GamepadInterface *gamepadPoller;
  TaskControlMethods *tc;

  GamepadControlThread();
  ~GamepadControlThread();

  virtual void open();
  virtual void step();
  virtual void close();

};
