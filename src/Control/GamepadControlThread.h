#pragma once

#include <Core/thread.h>
#include <Control/taskControl.h>

struct GamepadControlThread : Thread{
  struct Gamepad2Tasks *g2t;
  ACCESS(arr, gamepadState)
  ACCESS(CtrlTaskL, ctrlTasks)
  ACCESS(mlr::KinematicWorld, modelWorld)

  struct GamepadInterface *gamepadPoller;
  TaskControlMethods *tc;

  GamepadControlThread();
  ~GamepadControlThread();

  virtual void open();
  virtual void step();
  virtual void close();

};
