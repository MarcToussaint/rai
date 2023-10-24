/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "GamepadControlThread.h"
#include "TaskControlThread.h"
#include "gamepad2tasks.h"
#include <Hardware/gamepad/gamepad.h>

//===========================================================================

GamepadControlThread::GamepadControlThread()
  : Thread("Act_GamepadControl", 0.01),
    g2t(nullptr),
    gamepadPoller(nullptr),
    tc(nullptr) {
  threadLoop();
}

GamepadControlThread::~GamepadControlThread() {
  threadClose();
}

void GamepadControlThread::open() {
  gamepadPoller = new GamepadInterface;
  gamepadPoller->threadLoop();
}

void GamepadControlThread::step() {
  if(!g2t) {
    TaskControlThread* taskController = getThread<TaskControlThread>("TaskControlThread");
    CHECK(taskController, "that didn't work");
    taskController->waitForStatusSmallerThan(tsToOpen);
    tc = taskController->taskController;
    if(!tc) return;
    g2t = new Gamepad2Tasks(*tc, modelWorld.get(), taskController->q0);

    ctrlTasks.writeAccess();
//    taskController->taskController->qNullCostRef.active = false;
    for(CtrlObjective* t:ctrlTasks()) t->active = false;
    ctrlTasks().append(g2t->getTasks());
    ctrlTasks.deAccess();
  }

  arr gamepad = gamepadState.get();
  ctrlTasks.writeAccess();
  g2t->updateTasks(gamepad, modelWorld.get());
  ctrlTasks.deAccess();

  if(step_count>10 && stopButtons(gamepad)) threadClose();

//  if(step_count>10 && gamepad_shutdown) moduleShutdown()->incrementValue();
}

void GamepadControlThread::close() {
  gamepadPoller->threadClose();
  delete g2t; g2t=nullptr;
  delete gamepadPoller; gamepadPoller=nullptr;
}

//===========================================================================

