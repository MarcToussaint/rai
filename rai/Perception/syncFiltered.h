/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/thread.h>
#include <Perception/percept.h>

/// syncs percepts with modelWorld
struct SyncFiltered : Thread{
  Var<PerceptL> percepts_filtered;
  Var<mlr::KinematicWorld> outputWorld;
  VAR(mlr::KinematicWorld, modelWorld)

  SyncFiltered(const char* outputWorld_name);
  ~SyncFiltered();

  virtual void open();
  virtual void step();
  virtual void close(){}
};
