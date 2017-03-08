#pragma once

#include <Core/thread.h>
#include <Perception/percept.h>

/// syncs percepts with modelWorld
struct SyncFiltered : Thread{
  Access_typed<PerceptL> percepts_filtered;
  Access_typed<mlr::KinematicWorld> outputWorld;
  ACCESS(mlr::KinematicWorld, modelWorld)

  SyncFiltered(const char* outputWorld_name);
  ~SyncFiltered();

  virtual void open();
  virtual void step();
  virtual void close(){}
};
