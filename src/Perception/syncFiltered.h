#pragma once

#include <Core/thread.h>
#include <Perception/percept.h>

/// syncs percepts with modelWorld
struct SyncFiltered : Thread{
  Access_typed<PerceptL> percepts_filtered;
  ACCESSname(mlr::KinematicWorld, percWorld)

  SyncFiltered();
  ~SyncFiltered();

  virtual void open(){}
  virtual void step();
  virtual void close(){}
};
