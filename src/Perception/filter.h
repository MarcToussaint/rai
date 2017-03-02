#pragma once

#include <Core/thread.h>
#include <Core/array.h>

#include <Algo/hungarian.h>
#include <Perception/percept.h>

/// clears the perceptual inputs (which is a FIFO) and merges these into the filtered percepts
struct Filter : Thread{
  Access_typed<Percepts> percepts_input;
  Access_typed<Percepts> percepts_filtered;

  Filter();

  virtual void open(){}
  virtual void step();
  virtual void close(){}

private:
  double relevance_decay_factor = 0.99;
  double relevance_threshold = 0.25;
  double distance_threshold = 0.5;

  uint maxId = 0;

  arr costs;

  arr createCostMatrix(const Percepts& perceptualInputs, const Percepts& objectDatabase);
  Percepts assign(const Percepts& perceps, const Percepts& database, const Hungarian& ha);

  int revision = -1;
};

