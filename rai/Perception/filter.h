#pragma once

#include <Core/thread.h>
#include <Core/array.h>

#include <Algo/hungarian.h>
#include "percept.h"

/// clears the perceptual inputs (which is a FIFO) and merges these into the filtered percepts
struct Filter : Thread{
  Access<PerceptL> percepts_input;
  Access<PerceptL> percepts_filtered;
  Access<mlr::KinematicWorld> modelWorld;

  Filter();
  ~Filter();

  virtual void open();
  virtual void step();
  virtual void close(){}

  int verbose = 0;
  bool createNewPercepts = false;

private:
  double relevance_decay_factor = 0.99;
  double precision_transition = 20.;
  double precision_threshold = 0.25;
  double distance_threshold = 0.5;

  uint nextId = 1;

  arr costs;

  arr createCostMatrix(const PerceptL& inputs, const PerceptL& database);
  PerceptL assign(const PerceptL& inputs, const PerceptL& database, const Hungarian& ha);

  int revision = -1;
};

