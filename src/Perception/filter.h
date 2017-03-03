#pragma once

#include <Core/thread.h>
#include <Core/array.h>

#include <Algo/hungarian.h>
#include <Perception/percept.h>

/// clears the perceptual inputs (which is a FIFO) and merges these into the filtered percepts
struct Filter : Thread{
  Access_typed<PerceptL> percepts_input;
  Access_typed<PerceptL> percepts_filtered;

  Filter();
  ~Filter();

  virtual void open(){}
  virtual void step();
  virtual void close(){}

  int verbose = 2;
private:
  double relevance_decay_factor = 0.99;
  double relevance_threshold = 0.25;
  double distance_threshold = 0.5;

  uint maxId = 0;

  arr costs;

  arr createCostMatrix(const PerceptL& perceptualInputs, const PerceptL& objectDatabase);
  PerceptL assign(const PerceptL& perceps, const PerceptL& database, const Hungarian& ha);

  int revision = -1;
};

