/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.ipp"

struct RRT {
 private:
  unique_ptr<struct sRRT> self;

 public:
  RRT(const arr& q0, double _stepsize);
  double getProposalTowards(arr& proposal, const arr& q);
  void add(const arr& q);

  //some access routines
  double getStepsize();
  uint getNearest();
  uint getParent(uint i);
  uint getNumberNodes();
  arr getNode(uint i);
  void getRandomNode(arr& q);
  arr getRandomNode();
};
