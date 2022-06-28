/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "rrt.h"
#include "ann.h"

struct sRRT {
  ANN ann;
  uintA parent;
  double stepsize;
  uint nearest;
};

RRT::RRT(const arr& q0, double _stepsize) {
  self = make_unique<sRRT>();
  self->ann   .append(q0); //append q as the root of the tree
  self->parent.append(0);    //q has itself as parent
  self->stepsize = _stepsize;
}
double RRT::getProposalTowards(arr& proposal, const arr& q) {
  //find NN
  self->nearest=self->ann.getNN(q);

  //compute little step
  arr d = q - self->ann.X[self->nearest]; //difference vector between q and nearest neighbor
  double dist = length(d);
  if(dist > self->stepsize)
    proposal = self->ann.X[self->nearest] + self->stepsize/dist * d;
  else
    proposal = q;
  return dist;
}
void RRT::add(const arr& q) {
  self->ann.append(q);
  self->parent.append(self->nearest);
}

//some access routines
double RRT::getStepsize() { return self->stepsize; }
uint RRT::getNearest() { return self->nearest; }
uint RRT::getParent(uint i) { return self->parent(i); }
uint RRT::getNumberNodes() { return self->ann.X.d0; }
arr RRT::getNode(uint i) { return self->ann.X[i]; }
void RRT::getRandomNode(arr& q) { q = self->ann.X[rnd(self->ann.X.d0)]; }
arr RRT::getRandomNode() { return self->ann.X[rnd(self->ann.X.d0)]; }
