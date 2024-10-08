/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "PathResult.h"
#include "ConfigurationProblem.h"
#include "../Optim/NLP.h"
#include "../Algo/ann.h"

/// just a data structure, no algorithms
struct RRT_SingleTree {
  ANN ann;         //ann stores all points added to the tree in ann.X
  uintA parent;    //for each point we store the index of the parent node
  rai::Array<shared_ptr<QueryResult>> queries; //for each point we store the query result

  //fields for display (GLDrawer..)
  arr disp3d;
  Mutex drawMutex;

  uint nearestID = UINT_MAX; //nearest node from the last 'getProposalToward' call!

  RRT_SingleTree(const arr& q0, const shared_ptr<QueryResult>& q0_qr);

  //core method
  double getNearest(const arr& target);
  arr getProposalTowards(const arr& target, double stepsize);

  arr getNewSample(const arr& target, double stepsize, double p_sideStep, bool& isSideStep, const uint recursionDepth);

  //trivial
  uint add(const arr& q, uint parentID, const shared_ptr<QueryResult>& _qr);

  //trivial access routines
  uint getParent(uint i) { return parent(i); }
  uint getNumberNodes() { return ann.X.d0; }
  uint getDim() { return ann.X.d1; }
  arr getNode(uint i) { return ann.X[i].copy(); }
  arr getLast() { return ann.X[ann.X.d0-1].copy(); }
  arr getRandomNode() { return ann.X[rnd(ann.X.d0)].copy(); }
  arr getPathFromNode(uint fromID);

  arr getSideStep(std::shared_ptr<QueryResult> qr);
};

//===========================================================================

///algorithms
struct RRT_PathFinder {
  ConfigurationProblem& P;
  shared_ptr<RRT_SingleTree> rrt0;
  shared_ptr<RRT_SingleTree> rrtT;

  //parameters
  double stepsize;
  int maxIters=5000;
  int verbose;
  int subsampleChecks=0;
  double p_forwardStep=.5;
  double p_sideStep=.0;
  double p_backwardStep=.0;

  //counters
  uint iters=0;
  uint n_backStep=0, n_backStepGood=0, n_sideStep=0, n_sideStepGood=0, n_forwardStep=0, n_forwardStepGood=0, n_rndStep=0, n_rndStepGood=0;

  //output
  arr path;

  RRT_PathFinder(ConfigurationProblem& _P, const arr& starts, const arr& goals, double _stepsize = -1., int _subsampleChecks=-1, int maxIters=-1, int _verbose=-1);
  ~RRT_PathFinder() {}

  int stepConnect();
  void planForward(const arr& q0, const arr& qT);
  arr planConnect(); //default numbers: equivalent to standard bidirect

  bool growTreeTowardsRandom(RRT_SingleTree& rrt);
  bool growTreeToTree(RRT_SingleTree& rrt_A, RRT_SingleTree& rrt_B);

  arr run(double timeBudget=1.); //obsolete

 private:
  rai::Configuration DISP;
};

//===========================================================================

namespace rai {

struct PathFinder : NonCopyable {
  std::shared_ptr<ConfigurationProblem> problem;
  std::shared_ptr<RRT_PathFinder> rrtSolver;
  std::shared_ptr<SolverReturn> ret;

  void setProblem(const rai::Configuration& C, const arr& starts, const arr& goals, double collisionTolerance=-1.);

  void setExplicitCollisionPairs(const StringA& collisionPairs);

  shared_ptr<SolverReturn> solve();

  arr get_resampledPath(uint T);
};

} //namespace

//===========================================================================

void revertPath(arr& path);
