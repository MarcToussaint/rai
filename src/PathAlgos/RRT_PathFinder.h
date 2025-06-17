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

namespace rai {

//===========================================================================

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
  arr getNewSample(const arr& target, double stepsize);

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
};

//===========================================================================

struct RRT_PathFinder_Options {
  RAI_PARAM("rrt/", int, verbose, 0)
  RAI_PARAM("rrt/", double, stepsize, .1)
  RAI_PARAM("rrt/", int, subsamples, 4)
  RAI_PARAM("rrt/", int, maxIters, 5000)
  RAI_PARAM("rrt/", double, p_connect, .5)
  RAI_PARAM("rrt/", double, collisionTolerance, 1e-4)
  RAI_PARAM("rrt/", bool, useBroadCollisions, true)
};

///algorithms
struct RRT_PathFinder : NonCopyable {
  RRT_PathFinder_Options opt;

  shared_ptr<ConfigurationProblem> P;
  shared_ptr<RRT_SingleTree> rrt0;
  shared_ptr<RRT_SingleTree> rrtT;
  shared_ptr<SolverReturn> ret;

  //counters
  uint iters=0;
  // uint n_backStep=0, n_backStepGood=0, n_sideStep=0, n_sideStepGood=0, n_forwardStep=0, n_forwardStepGood=0, n_rndStep=0, n_rndStepGood=0;

  //output
  arr path;

  //setup
  void setProblem(shared_ptr<Configuration> C);
  void setStartGoal(const arr& _starts, const arr& _goals);
  void setExplicitCollisionPairs(const StringA& collisionPairs);

  //solve
  shared_ptr<SolverReturn> solve();

  //output
  void view(bool pause, const char* txt=0, bool play=false);
  void report();
  arr get_resampledPath(uint T);

  //low-level
  int stepConnect();

public:

private:
  void planForward(const arr& q0, const arr& qT);
  bool growTreeTowardsRandom(RRT_SingleTree& rrt);
  bool growTreeToTree(RRT_SingleTree& rrt_A, RRT_SingleTree& rrt_B);
  rai::Configuration DISP;
   void ensure_DISP();
};

//===========================================================================

void revertPath(arr& path);

} //namespace
