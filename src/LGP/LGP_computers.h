/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Search/ComputeNode.h"
#include "../Search/AStar.h"
#include "../KOMO/komo.h"
#include "../KOMO/skeleton.h"
#include "../KOMO/pathTools.h"
#include "../Logic/folWorld.h"
#include "../PathAlgos/ConfigurationProblem.h"
#include "../PathAlgos/RRT_PathFinder.h"
#include "../Optim/NLP_GraphSolver.h"

namespace rai {

//===========================================================================

struct LGP_GlobalInfo {
  RAI_PARAM("LGP/", int, verbose, 1)
  RAI_PARAM("LGP/", double, skeleton_w0, 1.)
  RAI_PARAM("LGP/", double, skeleton_wP, 2.)
  RAI_PARAM("LGP/", double, waypoint_w0, 10.)
  RAI_PARAM("LGP/", double, waypoint_wP, 2.)
  RAI_PARAM("LGP/", int, waypointStopEvals, 1000)
  RAI_PARAM("LGP/", int, rrtStopEvals, 10000)
  RAI_PARAM("LGP/", double, rrtStepsize, .05)
  RAI_PARAM("LGP/", double, rrtTolerance, .03)
  RAI_PARAM("LGP/", double, pathCtrlCosts, 1.)
  RAI_PARAM("LGP/", int, pathStepsPerPhase, 30)
  RAI_PARAM("LGP/", double, collScale, 1e1)
  RAI_PARAM("LGP/", bool, useSequentialWaypointSolver, false)
};

//===========================================================================

struct LGPComp_root : ComputeNode {
  FOL_World& L;
  Configuration& C;
  bool useBroadCollisions=false;
  StringA explicitCollisions;
  StringA explicitLift;
  String explicitTerminalSkeleton;
  std::shared_ptr<rai::AStar> fol_astar;
  std::shared_ptr<LGP_GlobalInfo> info;
  bool fixedSkeleton=false;

  LGPComp_root(FOL_World& _L, Configuration& _C, bool useBroadCollisions, const StringA& explicitCollisions, const StringA& explicitLift, const String& explicitTerminalSkeleton);

  virtual void untimedCompute() {}
  virtual int getNumDecisions() { return -1.; }
//    virtual double effortHeuristic(){ return 11.+10.; }
  virtual double branchingPenalty_child(int i);

  virtual std::shared_ptr<ComputeNode> createNewChild(int i);
};

//===========================================================================

struct LGPcomp_Skeleton : ComputeNode {
  LGPComp_root* root=0;
  int num=0;
  Skeleton skeleton;
  //shared_ptr<SkeletonSolver> sol;
  rai::String planString;
  Array<Graph*> states;
  arr times;

  LGPcomp_Skeleton(LGPComp_root* _root, int num);
  LGPcomp_Skeleton(LGPComp_root* _root, const Skeleton& _skeleton);

  void createNLPs(const rai::Configuration& C);

  virtual void untimedCompute();

  virtual int getNumDecisions() { return -1.; }
//    virtual double branchingHeuristic(){ return root->info->waypoint_w0; }
//    virtual double effortHeuristic(){ return 10.+10.; }
  virtual double branchingPenalty_child(int i);

  virtual std::shared_ptr<ComputeNode> createNewChild(int i);

  int verbose() { return root->info->verbose; }
};

//===========================================================================

struct FactorBoundsComputer : ComputeNode {
  LGPcomp_Skeleton* sket;
  int seed=0;
  KOMO komoWaypoints;
  std::shared_ptr<NLP_Factored> nlp;
  uint t=0;

  FactorBoundsComputer(LGPcomp_Skeleton* _sket, int rndSeed);

  virtual void untimedCompute();
//    virtual double effortHeuristic(){ return 10.+1.*(komoWaypoints.T); }
  virtual int getNumDecisions() { return 1; }
  virtual std::shared_ptr<ComputeNode> createNewChild(int i);
};

//===========================================================================

struct PoseBoundsComputer : ComputeNode {
  LGPcomp_Skeleton* sket;
  int seed=0;
  uint t=0;

  PoseBoundsComputer(LGPcomp_Skeleton* _sket, int rndSeed);

  virtual void untimedCompute();
//    virtual double effortHeuristic(){ return 10.+1.*(sket->states.N); }
  virtual int getNumDecisions() { return 1; }
  virtual std::shared_ptr<ComputeNode> createNewChild(int i);
};

//===========================================================================

struct LGPcomp_Waypoints : ComputeNode {
  LGPcomp_Skeleton* sket;
  int seed=0;
  std::shared_ptr<KOMO> komoWaypoints;
  NLP_Solver sol;
  NLP_GraphSolver gsol;

  LGPcomp_Waypoints(LGPcomp_Skeleton* _sket, int rndSeed);

  virtual void untimedCompute();
//    virtual double effortHeuristic(){ return 10.+1.*(komoWaypoints->T); }
  virtual int getNumDecisions();
  virtual std::shared_ptr<ComputeNode> createNewChild(int i);
};

//===========================================================================

struct LGPcomp_RRTpath : ComputeNode {
  LGPcomp_Skeleton* sket=0;
  LGPcomp_Waypoints* ways=0;
  LGPcomp_RRTpath* prev=0;

  shared_ptr<Configuration> C;
  uint t;
  shared_ptr<RRT_PathFinder> rrt;
  arr q0, qT;
  arr path;

  LGPcomp_RRTpath(ComputeNode* _par, LGPcomp_Waypoints* _ways, uint _t);

  virtual void untimedCompute();
//    virtual double effortHeuristic(){ return 10.+1.*(ways->komoWaypoints->T-t-1); }

  virtual int getNumDecisions() { return 1; }
  virtual std::shared_ptr<ComputeNode> createNewChild(int i);
};

//===========================================================================

struct LGPcomp_OptimizePath : ComputeNode {
  LGPcomp_Skeleton* sket=0;
  LGPcomp_Waypoints* ways=0;

  shared_ptr<KOMO> komoPath;
  NLP_Solver sol;

  LGPcomp_OptimizePath(LGPcomp_Skeleton* _sket); //compute path from skeleton directly, without waypoints first
  LGPcomp_OptimizePath(LGPcomp_Waypoints* _ways); //compute path initialized from waypoints
  LGPcomp_OptimizePath(LGPcomp_RRTpath* _par, LGPcomp_Waypoints* _ways); //compute path initialized from series of RRT solutions

  virtual void untimedCompute();
//    virtual double effortHeuristic(){ return 0.; }

  virtual double sample() {
    CHECK(sol.ret, "");
    CHECK(sol.ret->done, "");
    if(sol.ret->ineq>1. || sol.ret->eq>4.) return 1e10;
    return sol.ret->ineq+sol.ret->eq;
  }

//        if(opt.verbose>0) LOG(0) <<*ret;
//        if(opt.verbose>1) cout <<komoPath.report(false, true, opt.verbose>2);
//        if(opt.verbose>0) komoPath.view(opt.verbose>1, STRING("optimized path\n" <<*ret));
//        //komoPath.checkGradients();
//        if(opt.verbose>1) komoPath.view_play(opt.verbose>2);

  virtual int getNumDecisions() { return 0; }
  virtual std::shared_ptr<ComputeNode> createNewChild(int i) { HALT("is terminal"); }
};

//===========================================================================

}//namespace
