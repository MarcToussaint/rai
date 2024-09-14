/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "LGP_computers.h"

#include "../Logic/folWorld.h"
#include "../Kin/kin.h"

namespace rai {

struct LGP_DomainInfo {
  RAI_PARAM("LGP/", int, verbose, 1)
  RAI_PARAM("LGP/", double, waypointBranching, 10.)
  RAI_PARAM("LGP/", int, waypointStopEvals, 1000)
  RAI_PARAM("LGP/", int, rrtStopEvals, 10000)
  RAI_PARAM("LGP/", double, pathCtrlCosts, 1.)
  RAI_PARAM("LGP/", double, collScale, 1e1)
  RAI_PARAM("LGP/", bool, useSequentialWaypointSolver, false)
};

struct LGP_SkeletonTool {
//  rai::Configuration C;
  std::shared_ptr<rai::FOL_World> L;
  std::shared_ptr<rai::LGPComp_root> lgproot;
  FOL_World_State* focusNode=0;

  LGP_SkeletonTool(rai::Configuration& C, const char* lgpFile);

  LGP_SkeletonTool(rai::Configuration& C, rai::FOL_World& L, bool genericCollisions, const StringA& explicitCollisions, const StringA& explicitLift, const String& explicitTerminalSkeleton);
  ~LGP_SkeletonTool();

  //view and edit the configuration associated to this LGP problem
  void viewConfig();

  //generate (and output) one more action plan (call multiple times to get top action plans)
  FOL_World_State* step_folPlan();

  //build (and display) the full ful decision tree up to given depth
  void buildTree(uint depth);

  //perform a sequence of computations through the completion tree, where the intA gives the branching coordinates
  // e.g. [1 2 0 0 0] takes the 2nd action plan (=skeleton), computes waypoints with seed=2, then performs three RRTs or two and final KOMO path
  void compute(const intA& branches);

  //calls CompletionTree search from the given root, which can be lgproot, or a fixed skeleton
  void solve(const std::shared_ptr<TreeSearchNode>& root);
  void solve_Skeleton(const rai::Skeleton& skeleton) {  solve(make_shared<rai::LGPcomp_Skeleton>(lgproot.get(), skeleton));  }
  void solve_LGP() {  solve(lgproot);  }
  void solve_Decisions(const String& seq);

  //terminal gui debugger - walk through logic tree and call optimizations
  void player();

  void report(ostream& os) const;

 private:
  void walkToNode(const String& seq);
  void displayTreeUsingDot();
  void writeNodeList(ostream& os=cout);
  void getSkeleton(Skeleton& skeleton, String& skeletonString);
  void expand(FOL_World_State* node);
  void optWaypoints(Skeleton& skeleton, const String& skeletonString);
  void optPath(Skeleton& skeleton, const String& skeletonString);
  void optFinalSlice(Skeleton& skeleton, const String& skeletonString);
  void optWaypoints() { Skeleton skeleton; String skeletonString; getSkeleton(skeleton, skeletonString); optWaypoints(skeleton, skeletonString); }
  void optFinalSlice() { Skeleton skeleton; String skeletonString; getSkeleton(skeleton, skeletonString); optFinalSlice(skeleton, skeletonString); }
  void optPath() { Skeleton skeleton; String skeletonString; getSkeleton(skeleton, skeletonString); optPath(skeleton, skeletonString); }
};

} //namespace
