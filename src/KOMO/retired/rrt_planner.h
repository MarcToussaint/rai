/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef _HEADER_GUARD_RRT_PLANNER_H_
#define _HEADER_GUARD_RRT_PLANNER_H_

#include "../Core/array.h"

struct KOMO;
struct OpenGL;

namespace rai {
struct Configuration;
struct RRTPlanner {
 private:
  unique_ptr<struct sRRTPlanner> self;
 public:
  Configuration* G;                 ///< the graph to plan in
  KOMO& problem;   ///< the KOMO gives the feasibility test for new states

  arr joint_max, joint_min; ///< in which range are the joints allowed (boundaries for the sample space)

  RRTPlanner(rai::Configuration* G, KOMO& problem, double stepsize, bool verbose = false);

  arr getTrajectoryTo(const arr& target, int max_iter=0); ///< returns the trajectory created by the RRT
};
}

#endif // _HEADER_GUARD_RRT_PLANNER_H_

