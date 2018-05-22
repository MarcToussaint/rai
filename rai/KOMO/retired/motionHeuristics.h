/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef RAI_MotionPlanner_h
#define RAI_MotionPlanner_h

#include "motion.h"

void threeStepGraspHeuristic(arr& q, KOMO& M, uint shapeId, uint verbose);
void setGraspGoals_Schunk(KOMO& M, uint T, uint shapeId, uint side, uint phase);
void setGraspGoals_PR2(KOMO& M, uint T, uint shapeId, uint side, uint phase);
void setPlaceGoals(KOMO& M, uint T, uint shapeId, int belowToShapeId, const arr& locationTo);
void setHomingGoals(KOMO& M, uint T);

double keyframeOptimizer(arr& x, KOMO& M, bool x_is_initialized, uint verbose);
void interpolate_trajectory(arr &q, const arr& q0, const arr& qT, uint T);
inline arr interpolate_trajectory(const arr& q0, const arr& qT, uint T) {
  arr q; interpolate_trajectory(q, q0, qT, T); return q;
}

#endif
