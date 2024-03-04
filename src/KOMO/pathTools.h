/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"
#include "../Kin/kin.h"
#include "../Algo/spline.h"

//-- PATH ANALYSIS

//central finite differencing to get velocities/acc (in same time resolution)
arr getVelocities_centralDifference(const arr& q, double tau);
arr getAccelerations_centralDifference(const arr& q, double tau);

//KOMO-convention (backward) finite differencing
arr getVel(const arr& x, const arr& tau);
arr getAcc(const arr& x, const arr& tau);
arr getJerk(const arr& x, const arr& tau);

double getMinDuration(const arr& q, double maxVel=1., double maxAcc=1.);

rai::BSpline getSpline(const arr& q, double duration=1., uint degree=2);

//test max velocities, collision avoidance, consistence with q_now
rai::String validatePath(const rai::Configuration& _C, const arr& q_now, const StringA& joints, const arr& q, const arr& times);

//-- PATH GENERATION

//generate a sine motion profile from q0 to qT in T steps
arr getSineProfile(const arr& q0, const arr& qT, uint T);

//call KOMO to compute a collision free start-goal path
std::pair<arr, arr> getStartGoalPath_obsolete(const rai::Configuration& K, const arr& target_q, const StringA& target_joints= {}, const char* endeff=nullptr, double up=.2, double down=.8);

struct Avoid {
  arr times;
  StringA frames;
  double dist;
};

//C is start configuration, qTarget is goal
//if endeffs are given, the goal is ONLY the endeff poses, not q
arr getStartGoalPath(rai::Configuration& C, const arr& qTarget, const arr& qHome,
                     const rai::Array<Avoid>& avoids= {},
                     StringA endeffectors= {}, bool endeffApproach=false, bool endeffRetract=false);
arr getStartGoalPath_new(rai::Configuration& C, const arr& qTarget, const arr& qHome,
                         const rai::Array<Avoid>& avoids= {},
                         StringA endeffectors= {}, bool endeffApproach=false, bool endeffRetract=false);

//-- PATH MODIFICATION

//return the same path backward
arr reversePath(const arr& q);

//append a reverse version of path to itself, including times
void mirrorDuplicate(std::pair<arr, arr>& path);

//convert to spline, then resample to new length
arr path_resample(const arr& q, double durationScale);

arr path_resampleLinear(const arr& q, uint T);

void makeMod2Pi(const arr& q0, arr& q1);

//-- POSE

struct PoseTool {
  rai::Configuration& C;
  int verbose;

  PoseTool(rai::Configuration& _C, int _verbose=0) : C(_C), verbose(_verbose) {}
  bool checkLimits(const arr& limits= {}, bool solve=false, bool assert=false); //default: use limites defined in C
  bool checkCollisions(const FrameL& collisionPairs= {}, bool solve=false, bool assert=false); //default: use broadphase method in C
  bool checkLimitsAndCollisions(const arr& limits= {}, const FrameL& collisionPairs= {}, bool solve=false, bool assert=false);
};

//bool checkCollisionsAndLimits(rai::Configuration& C, const FrameL& collisionPairs, const arr& limits, bool solveForFeasible, int verbose=1);
