/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "../Core/array.h"
#include "../Kin/kin.h"
#include "../Algo/spline.h"

//-- PATH ANALYSIS

//central finite differencing to get velocities/acc (in same time resolution)
arr getVelocities_centralDifference(const arr& q, double tau);
arr getAccelerations_centralDifference(const arr& q, double tau);

double getNaturalDuration(const arr& q, double maxVel=1., double maxAcc=1.);

rai::Spline getSpline(const arr& q, double duration=1., uint degree=2);

//test max velocities, collision avoidance, consistence with q_now
rai::String validatePath(const rai::Configuration& _C, const arr& q_now, const StringA& joints, const arr& q, const arr& times);

//-- PATH GENERATION

//generate a sine motion profile from q0 to qT in T steps
arr getSineProfile(const arr& q0, const arr& qT, uint T);

//call KOMO to compute a collision free start-goal path
std::pair<arr, arr> getStartGoalPath(const rai::Configuration& K, const arr& target_q, const StringA& target_joints= {}, const char* endeff=nullptr, double up=.2, double down=.8);

//-- PATH MODIFICATION

//return the same path backward
arr reversePath(const arr& q);

//append a reverse version of path to itself, including times
void mirrorDuplicate(std::pair<arr, arr>& path);

//convert to spline, then resample to new length
arr path_resample(const arr& q, double durationScale);

