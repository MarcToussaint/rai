/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */
#ifndef ADAPTIVEMOTIONEXECUTION_H
#define ADAPTIVEMOTIONEXECUTION_H

#include <Core/util.h>
#include <Core/array.h>
#include <Core/array.tpp>
#include <Kin/kin.h>
#include <Core/util.h>
#include <stdlib.h>
#include "mobject.h"
#include <Algo/spline.h>
#include <Gui/opengl.h>

struct AdaptiveMotionExecution{

    AdaptiveMotionExecution(mlr::KinematicWorld &_world, arr& _trajRef, double _dt, double _TRef, arr &_x0, arr &_q0, MObject &_goalMO, \
        bool _useOrientation);
    void printState();
    void plotState();
    void warpTrajectory();
    void iterate(arr &state, double _dtReal=0.);
    void getNextState(arr &state, arr &dstate);

    void moveGoal(arr &_pos);

    void computeIK(arr &q, arr &qd);
    mlr::KinematicWorld *world;

    double dt;
    double TRef;
    double dsRef;

    bool useOrientation;

    MObject *goalMO;

    // Actual Trajectory
    arr traj;
    arr x0; // start pos
    arr q0;
    arr s;
    arr goal;
    arr lastGoal;
    arr state;

    arr desState;
    arr desVel;

    // Costs
    arr posCosts;
    arr vecCosts;
    arr colCosts;

    // Wrapped Trajectory
    mlr::Path *trajWrap;

    // Reference Trajectory
    mlr::Path *trajRef;
    arr dtrajRef;
    arr goalRef;
    arr sRef;

    String scene;
    arr joints_bk;
};

#endif // ADAPTIVEMOTIONEXECUTION_H
