/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "adaptiveMotionExecution.h"

AdaptiveMotionExecution::AdaptiveMotionExecution(rai::Configuration& _world, arr& _trajRef, double _dt, double _TRef, arr& _x0, arr& _q0, MObject& _goalMO, \
    bool _useOrientation):
  world(&_world),
  dt(_dt),
  TRef(_TRef),
  useOrientation(_useOrientation),
  goalMO(&_goalMO),
  x0(_x0),
  q0(_q0) {
  goalRef = _trajRef[_trajRef.d0-1];
  dsRef = dt/TRef;
  sRef = linspace(0., 1., _trajRef.d0-1);

  if(useOrientation) {
    goalMO->setOrientation(goalRef({3, 5}));
  }

  traj = ~x0;
  s = ARR(0.);
  joints_bk.append(~q0);

  lastGoal = goalRef+(x0-_trajRef[0]);

  trajWrap = new rai::Path(_trajRef);
  trajRef = new rai::Path(_trajRef);
}

void AdaptiveMotionExecution::getNextState(arr& _state, arr& _dstate) {
  _state = desState;
  _dstate = desVel;
}

void AdaptiveMotionExecution::iterate(arr& _state, double _dtReal) {
  if(_dtReal > 0.) {
    dsRef = _dtReal/TRef;
  }

  state = _state;
  goal = goalMO->position;
  if(useOrientation) {
    goal.append(goalMO->orientation);
  }

  // update phase variable s
  if(traj.d0>2) {
    double goalRatio = length(goalRef - trajRef->eval(s.last()))/length(lastGoal-state);
    double stateRatio = length(state - traj[traj.d0-2])/length(traj[traj.d0-1] - traj[traj.d0-2]);
    s.append(s(s.d0-1) + dsRef*goalRatio*stateRatio);
  } else {
    s.append(s(s.d0-1) + dsRef);
  }

  // warp trajectory
  traj[traj.d0-1] = state;
  warpTrajectory();
  lastGoal = goal;

  // compute next state
  arr dir = trajWrap->getVelocity(s.last());
  dir = dir/length(dir);

//  desVel = dir*dsRef*length(trajRef->deval(s.last()))/dt;
  desVel = dir*length(trajRef->getVelocity(s.last()))/TRef;
  desState = traj[traj.d0-1] + desVel*dt;
  traj.append(desState);
}

void AdaptiveMotionExecution::warpTrajectory() {
  arr stateDiff = state-trajWrap->getPosition(s.last());
  arr goalDiff = goal-lastGoal;
  trajWrap->transform_CurrentFixed_EndBecomes(goal, s.last());
  trajWrap->transform_CurrentBecomes_EndFixed(state, s.last());
}

void AdaptiveMotionExecution::plotState() {
  if(scene.M==0) {
    scene = STRING("out");
  } else {
    scene = STRING("out/"<<scene);
  }
  cout << "Save Path: " << scene << endl;

  write(LIST<arr>(joints_bk), STRING(scene<<"/joints_bk.output"));
  write(LIST<arr>(ARR(dt)), STRING(scene<<"/dt.output"));
  write(LIST<arr>(goal), STRING(scene<<"/goal.output"));

  write(LIST<arr>(trajRef->points), STRING(scene<<"/trajRef.output"));
  write(LIST<arr>(trajWrap->points), STRING(scene<<"/trajWrap.output"));
  write(LIST<arr>(traj), STRING(scene<<"/traj.output"));

  gnuplot("set term wxt 1 title 'position 1'");
  gnuplot(STRING("plot '"<<scene<<"/trajRef.output' us 1,'"<<scene<<"/trajWrap.output' us 1, '"<<scene<<"/traj.output' us 1"));
  gnuplot("set term wxt 2 title 'position 2'");
  gnuplot(STRING("plot '"<<scene<<"/trajRef.output' us 2,'"<<scene<<"/trajWrap.output' us 2, '"<<scene<<"/traj.output' us 2"));
  gnuplot("set term wxt 3 title 'position 3'");
  gnuplot(STRING("plot '"<<scene<<"/trajRef.output' us 3,'"<<scene<<"/trajWrap.output' us 3, '"<<scene<<"/traj.output' us 3"));

  if(useOrientation) {
    gnuplot("set term wxt 4 title 'orientation 1'");
    gnuplot(STRING("plot '"<<scene<<"/trajRef.output' us 4,'"<<scene<<"/trajWrap.output' us 4, '"<<scene<<"/traj.output' us 4"));
    gnuplot("set term wxt 5 title 'orientation 2'");
    gnuplot(STRING("plot '"<<scene<<"/trajRef.output' us 5,'"<<scene<<"/trajWrap.output' us 5, '"<<scene<<"/traj.output' us 5"));
    gnuplot("set term wxt 6 title 'orientation 3'");
    gnuplot(STRING("plot '"<<scene<<"/trajRef.output' us 6,'"<<scene<<"/trajWrap.output' us 6, '"<<scene<<"/traj.output' us 6"));
  }

  write(LIST<arr>(s), STRING(scene<<"/s.output"));
  write(LIST<arr>(sRef), STRING(scene<<"/sRef.output"));
  gnuplot("set term wxt 7 title 'phase profile'");
  gnuplot(STRING("plot '"<<scene<<"/s.output' us 1, '"<<scene<<"/sRef.output' us 1"));

  //compute velocity of trajectory
  arr dtraj;
  resizeAs(dtraj, traj);
  for(uint j=0; j < traj.d0; j++) {
    for(uint i=0; i < traj.d1; i++) {
      if(j==0) {
        dtraj(j, i) = (traj(j+1, i)-traj(j, i))/dt;
      } else if(j==(traj.d0-1)) {
        dtraj(j, i) = (traj(j, i)-traj(j-1, i))/dt;
      } else {
        dtraj(j, i) = (traj(j+1, i)-traj(j-1, i))/(2*dt);
      }
    }
  }

  //compute velocity of input trajectory
  resizeAs(dtrajRef, trajRef->points);
  double dtRef = TRef/trajRef->points.d0;
  for(uint j=0; j < trajRef->points.d0; j++) {
    for(uint i=0; i < trajRef->points.d1; i++) {
      if(j==0) {
        dtrajRef(j, i) = (trajRef->points(j+1, i)-trajRef->points(j, i))/dtRef;
      } else if(j==(trajRef->points.d0-1)) {
        dtrajRef(j, i) = (trajRef->points(j, i)-trajRef->points(j-1, i))/dtRef;
      } else {
        dtrajRef(j, i) = (trajRef->points(j+1, i)-trajRef->points(j-1, i))/(2*dtRef);
      }
    }
  }

  write(LIST<arr>(sqrt(sum(sqr(~(~dtrajRef)({0, 2})), 1))), STRING(scene<<"/dtrajRef.output"));
  write(LIST<arr>(sqrt(sum(sqr(~(~dtraj)({0, 2})), 1))), STRING(scene<<"/dtraj.output"));
  gnuplot("set term wxt 11 title 'velocity profile'");
  gnuplot(STRING("plot '"<<scene<<"/dtrajRef.output' us 1,'"<<scene<<"/dtraj.output' us 1"));

}

void AdaptiveMotionExecution::printState() {
  cout << "TRef = " << TRef << endl;
  cout << "dt = " << dt << endl;
  cout << "goalRef = " << goalRef << endl;
  cout << "x0 = " << x0 << endl;
  cout << "dsRef = " << dsRef << endl;
}

//void AdaptiveMotionExecution::computeIK(arr &q, arr &qd)
//{
//    arr W, yPos, JPos, yVec, JVec, yPos_target,yVec_target, y_target, Phi, PhiJ, yCol,JCol,costs;

//    W.setDiag(1.,world->getJointStateDimension());  // W is equal the Id_n matrix
//    W = W*w_reg;

//    joints_bk.append(~q);

//    // Compute current task states
//    world->kinematicsPos(yPos, world->getBodyByName("endeff")->index);
//    world->jacobianPos(JPos, world->getBodyByName("endeff")->index);
//    world->kinematicsVec(yVec, world->getBodyByName("endeff")->index);
//    world->jacobianVec(JVec, world->getBodyByName("endeff")->index);

//    // iterate amex
//    arr y = yPos;
//    if (useOrientation) {
//      y.append(yVec);
//    }
//    iterate(y);

//    // next target
//    y_target = traj[traj.d0-1];

//    // task 1: POSITION
//    yPos_target = y_target({0,2});
//    costs = (yPos - yPos_target)/ fPos_deviation;
//    posCosts.append(~costs*costs);
//    Phi = ((yPos - yPos_target)/ fPos_deviation);
//    PhiJ = (JPos / fPos_deviation);

//    // task  2: ORIENTATION
//    if (useOrientation) {
//      yVec_target = y_target({3,5});
//      costs = (yVec - yVec_target)/ fVec_deviation;
//      vecCosts.append(~costs*costs);
//      Phi.append(costs);
//      PhiJ.append(JVec / fVec_deviation);
//    }

//    // task 3: COLLISION
//    if (useCollAvoid) {
//      world->phiCollision(yCol,JCol,0.15);
//      costs = yCol / yCol_deviation;
//      colCosts.append(~costs*costs);
//      Phi.append(costs);
//      PhiJ.append(JCol / yCol_deviation);
//    }

//    // compute joint updates
//    qd = -inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;
//    q += qd;
//}
