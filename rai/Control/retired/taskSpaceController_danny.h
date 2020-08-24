/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef TASKSPACECONTROLLER_H
#define TASKSPACECONTROLLER_H

#include "../Kin/kin.h"
#include "../Kin/taskMaps.h"
#include "../Core/array.h"
#include "../RosCom/roscom.h"
#include "../Algo/spline.h"
#include "../Core/thread.h"

struct LinTaskSpaceAccLaw {
  Feature* map;

  rai::Configuration* world;

  rai::String name;

  arr yRef;
  arr yDotRef;
  arr yDDotRef;

  arr Kp;
  arr Kd;
  arr C;

  arr trajectory;
  arr trajectoryDot;
  arr trajectoryDDot;

  rai::Spline* trajectorySpline;
  rai::Spline* trajectoryDotSpline;
  rai::Spline* trajectoryDDotSpline;

  bool trajectoryActive = false;
  bool trajectoryDotActive = false;
  bool trajectoryDDotActive = false;

  LinTaskSpaceAccLaw(Feature* map, rai::Configuration* world, rai::String name = "nonameLaw");

  void setRef(const arr& yRef = NoArr, const arr& yDotRef = NoArr, const arr& yDDotRef = NoArr);

  void setGains(arr Kp, arr Kd);

  void setC(arr C);

  void setTrajectory(uint trajLength, const arr& traj = NoArr, const arr& trajDot = NoArr, const arr& trajDDot = NoArr);
  void setSpline(rai::Spline* yS = nullptr, rai::Spline* yDotS = nullptr, rai::Spline* yDDotS = nullptr);

  void setTargetEvalSpline(double s);

  void setTrajectoryActive(bool active); // TODO

  arr getPhi();
  void getPhi(arr& y, arr& J);
  uint getPhiDim();

  arr getC();
  arr getKp();
  arr getKd();

  void getRef(arr& yRef, arr& yDotRef, arr& yDDotRef);
  arr getRef();
  arr getDotRef();
  arr getDDotRef();

  bool getTrajectoryActive(); //TODO
  bool getTrajectoryDotActive(); //TODO
  bool getTrajectoryDDotActive(); //TODO

  double getCosts();

};

struct ConstrainedTaskLaw : LinTaskSpaceAccLaw {

  arr force;
  arr alpha;
  double gamma = 0.0;
  ConstrainedTaskLaw(Feature* map, rai::Configuration* world, rai::String name = "") : LinTaskSpaceAccLaw(map, world, name) {}
  void setForce(arr force);
  arr getForce();
  void setAlpha(arr alpha);
  arr getAlpha();
  void setGamma(double gamma);
  double getGamma();

};

struct TaskSpaceController {
  rai::Array<LinTaskSpaceAccLaw*> taskSpaceAccLaws;
  rai::Configuration* world;

  bool gravity = false;

  rai::Array<ConstrainedTaskLaw*> constrainedTaskLaws;

  TaskSpaceController(rai::Configuration* world) : world(world) {}
  ~TaskSpaceController() {}

  void addLinTaskSpaceAccLaw(LinTaskSpaceAccLaw* law);
  void addConstrainedTaskLaw(ConstrainedTaskLaw* law);
  void calcOptimalControlProjected(arr& Kp, arr& Kd, arr& u0);
  void calcForceControl(arr& K_ft, arr& J_ft_inv, arr& fRef, double& gamma);
  void generateTaskSpaceTrajectoryFromJointSpace(const arr& jointSpaceTrajectory, const arr& jointSpaceTrajectoryDot = NoArr, const arr& jointSpaceDDotTrajectory = NoArr);
  void generateTaskSpaceSplines();

  void setGravity(bool gravity);
};

#endif // TASKSPACECONTROLLER_H
