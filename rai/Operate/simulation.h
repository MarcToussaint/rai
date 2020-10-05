/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Gui/opengl.h"
#include "../Geo/mesh.h"
#include "../Kin/kin.h"
#include "../Kin/frame.h"
#include <iomanip>

//=============================================================================

struct ContactEntry {
  rai::String objA, objB;
  double d;
  void write(ostream& os) const { os <<"Contact " <<objA <<"--" <<objB <<" distance=" <<d; }
};

typedef rai::Array<ContactEntry> ContactA;

struct PlanSkeletonEntry {
  StringA symbols;
  double timeStart=-1.;
  double timeTo=-1.;
  PlanSkeletonEntry() {}
  PlanSkeletonEntry(StringA symbols, double _timeFrom, double _timeTo):symbols(symbols), timeStart(_timeFrom), timeTo(_timeTo) {}
  void write(ostream& os) const { symbols.write(os, " ", nullptr, "()"); os <<" from " <<timeStart <<" to " <<timeTo; }
};
stdOutPipe(PlanSkeletonEntry)

struct Plan {
  arr robot_joint_path;
  arr tau;
  rai::Array<PlanSkeletonEntry> skeleton;
};

//=============================================================================

struct Simulation : GLDrawer {
  struct Simulation_self* self=0;
  rai::Configuration K;

  Simulation(const rai::Configuration& _K, double dt=.01);
  ~Simulation();

  //-- stepping forward in time
  void stepKin();

  //--
  void setJointState(const uintA& joints, const arr& q_ref);
  void setJointStateSafe(arr q_ref, StringA& jointsInLimit, StringA& collisionPairs);

  //-- the narrow action interface
  StringA getRobotJoints(); //info on the joints; the plan needs to have same dimensionality
  void setUsedRobotJoints(const uintA& joints);
  void exec(const Plan& plan, bool append=true);
  void exec(const arr& robot_joint_path, const arr& tau, bool append=true);
  void exec(const StringA& command);
  void stop(bool hard=false);

  //-- feedback from action execution
  double getTimeToGo();
  double trackingError();

  //-- true state info
  arr getJointState();
  arr getFrameState();

  //TO BE REMOVED -> perceptual
  arr getObjectPoses(const StringA& objects= {});
  StringA getJointNames();
  StringA getObjectNames();
  ContactA getContactInfo(double margin=.1);

  void glDraw(OpenGL&);
};

