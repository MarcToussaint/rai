/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "kin.h"

namespace rai {
struct PhysX_Options {
  RAI_PARAM("physx/", int, verbose, 1)
  RAI_PARAM("physx/", bool, yGravity, false)
  RAI_PARAM("physx/", double, angularDamping, .1)
  RAI_PARAM("physx/", double, jointFriction, 0.05)
  RAI_PARAM("physx/", double, defaultFriction, 1.)
  RAI_PARAM("physx/", double, defaultRestitution, .1) //restitution=1 should be elastic...
  RAI_PARAM("physx/", double, motorKp, 1000.)
  RAI_PARAM("physx/", double, motorKd, 100.)
};
}//namespace

struct PhysXInterface {
  struct PhysXInterface_self* self=0;

  PhysXInterface(rai::Configuration& C, int verbose=1, const rai::PhysX_Options* _opt=0);
  ~PhysXInterface();

  void step(double tau=.01);

  void pushFrameStates(const rai::Configuration& C, const arr& frameVelocities=NoArr, bool onlyKinematic=false);
  void pullDynamicStates(rai::Configuration& C, arr& frameVelocities=NoArr);

  void pushMotorTargets(const rai::Configuration& C, const arr& qDot_ref=NoArr, bool setStatesInstantly=false);
  void pullMotorStates(rai::Configuration& C, arr& qDot);

  void changeObjectType(rai::Frame* f, int type);
  void addRigidJoint(rai::Frame* from, rai::Frame* to);
  void removeJoint(const rai::Frame* from, const rai::Frame* to);
  void postAddObject(rai::Frame* f);

  void setGravity(float grav);
  void disableGravity(rai::Frame* f, bool disable=true);
  void addForce(rai::Vector& force, rai::Frame* b);
  void addForce(rai::Vector& force, rai::Frame* b, rai::Vector& pos);

  rai::Configuration& getDebugConfig();
  rai::PhysX_Options& opt();
};
