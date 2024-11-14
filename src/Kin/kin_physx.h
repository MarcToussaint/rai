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
  RAI_PARAM("physx/", bool, softBody, false)
  RAI_PARAM("physx/", bool, multiBody, true)
  RAI_PARAM("physx/", bool, multiBodyDisableGravity, true)
  RAI_PARAM("physx/", bool, jointedBodies, false)
  RAI_PARAM("physx/", double, angularDamping, .1)
  RAI_PARAM("physx/", double, defaultFriction, 1.)
  RAI_PARAM("physx/", double, defaultRestitution, .1) //restitution=1 should be elastic...
  RAI_PARAM("physx/", double, motorKp, 1000.)
  RAI_PARAM("physx/", double, motorKd, 100.)
  RAI_PARAM("physx/", double, gripperKp, 10000.)
  RAI_PARAM("physx/", double, gripperKd, 100.)
};
}//namespace

struct PhysXInterface {
  struct PhysXInterface_self* self=0;

  PhysXInterface(const rai::Configuration& C, int verbose=1, const rai::PhysX_Options* _opt=0);
  ~PhysXInterface();

  void step(double tau=.01);

  void pushFrameStates(const rai::Configuration& C, const arr& frameVelocities=NoArr, bool onlyKinematic=false);
  void pullDynamicStates(rai::Configuration& C, arr& frameVelocities=NoArr);

  void pushMotorStates(const rai::Configuration& C, bool setInstantly=false, const arr& qDot=NoArr);
  void pullMotorStates(rai::Configuration& C, arr& qDot);

  void changeObjectType(rai::Frame* f, int type);
  void addJoint(rai::Joint* j);
  void removeJoint(rai::Joint* j);
  void postAddObject(rai::Frame* f);
  void setArticulatedBodiesKinematic(const rai::Configuration& C);

  void watch(bool pause=false, const char* txt=nullptr);

  void setGravity(float grav);
  void disableGravity(rai::Frame* f, bool disable=true);
  void addForce(rai::Vector& force, rai::Frame* b);
  void addForce(rai::Vector& force, rai::Frame* b, rai::Vector& pos);

  rai::PhysX_Options& opt();
};
