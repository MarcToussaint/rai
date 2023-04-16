/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
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
    RAI_PARAM("physx/", bool, jointedBodies, false)
    RAI_PARAM("physx/", double, angularDamping, .1)
    RAI_PARAM("physx/", double, defaultFriction, 1.)
    RAI_PARAM("physx/", double, defaultRestitution, .1)
    RAI_PARAM("physx/", double, motorKp, 1000.)
    RAI_PARAM("physx/", double, motorKd, 100.)
  };
}//namespace

struct PhysXInterface : GLDrawer {
  struct PhysXInterface_self* self=0;

  PhysXInterface(const rai::Configuration& C, int verbose=1);
  ~PhysXInterface();

  void step(double tau=.01);

  void pushKinematicStates(const rai::Configuration& C);
  void pushFullState(const rai::Configuration& C, const arr& frameVelocities=NoArr, bool onlyKinematic=false);
  void pullDynamicStates(rai::Configuration& C, arr& frameVelocities=NoArr);

  void setMotorQ(const arr& q_ref, const arr& qDot_ref);
  void setMotorQ(const rai::Configuration& C);

  void changeObjectType(rai::Frame* f, int type);
  void postAddObject(rai::Frame* f);
  void setArticulatedBodiesKinematic(const rai::Configuration& C);

  void glDraw(OpenGL&);
  void watch(bool pause=false, const char* txt=nullptr);

  void addForce(rai::Vector& force, rai::Frame* b);
  void addForce(rai::Vector& force, rai::Frame* b, rai::Vector& pos);

  rai::PhysX_Options& opt();
};
