/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

/// @file
/// @ingroup group_ors

#ifndef RAI_ors_physx_h
#define RAI_ors_physx_h
#include "kin.h"

namespace physx {
class PxMaterial;
}

/**
 * @defgroup rai_interface_physx Interface to PhysX
 * @ingroup rai_interfaces
 * @{
 */
struct PhysXInterface : GLDrawer {
  struct PhysXInterface_self* self;

  PhysXInterface(const rai::Configuration& world, bool verbose=false);
  ~PhysXInterface();

  void step(double tau=.01);

  void pushKinematicStates(const FrameL& frames);
  void pushFullState(const FrameL& frames, const arr& vels=NoArr, rai::Configuration* Kt_1=nullptr, rai::Configuration* Kt_2=nullptr, double tau=-1., bool onlyKinematic=false);
  void pullDynamicStates(FrameL& frames, arr& vels=NoArr);

  void setArticulatedBodiesKinematic(const rai::Configuration& C);
  void ShutdownPhysX();

  void glDraw(OpenGL&);
  void watch(bool pause=false, const char* txt=nullptr);

  void addForce(rai::Vector& force, rai::Frame* b);
  void addForce(rai::Vector& force, rai::Frame* b, rai::Vector& pos);
};

#endif
/// @}

