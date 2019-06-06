/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
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
  struct sPhysXInterface *s;
  
  PhysXInterface(const rai::KinematicWorld& world);
  ~PhysXInterface();
  
  void step(double tau=.01);
  
  void pushKinematicStates(const FrameL& frames);
  void pushFullState(const FrameL& frames, const arr& vels=NoArr, rai::KinematicWorld *Kt_1=NULL, rai::KinematicWorld *Kt_2=NULL, double tau=-1., bool onlyKinematic=false);
  void pullDynamicStates(FrameL& frames, arr &vels=NoArr);

  void setArticulatedBodiesKinematic(const rai::KinematicWorld& C);
  void ShutdownPhysX();
  
  void glDraw(OpenGL&);
  void watch(bool pause=false, const char* txt=NULL);
  
  void addForce(rai::Vector& force, rai::Frame* b);
  void addForce(rai::Vector& force, rai::Frame* b, rai::Vector& pos);
};

void bindOrsToPhysX(rai::KinematicWorld& graph, OpenGL& gl, PhysXInterface& physx);

#endif
/// @}

