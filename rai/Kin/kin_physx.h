/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

/// @file
/// @ingroup group_ors

#ifndef MLR_ors_physx_h
#define MLR_ors_physx_h
#include "kin.h"

namespace physx {
  class PxMaterial;
}

/**
 * @defgroup ors_interface_physx Interface to PhysX
 * @ingroup ors_interfaces
 * @{
 */
struct PhysXInterface : GLDrawer{
  mlr::KinematicWorld& world;
  struct sPhysXInterface *s;
  
  PhysXInterface(mlr::KinematicWorld& _world);
  ~PhysXInterface();
  
  void step(double tau=.03, bool withKinematicPush=true);
  
  void pushToPhysx(mlr::KinematicWorld *K=NULL, mlr::KinematicWorld *Kt_1=NULL, mlr::KinematicWorld *Kt_2=NULL, double tau=-1., bool onlyKinematic=true);
  void pullFromPhysx(mlr::KinematicWorld *K=NULL, arr &vels=NoArr);

  void setArticulatedBodiesKinematic();
  void ShutdownPhysX();

  void glDraw(OpenGL&);
  void watch(bool pause=false, const char* txt=NULL);

  void addForce(mlr::Vector& force, mlr::Frame* b);
  void addForce(mlr::Vector& force, mlr::Frame* b, mlr::Vector& pos);
};

void bindOrsToPhysX(mlr::KinematicWorld& graph, OpenGL& gl, PhysXInterface& physx);

#endif
/// @}


