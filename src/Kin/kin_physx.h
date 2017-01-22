/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
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
struct PhysXInterface {
  mlr::KinematicWorld& world;
  struct sPhysXInterface *s;
  
  PhysXInterface(mlr::KinematicWorld& _world);
  ~PhysXInterface();
  
  void step(double tau=1./60.);
  
  void pushToPhysx();
  void pullFromPhysx(double tau = 1./60.);

  void setArticulatedBodiesKinematic(uint agent=0);
  void ShutdownPhysX();

  void glDraw();

  void addForce(mlr::Vector& force, mlr::Body* b);
  void addForce(mlr::Vector& force, mlr::Body* b, mlr::Vector& pos);
};

void glPhysXInterface(void *classP);


void bindOrsToPhysX(mlr::KinematicWorld& graph, OpenGL& gl, PhysXInterface& physx);

#endif
/// @}
