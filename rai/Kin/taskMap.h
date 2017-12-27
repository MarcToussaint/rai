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


#pragma once
#include <Kin/kin.h>
#include <Kin/frame.h>

/// defines only a map (task space), not yet the costs in this space
struct TaskMap {
  uint order;       ///< 0=position, 1=vel, etc
  bool flipTargetSignOnNegScalarProduct; ///< for order==1 (vel mode), when taking temporal difference, flip sign when scalar product it negative [specific to quats -> move to special TM for quats only]
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& K, int t=-1) = 0; ///< this needs to be overloaded
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple, double tau, int t=-1); ///< if not overloaded this computes the generic pos/vel/acc depending on order
  virtual uint dim_phi(const mlr::KinematicWorld& K) = 0; ///< the dimensionality of $y$
  virtual uint dim_phi(const WorldL& Ktuple, int t){ return dim_phi(*Ktuple.last()); } ///< if not overloaded, returns dim_phi for last configuration

  TaskMap():order(0), flipTargetSignOnNegScalarProduct(false) {}
  virtual ~TaskMap() {}
  virtual mlr::String shortTag(const mlr::KinematicWorld& K){ NIY; }

  //-- helpers
  arr phi(const mlr::KinematicWorld& K){ arr y; phi(y,NoArr,K); return y; } ///< evaluate without computing Jacobian

  VectorFunction vf(mlr::KinematicWorld& K){ ///< direct conversion to vector function: use to check gradient or evaluate
    return [this, &K](arr& y, arr& J, const arr& x) -> void {
      K.setJointState(x);
      phi(y, J, K, -1);
    };
  }

  //-- creation
  static TaskMap *newTaskMap(const Graph& specs, const mlr::KinematicWorld& K); ///< creates a task map based on specs
  static TaskMap *newTaskMap(const Node* specs, const mlr::KinematicWorld& K); ///< creates a task map based on specs
};

inline uintA getKtupleDim(const WorldL& Ktuple){
  uintA dim(Ktuple.N);
  dim(0)=Ktuple(0)->q.N;
  for(uint i=1;i<dim.N;i++) dim(i) = dim(i-1)+Ktuple(i)->q.N;
  return dim;
}

inline int initIdArg(const mlr::KinematicWorld &K, const char* frameName){
  mlr::Frame *a = frameName ? K.getFrameByName(frameName):NULL;
  if(a) return a->ID;
  return -1;
}
