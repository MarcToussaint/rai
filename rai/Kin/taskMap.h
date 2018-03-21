/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once
#include <Kin/kin.h>
#include <Kin/frame.h>

/// defines only a map (task space), not yet the costs or constraints in this space
struct TaskMap {
  uint order;       ///< 0=position, 1=vel, etc
  bool flipTargetSignOnNegScalarProduct; ///< for order==1 (vel mode), when taking temporal difference, flip sign when scalar product it negative [specific to quats -> move to special TM for quats only]
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& K, int t=-1) = 0; ///< this needs to be overloaded
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple, double tau, int t=-1); ///< if not overloaded this computes the generic pos/vel/acc depending on order
  virtual uint dim_phi(const mlr::KinematicWorld& K) = 0; ///< the dimensionality of $y$
  virtual uint dim_phi(const WorldL& Ktuple, int t){ return dim_phi(*Ktuple.last()); } ///< if not overloaded, returns dim_phi for last configuration

  TaskMap() : order(0), flipTargetSignOnNegScalarProduct(false) {}
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
};


//these are frequently used by implementations of task maps

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

inline void expandJacobian(arr& J,const WorldL& Ktuple,int i=-1){
  uintA qdim = getKtupleDim(Ktuple);
  qdim.prepend(0);
  arr tmp = zeros(J.d0, qdim.last());
  tmp.setMatrixBlock(J, 0, qdim.elem(i-1));
  J = tmp;
}
