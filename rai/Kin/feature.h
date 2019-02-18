/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once
#include <Kin/kin.h>
#include <Kin/frame.h>

struct Value{
  arr y,J;
  Value(const arr& y, const arr& J) : y(y), J(J) {}
};

/// defines only a map (task space), not yet the costs or constraints in this space
struct Feature {
  uint order;       ///< 0=position, 1=vel, etc
  arr scale, target;     ///< optional linear transformation
  bool flipTargetSignOnNegScalarProduct; ///< for order==1 (vel mode), when taking temporal difference, flip sign when scalar product it negative [specific to quats -> move to special TM for quats only]
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& K) = 0; ///< this needs to be overloaded
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple); ///< if not overloaded this computes the generic pos/vel/acc depending on order
  virtual uint dim_phi(const rai::KinematicWorld& K) = 0; ///< the dimensionality of $y$
  virtual uint dim_phi(const WorldL& Ktuple) { return dim_phi(*Ktuple.last()); } ///< if not overloaded, returns dim_phi for last configuration
  
  Feature() : order(0), flipTargetSignOnNegScalarProduct(false) {}
  virtual ~Feature() {}
  virtual rai::String shortTag(const rai::KinematicWorld& K) { NIY; }
  virtual Graph getSpec(const rai::KinematicWorld& K){ return Graph({{"description", shortTag(K)}}); }
  
  //-- helpers
  arr phi(const rai::KinematicWorld& K) { arr y; phi(y,NoArr,K); return y; } ///< evaluate without computing Jacobian
  Value operator()(const WorldL& Ktuple){ arr y,J; phi(y, J, Ktuple); return Value(y,J); }
  
  VectorFunction vf(rai::KinematicWorld& K);
  VectorFunction vf(WorldL& Ktuple);
protected:
  void applyLinearTrans(arr& y, arr& J);
};

//these are frequently used by implementations of task maps

inline uintA getKtupleDim(const WorldL& Ktuple) {
  uintA dim(Ktuple.N);
  dim(0)=Ktuple(0)->getJointStateDimension();
  for(uint i=1; i<dim.N; i++) dim(i) = dim(i-1)+Ktuple(i)->getJointStateDimension();
  return dim;
}

inline int initIdArg(const rai::KinematicWorld &K, const char* frameName) {
  rai::Frame *a = frameName ? K.getFrameByName(frameName):NULL;
  if(a) return a->ID;
  return -1;
}

inline void expandJacobian(arr& J, const WorldL& Ktuple, int i=-1) {
  uintA qdim = getKtupleDim(Ktuple);
  qdim.prepend(0);
  arr tmp = zeros(J.d0, qdim.last());
//  CHECK_EQ(J.d1, qdim.elem(i)-qdim.elem(i-1), "");
  tmp.setMatrixBlock(J, 0, qdim.elem(i-1));
  J = tmp;
}

inline void padJacobian(arr& J, const WorldL& Ktuple) {
  uintA qdim = getKtupleDim(Ktuple);
  arr tmp = zeros(J.d0, qdim.last());
  tmp.setMatrixBlock(J, 0, 0);
  J = tmp;
}
