/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "kin.h"
#include "frame.h"
#include "featureSymbols.h"

struct Value {
  arr y, J;
  Value(const arr& y, const arr& J) : y(y), J(J) {}
};

/// defines only a map (task space), not yet the costs or constraints in this space
struct Feature {
  uint order;          ///< 0=position, 1=vel, etc
  arr  scale, target;  ///< optional linear transformation
  bool flipTargetSignOnNegScalarProduct; ///< for order==1 (vel mode), when taking temporal difference, flip sign when scalar product it negative [specific to quats -> move to special TM for quats only]
  FeatureSymbol fs = FS_none;
  uintA frameIDs;

  Feature() : order(0), flipTargetSignOnNegScalarProduct(false) {}
  virtual ~Feature() {}

  //-- construction helpers
  Feature& setOrder(uint _order) { order=_order; return *this; }
  Feature& setScale(const arr& _scale) { scale=_scale; return *this; }
  Feature& setTarget(const arr& _target) { target=_target; return *this; }

 protected:
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual void phi(arr& y, arr& J, const rai::Configuration& C) { HALT("one of the 'phi' needs to be implemented!"); } ///< this needs to be overloaded
  virtual void phi(arr& y, arr& J, const ConfigurationL& Ctuple); ///< if not overloaded this computes the generic pos/vel/acc depending on order
  virtual uint dim_phi(const rai::Configuration& C) { HALT("one of the 'dim_phi' needs to be implemented!"); } ///< the dimensionality of $y$
  virtual uint dim_phi(const ConfigurationL& Ctuple) { return dim_phi(*Ctuple.last()); } ///< if not overloaded, returns dim_phi for last configuration
  virtual uint dim_phi2(const FrameL& Ctuple) {  HALT("one of the 'dim_phi' needs to be implemented!"); }

 public:
  void __phi2(arr& y, arr& J, const FrameL& F) { phi2(y, J, F); applyLinearTrans(y, J); }
  void __phi(arr& y, arr& J, const rai::Configuration& C) { phi(y, J, C); applyLinearTrans(y, J); }
  void __phi(arr& y, arr& J, const ConfigurationL& Ctuple) { phi(y, J, Ctuple); applyLinearTrans(y, J); }
  uint __dim_phi(const rai::Configuration& C) { uint d=dim_phi(C); return applyLinearTrans_dim(d); }
  uint __dim_phi(const ConfigurationL& Ctuple) { uint d=dim_phi(Ctuple); return applyLinearTrans_dim(d); }
  uint __dim_phi2(const FrameL& F) { uint d=dim_phi2(F); return applyLinearTrans_dim(d); }

  virtual rai::String shortTag(const rai::Configuration& C) { return "without-description"; }
  virtual rai::Graph getSpec(const rai::Configuration& C) { return rai::Graph({{"description", shortTag(C)}}); }

  //-- evaluation helpers
  arr phi(const ConfigurationL& Ctuple) { arr y; __phi(y, NoArr, Ctuple); return y; } ///< evaluate without computing Jacobian
  arr phiRaw(const ConfigurationL& Ctuple) { arr y; phi(y, NoArr, Ctuple); return y; } ///< evaluate without computing Jacobian
  Value operator()(const ConfigurationL& Ctuple) { arr y, J; __phi(y, J, Ctuple); return Value(y, J); }
  Value operator()(const rai::Configuration& C) { arr y, J; __phi(y, J, C); return Value(y, J); }
  Value eval(const rai::Configuration& C) { arr y, J; __phi(y, J, C); return Value(y, J); }
  Value eval(const ConfigurationL& Ctuple) { arr y, J; __phi(y, J, Ctuple); return Value(y, J); }

  //-- for direct gradient checking (move outside)
  VectorFunction vf(rai::Configuration& C);
  VectorFunction vf(ConfigurationL& Ctuple);
 private:
  void applyLinearTrans(arr& y, arr& J);
  uint applyLinearTrans_dim(uint d);
};

//these are frequently used by implementations of task maps

//TODO: return with a zero in front..
inline uintA getKtupleDim(const ConfigurationL& Ctuple) {
  uintA dim(Ctuple.N);
  dim(0)=Ctuple(0)->getJointStateDimension();
  for(uint i=1; i<dim.N; i++) dim(i) = dim(i-1)+Ctuple(i)->getJointStateDimension();
  return dim;
}

inline int initIdArg(const rai::Configuration& C, const char* frameName) {
  rai::Frame* a = 0;
  if(frameName && frameName[0]) a = C.getFrameByName(frameName);
  if(a) return a->ID;
//  HALT("frame '" <<frameName <<"' does not exist");
  return -1;
}

inline void expandJacobian(arr& J, const ConfigurationL& Ctuple, int i=-1) {
  CHECK(i<(int)Ctuple.N && -i<=(int)Ctuple.N, "")
  if(Ctuple.N==1) return;
  uintA qdim = getKtupleDim(Ctuple);
  qdim.prepend(0);
  if(!isSparseMatrix(J)) {
    arr tmp = zeros(J.d0, qdim.last());
    //  CHECK_EQ(J.d1, qdim.elem(i)-qdim.elem(i-1), "");
    tmp.setMatrixBlock(J, 0, qdim.elem(i-1));
    J = tmp;
  } else {
    J.sparse().reshape(J.d0, qdim.last());
    J.sparse().rowShift(qdim.elem(i-1));
  }
}

inline void padJacobian(arr& J, const ConfigurationL& Ctuple) {
  uintA qdim = getKtupleDim(Ctuple);
  arr tmp = zeros(J.d0, qdim.last());
  tmp.setMatrixBlock(J, 0, 0);
  J = tmp;
}
