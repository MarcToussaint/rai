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

/// defines only a map (task space), not yet the costs or constraints in this space
struct Feature {
  uint order = 0;          ///< 0=position, 1=vel, etc
  arr  scale, target;  ///< optional linear transformation
  bool flipTargetSignOnNegScalarProduct = false; ///< for order==1 (vel mode), when taking temporal difference, flip sign when scalar product it negative [specific to quats -> move to special TM for quats only]
  bool diffInsteadOfVel = false;
  int timeIntegral = 0;
  FeatureSymbol fs = FS_none;
  uintA frameIDs;

  Feature() {}
  Feature(const uintA& _frameIDs, uint _order) : order(_order), frameIDs(_frameIDs) {}
  virtual ~Feature() {}

  //-- construction helpers
  Feature& setOrder(uint _order) { order=_order; return *this; }
  Feature& setScale(const arr& _scale) { scale=_scale; return *this; }
  Feature& setTarget(const arr& _target) { target=_target; return *this; }
  Feature& setFrameIDs(const uintA& _frameIDs) { frameIDs=_frameIDs; return *this; }
  Feature& setFrameIDs(const StringA& frames, const rai::Configuration& C) { setFrameIDs( C.getFrameIDs(frames) ); return *this; }
  Feature& setDiffInsteadOfVel(){ diffInsteadOfVel=true; return *this; }
  FrameL getFrames(const rai::Configuration& C, uint s=0);

protected:
  //-- core methods to imlement the feature -- to be overloaded (you can choose to overload phi or phi2
  virtual arr phi(const FrameL& F);
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F) {  NIY; }

 public:
  arr eval(const FrameL& F) { arr y = phi(F); applyLinearTrans(y); return y; }
//  Value eval(const FrameL& F) { arr y, J; eval(y, J, F); return Value(y, J); }
  arr eval(const rai::Configuration& C) { return eval(getFrames(C)); }
  uint dim(const FrameL& F) { uint d=dim_phi2(F); return applyLinearTrans_dim(d); }
  VectorFunction vf2(const FrameL& F);

  virtual rai::String shortTag(const rai::Configuration& C);
  virtual rai::Graph getSpec(const rai::Configuration& C) { return rai::Graph({{"description", shortTag(C)}}); }

  //automatic finite difference definition of higher order features
  arr phi_finiteDifferenceReduce(const FrameL& F);
private:
  void applyLinearTrans(arr& y);
  uint applyLinearTrans_dim(uint d);
};

template<class T> arr evalFeature(const FrameL& F, uint order=0){ return T().setOrder(order).eval(F); }

//these are frequently used by implementations of task maps

//TODO: return with a zero in front..
//inline uintA getKtupleDim(const ConfigurationL& Ctuple) {
//  uintA dim(Ctuple.N);
//  dim(0)=Ctuple(0)->getJointStateDimension();
//  for(uint i=1; i<dim.N; i++) dim(i) = dim(i-1)+Ctuple(i)->getJointStateDimension();
//  return dim;
//}

inline int initIdArg(const rai::Configuration& C, const char* frameName) {
  rai::Frame* a = 0;
  if(frameName && frameName[0]) a = C.getFrame(frameName);
  if(a) return a->ID;
//  HALT("frame '" <<frameName <<"' does not exist");
  return -1;
}

//inline void expandJacobian(arr& J, const ConfigurationL& Ctuple, int i=-1) {
//  CHECK(i<(int)Ctuple.N && -i<=(int)Ctuple.N, "")
//  if(Ctuple.N==1) return;
//  uintA qdim = getKtupleDim(Ctuple);
//  qdim.prepend(0);
//  if(!isSparseMatrix(J)) {
//    arr tmp = zeros(J.d0, qdim.last());
//    //  CHECK_EQ(J.d1, qdim.elem(i)-qdim.elem(i-1), "");
//    tmp.setMatrixBlock(J, 0, qdim.elem(i-1));
//    J = tmp;
//  } else {
//    J.sparse().reshape(J.d0, qdim.last());
//    J.sparse().rowShift(qdim.elem(i-1));
//  }
//}

//inline void padJacobian(arr& J, const ConfigurationL& Ctuple) {
//  uintA qdim = getKtupleDim(Ctuple);
//  if(!isSpecial(J)){
//    arr tmp = zeros(J.d0, qdim.last());
//    tmp.setMatrixBlock(J, 0, 0);
//    J = tmp;
//  }else{
//    if(J.isSparse()){
//      J.sparse().reshape(J.d0, qdim.last());
//    } else if(!J){
//      return;
//    } else NIY;
//  }
//}

template<class T>
std::shared_ptr<Feature> make_feature(const StringA& frames, const rai::Configuration& C, const arr& scale=NoArr, const arr& target=NoArr, int order=-1){
  std::shared_ptr<Feature> f = make_shared<T>();

  if(!!frames && frames.N){
    CHECK(!f->frameIDs.N, "frameIDs are already set");
    if(frames.N==1 && frames.scalar()=="ALL") f->frameIDs = framesToIndices(C.frames);
    else f->frameIDs = C.getFrameIDs(frames);
  }

  if(!!scale) {
    if(!f->scale.N) f->scale = scale;
    else if(scale.N==1) f->scale *= scale.scalar();
    else if(scale.N==f->scale.N) f->scale *= scale.scalar();
    else NIY;
  }

  if(!!target) f->target = target;

  if(order>=0) f->order = order;

  return f;
}
