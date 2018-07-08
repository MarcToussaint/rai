#include "featureSymbols.h"

#include <Kin/TM_default.h>
#include <Kin/TM_proxy.h>
#include <Kin/TM_qItself.h>
#include <Kin/TM_PairCollision.h>
#include <Kin/TM_transition.h>
#include <Kin/TM_qLimits.h>
//#include <Kin/proxy.h>

template<> const char* rai::Enum<FeatureSymbol>::names []= {
  "FS_position",
  "FS_positionDiff",
  "FS_positionRel",
  "FS_quaternion",
  "FS_quaternionDiff",
  "FS_quaternionRel",
  "FS_pose",
  "FS_poseDiff",
  "FS_poseRel",
  "FS_vectorX",
  "FS_vectorXDiff",
  "FS_vectorXRel",
  "FS_vectorY",
  "FS_vectorYDiff",
  "FS_vectorYRel",
  "FS_vectorZ",
  "FS_vectorZDiff",
  "FS_vectorZRel",
  "FS_scalarProductZ",
  "FS_gazeAt",
  "FS_accumulatedCollisions",
  "FS_jointLimits",
  "FS_distance",
  "FS_qItself",
  "FS_aboveBox",
  "FS_insideBox",
  "FS_standingAbove",
  NULL
};


double shapeSize(const rai::KinematicWorld& K, const char* name, uint i=2) {
  rai::Frame *f = K.getFrameByName(name);
  rai::Shape *s = f->shape;
  if(!s) {
    for(rai::Frame *b:f->outLinks) if(b->name==name && b->shape) { s=b->shape; break; }
  }
  if(!s) return 0;
  return s->size(i);
}

TaskMap* symbols2feature(FeatureSymbol feat, const StringA& frames, const rai::KinematicWorld& world){
  if(feat==FS_distance) {  return new TM_PairCollision(world, frames(0), frames(1), TM_PairCollision::_negScalar, false); }
  if(feat==FS_aboveBox) {  return new TM_AboveBox(world, frames(1), frames(0), .05); }
  if(feat==FS_standingAbove) {
    double h = .5*(shapeSize(world, frames(0)) + shapeSize(world, frames(1)));
    TaskMap *relPos = new TM_Default(TMT_posDiff, world, frames(0), rai::Vector(0.,0.,h), frames(1), NoVector);
    return new TM_LinTrans(relPos, arr(1,3,{0.,0.,1.}), {});
  }

  if(feat==FS_position) {  return new TM_Default(TMT_pos, world, frames(0)); }
  if(feat==FS_positionDiff) {  return new TM_Default(TMT_posDiff, world, frames(0), NoVector, frames(1)); }
  if(feat==FS_positionRel) {  return new TM_Default(TMT_pos, world, frames(0), NoVector, frames(1)); }

  if(feat==FS_vectorX) {  return new TM_Default(TMT_vec, world, frames(0), Vector_x); }
  if(feat==FS_vectorXDiff) {  return new TM_Default(TMT_vecDiff, world, frames(0), Vector_x, frames(1), Vector_x); }
  if(feat==FS_vectorXRel) {  return new TM_Default(TMT_vec, world, frames(0), Vector_x, frames(1), Vector_x); }

  if(feat==FS_quaternion) {  return new TM_Default(TMT_quat, world, frames(0)); }
  if(feat==FS_quaternionDiff) {  return new TM_Default(TMT_quatDiff, world, frames(0), NoVector, frames(1), NoVector); }
  if(feat==FS_quaternionRel) {  return new TM_Default(TMT_quat, world, frames(0), NoVector, frames(1), NoVector); }

  if(feat==FS_pose) {  return new TM_Default(TMT_pose, world, frames(0)); }
  if(feat==FS_poseDiff) {  return new TM_Default(TMT_poseDiff, world, frames(0), NoVector, frames(1), NoVector); }
  if(feat==FS_poseRel) NIY;

  if(feat==FS_scalarProductZ) {  return new TM_Default(TMT_vecAlign, world, frames(0), Vector_z, frames(1), Vector_z); }

  if(feat==FS_gazeAt) { return new TM_Default(TMT_gazeAt, world, frames(0), NoVector, frames(1)); }

  if(feat==FS_accumulatedCollisions) {  return new TM_Proxy(TMT_allP, {}); }
  if(feat==FS_jointLimits) {  return new LimitsConstraint(.05); }

  if(feat==FS_qItself) { return new TM_qItself(); }

  HALT("can't interpret feature symbols: " <<feat);
  return 0;
}
