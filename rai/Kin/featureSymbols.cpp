#include "featureSymbols.h"

#include <Kin/TM_default.h>
#include <Kin/TM_proxy.h>
#include <Kin/TM_qItself.h>
#include <Kin/TM_PairCollision.h>
#include <Kin/TM_transition.h>
#include <Kin/TM_qLimits.h>
#include <Kin/TM_NewtonEuler.h>
#include <Kin/TM_ContactConstraints.h>
#include <Kin/TM_energy.h>
#include <Kin/TM_angVel.h>
//#include <Kin/proxy.h>

template<> const char* rai::Enum<FeatureSymbol>::names []= {
  "position",
  "positionDiff",
  "positionRel",
  "quaternion",
  "quaternionDiff",
  "quaternionRel",
  "pose",
  "poseDiff",
  "poseRel",
  "vectorX",
  "vectorXDiff",
  "vectorXRel",
  "vectorY",
  "vectorYDiff",
  "vectorYRel",
  "vectorZ",
  "vectorZDiff",
  "vectorZRel",
  "scalarProductZ",
  "gazeAt",
  "accumulatedCollisions",
  "jointLimits",
  "distance",
  "qItself",
  "aboveBox",
  "insideBox",
  "standingAbove",
  "physics",
  "contactConstraints",
  "energy",
  "transAccelerations",
  "transVelocities",
  NULL
};


double shapeSize(const rai::KinematicWorld& K, const char* name, uint i=2) {
  rai::Frame *f = K.getFrameByName(name);
  rai::Shape *s = f->shape;
  if(!s) {
    for(rai::Frame *b:f->parentOf) if(b->name==name && b->shape) { s=b->shape; break; }
  }
  if(!s) return 0;
  return s->size(i);
}

ptr<Feature> symbols2feature(FeatureSymbol feat, const StringA& frames, const rai::KinematicWorld& world, const arr& scale, const arr& target, int order){
  ptr<Feature> f;
  if(feat==FS_distance) {  f=make_shared<TM_PairCollision>(world, frames(0), frames(1), TM_PairCollision::_negScalar, false); }
  else if(feat==FS_aboveBox) {  f=make_shared<TM_AboveBox>(world, frames(1), frames(0), .05); }
  else if(feat==FS_standingAbove) {
    double h = .5*(shapeSize(world, frames(0)) + shapeSize(world, frames(1)));
    ptr<Feature> relPos = make_shared<TM_Default>(TMT_posDiff, world, frames(0), rai::Vector(0.,0.,h), frames(1), NoVector);
    f=make_shared<TM_LinTrans>(relPos, arr(1,3,{0.,0.,1.}), arr());
  }

  else if(feat==FS_position) {  f=make_shared<TM_Default>(TMT_pos, world, frames(0)); }
  else if(feat==FS_positionDiff) {  f=make_shared<TM_Default>(TMT_posDiff, world, frames(0), NoVector, frames(1)); }
  else if(feat==FS_positionRel) {  f=make_shared<TM_Default>(TMT_pos, world, frames(0), NoVector, frames(1)); }

  else if(feat==FS_vectorX) {  f=make_shared<TM_Default>(TMT_vec, world, frames(0), Vector_x); }
  else if(feat==FS_vectorXDiff) {  f=make_shared<TM_Default>(TMT_vecDiff, world, frames(0), Vector_x, frames(1), Vector_x); }
  else if(feat==FS_vectorXRel) {  f=make_shared<TM_Default>(TMT_vec, world, frames(0), Vector_x, frames(1), Vector_x); }

  else if(feat==FS_vectorY) {  f=make_shared<TM_Default>(TMT_vec, world, frames(0), Vector_y); }
  else if(feat==FS_vectorYDiff) {  f=make_shared<TM_Default>(TMT_vecDiff, world, frames(0), Vector_y, frames(1), Vector_y); }
  else if(feat==FS_vectorYRel) {  f=make_shared<TM_Default>(TMT_vec, world, frames(0), Vector_y, frames(1), Vector_y); }

  else if(feat==FS_vectorZ) {  f=make_shared<TM_Default>(TMT_vec, world, frames(0), Vector_z); }
  else if(feat==FS_vectorZDiff) {  f=make_shared<TM_Default>(TMT_vecDiff, world, frames(0), Vector_z, frames(1), Vector_z); }
  else if(feat==FS_vectorZRel) {  f=make_shared<TM_Default>(TMT_vec, world, frames(0), Vector_z, frames(1), Vector_z); }

  else if(feat==FS_quaternion) {  f=make_shared<TM_Default>(TMT_quat, world, frames(0)); }
  else if(feat==FS_quaternionDiff) {  f=make_shared<TM_Default>(TMT_quatDiff, world, frames(0), NoVector, frames(1), NoVector); }
  else if(feat==FS_quaternionRel) {  f=make_shared<TM_Default>(TMT_quat, world, frames(0), NoVector, frames(1), NoVector); }

  else if(feat==FS_pose) {  f=make_shared<TM_Default>(TMT_pose, world, frames(0)); }
  else if(feat==FS_poseDiff) {  f=make_shared<TM_Default>(TMT_poseDiff, world, frames(0), NoVector, frames(1), NoVector); }
  else if(feat==FS_poseRel)  {  f=make_shared<TM_Default>(TMT_pose, world, frames(0), NoVector, frames(1), NoVector); }

  else if(feat==FS_scalarProductXX) {  f=make_shared<TM_Default>(TMT_vecAlign, world, frames(0), Vector_x, frames(1), Vector_x); }
  else if(feat==FS_scalarProductXY) {  f=make_shared<TM_Default>(TMT_vecAlign, world, frames(0), Vector_x, frames(1), Vector_y); }
  else if(feat==FS_scalarProductXZ) {  f=make_shared<TM_Default>(TMT_vecAlign, world, frames(0), Vector_x, frames(1), Vector_z); }
  else if(feat==FS_scalarProductYX) {  f=make_shared<TM_Default>(TMT_vecAlign, world, frames(0), Vector_y, frames(1), Vector_x); }
  else if(feat==FS_scalarProductYY) {  f=make_shared<TM_Default>(TMT_vecAlign, world, frames(0), Vector_y, frames(1), Vector_y); }
  else if(feat==FS_scalarProductYZ) {  f=make_shared<TM_Default>(TMT_vecAlign, world, frames(0), Vector_y, frames(1), Vector_z); }
  else if(feat==FS_scalarProductZZ) {  f=make_shared<TM_Default>(TMT_vecAlign, world, frames(0), Vector_z, frames(1), Vector_z); }

  else if(feat==FS_gazeAt) { f=make_shared<TM_Default>(TMT_gazeAt, world, frames(0), NoVector, frames(1)); }

  else if(feat==FS_angularVel) { f=make_shared<TM_AngVel>(world, frames(0)); }

  else if(feat==FS_accumulatedCollisions) {  f=make_shared<TM_Proxy>(TMT_allP, uintA()); }
  else if(feat==FS_jointLimits) {  f=make_shared<LimitsConstraint>(.05); }

  else if(feat==FS_qItself) {
    if(!frames.N) f=make_shared<TM_qItself>();
    else f=make_shared<TM_qItself>(QIP_byJointNames, frames, world);
  }

  else if(feat==FS_physics) { f=make_shared<TM_NewtonEuler>(world, frames(0)); }
  else if(feat==FS_contactConstraints) { f=make_shared<TM_Contact_ForceIsNormal>(world, frames(0), frames(1)); }
  else if(feat==FS_energy) { f=make_shared<TM_Energy>(); }

  else if(feat==FS_transAccelerations) { f=make_shared<TM_Transition>(world); }
  else if(feat==FS_transVelocities) {
    auto map = make_shared<TM_Transition>(world);
    map->velCoeff = 1.;
    map->accCoeff = 0.;
    f = map;
  }
  else HALT("can't interpret feature symbols: " <<feat);

  if(!!scale) f->scale = scale;
  if(!!target) f->target = target;
  if(order>=0) f->order = order;

  return f;
}
