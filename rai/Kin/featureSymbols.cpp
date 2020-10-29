/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "featureSymbols.h"
#include "F_pose.h"
#include "F_collisions.h"
#include "F_qFeatures.h"
#include "F_forces.h"
#include "F_geometrics.h"

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
  "scalarProductXX",
  "scalarProductXY",
  "scalarProductXZ",
  "scalarProductYX",
  "scalarProductYY",
  "scalarProductYZ",
  "scalarProductZZ",
  "gazeAt",

  "angularVel",

  "accumulatedCollisions",
  "jointLimits",
  "distance",
  "oppose",

  "qItself",
  "qControl",

  "aboveBox",
  "insideBox",

  "pairCollision_negScalar",
  "pairCollision_vector",
  "pairCollision_normal",
  "pairCollision_p1",
  "pairCollision_p2",

  "standingAbove",

  "physics",
  "contactConstraints",
  "energy",

  "transAccelerations",
  "transVelocities",

  "qQuaternionNorms",
  nullptr
};

//fwd declarations
auto getCtrlFramesAndScale(const rai::Configuration& C) {
  struct Return { uintA frames; arr scale; } R;
  R.frames = C.getCtrlFramesAndScale(R.scale);
  return R;
}

double shapeSize(const rai::Configuration& K, const char* name, uint i=2) {
  rai::Frame* f = K.getFrame(name);
  rai::Shape* s = f->shape;
  if(!s) {
    for(rai::Frame* b:f->children) if(b->name==name && b->shape) { s=b->shape; break; }
  }
  if(!s) return 0;
  return s->size(i);
}

ptr<Feature> symbols2feature(FeatureSymbol feat, const StringA& frames, const rai::Configuration& C, const arr& scale, const arr& target, int order) {
  shared_ptr<Feature> f;
  if(feat==FS_distance) {  f=make_shared<F_PairCollision>(F_PairCollision::_negScalar, false); }
  else if(feat==FS_oppose) {  f=make_shared<F_GraspOppose>(); }
  else if(feat==FS_aboveBox) {  f=make_shared<F_AboveBox>(); }
  else if(feat==FS_insideBox) {  f=make_shared<F_InsideBox>(); }
  else if(feat==FS_standingAbove) {
//    double h = .5*(shapeSize(C, frames(0)) + shapeSize(C, frames(1)));
//    f = make_shared<TM_Default>(TMT_posDiff, C, frames(0), rai::Vector(0., 0., h), frames(1), NoVector);
    NIY;
    f->scale = arr({1, 3}, {0., 0., 1.});
  }

  else if(feat==FS_position) {  f = make_shared<F_Position>();  } //f=make_shared<TM_Default>(TMT_pos, C, frames(0)); }
  else if(feat==FS_positionDiff) {  f = make_shared<F_PositionDiff>();  } //f=make_shared<TM_Default>(TMT_posDiff, C, frames(0), NoVector, frames(1)); }
  else if(feat==FS_positionRel) {  f=make_shared<F_PositionRel>(); }

  else if(feat==FS_vectorX) {  f=make_shared<F_Vector>(Vector_x); }
  else if(feat==FS_vectorY) {  f=make_shared<F_Vector>(Vector_y); }
  else if(feat==FS_vectorZ) {  f=make_shared<F_Vector>(Vector_z); }

  else if(feat==FS_scalarProductXX) {  f=make_shared<F_ScalarProduct>(Vector_x, Vector_x); }
  else if(feat==FS_scalarProductXY) {  f=make_shared<F_ScalarProduct>(Vector_x, Vector_y); }
  else if(feat==FS_scalarProductXZ) {  f=make_shared<F_ScalarProduct>(Vector_x, Vector_z); }
  else if(feat==FS_scalarProductYX) {  f=make_shared<F_ScalarProduct>(Vector_y, Vector_x); }
  else if(feat==FS_scalarProductYY) {  f=make_shared<F_ScalarProduct>(Vector_y, Vector_y); }
  else if(feat==FS_scalarProductYZ) {  f=make_shared<F_ScalarProduct>(Vector_y, Vector_z); }
  else if(feat==FS_scalarProductZZ) {  f=make_shared<F_ScalarProduct>(Vector_z, Vector_z); }

  else if(feat==FS_quaternion) {  f=make_shared<F_Quaternion>(); }
  else if(feat==FS_quaternionDiff) {  f=make_shared<F_QuaternionDiff>(); }
  else if(feat==FS_quaternionRel) {  f=make_shared<F_QuaternionRel>(); }

  else if(feat==FS_pose) {  f=make_shared<F_Pose>(); }
  else if(feat==FS_poseDiff) {  f=make_shared<F_PoseDiff>(); }
  else if(feat==FS_poseRel)  {  f=make_shared<F_PoseRel>(); }

  else if(feat==FS_vectorXDiff) {  f=make_shared<F_VectorDiff>(Vector_x, Vector_x); }
  else if(feat==FS_vectorXRel) {  f=make_shared<F_VectorRel>(Vector_x); }

  else if(feat==FS_vectorYDiff) {  f=make_shared<F_VectorDiff>(Vector_y, Vector_y); }
  else if(feat==FS_vectorYRel) {  f=make_shared<F_VectorRel>(Vector_y); }

  else if(feat==FS_vectorZDiff) {  f=make_shared<F_VectorDiff>(Vector_z, Vector_z); }
  else if(feat==FS_vectorZRel) {  f=make_shared<F_VectorRel>(Vector_z); }


  else if(feat==FS_pairCollision_negScalar) {  f=make_shared<F_PairCollision>(F_PairCollision::_negScalar, false); }
  else if(feat==FS_pairCollision_vector) {     f=make_shared<F_PairCollision>(F_PairCollision::_vector, false); }
  else if(feat==FS_pairCollision_normal) {     f=make_shared<F_PairCollision>(F_PairCollision::_normal, true); }
  else if(feat==FS_pairCollision_p1) {         f=make_shared<F_PairCollision>(F_PairCollision::_p1, false); }
  else if(feat==FS_pairCollision_p2) {         f=make_shared<F_PairCollision>(F_PairCollision::_p2, false); }

  else if(feat==FS_gazeAt) {
    f=make_shared<F_PositionRel>();
    f->scale = arr({2,3}, {1., 0., 0., 0., 1., 0.}); //pick the xy- coordinated
  }

  else if(feat==FS_angularVel) { f=make_shared<F_AngVel>(); }

  else if(feat==FS_accumulatedCollisions) {
    f=make_shared<F_AccumulatedCollisions>();
    if(!frames.N) f->frameIDs = framesToIndices(C.frames);
  }
  else if(feat==FS_jointLimits) {
    f=make_shared<F_qLimits2>();
    if(!frames.N) f->frameIDs = framesToIndices(C.frames);
  }

  else if(feat==FS_qItself) {
    if(!frames.N) f=make_shared<F_qItself>(F_qItself::allActiveJoints, frames, C);
    else f=make_shared<F_qItself>();
  }

  else if(feat==FS_qControl) {
    CHECK(!frames.N, "NIY");
    arr _scale;
    uintA F = C.getCtrlFramesAndScale(_scale);
    f = make_shared<F_qItself>(F);
    f->setScale(_scale);
  }

  else if(feat==FS_physics) { f=make_shared<F_NewtonEuler>(); }
  else if(feat==FS_contactConstraints) { f=make_shared<F_fex_ForceIsNormal>(); }
  else if(feat==FS_energy) { f=make_shared<F_Energy>(); }

  else if(feat==FS_transAccelerations) { HALT("obsolete"); /*f=make_shared<TM_Transition>(world);*/ }
  else if(feat==FS_transVelocities) {
    HALT("obsolete");
//    auto map = make_shared<TM_Transition>(world);
//    map->velCoeff = 1.;
//    map->accCoeff = 0.;
//    f = map;
  }
  else if(feat==FS_qQuaternionNorms) {
    f = make_shared<F_qQuaternionNorms>();
    for(auto *j:C.activeJoints) if(j->type==rai::JT_quatBall || j->type==rai::JT_free || j->type==rai::JT_rigid) f->frameIDs.append(j->frame->ID);
  }

  else HALT("can't interpret feature symbols: " <<feat);

//  if(!f->frameIDs.N) f->frameIDs = C.getFrameIDs(frames);
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

  f->fs = feat;

  return f;
}
