/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "featureSymbols.h"
#include "F_pose.h"
#include "TM_default.h"
#include "TM_proxy.h"
#include "F_qFeatures.h"
#include "F_PairCollision.h"
#include "F_dynamics.h"
#include "F_contacts.h"
#include "TM_angVel.h"
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
  nullptr
};

//fwd declarations
auto getQFramesAndScale(const rai::Configuration& C) {
  struct Return { uintA frames; arr scale; } R;
  for(rai::Frame* f : C.frames) {
    if(f->joint && f->joint->active && f->joint->dim>0 && f->joint->H>0. && f->joint->type!=rai::JT_tau) {
      CHECK(!f->joint->mimic, "");
      R.frames.append(TUP(f->ID, f->parent->ID));
      R.scale.append(f->joint->H, f->joint->dim);
    }
  }
  R.frames.reshape(-1, 2);
  //  cout <<scale <<endl <<world.getHmetric() <<endl;
  return R;
}

double shapeSize(const rai::Configuration& K, const char* name, uint i=2) {
  rai::Frame* f = K.getFrameByName(name);
  rai::Shape* s = f->shape;
  if(!s) {
    for(rai::Frame* b:f->children) if(b->name==name && b->shape) { s=b->shape; break; }
  }
  if(!s) return 0;
  return s->size(i);
}

ptr<Feature> symbols2feature(FeatureSymbol feat, const StringA& frames, const rai::Configuration& C, const arr& scale, const arr& target, int order) {
  shared_ptr<Feature> f;
  if(feat==FS_distance) {  f=make_shared<F_PairCollision>(C, frames(0), frames(1), F_PairCollision::_negScalar, false); }
  else if(feat==FS_oppose) {  f=make_shared<F_GraspOppose>(C, frames(0), frames(1), frames(2)); }
  else if(feat==FS_aboveBox) {  f=make_shared<TM_AboveBox>(C, frames(1), frames(0), .0); }
  else if(feat==FS_standingAbove) {
    double h = .5*(shapeSize(C, frames(0)) + shapeSize(C, frames(1)));
    f = make_shared<TM_Default>(TMT_posDiff, C, frames(0), rai::Vector(0., 0., h), frames(1), NoVector);
    f->scale = arr({1, 3}, {0., 0., 1.});
  }

#ifdef RAI_NEW_FEATURES
  else if(feat==FS_position) {  f = make_shared<F_Position>();  } //f=make_shared<TM_Default>(TMT_pos, C, frames(0)); }
  else if(feat==FS_positionDiff) {  f = make_shared<F_PositionDiff>();  } //f=make_shared<TM_Default>(TMT_posDiff, C, frames(0), NoVector, frames(1)); }
  else if(feat==FS_positionRel) {  f=make_shared<TM_Default>(TMT_pos, C, frames(0), NoVector, frames(1)); }

  else if(feat==FS_quaternion) {  f=make_shared<F_Quaternion>(); }
  else if(feat==FS_quaternionDiff) {  f=make_shared<F_QuaternionDiff>(); }
  else if(feat==FS_quaternionRel) {  f=make_shared<TM_Default>(TMT_quat, C, frames(0), NoVector, frames(1), NoVector); }
#else
  else if(feat==FS_position) {  f = make_shared<F_Position>();  } //f=make_shared<TM_Default>(TMT_pos, C, frames(0)); }
  else if(feat==FS_positionDiff) {  f = make_shared<F_PositionDiff>();  } //f=make_shared<TM_Default>(TMT_posDiff, C, frames(0), NoVector, frames(1)); }
//  else if(feat==FS_position) {  f=make_shared<TM_Default>(TMT_pos, C, frames(0)); }
//  else if(feat==FS_positionDiff) {  f=make_shared<TM_Default>(TMT_posDiff, C, frames(0), NoVector, frames(1)); }
  else if(feat==FS_positionRel) {  f=make_shared<TM_Default>(TMT_pos, C, frames(0), NoVector, frames(1)); }

  else if(feat==FS_quaternion) {  f=make_shared<TM_Default>(TMT_quat, C, frames(0)); }
  else if(feat==FS_quaternionDiff) {  f=make_shared<TM_Default>(TMT_quatDiff, C, frames(0), NoVector, frames(1), NoVector); }
  else if(feat==FS_quaternionRel) {  f=make_shared<TM_Default>(TMT_quat, C, frames(0), NoVector, frames(1), NoVector); }
#endif

  else if(feat==FS_pose) {  f=make_shared<F_Pose>(C, frames(0)); }
  else if(feat==FS_poseDiff) {  f=make_shared<F_PoseDiff>(C, frames(0), frames(1)); }
  else if(feat==FS_poseRel)  {  f=make_shared<F_PoseRel>(C, frames(0), frames(1)); }

  else if(feat==FS_vectorX) {  f=make_shared<TM_Default>(TMT_vec, C, frames(0), Vector_x); }
  else if(feat==FS_vectorXDiff) {  f=make_shared<TM_Default>(TMT_vecDiff, C, frames(0), Vector_x, frames(1), Vector_x); }
  else if(feat==FS_vectorXRel) {  f=make_shared<TM_Default>(TMT_vec, C, frames(0), Vector_x, frames(1), Vector_x); }

  else if(feat==FS_vectorY) {  f=make_shared<TM_Default>(TMT_vec, C, frames(0), Vector_y); }
  else if(feat==FS_vectorYDiff) {  f=make_shared<TM_Default>(TMT_vecDiff, C, frames(0), Vector_y, frames(1), Vector_y); }
  else if(feat==FS_vectorYRel) {  f=make_shared<TM_Default>(TMT_vec, C, frames(0), Vector_y, frames(1), Vector_y); }

  else if(feat==FS_vectorZ) {  f=make_shared<TM_Default>(TMT_vec, C, frames(0), Vector_z); }
  else if(feat==FS_vectorZDiff) {  f=make_shared<TM_Default>(TMT_vecDiff, C, frames(0), Vector_z, frames(1), Vector_z); }
  else if(feat==FS_vectorZRel) {  f=make_shared<TM_Default>(TMT_vec, C, frames(0), Vector_z, frames(1), Vector_z); }


  else if(feat==FS_scalarProductXX) {  f=make_shared<TM_Default>(TMT_vecAlign, C, frames(0), Vector_x, frames(1), Vector_x); }
  else if(feat==FS_scalarProductXY) {  f=make_shared<TM_Default>(TMT_vecAlign, C, frames(0), Vector_x, frames(1), Vector_y); }
  else if(feat==FS_scalarProductXZ) {  f=make_shared<TM_Default>(TMT_vecAlign, C, frames(0), Vector_x, frames(1), Vector_z); }
  else if(feat==FS_scalarProductYX) {  f=make_shared<TM_Default>(TMT_vecAlign, C, frames(0), Vector_y, frames(1), Vector_x); }
  else if(feat==FS_scalarProductYY) {  f=make_shared<TM_Default>(TMT_vecAlign, C, frames(0), Vector_y, frames(1), Vector_y); }
  else if(feat==FS_scalarProductYZ) {  f=make_shared<TM_Default>(TMT_vecAlign, C, frames(0), Vector_y, frames(1), Vector_z); }
  else if(feat==FS_scalarProductZZ) {  f=make_shared<TM_Default>(TMT_vecAlign, C, frames(0), Vector_z, frames(1), Vector_z); }

  else if(feat==FS_pairCollision_negScalar) {  f=make_shared<F_PairCollision>(C, frames(0), frames(1), F_PairCollision::_negScalar, false); }
  else if(feat==FS_pairCollision_vector) {     f=make_shared<F_PairCollision>(C, frames(0), frames(1), F_PairCollision::_vector, false); }
  else if(feat==FS_pairCollision_normal) {     f=make_shared<F_PairCollision>(C, frames(0), frames(1), F_PairCollision::_normal, true); }
  else if(feat==FS_pairCollision_p1) {         f=make_shared<F_PairCollision>(C, frames(0), frames(1), F_PairCollision::_p1, false); }
  else if(feat==FS_pairCollision_p2) {         f=make_shared<F_PairCollision>(C, frames(0), frames(1), F_PairCollision::_p2, false); }

  else if(feat==FS_gazeAt) { f=make_shared<TM_Default>(TMT_gazeAt, C, frames(0), NoVector, frames(1)); }

  else if(feat==FS_angularVel) { f=make_shared<TM_AngVel>(C, frames(0)); }

  else if(feat==FS_accumulatedCollisions) {
    if(frames.N) f=make_shared<TM_Proxy>(TMT_allP, stringListToFrameIndices(frames, C));
    else f=make_shared<TM_Proxy>(TMT_allP, framesToIndices(C.frames));
  }
  else if(feat==FS_jointLimits) {  f=make_shared<F_qLimits>(); }

  else if(feat==FS_qItself) {
#ifdef RAI_NEW_FEATURES
    if(!frames.N) f=make_shared<F_qItself>(F_qItself::allActiveJoints, frames, C);
#else
    if(!frames.N) f=make_shared<F_qItself>();
#endif
    else f=make_shared<F_qItself>(F_qItself::byJointNames, frames, C);
  }

  else if(feat==FS_qControl) {
    CHECK(!frames.N, "NIY");
    auto F = getQFramesAndScale(C);
    f = make_shared<F_qItself>(F.frames);
    f->scale = F.scale;
  }

  else if(feat==FS_physics) { f=make_shared<F_NewtonEuler>(C, frames(0)); }
  else if(feat==FS_contactConstraints) { f=make_shared<TM_Contact_ForceIsNormal>(C, frames(0), frames(1)); }
  else if(feat==FS_energy) { f=make_shared<F_Energy>(); }

  else if(feat==FS_transAccelerations) { HALT("obsolete"); /*f=make_shared<TM_Transition>(world);*/ }
  else if(feat==FS_transVelocities) {
    HALT("obsolete");
//    auto map = make_shared<TM_Transition>(world);
//    map->velCoeff = 1.;
//    map->accCoeff = 0.;
//    f = map;
  } else HALT("can't interpret feature symbols: " <<feat);

  if(!!scale) {
    if(!f->scale.N) f->scale = scale;
    else if(scale.N==1) f->scale *= scale.scalar();
    else if(scale.N==f->scale.N) f->scale *= scale.scalar();
    else NIY;
  }
  if(!!target) f->target = target;
  if(order>=0) f->order = order;

  f->fs = feat;

  if(!f->frameIDs.N) f->frameIDs = stringListToFrameIndices(frames, C);

  return f;
}
