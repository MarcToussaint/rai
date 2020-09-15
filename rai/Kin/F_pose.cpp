/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_pose.h"
#include "TM_default.h"

//===========================================================================

void F_Position::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 1, "");
  rai::Frame *f = F.elem(0);
  f->C.kinematicsPos(y, J, f);
}

//===========================================================================

void F_PositionDiff::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame *f1 = F.elem(0);
  rai::Frame *f2 = F.elem(1);
  arr y2, J2;
  f1->C.kinematicsPos(y, J, f1);
  f2->C.kinematicsPos(y2, J2, f2);
  y -= y2;
  J -= J2;
}

//===========================================================================

void F_PositionRel::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame *f1 = F.elem(0);
  rai::Frame *f2 = F.elem(1);
  arr y1, y2, J1, J2;
  f1->C.kinematicsPos(y1, J1, f1);
  f2->C.kinematicsPos(y2, J2, f2);
  arr Rinv = ~(f2->ensure_X().rot.getArr());
  y = Rinv * (y1 - y2);
  if(!!J) {
    arr A;
    f2->C.jacobian_angular(A, f2);
    J = Rinv * (J1 - J2 - crossProduct(A, y1 - y2));
  }
}

//===========================================================================

void F_Vector::phi2(arr& y, arr& J, const FrameL& F){
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 1, "");
  rai::Frame *f = F.elem(0);
  f->C.kinematicsVec(y, J, f, vec);
}

//===========================================================================

void F_VectorDiff::phi2(arr& y, arr& J, const FrameL& F){
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame *f1 = F.elem(0);
  rai::Frame *f2 = F.elem(1);
  arr y2, J2;
  f1->C.kinematicsVec(y, J, f1, vec1);
  f2->C.kinematicsVec(y2, J2, f2, vec2);
  y -= y2;
  J -= J2;
}

//===========================================================================

void F_VectorRel::phi2(arr& y, arr& J, const FrameL& F){
  NIY;
}

//===========================================================================

void F_Quaternion::phi2(arr& y, arr& J, const FrameL& F){
  flipTargetSignOnNegScalarProduct = true;
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 1, "");
  rai::Frame *f = F.elem(0);

  f->C.kinematicsQuat(y, J, f);
}

//===========================================================================

void F_QuaternionDiff::phi2(arr& y, arr& J, const FrameL& F){
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame *f1 = F.elem(0);
  rai::Frame *f2 = F.elem(1);
  arr y2, J2;
  f1->C.kinematicsQuat(y, J, f1);
  f2->C.kinematicsQuat(y2, J2, f2);
  if(scalarProduct(y, y2)>=0.) {
    y -= y2;
    J -= J2;
  } else {
    y += y2;
    J += J2;
  }
}

//===========================================================================

void F_QuaternionRel::phi2(arr& y, arr& J, const FrameL& F){
  flipTargetSignOnNegScalarProduct = true;
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame *f1 = F.elem(0);
  rai::Frame *f2 = F.elem(1);

  arr qa, qb, Ja, Jb;
  f1->C.kinematicsQuat(qb, Jb, f1);
  f2->C.kinematicsQuat(qa, Ja, f2);

  arr Jya, Jyb;
  arr ainv = qa;
  if(qa(0)!=1.) ainv(0) *= -1.;
  quat_concat(y, Jya, Jyb, ainv, qb);
  if(qa(0)!=1.) for(uint i=0; i<Jya.d0; i++) Jya(i, 0) *= -1.;

  J = Jya * Ja + Jyb * Jb;
  checkNan(J);
}

//===========================================================================

void F_ScalarProduct::phi2(arr& y, arr& J, const FrameL& F){
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame *f1 = F.elem(0);
  rai::Frame *f2 = F.elem(1);

  CHECK(fabs(vec1.length()-1.)<1e-4, "vector references must be normalized");
  CHECK(fabs(vec2.length()-1.)<1e-4, "vector references must be normalized");

  arr zi, Ji, zj, Jj;
  f1->C.kinematicsVec(zi, Ji, f1, vec1);
  f2->C.kinematicsVec(zj, Jj, f2, vec2);

  y.resize(1);
  y(0) = scalarProduct(zi, zj);
  J = ~zj * Ji + ~zi * Jj;
}

//===========================================================================

void F_Pose::phi2(arr& y, arr& J, const FrameL& F) {
  auto pos = evalFeature<F_Position>(F, order);
  auto quat = evalFeature<F_Quaternion>(F, order);
  y.setBlockVector(pos.y, quat.y);
  J.setBlockMatrix(pos.J, quat.J);
}

//===========================================================================

void F_PoseDiff::phi2(arr& y, arr& J, const FrameL& F) {
  auto pos = evalFeature<F_PositionDiff>(F, order);
  auto quat = evalFeature<F_QuaternionDiff>(F, order);
  y.setBlockVector(pos.y, quat.y);
  J.setBlockMatrix(pos.J, quat.J);
}

//===========================================================================

void F_PoseRel::phi2(arr& y, arr& J, const FrameL& F) {
  auto pos = evalFeature<F_PositionRel>(F, order);
  auto quat = evalFeature<F_QuaternionRel>(F, order);
  y.setBlockVector(pos.y, quat.y);
  J.setBlockMatrix(pos.J, quat.J);
}

//===========================================================================

TM_Align::TM_Align(const rai::Configuration& K, const char* iName, const char* jName)
  : i(-1), j(-1) {
  rai::Frame* a = iName ? K.getFrameByName(iName):nullptr;
  rai::Frame* b = jName ? K.getFrameByName(jName):nullptr;
  if(a) i=a->ID;
  if(b) j=b->ID;
}

void TM_Align::phi(arr& y, arr& J, const rai::Configuration& K) {
  y.resize(3);
  if(!!J) J.resize(3, K.q.N);

  rai::Frame* body_i = K.frames(i);
  rai::Frame* body_j = K.frames(j);

  arr zi, Ji, zj, Jj;

  K.kinematicsVec(zi, Ji, body_i, Vector_z);
  K.kinematicsVec(zj, Jj, body_j, Vector_x);
  y(0) = scalarProduct(zi, zj);
  if(!!J) J[0] = ~zj * Ji + ~zi * Jj;

  K.kinematicsVec(zi, Ji, body_i, Vector_z);
  K.kinematicsVec(zj, Jj, body_j, Vector_y);
  y(1) = scalarProduct(zi, zj);
  if(!!J) J[1] = ~zj * Ji + ~zi * Jj;

  K.kinematicsVec(zi, Ji, body_i, Vector_y);
  K.kinematicsVec(zj, Jj, body_j, Vector_x);
  y(2) = scalarProduct(zi, zj);
  if(!!J) J[2] = ~zj * Ji + ~zi * Jj;
}





