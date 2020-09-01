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
  rai::Vector p = f->ensure_X().pos;
  y = p.getArr();
  f->C.jacobian_pos(J, f, p);
}

//===========================================================================

void F_PositionDiff::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame *f1 = F.elem(0);
  rai::Frame *f2 = F.elem(1);

  rai::Vector p1 = f1->ensure_X().pos;
  rai::Vector p2 = f2->ensure_X().pos;
  y = (p1-p2).getArr();
  arr J2;
  f1->C.jacobian_pos(J, f1, p1);
  f2->C.jacobian_pos(J2, f2, p2);
  J -= J2;
}

//===========================================================================

void F_Quaternion::phi2(arr& y, arr& J, const FrameL& F){
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 1, "");
  rai::Frame *f = F.elem(0);

  f->C.kinematicsQuat(y, J, f);
}

//===========================================================================

void F_QuaternionDiff::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame *f1 = F.elem(0);
  rai::Frame *f2 = F.elem(1);

  arr y2, J2;
  f1->C.kinematicsQuat(y, J, f1);
  f2->C.kinematicsQuat(y2, J2, f2);

  if(scalarProduct(y, y2)>=0.) {
    y -= y2;
    if(!!J) J -= J2;
  } else {
    y += y2;
    if(!!J) J += J2;
  }
}

//===========================================================================

void F_Pose::phi(arr& y, arr& J, const ConfigurationL& Ctuple) {
#if 0
  arr yq, Jq, yp, Jp;
  TM_Default tmp(TMT_pos, a);
  tmp.order = order;
  tmp.type = TMT_pos;
  tmp.Feature::__phi(yp, Jp, Ctuple);
  tmp.type = TMT_quat;
  tmp.flipTargetSignOnNegScalarProduct=true;
  tmp.Feature::__phi(yq, Jq, Ctuple);

  y.setBlockVector(yp, yq);
  J.setBlockMatrix(Jp, Jq);
#else
  auto pos = F_Position({a}, order).eval(Ctuple);
  auto quat = F_Quaternion({a}, order).eval(Ctuple);
  y.setBlockVector(pos.y, quat.y);
  J.setBlockMatrix(pos.J, quat.J);
#endif

}

//===========================================================================

void F_PoseDiff::phi(arr& y, arr& J, const ConfigurationL& Ctuple) {
#if 0
  arr yq, Jq, yp, Jp;
  TM_Default tmp(TMT_posDiff, a, NoVector, b, NoVector);
  tmp.order = order;
  tmp.type = TMT_posDiff;
  tmp.Feature::__phi(yp, Jp, Ctuple);
  tmp.type = TMT_quatDiff;
  tmp.flipTargetSignOnNegScalarProduct=true;
  tmp.Feature::__phi(yq, Jq, Ctuple);

  y.setBlockVector(yp, yq);
  J.setBlockMatrix(Jp, Jq);
#else
  auto pos = F_PositionDiff({a,b}, order).eval(Ctuple);
  auto quat = F_QuaternionDiff({a,b}, order).eval(Ctuple);
  y.setBlockVector(pos.y, quat.y);
  J.setBlockMatrix(pos.J, quat.J);
#endif
}

//===========================================================================

void F_PoseRel::phi(arr& y, arr& J, const ConfigurationL& Ctuple) {
  arr yq, Jq, yp, Jp;
  TM_Default tmp(TMT_pos, a, NoVector, b, NoVector);
  tmp.order = order;
  tmp.type = TMT_pos;
  tmp.Feature::__phi(yp, Jp, Ctuple);
  tmp.type = TMT_quat;
  tmp.flipTargetSignOnNegScalarProduct=true;
  tmp.Feature::__phi(yq, Jq, Ctuple);

  y.setBlockVector(yp, yq);
  J.setBlockMatrix(Jp, Jq);
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

rai::String TM_Align::shortTag(const rai::Configuration& G) {
  return STRING("TM_Align:"<<(i<0?"WORLD":G.frames(i)->name) <<':' <<(j<0?"WORLD":G.frames(j)->name));
}
