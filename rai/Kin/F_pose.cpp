/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_pose.h"
#include "TM_default.h"

//===========================================================================

void F_Pose::phi(arr& y, arr& J, const ConfigurationL& Ctuple) {
#if 1
  arr yq, Jq, yp, Jp;
  TM_Default tmp(TMT_pos, a);
  tmp.order = order;
  tmp.type = TMT_pos;
  tmp.Feature::__phi(yp, (!!J?Jp:NoArr), Ctuple);
  tmp.type = TMT_quat;
  tmp.flipTargetSignOnNegScalarProduct=true;
  tmp.Feature::__phi(yq, (!!J?Jq:NoArr), Ctuple);
  y.resize(yp.N+yq.N);
  y.setVectorBlock(yp, 0);
  y.setVectorBlock(yq, 3);
  if(!!J) {
    J.resize(y.N, Jp.d1);
    J.setMatrixBlock(Jp, 0, 0);
    J.setMatrixBlock(Jq, 3, 0);
  }
#else //should be identical
  if(order==2) {
    arr p0, p1, p2, J0, J1, J2;
    Ctuple(-3)->kinematicsPos(p0, J0, Ctuple(-3)->frames(a));  expandJacobian(J0, Ctuple, -3);
    Ctuple(-2)->kinematicsPos(p1, J1, Ctuple(-2)->frames(a));  expandJacobian(J1, Ctuple, -2);
    Ctuple(-1)->kinematicsPos(p2, J2, Ctuple(-1)->frames(a));  expandJacobian(J2, Ctuple, -1);

    y = p0 - 2.*p1 + p2;
    if(!!J) J = J0 - 2.*J1 + J2;

    arr q0, q1, q2; //, J0, J1, J2;
    Ctuple(-3)->kinematicsQuat(q0, J0, Ctuple(-3)->frames(a));  expandJacobian(J0, Ctuple, -3);
    Ctuple(-2)->kinematicsQuat(q1, J1, Ctuple(-2)->frames(a));  expandJacobian(J1, Ctuple, -2);
    Ctuple(-1)->kinematicsQuat(q2, J2, Ctuple(-1)->frames(a));  expandJacobian(J2, Ctuple, -1);

    if(scalarProduct(q0, q1)<-0.) { q0*=-1.; J0*=-1.; }
    if(scalarProduct(q2, q1)<-0.) { q2*=-1.; J2*=-1.; }

    arr yq = q0 - 2.*q1 + q2;
    arr Jq = J0 - 2.*J1 + J2;

    double tau = Ctuple(-2)->frames(0)->tau;
    tau=1.;
    if(tau) {
      CHECK_GE(tau, 1e-10, "");
      yq /= tau*tau;
      if(!!J) Jq /= tau*tau;
    }

  } else if(order==1) {
    arr p0, p1, J0, J1;
    Ctuple(-2)->kinematicsPos(p0, J0, Ctuple(-2)->frames(a));  expandJacobian(J0, Ctuple, -2);
    Ctuple(-1)->kinematicsPos(p1, J1, Ctuple(-1)->frames(a));  expandJacobian(J1, Ctuple, -1);

    y = p1 - p0;
    if(!!J) J = J1 - J0;

    arr q0, q1; //, J0, J1, J2;
    Ctuple(-2)->kinematicsQuat(q0, J0, Ctuple(-2)->frames(a));  expandJacobian(J0, Ctuple, -2);
    Ctuple(-1)->kinematicsQuat(q1, J1, Ctuple(-1)->frames(a));  expandJacobian(J1, Ctuple, -1);

    if(scalarProduct(q0, q1)<-0.) { q0*=-1.; J0*=-1.; }

    arr yq = q1 - q0;
    arr Jq = J1 - J0;

    y.append(yq);
    if(!!J) J.append(Jq);

    double tau = Ctuple(-2)->frames(0)->tau;
    if(tau) {
      CHECK_GE(tau, 1e-10, "");
      y /= tau;
      if(!!J) J /= tau;
    }
  }
#endif
}

//===========================================================================

void F_PoseDiff::phi(arr& y, arr& J, const ConfigurationL& Ctuple) {
  arr yq, Jq, yp, Jp;
  TM_Default tmp(TMT_posDiff, a, NoVector, b, NoVector);
  tmp.order = order;
  tmp.type = TMT_posDiff;
  tmp.Feature::__phi(yp, (!!J?Jp:NoArr), Ctuple);
  tmp.type = TMT_quatDiff;
  tmp.flipTargetSignOnNegScalarProduct=true;
  tmp.Feature::__phi(yq, (!!J?Jq:NoArr), Ctuple);
  y.resize(yp.N+yq.N);
  y.setVectorBlock(yp, 0);
  y.setVectorBlock(yq, 3);
  if(!!J) {
    J.resize(y.N, Jp.d1);
    J.setMatrixBlock(Jp, 0, 0);
    J.setMatrixBlock(Jq, 3, 0);
  }
}

//===========================================================================

void F_PoseRel::phi(arr& y, arr& J, const ConfigurationL& Ctuple) {
  arr yq, Jq, yp, Jp;
  TM_Default tmp(TMT_pos, a, NoVector, b, NoVector);
  tmp.order = order;
  tmp.type = TMT_pos;
  tmp.Feature::__phi(yp, (!!J?Jp:NoArr), Ctuple);
  tmp.type = TMT_quat;
  tmp.flipTargetSignOnNegScalarProduct=true;
  tmp.Feature::__phi(yq, (!!J?Jq:NoArr), Ctuple);
  y.resize(yp.N+yq.N);
  y.setVectorBlock(yp, 0);
  y.setVectorBlock(yq, 3);
  if(!!J) {
    J.resize(y.N, Jp.d1);
    J.setMatrixBlock(Jp, 0, 0);
    J.setMatrixBlock(Jq, 3, 0);
  }
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
