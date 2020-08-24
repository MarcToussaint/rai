/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_ImpulseExchange.h"
#include "TM_default.h"
#include "F_PairCollision.h"

void TM_ImpulsExchange::phi(arr& y, arr& J, const ConfigurationL& Ktuple) {
  CHECK_GE(Ktuple.N, 3, "");
  CHECK_GE(order, 2, "");

  arr a1, J1, a2, J2, v1, Jv1, v2, Jv2;

  //acceleration (=impulse change) of object 1
  TM_Default pos1(TMT_pos, i);
  pos1.order=2;
  pos1.Feature::__phi(a1, (!!J?J1:NoArr), Ktuple);

//  {
//    rai::Configuration &K = *Ktuple.last();
//    rai::Frame *a = K(i)->getUpwardLink();
//    if(a->flags && a->flags & (1<<FL_kinematic)){
//      pos1.order=1;
//      pos1.Feature::__phi(a1, (!!J?J1:NoArr), Ktuple);
//      a1 *= -1.;
//      if(!!J) J1 *= -1.;
//    }
//  }

  //acceleration (=impulse change) of object 2
  TM_Default pos2(TMT_pos, j);
  pos2.order=2;
  pos2.Feature::__phi(a2, (!!J?J2:NoArr), Ktuple);

  //projection matrix onto 'table' to which object 2 will be attached
  arr P;
  {
    rai::Configuration& K = *Ktuple.last();
    rai::Frame* b = K(j)->getUpwardLink();
    if(b->joint && b->joint->type==rai::JT_transXYPhi) {
      arr R = b->joint->X().rot.getArr();
      arr j1=R[0], j2=R[1];
      P = (j1^j1) + (j2^j2);
    }
  }

  //first constraint: R = m1 dv1 = - m2 dv2
  y = a1+a2;
  if(!!J) J = J1+J2;

  if(P.N) {
    y = P*y;
    if(!!J) J = P*J;
  }

  arr c, Jc;
  F_PairCollision coll(i, j, F_PairCollision::_vector, true);
  coll.phi(c, (!!J?Jc:NoArr), *Ktuple(-2));
  uintA qdim = getKtupleDim(Ktuple);
  arr Jcc = zeros(3, qdim.last());
  if(!!J) Jcc.setMatrixBlock(Jc, 0, qdim(0));

#if 1
  double z=.5;
  arr R  = a2-z*a1;
  arr JR = J2-z*J1;
  if(sumOfSqr(c)>1e-16) {

    // R is || to c
#if 1
    normalizeWithJac(c, Jcc);
    double sign = +1.;
    if(scalarProduct(c, R)>0.) sign = -1.; //HMM is that sign not bad?
    y.append(R - c*sign*scalarProduct(c, R));
    if(!!J) J.append(JR - sign*(c*~c*JR + c*~R*Jcc + scalarProduct(c, R)*Jcc));
#else
    normalizeWithJac(c, Jcc);
    normalizeWithJac(R, JR);
    y.append(c + R);
    if(!!J) J.append(Jcc + JR);
#endif

    // R is pointing exactly in the direction of c (redundant with above! But I need the inequality constraint R^T c > 0 here!!)
    //fully inelastic:
//    y.append(scalarProduct(c, v2-v1));
//    if(!!J) J.append(~c*(Jv2-Jv1) + ~(v2-v1)*Jcc);

//    normalizeWithJac(R, JR);
//    y.append(scalarProduct(c,R) - 1.);
//    if(!!J) J.append(~c*JR + ~R*Jcc);
  } else {
    y.append(zeros(3));
    if(!!J) J.append(zeros(3, JR.d1));
  }
#else
  arr d=a2-a1;
  arr Jd=J2-J1;
  if(sumOfSqr(d)>1e-16 && sumOfSqr(c)>1e-16) {
    normalizeWithJac(d, Jd);
    normalizeWithJac(c, Jcc);
    y.append(d + c);
    if(!!J) J.append(Jd + Jcc);
  } else {
    d = 1.;
    Jd.setZero();
    y.append(d);
    if(!!J) J.append(Jd);
  }
#endif

  checkNan(y);
  if(!!J) checkNan(J);
  CHECK_EQ(y.N, dim_phi(*Ktuple.last()), "");
  if(!!J) CHECK_EQ(J.d0, y.N, "");
}

void TM_ImpulsExchange_weak::phi(arr& y, arr& J, const ConfigurationL& Ktuple) {
  CHECK_GE(Ktuple.N, 3, "");
  CHECK_GE(order, 2, "");

  arr a1, J1, a2, J2;

  TM_Default pos1(TMT_pos, i);
  pos1.order=2;
  pos1.Feature::__phi(a1, (!!J?J1:NoArr), Ktuple);

  TM_Default pos2(TMT_pos, j);
  pos2.order=2;
  pos2.Feature::__phi(a2, (!!J?J2:NoArr), Ktuple);

  arr c, Jc;
  F_PairCollision coll(i, j, F_PairCollision::_vector, true);
  coll.phi(c, (!!J?Jc:NoArr), *Ktuple(-2));
  uintA qdim = getKtupleDim(Ktuple);
  arr Jcc = zeros(3, qdim.last());
  if(!!J) Jcc.setMatrixBlock(Jc, 0, qdim(0));

  y = ARR(1., 1., 1.);
  if(!!J) J = zeros(3, J1.d1);

  arr d=a2-a1;
  arr Jd=J2-J1;
  if(sumOfSqr(d)>1e-16 && sumOfSqr(c)>1e-16) {
    normalizeWithJac(d, Jd);
    normalizeWithJac(c, Jcc);
    y = d + c;
    if(!!J) J = Jd + Jcc;
  }

  checkNan(y);
  if(!!J) checkNan(J);
  CHECK_EQ(y.N, dim_phi(*Ktuple.last()), "");
  if(!!J) CHECK_EQ(J.d0, y.N, "");
}
