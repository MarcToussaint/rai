#include "TM_ImpulseExchange.h"
#include "taskMap_default.h"
#include "taskMap_PairCollision.h"

void TM_ImpulsExchange::phi(arr &y, arr &J, const WorldL &Ktuple, double tau, int t){
  CHECK(Ktuple.N>=3, "");
  CHECK(order>=2,"");

  arr a1, J1, a2, J2;

  TaskMap_Default pos1(posTMT, i);
  pos1.order=2;
  pos1.TaskMap::phi(a1, (&J?J1:NoArr), Ktuple, tau, t);

  TaskMap_Default pos2(posTMT, j);
  pos2.order=2;
  pos2.TaskMap::phi(a2, (&J?J2:NoArr), Ktuple, tau, t);

  y = a1+a2;
  if(&J) J = J1+J2;

  arr c,Jc;
  TaskMap_PairCollision coll(i, j, false, true);
  coll.phi(c, (&J?Jc:NoArr), *Ktuple(-2), t);
  uintA qdim = getKtupleDim(Ktuple);
  arr Jcc = zeros(3, qdim.last());
  if(&J) Jcc.setMatrixBlock(Jc, 0, qdim(0));

  arr d=a2-a1;
  arr Jd=J2-J1;
  if(sumOfSqr(d)>1e-16 && sumOfSqr(c)>1e-16){
    normalizeWithJac(d, Jd);
    normalizeWithJac(c, Jcc);
    y.append(d + c);
    if(&J) J.append(Jd + Jcc);
  }else{
    d = 1.;
    Jd.setZero();
    y.append(d);
    if(&J) J.append(Jd);
  }
  checkNan(y);
  if(&J) checkNan(J);
  CHECK_EQ(y.N, dim_phi(*Ktuple.last()), "");
  if(&J) CHECK_EQ(J.d0, y.N, "");
}

void TM_ImpulsExchange_weak::phi(arr &y, arr &J, const WorldL &Ktuple, double tau, int t){
  CHECK(Ktuple.N>=3, "");
  CHECK(order>=2,"");

  arr a1, J1, a2, J2;

  TaskMap_Default pos1(posTMT, i);
  pos1.order=2;
  pos1.TaskMap::phi(a1, (&J?J1:NoArr), Ktuple, tau, t);

  TaskMap_Default pos2(posTMT, j);
  pos2.order=2;
  pos2.TaskMap::phi(a2, (&J?J2:NoArr), Ktuple, tau, t);

  arr c,Jc;
  TaskMap_PairCollision coll(i, j, false, true);
  coll.phi(c, (&J?Jc:NoArr), *Ktuple(-2), t);
  uintA qdim = getKtupleDim(Ktuple);
  arr Jcc = zeros(3, qdim.last());
  if(&J) Jcc.setMatrixBlock(Jc, 0, qdim(0));

  y = ARR(1., 1., 1.);
  if(&J) J = zeros(3, J1.d1);

  arr d=a2-a1;
  arr Jd=J2-J1;
  if(sumOfSqr(d)>1e-16 && sumOfSqr(c)>1e-16){
    normalizeWithJac(d, Jd);
    normalizeWithJac(c, Jcc);
    y = d + c;
    if(&J) J = Jd + Jcc;
  }

  checkNan(y);
  if(&J) checkNan(J);
  CHECK_EQ(y.N, dim_phi(*Ktuple.last()), "");
  if(&J) CHECK_EQ(J.d0, y.N, "");
}
