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
  TaskMap_PairCollision coll(*Ktuple(-2), "ball1", "ball2", false, true);
  coll.phi(c, (&J?Jc:NoArr), *Ktuple(-2), t);
  uintA qdim = getKtupleDim(Ktuple);
  arr Jcc = zeros(3, qdim.last());
  if(&J) Jcc.setMatrixBlock(Jc, 0, qdim(0));

  normalizeWithJac(a2, J2);
  normalizeWithJac(c, Jcc);
  y.append(a2 + c);
  if(&J) J.append(J2 + Jcc);
}
