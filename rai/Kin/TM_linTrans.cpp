#include "TM_linTrans.h"

void TM_LinTrans::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  map->phi(y, J, G, t);
  if(A.N){
    y = A*y;
    if(&J) J = A*J;
  }
  if(a.N) y += a;
}

uint TM_LinTrans::dim_phi(const mlr::KinematicWorld& G){
  return A.d0;
}
