#include "taskMap_linTrans.h"

void TaskMap_LinTrans::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  map->phi(y, J, G, t);
  y = A*y;
  if(a.N) y += a;
  if(&J) J = A*J;
}

uint TaskMap_LinTrans::dim_phi(const mlr::KinematicWorld& G){
  return A.d0;
}
