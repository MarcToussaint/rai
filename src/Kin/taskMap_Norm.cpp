#include "taskMap_Norm.h"

void TaskMap_Norm::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  map->phi(y, J, G, t);
  double l = sqrt(sumOfSqr(y));
  if(&J) J = ~(y/l)*J;
  y = ARR(l);
}

uint TaskMap_Norm::dim_phi(const mlr::KinematicWorld& G){
  return 1;
}
