#include "TM_Max.h"

void TM_Max::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  map->phi(y, J, G, t);
  uint i=argmax(y);
  y = ARR( y(i) );
  if(&J) J=~J[i];
}

