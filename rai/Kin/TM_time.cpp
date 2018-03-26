#include "TM_time.h"

void TM_Time::phi(arr &y, arr &J, const rai::KinematicWorld &K){
  y = ARR( K.frames(0)->time );

  if(&J){
    K.jacobianTime(J, K.frames(0));
  }
}
