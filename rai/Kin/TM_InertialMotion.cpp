#include "TM_InertialMotion.h"
#include "flag.h"
#include "taskMap_default.h"

void TM_InertialMotion::phi(arr &y, arr &J, const WorldL &Ktuple, double tau, int t){
  mlr::KinematicWorld& K = *Ktuple(-1);

  arr acc, Jacc;
  arr acc_ref = {0.,0.,g};
  arr Jacc_ref = zeros(3, K.q.N);
  {
    mlr::Frame *a = K.frames(i);
    if(a->flags & (1<<FT_gravityAcc)) MLR_MSG("frame '" <<a->name <<"' has InertialMotion AND Gravity objectivies");
    if(!(a->flags & (1<<FT_noQControlCosts))) MLR_MSG("frame '" <<a->name <<"' has InertialMotion AND control cost objectivies");
  }

  TaskMap_Default pos(posTMT, i);
  pos.order=2;
  pos.TaskMap::phi(acc, (&J?Jacc:NoArr), Ktuple, tau, t);

  y = acc - acc_ref;
  if(&J){
    J = zeros(3, Jacc.d1);
    J.setMatrixBlock(-Jacc_ref, 0, Jacc.d1-Jacc_ref.d1);
    J += Jacc;
  }
}

uint TM_InertialMotion::dim_phi(const WorldL &Ktuple, int t){
  return 3;
}

