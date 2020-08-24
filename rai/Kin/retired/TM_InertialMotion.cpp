/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_InertialMotion.h"
#include "flag.h"
#include "TM_default.h"

void TM_InertialMotion::phi(arr& y, arr& J, const ConfigurationL& Ktuple) {
  rai::Configuration& K = *Ktuple(-1);

  arr acc, Jacc;
  arr acc_ref = {0., 0., g};
  arr Jacc_ref = zeros(3, K.q.N);
  {
    rai::Frame* a = K.frames(i);
    if(a->flags & (1<<FL_gravityAcc)) RAI_MSG("frame '" <<a->name <<"' has InertialMotion AND Gravity objectivies");
    if(a->joint && a->joint->H && !(a->flags && !(a->flags & (1<<FL_normalControlCosts))))
      RAI_MSG("frame '" <<a->name <<"' has InertialMotion AND control cost objectivies");
  }

  TM_Default pos(TMT_pos, i);
  pos.order=2;
  pos.Feature::__phi(acc, (!!J?Jacc:NoArr), Ktuple);

  y = acc - acc_ref;
  if(!!J) {
    J = zeros(3, Jacc.d1);
    J.setMatrixBlock(-Jacc_ref, 0, Jacc.d1-Jacc_ref.d1);
    J += Jacc;
  }

  if(Ktuple(-2)->frames(i)->flags & (1<<FL_impulseExchange)) {
    y.setZero();
    if(!!J) J.setZero();
  }
}

uint TM_InertialMotion::dim_phi(const ConfigurationL& Ktuple) {
  return 3;
}

