/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_energy.h"
#include <Kin/TM_angVel.h>
#include <Kin/TM_default.h>

TM_Energy::TM_Energy() {
  order=1;
  gravity = rai::getParameter<double>("TM_Physics/gravity", 9.81);
}

void TM_Energy::phi(arr &y, arr &J, const WorldL &Ktuple) {
  if(order==2){
    arr y0, y1, J0, J1;
    order=1;
    phi(y0, J0, Ktuple({-3,-2}));
    phi(y1, J1, Ktuple({-2,-1}));
    order=2;

    y = y1 - y0;
    if(!!J){
      uintA qdim = getKtupleDim(Ktuple);
      J.resize(y.N, qdim.last()).setZero();
      CHECK_EQ(J0.d1, qdim(1), "");
      CHECK_EQ(J1.d1, qdim(2)-qdim(0), "");
      for(uint i=0;i<y.N;i++){
        for(uint j=0;j<J0.d1;j++) J(i,j) -= J0(i,j);
        for(uint j=0;j<J1.d1;j++) J(i,qdim(0)+j) += J1(i,j);
      }
    }
    return;
  }

  CHECK_EQ(order, 1, "");

  rai::KinematicWorld& K = *Ktuple(-1);

  double E=0.;
  arr p, Jp, v, Jv, w, Jw;

  uintA qdim = getKtupleDim(Ktuple);
  if(!!J) J = zeros(1, qdim.last());

  for(rai::Frame *a:K.frames) {
    if(a->inertia){

      TM_Default pos(TMT_posDiff, a->ID);
      pos.order=0;
      pos.Feature::__phi(p, (!!J?Jp:NoArr), Ktuple);

      pos.order=1;
      pos.Feature::__phi(v, (!!J?Jv:NoArr), Ktuple);


//      TM_AngVel rot(a->ID);
//      rot.order=1;
//      rot.phi(w, (!!J?Jw:NoArr), Ktuple);

      double m=a->inertia->mass;
//      rai::Quaternion &rot = f->X.rot;
//      I=(rot).getMatrix() * f->inertia->matrix * (-rot).getMatrix();
      E += .5*m*sumOfSqr(v);
      E += gravity * m * p(2); //p(2)=height //(a->X*a->inertia->com).z;
//      E += .5*m*sumOfSqr(w); //(w*(I*w));

      if(!!J){
        J += (m*~v) * Jv;
        J += (gravity*m) * Jp[2];
      }
    }
  }

  y = ARR(E);
}

uint TM_Energy::dim_phi(const WorldL &Ktuple) {
  return 1;
}
