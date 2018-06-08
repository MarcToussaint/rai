#include "TM_angVel.h"

void TM_AngVel::phi(arr& y, arr& J, const WorldL& Ktuple) {
  if(order==2){
    arr y0, y1, J0, J1;
    order=1;
    phi(y0, J0, Ktuple({-3,-2}));
    phi(y1, J1, Ktuple({-2,-1}));
    order=2;

    y = y1 - y0;
    if(&J){
      CHECK(Ktuple.N==3,"");
      uint d0=Ktuple(-3)->q.N;
      uint d1=Ktuple(-2)->q.N;
      uint d2=Ktuple(-1)->q.N;
      J.resize(y.N, d0+d1+d2).setZero();
      CHECK_EQ(J0.d1, d0+d1, "");
      CHECK_EQ(J1.d1, d1+d2, "");
      for(uint i=0;i<y.N;i++){
        for(uint j=0;j<J0.d1;j++) J(i,j) -= J0(i,j);
        for(uint j=0;j<J1.d1;j++) J(i,d0+j) += J1(i,j);
      }
    }
    return;
  }

  CHECK(order==1,"");

//  double tau = Ktuple(-1)->frames(0)->time; //- Ktuple(-2)->frames(0)->time;
  rai::Frame *a0 = Ktuple(-2)->frames(i);
  rai::Frame *a1 = Ktuple(-1)->frames(i);

  y.resize(3);
  arr q0,q1,Jq0,Jq1;
  Ktuple(-2)->kinematicsQuat(q0, Jq0, a0);
  Ktuple(-1)->kinematicsQuat(q1, Jq1, a1);
  arr J0, J1;
  quat_diffVector(y, J0, J1, q0, q1);

//  quat_concat(y, J0, J1, q0, q1);
//  y = q1;

//  y /= tau;
  checkNan(y);

  if(&J){
    if(Ktuple.N==3){
      J = catCol(zeros(y.N, Ktuple(-3)->q.N), J0 * Jq0, J1 * Jq1);
    }else{
      J = catCol(J0 * Jq0, J1 * Jq1);
    }

//    J /= tau;
//    J = Jq1;
//    expandJacobian(J, Ktuple, 2);
    checkNan(J);
  }
}

uint TM_AngVel::dim_phi(const rai::KinematicWorld &G){ return 3; }
