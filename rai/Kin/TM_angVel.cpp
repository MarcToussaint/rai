#include "TM_angVel.h"
#include "TM_default.h"
#include "flag.h"

void TM_AngVel::phi(arr& y, arr& J, const WorldL& Ktuple) {
  if(order==2){
    arr y0, y1, J0, J1;
    order=1;
    phi(y0, J0, Ktuple({-3,-2}));
    phi(y1, J1, Ktuple({-2,-1}));
    order=2;

    y = y1 - y0;
    if(&J){
      CHECK_EQ(Ktuple.N, 3,"");
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

  CHECK_EQ(order, 1,"");

//  double tau = Ktuple(-1)->frames(0)->time; //- Ktuple(-2)->frames(0)->time;
  rai::Frame *f0 = Ktuple(-2)->frames(i);
  rai::Frame *f1 = Ktuple(-1)->frames(i);

  y.resize(3);
  arr a,b,Ja,Jb;
  Ktuple(-2)->kinematicsQuat(a, Ja, f0);
  Ktuple(-1)->kinematicsQuat(b, Jb, f1);
  arr J0, J1;
  quat_diffVector(y, J0, J1, a, b);
//  y /= tau;
  checkNan(y);

  if(&J){
    if(Ktuple.N==3){
      J = catCol(zeros(y.N, Ktuple(-3)->q.N), J0 * Ja, J1 * Jb);
    }else{
      J = catCol(J0 * Ja, J1 * Jb);
    }
//    J /= tau;
//    J = Jq1;
//    expandJacobian(J, Ktuple, 2);
    checkNan(J);
  }
}

uint TM_AngVel::dim_phi(const rai::KinematicWorld &G){ return 3; }

//===========================================================================

void TM_LinAngVel::phi(arr& y, arr& J, const WorldL& Ktuple){
  y.resize(6);
  if(&J) J.resize(6, getKtupleDim(Ktuple).last()).setZero();

  if(Ktuple.elem(-1)->frames(i)->flags & (1<<FL_impulseExchange)){
    return;
  }

//  rai::Frame *b0 = Ktuple.elem(-2)->frames(i);    CHECK(&b0->K==Ktuple.elem(-2),"");
//  rai::Frame *b1 = Ktuple.elem(-1)->frames(i);    CHECK(&b1->K==Ktuple.elem(-1),"");
//  cout <<"SWITCH " <<b0->parent->name <<'-' <<b0->name <<" => " <<b1->parent->name <<'-' <<b1->name <<endl;

  TM_Default lin(TMT_pos, i);
  lin.order=order;
  lin.Feature::phi(y({0,2})(), (&J?J({0,2})():NoArr), Ktuple);

  TM_AngVel ang(i);
  ang.order=order;
  ang.phi(y({3,5})(), (&J?J({3,5})():NoArr), Ktuple);
}

uint TM_LinAngVel::dim_phi(const rai::KinematicWorld& G){ return 6; }
