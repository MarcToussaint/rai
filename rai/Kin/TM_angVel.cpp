#include "TM_angVel.h"
#include "TM_default.h"
#include "flag.h"

void angVel_base(const rai::KinematicWorld& K0, rai::KinematicWorld& K1, uint i, arr& y, arr& J){
  rai::Frame *f0 = K0.frames(i);
  rai::Frame *f1 = K1.frames(i);

  arr a,b,y_tmp,Ja,Jb;
  K0.kinematicsQuat(a, Ja, f0);
  K1.kinematicsQuat(b, Jb, f1);
  arr J0, J1;
//  quat_diffVector(y, J0, J1, a, b);
  arr dq = b-a;
  a(0) *=-1.;
  quat_concat(y_tmp, J0, J1, dq, a);
  for(uint i=0;i<J1.d0;i++) J1(i,0) *= -1.;
  y_tmp.remove(0);
  J0.delRows(0);
  J1.delRows(0);

  y = y_tmp;

#if 0
  y = dq;
  arr FLIP = eye(4); FLIP(0,0)=-1.;
  a = FLIP*a;
  quat_concat(y, J0, J1, dq, a);
  J1 = J1 * FLIP;
  cout <<dq <<'\n' <<J0 <<J1 <<endl;
//  for(uint i=0;i<J1.d0;i++) J1(i,0) *= -1.;
  if(&J){
    if(Ktuple.N==3){
      J = catCol(zeros(y.N, Ktuple(-3)->q.N), (J1-J0)*Ja, J0*Jb);
    }else{
      J = catCol((J1-J0)*Ja, J0*Jb);
    }
  }
  return;
#endif

#if 0
  f1 = f1->getUpwardLink();
  if(f1->joint->type==rai::JT_free){
    uint i=f1->joint->qIndex;
    y = Ktuple(-1)->q({i+4, i+6});
    if(&J){
      J.resize(3, Ktuple(-1)->q.N).setZero();
      for(uint j=0;j<3;j++) J(j,i+4+j) = 1.;
      if(Ktuple.N==3){
        J = catCol(zeros(y.N, Ktuple(-3)->q.N), zeros(y.N, Ktuple(-2)->q.N), J);
      }else{
        J = catCol(zeros(y.N, Ktuple(-2)->q.N), J);
      }
    }
    return;
  }else{
    y = b;
  }
  y.remove(0);
  J0.setId(4);
  J0.delRows(0);
  J1.setZero();
#endif

  checkNan(y);

  if(&J){
    J = catCol((J1-J0)*Ja, J0*Jb);
    checkNan(J);
  }
}

void TM_AngVel::phi(arr& y, arr& J, const WorldL& Ktuple) {
  if(order==1){
    arr J_tmp;
    angVel_base(*Ktuple(-2), *Ktuple(-1), i, y, J_tmp);

    if(&J){
      if(Ktuple.N==3) J = catCol(zeros(y.N, Ktuple(-3)->q.N), J_tmp);
      else J=J_tmp;
    }

#if 1
    double tau = Ktuple(-1)->frames(0)->time;
    CHECK_GE(tau, 1e-10, "");
    y /= tau;
    if(&J){
      J /=tau;
      arr Jtau;  Ktuple(-1)->jacobianTime(Jtau, Ktuple(-1)->frames(0));  expandJacobian(Jtau, Ktuple, -1);
      J += (-1./tau)*y*Jtau;
    }
#endif
    return;
  }

  if(order==2){
    arr y0, y1, J0, J1;
    angVel_base(*Ktuple(-3), *Ktuple(-2), i, y0, J0);
    angVel_base(*Ktuple(-2), *Ktuple(-1), i, y1, J1);

    y = y1 - y0;

#if 1
    double tau = Ktuple(-1)->frames(0)->time;
    CHECK_GE(tau, 1e-10, "");
    double tau2=tau*tau;
    y /= tau2;
#endif

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

#if 1
      J /= tau2;
      arr Jtau;  Ktuple(-1)->jacobianTime(Jtau, Ktuple(-1)->frames(0));  expandJacobian(Jtau, Ktuple, -1);
      J += (-2./tau)*y*Jtau;
#endif
    }
    return;
  }

  HALT("shoudn't be here");
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
