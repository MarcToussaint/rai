#include "F_pushed.h"
#include "F_static.h"
#include <Kin/kin.h>
#include "TM_angVel.h"
#include <Kin/contact.h>

void POA_vel(arr& y, arr& J, const WorldL& Ktuple, rai::Contact* con, bool b_or_a);

F_pushed::F_pushed(int iShape) : i(iShape){
  order=1;
}

void F_pushed::phi(arr& y, arr& J, const WorldL& Ktuple){
  CHECK_EQ(order, 1, "");

  rai::Frame *a = Ktuple(-2)->frames(i);

  //get linear and angular velocities
  arr v, Jv;
  TM_LinVel pos(i);
  pos.order=1;
  pos.phi(v, (!!J?Jv:NoArr), Ktuple);

  arr w, Jw;
  TM_AngVel rot(i);
  rot.order=1;
  rot.phi(w, (!!J?Jw:NoArr), Ktuple);

  y.resize(6).setZero();

  y.setVectorBlock(v, 0);
  y.setVectorBlock(w, 3);
  if(!!J) {
    J.resize(y.N, Jv.d1).setZero();
    J.setMatrixBlock(Jv, 0, 0);
    J.setMatrixBlock(Jw, 3, 0);
  }

  if(a->contacts.N==0){
    return;
  }

  double mass=1;
  arr Imatrix = diag(.02, 3);
  if(a->inertia){
    mass = a->inertia->mass;
    Imatrix = 2.*conv_mat2arr(a->inertia->matrix);
  }

//  mass = 1./mass;
//  Imatrix = inverse_SymPosDef(Imatrix);

  arr one_over_mass(6);
  for(uint i=0;i<3;i++) one_over_mass(i) = 1./mass;
  for(uint i=0;i<3;i++) one_over_mass(i+3) = 1./Imatrix(i,i);

  Value F = F_netForce(a->ID, false, true)(Ktuple);

  y += one_over_mass % F.y;
  if(!!J){
    J += one_over_mass % F.J;
  }



#if 0
  if(a->contacts.N==1){
    rai::Contact *con = a->contacts.scalar();
//    double sign = +1.;
    CHECK(&con->a==a || &con->b==a, "");
//    if(&con->b==a) sign=-1.;

    //get the POA
    arr cp, Jcp;
    Ktuple(-2)->kinematicsContactPOA(cp, Jcp, con);
    if(!!J) expandJacobian(Jcp, Ktuple, -2);

    //get object center
    arr p, Jp;
    Ktuple(-2)->kinematicsPos(p, Jp, a);
    if(!!J) expandJacobian(Jp, Ktuple, -2);

#if 0
    //get object vel
    arr v, Jv;
    TM_LinVel vel(a->ID);
    vel.phi(v, Jv, Ktuple);

    //object ang vel
    arr w, Jw;
    TM_AngVel ang(a->ID);
    ang.phi(w, Jw, Ktuple);
#else
    Value v = TM_LinVel(a->ID)(Ktuple);
    Value w = TM_AngVel(a->ID)(Ktuple);
#endif

    //relation between angular and linear displacement
    y = w.y - 100.*crossProduct(cp - p, v.y);
    if(!!J){
      J = w.J;
      J -= 100.*(skew(cp-p)*v.J - skew(v.y)*(Jcp - Jp));
    }
  }

  if(a->contacts.N>1){
    NIY;
  }
#endif
}
