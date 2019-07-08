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

  //get linear and angular velocities
  TM_LinAngVel pos(i);
  pos.order=1;
  pos.phi(y, J, Ktuple);

  double mass=1.;
  arr Imatrix = diag(.03, 3);
  rai::Frame *a = Ktuple(-2)->frames(i);
  if(a->inertia){
    mass = a->inertia->mass;
    Imatrix = 2.*conv_mat2arr(a->inertia->matrix);
  }
  arr one_over_mass(6);
  for(uint i=0;i<3;i++) one_over_mass(i) = 1./mass;
  for(uint i=0;i<3;i++) one_over_mass(i+3) = 1./Imatrix(i,i);
  double forceScaling = 1e1;
  one_over_mass *= forceScaling;

  Value F = F_netForce(a->ID, false, true)(Ktuple);

  y += one_over_mass % F.y;
  if(!!J) J += one_over_mass % F.J;
}
