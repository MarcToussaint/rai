/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_pushed.h"
#include "F_static.h"
#include "TM_angVel.h"
#include "kin.h"
#include "forceExchange.h"

void POA_vel(arr& y, arr& J, const ConfigurationL& Ktuple, rai::ForceExchange* con, bool b_or_a);

F_pushed::F_pushed(int iShape) : i(iShape) {
  order=1;
}

void F_pushed::phi(arr& y, arr& J, const ConfigurationL& Ktuple) {
  CHECK_EQ(order, 1, "");

  //get linear and angular velocities
  TM_LinAngVel pos(i);
  pos.order=1;
  pos.phi(y, J, Ktuple);

  double mass=1.;
  arr Imatrix = diag(.03, 3);
  rai::Frame* a = Ktuple(-2)->frames(i);
  if(a->inertia) {
    mass = a->inertia->mass;
    Imatrix = 2.*conv_mat2arr(a->inertia->matrix);
  }
  arr one_over_mass(6);
  for(uint i=0; i<3; i++) one_over_mass(i) = 1./mass;
  for(uint i=0; i<3; i++) one_over_mass(i+3) = 1./Imatrix(i, i);
  double forceScaling = 1e1;
  one_over_mass *= forceScaling;

  Value F = F_netForce(a->ID, false, true)(Ktuple);

  y += one_over_mass % F.y;
  if(!!J) J += one_over_mass % F.J;
}
