/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_qUncertainties.h"
#include "frame.h"

TM_qUncertainties::TM_qUncertainties() {}

void TM_qUncertainties::phi(arr& q, arr& J, const rai::KinematicWorld& G) {
  uint n=dim_phi(G);
  
  q.resize(n);
  if(!!J) J.resize(n, G.getJointStateDimension()).setZero();
  
  uint i=0;
  for(rai::Joint *j : G.fwdActiveJoints) if(j->uncertainty) {
      for(uint k=j->dim; k<2*j->dim; k++) {
        q(i) = G.q.elem(j->qIndex+k);
        if(!!J) J(i, j->qIndex+k) = 1.;
        i++;
      }
    }
  CHECK_EQ(i, n, "");
}

uint TM_qUncertainties::dim_phi(const rai::KinematicWorld& G) {
  uint n=0;
  for(rai::Joint *j : G.fwdActiveJoints) if(j->uncertainty) n += j->dim;
  return n;
}

rai::String TM_qUncertainties::shortTag(const rai::KinematicWorld& G) {
  return STRING("qUncertainties");
}
