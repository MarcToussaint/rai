/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "taskMap_qUncertainties.h"
#include "frame.h"

TaskMap_qUncertainties::TaskMap_qUncertainties() {}

void TaskMap_qUncertainties::phi(arr& q, arr& J, const mlr::KinematicWorld& G, int t) {
  uint n=dim_phi(G);

  q.resize(n);
  if(&J) J.resize(n, G.getJointStateDimension()).setZero();

  uint i=0;
  for(mlr::Joint *j : G.fwdActiveJoints) if(j->uncertainty){
    for(uint k=j->dim; k<2*j->dim; k++){
      q(i) = G.q.elem(j->qIndex+k);
      if(&J) J(i, j->qIndex+k) = 1.;
      i++;
    }
  }
  CHECK_EQ(i, n, "");
}

uint TaskMap_qUncertainties::dim_phi(const mlr::KinematicWorld& G) {
  uint n=0;
  for(mlr::Joint *j : G.fwdActiveJoints) if(j->uncertainty) n += j->dim;
  return n;
}

mlr::String TaskMap_qUncertainties::shortTag(const mlr::KinematicWorld& G){
  return STRING("qUncertainties");
}
