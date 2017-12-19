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


#include "TM_FlagConstraints.h"
#include "taskMap_qItself.h"
#include "taskMap_default.h"
#include "frame.h"
#include "flag.h"

bool JointDidNotSwitch(const mlr::Frame *a1, const WorldL& Ktuple){
  CHECK_EQ(&a1->K, Ktuple.last(), "");
  if(a1->ID >= Ktuple(-2)->frames.N) return false;
  mlr::Frame *a0 = Ktuple(-2)->frames(a1->ID);
  mlr::Joint *j0 = a0->joint, *j1=a1->joint;
  if(!j0 || !j1
     || j0->type!=j1->type
     || j0->constrainToZeroVel!=j1->constrainToZeroVel
     || j0->from()->ID!=j1->from()->ID) return false;
  return true;
}

uint TaskMap_FlagConstraints::dim_phi(const WorldL& Ktuple, int t){
  uint d=0;
  for(mlr::Frame *a : Ktuple.last()->frames){
    if(a->flags & (1<<FT_zeroVel)) d += 7;
    if(a->flags & (1<<FT_zeroAcc)) d += 7;
//    if(a->flags & (1<<FT_zeroQVel)) if(JointDidNotSwitch(a, Ktuple)) d += a->joint->dim;
  }
  return d;
}

void TaskMap_FlagConstraints::phi(arr& y, arr& J, const WorldL& Ktuple, double tau, int t){
  CHECK(order==2,"");

  mlr::KinematicWorld& K = *Ktuple.last();

  y.resize(dim_phi(Ktuple, t)).setZero();
  if(&J){
    uintA xbarDim=getKtupleDim(Ktuple);
    J.resize(y.N, xbarDim.last()).setZero();
  }

  uint d=0;
  for(mlr::Frame *a : K.frames){
    if(a->flags & (1<<FT_zeroVel)){
      TaskMap_Default pos(posTMT, a->ID);
      pos.order=1;
      pos.TaskMap::phi(y({d,d+2})(), (&J?J({d,d+2})():NoArr), Ktuple, tau, t);

      TaskMap_Default quat(quatTMT, a->ID); //mt: NOT quatDiffTMT!! (this would compute the diff to world, which zeros the w=1...)
      // flip the quaternion sign if necessary
      quat.flipTargetSignOnNegScalarProduct = true;
      quat.order=1;
      quat.TaskMap::phi(y({d+3,d+6})(), (&J?J({d+3,d+6})():NoArr), Ktuple, tau, t);

      d += 7;
    }

    if(a->flags & (1<<FT_zeroAcc)){
      TaskMap_Default pos(posTMT, a->ID);
      pos.order=2;
      pos.TaskMap::phi(y({d,d+2})(), (&J?J({d,d+2})():NoArr), Ktuple, tau, t);

      TaskMap_Default quat(quatTMT, a->ID); //mt: NOT quatDiffTMT!! (this would compute the diff to world, which zeros the w=1...)
      // flip the quaternion sign if necessary
      quat.flipTargetSignOnNegScalarProduct = true;
      quat.order=2;
      quat.TaskMap::phi(y({d+3,d+6})(), (&J?J({d+3,d+6})():NoArr), Ktuple, tau, t);

      d += 7;
    }

//    if(a->flags & (1<<FT_zeroQVel)) if(JointDidNotSwitch(a, Ktuple)){
//      uint jdim = a->joint->dim;

//      TaskMap_qItself q({a->ID}, false);
//      q.order=1;
//      q.TaskMap::phi(y({d,d+jdim-1})(), (&J?J({d,d+jdim-1})():NoArr), Ktuple, tau, t);

//      d += jdim;
//    }

  }

  CHECK_EQ(d, y.N, "");
}
