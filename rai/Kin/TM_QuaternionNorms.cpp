/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_QuaternionNorms.h"
#include "TM_qItself.h"
#include "frame.h"

void TM_QuaternionNorms::phi(arr &y, arr &J, const rai::KinematicWorld &G) {
  uint n=dim_phi(G);
  y.resize(n);
  if(!!J) J.resize(n, G.q.N).setZero();
  uint i=0;
  for(const rai::Joint *j: G.fwdActiveJoints) if(j->type==rai::JT_quatBall || j->type==rai::JT_free || j->type==rai::JT_XBall) {
      arr q;
      if(j->type==rai::JT_quatBall) q.referToRange(G.q, j->qIndex+0, j->qIndex+3);
      if(j->type==rai::JT_XBall)    q.referToRange(G.q, j->qIndex+1, j->qIndex+4);
      if(j->type==rai::JT_free)     q.referToRange(G.q, j->qIndex+3, j->qIndex+6);
      double norm = sumOfSqr(q);
      y(i) = norm - 1.;
      
      if(!!J) {
        if(j->type==rai::JT_quatBall) J(i, {j->qIndex+0, j->qIndex+3}) = 2.*q;
        if(j->type==rai::JT_XBall)    J(i, {j->qIndex+1, j->qIndex+4}) = 2.*q;
        if(j->type==rai::JT_free)     J(i, {j->qIndex+3, j->qIndex+6}) = 2.*q;
      }
      i++;
    }
}

uint TM_QuaternionNorms::dim_phi(const rai::KinematicWorld &G) {
  uint i=0;
  for(const rai::Joint* j:G.fwdActiveJoints) {
    if(j->type==rai::JT_quatBall || j->type==rai::JT_free || j->type==rai::JT_XBall) i++;
  }
  return i;
}
