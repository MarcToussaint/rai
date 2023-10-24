/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_retired.h"

//===========================================================================

void TM_qZeroVels::phi(arr& y, arr& J, const ConfigurationL& Ktuple) {
  HALT("deprecated")
  CHECK_EQ(order, 1, "NIY");
  CHECK_GE(Ktuple.N, order+1, "I need at least " <<order+1 <<" configurations to evaluate");
  uint k=order;

  double tau = Ktuple(-1)->frames(0)->tau; // - Ktuple(-2)->frames(0)->time;
  double tau2=tau*tau, tau3=tau2*tau;
  arrA q_bar(k+1), J_bar(k+1);
  //-- read out the task variable from the k+1 configurations
  uint offset = Ktuple.N-1-k; //G.N might contain more configurations than the order of THIS particular task -> the front ones are not used

  rai::Joint* j;
  for(rai::Frame* f:Ktuple.last()->frames) if((j=f->joint) && j->active) {
      rai::Joint* jmatch = Ktuple.last(-2)->getJointByFrameIndices(j->from()->ID, j->frame->ID);
      if(jmatch && j->type!=jmatch->type) jmatch=nullptr;
      if(jmatch) {
        for(uint i=0; i<j->qDim(); i++) {
          q_bar(0).append(Ktuple.last(-2)->q(jmatch->qIndex+i));
          q_bar(1).append(Ktuple.last(-1)->q(j     ->qIndex+i));
          J_bar(0).append(eyeVec(Ktuple.last(-2)->q.N, jmatch->qIndex+i));
          J_bar(1).append(eyeVec(Ktuple.last(-1)->q.N, j     ->qIndex+i));
        }
      }
    }
  if(!q_bar(0).N) { y.clear(); if(!!J) J.clear(); return; }
  J_bar(0).reshape(q_bar(0).N, J_bar(0).N/q_bar(0).N);
  J_bar(1).reshape(q_bar(1).N, J_bar(1).N/q_bar(1).N);

  if(k==1)  y = (q_bar(1)-q_bar(0))/tau; //penalize velocity
  if(k==2)  y = (q_bar(2)-2.*q_bar(1)+q_bar(0))/tau2; //penalize acceleration
  if(k==3)  y = (q_bar(3)-3.*q_bar(2)+3.*q_bar(1)-q_bar(0))/tau3; //penalize jerk
  if(!!J) {
    uintA qidx(Ktuple.N);
    qidx(0)=0;
    for(uint i=1; i<Ktuple.N; i++) qidx(i) = qidx(i-1)+Ktuple(i-1)->q.N;
    J = zeros(y.N, qidx.last()+Ktuple.last()->q.N);
    if(k==1) { J.setMatrixBlock(J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau; }
    if(k==2) { J.setMatrixBlock(J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(-2.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(J_bar(0), 0, qidx(offset+0));  J/=tau2; }
    if(k==3) { J.setMatrixBlock(J_bar(3), 0, qidx(offset+3));  J.setMatrixBlock(-3.*J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(3.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau3; }
  }
}

uint TM_qZeroVels::dim_phi(const ConfigurationL& Ktuple) {
  if(order==0) return dim_phi(*Ktuple.last());
  else {
    if(dimPhi.find(Ktuple.last()) == dimPhi.end()) {
      arr y;
      phi(y, NoArr, Ktuple);
      dimPhi[Ktuple.last()] = y.N;
      return y.N;
    } else {
      return dimPhi[Ktuple.last()];
    }
  }
  return 0;
}

//===========================================================================
