/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "taskMap.h"
#include "TM_qItself.h"
#include "TM_GJK.h"
#include "TM_FixSwitchedObjects.h"
#include "frame.h"

//===========================================================================

void TaskMap::phi(arr& y, arr& J, const WorldL& Ktuple) {
  CHECK(Ktuple.N>=order+1,"I need at least " <<order+1 <<" configurations to evaluate");
  uint k=order;
  if(k==0) { // basic case: order=0
    arr J_bar;
    phi(y, (&J?J_bar:NoArr), *Ktuple.last());
    if(&J) {
      uint qidx=0;
      for(uint i=0; i<Ktuple.N; i++) qidx+=Ktuple(i)->q.N;
      J.resize(y.N, qidx).setZero();
      J.setMatrixBlock(J_bar, 0, qidx-J_bar.d1);
    }
    return;
  }
  arrA y_bar, J_bar;
  
  double tau = Ktuple(-1)->frames(0)->time - Ktuple(-2)->frames(0)->time;
  double tau2=tau*tau, tau3=tau2*tau;
  y_bar.resize(k+1);
  J_bar.resize(k+1);
  //-- read out the task variable from the k+1 configurations
  uint offset = Ktuple.N-1-k; //G.N might contain more configurations than the order of THIS particular task -> the front ones are not used
  for(uint i=0; i<=k; i++)
    phi(y_bar(i), (&J?J_bar(i):NoArr), *Ktuple(offset+i));
    
  // check for quaternion
  if(k==1 && flipTargetSignOnNegScalarProduct) {
    if(scalarProduct(y_bar(1), y_bar(0))<-.0) { y_bar(0) *= -1.;  if(&J) J_bar(0) *= -1.; }
  }
  // NIY
  if(k==2 && flipTargetSignOnNegScalarProduct) {
    if(scalarProduct(y_bar(2), y_bar(0))<-.0) { y_bar(0) *= -1.;  if(&J) J_bar(0) *= -1.; }
    if(scalarProduct(y_bar(2), y_bar(1))<-.0) { y_bar(1) *= -1.;  if(&J) J_bar(1) *= -1.; }
  }
  if(k==3 && flipTargetSignOnNegScalarProduct) HALT("Quaternion flipping NIY for jerk");
  
  if(k==1)  y = (y_bar(1)-y_bar(0))/tau; //penalize velocity
  if(k==2)  y = (y_bar(2)-2.*y_bar(1)+y_bar(0))/tau2; //penalize acceleration
  if(k==3)  y = (y_bar(3)-3.*y_bar(2)+3.*y_bar(1)-y_bar(0))/tau3; //penalize jerk
  if(&J) {
    uintA qidx(Ktuple.N);
    qidx(0)=0;
    for(uint i=1; i<Ktuple.N; i++) qidx(i) = qidx(i-1)+Ktuple(i-1)->q.N;
    J = zeros(y.N, qidx.last()+Ktuple.last()->q.N);
    if(k==1) { J.setMatrixBlock(J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau; }
    if(k==2) { J.setMatrixBlock(J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(-2.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(J_bar(0)   , 0, qidx(offset+0));  J/=tau2; }
    if(k==3) { J.setMatrixBlock(J_bar(3), 0, qidx(offset+3));  J.setMatrixBlock(-3.*J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(3.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau3; }

#if 0
    arr Jtau1;  Ktuple(-1)->jacobianTime(Jtau1, Ktuple(-1)->frames(0));  expandJacobian(Jtau1, Ktuple, -1);
    arr Jtau2;  Ktuple(-2)->jacobianTime(Jtau2, Ktuple(-2)->frames(0));  expandJacobian(Jtau2, Ktuple, -2);
    arr Jtau = Jtau1 - Jtau2;
    J += (-1.5/tau)*y*Jtau;
#endif
  }
}

