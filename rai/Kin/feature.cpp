/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "feature.h"
#include "TM_qItself.h"
#include "TM_GJK.h"
#include "TM_FixSwitchedObjects.h"
#include "frame.h"

//===========================================================================

void Feature::phi(arr& y, arr& J, const WorldL& Ktuple) {
  CHECK_GE(Ktuple.N, order+1,"I need at least " <<order+1 <<" configurations to evaluate");
  if(order==0) {
    phi(y, J, *Ktuple(-1));
    if(!!J) expandJacobian(J, Ktuple, -1);
    return;
  }

#if 1
  arr y0, y1, Jy0, Jy1;
  order--;
  phi(y0, (!!J?Jy0:NoArr), Ktuple({0,-2}));  if(!!J) padJacobian(Jy0, Ktuple);
  phi(y1, (!!J?Jy1:NoArr), Ktuple);
  order++;

  if(flipTargetSignOnNegScalarProduct) if(scalarProduct(y0, y1)<-.0) { y0 *= -1.;  if(!!J) Jy0 *= -1.; }

  y = y1-y0;
  if(!!J) J = Jy1 - Jy0;

  if(Ktuple(-1)->hasTimeJoint()){
    double tau; arr Jtau;
    Ktuple(-1)->kinematicsTau(tau, (!!J?Jtau:NoArr));
    CHECK_GE(tau, 1e-10, "");

    y /= tau;
    if(!!J){
      J /= tau;
      expandJacobian(Jtau, Ktuple, -1);
      J += (-1./tau)*y*Jtau;
    }
  }
#else

  uint k = order;
  arrA y_bar, J_bar;
  
  double tau = Ktuple(-1)->frames(0)->tau; // - Ktuple(-2)->frames(0)->time;
  double tau_1 = Ktuple(-2)->frames(0)->tau; // - Ktuple(-2)->frames(0)->time;
  if(tau<=0.) tau=1.;
  if(order) CHECK_GE(tau, 1e-10, "");
  double tau2=tau*tau, tau3=tau2*tau;
  y_bar.resize(k+1);
  J_bar.resize(k+1);
  //-- read out the task variable from the k+1 configurations
  uint offset = Ktuple.N-1-k; //G.N might contain more configurations than the order of THIS particular task -> the front ones are not used
  for(uint i=0; i<=k; i++)
    phi(y_bar(i), (!!J?J_bar(i):NoArr), *Ktuple(offset+i));
    
  // check for quaternion
  if(k==1 && flipTargetSignOnNegScalarProduct) {
    if(scalarProduct(y_bar(1), y_bar(0))<-.0) { y_bar(0) *= -1.;  if(!!J) J_bar(0) *= -1.; }
  }
  // NIY
  if(k==2 && flipTargetSignOnNegScalarProduct) {
    if(scalarProduct(y_bar(2), y_bar(0))<-.0) { y_bar(0) *= -1.;  if(!!J) J_bar(0) *= -1.; }
    if(scalarProduct(y_bar(2), y_bar(1))<-.0) { y_bar(1) *= -1.;  if(!!J) J_bar(1) *= -1.; }
  }
  if(k==3 && flipTargetSignOnNegScalarProduct) HALT("Quaternion flipping NIY for jerk");
  
  if(k==1)  y = (y_bar(1)-y_bar(0))/tau; //penalize velocity
  if(k==2)  y = (y_bar(2)-2.*y_bar(1)+y_bar(0))/tau2; //penalize acceleration
  if(k==3)  y = (y_bar(3)-3.*y_bar(2)+3.*y_bar(1)-y_bar(0))/tau3; //penalize jerk
  if(!!J) {
    uintA qidx = getKtupleDim(Ktuple);
    qidx.prepend(0);
    J = zeros(y.N, qidx.last());
    if(k==1) { J.setMatrixBlock(J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau; }
    if(k==2) { J.setMatrixBlock(J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(-2.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(J_bar(0)   , 0, qidx(offset+0));  J/=tau2; }
    if(k==3) { J.setMatrixBlock(J_bar(3), 0, qidx(offset+3));  J.setMatrixBlock(-3.*J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(3.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau3; }

#if 1
    arr Jtau;  Ktuple(-1)->jacobianTime(Jtau, Ktuple(-1)->frames(0));  expandJacobian(Jtau, Ktuple, -1);
//    arr Jtau2;  Ktuple(-2)->jacobianTime(Jtau2, Ktuple(-2)->frames(0));  expandJacobian(Jtau2, Ktuple, -2);
//    arr Jtau = Jtau1 - Jtau2;
    if(k==1) J += (-1./tau)*y*Jtau;
    if(k==2) J += (-2./tau)*y*Jtau;
#endif
  }
#endif
}

VectorFunction Feature::vf(rai::KinematicWorld& K) { ///< direct conversion to vector function: use to check gradient or evaluate
    return [this, &K](arr& y, arr& J, const arr& x) -> void {
        K.setJointState(x);
        phi(y, J, K);
    };
}


VectorFunction Feature::vf(WorldL& Ktuple) { ///< direct conversion to vector function: use to check gradient or evaluate
    return [this, &Ktuple](arr& y, arr& J, const arr& x) -> void {
        uintA qdim = getKtupleDim(Ktuple);
        qdim.prepend(0);
        for(uint i=0;i<Ktuple.N;i++)
            Ktuple(i)->setJointState(x({qdim(i), qdim(i+1)-1}));
        phi(y, J, Ktuple);
    };
}
