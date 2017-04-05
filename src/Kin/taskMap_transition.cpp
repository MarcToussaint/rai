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


#include "taskMap_transition.h"
#include "taskMap_qItself.h"

TaskMap_Transition::TaskMap_Transition(const mlr::KinematicWorld& G, bool fixJointsOnly)
  : fixJointsOnly(fixJointsOnly){
  posCoeff = mlr::getParameter<double>("Motion/TaskMapTransition/posCoeff",.0);
  velCoeff = mlr::getParameter<double>("Motion/TaskMapTransition/velCoeff",.0);
  accCoeff = mlr::getParameter<double>("Motion/TaskMapTransition/accCoeff",1.);

  //transition cost metric
  H_rate = mlr::getParameter<double>("Hrate", 1.);
  arr H_diag;
  if(mlr::checkParameter<arr>("Hdiag")) {
    H_diag = mlr::getParameter<arr>("Hdiag");
  } else {
    H_diag = G.getHmetric(); //G.naturalQmetric();
  }
  H_rate_diag = H_rate*H_diag;
}

uint TaskMap_Transition::dim_phi(const WorldL& G, int t){
  bool handleSwitches=fixJointsOnly;
  uint qN=G(0)->q.N;
  for(uint i=0;i<G.N;i++) if(G.elem(i)->q.N!=qN){ handleSwitches=true; break; }
//  handleSwitches=true;

  if(!handleSwitches){
    return G.last()->getJointStateDimension();
  }else{
//    for(uint i=0;i<G.N;i++) cout <<i <<' ' <<G(i)->joints.N <<' ' <<G(i)->q.N <<' ' <<G(i)->getJointStateDimension() <<endl;
    mlr::Array<mlr::Joint*> matchingJoints = getMatchingJoints(G.sub(-1-order,-1), fixJointsOnly);
    uint ydim=0;
    for(uint i=0;i<matchingJoints.d0;i++){
//      cout <<i <<' ' <<matchingJoints(i,0)->qIndex <<' ' <<matchingJoints(i,0)->qDim() <<' ' <<matchingJoints(i,0)->name <<endl;
      ydim += matchingJoints(i,0)->qDim();
    }
    return ydim;
  }
  return uint(-1);
}

void TaskMap_Transition::phi(arr& y, arr& J, const WorldL& G, double tau, int t){
  if(G.last()->q_agent!=0){ //we're referring to a graph set to non-zero agent!
    HALT("what is this code about? why is the q_agent involved here?");
    uint n=G.last()->getJointStateDimension();
    CHECK(n!=H_rate_diag.N,"just checking...");
    y.resize(n).setZero();
    if(&J) J.resize(y.N, order+1, n).setZero();
    return;
  }

  bool handleSwitches=fixJointsOnly;
  uint qN=G(0)->q.N;
  for(uint i=0;i<G.N;i++) if(G(i)->q.N!=qN){ handleSwitches=true; break; }
//  handleSwitches=true;

  if(!handleSwitches){ //simple implementation
    //-- transition costs
    double h = H_rate*sqrt(tau), tau2=tau*tau;
//    arr h = sqrt(H_rate_diag)*sqrt(tau);
    y.resize(G.last()->q.N).setZero();
    if(order==1) velCoeff = 1.;
    if(order>=0 && posCoeff) y +=  posCoeff      *(G.elem(-1)->q); //penalize position
#if 0
    if(order>=1 && velCoeff) y += (velCoeff/tau) *(G.elem(-2)->q - G.elem(-1)->q); //penalize velocity
    if(order>=2 && accCoeff) y += (accCoeff/tau2)*(G.elem(-3)->q - 2.*G.elem(-2)->q + G.elem(-1)->q); //penalize acceleration
#else //EQUIVALENT, but profiled - optimized for speed
    if(order>=1 && velCoeff){
      arr v=G.elem(-2)->q;
      v -= G.elem(-1)->q;
      v *= (velCoeff/tau);
      y += v;
    }
    if(order>=2 && accCoeff){
      arr a=G.elem(-2)->q;
      a *= -2.;
      a += G.elem(-3)->q;
      a += G.elem(-1)->q;
      a *= (accCoeff/tau2);
      y += a;
    }
#endif
    if(order>=3) NIY; //  y = (x_bar[3]-3.*x_bar[2]+3.*x_bar[1]-x_bar[0])/tau3; //penalize jerk

    //multiply with h...
    for(mlr::Joint *j:G.last()->joints) for(uint i=0;i<j->qDim();i++)
      y(j->qIndex+i) *= h*j->H;

    if(&J) {
      uint n = G.last()->q.N;
      J.resize(y.N, G.N, n).setZero();
      for(uint i=0;i<n;i++){
        if(order>=0 && posCoeff){ J(i,G.N-1-0,i) += posCoeff; }
        if(order>=1 && velCoeff){ J(i,G.N-1-1,i) += velCoeff/tau;  J(i,G.N-1-0,i) += -velCoeff/tau; }
#if 0
        if(order>=2 && accCoeff){ J(i,G.N-1-2,i) += accCoeff/tau2; J(i,G.N-1-1,i) += -2.*accCoeff/tau2;  J(i,G.N-1-0,i) += accCoeff/tau2; }
#else //EQUIVALENT, but profiled - optimized for speed
        if(order>=2 && accCoeff){
          uint j = i*J.d1*J.d2 + i;
          J.elem(j+(G.N-3)*J.d2) += accCoeff/tau2;
          J.elem(j+(G.N-2)*J.d2) += -2.*accCoeff/tau2;
          J.elem(j+(G.N-1)*J.d2) += accCoeff/tau2;
        }
#endif
        //      if(order>=3){ J(i,3,i) = 1.;  J(i,2,i) = -3.;  J(i,1,i) = +3.;  J(i,0,i) = -1.; }
      }
      J.reshape(y.N, G.N*n);
      for(mlr::Joint *j:G.last()->joints) for(uint i=0;i<j->qDim();i++){
#if 0
        J[j->qIndex+i] *= h*j->H;
#else //EQUIVALENT, but profiled - optimized for speed
        uint l = (j->qIndex+i)*J.d1;
        for(uint k=0;k<J.d1;k++) J.elem(l+k) *= h*j->H;
#endif
      }
    }
  }else{ //with switches
    mlr::Array<mlr::Joint*> matchingJoints = getMatchingJoints(G.sub(-1-order,-1), fixJointsOnly);
    double h = H_rate*sqrt(tau), tau2=tau*tau;

//    getSwitchedJoints(*G.elem(-2), *G.elem(-1), true);

    uint ydim=0;
    uintA qidx(G.N);
    for(uint i=0;i<matchingJoints.d0;i++) ydim += matchingJoints(i,0)->qDim();
    y.resize(ydim).setZero();
    if(&J) {
      qidx(0)=0;
      for(uint i=1;i<G.N;i++) qidx(i) = qidx(i-1)+G(i-1)->q.N;
      J.resize(ydim, qidx.last()+G.last()->q.N).setZero();
    }

    uint m=0;
    for(uint i=0;i<matchingJoints.d0;i++){
      mlr::Array<mlr::Joint*> joints = matchingJoints[i];
      uint jdim = joints(0)->qDim(), qi1=0, qi2=0, qi3=0;
      for(uint j=0;j<jdim;j++){
        if(order>=0) qi1 = joints.elem(-1)->qIndex+j;
        if(order>=1) qi2 = joints.elem(-2)->qIndex+j;
        if(order>=2 && accCoeff) qi3 = joints.elem(-3)->qIndex+j;
        double hj = h * joints.last()->H;
        //TODO: adding vels + accs before squareing does not make much sense?
        if(order>=0 && posCoeff) y(m) += posCoeff*hj       * (G.elem(-1)->q(qi1));
        if(order>=1 && velCoeff) y(m) += (velCoeff*hj/tau) * (G.elem(-1)->q(qi1) -    G.elem(-2)->q(qi2));
        if(order>=2 && accCoeff) y(m) += (accCoeff*hj/tau2)* (G.elem(-1)->q(qi1) - 2.*G.elem(-2)->q(qi2) + G.elem(-3)->q(qi3));
        if(&J){
          if(order>=0 && posCoeff){ J(m, qidx.elem(-1)+qi1) += posCoeff*hj; }
          if(order>=1 && velCoeff){ J(m, qidx.elem(-1)+qi1) += velCoeff*hj/tau;  J(m, qidx.elem(-2)+qi2) += -velCoeff*hj/tau; }
          if(order>=2 && accCoeff){ J(m, qidx.elem(-1)+qi1) += accCoeff*hj/tau2; J(m, qidx.elem(-2)+qi2) += -2.*accCoeff*hj/tau2; J(m, qidx.elem(-3)+qi3) += accCoeff*hj/tau2; }
        }
        m++;
      }
    }
    CHECK(m==ydim,"");
  }
}

