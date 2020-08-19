/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_transition.h"
#include "F_qFeatures.h"
#include "frame.h"
#include "flag.h"

TM_Transition::TM_Transition(const rai::Configuration& G, bool effectiveJointsOnly)
  : effectiveJointsOnly(effectiveJointsOnly) {
  posCoeff = rai::getParameter<double>("Motion/TaskMapTransition/posCoeff", .0);
  velCoeff = rai::getParameter<double>("Motion/TaskMapTransition/velCoeff", .0);
  accCoeff = rai::getParameter<double>("Motion/TaskMapTransition/accCoeff", 1.);

  order = 2;

  //transition cost metric
  H_rate = rai::getParameter<double>("Hrate", 1.);
  arr H_diag;
  if(rai::checkParameter<arr>("Hdiag")) {
    H_diag = rai::getParameter<arr>("Hdiag");
  } else {
    H_diag = G.getHmetric(); //G.naturalQmetric();
  }
  H_rate_diag = H_rate*H_diag;
}

uint TM_Transition::dim_phi(const ConfigurationL& G) {
  bool handleSwitches=effectiveJointsOnly;
  uint qN=G(0)->q.N;
  for(uint i=0; i<G.N; i++) if(G.elem(i)->q.N!=qN) { handleSwitches=true; break; }

  if(!handleSwitches) {
    return G.last()->getJointStateDimension();
  } else {
//    for(uint i=0;i<G.N;i++) cout <<i <<' ' <<G(i)->joints.N <<' ' <<G(i)->q.N <<' ' <<G(i)->getJointStateDimension() <<endl;
    rai::Array<rai::Joint*> matchingJoints = getMatchingJoints(G.sub(-1-order, -1), effectiveJointsOnly);
    uint ydim=0;
    for(uint i=0; i<matchingJoints.d0; i++) {
//      cout <<i <<' ' <<matchingJoints(i,0)->qIndex <<' ' <<matchingJoints(i,0)->qDim() <<' ' <<matchingJoints(i,0)->name <<endl;
      ydim += matchingJoints(i, 0)->qDim();
    }
    return ydim;
  }
  return uint(-1);
}

void TM_Transition::phi(arr& y, arr& J, const ConfigurationL& Ktuple) {
  if(velCoeff) CHECK(order>=1, "a velocity feature needs to have order>=1");
  if(accCoeff) CHECK(order>=2, "an acceleration feature needs to have order>=2");

  bool handleSwitches=effectiveJointsOnly;
  uint qN=Ktuple(0)->q.N;
  for(uint i=0; i<Ktuple.N; i++) if(Ktuple(i)->q.N!=qN) { handleSwitches=true; break; }

  double tau = Ktuple(-1)->frames(0)->tau; // - Ktuple(-2)->frames(0)->time;

  if(!handleSwitches) { //simple implementation
    //-- transition costs
    y.resize(Ktuple.last()->q.N).setZero();

    //individual weights
    double hbase = H_rate*sqrt(tau), tau2=tau*tau;
//    hbase = H_rate;
    arr h = zeros(y.N);
    for(rai::Joint* j:Ktuple.last()->fwdActiveJoints) for(uint i=0; i<j->qDim(); i++) {
        h(j->qIndex+i) = hbase*j->H;
        if(j->frame->flags && !(j->frame->flags & (1<<FL_normalControlCosts))) {
          h(j->qIndex+i)=0.;
        }
      }

    if(order==1) velCoeff = 1.;
    if(order>=0 && posCoeff) y +=  posCoeff      *(Ktuple.elem(-1)->q); //penalize position
#if 0
    if(order>=1 && velCoeff) y += (velCoeff/tau) *(G.elem(-2)->q - G.elem(-1)->q); //penalize velocity
    if(order>=2 && accCoeff) y += (accCoeff/tau2)*(G.elem(-3)->q - 2.*G.elem(-2)->q + G.elem(-1)->q); //penalize acceleration
#else //EQUIVALENT, but profiled - optimized for speed
    if(order>=1 && velCoeff) {
      arr v=Ktuple.elem(-2)->q;
      v -= Ktuple.elem(-1)->q;
      v *= (velCoeff/tau);
      y += v;
    }
    if(order>=2 && accCoeff) {
      arr a=Ktuple.elem(-2)->q;
      a *= -2.;
      a += Ktuple.elem(-3)->q;
      a += Ktuple.elem(-1)->q;
      a *= (accCoeff/tau2);
      y += a;
    }
#endif
    if(order>=3) NIY; //  y = (x_bar[3]-3.*x_bar[2]+3.*x_bar[1]-x_bar[0])/tau3; //penalize jerk

    //multiply with h...
#if 1
    y *= h;
#else
    for(rai::Joint* j:Ktuple.last()->fwdActiveJoints) for(uint i=0; i<j->qDim(); i++) {
        double hj = h*j->H;
        if(j->frame->flags && !(j->frame->flags & (1<<FL_normalControlCosts))) hj=0.;
        y(j->qIndex+i) *= hj;
      }
#endif

    if(!!J) {
      arr Jtau;  Ktuple(-1)->jacobianTime(Jtau, Ktuple(-1)->frames(0));  expandJacobian(Jtau, Ktuple, -1);
//      arr Jtau2;  Ktuple(-2)->jacobianTime(Jtau2, Ktuple(-2)->frames(0));  expandJacobian(Jtau2, Ktuple, -2);
//      arr Jtau = Jtau1 - Jtau2;

      uint n = Ktuple.last()->q.N;
      J.resize(y.N, Ktuple.N, n).setZero();
      for(uint i=0; i<n; i++) {
        if(order>=0 && posCoeff) { J(i, Ktuple.N-1-0, i) += posCoeff; }
        if(order>=1 && velCoeff) { J(i, Ktuple.N-1-1, i) += velCoeff/tau;  J(i, Ktuple.N-1-0, i) += -velCoeff/tau; }
#if 0
        if(order>=2 && accCoeff) { J(i, G.N-1-2, i) += accCoeff/tau2; J(i, G.N-1-1, i) += -2.*accCoeff/tau2;  J(i, G.N-1-0, i) += accCoeff/tau2; }
#else //EQUIVALENT, but profiled - optimized for speed
        if(order>=2 && accCoeff) {
          uint j = i*J.d1*J.d2 + i;
          J.elem(j+(Ktuple.N-3)*J.d2) += accCoeff/tau2;
          J.elem(j+(Ktuple.N-2)*J.d2) += -2.*accCoeff/tau2;
          J.elem(j+(Ktuple.N-1)*J.d2) += accCoeff/tau2;
        }
#endif
        //      if(order>=3){ J(i,3,i) = 1.;  J(i,2,i) = -3.;  J(i,1,i) = +3.;  J(i,0,i) = -1.; }
      }
      J.reshape(y.N, Ktuple.N*n);

#if 1
      J = h%J;
      J += (-1.5/tau)*y*Jtau;
#else
      for(rai::Joint* j: Ktuple.last()->fwdActiveJoints) for(uint i=0; i<j->qDim(); i++) {
          double hj = h*j->H;
          if(j->frame->flags && !(j->frame->flags & (1<<FL_normalControlCosts))) hj=0.;
#if 1
          uint k=j->qIndex+i;
          J[k] *= hj;
//        J[k] += y(k) * (0.5*hj/tau) * Jtau;
#else //EQUIVALENT, but profiled - optimized for speed
          uint l = (j->qIndex+i)*J.d1;
          for(uint k=0; k<J.d1; k++) J.elem(l+k) *= hj;
#endif
        }
#endif

    }
  } else { //with switches
    rai::Array<rai::Joint*> matchingJoints = getMatchingJoints(Ktuple.sub(-1-order, -1), effectiveJointsOnly);
    double h = H_rate*sqrt(tau), tau2=tau*tau;

    uint ydim=0;
    uintA qidx(Ktuple.N);
    for(uint i=0; i<matchingJoints.d0; i++) ydim += matchingJoints(i, 0)->qDim();
    y.resize(ydim).setZero();
    if(!!J) {
      qidx(0)=0;
      for(uint i=1; i<Ktuple.N; i++) qidx(i) = qidx(i-1)+Ktuple(i-1)->q.N;
      J.resize(ydim, qidx.last()+Ktuple.last()->q.N).setZero();
    }

    uint m=0;
    for(uint i=0; i<matchingJoints.d0; i++) {
      rai::Array<rai::Joint*> joints = matchingJoints[i];
      uint jdim = joints(0)->qDim(), qi1=0, qi2=0, qi3=0;
      for(uint j=0; j<jdim; j++) {
        if(order>=0) qi1 = joints.elem(-1)->qIndex+j;
        if(order>=1) qi2 = joints.elem(-2)->qIndex+j;
        if(order>=2 && accCoeff) qi3 = joints.elem(-3)->qIndex+j;
        rai::Joint* jl = joints.last();
        double hj = h * jl->H;
        if(jl->frame->flags && !(jl->frame->flags & (1<<FL_normalControlCosts))) hj=0.;
        //TODO: adding vels + accs before squareing does not make much sense!
        if(order>=0 && posCoeff) y(m) += posCoeff*hj       * (Ktuple.elem(-1)->q(qi1));
        if(order>=1 && velCoeff) y(m) += (velCoeff*hj/tau) * (Ktuple.elem(-1)->q(qi1) -    Ktuple.elem(-2)->q(qi2));
        if(order>=2 && accCoeff) y(m) += (accCoeff*hj/tau2)* (Ktuple.elem(-1)->q(qi1) - 2.*Ktuple.elem(-2)->q(qi2) + Ktuple.elem(-3)->q(qi3));
        if(!!J) {
          if(order>=0 && posCoeff) { J(m, qidx.elem(-1)+qi1) += posCoeff*hj; }
          if(order>=1 && velCoeff) { J(m, qidx.elem(-1)+qi1) += velCoeff*hj/tau;  J(m, qidx.elem(-2)+qi2) += -velCoeff*hj/tau; }
          if(order>=2 && accCoeff) { J(m, qidx.elem(-1)+qi1) += accCoeff*hj/tau2; J(m, qidx.elem(-2)+qi2) += -2.*accCoeff*hj/tau2; J(m, qidx.elem(-3)+qi3) += accCoeff*hj/tau2; }
        }
        m++;
      }
    }
    CHECK_EQ(m, ydim, "");
  }
}

