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

#include "taskMaps.h"
#include "proxy.h"
#include "frame.h"

//===========================================================================

void CollisionConstraint::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  G.kinematicsProxyCost(y, J, margin, true);
  y -= .5;
}

//===========================================================================


PairCollisionConstraint::PairCollisionConstraint(const mlr::KinematicWorld &G, const char *iShapeName, const char *jShapeName, double _margin)
  : i(G.getFrameByName(iShapeName)->ID),
    j(G.getFrameByName(jShapeName)->ID),
    margin(_margin) {
}

void PairCollisionConstraint::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  if(t>=0 && referenceIds.N){
    if(referenceIds.nd==1){  i=referenceIds(t); j=-1; }
    if(referenceIds.nd==2){  i=referenceIds(t,0); j=referenceIds(t,1); }
  }

  y.resize(1) = -1.; //default value if not overwritten below
  if(&J) J.resize(1,G.q.N).setZero();
  if(j>=0){ //against a concrete j
    for(mlr::Proxy *p: G.proxies){
      if((p->a==i && p->b==j) || (p->a==j && p->b==i)){
        G.kinematicsProxyConstraint(y, J, p, margin);
        break;
      }
    }
  }else if(j==-1){ //against all objects
    NIY; //this doesn't work, don't know why
    //first collect all relevant proxies
    ProxyL P;
    for(mlr::Proxy *p: G.proxies) if((p->a==i) || (p->b==i)) P.append(p);
    //Compute the softmax
    double alpha = 10.;
    double yHat=0.,yNorm=0.;
    for(mlr::Proxy *p: P){
      G.kinematicsProxyConstraint(y, NoArr, p, margin);
      double yi=y.scalar();
      double expyi=::exp(alpha*yi);
      yNorm += expyi;
      yHat  += expyi * yi;
    }
    yHat /= yNorm;
    //compute derivative
    if(&J){
      J.resize(1,G.getJointStateDimension()).setZero();
      arr Ji;
      for(mlr::Proxy *p: P){
        G.kinematicsProxyConstraint(y, Ji, p, margin);
        double yi=y.scalar();
        double expyi=::exp(alpha*yi);
        J += expyi * (1.+alpha*(yi-yHat)) * Ji;
      }
      J /= yNorm;
    }
    y.scalar() = yHat;
  }
}

//===========================================================================

PlaneConstraint::PlaneConstraint(const mlr::KinematicWorld &G, const char *iShapeName, const arr &_planeParams)
  : i(G.getFrameByName(iShapeName)->ID), planeParams(_planeParams){}

void PlaneConstraint::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  mlr::Frame *body_i = G.frames(i);
  mlr::Vector vec_i = 0;

  arr y_eff, J_eff;
  G.kinematicsPos(y_eff, (&J?J_eff:NoArr), body_i, vec_i);

  y_eff.append(1.); //homogeneous coordinates
  if(&J) J_eff.append(zeros(1,J_eff.d1));

  y.resize(1);
  y(0) = scalarProduct(y_eff, planeParams);
  if(&J) J = ~planeParams * J_eff;
}

//===========================================================================

void ConstraintStickiness::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  map.phi(y, J, G);
  for(uint j=0;j<y.N;j++) y(j) = -y(j);
  if(&J) for(uint j=0;j<J.d0;j++) J[j]() *= -1.;
}

//===========================================================================

void PointEqualityConstraint::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  mlr::Vector vec_i = ivec;
  mlr::Vector vec_j = jvec;
  mlr::Frame *body_i = i<0?NULL: G.frames(i);
  mlr::Frame *body_j = j<0?NULL: G.frames(j);
  mlr::Vector pi = body_i ? body_i->X * vec_i : vec_i;
  mlr::Vector pj = body_j ? body_j->X * vec_j : vec_j;
  y = conv_vec2arr(pi-pj);
  if(&J) {
    arr Ji, Jj;
    G.kinematicsPos(NoArr, Ji, body_i, vec_i);
    if(body_j){
      G.kinematicsPos(NoArr, Jj, body_j, vec_j);
      J = Ji - Jj;
    }else{
      J = Ji;
    }
  }
}

//===========================================================================

ContactEqualityConstraint::ContactEqualityConstraint(const mlr::KinematicWorld &G, const char *iShapeName, const char *jShapeName, double _margin)
  : i(G.getFrameByName(iShapeName)->ID),
    j(G.getFrameByName(jShapeName)->ID),
    margin(_margin) {
}

void ContactEqualityConstraint::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  y.resize(1) = 0.;
  if(&J) J.resize(1,G.q.N).setZero();
  for(mlr::Proxy *p: G.proxies){
    if((p->a==i && p->b==j) || (p->a==j && p->b==i)){
      G.kinematicsProxyConstraint(y, J, p, margin);
      break;
    }
  }
}

//===========================================================================


VelAlignConstraint::VelAlignConstraint(const mlr::KinematicWorld& G,
                   const char* iShapeName, const mlr::Vector& _ivec,
                   const char* jShapeName, const mlr::Vector& _jvec, double _target) {
  mlr::Frame *a = iShapeName ? G.getFrameByName(iShapeName):NULL;
  mlr::Frame *b = jShapeName ? G.getFrameByName(jShapeName):NULL;
  if(a) i=a->ID;
  if(b) j=b->ID;
  if(&_ivec) ivec=_ivec; else ivec.setZero();
  if(&_jvec) jvec=_jvec; else jvec.setZero();
  order = 1;
  target = _target;
}

void VelAlignConstraint::phi(arr& y, arr& J, const WorldL& G, double tau, int t) {
  uint k=order;

  // compute body j orientation
  arr y_j,J_j,J_bar_j;
  G(G.N-1)->kinematicsVec(y_j, (&J?J_bar_j:NoArr), G(G.N-1)->frames(j), jvec);

  if(&J){
    J_j = zeros(G.N, y_j.N, J_bar_j.d1);
    J_j[G.N-1]() = J_bar_j;
    arr tmp(J_j);
    tensorPermutation(J_j, tmp, TUP(1u,0u,2u));
    J_j.reshape(y_j.N, G.N*J_bar_j.d1);
  }

  // compute body i velocity
  arrA y_bar, J_bar;
  y_bar.resize(k+1);
  J_bar.resize(k+1);

  for(uint c=0;c<=k;c++) {
    G(G.N-1-c)->kinematicsPos(y_bar(c), (&J?J_bar(c):NoArr), G(G.N-1-c)->frames(i), ivec);
  }

  arr dy_i, dJ_i;
  dy_i = (y_bar(0)-y_bar(1));

  if (&J) {
    dJ_i = zeros(G.N, dy_i.N, J_bar(0).d1);
    dJ_i[G.N-1-1]() = -J_bar(1);
    dJ_i[G.N-1-0]() = J_bar(0);
    arr tmp(dJ_i);
    tensorPermutation(dJ_i, tmp, TUP(1u,0u,2u));
    dJ_i.reshape(dy_i.N, G.N*J_bar(0).d1);
  }

  // normalize dy_i
  if (length(dy_i) != 0) {
    if (&J) {
      double tmp = (~dy_i*dy_i).scalar();
      dJ_i = ( eye(dJ_i.d0) - (dy_i*~dy_i)/(tmp) )*dJ_i/(length(dy_i));
    }
    dy_i = dy_i/(length(dy_i));
  }

  innerProduct(y,~dy_i,y_j);

  if (&J) {
    J = ~dy_i*J_j + ~y_j*dJ_i;
    J = -J;
  }
  y = -y+target;

}

//===========================================================================

void qItselfConstraint::phi(arr& q, arr& J, const mlr::KinematicWorld& G, int t) {
  G.getJointState(q);
  if(M.N){
    if(M.nd==1){
      q=M%q; if(&J) J.setDiag(M);
    }else{
      q=M*q; if(&J) J=M;
    }
  }else{
    if(&J) J.setId(q.N);
  }
}


