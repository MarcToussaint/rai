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


#include <climits>
#include "taskMap_qItself.h"

TaskMap_qItself::TaskMap_qItself(bool relative_q0) : moduloTwoPi(true), relative_q0(relative_q0) {}

//TaskMap_qItself::TaskMap_qItself(uint singleQ, uint qN) : moduloTwoPi(true), relative_q0(false) { M=zeros(1,qN); M(0,singleQ)=1.; }

//TaskMap_qItself::TaskMap_qItself(const mlr::KinematicWorld& G, mlr::Joint* j)
//  : moduloTwoPi(true), relative_q0(false)  {
//  M = zeros(j->qDim(), G.getJointStateDimension() );
//  M.setMatrixBlock(eye(j->qDim()), 0, j->qIndex);
//}

//TaskMap_qItself::TaskMap_qItself(const mlr::KinematicWorld& G, const char* jointName)
//  : moduloTwoPi(true), relative_q0(false)  {
//  mlr::Joint *j = G.getJointByName(jointName);
//  if(!j) return;
//  M = zeros(j->qDim(), G.getJointStateDimension() );
//  M.setMatrixBlock(eye(j->qDim()), 0, j->qIndex);
//}

//TaskMap_qItself::TaskMap_qItself(const mlr::KinematicWorld& G, const char* jointName1, const char* jointName2)
//  : moduloTwoPi(true), relative_q0(false)  {
//  mlr::Joint *j1 = G.getJointByName(jointName1);
//  mlr::Joint *j2 = G.getJointByName(jointName2);
//  M = zeros(j1->qDim() + j2->qDim(), G.getJointStateDimension() );
//  M.setMatrixBlock(eye(j1->qDim()), 0, j1->qIndex);
//  M.setMatrixBlock(eye(j2->qDim()), j1->qDim(), j2->qIndex);
//}

TaskMap_qItself::TaskMap_qItself(TaskMap_qItself_PickMode pickMode, const StringA& picks, const mlr::KinematicWorld& K, bool relative_q0)
  : moduloTwoPi(true), relative_q0(relative_q0) {
  if(pickMode==QIP_byJointGroups){
    for(mlr::Joint *j:K.joints){
      bool pick=false;
      for(const mlr::String& s:picks) if(j->ats.getNode(s)){ pick=true; break; }
      if(pick) selectedBodies.append(j->to->index);
    }
    return;
  }
  if(pickMode==QIP_byJointNames){
    for(mlr::Joint *j:K.joints){
      bool pick=false;
      for(const mlr::String& s:picks) if(j->name==s){ pick=true; break; }
      if(pick) selectedBodies.append(j->to->index);
    }
    return;
  }
  NIY
}

TaskMap_qItself::TaskMap_qItself(uintA _selectedBodies, bool relative_q0)
  : selectedBodies(_selectedBodies), moduloTwoPi(true), relative_q0(relative_q0){
}

void TaskMap_qItself::phi(arr& q, arr& J, const mlr::KinematicWorld& G, int t) {
  if(!selectedBodies.N){
    G.getJointState(q);
    if(relative_q0){
      for(mlr::Joint* j:G.joints) if(j->q0 && j->qDim()==1) q(j->qIndex) -= j->q0;
    }
//    if(M.N){
//      if(M.nd==1){
//        q=M%q; if(&J) J.setDiag(M); //this fails if the dimensionalities of q are non-stationary!
//      }else{
//        q=M*q; if(&J) J=M;
//      }
//    }else{
      if(&J) J.setId(q.N);
//    }
  }else{
    uint n=dim_phi(G);
    q.resize(n);
    G.getJointState();
    if(&J) J.resize(n, G.q.N).setZero();
    uint m=0;
    uint qIndex=0;
    for(uint b:selectedBodies){
      mlr::Joint *j = G.bodies.elem(b)->inLinks.scalar();
//      CHECK_GE(j->qIndex, qIndex, "selectedBodies does not add joints in sorted order! I'm not sure this is correct!");
      qIndex = j->qIndex;
      for(uint k=0;k<j->qDim();k++){
        q(m) = G.q.elem(qIndex+k);
        if(relative_q0) q(m) -= j->q0;
        if(&J) J(m,qIndex+k) = 1.;
        m++;
      }
    }
    CHECK_EQ(n,m,"");
  }
}

void TaskMap_qItself::phi(arr& y, arr& J, const WorldL& G, double tau, int t){
  CHECK(G.N>=order+1,"I need at least " <<order+1 <<" configurations to evaluate");
  uint k=order;
  if(k==0) return TaskMap::phi(y, J, G, tau, t);

  double tau2=tau*tau, tau3=tau2*tau;
  arrA q_bar(k+1), J_bar(k+1);
  //-- read out the task variable from the k+1 configurations
  uint offset = G.N-1-k; //G.N might contain more configurations than the order of THIS particular task -> the front ones are not used
  for(uint i=0;i<=k;i++){
    phi(q_bar(i), J_bar(i), *G(offset+i), t-k+i);
//    q_bar(i) = G(offset+i)->q;
//    J_bar(i).setId(q_bar(i).N);
  }
  bool handleSwitches=false;
  uint qN=q_bar(0).N;
  for(uint i=0;i<=k;i++) if(q_bar(i).N!=qN){ handleSwitches=true; break; }
  if(handleSwitches){
    CHECK(!selectedBodies.N,"doesn't work for this...")
    uint nJoints = G(offset)->joints.N;
    JointL jointMatchLists(k+1, nJoints); //for each joint of [0], find if the others have it
    jointMatchLists.setZero();
    boolA useIt(nJoints);
    useIt = true;
    for(uint j_idx=0; j_idx<nJoints; j_idx++){
      mlr::Joint *j=G(offset)->joints(j_idx);
      for(uint i=0;i<=k;i++){
        mlr::Joint *jmatch = G(offset+i)->getJointByBodyNames(j->from->name, j->to->name);
        if(jmatch && j->type!=jmatch->type) jmatch=NULL;
        if(!jmatch){ useIt(j_idx) = false; break; }
        jointMatchLists(i, j_idx) = jmatch;
      }
    }

    arrA q_bar_mapped(k+1), J_bar_mapped(k+1);
    uint qidx, qdim;
    for(uint j_idx=0; j_idx<nJoints; j_idx++){
      if(useIt(j_idx)){
        for(uint i=0;i<=k;i++){
          qidx=jointMatchLists(i,j_idx)->qIndex;
          qdim=jointMatchLists(i,j_idx)->qDim();
          if(qdim){
            q_bar_mapped(i).append(q_bar(i)({qidx, qidx+qdim-1}));
            J_bar_mapped(i).append(J_bar(i)({qidx, qidx+qdim-1}));
          }
        }
      }
    }

    q_bar = q_bar_mapped;
    J_bar = J_bar_mapped;
  }

  if(k==1)  y = (q_bar(1)-q_bar(0))/tau; //penalize velocity
  if(k==2)  y = (q_bar(2)-2.*q_bar(1)+q_bar(0))/tau2; //penalize acceleration
  if(k==3)  y = (q_bar(3)-3.*q_bar(2)+3.*q_bar(1)-q_bar(0))/tau3; //penalize jerk
  if(&J) {
#if 1
    uintA qidx(G.N);
    qidx(0)=0;
    for(uint i=1;i<G.N;i++) qidx(i) = qidx(i-1)+G(i-1)->q.N;
    J = zeros(y.N, qidx.last()+G.last()->q.N);
    if(k==1){ J.setMatrixBlock(J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(   -J_bar(0), 0, qidx(offset+0));  J/=tau; }
    if(k==2){ J.setMatrixBlock(J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(-2.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(J_bar(0)   , 0, qidx(offset+0));  J/=tau2; }
    if(k==3){ J.setMatrixBlock(J_bar(3), 0, qidx(offset+3));  J.setMatrixBlock(-3.*J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(3.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau3; }
#else
    J = zeros(G.N, y.N, J_bar(0).d1);
    if(k==1){ J[offset+1]() =  J_bar(1);  J[offset+0]() =    -J_bar(0);  J/=tau; }
    if(k==2){ J[offset+2]() =  J_bar(2);  J[offset+1]() = -2.*J_bar(1);  J[offset+0]() = J_bar(0);  J/=tau2; }
    if(k==3){ J[offset+3]() =  J_bar(3);  J[offset+2]() = -3.*J_bar(2);  J[offset+1]() = 3.*J_bar(1);  J[offset+0]() = -J_bar(0);  J/=tau3; }
    arr tmp(J);
    tensorPermutation(J, tmp, TUP(1u,0u,2u));
    J.reshape(y.N, G.N*J_bar(0).d1);
#endif
  }
}


uint TaskMap_qItself::dim_phi(const mlr::KinematicWorld& G) {
  if(selectedBodies.N){
    uint n=0;
    for(uint b:selectedBodies) n+=G.bodies.elem(b)->inLinks.scalar()->qDim();
    return n;
  }
//  if(M.nd==2) return M.d0;
  return G.getJointStateDimension();
}

uint TaskMap_qItself::dim_phi(const WorldL& G, int t){
  if(t<0) return dim_phi(*G.last());

  while(dimPhi.N<=(uint)t) dimPhi.append(UINT_MAX);

  //empirically test the dimension:
  if(dimPhi(t)==UINT_MAX){
    arr y;
    phi(y, NoArr, G, 0.01, t);
    dimPhi(t) = y.N;
  }

  return dimPhi(t);
}

mlr::String TaskMap_qItself::shortTag(const mlr::KinematicWorld& G){
  mlr::String s="qItself";
  if(selectedBodies.N){
    if(selectedBodies.N<=3){
      for(uint b:selectedBodies) s <<':' <<G.bodies(b)->name;
    }else{
      s <<'#' <<selectedBodies.N;
    }
  }else{
    s <<":ALL";
  }
  return s;
}

//===========================================================================

void TaskMap_qZeroVels::phi(arr& y, arr& J, const WorldL& G, double tau, int t){
  CHECK(order==1,"NIY");
  CHECK(G.N>=order+1,"I need at least " <<order+1 <<" configurations to evaluate");
  uint k=order;

  double tau2=tau*tau, tau3=tau2*tau;
  arrA q_bar(k+1), J_bar(k+1);
  //-- read out the task variable from the k+1 configurations
  uint offset = G.N-1-k; //G.N might contain more configurations than the order of THIS particular task -> the front ones are not used

  for(mlr::Joint *j:G.last()->joints) if(j->constrainToZeroVel){
    mlr::Joint *jmatch = G.last(-2)->getJointByBodyIndices(j->from->index, j->to->index);
    if(jmatch && j->type!=jmatch->type) jmatch=NULL;
    if(jmatch){
      for(uint i=0;i<j->qDim();i++){
        q_bar(0).append(G.last(-2)->q(jmatch->qIndex+i));
        q_bar(1).append(G.last(-1)->q(j     ->qIndex+i));
        J_bar(0).append(eyeVec(G.last(-2)->q.N, jmatch->qIndex+i));
        J_bar(1).append(eyeVec(G.last(-1)->q.N, j     ->qIndex+i));
      }
    }
  }
  if(!q_bar(0).N){ y.clear(); if(&J) J.clear(); return; }
  J_bar(0).reshape(q_bar(0).N, J_bar(0).N/q_bar(0).N);
  J_bar(1).reshape(q_bar(1).N, J_bar(1).N/q_bar(1).N);

  if(k==1)  y = (q_bar(1)-q_bar(0))/tau; //penalize velocity
  if(k==2)  y = (q_bar(2)-2.*q_bar(1)+q_bar(0))/tau2; //penalize acceleration
  if(k==3)  y = (q_bar(3)-3.*q_bar(2)+3.*q_bar(1)-q_bar(0))/tau3; //penalize jerk
  if(&J) {
    uintA qidx(G.N);
    qidx(0)=0;
    for(uint i=1;i<G.N;i++) qidx(i) = qidx(i-1)+G(i-1)->q.N;
    J = zeros(y.N, qidx.last()+G.last()->q.N);
    if(k==1){ J.setMatrixBlock(J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(   -J_bar(0), 0, qidx(offset+0));  J/=tau; }
    if(k==2){ J.setMatrixBlock(J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(-2.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(J_bar(0)   , 0, qidx(offset+0));  J/=tau2; }
    if(k==3){ J.setMatrixBlock(J_bar(3), 0, qidx(offset+3));  J.setMatrixBlock(-3.*J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(3.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau3; }
  }
}

uint TaskMap_qZeroVels::dim_phi(const WorldL& G, int t){
  CHECK(t>=0,"");

  while(dimPhi.N<=(uint)t) dimPhi.append(UINT_MAX);

  //empirically test the dimension:
  if(dimPhi(t)==UINT_MAX){
    arr y;
    phi(y, NoArr, G, 0.01, t);
    dimPhi(t) = y.N;
  }

  return dimPhi(t);
}

//===========================================================================

mlr::Array<mlr::Joint*> getMatchingJoints(const WorldL& G, bool zeroVelJointsOnly){
  mlr::Array<mlr::Joint*> matchingJoints;
  mlr::Array<mlr::Joint*> matches(G.N);
  bool matchIsGood;

  for(mlr::Joint *j:G.last()->joints) if(!zeroVelJointsOnly || j->constrainToZeroVel){
    matches.setZero();
    matches.last() = j;
    matchIsGood=true;

    for(uint k=0;k<G.N-1;k++){ //go through other configs
      mlr::Joint *jmatch = G(k)->getJointByBodyIndices(j->from->index, j->to->index);
      if(!jmatch || j->type!=jmatch->type || j->constrainToZeroVel!=jmatch->constrainToZeroVel){
        matchIsGood=false;
        break;
      }
      matches(k) = jmatch;
    }

    if(matchIsGood) matchingJoints.append(matches);
  }
  matchingJoints.reshape(matchingJoints.N/G.N, G.N);
  return matchingJoints;
}

//===========================================================================

mlr::Array<mlr::Joint*> getSwitchedJoints(const mlr::KinematicWorld& G0, const mlr::KinematicWorld& G1, int verbose){
  mlr::Array<mlr::Joint*> switchedJoints;

  for(mlr::Joint *j1:G1.joints) {
    if(j1->from->index>=G0.bodies.N || j1->to->index>=G0.bodies.N){
      switchedJoints.append({NULL,j1});
      continue;
    }
    mlr::Joint *j0 = G0.getJointByBodyIndices(j1->from->index, j1->to->index);
    if(!j0 || j0->type!=j1->type){
      if(G0.bodies(j1->to->index)->inLinks.N==1){ //out-body had (in G0) one inlink...
        j0 = G0.bodies(j1->to->index)->inLinks.scalar();
      }
      switchedJoints.append({j0,j1});
//      }
    }
  }
  switchedJoints.reshape(switchedJoints.N/2, 2);

  if(verbose){
    for(uint i=0;i<switchedJoints.d0;i++){
      cout <<"Switch: "
          <<switchedJoints(i,0)->from->name <<'-' <<switchedJoints(i,0)->to->name
         <<" -> " <<switchedJoints(i,1)->from->name <<'-' <<switchedJoints(i,1)->to->name <<endl;
    }
  }

  return switchedJoints;
}

//===========================================================================

mlr::Array<mlr::Body*> getSwitchedBodies(const mlr::KinematicWorld& G0, const mlr::KinematicWorld& G1, int verbose){
  mlr::Array<mlr::Body*> switchedBodies;

  for(mlr::Body *b1:G1.bodies) {
    if(b1->index>=G0.bodies.N) continue;
    mlr::Body *b0 = G0.bodies(b1->index);
    if(b0->inLinks.N != b1->inLinks.N){ switchedBodies.append({b0,b1}); continue; }
    if(b0->inLinks.N){
      CHECK(b0->inLinks.N==1,"not a tree!?");
      mlr::Joint *j0 = b0->inLinks.scalar();
      mlr::Joint *j1 = b1->inLinks.scalar();
      if(j0->type!=j1->type){
        switchedBodies.append({b0,b1});
      }
    }
  }
  switchedBodies.reshape(switchedBodies.N/2, 2);

  if(verbose){
    for(uint i=0;i<switchedBodies.d0;i++){
      cout <<"Switch: "
          <<switchedBodies(i,0)->name /*<<'-' <<switchedBodies(i,0)->name*/
         <<" -> " <<switchedBodies(i,1)->name /*<<'-' <<switchedJoints(i,1)->to->name*/ <<endl;
    }
  }

  return switchedBodies;
}
