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
#include "frame.h"

TaskMap_qItself::TaskMap_qItself(bool relative_q0) : moduloTwoPi(true), relative_q0(relative_q0) {}

TaskMap_qItself::TaskMap_qItself(TaskMap_qItself_PickMode pickMode, const StringA& picks, const mlr::KinematicWorld& K, bool relative_q0)
  : moduloTwoPi(true), relative_q0(relative_q0) {
  if(pickMode==QIP_byJointGroups){
    for(mlr::Frame *f: K.frames){
      bool pick=false;
      for(const mlr::String& s:picks) if(f->ats.getNode(s)){ pick=true; break; }
      if(pick) selectedBodies.append(f->ID);
    }
    return;
  }
  if(pickMode==QIP_byJointNames){
      for(const mlr::String& s:picks){
          mlr::Frame *f = K.getFrameByName(s);
          if(!f) HALT("pick '" <<s <<"' not found");
          if(!f->joint()) HALT("pick '" <<s <<"' is not a joint");
          selectedBodies.append(f->ID);
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
      for(mlr::Joint* j: G.fwdActiveJoints) if(j->q0.N && j->qDim()==1) q(j->qIndex) -= j->q0.scalar();
    }
    if(&J) J.setId(q.N);
  }else{
    uint n=dim_phi(G);
    q.resize(n);
    G.getJointState();
    if(&J) J.resize(n, G.q.N).setZero();
    uint m=0;
    uint qIndex=0;
    for(uint b:selectedBodies){
      mlr::Joint *j = G.frames.elem(b)->joint();
      CHECK(j, "selected frame " <<b <<" ('" <<G.frames.elem(b)->name <<"') is not a joint");
      qIndex = j->qIndex;
      for(uint k=0;k<j->qDim();k++){
        q(m) = G.q.elem(qIndex+k);
        if(relative_q0 && j->q0.N) q(m) -= j->q0(k);
        if(&J) J(m, qIndex+k) = 1.;
        m++;
      }
    }
    CHECK_EQ(n, m,"");
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
  }

  bool handleSwitches=false;
  uint qN=q_bar(0).N;
  for(uint i=0;i<=k;i++) if(q_bar(i).N!=qN){ handleSwitches=true; break; }
  if(handleSwitches){
    CHECK(!selectedBodies.N,"doesn't work for this...")
    uint nFrames = G(offset)->frames.N;
    JointL jointMatchLists(k+1, nFrames); //for each joint of [0], find if the others have it
    jointMatchLists.setZero();
    boolA useIt(nFrames);
    useIt = true;
    for(uint i=0; i<nFrames; i++){
      mlr::Frame *f = G(offset)->frames(i);
      mlr::Joint *j = f->joint();
      if(j){
        for(uint s=0;s<=k;s++){
          mlr::Joint *jmatch = G(offset+s)->getJointByBodyNames(j->from()->name, j->to()->name);
          if(jmatch && j->type!=jmatch->type) jmatch=NULL;
          if(!jmatch){ useIt(i) = false; break; }
          jointMatchLists(s, i) = jmatch;
        }
      }else{
        useIt(i) = false;
      }
    }

    arrA q_bar_mapped(k+1), J_bar_mapped(k+1);
    uint qidx, qdim;
    for(uint i=0; i<nFrames; i++){
      if(useIt(i)){
        for(uint s=0;s<=k;s++){
          qidx = jointMatchLists(s,i)->qIndex;
          qdim = jointMatchLists(s,i)->qDim();
          if(qdim){
            q_bar_mapped(s).append(q_bar(s)({qidx, qidx+qdim-1}));
            J_bar_mapped(s).append(J_bar(s)({qidx, qidx+qdim-1}));
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
    uintA qidx(G.N);
    qidx(0)=0;
    for(uint i=1;i<G.N;i++) qidx(i) = qidx(i-1)+G(i-1)->q.N;
    J = zeros(y.N, qidx.last()+G.last()->q.N);
    if(k==1){ J.setMatrixBlock(J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(   -J_bar(0), 0, qidx(offset+0));  J/=tau; }
    if(k==2){ J.setMatrixBlock(J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(-2.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(J_bar(0)   , 0, qidx(offset+0));  J/=tau2; }
    if(k==3){ J.setMatrixBlock(J_bar(3), 0, qidx(offset+3));  J.setMatrixBlock(-3.*J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(3.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau3; }
  }
}


uint TaskMap_qItself::dim_phi(const mlr::KinematicWorld& G) {
  if(selectedBodies.N){
    uint n=0;
    for(uint b:selectedBodies) n+=G.frames.elem(b)->joint()->qDim();
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
      for(uint b:selectedBodies) s <<':' <<G.frames(b)->name;
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

  mlr::Joint *j;
  for(mlr::Frame *f:G.last()->frames) if((j=f->joint()) && j->constrainToZeroVel){
    mlr::Joint *jmatch = G.last(-2)->getJointByBodyIndices(j->from()->ID, j->to()->ID);
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

  mlr::Joint *j;
  for(mlr::Frame *f:G.last()->frames) if((j=f->joint()) && (!zeroVelJointsOnly || j->constrainToZeroVel)){
    matches.setZero();
    matches.last() = j;
    matchIsGood=true;

    for(uint k=0;k<G.N-1;k++){ //go through other configs
      mlr::Joint *jmatch = G(k)->getJointByBodyIndices(j->from()->ID, j->to()->ID);
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

  HALT("retired: we only look at switched objects");

  mlr::Array<mlr::Joint*> switchedJoints;

  mlr::Joint *j1;
  for(mlr::Frame *f: G1.frames) if((j1=f->joint())){
    if(j1->from()->ID>=G0.frames.N || j1->to()->ID>=G0.frames.N){
      switchedJoints.append({NULL,j1});
      continue;
    }
    mlr::Joint *j0 = G0.getJointByBodyIndices(j1->from()->ID, j1->to()->ID);
    if(!j0 || j0->type!=j1->type){
      if(G0.frames(j1->to()->ID)->joint()){ //out-body had (in G0) one inlink...
        j0 = G0.frames(j1->to()->ID)->joint();
      }
      switchedJoints.append({j0,j1});
//      }
    }
  }
  switchedJoints.reshape(switchedJoints.N/2, 2);

  if(verbose){
    for(uint i=0;i<switchedJoints.d0;i++){
      cout <<"Switch: "
          <<switchedJoints(i,0)->from()->name <<'-' <<switchedJoints(i,0)->to()->name
         <<" -> " <<switchedJoints(i,1)->from()->name <<'-' <<switchedJoints(i,1)->to()->name <<endl;
    }
  }

  return switchedJoints;
}

//===========================================================================

mlr::Array<mlr::Frame*> getSwitchedBodies(const mlr::KinematicWorld& G0, const mlr::KinematicWorld& G1, int verbose){
  mlr::Array<mlr::Frame*> switchedBodies;

  for(mlr::Frame *b1:G1.frames) {
    if(b1->ID>=G0.frames.N) continue; //b1 does not exist in G0 -> not a switched body
    mlr::Frame *b0 = G0.frames(b1->ID);
    mlr::Joint *j0 = b0->joint();
    mlr::Joint *j1 = b1->joint();
    if(!j0 != !j1){ switchedBodies.append({b0,b1}); continue; }
    if(j0){
      if(j0->type!=j1->type || j0->from()->ID!=j1->from()->ID){ //different joint type; or attached to different parent
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
