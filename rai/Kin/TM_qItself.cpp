/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include <climits>
#include "TM_qItself.h"
#include "frame.h"

TM_qItself::TM_qItself(bool relative_q0) : moduloTwoPi(true), relative_q0(relative_q0) {}

TM_qItself::TM_qItself(TM_qItself_PickMode pickMode, const StringA& picks, const rai::KinematicWorld& K, bool relative_q0)
  : moduloTwoPi(true), relative_q0(relative_q0) {
  if(pickMode==QIP_byJointGroups) {
    for(rai::Frame *f: K.frames) {
      bool pick=false;
      for(const rai::String& s:picks) if(f->ats.getNode(s)) { pick=true; break; }
      if(pick) selectedBodies.setAppend(f->ID);
    }
    return;
  }
  if(pickMode==QIP_byJointNames) {
    for(rai::String s:picks) {
      if(s(-2)==':') s.resize(s.N-2,true);
      rai::Frame *f = K.getFrameByName(s);
      if(!f) HALT("pick '" <<s <<"' not found");
      if(!f->joint) HALT("pick '" <<s <<"' is not a joint");
      selectedBodies.setAppend(f->ID);
    }
    return;
  }
  if(pickMode==QIP_byExcludeJointNames) {
    for(rai::Frame *f: K.fwdActiveSet) if(f->joint) {
        if(picks.contains(f->name)) continue;
        selectedBodies.setAppend(f->ID);
      }
    return;
  }
  NIY
}

TM_qItself::TM_qItself(const uintA& _selectedBodies, bool relative_q0)
  : selectedBodies(_selectedBodies), moduloTwoPi(true), relative_q0(relative_q0) {
}

void TM_qItself::phi(arr& q, arr& J, const rai::KinematicWorld& G) {
  if(!selectedBodies.N) {
    G.getJointState(q);
    if(relative_q0) {
      for(rai::Joint* j: G.fwdActiveJoints) if(j->q0.N && j->qDim()==1) q(j->qIndex) -= j->q0.scalar();
    }
    if(!!J) J.setId(q.N);
  } else {
    uint n=dim_phi(G);
    q.resize(n);
    G.getJointState();
    if(!!J) J.resize(n, G.q.N).setZero();
    uint m=0;
    uint qIndex=0;
    if(selectedBodies.N){
      for(uint i=0;i<selectedBodies.d0;i++) {
        rai::Joint *j=0;
        bool flipSign=false;
        if(selectedBodies.nd==1){
          rai::Frame *f = G.frames.elem(selectedBodies.elem(i));
          j = f->joint;
          CHECK(j, "selected frame " <<selectedBodies.elem(i) <<" ('" <<f->name <<"') is not a joint");
        }else{
          rai::Frame *a = G.frames.elem(selectedBodies(i,0));
          rai::Frame *b = G.frames.elem(selectedBodies(i,1));
          if(a->parent==b) j=a->joint;
          else if(b->parent==a){ j=b->joint; flipSign=true; }
          else HALT("a and b are not linked");
          CHECK(j, "");
        }
        qIndex = j->qIndex;
        for(uint k=0; k<j->qDim(); k++) {
          q(m) = G.q.elem(qIndex+k);
          if(flipSign) q(m) *= -1.;
          if(relative_q0 && j->q0.N) q(m) -= j->q0(k);
          if(!!J){
            if(flipSign) J(m, qIndex+k) = -1.;
            else J(m, qIndex+k) = 1.;
          }
          m++;
        }
      }
      CHECK_EQ(n, m,"");
    }
  }
}

void TM_qItself::phi(arr& y, arr& J, const WorldL& Ktuple) {
  CHECK_GE(Ktuple.N, order+1,"I need at least " <<order+1 <<" configurations to evaluate");
  uint k=order;
  if(k==0){
    phi(y, J, *Ktuple(-1));
    if(!!J) expandJacobian(J, Ktuple, -1);
    return;
  }
  
  double tau = Ktuple(-1)->frames(0)->tau; // - Ktuple(-2)->frames(0)->time;
  double tau2=tau*tau, tau3=tau2*tau;
  arrA q_bar(k+1), J_bar(k+1);
  //-- read out the task variable from the k+1 configurations
  uint offset = Ktuple.N-1-k; //G.N might contain more configurations than the order of THIS particular task -> the front ones are not used
  //before reading out, check if, in selectedBodies mode, some of the selected ones where switched
  uintA selectedBodies_org = selectedBodies;
  if(selectedBodies.N && selectedBodies.nd==1) {
    uintA sw = getSwitchedBodies(*Ktuple.elem(-2), *Ktuple.elem(-1));
    for(uint id:sw) selectedBodies.removeValue(id, false);
  }
  for(uint i=0; i<=k; i++) {
    phi(q_bar(i), J_bar(i), *Ktuple(offset+i));
  }
  selectedBodies = selectedBodies_org;
  
  bool handleSwitches=false;
  uint qN=q_bar(0).N;
  for(uint i=0; i<=k; i++) if(q_bar(i).N!=qN) { handleSwitches=true; break; }
  if(handleSwitches) { //when bodies are selected, switches don't have to be handled
    CHECK(!selectedBodies.N, "doesn't work for this...")
    uint nFrames = Ktuple(offset)->frames.N;
    JointL jointMatchLists(k+1, nFrames); //for each joint of [0], find if the others have it
    jointMatchLists.setZero();
    boolA useIt(nFrames);
    useIt = true;
    for(uint i=0; i<nFrames; i++) {
      rai::Frame *f = Ktuple(offset)->frames(i);
      rai::Joint *j = f->joint;
      if(j) {
        for(uint s=0; s<=k; s++) {
          rai::Joint *jmatch = Ktuple(offset+s)->getJointByBodyNames(j->from()->name, j->frame->name);
          if(jmatch && j->type!=jmatch->type) jmatch=NULL;
          if(!jmatch) { useIt(i) = false; break; }
          jointMatchLists(s, i) = jmatch;
        }
      } else {
        useIt(i) = false;
      }
    }
    
    arrA q_bar_mapped(k+1), J_bar_mapped(k+1);
    uint qidx, qdim;
    for(uint i=0; i<nFrames; i++) {
      if(useIt(i)) {
        for(uint s=0; s<=k; s++) {
          qidx = jointMatchLists(s,i)->qIndex;
          qdim = jointMatchLists(s,i)->qDim();
          if(qdim) {
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
  if(!!J) {
    uintA qidx(Ktuple.N);
    qidx(0)=0;
    for(uint i=1; i<Ktuple.N; i++) qidx(i) = qidx(i-1)+Ktuple(i-1)->q.N;
    J = zeros(y.N, qidx.last()+Ktuple.last()->q.N);
    if(k==1) { J.setMatrixBlock(J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau; }
    if(k==2) { J.setMatrixBlock(J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(-2.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(J_bar(0)   , 0, qidx(offset+0));  J/=tau2; }
    if(k==3) { J.setMatrixBlock(J_bar(3), 0, qidx(offset+3));  J.setMatrixBlock(-3.*J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(3.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau3; }

    arr Jtau;  Ktuple(-1)->jacobianTime(Jtau, Ktuple(-1)->frames(0));  expandJacobian(Jtau, Ktuple, -1);
//    arr Jtau2;  Ktuple(-2)->jacobianTime(Jtau2, Ktuple(-2)->frames(0));  expandJacobian(Jtau2, Ktuple, -2);
//    arr Jtau = Jtau1 - Jtau2;
    if(k==1) J += (-1./tau)*y*Jtau;
  }
}

uint TM_qItself::dim_phi(const rai::KinematicWorld& G) {
  if(selectedBodies.N) {
    uint n=0;
    for(uint i=0;i<selectedBodies.d0;i++) {
      rai::Joint *j=0;
      if(selectedBodies.nd==1){
        rai::Frame *f = G.frames.elem(selectedBodies.elem(i));
        j = f->joint;
        CHECK(j, "selected frame " <<selectedBodies.elem(i) <<" ('" <<f->name <<"') is not a joint");
      }else{
        rai::Frame *a = G.frames.elem(selectedBodies(i,0));
        rai::Frame *b = G.frames.elem(selectedBodies(i,1));
        if(a->parent==b) j=a->joint;
        else if(b->parent==a) j=b->joint;
        else HALT("a and b are not linked");
        CHECK(j, "");
      }
      n += j->qDim();
    }
    return n;
  }
  return G.getJointStateDimension();
}

uint TM_qItself::dim_phi(const WorldL& Ktuple) {
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
//  if(t<0) return dim_phi(*Ktuple.last());

//  while(dimPhi.N<=(uint)t) dimPhi.append(UINT_MAX);

//  //empirically test the dimension:
//  if(dimPhi(t)==UINT_MAX){
//    arr y;
//    phi(y, NoArr, Ktuple);
//    dimPhi(t) = y.N;
//  }

//  return dimPhi(t);
}

rai::String TM_qItself::shortTag(const rai::KinematicWorld& G) {
  rai::String s="qItself";
  if(selectedBodies.N) {
    if(selectedBodies.N<=3) {
      for(uint b:selectedBodies) s <<':' <<G.frames(b)->name;
    } else {
      s <<'#' <<selectedBodies.N;
    }
  } else {
    s <<":ALL";
  }
  return s;
}

//===========================================================================

void TM_qZeroVels::phi(arr& y, arr& J, const WorldL& Ktuple) {
  CHECK_EQ(order, 1,"NIY");
  CHECK_GE(Ktuple.N, order+1,"I need at least " <<order+1 <<" configurations to evaluate");
  uint k=order;
  
  double tau = Ktuple(-1)->frames(0)->tau; // - Ktuple(-2)->frames(0)->time;
  double tau2=tau*tau, tau3=tau2*tau;
  arrA q_bar(k+1), J_bar(k+1);
  //-- read out the task variable from the k+1 configurations
  uint offset = Ktuple.N-1-k; //G.N might contain more configurations than the order of THIS particular task -> the front ones are not used
  
  rai::Joint *j;
  for(rai::Frame *f:Ktuple.last()->frames) if((j=f->joint) && j->active && j->constrainToZeroVel) {
      rai::Joint *jmatch = Ktuple.last(-2)->getJointByBodyIndices(j->from()->ID, j->frame->ID);
      if(jmatch && j->type!=jmatch->type) jmatch=NULL;
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
    if(k==2) { J.setMatrixBlock(J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(-2.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(J_bar(0)   , 0, qidx(offset+0));  J/=tau2; }
    if(k==3) { J.setMatrixBlock(J_bar(3), 0, qidx(offset+3));  J.setMatrixBlock(-3.*J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(3.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau3; }
  }
}

uint TM_qZeroVels::dim_phi(const WorldL& Ktuple) {
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

rai::Array<rai::Joint*> getMatchingJoints(const WorldL& Ktuple, bool zeroVelJointsOnly) {
  rai::Array<rai::Joint*> matchingJoints;
  rai::Array<rai::Joint*> matches(Ktuple.N);
  bool matchIsGood;
  
  rai::Joint *j;
  for(rai::Frame *f:Ktuple.last()->frames) if((j=f->joint) && j->active && (!zeroVelJointsOnly || j->constrainToZeroVel)) {
      matches.setZero();
      matches.last() = j;
      matchIsGood=true;
      
      for(uint k=0; k<Ktuple.N-1; k++) { //go through other configs
        if(Ktuple(k)->frames.N<=j->frame->ID){ matchIsGood=false; break; }
        rai::Frame *fmatch = Ktuple(k)->frames(j->frame->ID);
        if(!fmatch){ matchIsGood=false; break; }
        rai::Joint *jmatch = fmatch->joint; //getJointByBodyIndices(j->from()->ID, j->frame->ID);
        if(!jmatch || j->type!=jmatch->type || j->constrainToZeroVel!=jmatch->constrainToZeroVel) {
          matchIsGood=false;
          break;
        }
        if(j->from() && j->from()->ID!=jmatch->from()->ID){
          matchIsGood=false;
          break;
        }
        matches(k) = jmatch;
      }
      
      if(matchIsGood) matchingJoints.append(matches);
    }
  matchingJoints.reshape(matchingJoints.N/Ktuple.N, Ktuple.N);
  return matchingJoints;
}

//===========================================================================

rai::Array<rai::Joint*> getSwitchedJoints(const rai::KinematicWorld& G0, const rai::KinematicWorld& G1, int verbose) {

  HALT("retired: we only look at switched objects");
  
  rai::Array<rai::Joint*> switchedJoints;
  
  rai::Joint *j1;
  for(rai::Frame *f: G1.frames) if((j1=f->joint) && j1->active) {
      if(j1->from()->ID>=G0.frames.N || j1->frame->ID>=G0.frames.N) {
        switchedJoints.append({NULL,j1});
        continue;
      }
      rai::Joint *j0 = G0.getJointByBodyIndices(j1->from()->ID, j1->frame->ID);
      if(!j0 || j0->type!=j1->type || j0->constrainToZeroVel!=j1->constrainToZeroVel) {
        if(G0.frames(j1->frame->ID)->joint) { //out-body had (in G0) one inlink...
          j0 = G0.frames(j1->frame->ID)->joint;
        }
        switchedJoints.append({j0,j1});
//      }
      }
    }
  switchedJoints.reshape(switchedJoints.N/2, 2);
  
  if(verbose) {
    for(uint i=0; i<switchedJoints.d0; i++) {
      cout <<"Switch: "
           <<switchedJoints(i,0)->from()->name <<'-' <<switchedJoints(i,0)->frame->name
           <<" -> " <<switchedJoints(i,1)->from()->name <<'-' <<switchedJoints(i,1)->frame->name <<endl;
    }
  }
  
  return switchedJoints;
}

//===========================================================================

bool isSwitched(rai::Frame *f0, rai::Frame *f1){
  rai::Joint *j0 = f0->joint;
  rai::Joint *j1 = f1->joint;
  if(!j0 != !j1) return true;
  if(j0) {
    if(j0->type!=j1->type
       || j0->constrainToZeroVel!=j1->constrainToZeroVel
       || (j0->from() && j0->from()->ID!=j1->from()->ID)) { //different joint type; or attached to different parent
      return true;
    }
  }
  return false;
}

//===========================================================================

uintA getSwitchedBodies(const rai::KinematicWorld& G0, const rai::KinematicWorld& G1, int verbose) {
  uintA switchedBodies;
  
  for(rai::Frame *b1:G1.frames) {
    uint id = b1->ID;
    if(id>=G0.frames.N) continue; //b1 does not exist in G0 -> not a switched body
    rai::Frame *b0 = G0.frames(id);
    rai::Joint *j0 = b0->joint;
    rai::Joint *j1 = b1->joint;
    if(!j1) continue; //don't report if j1 did not become an effective DOF
    if(!j0 != !j1) { switchedBodies.append(id); continue; }
    if(j0) {
      if(j0->type!=j1->type
         || j0->constrainToZeroVel!=j1->constrainToZeroVel
         || (j0->from() && j0->from()->ID!=j1->from()->ID)) { //different joint type; or attached to different parent
        switchedBodies.append(id);
      }
    }
  }
  
  if(verbose) {
    for(uint id : switchedBodies) {
      cout <<"Switch: "
           <<G0.frames(id)->name /*<<'-' <<switchedBodies(i,0)->name*/
           <<" -> " <<G1.frames(id)->name /*<<'-' <<switchedJoints(i,1)->to->name*/ <<endl;
    }
  }
  
  return switchedBodies;
}

//===========================================================================

uintA getNonSwitchedBodies(const WorldL& Ktuple) {
  uintA nonSwitchedBodies;

  rai::KinematicWorld& K0 = *Ktuple(0);
  for(rai::Frame *f0:K0.frames) {
    bool succ = true;
    uint id = f0->ID;
    for(uint i=1;i<Ktuple.N;i++){
      rai::KinematicWorld& K1 = *Ktuple(i);
      if(id>=K1.frames.N){ succ=false; break; }
      if(isSwitched(f0, K1.frames(id))){ succ=false; break; }
    }
    if(succ) nonSwitchedBodies.append(id);
  }
  return nonSwitchedBodies;
}
