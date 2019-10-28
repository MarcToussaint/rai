/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include <climits>
#include "F_qFeatures.h"
#include "frame.h"

//===========================================================================

F_qItself::F_qItself(bool relative_q0) : moduloTwoPi(true), relative_q0(relative_q0) {}

F_qItself::F_qItself(PickMode pickMode, const StringA& picks, const rai::Configuration& K, bool relative_q0)
  : moduloTwoPi(true), relative_q0(relative_q0) {
  if(pickMode==byJointGroups) {
    for(rai::Frame *f: K.frames) {
      bool pick=false;
      for(const rai::String& s:picks) if(f->ats.getNode(s)) { pick=true; break; }
      if(pick) selectedFrames.setAppend(f->ID);
    }
    return;
  }
  if(pickMode==byJointNames) {
    for(rai::String s:picks) {
      if(s(-2)==':') s.resize(s.N-2,true);
      rai::Frame *f = K.getFrameByName(s);
      if(!f) HALT("pick '" <<s <<"' not found");
      if(!f->joint) HALT("pick '" <<s <<"' is not a joint");
      selectedFrames.setAppend(f->ID);
    }
    return;
  }
  if(pickMode==byExcludeJointNames) {
    for(rai::Joint *j: K.activeJoints) {
        if(picks.contains(j->frame->name)) continue;
        selectedFrames.setAppend(j->frame->ID);
      }
    return;
  }
  NIY
}

F_qItself::F_qItself(const uintA& _selectedFrames, bool relative_q0)
  : selectedFrames(_selectedFrames), moduloTwoPi(true), relative_q0(relative_q0) {
}

void F_qItself::phi(arr& q, arr& J, const rai::Configuration& G) {
  if(!selectedFrames.nd) {
    q = G.getJointState();
    if(relative_q0) {
      for(rai::Joint* j: G.activeJoints) if(j->q0.N && j->qDim()==1) q(j->qIndex) -= j->q0.scalar();
    }
    if(!!J) J.setId(q.N);
  } else {
    uint n=dim_phi(G);
    q.resize(n);
    G.getJointState();
    if(!!J) J.resize(n, G.q.N).setZero();
    uint m=0;
    uint qIndex=0;
    if(selectedFrames.nd){
      for(uint i=0;i<selectedFrames.d0;i++) {
        rai::Joint *j=0;
        bool flipSign=false;
        if(selectedFrames.nd==1){
          rai::Frame *f = G.frames.elem(selectedFrames.elem(i));
          j = f->joint;
          CHECK(j, "selected frame " <<selectedFrames.elem(i) <<" ('" <<f->name <<"') is not a joint");
        }else{
          rai::Frame *a = G.frames.elem(selectedFrames(i,0));
          rai::Frame *b = G.frames.elem(selectedFrames(i,1));
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

void F_qItself::phi(arr& y, arr& J, const WorldL& Ktuple) {
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
  uintA selectedBodies_org = selectedFrames;
  if(selectedFrames.nd==1) {
    uintA sw = getSwitchedBodies(*Ktuple.elem(-2), *Ktuple.elem(-1));
    for(uint id:sw) selectedFrames.removeValue(id, false);
  }
  for(uint i=0; i<=k; i++) {
    phi(q_bar(i), J_bar(i), *Ktuple(offset+i));
  }
  selectedFrames = selectedBodies_org;
  
  bool handleSwitches=false;
  uint qN=q_bar(0).N;
  for(uint i=0; i<=k; i++) if(q_bar(i).N!=qN) { handleSwitches=true; break; }
  if(handleSwitches) { //when bodies are selected, switches don't have to be handled
    CHECK(!selectedFrames.nd, "doesn't work for this...")
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
          rai::Joint *jmatch = Ktuple(offset+s)->getJointByFrameNames(j->from()->name, j->frame->name);
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

    arr Jtau;  Ktuple(-1)->jacobian_time(Jtau, Ktuple(-1)->frames(0));  expandJacobian(Jtau, Ktuple, -1);
//    arr Jtau2;  Ktuple(-2)->jacobianTime(Jtau2, Ktuple(-2)->frames(0));  expandJacobian(Jtau2, Ktuple, -2);
//    arr Jtau = Jtau1 - Jtau2;
    if(k==1) J += (-1./tau)*y*Jtau;
  }
}

uint F_qItself::dim_phi(const rai::Configuration& G) {
  if(selectedFrames.nd) {
    uint n=0;
    for(uint i=0;i<selectedFrames.d0;i++) {
      rai::Joint *j=0;
      if(selectedFrames.nd==1){
        rai::Frame *f = G.frames.elem(selectedFrames.elem(i));
        j = f->joint;
        CHECK(j, "selected frame " <<selectedFrames.elem(i) <<" ('" <<f->name <<"') is not a joint");
      }else{
        rai::Frame *a = G.frames.elem(selectedFrames(i,0));
        rai::Frame *b = G.frames.elem(selectedFrames(i,1));
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

uint F_qItself::dim_phi(const WorldL& Ktuple) {
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

rai::String F_qItself::shortTag(const rai::Configuration& G) {
  rai::String s="qItself";
  if(selectedFrames.nd) {
    if(selectedFrames.N<=3) {
      for(uint b:selectedFrames) s <<'-' <<G.frames(b)->name;
    } else {
      s <<'#' <<selectedFrames.N;
    }
  } else {
    s <<"-ALL";
  }
  return s;
}

//===========================================================================

extern bool isSwitched(rai::Frame *f0, rai::Frame *f1);

void F_qZeroVel::phi(arr& y, arr& J, const WorldL& Ktuple){
  if(!Ktuple(-1)->frames(i)->joint){
    HALT("shouldn't be here");
    y.resize(0).setZero();
    if(!!J) J.resize(0, getKtupleDim(Ktuple).last()).setZero();
    return;
  }
  if(order==1 && isSwitched(Ktuple(-1)->frames(i), Ktuple(-2)->frames(i))){
    HALT("shouldn't be here");
    y.resize(Ktuple(-1)->frames(i)->joint->dim).setZero();
    if(!!J) J.resize(y.N, getKtupleDim(Ktuple).last()).setZero();
    return;
  }
  F_qItself q({(uint)i}, false);
  q.order=order;
  q.Feature::__phi(y, J, Ktuple);
  if(Ktuple(-1)->frames(i)->joint->type==rai::JT_transXYPhi){
    arr s = ARR(10.,10.,1.);
    y = s%y;
    if(!!J) J = s%J;
  }
  if(Ktuple(-1)->frames(i)->joint->type==rai::JT_free){
    arr s = ARR(10., 10., 10., 1., 1., 1., 1.);
    y = s%y;
    if(!!J) J = s%J;
  }
}

uint F_qZeroVel::dim_phi(const rai::Configuration& K){
  rai::Frame *a = K.frames(i);
  if(!a->joint){
    return 0;
//    LOG(-1) <<"reconfiguring to become link";
//    a = a->getUpwardLink();
//    i = a->ID;
  }
  return a->joint->dim;
}

//===========================================================================

rai::Array<rai::Joint*> getMatchingJoints(const WorldL& Ktuple, bool zeroVelJointsOnly) {
  rai::Array<rai::Joint*> matchingJoints;
  rai::Array<rai::Joint*> matches(Ktuple.N);
  bool matchIsGood;
  
  rai::Joint *j;
  for(rai::Frame *f:Ktuple.last()->frames) if((j=f->joint) && j->active && !zeroVelJointsOnly) {
      matches.setZero();
      matches.last() = j;
      matchIsGood=true;
      
      for(uint k=0; k<Ktuple.N-1; k++) { //go through other configs
        if(Ktuple(k)->frames.N<=j->frame->ID){ matchIsGood=false; break; }
        rai::Frame *fmatch = Ktuple(k)->frames(j->frame->ID);
        if(!fmatch){ matchIsGood=false; break; }
        rai::Joint *jmatch = fmatch->joint; //getJointByBodyIndices(j->from()->ID, j->frame->ID);
        if(!jmatch || j->type!=jmatch->type) {
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

void F_qLimits::phi(arr& y, arr& J, const rai::Configuration& G) {
//  if(!limits.N)
  limits=G.getLimits(); //G might change joint ordering (kinematic switches), need to query limits every time
  G.kinematicsLimitsCost(y, J, limits);
}

//===========================================================================

void F_qQuaternionNorms::phi(arr &y, arr &J, const rai::Configuration &G) {
  uint n=dim_phi(G);
  y.resize(n);
  if(!!J) J.resize(n, G.q.N).setZero();
  uint i=0;
  for(const rai::Joint *j: G.activeJoints) if(j->type==rai::JT_quatBall || j->type==rai::JT_free || j->type==rai::JT_XBall) {
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

uint F_qQuaternionNorms::dim_phi(const rai::Configuration &G) {
  uint i=0;
  for(const rai::Joint* j:G.activeJoints) {
    if(j->type==rai::JT_quatBall || j->type==rai::JT_free || j->type==rai::JT_XBall) i++;
  }
  return i;
}

//===========================================================================

rai::Array<rai::Joint*> getSwitchedJoints(const rai::Configuration& G0, const rai::Configuration& G1, int verbose) {

  HALT("retired: we only look at switched objects");
  
  rai::Array<rai::Joint*> switchedJoints;
  
  rai::Joint *j1;
  for(rai::Frame *f: G1.frames) if((j1=f->joint) && j1->active) {
      if(j1->from()->ID>=G0.frames.N || j1->frame->ID>=G0.frames.N) {
        switchedJoints.append({NULL,j1});
        continue;
      }
      rai::Joint *j0 = G0.getJointByFrameIndices(j1->from()->ID, j1->frame->ID);
      if(!j0 || j0->type!=j1->type) {
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
       || (j0->from() && j0->from()->ID!=j1->from()->ID)) { //different joint type; or attached to different parent
      return true;
    }
  }
  return false;
}

//===========================================================================

uintA getSwitchedBodies(const rai::Configuration& G0, const rai::Configuration& G1, int verbose) {
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

  rai::Configuration& K0 = *Ktuple(0);
  for(rai::Frame *f0:K0.frames) {
    bool succ = true;
    uint id = f0->ID;
    for(uint i=1;i<Ktuple.N;i++){
      rai::Configuration& K1 = *Ktuple(i);
      if(id>=K1.frames.N){ succ=false; break; }
      if(isSwitched(f0, K1.frames(id))){ succ=false; break; }
    }
    if(succ) nonSwitchedBodies.append(id);
  }
  return nonSwitchedBodies;
}
