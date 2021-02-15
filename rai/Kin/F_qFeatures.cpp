/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_qFeatures.h"
#include "frame.h"
#include <climits>

//===========================================================================

F_qItself::F_qItself(bool relative_q0) : relative_q0(relative_q0) {}

F_qItself::F_qItself(PickMode pickMode, const StringA& picks, const rai::Configuration& C, bool relative_q0)
  : relative_q0(relative_q0) {
  if(pickMode==allActiveJoints) {
    for(rai::Frame* f: C.frames) if(f->joint && f->parent && f->joint->active && f->joint->dim!=0){
      frameIDs.append(f->ID);
      frameIDs.append(f->parent->ID);
    }
    frameIDs.reshape(-1,2);
  }else if(pickMode==byJointGroups) {
    for(rai::Frame* f: C.frames) {
      bool pick=false;
      for(const rai::String& s:picks) if(f->ats.getNode(s)) { pick=true; break; }
      if(pick) frameIDs.setAppend(f->ID);
    }
  }else if(pickMode==byJointNames) {
    for(rai::String s:picks) {
      if(s(-2)==':') s.resize(s.N-2, true);
      rai::Frame* f = C.getFrame(s);
      if(!f) HALT("pick '" <<s <<"' not found");
      if(!f->joint) HALT("pick '" <<s <<"' is not a joint");
      frameIDs.setAppend(f->ID);
    }
  }else if(pickMode==byExcludeJointNames) {
    for(rai::Joint* j: C.activeJoints) {
      if(picks.contains(j->frame->name)) continue;
      frameIDs.setAppend(j->frame->ID);
    }
  }else{
    NIY
  }
}

F_qItself::F_qItself(const uintA& _selectedFrames, bool relative_q0)
  : relative_q0(relative_q0) {
  frameIDs = _selectedFrames;
  fs = FS_qItself;
}

void F_qItself::phi(arr& q, arr& J, const rai::Configuration& C) {
  CHECK(C._state_q_isGood, "");
  if(!frameIDs.nd) {
    q = C.getJointState();
    if(relative_q0) {
      for(rai::Joint* j: C.activeJoints) if(j->q0.N && j->qDim()==1) q(j->qIndex) -= j->q0.scalar();
    }
    if(!!J) J.setId(q.N);
  } else {
    uint n=dim_phi(C);
    C.kinematicsZero(q, J, n);
    uint m=0;
    if(frameIDs.nd) {
      for(uint i=0; i<frameIDs.d0; i++) {
        rai::Joint* j=0;
        bool flipSign=false;
        if(frameIDs.nd==1) {
          rai::Frame* f = C.frames.elem(frameIDs.elem(i));
          j = f->joint;
          CHECK(j, "selected frame " <<frameIDs.elem(i) <<" ('" <<f->name <<"') is not a joint");
        } else {
          rai::Frame* a = C.frames.elem(frameIDs(i, 0));
          rai::Frame* b = C.frames.elem(frameIDs(i, 1));
          if(a->parent==b) j=a->joint;
          else if(b->parent==a) { j=b->joint; flipSign=true; }
          else HALT("a and b are not linked");
          CHECK(j, "");
        }
        for(uint k=0; k<j->dim; k++) {
          if(j->active){
            q.elem(m) = C.q.elem(j->qIndex+k);
          }else{
            q.elem(m) = C.qInactive.elem(j->qIndex+k);
          }
          if(flipSign) q.elem(m) *= -1.;
          if(relative_q0 && j->q0.N) q.elem(m) -= j->q0(k);
          if(!!J && j->active) {
            if(flipSign) J.elem(m, j->qIndex+k) = -1.;
            else J.elem(m, j->qIndex+k) = 1.;
          }
          m++;
        }
      }
      CHECK_EQ(n, m, "");
    }
  }
}

void F_qItself::phi2(arr& q, arr& J, const FrameL& F) {
  if(order!=0){
    Feature::phi2(q, J, F);
    return;
  }
  uint n=dim_phi2(F);
  if(!n){ q.clear(); J.clear(); return; }
  rai::Configuration& C = F.last()->C;
  CHECK(C._state_q_isGood, "");
  C.kinematicsZero(q, J, n);
  uint m=0;
  CHECK(F.d0==1, "");
  FrameL FF = F[0];
//  FF.reshape(-1,2);
  for(uint i=0; i<FF.d0; i++) {
    rai::Joint* j=0;
    bool flipSign=false;
    if(FF.nd==1) {
      rai::Frame* f = FF.elem(i);
      j = f->joint;
      CHECK(j, "selected frame " <<FF.elem(i) <<" ('" <<f->name <<"') is not a joint");
    } else {
      rai::Frame* a = FF(i, 0);
      rai::Frame* b = FF(i, 1);
      if(a->parent==b) j=a->joint;
      else if(b->parent==a) { j=b->joint; flipSign=true; }
      else HALT("a and b are not linked");
      CHECK(j, "");
    }
    for(uint k=0; k<j->dim; k++) {
      if(j->active){
        q.elem(m) = C.q.elem(j->qIndex+k);
      }else{
        q.elem(m) = C.qInactive.elem(j->qIndex+k);
      }
      if(flipSign) q.elem(m) *= -1.;
      if(relative_q0 && j->q0.N) q.elem(m) -= j->q0(k);
      if(!!J && j->active) {
        if(flipSign) J.elem(m, j->qIndex+k) = -1.;
        else J.elem(m, j->qIndex+k) = 1.;
      }
      m++;
    }
  }
  CHECK_EQ(n, m, "");
}


uint F_qItself::dim_phi(const rai::Configuration& C) {
  if(frameIDs.nd) {
    uint n=0;
    for(uint i=0; i<frameIDs.d0; i++) {
      rai::Joint* j=0;
      if(frameIDs.nd==1) {
        rai::Frame* f = C.frames.elem(frameIDs.elem(i));
        j = f->joint;
        CHECK(j, "selected frame " <<frameIDs.elem(i) <<" ('" <<f->name <<"') is not a joint");
      } else {
        rai::Frame* a = C.frames.elem(frameIDs(i, 0));
        rai::Frame* b = C.frames.elem(frameIDs(i, 1));
        if(a->parent==b) j=a->joint;
        else if(b->parent==a) j=b->joint;
        else HALT("a (" <<a->name <<") and b (" <<b->name <<") are not linked");
        CHECK(j, "");
      }
      n += j->qDim();
    }
    return n;
  }
  return C.getJointStateDimension();
}

uint F_qItself::dim_phi2(const FrameL& F){
  uint m=0;
  FrameL FF = F[0];
  for(uint i=0; i<FF.d0; i++) {
    rai::Joint* j=0;
    if(FF.nd==1) {
      rai::Frame* f = FF.elem(i);
      j = f->joint;
      CHECK(j, "selected frame " <<FF.elem(i) <<" ('" <<f->name <<"') is not a joint");
    } else {
      rai::Frame* a = FF(i, 0);
      rai::Frame* b = FF(i, 1);
      if(a->parent==b) j=a->joint;
      else if(b->parent==a) j=b->joint;
      CHECK(j, "a (" <<a->name <<") and b (" <<b->name <<") are not linked");
    }
    m += j->dim;
  }
  return m;
}

//===========================================================================

void F_qZeroVel::phi2(arr& y, arr& J, const FrameL& F){
  CHECK_EQ(order, 1, "");
  F_qItself()
      .setOrder(order)
      .__phi2(y, J, F);
#if 1
  rai::Frame *f = F.last();
  if(f->joint->type==rai::JT_transXYPhi) {
    arr s = ARR(10., 10., 1.);
    y = s%y;
    if(!!J) J = s%J;
  }
  if(f->joint->type==rai::JT_free) {
    arr s = ARR(10., 10., 10., 1., 1., 1., 1.);
    y = s%y;
    if(!!J) J = s%J;
  }
#endif
}

uint F_qZeroVel::dim_phi2(const FrameL& F){
  return F_qItself()
      .setOrder(order)
      .__dim_phi2(F);
}

//===========================================================================

rai::Array<rai::Joint*> getMatchingJoints(const ConfigurationL& Ktuple, bool zeroVelJointsOnly) {
  rai::Array<rai::Joint*> matchingJoints;
  rai::Array<rai::Joint*> matches(Ktuple.N);
  bool matchIsGood;

  rai::Joint* j;
  for(rai::Frame* f:Ktuple.last()->frames) if((j=f->joint) && j->active && !zeroVelJointsOnly) {
      matches.setZero();
      matches.last() = j;
      matchIsGood=true;

      for(uint k=0; k<Ktuple.N-1; k++) { //go through other configs
        if(Ktuple(k)->frames.N<=j->frame->ID) { matchIsGood=false; break; }
        rai::Frame* fmatch = Ktuple(k)->frames.elem(j->frame->ID);
        if(!fmatch) { matchIsGood=false; break; }
        rai::Joint* jmatch = fmatch->joint; //getJointByBodyIndices(j->from()->ID, j->frame->ID);
        if(!jmatch || j->type!=jmatch->type) {
          matchIsGood=false;
          break;
        }
        if(j->from() && j->from()->ID!=jmatch->from()->ID) {
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

void F_qLimits2::phi2(arr& y, arr& J, const FrameL& F){
  uint M = dim_phi2(F);
  F.last()->C.kinematicsZero(y, J, M);
  uint m=0;
  for(rai::Frame *f: F){
    rai::Joint *j = f->joint;
    if(!j) continue;
    if(!j->limits.N) continue;
    uint d=j->qDim();
    for(uint k=0; k<d; k++) { //in case joint has multiple dimensions
      double lo = j->limits(2*k+0);
      double up = j->limits(2*k+1);
      uint i = j->qIndex+k;
      y.elem(m) = lo - f->C.q(i);
      if(!!J) J.elem(m, i) -= 1.;
      m++;
      y.elem(m) = f->C.q(i) - up;
      if(!!J) J.elem(m, i) += 1.;
      m++;
    }
  }
}

uint F_qLimits2::dim_phi2(const FrameL& F) {
  uint m=0;
  for(rai::Frame *f: F){
    rai::Joint *j = f->joint;
    if(!j) continue;
    if(!j->limits.N) continue;
    m += 2*j->qDim();
  }
  return m;
}

//===========================================================================

void F_qLimits::phi(arr& y, arr& J, const rai::Configuration& G) {
//  if(!limits.N)
  limits=G.getLimits(); //G might change joint ordering (kinematic switches), need to query limits every time
  NIY//G.kinematicsLimits(y, J, limits);
}

//===========================================================================

void F_qQuaternionNorms::phi2(arr& y, arr& J, const FrameL& F) {
  uint n=dim_phi2(F);
  if(!n){ y.clear(); J.clear(); return; }
  rai::Configuration& C=F.first()->C;
  C.kinematicsZero(y, J, n);
  uint i=0;
  for(const rai::Frame* f:F) {
    rai::Joint *j = f->joint;
    if(!j) continue;
    if(j->type==rai::JT_quatBall || j->type==rai::JT_free || j->type==rai::JT_XBall) {
      arr q;
      if(j->type==rai::JT_quatBall) q.referToRange(C.q, j->qIndex+0, j->qIndex+3);
      if(j->type==rai::JT_XBall)    q.referToRange(C.q, j->qIndex+1, j->qIndex+4);
      if(j->type==rai::JT_free)     q.referToRange(C.q, j->qIndex+3, j->qIndex+6);
      double norm = sumOfSqr(q);
      y(i) = norm - 1.;

      if(!!J) {
        if(j->type==rai::JT_quatBall) for(uint k=0;k<4;k++) J.elem(i,j->qIndex+0+k) = 2.*q.elem(k);
        if(j->type==rai::JT_XBall)    for(uint k=0;k<4;k++) J.elem(i,j->qIndex+1+k) = 2.*q.elem(k);
        if(j->type==rai::JT_free)     for(uint k=0;k<4;k++) J.elem(i,j->qIndex+3+k) = 2.*q.elem(k);
      }
      i++;
    }
  }
}

uint F_qQuaternionNorms::dim_phi2(const FrameL& F) {
  uint n=0;
  for(const rai::Frame* f:F) {
    rai::Joint *j = f->joint;
    if(!j) continue;
    if(j->type==rai::JT_quatBall || j->type==rai::JT_free || j->type==rai::JT_XBall) n++;
  }
  return n;
}

void F_qQuaternionNorms::setAllActiveQuats(const rai::Configuration& C){
  frameIDs.clear();
  for(const rai::Joint* j:C.activeJoints) {
    if(j->type==rai::JT_quatBall || j->type==rai::JT_free || j->type==rai::JT_XBall) frameIDs.append(j->frame->ID);
  }
}

//===========================================================================

void F_qTime::phi2(arr& y, arr& J, const FrameL& F) {
  if(order==0) {
    rai::Frame *f = F.scalar();
    double tau;
    f->C.kinematicsTau(tau, J, f);
    y.resize(1) = tau;
  }
  if(order==1) { //WARNING: this is neg velocity... for ineq constraint
    CHECK_EQ(F.N, 2, "");
    arr y0, y1, J0, J1;
    order=0;
    phi2(y0, J0, {F.elem(0)});
    phi2(y1, J1, {F.elem(1)});
    order=1;
    y = y0 - y1;
    if(!!J) J = J0 - J1;
  }
  if(order==2) {
    CHECK_EQ(F.N, 3, "");
    arr y0, y1, y2, J0, J1, J2;
    order=0;
    phi2(y0, J0, {F.elem(0)});
    phi2(y1, J1, {F.elem(1)});
    phi2(y2, J2, {F.elem(2)});
    order=2;
    y = y2 - 2.*y1 + y0;
    if(!!J)  J = J2 - 2.*J1 + J0;
  }
}

//===========================================================================

uintA getNonSwitchedFrames(const FrameL& A, const FrameL& B) {
  uintA nonSwitchedFrames;
  CHECK_EQ(A.N, B.N, "");

  for(uint i=0;i<A.N;i++) {
    rai::Frame* f0 = A.elem(i);
    rai::Frame* f1 = B.elem(i);
    if(!f0->joint || !f1->joint) continue;
    if(f0->joint->type!=f1->joint->type) continue;
    if(f0->joint->mimic || f1->joint->mimic) continue;
    if(f0->ID - f0->parent->ID != f1->ID-f1->parent->ID) continue; //comparing the DIFFERENCE in IDs between parent and joint
    nonSwitchedFrames.append(i);
  }
  return nonSwitchedFrames;
}
