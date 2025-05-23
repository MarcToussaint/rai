/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_qFeatures.h"

#include "frame.h"
#include "dof_forceExchange.h"
#include "dof_path.h"
#include "dof_direction.h"

#include <climits>

//===========================================================================

// F_qItself::F_qItself(bool relative_q0) : relative_q0(relative_q0) {}

// F_qItself::F_qItself(PickMode pickMode, const StringA& picks, const rai::Configuration& C, bool relative_q0)
//   : relative_q0(relative_q0) {
//   if(pickMode==allActiveJoints) {
//     for(rai::Frame* f: C.frames) if(f->joint && f->parent && f->joint->active && f->joint->dim!=0) {
//         frameIDs.append(f->ID);
//         frameIDs.append(f->parent->ID);
//       }
//     frameIDs.reshape(-1, 2);
//   } else if(pickMode==byJointNames) {
//     for(rai::String s:picks) {
//       if(s(-2)==':') s.resize(s.N-2, true);
//       rai::Frame* f = C.getFrame(s);
//       if(!f) HALT("pick '" <<s <<"' not found");
//       if(!f->joint) HALT("pick '" <<s <<"' is not a joint");
//       frameIDs.setAppend(f->ID);
//     }
//   } else if(pickMode==byExcludeJointNames) {
//     for(rai::Dof* j: C.activeDofs) {
//       if(picks.contains(j->frame->name)) continue;
//       frameIDs.setAppend(j->frame->ID);
//     }
//   } else {
//     NIY
//   }
// }

F_qItself::F_qItself(const uintA& _selectedFrames, bool relative_q0)
  : relative_q0(relative_q0) {
  frameIDs = _selectedFrames;
  fs = FS_qItself;
}

void F_qItself::selectActiveJointPairs(const FrameL& F){
    for(rai::Frame* f: F) if(f->joint && f->parent && f->joint->active && f->joint->dim!=0) {
        frameIDs.append(f->ID);
        frameIDs.append(f->parent->ID);
      }
    frameIDs.reshape(-1, 2);
}

#if 0
void F_qItself::phi(arr& q, arr& J, const rai::Configuration& C) {
  HALT("you're here??");
  CHECK(C._state_q_isGood, "");
  if(!frameIDs.nd) {
    q = C.getJointState();
    if(relative_q0) {
      for(rai::Dof* j: C.activeDofs) if(j->joint() && j->dim==1 && j->joint()->q0.N) q(j->qIndex) -= j->joint()->q0.scalar();
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
          if(j->active) {
            q.elem(m) = C.q.elem(j->qIndex+k);
          } else {
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
#endif

arr F_qItself::phi(const FrameL& F) {
  if(order!=0) return phi_finiteDifferenceReduce(F);
  uint n=dim_phi(F);
  if(!n) return arr{};
  rai::Configuration& C = F.last()->C;
  CHECK(C._state_q_isGood, "");
  arr q;
  C.kinematicsZero(q, q.J(), n);
  uint m=0;
  CHECK(F.d0==1, "");
  FrameL FF = F[0];
//  FF.reshape(-1,2);
  for(uint i=0; i<FF.d0; i++) {
    rai::Dof* j=0;
    bool flipSign=false;
    if(FF.nd==1) {
      rai::Frame* f = FF.elem(i);
      j = f->joint;
      if(!j) j = f->pathDof;
      CHECK(j, "selected frame " <<FF.elem(i) <<" ('" <<f->name <<"') is not a joint or pathDof");
    } else {
      rai::Frame* a = FF(i, 0);
      rai::Frame* b = FF(i, 1);
      if(a->parent==b) j=a->joint;
      else if(b->parent==a) { j=b->joint; flipSign=true; }
      else HALT("a and b are not linked");
      CHECK(j, "selected frame " <<FF(i,0) <<" ('" <<a->name <<"') is not a joint or pathDof");
    }
    for(uint k=0; k<j->dim; k++) {
      if(j->active) {
        q.p[m] = C.q.p[j->qIndex+k];
      } else {
        q.p[m] = C.qInactive.p[j->qIndex+k];
      }
      if(flipSign) q.elem(m) *= -1.;
      if(relative_q0 && j->q0.N) q.elem(m) -= j->q0(k);
      // if(!!J && j->active) {
      if(j->active){
        if(flipSign) q.J().elem(m, j->qIndex+k) = -1.;
        else q.J().elem(m, j->qIndex+k) = 1.;
      }
      m++;
    }
  }
  CHECK_EQ(n, m, "");
  return q;
}

// uint F_qItself::dim_phi(const rai::Configuration& C) {
//   if(frameIDs.nd) {
//     uint n=0;
//     for(uint i=0; i<frameIDs.d0; i++) {
//       rai::Joint* j=0;
//       if(frameIDs.nd==1) {
//         rai::Frame* f = C.frames.elem(frameIDs.elem(i));
//         j = f->joint;
//         CHECK(j, "selected frame " <<frameIDs.elem(i) <<" ('" <<f->name <<"') is not a joint");
//       } else {
//         rai::Frame* a = C.frames.elem(frameIDs(i, 0));
//         rai::Frame* b = C.frames.elem(frameIDs(i, 1));
//         if(a->parent==b) j=a->joint;
//         else if(b->parent==a) j=b->joint;
//         else HALT("a (" <<a->name <<") and b (" <<b->name <<") are not linked");
//         CHECK(j, "");
//       }
//       n += j->dim;
//     }
//     return n;
//   }
//   return C.getJointStateDimension();
// }

uint F_qItself::dim_phi(const FrameL& F) {
  uint m=0;
  FrameL FF = F[0];
  for(uint i=0; i<FF.d0; i++) {
    rai::Dof* j=0;
    if(FF.nd==1) {
      rai::Frame* f = FF.elem(i);
      j = f->joint;
      if(!j) j = f->pathDof;
      CHECK(j, "selected frame " <<FF.elem(i) <<" ('" <<f->name <<"') is not a joint or pathDof");
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

void F_q0Bias::phi2(arr& y, arr& J, const FrameL& F) {
  uint n=dim_phi(F);
  if(!n) { y.clear(); J.clear(); return; }
  rai::Configuration& C = F.last()->C;
  CHECK(C._state_q_isGood, "");
  C.kinematicsZero(y, J, n);
  uint m=0;
  for(rai::Frame* f:F) {
    rai::Dof* d = f->getDof();
    if(!d || !d->q0.N) continue;
    for(uint k=0; k<d->dim; k++) {
      if(d->active) {
        y.elem(m) = C.q.elem(d->qIndex+k);
      } else {
        y.elem(m) = C.qInactive.elem(d->qIndex+k);
      }
      y.elem(m) -= d->q0(k);
      if(!!J && d->active) J.elem(m, d->qIndex+k) = 1.;
      m++;
    }
  }
  CHECK_EQ(n, m, "");
}

uint F_q0Bias::dim_phi(const FrameL& F) {
  uint m=0;
  for(rai::Frame* f:F) {
    rai::Dof* d = f->getDof();
    if(!d || !d->q0.N) continue;
    CHECK_EQ(d->q0.N, d->dim, "");
    m+=d->dim;
  }
  return m;
}

//===========================================================================

void F_qZeroVel::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 1, "");
  y = F_qItself()
      .setOrder(order)
      .eval(F);
  //J = y.J();
#if 1
  rai::Frame* f = F.last();
  if(f->joint->type==rai::JT_transXYPhi) {
    arr s = arr{10., 10., 1.};
    y = s%y;
    //if(!!J) J = s%J;
  }
  if(f->joint->type==rai::JT_free) {
    arr s = arr{10., 10., 10., 1., 1., 1., 1.};
    y = s%y;
    //if(!!J) J = s%J;
  }
#endif
  if(!!J) J = y.J_reset();
}

uint F_qZeroVel::dim_phi(const FrameL& F) {
  return F_qItself()
         .setOrder(order)
         .dim(F);
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

DofL getDofs(const FrameL& F) {
  DofL dofs;
  for(rai::Frame* f: F) {
    if(f->joint && f->joint->active) {
      if(f->joint->limits.N) dofs.append(f->joint);
    }
    for(rai::ForceExchangeDof* fex:f->forces) if(&fex->a==f) {
        if(fex->active && fex->limits.N) dofs.append(fex);
      }
  }
  return dofs;
}

void F_qLimits::phi2(arr& y, arr& J, const FrameL& F) {
  uint M = dim_phi(F);
  F.last()->C.kinematicsZero(y, J, M);
  CHECK(F.last()->C._state_q_isGood, "");
  uint m=0;
  DofL dofs = getDofs(F);
  for(rai::Dof* dof: dofs) if(dof->limits.N) {
      for(uint k=0; k<dof->dim; k++) { //in case joint has multiple dimensions
        double lo = dof->limits.elem(k);
        double up = dof->limits.elem(dof->dim+k);
        if(up>=lo) {
          uint i = dof->qIndex+k;
          double qi = F.last()->C.q(i);
//        if(true){
//          if(qi < lo) LOG(0) <<dof->name() <<' ' <<k <<' ' <<qi <<'<' <<lo <<" violates lower limit";
//          if(qi > up) LOG(0) <<dof->name() <<' ' <<k <<' ' <<qi <<'>' <<up <<" violates upper limit";
//        }
          y.elem(m) = lo - qi;
          if(!!J) J.elem(m, i) -= 1.;
          m++;
          y.elem(m) = qi - up;
          if(!!J) J.elem(m, i) += 1.;
          m++;
        } else {
          m+=2;
        }
      }
    }
  CHECK_EQ(m, M, "");
}

uint F_qLimits::dim_phi(const FrameL& F) {
  uint m=0;
  DofL dofs = getDofs(F);
  for(rai::Dof* dof: dofs) if(dof->limits.N) m += 2*dof->dim;
  return m;
}

//===========================================================================

void F_qQuaternionNorms::phi2(arr& y, arr& J, const FrameL& F) {
  uint n=dim_phi(F);
  if(!n) { y.clear(); J.clear(); return; }
  rai::Configuration& C=F.first()->C;
  C.kinematicsZero(y, J, n);
  uint i=0;
  for(const rai::Frame* f:F) {
    {
      rai::Joint* j = f->joint;
      if(j && j->active && (j->type==rai::JT_circleZ || j->type==rai::JT_quatBall || j->type==rai::JT_free || j->type==rai::JT_XBall)) {
        arr q;
        if(j->type==rai::JT_circleZ)  q.referToRange(C.q, {j->qIndex+0, j->qIndex+1+1});
        if(j->type==rai::JT_quatBall) q.referToRange(C.q, {j->qIndex+0, j->qIndex+3+1});
        if(j->type==rai::JT_XBall)    q.referToRange(C.q, {j->qIndex+1, j->qIndex+4+1});
        if(j->type==rai::JT_free)     q.referToRange(C.q, {j->qIndex+3, j->qIndex+6+1});
        double norm = sumOfSqr(q);
        y(i) = norm - 1.;

	if(!!J) {
	  if(j->type==rai::JT_circleZ)  for(uint k=0; k<2; k++) J.elem(i, j->qIndex+0+k) = 2.*q.elem(k);
	  if(j->type==rai::JT_quatBall) for(uint k=0; k<4; k++) J.elem(i, j->qIndex+0+k) = 2.*q.elem(k);
	  if(j->type==rai::JT_XBall)    for(uint k=0; k<4; k++) J.elem(i, j->qIndex+1+k) = 2.*q.elem(k);
	  if(j->type==rai::JT_free)     for(uint k=0; k<4; k++) J.elem(i, j->qIndex+3+k) = 2.*q.elem(k);
	}
	i++;
      }
    }
    {
      rai::DirectionDof* dof = f->dirDof;
      if(dof && dof->active){
        arr q;
        q.referToRange(C.q, {dof->qIndex+0, dof->qIndex+2+1});
        double norm = sumOfSqr(q);
        y(i) = norm - 1.;

	if(!!J) {
	  for(uint k=0; k<3; k++) J.elem(i, dof->qIndex+k) = 2.*q.elem(k);
	}
	i++;
      }
    }
  }
}

uint F_qQuaternionNorms::dim_phi(const FrameL& F) {
  uint n=0;
  for(const rai::Frame* f:F) {
    rai::Joint* j = f->joint;
    if(j && j->active && (j->type==rai::JT_circleZ || j->type==rai::JT_quatBall || j->type==rai::JT_free || j->type==rai::JT_XBall)) n++;

    rai::DirectionDof* dof = f->dirDof;
    if(dof && dof->active) n++;
  }
  return n;
}

void F_qQuaternionNorms::setAllActiveQuats(const rai::Configuration& C) {
  frameIDs.clear();
  for(const rai::Dof* dof:C.activeDofs) {
    const rai::Joint* j = dof->joint();
    const rai::DirectionDof* dir = dof->frame->dirDof;

    if((j && (j->type==rai::JT_circleZ || j->type==rai::JT_quatBall || j->type==rai::JT_free || j->type==rai::JT_XBall)) || (dir)) frameIDs.append(j->frame->ID);
  }
}

//===========================================================================

void F_qTime::phi2(arr& y, arr& J, const FrameL& F) {
  if(order==0) {
    rai::Frame* f = F.scalar();
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

  for(uint i=0; i<A.N; i++) {
    rai::Frame* f0 = A.elem(i);
    rai::Frame* f1 = B.elem(i);
    if(!f0->joint || !f1->joint) continue;
    if(f0->joint->type!=f1->joint->type) continue;
    if(f0->joint->mimic || f1->joint->mimic) continue;
    if(f0->ID - f0->parent->ID != f1->ID-f1->parent->ID) continue; //comparing the DIFFERENCE in IDs between parent and joint
    if(f0->forces.N != f1->forces.N) continue;
    nonSwitchedFrames.append(i);
  }
  return nonSwitchedFrames;
}

uintA getSwitchedFrames(const FrameL& A, const FrameL& B) {
  uintA switchedFrames;
  CHECK_EQ(A.N, B.N, "");

  for(uint i=0; i<A.N; i++) {
    rai::Frame* f0 = A.elem(i);
    rai::Frame* f1 = B.elem(i);
    if(!f0->parent && !f1->parent) continue;
    if(f0->parent && f1->parent && f0->ID - f0->parent->ID == f1->ID - f1->parent->ID) continue;
    switchedFrames.append(i);
  }
  return switchedFrames;
}

