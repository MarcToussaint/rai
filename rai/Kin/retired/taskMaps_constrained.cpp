/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "taskMaps.h"
#include "proxy.h"
#include "frame.h"

//===========================================================================

void CollisionConstraint::phi(arr& y, arr& J, const rai::Configuration& G) {
  G.kinematicsProxyCost(y, J, margin);
  y -= .5;
}

//===========================================================================

PairCollisionConstraint::PairCollisionConstraint(const rai::Configuration& G, const char* iShapeName, const char* jShapeName, double _margin)
  : i(G.getFrameByName(iShapeName)->ID),
    j(G.getFrameByName(jShapeName)->ID),
    margin(_margin) {
}

void PairCollisionConstraint::phi(arr& y, arr& J, const rai::Configuration& G) {
  y.resize(1) = -1.; //default value if not overwritten below
  if(!!J) J.resize(1, G.q.N).setZero();
  if(j>=0) { //against a concrete j
    for(const rai::Proxy& p: G.proxies) {
      if(((int)p.a->ID==i && (int)p.b->ID==j) || ((int)p.a->ID==j && (int)p.b->ID==i)) {
        G.kinematicsProxyConstraint(y, J, p, margin);
        break;
      }
    }
  } else if(j==-1) { //against all objects
    NIY; //this doesn't work, don't know why
    //first collect all relevant proxies
    rai::Array<const rai::Proxy*> P;
    for(const rai::Proxy& p: G.proxies) if(((int)p.a->ID==i) || ((int)p.b->ID==i)) P.append(&p);
    //Compute the softmax
    double alpha = 10.;
    double yHat=0., yNorm=0.;
    for(const rai::Proxy* p: P) {
      G.kinematicsProxyConstraint(y, NoArr, *p, margin);
      double yi=y.scalar();
      double expyi=::exp(alpha*yi);
      yNorm += expyi;
      yHat  += expyi * yi;
    }
    yHat /= yNorm;
    //compute derivative
    if(!!J) {
      J.resize(1, G.getJointStateDimension()).setZero();
      arr Ji;
      for(const rai::Proxy* p: P) {
        G.kinematicsProxyConstraint(y, Ji, *p, margin);
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

PlaneConstraint::PlaneConstraint(const rai::Configuration& G, const char* iShapeName, const arr& _planeParams)
  : i(G.getFrameByName(iShapeName)->ID), planeParams(_planeParams) {}

void PlaneConstraint::phi(arr& y, arr& J, const rai::Configuration& G) {
  rai::Frame* body_i = G.frames(i);
  rai::Vector vec_i = 0;

  arr y_eff, J_eff;
  G.kinematicsPos(y_eff, (!!J?J_eff:NoArr), body_i, vec_i);

  y_eff.append(1.); //homogeneous coordinates
  if(!!J) J_eff.append(zeros(1, J_eff.d1));

  y.resize(1);
  y(0) = scalarProduct(y_eff, planeParams);
  if(!!J) J = ~planeParams * J_eff;
}

//===========================================================================

void ConstraintStickiness::phi(arr& y, arr& J, const rai::Configuration& G) {
  map.__phi(y, J, G);
  for(uint j=0; j<y.N; j++) y(j) = -y(j);
  if(!!J) for(uint j=0; j<J.d0; j++) J[j]() *= -1.;
}

//===========================================================================

void PointEqualityConstraint::phi(arr& y, arr& J, const rai::Configuration& G) {
  rai::Vector vec_i = ivec;
  rai::Vector vec_j = jvec;
  rai::Frame* a = i<0?nullptr: G.frames(i);
  rai::Frame* b = j<0?nullptr: G.frames(j);
  rai::Vector pi = a ? a->ensure_X() * vec_i : vec_i;
  rai::Vector pj = b ? b->ensure_X() * vec_j : vec_j;
  y = conv_vec2arr(pi-pj);
  if(!!J) {
    arr Ji, Jj;
    G.kinematicsPos(NoArr, Ji, a, vec_i);
    if(b) {
      G.kinematicsPos(NoArr, Jj, b, vec_j);
      J = Ji - Jj;
    } else {
      J = Ji;
    }
  }
}

//===========================================================================

ContactEqualityConstraint::ContactEqualityConstraint(const rai::Configuration& G, const char* iShapeName, const char* jShapeName, double _margin)
  : i(G.getFrameByName(iShapeName)->ID),
    j(G.getFrameByName(jShapeName)->ID),
    margin(_margin) {
}

void ContactEqualityConstraint::phi(arr& y, arr& J, const rai::Configuration& G) {
  y.resize(1) = 0.;
  if(!!J) J.resize(1, G.q.N).setZero();
  for(const rai::Proxy& p: G.proxies) {
    if(((int)p.a->ID==i && (int)p.b->ID==j) || ((int)p.a->ID==j && (int)p.b->ID==i)) {
      G.kinematicsProxyConstraint(y, J, p, margin);
      break;
    }
  }
}

//===========================================================================

VelAlignConstraint::VelAlignConstraint(const rai::Configuration& G,
                                       const char* iShapeName, const rai::Vector& _ivec,
                                       const char* jShapeName, const rai::Vector& _jvec, double _target) {
  rai::Frame* a = iShapeName ? G.getFrameByName(iShapeName):nullptr;
  rai::Frame* b = jShapeName ? G.getFrameByName(jShapeName):nullptr;
  if(a) i=a->ID;
  if(b) j=b->ID;
  if(!!_ivec) ivec=_ivec; else ivec.setZero();
  if(!!_jvec) jvec=_jvec; else jvec.setZero();
  order = 1;
  target = _target;
}

void VelAlignConstraint::phi(arr& y, arr& J, const ConfigurationL& G) {
  uint k=order;

  // compute body j orientation
  arr y_j, J_j, J_bar_j;
  G(G.N-1)->kinematicsVec(y_j, (!!J?J_bar_j:NoArr), G(G.N-1)->frames(j), jvec);

  if(!!J) {
    J_j = zeros(G.N, y_j.N, J_bar_j.d1);
    J_j[G.N-1]() = J_bar_j;
    arr tmp(J_j);
    tensorPermutation(J_j, tmp, TUP(1u, 0u, 2u));
    J_j.reshape(y_j.N, G.N*J_bar_j.d1);
  }

  // compute body i velocity
  arrA y_bar, J_bar;
  y_bar.resize(k+1);
  J_bar.resize(k+1);

  for(uint c=0; c<=k; c++) {
    G(G.N-1-c)->kinematicsPos(y_bar(c), (!!J?J_bar(c):NoArr), G(G.N-1-c)->frames(i), ivec);
  }

  arr dy_i, dJ_i;
  dy_i = (y_bar(0)-y_bar(1));

  if(!!J) {
    dJ_i = zeros(G.N, dy_i.N, J_bar(0).d1);
    dJ_i[G.N-1-1]() = -J_bar(1);
    dJ_i[G.N-1-0]() = J_bar(0);
    arr tmp(dJ_i);
    tensorPermutation(dJ_i, tmp, TUP(1u, 0u, 2u));
    dJ_i.reshape(dy_i.N, G.N*J_bar(0).d1);
  }

  // normalize dy_i
  if(length(dy_i) != 0) {
    if(!!J) {
      double tmp = (~dy_i*dy_i).scalar();
      dJ_i = (eye(dJ_i.d0) - (dy_i*~dy_i)/(tmp))*dJ_i/(length(dy_i));
    }
    dy_i = dy_i/(length(dy_i));
  }

  innerProduct(y, ~dy_i, y_j);

  if(!!J) {
    J = ~dy_i*J_j + ~y_j*dJ_i;
    J = -J;
  }
  y = -y+target;

}

//===========================================================================

void qItselfConstraint::phi(arr& q, arr& J, const rai::Configuration& G) {
  G.getJointState(q);
  if(M.N) {
    if(M.nd==1) {
      q=M%q; if(!!J) J.setDiag(M);
    } else {
      q=M*q; if(!!J) J=M;
    }
  } else {
    if(!!J) J.setId(q.N);
  }
}

