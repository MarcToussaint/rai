/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "../Kin/proxy.h"
#include "../Kin/feature.h"
#include "../Optim/constrained.h"
#include "../Geo/fclInterface.h"

#include "ConfigurationProblem.h"

ConfigurationProblem::ConfigurationProblem(rai::Configuration& _C, bool _useBroadCollisions, double _collisionTolerance, int _verbose)
  : C(_C),
    useBroadCollisions(_useBroadCollisions),
    collisionTolerance(_collisionTolerance),
    verbose(_verbose) {

  limits = C.getJointLimits();

  C.ensure_q();
  for(rai::Dof* dof:C.activeDofs) if(dof->joint()){
    if(dof->joint()->type==rai::JT_circleZ || dof->joint()->type==rai::JT_quatBall){
      sphericalCoordinates.append(uintA{dof->qIndex, dof->dim});
    }
  }
  sphericalCoordinates.reshape(-1,2);

  // C.fcl()->mode = rai::FclInterface::_distanceCutoff;
  C.fcl()->mode = rai::FclInterface::_broadPhaseOnly;
}

void ConfigurationProblem::setExplicitCollisionPairs(const StringA& _collisionPairs) {
  useBroadCollisions = false;
  collisionPairs = C.getFrameIDs(_collisionPairs);
  collisionPairs.reshape(-1, 2);
}

shared_ptr<QueryResult> ConfigurationProblem::query(const arr& x) {
  // if(limits.N) {
  //   for(uint i=0; i<x.N; i++) {
  //     if(limits(1, i)>limits(0, i) && (x.elem(i)<limits(0, i) || x.elem(i)>limits(1, i))) {
  //       //LOG(-1) <<"QUERY OUT OF LIMIT: joint " <<i <<": " <<x.elem(i) <<' ' <<limits[i];
  //     }
  //   }
  // }

  C.setJointState(x);
  if(useBroadCollisions) {
    C.stepFcl();
  } else {
    //CHECK(collisionPairs.N, "you need either explicit collision pairs or useBroadCollisions");
    C.proxies.resize(collisionPairs.d0);
    for(uint i=0; i<collisionPairs.d0; i++) {
      C.proxies(i).a = C.frames(collisionPairs(i, 0));
      C.proxies(i).b = C.frames(collisionPairs(i, 1));
      C.proxies(i).d = -0.;
    }
    for(rai::Proxy& p:C.proxies) p.calc_coll();
    C._state_proxies_isGood = true;
  }
  evals++;

  //C.view();

  shared_ptr<QueryResult> qr = make_shared<QueryResult>();

#if 1
    double D=0.;
    for(rai::Proxy& p:C.proxies){
      p.calc_coll();
      if(p.d<0.) D -= p.d;
    }
    qr->totalCollision = D;
    qr->isFeasible = (qr->totalCollision<collisionTolerance);
#else
    qr->totalCollision = C.getTotalPenetration();
    qr->isFeasible = (qr->totalCollision<collisionTolerance);
#endif

  //display (link of last joint)
  qr->disp3d = C.activeDofs.elem(-1)->frame->getPosition();
  if(verbose) {
    C.view(verbose>1, STRING("ConfigurationProblem query:\n" <<*qr));
  }

  return qr;
}

void QueryResult::write(std::ostream& os) const {
  os <<" isFeasible: " <<isFeasible;
}

void QueryResult::writeDetails(std::ostream& os, const ConfigurationProblem& P, double margin) const {
  write(os);
  for(const rai::Proxy& p:P.C.proxies) if(p.d<=margin) {
      os <<"\nproxy: " <<p;
    }
  os <<std::endl;
}
