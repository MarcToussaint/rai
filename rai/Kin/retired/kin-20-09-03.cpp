/// extension: containing deprecated functionalities
struct Configuration_ext : Configuration {
  arr qdot;

  void calc_fwdPropagateFrames();    ///< elementary forward kinematics; also computes all Shape frames
  void calc_Q_from_Frames();    ///< fill in the joint transformations assuming that frame poses are known (makes sense when reading files)

  void getJointState(arr& _q, arr& _qdot=NoArr) const;
  void setJointState(const arr& _q, const arr& _qdot=NoArr);

  /// @name Jacobians and kinematics (low level)
  void kinematicsPenetrations(arr& y, arr& J=NoArr, bool penetrationsOnly=true, double activeMargin=0.) const; ///< true: if proxy(i).distance>0. => y(i)=0; else y(i)=-proxy(i).distance
  void kinematicsProxyDist(arr& y, arr& J, const Proxy& p, double margin=.02, bool useCenterDist=true, bool addValues=false) const;
  void kinematicsContactCost(arr& y, arr& J, const ForceExchange* p, double margin=.0, bool addValues=false) const;
  void kinematicsContactCost(arr& y, arr& J, double margin=.0) const;
  void kinematicsProxyConstraint(arr& g, arr& J, const Proxy& p, double margin=.02) const;
  void kinematicsContactConstraints(arr& y, arr& J) const; //TODO: deprecated?
  void getLimitsMeasure(arr& x, const arr& limits, double margin=.1) const;

  /// @name active set selection
  void setAgent(uint) { NIY }

  /// @name High level (inverse) kinematics
  void inverseKinematicsPos(Frame& frame, const arr& ytarget, const rai::Vector& rel_offset=NoVector, int max_iter=3);

  /// @name dynamics
  void inertia(arr& M);

  /// @name forces and gravity
  void contactsToForces(double hook=.01, double damp=.0003);
  void gravityToForces(double g=-9.81);
  void frictionToForces(double coeff);
  void NewtonEuler_backward();

  /// @name collisions & proxies
  void filterProxiesToContacts(double margin=.01); ///< proxies are returns from a collision engine; contacts stable constraints
  void proxiesToContacts(double margin=.01); ///< proxies are returns from a collision engine; contacts stable constraints
};

/** @brief KINEMATICS: given the (absolute) frames of root nodes and the relative frames
    on the edges, this calculates the absolute frames of all other nodes (propagating forward
    through trees and testing consistency of loops). */
void rai::Configuration_ext::calc_fwdPropagateFrames() {
  HALT("don't use this anymore");
//  if(fwdActiveSet.N!=frames.N) calc_activeSets();
//  for(Frame *f:fwdActiveSet) if(f->parent) f->calc_X_from_parent();
}

/** @brief given the absolute frames of all nodes and the two rigid (relative)
    frames A & B of each edge, this calculates the dynamic (relative) joint
    frame X for each edge (which includes joint transformation and errors) */
void rai::Configuration_ext::calc_Q_from_Frames() {
  HALT("should never be called (anymore)");
  for(Frame* f:frames) if(f->parent) {
      f->calc_Q_from_parent();
    }
}

void rai::Configuration_ext::getJointState(arr& _q, arr& _qdot) const {
  if(!q.nd)((Configuration*)this)->ensure_q();
  _q=q;
  if(!!_qdot) {
    _qdot=qdot;
    if(!_qdot.N) _qdot.resizeAs(q).setZero();
  }
}

/** @brief sets the joint state vectors separated in positions and
  velocities */
void rai::Configuration_ext::setJointState(const arr& _q, const arr& _qdot) {
  Configuration::setJointState(q);
  if(!!_qdot) {
    CHECK_EQ(_qdot.N, q.N, "wrong joint velocity dimensionalities");
    qdot=_qdot;
  } else {
    qdot.clear();
  }
}

/** @brief return the configuration's inertia tensor $M$ (n x n tensor)*/
void rai::Configuration_ext::inertia(arr& M) {
  uint j1_idx, j2_idx;
  rai::Transformation Xa, Xi, Xj;
  Joint* j1, *j2;
  rai::Vector vi, vj, ti, tj;
  double tmp;

  uint N=getJointStateDimension();

  //initialize Jacobian
  M.resize(N, N);
  M.setZero();

  for(Frame* a: frames) {
    //get reference frame
    Xa = a->ensure_X();

    j1=a->joint;
    while(j1) {
      j1_idx=j1->qIndex;

      Xi = j1->from()->ensure_X();
      //      Xi.appendTransformation(j1->A);
      ti = Xi.rot.getX();

      vi = ti ^(Xa.pos-Xi.pos);

      j2=j1;
      while(j2) {
        j2_idx=j2->qIndex;

        Xj = j2->from()->ensure_X();
        //        Xj.appendTransformation(j2->A);
        tj = Xj.rot.getX();

        vj = tj ^(Xa.pos-Xj.pos);

        tmp = a->inertia->mass * (vi*vj);
        //tmp += scalarProduct(a->a.inertia, ti, tj);

        M(j1_idx, j2_idx) += tmp;

        j2=j2->from()->joint;
        if(!j2) break;
      }
      j1=j1->from()->joint;
      if(!j1) break;
    }
  }
  //symmetric: fill in other half
  for(j1_idx=0; j1_idx<N; j1_idx++) for(j2_idx=0; j2_idx<j1_idx; j2_idx++) M(j2_idx, j1_idx) = M(j1_idx, j2_idx);
}

void rai::Configuration_ext::proxiesToContacts(double margin) {
  for(Frame* f:frames) while(f->forces.N) delete f->forces.last();

  for(Proxy& p:proxies) {
    if(!p.collision) p.calc_coll(*this);
    ForceExchange* candidate=nullptr;
    for(ForceExchange* c:p.a->forces) {
      if((&c->a==p.a && &c->b==p.b) || (&c->a==p.b && &c->b==p.a)) {
        candidate = c;
        break;
      }
    }
    if(candidate) __merge(candidate, &p);
    else {
      if(p.collision->distance-(p.collision->rad1+p.collision->rad2)<margin) {
        rai::ForceExchange* c = new rai::ForceExchange(*p.a, *p.b);
        __merge(c, &p);
      }
    }
  }
  //phase 2: cleanup old and distant contacts
  NIY;
  //  rai::Array<ForceExchange*> old;
  //  for(Frame *f:frames) for(ForceExchange *c:f->forces) if(&c->a==f) {
  //    if(c->getDistance()>2.*margin) {
  //      old.append(c);
  //    }
  //  }
  //  for(ForceExchange *c:old) delete c;
}

/** similar to invDynamics using NewtonEuler; but only computing the backward pass */
void rai::Configuration_ext::NewtonEuler_backward() {
  CHECK(check_topSort(), "this needs a topology sorted configuration");
  uint N=frames.N;
  rai::Array<arr> h(N);
  arr Q(N, 6, 6);
  arr force(frames.N, 6);
  force.setZero();

  for(uint i=0; i<N; i++) {
    h(i).resize(6).setZero();
    Frame* f = frames.elem(i);
    if(f->joint) {
      h(i) = f->joint->get_h();
    }
    if(f->parent) {
      Q[i] = f->get_Q().getWrenchTransform();
    } else {
      Q[i].setId();
    }
    if(f->inertia) {
      force[f->ID] = f->inertia->getFrameRelativeWrench();
    }
  }

  for(uint i=N; i--;) {
    Frame* f = frames.elem(i);
    if(f->parent) force[f->parent->ID] += ~Q[i] * force[f->ID];
  }

  for(Frame* f:frames) {
    rai::Transformation R = f->ensure_X(); //rotate to world, but no translate to origin
    R.pos.setZero();
    force[f->ID] = ~R.getWrenchTransform() * force[f->ID];
    cout <<f->name <<":\t " <<force[f->ID] <<endl;
  }
}

/// compute forces from the current contacts
void rai::Configuration_ext::contactsToForces(double hook, double damp) {
  rai::Vector trans, transvel, force;
  for(const Proxy& p:proxies) if(p.d<0.) {
      //if(!i || proxies(i-1).a!=a || proxies(i-1).b!=b) continue; //no old reference sticking-frame
      //trans = p.rel.p - proxies(i-1).rel.p; //translation relative to sticking-frame
      trans    = p.posB-p.posA;
      //transvel = p.velB-p.velA;
      //d=trans.length();

      force.setZero();
      force += (hook) * trans; //*(1.+ hook*hook*d*d)
      //force += damp * transvel;
//      SL_DEBUG(1, cout <<"applying force: [" <<*p.a <<':' <<*p.b <<"] " <<force <<endl);

      NIY;
//    addForce(force, p.a, p.posA);
//    addForce(-force, p.b, p.posB);
    }
}

void rai::Configuration_ext::kinematicsPenetrations(arr& y, arr& J, bool penetrationsOnly, double activeMargin) const {
  CHECK(_state_proxies_isGood, "");

  y.resize(proxies.N).setZero();
  if(!!J) J.resize(y.N, getJointStateDimension()).setZero();
  uint i=0;
  for(const Proxy& p:proxies) {
    if(!p.collision)((Proxy*)&p)->calc_coll(*this);

    arr Jp1, Jp2;
    if(!!J) {
      jacobian_pos(Jp1, p.a, p.collision->p1);
      jacobian_pos(Jp2, p.b, p.collision->p2);
    }

    arr y_dist, J_dist;
    p.collision->kinDistance(y_dist, J_dist, Jp1, Jp2);

    if(!penetrationsOnly || y_dist.scalar()<activeMargin) {
      y(i) = -y_dist.scalar();
      if(!!J) J[i] = -J_dist;
    }
  }
}

void rai::Configuration_ext::kinematicsProxyDist(arr& y, arr& J, const Proxy& p, double margin, bool useCenterDist, bool addValues) const {
  y.resize(1);
  if(!!J) J.resize(1, getJointStateDimension());
  if(!addValues) { y.setZero();  if(!!J) J.setZero(); }

  y(0) = p.d;
  if(!!J) {
    arr Jpos;
    rai::Vector arel, brel;
    if(p.d>0.) { //we have a gradient on pos only when outside
      arel=p.a->X.rot/(p.posA-p.a->X.pos);
      brel=p.b->X.rot/(p.posB-p.b->X.pos);
      CHECK(p.normal.isNormalized(), "proxy normal is not normalized");
      arr normal; normal.referTo(&p.normal.x, 3); normal.reshape(1, 3);
      kinematicsPos(NoArr, Jpos, p.a, arel);  J += (normal*Jpos);
      kinematicsPos(NoArr, Jpos, p.b, brel);  J -= (normal*Jpos);
    }
  }
}

void rai::Configuration_ext::kinematicsContactCost(arr& y, arr& J, const ForceExchange* c, double margin, bool addValues) const {
  NIY;
  //  Feature *map = c->getTM_ContactNegDistance();
  //  arr y_dist, J_dist;
  //  map->phi(y_dist, J_dist, *this);
  //  y_dist *= -1.;
  //  if(!!J) J_dist *= -1.;

  //  y.resize(1);
  //  if(!!J) J.resize(1, getJointStateDimension());
  //  if(!addValues) { y.setZero();  if(!!J) J.setZero(); }

  //  if(y_dist.scalar()>margin) return;
  //  y += margin-y_dist.scalar();
  //  if(!!J)  J -= J_dist;
}

void rai::Configuration_ext::kinematicsContactCost(arr& y, arr& J, double margin) const {
  y.resize(1).setZero();
  jacobian_zero(J, 1);
  for(Frame* f:frames) for(ForceExchange* c:f->forces) if(&c->a==f) {
        kinematicsContactCost(y, J, c, margin, true);
      }
}

void rai::Configuration_ext::kinematicsProxyConstraint(arr& g, arr& J, const Proxy& p, double margin) const {
  if(!!J) J.resize(1, getJointStateDimension()).setZero();

  g.resize(1) = margin - p.d;

  //Jacobian
  if(!!J) {
    arr Jpos, normal;
    rai::Vector arel, brel;
    if(p.d>0.) { //we have a gradient on pos only when outside
      arel=p.a->X.rot/(p.posA-p.a->X.pos);
      brel=p.b->X.rot/(p.posB-p.b->X.pos);
      CHECK(p.normal.isNormalized(), "proxy normal is not normalized");
      normal.referTo(&p.normal.x, 3);
    } else { //otherwise take gradient w.r.t. centers...
      arel.setZero(); //a->X.rot/(p.cenA-a->X.pos);
      brel.setZero(); //b->X.rot/(p.cenB-b->X.pos);
      CHECK(p.normal.isNormalized(), "proxy normal is not normalized");
      normal.referTo(&p.normal.x, 3);
    }
    normal.reshape(1, 3);

    kinematicsPos(NoArr, Jpos, p.a, arel);  J -= (normal*Jpos);
    kinematicsPos(NoArr, Jpos, p.b, brel);  J += (normal*Jpos);
  }
}

void rai::Configuration_ext::kinematicsContactConstraints(arr& y, arr& J) const {
  J.clear();
  rai::Vector normal;
  uint con=0;
  arr Jpos, dnormal, grad(1, q.N);

  y.clear();
  for(const rai::Proxy& p: proxies) y.append(p.d);

  if(!J) return; //do not return the Jacobian

  rai::Vector arel, brel;
  for(const rai::Proxy& p: proxies) {
    arel.setZero();  arel=p.a->X.rot/(p.posA-p.a->X.pos);
    brel.setZero();  brel=p.b->X.rot/(p.posB-p.b->X.pos);

    CHECK(p.normal.isNormalized(), "proxy normal is not normalized");
    dnormal = p.normal.getArr(); dnormal.reshape(1, 3);
    grad.setZero();
    kinematicsPos(NoArr, Jpos, p.a, arel); grad += dnormal*Jpos; //moving a long normal b->a increases distance
    kinematicsPos(NoArr, Jpos, p.b, brel); grad -= dnormal*Jpos; //moving b long normal b->a decreases distance
    J.append(grad);
    con++;
  }
  J.reshape(con, q.N);
}

/// The position vec1, attached to b1, relative to the frame of b2 (plus vec2)
void rai::Configuration::kinematicsRelPos(arr& y, arr& J, Frame* a, const rai::Vector& vec1, Frame* b, const rai::Vector& vec2) const {
  arr y1, y2, J1, J2;
  kinematicsPos(y1, J1, a, vec1);
  kinematicsPos(y2, J2, b, vec2);
  arr Rinv = ~(b->ensure_X().rot.getArr());
  y = Rinv * (y1 - y2);
  if(!!J) {
    arr A;
    jacobian_angular(A, b);
    J = Rinv * (J1 - J2 - crossProduct(A, y1 - y2));
  }
}

