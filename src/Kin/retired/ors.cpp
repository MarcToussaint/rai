/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

/* dm 15.06.2006--these had to be changed with the switch to ODE 0.6
   void Body::copyFrameToOde(){
   CHECK(X.r.isNormalized(), "quaternion is not normalized!");
   CP3(b->posr.pos, X.p);                // dxBody changed in ode-0.6 ! 14. Jun 06 (hh)
   CP4(b->q, X.r); dQtoR(b->q, b->posr.R);
   CP3(b->lvel, X.v);
   CP3(b->avel, X.w);
   }
   void Body::getFrameFromOde(){
   CP3(X.p.v, b->posr.pos);
   CP4(X.r.q, b->q);
   CP3(X.v.v, b->lvel);
   CP3(X.w.v, b->avel);
   CHECK(X.r.isNormalized(), "quaternion is not normalized!");
   }
*/

//void rai::Configuration::addObject(rai::Body *b) {
//  bodies.append(b);
//  int ibody = bodies.N - 1;
//  uint i; rai::Shape *s;
//  for_list(Type,  s,  b->shapes) {
//    s->ibody = ibody;
//    s->index = shapes.N;
//    shapes.append(s);
//  }
//}

rai::Configuration* rai::Configuration::newClone() const {
  Graph* G=new Graph();
  G->q_dim=q_dim;
  listCopy(G->proxies, proxies);
  listCopy(G->shapes, shapes);
  listCopy(G->bodies, bodies);
  listCopy(G->joints, joints);
  // post-process coupled joints
  for(Joint* j: G->joints)
    if(j->mimic) {
      rai::String jointName;
      bool good = j->ats.find<rai::String>(jointName, "mimic");
      CHECK(good, "something is wrong");
      j->mimic = listFindByName(G->joints, jointName);
      if(!j->mimic) HALT("The joint '" <<*j <<"' is declared coupled to '" <<jointName <<"' -- but that doesn't exist!");
      j->type = j->mimic->type;
    }
  graphMakeLists(G->bodies, G->joints);
  uint i;  Shape* s;  Body* b;
  for_list(Type,  s,  G->shapes) {
    b=G->bodies(s->ibody);
    s->body=b;
    b->shapes.append(s);
  }
  return G;
}

//void rai::Configuration::copyShapesAndJoints(const Graph& G) {
//  uint i;  Shape *s;  Body *b;  Joint *j;
//  for_list(Type,  s,  shapes)(*s) = *G.shapes(i);
//  for_list(Type,  j,  joints)(*j) = *G.joints(i);
//  for_list(Type,  b,  bodies) b->shapes.clear();
//  for_list(Type,  s,  shapes) {
//    b=bodies(s->ibody);
//    s->body=b;
//    b->shapes.append(s);
//  }
//  calcBodyFramesFromJoints();
//}

///// find body index with specific name
//uint rai::Configuration::getBodyIndexByName(const char* name) const {
//  Body *b=getBodyByName(name);
//  return b?b->index:0;
//}

///// find shape index with specific name
//uint rai::Configuration::getShapeIndexByName(const char* name) const {
//  Shape *s=getShapeByName(name);
//  return s?s->index:0;
//}

/** @brief if two bodies touch, the are not yet connected, and one of them has
  the `glue' attribute, add a new edge of FIXED type between them */
//void rai::Configuration::glueTouchingBodies() {
//  uint i, A, B;
//  Body *a, *b;//, c;
//  bool ag, bg;
//  for(i=0; i<proxies.N; i++) {
//    A=proxies(i)->a; a=(A==(uint)-1?nullptr:bodies(A));
//    B=proxies(i)->b; b=(B==(uint)-1?nullptr:bodies(B));
//    if(!a || !b) continue;
//    ag=a->ats.find<bool>("glue");
//    bg=b->ats.find<bool>("glue");
//    if(ag || bg) {
//      //if(a->index > b->index){ c=a; a=b; b=c; } //order them topolgically
//      if(graphGetEdge<Body, Joint>(a, b)) continue;  //they are already connected
//      glueBodies(a, b);
//      //a->cont=b->cont=false;
//    }
//  }
//}

#if 0 //obsolete:
void rai::Configuration::getContactMeasure(arr& x, double margin, bool linear) const {
  x.resize(1);
  x=0.;
  uint i;
  Shape* a, *b;
  double d, discount;
  for(i=0; i<proxies.N; i++) if(!proxies(i)->age && proxies(i)->d<margin) {
      a=shapes(proxies(i)->a); b=shapes(proxies(i)->b);
      d=1.-proxies(i)->d/margin;
      //NORMALS ALWAYS GO FROM b TO a !!
      discount = 1.;
      if(!a->contactOrientation.isZero()) {  //object has an 'allowed contact orientation'
        CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
        CHECK(a->contactOrientation.isNormalized(), "contact orientation is not normalized");
        //double theta = ::acos( proxies(i)->normal*a->contactOrientation);
        double theta = .5*(proxies(i)->normal*a->contactOrientation-1.);
        discount *= theta*theta;
      }
      if(!b->contactOrientation.isZero()) {
        CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
        CHECK(a->contactOrientation.isNormalized(), "contact orientation is not normalized");
        //double theta = ::acos(-proxies(i)->normal*b->contactOrientation);
        double theta = .5*(-proxies(i)->normal*a->contactOrientation-1.);
        discount *= theta*theta;
      }
      if(!linear) x(0) += discount*d*d;
      else        x(0) += discount*d;
    }
}

/// gradient (=scalar Jacobian) of this contact cost
double rai::Configuration::getContactGradient(arr& grad, double margin, bool linear) const {
  rai::Vector normal;
  uint i;
  Shape* a, *b;
  double d, discount;
  double cost=0.;
  arr J, dnormal;
  grad.resize(1, getJointStateDimension(false));
  grad.setZero();
  rai::Vector arel, brel;
  for(i=0; i<proxies.N; i++) if(proxies(i)->d<margin) {
      a=shapes(proxies(i)->a); b=shapes(proxies(i)->b);
      d=1.-proxies(i)->d/margin;
      discount = 1.;
      if(!a->contactOrientation.isZero()) {  //object has an 'allowed contact orientation'
        CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
        CHECK(a->contactOrientation.isNormalized(), "contact orientation is not normalized");
        //double theta = ::acos( proxies(i)->normal*a->contactOrientation);
        double theta = .5*(proxies(i)->normal*a->contactOrientation-1.);
        discount *= theta*theta;
      }
      if(!b->contactOrientation.isZero()) {
        CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
        CHECK(a->contactOrientation.isNormalized(), "contact orientation is not normalized");
        //double theta = ::acos(-proxies(i)->normal*b->contactOrientation);
        double theta = .5*(-proxies(i)->normal*a->contactOrientation-1.);
        discount *= theta*theta;
      }
      if(!linear) cost += discount*d*d;
      else        cost += discount*d;

      arel.setZero();  arel=a->X.rot/(proxies(i)->posA-a->X.pos);
      brel.setZero();  brel=b->X.rot/(proxies(i)->posB-b->X.pos);

      CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
      dnormal.referTo(proxies(i)->normal.p(), 3); dnormal.reshape(1, 3);
      if(!linear) {
        jacobianPos(J, a->body->index, &arel); grad -= ((double)2.*discount*d)/margin*(dnormal*J);
        jacobianPos(J, b->body->index, &brel); grad += ((double)2.*discount*d)/margin*(dnormal*J);
      } else {
        jacobianPos(J, a->body->index, &arel); grad -= discount/margin*(dnormal*J);
        jacobianPos(J, b->body->index, &brel); grad += discount/margin*(dnormal*J);
      }
    }

  return cost;
}
#endif

#if 0 //alternative implementation : cost=1 -> contact, other discounting...
double rai::Configuration::getContactGradient(arr& grad, double margin) {
  rai::Vector normal;
  uint i;
  Shape* a, *b;
  double d, discount;
  double cost=0.;
  arr J, dnormal;
  grad.resize(1, jd);
  grad.setZero();
  rai::Transformation arel, brel;
  for(i=0; i<proxies.N; i++) if(!proxies(i)->age && proxies(i)->d<margin) {
      a=shapes(proxies(i)->a); b=shapes(proxies(i)->b);
      discount = 1.;
      if(!a->contactOrientation.isZero()) {  //object has an 'allowed contact orientation'
        CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
        CHECK(a->contactOrientation.isNormalized(), "contact orientation is not normalized");
        //double theta = ::acos( proxies(i)->normal*a->contactOrientation);
        double theta = .5*(proxies(i)->normal*a->contactOrientation-1.);
        discount *= theta*theta;
      }
      if(!b->contactOrientation.isZero()) {
        CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
        CHECK(a->contactOrientation.isNormalized(), "contact orientation is not normalized");
        //double theta = ::acos(-proxies(i)->normal*b->contactOrientation);
        double theta = .5*(-proxies(i)->normal*a->contactOrientation-1.);
        discount *= theta*theta;
      }
      double marg=(discount+.1)*margin;
      d=1.-proxies(i)->d/marg;
      if(d<0.) continue;
      cost += d*d;

      arel.setZero();  arel.p=a->X.r/(proxies(i)->posA-a->X.p);
      brel.setZero();  brel.p=b->X.r/(proxies(i)->posB-b->X.p);

      CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
      dnormal.referTo(proxies(i)->normal.v, 3); dnormal.reshape(1, 3);
      jacobianPos(J, a->body->index, &arel); grad -= (2.*d/marg)*(dnormal*J);
      jacobianPos(J, b->body->index, &brel); grad += (2.*d/marg)*(dnormal*J);
    }

  return cost;
}
#endif

/// [prelim] some kind of gyroscope
void rai::Configuration::getGyroscope(rai::Vector& up) const {
  up.set(0, 0, 1);
  up=bodies(0)->X.rot*up;
}

/** @brief returns a k-dim vector containing the penetration depths of all bodies */
void rai::Configuration::getPenetrationState(arr& vec) const {
  vec.resize(bodies.N);
  vec.setZero();
  rai::Vector d;
  uint i;
  for(i=0; i<proxies.N; i++) if(proxies(i)->d<0.) {
      d=proxies(i)->posB - proxies(i)->posA;

      if(proxies(i)->a!=-1) vec(proxies(i)->a) += d.length();
      if(proxies(i)->b!=-1) vec(proxies(i)->b) += d.length();
    }
}

/** @brief a vector describing the incoming forces (penetrations) on one object */
void rai::Configuration::getGripState(arr& grip, uint j) const {
  rai::Vector d, p;
  rai::Vector sumOfD; sumOfD.setZero();
  rai::Vector torque; torque.setZero();
  double sumOfAbsD = 0.;
  double varOfD = 0.;

  p.setZero();
  uint i, n=0;
  for(i=0; i<proxies.N; i++) if(proxies(i)->d<0.) {
      if(proxies(i)->a!=(int)j && proxies(i)->b!=(int)j) continue;

      n++;

      if(proxies(i)->a==(int)j) {
        d=proxies(i)->posB - proxies(i)->posA;
        p=proxies(i)->posA;
      }
      if(proxies(i)->b==(int)j) {
        d=proxies(i)->posA - proxies(i)->posB;
        p=proxies(i)->posB;
      }

      sumOfAbsD += d.length();
      sumOfD    += d;
      varOfD    += d.lengthSqr();
      torque    += (p - bodies(j)->X.pos) ^ d;

    }
  if(n) { varOfD = (varOfD - sumOfD*sumOfD) / n; }

  grip.resize(8);
  grip(0)=sumOfAbsD;
  grip(1)=varOfD;
  grip(2)=sumOfD.x;
  grip(3)=sumOfD.y;
  grip(4)=sumOfD.z;
  grip(5)=torque.x;
  grip(6)=torque.y;
  grip(7)=torque.z;
}

#if 0 //OBSOLETE
/// returns the number of touch-sensors
uint rai::Configuration::getTouchDimension() {
  Body* n;
  uint i=0, j;

  // count touchsensors
  for_list(Type,  n,  bodies) if(ats.find<double>(n->ats, "touchsensor", 0)) i++;
  td=i;
  return i;
}

/// returns the touch vector (penetrations) of all touch-sensors
void rai::Configuration::getTouchState(arr& touch) {
  if(!td) td=getTouchDimension();
  arr pen;
  getPenetrationState(pen);
  Body* n;
  uint i=0, j;
  for_list(Type,  n,  bodies) {
    if(ats.find<double>(n->ats, "touchsensor", 0)) {
      touch(i)=pen(n->index);
      i++;
    }
  }
  CHECK_EQ(i, td, "");
}
#endif

/** @brief get the center of mass, total velocity, and total angular momemtum */
void rai::Configuration::getTotals(rai::Vector& c, rai::Vector& v, rai::Vector& l, rai::Quaternion& ori) const {
  Body* n;
  uint j;
  double m, M;

  //dMass mass;
  rai::Matrix ID;
  //rai::Matrix TP;
  rai::Vector r, o;

  ID.setId();
  c.setZero();
  v.setZero();
  l.setZero();
  o.setZero();
  //Iall.setZero();
  M=0.;
  for_list(Type,  n,  bodies) {
    l+=n->inertia*n->X.angvel;
    //TP.setTensorProduct(n->X.p, n->X.p);
    //Iall+=m*((n->X.p*n->X.p)*ID + TP);

    m=n->mass;
    l+=m*(n->X.pos ^ n->X.vel);
    o+=m*n->X.rot.getVec(r);

    M+=m;
    c+=m*n->X.pos;
    v+=m*n->X.vel;
  }
  c/=M;
  v/=M;
  o/=M;
  ori.setVec(o);
}

/** @brief dump a list body pairs for which the upper conditions hold */
void rai::Configuration::reportGlue(std::ostream* os) {
  uint i, A, B;
  Body* a, *b;
  bool ag, bg;
  (*os) <<"Glue report: " <<endl;
  for(i=0; i<proxies.N; i++) {
    A=proxies(i)->a; a=(A==(uint)-1?nullptr:bodies(A));
    B=proxies(i)->b; b=(B==(uint)-1?nullptr:bodies(B));
    if(!a || !b) continue;
    ag=a->ats.find<bool>("glue");
    bg=b->ats.find<bool>("glue");
    if(ag || bg) {
      (*os)
          <<i <<' '
          <<a->index <<',' <<a->name <<'-'
          <<b->index <<',' <<b->name
          <<" d=" <<proxies(i)->d
          // <<" posA=" <<proxies(i)->posA
          // <<" posB=" <<proxies(i)->posB
          <<" norm=" <<proxies(i)->posB-proxies(i)->posA
          <<endl;
    }
  }
}
