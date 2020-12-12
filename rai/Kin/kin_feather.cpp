/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kin_feather.h"
#include "frame.h"
#include "kin.h"

/** interface and implementation to Featherstone's Articulated Body Algorithm

  See resources from http://users.rsise.anu.edu.au/~roy/spatial/index.html

  This is a direct port of the following two files
  http://users.rsise.anu.edu.au/~roy/spatial/ws04spatial.txt
  http://users.rsise.anu.edu.au/~roy/spatial/ws04abadyn.txt

  See also his slides on the spatial vector algebra
  http://users.rsise.anu.edu.au/~roy/spatial/slidesX4.pdf

  main changes for porting to C:

  indexing of arrays start from 0

  referencing sub arrays with [i] rather than {i}

  NOTE: Featherstone's rotation matricies have opposite convention than mine
*/
namespace Featherstone {
/// returns a cross-product matrix X such that \f$v \times y = X y\f$
void skew(arr& X, const double* v);

/// as above
arr skew(const double* v);

/** @brief MM6 coordinate transform from X-axis rotation.  Xrotx(h)
  calculates the MM6 coordinate transform matrix (for motion
  vectors) induced by a rotation about the +X axis by an angle h (in
  radians).  Positive rotation is anticlockwise: +Y axis rotates
  towards +Z axis.
*/
void Xrotx(arr& X, double h);

/** @brief MM6 coordinate transform from Y-axis rotation.  Xroty(h)
  calculates the MM6 coordinate transform matrix (for motion
  vectors) induced by a rotation about the +Y axis by an angle h (in
  radians).  Positive rotation is anticlockwise: +Z axis rotates
  towards +X axis.
*/
void Xroty(arr& X, double h);

/** @brief MM6 coordinate transform from Z-axis rotation.  Xrotz(h)
  calculates the MM6 coordinate transform matrix (for motion
  vectors) induced by a rotation about the +Z axis by an angle h (in
  radians).  Positive rotation is anticlockwise: +X axis rotates
  towards +Y axis.
*/
void Xrotz(arr& X, double h);

/** @brief MM6 coordinate transform from 3D translation vector.
  Xtrans(r) calculates the MM6 coordinate transform matrix (for
  motion vectors) induced by a shift of origin specified by the 3D
  vector r, which contains the x, y and z coordinates of the new
  location of the origin relative to the old.
*/
void Xtrans(arr& X, double* r);

/** @brief Calculate RBI from mass, CoM and rotational inertia.
  RBmci(m, c, I) calculate MF6 rigid-body inertia tensor for a body
  with mass m, centre of mass at c, and (3x3) rotational inertia
  about CoM of I.
*/
void RBmci(arr& rbi, double m, double* c, const rai::Matrix& I);

/** @brief MM6 cross-product tensor from M6 vector.  crossM(v)
  calculates the MM6 cross-product tensor of motion vector v such
  that crossM(v) * m = v X m (cross-product of v and m) where m is
  any motion vector or any matrix or tensor mapping to M6.
*/
void crossM(arr& vcross, const arr& v);

/// as above
arr crossM(const arr& v);

/** @brief FF6 cross-product tensor from M6 vector.  crossF(v)
  calculates the FF6 cross-product tensor of motion vector v such
  that crossF(v) * f = v X f (cross-product of v and f) where f is
  any force vector or any matrix or tensor mapping to F6.
*/
void crossF(arr& vcross, const arr& v);

/// as above
arr crossF(const arr& v);
}
//#define Qstate

#if 0
rai::Body* robotbody(uint i, const Featherstone::Robot& robot) { return robot.C->nodes(i); }

uint Featherstone::Robot::N() const { return C->nodes.N; }

int Featherstone::Robot::parent(uint i) const {
  rai::Joint* e=C->nodes(i)->joint();
  if(e) return e->from->index;
  return -1;
}

byte Featherstone::Robot::dof(uint i) const {
  rai::Body* n=C->nodes(i);
  if(n->fixed) return 0;
  if(n->hasJoint()) {
    switch(n->joint()->type) {
      case 0: return 1;
#ifndef Qstate
      case 4: return 3;
#else
      case 4: return 4;
#endif
    }
  }
  return 6;
}

const arr Featherstone::Robot::S(uint i) const {
  byte d_i=dof(i);
  arr S;
  rai::Quaternion r;
  rai::Matrix R;
  arr Ss, rr;
  switch(d_i) {
    case 0: S.resize(6, (uint)0); S.setZero(); break;
    case 1: S.resize(6, 1); S.setZero(); S(0, 0)=1.; break;
    case 3:
      S.resize(6, 3); S.setZero();
      S(0, 0)=1.; S(1, 1)=1.; S(2, 2)=1.;
      break;
      r = C->nodes(i)->joint()->X.r;
      r.invert();
      r.getMatrix(R.m);
      memmove(S.p, R.m, 9*sizeof(double));
      break;
    case 4:
      S.resize(6, 4); S.setZero();
      r = C->nodes(i)->joint()->X.r;
      r.invert();
      r.getMatrix(R.m);
      S(0, 0)= 0;  S(0, 1)= R(0, 0);  S(0, 2)= R(0, 1);  S(0, 3)= R(0, 2);
      S(1, 0)= 0;  S(1, 1)= R(1, 0);  S(1, 2)= R(1, 1);  S(1, 3)= R(1, 2);
      S(2, 0)= 0;  S(2, 1)= R(2, 0);  S(2, 2)= R(2, 1);  S(2, 3)= R(2, 2);
      S *= 2.;
      break;
    case 6:
      S.resize(6, 6); S.setZero();
      //S(0, 3)=1.; S(1, 4)=1.; S(2, 5)=1.;
      S(3, 0)=1.; S(4, 1)=1.; S(5, 2)=1.;
      break; //S(1, 1)=S(2, 2)=1.; break;
    //case 6: S.setId(6); break;
    default: NIY;
  }
  return S;
}

/* returns the transformation from the parent link to the i-th link */
const arr Featherstone::Robot::Xlink(uint i) const {
  //slide 15
  rai::Transformation f;
  rai::Joint* e1=C->nodes(i)->firstIn;
  if(!e1) {
    f.setZero();
  } else {
    rai::Joint* e0=e1->from->firstIn;
    if(e0) {
      f = e0->B;
      f.addRelativeFrame(e1->A);
      //f.addRelativeFrame(e1->X);
    } else {
      //NIY;
      f = e1->from->X;
      f.addRelativeFrame(e1->A);
      //f.addRelativeFrame(e1->X);
    }
  }
  arr X;
  FrameToMatrix(X, f);
  return X;
}

const arr Featherstone::Robot::Ilink(uint i) const {
  //taken from slide 27
  rai::Joint* e=C->nodes(i)->firstIn;
  double m=C->nodes(i)->mass;
  rai::Vector com;
  if(e) com = e->B.p; else com = C->nodes(i)->X.p;
  //arr Ic(3, 3);  Ic.setDiag(.1*m);
  arr I;
  RBmci(I, m, com.v, C->nodes(i)->inertia);
  return I;
}

const arr Featherstone::Robot::force(uint i) const {
  rai::Body* n=C->nodes(i);
  CHECK(n, "is not a body with input joint");
  rai::Joint* e=n->firstIn;
  //CHECK(e, "is not a body with input joint");
  rai::Transformation g;
  g=n->X;
  if(e) g.subRelativeFrame(e->B);
  rai::Vector fo = g.r/n->force;
  rai::Vector to;
  if(e) to = g.r/(n->torque + (g.r*e->B.p)^n->force);
  else  to = g.r/(n->torque);
  arr f(6);
  f(0)=to.x;  f(1)=to.y;  f(2)=to.z;
  f(3)=fo.x;  f(4)=fo.y;  f(5)=fo.z;
  return f;
}
#endif

void Featherstone::skew(arr& X, const double* v) {
  X.resize(3, 3);  X.setZero();
  X(0, 1) = -v[2];  X(1, 0) = v[2];
  X(1, 2) = -v[0];  X(2, 1) = v[0];
  X(2, 0) = -v[1];  X(0, 2) = v[1];
}

arr Featherstone::skew(const double* v) { arr X; skew(X, v); return X; }

void FrameToMatrix(arr& X, const rai::Transformation& f) {
  arr z(3, 3);  z.setZero();
  arr r(3, 3);  Featherstone::skew(r, &f.pos.x);
  arr R(3, 3);  f.rot.getMatrix(R.p);
  transpose(R);
  X.resize(6, 6);  X.setBlockMatrix(R, z, R*~r, R); //[[unklar!!]]
  //cout <<"\nz=" <<z <<"\nr=" <<r <<"\nR=" <<R <<"\nX=" <<X <<endl;
}

uint F_Link::dof() { if(type>=rai::JT_hingeX && type<=rai::JT_transZ) return 1; else return 0; }

void F_Link::setFeatherstones() {
  switch(type) {
    case -1:     CHECK_EQ(parent, -1, ""); _h.clear();  break;
    case rai::JT_rigid:
    case rai::JT_transXYPhi:
      qIndex=-1;
      _h=zeros(6);
      break;
    case rai::JT_hingeX: _h.resize(6).setZero(); _h(0)=1.; break;
    case rai::JT_hingeY: _h.resize(6).setZero(); _h(1)=1.; break;
    case rai::JT_hingeZ: _h.resize(6).setZero(); _h(2)=1.; break;
    case rai::JT_transX: _h.resize(6).setZero(); _h(3)=1.; break;
    case rai::JT_transY: _h.resize(6).setZero(); _h(4)=1.; break;
    case rai::JT_transZ: _h.resize(6).setZero(); _h(5)=1.; break;
    default: NIY;
  }
  Featherstone::RBmci(_I, mass, com.p(), inertia);

  updateFeatherstones();
}

void F_Link::updateFeatherstones() {
  FrameToMatrix(_Q, Q);

//  rai::Transformation XQ;
//  XQ=X;
//  XQ.appendTransformation(Q);
  rai::Vector fo = X.rot/force;
  rai::Vector to = X.rot/(torque + ((X.rot*com)^force));
  _f.resize(6);
  _f(0)=to.x;  _f(1)=to.y;  _f(2)=to.z;
  _f(3)=fo.x;  _f(4)=fo.y;  _f(5)=fo.z;
}

void FeatherstoneInterface::setGravity(double g) {
  rai::Vector grav(0, 0, g);
  for(rai::Frame* f: C.frames) {
    F_Link& link=tree(f->ID);
    link.force = link.mass * grav;
  }
}

void FeatherstoneInterface::update() {
  if(tree.N != C.frames.N) { //new instance -> create the tree
    CHECK_EQ(C.frames, sortedFrames, "Featherstone requires a sorted optimized frame tree (call optimizeTree and fwdIndexIDs)");
    tree.clear();
    tree.resize(C.frames.N);

    for(F_Link& link:tree) { link.parent=-1; link.qIndex=-1; link.com.setZero(); } //TODO: remove

    uint n=0;
    for(rai::Frame* f : C.frames) {
      F_Link& link=tree(f->ID);
      link.ID = f->ID;
      link.X = f->ensure_X();
      if(f->parent) { //is not a root
        link.parent = f->parent->ID;
        link.Q = f->get_Q();
        rai::Joint* j=f->joint;
        if(j && !j->mimic) {
          link.type   = j->type;
          link.qIndex = j->qIndex;
        } else {
          if(j && j->mimic) LOG(0) <<"Featherstone cannot handle mimic joint ('" <<f->name <<"') properly - assuming rigid";
          link.type   = rai::JT_rigid;
        }
//        if(j) CHECK_EQ(j->dim, link.dof(), "");
      }
      if(f->inertia) {
        link.com = f->inertia->com;
        link.mass=f->inertia->mass; CHECK(link.mass>0. || link.qIndex==-1, "a moving link without mass -> this will diverge");
        link.inertia=f->inertia->matrix;
      }
      n += link.dof();
    }
//    CHECK_EQ(n, C.getJointStateDimension(), "");
  } else { //just update an existing structure
    for(rai::Frame* f: C.frames) {
      F_Link& link=tree(f->ID);
      link.X = f->ensure_X();
      if(f->parent) link.Q = f->get_Q();
    }
  }

  for(F_Link& link:tree) link.setFeatherstones();
}

/*
----------- Xrotx.m ----------------------------------------------------------
*/
void Featherstone::Xrotx(arr& X, double h) {
  /*
  % Xrotx  MM6 coordinate transform from X-axis rotation.
  % Xrotx(h) calculates the MM6 coordinate transform matrix (for motion
  % vectors) induced by a rotation about the +X axis by an angle h (in radians).
  % Positive rotation is anticlockwise: +Y axis rotates towards +Z axis.
  */
  double c = cos(h), s = sin(h);
  X.resize(6, 6); X.setZero();
  X(0, 0)= X(3, 3)= 1.;
  X(1, 1)= X(2, 2)= X(4, 4)= X(5, 5)= c;
  X(1, 2)= X(4, 5)=  s;
  X(2, 1)= X(5, 4)= -s;
  /* X = [
     1  0  0  0  0  0 ;
     0  c  s  0  0  0 ;
     0 -s  c  0  0  0 ;
     0  0  0  1  0  0 ;
     0  0  0  0  c  s ;
     0  0  0  0 -s  c
      ]; */
}

/*
----------- Xroty.m ----------------------------------------------------------
*/
void Featherstone::Xroty(arr& X, double h) {
  /*
  % Xroty  MM6 coordinate transform from Y-axis rotation.
  % Xroty(h) calculates the MM6 coordinate transform matrix (for motion
  % vectors) induced by a rotation about the +Y axis by an angle h (in radians).
  % Positive rotation is anticlockwise: +Z axis rotates towards +X axis.
  */
  double c = cos(h), s = sin(h);
  X.resize(6, 6);  X.setZero();
  X(1, 1)= X(4, 4)= 1.;
  X(0, 0)= X(2, 2)= X(3, 3)= X(5, 5)= c;
  X(0, 2)= X(3, 5)= -s;
  X(2, 0)= X(5, 3)=  s;
  /* X = [
     c  0 -s  0  0  0 ;
     0  1  0  0  0  0 ;
     s  0  c  0  0  0 ;
     0  0  0  c  0 -s ;
     0  0  0  0  1  0 ;
     0  0  0  s  0  c
     ]; */
}

/*
----------- Xrotz.m ----------------------------------------------------------
*/
void Featherstone::Xrotz(arr& X, double h) {
  /*
  % Xrotz  MM6 coordinate transform from Z-axis rotation.
  % Xrotz(h) calculates the MM6 coordinate transform matrix (for motion
  % vectors) induced by a rotation about the +Z axis by an angle h (in radians).
  % Positive rotation is anticlockwise: +X axis rotates towards +Y axis.
  */
  double c = cos(h), s = sin(h);
  X.resize(6, 6);  X.setZero();
  X(2, 2)= X(5, 5)= 1.;
  X(0, 0)= X(1, 1)= X(3, 3)= X(4, 4)= c;
  X(0, 1)= X(3, 4)=  s;
  X(1, 0)= X(4, 3)= -s;
  /* X = [
     c  s  0  0  0  0 ;
     -s  c  0  0  0  0 ;
     0  0  1  0  0  0 ;
     0  0  0  c  s  0 ;
     0  0  0 -s  c  0 ;
     0  0  0  0  0  1 ]; */
}

/*
----------- Xtrans.m ---------------------------------------------------------
*/
void Featherstone::Xtrans(arr& X, double* r) {
  /*
  % Xtrans  MM6 coordinate transform from 3D translation vector.
  % Xtrans(r) calculates the MM6 coordinate transform matrix (for motion
  % vectors) induced by a shift of origin specified by the 3D vector r, which
  % contains the x, y and z coordinates of the new location of the origin
  % relative to the old.
  */
  X.resize(6, 6);  X.setId();
  X.setMatrixBlock(-skew(r), 3, 0);
  /* X = [
      1     0     0    0  0  0 ;
      0     1     0    0  0  0 ;
      0     0     1    0  0  0 ;
      0     r(3) -r(2) 1  0  0 ;
     -r(3)  0     r(1) 0  1  0 ;
      r(2) -r(1)  0    0  0  1
     ]; */
}

/*
----------- RBmci.m ----------------------------------------------------------
*/
void Featherstone::RBmci(arr& rbi, double m, double* c, const rai::Matrix& I) {
  /*
  % RBmci  Calculate RBI from mass, CoM and rotational inertia.
  % RBmci(m, c, I) calculate MF6 rigid-body inertia tensor for a body with
  % mass m, centre of mass at c, and (3x3) rotational inertia about CoM of I.
  */
  arr C(3, 3);
  skew(C, c);
  //C = [ 0, -c(3), c(2); c(3), 0, -c(1); -c(2), c(1), 0 ];
  arr II;
  II.referTo(&I.m00, 9);
  II.reshape(3, 3);

  rbi.setBlockMatrix(II + m*C*~C, m*C, m*~C, m*eye(3));
  //rbi = [ I + m*C*C', m*C; m*C', m*eye(3) ];
}

//===========================================================================
void Featherstone::crossF(arr& vcross, const arr& v) {
  /*
  % crossF  FF6 cross-product tensor from M6 vector.
  % crossF(v) calculates the FF6 cross-product tensor of motion vector v
  % such that crossF(v) * f = v X f (cross-product of v and f) where f is any
  % force vector or any matrix or tensor mapping to F6.
  */
  crossM(vcross, v);
  transpose(vcross);
  vcross *= (double)-1.;
  //vcross = -crossM(v)';
}

arr Featherstone::crossF(const arr& v) { arr X; crossF(X, v); return X; }

//===========================================================================
void Featherstone::crossM(arr& vcross, const arr& v) {
  /*
  % crossM  MM6 cross-product tensor from M6 vector.
  % crossM(v) calculates the MM6 cross-product tensor of motion vector v
  % such that crossM(v) * m = v X m (cross-product of v and m) where m is any
  % motion vector or any matrix or tensor mapping to M6.
  */
  CHECK(v.nd==1 && v.N==6, "");
  vcross.resize(6, 6);  vcross.setZero();

  arr vc;  skew(vc, v.p);
  vcross.setMatrixBlock(vc, 0, 0);
  vcross.setMatrixBlock(vc, 3, 3);
  vcross.setMatrixBlock(skew(v.p+3), 3, 0);
  /* vcross = [
      0    -v(3)  v(2)   0     0     0    ;
      v(3)  0    -v(1)   0     0     0    ;
     -v(2)  v(1)  0      0     0     0    ;
      0    -v(6)  v(5)   0    -v(3)  v(2) ;
      v(6)  0    -v(4)   v(3)  0    -v(1) ;
     -v(5)  v(4)  0     -v(2)  v(1)  0
     ]; */
}

arr Featherstone::crossM(const arr& v) { arr X; crossM(X, v); return X; }

//===========================================================================
#if 0
void Featherstone::invdyn_old(arr& tau, const Robot& robot, const arr& qd, const arr& qdd, const arr& grav) {
  /*
  % INVDYN  Calculate robot inverse dynamics.
  % invdyn(robot, q, qd, qdd) calculates the inverse dynamics of a robot using
  % the recursive Newton-Euler algorithm, evaluated in link coordinates.
  % Gravity is simulated by a fictitious base acceleration of [0, 0, 9.81] m/s^2
  % in base coordinates.  This can be overridden by supplying a 3D vector as
  % an optional fifth argument.
  */

  arr grav_accn(6);
  grav_accn.setZero();
  if(!grav.N) {
    //grav_accn(5)=9.81;
  } else {
    grav_accn.setVectorBlock(grav, 3);
  }

  uint i, N=robot.N(), d_i, n;
  rai::Array<arr> S(N), qd_i(N), qdd_i(N), tau_i(N);
  arr Xup(N, 6, 6), v(N, 6), f(N, 6), a(N, 6);
  arr Q;

  for(i=0, n=0; i<N; i++) {
    d_i=robot.dof(i);
    if(d_i) {
      qd_i(i) .referToRange(qd, n, n+d_i-1);
      qdd_i(i).referToRange(qdd, n, n+d_i-1);
      tau_i(i).referToRange(tau, n, n+d_i-1);
    } else {
      qd_i(i) .clear(); qd_i(i). resize(0);
      qdd_i(i).clear(); qdd_i(i).resize(0);
      tau_i(i).clear(); tau_i(i).resize(0);
    }
    n += d_i;
    S(i) = robot.S(i);
    if(robot.C->nodes(i)->hasJoint()) {
      FrameToMatrix(Q, robot.C->nodes(i)->joint()->X);
      Xup[i] = Q * robot.Xlink(i); //the transformation from the i-th to the j-th
    } else {
      Xup[i] = robot.Xlink(i); //the transformation from the i-th to the j-th
    }
  }
  CHECK(n==qd.N && n==qdd.N && n==tau.N, "")

  for(i=0; i<N; i++) {
    if(robot.parent(i) == -1) {
      v[i] = S(i) * qd_i(i);
      a[i] = Xup[i]*grav_accn + S(i)*qdd_i(i);
    } else {
      v[i] = Xup[i] * v[robot.parent(i)] + S(i) * qd_i(i);
      a[i] = Xup[i] * a[robot.parent(i)] + S(i) * qdd_i(i) + crossM(v[i])*S(i)*qd_i(i);
    }
    f[i] = robot.Ilink(i)*a[i] + crossF(v[i])*robot.Ilink(i)*v[i] - robot.force(i);

#if 0
    if(i) {
      rai::Transformation f, r, g;
      f=robot.C->nodes(i)->X;
      f.subRelativeFrame(robot.C->nodes(i)->joint()->B);
      arr vi(6);  vi.setVectorBlock(arr((f.r/f.w).v, 3), 0);  vi.setVectorBlock(arr((f.r/f.v).v, 3), 3);
      arr ai(6);  ai.setVectorBlock(arr((f.r/f.b).v, 3), 0);  ai.setVectorBlock(arr((f.r/f.a).v, 3), 3);

      cout <<"\ni=" <<i <<"\nv_i=" <<v[i] <<"\nf.(w, v)=" <<vi <<endl;
      cout <<"\na_i=" <<a[i] <<"\nf.(b, a)=" <<ai <<endl;
      CHECK(maxDiff(vi, v[i])<1e-4, "");
    }
#endif
  }

  for(i=N; i--;) {
    if(robot.dof(i)) {
      tau_i(i) = ~S(i) * f[i];
    }
    if(robot.parent(i) != -1) {
      f[robot.parent(i)] = f[robot.parent(i)] + ~Xup[i]*f[i];
    }
  }
}

//===========================================================================

void Featherstone::fwdDynamics_old(arr& qdd,
                                   const Robot& robot,
                                   const arr& qd,
                                   const arr& tau,
                                   const arr& grav) {
  /*
  % FDab  Forward Dynamics via Articulated-Body Algorithm
  % FDab(model, q, qd, tau, f_ext, grav_accn) calculates the forward dynamics of a
  % kinematic tree via the articulated-body algorithm.  q, qd and tau are
  % vectors of joint position, velocity and force variables; and the return
  % value is a vector of joint acceleration variables.  f_ext is a cell array
  % specifying external forces acting on the bodies.  If f_ext == {} then
  % there are no external forces; otherwise, f_ext{i} is a spatial force
  % vector giving the force acting on body i, expressed in body i
  % coordinates.  Empty cells in f_ext are interpreted as zero forces.
  % grav_accn is a 3D vector expressing the linear acceleration due to
  % gravity.  The arguments f_ext and grav_accn are optional, and default to
  % the values {} and [0, 0, -9.81], respectively, if omitted.
  */

  //CHANGE: default is gravity zero (assume to be included in external forces)
  arr a_grav(6);
  a_grav.setZero();
  if(grav.N) {
    a_grav.setVectorBlock(grav, 3);
  }

  int par;
  uint i, N=robot.N(), d_i, n;
  rai::Array<arr> h(N), qd_i(N), qdd_i(N), tau_i(N), I_h(N), h_I_h(N), inv_h_I_h(N), tau__h_fA(N);
  arr Xup(N, 6, 6), v(N, 6), dh_dq(N, 6), f(N, 6), IA(N, 6, 6), fA(N, 6), a(N, 6);
  arr vJ, Ia, fa;
  arr Q;

  for(i=0, n=0; i<N; i++) {
    //for general multi-dimensional joints, pick the sub-arrays
    d_i=robot.dof(i);
    if(d_i) {
      qd_i(i) .referToRange(qd, n, n+d_i-1);
      qdd_i(i).referToRange(qdd, n, n+d_i-1);
      tau_i(i).referToRange(tau, n, n+d_i-1);
    } else {
      qd_i(i) .clear(); qd_i(i). resize(0);
      qdd_i(i).clear(); qdd_i(i).resize(0);
      tau_i(i).clear(); tau_i(i).resize(0);
    }
    n += d_i;

    h(i) = robot.S(i);
    vJ = h(i) * qd_i(i); //equation (2), vJ = relative vel across joint i
    if(robot.C->nodes(i)->hasJoint()) {
      FrameToMatrix(Q, robot.C->nodes(i)->joint()->X);
      Xup[i] = Q * robot.Xlink(i); //the transformation from the i-th to the j-th
    } else {
      Xup[i] = robot.Xlink(i); //the transformation from the i-th to the j-th
    }
    if(robot.parent(i) == -1) {
      v[i] = vJ;
      dh_dq[i] = 0.;
    } else {
      v[i] = Xup[i] * v[robot.parent(i)] + vJ;
      dh_dq[i] = crossM(v[i]) * vJ;  //WHY??
    }
    // v[i] = total velocity, but in joint coordinates
    IA[i] = robot.Ilink(i);
    fA[i] = crossF(v[i]) * robot.Ilink(i) * v[i] - robot.force(i); //1st equation below (13)
  }

  for(i=N; i--;) {
    I_h(i) = IA[i] * h(i);
    if(robot.dof(i)) {
      h_I_h(i)      = ~h(i)*I_h(i);
      tau__h_fA(i) = tau_i(i) - ~h(i)*fA[i]; //[change from above] last term in (13), 2nd equation below (13)
    } else {
      h_I_h(i).clear();
      tau__h_fA(i).clear();
    }
    inverse(inv_h_I_h(i), h_I_h(i));
    par = robot.parent(i);
    if(par != -1) {
      Ia = IA[i] - I_h(i)*inv_h_I_h(i)*~I_h(i);
      fa = fA[i] + Ia*dh_dq[i] + I_h(i)*inv_h_I_h(i)*tau__h_fA(i);
      IA[par] = IA[par] + ~Xup[i] * Ia * Xup[i];         //equation (12)
      fA[par] = fA[par] + ~Xup[i] * fa;                  //equation (13)
    }
  }

  for(i=0; i<N; i++) {
    par=robot.parent(i);
    if(par == -1) {
      a[i] = Xup[i] * a_grav + dh_dq[i]; //[change from above]
    } else {
      a[i] = Xup[i] * a[par] + dh_dq[i]; //[change from above]
    }
    if(robot.dof(i)) {
      qdd_i(i) = inverse(h_I_h(i))*(tau__h_fA(i) - ~I_h(i)*a[i]); //equation (14)
    }
    a[i] = a[i] + h(i)*qdd_i(i); //equation above (14)
  }
}
#endif

//===========================================================================

/* Articulated Body Dynamics - exactly as in my `simulationSoftware notes',
   following the notation of Featherstone's recent short survey paper */
void FeatherstoneInterface::fwdDynamics_aba_nD(arr& qdd,
    const arr& qd,
    const arr& tau) {
  int par;
  uint i, N=tree.N, d_i, n;
  rai::Array<arr> h(N), qd_i(N), qdd_i(N), tau_i(N), I_h(N), h_I_h(N), u(N);
  arr Xup(N, 6, 6), v(N, 6), dh_dq(N, 6), IA(N, 6, 6), fA(N, 6), a(N, 6);
  qdd.resizeAs(tau);

  for(i=0, n=0; i<N; i++) {
    d_i=tree(i).dof();
    if(d_i) {
      qd_i(i) .referToRange(qd, n, n+d_i-1);
      qdd_i(i).referToRange(qdd, n, n+d_i-1);
      tau_i(i).referToRange(tau, n, n+d_i-1);
    } else {
      qd_i(i) .clear(); qd_i(i). resize(0);
      qdd_i(i).clear(); qdd_i(i).resize(0);
      tau_i(i).clear(); tau_i(i).resize(0);
    }
    n += d_i;
    h(i) = tree(i)._h;
    h(i).reshape(6, d_i);
    Xup[i] = tree(i)._Q; //the transformation from the i-th to the j-th
  }
  CHECK(n==qd.N && n==qdd.N && n==tau.N, "")

  for(i=0; i<N; i++) {
    par = tree(i).parent;
    if(par == -1) {
      v[i] = h(i) * qd_i(i);
      dh_dq[i] = 0.;
    } else {
      v[i] = Xup[i] * v[par] + h(i) * qd_i(i);
      dh_dq[i] = Featherstone::crossM(v[i]) * h(i) * qd_i(i);
    }
    IA[i] = tree(i)._I;
    fA[i] = Featherstone::crossF(v[i]) * tree(i)._I * v[i] - tree(i)._f;
  }

  for(i=N; i--;) {
    par = tree(i).parent;
    I_h(i) = IA[i] * h(i);
    if(tree(i).dof()) {
      h_I_h(i) = ~h(i)*I_h(i);
      u(i) = tau_i(i) - ~I_h(i)*dh_dq[i] - ~h(i)*fA[i];
    } else {
      h_I_h(i).clear(); h_I_h(i).resize(0);
      u(i).clear(); u(i).resize(0);
    }
    if(par != -1) {
      IA[par]() += ~Xup[i] * (IA[i] - I_h(i)*inverse(h_I_h(i))*~I_h(i)) * Xup[i];
      fA[par]() += ~Xup[i] * (fA[i] + IA[i]*dh_dq[i] + I_h(i)*inverse(h_I_h(i))*u(i));
    }
  }

  for(i=0; i<N; i++) {
    par=tree(i).parent;
    if(par == -1) {
      a[i] = 0; //Xup[i] * grav_accn;
    } else {
      a[i]() = Xup[i] * a[par];
    }
    if(tree(i).dof()) {
      qdd_i(i) = inverse(h_I_h(i))*(u(i) - ~I_h(i)*a[i]);
    }
    a[i] = a[i] + dh_dq[i] + h(i)*qdd_i(i);
  }
}

//===========================================================================

void FeatherstoneInterface::fwdDynamics_aba_1D(arr& qdd,
    const arr& qd,
    const arr& tau) {
  int par;
  int iq;
  uint i, N=tree.N;
  arr h(N, 6), I_h(N, 6), h_I_h(N), inv_h_I_h(N), taui(N), tau__h_fA(N);
  arr Xup(N, 6, 6), v(N, 6), dh_dq(N, 6), IA(N, 6, 6), fA(N, 6), a(N, 6);
  arr vJ, Ia, fa;
  qdd.resizeAs(tau);

  h.setZero();
  v.setZero();
  dh_dq.setZero();
  taui.setZero();

  //fwd: compute the velocities v[i] and external + Coriolis forces fA[i] of all bodies
  // v[i] = total velocity, but in joint coordinates
  for(i=0; i<N; i++) {
    F_Link& link = tree(i);
    iq  = link.qIndex;
    par = link.parent;
    Xup[i]() = link._Q; //the transformation from the i-th to the j-th
    if(par!=-1) {
      h[i]() = link._h;
      if(iq!=-1) {//is not a fixed joint
        vJ = h[i] * qd(iq); //equation (2), vJ = relative vel across joint i
        v[i]() = Xup[i] * v[par] + vJ; //eq (27)
        dh_dq[i]() = Featherstone::crossM(v[i]) * vJ;  //WHY??
        taui(i) = tau(iq);
      } else {
        v[i]() = Xup[i] * v[par]; //eq (27)
      }
    }
    IA[i]() = tree(i)._I;
    fA[i]() = Featherstone::crossF(v[i]) * (tree(i)._I * v[i]) - tree(i)._f;  //first part of eq (29)
  }

  //bwd: propagate tree inertia
  for(i=N; i--;) {
    F_Link& link = tree(i);
    par = link.parent;
    //eq (28)
    if(par!=-1) {
      if(link.qIndex!=-1) {
        I_h[i]()     = IA[i] * h[i];
        h_I_h(i)     = scalarProduct(h[i], I_h[i]);
        inv_h_I_h(i) = 1./h_I_h(i);
        tau__h_fA(i) = taui(i) - scalarProduct(h[i], fA[i]); //[change from above] last term in (13), 2nd equation below (13)
        Ia = IA[i] - I_h[i]*(inv_h_I_h(i)*~I_h[i]);
        fa = fA[i] + Ia*dh_dq[i] + I_h[i]*(inv_h_I_h(i)*tau__h_fA(i));
        IA[par] = IA[par] + ~Xup[i] * Ia * Xup[i];         //equation (12)
        fA[par] = fA[par] + ~Xup[i] * fa;                  //equation (13)
      } else {
        IA[par] = IA[par] + ~Xup[i] * IA[i] * Xup[i];         //equation (12)
        fA[par] = fA[par] + ~Xup[i] * fA[i];                  //equation (13)
      }
    }
  }

  for(i=0; i<N; i++) {
    F_Link& link = tree(i);
    iq = link.qIndex;
    par= link.parent;
    if(par != -1) {
      a[i] = Xup[i] * a[par] + dh_dq[i]; //[change from above]
      if(iq!=-1) {
        qdd(iq) = inv_h_I_h(i)*(tau__h_fA(i) - scalarProduct(I_h[i], a[i])); //equation (14)
        a[i] = a[i] + h[i]*qdd(iq); //equation above (14)
      }
    } else {
      a[i] = dh_dq[i]; //[change from above]
    }
  }
}

//===========================================================================

void FeatherstoneInterface::invDynamics(arr& tau,
                                        const arr& qd,
                                        const arr& qdd) {
  int par;
  uint i, N=tree.N, d_i, qidx;
  rai::Array<arr> h(N), qd_i(N), qdd_i(N), tau_i(N);
  arr Xup(N, 6, 6), v(N, 6), fJ(N, 6), a(N, 6);
  tau.resizeAs(qdd);

  for(i=0; i<N; i++) {
    d_i = tree(i).dof();
    qidx = tree(i).qIndex;
    if(d_i) {
      qd_i(i) .referToRange(qd, qidx, qidx+d_i-1);
      qdd_i(i).referToRange(qdd, qidx, qidx+d_i-1);
      tau_i(i).referToRange(tau, qidx, qidx+d_i-1);
    } else {
      qd_i(i) .clear(); qd_i(i). resize(0);
      qdd_i(i).clear(); qdd_i(i).resize(0);
      tau_i(i).clear(); tau_i(i).resize(0);
    }
    if(d_i!=0) {
      h(i) = tree(i)._h;
      h(i).reshape(6, d_i);
    } else {
      h(i).resize(6, 0u);
    }
    Xup[i] = tree(i)._Q; //the transformation from the i-th to the j-th
  }
  //CHECK(n==qd.N && n==qdd.N && n==tau.N, "")

  for(i=0; i<N; i++) {
    par = tree(i).parent;
    if(par == -1) {
      v[i] = h(i) * qd_i(i);
      a[i] = h(i) * qdd_i(i);
    } else {
      v[i] = Xup[i] * v[par] + h(i) * qd_i(i);
      a[i] = Xup[i] * a[par] + h(i) * qdd_i(i) + Featherstone::crossM(v[i]) * h(i) * qd_i(i);
    }
    //see featherstone-orin paper for definition of fJ (different to fA; it's about force equilibrium at a joint)
    fJ[i] = tree(i)._I*a[i] + Featherstone::crossF(v[i]) * tree(i)._I * v[i] - tree(i)._f;
  }

  for(i=N; i--;) {
    par = tree(i).parent;
    d_i = tree(i).dof();
    if(d_i!=0){
      qidx = tree(i).qIndex;
      tau_i(qidx) = ~h(i) * fJ[i];
    }
    if(par != -1)     fJ[par]() += ~Xup[i] * fJ[i];
  }
}

//===========================================================================

void FeatherstoneInterface::equationOfMotion(arr& H, arr& C,
    const arr& qd) {

  /*function  [H, C] = HandC( model, q, qd, f_ext, grav_accn )

  % HandC  Calculate coefficients of equation of motion.
  % [H, C]=HandC(model, q, qd, f_ext, grav_accn) calculates the coefficients of
  % the joint-space equation of motion, tau=H(q)qdd+C(d, qd, f_ext), where q,
  % qd and qdd are the joint position, velocity and acceleration vectors, H
  % is the joint-space inertia matrix, C is the vector of gravity,
  % external-force and velocity-product terms, and tau is the joint force
  % vector.  Algorithm: recursive Newton-Euler for C, and
  % Composite-Rigid-Body for H.  f_ext is a cell array specifying external
  % forces acting on the bodies.  If f_ext == {} then there are no external
  % forces; otherwise, f_ext{i} is a spatial force vector giving the force
  % acting on body i, expressed in body i coordinates.  Empty cells in f_ext
  % are interpreted as zero forces.  grav_accn is a 3D vector expressing the
  % linear acceleration due to gravity.  The arguments f_ext and grav_accn
  % are optional, and default to the values {} and [0, 0, -9.81], respectively,
  % if omitted.
  */

  int par;
  int iq, jq;
  uint i, j, N=tree.N;
  //CHECK_EQ(N-1,qd.N,"vels don't have right dimension")
  arr h(N, 6);
  arr Xup(N, 6, 6), v(N, 6), dh_dq(N, 6), IC(N, 6, 6), fvp(N, 6), avp(N, 6);
  arr vJ, fh;

  h.setZero();
  v.setZero();
  dh_dq.setZero();
  avp.setZero();

  for(i=0; i<N; i++) {
    iq  = tree(i).qIndex;
    par = tree(i).parent;
    Xup[i]() = tree(i)._Q; //the transformation from the i-th to the j-th
    if(par!=-1) {
      h[i]() = tree(i)._h;
      if(iq!=-1) {//is not a fixed joint
        vJ = h[i] * qd(iq); //equation (2), vJ = relative vel across joint i
        v[i]() = Xup[i]*v[par] + vJ;
        dh_dq[i]() = Featherstone::crossM(v[i]) * vJ;  //WHY??
        avp[i]() = Xup[i]*avp[par] + Featherstone::crossM(v[i])*vJ;
      } else {
        v[i]() = Xup[i] * v[par];
        avp[i]() = Xup[i] * avp[par];
      }
    }
    IC[i]() = tree(i)._I;
    fvp[i]() = tree(i)._I*avp[i] + Featherstone::crossF(v[i])*(tree(i)._I*v[i]) - tree(i)._f;
  }

  C.resize(qd.N).setZero();

  for(i=N; i--;) {
    iq  = tree(i).qIndex;
    par = tree(i).parent;
    if(iq!=-1) {
      C(iq) += scalarProduct(h[i], fvp[i]);
    }
    if(par!=-1) {
      fvp[par]() += ~Xup[i] * fvp[i];
      IC[par]() += ~Xup[i] * IC[i] * Xup[i];
    }
  }

  H.resize(qd.N, qd.N).setZero();

  for(i=0; i<N; i++) {
    iq = tree(i).qIndex;
    fh = IC[i] * h[i];
    if((int)iq!=-1) {
      H(iq, iq) += scalarProduct(h[i], fh);
    }
    j = i;
    while(tree(j).parent!=-1) {
      fh = ~Xup[j] * fh;
      j  = tree(j).parent;
      jq = tree(j).qIndex;
      if(jq!=-1 && iq!=-1) {
        double Hij = scalarProduct(h[j], fh);
        H(iq, jq) += Hij;
        H(jq, iq) += Hij;
      }
    }
  }

  //add friction for non-filled joints
  boolA filled(qd.N);
  filled=false;
  for(i=0; i<N; i++) { iq = tree(i).qIndex; if(iq!=-1) filled(iq)=true; }
  for(i=0; i<qd.N; i++) if(!filled(i)) {
      H(i, i) = 1.;
      //C(i) = -100.*qd(i);
    }
}

//===========================================================================

void FeatherstoneInterface::fwdDynamics_MF(arr& qdd,
    const arr& qd,
    const arr& u) {

  arr M, Minv, F;
  equationOfMotion(M, F, qd);
  inverse(Minv, M);
//  inverse_SymPosDef(Minv, M);

  qdd = Minv * (u - F);
}

// #else ///RAI_FEATHERSTONE
// void GraphToTree(F_LinkTree& tree, const rai::Configuration& C) { NIY; }
// void updateGraphToTree(F_LinkTree& tree, const rai::Configuration& C) { NIY; }
// void Featherstone::equationOfMotion(arr& H, arr& C,
//                                     const F_LinkTree& tree,
//                                     const arr& qd) { NIY; }
// void Featherstone::fwdDynamics_MF(arr& qdd,
//                                   const F_LinkTree& tree,
//                                   const arr& qd,
//                                   const arr& tau) { NIY; }
// void Featherstone::invDynamics(arr& tau,
//                                const F_LinkTree& tree,
//                                const arr& qd,
//                                const arr& qdd) { NIY; }
// #endif
