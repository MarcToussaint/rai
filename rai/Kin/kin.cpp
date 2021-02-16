/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kin.h"
#include "frame.h"
#include "forceExchange.h"
#include "uncertainty.h"
#include "proxy.h"
#include "kin_swift.h"
#include "kin_physx.h"
#include "kin_ode.h"
#include "kin_feather.h"
#include "featureSymbols.h"
#include "viewer.h"
#include "../Core/graph.h"
#include "../Geo/fclInterface.h"
#include "../Geo/qhull.h"
#include "../Geo/mesh_readAssimp.h"
#include "../GeoOptim/geoOptim.h"
#include "../Gui/opengl.h"
#include "../Algo/algos.h"
#include <iomanip>
#include <algorithm>
#include <sstream>
#include <climits>

#ifdef RAI_ASSIMP
#  include <assimp/Exporter.hpp>
#  include <assimp/scene.h>
#endif

#ifdef RAI_GL
#  include <GL/gl.h>
#  include <GL/glu.h>
#endif

namespace rai {

uint Configuration::setJointStateCount = 0;

//===========================================================================
//
// contants
//

Frame& NoFrame = *((Frame*)nullptr);
Shape& NoShape = *((Shape*)nullptr);
Joint& NoJoint = *((Joint*)nullptr);
Configuration __NoWorld;
Configuration& NoConfiguration = *((Configuration*)&__NoWorld);

uintA framesToIndices(const FrameL& frames) {
  uintA I;
  resizeAs(I, frames);
  for(uint i=0; i<frames.N; i++) I.elem(i) = frames.elem(i)->ID;
  return I;
}

uintA jointsToIndices(const JointL& joints) {
  uintA I;
  resizeAs(I, joints);
  for(uint i=0; i<joints.N; i++) I.elem(i) = joints.elem(i)->frame->ID;
  return I;
}

void makeConvexHulls(FrameL& frames, bool onlyContactShapes) {
  for(Frame* f: frames) if(f->shape && (!onlyContactShapes || f->shape->cont))
      f->shape->mesh().makeConvexHull();
}

void computeOptimalSSBoxes(FrameL& frames) {
  NIY;
#if 0
  //  for(Shape *s: shapes) s->mesh.computeOptimalSSBox(s->mesh.V);
  Shape* s;
  for(Frame* f: frames) if((s=f->shape)) {
      if(!(s->type()==ST_mesh && s->mesh.V.N)) continue;
      Transformation t;
      arr x;
      computeOptimalSSBox(s->mesh, x, t, s->mesh.V);
      s->type() = ST_ssBox;
      s->size(0)=2.*x(0); s->size(1)=2.*x(1); s->size(2)=2.*x(2); s->size(3)=x(3);
      s->mesh.setSSBox(s->size(0), s->size(1), s->size(2), s->size(3));
      s->frame.Q.appendTransformation(t);
    }
#endif
}

void computeMeshNormals(FrameL& frames, bool force) {
  for(Frame* f: frames) if(f->shape) {
      Shape* s = f->shape;
      if(force || s->mesh().V.d0!=s->mesh().Vn.d0 || s->mesh().T.d0!=s->mesh().Tn.d0) s->mesh().computeNormals();
      if(force || s->sscCore().V.d0!=s->sscCore().Vn.d0 || s->sscCore().T.d0!=s->sscCore().Tn.d0) s->sscCore().computeNormals();
    }
}

void computeMeshGraphs(FrameL& frames, bool force) {
  for(Frame* f: frames) if(f->shape) {
      Shape* s = f->shape;
      if(force || s->mesh().V.d0!=s->mesh().graph.N|| s->mesh().T.d0!=s->mesh().Tn.d0) s->mesh().buildGraph();
      if(force || s->sscCore().V.d0!=s->sscCore().graph.N || s->sscCore().T.d0!=s->sscCore().Tn.d0) s->sscCore().buildGraph();
    }
}

//===========================================================================
//
// Configuration
//

struct sConfiguration {
  shared_ptr<ConfigurationViewer> viewer;
  shared_ptr<SwiftInterface> swift;
  shared_ptr<FclInterface> fcl;
  unique_ptr<PhysXInterface> physx;
  unique_ptr<OdeInterface> ode;
  unique_ptr<FeatherstoneInterface> fs;
};

Configuration::Configuration() {
  self = make_unique<sConfiguration>();
}

Configuration::~Configuration() {
  //delete OpenGL and the extensions first!
  self->viewer.reset();
  self->swift.reset();
  self->fcl.reset();
  clear();
  self.reset();
}

/// make this a copy of C (copying all frames, forces & proxies)
void Configuration::copy(const Configuration& C, bool referenceSwiftOnCopy) {
  CHECK(this != &C, "never copy C onto itself");

  clear();
  orsDrawProxies = C.orsDrawProxies;
  jacMode = C.jacMode;

  //copy frames; first each Frame/Link/Joint directly, where all links go to the origin K (!!!); then relink to itself
  for(Frame* f:C.frames) new Frame(*this, f);
  for(Frame* f:C.frames) if(f->parent) frames.elem(f->ID)->linkFrom(frames.elem(f->parent->ID));
//  addFramesCopy(C.frames);

  //copy proxies; first they point to origin frames; afterwards, let them point to own frames
  copyProxies(C.proxies);
  //  proxies = K.proxies;
  //  for(Proxy& p:proxies) { p.a = frames.elem(p.a->ID); p.b = frames.elem(p.b->ID);  p.coll.reset(); }

  //copy contacts
  for(ForceExchange* ex:C.forces){
    new ForceExchange(*frames.elem(ex->a.ID), *frames.elem(ex->b.ID), ex->type, ex);
  }

  //copy swift reference
  if(referenceSwiftOnCopy) {
    self->swift = C.self->swift;
    self->fcl = C.self->fcl;
  }

  //copy vector state
  q = C.q;
  qInactive = C.qInactive;
  _state_q_isGood = C._state_q_isGood;
  ensure_indexedJoints();
}

bool Configuration::operator!() const { return this==&NoConfiguration; }

Frame* Configuration::addFrame(const char* name, const char* parent, const char* args) {
  Frame* f = new Frame(*this);
  f->name = name;

  if(parent && parent[0]) {
    Frame* p = getFrame(parent);
    if(p) {
      f->set_X() = p->ensure_X();
      f->linkFrom(p, true);
    }
  }

  if(args && args[0]) {
    String(args) >>f->ats;
    f->read(f->ats);
  }

//  if(f->parent) f->calc_X_from_parent();

  return f;
}

Frame* Configuration::addFile(const char* filename) {
  uint n=frames.N;
  FileToken file(filename, true);
  Graph G(file);
  readFromGraph(G, true);
  file.cd_start();
  if(frames.N==n) return 0; //no frames added
  return frames.elem(n); //returns 1st frame of added file
}

void Configuration::addAssimp(const char* filename) {
  AssimpLoader A(filename, true, true);
  //-- create all frames
  uint Nold = frames.N;
  for(uint i=0;i<A.names.N;i++){
    addFrame(A.names(i));
  }
  //-- link to parents
  for(uint i=0;i<A.names.N;i++){
    Frame* f = frames(Nold+i);
    if(A.parents(i).N){
      Frame* f = frames(Nold+i);
      rai::Frame *parent = getFrame(A.parents(i));
      if(parent) f->linkFrom(parent);
    }
    f->set_X() = A.poses(i);
  }
  for(uint i=0;i<A.names.N;i++){
    Frame* f = frames(Nold+i);
    if(A.meshes(i).N==1){
      if(A.meshes(i)(0).V.N){
        Shape* s = new Shape(*f);
        s->type() = ST_mesh;
        s->mesh() = A.meshes(i).scalar();
      }
    }else if(A.meshes(i).N>1){
      uint j=0;
      for(auto& mesh:A.meshes(i)){
        if(mesh.V.N){
          Frame* f1 = addFrame(STRING(f->name<<'_' <<j++));
          f1->linkFrom(f);
          f1->set_Q()->setZero();
          Shape* s = new Shape(*f1);
          s->type() = ST_mesh;
          s->mesh() = mesh;
        }
      }
    }
  }
}

#if 0
Frame* Configuration::addObject(ShapeType shape, const arr& size, const arr& col) {
  Frame* f = new Frame(*this);
  Shape* s = new Shape(*f);
  s->type() = shape;
  if(col.N) s->mesh().C = col;
  if(radius>0.) s->size() = ARR(radius);
  if(shape!=ST_mesh && shape!=ST_ssCvx) {
    if(size.N>=1) s->size() = size;
    s->createMeshes();
  } else {
    if(shape==ST_mesh) {
      s->mesh().V = size;
      s->mesh().V.reshape(-1, 3);
    }
    if(shape==ST_ssCvx) {
      s->sscCore().V = size;
      s->sscCore().V.reshape(-1, 3);
      CHECK(radius>0., "radius must be greater zero");
      s->size() = ARR(radius);
    }
  }
  return f;
}
#endif

#if 0
/// default initialization of a frame as object - see code
Frame* Configuration::addObject(const char* name, const char* parent, ShapeType shape, const arr& size, const arr& col, const arr& pos, const arr& rot, bool isSubFrame) {
  Frame* f = addFrame(name, parent);
  if(f->parent && !isSubFrame) f->setJoint(JT_rigid);
  f->setShape(shape, size);
  f->setContact(-1);
  if(col.N) f->setColor(col);
  if(f->parent) {
    if(pos.N) f->setRelativePosition(pos);
    if(rot.N) f->setRelativeQuaternion(rot);
  } else {
    if(pos.N) f->setPosition(pos);
    if(rot.N) f->setQuaternion(rot);
  }
  return f;
}
#endif

/// add copies of all given frames and forces, which can be from another Configuration -> \ref frames array becomes sliced! (a matrix)
void Configuration::addCopies(const FrameL& F, const ForceExchangeL& _forces) {
  //prepare an index FId -> thisId
  uint maxId=0;
  for(Frame* f:F) if(f->ID>maxId) maxId=f->ID;
  intA FId2thisId(maxId+1);
  FId2thisId = -1;

  //create new copied frames
  for(Frame* f:F) {
    Frame* f_new = new Frame(*this, f);
    FId2thisId(f->ID) = f_new->ID;

    //convert constant joints to mimic joints
    if(f->joint && f->ats["constant"]){
      Frame *f_orig = getFrame(f_new->name); //identify by name!!!
      if(f_orig!=f_new){
        CHECK(f_orig->joint, "");
        f_new->joint->setMimic(f_orig->joint);
      }
    }
  }

  //relink frames - special attention to mimic'ing
  for(Frame* f:F) if(f->parent && f->parent->ID<=maxId && FId2thisId(f->parent->ID)!=-1) {
    Frame* f_new = frames.elem(FId2thisId(f->ID));
    f_new->linkFrom(frames.elem(FId2thisId(f->parent->ID)));
    //take care of within-F mimic joints:
    if(f->joint && f->joint->mimic){
      CHECK(f->joint && f->joint->mimic, "");
      f_new->joint->setMimic(frames.elem(FId2thisId(f->joint->mimic->frame->ID))->joint);
    }
  }

  //copy force exchanges
  for(ForceExchange* ex:_forces){
    new ForceExchange(*frames.elem(FId2thisId(ex->a.ID)), *frames.elem(FId2thisId(ex->b.ID)), ex->type, ex);
  }

  if(!(frames.N%F.N)) frames.reshape(-1, F.N);
}

/// get first frame with given name
Frame* Configuration::getFrame(const char* name, bool warnIfNotExist, bool reverse) const {
  if(!reverse) {
    for(Frame* b: frames) if(b->name==name) return b;
  } else {
    for(uint i=frames.N; i--;) if(frames.elem(i)->name==name) return frames.elem(i);
  }
  if(warnIfNotExist) RAI_MSG("cannot find frame named '" <<name <<"'");
  return 0;
}

/// get all frames of given indices (almost same as \ref frames . Array::sub() )
FrameL Configuration::getFrames(const uintA& ids) const {
  if(frames.nd==1) return frames.sub(ids);
  FrameL F(ids.N);
  for(uint i=0;i<ids.N;i++) F.elem(i) = frames.elem(ids.elem(i));
  return F;
}

/// calls getFrame() for all given names
FrameL Configuration::getFrames(const StringA& names) const {
  if(!names.N) return FrameL();
  FrameL F(names.N);
  for(uint i=0; i<names.N; i++) {
    Frame* f = getFrame(names(i));
    if(!f) HALT("frame name '"<<names(i)<<"' doesn't exist");
    F.elem(i) = f;
  }
  return F;
}

/// calls getFrame() for all given names
uintA Configuration::getFrameIDs(const StringA& names) const {
  if(!names.N) return uintA();
  uintA I(names.N);
  for(uint i=0; i<names.N; i++) {
    Frame* f = getFrame(names(i));
    if(!f) HALT("frame name '"<<names(i)<<"' doesn't exist");
    I.elem(i) = f->ID;
  }
  return I;
}

FrameL Configuration::getJoints(bool activesOnly) const{
  FrameL F;
  for(auto* f:frames) if(f->joint && (!activesOnly || f->joint->active)) F.append(f);
  return F;
}

FrameL Configuration::getJointsSlice(uint t, bool activesOnly) const{
  FrameL F;
  for(auto* f:frames[t]){
    if((f->joint && (!activesOnly || f->joint->active))
       || f->forces.N)  F.append(f);
  }
  return F;
}

/// get the frame IDs of all active joints
uintA Configuration::getJointIDs() const {
  ((Configuration*)this)->ensure_indexedJoints();
  uintA joints(activeJoints.N);
  uint i=0;
  for(Joint* j:activeJoints) joints(i++) = j->frame->ID;
  return joints;
}

/// get the frame names of all active joints
StringA Configuration::getJointNames() const {
  ((Configuration*)this)->ensure_q();
  StringA names(getJointStateDimension());
  for(Joint* j:activeJoints) {
    String name=j->frame->name;
    if(!name) name <<'q' <<j->qIndex;
    if(j->dim==1) names(j->qIndex) <<name;
    else for(uint i=0; i<j->dim; i++) names(j->qIndex+i) <<name <<':' <<i;

    if(j->uncertainty) {
      if(j->dim) {
        for(uint i=j->dim; i<2*j->dim; i++) names(j->qIndex+i) <<name <<":UC:" <<i;
      } else {
        names(j->qIndex+1) <<name <<":UC";
      }
    }
  }
  return names;
}

/// get the names of all frames
StringA Configuration::getFrameNames() const {
  StringA names(frames.N);
  for(uint i=0; i<frames.N; i++) {
    names(i) = frames.elem(i)->name;
  }
  return names;
}

/// get the (frame-ID, parent-ID) tuples and control scale for all active joints that represent controls
uintA Configuration::getCtrlFramesAndScale(arr& scale) const {
  uintA qFrames;
  for(rai::Frame* f : frames) {
    rai::Joint *j = f->joint;
    if(j && j->active && j->dim>0 && (!j->mimic) && j->H>0. && j->type!=rai::JT_tau && (!f->ats["constant"])) {
      qFrames.append(TUP(f->ID, f->parent->ID));
      if(!!scale) scale.append(j->H, j->dim);
    }
  }
  qFrames.reshape(-1, 2);
  return qFrames;
}

/// get all frames without parent
FrameL Configuration::getRoots() const {
  FrameL roots;
  for(Frame* f:frames) if(!f->parent) roots.append(f);
  return roots;
}

/// get all frames without parent or with joint
FrameL Configuration::getLinks() const {
  FrameL links;
  for(Frame* a:frames) if(!a->parent || a->joint) links.append(a);
  return links;
}

/// get the number of DOFs (size of the q-vector, including DOFs of joints and forces
uint Configuration::getJointStateDimension() const {
  ((Configuration*)this)->ensure_q();
  return q.N;
}

/// get the q-vector, including DOFs of joints and forces
const arr& Configuration::getJointState() const {
  if(!_state_q_isGood)((Configuration*)this)->ensure_q();
  return q;
}

/// get a q-vector for only a subset of joints (no force DOFs)
arr Configuration::getJointState(const FrameL& joints, bool activesOnly) const {
  ((Configuration*)this)->ensure_q();
  uint nd=0;
  for(Frame* f:joints) {
    Joint* j = f->joint;
    if(!j){ LOG(-1) <<"frame '" <<f->name <<"'[" <<f->ID <<"] is not a joint!"; continue; }
    if(!j->active && activesOnly) HALT("frame '" <<f->name <<"' is a joint, but INACTIVE!");
    if(!j->mimic){
      nd += j->dim;
    }
  }

  arr x(nd);
  nd=0;
  for(Frame* f:joints) {
    Joint* j = f->joint;
    if(!j) continue; //{ LOG(-1) <<"frame '" <<f->name <<"' is not a joint!"; continue; }
    if(!j->mimic){
      if(j->active){
        for(uint ii=0; ii<j->dim; ii++) x(nd+ii) = q(j->qIndex+ii);
      }else if(!activesOnly){
        for(uint ii=0; ii<j->dim; ii++) x(nd+ii) = qInactive(j->qIndex+ii);
      }else HALT("");
      nd += j->dim;
    }
  }
  CHECK_EQ(nd, x.N, "");
  return x;
}

/// get the (F.N,7)-matrix of all poses for all given frames
arr Configuration::getFrameState(const FrameL& F) const {
  arr X(F.N, 7);
  for(uint i=0; i<X.d0; i++) {
    X[i] = F.elem(i)->ensure_X().getArr7d();
  }
  return X;
}

/// set the q-vector (all joint and forces DOFs)
void Configuration::setJointState(const arr& _q) {
  setJointStateCount++; //global counter

#ifndef RAI_NOCHECK
  uint N=getJointStateDimension();
  CHECK_EQ(_q.N, N, "wrong joint state dimensionalities");
#endif

  q=_q;

  proxies.clear();

  _state_q_isGood=true;
  _state_proxies_isGood=false;
  for(Joint* j:activeJoints) {
    if(j->type!=JT_tau) {
      j->frame->_state_setXBadinBranch();
    }
  }
  calc_Q_from_q();
}

/// set the DOFs (joints and forces) for the given subset of frames
void Configuration::setJointState(const arr& _q, const FrameL& F, bool activesOnly) {
  setJointStateCount++; //global counter
  getJointState();

  uint nd=0;
  for(Frame* f:F) {
    Joint* j = f->joint;
    if(!j && !f->forces.N) HALT("frame '" <<f->name <<"' is not a joint and has no forces!");
    if(!j) continue;
    if(j->active){
      if(!j->mimic){
        for(uint ii=0; ii<j->dim; ii++) q.elem(j->qIndex+ii) = _q(nd+ii);
      }
      j->calc_Q_from_q(q, j->qIndex);
    }else{
      if(activesOnly) HALT("frame '" <<f->name <<"' is a joint, but INACTIVE!");
      if(!j->mimic){
        for(uint ii=0; ii<j->dim; ii++) qInactive.elem(j->qIndex+ii) = _q(nd+ii);
      }
      j->calc_Q_from_q(qInactive, j->qIndex);
    }
    if(!j->mimic) nd += j->dim;
  }
  for(Frame* f:F) {
    for(ForceExchange* c: f->forces) if(&c->a==f){
      c->calc_F_from_q(q, c->qIndex);
      nd += c->qDim();
    }
  }
  CHECK_EQ(_q.N, nd, "");

  proxies.clear();

  _state_q_isGood=true;
  _state_proxies_isGood=false;
}


/// set the pose of all frames as given by the (F.N,7)-matrix
void Configuration::setFrameState(const arr& X, const FrameL& F) {
  CHECK_EQ(X.d0, F.N, "X.d0=" <<X.d0 <<" is larger than frames.N=" <<F.N);
  for(Frame* f:F) f->_state_setXBadinBranch();
  for(uint i=0; i<F.N; i++) {
    Frame *f = F.elem(i);
    f->X.set(X[i]);
    f->X.rot.normalize();
    f->_state_X_isGood = true;
  }
  for(Frame* f:F) if(f->parent){
    f->Q.setDifference(f->parent->ensure_X(), f->X);
    _state_q_isGood=false;
  }
}

/// set the 'tau-coordinate' (time interval from previous time slice) for equal for all frames
void Configuration::setTaus(double tau) {
  for(Frame* a:frames) a->tau = tau;
}

/// selects only the joints of the given frames to be active -- the q-vector (and Jacobians) will refer only to those DOFs
void Configuration::selectJoints(const FrameL& F, bool notThose) {
  for(Frame* f: frames) if(f->joint) f->joint->active = notThose;
  for(Frame* f: F) if(f && f->joint){
    f->joint->active = !notThose;
    if(f->joint->mimic) f->joint->mimic->active = f->joint->active; //activate also the joint mimic'ed
  }
  for(Frame* f: frames) if(f && f->joint && f->joint->mimic){ //mimic's of active joints are active as well
    if(f->joint->mimic->active) f->joint->active = true;
  }
  reset_q();
//  ensure_indexedJoints();
//  ensure_q();
//  checkConsistency();
}

/// select joint frames that have a given attribute in the g-file
void Configuration::selectJointsByGroup(const StringA& groupNames, bool notThose) {
  FrameL F;
  for(Frame* f:frames) if(f->joint) {
    for(const String& s:groupNames) if(f->ats[s]) { F.append(f); break; }
  }
  selectJoints(F, notThose);
}

/// select joint frames by name (names may refer to any part of a link, not necessarily the joint frame)
void Configuration::selectJointsByName(const StringA& names, bool notThose) {
  FrameL F;
  for(const String& s:names) {
    Frame* f = getFrame(s);
    CHECK(f, "");
    f = f->getUpwardLink();
    CHECK(f->joint, "");
    F.append(f);
  }
  selectJoints(F, notThose);
}

/// select joint frames of trees given by the set of roots
void Configuration::selectJointsBySubtrees(const FrameL& roots, bool notThose) {
  FrameL F;
  for(Frame* f: roots) {
    F.append(f);
    f->getSubtree(F);
  }
  selectJoints(F, notThose);
}

/// returns diagonal of the metric in q-space determined by all joints' Joint::H
arr Configuration::getCtrlMetric() const {
  arr H = zeros(getJointStateDimension());
  for(Joint* j: activeJoints) {
    double h=j->H;
    //    CHECK(h>0.,"Hmetric should be larger than 0");
    if(j->type==JT_transXYPhi) {
      H(j->qIndex+0)=h*10.;
      H(j->qIndex+1)=h*10.;
      H(j->qIndex+2)=h;
    } else {
      for(uint k=0; k<j->qDim(); k++) H(j->qIndex+k)=h;
    }
  }
  return H;
}

/// returns diagonal of a natural metric in q-space, depending on tree depth
arr Configuration::getNaturalCtrlMetric(double power) const {
  HALT("don't use this anymore. use getHmetric instead");
#if 0
  if(!q.N) getJointStateDimension();
  arr Wdiag(q.N);
  Wdiag=1.;
  return Wdiag;
#else
  //compute generic q-metric depending on tree depth
  arr BM(frames.N);
  BM=1.;
  for(uint i=BM.N; i--;) {
    for(uint j=0; j<frames.elem(i)->children.N; j++) {
      BM(i) = MAX(BM(frames.elem(i)->children(j)->ID)+1., BM(i));
      //      BM(i) += BM(bodies(i)->children(j)->to->index);
    }
  }
  if(!q.N) getJointStateDimension();
  arr Wdiag(q.N);
  for(Joint* j: activeJoints) {
    for(uint i=0; i<j->qDim(); i++) {
      Wdiag(j->qIndex+i) = ::pow(BM(j->frame->ID), power);
    }
  }
  return Wdiag;
#endif
}

/// returns the vector of joint limts */
arr Configuration::getLimits() const {
  uint N=getJointStateDimension();
  arr limits(N, 2);
  limits.setZero();
  for(Joint* j: activeJoints) {
    uint i=j->qIndex;
    uint d=j->qDim();
    for(uint k=0; k<d; k++) { //in case joint has multiple dimensions
      if(j->limits.N) {
        limits(i+k, 0)=j->limits(2*k+0); //lo
        limits(i+k, 1)=j->limits(2*k+1); //up
      }
    }
  }
  for(ForceExchange* f: forces) {
    uint i=f->qIndex;
    uint d=f->qDim();
    CHECK_EQ(d, 6, "");
    for(uint k=0; k<3; k++) {
      limits(i+k, 0)=-10.; //lo
      limits(i+k, 1)=+10.; //up
    }
    for(uint k=3; k<6; k++) {
      limits(i+k, 0)=-1.; //lo
      limits(i+k, 1)=+1.; //up
    }
  }
//    cout <<"limits:" <<limits <<endl;
  return limits;
}

/// get the total energy of the configuration
double Configuration::getEnergy(const arr& qdot) {
  double m, v, E;
  Matrix I;
  Vector w;

  arr vel = calc_fwdPropagateVelocities(qdot);

  E=0.;
  for(Frame* f: frames) if(f->inertia) {
      Vector linVel = vel(f->ID, 0, {});
      Vector angVel = vel(f->ID, 1, {});

      m=f->inertia->mass;
      const Quaternion& rot = f->ensure_X().rot;
      I=(rot).getMatrix() * f->inertia->matrix * (-rot).getMatrix();
      v = linVel.length();
      w = angVel;
      E += .5*m*v*v;
      E += 9.81 * m * (f->ensure_X()*f->inertia->com).z;
      E += .5*(w*(I*w));
    }

  return E;
}

/// get the sum of all shape penetrations -- PRECONDITION: proxies have been computed (with stepSwift() or stepFcl())
double Configuration::getTotalPenetration() {
  CHECK(_state_proxies_isGood, "");

  double D=0.;
  for(const Proxy& p:proxies) {
    //early check: if swift is way out of collision, don't bother computing it precise
    if(p.d > p.a->shape->radius()+p.b->shape->radius()+.01) continue;
    //exact computation
    if(!p.collision)((Proxy*)&p)->calc_coll();
    double d = p.collision->getDistance();
    if(d<0.) D -= d;
  }
  //  for(Frame *f:frames) for(ForceExchange *c:f->forces) if(&c->a==f) {
  //        double d = c->getDistance();
  //      }
  return D;
}


Graph Configuration::reportForces() {
  Graph G;
  for(ForceExchange* f:forces) {
    Graph& g = G.newSubgraph();
    g.newNode<String>({"from"}, {}, f->a.name);
    g.newNode<String>({"to"}, {}, f->b.name);
    g.newNode<arr>({"force"}, {}, f->force);
    g.newNode<arr>({"torque"}, {}, f->torque);
    g.newNode<arr>({"poa"}, {}, f->poa);
  }
  return G;
}

/// checks if all names of the bodies are disjoint
bool Configuration::checkUniqueNames() const {
  for(Frame* a:  frames) for(Frame* b: frames) {
      if(a==b) break;
      if(a->name==b->name) return false;
    }
  return true;
}

/// compute the natural order of all frames (first adding all root frames, then expanding using breadth-first search)
FrameL Configuration::calc_topSort() const {
  FrameL order;
  boolA done = consts<byte>(false, frames.N);

  FrameL fringe = getRoots();
  if(frames.N) CHECK(fringe.N, "none of the frames is a root -- must be loopy!");

  while(fringe.N) {
    Frame* a = fringe.popFirst();
    order.append(a);
    done(a->ID) = true;

    for(Frame* ch : a->children) fringe.append(ch);
  }

  for(uint i=0; i<done.N; i++) if(!done(i)) LOG(-1) <<"not done: " <<frames.elem(i)->name <<endl;
  CHECK_EQ(order.N, frames.N, "can't top sort");

  return order;
}

/// check if the current \ref frames is topologically sorted
bool Configuration::check_topSort() const {
  //compute levels
  intA level = consts<int>(0, frames.N);
  for(Frame* f: frames) if(f->parent) level(f->ID) = level(f->parent->ID)+1;
  //check levels are strictly increasing across links
  for(Frame* f: frames) if(f->parent && level(f->parent->ID) >= level(f->ID)) return false;

  return true;
}


/// clear all frames, forces & proxies
void Configuration::clear() {
//  glClose();
  swiftDelete();

  reset_q();
  proxies.clear(); //while(proxies.N){ delete proxies.last(); /*checkConsistency();*/ }
  while(frames.N) { delete frames.last(); /*checkConsistency();*/ }
  reset_q();

  _state_proxies_isGood=false;
}

/// clear the q-vector
void Configuration::reset_q() {
  q.clear();
  qInactive.clear();
  activeJoints.clear();

  _state_indexedJoints_areGood=false;
  _state_q_isGood=false;
}

/** @brief re-orient all joints (edges) such that n becomes
  the root of the configuration */
void Configuration::reconfigureRoot(Frame* newRoot, bool ofLinkOnly) {
  FrameL pathToOldRoot;

  if(ofLinkOnly) pathToOldRoot = newRoot->getPathToUpwardLink(true);
  else pathToOldRoot = newRoot->getPathToRoot();

//  listWrite(pathToOldRoot);

  Frame* oldRoot=pathToOldRoot.first();
  Frame* rootParent=oldRoot->parent;
  if(rootParent) oldRoot->unLink();

  for(Frame* f : pathToOldRoot) {
    if(f->parent) flipFrames(f->parent, f);
  }

//  if(rootParent){
//    newRoot->linkFrom(rootParent);
//    newRoot->setJoint(JT_rigid);
//  }

//  checkConsistency();
}

/** @brief revert the topological orientation of a joint (edge),
   e.g., when choosing another body as root of a tree */
void Configuration::flipFrames(Frame* a, Frame* b) {
  CHECK_EQ(b->parent, a, "");
  CHECK(!a->parent, "");
  CHECK(!a->joint, "");
  if(b->joint) {
    b->joint->flip();
  }
  a->Q = -b->Q;
  b->Q.setZero();
  b->unLink();
  a->linkFrom(b);
}

void Configuration::pruneRigidJoints() {
  for(Frame* f:frames){
    if(f->joint && f->joint->type == JT_rigid) f->setJoint(JT_none);
  }
}

void Configuration::pruneInactiveJoints() {
  for(Frame* f:frames){
    if(f->joint && !f->joint->active) f->setJoint(JT_none);
  }
}

void Configuration::reconnectLinksToClosestJoints() {
  reset_q();
  for(Frame* f:frames) if(f->parent) {
#if 0
      Frame* link = f->parent;
      Transformation Q=f->Q;
      while(link->parent && !link->joint) { //walk down links until this is a joint
        Q = link->Q * Q;                 //accumulate transforms
        link = link->parent;
      }
#else
      Transformation Q;
      Frame* link = f->getUpwardLink(Q);
      Q.rot.normalize();
#endif
      if(f->joint && !Q.rot.isZero) continue; //only when rot is zero you can subsume the Q transformation into the Q of the joint
      if(link!=f) { //there is a link's root
        if(link!=f->parent) { //we can rewire to the link's root
          f->parent->children.removeValue(f);
          link->children.append(f);
          f->parent = link;
          f->set_Q() = Q;
        }
      }
    }
}

void Configuration::pruneUselessFrames(bool pruneNamed, bool pruneNonContactNonMarker) {
  for(uint i=frames.N; i--;) {
    Frame* f=frames.elem(i);
    if((pruneNamed || !f->name) && !f->children.N && !f->joint && !f->inertia) {
      if(!f->shape)
        delete f; //that's all there is to do
      else if(pruneNonContactNonMarker && !f->shape->cont && f->shape->type()!=ST_marker)
        delete f;
    }
  }
}

void Configuration::optimizeTree(bool _pruneRigidJoints, bool pruneNamed, bool pruneNonContactNonMarker) {
  if(_pruneRigidJoints) pruneRigidJoints(); //problem: rigid joints bear the semantics of where a body ends
  reconnectLinksToClosestJoints();
  pruneUselessFrames(pruneNamed, pruneNonContactNonMarker);
  ensure_indexedJoints();
  checkConsistency();
}

void Configuration::sortFrames() {
  frames = calc_topSort();
  uint i=0;
  for(Frame* f: frames) f->ID = i++;
}

void Configuration::makeObjectsFree(const StringA& objects, double H_cost) {
  for(auto s:objects) {
    Frame* a = getFrame(s, true);
    CHECK(a, "");
    a = a->getUpwardLink();
    if(!a->parent) a->linkFrom(frames.first());
    if(!a->joint) new Joint(*a);
    a->joint->makeFree(H_cost);
  }
}

void Configuration::addTauJoint() {
  Joint* jt = new Joint(*frames.first(), JT_tau);
  jt->H = 0.;
}

bool Configuration::hasTauJoint(Frame *f) {
  if(!f) f = frames.first();
  else f = f->getRoot();
  return f && f->joint && (f->joint->type==JT_tau);
}

bool Configuration::checkConsistency() const {
  /* state consistency concept:
     the configuration represents exactly one concrete current configuration, not multiple different ones in q or Q or X
     'good' means correctly reflecting the current configuration
     the relative transforms Q of all frames are ALWAYS 'good'
     when a frame has no parent, Q is identical to X and X is 'good'
     when a frame has a parent, X may be non-good, but can always be computed using ensure_X
     q may generaly be non-good
     when setJointState is called, q becomes good and all Q are recomputed to stay consistent
     when frame_setX... is called, q becomes non-good, all frame descendents will have non-good X, Q for that frame is recomputed (from its parent's X), and X for that frame is good as well

     when initially loading a configuration, q and all X are typically non-good
     */
  //check qdim
  if(_state_q_isGood) {
    CHECK_EQ(1, q.nd, "");

    //count dimensions yourself and check...
    uint myqdim = 0;
    for(Joint* j: activeJoints) {
      if(j->mimic) {
        CHECK_EQ(j->qIndex, j->mimic->qIndex, "");
      } else {
        CHECK_EQ(j->qIndex, myqdim, "joint indexing is inconsistent");
        if(!j->uncertainty)
          myqdim += j->qDim();
        else
          myqdim += 2*j->qDim();
      }
    }
    for(ForceExchange* c: forces) {
      CHECK_EQ(c->qDim(), 6, "");
      CHECK_EQ(c->qIndex, myqdim, "joint indexing is inconsistent");
      myqdim += c->qDim();
    }
    CHECK_EQ(myqdim, q.N, "qdim is wrong");

    //consistency with Q
    for(Frame* f : frames) if(f->joint){
      Joint *j = f->joint;
      arr jq = j->calc_q_from_Q(f->Q);
      CHECK_EQ(jq.N, j->dim, "");
      if(j->active){
        for(uint i=0; i<jq.N; i++) CHECK_ZERO(jq.elem(i) - q.elem(j->qIndex+i), 1e-6, "joint vector q and relative transform Q do not match for joint '" <<j->frame->name <<"', index " <<i);
      }else{
        for(uint i=0; i<jq.N; i++) CHECK_ZERO(jq.elem(i) - qInactive.elem(j->qIndex+i), 1e-6, "joint vector q and relative transform Q do not match for joint '" <<j->frame->name <<"', index " <<i);
      }
    }
  }

  for(Frame* a: frames) {
    CHECK(&a->C, "");
    CHECK(&a->C==this, "");
    CHECK_EQ(a, frames.elem(a->ID), "");
    for(Frame* b: a->children) CHECK_EQ(b->parent, a, "");
    if(a->joint) CHECK_EQ(a->joint->frame, a, "");
    if(a->shape) CHECK_EQ(&a->shape->frame, a, "");
    if(a->inertia) CHECK_EQ(&a->inertia->frame, a, "");
    a->ats.checkConsistency();

    a->Q.checkNan();
    a->X.checkNan();
    CHECK_ZERO(a->Q.rot.normalization()-1., 1e-6, "");
    CHECK_ZERO(a->X.rot.normalization()-1., 1e-6, "");

    // frame has no parent -> Q needs to be zero, X is good
    if(!a->parent) {
      CHECK(a->_state_X_isGood, "");
      CHECK(a->Q.isZero(), "");
    }
    // frame has a parent -> X may be non-good, otherwise it must be consistent with Q
    if(a->parent && a->_state_X_isGood) {
      CHECK(a->parent->_state_X_isGood, "");
      Transformation test = a->parent->X * a->Q;
      CHECK_ZERO((a->X / test).diffZero(), 1e-6, "");
    }
  }

  Joint* j;
  for(Frame* f: frames) if((j=f->joint)) {
      if(j->type.x!=JT_tau) {
        CHECK(j->from(), "");
        CHECK(j->from()->children.findValue(j->frame)>=0, "");
      }
      CHECK_EQ(j->frame->joint, j, "");
      CHECK_GE(j->type.x, 0, "");
      CHECK_LE(j->type.x, JT_tau, "");
      CHECK_EQ(j->dim, j->getDimFromType(), "");

      if(j->mimic) {
        CHECK(j->mimic>(void*)1, "mimic was not parsed correctly");
        CHECK(frames.contains(j->mimic->frame), "mimic points to a frame outside this kinematic configuration");
      } else {
      }

      for(Joint* m:j->mimicers) {
        CHECK_EQ(m->mimic, j, "");
      }
    }

  //check topsort
  if(_state_indexedJoints_areGood) {
    intA level = consts<int>(0, frames.N);
    //compute levels
//    for(Frame *f: fwdActiveSet)
//      if(f->parent) level(f->ID) = level(f->parent->ID)+1;
//    //check levels are strictly increasing across links
//    for(Frame *f: fwdActiveSet) if(f->parent) {
//      CHECK(level(f->parent->ID) < level(f->ID), "joint from '" <<f->parent->name <<"'[" <<f->parent->ID <<"] to '" <<f->name <<"'[" <<f->ID <<"] does not go forward");
//    }
  }

  //check active sets
  if(_state_indexedJoints_areGood) {
    boolA jointIsInActiveSet = consts<byte>(false, frames.N);
    for(Joint* j: activeJoints) { CHECK(j->active, ""); jointIsInActiveSet.elem(j->frame->ID)=true; }
    if(q.nd) {
      for(Frame* f: frames) if(f->joint && f->joint->active && f->joint->type!=JT_rigid) CHECK(jointIsInActiveSet(f->ID), "");
    }
  }

  //check isZero for all transformations
  for(Frame* a: frames) {
    a->X.pos.checkZero();
    a->X.rot.checkZero();
    a->Q.pos.checkZero();
    a->Q.rot.checkZero();
  }

  for(const Proxy& p : proxies) {
    CHECK_EQ(this, &p.a->C, "");
    CHECK_EQ(this, &p.b->C, "");
  }

  return true;
}

Joint* Configuration::attach(Frame* a, Frame* b) {
  b = b->getUpwardLink();
  if(b->parent) b->unLink();
  b->linkFrom(a, true);
  return new Joint(*b, JT_rigid);
}

Joint* Configuration::attach(const char* _a, const char* _b) {
  return attach(getFrame(_a), getFrame(_b));
}

void exclude(uintA& ex, FrameL& F1, FrameL& F2) {
  for(Frame* f1:F1) {
    for(Frame* f2:F2) {
      if(f1->ID < f2->ID) {
        ex.append(TUP(f1->ID, f2->ID));
      }
    }
  }
}

uintA Configuration::getCollisionExcludeIDs(bool verbose) {
  uintA ex;
  for(Frame* f: frames) if(f->shape){
    if(!f->shape->cont) ex.append(f->ID);
  }
  return ex;
}

uintA Configuration::getCollisionExcludePairIDs(bool verbose) {
  /* exclude collision pairs:
    -- no collisions between shapes of same body
    -- no collisions between linked bodies
    -- no collisions between bodies liked via the tree via 3 links
  */

  uintA ex;

  //shapes within a link
  FrameL links = getLinks();
  for(Frame* f: links) {
    FrameL F = {f};
    f->getRigidSubFrames(F);
    for(uint i=F.N; i--;) if(!F(i)->shape || !F(i)->shape->cont) F.remove(i);
    if(F.N>1) {
      if(verbose) {
        LOG(0) <<"excluding intra-link collisions: ";
        cout <<"           ";  listWriteNames(F, cout);  cout <<endl;
      }
      exclude(ex, F, F);
    }
  }

  //deactivate upward, depending on cont parameter (-1 indicates deactivate with parent)
  for(Frame* f: frames) {
    if(f->shape && f->shape->cont<0) {
      FrameL F, P;
      Frame* p = f->getUpwardLink();
      F = {p};
      p->getRigidSubFrames(F);
      for(uint i=F.N; i--;) if(!F(i)->shape || !F(i)->shape->cont) F.remove(i);

      for(char i=0; i<-f->shape->cont; i++) {
        p = p->parent;
        if(!p) break;
        p = p->getUpwardLink();
        P = {p};
        p->getRigidSubFrames(P);
        for(uint i=P.N; i--;) if(!P(i)->shape || !P(i)->shape->cont) P.remove(i);

        if(F.N && P.N) {
          if(verbose) {
            LOG(0) <<"excluding between-sets collisions: ";
            cout <<"           ";  listWriteNames(F, cout);  cout <<endl;
            cout <<"           ";  listWriteNames(P, cout);  cout <<endl;
          }
          exclude(ex, F, P);
        }
      }
    }
  }

  ex.reshape(-1,2);
  return ex;
}

/// creates uniques names by prefixing the node-index-number to each name */
void Configuration::prefixNames(bool clear) {
  if(!clear) for(Frame* a: frames) a->name=STRING('_' <<a->ID <<'_' <<a->name);
  else       for(Frame* a: frames) a->name.clear() <<a->ID;
}

void Configuration::calc_indexedActiveJoints() {
  reset_q();

  //-- collect active joints
  activeJoints.clear();
  for(Frame* f:frames) if(f->joint && f->joint->active && f->joint->type!=JT_rigid)
      activeJoints.append(f->joint);

  _state_indexedJoints_areGood=true;

  //-- count active DOFs
  uint qcount=0;
  for(Joint* j: activeJoints) {
    j->dim = j->getDimFromType();
    if(!j->mimic) {
      j->qIndex = qcount;
      if(!j->uncertainty)
        qcount += j->qDim();
      else
        qcount += 2*j->qDim();
    } else {
      CHECK(j->mimic->active, "active joint '" << j->frame->name <<"' mimics inactive joint '" <<j->mimic->frame->name <<"'");
      j->qIndex = j->mimic->qIndex;
    }
  }
  for(ForceExchange* c: forces) {
    CHECK_EQ(c->qDim(), 6, "");
    c->qIndex = qcount;
    qcount += c->qDim();
  }

  //-- resize q
  q.resize(qcount).setZero();
  _state_q_isGood = false;

  //-- count inactive DOFs
  qcount=0;
  for(Frame* f:frames) if(f->joint && !f->joint->active){ //include counting mimic'ing!
    Joint *j = f->joint;
    j->dim = j->getDimFromType();
    j->qIndex = qcount;
    if(!j->uncertainty)
      qcount += j->qDim();
    else
      qcount += 2*j->qDim();
  }

  //-- resize qInactive
  qInactive.resize(qcount).setZero();
}


void Configuration::calc_q_from_Q() {
  ensure_indexedJoints();

  q.setZero();
  qInactive.setZero();

  uint n=0;
  //-- active joints (part of the DOFs)
  for(Joint* j: activeJoints) {
    if(j->mimic) continue; //don't count dependent joints
    CHECK_EQ(j->qIndex, n, "joint indexing is inconsistent");
    arr joint_q = j->calc_q_from_Q(j->frame->Q);
    CHECK_EQ(joint_q.N, j->dim, "");
    if(!j->dim) continue; //nothing to do
    q.setVectorBlock(joint_q, j->qIndex);
    n += j->dim;
    if(j->uncertainty) {
      q.setVectorBlock(j->uncertainty->sigma, j->qIndex+j->dim);
      n += j->dim;
    }
  }

  //-- forces (part of the DOFs)
  for(ForceExchange* c: forces) {
    CHECK_EQ(c->qIndex, n, "joint indexing is inconsistent");
    arr contact_q = c->calc_q_from_F();
    CHECK_EQ(contact_q.N, c->qDim(), "");
    q.setVectorBlock(contact_q, c->qIndex);
    n += c->qDim();
  }
  CHECK_EQ(n, q.N, "");

  //-- inactive joints (not part of DOFs)
  n=0;
  for(Frame* f: frames) if(f->joint && !f->joint->active){ //this includes mimic'ing joints!
    Joint *j = f->joint;
    CHECK_EQ(j->qIndex, n, "joint indexing is inconsistent");
    arr joint_q = j->calc_q_from_Q(f->Q);
    CHECK_EQ(joint_q.N, j->dim, "");
    if(!f->joint->dim) continue; //nothing to do
    qInactive.setVectorBlock(joint_q, j->qIndex);
    n += j->dim;
  }
  CHECK_EQ(n, qInactive.N, "");
}

void Configuration::calc_Q_from_q() {
  CHECK(_state_q_isGood, "");
  CHECK(_state_indexedJoints_areGood, "");

  uint n=0;
  for(Joint* j: activeJoints) {
    if(!j->mimic) CHECK_EQ(j->qIndex, n, "joint indexing is inconsistent");
    j->calc_Q_from_q(q, j->qIndex);
    if(!j->mimic) {
      n += j->dim;
      if(j->uncertainty) {
        j->uncertainty->sigma = q.sub(j->qIndex+j->dim, j->qIndex+2*j->dim-1);
        n += j->dim;
      }
    }
  }
  for(ForceExchange* c: forces) {
    CHECK_EQ(c->qIndex, n, "joint indexing is inconsistent");
    c->calc_F_from_q(q, c->qIndex);
    n += c->qDim();
  }
  CHECK_EQ(n, q.N, "");
}

arr Configuration::calc_fwdPropagateVelocities(const arr& qdot) {
  CHECK(check_topSort(), "this needs a top sorted configuration")
  arr vel(frames.N, 2, 3);  //for every frame we have a linVel and angVel, each 3D
  vel.setZero();
  Transformation f;
  Vector linVel, angVel, q_vel, q_angvel;
  for(Frame* f : frames) { //this has no bailout for loopy graphs!
    f->ensure_X();
    if(f->parent) {
      Frame* from = f->parent;
      Joint* j = f->joint;
      if(j) {
        linVel = vel(from->ID, 0, {});
        angVel = vel(from->ID, 1, {});

        if(j->type==JT_hingeX) {
          q_vel.setZero();
          q_angvel.set(qdot(j->qIndex), 0., 0.);
        } else if(j->type==JT_transX) {
          q_vel.set(qdot(j->qIndex), 0., 0.);
          q_angvel.setZero();
        } else if(j->type==JT_rigid) {
          q_vel.setZero();
          q_angvel.setZero();
        } else if(j->type==JT_transXYPhi) {
          q_vel.set(qdot(j->qIndex), qdot(j->qIndex+1), 0.);
          q_angvel.set(0., 0., qdot(j->qIndex+2));
        } else NIY;

        Matrix R = j->X().rot.getMatrix();
        Vector qV(R*q_vel); //relative vel in global coords
        Vector qW(R*q_angvel); //relative ang vel in global coords
        linVel += angVel^(f->X.pos - from->X.pos);
        /*if(!isLinkTree) */linVel += qW^(f->get_X().pos - j->X().pos);
        linVel += qV;
        angVel += qW;

        for(uint i=0; i<3; i++) vel(f->ID, 0, i) = linVel(i);
        for(uint i=0; i<3; i++) vel(f->ID, 1, i) = angVel(i);
      } else {
        linVel = vel(from->ID, 0, {});
        angVel = vel(from->ID, 1, {});

        linVel += angVel^(f->get_X().pos - from->get_X().pos);

        for(uint i=0; i<3; i++) vel(f->ID, 0, i) = linVel(i);
        for(uint i=0; i<3; i++) vel(f->ID, 1, i) = angVel(i);
      }
    }
  }
  return vel;
}

//===========================================================================
//
// core: kinematics and dynamics
//

/// returns the 'Jacobian' of a zero n-vector (initializes Jacobian to proper sparse/dense/rowShifted/noArr)
void Configuration::jacobian_zero(arr& J, uint n) const {
  uint N=getJointStateDimension();
  if(jacMode==JM_dense) {
    J.resize(n, N).setZero();
  } else if(jacMode==JM_sparse){
    J.sparse().resize(n, N, 0);
  } else if(jacMode==JM_rowShifted){
    //estimate! the width
    uint width = N;
    if(frames.nd==2 && frames.d0>3){ width /= (frames.d0/4); width += 10; }
    J.rowShifted().resize(n, N, width);
  } else if(jacMode==JM_noArr){
    J.setNoArr();
  } else NIY;
}

void Configuration::kinematicsZero(arr& y, arr& J, uint n) const {
  y.resize(n).setZero();
  jacobian_zero(J, n);
}

/// what is the linear velocity of a world point (pos_world) attached to frame a for a given joint velocity?
void Configuration::jacobian_pos(arr& J, Frame* a, const Vector& pos_world) const {
  CHECK_EQ(&a->C, this, "");

  a->ensure_X();

  uint N=getJointStateDimension();
  jacobian_zero(J, 3);
  if(!J) return;

  while(a) { //loop backward down the kinematic tree
    if(!a->parent) break; //frame has no inlink -> done
    Joint* j=a->joint;
    if(j && j->active) {
      uint j_idx=j->qIndex;
      if(j_idx>=N) if(j->active) CHECK_EQ(j->type, JT_rigid, "");
      if(j_idx<N) {
        if(j->type==JT_hingeX || j->type==JT_hingeY || j->type==JT_hingeZ) {
          Vector tmp = j->axis ^ (pos_world-j->X()*j->Q().pos);
          tmp *= j->scale;
          J.elem(0, j_idx) += tmp.x;
          J.elem(1, j_idx) += tmp.y;
          J.elem(2, j_idx) += tmp.z;
        } else if(j->type==JT_transX || j->type==JT_transY || j->type==JT_transZ || j->type==JT_XBall) {
          J.elem(0, j_idx) += j->scale * j->axis.x;
          J.elem(1, j_idx) += j->scale * j->axis.y;
          J.elem(2, j_idx) += j->scale * j->axis.z;
        } else if(j->type==JT_transXY) {
          arr R = j->X().rot.getArr();
          R *= j->scale;
          J.setMatrixBlock(R.sub(0, -1, 0, 1), 0, j_idx);
        } else if(j->type==JT_transXYPhi) {
          arr R = j->X().rot.getArr();
          R *= j->scale;
          J.setMatrixBlock(R.sub(0, -1, 0, 1), 0, j_idx);
          Vector tmp = j->axis ^ (pos_world-(j->X().pos + j->X().rot*a->Q.pos));
          tmp *= j->scale;
          J.elem(0, j_idx+2) += tmp.x;
          J.elem(1, j_idx+2) += tmp.y;
          J.elem(2, j_idx+2) += tmp.z;
        } else if(j->type==JT_phiTransXY) {
          Vector tmp = j->axis ^ (pos_world-j->X().pos);
          tmp *= j->scale;
          J.elem(0, j_idx) += tmp.x;
          J.elem(1, j_idx) += tmp.y;
          J.elem(2, j_idx) += tmp.z;
          arr R = (j->X().rot*a->Q.rot).getArr();
          R *= j->scale;
          J.setMatrixBlock(R.sub(0, -1, 0, 1), 0, j_idx+1);
        }
        if(j->type==JT_XBall) {
          arr R = conv_vec2arr(j->X().rot.getX());
          R *= j->scale;
          R.reshape(3, 1);
          J.setMatrixBlock(R, 0, j_idx);
        }
        if(j->type==JT_trans3 || j->type==JT_free) {
          arr R = j->X().rot.getArr();
          R *= j->scale;
          J.setMatrixBlock(R, 0, j_idx);
        }
        if(j->type==JT_quatBall || j->type==JT_free || j->type==JT_XBall) {
          uint offset = 0;
          if(j->type==JT_XBall) offset=1;
          if(j->type==JT_free) offset=3;
          arr Jrot = j->X().rot.getArr() * a->Q.rot.getJacobian(); //transform w-vectors into world coordinate
          Jrot = crossProduct(Jrot, conv_vec2arr(pos_world-(j->X().pos+j->X().rot*a->Q.pos)));  //cross-product of all 4 w-vectors with lever
          Jrot /= sqrt(sumOfSqr(q({j->qIndex+offset, j->qIndex+offset+3})));   //account for the potential non-normalization of q
          //          for(uint i=0;i<4;i++) for(uint k=0;k<3;k++) J.elem(k,j_idx+offset+i) += Jrot(k,i);
          Jrot *= j->scale;
          J.setMatrixBlock(Jrot, 0, j_idx+offset);
        }
      }
    }
    a = a->parent;
  }

//  if(isSparseMatrix(J) && xIndex) {
//    J.sparse().reshape(J.d0, J.d1+xIndex);
//    J.sparse().rowShift(xIndex);
//  }
}

/// what is the angular velocity of frame a for a given joint velocity?
void Configuration::jacobian_angular(arr& J, Frame* a) const {
  a->ensure_X();

  uint N = getJointStateDimension();
  jacobian_zero(J, 3);
  if(!J) return;

  while(a) { //loop backward down the kinematic tree
    Joint* j=a->joint;
    if(j && j->active) {
      uint j_idx=j->qIndex;
      if(j_idx>=N) CHECK_EQ(j->type, JT_rigid, "");
      if(j_idx<N) {
        if((j->type>=JT_hingeX && j->type<=JT_hingeZ) || j->type==JT_transXYPhi || j->type==JT_phiTransXY) {
          if(j->type==JT_transXYPhi) j_idx += 2; //refer to the phi only
          J.elem(0, j_idx) += j->scale * j->axis.x;
          J.elem(1, j_idx) += j->scale * j->axis.y;
          J.elem(2, j_idx) += j->scale * j->axis.z;
        }
        if(j->type==JT_quatBall || j->type==JT_free || j->type==JT_XBall) {
          uint offset = 0;
          if(j->type==JT_XBall) offset=1;
          if(j->type==JT_free) offset=3;
          arr Jrot = j->X().rot.getArr() * a->get_Q().rot.getJacobian(); //transform w-vectors into world coordinate
          Jrot /= sqrt(sumOfSqr(q({j->qIndex+offset, j->qIndex+offset+3}))); //account for the potential non-normalization of q
          //          for(uint i=0;i<4;i++) for(uint k=0;k<3;k++) J.elem(k,j_idx+offset+i) += Jrot(k,i);
          Jrot *= j->scale;
          J.setMatrixBlock(Jrot, 0, j_idx+offset);
        }
        //all other joints: J=0 !!
      }
    }
    a = a->parent;
  }
}

/// how does the time coordinate of frame a change with q-change?
void Configuration::jacobian_tau(arr& J, Frame* a) const {
  HALT("use kinematicsTau?");
  CHECK_EQ(&a->C, this, "");

  //get Jacobian
  uint N=getJointStateDimension();
  jacobian_zero(J, 1);
  if(!J) return;

  while(a) { //loop backward down the kinematic tree
    Joint* j=a->joint;
    if(j && j->active) {
      uint j_idx=j->qIndex;
      if(j_idx>=N) CHECK_EQ(j->type, JT_rigid, "");
      if(j_idx<N) {
        if(j->type==JT_tau) {
          J.elem(0, j_idx) += 1e-1;
        }
      }
    }
    if(!a->parent) break; //frame has no inlink -> done
    a = a->parent;
  }
}

/** @brief return the jacobian \f$J = \frac{\partial\phi_i(q)}{\partial q}\f$ of the position
  of the i-th body (3 x n tensor)*/
void Configuration::kinematicsPos(arr& y, arr& J, Frame* a, const Vector& rel) const {
  CHECK_EQ(&a->C, this, "given frame is not element of this Configuration");

  Vector pos_world = a->ensure_X().pos;
  if(!!rel && !rel.isZero) pos_world += a->ensure_X().rot*rel;
  if(!!y) y = conv_vec2arr(pos_world);
  if(!!J) jacobian_pos(J, a, pos_world);
}

/* takes the joint state x and returns the jacobian dz of
   the position of the ith body (w.r.t. all joints) -> 2D array */
/// Jacobian of the i-th body's z-orientation vector
void Configuration::kinematicsVec(arr& y, arr& J, Frame* a, const Vector& vec) const {
  CHECK_EQ(&a->C, this, "");
  CHECK(!!vec, "need a vector");

  Vector vec_world;
  vec_world = a->ensure_X().rot*vec;
  if(!!y) y = conv_vec2arr(vec_world);
  if(!!J) {
    arr A;
    jacobian_angular(A, a);
    J = crossProduct(A, conv_vec2arr(vec_world));
  }
}

/* takes the joint state x and returns the jacobian dz of
   the position of the ith body (w.r.t. all joints) -> 2D array */
/// Jacobian of the i-th body's z-orientation vector
void Configuration::kinematicsQuat(arr& y, arr& J, Frame* a) const { //TODO: allow for relative quat
  CHECK_EQ(&a->C, this, "");

  const Quaternion& rot_a = a->ensure_X().rot;
  if(!!y) y = rot_a.getArr4d();
  arr ROT_A = rot_a.getQuaternionMultiplicationMatrix();

  arr A;
  jacobian_angular(A, a);
  if(!A){
    J.setNoArr();
    return;
  }
  if(A.isSparse()) {
    J = A;
    J.sparse().reshape(4, A.d1);
    J.sparse().colShift(1);
    J *= .5;
    J = ROT_A * J;
  } else if(isRowShifted(A)) {
    J = A;
    J *= .5;
    J.rowShifted().insRow(0);
    J = ROT_A * J;
  } else if(!isSpecial(A)) {
    J.resize(4, A.d1).setZero();
    J.setMatrixBlock(A, 1, 0);
    J *= .5;
    J = ROT_A * J;
  } else NIY;
}

void Configuration::kinematicsTau(double& tau, arr& J, Frame* a) const {
  if(!a) a=frames.first();
  else a = a->getRoot();
  CHECK(a && a->joint && a->joint->type==JT_tau, "this configuration does not have a tau DOF");

  Joint* j = a->joint;
  tau = a->tau;
  if(!!J) {
    jacobian_zero(J, 1);
    J.elem(0, j->qIndex) += 1e-1;
  }
}

/** @brief return the jacobian \f$J = \frac{\partial\phi_i(q)}{\partial q}\f$ of the position
  of the i-th body W.R.T. the 6 axes of an arbitrary shape-frame, NOT the robot's joints (3 x 6 tensor)
  WARNING: this does not check if s is actually in the kinematic chain from root to b.
*/
void Configuration::kinematicsPos_wrtFrame(arr& y, arr& J, Frame* b, const Vector& rel, Frame* s) const {
  if(!b && !!J) { J.resize(3, getJointStateDimension()).setZero();  return; }

  //get position
  Vector pos_world = b->ensure_X().pos;
  if(!!rel) pos_world += b->ensure_X().rot*rel;
  if(!!y) y = conv_vec2arr(pos_world); //return the output
  if(!J) return; //do not return the Jacobian

  //get Jacobian
  J.resize(3, 6).setZero();
  Vector diff = pos_world - s->ensure_X().pos;
  Array<Vector> axes = {s->ensure_X().rot.getX(), s->ensure_X().rot.getY(), s->ensure_X().rot.getZ()};

  //3 translational axes
  for(uint i=0; i<3; i++) {
    J(0, i) += axes(i).x;
    J(1, i) += axes(i).y;
    J(2, i) += axes(i).z;
  }

  //3 rotational axes
  for(uint i=0; i<3; i++) {
    Vector tmp = axes(i) ^ diff;
    J(0, 3+i) += tmp.x;
    J(1, 3+i) += tmp.y;
    J(2, 3+i) += tmp.z;
  }
}

/** @brief return the Hessian \f$H = \frac{\partial^2\phi_i(q)}{\partial q\partial q}\f$ of the position
  of the i-th body (3 x n x n tensor) */
void Configuration::hessianPos(arr& H, Frame* a, Vector* rel) const {
  HALT("this is buggy: a sign error: see examples/Kin/ors testKinematics");
  Joint* j1, *j2;
  uint j1_idx, j2_idx;
  Vector tmp, pos_a;

  uint N=getJointStateDimension();

  //initialize Jacobian
  H.resize(3, N, N);
  H.setZero();

  //get reference frame
  pos_a = a->ensure_X().pos;
  if(rel) pos_a += a->ensure_X().rot*(*rel);

  if((j1=a->joint)) {
    while(j1) {
      j1_idx=j1->qIndex;

      j2=j1;
      while(j2) {
        j2_idx=j2->qIndex;

        if(j1->type>=JT_hingeX && j1->type<=JT_hingeZ && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //both are hinges
          tmp = j2->axis ^ (j1->axis ^ (pos_a-j1->X().pos));
          H(0, j1_idx, j2_idx) = H(0, j2_idx, j1_idx) = tmp.x;
          H(1, j1_idx, j2_idx) = H(1, j2_idx, j1_idx) = tmp.y;
          H(2, j1_idx, j2_idx) = H(2, j2_idx, j1_idx) = tmp.z;
        } else if(j1->type>=JT_transX && j1->type<=JT_transZ && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans, j=hinge
          tmp = j1->axis ^ j2->axis;
          H(0, j1_idx, j2_idx) = H(0, j2_idx, j1_idx) = tmp.x;
          H(1, j1_idx, j2_idx) = H(1, j2_idx, j1_idx) = tmp.y;
          H(2, j1_idx, j2_idx) = H(2, j2_idx, j1_idx) = tmp.z;
        } else if(j1->type==JT_transXY && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans3, j=hinge
          NIY;
        } else if(j1->type==JT_transXYPhi && j1->type==JT_phiTransXY && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans3, j=hinge
          NIY;
        } else if(j1->type==JT_trans3 && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans3, j=hinge
          Matrix R, A;
          j1->X().rot.getMatrix(R.p());
          A.setSkew(j2->axis);
          R = R*A;
          H(0, j1_idx, j2_idx) = H(0, j2_idx, j1_idx) = R.m00;
          H(1, j1_idx, j2_idx) = H(1, j2_idx, j1_idx) = R.m10;
          H(2, j1_idx, j2_idx) = H(2, j2_idx, j1_idx) = R.m20;
          H(0, j1_idx+1, j2_idx) = H(0, j2_idx, j1_idx+1) = R.m01;
          H(1, j1_idx+1, j2_idx) = H(1, j2_idx, j1_idx+1) = R.m11;
          H(2, j1_idx+1, j2_idx) = H(2, j2_idx, j1_idx+1) = R.m21;
          H(0, j1_idx+2, j2_idx) = H(0, j2_idx, j1_idx+2) = R.m02;
          H(1, j1_idx+2, j2_idx) = H(1, j2_idx, j1_idx+2) = R.m12;
          H(2, j1_idx+2, j2_idx) = H(2, j2_idx, j1_idx+2) = R.m22;
        } else if(j1->type>=JT_hingeX && j1->type<=JT_hingeZ && j2->type>=JT_transX && j2->type<=JT_trans3) { //i=hinge, j=trans
          //nothing! Hessian is zero (ej is closer to root than ei)
        } else NIY;

        j2=j2->from()->joint;
        if(!j2) break;
      }
      j1=j1->from()->joint;
      if(!j1) break;
    }
  }
}

void Configuration::equationOfMotion(arr& M, arr& F, const arr& qdot, bool gravity) {
  fs().update();
  fs().setGravity();
  fs().equationOfMotion(M, F, qdot);
}

/** @brief return the joint accelerations \f$\ddot q\f$ given the
  joint torques \f$\tau\f$ (computed via Featherstone's Articulated Body Algorithm in O(n)) */
void Configuration::fwdDynamics(arr& qdd, const arr& qd, const arr& tau, bool gravity) {
  fs().update();
  fs().setGravity();
  //  cout <<tree <<endl;
  fs().fwdDynamics_MF(qdd, qd, tau);
  //  fs().fwdDynamics_aba_1D(qdd, qd, tau); //works
  //  fwdDynamics_aba_nD(qdd, tree, qd, tau); //does not work
}

/** @brief return the necessary joint torques \f$\tau\f$ to achieve joint accelerations
  \f$\ddot q\f$ (computed via the Recursive Newton-Euler Algorithm in O(n)) */
void Configuration::inverseDynamics(arr& tau, const arr& qd, const arr& qdd, bool gravity) {
  fs().update();
  fs().setGravity();
  fs().invDynamics(tau, qd, qdd);
}

/*void Configuration::impulsePropagation(arr& qd1, const arr& qd0){
  static Array<Featherstone::Link> tree;
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  mimickImpulsePropagation(tree);
  Featherstone::RF_abd(qdd, tree, qd, tau);
}*/


/// return a OpenGL extension
shared_ptr<ConfigurationViewer>& Configuration::gl(const char* window_title, bool offscreen) {
  if(!self->viewer) {
    self->viewer = make_shared<ConfigurationViewer>();
  }
  return self->viewer;
}

/// return a Swift extension
std::shared_ptr<SwiftInterface> Configuration::swift() {
  if(self->swift && self->swift->swiftID.N != frames.N) self->swift.reset();
  if(!self->swift){
    self->swift = make_shared<SwiftInterface>(frames, .1, 0);
    self->swift->deactivate(getCollisionExcludeIDs());
    self->swift->deactivatePairs(getCollisionExcludePairIDs());
  }
  return self->swift;
}

std::shared_ptr<FclInterface> Configuration::fcl() {
  if(!self->fcl) {
    Array<ptr<Mesh>> geometries(frames.N);
    for(Frame* f:frames) {
      if(f->shape && f->shape->cont) {
        if(!f->shape->mesh().V.N) f->shape->createMeshes();
        geometries(f->ID) = f->shape->_mesh;
      }
    }
    self->fcl = make_shared<FclInterface>(geometries, .0); //-1.=broadphase only -> many proxies
  }
  return self->fcl;
}

void Configuration::swiftDelete() {
  self->swift.reset();
}

/// return a PhysX extension
PhysXInterface& Configuration::physx() {
  if(!self->physx) {
    self->physx = make_unique<PhysXInterface>(*this);
    //    s->physx->setArticulatedBodiesKinematic();
  }
  return *self->physx;
}

/// return a ODE extension
OdeInterface& Configuration::ode() {
  if(!self->ode) self->ode = make_unique<OdeInterface>(*this);
  return *self->ode;
}

FeatherstoneInterface& Configuration::fs() {
  if(!self->fs) self->fs = make_unique<FeatherstoneInterface>(*this);
  return *self->fs;
}

bool Configuration::hasView(){
  return !!self->viewer;
}

int Configuration::watch(bool pause, const char* txt) {
//  gl()->pressedkey=0;
  int key = gl()->setConfiguration(*this, txt, pause);
//  if(pause) {
//    if(!txt) txt="Config::watch";
//    key = watch(true, txt);
//  } else {
//    key = watch(false, txt, true);
//  }
  return key;
}

void Configuration::glClose() {
  if(self && self->viewer) self->viewer.reset();
}

#if 0
void Configuration::saveVideoPic(uint& t, const char* pathPrefix) {
  write_ppm(gl()->captureImage, STRING(pathPrefix <<std::setw(4)<<std::setfill('0')<<t++<<".ppm"));
}

void Configuration::glAdd(void (*call)(void*, OpenGL&), void* classP) {
  gl()->add(call, classP);
}

int Configuration::glAnimate() {
  return animateConfiguration(*this, nullptr);
}

void Configuration::glGetMasks(int w, int h, bool rgbIndices) {
  if(s->gl && !s->gl->offscreen) {
    LOG(0) <<"can't make this offscreen anymore!";
  } else gl(nullptr, true);

  gl()->clear();
  gl()->addDrawer(this);
  if(rgbIndices) {
    gl()->drawMode_idColor = true;
    gl()->setClearColors(0, 0, 0, 0);
    orsDrawMarkers = orsDrawJoints = orsDrawProxies = false;
  }

  gl()->update(nullptr, true);

  gl()->clear();
  gl()->add(glStandardScene, 0);
  gl()->addDrawer(this);
  if(rgbIndices) {
    gl()->setClearColors(1, 1, 1, 0);
    gl()->drawMode_idColor = false;
    orsDrawMarkers = orsDrawJoints = orsDrawProxies = true;
  }
}
#endif


void Configuration::addProxies(const uintA& collisionPairs) {
  //-- filter the collisions
  boolA filter(collisionPairs.d0);
  uint n=0;
#if 1
  for(uint i=0; i<collisionPairs.d0; i++) {
    bool canCollide = frames.elem(collisionPairs(i, 0))->shape->canCollideWith(frames.elem(collisionPairs(i, 1)));
    filter(i) = canCollide;
    if(canCollide) n++;
  }
#else
  filter = true;
  n = collisionPairs.d0;
#endif
  //-- copy them into proxies
  uint j = proxies.N;
  proxies.resizeCopy(j+n);
  for(uint i=0; i<collisionPairs.d0; i++) {
    if(filter(i)) {
      Proxy& p = proxies(j);
      p.a = frames.elem(collisionPairs(i, 0));
      p.b = frames.elem(collisionPairs(i, 1));
      p.d = -0.;
      p.posA = frames.elem(collisionPairs(i, 0))->getPosition();
      p.posB = frames.elem(collisionPairs(i, 1))->getPosition();
      j++;
    }
  }
}

void Configuration::stepSwift() {
  arr X = getFrameState();
  uintA collisionPairs = swift()->step(X, false);
  //  reportProxies();
  //  watch(true);
  //  gl()->closeWindow();
  proxies.clear();
  addProxies(collisionPairs);

  _state_proxies_isGood=true;
}

void Configuration::stepFcl() {
  //-- get the frame state of collision objects
  arr X = getFrameState();
  //-- step fcl
  fcl()->step(X);
  //-- add as proxies
  proxies.clear();
  addProxies(fcl()->collisions);

  _state_proxies_isGood=true;
}

void Configuration::stepPhysx(double tau) {
  physx().step(tau);
}

void Configuration::stepOde(double tau) {
#ifdef RAI_ODE
  ode().setMotorVel(qdot, 100.);
  ode().step(tau);
  ode().importStateFromOde();
#endif
}

void Configuration::stepDynamics(arr& qdot, const arr& Bu_control, double tau, double dynamicNoise, bool gravity) {

  struct DiffEqn:VectorFunction {
    Configuration& S;
    const arr& Bu;
    bool gravity;
    DiffEqn(Configuration& _S, const arr& _Bu, bool _gravity):S(_S), Bu(_Bu), gravity(_gravity) {
      VectorFunction::operator=([this](arr& y, arr& J, const arr& x) -> void {
        this->fv(y, J, x);
      });
    }
    void fv(arr& y, arr& J, const arr& x) {
      S.setJointState(x[0]);
      arr M, Minv, F;
      S.equationOfMotion(M, F, x[1], gravity);
      inverse_SymPosDef(Minv, M);
      //Minv = inverse(M); //TODO why does symPosDef fail?
      y = Minv * (Bu - F);
    }
  } eqn(*this, Bu_control, gravity);

#if 0
  arr M, Minv, F;
  getDynamics(M, F);
  inverse_SymPosDef(Minv, M);

  //noisy Euler integration (Runge-Kutte4 would be much more precise...)
  qddot = Minv * (u_control - F);
  if(dynamicNoise) rndGauss(qddot, dynamicNoise, true);
  q    += tau * qdot;
  qdot += tau * qddot;
  arr x1=cat(s->q, s->qdot).reshape(2, s->q.N);
#else
  arr x1;
  rk4_2ndOrder(x1, cat(q, qdot).reshape(2, q.N), eqn, tau);
  if(dynamicNoise) rndGauss(x1[1](), ::sqrt(tau)*dynamicNoise, true);
#endif

  setJointState(x1[0]);
  qdot = x1[1];
}

void __merge(ForceExchange* c, Proxy* p) {
  CHECK(&c->a==p->a && &c->b==p->b, "");
  if(!p->collision) p->calc_coll();
  //  c->a_rel = c->a.X / Vector(p->coll->p1);
  //  c->b_rel = c->b.X / Vector(p->coll->p2);
  //  c->a_norm = c->a.X.rot / Vector(-p->coll->normal);
  //  c->b_norm = c->b.X.rot / Vector(p->coll->normal);
  //  c->a_type = c->b_type=1;
  //  c->a_rad = c->a.shape->size(3);
  //  c->b_rad = c->b.shape->size(3);
}

#if 0
double __matchingCost(ForceExchange* c, Proxy* p) {
  double cost=0.;
  if(!p->coll) p->calc_coll(c->a.K);
  cost += sqrDistance(c->a.X * c->a_rel, p->coll->p1);
  cost += sqrDistance(c->b.X * c->b_rel, p->coll->p2);
  //normal costs? Perhaps not, if p does not have a proper normal!
  return cost;
}

void __new(Configuration& K, Proxy* p) {
  ForceExchange* c = new ForceExchange(*p->a, *p->b);
  __merge(c, p);
}

void Configuration::filterProxiesToContacts(double margin) {
  for(Proxy& p:proxies) {
    if(!p.coll) p.calc_coll(*this);
    if(p.coll->distance-(p.coll->rad1+p.coll->rad2)>margin) continue;
    ForceExchange* candidate=nullptr;
    double candidateMatchingCost=0.;
    for(ForceExchange* c:p.a->forces) {
      if((&c->a==p.a && &c->b==p.b) || (&c->a==p.b && &c->b==p.a)) {
        double cost = __matchingCost(c, &p);
        if(!candidate || cost<candidateMatchingCost) {
          candidate = c;
          candidateMatchingCost = cost;
        }
        //in any case: adapt the normals:
        c->a_norm = c->a.X.rot / Vector(-p.coll->normal);
        c->b_norm = c->b.X.rot / Vector(p.coll->normal);
      }
    }
    if(candidate && ::sqrt(candidateMatchingCost)<.05) { //cost is roughly measured in sqr-meters
      __merge(candidate, &p);
    } else {
      __new(*this, &p);
    }
  }
  //phase 2: cleanup old and distant contacts
  Array<ForceExchange*> old;
  for(Frame* f:frames) for(ForceExchange* c:f->forces) if(&c->a==f) {
        if(/*c->get_pDistance()>margin+.05 ||*/ c->getDistance()>margin) old.append(c);
      }
  for(ForceExchange* c:old) delete c;
}
#endif

void Configuration::copyProxies(const ProxyA& _proxies) {
  proxies.clear();
  proxies.resize(_proxies.N);
  for(uint i=0; i<proxies.N; i++) proxies(i).copy(*this, _proxies(i));
}

/// prototype for \c operator<<
void Configuration::write(std::ostream& os) const {
  for(Frame* f: frames) if(!f->name.N) f->name <<'_' <<f->ID;
  for(Frame* f: frames) { //fwdActiveSet) {
    //    os <<"frame " <<f->name;
    //    if(f->parent) os <<'(' <<f->parent->name <<')';
    //    os <<" \t{ ";
    f->write(os);
    //    os <<" }\n";
  }
  os <<std::endl;
  //  for(Frame *f: frames) if(f->shape){
  //    os <<"shape ";
  //    os <<"(" <<f->name <<"){ ";
  //    f->shape->write(os);  os <<" }\n";
  //  }
  //  os <<std::endl;
  //  for(Frame *f: fwdActiveSet) if(f->parent) {
  //    if(f->joint){
  //      os <<"joint ";
  //      os <<"(" <<f->parent->name <<' ' <<f->name <<"){ ";
  //      f->joint->write(os);  os <<" }\n";
  //    }else{
  //      os <<"link ";
  //      os <<"(" <<f->parent->name <<' ' <<f->name <<"){ ";
  //      f->write(os);  os <<" }\n";
  //    }
  //  }
}

void Configuration::write(Graph& G) const {
  for(Frame* f: frames) if(!f->name.N) f->name <<'_' <<f->ID;
  for(Frame* f: frames) f->write(G.newSubgraph({f->name}));
}

/// write a URDF file
void Configuration::writeURDF(std::ostream& os, const char* robotName) const {
  os <<"<?xml version=\"1.0\"?>\n";
  os <<"<robot name=\"" <<robotName <<"\">\n";

  //-- write base_link first

  FrameL bases;
  for(Frame* a:frames) { if(!a->parent) a->getRigidSubFrames(bases); }
  os <<"<link name=\"base_link\">\n";
  for(Frame* a:frames) {
    if(a->shape && a->shape->type()!=ST_mesh && a->shape->type()!=ST_marker) {
      os <<"  <visual>\n    <geometry>\n";
      arr& size = a->shape->size();
      switch(a->shape->type()) {
        case ST_box:       os <<"      <box size=\"" <<size({0, 2}) <<"\" />\n";  break;
        case ST_cylinder:  os <<"      <cylinder length=\"" <<size.elem(-2) <<"\" radius=\"" <<size.elem(-1) <<"\" />\n";  break;
        case ST_sphere:    os <<"      <sphere radius=\"" <<size.last() <<"\" />\n";  break;
        case ST_mesh:      os <<"      <mesh filename=\"" <<a->ats.get<FileToken>("mesh").name <<'"';
          if(a->ats["meshscale"]) os <<" scale=\"" <<a->ats.get<arr>("meshscale") <<'"';
          os <<" />\n";  break;
        default:           os <<"      <UNKNOWN_" <<a->shape->type() <<" />\n";  break;
      }
      os <<"      <material> <color rgba=\"" <<a->shape->mesh().C <<"\" /> </material>\n";
      os <<"    </geometry>\n";
      //      os <<"  <origin xyz=\"" <<a->Q.pos.x <<' ' <<a->Q.pos.y <<' ' <<a->Q.pos.z <<"\" />\n";
      os <<"  <inertial>  <mass value=\"1\"/>  </inertial>\n";
      os <<"  </visual>\n";
    }
  }
  os <<"</link>" <<endl;

  for(Frame* a:frames) if(a->joint) {
      os <<"<link name=\"" <<a->name <<"\">\n";

      FrameL shapes;
      a->getRigidSubFrames(shapes);
      for(Frame* b:shapes) {
        if(b->shape && b->shape->type()!=ST_mesh && b->shape->type()!=ST_marker) {
          os <<"  <visual>\n    <geometry>\n";
          arr& size = b->shape->size();
          switch(b->shape->type()) {
            case ST_box:       os <<"      <box size=\"" <<size({0, 2}) <<"\" />\n";  break;
            case ST_cylinder:  os <<"      <cylinder length=\"" <<size.elem(-2) <<"\" radius=\"" <<size.elem(-1) <<"\" />\n";  break;
            case ST_sphere:    os <<"      <sphere radius=\"" <<size.last() <<"\" />\n";  break;
            case ST_mesh:      os <<"      <mesh filename=\"" <<b->ats.get<FileToken>("mesh").name <<'"';
              if(b->ats["meshscale"]) os <<" scale=\"" <<b->ats.get<arr>("meshscale") <<'"';
              os <<" />\n";  break;
            default:           os <<"      <UNKNOWN_" <<b->shape->type() <<" />\n";  break;
          }
          os <<"      <material> <color rgba=\"" <<b->shape->mesh().C <<"\" /> </material>\n";
          os <<"    </geometry>\n";
          os <<"  <origin xyz=\"" <<b->get_Q().pos.getArr() <<"\" rpy=\"" <<b->get_Q().rot.getEulerRPY() <<"\" />\n";
          os <<"  <inertial>  <mass value=\"1\"/>  </inertial>\n";
          os <<"  </visual>\n";
        }
      }
      os <<"</link>" <<endl;

      os <<"<joint name=\"" <<a->name <<"\" type=\"fixed\" >\n";
      Transformation Q=0;
      Frame* p=a->parent;
      while(p && !p->joint) { Q=p->get_Q()*Q; p=p->parent; }
      if(!p)    os <<"  <parent link=\"base_link\"/>\n";
      else      os <<"  <parent link=\"" <<p->name <<"\"/>\n";
      os <<"  <child  link=\"" <<a->name <<"\"/>\n";
      os <<"  <origin xyz=\"" <<Q.pos.getArr() <<"\" rpy=\"" <<Q.rot.getEulerRPY() <<"\" />\n";
      os <<"</joint>" <<endl;
    }

  os <<"</robot>";
}

#ifdef RAI_ASSIMP

/// write a 3D model file using the assimp library (esp collada to include the tree structure)
void Configuration::writeCollada(const char* filename, const char* format) const {
  // get mesh frames
  // create a new scene
  aiScene scene;
  scene.mRootNode = new aiNode("root");
  // create a dummy material
  scene.mMaterials = new aiMaterial *[2];
  scene.mNumMaterials = 2;
  scene.mMaterials[0] = new aiMaterial();
  scene.mMaterials[1] = new aiMaterial();
  float op = 0.5;
  scene.mMaterials[1]->AddProperty(&op, 1, AI_MATKEY_OPACITY);
  // create meshes
  //count meshes
  uint n_meshes=0;
  for(Frame* f:frames) if(f->shape && f->shape->type()!=ST_marker) n_meshes++;
  scene.mMeshes = new aiMesh *[n_meshes];
  scene.mNumMeshes = n_meshes;
  // create all nodes
  n_meshes=0;
  arr T = eye(4);
  Array<aiNode*> nodes(frames.N);
  for(Frame *f:frames) {
    aiNode* node = new aiNode(f->name.p);
    nodes(f->ID) = node;
    // create a mesh?
    if(f->shape && f->shape->type()!=ST_marker){
      aiMesh* mesh = scene.mMeshes[n_meshes] = new aiMesh();
      Mesh& M = f->shape->mesh();
      buildAiMesh(M, mesh);
      double alpha = f->shape->alpha();
      if(alpha==1.)
        mesh->mMaterialIndex = 0;
      else
        mesh->mMaterialIndex = 1;
      //associate with node
      node->mMeshes = new unsigned[1];
      node->mNumMeshes = 1;
      node->mMeshes[0] = n_meshes;
      n_meshes++;
    }else{
      node->mMeshes = 0;
      node->mNumMeshes = 0;
    }
    f->get_Q().getAffineMatrix(T.p);
    for(uint j=0; j<4; j++) for(uint k=0; k<4; k++) {
      node->mTransformation[j][k] = T(j, k);
    }
  }
  //connect parent/children
  //count roots
  uint n_roots=0;
  for(Frame *f:frames) if(!f->parent) n_roots++;
  scene.mRootNode->mChildren = new aiNode* [n_roots];
  scene.mRootNode->mNumChildren = n_roots;
  n_roots=0;
  for(Frame *f:frames) {
    aiNode* node = nodes(f->ID);
    if(f->parent){
      node->mParent = nodes(f->parent->ID);
    }else{
      node->mParent = scene.mRootNode;
      scene.mRootNode->mChildren[n_roots++] = node;
    }
    node->mChildren = new aiNode* [f->children.N];
    node->mNumChildren = f->children.N;
    for(uint i=0;i<f->children.N;i++){
      node->mChildren[i] = nodes(f->children(i)->ID);
    }
  }
  // export
  Assimp::Exporter exporter;
  exporter.Export(&scene, format, filename);
}
#else
void Configuration::writeCollada(const char* filename, const char* format) const {
  NICO
}
#endif //ASSIMP

/// write meshes into an own binary array file format
void Configuration::writeMeshes(const char* pathPrefix) const {
  for(Frame* f:frames) {
    if(f->shape &&
        (f->shape->type()==ST_mesh || f->shape->type()==ST_ssCvx)) {
      String filename = pathPrefix;
#if 0
      filename <<f->name <<".arr";
      f->ats.getNew<String>("mesh") = filename;
      if(f->shape->type()==ST_mesh) f->shape->mesh().writeArr(FILE(filename));
      if(f->shape->type()==ST_ssCvx) f->shape->sscCore().writeArr(FILE(filename));
#else
      filename <<f->name <<".ply";
      f->ats.getNew<FileToken>("mesh").name = filename;
      if(f->shape->type()==ST_mesh) f->shape->mesh().writePLY(filename.p);
      if(f->shape->type()==ST_ssCvx) f->shape->sscCore().writePLY(filename.p);
#endif
    }
  }
}

/// prototype for \c operator>>
void Configuration::read(std::istream& is) {
  Graph G(is);
  G.checkConsistency();
  //  cout <<"***KVG:\n" <<G <<endl;
  //  FILE("z.G") <<G;
  readFromGraph(G);
}

Graph Configuration::getGraph() const {
#if 1
  Graph G;
  //first just create nodes
  for(Frame* f: frames) G.newNode<bool>({STRING(f->name <<" [" <<f->ID <<']')}, {});
  for(Frame* f: frames) {
    Node* n = G.elem(f->ID);
    if(f->parent) {
      n->addParent(G.elem(f->parent->ID));
      n->key = STRING("Q= " <<f->get_Q());
    }
    if(f->joint) {
      n->key = (STRING("joint " <<f->joint->type));
    }
    if(f->shape) {
      n->key = (STRING("shape " <<f->shape->type()));
    }
    if(f->inertia) {
      n->key = (STRING("inertia m=" <<f->inertia->mass));
    }
  }
#else
  Graph G;
  //first just create nodes
  for(Frame* f: frames) G.newSubgraph({f->name}, {});

  for(Frame* f: frames) {
    Graph& ats = G.elem(f->ID)->graph();

    ats.newNode<Transformation>({"X"}, {}, f->X);

    if(f->shape) {
      ats.newNode<int>({"shape"}, {}, f->shape->type);
    }

    if(f->link) {
      G.elem(f->ID)->addParent(G.elem(f->link->from->ID));
      if(f->link->joint) {
        ats.newNode<int>({"joint"}, {}, f->joint()->type);
      } else {
        ats.newNode<Transformation>({"Q"}, {}, f->link->Q);
      }
    }
  }
#endif
  G.checkConsistency();
  return G;
}

struct Link {
  Frame* joint=nullptr;
  FrameL frames;
  Frame* from() {
    Frame* a = joint->parent;
    while(a && !a->joint) a=a->parent;
    return a;
  }
};

//Array<Link *> Configuration::getLinks(){
//  Array<Link*> links;

//  FrameL bases;
//  for(Frame *a:frames){ if(!a->parent) a->getRigidSubFrames(bases); }
//  Link *l = links.append(new Link);
//  l->frames = bases;

//  for(Frame *a:frames) if(a->joint){
//    Link *l = links.append(new Link);
//    l->joint = a;
//    a->getRigidSubFrames(l->frames);
//  }
//  return links;
//}

void Configuration::displayDot() {
  Graph G = getGraph();
  G.displayDot();
}

void Configuration::report(std::ostream& os) const {
  uint nShapes=0, nUc=0;
  for(Frame* f:frames) if(f->shape) nShapes++;
  for(Joint* j:activeJoints) if(j->uncertainty) nUc++;

  os <<"Config: q.N=" <<getJointStateDimension()
     <<" #frames=" <<frames.N
     <<" #joints=" <<activeJoints.N
     <<" #shapes=" <<nShapes
     <<" #ucertainties=" <<nUc
     <<" #proxies=" <<proxies.N
     <<" #forces=" <<forces.N
     <<" #evals=" <<setJointStateCount
     <<endl;

//  os <<" limits=" <<getLimits() <<endl;
  //  os <<" joints=" <<getJointNames() <<endl;
}

void Configuration::readFromGraph(const Graph& G, bool addInsteadOfClear) {
  if(!addInsteadOfClear) clear();

  FrameL node2frame(G.N);
  node2frame.setZero();

  //all the special cases with %body %joint %shape is only for backward compatibility. New: just frames

  NodeL bs = G.getNodesWithTag("%body");
  for(Node* n:  bs) {
    CHECK(n->isGraph(), "bodies must have value Graph");
    CHECK(n->graph().findNode("%body"), "");

    Frame* b=new Frame(*this);
    node2frame(n->index) = b;
    b->name=n->key;
    b->ats.copy(n->graph(), false, true);
    b->read(b->ats);
  }

  for(Node* n: G) {
    CHECK(n->isGraph(), "frame must have value Graph");
    if(n->graph().findNode("%body") || n->graph().findNode("%shape") || n->graph().findNode("%joint")
        || n->key=="joint" || n->key=="shape") continue;
    //    CHECK_EQ(n->keys(0),"frame","");
    CHECK_LE(n->parents.N, 2, "frames must have no or one parent: specs=" <<*n <<' ' <<n->index);

    if(n->parents.N<=1) { //normal frame

      Frame* b = nullptr;
      if(!n->parents.N) b = new Frame(*this);
      else if(n->parents.N==1) b = new Frame(node2frame(n->parents(0)->index)); //getFrameByName(n->parents(0)->key));
      else HALT("a frame can only have one parent");
      node2frame(n->index) = b;
      b->name=n->key;
      b->ats.copy(n->graph(), false, true);
      b->read(b->ats);

    } else { //this is an inserted joint -> 2 frames (pre-joint and joint)

      Frame* from = node2frame(n->parents(0)->index);
      Frame* to   = node2frame(n->parents(1)->index);
      CHECK(from, "JOINT: from '" <<n->parents(0)->key <<"' does not exist ["<<*n <<"]");
      CHECK(to, "JOINT: to '" <<n->parents(1)->key <<"' does not exist ["<<*n <<"]");

      //generate a pre node
      Frame* pre = from;
      if(n->graph().findNode("A")) {
        pre = new Frame(from);
        pre->name = n->key;
        pre->name <<"_pre";
        pre->set_Q()->read(n->graph().get<String>("A"));
        n->graph().delNode(n->graph().findNode("A"));
        n->graph().index();
      }

      //generate a frame, as below
      Frame* b = new Frame(pre);
      node2frame(n->index) = b;
      b->name=n->key;

      //connect the post node and impose the post node relative transform
      to->linkFrom(b, false);
      if(n->graph().findNode("B")) {
        to->set_Q()->read(n->graph().get<String>("B"));
        n->graph().delNode(n->graph().findNode("B"));
        n->graph().index();
      }

      b->ats.copy(n->graph(), false, true);
      b->read(b->ats);

    }
  }

  NodeL ss = G.getNodesWithTag("%shape");
  ss.append(G.getNodes("shape"));
  for(Node* n: ss) {
    CHECK_LE(n->parents.N, 1, "shapes must have no or one parent");
    CHECK(n->isGraph(), "shape must have value Graph");
    CHECK(n->key=="shape" || n->graph().findNode("%shape"), "");

    Frame* f = new Frame(*this);
    f->name=n->key;
    f->ats.copy(n->graph(), false, true);
    Shape* s = new Shape(*f);
    s->read(f->ats);

    if(n->parents.N==1) {
      Frame* b = listFindByName(frames, n->parents(0)->key);
      CHECK(b, "could not find frame '" <<n->parents(0)->key <<"'");
      f->linkFrom(b);
      if(f->ats["rel"]) n->graph().get(f->Q, "rel");
    }
  }

  NodeL js = G.getNodesWithTag("%joint");
  js.append(G.getNodes("joint"));
  for(Node* n: js) {
    CHECK_EQ(n->parents.N, 2, "joints must have two parents: specs=" <<*n <<' ' <<n->index);
    CHECK(n->isGraph(), "joints must have value Graph: specs=" <<*n <<' ' <<n->index);
    CHECK(n->key=="joint" || n->graph().findNode("%joint"), "");

    Frame* from=listFindByName(frames, n->parents(0)->key);
    Frame* to=listFindByName(frames, n->parents(1)->key);
    CHECK(from, "JOINT: from '" <<n->parents(0)->key <<"' does not exist ["<<*n <<"]");
    CHECK(to, "JOINT: to '" <<n->parents(1)->key <<"' does not exist ["<<*n <<"]");

    Frame* f=new Frame(*this);
    if(n->key.N && n->key!="joint") {
      f->name=n->key;
    } else {
      f->name <<'|' <<to->name; //the joint frame is actually the link frame of all child frames
    }
    f->ats.copy(n->graph(), false, true);

    f->linkFrom(from);
    to->linkFrom(f);

    Joint* j=new Joint(*f);
    j->read(f->ats);
  }

  //if the joint is coupled to another:
  {
    Joint* j;
    for(Frame* f: frames) if((j=f->joint) && j->mimic==(Joint*)1) {
        Node* mim = f->ats["mimic"];
        String jointName;
        if(mim->isOfType<String>()) jointName = mim->get<String>();
        else if(mim->isOfType<NodeL>()) {
          NodeL nodes = mim->get<NodeL>();
          jointName = nodes.scalar()->key;
        } else {
          HALT("could not retrieve minimick frame for joint '" <<f->name <<"' from ats '" <<f->ats <<"'");
        }
        Frame* mimicFrame = getFrame(jointName, true, true);
        CHECK(mimicFrame, "the argument to 'mimic', '" <<jointName <<"' is not a frame name");
        j->mimic=0; //UNDO the =(Joint*)1
        j->setMimic( mimicFrame->joint );
        if(!j->mimic) HALT("The joint '" <<*j <<"' is declared mimicking '" <<jointName <<"' -- but that doesn't exist!");
        j->type = j->mimic->type;
        j->q0 = j->mimic->q0;
        j->calc_Q_from_q(j->q0, 0);

        delete mim;
        f->ats.index();
      }
  }

  NodeL ucs = G.getNodesWithTag("%Uncertainty");
  ucs.append(G.getNodes("Uncertainty"));
  for(Node* n: ucs) {
    CHECK_EQ(n->parents.N, 1, "Uncertainties must have one parent");
    CHECK(n->isGraph(), "Uncertainties must have value Graph");
    CHECK(n->key=="Uncertainty" || n->graph().findNode("%Uncertainty"), "");

    Frame* f = getFrame(n->parents(0)->key);
    CHECK(f, "");
    Joint* j = f->joint;
    CHECK(j, "Uncertainty parent must be a joint");
    Uncertainty* uc = new Uncertainty(j);
    uc->read(n->graph());
  }

  //-- clean up the graph
//  calc_q();
//  calc_fwdPropagateFrames();
  checkConsistency();
}

/// dump the list of current proximities on the screen
void Configuration::reportProxies(std::ostream& os, double belowMargin, bool brief) const {
  CHECK(_state_proxies_isGood, "");

  os <<"Proximity report: #" <<proxies.N <<endl;
  uint i=0;
  for(const Proxy& p: proxies) {
    if(p.d>belowMargin) continue;
    os  <<i;
    p.write(os, brief);
    os <<endl;
    i++;
  }
  os <<"ForceExchange report:" <<endl;
  for(Frame* a:frames) for(ForceExchange* c:a->forces){
    if(&c->a==a) {
      c->coll();
      os <<*c <<endl;
    }
  }
}

bool ProxySortComp(const Proxy* a, const Proxy* b) {
  return (a->a < b->a) || (a->a==b->a && a->b<b->b) || (a->a==b->a && a->b==b->b && a->d < b->d);
}

void Configuration::kinematicsPenetration(arr& y, arr& J, const Proxy& p, double margin, bool addValues) const {
  CHECK(p.a->shape, "");
  CHECK(p.b->shape, "");

  //early check: if swift is way out of collision, don't bother computing it precise
  if(p.d > p.a->shape->radius() + p.b->shape->radius() + .01 + margin) return;

  if(!p.collision)((Proxy*)&p)->calc_coll();

  if(p.collision->getDistance()>margin) return;

  arr Jp1, Jp2;
  jacobian_pos(Jp1, p.a, p.collision->p1);
  jacobian_pos(Jp2, p.b, p.collision->p2);

  arr y_dist, J_dist;
  p.collision->kinDistance(y_dist, J_dist, Jp1, Jp2);

  if(y_dist.scalar()>margin) return;
  if(addValues){
    y += margin-y_dist.scalar();
    J -= J_dist;
  }else{
    y = margin-y_dist.scalar();
    J = J_dist;
  }
}

/// measure (=scalar kinematics) for the contact cost summed over all bodies
void Configuration::kinematicsPenetration(arr& y, arr& J, double margin) const {
  CHECK(_state_proxies_isGood, "");

  y.resize(1).setZero();
  jacobian_zero(J, 1);
  for(const Proxy& p:proxies) { /*if(p.d<margin)*/
    kinematicsPenetration(y, J, p, margin, true);
  }
}

/// set the jacMode flag to match with the type of the given J
void Configuration::setJacModeAs(const arr& J){
  if(!isSpecial(J)) jacMode = JM_dense;
  else if(J.isSparse()) jacMode = JM_sparse;
  else if(isNoArr(J)) jacMode = JM_noArr;
  else if(isEmptyShape(J)) jacMode = JM_emptyShape;
  else NIY;
}

/// return a feature for a given frame(s)
std::shared_ptr<Feature> Configuration::feature(FeatureSymbol fs, const StringA& frames) const {
  return symbols2feature(fs, frames, *this);
}

/// evaluate a feature for a given frame(s)
void Configuration::evalFeature(arr& y, arr& J, FeatureSymbol fs, const StringA& frames) const {
  feature(fs, frames)->__phi(y, J, *this);
}

/// Compute the new configuration q such that body is located at ytarget (with deplacement rel).
void Configuration::inverseKinematicsPos(Frame& frame, const arr& ytarget,
    const Vector& rel_offset, int max_iter) {
  arr q0 = getJointState();
  arr q = q0;
  arr y; // endeff pos
  arr J; // Jacobian
  arr invJ;
  arr I = eye(q.N);

  // general inverse kinematic update
  // first iteration: $q* = q' + J^# (y* - y')$
  // next iterations: $q* = q' + J^# (y* - y') + (I - J# J)(q0 - q')$
  for(int i = 0; i < max_iter; i++) {
    kinematicsPos(y, J, &frame, rel_offset);
    invJ = ~J * inverse(J * ~J);  // inverse_SymPosDef should work!?
    q = q + invJ * (ytarget - y);

    if(i > 0) {
      q += (I - invJ * J) * (q0 - q);
    }
    setJointState(q);
  }
}

#if 0
/// center of mass of the whole configuration (3 vector)
double Configuration::getCenterOfMass(arr& x_) const {
  double M=0.;
  Vector x;
  x.setZero();
  for(Frame* f: frames) if(f->inertia) {
      M += f->inertia->mass;
      x += f->inertia->mass*f->X.pos;
    }
  x /= M;
  x_ = conv_vec2arr(x);
  return M;
}

/// gradient (Jacobian) of the COM w.r.t. q (3 x n tensor)
void Configuration::getComGradient(arr& grad) const {
  double M=0.;
  arr J(3, getJointStateDimension());
  grad.resizeAs(J); grad.setZero();
  for(Frame* f: frames) if(f->inertia) {
      M += f->inertia->mass;
      kinematicsPos(NoArr, J, f);
      grad += f->inertia->mass * J;
    }
  grad/=M;
}

const Proxy* Configuration::getContact(uint a, uint b) const {
  for(const Proxy& p: proxies) if(p.d<0.) {
      if(p.a->ID==a && p.b->ID==b) return &p;
      if(p.a->ID==b && p.b->ID==a) return &p;
    }
  return nullptr;
}

#endif



void Configuration::glDraw(OpenGL& gl) {
  glDraw_sub(gl, frames);

  bool displayUncertainties = false;
  for(Joint* j:activeJoints) if(j->uncertainty) {
      displayUncertainties=true; break;
    }

  if(displayUncertainties) {
    arr q_org = getJointState();
    for(Joint* j:activeJoints) if(j->uncertainty) {
        for(uint i=0; i<j->qDim(); i++) {
          arr q=q_org;
          q(j->qIndex+i) -= j->uncertainty->sigma(i);
          setJointState(q);
          glDraw_sub(gl, frames);
          q=q_org;
          q(j->qIndex+i) += j->uncertainty->sigma(i);
          setJointState(q);
          glDraw_sub(gl, frames);
        }
      }
    setJointState(q_org);
  }
}

/// GL routine to draw a Configuration
void Configuration::glDraw_sub(OpenGL& gl, const FrameL& F, int drawOpaqueOrTransparanet) {
#ifdef RAI_GL
  Transformation f;
  double GLmatrix[16];

  glPushMatrix();

  glColor(.5, .5, .5);

  if(drawOpaqueOrTransparanet!=2) {
    if(orsDrawVisualsOnly) {
      orsDrawProxies=orsDrawJoints=orsDrawMarkers=false;
    }

    //proxies
//    if(orsDrawProxies) for(const Proxy& p: proxies) {
//        ((Proxy*)&p)->glDraw(gl);
//      }

    for(Frame* fr: F) for(ForceExchange* f:fr->forces) {
      if(f->sign(fr)>0.) f->glDraw(gl);
    }

    //joints
    Joint* e;
    if(orsDrawJoints) for(Frame* fr: F) if((e=fr->joint)) {
          //set name (for OpenGL selection)
          glPushName((fr->ID <<2) | 2);

          //    double s=e->A.pos.length()+e->B.pos.length(); //some scale
          double s=.1;

          //    //from body to joint
          //    f=e->from->X;
          //    f.getAffineMatrixGL(GLmatrix);
          //    glLoadMatrixd(GLmatrix);
          //    glColor(1, 1, 0);
          //    //glDrawSphere(.1*s);
          //    glBegin(GL_LINES);
          //    glVertex3f(0, 0, 0);
          //    glVertex3f(e->A.pos.x, e->A.pos.y, e->A.pos.z);
          //    glEnd();

          //joint frame A
          //    f.appendTransformation(e->A);
          f.getAffineMatrixGL(GLmatrix);
          glLoadMatrixd(GLmatrix);
          glDrawAxes(s);
          glColor(1, 0, 0);
          glRotatef(90, 0, 1, 0);  glDrawCylinder(.05*s, .3*s);  glRotatef(-90, 0, 1, 0);

          //joint frame B
          f.appendTransformation(fr->get_Q());
          f.getAffineMatrixGL(GLmatrix);
          glLoadMatrixd(GLmatrix);
          glDrawAxes(s);

          //    //from joint to body
          //    glColor(1, 0, 1);
          //    glBegin(GL_LINES);
          //    glVertex3f(0, 0, 0);
          //    glVertex3f(e->B.pos.x, e->B.pos.y, e->B.pos.z);
          //    glEnd();
          //    glTranslatef(e->B.pos.x, e->B.pos.y, e->B.pos.z);
          //    //glDrawSphere(.1*s);

          glPopName();
        }
  }

  //shapes
  if(orsDrawBodies) {
    if(drawOpaqueOrTransparanet==0 || drawOpaqueOrTransparanet==1) {
      //first non-transparent
      for(Frame* f: F) if(f->shape && f->shape->alpha()==1.) {
          gl.drawId(f->ID);
          f->shape->glDraw(gl);
        }
    }
    if(drawOpaqueOrTransparanet==0 || drawOpaqueOrTransparanet==2) {
      for(Frame* f: F) if(f->shape && f->shape->alpha()<1.) {
          gl.drawId(f->ID);
          f->shape->glDraw(gl);
        }
    }
  }

  glPopMatrix();
#endif
}

//===========================================================================

void kinVelocity(arr& y, arr& J, uint frameId, const ConfigurationL& Ktuple, double tau) {
  CHECK_GE(Ktuple.N, 1, "");
  Configuration& K0 = *Ktuple(-2);
  Configuration& K1 = *Ktuple(-1);
  Frame* f0 = K0.frames.elem(frameId);
  Frame* f1 = K1.frames.elem(frameId);

  arr y0, J0;
  K0.kinematicsPos(y0, J0, f0);
  K1.kinematicsPos(y, J, f1);
  y -= y0;
  J -= J0;
  y /= tau;
  J /= tau;
}

//===========================================================================
//
// helper routines -- in a classical C interface
//

#undef LEN

double forceClosureFromProxies(Configuration& K, uint frameIndex, double distanceThreshold, double mu, double torqueWeights) {
  Vector c, cn;
  arr C, Cn;
  for(const Proxy& p: K.proxies) {
    int body_a = p.a?p.a->ID:-1;
    int body_b = p.b?p.b->ID:-1;
    if(p.d<distanceThreshold && (body_a==(int)frameIndex || body_b==(int)frameIndex)) {
      if(body_a==(int)frameIndex) {
        c = p.posA;
        cn=-p.normal;
      } else {
        c = p.posB;
        cn= p.normal;
      }
      C.append(conv_vec2arr(c));
      Cn.append(conv_vec2arr(cn));
    }
  }
  C .reshape(C.N/3, 3);
  Cn.reshape(C.N/3, 3);
  double fc=forceClosure(C, Cn, K.frames.elem(frameIndex)->ensure_X().pos, mu, torqueWeights, nullptr);
  return fc;
}


//===========================================================================
//===========================================================================
// opengl
//===========================================================================
//===========================================================================

void displayState(const arr& x, Configuration& G, const char* tag) {
  G.setJointState(x);
  G.watch(true, tag);
}

void displayTrajectory(const arr& _x, int steps, Configuration& G, const KinematicSwitchL& switches, const char* tag, double delay, uint dim_z, bool copyG) {
  NIY;
#if 0
  if(!steps) return;
  Shape* s;
  for(Frame* f : G.frames) if((s=f->shape)) {
      if(s->mesh.V.d0!=s->mesh.Vn.d0 || s->mesh.T.d0!=s->mesh.Tn.d0) {
        s->mesh.computeNormals();
      }
    }
  Configuration* Gcopy;
  if(switches.N) copyG=true;
  if(!copyG) Gcopy=&G;
  else {
    Gcopy = new Configuration;
    Gcopy->copy(G, true);
  }
  arr x, z;
  if(dim_z) {
    x.referToRange(_x, 0, -dim_z-1);
    z.referToRange(_x, -dim_z, -1);
  } else {
    x.referTo(_x);
  }
  uint n=Gcopy->getJointStateDimension()-dim_z;
  x.reshape(x.N/n, n);
  uint num, T=x.d0-1;
  if(steps==1 || steps==-1) num=T; else num=steps;
  for(uint k=0; k<=(uint)num; k++) {
    uint t = (T?(k*T/num):0);
    if(switches.N) {
      for(KinematicSwitch* sw: switches)
        if(sw->timeOfApplication==t)
          sw->apply(*Gcopy);
    }
    if(dim_z) Gcopy->setJointState(cat(x[t], z));
    else Gcopy->setJointState(x[t]);
    if(delay<0.) {
      if(delay<-10.) FILE("z.graph") <<*Gcopy;
      Gcopy->watch(true, STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
    } else {
      Gcopy->watch(false, STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
      if(delay) wait(delay);
    }
  }
  if(steps==1)
    Gcopy->watch(true, STRING(tag <<" (time " <<std::setw(3) <<T <<'/' <<T <<')').p);
  if(copyG) delete Gcopy;
#endif
}

/* please don't remove yet: code for displaying edges might be useful...

void glDrawOdeWorld(void *classP){
  _glDrawOdeWorld((dWorldID)classP);
}

void _glDrawOdeWorld(dWorldID world)
{
  glStandardLight();
  glColor(3);
  glDrawFloor(4);
  uint i;
  Color c;
  dVector3 vec, vec2;
  dBodyID b;
  dGeomID g, gg;
  dJointID j;
  dReal a, al, ah, r, len;
  glPushName(0);
  int t;

  //bodies
  for(i=0, b=world->firstbody;b;b=(dxBody*)b->next){
    i++;
    glPushName(i);

    //if(b->userdata){ glDrawBody(b->userdata); }
    c.setIndex(i); glColor(c.r, c.g, c.b);
    glShadeModel(GL_FLAT);

    //bodies
    for(g=b->geom;g;g=dGeomGetBodyNext(g)){
      if(dGeomGetClass(g)==dGeomTransformClass){
  ((dxGeomTransform*)g)->computeFinalTx();
        glTransform(((dxGeomTransform*)g)->final_pos, ((dxGeomTransform*)g)->final_R);
  gg=dGeomTransformGetGeom(g);
      }else{
  glTransform(g->pos, g->R);
  gg=g;
      }
      b = dGeomGetBody(gg);
      // set the color of the body, 4. Mar 06 (hh)
      c.r = ((Body*)b->userdata)->cr;
      c.g = ((Body*)b->userdata)->cg;
      c.b = ((Body*)b->userdata)->cb;
      glColor(c.r, c.g, c.b);

      switch(dGeomGetClass(gg))
  {
  case dSphereClass:
    glDrawSphere(dGeomSphereGetRadius(gg));
    break;
  case dBoxClass:
    dGeomBoxGetLengths(gg, vec);
    glDrawBox(vec[0], vec[1], vec[2]);
    break;
  case dCCylinderClass: // 6. Mar 06 (hh)
    dGeomCCylinderGetParams(gg, &r, &len);
    glDrawCappedCylinder(r, len);
    break;
  default: HALT("can't draw that geom yet");
  }
      glPopMatrix();
    }

    // removed shadows,  4. Mar 06 (hh)

    // joints

      dxJointNode *n;
      for(n=b->firstjoint;n;n=n->next){
      j=n->joint;
      t=dJointGetType(j);
      if(t==dJointTypeHinge){
      dJointGetHingeAnchor(j, vec);
      a=dJointGetHingeAngle(j);
      al=dJointGetHingeParam(j, dParamLoStop);
      ah=dJointGetHingeParam(j, dParamHiStop);
      glPushMatrix();
      glTranslatef(vec[0], vec[1], vec[2]);
      dJointGetHingeAxis(j, vec);
      glBegin(GL_LINES);
      glColor3f(1, 0, 0);
      glVertex3f(0, 0, 0);
      glVertex3f(LEN*vec[0], LEN*vec[1], LEN*vec[2]);
      glEnd();
      //glDrawText(STRING(al <<'<' <<a <<'<' <<ah), LEN*vec[0], LEN*vec[1], LEN*vec[2]);
      glPopMatrix();
      }
      if(t==dJointTypeAMotor){
  glPushMatrix();
  glTranslatef(b->pos[0], b->pos[1], b->pos[2]);
  dJointGetAMotorAxis(j, 0, vec);
  glBegin(GL_LINES);
  glColor3f(1, 1, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(LEN*vec[0], LEN*vec[1], LEN*vec[2]);
  glEnd();
  glPopMatrix();
      }
      if(t==dJointTypeBall){
  dJointGetBallAnchor(j, vec);
  dJointGetBallAnchor2(j, vec2);
  glPushMatrix();
  glTranslatef(vec[0], vec[1], vec[2]);
  glBegin(GL_LINES);
  glColor3f(1, 0, 0);
  glVertex3f(-.05, 0, 0);
  glVertex3f(.05, 0, 0);
  glVertex3f(0, -.05, 0);
  glVertex3f(0, .05, 0);
  glVertex3f(0, 0, -.05);
  glVertex3f(0, 0, .05);
  glEnd();
  glPopMatrix();
  glPushMatrix();
  glTranslatef(vec2[0], vec2[1], vec2[2]);
  glBegin(GL_LINES);
  glColor3f(1, 0, 0);
  glVertex3f(-.05, 0, 0);
  glVertex3f(.05, 0, 0);
  glVertex3f(0, -.05, 0);
  glVertex3f(0, .05, 0);
  glVertex3f(0, 0, -.05);
  glVertex3f(0, 0, .05);
  glEnd();
  glPopMatrix();
      }
    }
      glPopName();
  }
  glPopName();
}
*/

int animateConfiguration(Configuration& C, Inotify* ino) {
  arr x, x0;
  x0 = C.getJointState();
  arr lim = C.getLimits();
  const int steps = 50;
  C.checkConsistency();
  StringA jointNames = C.getJointNames();

  //  uint saveCount=0;

  C.gl()->resetPressedKey();
  for(uint i=x0.N; i--;) {
    x=x0;
    double upper_lim = lim(i, 1);
    double lower_lim = lim(i, 0);
    double delta = upper_lim - lower_lim;
    double center = lower_lim + .5*delta;
    if(delta<=1e-10) { center=x0(i); delta=1.; }
    double offset = acos(2. * (x0(i) - center) / delta);
    if(offset!=offset) offset=0.; //if NAN

    for(uint t=0; t<steps; t++) {
      if(ino && ino->poll(false, true)) return -1;

      x(i) = center + (delta*(0.5*cos(RAI_2PI*t/steps + offset)));
      // Joint limits
      checkNan(x);
      C.setJointState(x);
      int key = C.watch(false, STRING("DOF = " <<i <<" : " <<jointNames(i) <<lim[i]));

      if(key==13 || key==32 || key==27 || key=='q') {
        C.setJointState(x0);
        return key;
      }
      wait(0.01);
    }
  }
  C.setJointState(x0);
  return C.watch(true);
}

Frame* movingBody=nullptr;
Vector selpos;
double seld, selx, sely, selz;

struct EditConfigurationClickCall:OpenGL::GLClickCall {
  Configuration* ors;
  EditConfigurationClickCall(Configuration& _ors) { ors=&_ors; }
  bool clickCallback(OpenGL& gl) {
    OpenGL::GLSelect* top=gl.topSelection;
    if(!top) return false;
    uint i=top->name;
    cout <<"CLICK call: id = 0x" <<std::hex <<gl.topSelection->name <<" : ";
    gl.text.clear();
    if((i&3)==1) {
      Frame* s=ors->frames.elem(i>>2);
      gl.text <<"shape selection: shape=" <<s->name <<" X=" <<s->ensure_X() <<endl;
      //      listWrite(s->ats, gl.text, "\n");
      cout <<gl.text;
    }
    if((i&3)==2) {
      Joint* j = ors->frames.elem(i>>2)->joint;
      gl.text
          <<"edge selection: " <<j->from()->name <<' ' <<j->frame->name
          //         <<"\nA=" <<j->A <<"\nQ=" <<j->Q <<"\nB=" <<j->B
          <<endl;
      //      listWrite(j->ats, gl.text, "\n");
      cout <<gl.text;
    }
    cout <<endl;
    return true;
  }
};

struct EditConfigurationHoverCall:OpenGL::GLHoverCall {
  Configuration* ors;
  EditConfigurationHoverCall(Configuration& _ors);// { ors=&_ors; }
  bool hoverCallback(OpenGL& gl) {
    //    if(!movingBody) return false;
    if(!movingBody) {
      Joint* j=nullptr;
      Frame* s=nullptr;
      timerStart(true);
      gl.Select(true);
      OpenGL::GLSelect* top=gl.topSelection;
      if(!top) return false;
      uint i=top->name;
      cout <<timerRead() <<"HOVER call: id = 0x" <<std::hex <<gl.topSelection->name <<endl;
      if((i&3)==1) s=ors->frames.elem(i>>2);
      if((i&3)==2) j=ors->frames.elem(i>>2)->joint;
      gl.text.clear();
      if(s) {
        gl.text <<"shape selection: body=" <<s->name <<" X=" <<s->ensure_X();
      }
      if(j) {
        gl.text
            <<"edge selection: " <<j->from()->name <<' ' <<j->frame->name
            //           <<"\nA=" <<j->A <<"\nQ=" <<j->Q <<"\nB=" <<j->B
            <<endl;
        //        listWrite(j->ats, gl.text, "\n");
      }
    } else {
      //gl.Select();
      //double x=0, y=0, z=seld;
      //double x=(double)gl.mouseposx/gl.width(), y=(double)gl.mouseposy/gl.height(), z=seld;
      double x=gl.mouseposx, y=gl.mouseposy, z=seld;
      gl.unproject(x, y, z, true);
      cout <<"x=" <<x <<" y=" <<y <<" z=" <<z <<" d=" <<seld <<endl;
      movingBody->setPosition(selpos.getArr() + ARR(x-selx, y-sely, z-selz));
    }
    return true;
  }
};

EditConfigurationHoverCall::EditConfigurationHoverCall(Configuration& _ors) {
  ors=&_ors;
}

struct EditConfigurationKeyCall:OpenGL::GLKeyCall {
  Configuration& K;
  bool& exit;
  EditConfigurationKeyCall(Configuration& _K, bool& _exit): K(_K), exit(_exit) {}
  bool keyCallback(OpenGL& gl) {
    if(false && gl.pressedkey==' ') { //grab a body
      if(movingBody) { movingBody=nullptr; return true; }
      Joint* j=nullptr;
      Frame* s=nullptr;
      gl.Select();
      OpenGL::GLSelect* top=gl.topSelection;
      if(!top) { cout <<"No object below mouse!" <<endl;  return false; }
      uint i=top->name;
      //cout <<"HOVER call: id = 0x" <<std::hex <<gl.topSelection->name <<endl;
      if((i&3)==1) s=K.frames.elem(i>>2);
      if((i&3)==2) j=K.frames.elem(i>>2)->joint;
      if(s) {
        cout <<"selected shape " <<s->name <<" of body " <<s->name <<endl;
        selx=top->x;
        sely=top->y;
        selz=top->z;
        seld=top->dmin;
        cout <<"x=" <<selx <<" y=" <<sely <<" z=" <<selz <<" d=" <<seld <<endl;
        selpos = s->ensure_X().pos;
        movingBody=s;
      }
      if(j) {
        cout <<"selected joint " <<j->frame->ID <<" connecting " <<j->from()->name <<"--" <<j->frame->name <<endl;
      }
      return true;
    } else switch(gl.pressedkey) {
        case '1':  K.orsDrawBodies^=1;  break;
        case '2':  K.orsDrawShapes^=1;  break;
        case '3':  K.orsDrawJoints^=1;  K.orsDrawMarkers^=1; break;
        case '4':  K.orsDrawProxies^=1;  break;
        case '5':  gl.reportSelects^=1;  break;
        case '6':  gl.reportEvents^=1;  break;
        case 'j':  gl.camera.X.pos += gl.camera.X.rot*Vector(0, 0, .1);  break;
        case 'k':  gl.camera.X.pos -= gl.camera.X.rot*Vector(0, 0, .1);  break;
        case 'i':  gl.camera.X.pos += gl.camera.X.rot*Vector(0, .1, 0);  break;
        case ',':  gl.camera.X.pos -= gl.camera.X.rot*Vector(0, .1, 0);  break;
        case 'l':  gl.camera.X.pos += gl.camera.X.rot*Vector(.1, .0, 0);  break;
        case 'h':  gl.camera.X.pos -= gl.camera.X.rot*Vector(.1, 0, 0);  break;
        case 'a':  gl.camera.focus(
            (gl.camera.X.rot*(gl.camera.foc - gl.camera.X.pos)
             ^ gl.camera.X.rot*Vector(1, 0, 0)) * .001
            + gl.camera.foc);
          break;
        case 's':  gl.camera.X.pos +=
            (
              gl.camera.X.rot*(gl.camera.foc - gl.camera.X.pos)
              ^(gl.camera.X.rot * Vector(1., 0, 0))
            ) * .01;
          break;
        case 'q' :
          cout <<"EXITING" <<endl;
          exit=true;
          break;
      }
    gl.postRedrawEvent(true);
    return true;
  }
};

void editConfiguration(const char* filename, Configuration& C) {
  C.checkConsistency();

  //  gl.exitkeys="1234567890qhjklias, "; //TODO: move the key handling to the keyCall!
  bool exit=false;
  //  gl.addHoverCall(new EditConfigurationHoverCall(K));
//  K.gl()->addKeyCall(new EditConfigurationKeyCall(K,exit));
//  K.gl()->addClickCall(new EditConfigurationClickCall(K));
  Inotify ino(filename);
  for(; !exit;) {
    cout <<"reloading `" <<filename <<"' ... " <<std::endl;
    Configuration C_tmp;
    {
      FileToken file(filename, true);
      Graph G;
      try {
        lineCount=1;
        G.read(file);
        G.checkConsistency();
      } catch(std::runtime_error& err) {
        cout <<"g-File Synax Error line " <<lineCount <<": " <<err.what() <<" -- please check the file and re-save" <<endl;
      }
      try {
        C_tmp.readFromGraph(G);
        C = C_tmp;
        C.report();
      } catch(std::runtime_error& err) {
        cout <<"Configuration initialization failed: " <<err.what() <<" -- please check the file and re-save" <<endl;
      }
      file.cd_start(); //important: also on crash - cd back to original
    }
    cout <<"watching..." <<endl;
    int key = -1;
    C.gl()->recopyMeshes(C);
    C.gl()->resetPressedKey();
    for(;;) {
      key = C.gl()->setConfiguration(C, "waiting for file change", false);
      if(key==13 || key==32 || key==27 || key=='q') break;
      if(ino.poll(false, true)) break;
      wait(.2);
    }
    if(exit) break;
    if(key==13 || key==32) {
      cout <<"animating.." <<endl;
      //while(ino.pollForModification());
      key = animateConfiguration(C, &ino);
    }
    if(key==27 || key=='q') break;
    if(key==-1) continue;

    if(!getInteractivity()) {
      exit=true;
    }
  }
}

}//namespace

//===========================================================================
//
// template instantiations
//

#include "../Core/util.ipp"
template rai::Array<rai::Shape*>::Array(uint);
//template Shape* listFindByName(const Array<Shape*>&,const char*);

#include "../Core/array.ipp"
template rai::Array<rai::Joint*>::Array();
