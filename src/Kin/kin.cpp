/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kin.h"
#include "frame.h"
#include "forceExchange.h"
#include "dof_particles.h"
#include "dof_path.h"
#include "proxy.h"
//#include "kin_swift.h"
#include "kin_physx.h"
#include "kin_ode.h"
#include "kin_feather.h"
#include "featureSymbols.h"
#include "viewer.h"
#include "simulation.h"
#include "../Core/graph.h"
#include "../Core/util.h"
#include "../Geo/fclInterface.h"
#include "../Geo/qhull.h"
#include "../Geo/assimpInterface.h"
#include "../Gui/opengl.h"
#include "../Algo/rungeKutta.h"
#include "../Algo/spline.h"
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

StringA framesToNames(const FrameL& frames) {
  StringA names;
  resizeAs(names, frames);
  for(uint i=0; i<frames.N; i++) {
    names.elem(i) = frames.elem(i)->name;
  }
  return names;
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
      if(force || s->mesh().V.d0!=s->mesh().Vn.d0 || s->mesh().T.d0!=s->mesh().Tn.d0) s->mesh().computeTriNormals();
      if(force || s->sscCore().V.d0!=s->sscCore().Vn.d0 || s->sscCore().T.d0!=s->sscCore().Tn.d0) s->sscCore().computeTriNormals();
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
  //shared_ptr<SwiftInterface> swift;
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
  //self->swift.reset();
  self->fcl.reset();
  clear();
  self.reset();
}

/// make this a copy of C (copying all frames, forces & proxies)
void Configuration::copy(const Configuration& C, bool referenceFclOnCopy) {
  CHECK(this != &C, "never copy C onto itself");

  clear();
  jacMode = C.jacMode;

  //copy frames; first each Frame/Link/Joint directly, where all links go to the origin K (!!!); then relink to itself
  for(Frame* f:C.frames) new Frame(*this, f);
  for(Frame* f:C.frames) {
    if(f->parent) frames.elem(f->ID)->setParent(frames.elem(f->parent->ID));
    if(f->prev) frames.elem(f->ID)->prev = frames.elem(f->prev->ID);
  }
//  addFramesCopy(C.frames);
  frames.reshapeAs(C.frames);

  //copy proxies; first they point to origin frames; afterwards, let them point to own frames
  copyProxies(C.proxies);
  //  proxies = K.proxies;
  //  for(Proxy& p:proxies) { p.a = frames.elem(p.a->ID); p.b = frames.elem(p.b->ID);  p.coll.reset(); }

  //copy contacts
  for(Dof* dof:C.otherDofs) {
    const ForceExchange* ex = dof->fex();
    if(ex) new ForceExchange(*frames.elem(ex->a.ID), *frames.elem(ex->b.ID), ex->type, ex);
  }

  //copy fcl reference
  if(referenceFclOnCopy) {
    //self->swift = C.self->swift;
    self->fcl = C.self->fcl;
  }

  //copy vector state
  calc_indexedActiveJoints(true);
  q = C.q;
  qInactive = C.qInactive;
  _state_indexedJoints_areGood = C._state_indexedJoints_areGood;
  _state_q_isGood = C._state_q_isGood;
  _state_proxies_isGood = C._state_proxies_isGood;
}

bool Configuration::operator!() const { return this==&NoConfiguration; }

Frame* Configuration::addFrame(const char* name, const char* parent, const char* args, bool warnDuplicateName) {
  if(name && warnDuplicateName) { //check name duplication
    Frame* exists = getFrame(name, false);
    if(exists) { LOG(-1) <<"frame already exists! returning existing without modifications!"; return exists; }
  }

  Frame* f = new Frame(*this);

  if(name) f->name = name;

  if(parent && parent[0]) {
    Frame* p = getFrame(parent);
    if(p) {
      f->set_X() = p->ensure_X();
      f->setParent(p, true);
    }
  }

  if(args && args[0]) {
    if(!f->ats) f->ats = make_shared<Graph>();
    String(args) >>*(f->ats);
    f->read(*f->ats);
  }

//  if(f->parent) f->calc_X_from_parent();

  return f;
}

Frame* Configuration::addFile(const char* filename, const char* namePrefix) {
  uint n=frames.N;
  FileToken file(filename, true);
  Graph G(file);
  if(namePrefix && namePrefix[0]) {
    for(Node* n:G) {
      n->key.prepend(namePrefix);
      rai::String* tmp=0;
      if(n->is<Graph>()) tmp=n->graph().find<rai::String>("mimic");
      if(tmp) tmp->prepend(namePrefix);
    }
  }
  readFromGraph(G, true);
  file.cd_start();
  if(frames.N==n) return 0; //no frames added
  return frames.elem(n); //returns 1st frame of added file
}

Frame* Configuration::addAssimp(const char* filename) {
  AssimpLoader A(filename, true, true);
  //-- create all frames
  uint Nold = frames.N;
  for(uint i=0; i<A.names.N; i++) {
    addFrame(A.names(i));
  }
  //-- link to parents
  for(uint i=0; i<A.names.N; i++) {
    Frame* f = frames(Nold+i);
    if(A.parents(i).N) {
      int j = A.names.findValue(A.parents(i));
      CHECK_GE(j, 0, "parent name not found");
      CHECK_LE(j, (int)i, "parent is later frame!");
      rai::Frame* parent = frames(Nold+j);
      f->setParent(parent);
    }
    f->set_X() = A.poses(i);
  }
  //-- set meshes
  for(uint i=0; i<A.names.N; i++) {
    Frame* f = frames(Nold+i);
    if(A.meshes(i).N==1) {
      if(A.meshes(i)(0).V.N) {
        Shape* s = new Shape(*f);
        s->type() = ST_mesh;
        s->mesh() = A.meshes(i).scalar();
      }
    } else if(A.meshes(i).N>1) {
      uint j=0;
      for(auto& mesh:A.meshes(i)) {
        if(mesh.V.N) {
          Frame* f1 = addFrame(STRING(f->name<<'_' <<j++));
          f1->setParent(f);
          f1->set_Q()->setZero();
          Shape* s = new Shape(*f1);
          s->type() = ST_mesh;
          s->mesh() = mesh;
        }
      }
    }
  }
  return frames(Nold+0); //root frame
}

#if 0
Frame* Configuration::addObject(ShapeType shape, const arr& size, const arr& col) {
  Frame* f = new Frame(*this);
  Shape* s = new Shape(*f);
  s->type() = shape;
  if(col.N) s->mesh().C = col;
  if(radius>0.) s->size() = arr{radius};
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
      s->size() = arr{radius};
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
Frame* Configuration::addCopy(const FrameL& F, const DofL& _dofs, const str& prefix) {
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
    if(f->joint && f->joint->isStable) {
      Frame* f_orig = getFrame(f_new->name); //identify by name!!!
      if(f_orig!=f_new) {
        CHECK(f_orig->joint, "");
        f_new->joint->setMimic(f_orig->joint);
      }
    }

    //auto-create prev link if names match
    if(f_new->ID>=F.N) {
      rai::Frame* p = frames(f_new->ID-F.N);
      if(p->name==f_new->name){ f_new->prev = p;  f_new->time = p->time + f_new->tau; }
    }
  }

  //relink frames - special attention to mimic'ing
  for(Frame* f:F) if(f->parent) {
      if(f->parent->ID>maxId || FId2thisId(f->parent->ID)==-1) {
        LOG(-1) <<"can't relink frame '" <<*f <<"'";
      }
      Frame* f_new = frames.elem(FId2thisId(f->ID));
      f_new->setParent(frames.elem(FId2thisId(f->parent->ID)));
      //take care of within-F mimic joints:
      if(f->joint && f->joint->mimic) {
        if(f->joint->mimic->frame->ID<maxId && FId2thisId(f->joint->mimic->frame->ID)!=-1) {
          f_new->joint->setMimic(frames.elem(FId2thisId(f->joint->mimic->frame->ID))->joint, true);
        }
      }
    }

  //copy force exchanges
  for(Dof* dof:_dofs) {
    const ForceExchange* ex = dof->fex();
    if(ex) new ForceExchange(*frames.elem(FId2thisId(ex->a.ID)), *frames.elem(FId2thisId(ex->b.ID)), ex->type, ex);
  }

  if(!(frames.N%F.N)) frames.reshape(-1, F.N);

  uint startId = FId2thisId(F.first()->ID);
  if(prefix.N) {
    for(uint i=startId; i<frames.N; i++) frames.elem(i)->name.prepend(prefix);
  }

  return frames.elem(startId);
}

/// same as addCopies() with C.frames and C.forces
Frame* Configuration::addConfigurationCopy(const Configuration& C, const str& prefix, double tau) {
  Frame* f=addCopy(C.frames, C.otherDofs, prefix);
  if(tau>=0.) f->tau=tau;
  return f;
}

void Configuration::delFrame(const char* name){
  rai::Frame* p = getFrame(name, true);
  if(p) delete p;
}

void Configuration::delSubtree(const char* name){
  rai::Frame* p = getFrame(name, true);
  if(!p) return;
  FrameL F = p->getSubtree();
  for(rai::Frame *f:F) delete f;
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
  FrameL F;
  resizeAs(F, ids);
  for(uint i=0; i<ids.N; i++) F.elem(i) = frames.elem(ids.elem(i));
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
  uintA I;
  resizeAs(I, names);
  for(uint i=0; i<names.N; i++) {
    Frame* f = getFrame(names.elem(i));
    if(!f) HALT("frame name '"<<names.elem(i)<<"' doesn't exist");
    I.elem(i) = f->ID;
  }
  return I;
}

FrameL Configuration::getJoints(bool activesOnly) const {
  FrameL F;
  for(auto* f:frames) if(f->joint && (!activesOnly || f->joint->active)) F.append(f);
  return F;
}

FrameL Configuration::getJointsSlice(uint t, bool activesOnly) const {
  FrameL F;
  for(auto* f:frames[t]) {
    if((f->joint && (!activesOnly || f->joint->active))
        || f->forces.N)  F.append(f);
  }
  return F;
}

FrameL Configuration::getJointsSlice(const FrameL& slice, bool activesOnly) const {
  FrameL F;
  for(auto* f:slice) {
    if((f->joint && (!activesOnly || f->joint->active))
        || f->forces.N)  F.append(f);
  }
  return F;
}

/// get the frame IDs of all active joints
uintA Configuration::getDofIDs() const {
  ((Configuration*)this)->ensure_indexedJoints();
  uintA dofIDs(activeDofs.N);
  uint i=0;
  for(Dof* d:activeDofs) {
    CHECK(d->frame, "dof " <<*d <<" does not have frame!");
    dofIDs.elem(i++) = d->frame->ID;
  }
  return dofIDs;
}

/// get the frame names of all active joints
StringA Configuration::getJointNames() const {
  ((Configuration*)this)->ensure_q();
  StringA names(getJointStateDimension());
  for(Dof* j:activeDofs) {
    String name=j->frame->name;
    if(!name) name <<'q' <<j->qIndex;
    if(j->dim==1) names(j->qIndex) <<name;
    else for(uint i=0; i<j->dim; i++) names(j->qIndex+i) <<name <<':' <<i;
  }
  return names;
}

DofL Configuration::getDofs(const FrameL& F, bool actives, bool inactives, bool mimics) const {
  DofL dofs;
  for(Frame* f:F) {
    Dof* dof = f->joint;
    if(dof && ((actives && dof->active) || (inactives && !dof->active)) && (mimics || !dof->mimic)) {
      dofs.append(dof);
    }
    dof = f->pathDof;
    if(dof && ((actives && dof->active) || (inactives && !dof->active)) && (mimics || !dof->mimic)) {
      dofs.append(dof);
    }
    for(ForceExchange* dof: f->forces) if(&dof->a==f) {
        if(dof && ((actives && dof->active) || (inactives && !dof->active)) && (mimics || !dof->mimic)) {
          dofs.append(dof);
        }
      }
  }
  return dofs;
}

/// get the names of all frames
StringA Configuration::getFrameNames() const {
  return framesToNames(frames);
}

/// get the (frame-ID, parent-ID) tuples and control scale for all active joints that represent controls
uintA Configuration::getCtrlFramesAndScale(arr& scale) const {
  uintA qFrames;
  for(rai::Frame* f : frames) {
    rai::Joint* j = f->joint;
    if(j && j->active && j->dim>0 && (!j->mimic) && j->H>0. && j->type!=rai::JT_tau && !j->isStable) {
      qFrames.append(uintA{f->ID, f->parent->ID});
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

/// get all frames without parent or with a PartBreak joint
FrameL Configuration::getParts() const {
  FrameL F;
  for(Frame* a:frames) if(!a->parent || (a->joint && a->joint->isPartBreak)) F.append(a);
  return F;
}

/// get all frames without parent or with joint
FrameL Configuration::getLinks() const {
  FrameL links;
  for(Frame* a:frames) if(!a->parent || a->joint) links.append(a);
  return links;
}

rai::Array<DofL> Configuration::getPartsDofs() const {
  FrameL parts = getParts();
  rai::Array<DofL> dofs(parts.N);
  uint nonzero=0;
  uint totalDim=0;
  for(uint i=0; i<parts.N; i++) {
    Frame* f = parts.elem(i);
    FrameL sub = {f};
    f->getPartSubFrames(sub);
    DofL D = getDofs(sub, true, false);
    DofL F;
    if(D.N) {
      //remove mimics
      for(uint i=D.N; i--;) if(D(i)->mimic) D.remove(i);
      //separate forces
      for(uint i=D.N; i--;) if(D(i)->fex()) {
          F.append(D(i));
          D.remove(i);
        }
      //count dofs
      uint d=0;
      for(Dof* dof:D) d+=dof->dim;
      //add normal
      if(d) {
        dofs(nonzero++) = D;
        //cout <<'#' <<i <<' ' <<f->name <<' ' <<D.modList() <<endl;
        for(Dof* dof:D) totalDim += dof->dim;
      }
      //add forces
      if(F.N) {
        dofs(nonzero++) = F;
        //cout <<"#FORCES" <<i <<' ' <<f->name <<' ' <<F.modList() <<endl;
        for(Dof* dof:F) totalDim += dof->dim;
      }
    }
  }
  dofs.resizeCopy(nonzero);
  //cout <<"total dim:" <<totalDim <<endl;
  return dofs;
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

arr Configuration::getDofState(const DofL& dofs) const {
  ((Configuration*)this)->ensure_q();

  uint n=0;
  for(Dof* dof: dofs) {
    if(!dof->mimic) n += dof->dim;
  }

  arr x(n);
  n=0;
  for(Dof* dof:dofs) {
    if(!dof->mimic) {
      if(dof->active) {
        for(uint ii=0; ii<dof->dim; ii++) x(n+ii) = q.elem(dof->qIndex+ii);
      } else {
        for(uint ii=0; ii<dof->dim; ii++) x(n+ii) = qInactive(dof->qIndex+ii);
      }
      n += dof->dim;
    }
  }
  CHECK_EQ(n, x.N, "");
  return x;
}

arr Configuration::getDofHomeState(const DofL& dofs) const {
  ((Configuration*)this)->ensure_indexedJoints();

  uint n=0;
  for(Dof* dof: dofs) if(!dof->mimic) n += dof->dim;

  arr x(n);
  n=0;
  for(Dof* dof:dofs) {
    if(!dof->mimic) {
      for(uint ii=0; ii<dof->dim; ii++) x(n+ii) = dof->q0.elem(ii);
      n += dof->dim;
    }
  }
  CHECK_EQ(n, x.N, "");
  return x;
}

/// get the (F.N,7)-matrix of all poses for all given frames
arr Configuration::getFrameState(const FrameL& F) const {
  arr X(F.N, 7);
  for(uint i=0; i<X.d0; i++) {
    rai::Transformation Xi = F.elem(i)->ensure_X();
    Xi.rot.uniqueSign();
    memmove(X.p+7*i+0, &Xi.pos.x, 3*X.sizeT);
    memmove(X.p+7*i+3, &Xi.rot.w, 4*X.sizeT);
//    X[i] = Xi.getArr7d();
  }
  return X;
}

/// set the q-vector (all joint and force DOFs)
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
//  for(Dof* j:activeDofs) {
//    if(j->joint() && j->joint()->type!=JT_tau) {
//      j->frame->_state_setXBadinBranch();
//    }
//  }
  calc_Q_from_q();
}

/// set the DOFs (joints and forces) for the given subset of frames
void Configuration::setDofState(const arr& _q, const DofL& dofs, bool mimicsIncludedInQ) {
  setJointStateCount++; //global counter
  ensure_q();

  uint nd=0;
  for(Dof* j:dofs) {
    if(!j) continue;
    if(mimicsIncludedInQ && j->mimic) j = j->mimic; //equivalent to setting state of mimic dof!!
    if(!j->mimic) CHECK_LE(nd+j->dim, _q.N, "given q-vector too small");
    if(j->active) {
      if(!j->mimic) for(uint ii=0; ii<j->dim; ii++) q.elem(j->qIndex+ii) = _q(nd+ii);
      j->setDofs(q, j->qIndex);
    } else {
      if(!j->mimic) for(uint ii=0; ii<j->dim; ii++) qInactive.elem(j->qIndex+ii) = _q(nd+ii);
      j->setDofs(qInactive, j->qIndex);
    }
    if(!j->mimic) nd += j->dim;
  }
  CHECK_EQ(_q.N, nd, "given q-vector has wrong size");

  proxies.clear();

  _state_q_isGood=true;
  _state_proxies_isGood=false;
}

/// set the pose of all frames as given by the (F.N,7)-matrix
void Configuration::setFrameState(const arr& X, const FrameL& F) {
  CHECK_GE(X.d0, F.N, "X.d0=" <<X.d0 <<" not equal to frames.N=" <<F.N);
  for(Frame* f:F) f->_state_setXBadinBranch();
  for(uint i=0; i<F.N; i++) {
    Frame* f = F.elem(i);
    f->X.set(X[i]);
    f->X.rot.normalize();
    f->_state_X_isGood = true;
  }
  for(Frame* f:F) if(f->parent) {
      f->Q.setDifference(f->parent->ensure_X(), f->X);
      _state_q_isGood=false;
    }
}

/// set the 'tau-coordinate' (time interval from previous time slice) for equal for all frames
void Configuration::setTaus(double tau) {
  for(Frame* a:frames) a->tau = tau;
}

/// set the 'tau-coordinate' (time interval from previous time slice) for equal for all frames
void Configuration::setTaus(const arr& tau) {
  CHECK_EQ(frames.nd, 2, "only for matrix of frames (=series of configurations)");
  CHECK_EQ(frames.d0, tau.N, "need taus for each slice");
  for(uint t=0; t<frames.d0; t++) frames(t, 0)->tau = tau(t);
}

void Configuration::setRandom(uint timeSlices_d1, int verbose) {
  ensure_indexedJoints();
  for(Dof* d:activeDofs) d->setRandom(timeSlices_d1, verbose);
  _state_q_isGood=false;
  checkConsistency();
}

void Configuration::setDofBiasesToCurrent() {
  ensure_q();
  for(Dof* d:activeDofs) d->q0 = d->getDofState();
}

void Configuration::setActiveDofs(const DofL& dofs) {
  for(rai::Frame* f:frames) if(f->joint) f->joint->active=false;
  for(Dof* d: otherDofs) d->active = false;
  DofL mimics;
  for(Dof* d:dofs) {
    d->active = true;
    if(d->mimic) mimics.append(d->mimic); //activate also the joint mimic'ed
    for(Dof* dd:d->mimicers) mimics.append(dd); //activate also mimicing joints
  }
  reset_q();
  activeDofs = dofs;
  for(Dof* d:mimics) {
    d->active = true;
    activeDofs.setAppend(d);
  }
  calc_indexedActiveJoints(false);
//  checkConsistency();
}

void Configuration::selectJoints(const FrameL& F, bool notThose) {
  DofL D(F.N);
  D.setZero();
  uint n=0;
  for(Frame* f: F) if(f && f->joint) D.elem(n++) = f->joint;
  D.resizeCopy(n);
  selectJoints(D, notThose);
}

/// selects only the joints of the given frames to be active -- the q-vector (and Jacobians) will refer only to those DOFs
void Configuration::selectJoints(const DofL& dofs, bool notThose) {
  for(Frame* f: frames) if(f->joint) f->joint->active = notThose;
  for(Dof* d: otherDofs) d->active = notThose;
  for(Dof* dof: dofs) if(dof) {
      dof->active = !notThose;
      if(dof->mimic) dof->mimic->active = dof->active; //activate also the joint mimic'ed
    }
  for(Frame* f: frames) if(f && f->joint && f->joint->mimic) { //mimic's of active joints are active as well
      if(f->joint->mimic->active) f->joint->active = true;
    }
  reset_q();
//  ensure_indexedJoints();
//  ensure_q();
//  checkConsistency();
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

/// select joint frames that have a given attribute in the gfile
void Configuration::selectJointsByAtt(const StringA& attNames, bool notThose) {
  FrameL F;
  for(Frame* f:frames) if(f->joint) {
      for(const String& s:attNames) if((*f->ats)[s]) { F.append(f); break; }
    }
  selectJoints(F, notThose);
}

/// returns diagonal of the metric in q-space determined by all joints' Joint::H
arr Configuration::getCtrlMetric() const {
  arr H = zeros(getJointStateDimension());
  for(const Dof* dof:activeDofs) {
    const Joint* j = dof->joint();
    if(j) {
      double h=j->H;
      //    CHECK(h>0.,"Hmetric should be larger than 0");
      if(j->type==JT_transXYPhi) {
        H(j->qIndex+0)=h*10.;
        H(j->qIndex+1)=h*10.;
        H(j->qIndex+2)=h;
      } else {
        for(uint k=0; k<j->dim; k++) H(j->qIndex+k)=h;
      }
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
  for(Dof* j:activeDofs) {
    for(uint i=0; i<j->dim; i++) {
      Wdiag(j->qIndex+i) = ::pow(BM(j->frame->ID), power);
    }
  }
  return Wdiag;
#endif
}

/// returns the vector of joint limts */
arr Configuration::getJointLimits(const DofL& dofs) const {
  uint N=0;
  for(Dof* d:dofs) if(!d->mimic) N += d->dim;
  arr limits(2, N);
  limits.setZero();
  for(uint i=0; i<N; i++) limits(1, i)=-1.;
  N=0;
  for(Dof* d:dofs) if(!d->mimic) {
      for(uint k=0; k<d->dim; k++) { //in case joint has multiple dimensions
        if(d->limits.N) {
          limits(0, N+k) = d->limits.elem(2*k+0); //lo
          limits(1, N+k) = d->limits.elem(2*k+1); //up
        }
      }
      N += d->dim;
    }
  return limits;
}

arr Configuration::getTorqueLimits(const DofL& dofs, uint index) const {
  uint N=0;
  for(Dof* d:dofs) if(!d->mimic) N += d->dim;
  arr limits(N);
  limits=-1.;
  N=0;
  for(Dof* d:dofs) if(!d->mimic) {
      CHECK_EQ(d->dim, 1, "");
      if(d->limits.N>index) limits(N) = d->limits.elem(index);
      N += d->dim;
    }
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

/// get the sum of all shape penetrations -- PRECONDITION: proxies have been computed (with stepFcl())
double Configuration::getTotalPenetration() {
  fcl()->mode = rai::FclInterface::_broadPhaseOnly;
  ensure_proxies(true);

  double D=0.;
  for(const Proxy& p:proxies) {
    //early check: if proxy is way out of collision, don't bother computing it precise
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

bool Configuration::getCollisionFree() {
  fcl()->mode = rai::FclInterface::_binaryCollisionAll;
  ensure_proxies(false);

  bool feas=true;
  for(const rai::Proxy& p:proxies) if(p.d<=0.) { feas=false; break; }
  return feas;
}

Graph Configuration::reportForces() {
  Graph G;
  for(Dof* dof : otherDofs) {
    const ForceExchange* ex = dof->fex();
    if(ex) {
      Graph& g = G.addSubgraph();
      g.add<String>("from", ex->a.name);
      g.add<String>("to", ex->b.name);
      g.add<arr>("force", ex->force);
      g.add<arr>("torque", ex->torque);
      g.add<arr>("poa", ex->poa);
    }
  }
  return G;
}

/// checks if all names of the bodies are disjoint
bool Configuration::checkUniqueNames(bool makeUnique) {
  for(Frame* a: frames) for(Frame* b: frames) {
      if(a==b) break;
      if(a->name==b->name) {
        if(!makeUnique) return false;
        else a->name <<'_' <<a->ID;
      }
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
//  swiftDelete();
//  if(self && self->viewer) self->viewer.reset();
  if(self && self->fcl) self->fcl.reset();

  reset_q();
  proxies.clear(); //while(proxies.N){ delete proxies.last(); /*checkConsistency();*/ }
  while(frames.N) { delete frames.last(); /*checkConsistency();*/ }
  reset_q();
  //if(self->viewer) self->viewer->recopyMeshes(*this);

  _state_proxies_isGood=false;
}

/// clear the q-vector
void Configuration::reset_q() {
  q.clear();
  qInactive.clear();
  activeDofs.clear();

  _state_indexedJoints_areGood=false;
  _state_q_isGood=false;
}


/** @brief re-orient all joints (edges) such that n becomes
  the root of the configuration */
void Configuration::reconfigureRoot(Frame* newRoot, bool untilPartBreak) {
  newRoot->makeRoot(untilPartBreak);
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
  a->setParent(b);
}

void Configuration::pruneRigidJoints() {
  for(Frame* f:frames) {
    if(f->joint && f->joint->type == JT_rigid) f->setJoint(JT_none);
  }
}

void Configuration::pruneInactiveJoints() {
  for(Frame* f:frames) {
    if(f->joint && !f->joint->active) f->setJoint(JT_none);
  }
}

void Configuration::reconnectShapesToParents() {
  reset_q();
  for(Frame* f:frames) if(f->parent && !f->joint && f->shape) {
    if(!f->parent->shape && f->get_Q().isZero()){
      new Shape(*f->parent, f->shape);
      if(f->ats){
        FileToken *fil=f->ats->find<FileToken>("mesh");
        if(fil) f->parent->ats->add("mesh", *fil);
      }
      delete f->shape;
      f->shape=0;
    }
  }
}

void Configuration::reconnectLinksToClosestJoints() {
  reset_q();
  for(Frame* f:frames) if(f->parent) {
      Transformation Q;
      Frame* link = f->getUpwardLink(Q);
      Q.rot.normalize();

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

void Configuration::pruneUselessFrames(bool pruneNamed, bool pruneNonContactNonMarker, bool pruneNonVisible) {
  for(uint i=frames.N; i--;) {
    Frame* f=frames.elem(i);
    if((pruneNamed || !f->name) && !f->children.N && !f->joint && !f->inertia) {
      if(!f->shape){
        delete f; //that's all there is to do
      }else if(pruneNonContactNonMarker && !f->shape->cont && f->shape->type()!=ST_marker){
        delete f;
      }else if(pruneNonVisible && f->shape->alpha()<1.){
        delete f;
      }
    }
  }
}

void Configuration::optimizeTree(bool _pruneRigidJoints, bool pruneNamed, bool pruneNonContactNonMarker, bool pruneNonVisible) {
  if(_pruneRigidJoints) pruneRigidJoints(); //problem: rigid joints bear the semantics of where a body ends
  reconnectLinksToClosestJoints();
  reconnectShapesToParents();
  pruneUselessFrames(pruneNamed, pruneNonContactNonMarker, pruneNonVisible);
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
    if(!a->parent) a->setParent(frames.first());
    if(!a->joint) new Joint(*a);
    a->joint->makeFree(H_cost);
  }
}

void Configuration::addTauJoint() {
  Joint* jt = new Joint(*frames.first(), JT_tau);
  jt->H = 0.;
}

bool Configuration::hasTauJoint(Frame* f) {
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
    //count dimensions yourself and check...
    uint myqdim = 0;
    for(Dof* j:activeDofs) {
      if(j->mimic) {
        CHECK_EQ(j->qIndex, j->mimic->qIndex, "");
        CHECK_EQ(j->active, j->mimic->active, "");
      } else {
        CHECK_EQ(j->qIndex, myqdim, "joint indexing is inconsistent");
        myqdim += j->dim;
      }
    }

    //consistency with Q
    for(Frame* f : frames) if(f->joint) {
        Joint* j = f->joint;
        arr jq = j->calcDofsFromConfig();
        CHECK_EQ(jq.N, j->dim, "");
        arr tmpq;
        if(j->active) {
          tmpq.referToRange(q, j->qIndex, j->qIndex+j->dim-1);
        } else {
          tmpq.referToRange(qInactive, j->qIndex, j->qIndex+j->dim-1);
          //for(uint i=0; i<jq.N; i++) CHECK_ZERO(std::fmod(jq.elem(i) - qInactive.elem(j->qIndex+i), RAI_2PI), 2e-5, "joint vector q and relative transform Q do not match for joint '" <<j->frame->name <<"', index " <<i);
        }
        if(j->type==JT_quatBall) { op_normalize(tmpq); }
        if(j->type==JT_free) { op_normalize(tmpq({3, 6}).noconst()); }
        //for(uint i=0; i<jq.N; i++) CHECK_ZERO(std::fmod(jq.elem(i) - tmpq.elem(i), RAI_2PI), 2e-5, "joint vector q and relative transform Q do not match for joint '" <<j->frame->name <<"', index " <<i);
      }
  }

  for(Frame* a: frames) {
    CHECK_EQ(&a->C, this, "");
    CHECK_EQ(a, frames.elem(a->ID), "");
    for(Frame* b: a->children) CHECK_EQ(b->parent, a, "");
    if(a->joint) CHECK_EQ(a->joint->frame, a, "");
    if(a->shape) CHECK_EQ(&a->shape->frame, a, "");
    if(a->inertia) CHECK_EQ(&a->inertia->frame, a, "");
    if(a->ats) a->ats->checkConsistency();

    a->Q.checkNan();
    a->X.checkNan();
    CHECK_ZERO(a->Q.rot.normalization()-1., 1e-6, "");
    if(a->_state_X_isGood) CHECK_ZERO(a->X.rot.normalization()-1., 1e-6, "");

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
    }

  for(Dof* j:activeDofs) {
    CHECK_EQ(&j->frame->C, this, "");
    if(j->mimic) {
      CHECK(j->mimic>(void*)1, "mimic was not parsed correctly");
      CHECK(frames.contains(j->mimic->frame), "mimic points to a frame outside this kinematic configuration");
      CHECK_EQ(j->active, j->mimic->active, "");
      CHECK_EQ(j->qIndex, j->mimic->qIndex, "");
      CHECK_EQ(j->dim, j->mimic->dim, "");
    }

    for(Joint* m:j->mimicers) {
      CHECK_EQ(m->mimic, j, "");
    }
  }

  for(Dof* d:otherDofs) {
    CHECK_EQ(&d->frame->C, this, "");
    if(d->mimic) {
      CHECK_EQ(d->active, d->mimic->active, "");
      CHECK_EQ(d->qIndex, d->mimic->qIndex, "");
      CHECK_EQ(d->dim, d->mimic->dim, "");
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
    for(Dof* j: activeDofs) { CHECK(j->active, ""); jointIsInActiveSet.elem(j->frame->ID)=true; }
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

  //check proxies
  for(const Proxy& p : proxies) {
    CHECK(p.a, "ill defined proxy");
    CHECK(p.b, "ill defined proxy");
    CHECK_EQ(this, &p.a->C, "");
    CHECK_EQ(this, &p.b->C, "");
  }

  return true;
}

Joint* Configuration::attach(Frame* a, Frame* b) {
  b = b->getUpwardLink();
  if(a->isChildOf(b, 1000)) LOG(-1) <<"attaching '" <<b->name <<"' to '" <<a->name <<"' creates a kinematic loop";
  if(b->parent) b->unLink();
  b->setParent(a, true);
  return new Joint(*b, JT_rigid);
}

Joint* Configuration::attach(const char* _a, const char* _b) {
  return attach(getFrame(_a), getFrame(_b));
}

uintAA Configuration::getCollisionExcludePairIDs(int verbose) {
  uintAA ex(frames.N);

  //go through all parts, within each part, check pair-wise
  FrameL parts = getParts();
  for(Frame* f: parts) {
    FrameL F = {f};
    f->getPartSubFrames(F);
    if(f->parent) { //add also parent link frames as potential excludes
      rai::Frame* p = f->parent->getUpwardLink();
      F.append(p);
      p->getPartSubFrames(F);
    }
    for(Frame* f1: F) if(f1->shape && f1->shape->cont) {
        for(Frame* f2: F) if(f2->ID>f1->ID && f2->shape && f2->shape->cont) {
            bool canCollide = f1->shape->canCollideWith(f2);
            if(!canCollide) {
              if(verbose) LOG(0) <<"excluding: "  <<f1->ID <<'.' <<f1->name  <<' ' <<f2->ID <<'.' <<f2->name;
              ex(f1->ID).setAppendInSorted(f2->ID);
            }
          }
      }
  }
  return ex;
}

FrameL Configuration::getCollidablePairs() {
  FrameL coll;

  //shapes within a link
  for(Frame* A:frames) if(A->shape) for(Frame* B:frames) if(B->shape) {
          if(A->ID>=B->ID) continue;
          if(A->shape->canCollideWith(B)) { coll.append(A); coll.append(B); }
        }

  coll.reshape(-1, 2);
  return coll;
}

/// creates uniques names by prefixing the node-index-number to each name */
void Configuration::prefixNames(bool clear) {
  if(!clear) for(Frame* a: frames) a->name=STRING('_' <<a->ID <<'_' <<a->name);
  else       for(Frame* a: frames) a->name.clear() <<a->ID;
}

void Configuration::calc_indexedActiveJoints(bool resetActiveJointSet) {
  if(resetActiveJointSet) {
    reset_q();

    //-- collect active dofs
    activeDofs.clear();
    for(Frame* f:frames) {
      if(f->joint && !f->joint->dim) f->joint->active=false;
      if(f->joint && f->joint->active) {
        activeDofs.append(f->joint);
      }
      if(f->particleDofs && f->particleDofs->active) {
        activeDofs.append(f->particleDofs);
      }
      if(f->pathDof && f->pathDof->active) {
        activeDofs.append(f->pathDof);
      }
      for(rai::ForceExchange* fex:f->forces) {
        if(fex->frame==f && fex->active) {
          activeDofs.append(fex);
        }
      }
    }
  } else {
    //we assume the activeJoints were set properly before!
  }

  _state_indexedJoints_areGood=true;

  //-- count active DOFs
  uint qcount=0;
  for(Dof* d: activeDofs) if(!d->mimic) {
      d->qIndex = qcount;
      qcount += d->dim;
    }
  for(Dof* d: activeDofs) if(d->mimic) {
      CHECK(d->mimic->active, "active dof '" << d->frame->name <<"' mimics inactive dof '" <<d->mimic->frame->name <<"'");
      d->qIndex = d->mimic->qIndex;
    }

  //-- resize q
  q.resize(qcount).setZero();
  _state_q_isGood = false;

  //-- count inactive DOFs
  DofL inactiveDofs;
  for(Frame* f:frames) if(f->joint && !f->joint->active) inactiveDofs.append(f->joint);
  for(Dof* d:otherDofs) if(!d->active) inactiveDofs.append(d);
  qcount=0;
  for(Dof* d:inactiveDofs) if(!d->mimic) {
      d->qIndex = qcount;
      qcount += d->dim;
    }
  for(Dof* d:inactiveDofs) if(d->mimic) {
      //CHECK(!d->mimic->active, "inactive dof'" << d->frame->name <<"'[" <<d->frame->ID <<"] mimics active dof'" <<d->mimic->frame->name <<"'[" <<d->mimic->frame->ID <<']');
      d->qIndex = d->mimic->qIndex;
    }

  //-- resize qInactive
  qInactive.resize(qcount).setZero();
}

void Configuration::calcDofsFromConfig() {
  ensure_indexedJoints();

  q.setZero();
  qInactive.setZero();

  uint n=0;
  //-- active dofs
  for(Dof* d: activeDofs) {
    if(d->mimic) continue; //don't count dependent joints
    CHECK_EQ(d->qIndex, n, "joint indexing is inconsistent");
    arr joint_q = d->calcDofsFromConfig();
    CHECK_EQ(joint_q.N, d->dim, "");
    if(!d->dim) continue; //nothing to do
    q.setVectorBlock(joint_q, d->qIndex);
    n += d->dim;
  }

  //-- inactive dofs
  DofL inactiveDofs;
  for(Frame* f:frames) if(f->joint && !f->joint->active) inactiveDofs.append(f->joint);
  for(Dof* d:otherDofs) if(!d->active) inactiveDofs.append(d);
  n=0;
  for(Dof* d: inactiveDofs) {
    if(d->mimic) continue; //don't count dependent joints
    CHECK_EQ(d->qIndex, n, "joint indexing is inconsistent");
    arr joint_q = d->calcDofsFromConfig();
    CHECK_EQ(joint_q.N, d->dim, "");
    if(!d->dim) continue; //nothing to do
    qInactive.setVectorBlock(joint_q, d->qIndex);
    n += d->dim;
  }
  CHECK_EQ(n, qInactive.N, "");

  _state_q_isGood=true;
}

void Configuration::calc_Q_from_q() {
  CHECK(_state_q_isGood, "");
  CHECK(_state_indexedJoints_areGood, "");

  uint n=0;
  for(Dof* j: activeDofs) {
    if(!j->mimic) CHECK_EQ(j->qIndex, n, "joint indexing is inconsistent");
    j->setDofs(q, j->qIndex);
    if(!j->mimic) n += j->dim;
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

void Configuration::ensure_proxies(bool fine) {
  if(!_state_proxies_isGood) stepFcl(); //broadphase
  if(fine) for(Proxy& p: proxies) if(!p.collision) p.calc_coll(); //fine
}

//===========================================================================
//
// core: kinematics and dynamics
//

/// returns the 'Jacobian' of a zero n-vector (initializes Jacobian to proper sparse/dense/rowShifted/noArr)
void Configuration::jacobian_zero(arr& J, uint n) const {
  if(!J) return;
  uint N=getJointStateDimension();
  if(jacMode==JM_dense) {
    J.resize(n, N).setZero();
  } else if(jacMode==JM_sparse) {
    J.sparse().resize(n, N, 0);
  } else if(jacMode==JM_rowShifted) {
    //estimate! the width
    uint width = N;
    if(frames.nd==2 && frames.d0>3) { width /= (frames.d0/4); width += 10; }
    J.rowShifted().resize(n, N, width);
  } else if(jacMode==JM_noArr) {
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
  CHECK(_state_indexedJoints_areGood, "");

  a->ensure_X();

  uint N=getJointStateDimension();
  jacobian_zero(J, 3);
  if(!J) return;

  while(a) { //loop backward down the kinematic tree
    if(!a->parent) break; //frame has no inlink -> done
    Joint* j=a->joint;
    if(j && j->active) {
      uint j_idx=j->qIndex;
      CHECK_LE(j_idx, q.N, "");
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
        if(j->type==JT_generic) {
          arr R = j->frame->parent->get_X().rot.getArr();
          R *= j->scale;
          arr Rt =~R;
          Vector d = (pos_world-j->X()*j->Q().pos);
          arr D = skew(d.getArr());

          for(uint i=0; i<j->code.N; i++) {
            switch(j->code[i]) {
              case 't': break;
              case 'x':  J.setMatrixBlock(Rt[0], 0, j_idx+i);  break;
              case 'X':  J.setMatrixBlock(-Rt[0], 0, j_idx+i);  break;
              case 'y':  J.setMatrixBlock(Rt[1], 0, j_idx+i);  break;
              case 'Y':  J.setMatrixBlock(-Rt[1], 0, j_idx+i);  break;
              case 'z':  J.setMatrixBlock(Rt[2], 0, j_idx+i);  break;
              case 'Z':  J.setMatrixBlock(-Rt[2], 0, j_idx+i);  break;
              case 'a':  J.setMatrixBlock(-D*Rt[0], 0, j_idx+i);  break;
              case 'A':  J.setMatrixBlock(D*Rt[0], 0, j_idx+i);  break;
              case 'b':  J.setMatrixBlock(-D*Rt[1], 0, j_idx+i);  break;
              case 'B':  J.setMatrixBlock(D*Rt[1], 0, j_idx+i);  break;
              case 'c':  J.setMatrixBlock(-D*Rt[2], 0, j_idx+i);  break;
              case 'C':  J.setMatrixBlock(D*Rt[2], 0, j_idx+i);  break;
              case 'w': {
                arr Jrot = j->X().rot.getArr() * a->Q.rot.getJacobian(); //transform w-vectors into world coordinate
                Jrot *= j->scale;
                Jrot = crossProduct(Jrot, conv_vec2arr(d));  //cross-product of all 4 w-vectors with lever
                Jrot /= sqrt(sumOfSqr(q({j_idx+i, j_idx+i+3})));   //account for the potential non-normalization of q
                J.setMatrixBlock(Jrot, 0, j_idx+i);
                i+=3;
              } break;
            }
          }
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
    //above a->joint, now a->pathDof (TODO: systematic for any dof, as below)
    PathDof* d=a->pathDof;
    if(d && d->active) {
      arr Jpos, Jang;
      d->getJacobians(Jpos, Jang);
      if(Jang.N) { //angular part: cross-product of rows with lever
        Jang = crossProduct(Jang, conv_vec2arr(pos_world-a->getPosition()));
        J.setMatrixBlock(Jang, 0, d->qIndex);
      }
      if(Jpos.N) { //translational part: direct
        J.setMatrixBlock(Jpos, 0, d->qIndex);
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
        if(j->type==JT_generic) {
          arr R = j->frame->parent->get_X().rot.getArr();
          R *= j->scale;
          arr Rt =~R;

          for(uint i=0; i<j->code.N; i++) {
            switch(j->code[i]) {
              case 't': break;
              case 'a':  J.setMatrixBlock(Rt[0], 0, j_idx+i);  break;
              case 'A':  J.setMatrixBlock(-Rt[0], 0, j_idx+i);  break;
              case 'b':  J.setMatrixBlock(Rt[1], 0, j_idx+i);  break;
              case 'B':  J.setMatrixBlock(-Rt[1], 0, j_idx+i);  break;
              case 'c':  J.setMatrixBlock(Rt[2], 0, j_idx+i);  break;
              case 'C':  J.setMatrixBlock(-Rt[2], 0, j_idx+i);  break;
              case 'w': {
                arr Jrot = j->X().rot.getArr() * a->Q.rot.getJacobian(); //transform w-vectors into world coordinate
                Jrot *= j->scale;
                Jrot /= sqrt(sumOfSqr(q({j_idx+i, j_idx+i+3}))); //account for the potential non-normalization of q
                J.setMatrixBlock(Jrot, 0, j_idx+i);
                i+=3;
              } break;
            }
          }

        }
        //all other joints: J=0 !!
      }
    }
    //above a->joint, now a->pathDof (TODO: systematic for any dof, as below)
    PathDof* d=a->pathDof;
    if(d && d->active) {
      arr Jpos, Jang;
      d->getJacobians(Jpos, Jang);
      if(Jang.N) { //angular part: direct
        J.setMatrixBlock(Jang, 0, d->qIndex);
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

/// Jacobian of the i-th body's orientation matrix (flattened as 9-vector)
void Configuration::kinematicsMat(arr& y, arr& J, Frame* a) const {
  CHECK_EQ(&a->C, this, "");

  arr R = a->ensure_X().rot.getMatrix().getArr();
  transpose(R); //the transpose has easier Jacobian...
  if(!!y) {
    y = R;
    y.reshape(9);
  }
  if(!!J) {
    arr A;
    jacobian_angular(A, a);
    jacobian_zero(J, 9);
    if(A.N) {
      J.setMatrixBlock(crossProduct(A, R[0]), 0, 0);
      J.setMatrixBlock(crossProduct(A, R[1]), 3, 0);
      J.setMatrixBlock(crossProduct(A, R[2]), 6, 0);
    }
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
  if(!A) {
    J.setNoArr();
    return;
  }
  if(isSparse(A)) {
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

  tau = a->tau;
  if(!!J) {
    jacobian_zero(J, 1);
    if(a && a->joint && a->joint->type==JT_tau) {
      //    CHECK(a && a->joint && a->joint->type==JT_tau, "this configuration does not have a tau DOF");
      J.elem(0, a->joint->qIndex) += 1e-1;
    }
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
  if(gravity) fs().setGravity(); else fs().setGravity(0.);
  fs().equationOfMotion(M, F, qdot);
}

/** @brief return the joint accelerations \f$\ddot q\f$ given the
  joint torques \f$\tau\f$ (computed via Featherstone's Articulated Body Algorithm in O(n)) */
void Configuration::fwdDynamics(arr& qdd, const arr& qd, const arr& tau, bool gravity) {
  fs().update();
  if(gravity) fs().setGravity(); else fs().setGravity(0.);
  //  cout <<tree <<endl;
//  fs().fwdDynamics_MF(qdd, qd, tau);
    fs().fwdDynamics_aba_1D(qdd, qd, tau); //works
  //  fwdDynamics_aba_nD(qdd, tree, qd, tau); //does not work
}

/** @brief return the necessary joint torques \f$\tau\f$ to achieve joint accelerations
  \f$\ddot q\f$ (computed via the Recursive Newton-Euler Algorithm in O(n)) */
void Configuration::inverseDynamics(arr& tau, const arr& qd, const arr& qdd, bool gravity) {
  fs().update();
  if(gravity) fs().setGravity(); else fs().setGravity(0.);
#if 1
  fs().invDynamics(tau, qd, qdd);
#else
  arr M, F;
  fs().equationOfMotion(M, F, qdd);
  tau = M * qdd + F;
#endif
}

/*void Configuration::impulsePropagation(arr& qd1, const arr& qd0){
  static Array<Featherstone::Link> tree;
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  mimickImpulsePropagation(tree);
  Featherstone::RF_abd(qdd, tree, qd, tau);
}*/

/// return a OpenGL extension
std::shared_ptr<ConfigurationViewer>& Configuration::get_viewer(const char* window_title, bool offscreen) {
  if(!self->viewer) {
    self->viewer = make_shared<ConfigurationViewer>();
  }
  return self->viewer;
}

OpenGL& Configuration::gl() {
  return get_viewer()->ensure_gl();
}

void Configuration::view_lock(const char* _lockInfo) {
  if(self->viewer) {
    gl().dataLock.lock(_lockInfo);
  }
}

void Configuration::view_unlock() {
  if(self->viewer) {
    gl().dataLock.unlock();
  }
}

/// return a Swift extension
/*
std::shared_ptr<SwiftInterface> Configuration::swift() {
  if(self->swift && self->swift->swiftID.N != frames.N) self->swift.reset();
  if(!self->swift){
    self->swift = make_shared<SwiftInterface>(frames, .1, 0);
    self->swift->deactivate(getCollisionExcludeIDs());
    self->swift->deactivatePairs(getCollisionExcludePairIDs());
  }
  return self->swift;
}
*/

std::shared_ptr<FclInterface> Configuration::fcl(int verbose) {
  if(!self->fcl) {
    Array<Shape*> geometries(frames.N);
    Array<Shape*>::memMove=1;
    geometries.setZero();
    for(Frame* f:frames) {
      if(f->shape && f->shape->cont) {
        CHECK(f->shape->type()!=rai::ST_marker, "collision object can't be a marker");
        if(!f->shape->mesh().V.N) f->shape->createMeshes();
        CHECK(f->shape->mesh().V.N, "collision object with no vertices");
        geometries(f->ID) = f->shape;
        if(verbose>0) LOG(0) <<"  adding to FCL interface: " <<f->name;
      } else {
        if(verbose>0) LOG(0) <<"  SKIPPING from FCL interface: " <<f->name;
      }
    }
    self->fcl = make_shared<FclInterface>(geometries, getCollisionExcludePairIDs(), FclInterface::_broadPhaseOnly); //-1.=broadphase only -> many proxies, 0.=binary, .1=exact margin (slow)
  }
  return self->fcl;
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

bool Configuration::hasView() {
  return !!self->viewer;
}

int Configuration::view(bool pause, const char* txt) {
//  gl()->resetPressedKey();
  for(Frame* f:frames) f->ensure_X();
  int key = get_viewer()->updateConfiguration(*this).view(pause, txt);
//  if(pause) {
//    if(!txt) txt="Config::watch";
//    key = watch(true, txt);
//  } else {
//    key = watch(false, txt, true);
//  }
  return key;
}

void Configuration::view_close() {
  if(self && self->viewer) self->viewer.reset();
}

void Configuration::set_viewer(std::shared_ptr<ConfigurationViewer>& _viewer){
  self->viewer = _viewer;
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
  //-- copy them into proxies
  uint j = proxies.N;
  proxies.resizeCopy(j+collisionPairs.d0);
  for(uint i=0; i<collisionPairs.d0; i++) {
    uint a = collisionPairs.elem(2*i);
    uint b = collisionPairs.elem(2*i+1);
    if(a<b) { uint z=a; a=b; b=z; }
    rai::Frame* f1 = frames.elem(a);
    rai::Frame* f2 = frames.elem(b);
#if 0
    bool canCollide = f1->shape->canCollideWith(f2);
    if(!canCollide) {
      LOG(0) <<"you should not be here! filtering out: " <<f1->ID <<'.' <<f1->name  <<' ' <<f2->ID <<'.' <<f2->name;
      continue;
    }
#endif
    Proxy& p = proxies.elem(j);
    p.a = f1;
    p.b = f2;
    p.d = -0.;
    p.posA = f1->getPosition();
    p.posB = f2->getPosition();
    j++;
  }
}

/*
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
*/

void Configuration::stepFcl() {
  //-- get the frame state of collision objects
#if 0
  arr X = getFrameState();
#else
  static arr X;
  X.resize(frames.N, 7).setZero();
  for(uint i=0; i<X.d0; i++) {
    rai::Frame* f = frames.elem(i);
    if(f->shape && f->shape->cont) X[i] = f->ensure_X().getArr7d();
  }
#endif
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

  auto eqn = [&](const arr& x) -> arr {
    setJointState(x[0]);
    arr M, Minv, F;
    equationOfMotion(M, F, x[1], gravity);
    inverse_SymPosDef(Minv, M);
    //Minv = inverse(M); //TODO why does symPosDef fail?
    arr y = Minv * (Bu_control - F);
    return y;
  };

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
  rk4_2ndOrder(x1, (q, qdot).reshape(2, q.N), eqn, tau);
  if(dynamicNoise) rndGauss(x1[1].noconst(), ::sqrt(tau)*dynamicNoise, true);
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
void Configuration::write(std::ostream& os, bool explicitlySorted) const {
#if 0
  for(Frame* f: frames) if(!f->name.N) f->name <<'_' <<f->ID;
  if(!explicitlySorted) {
    for(Frame* f: frames) f->write(os);
  } else {
    FrameL sorted = calc_topSort();
    for(Frame* f: sorted) f->write(os);
  }
  os <<endl;
#else
  Graph G;
  write(G);
  G.write(os, "\n", 0, -1, true);
#endif
}

void Configuration::write(Graph& G) const {
  for(Frame* f: frames) if(!f->name.N) f->name <<'_' <<f->ID;
  for(Frame* f: frames) f->write(G.addSubgraph(f->name));
  for(uint i=0; i<frames.N; i++) if(frames.elem(i)->parent) {
      G.elem(i)->addParent(G.elem(frames.elem(i)->parent->ID));
    }
}

/// write a URDF file
void Configuration::writeURDF(std::ostream& os, const char* robotName) const {
  os <<"<?xml version=\"1.0\"?>\n";
  os <<"<robot name=\"" <<robotName <<"\">\n";

  //-- write base_link first

  FrameL bases;
  for(Frame* a:frames) { if(!a->parent) a->getRigidSubFrames(bases, false); }
  os <<"<link name=\"base_link\">\n";
  for(Frame* a:frames) {
    if(a->shape && a->shape->type()!=ST_mesh && a->shape->type()!=ST_marker) {
      os <<"  <visual>\n    <geometry>\n";
      arr& size = a->shape->size;
      switch(a->shape->type()) {
        case ST_box:       os <<"      <box size=\"" <<size({0, 2}) <<"\" />\n";  break;
        case ST_cylinder:  os <<"      <cylinder length=\"" <<size.elem(-2) <<"\" radius=\"" <<size.elem(-1) <<"\" />\n";  break;
        case ST_sphere:    os <<"      <sphere radius=\"" <<size.last() <<"\" />\n";  break;
        case ST_mesh:      os <<"      <mesh filename=\"" <<a->ats->get<FileToken>("mesh").name <<'"';
          if((*a->ats)["meshscale"]) os <<" scale=\"" <<a->ats->get<arr>("meshscale") <<'"';
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
      a->getRigidSubFrames(shapes, false);
      for(Frame* b:shapes) {
        if(b->shape && b->shape->type()!=ST_mesh && b->shape->type()!=ST_marker) {
          os <<"  <visual>\n    <geometry>\n";
          arr& size = b->shape->size;
          switch(b->shape->type()) {
            case ST_box:       os <<"      <box size=\"" <<size({0, 2}) <<"\" />\n";  break;
            case ST_cylinder:  os <<"      <cylinder length=\"" <<size.elem(-2) <<"\" radius=\"" <<size.elem(-1) <<"\" />\n";  break;
            case ST_sphere:    os <<"      <sphere radius=\"" <<size.last() <<"\" />\n";  break;
            case ST_mesh:      os <<"      <mesh filename=\"" <<b->ats->get<FileToken>("mesh").name <<'"';
              if((*b->ats)["meshscale"]) os <<" scale=\"" <<b->ats->get<arr>("meshscale") <<'"';
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
  // create two dummy materials, one transparent
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
  for(Frame* f:frames) {
    aiNode* node = new aiNode(f->name.p);
    nodes(f->ID) = node;
    // create a mesh?
    if(f->shape && f->shape->type()!=ST_marker) {
      aiMesh* mesh = scene.mMeshes[n_meshes] = new aiMesh();
      Mesh& M = f->shape->mesh();
      buildAiMesh(M, mesh);
      if(f->shape->alpha()==1.)
        mesh->mMaterialIndex = 0;
      else
        mesh->mMaterialIndex = 1;
      //associate with node
      node->mMeshes = new unsigned[1];
      node->mNumMeshes = 1;
      node->mMeshes[0] = n_meshes;
      n_meshes++;
    } else {
      node->mMeshes = 0;
      node->mNumMeshes = 0;
    }
    // add mass?
    if(f->inertia) {
      node->mMetaData = new aiMetadata();
      node->mMetaData->Add<double>("mass", f->inertia->mass);
    }
    if(f->parent) f->get_Q().getAffineMatrix(T.p);
    else f->get_X().getAffineMatrix(T.p);
    for(uint j=0; j<4; j++) for(uint k=0; k<4; k++) {
        node->mTransformation[j][k] = T(j, k);
      }
  }
  //connect parent/children
  //count roots
  uint n_roots=0;
  for(Frame* f:frames) if(!f->parent) n_roots++;
  scene.mRootNode->mChildren = new aiNode* [n_roots];
  scene.mRootNode->mNumChildren = n_roots;
  n_roots=0;
  for(Frame* f:frames) {
    aiNode* node = nodes(f->ID);
    if(f->parent) {
      node->mParent = nodes(f->parent->ID);
    } else {
      node->mParent = scene.mRootNode;
      scene.mRootNode->mChildren[n_roots++] = node;
    }
    node->mChildren = new aiNode* [f->children.N];
    node->mNumChildren = f->children.N;
    for(uint i=0; i<f->children.N; i++) {
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
        (f->shape->type()==ST_mesh || f->shape->type()==ST_ssCvx || f->shape->type()==ST_sdf)) {
      String filename = pathPrefix;
      if(!f->ats) f->ats = make_shared<Graph>();
      filename <<f->name <<".mesh";
      f->ats->getNew<FileToken>("mesh").name = filename;
      if(f->shape->type()==ST_mesh || f->shape->type()==ST_sdf) f->shape->mesh().writeArr(FILE(filename));
      else if(f->shape->type()==ST_ssCvx) f->shape->sscCore().writeArr(FILE(filename));
      else if(f->shape->_sdf) {
        filename.clear() <<pathPrefix <<f->name <<".vol";
        f->ats->getNew<FileToken>("sdf").name = filename;
        f->shape->_sdf->write(FILE(filename));
      }
    }
  }
}

/// write meshes into a single ply file
void Configuration::writeMesh(const char* filename) const {
  rai::Mesh all;
  for(Frame* f:frames) {
    if(f->shape && f->shape->_mesh) {
      all.addMesh(f->shape->mesh(), f->ensure_X());
    }
  }
  all.writePLY(filename);
//  writeAssimp(all, filename, "ply");
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
  for(Frame* f: frames) G.add<bool>({STRING(f->name <<" [" <<f->ID <<']')}, {});
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

    ats.newNode<Transformation>({"X"}, f->X);

    if(f->shape) {
      ats.newNode<int>({"shape"}, f->shape->type);
    }

    if(f->link) {
      G.elem(f->ID)->addParent(G.elem(f->link->from->ID));
      if(f->link->joint) {
        ats.newNode<int>({"joint"}, f->joint()->type);
      } else {
        ats.newNode<Transformation>({"Q"}, f->link->Q);
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
//  for(Dof* j:activeJoints) if(j->uncertainty) nUc++;

  os <<"Configuration: q.N=" <<getJointStateDimension()
     <<" #frames=" <<frames.N
     <<" #dofs=" <<activeDofs.N
     <<" #shapes=" <<nShapes
     <<" #ucertainties=" <<nUc
     <<" #proxies=" <<proxies.N
     <<" #forces=" <<otherDofs.N
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
  Node* qAngles=0;

  //-- each node becomes a frame
  for(Node* n: G) {
    if(!n->is<Graph>()) {
      CHECK_EQ(n->key, "q", "only non-graph node is q:[joint angles]!");
      qAngles=n;
      continue;
    }
    //    CHECK_EQ(n->keys(0),"frame","");
    CHECK_LE(n->parents.N, 2, "frames must have no or one parent: specs=" <<*n <<' ' <<n->index);

    Frame* f = new Frame(*this);
    f->name=n->key;

    node2frame(n->index) = f;
  }

  //-- post-process: check for parents
  for(Node* n: G) {
    Frame* f = node2frame(n->index);
    if(!f) continue;

    if(n->parents.N>=1) {
      Frame* p = node2frame(n->parents(0)->index);
      CHECK(p, "parent frame '" <<n->parents(0)->key <<"' does not yet exist - is graph DAG?");
      f->setParent(p);

      if(n->parents.N==2) { //this is an inserted joint -> 2 frames (pre-joint and joint)
        Frame* post   = node2frame(n->parents(1)->index);
        CHECK(post, "JOINT: to '" <<n->parents(1)->key <<"' does not exist ["<<*n <<"]");
        CHECK(!p->isChildOf(post, INT32_MAX), "you can't insert joint in loops!");

        //connect the new frame and optionally impose the post node relative transform
        post->setParent(f, false);
      }
    }

    f->ats = make_shared<Graph>();
    f->ats->copy(n->graph(), false, true);
    f->read(n->graph());
  }

  // mimic joints: if the joint is coupled to another:
  {
    Joint* j;
    for(Frame* f: frames) if((j=f->joint) && j->mimic==(Joint*)1) {
        Node* mim = (*f->ats)["mimic"];
        String jointName;
        if(mim->is<String>()) jointName = mim->as<String>();
        else if(mim->is<NodeL>()) {
          NodeL nodes = mim->as<NodeL>();
          jointName = nodes.scalar()->key;
        } else {
          HALT("could not retrieve minimick frame for joint '" <<f->name <<"' from ats '" <<f->ats <<"'");
        }
        Frame* mimicFrame = getFrame(jointName, true, true);
        CHECK(mimicFrame, "the argument to 'mimic', '" <<jointName <<"' is not a frame name");
        j->mimic=0; //UNDO the =(Joint*)1
        j->setMimic(mimicFrame->joint);
        if(!j->mimic) HALT("The joint '" <<*j <<"' is declared mimicking '" <<jointName <<"' -- but that doesn't exist!");
        j->type = j->mimic->type;
        j->q0 = j->mimic->q0;
        j->active = j->mimic->active;
        j->setDofs(j->q0, 0);

        delete mim;
        f->ats->index();
      }
  }

  // NodeL ucs = G.getNodesWithTag("%Uncertainty");
  // ucs.append(G.getNodes("Uncertainty"));
  // for(Node* n: ucs) {
  //   CHECK_EQ(n->parents.N, 1, "Uncertainties must have one parent");
  //   CHECK(n->isGraph(), "Uncertainties must have value Graph");
  //   CHECK(n->key=="Uncertainty" || n->graph().findNode("%Uncertainty"), "");

  //   Frame* f = getFrame(n->parents(0)->key);
  //   CHECK(f, "");
  //   Joint* j = f->joint;
  //   CHECK(j, "Uncertainty parent must be a joint");
  //   Uncertainty* uc = new Uncertainty(j);
  //   uc->read(n->graph());
  // }

  //-- clean up the graph
//  calc_q();
//  calc_fwdPropagateFrames();

  if(qAngles) {
    setJointState(qAngles->as<arr>());
  }

  checkConsistency();
}

/// dump the list of current proximities on the screen
void Configuration::reportProxies(std::ostream& os, double belowMargin, bool brief) const {
  CHECK(_state_proxies_isGood, "");

  os <<"Proximity report: #" <<proxies.N <<endl;
  uint i=0;
  double pen = 0.;
  for(const Proxy& p: proxies) {
    if(p.d>belowMargin) continue;
    if(p.d<0.) pen -= p.d;
    os <<"  " <<i++;
    p.write(os, brief);
    os <<endl;
  }
  cout <<"  TOTAL PENETRATION: " <<pen <<endl;
  os <<"ForceExchange report:" <<endl;
  for(Frame* a:frames) for(ForceExchange* c:a->forces) {
      if(&c->a==a) {
        c->coll();
        os <<*c <<endl;
      }
    }
}

void Configuration::reportLimits(std::ostream& os) const {
  os <<"Limits report:" <<endl;
  for(Dof* d:activeDofs) {
    if(d->limits.N) {
      arr q = d->calcDofsFromConfig();
      arr l = d->limits;
      bool good=true;
//      if(d->dim>1) {
        l = ~l.reshape(-1, 2);
        good = boundCheck(q, l);
//      } else {
//        good = boundCheck(q, l({0, 0}), l({1, 1}));
//      }
      if(!good) LOG(0) <<d->name() <<" violates limits";
    }
  }
}

bool ProxySortComp(const Proxy* a, const Proxy* b) {
  return (a->a < b->a) || (a->a==b->a && a->b<b->b) || (a->a==b->a && a->b==b->b && a->d < b->d);
}

void Configuration::kinematicsPenetration(arr& y, arr& J, const Proxy& p, double margin, bool addValues) const {
  CHECK(p.a->shape, "");
  CHECK(p.b->shape, "");

  //early check: if estimate is way out of collision, don't bother computing it precise
  if(p.d > p.a->shape->radius() + p.b->shape->radius() + .01 + margin) return;

  if(!p.collision)((Proxy*)&p)->calc_coll();

  if(p.collision->getDistance()>margin) return;

  arr Jp1, Jp2;
  jacobian_pos(Jp1, p.a, p.collision->p1);
  jacobian_pos(Jp2, p.b, p.collision->p2);

  arr y_dist, J_dist;
  p.collision->kinDistance(y_dist, J_dist, Jp1, Jp2);

  if(y_dist.scalar()>margin) return;
  if(addValues) {
    y += margin-y_dist.scalar();
    J -= J_dist;
  } else {
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
void Configuration::setJacModeAs(const arr& J) {
  if(!isSpecial(J)) jacMode = JM_dense;
  else if(isSparse(J)) jacMode = JM_sparse;
  else if(isNoArr(J)) jacMode = JM_noArr;
  else if(isEmptyShape(J)) jacMode = JM_emptyShape;
  else NIY;
}

/// return a feature for a given frame(s)
std::shared_ptr<Feature> Configuration::feature(FeatureSymbol fs, const StringA& frames, const arr& scale, const arr& target, int order) const {
  return symbols2feature(fs, frames, *this, scale, target, order);
}

arr Configuration::eval(FeatureSymbol fs, const StringA& frames, const arr& scale, const arr& target, int order) {
  ensure_q();
  std::shared_ptr<Feature> f = feature(fs, frames, scale, target, order);
  return f->eval(getFrames(f->frameIDs));
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
  G.view(true, tag);
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

void Configuration::animateSpline(uint T) {
  arr x0 = getJointState();
  arr X = rand(T+2, x0.N);
  arr bounds = getJointLimits();
  X = X%(bounds[1]-bounds[0]) + repmat(~bounds[0], T+2, 1);
  X[0] = x0;
  X[-1] = x0;
  rai::BSpline S;
  S.set(2, X, grid(1, 0., double(T+1), T+1));
  double tau = .02;
  for(double t=0.; t<=T+1; t+=tau) {
    setJointState(S.eval(t));
    view();
    rai::wait(tau);
  }
}

int Configuration::animate(Inotify* ino) {
  arr x, x0;
  x0 = getJointState();
  arr lim = getJointLimits();
  const int steps = 50;
  checkConsistency();
  StringA jointNames = getJointNames();

  get_viewer()->raiseWindow();
  get_viewer()->_resetPressedKey();
  for(uint i=x0.N; i--;) {
    x=x0;
    double upper_lim = lim(1, i);
    double lower_lim = lim(0, i);
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
      setJointState(x);
      int key = view(false, STRING("DOF = " <<i <<" : " <<jointNames(i) <<" [" <<lower_lim <<", " <<upper_lim <<']'));

      if(key){ //==13 || key==27 || key=='q') {
        setJointState(x0);
        return key;
      }
      wait(0.01);
    }
  }
  setJointState(x0);
  return view(true);
}

Frame* movingBody=nullptr;
Vector selpos;
double seld, selx, sely, selz;

struct EditConfigurationHoverCall:OpenGL::GLHoverCall {
  Configuration* ors;
  EditConfigurationHoverCall(Configuration& _ors);// { ors=&_ors; }
  bool hoverCallback(OpenGL& gl) {
    //    if(!movingBody) return false;
    if(!movingBody) {
      Joint* j=nullptr;
      Frame* s=nullptr;
      timerStart(true);
      NIY; //gl.Select(true);
//      OpenGL::GLSelect* top=gl.topSelection;
      uint i=gl.selectID;
      if(!i) return false;
//      cout <<timerRead() <<"HOVER call: id = 0x" <<std::hex <<gl.topSelection->name <<endl;
      if((i&3)==1) s=ors->frames.elem(i>>2);
      if((i&3)==2) j=ors->frames.elem(i>>2)->joint;
//      gl.text.clear();
      if(s) {
        NIY; //gl.text <<"shape selection: body=" <<s->name <<" X=" <<s->ensure_X();
      }
      if(j) {
        NIY;
        /*
        gl.text
            <<"edge selection: " <<j->from()->name <<' ' <<j->frame->name
            //           <<"\nA=" <<j->A <<"\nQ=" <<j->Q <<"\nB=" <<j->B
            <<endl;
            */
        //        listWrite(j->ats, gl.text, "\n");
      }
    } else {
      //gl.Select();
      //double x=0, y=0, z=seld;
      //double x=(double)gl.mouseposx/gl.width(), y=(double)gl.mouseposy/gl.height(), z=seld;
      double x=gl.mouseposx, y=gl.mouseposy, z=seld;
      NIY; //      gl.unproject(x, y, z, true);
      cout <<"x=" <<x <<" y=" <<y <<" z=" <<z <<" d=" <<seld <<endl;
      movingBody->setPosition(selpos.getArr() + arr{x-selx, y-sely, z-selz});
    }
    return true;
  }
};

EditConfigurationHoverCall::EditConfigurationHoverCall(Configuration& _ors) {
  ors=&_ors;
}

struct EditConfigurationKeyCall:OpenGL::GLKeyCall {
  Configuration& C;
  EditConfigurationKeyCall(Configuration& _C): C(_C) {}
  bool keyCallback(OpenGL& gl) {
    if(!gl.keyIsDown) return true;
    switch(gl.pressedkey) {
        case '1':  gl.drawOptions.drawShapes^=1;  break;
        case '2':  gl.drawOptions.drawJoints^=1;  break;
        case '3':  gl.drawOptions.drawProxies^=1;  break;
        case '4':  gl.drawOptions.drawVisualsOnly^=1;  break;
        case '6':  gl.reportEvents^=1;  break;
        case '7':  gl.drawOptions.drawMode_idColor^=1; gl.drawOptions.drawColors^=1;  break;
//        case 'o':  gl.camera.X.pos += gl.camera.X.rot*Vector(0, 0, .1);  break;
//        case 'u':  gl.camera.X.pos -= gl.camera.X.rot*Vector(0, 0, .1);  break;
//        case 'k':  gl.camera.X.pos += gl.camera.X.rot*Vector(0, .1, 0);  break;
//        case 'i':  gl.camera.X.pos -= gl.camera.X.rot*Vector(0, .1, 0);  break;
//        case 'j':  gl.camera.X.pos += gl.camera.X.rot*Vector(.1, 0, 0);  break; //right
//        case 'l':  gl.camera.X.pos -= gl.camera.X.rot*Vector(.1, 0, 0);  break; //left
      }
    gl.postRedrawEvent(true);
    return true;
  }
};

void Configuration::watchFile(const char* filename) {
  checkConsistency();

  std::shared_ptr<ConfigurationViewer> V = get_viewer();

  //  gl.exitkeys="1234567890qhjklias, "; //TODO: move the key handling to the keyCall!
  //  gl.addHoverCall(new EditConfigurationHoverCall(K));
  EditConfigurationKeyCall key_callback(*this);
  V->ensure_gl().addKeyCall(&key_callback);
//  V->ensure_gl().addClickCall(new EditConfigurationClickCall(*this));
  V->ensure_gl().setTitle(STRING("ConfigView <" <<filename <<">"));
//  V->text = "waiting for file change ('h' for help)";
  //  gl()->ensure_gl().reportEvents=true;
  Inotify ino(filename);
  for(;;) {
    //-- LOADING
    LOG(0) <<"reloading `" <<filename <<"' ... ";
    {
      bool succ=true;
      FileToken file(filename, true);
      Graph G;
      try {
        lineCount=1;
        G.read(file);
        G.checkConsistency();
      } catch(std::runtime_error& err) {
        LOG(0) <<"g-File Synax Error line " <<lineCount <<": " <<err.what();
        succ=false;
      }

      if(succ) {
        try {
          Configuration C_tmp;
          C_tmp.readFromGraph(G);
          {
            V->ensure_gl().dataLock(RAI_HERE);
            copy(C_tmp, false);
          }
          report();
        } catch(std::runtime_error& err) {
          LOG(0) <<"Configuration initialization failed: " <<err.what();
          succ=false;
        }
      }
      file.cd_start(); //important: also on crash - cd back to original

      if(!succ) LOG(0) <<"file loading failed -- please check the file and re-save";
    }

    //-- WATCHING
    LOG(0) <<"watching...";
    V->updateConfiguration(*this, {}, true);
    V->_resetPressedKey();
    int key = V->view(false, "waiting for file change ('h' for help, 'q' to close)");
    for(;;) {
      key = V->gl->pressedkey;
//      V->_resetPressedKey();
      if(key==13 || key==27 || key=='q') break;
      if(!rai::getInteractivity()) break;
      if(key=='h') {
        V->text = "HELP:\n"
                             "RIGHT CLICK - set focus point (move view and set center of rotation)\n"
                             "LEFT CLICK - rotate (ball; or around z at view rim)\n"
                             "q - quit\n"
                             "[ENTER] - force reload\n"
                             "[SPACE] - write object info\n"
                             "SHIFT-LEFT CLICK - move view\n"
                             "a - animate\n"
                             "i - write info\n"
                             "c - compute and write collisions\n"
                             "s - simulate in PhysX\n"
                             "r - random sample a new configuration\n"
                             "x - export to multiple files (.g .urdf. ply. dae)\n"
                             "1..7 - view options\n"
                             "h - help";
        V->updateConfiguration(*this).view(false);
        cout <<V->text <<endl;
      } else if(key=='s') { //simulate
        rai::Simulation S(*this, S._physx, 2);
        S.loadTeleopCallbacks();

        double tau=.01;
        Metronome tic(tau);
        while(!S.teleopCallbacks->stop) {
          tic.waitForTic();
          S.step({}, tau, S._position);
          //C.ensure_proxies();
          //C.getTotalPenetration();
          //C.reportProxies();
          //C.view();
        }
        V->updateConfiguration(*this).view(false);
      } else if(key=='i') {
        LOG(0) <<"INFO:";
        report(cout);
        cout <<"joints: " <<getJointNames() <<endl;
        gl().camera.report(cout);
      } else if(key=='c') { //compute collisions
        ensure_proxies();
        double p = getTotalPenetration();
        double eps=.1;
        V->text.clear();
        reportProxies(V->text, eps, true);
        V->text <<"TOTAL PENETRATION: " <<p <<endl;
        V->updateConfiguration(*this).view(false);
#if 0
        FrameL collisionPairs = C.getCollisionAllPairs();
        //      cout <<" CollisionPairs:" <<endl;
        //      for(uint i=0;i<collisionPairs.d0;i++) cout <<collisionPairs(i,0)->name <<'-' <<collisionPairs(i,1)->name <<endl;
        auto coll = F_PairCollision().eval(collisionPairs);
        bool doesCollide=false;
        for(uint i=0; i<coll.y.N; i++) {
          if(coll.y.elem(i)>-eps) {
            LOG(-1) <<"in collision: " <<collisionPairs(i, 0)->name <<'-' <<collisionPairs(i, 1)->name <<' ' <<coll.y.elem(i);
            doesCollide=true;
          }
        }
#endif
      } else if(key==' ') { //grab a body
        OpenGL& gl = V->ensure_gl();
        V->renderUntil=_solid;
        V->renderFlatColors=true;
#if 0
        gl.beginContext();
        gl.Render(gl.width, gl.height, 0, true);
        gl.endContext();
#else
        gl.update(false, true);
#endif
        V->renderUntil=_all;
        V->renderFlatColors=false;
        write_ppm(gl.captureImage, "z.ppm");
        uint id = color2id(&gl.captureImage(gl.mouseposy, gl.mouseposx, 0));
        float d = gl.captureDepth(gl.mouseposy, gl.mouseposx);
        arr x = {double(gl.mouseposx), double(gl.mouseposy), d};
        if(d<.01 || d==1.) {
          cout <<"NO SELECTION: SELECTION DEPTH = " <<d <<' ' <<gl.camera.glConvertToTrueDepth(d) <<endl;
        } else {
          gl.camera.unproject_fromPixelsAndGLDepth(x, gl.width, gl.height);
        }
        cout <<"SELECTION id: " <<id <<" world coords:" <<x <<endl;
        if(id<frames.N) cout <<*frames.elem(id) <<endl;
      } else if(key=='r') { //random sample
        LOG(0) <<"setting random config";
        for(rai::Dof* d:activeDofs) d->sampleUniform=1.;
        setRandom();
        V->updateConfiguration(*this).view(false);
      } else if(key=='x') { //export
        LOG(0) <<"exporting";
        FILE("z.g") <<*this;
        writeURDF(FILE("z.urdf"));
        writeMesh("z.ply");
        writeCollada("z.dae");
      }else if(key=='a') {
        LOG(0) <<"animating..";
        //while(ino.pollForModification());
        key = animate(&ino);
      }else{
        if(key){
          V->text = "waiting for file change ('h' for help)";
        }
      }
      V->_resetPressedKey();

      if(ino.poll(false, true)) break;
      wait(.1);
    }

    //-- ANIMATING
    //if(key) cout <<"*** KEYout:" <<key <<endl;
    if(key==27 || key=='q') break;
    if(key==-1) continue;
    if(!getInteractivity()) break;
  }

  V->ensure_gl().keyCalls.remove(-1);
}

}//namespace

//===========================================================================
//
// template instantiations
//

#include "../Core/util.ipp"
template rai::Array<rai::Shape*>::Array(uint);
//template Shape* listFindByName(const Array<Shape*>&,const char*);
