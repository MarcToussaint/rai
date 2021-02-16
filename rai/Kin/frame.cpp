/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "frame.h"
#include "kin.h"
#include "uncertainty.h"
#include "forceExchange.h"
#include "../Geo/analyticShapes.h"
#include <climits>

#ifdef RAI_GL
#include "../Gui/opengl.h"
#endif

//===========================================================================

template<> const char* rai::Enum<rai::JointType>::names []= {
  "hingeX", "hingeY", "hingeZ", "transX", "transY", "transZ", "transXY", "trans3", "transXYPhi", "universal", "rigid", "quatBall", "phiTransXY", "XBall", "free", "tau", nullptr
};

template<> const char* rai::Enum<rai::BodyType>::names []= {
  "dynamic", "kinematic", "static", nullptr
};

rai::Transformation_Xtoken::~Transformation_Xtoken() { f._state_updateAfterTouchingX(); }
rai::Transformation_Qtoken::~Transformation_Qtoken() { f._state_updateAfterTouchingQ(); }

rai::Transformation* rai::Transformation_Xtoken::operator->() { f.ensure_X(); return &f.X; }
rai::Transformation* rai::Transformation_Qtoken::operator->() { return &f.Q; }
rai::Transformation& rai::Transformation_Xtoken::operator*() { f.ensure_X(); return f.X; }
rai::Transformation& rai::Transformation_Qtoken::operator*() { return f.Q; }

void rai::Transformation_Xtoken::operator=(const rai::Transformation& _X) { f.X=_X; }
void rai::Transformation_Qtoken::operator=(const rai::Transformation& _Q) { f.Q=_Q; }

//===========================================================================
//
// Frame
//

bool rai_Kin_frame_ignoreQuatNormalizationWarning = false;

rai::Frame::Frame(Configuration& _K, const Frame* copyFrame)
  : C(_K) {

  ID=C.frames.N;
  C.frames.append(this);
  if(copyFrame) {
    const Frame& f = *copyFrame;
    name=f.name; Q=f.Q; X=f.X; _state_X_isGood=f._state_X_isGood; tau=f.tau; ats=f.ats;
    //we cannot copy link! because we can't know if the frames already exist. Configuration::copy copies the rel's !!
    if(copyFrame->joint) new Joint(*this, copyFrame->joint);
    if(copyFrame->shape) new Shape(*this, copyFrame->shape);
    if(copyFrame->inertia) new Inertia(*this, copyFrame->inertia);
  }
}

rai::Frame::Frame(Frame* _parent)
  : Frame(_parent->C) {
  CHECK(_parent, "");
  _state_X_isGood=false;
  linkFrom(_parent, false);
}

rai::Frame::~Frame() {
  while(forces.N) delete forces.last();
  if(joint) delete joint;
  if(shape) delete shape;
  if(inertia) delete inertia;
  if(parent) unLink();
  while(children.N) children.last()->unLink();
  CHECK_EQ(this, C.frames.elem(ID), "")
  C.frames.remove(ID);
  listReindex(C.frames);
  C.reset_q();
}

void rai::Frame::calc_X_from_parent() {
  CHECK(parent, "");
  CHECK(parent->_state_X_isGood, "");

  tau = parent->tau;
  Transformation& from = parent->X;
  X = from;
  X.appendTransformation(Q);
  CHECK_EQ(X.pos.x, X.pos.x, "NAN transformation:" <<from <<'*' <<Q);
  if(joint) {
    Joint* j = joint;
    if(j->type==JT_hingeX || j->type==JT_transX || j->type==JT_XBall)  j->axis = from.rot.getX();
    if(j->type==JT_hingeY || j->type==JT_transY)  j->axis = from.rot.getY();
    if(j->type==JT_hingeZ || j->type==JT_transZ)  j->axis = from.rot.getZ();
    if(j->type==JT_transXYPhi)  j->axis = from.rot.getZ();
    if(j->type==JT_phiTransXY)  j->axis = from.rot.getZ();
  }

  _state_X_isGood=true;
  C._state_proxies_isGood = false;
}

void rai::Frame::calc_Q_from_parent(bool enforceWithinJoint) {
  CHECK(parent, "");
  CHECK(_state_X_isGood, "");

  Q.setDifference(parent->ensure_X(), X);
  if(joint && enforceWithinJoint) {
    arr q = joint->calc_q_from_Q(Q);
    joint->calc_Q_from_q(q, 0);
  }
  _state_updateAfterTouchingQ();
}

const rai::Transformation& rai::Frame::ensure_X() {
#if 0 //for testing loops
  {
    rai::Frame* f=parent;
    while(f) {
      CHECK(f!=this, "");
      f=f->parent;
    }
  }
#endif

  if(!_state_X_isGood) { if(parent) { parent->ensure_X(); calc_X_from_parent(); } }
  CHECK(_state_X_isGood, "");
  return X;
}

const rai::Transformation& rai::Frame::get_Q() const {
  return Q;
}

const rai::Transformation& rai::Frame::get_X() const {
  CHECK(_state_X_isGood, "");
  return X;
}

void rai::Frame::_state_updateAfterTouchingX() {
  _state_setXBadinBranch();
  _state_X_isGood = true;
  if(parent) {
    Q.setDifference(parent->ensure_X(), X);
    _state_updateAfterTouchingQ();
  }
}

void rai::Frame::_state_updateAfterTouchingQ() {
  CHECK(parent, "can't set Q for a root frame '" <<name <<"'");
  _state_setXBadinBranch();
  if(joint) C._state_q_isGood = false;
}

void rai::Frame::getRigidSubFrames(FrameL& F) {
  for(Frame* child:children)
    if(!child->joint) { F.append(child); child->getRigidSubFrames(F); }
}

void rai::Frame::getPartSubFrames(FrameL& F) {
  for(Frame* child:children)
    if(!child->joint || !child->joint->isPartBreak()) { F.append(child); child->getRigidSubFrames(F); }
}

void rai::Frame::getSubtree(FrameL& F) {
  for(Frame* child:children) { F.append(child); child->getSubtree(F); }
}

rai::Frame*rai::Frame::getRoot(){
  rai::Frame* f = this;
  while(f->parent) f = f->parent;
  return f;
}

FrameL rai::Frame::getPathToRoot() {
  FrameL pathToRoot;
  rai::Frame* f = this;
  while(f) {
    pathToRoot.prepend(f);
    f = f->parent;
  }
  return pathToRoot;
}

rai::Frame* rai::Frame::getUpwardLink(rai::Transformation& Qtotal, bool untilPartBreak) const {
  if(!!Qtotal) Qtotal.setZero();
  const Frame* f=this;
  while(f->parent) {
    if(!untilPartBreak) {
      if(f->joint) break;
    } else {
      if(f->joint && f->joint->isPartBreak()) break;
    }
    if(!!Qtotal) Qtotal = f->Q*Qtotal;
    f = f->parent;
  }
  return (Frame*)f;
}

rai::Frame* rai::Frame::getDownwardLink(bool untilPartBreak) const {
  const Frame* f=this;
  while(f->children.N) {
    Frame* ch = f->children.first();
    if(!untilPartBreak) {
      if(ch->joint) break;
    } else {
      if(ch->joint && ch->joint->isPartBreak()) break;
    }
    f = ch;
  }
  return (Frame*)f;
}

FrameL rai::Frame::getPathToUpwardLink(bool untilPartBreak) {
  FrameL pathToLink;
  rai::Frame* f = this;
  while(f) {
    pathToLink.prepend(f);
    if(!untilPartBreak) {
      if(f->joint) break;
    } else {
      if(f->joint && f->joint->isPartBreak()) break;
//          && (f->joint->type==JT_rigid || f->joint->type==JT_free) //!(f->joint->type>=JT_hingeX && f->joint->type<=JT_hingeZ)
//          && !f->joint->mimic) break;
    }
    f = f->parent;
  }
  return pathToLink;
}

rai::Shape& rai::Frame::getShape() {
  if(!shape) shape = new Shape(*this);
  return *shape;
}

rai::Inertia& rai::Frame::getInertia() {
  if(!inertia) inertia = new Inertia(*this);
  return *inertia;
}

const char* rai::Frame::isPart() {
  rai::String* p = ats.find<rai::String>("part");
  if(p) return p->p;
  return 0;
}

void rai::Frame::prefixSubtree(const char* prefix) {
  FrameL F = {this};
  getSubtree(F);
  for(auto* f:F) f->name.prepend(prefix);

}

void rai::Frame::_state_setXBadinBranch() {
  if(_state_X_isGood) { //no need to propagate to children if already bad
    _state_X_isGood=false;
    for(Frame* child:children) child->_state_setXBadinBranch();
  }
}

void rai::Frame::read(const Graph& ats) {
  //interpret some of the attributes
  Node* n;
  if((n=ats["X"])) {
    if(n->isOfType<String>()) set_X()->read(n->get<String>().resetIstream());
    else if(n->isOfType<arr>()) set_X()->set(n->get<arr>());
    else NIY;
    set_X()->rot.normalize();
  }
  if((n=ats["pose"])) {
    if(n->isOfType<String>()) set_X()->read(n->get<String>().resetIstream());
    else if(n->isOfType<arr>()) set_X()->set(n->get<arr>());
    else NIY;
    set_X()->rot.normalize();
  }
  if((n=ats["Q"])) {
    if(n->isOfType<String>()) set_Q()->read(n->get<String>().resetIstream());
    else if(n->isOfType<arr>()) set_Q()->set(n->get<arr>());
    else NIY;
    set_Q()->rot.normalize();
  }

  if(ats["type"]) ats["type"]->key = "shape"; //compatibility with old convention: 'body { type... }' generates shape

  if(ats["joint"]) {
    if(ats["B"]) { //there is an extra transform from the joint into this frame -> create an own joint frame
      Frame* f = new Frame(parent);
      f->name <<'|' <<name; //the joint frame is actually the link frame of all child frames
      f->ats.copy(ats, false, true);
      this->unLink();
      this->linkFrom(f, false);
      new Joint(*f);
      f->joint->read(ats);
    } else {
      new Joint(*this);
      joint->read(ats);
    }
  }
  if(ats["shape"] || ats["mesh"]) { shape = new Shape(*this); shape->read(ats); }
  if(ats["mass"]) { inertia = new Inertia(*this); inertia->read(ats); }

  if(ats["collisionCore"]) {
    arr core = ats.get<arr>("collisionCore");
    core.reshape(-1, 3);
    double r = ats.get<double>("collisionCore_radius");

    Shape* sh=0;
    if(!shape) {
      shape = new Shape(*this);
      sh = shape;
    } else {
      Frame* f = new Frame(this);
      sh = new Shape(*f);
    }
    sh->type() = rai::ST_ssCvx;
    sh->sscCore().V = core;
    sh->size = ARR(r);
    sh->mesh().C = ARR(1., 1., 0., .5);
    sh->mesh().setSSCvx(core, r);
  }
}

void rai::Frame::write(Graph& G) {
  if(parent) G.newNode<rai::String>({"parent"}, {}, parent->name);

  if(parent) {
    if(!Q.isZero()) G.newNode<arr>({"Q"}, {}, Q.getArr7d());
  } else {
    if(!X.isZero()) G.newNode<arr>({"X"}, {}, X.getArr7d());
  }

  if(joint) joint->write(G);
  if(shape) shape->write(G);
  if(inertia) inertia->write(G);

  StringA avoid = {"Q", "pose", "rel", "X", "from", "to", "q", "shape", "joint", "type", "color", "size", "contact", "mesh", "meshscale", "mass", "limits", "ctrl_H", "axis", "A", "B", "mimic"};
  for(Node* n : ats) {
    if(!n->key.startsWith("%") && !avoid.contains(n->key)) {
      n->newClone(G);
    }
  }
}

void rai::Frame::write(std::ostream& os) const {
  os <<name;

  if(parent) os <<" (" <<parent->name <<')';

  os <<" \t{ ";

  if(parent) {
    if(!Q.isZero()) os <<" Q:" <<Q;
  } else {
    if(!X.isZero()) os <<" X:" <<X;
  }
//  if(parent) os <<"parent:" <<parent->name;

  if(joint) joint->write(os);
  if(shape) shape->write(os);
  if(inertia) inertia->write(os);

  StringA avoid = {"Q", "pose", "rel", "X", "from", "to", "q", "shape", "joint", "type", "color", "size", "contact", "mesh", "meshscale", "mass", "limits", "ctrl_H", "axis", "A", "B", "mimic"};
  for(Node* n : ats) {
    if(!n->key.startsWith("%") && !avoid.contains(n->key)) os <<", " <<*n;
  }

  os <<" }\n";
  //  if(mass) os <<"mass:" <<mass <<' ';
  //  if(type!=BT_dynamic) os <<"dyntype:" <<(int)type <<' ';
  //  uint i; Node *a;
  //  for(Type *  a:  ats)
  //      if(a->keys(0)!="X" && a->keys(0)!="pose") os <<*a <<' ';
}

/************* USER INTERFACE **************/

rai::Frame& rai::Frame::setShape(rai::ShapeType shape, const arr& size) {
  getShape().type() = shape;
  getShape().size() = size;
  getShape().createMeshes();
  return *this;
}

rai::Frame& rai::Frame::setPose(const rai::Transformation& _X) {
  ensure_X();
  X = _X;
  _state_updateAfterTouchingX();
  return *this;
}

rai::Frame& rai::Frame::setPosition(const arr& pos) {
  ensure_X();
  X.pos.set(pos);
  _state_updateAfterTouchingX();
  return *this;
}

rai::Frame& rai::Frame::setQuaternion(const arr& quat) {
  ensure_X();
  X.rot.set(quat);
  X.rot.normalize();
  _state_updateAfterTouchingX();
  return *this;
}

rai::Frame& rai::Frame::setRelativePosition(const arr& pos) {
  CHECK(parent, "you cannot set relative position for a frame without parent");
  Q.pos.set(pos);
  _state_updateAfterTouchingQ();
  return *this;
}

rai::Frame& rai::Frame::setRelativeQuaternion(const arr& quat) {
  CHECK(parent, "you cannot set relative position for a frame without parent");
  Q.rot.set(quat);
  Q.rot.normalize();
  _state_updateAfterTouchingQ();
  return *this;
}

rai::Frame& rai::Frame::setPointCloud(const arr& points, const byteA& colors) {
  getShape().type() = ST_pointCloud;
  if(!points.N) {
    cerr <<"given point cloud has zero size" <<endl;
    return *this;
  }
  getShape().mesh().V.clear().operator=(points).reshape(-1, 3);
  if(colors.N) {
    getShape().mesh().C.clear().operator=(convert<double>(byteA(colors))/255.).reshape(-1, 3);
  }
  return *this;
}

rai::Frame& rai::Frame::setConvexMesh(const arr& points, const byteA& colors, double radius) {
  if(!radius) {
    getShape().type() = ST_mesh;
    getShape().mesh().V.clear().operator=(points).reshape(-1, 3);
    getShape().mesh().makeConvexHull();
    getShape().size.clear();
  } else {
    getShape().type() = ST_ssCvx;
    getShape().sscCore().V.clear().operator=(points).reshape(-1, 3);
    getShape().sscCore().makeConvexHull();
    getShape().mesh().setSSCvx(getShape().sscCore().V, radius);
    getShape().size = ARR(radius);
  }
  if(colors.N) {
    getShape().mesh().C.clear().operator=(convert<double>(byteA(colors))/255.).reshape(-1, 3);
  }
  return *this;
}

rai::Frame& rai::Frame::setColor(const arr& color) {
  getShape().mesh().C = color;
  return *this;
}

rai::Frame& rai::Frame::setJoint(rai::JointType jointType) {
  if(joint) { delete joint; joint=nullptr; }
  if(jointType != JT_none) {
    new Joint(*this, jointType);
  }
  if(jointType == JT_free) { joint->limits = {-10.,10,-10,10,-10,10, -1.,1,-1,1,-1,1,-1,1}; }
  return *this;
}

rai::Frame& rai::Frame::setContact(int cont) {
  getShape().cont = cont;
  return *this;
}

rai::Frame& rai::Frame::setMass(double mass) {
  if(mass<0.) {
    if(inertia) delete inertia;
  } else {
    getInertia().mass = mass;
  }
  return *this;
}

rai::Frame& rai::Frame::addAttribute(const char* key, double value) {
  ats.newNode<double>(key, {}, value);
  return *this;
}

rai::Frame& rai::Frame::setJointState(const arr& q) {
  CHECK(joint, "cannot setJointState for a non-joint");
  CHECK_EQ(q.N, joint->dim, "given q has wrong dimension");
  joint->calc_Q_from_q(arr{q}, 0);
  C._state_q_isGood = false;
  return *this;
}

arr rai::Frame::getSize() {
  return getShape().size;
}

arr rai::Frame::getMeshPoints() {
  return getShape().mesh().V;
}

uintA rai::Frame::getMeshTriangles() {
  return getShape().mesh().T;
}

arr rai::Frame::getMeshCorePoints() {
  return getShape().sscCore().V;
}

arr rai::Frame::getJointState() const {
  CHECK(joint, "cannot setJointState for a non-joint");
  return joint->calc_q_from_Q(Q);
}

/***********************************************************/

rai::Frame* rai::Frame::insertPreLink(const rai::Transformation& A) {
  //new frame between: parent -> f -> this
  Frame* f;

  if(parent) {
    f = new Frame(parent);
    parent->children.removeValue(this);
    f->name <<parent->name <<'>' <<name;
  } else {
    f = new Frame(C);
    f->name <<"NIL>" <<name;
  }
  parent=f;
  parent->children.append(this);

  if(!!A) f->Q=A; else f->Q.setZero();
  f->_state_updateAfterTouchingQ();

  return f;
}

rai::Frame* rai::Frame::insertPostLink(const rai::Transformation& B) {
  //new frame between: parent -> this -> f
  Frame* f = new Frame(C);
  if(name) f->name <<'<' <<name;

  //reconnect all outlinks from -> to
  f->children = children;
  for(Frame* b:children) b->parent = f;
  children.clear();

  if(!!B) f->Q=B; else f->Q.setZero();
  f->_state_updateAfterTouchingQ();

  f->linkFrom(this, false);

  return f;
}

void rai::Frame::unLink() {
  CHECK(parent, "");
  ensure_X();
  parent->children.removeValue(this);
  parent=nullptr;
  Q.setZero();
  if(joint) {  delete joint;  joint=nullptr;  }
}

void rai::Frame::linkFrom(rai::Frame* _parent, bool adoptRelTransform) {
  CHECK(_parent, "you need to set a parent to link from");
  CHECK(!parent, "this frame ('" <<name <<"') is already linked to a parent");
  if(parent==_parent) return;

  if(adoptRelTransform) ensure_X();

  parent=_parent;
  parent->children.append(this);

  if(adoptRelTransform) calc_Q_from_parent();
  _state_updateAfterTouchingQ();
}

bool rai::Frame::isChildOf(const rai::Frame* par, int order) const {
  Frame* p = parent;
  while(p) {
    if(p->joint) order--;
    if(order<0) return false;
    if(p==par) return true;
    p = p->parent;
  }
  return false;
}

rai::Joint::Joint(rai::Frame& f, rai::JointType _type) : Joint(f, (Joint*)nullptr) {
  CHECK(frame->parent || _type==JT_tau, "a frame without parent cannot be a joint");
  setType(_type);
}

rai::Joint::Joint(Frame& f, Joint* copyJoint)
  : frame(&f), qIndex(UINT_MAX) {
  CHECK(!frame->joint, "the Link already has a Joint");
  frame->joint = this;
  frame->C.reset_q();

  if(copyJoint) {
    qIndex=copyJoint->qIndex; dim=copyJoint->dim;
    type=copyJoint->type; axis=copyJoint->axis; limits=copyJoint->limits; q0=copyJoint->q0; H=copyJoint->H; scale=copyJoint->scale;
    active=copyJoint->active;

    if(copyJoint->mimic){
      setMimic(frame->C.frames.elem(copyJoint->mimic->frame->ID)->joint);
    }

    if(copyJoint->uncertainty) {
      new Uncertainty(this, copyJoint->uncertainty);
    }
  }
}

rai::Joint::Joint(Frame& from, Frame& f, Joint* copyJoint)
  : Joint(f, copyJoint) {
  frame->linkFrom(&from, false);
}

rai::Joint::~Joint() {
  frame->C.reset_q();
  frame->joint = nullptr;
  for(Joint *j:mimicers) j->mimic=0;
  if(mimic) mimic->mimicers.removeValue(this);
}

const rai::Transformation& rai::Joint::X() const {
  return frame->parent->get_X();
}

const rai::Transformation& rai::Joint::Q() const {
  return frame->get_Q();
}

void rai::Joint::setMimic(rai::Joint* j){
  if(!j){
    if(mimic) mimic->mimicers.removeValue(this);
    mimic=0;
  }else{
    CHECK(!mimic,"");
    mimic=j;
    mimic->mimicers.append(this);
  }
}

uint rai::Joint::qDim() {
  if(dim==UINT_MAX) dim=getDimFromType();
  return dim;
}

void rai::Joint::calc_Q_from_q(const arr& q_full, uint _qIndex) {
  if(type==JT_rigid) return;
  CHECK(dim!=UINT_MAX, "");
  CHECK_LE(_qIndex+dim, q_full.N, "");
  rai::Transformation& Q = frame->Q;
  Q.setZero();
  std::shared_ptr<arr> q_copy;
  double* qp;
  if(scale==1.) {
    qp = q_full.p + _qIndex;
  } else {
    q_copy = make_shared<arr>(dim);
    for(uint i=0; i<dim; i++) q_copy->elem(i) = q_full.elem(_qIndex+i);
    *q_copy *= scale;
    qp = q_copy->p;
  }
  if(mimic) {
    if(type!=JT_tau){
      Q = mimic->frame->get_Q();
    }else{
      frame->tau = mimic->frame->tau;
    }
  } else {
    switch(type) {
      case JT_hingeX: {
        Q.rot.setRadX(qp[0]);
      } break;

      case JT_hingeY: {
        Q.rot.setRadY(qp[0]);
      } break;

      case JT_hingeZ: {
        Q.rot.setRadZ(qp[0]);
      } break;

      case JT_universal: {
        rai::Quaternion rot1, rot2;
        rot1.setRadX(qp[0]);
        rot2.setRadY(qp[1]);
        Q.rot = rot1*rot2;
      } break;

      case JT_quatBall: {
        Q.rot.set(qp);
        {
          double n=Q.rot.normalization();
          if(!rai_Kin_frame_ignoreQuatNormalizationWarning) if(n<.1 || n>10.) LOG(-1) <<"quat normalization is extreme: " <<n;
        }
        Q.rot.normalize();
        Q.rot.isZero=false; //WHY? (gradient check fails without!)
      } break;

      case JT_free: {
        Q.pos.set(qp);
        Q.rot.set(qp+3);
        {
          double n=Q.rot.normalization();
          if(!rai_Kin_frame_ignoreQuatNormalizationWarning) if(n<.1 || n>10.) LOG(-1) <<"quat normalization is extreme: " <<n;
        }
        Q.rot.normalize();
        Q.rot.isZero=false;
      } break;

      case JT_XBall: {
        Q.pos.x = qp[0];
        Q.pos.y = 0.;
        Q.pos.z = 0.;
        Q.pos.isZero = false;
        Q.rot.set(qp+1);
        {
          double n=Q.rot.normalization();
          if(n<.1 || n>10.) LOG(-1) <<"quat normalization is extreme: " <<n;
        }
        Q.rot.normalize();
        Q.rot.isZero=false;
      } break;

      case JT_transX: {
        Q.pos = qp[0] * Vector_x;
      } break;

      case JT_transY: {
        Q.pos = qp[0] * Vector_y;
      } break;

      case JT_transZ: {
        Q.pos = qp[0] * Vector_z;
      } break;

      case JT_transXY: {
        Q.pos.set(qp[0], qp[1], 0.);
      } break;

      case JT_trans3: {
        Q.pos.set(qp);
      } break;

      case JT_transXYPhi: {
        Q.pos.set(qp[0], qp[1], 0.);
        Q.rot.setRadZ(qp[2]);
      } break;

      case JT_phiTransXY: {
        Q.rot.setRadZ(qp[0]);
        Q.pos = Q.rot*Vector(qp[1], qp[2], 0.);
      } break;

      case JT_rigid:
        break;

      case JT_tau:
        frame->tau = 1e-1 * qp[0];
        if(frame->tau<1e-10) frame->tau=1e-10;
        break;
      default: NIY;
    }
  }
  CHECK_EQ(Q.pos.x, Q.pos.x, "NAN transform");
  CHECK_EQ(Q.rot.w, Q.rot.w, "NAN transform");

  if(type!=JT_tau) {
    frame->_state_setXBadinBranch();
  }
  //    link->link = A * Q * B; //total rel transformation

  for(Joint* j:mimicers){
    if(type!=JT_tau){
      j->frame->Q = Q;
      j->frame->_state_setXBadinBranch();
    }else{
      j->frame->tau = frame->tau;
    }
  }
}

arr rai::Joint::calc_q_from_Q(const rai::Transformation& Q) const {
  arr q;
  switch(type) {
    case JT_hingeX:
    case JT_hingeY:
    case JT_hingeZ: {
      q.resize(1);
      //angle
      rai::Vector rotv;
      Q.rot.getRad(q(0), rotv);
      if(q(0)>RAI_PI) q(0)-=RAI_2PI;
      if(type==JT_hingeX && rotv*Vector_x<0.) q(0)=-q(0);
      if(type==JT_hingeY && rotv*Vector_y<0.) q(0)=-q(0);
      if(type==JT_hingeZ && rotv*Vector_z<0.) q(0)=-q(0);
    } break;

    case JT_universal: {
      q.resize(2);
      //angle
      if(fabs(Q.rot.w)>1e-15) {
        q(0) = 2.0 * atan(Q.rot.x/Q.rot.w);
        q(1) = 2.0 * atan(Q.rot.y/Q.rot.w);
      } else {
        q(0) = RAI_PI;
        q(1) = RAI_PI;
      }
    } break;

    case JT_quatBall: {
      q.resize(4);
      q(0)=Q.rot.w;
      q(1)=Q.rot.x;
      q(2)=Q.rot.y;
      q(3)=Q.rot.z;
    } break;

    case JT_transX: {
      q.resize(1);
      q(0)=Q.pos.x;
    } break;
    case JT_transY: {
      q.resize(1);
      q(0)=Q.pos.y;
    } break;
    case JT_transZ: {
      q.resize(1);
      q(0)=Q.pos.z;
    } break;
    case JT_transXY: {
      q.resize(2);
      q(0)=Q.pos.x;
      q(1)=Q.pos.y;
    } break;
    case JT_transXYPhi: {
      q.resize(3);
      q(0)=Q.pos.x;
      q(1)=Q.pos.y;
      rai::Vector rotv;
      Q.rot.getRad(q(2), rotv);
      if(q(2)>RAI_PI) q(2)-=RAI_2PI;
      if(rotv*Vector_z<0.) q(2)=-q(2);
    } break;
    case JT_phiTransXY: {
      q.resize(3);
      rai::Vector rotv;
      Q.rot.getRad(q(0), rotv);
      if(q(0)>RAI_PI) q(0)-=RAI_2PI;
      if(rotv*Vector_z<0.) q(0)=-q(0);
      rai::Vector relpos = Q.rot/Q.pos;
      q(1)=relpos.x;
      q(2)=relpos.y;
    } break;
    case JT_trans3: {
      q.resize(3);
      q(0)=Q.pos.x;
      q(1)=Q.pos.y;
      q(2)=Q.pos.z;
    } break;
    case JT_rigid:
      break;
    case JT_free:
      q.resize(7);
      q(0)=Q.pos.x;
      q(1)=Q.pos.y;
      q(2)=Q.pos.z;
      q(3)=Q.rot.w;
      q(4)=Q.rot.x;
      q(5)=Q.rot.y;
      q(6)=Q.rot.z;
      break;
    case JT_XBall:
      q.resize(5);
      q(0)=Q.pos.x;
      q(1)=Q.rot.w;
      q(2)=Q.rot.x;
      q(3)=Q.rot.y;
      q(4)=Q.rot.z;
      break;
    case JT_tau:
      q.resize(1);
      q(0) = 1e1 * frame->tau;
      break;
    default: NIY;
  }
  q /= scale;
  return q;
}

arr rai::Joint::getScrewMatrix() {
  CHECK(dim!=UINT_MAX, "");
  arr S(2, dim, 3);
  S.setZero();
  rai::Vector axis;

  if(type==JT_hingeX) {
    axis = X().rot.getX();
    S(0, 0, {}) = axis.getArr();
    S(1, 0, {}) = (-axis ^ X().pos).getArr();
  }
  if(type==JT_hingeY) {
    axis = X().rot.getY();
    S(0, 0, {}) = axis.getArr();
    S(1, 0, {}) = (-axis ^ X().pos).getArr();
  }
  if(type==JT_hingeZ) {
    axis = X().rot.getZ();
    S(0, 0, {}) = axis.getArr();
    S(1, 0, {}) = (-axis ^ X().pos).getArr();
  } else if(type==JT_transX) {
    axis = X().rot.getX();
    S(1, 0, {}) = axis.getArr();
  } else if(type==JT_transY) {
    axis = X().rot.getY();
    S(1, 0, {}) = axis.getArr();
  } else if(type==JT_transZ) {
    axis = X().rot.getZ();
    S(1, 0, {}) = axis.getArr();
  } else if(type==JT_transXY) {
    if(mimic) NIY;
    arr R = X().rot.getArr();
    S[1] = R({0, 1});
  } else if(type==JT_transXYPhi) {
    if(mimic) NIY;
    arr R = X().rot.getArr();
    axis = R[2];
    S(1, 0, {}) = R[0];
    S(1, 1, {}) = R[1];
    S(0, 2, {}) = axis.getArr();
    S(1, 2, {}) = (-axis ^ (X().pos + X().rot*Q().pos)).getArr();
  } else if(type==JT_phiTransXY) {
    if(mimic) NIY;
    axis = X().rot.getX();
    S(0, 0, {}) = axis.getArr();
    S(1, 0, {}) = (-axis ^ X().pos).getArr();
    arr R = (X().rot*Q().rot).getArr();
    S[1] = R({0, 1});
  }
  if(type==JT_trans3 || type==JT_free) {
    if(mimic) NIY;
    arr R = X().rot.getArr();
    S[1] = R;
  }
  if(type==JT_quatBall || type==JT_free) {
    uint offset=0;
    if(type==JT_free) offset=3;
    arr Jrot = X().rot.getArr() * Q().rot.getJacobian(); //transform w-vectors into world coordinate
    NIY; //Jrot /= sqrt(sumOfSqr( q({qIndex+offset, qIndex+offset+3}) )); //account for the potential non-normalization of q
    //    Jrot = crossProduct(Jrot, conv_vec2arr(pos_world-(X().pos+X().rot*Q().pos)) ); //cross-product of all 4 w-vectors with lever
    for(uint i=0; i<4; i++) for(uint k=0; k<3; k++) S(0, i+offset, k) = Jrot(k, i);
    Jrot = crossProduct(Jrot, conv_vec2arr(-(X().pos+X().rot*Q().pos)));  //cross-product of all 4 w-vectors with lever
    for(uint i=0; i<4; i++) for(uint k=0; k<3; k++) S(1, i+offset, k) = Jrot(k, i);
  }
  return S;
}

uint rai::Joint::getDimFromType() const {
  if(type>=JT_hingeX && type<=JT_transZ) return 1;
  if(type==JT_transXY) return 2;
  if(type==JT_transXYPhi) return 3;
  if(type==JT_phiTransXY) return 3;
  if(type==JT_trans3) return 3;
  if(type==JT_universal) return 2;
  if(type==JT_quatBall) return 4;
  if(type==JT_free) return 7;
  if(type==JT_rigid || type==JT_none) return 0;
  if(type==JT_XBall) return 5;
  if(type==JT_tau) return 1;
  HALT("shouldn't be here");
  return 0;
}

arr rai::Joint::get_h() const {
  arr h(6);
  h.setZero();
  switch(type) {
    case rai::JT_rigid:
    case rai::JT_transXYPhi: break;
    case rai::JT_hingeX: h.resize(6).setZero(); h(0)=1.; break;
    case rai::JT_hingeY: h.resize(6).setZero(); h(1)=1.; break;
    case rai::JT_hingeZ: h.resize(6).setZero(); h(2)=1.; break;
    case rai::JT_transX: h.resize(6).setZero(); h(3)=1.; break;
    case rai::JT_transY: h.resize(6).setZero(); h(4)=1.; break;
    case rai::JT_transZ: h.resize(6).setZero(); h(5)=1.; break;
    default: NIY;
  }
  return h;
}

double& rai::Joint::getQ() {
  return frame->C.q.elem(qIndex);
}

void rai::Joint::makeRigid() {
  setType(JT_rigid);
}

void rai::Joint::makeFree(double H_cost) {
  setType(JT_free);
  H=H_cost;
}

void rai::Joint::setType(rai::JointType _type) {
  if(type!=_type) {
    type = _type;
    dim = getDimFromType();
    frame->C.reset_q();
  }
}

void rai::Joint::flip() {
  frame->joint = 0;
  frame = frame->parent;
  CHECK(!frame->joint, "");
  frame->joint = this;
  frame->C.reset_q();
}

void rai::Joint::read(const Graph& G) {
  double d=0.;
  rai::String str;

  rai::Transformation A=0, B=0;

  G.get(A, "A");
  G.get(A, "from");
  if(G["BinvA"]) B.setInverse(A);
  G.get(B, "B");
  G.get(B, "to");

  //axis
  arr axis;
  if(G.get(axis, "axis")) {
    CHECK_EQ(axis.N, 3, "");
    Vector ax(axis);
    Transformation f;
    f.setZero();
    f.rot.setDiff(Vector_x, ax);
    A = A * f;
    B = -f * B;
  }

  if(!B.isZero()) {
    //new frame between: from -> f -> to
    CHECK_EQ(frame->children.N, 1, "");
    Frame* follow = frame->children.scalar();

    CHECK(follow->parent, "");
    CHECK(!follow->joint, "");
    follow->set_Q() = B;
  }

  if(!A.isZero()) {
    frame->insertPreLink(A);
  }

  Node* n;
  if((n=G["Q"])) {
    if(n->isOfType<String>()) frame->set_Q()->read(n->get<String>().resetIstream());
    else if(n->isOfType<arr>()) frame->set_Q()->set(n->get<arr>());
    else NIY;
    frame->set_Q()->rot.normalize();
  }
  G.get(H, "ctrl_H");
  G.get(scale, "joint_scale");
  if(G.get(d, "joint"))        type=(JointType)d;
  else if(G.get(str, "joint")) { str >>type; }
  else if(G.get(d, "type"))    type=(JointType)d;
  else if(G.get(str, "type"))  { str >>type; }
  else type=JT_rigid;

  dim = getDimFromType();

  if(G.get(d, "q")) {
    if(!dim) { //HACK convention
      frame->set_Q()->rot.setRad(d, 1., 0., 0.);
    } else {
      CHECK(dim!=UINT_MAX, "setting q (in config file) for 0-dim joint");
      CHECK(dim, "setting q (in config file) for 0-dim joint");
      q0 = consts<double>(d, dim);
      calc_Q_from_q(q0, 0);
    }
  } else if(G.get(q0, "q")) {
    CHECK_EQ(q0.N, dim, "given q (in config file) does not match dim");
    calc_Q_from_q(q0, 0);
  } else {
    //    link->Q.setZero();
    q0 = calc_q_from_Q(frame->Q);
  }

  //limit
  arr ctrl_limits;
  G.get(limits, "limits");
  if(limits.N && type!=JT_rigid && !mimic) {
    CHECK(limits.N>=2*qDim()/* || limits.N==2*qDim()+3*/, "parsed limits have wrong dimension: either lo-hi or lo-hi-vel-eff-acc PER DOF");
  }
  G.get(ctrl_limits, "ctrl_limits");
  if(ctrl_limits.N && type!=JT_rigid) {
    if(!limits.N) limits.resizeAs(ctrl_limits).setZero();
    CHECK_EQ(3, ctrl_limits.N, "parsed ctrl_limits have wrong dimension");
    limits.append(ctrl_limits);
  }

  //coupled to another joint requires post-processing by the Graph::read!!
  if(G["mimic"]) {
    mimic=(Joint*)1;
  }
}

void rai::Joint::write(Graph& g) {
  g.newNode<Enum<JointType>>({"joint"}, {}, type);
  if(H!=1.) g.newNode<double>({"ctrl_H"}, {}, H);
  if(scale!=1.) g.newNode<double>({"joint_scale"}, {}, scale);
  if(limits.N) g.newNode<arr>({"limits"}, {}, limits);
  if(mimic) g.newNode<rai::String>({"mimic"}, {}, STRING('(' <<mimic->frame->name <<')'));
}

void rai::Joint::write(std::ostream& os) const {
  os <<", joint:" <<type;
  if(H!=1.) os <<", ctrl_H:" <<H;
  if(scale!=1.) os <<", joint_scale:" <<scale;
  if(limits.N) os <<", limits:" <<limits;
  if(mimic) {
    os <<", mimic:(" <<mimic->frame->name <<')';
  }
}

//===========================================================================
//
// Shape
//

rai::Shape::Shape(Frame& f, const Shape* copyShape)
  : frame(f), _type(ST_none) {

  CHECK(!frame.shape, "this frame ('" <<frame.name <<"') already has a shape attached");
  frame.shape = this;
  if(copyShape) {
    const Shape& s = *copyShape;
    if(s._mesh) _mesh = s._mesh; //shallow shared_ptr copy!
    if(s._sscCore) _sscCore = s._sscCore; //shallow shared_ptr copy!
    _type = s._type;
    size = s.size;
    cont = s.cont;
  } else {
    mesh().C= {.8, .8, .8};
  }
}

rai::Shape::~Shape() {
  frame.shape = nullptr;
}

void rai::Shape::read(const Graph& ats) {

  {
    double d;
    arr x;
    rai::String str;
    rai::FileToken fil;

    ats.get(size, "size");
    if(ats.get(d, "shape"))        { type()=(ShapeType)(int)d;}
    else if(ats.get(str, "shape")) { str>> type(); }
    else if(ats.get(d, "type"))    { type()=(ShapeType)(int)d;}
    else if(ats.get(str, "type"))  { str>> type(); }
    if(ats.get(str, "mesh"))     { mesh().read(FILE(str), str.getLastN(3).p, str); }
    else if(ats.get(fil, "mesh"))     {
      fil.cd_file();
      mesh().read(fil.getIs(), fil.name.getLastN(3).p, fil.name);
//      cout <<"MESH: " <<mesh().V.dim() <<endl;
    }
    if(ats.get(fil, "texture"))     {
      fil.cd_file();
      read_png(mesh().texImg, fil.name, true);
//      cout <<"TEXTURE: " <<mesh().texImg.dim() <<endl;
    }
    if(ats.get(d, "meshscale"))  { mesh().scale(d); }
    if(ats.get(x, "meshscale"))  { mesh().scale(x(0), x(1), x(2)); }
    if(ats.get(mesh().C, "color")) {
      CHECK(mesh().C.N==3 || mesh().C.N==4, "color needs to be 3D or 4D (floats)");
    }

    if(mesh().V.N && type()==ST_none) type()=ST_mesh;

    //colored box?
    if(ats["coloredBox"]) {
      CHECK_EQ(mesh().V.d0, 8, "I need a box");
      arr col=mesh().C;
      mesh().C.resize(mesh().T.d0, 3);
      for(uint i=0; i<mesh().C.d0; i++) {
        if(i==2 || i==3) mesh().C[i] = col; //arr(color, 3);
        else if(i>=4 && i<=7) mesh().C[i] = 1.;
        else mesh().C[i] = .5;
      }
    }

    createMeshes();
  }

  if(ats["contact"]) {
    double d;
    if(ats.get(d, "contact")) cont = (char)d;
    else cont=1;
  }

  //center the mesh:
  if(type()==rai::ST_mesh && mesh().V.N) {
    if(ats["rel_includes_mesh_center"]) {
      mesh().center();
    }
    //    if(c.length()>1e-8 && !ats["rel_includes_mesh_center"]){
    //      frame.link->Q.addRelativeTranslation(c);
    //      frame.ats.newNode<bool>({"rel_includes_mesh_center"}, {}, true);
    //    }
  }

  //compute the bounding radius
//  if(mesh().V.N) mesh_radius = mesh().getRadius();
}

void rai::Shape::write(std::ostream& os) const {
  os <<", shape:" <<_type;
  if(_type!=ST_mesh) os <<", size:" <<size;

  Node* n;
  if((n=frame.ats["color"])) os <<", " <<*n;
  if((n=frame.ats["mesh"])) os <<", " <<*n;
  if((n=frame.ats["meshscale"])) os <<", " <<*n;
  if(cont) os <<", contact:" <<(int)cont;
}

void rai::Shape::write(Graph& g) {
  g.newNode<rai::Enum<ShapeType>>({"shape"}, {}, type());
  if(type()!=ST_mesh)
    g.newNode<arr>({"size"}, {}, size);
  if(mesh().C.N>0 && mesh().C.N<=4)
    g.newNode<arr>({"color"}, {}, mesh().C);
  if(cont) g.newNode<int>({"contact"}, {}, cont);
}

void rai::Shape::glDraw(OpenGL& gl) {
#ifdef RAI_GL
  //set name (for OpenGL selection)
  glPushName((frame.ID <<2) | 1);
  if(frame.C.orsDrawColors && !frame.C.orsDrawIndexColors && !gl.drawOptions.drawMode_idColor) {
    if(mesh().C.N) glColor(mesh().C); //color[0], color[1], color[2], color[3]*world.orsDrawAlpha);
    else   glColor(.5, .5, .5);
  }
  if(frame.C.orsDrawIndexColors) gl.drawId(frame.ID);

  double GLmatrix[16];
  frame.ensure_X().getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);

  if(!frame.C.orsDrawShapes) {
    double scale=.33*(size(0)+size(1)+size(2) + 2.*size(3)); //some scale
    if(!scale) scale=1.;
    scale*=.3;
    glDrawAxes(scale);
    glColor(0, 0, .5);
    glDrawSphere(.1*scale);
  }

  //default!
  if(frame.C.orsDrawShapes) {
    CHECK(_type!=rai::ST_none, "shape type is not initialized");

    if(_type==rai::ST_marker) {
      if(frame.C.orsDrawMarkers) {
        CHECK_GE(size.N, 1, "need a marker size");
        if(size(0)>0.){
          glDrawDiamond(size(0)/5., size(0)/5., size(0)/5.);
          glDrawAxes(size(0), !gl.drawOptions.drawMode_idColor);
        }else if(size(0)<0.){
          glDrawAxis(-size(0));
        }
      }
    } else {
      if(!mesh().V.N) {
        LOG(1) <<"trying to draw empty mesh";
      } else {
        mesh().glDraw(gl);
      }
    }
  }

  if(frame.C.orsDrawZlines) {
    glColor(0, .7, 0);
    glBegin(GL_LINES);
    glVertex3d(0., 0., 0.);
    glVertex3d(0., 0., -frame.ensure_X().pos.z);
    glEnd();
  }

  if(frame.C.orsDrawFrameNames) {
    glColor(1, 1, 1);
    glDrawText(frame.name, 0, 0, 0);
  }

  glPopName();
#endif
}

void rai::Shape::createMeshes() {
  //create mesh for basic shapes
  switch(_type) {
    case rai::ST_none: HALT("shapes should have a type - somehow wrong initialization..."); break;
    case rai::ST_box:
      mesh().clear();
      mesh().setBox();
      mesh().scale(size(0), size(1), size(2));
      break;
    case rai::ST_sphere: {
      sscCore().V = arr({1, 3}, {0., 0., 0.});
      double rad=1;
      if(size.N) rad=size(-1);
      mesh().setSSCvx(sscCore().V, rad);
    } break;
    case rai::ST_cylinder:
      CHECK(size(-1)>1e-10, "");
      mesh().setCylinder(size(-1), size(-2));
      break;
    case rai::ST_capsule:
      CHECK(size(-1)>1e-10, "");
      sscCore().V = arr({2, 3}, {0., 0., -.5*size(-2), 0., 0., .5*size(-2)});
      mesh().setSSCvx(sscCore().V, size(-1));
      break;
    case rai::ST_marker:
      break;
    case rai::ST_mesh:
    case rai::ST_pointCloud:
//      if(!mesh().V.N) LOG(-1) <<"mesh needs to be loaded";
      break;
    case rai::ST_ssCvx:
      CHECK(size(-1)>1e-10, "");
      if(!sscCore().V.N) {
        CHECK(mesh().V.N, "mesh or sscCore needs to be loaded");
        sscCore() = mesh();
      }
      mesh().setSSCvx(sscCore().V, size.last());
      break;
    case rai::ST_ssBox: {
      if(size(3)<1e-10) {
        sscCore().setBox();
        sscCore().scale(size(0), size(1), size(2));
        mesh() = sscCore();
        break;
      }
      double r = size(3);
      CHECK(size.N==4 && r>1e-10, "");
      for(uint i=0; i<3; i++) if(size(i)<2.*r) size(i) = 2.*r;
      sscCore().setBox();
      sscCore().scale(size(0)-2.*r, size(1)-2.*r, size(2)-2.*r);
      mesh().setSSBox(size(0), size(1), size(2), r);
      //      mesh().setSSCvx(sscCore, r);
    } break;
    case rai::ST_ssBoxElip: {
      CHECK_EQ(size.N, 7, "");
      double r = size(-1);
      for(uint i=0; i<3; i++) if(size(i)<2.*r) size(i) = 2.*r;
      rai::Mesh box;
      box.setBox();
      box.scale(size(0)-2.*r, size(1)-2.*r, size(2)-2.*r);
      rai::Mesh elip;
      elip.setSphere();
      elip.scale(size(3), size(4), size(5));
      sscCore().setSSCvx(MinkowskiSum(box.V, elip.V), 0);
      mesh().setSSCvx(sscCore().V, r);
    } break;
    default: {
      HALT("createMeshes not possible for shape type '" <<_type <<"'");
    }
  }
//  auto func = functional(false);
//  if(func){
//    mesh().setImplicitSurfaceBySphereProjection(*func, 2.);
//  }
}

shared_ptr<ScalarFunction> rai::Shape::functional(bool worldCoordinates){
  rai::Transformation pose = 0;
  if(worldCoordinates) pose = frame.ensure_X();
  //create mesh for basic shapes
  switch(_type) {
    case rai::ST_none: HALT("shapes should have a type - somehow wrong initialization..."); break;
    case rai::ST_box:
      return make_shared<DistanceFunction_ssBox>(pose, size(0), size(1), size(2), 0.);
    case rai::ST_sphere:
      return make_shared<DistanceFunction_Sphere>(pose, radius());
    case rai::ST_cylinder:
      return make_shared<DistanceFunction_Cylinder>(pose, size(-2), size(-1));
    case rai::ST_capsule:
      return make_shared<DistanceFunction_Capsule>(pose, size(-2), size(-1));
    case rai::ST_ssBox: {
      return make_shared<DistanceFunction_ssBox>(pose, size(0), size(1), size(2), size(3));
    default:
      return shared_ptr<ScalarFunction>();
    }
  }

}

rai::Inertia::Inertia(Frame& f, Inertia* copyInertia) : frame(f), type(BT_dynamic) {
  CHECK(!frame.inertia, "this frame ('" <<frame.name <<"') already has inertia");
  frame.inertia = this;
  if(copyInertia) {
    mass = copyInertia->mass;
    matrix = copyInertia->matrix;
    type = copyInertia->type;
    com = copyInertia->com;
  }
}

rai::Inertia::~Inertia() {
  frame.inertia = nullptr;
}

void rai::Inertia::defaultInertiaByShape() {
  CHECK(frame.shape, "");

  //add inertia to the body
  switch(frame.shape->type()) {
    case ST_sphere:   inertiaSphere(matrix.p(), mass, 1000., frame.shape->radius());  break;
    case ST_ssBox:
    case ST_box:      inertiaBox(matrix.p(), mass, 1000., frame.shape->size(0), frame.shape->size(1), frame.shape->size(2));  break;
    case ST_capsule:
    case ST_cylinder: inertiaCylinder(matrix.p(), mass, 1000., frame.shape->size(-2), frame.shape->size(-1));  break;
    default: HALT("not implemented for this shape type");
  }
}

arr rai::Inertia::getFrameRelativeWrench() {
  NIY;
  arr f(6);
//  rai::Vector fo = frame.ensure_X().rot/force;
//  rai::Vector to = frame.ensure_X().rot/(torque + ((frame.ensure_X().rot*com)^force));
//  f(0)=to.x;  f(1)=to.y;  f(2)=to.z;
//  f(3)=fo.x;  f(4)=fo.y;  f(5)=fo.z;
  return f;
}

void rai::Inertia::write(std::ostream& os) const {
  os <<", mass:" <<mass;
}

void rai::Inertia::write(Graph& g) {
  g.newNode<double>({"mass"}, {}, mass);
}

void rai::Inertia::read(const Graph& G) {
  double d;
  if(G.get(d, "mass")) {
    mass=d;
    matrix.setId();
    matrix *= .2*d;
    if(frame.shape) defaultInertiaByShape();
  }
  if(G["fixed"])       type=BT_static;
  if(G["static"])      type=BT_static;
  if(G["kinematic"])   type=BT_kinematic;
  if(G["dynamic"])     type=BT_dynamic;
  if(G.get(d, "dyntype")) type=(BodyType)d;
}

RUN_ON_INIT_BEGIN(frame)
FrameL::memMove=true;
JointL::memMove=true;
RUN_ON_INIT_END(frame)
