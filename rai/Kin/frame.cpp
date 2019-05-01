/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include <climits>
#include "frame.h"
#include "kin.h"
#include "uncertainty.h"
#include "contact.h"
#include "flag.h"

#ifdef RAI_GL
#include <Gui/opengl.h>
#endif

extern bool Geo_mesh_drawColors;

//===========================================================================

template<> const char* rai::Enum<rai::ShapeType>::names []= {
  "box", "sphere", "capsule", "mesh", "cylinder", "marker", "SSBox", "pointCloud", "ssCvx", "ssBox", NULL
};

template<> const char* rai::Enum<rai::JointType>::names []= {
  "hingeX", "hingeY", "hingeZ", "transX", "transY", "transZ", "transXY", "trans3", "transXYPhi", "universal", "rigid", "quatBall", "phiTransXY", "XBall", "free", "time", NULL
};

template<> const char* rai::Enum<rai::BodyType>::names []= {
  "dynamic", "kinematic", "static", NULL
};

//===========================================================================
//
// Frame
//

bool rai_Kin_frame_ignoreQuatNormalizationWarning = false;

rai::Frame::Frame(KinematicWorld& _K, const Frame* copyFrame)
  : K(_K) {
  
  ID=K.frames.N;
  K.frames.append(this);
  if(copyFrame) {
    const Frame& f = *copyFrame;
    name=f.name; Q=f.Q; X=f.X; tau=f.tau; ats=f.ats; active=f.active; flags=f.flags;
    //we cannot copy link! because we can't know if the frames already exist. KinematicWorld::copy copies the rel's !!
    if(copyFrame->joint) new Joint(*this, copyFrame->joint);
    if(copyFrame->shape) new Shape(*this, copyFrame->shape);
    if(copyFrame->inertia) new Inertia(*this, copyFrame->inertia);
  }
}

rai::Frame::Frame(Frame *_parent)
  : Frame(_parent->K) {
  CHECK(_parent, "");
  linkFrom(_parent);
}

rai::Frame::~Frame() {
  if(joint) delete joint;
  if(shape) delete shape;
  if(inertia) delete inertia;
  if(parent) unLink();
  while(contacts.N) delete contacts.last();
  while(parentOf.N) parentOf.last()->unLink();
//  K.frames.removeValue(this);
  CHECK_EQ(this, K.frames(ID), "")
  K.frames.remove(ID);
  listReindex(K.frames);
}

void rai::Frame::calc_X_from_parent() {
  CHECK(parent, "");
  tau = parent->tau;
  Transformation &from = parent->X;
  X = from;
  X.appendTransformation(Q);
  CHECK_EQ(X.pos.x, X.pos.x, "NAN transformation:" <<from <<'*' <<Q);
  if(joint) {
    Joint *j = joint;
    if(j->type==JT_hingeX || j->type==JT_transX || j->type==JT_XBall)  j->axis = from.rot.getX();
    if(j->type==JT_hingeY || j->type==JT_transY)  j->axis = from.rot.getY();
    if(j->type==JT_hingeZ || j->type==JT_transZ)  j->axis = from.rot.getZ();
    if(j->type==JT_transXYPhi)  j->axis = from.rot.getZ();
    if(j->type==JT_phiTransXY)  j->axis = from.rot.getZ();
  }
}

void rai::Frame::calc_Q_from_parent(bool enforceWithinJoint) {
  CHECK(parent,"");
  Q.setDifference(parent->X, X);
  if(joint && enforceWithinJoint) {
    arr q = joint->calc_q_from_Q(Q);
    joint->calc_Q_from_q(q, 0);
  }
}

void rai::Frame::getRigidSubFrames(FrameL &F) {
  for(Frame *f:parentOf) if(!f->joint) { F.append(f); f->getRigidSubFrames(F); }
}

rai::Shape& rai::Frame::getShape(){
  if(!shape) shape = new Shape(*this);
  return *shape;
}

rai::Inertia &rai::Frame::getInertia() {
  if(!inertia) inertia = new Inertia(*this);
  return *inertia;
}

rai::Frame *rai::Frame::getUpwardLink(rai::Transformation &Qtotal, bool untilRigid) const {
  if(!!Qtotal) Qtotal.setZero();
  const Frame *p=this;
  while(p->parent) {
    if(!untilRigid){
      if(p->joint) break;
    }else{
      if(p->joint && p->joint->getDimFromType()!=1 && !p->joint->mimic) break;
    }
    if(!!Qtotal) Qtotal = p->Q*Qtotal;
    p = p->parent;
  }
  return (Frame*)p;
}

void rai::Frame::read(const Graph& ats) {
  //interpret some of the attributes
  ats.get(X, "X");
  ats.get(X, "pose");
  ats.get(Q, "Q");
  
  if(ats["type"]) ats["type"]->keys.last() = "shape"; //compatibility with old convention: 'body { type... }' generates shape
  
  if(ats["joint"]) {
    if(ats["B"]){ //there is an extra transform from the joint into this frame -> create an own joint frame
      Frame *f=new Frame(parent);
      f->name <<'|' <<name; //the joint frame is actually the link frame of all child frames
      f->ats.copy(ats, false, true);
      this->unLink();
      this->linkFrom(f);
      new Joint(*f);
      f->joint->read(ats);
    }else{
      new Joint(*this);
      joint->read(ats);
    }
  }
  if(ats["shape"] || ats["mesh"]) { shape = new Shape(*this); shape->read(ats); }
  if(ats["mass"]) { inertia = new Inertia(*this); inertia->read(ats); }
}

void rai::Frame::write(Graph& G){
  if(parent) G.newNode<rai::String>({"parent"}, {}, parent->name);
  if(joint) joint->write(G);
  if(shape) shape->write(G);
  if(inertia) inertia->write(G);

  if(parent) {
    if(!Q.isZero()) G.newNode<arr>({"Q"}, {}, Q.getArr7d());
  } else {
    if(!X.isZero()) G.newNode<arr>({"X"}, {}, X.getArr7d());
  }

  for(Node *n : ats) {
    StringA avoid = {"Q", "pose", "rel", "X", "from", "to", "q", "shape", "joint", "type", "color", "size", "contact", "mesh", "meshscale", "mass", "limits", "ctrl_H", "axis", "A", "B", "mimic"};
    if(!avoid.contains(n->keys.last())){
      n->newClone(G);
    }
  }
}

void rai::Frame::write(std::ostream& os) const {
  os <<name;

  if(parent) os <<" (" <<parent->name <<')';

  os <<" \t{ ";

//  if(parent) os <<"parent:" <<parent->name;

  if(joint) joint->write(os);
  if(shape) shape->write(os);
  if(inertia) inertia->write(os);
  
  if(parent) {
    if(!Q.isZero()) os <<" Q:<" <<Q <<'>';
  } else {
    if(!X.isZero()) os <<" X:<" <<X <<'>';
  }
  
  if(flags) {
    Enum<FrameFlagType> fl;
    os <<" FLAGS:";
    for(int i=0;; i++) {
      fl.x = FrameFlagType(i);
      if(!fl.name()) break;
      if(flags & (1<<fl.x)) os <<' ' <<fl.name();
    }
  }
  
  for(Node *n : ats) {
    StringA avoid = {"Q", "pose", "rel", "X", "from", "to", "q", "shape", "joint", "type", "color", "size", "contact", "mesh", "meshscale", "mass", "limits", "ctrl_H", "axis", "A", "B", "mimic"};
    if(!avoid.contains(n->keys.last())) os <<' ' <<*n;
  }
  
  os <<" }\n";
  //  if(mass) os <<"mass:" <<mass <<' ';
  //  if(type!=BT_dynamic) os <<"dyntype:" <<(int)type <<' ';
  //  uint i; Node *a;
  //  for(Type *  a:  ats)
  //      if(a->keys(0)!="X" && a->keys(0)!="pose") os <<*a <<' ';
}


/************* USER INTERFACE **************/

void rai::Frame::setShape(rai::ShapeType shape, const std::vector<double>& size){
  getShape().type() = shape;
  getShape().size() = size;
  getShape().createMeshes();
}

void rai::Frame::setPosition(const std::vector<double>& pos){
  X.pos.set(pos);
}

void rai::Frame::setQuaternion(const std::vector<double>& quat){
  X.rot.set(quat);
  X.rot.normalize();
}

void rai::Frame::setRelativePosition(const std::vector<double>& pos){
  Q.pos.set(pos);
  calc_X_from_parent();
}

void rai::Frame::setRelativeQuaternion(const std::vector<double>& quat){
  Q.rot.set(quat);
  Q.rot.normalize();
  calc_X_from_parent();
}

void rai::Frame::setPointCloud(const std::vector<double>& points){
  getShape().type() = ST_pointCloud;
  if(!points.size()){
    cerr <<"given point cloud has zero size" <<endl;
    return;
  }
  getShape().mesh().V.clear().operator=(points).reshape(-1, 3);
}

void rai::Frame::setConvexMesh(const std::vector<double>& points){
  getShape().type() = ST_mesh;
  getShape().mesh().V.clear().operator=(points).reshape(-1, 3);
  getShape().mesh().makeConvexHull();
}

void rai::Frame::setColor(const std::vector<double>& color){
  getShape().mesh().C = color;
}

arr rai::Frame::getMeshPoints(){
  return getShape().mesh().V;
}

/***********************************************************/

rai::Frame* rai::Frame::insertPreLink(const rai::Transformation &A) {
  //new frame between: parent -> f -> this
  Frame *f = new Frame(K);

  if(parent) {
    f->linkFrom(parent);
    parent->parentOf.removeValue(this);
    f->name <<parent->name <<'>' <<name;
  }else{
    f->name <<"NIL>" <<name;
  }
  parent=f;
  parent->parentOf.append(this);
  
  if(!!A) f->Q = A;
  
  return f;
}

rai::Frame* rai::Frame::insertPostLink(const rai::Transformation &B) {
  //new frame between: parent -> this -> f
  Frame *f = new Frame(K);
  if(name) f->name <<'<' <<name;
  
  //reconnect all outlinks from -> to
  f->parentOf = parentOf;
  for(Frame *b:parentOf) b->parent = f;
  parentOf.clear();
  f->Q = B;
  f->linkFrom(this);
  
  return f;
}

void rai::Frame::unLink() {
  CHECK(parent,"");
  parent->parentOf.removeValue(this);
  parent=NULL;
  Q.setZero();
  if(joint) { delete joint; joint=NULL; }
}

void rai::Frame::linkFrom(rai::Frame *_parent, bool adoptRelTransform) {
  CHECK(_parent,"you need to set a parent to link from");
  CHECK(!parent,"this frame is already linked to a parent");
  if(parent==_parent) return;
  parent=_parent;
  parent->parentOf.append(this);
  if(adoptRelTransform) Q = X/parent->X;
}

bool rai::Frame::isChildOf(const rai::Frame* par, int order) const{
  Frame *p = parent;
  while(p){
    if(p->joint) order--;
    if(order<0) return false;
    if(p==par) return true;
    p = p->parent;
  }
  return false;
}

rai::Joint::Joint(rai::Frame& f, rai::JointType _type) : Joint(f, (Joint*)NULL){
  setType(_type);
}

rai::Joint::Joint(Frame &f, Joint *copyJoint)
  : frame(&f), qIndex(UINT_MAX){
  CHECK(!frame->joint, "the Link already has a Joint");
  frame->joint = this;
  frame->K.reset_q();
  
  if(copyJoint) {
    qIndex=copyJoint->qIndex; dim=copyJoint->dim; mimic=reinterpret_cast<Joint*>(copyJoint->mimic?1l:0l); constrainToZeroVel=copyJoint->constrainToZeroVel;
    type=copyJoint->type; axis=copyJoint->axis; limits=copyJoint->limits; q0=copyJoint->q0; H=copyJoint->H;
    active=copyJoint->active;
    
    if(copyJoint->mimic) {
      mimic = frame->K.frames(copyJoint->mimic->frame->ID)->joint;
    }
    
    if(copyJoint->uncertainty) {
      new Uncertainty(this, copyJoint->uncertainty);
    }
  }
}

rai::Joint::Joint(Frame &from, Frame &f, Joint *copyJoint)
  : Joint(f, copyJoint) {
  frame->linkFrom(&from);
}

rai::Joint::~Joint() {
  frame->K.reset_q();
  frame->joint = NULL;
  //if(frame->parent) frame->unLink();
}

void rai::Joint::calc_Q_from_q(const arr &q, uint _qIndex) {
  rai::Transformation &Q = frame->Q;
//  if(type!=JT_rigid) Q.setZero();
  if(mimic) {
    Q = mimic->frame->Q;
  } else {
    switch(type) {
      case JT_hingeX: {
        Q.rot.setRadX(q.elem(_qIndex));
      } break;
      
      case JT_hingeY: {
        Q.rot.setRadY(q.elem(_qIndex));
      } break;
      
      case JT_hingeZ: {
        Q.rot.setRadZ(q.elem(_qIndex));
      } break;
      
      case JT_universal: {
        rai::Quaternion rot1, rot2;
        rot1.setRadX(q.elem(_qIndex));
        rot2.setRadY(q.elem(_qIndex+1));
        Q.rot = rot1*rot2;
      } break;
      
      case JT_quatBall: {
        Q.rot.set(q.p+_qIndex);
        {
          double n=Q.rot.normalization();
          if(!rai_Kin_frame_ignoreQuatNormalizationWarning) if(n<.1 || n>10.) LOG(-1) <<"quat normalization is extreme: " <<n;
        }
        Q.rot.normalize();
        Q.rot.isZero=false; //WHY? (gradient check fails without!)
      } break;
      
      case JT_free: {
        Q.pos.set(q.p+_qIndex);
        Q.rot.set(q.p+_qIndex+3);
        {
          double n=Q.rot.normalization();
          if(!rai_Kin_frame_ignoreQuatNormalizationWarning) if(n<.1 || n>10.) LOG(-1) <<"quat normalization is extreme: " <<n;
        }
        Q.rot.normalize();
        Q.rot.isZero=false;
      } break;
      
      case JT_XBall: {
        Q.pos.x = q.elem(_qIndex);
        Q.pos.y = 0.;
        Q.pos.z = 0.;
        Q.pos.isZero = false;
        Q.rot.set(q.p+_qIndex+1);
        {
          double n=Q.rot.normalization();
          if(n<.1 || n>10.) LOG(-1) <<"quat normalization is extreme: " <<n;
        }
        Q.rot.normalize();
        Q.rot.isZero=false;
      } break;
      
      case JT_transX: {
        Q.pos = q.elem(_qIndex)*Vector_x;
      } break;
      
      case JT_transY: {
        Q.pos = q.elem(_qIndex)*Vector_y;
      } break;
      
      case JT_transZ: {
        Q.pos = q.elem(_qIndex)*Vector_z;
      } break;
      
      case JT_transXY: {
        Q.pos.set(q.elem(_qIndex), q.elem(_qIndex+1), 0.);
      } break;
      
      case JT_trans3: {
        Q.pos.set(q.elem(_qIndex), q.elem(_qIndex+1), q.elem(_qIndex+2));
      } break;
      
      case JT_transXYPhi: {
        Q.pos.set(q.elem(_qIndex), q.elem(_qIndex+1), 0.);
        Q.rot.setRadZ(q.elem(_qIndex+2));
      } break;
      
      case JT_phiTransXY: {
        Q.rot.setRadZ(q.elem(_qIndex));
        Q.pos = Q.rot*Vector(q.elem(_qIndex+1), q.elem(_qIndex+2), 0.);
      } break;
      
      case JT_rigid:
        break;
        
      case JT_time:
        frame->tau = 1e-1 * q.elem(_qIndex);
        if(frame->tau<1e-10) frame->tau=1e-10;
        break;
      default: NIY;
    }
  }
  CHECK_EQ(Q.pos.x, Q.pos.x, "NAN transform");
  CHECK_EQ(Q.rot.w, Q.rot.w, "NAN transform");
  
  //    link->link = A * Q * B; //total rel transformation
}

arr rai::Joint::calc_q_from_Q(const rai::Transformation &Q) const {
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
    case JT_time:
      q.resize(1);
      q(0) = 1e1 * frame->tau;
      break;
    default: NIY;
  }
  return q;
}

arr rai::Joint::getScrewMatrix() {
  arr S(2, dim, 3);
  S.setZero();
  rai::Vector axis;
  
  if(type==JT_hingeX) {
    axis = X().rot.getX();
    S(0,0, {}) = axis.getArr();
    S(1,0, {}) = (-axis ^ X().pos).getArr();
  }
  if(type==JT_hingeY) {
    axis = X().rot.getY();
    S(0,0, {}) = axis.getArr();
    S(1,0, {}) = (-axis ^ X().pos).getArr();
  }
  if(type==JT_hingeZ) {
    axis = X().rot.getZ();
    S(0,0, {}) = axis.getArr();
    S(1,0, {}) = (-axis ^ X().pos).getArr();
  } else if(type==JT_transX) {
    axis = X().rot.getX();
    S(1,0, {}) = axis.getArr();
  } else if(type==JT_transY) {
    axis = X().rot.getY();
    S(1,0, {}) = axis.getArr();
  } else if(type==JT_transZ) {
    axis = X().rot.getZ();
    S(1,0, {}) = axis.getArr();
  } else if(type==JT_transXY) {
    if(mimic) NIY;
    arr R = X().rot.getArr();
    S[1] = R({0,1});
  } else if(type==JT_transXYPhi) {
    if(mimic) NIY;
    arr R = X().rot.getArr();
    axis = R[2];
    S(1,0, {}) = R[0];
    S(1,1, {}) = R[1];
    S(0,2, {}) = axis.getArr();
    S(1,2, {}) = (-axis ^ (X().pos + X().rot*Q().pos)).getArr();
  } else if(type==JT_phiTransXY) {
    if(mimic) NIY;
    axis = X().rot.getX();
    S(0,0, {}) = axis.getArr();
    S(1,0, {}) = (-axis ^ X().pos).getArr();
    arr R = (X().rot*Q().rot).getArr();
    S[1] = R({0,1});
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
    for(uint i=0; i<4; i++) for(uint k=0; k<3; k++) S(0, i+offset, k) = Jrot(k,i);
    Jrot = crossProduct(Jrot, conv_vec2arr(-(X().pos+X().rot*Q().pos)));  //cross-product of all 4 w-vectors with lever
    for(uint i=0; i<4; i++) for(uint k=0; k<3; k++) S(1, i+offset, k) = Jrot(k,i);
  }
  return S;
}

uint rai::Joint::getDimFromType() const {
  if(mimic) return 0;
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
  if(type==JT_time) return 1;
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
  return frame->K.q.elem(qIndex);
}

void rai::Joint::makeRigid() {
  if(type!=JT_rigid){
    type=JT_rigid; frame->K.reset_q();
  }
}

void rai::Joint::makeFree(double H_cost){
  setType(JT_free);
  H=H_cost;
}

void rai::Joint::setType(rai::JointType _type){
  if(type!=_type){
    type = _type;
    dim = getDimFromType();
    frame->K.reset_q();
  }
}

void rai::Joint::flip(){
  frame->joint = 0;
  frame = frame->parent;
  CHECK(!frame->joint, "");
  frame->joint = this;
  frame->K.reset_q();
}

void rai::Joint::read(const Graph &G) {
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
    CHECK_EQ(frame->parentOf.N, 1,"");
    Frame *follow = frame->parentOf.scalar();
    
    CHECK(follow->parent, "");
    CHECK(!follow->joint, "");
    follow->Q = B;
    B.setZero();
  }
  
  if(!A.isZero()) {
    frame->insertPreLink(A);
    A.setZero();
  }
  
  G.get(frame->Q, "Q");
  G.get(H, "ctrl_H");
  if(G.get(d, "joint"))        type=(JointType)d;
  else if(G.get(str, "joint")) { str >>type; }
  else if(G.get(d, "type"))    type=(JointType)d;
  else if(G.get(str, "type"))  { str >>type; }
  else type=JT_hingeX;
  
  dim = getDimFromType();
  
  if(G.get(d, "q")) {
    if(!dim) { //HACK convention
      frame->Q.rot.setRad(d, 1., 0., 0.);
    } else {
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
    CHECK(limits.N>=2*qDim()/* || limits.N==2*qDim()+3*/, "parsed limits have wrong dimension: either lo-hi or lo-hi-vel-eff-acc");
  }
  G.get(ctrl_limits, "ctrl_limits");
  if(ctrl_limits.N && type!=JT_rigid) {
    if(!limits.N) limits.resizeAs(ctrl_limits).setZero();
    CHECK_EQ(3,ctrl_limits.N, "parsed ctrl_limits have wrong dimension");
    limits.append(ctrl_limits);
  }
  
  //coupled to another joint requires post-processing by the Graph::read!!
  if(G["mimic"]) {
    mimic=(Joint*)1;
    dim=0;
  }
}

void rai::Joint::write(Graph& g){
  g.newNode<Enum<JointType>>({"joint"}, {}, type);
  if(H!=1.) g.newNode<double>({"ctrl_H"}, {}, H);
  if(limits.N) g.newNode<arr>({"limits"}, {}, limits);
  if(mimic) g.newNode<rai::String>({"mimic"}, {}, STRING('(' <<mimic->frame->name <<')'));
}

void rai::Joint::write(std::ostream& os) const {
  os <<" joint:" <<type;
  if(H) os <<" ctrl_H:"<<H;
  if(limits.N) os <<" limits:[" <<limits <<"]";
  if(mimic) {
    os <<" mimic:(" <<mimic->frame->name <<')';
  }
  os <<' ';
//  Node *n;
//  if((n=frame->ats["Q"])) os <<*n <<' ';
//  if((n=frame->ats["q"])) os <<*n <<' ';
}

//===========================================================================
//
// Shape
//

rai::Shape::Shape(Frame &f, const Shape *copyShape)
  : frame(f), _type(ST_none) {
  
  CHECK(!frame.shape, "this frame already has a shape attached");
  frame.shape = this;
  if(copyShape) {
    const Shape& s = *copyShape;
    if(s._mesh) _mesh = s._mesh; //shallow shared_ptr copy!
    if(s._sscCore) _sscCore = s._sscCore; //shallow shared_ptr copy!
    _type = s._type;
    size = s.size;
    cont = s.cont;
    visual = s.visual;
  }else{
    mesh().C={.8,.8,.8};
  }
}

rai::Shape::~Shape() {
  frame.shape = NULL;
}

void rai::Shape::setMeshMimic(const rai::Frame *f) {
  CHECK(!_mesh, "");
  CHECK(f->shape->_mesh, "");
  _mesh = f->shape->_mesh;
}

void rai::Shape::read(const Graph& ats) {

  {
    double d;
    arr x;
    rai::String str;
    rai::FileToken fil;

    ats.get(size, "size");
    if(ats.get(mesh().C, "color")) {
      CHECK(mesh().C.N==3 || mesh().C.N==4,"");
      //    if(x.N==3){ memmove(color, x.p, 3*sizeof(double)); color[3]=1.; }
      //    else memmove(color, x.p, 4*sizeof(double));
    }
    if(ats.get(d, "shape"))        { type()=(ShapeType)(int)d;}
    else if(ats.get(str, "shape")) { str>> type(); }
    else if(ats.get(d, "type"))    { type()=(ShapeType)(int)d;}
    else if(ats.get(str, "type"))  { str>> type(); }
    if(ats.get(str, "mesh"))     { mesh().read(FILE(str), str.getLastN(3).p, str); }
    else if(ats.get(fil, "mesh"))     {
      fil.cd_file();
      mesh().read(fil.getIs(), fil.name.getLastN(3).p, fil.name);
    }
    if(ats.get(d, "meshscale"))  { mesh().scale(d); }
    if(ats.get(x, "meshscale"))  { mesh().scale(x(0), x(1), x(2)); }

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


  
  if(ats["contact"]){
    double d;
    if(ats.get(d, "contact")) cont = (char)d;
    else cont=1;
  }
  if(ats["noVisual"]){
    visual=false;
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
  os <<" shape:" <<_type;
  if(_type!=ST_mesh) os <<" size:[" <<size <<"]";
  
  Node *n;
  if((n=frame.ats["color"])) os <<' ' <<*n;
  if((n=frame.ats["mesh"])) os <<' ' <<*n;
  if((n=frame.ats["meshscale"])) os <<' ' <<*n;
  if(cont) os <<" contact:" <<(int)cont;
}

void rai::Shape::write(Graph& g){
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
  if(frame.K.orsDrawColors && !frame.K.orsDrawIndexColors && !gl.drawMode_idColor) {
    if(mesh().C.N) glColor(mesh().C); //color[0], color[1], color[2], color[3]*world.orsDrawAlpha);
    else   glColor(.5, .5, .5);
  }
  if(frame.K.orsDrawIndexColors) gl.drawId(frame.ID);
  
  double GLmatrix[16];
  frame.X.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  
  if(!frame.K.orsDrawShapes) {
    double scale=.33*(size(0)+size(1)+size(2) + 2.*size(3)); //some scale
    if(!scale) scale=1.;
    scale*=.3;
    glDrawAxes(scale);
    glColor(0, 0, .5);
    glDrawSphere(.1*scale);
  }

  //default!
  if(frame.K.orsDrawShapes) {
    CHECK(_type!=rai::ST_none, "shape type is not initialized");

    if(gl.drawMode_idColor) Geo_mesh_drawColors=false; else Geo_mesh_drawColors=true;
    if(_type==rai::ST_marker){
      if(frame.K.orsDrawMarkers)
        glDrawDiamond(size(0)/5., size(0)/5., size(0)/5.); glDrawAxes(size(0), !gl.drawMode_idColor);
    }else{
      CHECK_GE(mesh().V.N, 1, "cannot draw empty mesh");
      mesh().glDraw(gl);
    }
  }

  if(frame.K.orsDrawZlines) {
    glColor(0, .7, 0);
    glBegin(GL_LINES);
    glVertex3d(0., 0., 0.);
    glVertex3d(0., 0., -frame.X.pos.z);
    glEnd();
  }
  
  if(frame.K.orsDrawBodyNames) {
    glColor(1,1,1);
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
    case rai::ST_sphere:{
      sscCore().V = arr(1,3, {0.,0.,0.});
      double rad=1;
      if(size.N==1) rad=size(0);
      else rad=size(3);
      mesh().setSSCvx(sscCore(), rad);
      //      mesh().setSphere();
      //      mesh().scale(size(3), size(3), size(3));
    } break;
    case rai::ST_cylinder:
      CHECK(size(3)>1e-10,"");
      mesh().setCylinder(size(3), size(2));
      break;
    case rai::ST_capsule:
      CHECK(size(-1)>1e-10,"");
      sscCore().V = arr(2,3, {0.,0.,-.5*size(-2), 0.,0.,.5*size(-2)});
      mesh().setSSCvx(sscCore(), size(-1));
      //      mesh().setCappedCylinder(size(3), size(2));
      //      mesh().setSSBox(2.*size(3), 2.*size(3), size(2)+2.*size(3), size(3));
      break;
    case rai::ST_retired_SSBox:
      HALT("deprecated?");
      mesh().setSSBox(size(0), size(1), size(2), size(3));
      break;
    case rai::ST_marker:
      break;
    case rai::ST_mesh:
    case rai::ST_pointCloud:
      if(!mesh().V.N) LOG(-1) <<"mesh needs to be loaded";
      break;
    case rai::ST_ssCvx:
      CHECK(size(-1)>1e-10,"");
      if(!sscCore().V.N) {
        CHECK(mesh().V.N, "mesh or sscCore needs to be loaded");
        sscCore() = mesh();
      }
      mesh().setSSCvx(sscCore(), size(3));
      break;
    case rai::ST_ssBox: {
      if(size(3)<1e-10) {
        sscCore().setBox();
        sscCore().scale(size(0), size(1), size(2));
        mesh() = sscCore();
        break;
      }
      double r = size(3);
      CHECK(size.N==4 && r>1e-10,"");
      for(uint i=0;i<3;i++) if(size(i)<2.*r) size(i) = 2.*r;
      sscCore().setBox();
      sscCore().scale(size(0)-2.*r, size(1)-2.*r, size(2)-2.*r);
      mesh().setSSBox(size(0), size(1), size(2), r);
      //      mesh().setSSCvx(sscCore, r);
    } break;
    default: NIY;
  }
}

rai::Inertia::Inertia(Frame &f, Inertia *copyInertia) : frame(f), type(BT_kinematic) {
  CHECK(!frame.inertia, "this frame already has inertia");
  frame.inertia = this;
  if(copyInertia) {
    mass = copyInertia->mass;
    matrix = copyInertia->matrix;
    type = copyInertia->type;
    com = copyInertia->com;
    force = copyInertia->force;
    torque = copyInertia->torque;
  }
}

rai::Inertia::~Inertia() {
  frame.inertia = NULL;
}

void rai::Inertia::defaultInertiaByShape() {
  CHECK(frame.shape, "");
  
  //add inertia to the body
  switch(frame.shape->type()) {
    case ST_sphere:   inertiaSphere(matrix.p(), mass, 1000., frame.shape->radius());  break;
    case ST_ssBox:
    case ST_box:      inertiaBox(matrix.p(), mass, 1000., frame.shape->size(0), frame.shape->size(1), frame.shape->size(2));  break;
    case ST_capsule:
    case ST_cylinder: inertiaCylinder(matrix.p(), mass, 1000., frame.shape->size(2), frame.shape->size(3));  break;
    default: HALT("not implemented for this shape type");
  }
}

arr rai::Inertia::getFrameRelativeWrench() {
  arr f(6);
  rai::Vector fo = frame.X.rot/force;
  rai::Vector to = frame.X.rot/(torque + ((frame.X.rot*com)^force));
  f(0)=to.x;  f(1)=to.y;  f(2)=to.z;
  f(3)=fo.x;  f(4)=fo.y;  f(5)=fo.z;
  return f;
}

void rai::Inertia::write(std::ostream &os) const {
  os <<" mass:" <<mass;
}

void rai::Inertia::write(Graph& g){
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
  if(G.get(d,"dyntype")) type=(BodyType)d;
}

RUN_ON_INIT_BEGIN(frame)
JointL::memMove=true;
RUN_ON_INIT_END(frame)
