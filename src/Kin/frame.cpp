#include <climits>
#include "frame.h"
#include "kin.h"

//===========================================================================
//
// Frame
//

//mlr::Body::Body() { reset(); }

//mlr::Body::Body(const Body& b) { reset(); *this=b; }

mlr::Frame::Frame(KinematicWorld& _world, const Frame* copyBody)
  : world(_world), vel(0), angvel(0) {
  reset();
  index=world.bodies.N;
  world.bodies.append(this);
  if(copyBody){
    *this=*copyBody;
    if(copyBody->hasJoint()){
      Joint *j = copyBody->joint();
      new Joint(world.bodies(j->from->index), this, j);
    }
  }
}

mlr::Frame::~Frame() {
  reset();
  if(rel) delete rel;
  while(outLinks.N) delete outLinks.last();
  while(shapes.N) delete shapes.last();
  world.bodies.removeValue(this);
  listReindex(world.bodies);
}

void mlr::Frame::reset() {
  ats.clear();
  X.setZero();
  type=BT_dynamic;
  shapes.memMove=true;
  com.setZero();
  mass = 0.;
  inertia.setZero();
  vel.setZero();
  angvel.setZero();
}

void mlr::Frame::parseAts() {
  //interpret some of the attributes
  arr x;
  mlr::String str;
  ats.get(X, "X");
  ats.get(X, "pose");

  //mass properties
  double d;
  if(ats.get(d, "mass")) {
    mass=d;
    inertia.setId();
    inertia *= .2*d;
  }

  type=BT_dynamic;
  if(ats["fixed"])       type=BT_static;
  if(ats["static"])      type=BT_static;
  if(ats["kinematic"])   type=BT_kinematic;
  if(ats.get(d,"dyntype")) type=(BodyType)d;

  // SHAPE handling //TODO: remove this code!
  Node* item;
  // a mesh which consists of multiple convex sub meshes creates multiple
  // shapes that belong to the same body
  item = ats.getNode("meshes");
  if(item){
    HALT("this is deprecated");
    mlr::FileToken *file = item->getValue<mlr::FileToken>();
    CHECK(file,"somethings wrong");

    // if mesh is not .obj we only have one shape
    if(!file->name.endsWith("obj")) {
      new Shape(world, *this);
    }else{  // if .obj file create Shape for all submeshes
      auto subMeshPositions = getSubMeshPositions(file->name);
      for(uint i=0;i<subMeshPositions.d0;i++){
        auto parsing_pos = subMeshPositions[i];
        Shape *s = new Shape(world, *this);
        s->mesh.parsing_pos_start = parsing_pos(0);
        s->mesh.parsing_pos_end = parsing_pos(1);
    //TODO: use Shape::parseAts instead of doing the same things here again!!
        s->mesh.readObjFile(file->getIs());
        s->type=ST_mesh;
      }
    }
  }

  // add shape if there is no shape exists yet
  if(ats.getNode("type") && !shapes.N){
    Shape *s = new Shape(world, *this);
    s->name = name;
  }

  // copy body attributes to shapes
  for(Shape *s:shapes) { s->ats=ats;  s->parseAts(); }
  //TODO check if this works! coupled to the listDelete below
  Node *it=ats["type"]; if(it){ delete it; }
}

void mlr::Frame::write(std::ostream& os) const {
  if(!X.isZero()) os <<"pose=<T " <<X <<" > ";
  if(mass) os <<"mass=" <<mass <<' ';
  if(type!=BT_dynamic) os <<"dyntype=" <<(int)type <<' ';
//  uint i; Node *a;
//  for(Type *  a:  ats)
//      if(a->keys(0)!="X" && a->keys(0)!="pose") os <<*a <<' ';
}

void mlr::Frame::read(std::istream& is) {
  reset();
  ats.read(is);
  if(!is.good()) HALT("body '" <<name <<"' read error: in ");
  parseAts();
}

namespace mlr {
std::ostream& operator<<(std::ostream& os, const Frame& x) { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Shape& x) { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Joint& x) { x.write(os); return os; }
}


mlr::Joint *mlr::Frame::joint() const{
    CHECK(rel, "this is not a relative frame");
    CHECK(rel->joint, "this is not a joint");
    return rel->joint;
}

mlr::Frame *mlr::Frame::from() const{
    CHECK(rel, "this is not a relative frame");
    return rel->from;
}

uint mlr::Frame::numInputs() const{
    if(rel) return 1;
    return 0;
}

bool mlr::Frame::hasJoint() const{
    return rel && rel->joint;
}


mlr::Joint::Joint(Frame *f, Frame *t, Joint *copyJoint)
  : index(0), qIndex(UINT_MAX), q0(0.), H(1.), mimic(NULL), from(f), to(t), constrainToZeroVel(false), agent(0) {
  to->rel = new FrameRel(from);
  to->rel->joint = this;
  CHECK_EQ(to->joint()->to, to, "");
  from->outLinks.append(this);
  index = from->world.joints.N;
  from->world.joints.append(this);
  if(copyJoint){
      qIndex=copyJoint->qIndex; dim=copyJoint->dim; mimic=reinterpret_cast<Joint*>(copyJoint->mimic?1l:0l); agent=copyJoint->agent; constrainToZeroVel=copyJoint->constrainToZeroVel;
      type=copyJoint->type; A=copyJoint->A; Q=copyJoint->Q; B=copyJoint->B; X=copyJoint->X; axis=copyJoint->axis; limits=copyJoint->limits; q0=copyJoint->q0; H=copyJoint->H;
  }
}

mlr::Joint::~Joint() {
  from->outLinks.removeValue(this);
  to->rel->joint = NULL;
  from->world.joints.removeValue(this);
  listReindex(from->world.joints);
}

void mlr::Joint::calc_Q_from_q(const arr &q, uint _qIndex){
    if(mimic){
        Q = mimic->Q;
    }else{
        Q.setZero();
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

        case JT_universal:{
            mlr::Quaternion rot1, rot2;
            rot1.setRadX(q.elem(_qIndex));
            rot2.setRadY(q.elem(_qIndex+1));
            Q.rot = rot1*rot2;
        } break;

        case JT_quatBall:{
            Q.rot.set(q.p+_qIndex);
            {
                double n=Q.rot.normalization();
//                if(n<.5 || n>2.) LOG(-1) <<"quat normalization is extreme: " <<n <<endl;
            }
            Q.rot.normalize();
            Q.rot.isZero=false; //WHY? (gradient check fails without!)
        } break;

        case JT_free:{
            Q.pos.set(q.p+_qIndex);
            Q.rot.set(q.p+_qIndex+3);
            {
                double n=Q.rot.normalization();
//                if(n<.5 || n>2.) LOG(-1) <<"quat normalization is extreme: " <<n <<endl;
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

        case JT_glue:
        case JT_rigid:
            Q.setZero();
            break;
        default: NIY;
        }
    }
    CHECK_EQ(Q.pos.x, Q.pos.x, "NAN transform");
    CHECK_EQ(Q.rot.w, Q.rot.w, "NAN transform");
}

arr mlr::Joint::calc_q_from_Q(const mlr::Transformation &Q) const{
    arr q;
    switch(type) {
    case JT_hingeX:
    case JT_hingeY:
    case JT_hingeZ: {
        q.resize(1);
        //angle
        mlr::Vector rotv;
        Q.rot.getRad(q(0), rotv);
        if(q(0)>MLR_PI) q(0)-=MLR_2PI;
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
            q(0) = MLR_PI;
            q(1) = MLR_PI;
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
        q.resize(1);
        q(0)=Q.pos.x;
        q(1)=Q.pos.y;
    } break;
    case JT_transXYPhi: {
        q.resize(3);
        q(0)=Q.pos.x;
        q(1)=Q.pos.y;
        mlr::Vector rotv;
        Q.rot.getRad(q(2), rotv);
        if(q(2)>MLR_PI) q(2)-=MLR_2PI;
        if(rotv*Vector_z<0.) q(2)=-q(2);
    } break;
    case JT_phiTransXY: {
        q.resize(3);
        mlr::Vector rotv;
        Q.rot.getRad(q(0), rotv);
        if(q(0)>MLR_PI) q(0)-=MLR_2PI;
        if(rotv*Vector_z<0.) q(0)=-q(0);
        mlr::Vector relpos = Q.rot/Q.pos;
        q(1)=relpos.x;
        q(2)=relpos.y;
    } break;
    case JT_trans3: {
        q.resize(3);
        q(0)=Q.pos.x;
        q(1)=Q.pos.y;
        q(2)=Q.pos.z;
    } break;
    case JT_glue:
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
    default: NIY;
    }
    return q;
}

uint mlr::Joint::getDimFromType() const {
  if(mimic) return 0;
  if(type>=JT_hingeX && type<=JT_transZ) return 1;
  if(type==JT_transXY) return 2;
  if(type==JT_transXYPhi) return 3;
  if(type==JT_phiTransXY) return 3;
  if(type==JT_trans3) return 3;
  if(type==JT_universal) return 2;
  if(type==JT_quatBall) return 4;
  if(type==JT_free) return 7;
  if(type==JT_glue || type==JT_rigid || type==JT_none) return 0;
  HALT("shouldn't be here");
  return 0;
}

#if 0
mlr::Joint::Joint(KinematicWorld& G, Frame *f, Frame *t, const Joint* copyJoint)
  : world(G), index(0), qIndex(UINT_MAX), from(f), to(t), mimic(NULL), agent(0), constrainToZeroVel(false), q0(0.), H(1.) {
  reset();
  if(copyJoint) *this=*copyJoint;
  index=world.joints.N;
  world.joints.append(this);
  f->outLinks.append(this);
  t-> inLinks.append(this);
  world.q.clear();
  world.qdot.clear();
  world.qdim.clear();
}

mlr::Joint::~Joint() {
  world.checkConsistency();
  reset();
  if(from){ from->outLinks.removeValue(this); listReindex(from->outLinks); }
  if(to){   to->inLinks.removeValue(this); listReindex(to->inLinks); }
  world.joints.removeValue(this);
  listReindex(world.joints);
  world.q.clear();
  world.qdot.clear();
  world.qdim.clear();
}

void mlr::Joint::reset() {
  ats.clear(); A.setZero(); B.setZero(); Q.setZero(); X.setZero(); axis.setZero(); limits.clear(); q0=0.; H=1.; type=JT_none;
//  locker=NULL;
}

void mlr::Joint::parseAts() {
  //interpret some of the attributes
  double d=0.;
  mlr::String str;
  ats.get(A, "A");
  ats.get(A, "from");
  if(ats["BinvA"]) B.setInverse(A);
  ats.get(B, "B");
  ats.get(B, "to");
  ats.get(Q, "Q");
  ats.get(X, "X");
  ats.get(H, "ctrl_H");
  if(ats.get(d, "type")) type=(JointType)d;
  else if(ats.get(str, "type")) { str>> type; }
  else type=JT_hingeX;
  if(type==JT_rigid && !Q.isZero()){ A.appendTransformation(Q); Q.setZero(); }
  if(ats.get(d, "q")){
    q0=d;
    if(type==JT_hingeX) Q.addRelativeRotationRad(d, 1., 0., 0.);
    if(type==JT_rigid)  A.addRelativeRotationRad(d, 1., 0., 0.);
    if(type==JT_transX) Q.addRelativeTranslation(d, 0., 0.);
  }else q0=0.;
  if(ats.get(d, "agent")) agent=(uint)d;
  if(ats["fixed"]) agent=UINT_MAX;
  //axis
  arr axis;
  ats.get(axis, "axis");
  if(axis.N) {
    CHECK_EQ(axis.N,3,"");
    Vector ax(axis);
    Transformation f;
    f.setZero();
    f.rot.setDiff(Vector_x, ax);
    A = A * f;
    B = -f * B;
  }
  //limit
  arr ctrl_limits;
  ats.get(limits, "limits");
  if(limits.N && type!=JT_rigid && !mimic){
    CHECK_EQ(limits.N,2*qDim(), "parsed limits have wrong dimension");
  }
  ats.get(ctrl_limits, "ctrl_limits");
  if(ctrl_limits.N && type!=JT_rigid){
    if(!limits.N) limits.resizeAs(ctrl_limits).setZero();
    CHECK_EQ(3,ctrl_limits.N, "parsed ctrl_limits have wrong dimension");
    limits.append(ctrl_limits);
  }
  //coupled to another joint requires post-processing by the Graph::read!!
  if(ats["mimic"]) mimic=(Joint*)1;
}

void mlr::Joint::applyTransformation(mlr::Transformation& f, const arr& q){
  switch(type) {
    case JT_hingeX:{
//      f.addRelativeRotationRad(q.elem(qIndex),1.,0.,0.);
      f.rot.addX(q.elem(qIndex));
    } break;

    case JT_hingeY: {
//      f.addRelativeRotationRad(q.elem(qIndex),0.,1.,0.);
      f.rot.addY(q.elem(qIndex));
    } break;

    case JT_hingeZ: {
//      f.addRelativeRotationRad(q.elem(qIndex),0.,0.,1.);
      f.rot.addZ(q.elem(qIndex));
    } break;

    case JT_universal:{
      f.addRelativeRotationRad(q.elem(qIndex),1.,0.,0.);
      f.addRelativeRotationRad(q.elem(qIndex+1),0.,1.,0.);
    } break;

    case JT_quatBall:{
      mlr::Quaternion rot;
      rot.set(q.p+qIndex);
      {
          double n=rot.normalization();
          if(n<.5 || n>2.) LOG(-1) <<"quat normalization is extreme: " <<n <<endl;
      }
      rot.normalize();
      rot.isZero=false;
      f.addRelativeRotation(rot);
    } break;

    case JT_free:{
      mlr::Transformation t;
      t.pos.set(q.p+qIndex);
      t.rot.set(q.p+qIndex+3);
      {
          double n=t.rot.normalization();
          if(n<.5 || n>2.) LOG(-1) <<"quat normalization is extreme: " <<n <<endl;
      }
      t.rot.normalize();
      t.rot.isZero=false;
      f.appendTransformation(t);
    } break;

    case JT_transX: {
      f.addRelativeTranslation(q.elem(qIndex),0.,0.);
    } break;

    case JT_transY: {
      f.addRelativeTranslation(0., q.elem(qIndex), 0.);
    } break;

    case JT_transZ: {
      f.addRelativeTranslation(0., 0., q.elem(qIndex));
    } break;

    case JT_transXY: {
      f.addRelativeTranslation(q.elem(qIndex), q.elem(qIndex+1), 0.);
    } break;

    case JT_trans3: {
      f.addRelativeTranslation(q.elem(qIndex), q.elem(qIndex+1), q.elem(qIndex+2));
    } break;

    case JT_transXYPhi: {
      f.addRelativeTranslation(q.elem(qIndex), q.elem(qIndex+1), 0.);
      f.addRelativeRotationRad(q.elem(qIndex+2),0.,0.,1.);
    } break;

    case JT_phiTransXY: {
      f.addRelativeRotationRad(q.elem(qIndex+2),0.,0.,1.);
      f.addRelativeTranslation(q.elem(qIndex), q.elem(qIndex+1), 0.);
    } break;

    case JT_glue:
    case JT_rigid:
      break;
    default: NIY;
  }
}

void mlr::Joint::makeRigid(){
    A.appendTransformation(Q);
    Q.setZero();
    type = JT_rigid;
}

void mlr::Joint::write(std::ostream& os) const {
  os <<"type=" <<type <<' ';
  if(!A.isZero()) os <<"from=<T " <<A <<" > ";
  if(!B.isZero()) os <<"to=<T " <<B <<" > ";
  if(!Q.isZero()) os <<"Q=<T " <<Q <<" > ";
  for(Node * a: ats)
  if(a->keys(0)!="A" && a->keys(0)!="from"
      && a->keys(0)!="axis" //because this was subsumed in A during read
      && a->keys(0)!="B" && a->keys(0)!="to"
      && a->keys(0)!="Q" && a->keys(0)!="q"
      && a->keys(0)!="type") os <<*a <<' ';
}

void mlr::Joint::read(std::istream& is) {
  reset();
  ats.read(is);
  if(!is.good()) HALT("joint (" <<from->name <<' ' <<to->name <<") read read error");
  parseAts();
}


#endif

void mlr::Joint::read(const Graph &G){
    double d=0.;
    mlr::String str;
    G.get(A, "A");
    G.get(A, "from");
    if(G["BinvA"]) B.setInverse(A);
    G.get(B, "B");
    G.get(B, "to");
    G.get(Q, "Q");
    G.get(X, "X");
    G.get(H, "ctrl_H");
    if(G.get(d, "type")) type=(JointType)d;
    else if(G.get(str, "type")) { str >>type; }
    else type=JT_hingeX;
    if(type==JT_rigid && !Q.isZero()){ A.appendTransformation(Q); Q.setZero(); }

    dim = getDimFromType();

    if(G.get(d, "q")){
      q0 = consts<double>(d, dim);
      calc_Q_from_q(q0, 0);
    }else if(G.get(q0, "q")){
      calc_Q_from_q(q0, 0);
    }else{
        Q.setZero();
        q0 = calc_q_from_Q(Q);
    }

    if(G.get(d, "agent")) agent=(uint)d;

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

    //limit
    arr ctrl_limits;
    G.get(limits, "limits");
    if(limits.N && type!=JT_rigid && !mimic){
      CHECK_EQ(limits.N, 2*qDim(), "parsed limits have wrong dimension");
    }
    G.get(ctrl_limits, "ctrl_limits");
    if(ctrl_limits.N && type!=JT_rigid){
      if(!limits.N) limits.resizeAs(ctrl_limits).setZero();
      CHECK_EQ(3,ctrl_limits.N, "parsed ctrl_limits have wrong dimension");
      limits.append(ctrl_limits);
    }

    //coupled to another joint requires post-processing by the Graph::read!!
    if(G["mimic"]) mimic=(Joint*)1;

}

void mlr::Joint::write(std::ostream& os) const {
  os <<"type=" <<type <<' ';
  if(!A.isZero()) os <<"from=<T " <<A <<" > ";
  if(!B.isZero()) os <<"to=<T " <<B <<" > ";
  if(!Q.isZero()) os <<"Q=<T " <<Q <<" > ";
}
