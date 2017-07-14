#include <climits>
#include "frame.h"
#include "kin.h"

#ifdef MLR_GL
#include <Gui/opengl.h>
#endif

//===========================================================================
//
// Frame
//

//mlr::Body::Body() { reset(); }

//mlr::Body::Body(const Body& b) { reset(); *this=b; }

mlr::Frame::Frame(KinematicWorld& _world, const Frame* copyBody)
  : world(_world), type(BT_dynamic){

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
  if(rel) delete rel;
  while(outLinks.N) delete outLinks.last();
  if(shape) delete shape;
  world.bodies.removeValue(this);
  listReindex(world.bodies);
}

void mlr::Frame::parseAts(const Graph& ats) {
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
      new Shape(this);
    }else{  // if .obj file create Shape for all submeshes
      auto subMeshPositions = getSubMeshPositions(file->name);
      for(uint i=0;i<subMeshPositions.d0;i++){
        auto parsing_pos = subMeshPositions[i];
        Shape *s = new Shape(this);
        s->mesh.parsing_pos_start = parsing_pos(0);
        s->mesh.parsing_pos_end = parsing_pos(1);
    //TODO: use Shape::parseAts instead of doing the same things here again!!
        s->mesh.readObjFile(file->getIs());
        s->type=ST_mesh;
      }
    }
  }

  // add shape if there is no shape exists yet
  if(ats.getNode("type") && !shape){
    shape = new Shape(this);
  }

  // copy body attributes to shapes
  if(shape) shape->read(ats);
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

namespace mlr {
std::ostream& operator<<(std::ostream& os, const Frame& x) { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Shape& x) { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Joint& x) { x.write(os); return os; }
}

mlr::FrameRel::~FrameRel(){
  from->outLinks.removeValue(to);
  if(joint) delete joint;
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
  to->rel = new FrameRel(from, to);
  to->rel->joint = this;
  CHECK_EQ(to->joint()->to, to, "");
  index = from->world.joints.N;
  from->world.joints.append(this);
  if(copyJoint){
      qIndex=copyJoint->qIndex; dim=copyJoint->dim; mimic=reinterpret_cast<Joint*>(copyJoint->mimic?1l:0l); agent=copyJoint->agent; constrainToZeroVel=copyJoint->constrainToZeroVel;
      type=copyJoint->type; A=copyJoint->A; Q=copyJoint->Q; B=copyJoint->B; X=copyJoint->X; axis=copyJoint->axis; limits=copyJoint->limits; q0=copyJoint->q0; H=copyJoint->H;
  }
}

mlr::Joint::~Joint() {
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

//    to->rel->rel = A * Q * B; //total rel transformation
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

//===========================================================================
//
// Shape
//

mlr::Shape::Shape(Frame* b, const Shape *copyShape, bool referenceMeshOnCopy)
  : frame(b), type(ST_none) {
  size = {1.,1.,1.};
  mesh.C = consts<double>(.8, 3); //color[0]=color[1]=color[2]=.8; color[3]=1.;

  CHECK(b,"");
  CHECK(!b->shape, "this frame already has a geom attached");
  index=b->world.shapes.N;
  b->world.shapes.append(this);
  b->shape = this;
  if(copyShape){
    const Shape& s = *copyShape;
    type=s.type;
    size=s.size;
    if(!referenceMeshOnCopy){
      mesh=s.mesh;
      sscCore=s.sscCore;
    }else{
      mesh.V.referTo(s.mesh.V);
      mesh.T.referTo(s.mesh.T);
      mesh.C.referTo(s.mesh.C);
      mesh.Vn.referTo(s.mesh.Vn);
      sscCore.V.referTo(s.sscCore.V);
      sscCore.T.referTo(s.sscCore.T);
      sscCore.C.referTo(s.sscCore.C);
      sscCore.Vn.referTo(s.sscCore.Vn);
    }
    mesh_radius=s.mesh_radius; cont=s.cont;
  }
}

mlr::Shape::~Shape() {
  frame->shape = NULL;
  frame->world.shapes.removeValue(this);
  listReindex(frame->world.shapes);
}

void mlr::Shape::read(const Graph& ats) {
  double d;
  arr x;
  mlr::String str;
  mlr::FileToken fil;
  mlr::Transformation t;
  ats.get(size, "size");
  if(ats.get(mesh.C, "color")){
    CHECK(mesh.C.N==3 || mesh.C.N==4,"");
//    if(x.N==3){ memmove(color, x.p, 3*sizeof(double)); color[3]=1.; }
    //    else memmove(color, x.p, 4*sizeof(double));
  }
  if(ats.get(d, "type"))       { type=(ShapeType)(int)d;}
  else if(ats.get(str, "type")) { str>> type; }
  if(ats["contact"])           { cont=true; }
  if(ats.get(fil, "mesh"))     { mesh.read(fil.getIs(), fil.name.getLastN(3).p, fil.name); }
  if(ats.get(d, "meshscale"))  { mesh.scale(d); }

  //create mesh for basic shapes
  switch(type) {
    case mlr::ST_none: HALT("shapes should have a type - somehow wrong initialization..."); break;
    case mlr::ST_box:
      mesh.setBox();
      mesh.scale(size(0), size(1), size(2));
      break;
    case mlr::ST_sphere:
      mesh.setSphere();
      mesh.scale(size(3), size(3), size(3));
      break;
    case mlr::ST_cylinder:
      CHECK(size(3)>1e-10,"");
      mesh.setCylinder(size(3), size(2));
      break;
    case mlr::ST_capsule:
      CHECK(size(3)>1e-10,"");
//      mesh.setCappedCylinder(size(3), size(2));
      sscCore.setBox();
      sscCore.scale(0., 0., size(2));
      mesh.setSSCvx(sscCore, size(3));
      break;
    case mlr::ST_retired_SSBox:
      HALT("deprecated?");
      mesh.setSSBox(size(0), size(1), size(2), size(3));
      break;
    case mlr::ST_marker:
      break;
    case mlr::ST_mesh:
    case mlr::ST_pointCloud:
      CHECK(mesh.V.N, "mesh needs to be loaded to draw mesh object");
      sscCore = mesh;
      sscCore.makeConvexHull();
      break;
    case mlr::ST_ssCvx:
      CHECK(size(3)>1e-10,"");
      CHECK(mesh.V.N, "mesh needs to be loaded to draw mesh object");
      sscCore=mesh;
      mesh.setSSCvx(sscCore, size(3));
      break;
    case mlr::ST_ssBox:
      CHECK(size.N==4 && size(3)>1e-10,"");
      sscCore.setBox();
      sscCore.scale(size(0)-2.*size(3), size(1)-2.*size(3), size(2)-2.*size(3));
      mesh.setSSBox(size(0), size(1), size(2), size(3));
//      mesh.setSSCvx(sscCore, size(3));
      break;
    default: NIY;
  }

  //center the mesh:
  if(type==mlr::ST_mesh && mesh.V.N){
    NIY;
//    Vector c = mesh.center();
//    if(c.length()>1e-8 && !ats["rel_includes_mesh_center"]){
//      rel.addRelativeTranslation(c);
//      ats.newNode<bool>({"rel_includes_mesh_center"}, {}, true);
//    }
  }

  //compute the bounding radius
  if(mesh.V.N) mesh_radius = mesh.getRadius();

  //colored box?
  if(ats["coloredBox"]){
    CHECK_EQ(mesh.V.d0, 8, "I need a box");
    arr col=mesh.C;
    mesh.C.resize(mesh.T.d0, 3);
    for(uint i=0;i<mesh.C.d0;i++){
      if(i==2 || i==3) mesh.C[i] = col; //arr(color, 3);
      else if(i>=4 && i<=7) mesh.C[i] = 1.;
      else mesh.C[i] = .5;
    }
  }


  //add inertia to the body
  if(frame) {
    Matrix I;
    double mass=-1.;
    switch(type) {
      case ST_sphere:   inertiaSphere(I.p(), mass, 1000., size(3));  break;
      case ST_box:      inertiaBox(I.p(), mass, 1000., size(0), size(1), size(2));  break;
      case ST_capsule:
      case ST_cylinder: inertiaCylinder(I.p(), mass, 1000., size(2), size(3));  break;
      case ST_none:
      default: ;
    }
    if(mass>0.){
      frame->mass += mass;
      frame->inertia += I;
    }
  }
}

void mlr::Shape::write(std::ostream& os) const {
  os <<"type=" <<type <<' ';
//  os <<"size=[" <<size(0) <<' '<<size(1) <<' '<<size(2) <<' '<<size(3) <<"] ";
//  if(!rel.isZero()) os <<"rel=<T " <<rel <<" > ";
}

#ifdef MLR_GL
void mlr::Shape::glDraw(OpenGL& gl) {
  //set name (for OpenGL selection)
  glPushName((index <<2) | 1);
  if(frame->world.orsDrawColors && !frame->world.orsDrawIndexColors) glColor(mesh.C); //color[0], color[1], color[2], color[3]*world.orsDrawAlpha);
  if(frame->world.orsDrawIndexColors) glColor3b((index>>16)&0xff, (index>>8)&0xff, index&0xff);


  double GLmatrix[16];
  frame->X.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);

  if(!frame->world.orsDrawShapes) {
    double scale=.33*(size(0)+size(1)+size(2) + 2.*size(3)); //some scale
    if(!scale) scale=1.;
    scale*=.3;
    glDrawAxes(scale);
    glColor(0, 0, .5);
    glDrawSphere(.1*scale);
  }
  if(frame->world.orsDrawShapes) {
    switch(type) {
      case mlr::ST_none: LOG(-1) <<"Shape '" <<frame->name <<"' has no joint type";  break;
      case mlr::ST_box:
        if(frame->world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(frame->world.orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawBox(size(0), size(1), size(2));
        break;
      case mlr::ST_sphere:
        if(frame->world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(frame->world.orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawSphere(size(3));
        break;
      case mlr::ST_cylinder:
        if(frame->world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(frame->world.orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawCylinder(size(3), size(2));
        break;
      case mlr::ST_capsule:
        if(frame->world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(frame->world.orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawCappedCylinder(size(3), size(2));
        break;
      case mlr::ST_retired_SSBox:
        HALT("deprecated??");
        if(frame->world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(frame->world.orsDrawMeshes){
          if(!mesh.V.N) mesh.setSSBox(size(0), size(1), size(2), size(3));
          mesh.glDraw(gl);
        }else NIY;
        break;
      case mlr::ST_marker:
        if(frame->world.orsDrawMarkers){
          glDrawDiamond(size(0)/5., size(0)/5., size(0)/5.); glDrawAxes(size(0));
        }
        break;
      case mlr::ST_mesh:
        CHECK(mesh.V.N, "mesh needs to be loaded to draw mesh object");
        if(frame->world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;
      case mlr::ST_ssCvx:
        CHECK(sscCore.V.N, "sscCore needs to be loaded to draw mesh object");
        if(!mesh.V.N) mesh.setSSCvx(sscCore, size(3));
        if(frame->world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;
      case mlr::ST_ssBox:
        if(!mesh.V.N || !sscCore.V.N){
          sscCore.setBox();
          sscCore.scale(size(0)-2.*size(3), size(1)-2.*size(3), size(2)-2.*size(3));
          mesh.setSSCvx(sscCore, size(3));
        }
        if(frame->world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;
      case mlr::ST_pointCloud:
        CHECK(mesh.V.N, "mesh needs to be loaded to draw point cloud object");
        if(frame->world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;

      default: HALT("can't draw that geom yet");
    }
  }
  if(frame->world.orsDrawZlines) {
    glColor(0, .7, 0);
    glBegin(GL_LINES);
    glVertex3d(0., 0., 0.);
    glVertex3d(0., 0., -frame->X.pos.z);
    glEnd();
  }

  if(frame->world.orsDrawBodyNames && frame){
    glColor(1,1,1);
    glDrawText(frame->name, 0, 0, 0);
  }

  glPopName();
}

#endif
