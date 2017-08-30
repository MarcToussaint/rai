#include <climits>
#include "frame.h"
#include "kin.h"
#include "uncertainty.h"

#ifdef MLR_GL
#include <Gui/opengl.h>
#endif

//===========================================================================
//
// Frame
//

mlr::Frame::Frame(KinematicWorld& _world, const Frame* copyFrame)
  : K(_world){

  ID=K.frames.N;
  K.frames.append(this);
  if(copyFrame){
    const Frame& f = *copyFrame;
    name=f.name; X=f.X; ats=f.ats; active=f.active;
    //we cannot copy link! because we can't know if the frames already exist. KinematicWorld::copy copies the rel's !!
    if(copyFrame->shape) new Shape(this, copyFrame->shape);
    if(copyFrame->inertia) new Inertia(this, copyFrame->inertia);
  }
}

mlr::Frame::~Frame() {
  while(outLinks.N) delete outLinks.last()->link;
  if(link) delete link;
  if(shape) delete shape;
  if(inertia) delete inertia;
  K.frames.removeValue(this);
  listReindex(K.frames);
  K.calc_q();
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
    inertia = new Inertia(this);
    inertia->mass=d;
    inertia->matrix.setId();
    inertia->matrix *= .2*d;
    inertia->type=BT_dynamic;
    inertia->read(ats);
  }

  // add shape if there is no shape exists yet
  if(ats.getNode("type") && !shape){
    shape = new Shape(this);
    shape->read(ats);
  }
}

void mlr::Frame::write(std::ostream& os) const {
  if(!X.isZero()) os <<"pose=<T " <<X <<" > ";
//  if(mass) os <<"mass=" <<mass <<' ';
//  if(type!=BT_dynamic) os <<"dyntype=" <<(int)type <<' ';
//  uint i; Node *a;
//  for(Type *  a:  ats)
//      if(a->keys(0)!="X" && a->keys(0)!="pose") os <<*a <<' ';
}

namespace mlr {
//std::ostream& operator<<(std::ostream& os, const Frame& x) { x.write(os); return os; }
//std::ostream& operator<<(std::ostream& os, const Shape& x) { x.write(os); return os; }
//std::ostream& operator<<(std::ostream& os, const Joint& x) { x.write(os); return os; }
}

mlr::Link::Link(mlr::Frame *_from, mlr::Frame *_to, mlr::Link *copyLink)
  : from(_from), to(_to) {
  CHECK(from, "needs a non-NULL frame");
  CHECK(to, "needs a non-NULL frame");
  CHECK(!to->link, "the Frame already has a Link")
  to->link = this;
  from->outLinks.append(to);
  if(copyLink){
    Q = copyLink->Q;
    if(copyLink->joint)
      new Joint(this, copyLink->joint);
  }
}

mlr::Link::~Link(){
  from->outLinks.removeValue(to);
  if(joint) delete joint;
  to->link = NULL;
}

mlr::Link* mlr::Link::insertPreLink(const mlr::Transformation &A){
  //new frame between: from -> f -> to
  Frame *f = new Frame(from->K);
  if(to->name) f->name <<'>' <<to->name;

  //disconnect from -> to
  from->outLinks.removeValue(to);

  //connect from -> f with new Link and no joint
  Link *newLink = new Link(from, f);
  newLink->Q = A;
  f->link = newLink;

  //connect f -> to with old Link and this joint
  f->outLinks.append(to);
  from = f;

  return newLink;
}

mlr::Link* mlr::Link::insertPostLink(const mlr::Transformation &B){
  //new frame between: from -> f -> to
  Frame *f = new Frame(from->K);
  if(to->name) f->name <<'<' <<to->name;

  //disconnect from -> to
  from->outLinks.removeValue(to);

  //connect f->to with new Link and no joint
  CHECK_EQ(to->link, this, "");
  to->link = NULL; //setting this to NULL enables the new in the next line
  Link *newLink = new Link(f, to);
  newLink->Q = B;
  to->link = newLink;

  //connect from->to: associate to->link to f
  from->outLinks.append(f);
  f->link = this;
  this->to = f;

  return newLink;
}

mlr::Joint *mlr::Frame::joint() const{
  if(!link) return NULL;
  return link->joint;
}

mlr::Frame *mlr::Frame::from() const{
  if(!link) return NULL;
  return link->from;
}

mlr::Joint::Joint(Link *_link, Joint *copyJoint)
  : dim(0), qIndex(UINT_MAX), q0(0.), H(1.), mimic(NULL), link(_link), constrainToZeroVel(false) {
  CHECK(!link->joint, "the Link already has a Joint");
  link->joint = this;
  link->to->K.reset_q();

  if(copyJoint){
    qIndex=copyJoint->qIndex; dim=copyJoint->dim; mimic=reinterpret_cast<Joint*>(copyJoint->mimic?1l:0l); constrainToZeroVel=copyJoint->constrainToZeroVel;
    type=copyJoint->type; axis=copyJoint->axis; limits=copyJoint->limits; q0=copyJoint->q0; H=copyJoint->H;
    active=copyJoint->active;

    if(copyJoint->mimic){
      mimic = link->to->K.frames(copyJoint->mimic->link->to->ID)->joint();
    }

    if(copyJoint->uncertainty){
      new Uncertainty(this, copyJoint->uncertainty);
    }
  }
}

mlr::Joint::~Joint() {
  link->to->K.reset_q();
  link->joint = NULL;
}

void mlr::Joint::calc_Q_from_q(const arr &q, uint _qIndex){
  mlr::Transformation &Q = link->Q;
    if(mimic){
        Q = mimic->link->Q;
    }else{
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
                if(n<.5 || n>2.) LOG(-1) <<"quat normalization is extreme: " <<n <<endl;
            }
            Q.rot.normalize();
            Q.rot.isZero=false; //WHY? (gradient check fails without!)
        } break;

        case JT_free:{
            Q.pos.set(q.p+_qIndex);
            Q.rot.set(q.p+_qIndex+3);
            {
                double n=Q.rot.normalization();
                if(n<.5 || n>2.) LOG(-1) <<"quat normalization is extreme: " <<n <<endl;
            }
            Q.rot.normalize();
            Q.rot.isZero=false;
        } break;

        case JT_XBall:{
            Q.pos.x = q.elem(_qIndex);
            Q.pos.y = 0.;
            Q.pos.z = 0.;
            Q.pos.isZero = false;
            Q.rot.set(q.p+_qIndex+1);
            {
                double n=Q.rot.normalization();
                if(n<.5 || n>2.) LOG(-1) <<"quat normalization is extreme: " <<n <<endl;
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
        default: NIY;
        }
    }
    CHECK_EQ(Q.pos.x, Q.pos.x, "NAN transform");
    CHECK_EQ(Q.rot.w, Q.rot.w, "NAN transform");

//    link->link = A * Q * B; //total rel transformation
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
        q.resize(2);
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
  if(type==JT_rigid || type==JT_none) return 0;
  if(type==JT_XBall) return 5;
  HALT("shouldn't be here");
  return 0;
}

void mlr::Joint::makeRigid(){
  type=JT_rigid; link->to->K.reset_q();
}

void mlr::Joint::read(const Graph &G){
    double d=0.;
    mlr::String str;

  mlr::Transformation A=0, B=0;

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

  if(!B.isZero()){
    //new frame between: from -> f -> to
    CHECK(link->to->outLinks.N==1,"");
    Frame *follow = link->to->outLinks.scalar();

    CHECK(follow->link, "");
    CHECK(!follow->joint(), "");
    follow->link->Q = B;
    B.setZero();
  }

  if(!A.isZero()){
    link->insertPreLink(A);
//    //new frame between: from -> f -> to
//    Frame *to = link->to;
//    Frame *from = link->from;
//    Frame *f = new Frame(from->K);
//    if(to->name) f->name <<'>' <<to->name;

//    //disconnect from -> to
//    from->outLinks.removeValue(to);

//    //connect from -> f with new Link and no joint
//    f->link = new Link(from, f);
//    f->link->Q = A;

//    //connect f -> to with old Link and this joint
//    f->outLinks.append(to);
//    link->from = f;

    A.setZero();
  }

  G.get(link->Q, "Q");
  G.get(H, "ctrl_H");
  if(G.get(d, "type")) type=(JointType)d;
  else if(G.get(str, "type")) { str >>type; }
  else type=JT_hingeX;

  dim = getDimFromType();

  if(G.get(d, "q")){
    if(!dim){ //HACK convention
      link->Q.rot.setRad(d, 1., 0., 0.);
    }else{
      CHECK(dim, "setting q (in config file) for 0-dim joint");
      q0 = consts<double>(d, dim);
      calc_Q_from_q(q0, 0);
    }
  }else if(G.get(q0, "q")){
    CHECK_EQ(q0.N, dim, "given q (in config file) does not match dim");
    calc_Q_from_q(q0, 0);
  }else{
//    link->Q.setZero();
    q0 = calc_q_from_Q(link->Q);
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
  if(G["mimic"]){
    mimic=(Joint*)1;
    dim=0;
  }
}

void mlr::Joint::write(std::ostream& os) const {
  os <<"type=" <<type <<' ';
  if(H) os <<"ctrl_H="<<H <<' ';
  if(limits.N) os <<"limits=[" <<limits <<"] ";
  if(mimic){
    os <<"mimic=" <<mimic->to()->name <<' ';
  }

  Node *n;
  if((n=to()->ats["Q"])) os <<*n <<' ';
  if((n=to()->ats["q"])) os <<*n <<' ';
}

//===========================================================================
//
// Shape
//

mlr::Shape::Shape(Frame* b, const Shape *copyShape, bool referenceMeshOnCopy)
  : frame(b), type(ST_none) {
  size = {1.,1.,1.,.1};
  mesh.C = consts<double>(.8, 3); //color[0]=color[1]=color[2]=.8; color[3]=1.;

  CHECK(b,"");
  CHECK(!b->shape, "this frame already has a geom attached");
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
    if(ats["rel_includes_mesh_center"]){
      mesh.center();
    }
//    if(c.length()>1e-8 && !ats["rel_includes_mesh_center"]){
//      frame->link->Q.addRelativeTranslation(c);
//      frame->ats.newNode<bool>({"rel_includes_mesh_center"}, {}, true);
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
#if 0
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
      if(!body->inertia) body->iner
      NIY;
//      frame->mass += mass;
//      frame->inertia += I;
    }
  }
#endif
}

void mlr::Shape::write(std::ostream& os) const {
  os <<"type=" <<type <<' ';
  os <<"size=[" <<size <<"] ";

  Node *n;
  if((n=frame->ats["color"])) os <<*n <<' ';
  if((n=frame->ats["mesh"])) os <<*n <<' ';
  if((n=frame->ats["meshscale"])) os <<*n <<' ';
  if(cont) os <<"contact, ";
}

#ifdef MLR_GL
void mlr::Shape::glDraw(OpenGL& gl) {
  //set name (for OpenGL selection)
  glPushName((frame->ID <<2) | 1);
  if(frame->K.orsDrawColors && !frame->K.orsDrawIndexColors) glColor(mesh.C); //color[0], color[1], color[2], color[3]*world.orsDrawAlpha);
  if(frame->K.orsDrawIndexColors) glColor3b((frame->ID>>16)&0xff, (frame->ID>>8)&0xff, frame->ID&0xff);


  double GLmatrix[16];
  frame->X.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);

  if(!frame->K.orsDrawShapes) {
    double scale=.33*(size(0)+size(1)+size(2) + 2.*size(3)); //some scale
    if(!scale) scale=1.;
    scale*=.3;
    glDrawAxes(scale);
    glColor(0, 0, .5);
    glDrawSphere(.1*scale);
  }
  if(frame->K.orsDrawShapes) {
    switch(type) {
      case mlr::ST_none: LOG(-1) <<"Shape '" <<frame->name <<"' has no joint type";  break;
      case mlr::ST_box:
        if(frame->K.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(frame->K.orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawBox(size(0), size(1), size(2));
        break;
      case mlr::ST_sphere:
        if(frame->K.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(frame->K.orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawSphere(size(3));
        break;
      case mlr::ST_cylinder:
        if(frame->K.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(frame->K.orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawCylinder(size(3), size(2));
        break;
      case mlr::ST_capsule:
        if(frame->K.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(frame->K.orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawCappedCylinder(size(3), size(2));
        break;
      case mlr::ST_retired_SSBox:
        HALT("deprecated??");
        if(frame->K.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(frame->K.orsDrawMeshes){
          if(!mesh.V.N) mesh.setSSBox(size(0), size(1), size(2), size(3));
          mesh.glDraw(gl);
        }else NIY;
        break;
      case mlr::ST_marker:
        if(frame->K.orsDrawMarkers){
          glDrawDiamond(size(0)/5., size(0)/5., size(0)/5.); glDrawAxes(size(0));
        }
        break;
      case mlr::ST_mesh:
        CHECK(mesh.V.N, "mesh needs to be loaded to draw mesh object");
        if(frame->K.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;
      case mlr::ST_ssCvx:
        CHECK(sscCore.V.N, "sscCore needs to be loaded to draw mesh object");
        if(!mesh.V.N) mesh.setSSCvx(sscCore, size(3));
        if(frame->K.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;
      case mlr::ST_ssBox:
        if(!mesh.V.N || !sscCore.V.N){
          sscCore.setBox();
          sscCore.scale(size(0)-2.*size(3), size(1)-2.*size(3), size(2)-2.*size(3));
          mesh.setSSCvx(sscCore, size(3));
        }
        if(frame->K.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;
      case mlr::ST_pointCloud:
        CHECK(mesh.V.N, "mesh needs to be loaded to draw point cloud object");
        if(frame->K.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;

      default: HALT("can't draw that geom yet");
    }
  }
  if(frame->K.orsDrawZlines) {
    glColor(0, .7, 0);
    glBegin(GL_LINES);
    glVertex3d(0., 0., 0.);
    glVertex3d(0., 0., -frame->X.pos.z);
    glEnd();
  }

  if(frame->K.orsDrawBodyNames && frame){
    glColor(1,1,1);
    glDrawText(frame->name, 0, 0, 0);
  }

  glPopName();
}

#endif


mlr::Inertia::Inertia(mlr::Frame *f, Inertia *copyInertia) : frame(f), type(BT_dynamic) {
  CHECK(!frame->inertia, "this frame already has inertia");
  frame->inertia = this;
  if(copyInertia){
    mass = copyInertia->mass;
    matrix = copyInertia->matrix;
    type = copyInertia->type;
    com = copyInertia->com;
    force = copyInertia->force;
    torque = copyInertia->torque;
  }
}

mlr::Inertia::~Inertia(){
  frame->inertia = NULL;
}

void mlr::Inertia::read(const Graph& ats){
  double d;
  if(ats["fixed"])       type=BT_static;
  if(ats["static"])      type=BT_static;
  if(ats["kinematic"])   type=BT_kinematic;
  if(ats.get(d,"dyntype")) type=(BodyType)d;
}


RUN_ON_INIT_BEGIN(frame)
JointL::memMove=true;
RUN_ON_INIT_END(frame)
