/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


/**
 * @file
 * @ingroup group_ors
 */
/**
 * @addtogroup group_ors
 * @{
 */


#undef abs
#include <algorithm>
#include <sstream>
#include <climits>
#include "kin.h"
#include "kin_swift.h"
#include "kin_physx.h"
#include "kin_ode.h"
#include <Geo/qhull.h>
#include <GeoOptim/geoOptim.h>
#include <Gui/opengl.h>
#include <Algo/algos.h>
#include <iomanip>

#ifndef MLR_ORS_ONLY_BASICS
#  include <Core/graph.h>
//#  include <Plot/plot.h>
#endif

#ifdef MLR_extern_ply
#  include <Geo/ply/ply.h>
#endif

#ifdef MLR_GL
#  include <GL/gl.h>
#  include <GL/glu.h>
#endif

#define ORS_NO_DYNAMICS_IN_FRAMES

#define SL_DEBUG_LEVEL 1
#define SL_DEBUG(l, x) if(l<=SL_DEBUG_LEVEL) x;

#define Qstate

void lib_ors(){ cout <<"force loading lib/ors" <<endl; }

#define LEN .2

#ifndef MLR_ORS_ONLY_BASICS

uint mlr::KinematicWorld::setJointStateCount = 0;

//===========================================================================
//
// contants
//

mlr::Body& NoBody = *((mlr::Body*)NULL);
mlr::Shape& NoShape = *((mlr::Shape*)NULL);
mlr::Joint& NoJoint = *((mlr::Joint*)NULL);
mlr::KinematicWorld& NoWorld = *((mlr::KinematicWorld*)NULL);

template<> const char* mlr::Enum<mlr::ShapeType>::names []={
  "ST_box", "ST_sphere", "ST_capsule", "ST_mesh", "ST_cylinder", "ST_marker", "ST_SSBox", "ST_pointCloud", "ST_ssCvx", "ST_ssBox", NULL
};

template<> const char* mlr::Enum<mlr::JointType>::names []={
  "JT_hingeX", "JT_hingeY", "JT_hingeZ", "JT_transX", "JT_transY", "JT_transZ", "JT_transXY", "JT_trans3", "JT_transXYPhi", "JT_universal", "JT_rigid", "JT_quatBall", "JT_phiTransXY", "JT_glue", "JT_free", NULL
};

template<> const char* mlr::Enum<mlr::KinematicSwitch::OperatorSymbol>::names []={
  "deleteJoint", "addJointZero", "addJointAtFrom", "addJointAtTo", "addArticulated", NULL
};

//===========================================================================
//
// Body implementations
//

//mlr::Body::Body() { reset(); }

//mlr::Body::Body(const Body& b) { reset(); *this=b; }

mlr::Body::Body(KinematicWorld& _world, const Body* copyBody):world(_world), vel(0), angvel(0) {
  reset();
  index=world.bodies.N;
  world.bodies.append(this);
  if(copyBody) *this=*copyBody;
}

mlr::Body::~Body() {
  reset();
  while(inLinks.N) delete inLinks.last();
  while(outLinks.N) delete outLinks.last();
  while(shapes.N) delete shapes.last();
  world.bodies.removeValue(this);
  listReindex(world.bodies);
}

void mlr::Body::reset() {
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

void mlr::Body::parseAts() {
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

void mlr::Body::write(std::ostream& os) const {
  if(!X.isZero()) os <<"pose=<T " <<X <<" > ";
  if(mass) os <<"mass=" <<mass <<' ';
  if(type!=BT_dynamic) os <<"dyntype=" <<(int)type <<' ';
//  uint i; Node *a;
//  for(Type *  a:  ats)
//      if(a->keys(0)!="X" && a->keys(0)!="pose") os <<*a <<' ';
}

void mlr::Body::read(std::istream& is) {
  reset();
  ats.read(is);
  if(!is.good()) HALT("body '" <<name <<"' read error: in ");
  parseAts();
}

namespace mlr {
std::ostream& operator<<(std::ostream& os, const Body& x) { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Shape& x) { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Joint& x) { x.write(os); return os; }
}

//===========================================================================
//
// Shape implementations
//

mlr::Shape::Shape(KinematicWorld &_world, Body& b, const Shape *copyShape, bool referenceMeshOnCopy): world(_world), /*ibody(UINT_MAX),*/ body(NULL) {
  reset();
  CHECK(&world,"you need at least a world to attach this shape to!");
  index=world.shapes.N;
  world.shapes.append(this);
  if(&b){
    body = &b;
    b.shapes.append(this);
  }
  if(copyShape) copy(*copyShape, referenceMeshOnCopy);
}

mlr::Shape::~Shape() {
  reset();
  if(body){
    body->shapes.removeValue(this);
    listReindex(body->shapes);
  }
  world.shapes.removeValue(this);
  listReindex(world.shapes);
}

void mlr::Shape::copy(const Shape& s, bool referenceMeshOnCopy){
  name=s.name; X=s.X; rel=s.rel; type=s.type;
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
  ats=s.ats;
}

void mlr::Shape::parseAts() {
  double d;
  arr x;
  mlr::String str;
  mlr::FileToken fil;
  ats.get(rel, "rel");
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
    Vector c = mesh.center();
    if(c.length()>1e-8 && !ats["rel_includes_mesh_center"]){
      rel.addRelativeTranslation(c);
      ats.newNode<bool>({"rel_includes_mesh_center"}, {}, true);
    }
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
  if(body) {
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
      body->mass += mass;
      body->inertia += I;
    }
  }
}

void mlr::Shape::reset() {
  type=ST_none;
  size = {1.,1.,1.};
  ats.clear();
  X.setZero();
  rel.setZero();
  mesh.V.clear();
  mesh.C.clear();
  mesh.C = consts<double>(.8, 3); //color[0]=color[1]=color[2]=.8; color[3]=1.;
  mesh_radius=0.;
  cont=false;
}

void mlr::Shape::write(std::ostream& os) const {
  os <<"type=" <<type <<' ';
//  os <<"size=[" <<size(0) <<' '<<size(1) <<' '<<size(2) <<' '<<size(3) <<"] ";
  if(!rel.isZero()) os <<"rel=<T " <<rel <<" > ";
  for(Node * a: ats)
    if(a->keys.N && a->keys(0)!="rel" && a->keys(0)!="type" /*&& a->keys(0)!="size"*/) os <<*a <<' ';
}

void mlr::Shape::read(std::istream& is) {
  reset();
  ats.read(is);
  if(!is.good()) HALT("shape read error");
  parseAts();
}

#ifdef MLR_GL
void mlr::Shape::glDraw(OpenGL& gl) {
  //set name (for OpenGL selection)
  glPushName((index <<2) | 1);
  if(world.orsDrawColors && !world.orsDrawIndexColors) glColor(mesh.C); //color[0], color[1], color[2], color[3]*world.orsDrawAlpha);
  if(world.orsDrawIndexColors) glColor3b((index>>16)&0xff, (index>>8)&0xff, index&0xff);


  double GLmatrix[16];
  X.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);

  if(!world.orsDrawShapes) {
    double scale=.33*(size(0)+size(1)+size(2) + 2.*size(3)); //some scale
    if(!scale) scale=1.;
    scale*=.3;
    glDrawAxes(scale);
    glColor(0, 0, .5);
    glDrawSphere(.1*scale);
  }
  if(world.orsDrawShapes) {
    switch(type) {
      case mlr::ST_none: LOG(-1) <<"Shape '" <<name <<"' has no joint type";  break;
      case mlr::ST_box:
        if(world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(world.orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawBox(size(0), size(1), size(2));
        break;
      case mlr::ST_sphere:
        if(world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(world.orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawSphere(size(3));
        break;
      case mlr::ST_cylinder:
        if(world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(world.orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawCylinder(size(3), size(2));
        break;
      case mlr::ST_capsule:
        if(world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(world.orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawCappedCylinder(size(3), size(2));
        break;
      case mlr::ST_retired_SSBox:
        HALT("deprecated??");
        if(world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(world.orsDrawMeshes){
          if(!mesh.V.N) mesh.setSSBox(size(0), size(1), size(2), size(3));
          mesh.glDraw(gl);
        }else NIY;
        break;
      case mlr::ST_marker:
        if(world.orsDrawMarkers){
          glDrawDiamond(size(0)/5., size(0)/5., size(0)/5.); glDrawAxes(size(0));
        }
        break;
      case mlr::ST_mesh:
        CHECK(mesh.V.N, "mesh needs to be loaded to draw mesh object");
        if(world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;
      case mlr::ST_ssCvx:
        CHECK(sscCore.V.N, "sscCore needs to be loaded to draw mesh object");
        if(!mesh.V.N) mesh.setSSCvx(sscCore, size(3));
        if(world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;
      case mlr::ST_ssBox:
        if(!mesh.V.N || !sscCore.V.N){
          sscCore.setBox();
          sscCore.scale(size(0)-2.*size(3), size(1)-2.*size(3), size(2)-2.*size(3));
          mesh.setSSCvx(sscCore, size(3));
        }
        if(world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;
      case mlr::ST_pointCloud:
        CHECK(mesh.V.N, "mesh needs to be loaded to draw point cloud object");
        if(world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;

      default: HALT("can't draw that geom yet");
    }
  }
  if(world.orsDrawZlines) {
    glColor(0, .7, 0);
    glBegin(GL_LINES);
    glVertex3d(0., 0., 0.);
    glVertex3d(0., 0., -X.pos.z);
    glEnd();
  }

  if(world.orsDrawBodyNames && body){
    glColor(1,1,1);
    glDrawText(body->name, 0, 0, 0);
  }

  glPopName();
}
#endif

uintA stringListToShapeIndices(const mlr::Array<const char*>& names, const mlr::Array<mlr::Shape*>& shapes) {
  uintA I(names.N);
  for(uint i=0; i<names.N; i++) {
    mlr::Shape *s = listFindByName(shapes, names(i));
    if(!s) HALT("shape name '"<<names(i)<<"' doesn't exist");
    I(i) = s->index;
  }
  return I;
}

uintA shapesToShapeIndices(const mlr::Array<mlr::Shape*>& shapes) {
  uintA I;
  resizeAs(I, shapes);
  for(uint i=0; i<shapes.N; i++) I.elem(i) = shapes.elem(i)->index;
  return I;
}

void makeConvexHulls(ShapeL& shapes, bool onlyContactShapes){
  for(mlr::Shape *s: shapes) if(!onlyContactShapes || s->cont) s->mesh.makeConvexHull();
}

void computeOptimalSSBoxes(ShapeL& shapes){
//  for(mlr::Shape *s: shapes) s->mesh.computeOptimalSSBox(s->mesh.V);
  for(uint i=0;i<shapes.N;i++){
    mlr::Shape *s=shapes(i);
    if(!(s->type==mlr::ST_mesh && s->mesh.V.N)) continue;
    mlr::Transformation t;
    arr x;
    computeOptimalSSBox(s->mesh, x, t, s->mesh.V);
    s->type = mlr::ST_ssBox;
    s->size(0)=2.*x(0); s->size(1)=2.*x(1); s->size(2)=2.*x(2); s->size(3)=x(3);
    s->mesh.setSSBox(s->size(0), s->size(1), s->size(2), s->size(3));
    s->rel.appendTransformation(t);
  }
}

void computeMeshNormals(ShapeL& shapes){
  for(mlr::Shape *s: shapes){
    if(s->mesh.V.d0!=s->mesh.Vn.d0 || s->mesh.T.d0!=s->mesh.Tn.d0) s->mesh.computeNormals();
    if(s->sscCore.V.d0!=s->sscCore.Vn.d0 || s->sscCore.T.d0!=s->sscCore.Tn.d0) s->sscCore.computeNormals();
  }
}


//===========================================================================
//
// Joint implementations
//

bool always_unlocked(void*) { return false; }

mlr::Joint::Joint(KinematicWorld& G, Body *f, Body *t, const Joint* copyJoint)
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

uint mlr::Joint::qDim() {
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


mlr::Proxy::Proxy() {
  colorCode = 0;
}

void mlr::Proxy::glDraw(OpenGL& gl){
  glLoadIdentity();
  if(!colorCode){
    if(d>0.) glColor(.8,.2,.2);
    else glColor(1,0,0);
  }else glColor(colorCode);
  glBegin(GL_LINES);
  glVertex3dv(posA.p());
  glVertex3dv(posB.p());
  glEnd();
  mlr::Transformation f;
  f.pos=posA;
  f.rot.setDiff(mlr::Vector(0, 0, 1), posA-posB);
  double GLmatrix[16];
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glDisable(GL_CULL_FACE);
  glDrawDisk(.02);
  glEnable(GL_CULL_FACE);

  f.pos=posB;
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glDrawDisk(.02);
}

//===========================================================================
//
// Graph implementations
//

namespace mlr{
  struct sKinematicWorld{
    OpenGL *gl;
    SwiftInterface *swift;
    PhysXInterface *physx;
    OdeInterface *ode;
    bool swiftIsReference;
    sKinematicWorld():gl(NULL), swift(NULL), physx(NULL), ode(NULL), swiftIsReference(false) {}
    ~sKinematicWorld(){
      if(gl) delete gl;
      if(swift && !swiftIsReference) delete swift;
      if(physx) delete physx;
      if(ode) delete ode;
    }
  };
}

mlr::KinematicWorld::KinematicWorld():s(NULL),q_agent(0),isLinkTree(false) {
  bodies.memMove=joints.memMove=shapes.memMove=proxies.memMove=true;
  s=new sKinematicWorld;
}

mlr::KinematicWorld::KinematicWorld(const mlr::KinematicWorld& other):s(NULL),q_agent(0),isLinkTree(false)  {
  bodies.memMove=joints.memMove=shapes.memMove=proxies.memMove=true;
  s=new sKinematicWorld;
  copy( other );
}

mlr::KinematicWorld::KinematicWorld(const char* filename):s(NULL),q_agent(0),isLinkTree(false)  {
  bodies.memMove=joints.memMove=shapes.memMove=proxies.memMove=true;
  s=new sKinematicWorld;
  init(filename);
}
mlr::KinematicWorld::~KinematicWorld() {
  clear();
  delete s;
  s=NULL;
}

void mlr::KinematicWorld::init(const char* filename) {
  *this <<FILE(filename);
}

void mlr::KinematicWorld::clear() {
  qdim.clear();
  q.clear();
  qdot.clear();
  listDelete(proxies); checkConsistency();
  while(shapes.N){ delete shapes.last(); checkConsistency(); }
  while(joints.N){ delete joints.last(); checkConsistency();}
  while(bodies.N){ delete bodies.last(); checkConsistency();}
  isLinkTree=false;
}

void mlr::KinematicWorld::copy(const mlr::KinematicWorld& G, bool referenceMeshesAndSwiftOnCopy) {
  clear();
#if 1
  listCopy(proxies, G.proxies);
  for(Body *b:G.bodies) new Body(*this, b);
  for(Shape *s:G.shapes){
//    if(referenceMeshesAndSwiftOnCopy && !s->mesh.Vn.N) s->mesh.computeNormals(); // the copy references these normals -> if they're not precomputed, you can never display the copy
    new Shape(*this, (s->body?*bodies(s->body->index):NoBody), s, referenceMeshesAndSwiftOnCopy);
  }
  for(Joint *j:G.joints){
    Joint *jj=
        new Joint(*this, bodies(j->from->index), bodies(j->to->index), j);
    if(j->mimic) jj->mimic = joints(j->mimic->index);
  }
  if(referenceMeshesAndSwiftOnCopy){
    s->swift = G.s->swift;
    s->swiftIsReference=true;
  }
  q = G.q;
  qdot = G.qdot;
  qdim = G.qdim;
  q_agent = G.q_agent;
  isLinkTree = G.isLinkTree;
#else
  q = G.q;
  qdot = G.qdot;
  qdim = G.qdim;
  q_agent = G.q_agent;
  isLinkTree = G.isLinkTree;
  listCopy(proxies, G.proxies);
  listCopy(joints, G.joints);
  for(Joint *j: joints) if(j->mimic){
    mlr::String jointName;
    bool good = j->ats.find<mlr::String>(jointName, "mimic");
    CHECK(good, "something is wrong");
    j->mimic = listFindByName(G.joints, jointName);
    if(!j->mimic) HALT("The joint '" <<*j <<"' is declared coupled to '" <<jointName <<"' -- but that doesn't exist!");
    j->type = j->mimic->type;
  }
  listCopy(shapes, G.shapes);
  listCopy(bodies, G.bodies);
  graphMakeLists(bodies, joints);
  for(Body *  b:  bodies) b->shapes.clear();
  for(Shape *  s:  shapes) {
    b=bodies(s->ibody);
    s->body=b;
    b->shapes.append(s);
  }
#endif
}

/** @brief transforms (e.g., translates or rotates) the joints coordinate system):
  `adds' the transformation f to A and its inverse to B */
void mlr::KinematicWorld::transformJoint(mlr::Joint *e, const mlr::Transformation &f) {
  e->A = e->A * f;
  e->B = -f * e->B;
}

void mlr::KinematicWorld::makeLinkTree() {
//  for(Shape *s: shapes) {
//    if(s->mesh.V.N) s->mesh.transform(s->rel);
//    if(s->sscCore.V.N) s->sscCore.transform(s->rel);
//    s->rel.setZero();
//  }
  for(Joint *j: joints) {
    for(Shape *s: j->to->shapes)  s->rel = j->B * s->rel;
    for(Joint *j2: j->to->outLinks) j2->A = j->B * j2->A;
    j->B.setZero();
  }
  isLinkTree=true;
}

void mlr::KinematicWorld::jointSort(){
  BodyL order = graphGetTopsortOrder(bodies, joints);
  JointL joints_new;
  for(mlr::Body *b: order) if(b->inLinks.N) joints_new.append(b->inLinks.scalar());
  CHECK_EQ(joints_new.N, joints.N, "");
  joints = joints_new;
  for_list(mlr::Joint, j, joints) j->index=j_COUNT;
  qdim.clear();
  q.clear();
  qdot.clear();
  analyzeJointStateDimensions();
}

/** @brief KINEMATICS: given the (absolute) frames of root nodes and the relative frames
    on the edges, this calculates the absolute frames of all other nodes (propagating forward
    through trees and testing consistency of loops). */
void mlr::KinematicWorld::calc_fwdPropagateFrames() {
  for(Joint *j:joints){
    j->X = j->from->X;
    j->X.appendTransformation(j->A);
    if(j->type==JT_hingeX || j->type==JT_transX)  j->axis = j->X.rot.getX();
    if(j->type==JT_hingeY || j->type==JT_transY)  j->axis = j->X.rot.getY();
    if(j->type==JT_hingeZ || j->type==JT_transZ)  j->axis = j->X.rot.getZ();
    if(j->type==JT_transXYPhi)  j->axis = j->X.rot.getZ();
    if(j->type==JT_phiTransXY)  j->axis = j->X.rot.getZ();

    j->to->X=j->X;
#if 1
    j->to->X.appendTransformation(j->Q);
#else
    j->applyTransformation(j->to->X, q);
#endif
    CHECK_EQ(j->to->X.pos.x, j->to->X.pos.x, "");
    if(!isLinkTree) j->to->X.appendTransformation(j->B);
  }
  calc_fwdPropagateShapeFrames();
}

void mlr::KinematicWorld::calc_fwdPropagateShapeFrames() {
  for(Shape *s: shapes) {
    if(s->body){
      s->X = s->body->X;
      /*if(!isLinkTree)*/ s->X.appendTransformation(s->rel);
    }else{
      s->X = s->rel;
    }
  }
}

void mlr::KinematicWorld::calc_fwdPropagateVelocities(){
  mlr::Transformation f;
  Vector q_vel, q_angvel;
  for(Joint *j : joints){ //this has no bailout for loopy graphs!
    Body *from = j->from;
    Body *to = j->to;
    to->vel = from->vel;
    to->angvel = from->angvel;

    if(j->type==JT_hingeX){
        q_vel.setZero();
        q_angvel.set(qdot(j->qIndex) ,0., 0.);
    }else if(j->type==JT_transX){
        q_vel.set(qdot(j->qIndex), 0., 0.);
        q_angvel.setZero();
    }else if(j->type==JT_rigid){
        q_vel.setZero();
        q_angvel.setZero();
    }else if(j->type==JT_transXYPhi){
        q_vel.set(qdot(j->qIndex), qdot(j->qIndex+1), 0.);
        q_angvel.set(0.,0.,qdot(j->qIndex+2));
    }else NIY;

    Matrix R = j->X.rot.getMatrix();
    Vector qV(R*q_vel); //relative vel in global coords
    Vector qW(R*q_angvel); //relative ang vel in global coords
    to->vel += from->angvel^(to->X.pos - from->X.pos);
    if(!isLinkTree) to->vel += qW^(to->X.pos - j->X.pos);
    to->vel += qV;
    to->angvel += qW;
  }
}

void mlr::KinematicWorld::calc_missingAB_from_BodyAndJointFrames() {
  for(Joint *e: joints) {
    if(!e->X.isZero() && e->A.isZero() && e->B.isZero()) {
      e->A.setDifference(e->from->X, e->X);
      e->B.setDifference(e->X, e->to->X);
    }
  }
}

/** @brief given the absolute frames of all nodes and the two rigid (relative)
    frames A & B of each edge, this calculates the dynamic (relative) joint
    frame X for each edge (which includes joint transformation and errors) */
void mlr::KinematicWorld::calc_Q_from_BodyFrames() {
  for(Joint *j:joints) {
    mlr::Transformation A(j->from->X), B(j->to->X);
    A.appendTransformation(j->A);
    B.appendInvTransformation(j->B);
    j->Q.setDifference(A, B);
  }
}

/** @brief in all edge frames: remove any displacements, velocities and non-x rotations.
    After this, edges and nodes are not coherent anymore. You might want to call
    calcBodyFramesFromJoints() */
void mlr::KinematicWorld::clearJointErrors() {
  mlr::Vector xaxis(1, 0, 0);
  for(Joint *j:joints) {
    j->Q.rot.alignWith(xaxis);
  }
}

arr mlr::KinematicWorld::naturalQmetric(double power) const {
  HALT("don't use this anymore. use getHmetric instead");
#if 0
  if(!q.N) getJointStateDimension();
  arr Wdiag(q.N);
  Wdiag=1.;
  return Wdiag;
#else
  //compute generic q-metric depending on tree depth
  arr BM(bodies.N);
  BM=1.;
  for(uint i=BM.N; i--;) {
    for(uint j=0; j<bodies(i)->outLinks.N; j++) {
      BM(i) = mlr::MAX(BM(bodies(i)->outLinks(j)->to->index)+1., BM(i));
//      BM(i) += BM(bodies(i)->outLinks(j)->to->index);
    }
  }
  if(!q.N) getJointStateDimension();
  arr Wdiag(q.N);
  for(Joint *j: joints) {
    for(uint i=0; i<j->qDim(); i++) {
      if(j->agent==q_agent) Wdiag(j->qIndex+i) = ::pow(BM(j->to->index), power);
    }
  }
  return Wdiag;
#endif
}

/** @brief revert the topological orientation of a joint (edge),
   e.g., when choosing another body as root of a tree */
void mlr::KinematicWorld::revertJoint(mlr::Joint *j) {
  cout <<"reverting edge (" <<j->from->name <<' ' <<j->to->name <<")" <<endl;
  //revert
  j->from->outLinks.removeValue(j);
  j->to->inLinks.removeValue(j);
  Body *b=j->from; j->from=j->to; j->to=b;
  j->from->outLinks.append(j);
  j->to->inLinks.append(j);
  listReindex(j->from->outLinks);
  listReindex(j->from->inLinks);
  checkConsistency();

  mlr::Transformation f;
  f=j->A;
  j->A.setInverse(j->B);
  j->B.setInverse(f);
  f=j->Q;
  j->Q.setInverse(f);
}

/** @brief re-orient all joints (edges) such that n becomes
  the root of the configuration */
void mlr::KinematicWorld::reconfigureRoot(Body *root) {
  mlr::Array<Body*> list, list2;
  Body **m,**mstop;
  list.append(root);
  uintA level(bodies.N);
  level=0;
  int i=0;
  
  while(list.N>0) {
    i++;
    list2.clear();
    mstop=list.p+list.N;
    for(m=list.p; m!=mstop; m++) {
      level((*m)->index)=i;
      for_list(Joint,  e,  (*m)->inLinks) {
        if(!level(e->from->index)) { revertJoint(e); e_COUNT--; }
      }
      for(Joint *e: (*m)->outLinks) list2.append(e->to);
    }
    list=list2;
  }
  
  graphTopsort(bodies, joints);
}

void mlr::KinematicWorld::analyzeJointStateDimensions() {
  uint maxagent=0;
  for(Joint *j: joints) if(j->agent>maxagent) maxagent=j->agent;
  qdim.resize(maxagent+1);
  qdim.setZero();
  for(Joint *j: joints) {
    if(!j->mimic){
      j->qIndex = qdim(j->agent);
      qdim(j->agent) += j->qDim();
    }else{
      CHECK_EQ(j->agent, j->mimic->agent, "");
      j->qIndex = j->mimic->qIndex;
    }
  }
}

/** @brief returns the joint (actuator) dimensionality */
uint mlr::KinematicWorld::getJointStateDimension(int agent) const {
  if(agent==-1) agent=q_agent;
  CHECK(agent!=INT_MAX,"");
  if(!qdim.N){
    CHECK(!q.N && !qdim.N,"you've change q-dim (locked joints?) without clearing q,qdot");
    ((KinematicWorld*)this)->analyzeJointStateDimensions();
  }
  CHECK((uint)agent<qdim.N,"don't have agent # " <<agent <<" (analyzeJointStateDimensions returned [" <<qdim <<"])");
  return qdim(agent);
}

void mlr::KinematicWorld::getJointState(arr &_q, arr& _qdot, int agent) const {
  if(!qdim.N) ((KinematicWorld*)this)->analyzeJointStateDimensions();
  if(q.N!=getJointStateDimension(agent)) ((KinematicWorld*)this)->calc_q_from_Q(agent);

  _q=q;
  if(&_qdot){
    _qdot=qdot;
    if(!_qdot.N) _qdot.resizeAs(q).setZero();
  }
}

arr mlr::KinematicWorld::getJointState(int agent) const {
  if(!qdim.N) ((KinematicWorld*)this)->analyzeJointStateDimensions();
  if(q.N!=getJointStateDimension(agent)) ((KinematicWorld*)this)->calc_q_from_Q(agent);

  return q;
}

/** @brief returns the vector of joint limts */
arr mlr::KinematicWorld::getLimits() const {
  uint N=getJointStateDimension();
  arr limits(N,2);
  limits.setZero();
  for(Joint *j: joints) if(j->agent==q_agent){
    uint i=j->qIndex;
    uint d=j->qDim();
    for(uint k=0;k<d;k++){//in case joint has multiple dimensions
      if(j->limits.N){
        limits(i+k,0)=j->limits(0); //lo
        limits(i+k,1)=j->limits(1); //up
      }else{
        limits(i+k,0)=-1.; //lo
        limits(i+k,1)=+1.; //up
      }
    }
  }
//  cout <<"limits=" <<limits <<endl;
  return limits;
}

void mlr::KinematicWorld::zeroGaugeJoints() {
  Joint *e;
  mlr::Vector w;
  for(Body *n: bodies) if(n->type!=BT_static && n->inLinks.N) {
    e=n->inLinks(0);
    if(e) {
      e->A.appendTransformation(e->Q);
      e->Q.setZero();
    }
  }
}

arr mlr::KinematicWorld::calc_q_from_Q(mlr::Joint* j) {
  arr q;
  switch(j->type) {
    case JT_hingeX:
    case JT_hingeY:
    case JT_hingeZ: {
      q.resize(1);
      //angle
      mlr::Vector rotv;
      j->Q.rot.getRad(q(0), rotv);
      if(q(0)>MLR_PI) q(0)-=MLR_2PI;
      if(j->type==JT_hingeX && rotv*Vector_x<0.) q(0)=-q(0);
      if(j->type==JT_hingeY && rotv*Vector_y<0.) q(0)=-q(0);
      if(j->type==JT_hingeZ && rotv*Vector_z<0.) q(0)=-q(0);
    } break;

    case JT_universal: {
      q.resize(2);
      //angle
      if(fabs(j->Q.rot.w)>1e-15) {
        q(0) = 2.0 * atan(j->Q.rot.x/j->Q.rot.w);
        q(1) = 2.0 * atan(j->Q.rot.y/j->Q.rot.w);
      } else {
        q(0) = MLR_PI;
        q(1) = MLR_PI;
      }
    } break;

    case JT_quatBall: {
      q.resize(4);
      q(0)=j->Q.rot.w;
      q(1)=j->Q.rot.x;
      q(2)=j->Q.rot.y;
      q(3)=j->Q.rot.z;
    } break;

    case JT_transX: {
      q.resize(1);
      q(0)=j->Q.pos.x;
    } break;
    case JT_transY: {
      q.resize(1);
      q(0)=j->Q.pos.y;
    } break;
    case JT_transZ: {
      q.resize(1);
      q(0)=j->Q.pos.z;
    } break;
    case JT_transXY: {
      q.resize(1);
      q(0)=j->Q.pos.x;  
      q(1)=j->Q.pos.y;
    } break;
    case JT_transXYPhi: {
      q.resize(3);
      q(0)=j->Q.pos.x;
      q(1)=j->Q.pos.y;
      mlr::Vector rotv;
      j->Q.rot.getRad(q(2), rotv);
      if(q(2)>MLR_PI) q(2)-=MLR_2PI;
      if(rotv*Vector_z<0.) q(2)=-q(2);
    } break;
    case JT_phiTransXY: {
      q.resize(3);
      mlr::Vector rotv;
      j->Q.rot.getRad(q(0), rotv);
      if(q(0)>MLR_PI) q(0)-=MLR_2PI;
      if(rotv*Vector_z<0.) q(0)=-q(0);
      mlr::Vector relpos = j->Q.rot/j->Q.pos;
      q(1)=relpos.x;
      q(2)=relpos.y;
    } break;
    case JT_trans3: {
      q.resize(3);
      q(0)=j->Q.pos.x;
      q(1)=j->Q.pos.y;
      q(2)=j->Q.pos.z;
    } break;
    case JT_glue:
    case JT_rigid:
      break;
    case JT_free:
      q.resize(7);
      q(0)=j->Q.pos.x;
      q(1)=j->Q.pos.y;
      q(2)=j->Q.pos.z;
      q(3)=j->Q.rot.w;
      q(4)=j->Q.rot.x;
      q(5)=j->Q.rot.y;
      q(6)=j->Q.rot.z;
      break;
    default: NIY;
  }
  return q;
}

void mlr::KinematicWorld::calc_q_from_Q(int agent) {
  if(agent==-1) agent = q_agent;
//  mlr::Quaternion rot;
  
  uint N=getJointStateDimension(agent);
  q.resize(N);
  qdot.resize(N).setZero();

  uint n=0;
  for(Joint *j: joints) if(j->agent==(uint)agent){
    if(j->mimic) continue; //don't count dependent joints
    CHECK_EQ(j->qIndex,n,"joint indexing is inconsistent");
    arr joint_q = calc_q_from_Q(j);
    //TODO is there a better way?
    for(uint i=0; i<joint_q.N; ++i)
      q(n+i) = joint_q(i);
    n += joint_q.N;
  }
  CHECK_EQ(n,N,"");
}

void mlr::KinematicWorld::calc_Q_from_q(int agent){
  if(agent==-1) agent = q_agent;
  uint n=0;
  for(Joint *j: joints) if(j->agent==(uint)agent){
    if(j->mimic){
      j->Q=j->mimic->Q;
    }else{
      CHECK_EQ(j->qIndex,n,"joint indexing is inconsistent");
      switch(j->type) {
        case JT_hingeX: {
          j->Q.rot.setRadX(q.elem(n));
          n++;
        } break;

        case JT_hingeY: {
          j->Q.rot.setRadY(q.elem(n));
          n++;
        } break;

        case JT_hingeZ: {
          j->Q.rot.setRadZ(q.elem(n));
          n++;
        } break;

        case JT_universal:{
          mlr::Quaternion rot1, rot2;
          rot1.setRadX(q.elem(n));
          rot2.setRadY(q.elem(n+1));
          j->Q.rot = rot1*rot2;
          n+=2;
        } break;

        case JT_quatBall:{
          j->Q.rot.set(q.p+n);
          {
              double n=j->Q.rot.normalization();
              if(n<.5 || n>2.) LOG(-1) <<"quat normalization is extreme: " <<n <<endl;
          }
          j->Q.rot.normalize();
          j->Q.rot.isZero=false; //WHY? (gradient check fails without!)
          n+=4;
        } break;

        case JT_free:{
          j->Q.pos.set(q.p+n);
          j->Q.rot.set(q.p+n+3);
          {
              double n=j->Q.rot.normalization();
              if(n<.5 || n>2.) LOG(-1) <<"quat normalization is extreme: " <<n <<endl;
          }
          j->Q.rot.normalize();
          j->Q.rot.isZero=false;
          n+=7;
        } break;

        case JT_transX: {
          j->Q.pos = q.elem(n)*Vector_x;
          n++;
        } break;

        case JT_transY: {
          j->Q.pos = q.elem(n)*Vector_y;
          n++;
        } break;

        case JT_transZ: {
          j->Q.pos = q.elem(n)*Vector_z;
          n++;
        } break;

        case JT_transXY: {
          j->Q.pos.set(q.elem(n), q.elem(n+1), 0.);
          n+=2;
        } break;

        case JT_trans3: {
          j->Q.pos.set(q.elem(n), q.elem(n+1), q.elem(n+2));
          n+=3;
        } break;

        case JT_transXYPhi: {
          j->Q.pos.set(q.elem(n), q.elem(n+1), 0.);
          j->Q.rot.setRadZ(q.elem(n+2));
          n+=3;
        } break;

        case JT_phiTransXY: {
          j->Q.rot.setRadZ(q.elem(n));
          j->Q.pos = j->Q.rot*Vector(q.elem(n+1), q.elem(n+2), 0.);
          n+=3;
        } break;

        case JT_glue:
        case JT_rigid:
          j->Q.setZero();
          break;
        default: NIY;
      }
    }
  }

  CHECK_EQ(n,q.N,"");
}


/** @brief sets the joint state vectors separated in positions and
  velocities */
void mlr::KinematicWorld::setJointState(const arr& _q, const arr& _qdot, int agent) {
  setJointStateCount++; //global counter

  uint N=getJointStateDimension(agent);
  CHECK(_q.N==N && (!(&_qdot) || _qdot.N==N), "wrong joint state dimensionalities");
  q=_q;
  if(&_qdot) qdot=_qdot; else qdot.clear();

  calc_Q_from_q(agent);

  calc_fwdPropagateFrames();
}

void mlr::KinematicWorld::setAgent(uint agent){
  if(agent==q_agent) return; //nothing to do
  q_agent = agent;
  calc_q_from_Q();
}



//===========================================================================
//
// core: kinematics and dynamics
//

/** @brief return the jacobian \f$J = \frac{\partial\phi_i(q)}{\partial q}\f$ of the position
  of the i-th body (3 x n tensor)*/
void mlr::KinematicWorld::kinematicsPos(arr& y, arr& J, Body *b, const mlr::Vector& rel) const {
  if(!b){
    MLR_MSG("WARNING: calling kinematics for NULL body");
    if(&y) y.resize(3).setZero();
    if(&J) J.resize(3, getJointStateDimension()).setZero();
    return;
  }

  //get position
  mlr::Vector pos_world = b->X.pos;
  if(&rel) pos_world += b->X.rot*rel;
  if(&y) y = conv_vec2arr(pos_world); //return the output
  if(!&J) return; //do not return the Jacobian

  //get Jacobian
  uint N=getJointStateDimension();
  J.resize(3, N).setZero();
  if(b->inLinks.N) { //body a has no inLinks -> zero jacobian
    Joint *j=b->inLinks(0);
    while(j) { //loop backward down the kinematic tree
      uint j_idx=j->qIndex;
      if(j->agent==q_agent && j_idx>=N) CHECK(j->type==JT_glue || j->type==JT_rigid, "");
      if(j->agent==q_agent && j_idx<N){
        if(j->type==JT_hingeX || j->type==JT_hingeY || j->type==JT_hingeZ) {
          mlr::Vector tmp = j->axis ^ (pos_world-j->X.pos);
          J(0, j_idx) += tmp.x;
          J(1, j_idx) += tmp.y;
          J(2, j_idx) += tmp.z;
        }
        else if(j->type==JT_transX || j->type==JT_transY || j->type==JT_transZ) {
          J(0, j_idx) += j->axis.x;
          J(1, j_idx) += j->axis.y;
          J(2, j_idx) += j->axis.z;
        }
        else if(j->type==JT_transXY) {
          if(j->mimic) NIY;
          arr R = j->X.rot.getArr();
          J.setMatrixBlock(R.sub(0,-1,0,1), 0, j_idx);
        }
        else if(j->type==JT_transXYPhi) {
          if(j->mimic) NIY;
          arr R = j->X.rot.getArr();
          J.setMatrixBlock(R.sub(0,-1,0,1), 0, j_idx);
          mlr::Vector tmp = j->axis ^ (pos_world-(j->X.pos + j->X.rot*j->Q.pos));
          J(0, j_idx+2) += tmp.x;
          J(1, j_idx+2) += tmp.y;
          J(2, j_idx+2) += tmp.z;
        }
        else if(j->type==JT_phiTransXY) {
          if(j->mimic) NIY;
          mlr::Vector tmp = j->axis ^ (pos_world-j->X.pos);
          J(0, j_idx) += tmp.x;
          J(1, j_idx) += tmp.y;
          J(2, j_idx) += tmp.z;
          arr R = (j->X.rot*j->Q.rot).getArr();
          J.setMatrixBlock(R.sub(0,-1,0,1), 0, j_idx+1);
        }
        if(j->type==JT_trans3 || j->type==JT_free) {
          if(j->mimic) NIY;
          arr R = j->X.rot.getArr();
          J.setMatrixBlock(R, 0, j_idx);
        }
        if(j->type==JT_quatBall || j->type==JT_free) {
          uint offset = (j->type==JT_free)?3:0;
          arr Jrot = j->X.rot.getArr() * j->Q.rot.getJacobian(); //transform w-vectors into world coordinate
          Jrot = crossProduct(Jrot, conv_vec2arr(pos_world-(j->X.pos+j->X.rot*j->Q.pos)) ); //cross-product of all 4 w-vectors with lever
          Jrot /= sqrt(sumOfSqr( q({j->qIndex+offset, j->qIndex+offset+3}) )); //account for the potential non-normalization of q
          for(uint i=0;i<4;i++) for(uint k=0;k<3;k++) J(k,j_idx+offset+i) += Jrot(k,i);
        }
      }
      if(!j->from->inLinks.N) break;
      j=j->from->inLinks(0);
    }
  }
}

/** @brief return the jacobian \f$J = \frac{\partial\phi_i(q)}{\partial q}\f$ of the position
  of the i-th body W.R.T. the 6 axes of an arbitrary shape-frame, NOT the robot's joints (3 x 6 tensor)
  WARNING: this does not check if s is actually in the kinematic chain from root to b.
*/
void mlr::KinematicWorld::kinematicsPos_wrtFrame(arr& y, arr& J, Body *b, const mlr::Vector& rel, Shape *s) const {
  if(!b && &J){ J.resize(3, getJointStateDimension()).setZero();  return; }

  //get position
  mlr::Vector pos_world = b->X.pos;
  if(&rel) pos_world += b->X.rot*rel;
  if(&y) y = conv_vec2arr(pos_world); //return the output
  if(!&J) return; //do not return the Jacobian

  //get Jacobian
  J.resize(3, 6).setZero();
  mlr::Vector diff = pos_world - s->X.pos;
  mlr::Array<mlr::Vector> axes = {s->X.rot.getX(), s->X.rot.getY(), s->X.rot.getZ()};

  //3 translational axes
  for(uint i=0;i<3;i++){
    J(0, i) += axes(i).x;
    J(1, i) += axes(i).y;
    J(2, i) += axes(i).z;
  }

  //3 rotational axes
  for(uint i=0;i<3;i++){
    mlr::Vector tmp = axes(i) ^ diff;
    J(0, 3+i) += tmp.x;
    J(1, 3+i) += tmp.y;
    J(2, 3+i) += tmp.z;
  }
}

/** @brief return the Hessian \f$H = \frac{\partial^2\phi_i(q)}{\partial q\partial q}\f$ of the position
  of the i-th body (3 x n x n tensor) */
void mlr::KinematicWorld::hessianPos(arr& H, Body *b, mlr::Vector *rel) const {
  HALT("this is buggy: a sign error: see examples/Kin/ors testKinematics");
  Joint *j1, *j2;
  uint j1_idx, j2_idx;
  mlr::Vector tmp, pos_a;
  
  uint N=getJointStateDimension();
  
  //initialize Jacobian
  H.resize(3, N, N);
  H.setZero();
  
  //get reference frame
  pos_a = b->X.pos;
  if(rel) pos_a += b->X.rot*(*rel);
  
  if(b->inLinks.N) {
    j1=b->inLinks(0);
    while(j1) {
      CHECK_EQ(j1->agent,q_agent,"NIY");
      j1_idx=j1->qIndex;

      j2=j1;
      while(j2) {
        CHECK_EQ(j2->agent,q_agent,"NIY");
        j2_idx=j2->qIndex;

        if(j1->type>=JT_hingeX && j1->type<=JT_hingeZ && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //both are hinges
          tmp = j2->axis ^ (j1->axis ^ (pos_a-j1->X.pos));
          H(0, j1_idx, j2_idx) = H(0, j2_idx, j1_idx) = tmp.x;
          H(1, j1_idx, j2_idx) = H(1, j2_idx, j1_idx) = tmp.y;
          H(2, j1_idx, j2_idx) = H(2, j2_idx, j1_idx) = tmp.z;
        }
        else if(j1->type>=JT_transX && j1->type<=JT_transZ && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans, j=hinge
          tmp = j1->axis ^ j2->axis;
          H(0, j1_idx, j2_idx) = H(0, j2_idx, j1_idx) = tmp.x;
          H(1, j1_idx, j2_idx) = H(1, j2_idx, j1_idx) = tmp.y;
          H(2, j1_idx, j2_idx) = H(2, j2_idx, j1_idx) = tmp.z;
        }
        else if(j1->type==JT_transXY && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans3, j=hinge
          NIY;
        }
        else if(j1->type==JT_transXYPhi && j1->type==JT_phiTransXY && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans3, j=hinge
          NIY;
        }
        else if(j1->type==JT_trans3 && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans3, j=hinge
          Matrix R,A;
          j1->X.rot.getMatrix(R.p());
          A.setSkew(j2->axis);
          R = R*A;
          H(0, j1_idx  , j2_idx) = H(0, j2_idx  , j1_idx) = R.m00;
          H(1, j1_idx  , j2_idx) = H(1, j2_idx  , j1_idx) = R.m10;
          H(2, j1_idx  , j2_idx) = H(2, j2_idx  , j1_idx) = R.m20;
          H(0, j1_idx+1, j2_idx) = H(0, j2_idx, j1_idx+1) = R.m01;
          H(1, j1_idx+1, j2_idx) = H(1, j2_idx, j1_idx+1) = R.m11;
          H(2, j1_idx+1, j2_idx) = H(2, j2_idx, j1_idx+1) = R.m21;
          H(0, j1_idx+2, j2_idx) = H(0, j2_idx, j1_idx+2) = R.m02;
          H(1, j1_idx+2, j2_idx) = H(1, j2_idx, j1_idx+2) = R.m12;
          H(2, j1_idx+2, j2_idx) = H(2, j2_idx, j1_idx+2) = R.m22;
        }
        else if(j1->type>=JT_hingeX && j1->type<=JT_hingeZ && j2->type>=JT_transX && j2->type<=JT_trans3) { //i=hinge, j=trans
          //nothing! Hessian is zero (ej is closer to root than ei)
        }
        else NIY;

        if(!j2->from->inLinks.N) break;
        j2=j2->from->inLinks(0);
      }
      if(!j1->from->inLinks.N) break;
      j1=j1->from->inLinks(0);
    }
  }
}

/* takes the joint state x and returns the jacobian dz of
   the position of the ith body (w.r.t. all joints) -> 2D array */
/// Jacobian of the i-th body's z-orientation vector
void mlr::KinematicWorld::kinematicsVec(arr& y, arr& J, Body *b, const mlr::Vector& vec) const {
  //get the vectoreference frame
  mlr::Vector vec_referene;
  if(&vec) vec_referene = b->X.rot*vec;
  else     vec_referene = b->X.rot.getZ();
  if(&y) y = conv_vec2arr(vec_referene); //return the vec
  if(&J){
    arr A;
    axesMatrix(A, b);
    J = crossProduct(A, conv_vec2arr(vec_referene));
  }
}

/* takes the joint state x and returns the jacobian dz of
   the position of the ith body (w.r.t. all joints) -> 2D array */
/// Jacobian of the i-th body's z-orientation vector
void mlr::KinematicWorld::kinematicsQuat(arr& y, arr& J, Body *b) const { //TODO: allow for relative quat
  mlr::Quaternion rot_b = b->X.rot;
  if(&y) y = conv_quat2arr(rot_b); //return the vec
  if(&J){
    arr A;
    axesMatrix(A, b);
    J.resize(4, A.d1);
    for(uint i=0;i<J.d1;i++){
      mlr::Quaternion tmp(0., 0.5*A(0,i), 0.5*A(1,i), 0.5*A(2,i) ); //this is unnormalized!!
      tmp = tmp * rot_b;
      J(0, i) = tmp.w;
      J(1, i) = tmp.x;
      J(2, i) = tmp.y;
      J(3, i) = tmp.z;
    }
  }
}

//* This Jacobian directly gives the implied rotation vector: multiplied with \dot q it gives the angular velocity of body b */
void mlr::KinematicWorld::axesMatrix(arr& J, Body *b) const {
  uint N = getJointStateDimension();
  J.resize(3, N).setZero();
  if(b->inLinks.N) {
    Joint *j=b->inLinks(0);
    while(j) { //loop backward down the kinematic tree
      uint j_idx=j->qIndex;
      if(j->agent==q_agent && j_idx>=N) CHECK(j->type==JT_glue || j->type==JT_rigid, "");
      if(j->agent==q_agent && j_idx<N){
        if((j->type>=JT_hingeX && j->type<=JT_hingeZ) || j->type==JT_transXYPhi || j->type==JT_phiTransXY) {
          if(j->type==JT_transXYPhi) j_idx += 2; //refer to the phi only
          J(0, j_idx) += j->axis.x;
          J(1, j_idx) += j->axis.y;
          J(2, j_idx) += j->axis.z;
        }
        if(j->type==JT_quatBall || j->type==JT_free) {
          uint offset = (j->type==JT_free)?3:0;
          arr Jrot = j->X.rot.getArr() * j->Q.rot.getJacobian(); //transform w-vectors into world coordinate
          Jrot /= sqrt(sumOfSqr(q({j->qIndex+offset,j->qIndex+offset+3}))); //account for the potential non-normalization of q
          for(uint i=0;i<4;i++) for(uint k=0;k<3;k++) J(k,j_idx+offset+i) += Jrot(k,i);
        }
        //all other joints: J=0 !!
      }
      if(!j->from->inLinks.N) break;
      j=j->from->inLinks(0);
    }
  }
}

/// The position vec1, attached to b1, relative to the frame of b2 (plus vec2)
void mlr::KinematicWorld::kinematicsRelPos(arr& y, arr& J, Body *b1, const mlr::Vector& vec1, Body *b2, const mlr::Vector& vec2) const {
  arr y1,y2,J1,J2;
  kinematicsPos(y1, J1, b1, vec1);
  kinematicsPos(y2, J2, b2, vec2);
  arr Rinv = ~(b2->X.rot.getArr());
  y = Rinv * (y1 - y2);
  if(&J){
    arr A;
    axesMatrix(A, b2);
    J = Rinv * (J1 - J2 - crossProduct(A, y1 - y2));
  }
}

/// The vector vec1, attached to b1, relative to the frame of b2
void mlr::KinematicWorld::kinematicsRelVec(arr& y, arr& J, Body *b1, const mlr::Vector& vec1, Body *b2) const {
  arr y1,J1;
  kinematicsVec(y1, J1, b1, vec1);
//  kinematicsVec(y2, J2, b2, vec2);
  arr Rinv = ~(b2->X.rot.getArr());
  y = Rinv * y1;
  if(&J){
    arr A;
    axesMatrix(A, b2);
    J = Rinv * (J1 - crossProduct(A, y1));
  }
}

/// The position vec1, attached to b1, relative to the frame of b2 (plus vec2)
void mlr::KinematicWorld::kinematicsRelRot(arr& y, arr& J, Body *b1, Body *b2) const {
  mlr::Quaternion rot_b = b1->X.rot;
  if(&y) y = conv_vec2arr(rot_b.getVec());
  if(&J){
    double phi=acos(rot_b.w);
    double s=2.*phi/sin(phi);
    double ss=-2./(1.-mlr::sqr(rot_b.w)) * (1.-phi/tan(phi));
    arr A;
    axesMatrix(A, b1);
    J = 0.5 * (rot_b.w*A*s + crossProduct(A, y));
    J -= 0.5 * ss/s/s*(y*~y*A);
  }
}

/** @brief return the configuration's inertia tensor $M$ (n x n tensor)*/
void mlr::KinematicWorld::inertia(arr& M) {
  uint j1_idx, j2_idx;
  mlr::Transformation Xa, Xi, Xj;
  Joint *j1, *j2;
  mlr::Vector vi, vj, ti, tj;
  double tmp;
  
  uint N=getJointStateDimension();
  
  //initialize Jacobian
  M.resize(N, N);
  M.setZero();
  
  for(Body *a: bodies) {
    //get reference frame
    Xa = a->X;
    
    j1=a->inLinks(0);
    while(j1) {
      j1_idx=j1->qIndex;
      
      Xi = j1->from->X;
      Xi.appendTransformation(j1->A);
      ti = Xi.rot.getX();
      
      vi = ti ^(Xa.pos-Xi.pos);
      
      j2=j1;
      while(j2) {
        j2_idx=j2->qIndex;
        
        Xj = j2->from->X;
        Xj.appendTransformation(j2->A);
        tj = Xj.rot.getX();
        
        vj = tj ^(Xa.pos-Xj.pos);
        
        tmp = a->mass * (vi*vj);
        //tmp += scalarProduct(a->a.inertia, ti, tj);
        
        M(j1_idx, j2_idx) += tmp;
        
        if(!j2->from->inLinks.N) break;
        j2=j2->from->inLinks(0);
      }
      if(!j1->from->inLinks.N) break;
      j1=j1->from->inLinks(0);
    }
  }
  //symmetric: fill in other half
  for(j1_idx=0; j1_idx<N; j1_idx++) for(j2_idx=0; j2_idx<j1_idx; j2_idx++) M(j2_idx, j1_idx) = M(j1_idx, j2_idx);
}

void mlr::KinematicWorld::equationOfMotion(arr& M, arr& F, bool gravity) {
  mlr::LinkTree tree; //TODO: HACK!! Danny: Why was there a static? This fails if there are more than 2 worlds
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  if(gravity){
    clearForces();
    gravityToForces();
  }
  if(!qdot.N) qdot.resize(q.N).setZero();
  mlr::equationOfMotion(M, F, tree, qdot);
}

/** @brief return the joint accelerations \f$\ddot q\f$ given the
  joint torques \f$\tau\f$ (computed via Featherstone's Articulated Body Algorithm in O(n)) */
void mlr::KinematicWorld::fwdDynamics(arr& qdd, const arr& qd, const arr& tau) {
  static mlr::LinkTree tree;
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  //cout <<tree <<endl;
  //mlr::fwdDynamics_aba_1D(qdd, tree, qd, tau);
  //mlr::fwdDynamics_aba_nD(qdd, tree, qd, tau);
  mlr::fwdDynamics_MF(qdd, tree, qd, tau);
}

/** @brief return the necessary joint torques \f$\tau\f$ to achieve joint accelerations
  \f$\ddot q\f$ (computed via the Recursive Newton-Euler Algorithm in O(n)) */
void mlr::KinematicWorld::inverseDynamics(arr& tau, const arr& qd, const arr& qdd) {
  static mlr::LinkTree tree;
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  mlr::invDynamics(tau, tree, qd, qdd);
}

/*void mlr::KinematicWorld::impulsePropagation(arr& qd1, const arr& qd0){
  static mlr::Array<Featherstone::Link> tree;
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  mimickImpulsePropagation(tree);
  Featherstone::RF_abd(qdd, tree, qd, tau);
}*/

/// [prelim] some heuristic measure for the joint errors
double mlr::KinematicWorld::getJointErrors() const {
  double err=0.0;
  for(Joint * e: joints) err+=e->Q.pos.lengthSqr();
  return ::sqrt(err);
}

/** @brief checks if all names of the bodies are disjoint */
bool mlr::KinematicWorld::checkUniqueNames() const {
  for(Body *  n:  bodies) for(Body *b: bodies) {
    if(n==b) break;
    if(n->name==b->name) return false;
  }
  return true;
}

/** @brief checks if all names of the bodies are disjoint */
void mlr::KinematicWorld::setShapeNames() {
  for(Body *b: bodies){
    uint i=0;
    for(Shape *s:b->shapes){
      if(!s->name.N){ s->name = b->name; s->name <<'_' <<i; }
      i++;
    }
  }
}

/// find body with specific name
mlr::Body* mlr::KinematicWorld::getBodyByName(const char* name, bool warnIfNotExist) const {
  for(Body *b: bodies) if(b->name==name) return b;
  if(strcmp("glCamera", name)!=0)
  if(warnIfNotExist) MLR_MSG("cannot find Body named '" <<name <<"' in Graph");
  return 0;
}

/// find shape with specific name
mlr::Shape* mlr::KinematicWorld::getShapeByName(const char* name, bool warnIfNotExist) const {
  for(Shape *s: shapes) if(s->name==name) return s;
  if(warnIfNotExist) MLR_MSG("cannot find Shape named '" <<name <<"' in Graph");
  return NULL;
}

/// find shape with specific name
mlr::Joint* mlr::KinematicWorld::getJointByName(const char* name, bool warnIfNotExist) const {
  for(Joint *j: joints) if(j->name==name) return j;
  if(warnIfNotExist) MLR_MSG("cannot find Joint named '" <<name <<"' in Graph");
  return NULL;
}

/// find joint connecting two bodies
mlr::Joint* mlr::KinematicWorld::getJointByBodies(const Body* from, const Body* to) const {
  for(Joint *j: to->inLinks) if(j->from==from) return j;
  return NULL;
}

/// find joint connecting two bodies with specific names
mlr::Joint* mlr::KinematicWorld::getJointByBodyNames(const char* from, const char* to) const {
  Body *f = getBodyByName(from);
  Body *t = getBodyByName(to);
  if(!f || !t) return NULL;
  return graphGetEdge<Body, Joint>(f, t);
}

/// find joint connecting two bodies with specific names
mlr::Joint* mlr::KinematicWorld::getJointByBodyIndices(uint ifrom, uint ito) const {
  if(ifrom>=bodies.N || ito>=bodies.N) return NULL;
  Body *f = bodies(ifrom);
  Body *t = bodies(ito);
  return getJointByBodies(f, t);
}

ShapeL mlr::KinematicWorld::getShapesByAgent(const uint agent) const {
  ShapeL agent_shapes;
  for(mlr::Joint *j : joints) {
    if(j->agent==agent) {
      ShapeL tmp;
      tmp.append(j->from->shapes);
      tmp.append(j->to->shapes);
      for(mlr::Shape* s : tmp) {
        if (!agent_shapes.contains(s)) agent_shapes.append(s);
      }
    }
  }
  return agent_shapes;
}

uintA mlr::KinematicWorld::getShapeIdxByAgent(const uint agent) const {
  uintA agent_shape_idx;
  ShapeL agent_shapes = getShapesByAgent(agent);
  for(mlr::Shape* s : agent_shapes)
    agent_shape_idx.append(s->index);
  return agent_shape_idx;
}

/** @brief creates uniques names by prefixing the node-index-number to each name */
void mlr::KinematicWorld::prefixNames() {
  for(Body * n: bodies) n->name=STRING(n->index<< n->name);
}

/// return a OpenGL extension
OpenGL& mlr::KinematicWorld::gl(const char* window_title){
  if(!s->gl){
    s->gl = new OpenGL(window_title);
    s->gl->add(glStandardScene, 0);
    s->gl->addDrawer(this);
    s->gl->camera.setDefault();
  }
  return *s->gl;
}

/// return a Swift extension
SwiftInterface& mlr::KinematicWorld::swift(){
  if(!s->swift) s->swift = new SwiftInterface(*this);
  return *s->swift;
}

void mlr::KinematicWorld::swiftDelete() {
  delete s->swift;
  s->swift = nullptr;
}

/// return a PhysX extension
PhysXInterface& mlr::KinematicWorld::physx(){
  if(!s->physx){
    s->physx = new PhysXInterface(*this);
    s->physx->setArticulatedBodiesKinematic();
  }
  return *s->physx;
}

/// return a ODE extension
OdeInterface& mlr::KinematicWorld::ode(){
  if(!s->ode) s->ode = new OdeInterface(*this);
  return *s->ode;
}

void mlr::KinematicWorld::watch(bool pause, const char* txt){
  if(pause) gl().watch(txt);
  else gl().update(txt);
}

void mlr::KinematicWorld::glAnimate(){
  animateConfiguration(*this, NULL);
}

void mlr::KinematicWorld::glGetMasks(int w, int h, bool rgbIndices){
  gl().clear();
  gl().addDrawer(this);
  if(rgbIndices){
    gl().setClearColors(0,0,0,0);
    orsDrawIndexColors = true;
    orsDrawMarkers = orsDrawJoints = orsDrawProxies = false;
  }
  gl().renderInBack(true, true, w, h);
//  indexRgb = gl().captureImage;
//  depth = gl().captureDepth;

  gl().clear();
  gl().add(glStandardScene, 0);
  gl().addDrawer(this);
  if(rgbIndices){
    gl().setClearColors(1,1,1,0);
    orsDrawIndexColors = false;
    orsDrawMarkers = orsDrawJoints = orsDrawProxies = true;
  }
}

void mlr::KinematicWorld::stepSwift(){
  swift().step(*this, false);
}

void mlr::KinematicWorld::stepPhysx(double tau){
  physx().step(tau);
}

void mlr::KinematicWorld::stepOde(double tau){
#ifdef MLR_ODE
  ode().setMotorVel(qdot, 100.);
  ode().step(tau);
  ode().importStateFromOde();
#endif
}

void mlr::KinematicWorld::stepDynamics(const arr& Bu_control, double tau, double dynamicNoise, bool gravity){

  struct DiffEqn:VectorFunction{
    mlr::KinematicWorld &S;
    const arr& Bu;
    bool gravity;
    DiffEqn(mlr::KinematicWorld& _S, const arr& _Bu, bool _gravity):S(_S), Bu(_Bu), gravity(_gravity){
      VectorFunction::operator=( [this](arr& y, arr& J, const arr& x) -> void {
        this->fv(y, J, x);
      } );
    }
    void fv(arr& y, arr& J, const arr& x){
      S.setJointState(x[0], x[1]);
      arr M,Minv,F;
      S.equationOfMotion(M, F, gravity);
      inverse_SymPosDef(Minv, M);
      //Minv = inverse(M); //TODO why does symPosDef fail?
      y = Minv * (Bu - F);
    }
  } eqn(*this, Bu_control, gravity);

#if 0
  arr M,Minv,F;
  getDynamics(M, F);
  inverse_SymPosDef(Minv,M);

  //noisy Euler integration (Runge-Kutte4 would be much more precise...)
  qddot = Minv * (u_control - F);
  if(dynamicNoise) rndGauss(qddot, dynamicNoise, true);
  q    += tau * qdot;
  qdot += tau * qddot;
  arr x1=cat(s->q, s->qdot).reshape(2,s->q.N);
#else
  arr x1;
  rk4_2ndOrder(x1, cat(q, qdot).reshape(2,q.N), eqn, tau);
  if(dynamicNoise) rndGauss(x1[1](), ::sqrt(tau)*dynamicNoise, true);
#endif

  setJointState(x1[0], x1[1]);
}

/** @brief prototype for \c operator<< */
void mlr::KinematicWorld::write(std::ostream& os) const {
  for(Body *b: bodies) {
    os <<"body " <<b->name <<" { ";
    b->write(os);  os <<" }\n";
  }
  os <<std::endl;
  for(Shape *s: shapes) {
    os <<"shape ";
    if(s->name.N) os <<s->name <<' ';
    os <<"(" <<(s->body?(char*)s->body->name:"") <<"){ ";
    s->write(os);  os <<" }\n";
  }
  os <<std::endl;
  for(Joint *j: joints) {
    os <<"joint ";
    if (j->name.N) os <<j->name <<' ';
    os <<"(" <<j->from->name <<' ' <<j->to->name <<"){ ";
    j->write(os);  os <<" }\n";
  }
}

#define DEBUG(x) //x

/** @brief prototype for \c operator>> */
void mlr::KinematicWorld::read(std::istream& is) {
  Graph G(is);
  G.checkConsistency();
//  cout <<"***KVG:\n" <<G <<endl;
  init(G);
}

void mlr::KinematicWorld::init(const Graph& G) {
  clear();

  NodeL bs = G.getNodes("body");
  for(Node *n:  bs) {
    CHECK_EQ(n->keys(0),"body","");
    CHECK(n->isGraph(), "bodies must have value Graph");
    
    Body *b=new Body(*this);
    if(n->keys.N>1) b->name=n->keys.last();
    b->ats.copy(n->graph(), false, true);
    if(n->keys.N>2) b->ats.newNode<bool>({n->keys.last(-1)});
    b->parseAts();
  }

  NodeL ss = G.getNodes("shape");
  for(Node *n: ss) {
    CHECK_EQ(n->keys(0),"shape","");
    CHECK(n->parents.N<=1,"shapes must have no or one parent");
    CHECK(n->isGraph(),"shape must have value Graph");
    
    Shape *s;
    if(n->parents.N==1){
      Body *b = listFindByName(bodies, n->parents(0)->keys.last());
      CHECK(b,"");
      s=new Shape(*this, *b);
    }else{
      s=new Shape(*this, NoBody);
    }
    if(n->keys.N>1) s->name=n->keys.last();
    s->ats.copy(n->graph(), false, true);
    if(n->keys.N>2) s->ats.newNode<bool>({n->keys.last(-1)});
    s->parseAts();
  }
  
  uint nCoupledJoints=0;
  NodeL js = G.getNodes("joint");
  for(Node *n: js) {
    CHECK_EQ(n->keys(0),"joint","joints must be declared as joint: specs=" <<*n <<' ' <<n->index);
    CHECK_EQ(n->parents.N,2,"joints must have two parents: specs=" <<*n <<' ' <<n->index);
    CHECK(n->isGraph(),"joints must have value Graph: specs=" <<*n <<' ' <<n->index);
    
    Body *from=listFindByName(bodies, n->parents(0)->keys.last());
    Body *to=listFindByName(bodies, n->parents(1)->keys.last());
    CHECK(from,"JOINT: from '" <<n->parents(0)->keys.last() <<"' does not exist ["<<*n <<"]");
    CHECK(to,"JOINT: to '" <<n->parents(1)->keys.last() <<"' does not exist ["<<*n <<"]");
    Joint *j=new Joint(*this, from, to);
    if(n->keys.N>1) j->name=n->keys.last();
    j->ats.copy(n->graph(), false, true);
    if(n->keys.N>2) j->ats.newNode<bool>({n->keys.last(-1)});
    j->parseAts();

    //if the joint is coupled to another:
    if(j->mimic) nCoupledJoints++;
  }

  if(nCoupledJoints){
    for(Joint *j: joints) if(j->mimic){
      mlr::String jointName;
      bool good = j->ats.get(jointName, "mimic");
      CHECK(good, "something is wrong");
      if(!jointName.N){ j->mimic=NULL; continue; }
      j->mimic = listFindByName(joints, jointName);
      if(!j->mimic) HALT("The joint '" <<*j <<"' is declared coupled to '" <<jointName <<"' -- but that doesn't exist!");
      j->type = j->mimic->type;
    }
  }

  //-- clean up the graph
  analyzeJointStateDimensions();
//  topSort();
  jointSort();
  checkConsistency();
  //makeLinkTree();
  calc_missingAB_from_BodyAndJointFrames();
  analyzeJointStateDimensions();
  calc_q_from_Q();
  calc_fwdPropagateFrames();
}

void mlr::KinematicWorld::writePlyFile(const char* filename) const {
  ofstream os;
  mlr::open(os, filename);
  uint nT=0,nV=0;
  uint j;
  mlr::Mesh *m;
  for(Shape *s: shapes) { nV += s->mesh.V.d0; nT += s->mesh.T.d0; }
  
  os <<"\
ply\n\
format ascii 1.0\n\
element vertex " <<nV <<"\n\
property float x\n\
property float y\n\
property float z\n\
property uchar red\n\
property uchar green\n\
property uchar blue\n\
element face " <<nT <<"\n\
property list uchar int vertex_index\n\
end_header\n";

  uint k=0;
  mlr::Transformation t;
  mlr::Vector v;
  for(Shape * s: shapes) {
    m = &s->mesh;
    arr col = m->C;
    CHECK(col.N==3,"");
    t = s->X;
    if(m->C.d0!=m->V.d0) {
      m->C.resizeAs(m->V);
      for(j=0; j<m->C.d0; j++) m->C[j]=col;
    }
    for(j=0; j<m->V.d0; j++) {
      v.set(m->V(j, 0), m->V(j, 1), m->V(j, 2));
      v = t*v;
      os <<' ' <<v.x <<' ' <<v.y <<' ' <<v.z
         <<' ' <<int(255.f*m->C(j, 0)) <<' ' <<int(255.f*m->C(j, 1)) <<' ' <<int(255.f*m->C(j, 2)) <<endl;
    }
    k+=j;
  }
  uint offset=0;
  for(Shape *s: shapes) {
    m=&s->mesh;
    for(j=0; j<m->T.d0; j++) {
      os <<"3 " <<offset+m->T(j, 0) <<' ' <<offset+m->T(j, 1) <<' ' <<offset+m->T(j, 2) <<endl;
    }
    offset+=m->V.d0;
  }
}

/// dump the list of current proximities on the screen
void mlr::KinematicWorld::reportProxies(std::ostream& os, double belowMargin, bool brief) const{
  os <<"Proximity report: #" <<proxies.N <<endl;
  for_list(Proxy, p, proxies) {
    if(belowMargin>0. && p->d>belowMargin) continue;
    mlr::Shape *a = shapes(p->a);
    mlr::Shape *b = shapes(p->b);
    os  <<p_COUNT <<" ("
        <<a->name <<':' <<a->body->name <<")-("
        <<b->name <<':' <<b->body->name
        <<") d=" <<p->d;
    if(!brief)
     os <<" |A-B|=" <<(p->posB-p->posA).length()
        <<" cenD=" <<p->cenD
//        <<" d^2=" <<(p->posB-p->posA).lengthSqr()
        <<" v=" <<(p->posB-p->posA)
        <<" normal=" <<p->normal
        <<" posA=" <<p->posA
        <<" posB=" <<p->posB;
    os <<endl;
  }
}

bool ProxySortComp(const mlr::Proxy *a, const mlr::Proxy *b) {
  return (a->a < b->a) || (a->a==b->a && a->b<b->b) || (a->a==b->a && a->b==b->b && a->d < b->d);
}

void mlr::KinematicWorld::glueBodies(Body *f, Body *t) {
  Joint *j = new Joint(*this, f, t);
  j->A.setDifference(f->X, t->X);
  j->type=JT_rigid;
  j->Q.setZero();
  j->B.setZero();
  isLinkTree=false;
}


/// clear all forces currently stored at bodies
void mlr::KinematicWorld::clearForces() {
  for(Body *  n:  bodies) {
    n->force.setZero();
    n->torque.setZero();
  }
}

/// apply a force on body n 
void mlr::KinematicWorld::addForce(mlr::Vector force, mlr::Body *n) {
  n->force += force;
  if (!s->physx) {
    NIY;
  }
  else {
    s->physx->addForce(force, n);
  }
  //n->torque += (pos - n->X.p) ^ force;
}

/// apply a force on body n at position pos (in world coordinates)
void mlr::KinematicWorld::addForce(mlr::Vector force, mlr::Body *n, mlr::Vector pos) {
  n->force += force;
  if (!s->physx) {
    NIY;
  }
  else {
    s->physx->addForce(force, n, pos);
  }
  //n->torque += (pos - n->X.p) ^ force;
}

void mlr::KinematicWorld::frictionToForces(double coeff) {
  HALT("never do this: add it directly in the equations...");
  mlr::Vector a;
  mlr::Transformation X;
  for(Joint *j:joints) {
    X = j->from->X;
    X.appendTransformation(j->A);
    a = X.rot.getX();//rotation axis

NIY;
//    v=j->Q.angvel.length();
//    if(j->Q.angvel*Vector_x<0.) v=-v;
    
//    j->from->torque -= (coeff*v)*a;
//    j->to->torque   += (coeff*v)*a;
  }
}

void mlr::KinematicWorld::gravityToForces() {
  mlr::Vector g(0, 0, -9.81);
  for(Body *  n:  bodies) n->force += n->mass * g;
}

/// compute forces from the current contacts
void mlr::KinematicWorld::contactsToForces(double hook, double damp) {
  mlr::Vector trans, transvel, force;
  uint i;
  int a, b;
  for(i=0; i<proxies.N; i++) if(proxies(i)->d<0.) {
      a=proxies(i)->a; b=proxies(i)->b;
      
      //if(!i || proxies(i-1).a!=a || proxies(i-1).b!=b) continue; //no old reference sticking-frame
      //trans = proxies(i)->rel.p - proxies(i-1).rel.p; //translation relative to sticking-frame
      trans    = proxies(i)->posB-proxies(i)->posA;
      //transvel = proxies(i)->velB-proxies(i)->velA;
      //d=trans.length();
      
      force.setZero();
      force += (hook) * trans; //*(1.+ hook*hook*d*d)
      //force += damp * transvel;
      SL_DEBUG(1, cout <<"applying force: [" <<a <<':' <<b <<"] " <<force <<endl);
      
      if(a!=-1) addForce(force, shapes(a)->body, proxies(i)->posA);
      if(b!=-1) addForce(-force, shapes(b)->body, proxies(i)->posB);
    }
}

void mlr::KinematicWorld::kinematicsProxyDist(arr& y, arr& J, Proxy *p, double margin, bool useCenterDist, bool addValues) const {
  mlr::Shape *a = shapes(p->a);
  mlr::Shape *b = shapes(p->b);

  y.resize(1);
  if(&J) J.resize(1, getJointStateDimension());
  if(!addValues){ y.setZero();  if(&J) J.setZero(); }

//  //costs
//  if(a->type==mlr::ST_sphere && b->type==mlr::ST_sphere){
//    mlr::Vector diff=a->X.pos-b->X.pos;
//    double d = diff.length() - a->size(3) - b->size(3);
//    y(0) = d;
//    if(&J){
//      arr Jpos;
//      arr normal = conv_vec2arr(diff)/diff.length(); normal.reshape(1, 3);
//      kinematicsPos(NoArr, Jpos, a->body);  J += (normal*Jpos);
//      kinematicsPos(NoArr, Jpos, b->body);  J -= (normal*Jpos);
//    }
//    return;
//  }
  y(0) = p->d;
  if(&J){
    arr Jpos;
    mlr::Vector arel, brel;
    if(p->d>0.) { //we have a gradient on pos only when outside
      arel=a->X.rot/(p->posA-a->X.pos);
      brel=b->X.rot/(p->posB-b->X.pos);
      CHECK(p->normal.isNormalized(), "proxy normal is not normalized");
      arr normal; normal.referTo(&p->normal.x, 3); normal.reshape(1, 3);
      kinematicsPos(NoArr, Jpos, a->body, arel);  J += (normal*Jpos);
      kinematicsPos(NoArr, Jpos, b->body, brel);  J -= (normal*Jpos);
    }
  }
}

void mlr::KinematicWorld::kinematicsProxyCost(arr& y, arr& J, Proxy *p, double margin, bool useCenterDist, bool addValues) const {
  mlr::Shape *a = shapes(p->a);
  mlr::Shape *b = shapes(p->b);
  CHECK(a->mesh_radius>0.,"");
  CHECK(b->mesh_radius>0.,"");

  y.resize(1);
  if(&J) J.resize(1, getJointStateDimension());
  if(!addValues){ y.setZero();  if(&J) J.setZero(); }

  //costs
  if(a->type==mlr::ST_sphere && b->type==mlr::ST_sphere){
    mlr::Vector diff=a->X.pos-b->X.pos;
    double d = diff.length() - a->size(3) - b->size(3);
    y(0) = 1. - d/margin;
    if(&J){
      arr Jpos;
      arr normal = conv_vec2arr(diff)/diff.length(); normal.reshape(1, 3);
      kinematicsPos(NoArr, Jpos, a->body);  J -= 1./margin*(normal*Jpos);
      kinematicsPos(NoArr, Jpos, b->body);  J += 1./margin*(normal*Jpos);
    }
    return;
  }
  double ab_radius = margin + 10.*(a->mesh_radius+b->mesh_radius);
  CHECK(p->d<(1.+1e-6)*margin, "something's really wierd here!");
  CHECK(p->cenD<(1.+1e-6)*ab_radius, "something's really wierd here! You disproved the triangle inequality :-)");
  double d1 = 1.-p->d/margin;
  double d2 = 1.-p->cenD/ab_radius;
  if(d2<0.) d2=0.;
  if(!useCenterDist) d2=1.;
  y(0) += d1*d2;
 
  //Jacobian
  if(&J){
    arr Jpos;
    mlr::Vector arel, brel;
    if(p->d>0.) { //we have a gradient on pos only when outside
      arel=a->X.rot/(p->posA-a->X.pos);
      brel=b->X.rot/(p->posB-b->X.pos);
      CHECK(p->normal.isNormalized(), "proxy normal is not normalized");
      arr normal; normal.referTo(&p->normal.x, 3); normal.reshape(1, 3);
          
      kinematicsPos(NoArr, Jpos, a->body, arel);  J -= d2/margin*(normal*Jpos);
      kinematicsPos(NoArr, Jpos, b->body, brel);  J += d2/margin*(normal*Jpos);
    }
        
    if(useCenterDist && d2>0.){
      arel=a->X.rot/(p->cenA-a->X.pos);
      brel=b->X.rot/(p->cenB-b->X.pos);
//      CHECK(p->cenN.isNormalized(), "proxy normal is not normalized");
      if(!p->cenN.isNormalized()){
        MLR_MSG("proxy->cenN is not normalized: objects seem to be at exactly the same place");
      }else{
        arr normal; normal.referTo(&p->cenN.x, 3); normal.reshape(1, 3);
        
        kinematicsPos(NoArr, Jpos, a->body, arel);  J -= d1/ab_radius*(normal*Jpos);
        kinematicsPos(NoArr, Jpos, b->body, brel);  J += d1/ab_radius*(normal*Jpos);
      }
    }
  }
}

/// measure (=scalar kinematics) for the contact cost summed over all bodies
void mlr::KinematicWorld::kinematicsProxyCost(arr &y, arr& J, double margin, bool useCenterDist) const {
  y.resize(1).setZero();
  if(&J) J.resize(1, getJointStateDimension()).setZero();
  for(Proxy *p:proxies) if(p->d<margin) {
    kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
  }
}

void mlr::KinematicWorld::kinematicsProxyConstraint(arr& g, arr& J, Proxy *p, double margin) const {
  if(&J) J.resize(1, getJointStateDimension()).setZero();

  g.resize(1) = margin - p->d;

  //Jacobian
  if(&J){
    arr Jpos, normal;
    mlr::Vector arel,brel;
    mlr::Shape *a = shapes(p->a);
    mlr::Shape *b = shapes(p->b);
    if(p->d>0.) { //we have a gradient on pos only when outside
      arel=a->X.rot/(p->posA-a->X.pos);
      brel=b->X.rot/(p->posB-b->X.pos);
      CHECK(p->normal.isNormalized(), "proxy normal is not normalized");
      normal.referTo(&p->normal.x, 3);
    } else { //otherwise take gradient w.r.t. centers...
      arel=a->X.rot/(p->cenA-a->X.pos);
      brel=b->X.rot/(p->cenB-b->X.pos);
      CHECK(p->cenN.isNormalized(), "proxy normal is not normalized");
      normal.referTo(&p->cenN.x, 3);
    }
    normal.reshape(1, 3);

    kinematicsPos(NoArr, Jpos, a->body, arel);  J -= (normal*Jpos);
    kinematicsPos(NoArr, Jpos, b->body, brel);  J += (normal*Jpos);
  }
}

void mlr::KinematicWorld::kinematicsContactConstraints(arr& y, arr &J) const {
  J.clear();
  mlr::Vector normal;
  uint i, con=0;
  Shape *a, *b;
  arr Jpos, dnormal, grad(1, q.N);

  y.clear();
  for(i=0; i<proxies.N; i++) y.append(proxies(i)->d);

  if(!&J) return; //do not return the Jacobian

  mlr::Vector arel, brel;
  for(i=0; i<proxies.N; i++) {
    a=shapes(proxies(i)->a); b=shapes(proxies(i)->b);
    
    arel.setZero();  arel=a->X.rot/(proxies(i)->posA-a->X.pos);
    brel.setZero();  brel=b->X.rot/(proxies(i)->posB-b->X.pos);
    
    CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
    dnormal.referTo(proxies(i)->normal.p(), 3); dnormal.reshape(1, 3);
    grad.setZero();
    kinematicsPos(NoArr, Jpos, a->body, arel); grad += dnormal*Jpos; //moving a long normal b->a increases distance
    kinematicsPos(NoArr, Jpos, b->body, brel); grad -= dnormal*Jpos; //moving b long normal b->a decreases distance
    J.append(grad);
    con++;
  }
  J.reshape(con, q.N);
}

void mlr::KinematicWorld::kinematicsLimitsCost(arr &y, arr &J, const arr& limits, double margin) const {
  y.resize(1).setZero();
  if(&J) J.resize(1, getJointStateDimension()).setZero();
  double d;
  for(uint i=0; i<limits.d0; i++) if(limits(i,1)>limits(i,0)){ //only consider proper limits (non-zero interval)
    double m = margin*(limits(i,1)-limits(i,0));
    d = limits(i, 0) + m - q(i); //lo
    if(d>0.) {  y(0) += d/m;  if(&J) J(0, i)-=1./m;  }
    d = q(i) - limits(i, 1) + m; //up
    if(d>0.) {  y(0) += d/m;  if(&J) J(0, i)+=1./m;  }
  }
}

/// Compute the new configuration q such that body is located at ytarget (with deplacement rel).
void mlr::KinematicWorld::inverseKinematicsPos(Body& body, const arr& ytarget,
                                               const mlr::Vector& rel_offset, int max_iter) {
  arr q0, q;
  getJointState(q0);
  q = q0;
  arr y; // endeff pos
  arr J; // Jacobian
  arr invJ;
  arr I = eye(q.N);

  // general inverse kinematic update
  // first iteration: $q* = q' + J^# (y* - y')$
  // next iterations: $q* = q' + J^# (y* - y') + (I - J# J)(q0 - q')$
  for (int i = 0; i < max_iter; i++) {
    kinematicsPos(y, J, &body, rel_offset);
    invJ = ~J * inverse(J * ~J);  // inverse_SymPosDef should work!?
    q = q + invJ * (ytarget - y);

    if (i > 0) {
      q += (I - invJ * J) * (q0 - q);
    }
    setJointState(q);
  }
}

/// center of mass of the whole configuration (3 vector)
double mlr::KinematicWorld::getCenterOfMass(arr& x_) const {
  double M=0.;
  mlr::Vector x;
  x.setZero();
  for(Body *  n:  bodies) {
    M+=n->mass;
    x+=n->mass*n->X.pos;
  }
  x/=M;
  x_ = conv_vec2arr(x);
  return M;
}

/// gradient (Jacobian) of the COM w.r.t. q (3 x n tensor)
void mlr::KinematicWorld::getComGradient(arr &grad) const {
  double M=0.;
  arr J(3, getJointStateDimension());
  grad.resizeAs(J); grad.setZero();
  for(Body * n: bodies) {
    M += n->mass;
    kinematicsPos(NoArr, J, n);
    grad += n->mass * J;
  }
  grad/=M;
}

mlr::Proxy* mlr::KinematicWorld::getContact(uint a, uint b) const {
  uint i;
  for(i=0; i<proxies.N; i++) if(proxies(i)->d<0.) {
      if(proxies(i)->a==(int)a && proxies(i)->b==(int)b) return proxies(i);
      if(proxies(i)->a==(int)b && proxies(i)->b==(int)a) return proxies(i);
    }
  return NULL;
}

arr mlr::KinematicWorld::getHmetric() const{
  arr H = zeros(getJointStateDimension());
  for(mlr::Joint *j:joints) if(j->agent==q_agent){
    double h=j->H;
    CHECK(h>0.,"Hmetric should be larger than 0");
    if(j->type==JT_transXYPhi){
      H(j->qIndex+0)=h*10.;
      H(j->qIndex+1)=h*10.;
      H(j->qIndex+2)=h;
    }else{
      for(uint k=0;k<j->qDim();k++) H(j->qIndex+k)=h;
    }
  }
  return H;
}

/** @brief */
double mlr::KinematicWorld::getEnergy() {
  double m, v, E;
  mlr::Matrix I;
  mlr::Vector w;

  calc_fwdPropagateVelocities();
  
  E=0.;
  for(Body *b: bodies) {
    m=b->mass;
    mlr::Quaternion &rot = b->X.rot;
    I=(rot).getMatrix() * b->inertia * (-rot).getMatrix();
    v=b->vel.length();
    w=b->angvel;
    E += .5*m*v*v;
    E += 9.81 * m * (b->X*b->com).z;
    E += .5*(w*(I*w));
  }
  
  return E;
}

void mlr::KinematicWorld::removeUselessBodies(int verbose) {
  //-- remove bodies and their in-joints
  for_list_rev(Body, b, bodies) if(!b->shapes.N && !b->outLinks.N) {
    if(verbose>0) LOG(0) <<" -- removing useless body " <<b->name <<endl;
    delete b;
  }
  //-- reindex
  listReindex(bodies);
  listReindex(joints);
  checkConsistency();
//  for(Joint * j: joints) j->index=j_COUNT;  j->ifrom = j->from->index;  j->ito = j->to->index;  }
//  for(Shape *s: shapes) s->ibody = s->body->index;
  //-- clear all previous index related things
  qdim.clear();
  proxies.clear();
  analyzeJointStateDimensions();
  calc_q_from_Q();
}

bool mlr::KinematicWorld::checkConsistency(){
  if(qdim.N){
    uint N=getJointStateDimension();
    CHECK_EQ(N, qdim(q_agent), "");
    if(q.N) CHECK_EQ(N, q.N, "");
    if(qdot.N) CHECK_EQ(N, qdot.N, "");

    uintA myqdim(qdim.N);
    myqdim.setZero();
    for(Joint *j: joints){
      CHECK(j->agent<qdim.N, "");

      if(j->mimic){
        CHECK_EQ(j->qIndex, j->mimic->qIndex, "");
      }else{
        CHECK_EQ(j->qIndex, myqdim(j->agent), "joint indexing is inconsistent");
        myqdim(j->agent) += j->qDim();
      }
    }
    CHECK_EQ(myqdim, qdim, "qdim is wrong");
  }

  for(Body *b: bodies){
    CHECK(&b->world, "");
    CHECK(&b->world==this,"");
    CHECK_EQ(b, bodies(b->index), "");
    for(Joint *j: b->outLinks) CHECK_EQ(j->from,b,"");
    for(Joint *j: b->inLinks)  CHECK_EQ(j->to,b,"");
    for(Shape *s: b->shapes)   CHECK_EQ(s->body,b,"");
    b->ats.checkConsistency();
  }

  for(Joint *j: joints){
    CHECK(&j->world && j->from && j->to, "");
    CHECK(&j->world==this,"");
    CHECK_EQ(j, joints(j->index), "");
    CHECK(j->from->outLinks.findValue(j)>=0,"");
    CHECK(j->to->inLinks.findValue(j)>=0,"");
    CHECK_GE(j->type.x, 0, "");
    CHECK_LE(j->type.x, JT_free, "");
    j->ats.checkConsistency();
  }

  //check topsort
  intA level = consts<int>(0, bodies.N);
  //compute levels
  for(Joint *j: joints) level(j->to->index) = level(j->from->index)+1;

  for(Joint *j: joints){
      CHECK(level(j->from->index) < level(j->to->index), "joint does not go forward");
      if(j->index) CHECK_LE(level(joints(j->index-1)->to->index), level(j->to->index), "joints are not sorted");
  }
  for(Body *b: bodies){
    for(Joint *j: b->inLinks)  CHECK(level(j->from->index) < level(b->index), "topsort failed");
  }

  for(Shape *s: shapes){
    CHECK(&s->world, "");
    CHECK(&s->world==this,"");
    CHECK_EQ(s,shapes(s->index),"");
    if(s->body) CHECK(s->body->shapes.findValue(s)>=0,"");
    s->ats.checkConsistency();
  }
  return true;
}

void mlr::KinematicWorld::meldFixedJoints(int verbose) {
  checkConsistency();
  for(Joint *j: joints) if(j->type==JT_rigid) {
    if(verbose>0) LOG(0) <<" -- melding fixed joint " <<j->name <<" (" <<j->from->name <<' ' <<j->to->name <<" )" <<endl;
    Body *a = j->from;
    Body *b = j->to;
    Transformation bridge = j->A * j->Q * j->B;
    //reassociate shapes with a
    for(Shape *s: b->shapes) {
      s->body=a;
      s->rel = bridge * s->rel;
      a->shapes.append(s);
    }
    b->shapes.clear();
    //joints from b-to-c now become joints a-to-c
    for(Joint *jj: b->outLinks) {
      jj->from=a;
      jj->A = bridge * jj->A;
      a->outLinks.append(jj);
    }
    b->outLinks.clear();
    //reassociate mass
    a->mass += b->mass;
    a->inertia += b->inertia;
    b->mass = 0.;
  }
  jointSort();
  calc_q_from_Q();
  checkConsistency();
  //-- remove fixed joints and reindex
  for_list_rev(Joint, jj, joints) if(jj->type==JT_rigid) delete jj;
  listReindex(joints);
  //for(Joint * j: joints) { j->index=j_COUNT;  j->ifrom = j->from->index;  j->ito = j->to->index;  }
  checkConsistency();
}

/// GL routine to draw a mlr::KinematicWorld
#ifdef MLR_GL
void mlr::KinematicWorld::glDraw(OpenGL& gl) {
  uint i=0;
  mlr::Transformation f;
  double GLmatrix[16];

  glPushMatrix();

  glColor(.5, .5, .5);

  //bodies
  if(orsDrawBodies) for(Shape *s: shapes) {
    s->glDraw(gl);
    i++;
    if(orsDrawLimit && i>=orsDrawLimit) break;
  }

  //joints
  if(orsDrawJoints) for(Joint *e: joints) {
    //set name (for OpenGL selection)
    glPushName((e->index <<2) | 2);

    double s=e->A.pos.length()+e->B.pos.length(); //some scale
    s*=.25;

    //from body to joint
    f=e->from->X;
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glColor(1, 1, 0);
    //glDrawSphere(.1*s);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(e->A.pos.x, e->A.pos.y, e->A.pos.z);
    glEnd();

    //joint frame A
    f.appendTransformation(e->A);
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glDrawAxes(s);
    glColor(1, 0, 0);
    glRotatef(90, 0, 1, 0);  glDrawCylinder(.05*s, .3*s);  glRotatef(-90, 0, 1, 0);

    //joint frame B
    f.appendTransformation(e->Q);
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glDrawAxes(s);

    //from joint to body
    glColor(1, 0, 1);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(e->B.pos.x, e->B.pos.y, e->B.pos.z);
    glEnd();
    glTranslatef(e->B.pos.x, e->B.pos.y, e->B.pos.z);
    //glDrawSphere(.1*s);

    glPopName();
    i++;
    if(orsDrawLimit && i>=orsDrawLimit) break;
  }

  //proxies
  if(orsDrawProxies) for(Proxy *proxy: proxies) proxy->glDraw(gl);

  glPopMatrix();
}
#endif

//===========================================================================
//
// Kinematic Switch
//

mlr::KinematicSwitch::KinematicSwitch()
  : symbol(none), jointType(JT_none), timeOfApplication(UINT_MAX), fromId(UINT_MAX), toId(UINT_MAX), agent(0){
  jA.setZero();
  jB.setZero();
}

mlr::KinematicSwitch::KinematicSwitch(OperatorSymbol op, JointType type, const char* ref1, const char* ref2, const mlr::KinematicWorld& K, uint _timeOfApplication, const mlr::Transformation& jFrom, const mlr::Transformation& jTo, uint agent)
  : symbol(op), jointType(type), timeOfApplication(_timeOfApplication), fromId(UINT_MAX), toId(UINT_MAX), agent(agent){
  if(ref1) fromId = K.getShapeByName(ref1)->index;
  if(ref2) toId = K.getShapeByName(ref2)->index;
  if(&jFrom) jA = jFrom;
  if(&jTo)   jB = jTo;
}

#define STEP(t) (floor(t*double(stepsPerPhase) + .500001))-1

void mlr::KinematicSwitch::setTimeOfApplication(double time, bool before, int stepsPerPhase, uint T){
  if(stepsPerPhase<0) stepsPerPhase=T;
  timeOfApplication = STEP(time)+(before?0:1);
}

void mlr::KinematicSwitch::apply(KinematicWorld& G){
  Shape *from=NULL, *to=NULL;
  if(fromId!=UINT_MAX) from=G.shapes(fromId);
  if(toId!=UINT_MAX) to=G.shapes(toId);
  if(fromId==UINT_MAX){
    CHECK_EQ(symbol, deleteJoint, "");
    CHECK(to,"");
    mlr::Body *b = to->body;
    CHECK_LE(b->inLinks.N, 1,"");
    if(b->inLinks.N){
      from = b->inLinks(0)->from->shapes.first();
    }else{
      return;
    }
  }

  if(symbol==deleteJoint){
    Joint *j = G.getJointByBodies(from->body, to->body);
    CHECK(j,"can't find joint between '"<<from->name <<"--" <<to->name <<"' Deleted before?");
    delete j;
    G.jointSort();
    return;
  }
  G.isLinkTree=false;
  if(symbol==addJointZero || symbol==addActuated){
    Joint *j = new Joint(G, from->body, to->body);
    if(agent) j->agent=agent;
    if(symbol==addJointZero) j->constrainToZeroVel=true;
    else                     j->constrainToZeroVel=false;
    j->type=jointType;
    j->A = from->rel * jA;
    j->B = jB * (-to->rel);
    G.jointSort();
    G.calc_fwdPropagateFrames();
    return;
  }
  if(symbol==addJointAtFrom){
    Joint *j = new Joint(G, from->body, to->body);
    if(agent) j->agent=agent;
    j->constrainToZeroVel=true;
    j->type=jointType;
    j->B.setDifference(from->body->X, to->body->X);
    j->A.setZero();
    G.jointSort();
    G.calc_fwdPropagateFrames();
    return;
  }
  if(symbol==addJointAtTo){
    Joint *j = new Joint(G, from->body, to->body);
    if(agent) j->agent=agent;
    j->constrainToZeroVel=true;
    j->type=jointType;
    j->A.setDifference(from->body->X, to->body->X);
    j->B.setZero();
    G.jointSort();
    G.calc_fwdPropagateFrames();
    return;
  }
  if(symbol==addSliderMechanism){
    HALT("I think it is better if there is fixed slider mechanisms in the world, that may jump; no dynamic creation of bodies");
    Body *slider1 = new Body(G); //{ type=ST_box size=[.2 .1 .05 0] color=[0 0 0] }
    Body *slider2 = new Body(G); //{ type=ST_box size=[.2 .1 .05 0] color=[1 0 0] }
    Shape *s1 = new Shape(G, *slider1); s1->type=ST_box; s1->size={.2,.1,.05}; s1->mesh.C={0.,0,0};
    Shape *s2 = new Shape(G, *slider2); s2->type=ST_box; s2->size={.2,.1,.05}; s2->mesh.C={1.,0,0};

    //placement of the slider1 on the table -> fixed
    Joint *j1 = new Joint(G, from->body, slider1);
    j1->type = JT_transXYPhi;
    j1->constrainToZeroVel=true;
    j1->A = from->rel * jA;
    //the actual sliding translation -> articulated
    Joint *j2 = new Joint(G, slider1, slider2);
    j2->type = JT_transX;
    j2->constrainToZeroVel=false;
    //orientation of the object on the slider2 -> fixed
    Joint *j3 = new Joint(G, slider2, to->body);
    j3->type = JT_hingeZ;
    j3->constrainToZeroVel=true;
    j3->B = jB * (-to->rel);

    G.jointSort();
    G.calc_fwdPropagateFrames();
    return;
  }
  HALT("shouldn't be here!");
}

void mlr::KinematicSwitch::temporallyAlign(const mlr::KinematicWorld& Gprevious, mlr::KinematicWorld& G, bool copyFromBodies){
  if(symbol==addJointAtFrom){
    Joint *j = G.getJointByBodies(G.shapes(fromId)->body, G.shapes(toId)->body);
    if(!j/* || j->type!=jointType*/) HALT("");
    if(copyFromBodies){
      j->B.setDifference(Gprevious.shapes(fromId)->body->X, Gprevious.shapes(toId)->body->X);
    }else{//copy from previous, if exists
      Joint *jprev = Gprevious.getJointByBodies(Gprevious.shapes(fromId)->body, Gprevious.shapes(toId)->body);
      if(!jprev || jprev->type!=j->type){//still copy from bodies
        j->B.setDifference(Gprevious.shapes(fromId)->body->X, Gprevious.shapes(toId)->body->X);
      }else{
        j->B = jprev->B;
      }
    }
//    j->A.setZero();
    G.calc_fwdPropagateFrames();
    return;
  }
  if(symbol==addJointAtTo){
    Joint *j = G.getJointByBodies(G.shapes(fromId)->body, G.shapes(toId)->body);
    if(!j || j->type!=jointType) return; //HALT(""); //return;
    if(copyFromBodies){
      j->A.setDifference(Gprevious.shapes(fromId)->body->X, Gprevious.shapes(toId)->body->X);
    }else{
      Joint *jprev = Gprevious.getJointByBodies(Gprevious.shapes(fromId)->body, Gprevious.shapes(toId)->body);
      if(!jprev || jprev->type!=j->type){
        j->A.setDifference(Gprevious.shapes(fromId)->body->X, Gprevious.shapes(toId)->body->X);
      }else{
        j->A = jprev->A;
      }
    }
//    j->B.setZero();
    G.calc_fwdPropagateFrames();
    return;
  }
}

mlr::String mlr::KinematicSwitch::shortTag(const mlr::KinematicWorld* G) const{
  mlr::String str;
  str <<"  timeOfApplication=" <<timeOfApplication;
  str <<"  symbol=" <<symbol;
  str <<"  jointType=" <<jointType;
  str <<"  fromId=" <<(fromId==UINT_MAX?"NULL":(G?G->shapes(fromId)->name:STRING(fromId)));
  str <<"  toId=" <<(G?G->shapes(toId)->name:STRING(toId)) <<endl;
  return str;
}

void mlr::KinematicSwitch::write(std::ostream& os) const{
  os <<"  timeOfApplication=" <<timeOfApplication;
  os <<"  symbol=" <<symbol;
  os <<"  jointType=" <<jointType;
  os <<"  fromId=" <<fromId;
  os <<"  toId=" <<toId <<endl;
}

//===========================================================================

mlr::KinematicSwitch* mlr::KinematicSwitch::newSwitch(const Node *specs, const mlr::KinematicWorld& world, int stepsPerPhase, uint T){
  if(specs->parents.N<2) return NULL;

  //-- get tags
  mlr::String& tt=specs->parents(0)->keys.last();
  mlr::String& type=specs->parents(1)->keys.last();
  const char *ref1=NULL, *ref2=NULL;
  if(specs->parents.N>2) ref1=specs->parents(2)->keys.last().p;
  if(specs->parents.N>3) ref2=specs->parents(3)->keys.last().p;

  if(tt!="MakeJoint") return NULL;
  mlr::KinematicSwitch* sw = newSwitch(type, ref1, ref2, world, stepsPerPhase + 1);

  if(specs->isGraph()){
    const Graph& params = specs->graph();
    sw->setTimeOfApplication(params.get<double>("time",1.), params.get<bool>("time", false), stepsPerPhase, T);
//    sw->timeOfApplication = *stepsPerPhase + 1;
    params.get(sw->jA, "from");
    params.get(sw->jB, "to");
  }
  return sw;
}

mlr::KinematicSwitch* mlr::KinematicSwitch::newSwitch(const mlr::String& type, const char* ref1, const char* ref2, const mlr::KinematicWorld& world, uint _timeOfApplication, const mlr::Transformation& jFrom, const mlr::Transformation& jTo){
  //-- create switch
  mlr::KinematicSwitch *sw= new mlr::KinematicSwitch();
  if(type=="addRigid"){ sw->symbol=mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_rigid; }
//  else if(type=="addRigidRel"){ sw->symbol = mlr::KinematicSwitch::addJointAtTo; sw->jointType=mlr::JT_rigid; }
  else if(type=="rigidAtTo"){ sw->symbol = mlr::KinematicSwitch::addJointAtTo; sw->jointType=mlr::JT_rigid; }
  else if(type=="rigidAtFrom"){ sw->symbol = mlr::KinematicSwitch::addJointAtFrom; sw->jointType=mlr::JT_rigid; }
  else if(type=="rigidZero"){ sw->symbol = mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_rigid; }
  else if(type=="transXActuated"){ sw->symbol = mlr::KinematicSwitch::addActuated; sw->jointType=mlr::JT_transX; }
  else if(type=="transXYPhiAtFrom"){ sw->symbol = mlr::KinematicSwitch::addJointAtFrom; sw->jointType=mlr::JT_transXYPhi; }
  else if(type=="transXYPhiZero"){ sw->symbol = mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_transXYPhi; }
  else if(type=="transXYPhiActuated"){ sw->symbol = mlr::KinematicSwitch::addActuated; sw->jointType=mlr::JT_transXYPhi; }
  else if(type=="freeAtTo"){ sw->symbol = mlr::KinematicSwitch::addJointAtTo; sw->jointType=mlr::JT_free; }
  else if(type=="freeZero"){ sw->symbol = mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_free; }
  else if(type=="freeActuated"){ sw->symbol = mlr::KinematicSwitch::addActuated; sw->jointType=mlr::JT_free; }
  else if(type=="ballZero"){ sw->symbol = mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_quatBall; }
  else if(type=="hingeZZero"){ sw->symbol = mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_hingeZ; }
  else if(type=="sliderMechanism"){ sw->symbol = mlr::KinematicSwitch::addSliderMechanism; }
  else if(type=="delete"){ sw->symbol = mlr::KinematicSwitch::deleteJoint; }
  else HALT("unknown type: "<< type);
  if(ref1) sw->fromId = world.getShapeByName(ref1)->index;
  if(ref2) sw->toId = world.getShapeByName(ref2)->index;
//  if(!ref2){
//    CHECK_EQ(sw->symbol, mlr::KinematicSwitch::deleteJoint, "");
//    mlr::Body *b = fromShape->body;
//    if(b->inLinks.N==1){
////      CHECK_EQ(b->outLinks.N, 0, "");
//      sw->toId = sw->fromId;
//      sw->fromId = b->inLinks(0)->from->shapes.first()->index;
//    }else if(b->outLinks.N==1){
//      CHECK_EQ(b->inLinks.N, 0, "");
//      sw->toId = b->outLinks(0)->from->shapes.first()->index;
//    }else if(b->inLinks.N==0 && b->outLinks.N==0){
//      MLR_MSG("No link to delete for shape '" <<ref1 <<"'");
//      delete sw;
//      return NULL;
//    }else HALT("that's ambiguous");
//  }else{

  sw->timeOfApplication = _timeOfApplication;
  if(&jFrom) sw->jA = jFrom;
  if(&jTo) sw->jB = jTo;
  return sw;
}

const char* mlr::KinematicSwitch::name(mlr::KinematicSwitch::OperatorSymbol s){
  HALT("deprecated");
  static const char* names[] = { "deleteJoint", "addJointZero", "addJointAtFrom", "addJointAtTo", "addArticulated" };
  if(s==none) return "none";
  return names[(int)s];
}


//===========================================================================
//
// helper routines -- in a classical C interface
//

#endif

#undef LEN

double forceClosureFromProxies(mlr::KinematicWorld& ORS, uint bodyIndex, double distanceThreshold, double mu, double torqueWeights) {
  mlr::Vector c, cn;
  arr C, Cn;
  for(mlr::Proxy * p: ORS.proxies){
    int body_a = ORS.shapes(p->a)->body?ORS.shapes(p->a)->body->index:-1;
    int body_b = ORS.shapes(p->b)->body?ORS.shapes(p->b)->body->index:-1;
    if(p->d<distanceThreshold && (body_a==(int)bodyIndex || body_b==(int)bodyIndex)) {
      if(body_a==(int)bodyIndex) {
        c = p->posA;
        cn=-p->normal;
      } else {
        c = p->posB;
        cn= p->normal;
      }
      C.append(conv_vec2arr(c));
      Cn.append(conv_vec2arr(cn));
    }
  }
  C .reshape(C.N/3, 3);
  Cn.reshape(C.N/3, 3);
  double fc=forceClosure(C, Cn, ORS.bodies(bodyIndex)->X.pos, mu, torqueWeights, NULL);
  return fc;
}

void transferQbetweenTwoWorlds(arr& qto, const arr& qfrom, const mlr::KinematicWorld& to, const mlr::KinematicWorld& from){
  arr q = to.getJointState();
  uint T = qfrom.d0;
  uint Nfrom = qfrom.d1;

  if (qfrom.d1==0) {T = 1; Nfrom = qfrom.d0;}

  qto = repmat(~q,T,1);

  intA match(Nfrom);
  match = -1;
  for(mlr::Joint* jfrom:from.joints){
    mlr::Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++){
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0;i<match.N;i++) if(match(i)!=-1){
    for(uint t=0;t<T;t++){
      if (qfrom.d1==0) {
        qto(t, match(i)) = qfrom(i);
      } else {
        qto(t, match(i)) = qfrom(t,i);
      }
    }
  }

  if (qfrom.d1==0) qto.reshape(qto.N);
}

#if 0 //nonsensical
void transferQDotbetweenTwoWorlds(arr& qDotTo, const arr& qDotFrom, const mlr::KinematicWorld& to, const mlr::KinematicWorld& from){
  //TODO: for saveness reasons, the velocities are zeroed.
  arr qDot;
  qDot = zeros(to.getJointStateDimension());
  uint T, dim;
  if(qDotFrom.d1 > 0) {
    T = qDotFrom.d0;
    qDotTo = repmat(~qDot,T,1);
    dim = qDotFrom.d1;
  } else {
    T = 1;
    qDotTo = qDot;
    dim = qDotFrom.d0;
  }

  intA match(dim);
  match = -1;
  for(mlr::Joint* jfrom:from.joints){
    mlr::Joint* jto = to.getJointByName(jfrom->name, false); //OLD: to.getJointByBodyNames(jfrom->from->name, jfrom->to->name); why???
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++){
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }
  if(qDotFrom.d1 > 0) {
    for(uint i=0;i<match.N;i++) if(match(i)!=-1){
      for(uint t=0;t<T;t++){
        qDotTo(t, match(i)) = qDotFrom(t,i);
      }
    }
  } else {
    for(uint i=0;i<match.N;i++) if(match(i)!=-1){
      qDotTo(match(i)) = qDotFrom(i);
    }
  }

}

void transferKpBetweenTwoWorlds(arr& KpTo, const arr& KpFrom, const mlr::KinematicWorld& to, const mlr::KinematicWorld& from){
  KpTo = zeros(to.getJointStateDimension(),to.getJointStateDimension());
  //use Kp gains from ors file for toWorld, if there are no entries of this joint in fromWorld
  for_list(mlr::Joint, j, to.joints) {
    if(j->qDim()>0) {
      arr *info;
      info = j->ats.find<arr>("gains");
      if(info) {
        KpTo(j->qIndex,j->qIndex)=info->elem(0);
      }
    }
  }

  intA match(KpFrom.d0);
  match = -1;
  for(mlr::Joint* jfrom : from.joints){
    mlr::Joint* jto = to.getJointByName(jfrom->name, false); // OLD: mlr::Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++){
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0;i<match.N;i++) {
    for(uint j=0;j<match.N;j++){
      KpTo(match(i), match(j)) = KpFrom(i,j);
    }
  }
}

void transferKdBetweenTwoWorlds(arr& KdTo, const arr& KdFrom, const mlr::KinematicWorld& to, const mlr::KinematicWorld& from) {
  KdTo = zeros(to.getJointStateDimension(),to.getJointStateDimension());

  //use Kd gains from ors file for toWorld, if there are no entries of this joint in fromWorld
  for_list(mlr::Joint, j, to.joints) {
    if(j->qDim()>0) {
      arr *info;
      info = j->ats.find<arr>("gains");
      if(info) {
        KdTo(j->qIndex,j->qIndex)=info->elem(1);
      }
    }
  }

  intA match(KdFrom.d0);
  match = -1;
  for(mlr::Joint* jfrom : from.joints){
    mlr::Joint* jto = to.getJointByName(jfrom->name, false); // OLD: mlr::Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++){
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0;i<match.N;i++) {
    for(uint j=0;j<match.N;j++){
      KdTo(match(i), match(j)) = KdFrom(i,j);
    }
  }
}


void transferU0BetweenTwoWorlds(arr& u0To, const arr& u0From, const mlr::KinematicWorld& to, const mlr::KinematicWorld& from){
  u0To = zeros(to.getJointStateDimension());

  intA match(u0From.d0);
  match = -1;
  for(mlr::Joint* jfrom : from.joints){
    mlr::Joint* jto = to.getJointByName(jfrom->name, false); // OLD: mlr::Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++){
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0;i<match.N;i++) {
    u0To(match(i)) = u0From(i);
  }
}


void transferKI_ft_BetweenTwoWorlds(arr& KI_ft_To, const arr& KI_ft_From, const mlr::KinematicWorld& to, const mlr::KinematicWorld& from){
  uint numberOfColumns = KI_ft_From.d1;
  if(KI_ft_From.d1 == 0) {
    numberOfColumns = 1;
    KI_ft_To = zeros(to.getJointStateDimension());
  } else {
    KI_ft_To = zeros(to.getJointStateDimension(), KI_ft_From.d1);
  }

  intA match(KI_ft_From.d0);
  match = -1;
  for(mlr::Joint* jfrom : from.joints){
    mlr::Joint* jto = to.getJointByName(jfrom->name, false); // OLD: mlr::Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++){
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0;i<match.N;i++) {
    for(uint j=0;j < numberOfColumns;j++){
      if(numberOfColumns > 1) {
        KI_ft_To(match(i), j) = KI_ft_From(i,j);
      } else {
        KI_ft_To(match(i)) = KI_ft_From(i);
      }
    }
  }
}
#endif

//===========================================================================
//===========================================================================
// opengl
//===========================================================================
//===========================================================================



#ifndef MLR_ORS_ONLY_BASICS

/**
 * @brief Bind ors to OpenGL.
 * Afterwards OpenGL can show the ors graph.
 *
 * @param graph the ors graph.
 * @param gl OpenGL which shows the ors graph.
 */
void bindOrsToOpenGL(mlr::KinematicWorld& graph, OpenGL& gl) {
  gl.add(glStandardScene, 0);
  gl.add(mlr::glDrawGraph, &graph);
//  gl.setClearColors(1., 1., 1., 1.);

  mlr::Body* glCamera = graph.getBodyByName("glCamera");
  if(glCamera) {
    gl.camera.X = glCamera->X;
    gl.resize(500,500);
  } else {
    gl.camera.setPosition(10., -15., 8.);
    gl.camera.focus(0, 0, 1.);
    gl.camera.upright();
  }
  gl.update();
}
#endif

#ifndef MLR_ORS_ONLY_BASICS

/// static GL routine to draw a mlr::KinematicWorld
void mlr::glDrawGraph(void *classP) {
  ((mlr::KinematicWorld*)classP)->glDraw(NoOpenGL);
}

void mlr::glDrawProxies(void *P){
  ProxyL& proxies = *((ProxyL*)P);
  glPushMatrix();
  for(mlr::Proxy* p:proxies) p->glDraw(NoOpenGL);
  glPopMatrix();
}



void displayState(const arr& x, mlr::KinematicWorld& G, const char *tag){
  G.setJointState(x);
  G.gl().watch(tag);
}

void displayTrajectory(const arr& _x, int steps, mlr::KinematicWorld& G, const KinematicSwitchL& switches, const char *tag, double delay, uint dim_z, bool copyG) {
  if(!steps) return;
  for(mlr::Shape *s : G.shapes) if(s->mesh.V.d0!=s->mesh.Vn.d0 || s->mesh.T.d0!=s->mesh.Tn.d0) {
    s->mesh.computeNormals();
  }
  mlr::KinematicWorld *Gcopy;
  if(switches.N) copyG=true;
  if(!copyG) Gcopy=&G;
  else{
    Gcopy = new mlr::KinematicWorld;
    Gcopy->copy(G,true);
  }
  arr x,z;
  if(dim_z){
    x.referToRange(_x,0,-dim_z-1);
    z.referToRange(_x,-dim_z,-1);
  }else{
    x.referTo(_x);
  }
  uint n=Gcopy->getJointStateDimension()-dim_z;
  x.reshape(x.N/n,n);
  uint num, T=x.d0-1;
  if(steps==1 || steps==-1) num=T; else num=steps;
  for(uint k=0; k<=(uint)num; k++) {
    uint t = (T?(k*T/num):0);
    if(switches.N){
      for(mlr::KinematicSwitch *sw: switches)
        if(sw->timeOfApplication==t)
          sw->apply(*Gcopy);
    }
    if(dim_z) Gcopy->setJointState(cat(x[t], z));
    else Gcopy->setJointState(x[t]);
    if(delay<0.){
      if(delay<-10.) FILE("z.graph") <<*Gcopy;
      Gcopy->gl().watch(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
    }else{
      Gcopy->gl().update(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
      if(delay) mlr::wait(delay);
    }
  }
  if(steps==1)
    Gcopy->gl().watch(STRING(tag <<" (time " <<std::setw(3) <<T <<'/' <<T <<')').p);
  if(copyG) delete Gcopy;
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

int animateConfiguration(mlr::KinematicWorld& C, Inotify *ino) {
  arr x, x0;
  uint t, i;
  C.getJointState(x0);
  arr lim = C.getLimits();
  C.gl().pressedkey=0;
  const int steps = 50;
  for(i=x0.N; i--;) {
    x=x0;
    double upper_lim = lim(i,1);
    double lower_lim = lim(i,0);
    double delta = upper_lim - lower_lim;
    double center = lower_lim + .5*delta;
    if(delta<=1e-10){ center=x0(i); delta=1.; }
    double offset = acos( 2. * (x0(i) - center) / delta );
    if(offset!=offset) offset=0.; //if NAN

    for(t=0; t<steps; t++) {
      if(C.gl().pressedkey==13 || C.gl().pressedkey==27 || C.gl().pressedkey=='q') return C.gl().pressedkey;
      if(ino && ino->pollForModification()) return 13;

      x(i) = center + (delta*(0.5*cos(MLR_2PI*t/steps + offset)));
      // Joint limits
      checkNan(x);
      C.setJointState(x);
      C.gl().update(STRING("DOF = " <<i), false, false, true);
      mlr::wait(0.01);
    }
  }
  C.setJointState(x0);
  return C.gl().update("", false, false, true);
}


mlr::Body *movingBody=NULL;
mlr::Vector selpos;
double seld, selx, sely, selz;

struct EditConfigurationClickCall:OpenGL::GLClickCall {
  mlr::KinematicWorld *ors;
  EditConfigurationClickCall(mlr::KinematicWorld& _ors) { ors=&_ors; }
  bool clickCallback(OpenGL& gl) {
    OpenGL::GLSelect *top=gl.topSelection;
    if(!top) return false;
    uint i=top->name;
    cout <<"CLICK call: id = 0x" <<std::hex <<gl.topSelection->name <<" : ";
    gl.text.clear();
    if((i&3)==1) {
      mlr::Shape *s=ors->shapes(i>>2);
      gl.text <<"shape selection: shape=" <<s->name <<" body=" <<s->body->name <<" X=" <<s->X <<endl;
//      listWrite(s->ats, gl.text, "\n");
      cout <<gl.text;
    }
    if((i&3)==2) {
      mlr::Joint *j=ors->joints(i>>2);
      gl.text
          <<"edge selection: " <<j->from->name <<' ' <<j->to->name
         <<"\nA=" <<j->A <<"\nQ=" <<j->Q <<"\nB=" <<j->B <<endl;
//      listWrite(j->ats, gl.text, "\n");
      cout <<gl.text;
    }
    cout <<endl;
    return true;
  }
};

struct EditConfigurationHoverCall:OpenGL::GLHoverCall {
  mlr::KinematicWorld *ors;
  EditConfigurationHoverCall(mlr::KinematicWorld& _ors);// { ors=&_ors; }
  bool hoverCallback(OpenGL& gl) {
//    if(!movingBody) return false;
    if(!movingBody) {
      mlr::Joint *j=NULL;
      mlr::Shape *s=NULL;
      mlr::timerStart(true);
      gl.Select(true);
      OpenGL::GLSelect *top=gl.topSelection;
      if(!top) return false;
      uint i=top->name;
      cout <<mlr::timerRead() <<"HOVER call: id = 0x" <<std::hex <<gl.topSelection->name <<endl;
      if((i&3)==1) s=ors->shapes(i>>2);
      if((i&3)==2) j=ors->joints(i>>2);
      gl.text.clear();
      if(s) {
        gl.text <<"shape selection: body=" <<s->body->name <<" X=" <<s->body->X <<" ats=" <<endl;
        listWrite(s->ats, gl.text, "\n");
      }
      if(j) {
        gl.text
            <<"edge selection: " <<j->from->name <<' ' <<j->to->name
           <<"\nA=" <<j->A <<"\nQ=" <<j->Q <<"\nB=" <<j->B <<endl;
        listWrite(j->ats, gl.text, "\n");
      }
    } else {
      //gl.Select();
      //double x=0, y=0, z=seld;
      //double x=(double)gl.mouseposx/gl.width(), y=(double)gl.mouseposy/gl.height(), z=seld;
      double x=gl.mouseposx, y=gl.mouseposy, z=seld;
      gl.unproject(x, y, z, true);
      cout <<"x=" <<x <<" y=" <<y <<" z=" <<z <<" d=" <<seld <<endl;
      movingBody->X.pos = selpos + ARR(x-selx, y-sely, z-selz);
    }
    return true;
  }
};

EditConfigurationHoverCall::EditConfigurationHoverCall(mlr::KinematicWorld& _ors) {
  ors=&_ors;
}

struct EditConfigurationKeyCall:OpenGL::GLKeyCall {
  mlr::KinematicWorld &K;
  bool &exit;
  EditConfigurationKeyCall(mlr::KinematicWorld& _K, bool& _exit): K(_K), exit(_exit){}
  bool keyCallback(OpenGL& gl) {
    if(gl.pressedkey==' '){ //grab a body
      if(movingBody) { movingBody=NULL; return true; }
      mlr::Joint *j=NULL;
      mlr::Shape *s=NULL;
      gl.Select();
      OpenGL::GLSelect *top=gl.topSelection;
      if(!top) { cout <<"No object below mouse!" <<endl;  return false; }
      uint i=top->name;
      //cout <<"HOVER call: id = 0x" <<std::hex <<gl.topSelection->name <<endl;
      if((i&3)==1) s=K.shapes(i>>2);
      if((i&3)==2) j=K.joints(i>>2);
      if(s) {
        cout <<"selected shape " <<s->name <<" of body " <<s->body->name <<endl;
        selx=top->x;
        sely=top->y;
        selz=top->z;
        seld=top->dmin;
        cout <<"x=" <<selx <<" y=" <<sely <<" z=" <<selz <<" d=" <<seld <<endl;
        selpos = s->body->X.pos;
        movingBody=s->body;
      }
      if(j) {
        cout <<"selected joint " <<j->index <<" connecting " <<j->from->name <<"--" <<j->to->name <<endl;
      }
      return true;
    }else switch(gl.pressedkey) {
      case '1':  K.orsDrawBodies^=1;  break;
      case '2':  K.orsDrawShapes^=1;  break;
      case '3':  K.orsDrawJoints^=1;  K.orsDrawMarkers^=1; break;
      case '4':  K.orsDrawProxies^=1;  break;
      case '5':  gl.reportSelects^=1;  break;
      case '6':  gl.reportEvents^=1;  break;
      case '7':  K.writePlyFile("z.ply");  break;
      case 'j':  gl.camera.X.pos += gl.camera.X.rot*mlr::Vector(0, 0, .1);  break;
      case 'k':  gl.camera.X.pos -= gl.camera.X.rot*mlr::Vector(0, 0, .1);  break;
      case 'i':  gl.camera.X.pos += gl.camera.X.rot*mlr::Vector(0, .1, 0);  break;
      case ',':  gl.camera.X.pos -= gl.camera.X.rot*mlr::Vector(0, .1, 0);  break;
      case 'l':  gl.camera.X.pos += gl.camera.X.rot*mlr::Vector(.1, .0, 0);  break;
      case 'h':  gl.camera.X.pos -= gl.camera.X.rot*mlr::Vector(.1, 0, 0);  break;
      case 'a':  gl.camera.focus(
          (gl.camera.X.rot*(gl.camera.foc - gl.camera.X.pos)
           ^ gl.camera.X.rot*mlr::Vector(1, 0, 0)) * .001
          + gl.camera.foc);
        break;
      case 's':  gl.camera.X.pos +=
          (
            gl.camera.X.rot*(gl.camera.foc - gl.camera.X.pos)
            ^(gl.camera.X.rot * mlr::Vector(1., 0, 0))
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

void editConfiguration(const char* filename, mlr::KinematicWorld& C) {
//  gl.exitkeys="1234567890qhjklias, "; //TODO: move the key handling to the keyCall!
  bool exit=false;
//  C.gl().addHoverCall(new EditConfigurationHoverCall(C));
  C.gl().addKeyCall(new EditConfigurationKeyCall(C,exit));
  C.gl().addClickCall(new EditConfigurationClickCall(C));
  Inotify ino(filename);
  for(;!exit;) {
    cout <<"reloading `" <<filename <<"' ... " <<std::endl;
    mlr::KinematicWorld W;
    try {
      mlr::lineCount=1;
      W <<FILE(filename);
      C.gl().dataLock.writeLock();
      C = W;
      C.gl().dataLock.unlock();
    } catch(const char* msg) {
      cout <<"line " <<mlr::lineCount <<": " <<msg <<" -- please check the file and press ENTER" <<endl;
      C.gl().watch();
      continue;
    }
    C.gl().update();
    if(exit) break;
    cout <<"animating.." <<endl;
    //while(ino.pollForModification());
    animateConfiguration(C, &ino);
    if(exit) break;
    cout <<"watching..." <<endl;
#if 0
    ino.waitForModification();
#else
    C.gl().watch();
#endif
    if(!mlr::getInteractivity()){
      exit=true;
    }
  }
}

//#endif

#else ///MLR_GL
#ifndef MLR_ORS_ONLY_BASICS
void bindOrsToOpenGL(mlr::KinematicWorld&, OpenGL&) { NICO };
void mlr::KinematicWorld::glDraw(OpenGL&) { NICO }
void mlr::glDrawGraph(void *classP) { NICO }
void editConfiguration(const char* orsfile, mlr::KinematicWorld& C) { NICO }
void animateConfiguration(mlr::KinematicWorld& C, Inotify*) { NICO }
void glTransform(const mlr::Transformation&) { NICO }
void displayTrajectory(const arr&, int, mlr::KinematicWorld&, const char*, double) { NICO }
void displayState(const arr&, mlr::KinematicWorld&, const char*) { NICO }
#endif
#endif
/** @} */

//===========================================================================
//===========================================================================
// featherstone
//===========================================================================
//===========================================================================

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
void skew(arr& X, const double *v);

/// as above
arr skew(const double *v);

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
void RBmci(arr& rbi, double m, double *c, const mlr::Matrix& I);

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
mlr::Body *robotbody(uint i, const Featherstone::Robot& robot) { return robot.C->nodes(i); }

uint Featherstone::Robot::N() const { return C->nodes.N; }

int Featherstone::Robot::parent(uint i) const {
  mlr::Joint *e=C->nodes(i)->inLinks(0);
  if(e) return e->from->index;
  return -1;
}

byte Featherstone::Robot::dof(uint i) const {
  mlr::Body *n=C->nodes(i);
  if(n->fixed) return 0;
  if(n->inLinks.N) {
    switch(n->inLinks(0)->type) {
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
  mlr::Quaternion r;
  mlr::Matrix R;
  arr Ss, rr;
  switch(d_i) {
    case 0: S.resize(6, (uint)0); S.setZero(); break;
    case 1: S.resize(6, 1); S.setZero(); S(0, 0)=1.; break;
    case 3:
      S.resize(6, 3); S.setZero();
      S(0, 0)=1.; S(1, 1)=1.; S(2, 2)=1.;
      break;
      r = C->nodes(i)->inLinks(0)->X.r;
      r.invert();
      r.getMatrix(R.m);
      memmove(S.p, R.m, 9*sizeof(double));
      break;
    case 4:
      S.resize(6, 4); S.setZero();
      r = C->nodes(i)->inLinks(0)->X.r;
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
  mlr::Transformation f;
  mlr::Joint *e1=C->nodes(i)->firstIn;
  if(!e1) {
    f.setZero();
  } else {
    mlr::Joint *e0=e1->from->firstIn;
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
  mlr::Joint *e=C->nodes(i)->firstIn;
  double m=C->nodes(i)->mass;
  mlr::Vector com;
  if(e) com = e->B.p; else com = C->nodes(i)->X.p;
  //arr Ic(3, 3);  Ic.setDiag(.1*m);
  arr I;
  RBmci(I, m, com.v, C->nodes(i)->inertia);
  return I;
}

const arr Featherstone::Robot::force(uint i) const {
  mlr::Body *n=C->nodes(i);
  CHECK(n, "is not a body with input joint");
  mlr::Joint *e=n->firstIn;
  //CHECK(e, "is not a body with input joint");
  mlr::Transformation g;
  g=n->X;
  if(e) g.subRelativeFrame(e->B);
  mlr::Vector fo = g.r/n->force;
  mlr::Vector to;
  if(e) to = g.r/(n->torque + (g.r*e->B.p)^n->force);
  else  to = g.r/(n->torque);
  arr f(6);
  f(0)=to.x;  f(1)=to.y;  f(2)=to.z;
  f(3)=fo.x;  f(4)=fo.y;  f(5)=fo.z;
  return f;
}
#endif

void Featherstone::skew(arr& X, const double *v) {
  X.resize(3, 3);  X.setZero();
  X(0, 1) = -v[2];  X(1, 0) = v[2];
  X(1, 2) = -v[0];  X(2, 1) = v[0];
  X(2, 0) = -v[1];  X(0, 2) = v[1];
}

arr Featherstone::skew(const double *v) { arr X; skew(X, v); return X; }

void FrameToMatrix(arr &X, const mlr::Transformation& f) {
  arr z(3, 3);  z.setZero();
  arr r(3, 3);  Featherstone::skew(r, &f.pos.x);
  arr R(3, 3);  f.rot.getMatrix(R.p);
  transpose(R);
  X.resize(6, 6);  X.setBlockMatrix(R, z, R*~r, R); //[[unklar!!]]
  //cout <<"\nz=" <<z <<"\nr=" <<r <<"\nR=" <<R <<"\nX=" <<X <<endl;
}

void mlr::Link::setFeatherstones() {
  switch(type) {
    case -1:     CHECK_EQ(parent,-1, ""); _h.clear();  break;
    case JT_rigid:
    case JT_transXYPhi:
      qIndex=-1;
      _h=zeros(6);
      break;
    case JT_hingeX: _h.resize(6); _h.setZero(); _h(0)=1.; break;
    case JT_hingeY: _h.resize(6); _h.setZero(); _h(1)=1.; break;
    case JT_hingeZ: _h.resize(6); _h.setZero(); _h(2)=1.; break;
    case JT_transX: _h.resize(6); _h.setZero(); _h(3)=1.; break;
    case JT_transY: _h.resize(6); _h.setZero(); _h(4)=1.; break;
    case JT_transZ: _h.resize(6); _h.setZero(); _h(5)=1.; break;
    default: NIY;
  }
  Featherstone::RBmci(_I, mass, com.p(), inertia);
  
  updateFeatherstones();
}

void mlr::Link::updateFeatherstones() {
  FrameToMatrix(_A, A);
  FrameToMatrix(_Q, Q);
  
  mlr::Transformation XQ;
  XQ=X;
  XQ.appendTransformation(Q);
  mlr::Vector fo = XQ.rot/force;
  mlr::Vector to = XQ.rot/(torque + ((XQ.rot*com)^force));
  _f.resize(6);
  _f(0)=to.x;  _f(1)=to.y;  _f(2)=to.z;
  _f(3)=fo.x;  _f(4)=fo.y;  _f(5)=fo.z;
}

void GraphToTree(mlr::Array<mlr::Link>& tree, const mlr::KinematicWorld& C) {
  tree.resize(C.bodies.N);
  
  for(mlr::Link& link:tree){ link.parent=-1; link.qIndex=-1; }

  for(mlr::Body* body:C.bodies) {
    mlr::Link& link=tree(body->index);
    if(body->inLinks.N && body->inLinks(0)->qDim()) { //is not a root
      CHECK_EQ(body->inLinks.N,1, "this is not a tree");
      mlr::Joint *j=body->inLinks(0);
      
      link.type   = j->type;
      link.qIndex = j->qIndex;
      link.parent = j->from->index;
      
      link.com = j->B*body->com;

      if(j->from->inLinks.N) link.A=j->from->inLinks(0)->B;
      else link.A=j->from->X;
      link.A.appendTransformation(j->A);
      
      link.X = j->from->X;
      link.X.appendTransformation(j->A);
      
      link.Q=j->Q;
    } else {
//      CHECK_EQ(body->inLinks.N,0, "dammit");
      
      link.type=-1;
      link.qIndex=-1;
      link.parent=-1;
      link.com=body->X*body->com;
      link.A.setZero();
      link.X.setZero();
      link.Q.setZero();
    }
    link.mass=body->mass; CHECK(link.mass>0. || link.qIndex==-1, "a moving link without mass -> this will diverge");
    link.inertia=body->inertia;
    link.force=body->force;
    link.torque=body->torque;
  }

  for(mlr::Link& link:tree) link.setFeatherstones();
}

void updateGraphToTree(mlr::Array<mlr::Link>& tree, const mlr::KinematicWorld& C) {
  CHECK_EQ(tree.N,C.bodies.N, "");
  
  uint i;
  mlr::Body *p;
  mlr::Joint *e;
  mlr::Transformation f;
  i=0;
  for_list(mlr::Body, n, C.bodies) {
    i=n_COUNT;
    if(n->inLinks.N) {
      e=n->inLinks(0);
      p=e->from;
      
      if(!p->inLinks.N) {
        f=p->X;
        f.appendTransformation(e->A);
        tree(i).A=f;
      }
      
      f = p->X;
      f.appendTransformation(e->A);
      tree(i).X=f;
      
      tree(i).Q=e->Q;
    } else {
      tree(i).com=n->X.pos;
    }
    tree(i).force=n->force;
    tree(i).torque=n->torque;
  }
  for(i=0; i<tree.N; i++) tree(i).updateFeatherstones();
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
void Featherstone::RBmci(arr& rbi, double m, double *c, const mlr::Matrix& I) {
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
  mlr::Array<arr> S(N), qd_i(N), qdd_i(N), tau_i(N);
  arr Xup(N, 6, 6), v(N, 6), f(N, 6), a(N, 6);
  arr Q;
  
  for(i=0, n=0; i<N; i++) {
    d_i=robot.dof(i);
    if(d_i) {
      qd_i(i) .referToRange(qd , n, n+d_i-1);
      qdd_i(i).referToRange(qdd, n, n+d_i-1);
      tau_i(i).referToRange(tau, n, n+d_i-1);
    } else {
      qd_i(i) .clear(); qd_i(i). resize(0);
      qdd_i(i).clear(); qdd_i(i).resize(0);
      tau_i(i).clear(); tau_i(i).resize(0);
    }
    n += d_i;
    S(i) = robot.S(i);
    if(robot.C->nodes(i)->inLinks.N) {
      FrameToMatrix(Q, robot.C->nodes(i)->inLinks(0)->X);
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
      mlr::Transformation f, r, g;
      f=robot.C->nodes(i)->X;
      f.subRelativeFrame(robot.C->nodes(i)->inLinks(0)->B);
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
  mlr::Array<arr> h(N), qd_i(N), qdd_i(N), tau_i(N), I_h(N), h_I_h(N), inv_h_I_h(N), tau__h_fA(N);
  arr Xup(N, 6, 6), v(N, 6), dh_dq(N, 6), f(N, 6), IA(N, 6, 6), fA(N, 6), a(N, 6);
  arr vJ, Ia, fa;
  arr Q;
  
  for(i=0, n=0; i<N; i++) {
    //for general multi-dimensional joints, pick the sub-arrays
    d_i=robot.dof(i);
    if(d_i) {
      qd_i(i) .referToRange(qd , n, n+d_i-1);
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
    if(robot.C->nodes(i)->inLinks.N) {
      FrameToMatrix(Q, robot.C->nodes(i)->inLinks(0)->X);
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
void mlr::fwdDynamics_aba_nD(arr& qdd,
                             const mlr::LinkTree& tree,
                             const arr& qd,
                             const arr& tau) {
  int par;
  uint i, N=tree.N, d_i, n;
  mlr::Array<arr> h(N), qd_i(N), qdd_i(N), tau_i(N), I_h(N), h_I_h(N), u(N);
  arr Xup(N, 6, 6), v(N, 6), dh_dq(N, 6), IA(N, 6, 6), fA(N, 6), a(N, 6);
  qdd.resizeAs(tau);
  
  for(i=0, n=0; i<N; i++) {
    d_i=tree(i).dof();
    if(d_i) {
      qd_i(i) .referToRange(qd , n, n+d_i-1);
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
    Xup[i] = tree(i)._Q * tree(i)._A; //the transformation from the i-th to the j-th
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

void mlr::fwdDynamics_aba_1D(arr& qdd,
                             const mlr::LinkTree& tree,
                             const arr& qd,
                             const arr& tau) {
  int par;
  uint i, N=tree.N, iq;
  arr h(N, 6), I_h(N, 6), h_I_h(N), inv_h_I_h(N), taui(N), tau__h_fA(N);
  arr Xup(N, 6, 6), v(N, 6), dh_dq(N, 6), IA(N, 6, 6), fA(N, 6), a(N, 6);
  arr vJ, Ia, fa;
  qdd.resizeAs(tau);
  
  //fwd: compute the velocities v[i] and external + Coriolis forces fA[i] of all bodies
  for(i=0; i<N; i++) {
    iq = tree(i).qIndex;
    par = tree(i).parent;
    Xup[i]() = tree(i)._Q * tree(i)._A; //the transformation from the i-th to the j-th
    if(par != -1) {
      h[i]() = tree(i)._h;
      vJ = h[i] * qd(iq); //equation (2), vJ = relative vel across joint i
      v[i]() = Xup[i] * v[par] + vJ; //eq (27)
      dh_dq[i]() = Featherstone::crossM(v[i]) * vJ;  //WHY??
      taui(i)=tau(iq);
    } else {
      h[i]() = 0.;
      v[i]() = 0.;
      dh_dq[i] = 0.;
      taui(i)=0.;
    }
    // v[i] = total velocity, but in joint coordinates
    IA[i] = tree(i)._I;
    //first part of eq (29)
    fA[i] = Featherstone::crossF(v[i]) * (tree(i)._I * v[i]) - tree(i)._f;
  }
  
  //bwd: propagate tree inertia
  for(i=N; i--;) {
    par = tree(i).parent;
    //eq (28)
    I_h[i]()      = IA[i] * h[i];
    h_I_h(i)      = scalarProduct(h[i], I_h[i]);
    inv_h_I_h(i)  = 1./h_I_h(i);
    tau__h_fA(i) = taui(i) - scalarProduct(h[i], fA[i]); //[change from above] last term in (13), 2nd equation below (13)
    if(par != -1) {
      Ia = IA[i] - I_h[i]*(inv_h_I_h(i)*~I_h[i]);
      fa = fA[i] + Ia*dh_dq[i] + I_h[i]*(inv_h_I_h(i)*tau__h_fA(i));
      IA[par] = IA[par] + ~Xup[i] * Ia * Xup[i];         //equation (12)
      fA[par] = fA[par] + ~Xup[i] * fa;                  //equation (13)
    }
  }
  
  for(i=0; i<N; i++) {
    iq = tree(i).qIndex;
    par= tree(i).parent;
    if(par != -1) {
      a[i] = Xup[i] * a[par] + dh_dq[i]; //[change from above]
      qdd(iq) = inv_h_I_h(i)*(tau__h_fA(i) - scalarProduct(I_h[i], a[i])); //equation (14)
      a[i] = a[i] + h[i]*qdd(iq); //equation above (14)
    } else {
      a[i] = dh_dq[i]; //[change from above]
    }
  }
}

//===========================================================================

void mlr::invDynamics(arr& tau,
                      const mlr::LinkTree& tree,
                      const arr& qd,
                      const arr& qdd) {
  int par;
  uint i, N=tree.N, d_i, n;
  mlr::Array<arr> h(N), qd_i(N), qdd_i(N), tau_i(N);
  arr Xup(N, 6, 6), v(N, 6), fJ(N, 6), a(N, 6);
  tau.resizeAs(qdd);
  
  for(i=0, n=0; i<N; i++) {
    d_i=tree(i).dof();
    if(d_i) {
      qd_i(i) .referToRange(qd , n, n+d_i-1);
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
    Xup[i] = tree(i)._Q * tree(i)._A; //the transformation from the i-th to the j-th
  }
  CHECK(n==qd.N && n==qdd.N && n==tau.N, "")
  
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
    if(tree(i).dof()) tau_i(i) = ~h(i) * fJ[i];
    if(par != -1)     fJ[par]() += ~Xup[i] * fJ[i];
  }
}


//===========================================================================

void mlr::equationOfMotion(arr& H, arr& C,
                           const mlr::LinkTree& tree,
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
  
  avp.setZero();
  
  for(i=0; i<N; i++) {
    iq  = tree(i).qIndex;
    par = tree(i).parent;
    Xup[i]() = tree(i)._Q * tree(i)._A; //the transformation from the i-th to the j-th
    if(par!=-1) {
      h[i]() = tree(i)._h;
      if(iq!=-1) {//is not a fixed joint
        vJ = h[i] * qd(iq); //equation (2), vJ = relative vel across joint i
      } else{
        vJ = zeros(6);
      }
      v[i]() = Xup[i] * v[par] + vJ;
      dh_dq[i]() = Featherstone::crossM(v[i]) * vJ;  //WHY??
      avp[i]() = Xup[i]*avp[par] + Featherstone::crossM(v[i])*vJ;
    } else {
      h[i]() = 0.;
      v[i]() = 0.;
      dh_dq[i] = 0.;
      avp[i]() = 0.;
    }
    // v[i] = total velocity, but in joint coordinates
    IC[i]() = tree(i)._I;
    fvp[i] = tree(i)._I*avp[i] + Featherstone::crossF(v[i])*(tree(i)._I*v[i]) - tree(i)._f;
  }
  
  C = zeros(qd.N);
  
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
  
  H = zeros(qd.N, qd.N);
  
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
  for(i=0;i<N;i++){ iq = tree(i).qIndex; if(iq!=-1) filled(iq)=true; }
  for(i=0;i<qd.N;i++) if(!filled(i)){
    H(i,i) = 1.;
//    C(i) = -100.*qd(i);
  }
}

//===========================================================================

void mlr::fwdDynamics_MF(arr& qdd,
                         const mlr::LinkTree& tree,
                         const arr& qd,
                         const arr& u) {
                         
  arr M, Minv, F;
  equationOfMotion(M, F, tree, qd);
  inverse(Minv, M);
//  inverse_SymPosDef(Minv, M);
  
  qdd = Minv * (u - F);
}

// #else ///MLR_FEATHERSTONE
// void GraphToTree(mlr::LinkTree& tree, const mlr::KinematicWorld& C) { NIY; }
// void updateGraphToTree(mlr::LinkTree& tree, const mlr::KinematicWorld& C) { NIY; }
// void Featherstone::equationOfMotion(arr& H, arr& C,
//                                     const mlr::LinkTree& tree,
//                                     const arr& qd) { NIY; }
// void Featherstone::fwdDynamics_MF(arr& qdd,
//                                   const mlr::LinkTree& tree,
//                                   const arr& qd,
//                                   const arr& tau) { NIY; }
// void Featherstone::invDynamics(arr& tau,
//                                const mlr::LinkTree& tree,
//                                const arr& qd,
//                                const arr& qdd) { NIY; }
// #endif
/** @} */


//===========================================================================
//===========================================================================
// template instantiations
//===========================================================================
//===========================================================================

#include <Core/util.tpp>

#ifndef  MLR_ORS_ONLY_BASICS
template mlr::Array<mlr::Shape*>::Array(uint);
template mlr::Shape* listFindByName(const mlr::Array<mlr::Shape*>&,const char*);

#include <Core/array.tpp>
template mlr::Array<mlr::Joint*>::Array();
#endif
/** @} */
