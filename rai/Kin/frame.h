/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/util.h>
#include <Geo/geo.h>
#include <Core/graph.h>
#include <Geo/mesh.h>
#include <Geo/geoms.h>

/* TODO:
 * replace the types by more fundamental:
 *  shapes: ssbox or ssmesh -- nothing else
 *  joint: 7bits
 *  body: maybe as is
 *
 * Collisions: The Proxies in Kin should only call GJK or exact ssbox-distance --> no use of center-of-mesh anymore!
 *
 */

namespace rai {
struct Frame;
struct Joint;
struct Shape;
struct Inertia;
struct Contact;
//enum ShapeType { ST_none=-1, ST_box=0, ST_sphere, ST_capsule, ST_mesh, ST_cylinder, ST_marker, ST_retired_SSBox, ST_pointCloud, ST_ssCvx, ST_ssBox };
enum JointType { JT_none=-1, JT_hingeX=0, JT_hingeY=1, JT_hingeZ=2, JT_transX=3, JT_transY=4, JT_transZ=5, JT_transXY=6, JT_trans3=7, JT_transXYPhi=8, JT_universal=9, JT_rigid=10, JT_quatBall=11, JT_phiTransXY=12, JT_XBall, JT_free, JT_time };
enum BodyType  { BT_none=-1, BT_dynamic=0, BT_kinematic, BT_static };
}

typedef rai::Array<rai::Frame*> FrameL;
typedef rai::Array<rai::Joint*> JointL;
typedef rai::Array<rai::Shape*> ShapeL;

extern rai::Frame& NoFrame;
//extern rai::Shape& NoShape;
//extern rai::Joint& NoJoint;

namespace rai {

//===========================================================================

/// a Frame can have a link (also joint), shape (visual or coll), and/or intertia (mass) attached to it
struct Frame : NonCopyable{
  struct KinematicWorld& K;  ///< a Frame is uniquely associated with a KinematicConfiguration
  uint ID;                   ///< unique identifier
  String name;               ///< name
  Frame *parent=NULL;        ///< parent frame
  FrameL parentOf;           ///< list of children [TODO: rename]
  Transformation Q=0;        ///< relative transform to parent
  Transformation X=0;        ///< frame's absolute pose
  double tau=0.;            ///< frame's absolute time (could be thought as part of the transformation X in space-time)
  Graph ats;                 ///< list of any-type attributes
  bool active=true;          ///< if false, this frame is skipped in computations (e.g. in fwd propagation)
  int flags=0;               ///< various flags that are used by task maps to impose costs/constraints in KOMO
  
  //attachments to the frame
  Joint *joint=NULL;         ///< this frame is an articulated joint
  Shape *shape=NULL;         ///< this frame has a (collision or visual) geometry
  Inertia *inertia=NULL;     ///< this frame has inertia (is a mass)
  Array<Contact*> contacts;  ///< this frame is in (near-) contact with other frames
  
  Frame(KinematicWorld& _K, const Frame *copyBody=NULL);
  Frame(Frame *_parent);
  ~Frame();
  
  void calc_X_from_parent();
  void calc_Q_from_parent(bool enforceWithinJoint = true);
  
  Frame* insertPreLink(const rai::Transformation& A=0);
  Frame* insertPostLink(const rai::Transformation& B=0);
  void unLink();
  void linkFrom(Frame *_parent, bool adoptRelTransform=false);
  bool isChildOf(const Frame* par, int order=1) const;
  
  Shape& getShape();
  Inertia& getInertia();
  
  void getRigidSubFrames(FrameL& F); ///< recursively collect all rigidly attached sub-frames (e.g., shapes of a link), (THIS is not included)
  Frame* getUpwardLink(rai::Transformation& Qtotal=NoTransformation, bool untilRigid=false) const; ///< recurse upward BEFORE the next joint and return relative transform (this->Q is not included!b)
  
  void read(const Graph &ats);
  void write(Graph &G);
  void write(std::ostream& os) const;

  //-- HIGHER LEVEL USER INTERFACE
  void setShape(rai::ShapeType shape, const std::vector<double>& size);
  void setPosition(const std::vector<double>& pos);
  void setQuaternion(const std::vector<double>& quat);
  void setRelativePosition(const std::vector<double>& pos);
  void setRelativeQuaternion(const std::vector<double>& quat);
  void setPointCloud(const std::vector<double>& points, const std::vector<byte>& colors);
  void setConvexMesh(const std::vector<double>& points, const std::vector<byte>& colors);
  void setColor(const std::vector<double>& color);

  arr getPosition(){ return X.pos.getArr(); }
  arr getQuaternion(){ return X.rot.getArr4d(); }
  arr getRotationMatrix(){ return X.rot.getArr(); }
  arr getRelativePosition(){ return Q.pos.getArr(); }
  arr getRelativeQuaternion(){ return Q.rot.getArr(); }
  arr getMeshPoints();
};
stdOutPipe(Frame)

//===========================================================================

/// for a Frame with Joint-Link, the relative transformation 'Q' is articulated
struct Joint : NonCopyable{
  Frame *frame;
  
  // joint information
  uint dim=0;
  uint qIndex;
  byte generator;    ///< (7bits), h in Featherstone's code (indicates basis vectors of the Lie algebra, but including the middle quaternion w)
  arr limits;        ///< joint limits (lo, up, [maxvel, maxeffort])
  arr q0;            ///< joint null position
  double H=1.;       ///< control cost scalar
  
  Joint *mimic=NULL; ///< if non-NULL, this joint's state is identical to another's
  
  Vector axis=0;          ///< joint axis (same as X.rot.getX() for standard hinge joints)
  Enum<JointType> type;   ///< joint type
  bool constrainToZeroVel=false;
  bool active=true;  ///< if false, this joint is not considered part of the q-vector
  
  //attachments to the joint
  struct Uncertainty *uncertainty=NULL;
  
  Joint(Frame& f, JointType type);
  Joint(Frame& f, Joint* copyJoint=NULL);
  Joint(Frame& from, Frame& f, Joint* copyJoint=NULL);
  ~Joint();
  
  const Transformation& X() const { return frame->parent->X; }
  const Transformation& Q() const { return frame->Q; }
  Frame *from() const { return frame->parent; }
  
  uint qDim() { return dim; }
  void calc_Q_from_q(const arr& q, uint n);
  arr calc_q_from_Q(const Transformation &Q) const;
  arr getScrewMatrix();
  uint getDimFromType() const;
  arr get_h() const;
  
  //access the K's q vector
  double& getQ();
  
  void makeRigid();
  void makeFree(double H_cost=0.);
  void setType(JointType _type);
  void flip();

  void read(const Graph& G);
  void write(Graph &g);
  void write(std::ostream& os) const;
};
stdOutPipe(Joint)

//===========================================================================

/// a Frame with Inertia has mass and, in physical simulation, has forces associated with it
struct Inertia : NonCopyable {
  Frame& frame;
  double mass=-1.;
  Matrix matrix=0;
  Enum<BodyType> type;
  Vector com=0;             ///< its center of mass
  Vector force=0, torque=0; ///< current forces applying on the body
  
  Inertia(Frame& f, rai::Inertia *copyInertia=NULL);
  ~Inertia();
  
  void defaultInertiaByShape();
  arr getFrameRelativeWrench();
  
  void write(std::ostream& os) const;
  void write(Graph &g);
  void read(const Graph& G);
};
stdOutPipe(Inertia)

//===========================================================================

/// a Frame with Shape is a collision or visual object
struct Shape : NonCopyable, GLDrawer {
  Frame& frame;
  ptr<Geom> geom;
  
  Geom& getGeom(); ///< creates a geom if not yet initialized
  void setGeomMimic(const Frame* f);
  Enum<ShapeType>& type() { return getGeom().type; }
  arr& size() { return getGeom().size; }
  double& size(uint i) { return getGeom().size.elem(i); }
  double radius() { arr &size = getGeom().size; if(size.N==1) return size(0); if(size.N>=4) return size(3); return 0.; }
  Mesh& mesh() { if(!getGeom().mesh.V.N) geom->createMeshes(); return geom->mesh; }
  Mesh& sscCore() { return getGeom().sscCore; }
  double alpha() { arr& C=getGeom().mesh.C; if(C.N==4) return C(3); return 1.; }
  
//  Enum<ShapeType> type;
//  arr size;
//  Mesh mesh, sscCore;
//  double mesh_radius=0.;
  char cont=0;           ///< are contacts registered (or filtered in the callback)
  bool visual=true;
  
  Shape(Frame& f, const Shape *copyShape=NULL); //new Shape, being added to graph and body's shape lists
  virtual ~Shape();

  bool canCollideWith(const Frame *f) const{
    if(!cont) return false;
    if(!f->shape || !f->shape->cont) return false;
    Frame *a = frame.getUpwardLink();
    Frame *b = f->getUpwardLink();
    if(a==b) return false;
    if(cont<0) if(a->isChildOf(b, -cont)) return false;
    if(f->shape->cont<0)  if(b->isChildOf(a, -f->shape->cont)) return false;
    return true;
  }

  void read(const Graph &ats);
  void write(std::ostream& os) const;
  void write(Graph &g);
  void glDraw(OpenGL&);
};

//===========================================================================

}// namespace rai

