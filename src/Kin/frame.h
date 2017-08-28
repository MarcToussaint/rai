#pragma once

#include <Core/util.h>
#include <Geo/geo.h>
#include <Core/graph.h>
#include <Geo/mesh.h>

/* TODO:
 * replace the types by more fundamental:
 *  shapes: ssbox or ssmesh -- nothing else
 *  joint: 7bits
 *  body: maybe as is
 *
 * Shape: refer to GeomStore instead of own mesh
 *
 * Collisions: The Proxies in Kin should only call GJK or exact ssbox-distance --> no use of center-of-mesh anymore!
 *
 */

namespace mlr{
struct Frame;
struct Joint;
struct Shape;
enum ShapeType { ST_none=-1, ST_box=0, ST_sphere, ST_capsule, ST_mesh, ST_cylinder, ST_marker, ST_retired_SSBox, ST_pointCloud, ST_ssCvx, ST_ssBox };
enum JointType { JT_none=-1, JT_hingeX=0, JT_hingeY=1, JT_hingeZ=2, JT_transX=3, JT_transY=4, JT_transZ=5, JT_transXY=6, JT_trans3=7, JT_transXYPhi=8, JT_universal=9, JT_rigid=10, JT_quatBall=11, JT_phiTransXY=12, JT_XBall, JT_free };
enum BodyType  { BT_none=-1, BT_dynamic=0, BT_kinematic, BT_static };
}

typedef mlr::Array<mlr::Frame*> FrameL;
typedef mlr::Array<mlr::Joint*> JointL;
typedef mlr::Array<mlr::Shape*> ShapeL;

extern mlr::Frame& NoFrame;
//extern mlr::Shape& NoShape;
//extern mlr::Joint& NoJoint;

namespace mlr{

//===========================================================================

/// a Frame can have a link (also joint), shape (visual or coll), and/or intertia (mass) attached to it
struct Frame {
  struct KinematicWorld& K;  ///< a Frame is uniquely associated with a KinematicConfiguration
  uint ID;                   ///< unique identifier
  mlr::String name;          ///< name
  Transformation X=0;        ///< body's absolute pose
  FrameL outLinks;           ///< lists of in and out joints
  Graph ats;                 ///< list of any-type attributes
  bool active=true;          ///< if false, this frame is skipped in computations (e.g. in fwd propagation)

  //attachments to the frame
  struct Link *link=NULL;            ///< this frame is a child or a parent frame, with fixed or articulated relative transformation
  struct Shape *shape=NULL;          ///< this frame has a (collision or visual) geometry
  struct Inertia *inertia=NULL; ///< this frame has inertia (is a mass)

  Frame(KinematicWorld& _world, const Frame *copyBody=NULL);
  ~Frame();

  struct Joint* joint() const;
  Frame *from() const;
  uint numInputs() const{ if(link) return 1; return 0; } //TODO: remove: use KinConf specific topSort, not generic; remove generic top sort..

  void parseAts(const Graph &ats);
  void write(std::ostream& os) const;
};
stdOutPipe(Frame)

//===========================================================================

/// a Frame with Link has a relative transformation 'Q' from the predecessor frame 'from'
struct Link{
  Frame *from;
  Frame *to;
  mlr::Transformation Q=0;

  //attachments to the link
  struct Joint *joint=NULL;    ///< this link is an articulated joint

  Link(Frame* _from, Frame* _to, Link * copyRel=NULL);
  ~Link();

  Link* insertPreLink(const mlr::Transformation& A);
  Link* insertPostLink(const mlr::Transformation& B);
  void write(std::ostream& os) const{
    os <<Q;
  }
};

//===========================================================================

/// for a Frame with Joint-Link, the relative transformation 'Q' is articulated
struct Joint{
  // joint information
  uint dim;
  uint qIndex;
  byte generator;   ///< (7bits), h in Featherstone's code (indicates basis vectors of the Lie algebra, but including the middle quaternion w)
  arr limits;       ///< joint limits (lo, up, [maxvel, maxeffort])
  arr q0;           ///< joint null position
  double H=1.;      ///< control cost scalar

  Joint *mimic;     ///< if non-NULL, this joint's state is identical to another's

  Link *link;
  Vector axis=0;          ///< joint axis (same as X.rot.getX() for standard hinge joints)
  Enum<JointType> type;   ///< joint type
  bool constrainToZeroVel;
  bool active=true; ///< if false, this joint is not considered part of the q-vector

  //attachments to the joint
  struct Uncertainty *uncertainty=NULL;

  Joint(Link *_link, Joint* copyJoint=NULL);
  Joint(Frame* _from, Frame* _to, Joint* copyJoint=NULL) : Joint(new Link(_from, _to), copyJoint) {}
  ~Joint();

  const Transformation& X() const{ return link->from->X; }
  const Transformation& Q() const{ return link->Q; }
  Frame *from() const{ return link->from; }
  Frame *to() const{ return link->to; }

  uint qDim(){ return dim; }
  void calc_Q_from_q(const arr& q, uint n);
  arr calc_q_from_Q(const Transformation &Q) const;
  uint getDimFromType() const;

  void makeRigid();

  void write(std::ostream& os) const;
  void read(const Graph& G);
};

stdOutPipe(Joint)

//===========================================================================

struct FrameGeom{
  struct GeomStore& store;
  int geomID;
};

//===========================================================================

/// a Frame with Inertia has mass and, in physical simulation, has forces associated with it
struct Inertia{
  Frame *frame;
  double mass;
  Matrix matrix;
  Enum<BodyType> type;
  Vector com=0;             ///< its center of mass
  Vector force=0, torque=0; ///< current forces applying on the body

  Inertia(Frame *f, mlr::Inertia *copyInertia=NULL);
  ~Inertia();

  void read(const Graph& ats);
};

//===========================================================================

/// a Frame with Shape is a collision or visual object
struct Shape : GLDrawer{
  Frame *frame;

  Enum<ShapeType> type;
  arr size;
  Mesh mesh, sscCore;
  double mesh_radius=0.;
  bool cont=false;           ///< are contacts registered (or filtered in the callback)

  Shape(Frame* b, const Shape *copyShape=NULL, bool referenceMeshOnCopy=false); //new Shape, being added to graph and body's shape lists
  virtual ~Shape();
  void read(const Graph &ats);
  void write(std::ostream& os) const;
  void glDraw(OpenGL&);
};

//===========================================================================

}// namespace mlr

