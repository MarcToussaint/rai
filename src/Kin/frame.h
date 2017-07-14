#pragma once

#include <Core/util.h>
#include <Geo/geo.h>
#include <Core/graph.h>

namespace mlr{
struct Shape;
struct Joint;
enum ShapeType { ST_none=-1, ST_box=0, ST_sphere, ST_capsule, ST_mesh, ST_cylinder, ST_marker, ST_retired_SSBox, ST_pointCloud, ST_ssCvx, ST_ssBox };
enum JointType { JT_none=-1, JT_hingeX=0, JT_hingeY=1, JT_hingeZ=2, JT_transX=3, JT_transY=4, JT_transZ=5, JT_transXY=6, JT_trans3=7, JT_transXYPhi=8, JT_universal=9, JT_rigid=10, JT_quatBall=11, JT_phiTransXY=12, JT_glue, JT_free };
enum BodyType  { BT_none=-1, BT_dynamic=0, BT_kinematic, BT_static };
}

typedef mlr::Array<mlr::Joint*> JointL;
typedef mlr::Array<mlr::Shape*> ShapeL;

namespace mlr{

/// a rigid body (inertia properties, lists of attached joints & shapes)
struct Frame {
  struct KinematicWorld& world;
  uint index;          ///< unique identifier TODO:do we really need index??

  mlr::String name;     ///< name
  Transformation X;    ///< body's absolute pose

  Graph ats;   ///< list of any-type attributes

  //attachments to the frame
  struct FrameRel  *rel=NULL;        ///< this frame is a child or a parent frame, with fixed relative transformation
//  struct FrameGeom *geom=NULL;       ///< this frame has a (collision or visual) geometry
//  struct FrameInertia *inertia=NULL; ///< this frame has inertia (is a mass)


//  JointL inLinks;
  JointL outLinks;       ///< lists of in and out joints

  //dynamic properties
  Enum<BodyType> type;          ///< is globally fixed?
  double mass;           ///< its mass
  Matrix inertia;      ///< its inertia tensor
  Vector com;          ///< its center of gravity
  Vector force, torque; ///< current forces applying on the body
  Vector vel, angvel;   ///< linear and angular velocities

  ShapeL shapes;

  Frame(KinematicWorld& _world, const Frame *copyBody=NULL);
  ~Frame();

  struct Joint* joint() const;
  struct Frame* from() const;
  uint numInputs() const;
  bool hasJoint() const;

  void operator=(const Frame& b) {
    name=b.name; X=b.X; ats=b.ats;
    type=b.type; mass=b.mass; inertia=b.inertia; com=b.com; force=b.force; torque=b.torque;
  }
  void reset();
  void parseAts();
  void write(std::ostream& os) const;
  void read(std::istream& is);
};

struct FrameRel{
  Frame *from;
  mlr::Transformation rel;

  struct Joint *joint=NULL;    ///< this frame is an articulated joint

  FrameRel(Frame* _from) : from(_from) {}
};

struct Joint{
    uint index;

  // joint information
  uint dim;
  uint qIndex;
  byte generator; ///< (7bits), h in Featherstone's code (indicates basis vectors of the Lie algebra, but including the middle quaternion w)
  arr limits;        ///< joint limits (lo, up, [maxvel, maxeffort])
  arr q0;            ///< joint null position
  double H=1.;          ///< control cost scalar

  Joint *mimic;       ///< if non-NULL, this joint's state is identical to another's

  Frame* from;
  Frame* to;
  Transformation A=0;     ///< transformation from parent body to joint (attachment, usually static)
  Transformation Q=0;     ///< transformation within the joint (usually dynamic)
  Transformation B=0;     ///< transformation from joint to child body (attachment, usually static)
  Transformation X=0;     ///< joint pose in world coordinates (same as from->X*A)
  Vector axis=0;          ///< joint axis (same as X.rot.getX() for standard hinge joints)
  Enum<JointType> type;   ///< joint type
  bool constrainToZeroVel;
  uint agent;           ///< associate this Joint to a specific agent (0=default robot)

  Joint(Frame* _from, Frame* _to, Joint* copyJoint=NULL);
  ~Joint();

  uint qDim(){ return dim; }
  void calc_Q_from_q(const arr& q, uint n);
  arr calc_q_from_Q(const Transformation &Q) const;
  uint getDimFromType() const;
  void write(std::ostream& os) const;
  void read(const Graph& G);
};

struct FrameGeom{
  struct GeomStore& store;
  int geomID;
};

struct FrameInertia{
  arr centerOfMass;
  double mass;
  arr inertiaTensor;
};

} //namespace MLR

//===========================================================================

#if 0
/// a joint
struct Joint {
  KinematicWorld& world;
  uint index;           ///< unique identifier
  uint qIndex;          ///< index where this joint appears in the q-state-vector
  Frame *from, *to;      ///< pointers to from and to bodies
  Joint *mimic;         ///< if non-NULL, this joint's state is identical to another's
  uint agent;           ///< associate this Joint to a specific agent (0=default robot)
  bool constrainToZeroVel; ///< HACK yet: when creating new 'virtual' joints, constrain them to zero vel in paths

  mlr::String name;      ///< name
  Enum<JointType> type;       ///< joint type
  Transformation A;     ///< transformation from parent body to joint (attachment, usually static)
  Transformation Q;     ///< transformation within the joint (usually dynamic)
  Transformation B;     ///< transformation from joint to child body (attachment, usually static)
  Transformation X;     ///< joint pose in world coordinates (same as from->X*A)
  Vector axis;          ///< joint axis (same as X.rot.getX() for standard hinge joints)
  arr limits;           ///< joint limits (lo, up, [maxvel, maxeffort])
  double q0;            ///< joint null position
  double H;             ///< control cost factor
  Graph ats;    ///< list of any-type attributes

  Joint(KinematicWorld& G, Frame *f, Frame *t, const Joint *copyJoint=NULL); //new Shape, being added to graph and body's joint lists
  ~Joint();
  Joint(const Joint &j);
  void operator=(const Joint& j) {
    qIndex=j.qIndex; mimic=reinterpret_cast<Joint*>(j.mimic?1l:0l); agent=j.agent; constrainToZeroVel=j.constrainToZeroVel;
    name=j.name; type=j.type; A=j.A; Q=j.Q; B=j.B; X=j.X; axis=j.axis; limits=j.limits; q0=j.q0; H=j.H;
    ats=j.ats;
  }
  void reset();
  void parseAts();
  uint qDim();
  void applyTransformation(mlr::Transformation& f, const arr& q);
  void makeRigid();
  void write(std::ostream& os) const;
  void read(std::istream& is);
  mlr::String tag(){ return STRING(name <<':' <<type <<':' <<from->name <<'-' <<to->name); }
};
#endif
