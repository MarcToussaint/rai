#pragma once

#include <Core/util.h>
#include <Geo/geo.h>
#include <Core/graph.h>
#include <Geo/mesh.h>

namespace mlr{
struct Frame;
struct Shape;
struct Joint;
enum ShapeType { ST_none=-1, ST_box=0, ST_sphere, ST_capsule, ST_mesh, ST_cylinder, ST_marker, ST_retired_SSBox, ST_pointCloud, ST_ssCvx, ST_ssBox };
enum JointType { JT_none=-1, JT_hingeX=0, JT_hingeY=1, JT_hingeZ=2, JT_transX=3, JT_transY=4, JT_transZ=5, JT_transXY=6, JT_trans3=7, JT_transXYPhi=8, JT_universal=9, JT_rigid=10, JT_quatBall=11, JT_phiTransXY=12, JT_glue, JT_free };
enum BodyType  { BT_none=-1, BT_dynamic=0, BT_kinematic, BT_static };
}

typedef mlr::Array<mlr::Frame*> FrameL;
typedef mlr::Array<mlr::Joint*> JointL;
typedef mlr::Array<mlr::Shape*> ShapeL;

namespace mlr{

/// a rigid body (inertia properties, lists of attached joints & shapes)
struct Frame {
  struct KinematicWorld& K;
  uint ID;          ///< unique identifier TODO:do we really need index??
  mlr::String name;     ///< name
  Transformation X=0;    ///< body's absolute pose

  FrameL outLinks;       ///< lists of in and out joints

  //attachments to the frame
  struct FrameRel  *rel=NULL;        ///< this frame is a child or a parent frame, with fixed relative transformation
  struct Shape *shape=NULL;       ///< this frame has a (collision or visual) geometry
  struct FrameInertia *inertia=NULL; ///< this frame has inertia (is a mass)

  Graph ats;   ///< list of any-type attributes

  Frame(KinematicWorld& _world, const Frame *copyBody=NULL);
  ~Frame();

  struct Joint* joint() const;
  struct Frame* from() const;
  uint numInputs() const;
  bool hasJoint() const;

  void parseAts(const Graph &ats);
  void write(std::ostream& os) const;
};

struct FrameRel{
  Frame *from;
  Frame *to;
  mlr::Transformation rel=0;

  struct Joint *joint=NULL;    ///< this frame is an articulated joint

  FrameRel(Frame* _from, Frame* _to) : from(_from), to(_to) {
    from->outLinks.append(to);
    if(to->rel){
      CHECK_EQ(to->rel, this,"");
    }else{
      to->rel = this;
    }
  }

  ~FrameRel();
};

struct Joint{
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
  Matrix matrix;
  Enum<BodyType> type;
  Vector com=0;          ///< its center of gravity
  Vector force=0, torque=0; ///< current forces applying on the body
  Vector vel=0, angvel=0;   ///< linear and angular velocities

  FrameInertia(Frame *f) : type(BT_dynamic) {}

  void read(const Graph& ats);
};

} //namespace mlr

//===========================================================================

namespace mlr{

/// a shape (geometric shape like cylinder/mesh or just marker, associated to a body)
struct Shape : GLDrawer{
  uint ID;
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

}// namespace mlr
