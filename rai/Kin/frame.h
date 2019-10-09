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
 *
 */

namespace rai {
struct Frame;
struct Joint;
struct Shape;
struct Inertia;
struct Contact;
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
  //
  // Transformation tokens allow setting frame state (in destructor) after manipulating a transform
  //

  struct Transformation_Xtoken{
    Frame& f;
    Transformation_Xtoken(Frame& _f):f(_f) {}
    ~Transformation_Xtoken();
    Transformation* operator->();
    Transformation& operator*();
    void operator=(const Transformation&);
  };

  struct Transformation_Qtoken{
    Frame& f;
    Transformation_Qtoken(Frame& _f):f(_f) {}
    ~Transformation_Qtoken();
    Transformation* operator->();
    Transformation& operator*();
    void operator=(const Transformation&);
  };

//===========================================================================

/// a Frame can have a link (also joint), shape (visual or coll), and/or intertia (mass) attached to it
struct Frame : NonCopyable{
  struct Configuration& K;  ///< a Frame is uniquely associated with a KinematicConfiguration
  uint ID;                   ///< unique identifier
  String name;               ///< name
  Frame *parent=NULL;        ///< parent frame
  FrameL parentOf;           ///< list of children [TODO: rename]
protected:
  Transformation Q=0;        ///< relative transform to parent
  Transformation X=0;        ///< frame's absolute pose
  //data structure state (lazy evaluation leave the state structure out of sync)
  bool _state_X_isGood=true; // X represents the current state
  void _state_setXBadinBranch();
  void _state_updateAfterTouchingX();
  void _state_updateAfterTouchingQ();
  //low-level fwd kinematics computation
  void calc_X_from_parent();
  void calc_Q_from_parent(bool enforceWithinJoint = true);
public:
  double tau=0.;             ///< frame's absolute time (could be thought as part of the transformation X in space-time)
  Graph ats;                 ///< list of any-type attributes
  
  //attachments to the frame
  Joint *joint=NULL;         ///< this frame is an articulated joint
  Shape *shape=NULL;         ///< this frame has a (collision or visual) geometry
  Inertia *inertia=NULL;     ///< this frame has inertia (is a mass)
  Array<Contact*> contacts;  ///< this frame is in (near-) contact with other frames

  Frame(Configuration& _K, const Frame *copyFrame=NULL);
  Frame(Frame *_parent);
  ~Frame();
  
  //accessors to attachments
  Shape& getShape();
  Inertia& getInertia();

  const Transformation& ensure_X();
  const Transformation& get_Q();
  const Transformation& get_X() const;
  Transformation_Xtoken set_X(){ return Transformation_Xtoken(*this); }
  Transformation_Qtoken set_Q(){ return Transformation_Qtoken(*this); }

  //structural operations
  Frame* insertPreLink(const rai::Transformation& A=0);
  Frame* insertPostLink(const rai::Transformation& B=0);
  void unLink();
  void linkFrom(Frame *_parent, bool adoptRelTransform=false);

  //structural information/retrieval
  bool isChildOf(const Frame* par, int order=1) const;
  void getRigidSubFrames(FrameL& F); ///< recursively collect all rigidly attached sub-frames (e.g., shapes of a link), (THIS is not included)
  void getPartSubFrames(FrameL& F); ///< recursively collect all frames of this part
  FrameL getPathToRoot();
  Frame* getUpwardLink(rai::Transformation& Qtotal=NoTransformation, bool untilPartBreak=false) const; ///< recurse upward BEFORE the next joint and return relative transform (this->Q is not included!b)
  FrameL getPathToUpwardLink(bool untilPartBreak=false); ///< recurse upward BEFORE the next joint and return relative transform (this->Q is not included!b)
  const char* isPart();

  //I/O
  void read(const Graph &ats);
  void write(Graph &G);
  void write(std::ostream& os) const;

  //-- HIGHER LEVEL USER INTERFACE
  void setShape(rai::ShapeType shape, const std::vector<double>& size);
  void setPose(const rai::Transformation& _X);
  void setPosition(const std::vector<double>& pos);
  void setQuaternion(const std::vector<double>& quat);
  void setRelativePosition(const std::vector<double>& pos);
  void setRelativeQuaternion(const std::vector<double>& quat);
  void setPointCloud(const std::vector<double>& points, const std::vector<byte>& colors={});
  void setConvexMesh(const std::vector<double>& points, const std::vector<byte>& colors={}, double radius=0.);
  void setColor(const std::vector<double>& color);
  void setJoint(rai::JointType jointType);
  void setContact(int cont);
  void setMass(double mass);

  arr getPosition(){ return ensure_X().pos.getArr(); }
  arr getQuaternion(){ return ensure_X().rot.getArr4d(); }
  arr getRotationMatrix(){ return ensure_X().rot.getArr(); }
  arr getRelativePosition(){ return ensure_X().pos.getArr(); }
  arr getRelativeQuaternion(){ return ensure_X().rot.getArr(); }
  arr getMeshPoints();
  arr getMeshCorePoints();

  friend struct Configuration;
  friend struct Configuration_ext;
  friend struct KinematicSwitch;
  friend struct Joint;
  friend struct Transformation_Xtoken;
  friend struct Transformation_Qtoken;
};
stdOutPipe(Frame)

//===========================================================================

/// for a Frame with Joint-Link, the relative transformation 'Q' is articulated
struct Joint : NonCopyable{
  Frame *frame;      ///< this is the frame that Joint articulates! I.e., the output frame
  
  // joint information
  uint dim=UINT_MAX;
  uint qIndex;
  byte generator;    ///< (7bits), h in Featherstone's code (indicates basis vectors of the Lie algebra, but including the middle quaternion w)
  arr limits;        ///< joint limits (lo, up, [maxvel, maxeffort])
  arr q0;            ///< joint null position
  double H=1.;       ///< control cost scalar
  double scale=1.;   ///< scaling robot-q = scale * q-vector
  
  Joint *mimic=NULL; ///< if non-NULL, this joint's state is identical to another's
  
  Vector axis=0;          ///< joint axis (same as X.rot.getX() for standard hinge joints)
  Enum<JointType> type;   ///< joint type
  bool active=true;  ///< if false, this joint is not considered part of the q-vector
  
  //attachments to the joint
  struct Uncertainty *uncertainty=NULL;
  
  Joint(Frame& f, JointType type);
  Joint(Frame& f, Joint* copyJoint=NULL);
  Joint(Frame& from, Frame& f, Joint* copyJoint=NULL);
  ~Joint();
  
  const Transformation& X() const; ///< the frame where the joint STARTS (i.e. parent->X)
  const Transformation& Q() const; ///< the transformation realized by this joint (i.e. from parent->X to frame->X)
  Frame *from() const { return frame->parent; }
  
  uint qDim();
  void calc_Q_from_q(const arr& q, uint n);
  arr calc_q_from_Q(const Transformation &Q) const;
  arr getScrewMatrix();
  uint getDimFromType() const;
  arr get_h() const;
  
  bool isPartBreak(){ return (dim!=1 && !mimic) || type==JT_time; }

  //access the K's q vector
  double& getQ();
  
  //structural operations
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
//  Vector force=0, torque=0; ///< current forces applying on the body
  
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
  ptr<Mesh> _mesh;
  ptr<Mesh> _sscCore;
  Enum<ShapeType> _type;
  arr size;

  void setMeshMimic(const Frame* f);
  double radius() { if(size.N) return size(-1); return 0.; }
  Enum<ShapeType>& type() { return _type; }
  Mesh& mesh() { if(!_mesh) _mesh = make_shared<Mesh>();  return *_mesh; }
  Mesh& sscCore() { if(!_sscCore) _sscCore = make_shared<Mesh>();  return *_sscCore; }
  double alpha() { arr& C=mesh().C; if(C.N==4) return C(3); return 1.; }

  void createMeshes();

  char cont=0;           ///< are contacts registered (or filtered in the callback)
  bool visual=true;
  
  Shape(Frame& f, const Shape *copyShape=NULL); //new Shape, being added to graph and frame's shape lists
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

