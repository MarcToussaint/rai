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


#ifndef MLR_ors_h
#define MLR_ors_h

#include <Core/util.h>
#include <Core/array.h>
#include <Core/graph.h>
#include <Geo/geo.h>
#include <Geo/mesh.h>

#include "frame.h"

/// @file
/// @ingroup group_ors

struct OpenGL;
struct PhysXInterface;
struct SwiftInterface;
struct OdeInterface;

/// @addtogroup group_ors
/// @{

//===========================================================================


namespace mlr{

/// @addtogroup ors_basic_data_structures
/// @{
//enum BodyType  { BT_none=-1, BT_dynamic=0, BT_kinematic, BT_static };
/// @}

struct Joint;
struct Shape;
struct Frame;
struct KinematicWorld;
struct Proxy;
struct KinematicSwitch;

} // END of namespace

//===========================================================================

typedef mlr::Array<mlr::Joint*> JointL;
typedef mlr::Array<mlr::Shape*> ShapeL;
typedef mlr::Array<mlr::Frame*>  BodyL;
typedef mlr::Array<mlr::Proxy*> ProxyL;
typedef mlr::Array<mlr::KinematicSwitch*> KinematicSwitchL;
typedef mlr::Array<mlr::KinematicWorld*> WorldL;

//===========================================================================

namespace mlr {
/// @addtogroup ors_basic_data_structures
/// @{

//===========================================================================




//===========================================================================

/// a data structure to store proximity information (when two shapes become close) --
/// as return value from external collision libs
struct Proxy : GLDrawer {
  //TODO: have a ProxyL& L as above...
  int a;              ///< index of shape A (-1==world) //TODO: would it be easier if this were mlr::Shape* ? YES -> Do it!
  int b;              ///< index of shape B
  Vector posA, cenA;  ///< contact or closest point position on surface of shape A (in world coordinates)
  Vector posB, cenB;  ///< contact or closest point position on surface of shape B (in world coordinates)
  Vector normal, cenN;   ///< contact normal, pointing from B to A (proportional to posA-posB)
  double d, cenD;           ///< distance (positive) or penetration (negative) between A and B
  uint colorCode;
  Proxy();
  void glDraw(OpenGL&);
};

//===========================================================================

/// data structure to store a whole physical situation (lists of bodies, joints, shapes, proxies)
struct KinematicWorld : GLDrawer{
  struct sKinematicWorld *s;

  /// @name data fields
  uint qdim;  ///< dimensionality
  arr q, qdot; ///< the current joint configuration vector and velocities

  FrameL frames;
  FrameL fwdActiveSet;

  ProxyL proxies; ///< list of current proximities between bodies

  bool isLinkTree;
  static uint setJointStateCount;

  //global options
  bool orsDrawJoints=true, orsDrawShapes=true, orsDrawBodies=true, orsDrawProxies=true, orsDrawMarkers=true, orsDrawColors=true, orsDrawIndexColors=false;
  bool orsDrawMeshes=true, orsDrawCores=false, orsDrawZlines=false;
  bool orsDrawBodyNames=false;
  double orsDrawAlpha=1.;
  uint orsDrawLimit=0;
  
  /// @name constructors
  KinematicWorld();
  KinematicWorld(const mlr::KinematicWorld& other);
  KinematicWorld(const char* filename);
  virtual ~KinematicWorld();
  void operator=(const mlr::KinematicWorld& G){ copy(G); }
  void copy(const mlr::KinematicWorld& G, bool referenceMeshesAndSwiftOnCopy=false);
  
  /// @name initializations
  void init(const char* filename);
  void init(const Graph& G);

  /// @name access
  Frame *getFrameByName(const char* name, bool warnIfNotExist=true) const;
  Joint *getJointByBodies(const Frame* from, const Frame* to) const;
  Joint *getJointByBodyNames(const char* from, const char* to) const;
  Joint *getJointByBodyIndices(uint ifrom, uint ito) const;

  bool checkUniqueNames() const;
  void prefixNames();

  /// @name changes of configuration
  void clear();
  void makeTree(Frame *root){ reconfigureRoot(root); makeLinkTree(); }
  //-- low level: don't use..
  void revertJoint(Joint *e);
  void reconfigureRoot(Frame *root);  ///< n becomes the root of the kinematic tree; joints accordingly reversed; lists resorted
  void transformJoint(Joint *e, const mlr::Transformation &f); ///< A <- A*f, B <- f^{-1}*B
  void makeLinkTree();            ///< modify transformations so that B's become identity
  void jointSort();
  void glueBodies(Frame *a, Frame *b);
  void meldFixedJoints(int verbose=0);         ///< prune fixed joints; shapes of fixed bodies are reassociated to non-fixed boides
  void removeUselessBodies(int verbose=0);     ///< prune non-articulated bodies; they become shapes of other bodies
  bool checkConsistency();
  void analyzeJointStateDimensions(); ///< sort of private: count the joint dimensionalities and assign j->q_index

  /// @name computations on the graph
  void calc_Q_from_q(); ///< from the set (q,qdot) compute the joint's Q transformations
  void calc_q_from_Q();  ///< updates (q,qdot) based on the joint's Q transformations
  void calc_fwdPropagateFrames();    ///< elementary forward kinematics; also computes all Shape frames
  arr calc_fwdPropagateVelocities();    ///< elementary forward kinematics; also computes all Shape frames
  void calc_Q_from_BodyFrames();    ///< fill in the joint transformations assuming that body poses are known (makes sense when reading files)
//  void calc_missingAB_from_BodyAndJointFrames();    ///< fill in the missing joint relative transforms (A & B) if body and joint world poses are known


  /// @name get state
  uint getJointStateDimension() const;
  void getJointState(arr &_q, arr& _qdot=NoArr) const;
  arr getJointState() const;
  arr naturalQmetric(double power=.5) const;               ///< returns diagonal of a natural metric in q-space, depending on tree depth
  arr getLimits() const;

  /// @name set state
  void setJointState(const arr& _q, const arr& _qdot=NoArr);

  /// @name kinematics
  void kinematicsPos (arr& y, arr& J, Frame *b, const Vector& rel=NoVector) const; //TODO: make vector& not vector*
  void kinematicsVec (arr& y, arr& J, Frame *b, const mlr::Vector& vec=NoVector) const;
  void kinematicsQuat(arr& y, arr& J, Frame *b) const;
  void hessianPos(arr& H, Frame *b, mlr::Vector *rel=0) const;
  void axesMatrix(arr& J, Frame *b) const;
  void kinematicsRelPos (arr& y, arr& J, Frame *b1, const mlr::Vector& vec1, Frame *b2, const mlr::Vector& vec2) const;
  void kinematicsRelVec (arr& y, arr& J, Frame *b1, const mlr::Vector& vec1, Frame *b2) const;
  void kinematicsRelRot (arr& y, arr& J, Frame *b1, Frame *b2) const;

  void kinematicsProxyDist(arr& y, arr& J, Proxy *p, double margin=.02, bool useCenterDist=true, bool addValues=false) const;
  void kinematicsProxyCost(arr& y, arr& J, Proxy *p, double margin=.02, bool useCenterDist=true, bool addValues=false) const;
  void kinematicsProxyCost(arr& y, arr& J, double margin=.02, bool useCenterDist=true) const;
  void kinematicsProxyConstraint(arr& g, arr& J, Proxy *p, double margin=.02) const;
  void kinematicsContactConstraints(arr& y, arr &J) const; //TODO: deprecated?
  void kinematicsPos_wrtFrame(arr& y, arr& J, Frame *b, const mlr::Vector& rel, Shape *s) const;
  void getLimitsMeasure(arr &x, const arr& limits, double margin=.1) const;
  void kinematicsLimitsCost(arr& y, arr& J, const arr& limits, double margin=.1) const;

  /// @name High level (inverse) kinematics
  void inverseKinematicsPos(Frame& body, const arr& ytarget, const mlr::Vector& rel_offset=NoVector, int max_iter=3);

  /// @name dynamics
  void fwdDynamics(arr& qdd, const arr& qd, const arr& tau);
  void inverseDynamics(arr& tau, const arr& qd, const arr& qdd);
  void equationOfMotion(arr& M, arr& F, bool gravity=true);
  void inertia(arr& M);

  /// @name older 'kinematic maps'
  double getCenterOfMass(arr& com) const;
  void getComGradient(arr &grad) const;

  double getEnergy();
  mlr::Proxy* getContact(uint a, uint b) const;

  /// @name get infos
  arr getHmetric() const;

  /// @name forces and gravity
  void clearForces();
  void addForce(mlr::Vector force, Frame *n);
  void addForce(mlr::Vector force, Frame *n, mlr::Vector pos);
  void contactsToForces(double hook=.01, double damp=.0003);
  void gravityToForces();
  void frictionToForces(double coeff);
  
  /// @name extensions on demand
  OpenGL& gl(const char* window_title=NULL);
  SwiftInterface& swift();
  void swiftDelete();
  PhysXInterface& physx();
  OdeInterface& ode();
  void watch(bool pause=false, const char* txt=NULL);
  void glAnimate();
  void glGetMasks(int w=-1, int h=-1, bool rgbIndices=true);
  void stepSwift();
  void stepPhysx(double tau);
  void stepOde(double tau);
  void stepDynamics(const arr& u_control, double tau, double dynamicNoise = 0.0, bool gravity = true);

  /// @name I/O
  void write(std::ostream& os) const;
  void read(std::istream& is);
  void glDraw(struct OpenGL&);

  void reportProxies(std::ostream& os=std::cout, double belowMargin=-1., bool brief=true) const;
  void writePlyFile(const char* filename) const; //TODO: move outside
};

//===========================================================================

struct KinematicSwitch{ //TODO: move to src/Motion
  enum OperatorSymbol{ none=-1, deleteJoint=0, addJointZero, addJointAtFrom, addJointAtTo, addActuated, addSliderMechanism };
  Enum<OperatorSymbol> symbol;
  Enum<JointType> jointType;
  uint timeOfApplication;
  uint fromId, toId;
  mlr::Transformation jA,jB;
  KinematicSwitch();
  KinematicSwitch(OperatorSymbol op, JointType type,
                  const char* ref1, const char* ref2,
                  const mlr::KinematicWorld& K, uint _timeOfApplication,
                  const mlr::Transformation& jFrom=NoTransformation, const mlr::Transformation& jTo=NoTransformation);
  void setTimeOfApplication(double time, bool before, int stepsPerPhase, uint T);
//  KinematicSwitch(const Node *specs, const KinematicWorld& world, uint T);
  void apply(KinematicWorld& G);
  void temporallyAlign(const KinematicWorld& Gprevious, KinematicWorld& G, bool copyFromBodies);
  mlr::String shortTag(const KinematicWorld* G) const;
  void write(std::ostream& os) const;
  static KinematicSwitch* newSwitch(const Node *specs, const mlr::KinematicWorld& world, int stepsPerPhase, uint T);
  static KinematicSwitch* newSwitch(const mlr::String& type, const char* ref1, const char* ref2, const mlr::KinematicWorld& world, uint _timeOfApplication, const mlr::Transformation& jFrom=NoTransformation, const mlr::Transformation& jTo=NoTransformation);
  static const char* name(OperatorSymbol s);
};
/// @} // END of group ors_basic_data_structures
} // END ors namespace
stdOutPipe(mlr::KinematicSwitch)

//===========================================================================
//
// constants
//

extern mlr::Frame& NoBody;
extern mlr::Shape& NoShape;
extern mlr::Joint& NoJoint;
extern mlr::KinematicWorld& NoWorld;


//===========================================================================
//
// operators
//

namespace mlr {
//std::istream& operator>>(std::istream&, Body&);
//std::istream& operator>>(std::istream&, Joint&);
//std::istream& operator>>(std::istream&, Shape&);
std::ostream& operator<<(std::ostream&, const Frame&);
std::ostream& operator<<(std::ostream&, const Joint&);
std::ostream& operator<<(std::ostream&, const Shape&);
stdPipes(KinematicWorld);
}


//===========================================================================
//
// OpenGL static draw functions
//

namespace mlr {
void glDrawGraph(void*);
void glDrawProxies(void*);

}

#ifndef MLR_ORS_ONLY_BASICS

uintA stringListToShapeIndices(const mlr::Array<const char*>& names, const ShapeL& shapes);
uintA shapesToShapeIndices(const FrameL &shapes);

//===========================================================================
//
// C-style functions
//

void lib_ors();
void makeConvexHulls(FrameL& frames, bool onlyContactShapes=true);
void computeOptimalSSBoxes(FrameL& frames);
void computeMeshNormals(FrameL& frames);
double forceClosureFromProxies(mlr::KinematicWorld& C, uint bodyIndex,
                               double distanceThreshold=0.01,
                               double mu=.5,     //friction coefficient
                               double discountTorques=1.);  //friction coefficient

void transferQbetweenTwoWorlds(arr& qto, const arr& qfrom, const mlr::KinematicWorld& to, const mlr::KinematicWorld& from);
void transferQDotbetweenTwoWorlds(arr& qDotTo, const arr& qDotFrom, const mlr::KinematicWorld& to, const mlr::KinematicWorld& from);
void transferKpBetweenTwoWorlds(arr& KpTo, const arr& KpFrom, const mlr::KinematicWorld& to, const mlr::KinematicWorld& from);
void transferKdBetweenTwoWorlds(arr& KdTo, const arr& KdFrom, const mlr::KinematicWorld& to, const mlr::KinematicWorld& from);
void transferU0BetweenTwoWorlds(arr& u0To, const arr& u0From, const mlr::KinematicWorld& to, const mlr::KinematicWorld& from);
void transferKI_ft_BetweenTwoWorlds(arr& KI_ft_To, const arr& KI_ft_From, const mlr::KinematicWorld& to, const mlr::KinematicWorld& from);

//===========================================================================
// routines using external interfaces.
//===========================================================================
/// @addtogroup ors_interfaces
/// @{
//===========================================================================
/// @defgroup ors_interface_opengl Interface to OpenGL.
/// @{
// OPENGL interface
struct OpenGL;

void displayState(const arr& x, mlr::KinematicWorld& G, const char *tag);
void displayTrajectory(const arr& x, int steps, mlr::KinematicWorld& G, const KinematicSwitchL& switches, const char *tag, double delay=0., uint dim_z=0, bool copyG=false);
inline void displayTrajectory(const arr& x, int steps, mlr::KinematicWorld& G, const char *tag, double delay=0., uint dim_z=0, bool copyG=false){
  displayTrajectory(x, steps, G, {}, tag, delay, dim_z, copyG);
}
void editConfiguration(const char* orsfile, mlr::KinematicWorld& G);
int animateConfiguration(mlr::KinematicWorld& G, struct Inotify *ino=NULL);
//void init(mlr::KinematicWorld& G, OpenGL& gl, const char* orsFile);
void bindOrsToOpenGL(mlr::KinematicWorld& graph, OpenGL& gl); //TODO: should be outdated!
/// @} // END of group ors_interface_opengl


//===========================================================================
/// @defgroup ors_interface_featherstone FEATHERSTONE Interface.
/// @todo is all the following stuff really featherstone? MT: yes
/// @{
namespace mlr {
struct Link {
  int type;
  int qIndex;
  int parent;
  mlr::Transformation
  X, A, Q;
  mlr::Vector com, force, torque;
  double mass;
  mlr::Matrix inertia;
  uint dof() { if(type>=JT_hingeX && type<=JT_transZ) return 1; else return 0; }
  
  arr _h, _A, _Q, _I, _f; //featherstone types
  void setFeatherstones();
  void updateFeatherstones();
  void write(ostream& os) const {
    os <<"*type=" <<type <<" index=" <<qIndex <<" parent=" <<parent <<endl
       <<" XAQ=" <<X <<A <<Q <<endl
       <<" cft=" <<com <<force <<torque <<endl
       <<" mass=" <<mass <<inertia <<endl;
  }
};

typedef mlr::Array<mlr::Link> LinkTree;

void equationOfMotion(arr& M, arr& F, const LinkTree& tree,  const arr& qd);
void fwdDynamics_MF(arr& qdd, const LinkTree& tree, const arr& qd, const arr& u);
void fwdDynamics_aba_nD(arr& qdd, const LinkTree& tree, const arr& qd, const arr& tau);
void fwdDynamics_aba_1D(arr& qdd, const LinkTree& tree, const arr& qd, const arr& tau);
void invDynamics(arr& tau, const LinkTree& tree, const arr& qd, const arr& qdd);

}
stdOutPipe(mlr::Link);

void GraphToTree(mlr::LinkTree& tree, const mlr::KinematicWorld& C);
void updateGraphToTree(mlr::LinkTree& tree, const mlr::KinematicWorld& C);
/// @}


//===========================================================================
/// @defgroup ors_interface_blender Blender interface.
/// @{
inline void readBlender(const char* filename, mlr::Mesh& mesh, mlr::KinematicWorld& bl){ NICO }
/// @}

/// @} // END of group ors_interfaces
//===========================================================================
#endif //MLR_ORS_ONLY_BASICS

/// @}

#endif //MLR_ors_h
