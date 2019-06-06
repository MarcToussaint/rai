/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef RAI_kin_h
#define RAI_kin_h

#include <Core/array.h>
#include <Geo/geo.h>
#include <Geo/geoms.h>
#include "featureSymbols.h"

struct OpenGL;
struct PhysXInterface;
struct SwiftInterface;
struct OdeInterface;
struct FeatherstoneInterface;

//===========================================================================

namespace rai {

struct Joint;
struct Shape;
struct Frame;
struct Proxy;
struct Contact;
struct KinematicWorld;
struct KinematicSwitch;

struct FclInterface;

} // namespace rai

//===========================================================================

extern rai::KinematicWorld& NoWorld;

typedef rai::Array<rai::Joint*> JointL;
//typedef rai::Array<rai::Shape*> ShapeL;
typedef rai::Array<rai::Frame*> FrameL;
typedef rai::Array<rai::Proxy*> ProxyL;
typedef rai::Array<rai::Proxy> ProxyA;
typedef rai::Array<rai::Contact*> ContactL;
typedef rai::Array<rai::KinematicSwitch*> KinematicSwitchL;
typedef rai::Array<rai::KinematicWorld*> WorldL;

//===========================================================================

namespace rai {

/// data structure to store a whole physical situation (lists of bodies, joints, shapes, proxies)
struct KinematicWorld : GLDrawer {
  struct sKinematicWorld *s;
  
  //-- fundamental structure
  FrameL frames;
  
  //-- derived: computed with calc_q(); reset with reset_q()
  arr q, qdot; ///< the current joint configuration vector and velocities
  FrameL fwdActiveSet;
  JointL fwdActiveJoints;
  ContactL contacts;
  
  ProxyA proxies; ///< list of current proximities between bodies
  
  static uint setJointStateCount;
  
  //global options -> TODO: refactor away from here
  bool orsDrawJoints=false, orsDrawShapes=true, orsDrawBodies=true, orsDrawProxies=true, orsDrawMarkers=true, orsDrawColors=true, orsDrawIndexColors=false;
  bool orsDrawVisualsOnly=false, orsDrawMeshes=true, orsDrawCores=false, orsDrawZlines=false;
  bool orsDrawBodyNames=false;
  double orsDrawAlpha=1.;
  uint orsDrawLimit=0;
  
  /// @name constructors
  KinematicWorld();
  KinematicWorld(const rai::KinematicWorld& other, bool referenceSwiftOnCopy=false);
  KinematicWorld(const char* filename);
  virtual ~KinematicWorld();
  void operator=(const rai::KinematicWorld& K) { copy(K); }
  void copy(const rai::KinematicWorld& K, bool referenceSwiftOnCopy=false);
  bool operator!() const;

  /// @name initializations
  void init(const char* filename);
  void init(const Graph& G, bool addInsteadOfClear=false);
  Frame* addFile(const char* filename);
  Frame* addFile(const char* filename, const char* parentOfRoot, const rai::Transformation& relOfRoot);
  void addAssimp(const char* filename);
  Frame* addFrame(const char* name, const char* parent=NULL, const char* args=NULL);
//  Frame* addObject(rai::ShapeType shape, const arr& size={}, const arr& col={});
  Frame* addObject(const char* name, const char* parent, rai::ShapeType shape, const arr& size={}, const arr& col={}, const arr& pos={}, const arr& rot={});
  void addFramesCopy(const FrameL& F);

  /// @name access
  Frame *operator[](const char* name) { return getFrameByName(name, true); }
  Frame *operator()(int i) { return frames(i); }
  Frame *getFrameByName(const char* name, bool warnIfNotExist=true, bool reverse=false) const;
  FrameL getFramesByNames(const StringA& frameNames) const;
//  Link  *getLinkByBodies(const Frame* from, const Frame* to) const;
  Joint *getJointByBodies(const Frame* from, const Frame* to) const;
  Joint *getJointByBodyNames(const char* from, const char* to) const;
  Joint *getJointByBodyIndices(uint ifrom, uint ito) const;
  uintA getQindicesByNames(const StringA& jointNames) const;
  StringA getJointNames() const;
  StringA getFrameNames() const;

  bool checkUniqueNames() const;
  void prefixNames(bool clear=false);
  
  /// @name structural operations, changes of configuration
  void clear();
  void reset_q();
  FrameL calc_topSort() const;
  bool check_topSort() const;
  void calc_activeSets();
  void calc_q();
  void reconfigureRootOfSubtree(Frame *root);  ///< n becomes the root of the kinematic tree; joints accordingly reversed; lists resorted
  void flipFrames(rai::Frame *a, rai::Frame *b);
  void pruneRigidJoints(int verbose=0);        ///< delete rigid joints -> they become just links
  void reconnectLinksToClosestJoints();        ///< re-connect all links to closest joint
  void pruneUselessFrames(bool pruneNamed=false, bool pruneNonContactNonMarker=false);  ///< delete frames that have no name, joint, and shape
  void optimizeTree(bool _pruneRigidJoints=false, bool pruneNamed=false, bool pruneNonContactNonMarker=false);        ///< call the three above methods in this order
  void sortFrames();
  void makeObjectsFree(const StringA& objects, double H_cost=0.);
  void addTimeJoint();
  bool hasTimeJoint();
  bool checkConsistency() const;
  Joint* attach(Frame* a, Frame* b);
  Joint* attach(const char *a, const char *b);
  FrameL getParts() const;

  uint analyzeJointStateDimensions() const; ///< sort of private: count the joint dimensionalities and assign j->q_index
  
  /// @name computations on the graph
  void calc_Q_from_q(); ///< from the set (q,qdot) compute the joint's Q transformations
  void calc_q_from_Q();  ///< updates (q,qdot) based on the joint's Q transformations
  void calc_fwdPropagateFrames();    ///< elementary forward kinematics; also computes all Shape frames
  arr calc_fwdPropagateVelocities();    ///< elementary forward kinematics; also computes all Shape frames
  void calc_Q_from_BodyFrames();    ///< fill in the joint transformations assuming that body poses are known (makes sense when reading files)
  
  /// @name get state
  uint getJointStateDimension() const;
  void getJointState(arr &_q, arr& _qdot=NoArr) const;
  arr getJointState() const;
  arr getJointState(const StringA&) const;
  arr getJointState(const uintA&) const;
  arr getFrameState() const;
  arr naturalQmetric(double power=.5) const;               ///< returns diagonal of a natural metric in q-space, depending on tree depth
  arr getLimits() const;
  
  /// @name active set selection
  void selectJointsByGroup(const StringA& groupNames, bool OnlyTheseOrNotThese=true, bool deleteInsteadOfLock=true);
  void selectJointsByName(const StringA&, bool notThose=false);
  
  /// @name set state
  void setJointState(const arr& _q, const arr& _qdot=NoArr);
  void setJointState(const arr& _q, const StringA&);
  void setJointState(const arr& _q, const uintA&);
  void setFrameState(const arr& X, const StringA& frameNames={}, bool calc_q_from_X=true, bool warnOnDifferentDim=true);
  void setTimes(double t);
  void operator=(const arr& X){
    if(X.d0==frames.N) setFrameState(X);
    else if(X.d0==getJointStateDimension()) setJointState(X);
    else HALT("wrong dimension");
  }

  /// @name features
  void evalFeature(arr& y, arr& J, FeatureSymbol fs, const StringA &symbols) const;

  /// @name kinematics (low level)
  void kinematicsPos(arr& y, arr& J, Frame *a, const Vector& rel=NoVector) const;  //TODO: make vector& not vector*
  void kinematicsVec(arr& y, arr& J, Frame *a, const Vector& vec=NoVector) const;
  void kinematicsQuat(arr& y, arr& J, Frame *a) const;
  void hessianPos(arr& H, Frame *a, Vector *rel=0) const;
  void jacobianPos(arr& J, Frame *a, const rai::Vector& pos_world) const; //usually called internally with kinematicsPos
  void kinematicsTau(double& tau, arr& J) const;
  void jacobianTime(arr& J, Frame*a) const;
  void axesMatrix(arr& J, Frame *a) const; //usually called internally with kinematicsVec or Quat
  void kinematicsRelPos(arr& y, arr& J, Frame *a, const Vector& vec1, Frame *b, const Vector& vec2) const;
  void kinematicsRelVec(arr& y, arr& J, Frame *a, const Vector& vec1, Frame *b) const;
  void kinematicsRelRot(arr& y, arr& J, Frame *a, Frame *b) const;

  void kinematicsContactPOA(arr& y, arr& J, Contact *c) const;
  void kinematicsContactForce(arr& y, arr& J, Contact *c) const;
  
  void kinematicsPenetrations(arr& y, arr& J=NoArr, bool penetrationsOnly=true, double activeMargin=0.) const; ///< true: if proxy(i).distance>0. => y(i)=0; else y(i)=-proxy(i).distance
  void kinematicsProxyDist(arr& y, arr& J, const Proxy& p, double margin=.02, bool useCenterDist=true, bool addValues=false) const;
  void kinematicsProxyCost(arr& y, arr& J, const Proxy& p, double margin=.0, bool addValues=false) const;
  void kinematicsProxyCost(arr& y, arr& J, double margin=.0) const;
  void kinematicsContactCost(arr& y, arr& J, const Contact *p, double margin=.0, bool addValues=false) const;
  void kinematicsContactCost(arr& y, arr& J, double margin=.0) const;
  void kinematicsProxyConstraint(arr& g, arr& J, const Proxy& p, double margin=.02) const;
  void kinematicsContactConstraints(arr& y, arr &J) const; //TODO: deprecated?
  void kinematicsPos_wrtFrame(arr& y, arr& J, Frame *b, const rai::Vector& rel, Frame *s) const;
  void getLimitsMeasure(arr &x, const arr& limits, double margin=.1) const;
  void kinematicsLimitsCost(arr& y, arr& J, const arr& limits, double margin=.1) const;
  
  /// @name active set selection
  void setAgent(uint) { NIY }
  
  /// @name High level (inverse) kinematics
  void inverseKinematicsPos(Frame& body, const arr& ytarget, const rai::Vector& rel_offset=NoVector, int max_iter=3);
  
  /// @name dynamics
  void fwdDynamics(arr& qdd, const arr& qd, const arr& tau, bool gravity=true);
  void inverseDynamics(arr& tau, const arr& qd, const arr& qdd, bool gravity=true);
  void equationOfMotion(arr& M, arr& F, bool gravity=true);
  void inertia(arr& M);
  double getEnergy();
  
  /// @name get infos
  arr getHmetric() const;
  
  /// @name forces and gravity
  void clearForces();
  void addForce(rai::Vector force, Frame *n);
  void addForce(rai::Vector force, Frame *n, rai::Vector pos);
  void contactsToForces(double hook=.01, double damp=.0003);
  void gravityToForces(double g=-9.81);
  void frictionToForces(double coeff);
  void NewtonEuler_backward();
  
  /// @name extensions on demand
  OpenGL& gl(const char* window_title=NULL);
  SwiftInterface& swift();
  FclInterface& fcl();
  void swiftDelete();
  PhysXInterface& physx();
  OdeInterface& ode();
  FeatherstoneInterface& fs();
  int watch(bool pause=false, const char* txt=NULL);
  void saveVideoPic(uint& t, const char* pathPrefix="vid/");
  void glAdd(void (*call)(void*, OpenGL&), void* classP);
  int glAnimate();
  void glClose();
  void glGetMasks(int w=-1, int h=-1, bool rgbIndices=true);
  void stepSwift();
  void stepFcl();
  void stepPhysx(double tau);
  void stepOde(double tau);
  void stepDynamics(const arr& u_control, double tau, double dynamicNoise = 0.0, bool gravity = true);
  
  /// @name contacts
  void filterProxiesToContacts(double margin=.01); ///< proxies are returns from a collision engine; contacts stable constraints
  void proxiesToContacts(double margin=.01); ///< proxies are returns from a collision engine; contacts stable constraints
  double totalContactPenetration(); ///< proxies are returns from a collision engine; contacts stable constraints
  void copyProxies(const KinematicWorld& K);

  /// @name I/O
  void write(std::ostream& os) const;
  void write(Graph& G) const;
  void writeURDF(std::ostream& os, const char *robotName="myrobot") const;
  void writeMeshes(const char* pathPrefix="meshes/") const;
  void read(std::istream& is);
  void glDraw(struct OpenGL&);
  void glDraw_sub(struct OpenGL&);
  Graph getGraph() const;
  Array<Frame*> getLinks() const;
  void displayDot();
  
  //some info
  void report(std::ostream& os=std::cout) const;
  void reportProxies(std::ostream& os=std::cout, double belowMargin=1., bool brief=true) const;
  void writePlyFile(const char* filename) const; //TODO: move outside
  
  friend struct KinematicSwitch;
};

} //namespace rai

stdPipes(rai::KinematicWorld)

//===========================================================================
//
// OpenGL static draw functions
//

namespace rai {
void glDrawGraph(void*, OpenGL& gl);
}

uintA stringListToShapeIndices(const rai::Array<const char*>& names, const FrameL& shapes);
uintA shapesToShapeIndices(const FrameL &shapes);

//===========================================================================
//
// C-style functions
//

void lib_ors();
void makeConvexHulls(FrameL& frames, bool onlyContactShapes=true);
void computeOptimalSSBoxes(FrameL& frames);
void computeMeshNormals(FrameL& frames, bool force=false);
double forceClosureFromProxies(rai::KinematicWorld& C, uint bodyIndex,
                               double distanceThreshold=0.01,
                               double mu=.5,     //friction coefficient
                               double discountTorques=1.);  //friction coefficient

void transferQbetweenTwoWorlds(arr& qto, const arr& qfrom, const rai::KinematicWorld& to, const rai::KinematicWorld& from);
void transferQDotbetweenTwoWorlds(arr& qDotTo, const arr& qDotFrom, const rai::KinematicWorld& to, const rai::KinematicWorld& from);
void transferKpBetweenTwoWorlds(arr& KpTo, const arr& KpFrom, const rai::KinematicWorld& to, const rai::KinematicWorld& from);
void transferKdBetweenTwoWorlds(arr& KdTo, const arr& KdFrom, const rai::KinematicWorld& to, const rai::KinematicWorld& from);
void transferU0BetweenTwoWorlds(arr& u0To, const arr& u0From, const rai::KinematicWorld& to, const rai::KinematicWorld& from);
void transferKI_ft_BetweenTwoWorlds(arr& KI_ft_To, const arr& KI_ft_From, const rai::KinematicWorld& to, const rai::KinematicWorld& from);

void displayState(const arr& x, rai::KinematicWorld& G, const char *tag);
void displayTrajectory(const arr& x, int steps, rai::KinematicWorld& G, const KinematicSwitchL& switches, const char *tag, double delay=0., uint dim_z=0, bool copyG=false);
inline void displayTrajectory(const arr& x, int steps, rai::KinematicWorld& G, const char *tag, double delay=0., uint dim_z=0, bool copyG=false) {
  displayTrajectory(x, steps, G, {}, tag, delay, dim_z, copyG);
}
void editConfiguration(const char* orsfile, rai::KinematicWorld& G);
int animateConfiguration(rai::KinematicWorld& G, struct Inotify *ino=NULL);

void kinVelocity(arr& y, arr& J, uint frameId, const WorldL& Ktuple, double tau);
void kinAngVelocity(arr& y, arr& J, uint frameId, const WorldL& Ktuple, double tau);

#endif //RAI_ors_h
