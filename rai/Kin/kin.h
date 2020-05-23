/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "featureSymbols.h"
#include "../Core/array.h"
#include "../Geo/geo.h"
#include "../Geo/geoms.h"

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
struct ForceExchange;
struct Configuration;
struct KinematicSwitch;

struct FclInterface;
struct ConfigurationViewer;

} // namespace rai

//===========================================================================

extern rai::Configuration& NoWorld;

typedef rai::Array<rai::Joint*> JointL;
//typedef rai::Array<rai::Shape*> ShapeL;
typedef rai::Array<rai::Frame*> FrameL;
typedef rai::Array<rai::Proxy*> ProxyL;
typedef rai::Array<rai::Proxy> ProxyA;
typedef rai::Array<rai::ForceExchange*> ForceExchangeL;
typedef rai::Array<rai::KinematicSwitch*> KinematicSwitchL;
typedef rai::Array<rai::Configuration*> ConfigurationL;

//===========================================================================

namespace rai {

/// data structure to store a whole physical situation (lists of bodies, joints, shapes, proxies)
struct Configuration : GLDrawer {
  unique_ptr<struct sConfiguration> self;

  //-- fundamental structure
  FrameL frames;     ///< list of coordinate frames, with shapes, joints, inertias attached
  ForceExchangeL forces; ///< list of force exchanges between frames
  ProxyA proxies;    ///< list of current collision proximities between frames
  arr q;             ///< the current configuration state (DOF) vector

  //-- derived: computed with ensure_activeSets(); reset with reset_q()
  JointL activeJoints;

  //-- data structure state (lazy evaluation leave the state structure out of sync)
  bool _state_indexedJoints_areGood=false; // the active sets, incl. their topological sorting, are up to date
  bool _state_q_isGood=false; // the q-vector represents the current relative transforms (and force dofs)
  bool _state_proxies_isGood=false; // the proxies have been created for the current state
  //TODO: need a _state for all the plugin engines (SWIFT, PhysX)? To auto-reinitialize them when the config changed structurally?

  bool useSparseJacobians=false;
  int xIndex=0;   // the start-index of this configuration in a larger decision variable x (e.g. if x is a path of configurations) (analogous to qIndex of a joint)

  static uint setJointStateCount;

  //global options -> TODO: refactor away from here
  bool orsDrawJoints=false, orsDrawShapes=true, orsDrawBodies=true, orsDrawProxies=true, orsDrawMarkers=true, orsDrawColors=true, orsDrawIndexColors=false;
  bool orsDrawVisualsOnly=false, orsDrawMeshes=true, orsDrawCores=false, orsDrawZlines=false;
  bool orsDrawFrameNames=false;
  double orsDrawAlpha=1.;
  uint orsDrawLimit=0;

  /// @name constructors
  Configuration();
  Configuration(const rai::Configuration& other, bool referenceSwiftOnCopy=false);
  Configuration(const char* filename);
  virtual ~Configuration();

  /// @name copy
  void operator=(const rai::Configuration& K) { copy(K); }
  void copy(const rai::Configuration& K, bool referenceSwiftOnCopy=false);
  bool operator!() const;

  /// @name initializations, building configurations
  void init(const char* filename);
  void init(const Graph& G, bool addInsteadOfClear=false);
  Frame* addFile(const char* filename);
  Frame* addFile(const char* filename, const char* parentOfRoot, const rai::Transformation& relOfRoot);
  void addAssimp(const char* filename);
  Frame* addFrame(const char* name, const char* parent=nullptr, const char* args=nullptr);
  Frame* addObject(const char* name, const char* parent, rai::ShapeType shape, const arr& size= {}, const arr& col= {}, const arr& pos= {}, const arr& rot= {}, bool isSubFrame=false);
  void addFramesCopy(const FrameL& F);

  /// @name access
  Frame* operator[](const char* name) { return getFrameByName(name, true); }
  Frame* operator()(int i) const { return frames(i); }
  Frame* getFrameByName(const char* name, bool warnIfNotExist=true, bool reverse=false) const;
  FrameL getFramesByNames(const StringA& frameNames) const;
  Joint* getJointByFrames(const Frame* from, const Frame* to) const;
  Joint* getJointByFrameNames(const char* from, const char* to) const;
  Joint* getJointByFrameIndices(uint ifrom, uint ito) const;
  uintA getQindicesByNames(const StringA& jointNames) const;
  StringA getJointNames() const;
  StringA getFrameNames() const;
  uintA getNormalJointFramesAndScale(arr& scale=NoArr) const;

  bool checkUniqueNames() const;
  void prefixNames(bool clear=false);

  /// @name structural operations, changes of configuration
  void clear();
  void reset_q();
  FrameL calc_topSort() const;
  bool check_topSort() const;

  void reconfigureRoot(Frame* newRoot, bool ofLinkOnly);  ///< n becomes the root of the kinematic tree; joints accordingly reversed; lists resorted
  void flipFrames(rai::Frame* a, rai::Frame* b);
  void pruneRigidJoints(int verbose=0);        ///< delete rigid joints -> they become just links
  void reconnectLinksToClosestJoints();        ///< re-connect all links to closest joint
  void pruneUselessFrames(bool pruneNamed=false, bool pruneNonContactNonMarker=false);  ///< delete frames that have no name, joint, and shape
  void optimizeTree(bool _pruneRigidJoints=false, bool pruneNamed=false, bool pruneNonContactNonMarker=false);        ///< call the three above methods in this order
  void sortFrames();
  void makeObjectsFree(const StringA& objects, double H_cost=0.);
  void addTauJoint();
  bool hasTauJoint();
  bool checkConsistency() const;
  Joint* attach(Frame* a, Frame* b);
  Joint* attach(const char* a, const char* b);
  FrameL getParts() const;
  uintA getCollisionExcludePairIDs(bool verbose=false);

  /// @name computations on the tree
  void calc_indexedActiveJoints(); ///< sort of private: count the joint dimensionalities and assign j->q_index
  void calc_Q_from_q();  ///< from q compute the joint's Q transformations
  void calc_q_from_Q();  ///< updates q based on the joint's Q transformations
  arr calc_fwdPropagateVelocities(const arr& qdot);    ///< elementary forward kinematics

  /// @name ensure state consistencies
  void ensure_indexedJoints() {   if(!_state_indexedJoints_areGood) calc_indexedActiveJoints();  }
  void ensure_q() {  if(!_state_q_isGood) calc_q_from_Q();  }
  void ensure_proxies() {  if(!_state_proxies_isGood) stepSwift();  }

  /// @name get state
  uint getJointStateDimension() const;
  const arr& getJointState() const;
  arr getJointState(const FrameL&) const;
  arr getJointState(const uintA&) const;
  arr getJointState(const StringA&) const;
  arr getFrameState() const;
  arr naturalQmetric(double power=.5) const;               ///< returns diagonal of a natural metric in q-space, depending on tree depth
  arr getLimits() const;

  /// @name active set selection
  void selectJointsByGroup(const StringA& groupNames, bool OnlyTheseOrNotThese=true, bool deleteInsteadOfLock=true);
  void selectJointsByName(const StringA&, bool notThose=false);
  void selectJointsBySubtrees(const StringA& roots, bool notThose=false);

  /// @name set state
  void setJointState(const arr& _q);
  void setJointState(const arr& _q, const StringA&);
  void setJointState(const arr& _q, const uintA&);
  void setJointState(const arr& _q, const FrameL&);
  void setFrameState(const arr& X, const StringA& frameNames= {}, bool warnOnDifferentDim=true);
  void setDofsForTree(const arr& q, rai::Frame* root);
  void setTimes(double t);
  void operator=(const arr& X) {
    if(X.d0==frames.N) setFrameState(X);
    else if(X.d0==getJointStateDimension()) setJointState(X);
    else HALT("wrong dimension");
  }

  /// @name variable (groups of DOFs, e.g. agents, joints, contacts) interface
  FrameL vars_frames;
  void vars_ensureFrames();
  uint vars_getNum(){ vars_ensureFrames();  return vars_frames.N; }
  const String& vars_getName(uint i);
  uint vars_getDim(uint i);
  void vars_activate(uint i);
  void vars_deactivate(uint i);
  void vars_qIndex2varIndex(uint& varId, uint& varIndex, uint qIndex);

  /// @name Jacobians and kinematics (low level)
  /// what is the linear velocity of a world point (pos_world) attached to frame a for a given joint velocity?
  void jacobian_pos(arr& J, Frame* a, const rai::Vector& pos_world) const; //usually called internally with kinematicsPos
  /// what is the angular velocity of frame a for a given joint velocity?
  void jacobian_angular(arr& J, Frame* a) const; //usually called internally with kinematicsVec or Quat
  /// how does the time coordinate of frame a change with q-change?
  void jacobian_tau(arr& J, Frame* a) const;

  void kinematicsPos(arr& y, arr& J, Frame* a, const Vector& rel=NoVector) const;
  void kinematicsVec(arr& y, arr& J, Frame* a, const Vector& vec=NoVector) const;
  void kinematicsQuat(arr& y, arr& J, Frame* a) const;
  void kinematicsPos_wrtFrame(arr& y, arr& J, Frame* b, const rai::Vector& rel, Frame* self) const;
  void hessianPos(arr& H, Frame* a, Vector* rel=0) const;
  void kinematicsTau(double& tau, arr& J) const;
  void kinematicsRelPos(arr& y, arr& J, Frame* a, const Vector& vec1, Frame* b, const Vector& vec2) const;
  void kinematicsRelVec(arr& y, arr& J, Frame* a, const Vector& vec1, Frame* b) const;

  void kinematicsContactPOA(arr& y, arr& J, ForceExchange* c) const;
  void kinematicsContactForce(arr& y, arr& J, ForceExchange* c) const;

  void kinematicsProxyCost(arr& y, arr& J, const Proxy& p, double margin=.0, bool addValues=false) const;
  void kinematicsProxyCost(arr& y, arr& J, double margin=.0) const;

  void kinematicsLimitsCost(arr& y, arr& J, const arr& limits, double margin=.1) const;

  /// @name features
  std::shared_ptr<Feature> feature(FeatureSymbol fs, const StringA& frames= {}) const;
  void evalFeature(arr& y, arr& J, FeatureSymbol fs, const StringA& frames= {}) const;

  /// @name high level inverse kinematics
  void inverseKinematicsPos(Frame& frame, const arr& ytarget, const rai::Vector& rel_offset=NoVector, int max_iter=3);

  /// @name get infos
  arr getHmetric() const;

  /// @name extensions on demand
  ConfigurationViewer& gl(const char* window_title=nullptr, bool offscreen=false);
  SwiftInterface& swift();
  FclInterface& fcl();
  void swiftDelete();
  PhysXInterface& physx();
  OdeInterface& ode();
  FeatherstoneInterface& fs();
  int watch(bool pause=false, const char* txt=nullptr);
  void saveVideoPic(uint& t, const char* pathPrefix="vid/");
  void glAdd(void (*call)(void*, OpenGL&), void* classP);
  int glAnimate();
  void glClose();
  void stepSwift();
  void stepFcl();
  void stepPhysx(double tau);
  void stepOde(double tau);
  void stepDynamics(arr& qdot, const arr& u_control, double tau, double dynamicNoise = 0.0, bool gravity = true);

  /// @name dynamics based on the fs() interface
  void equationOfMotion(arr& M, arr& F, const arr& qdot, bool gravity=true);
  void fwdDynamics(arr& qdd, const arr& qd, const arr& tau, bool gravity=true);
  void inverseDynamics(arr& tau, const arr& qd, const arr& qdd, bool gravity=true);
  double getEnergy(const arr& qdot);

  /// @name collisions & proxies
  double totalCollisionPenetration(); ///< proxies are returns from a collision engine; contacts stable constraints
  void copyProxies(const ProxyA& _proxies);

  /// @name I/O
  void write(std::ostream& os) const;
  void write(Graph& G) const;
  void writeURDF(std::ostream& os, const char* robotName="myrobot") const;
  void writeCollada(const char* filename) const;
  void writeMeshes(const char* pathPrefix="meshes/") const;
  void read(std::istream& is);
  void glDraw(struct OpenGL&);
  void glDraw_sub(struct OpenGL&, int drawOpaqueOrTransparanet=0);
  Graph getGraph() const;
  Array<Frame*> getLinks() const;
  void displayDot();

  //some info
  void report(std::ostream& os=std::cout) const;
  void reportProxies(std::ostream& os=std::cout, double belowMargin=1., bool brief=true) const;
  
  friend struct KinematicSwitch;
};

/// extension: containing deprecated functionalities
struct Configuration_ext : Configuration {
  arr qdot;

  void calc_fwdPropagateFrames();    ///< elementary forward kinematics; also computes all Shape frames
  void calc_Q_from_Frames();    ///< fill in the joint transformations assuming that frame poses are known (makes sense when reading files)

  void getJointState(arr& _q, arr& _qdot=NoArr) const;
  void setJointState(const arr& _q, const arr& _qdot=NoArr);

  /// @name Jacobians and kinematics (low level)
  void kinematicsPenetrations(arr& y, arr& J=NoArr, bool penetrationsOnly=true, double activeMargin=0.) const; ///< true: if proxy(i).distance>0. => y(i)=0; else y(i)=-proxy(i).distance
  void kinematicsProxyDist(arr& y, arr& J, const Proxy& p, double margin=.02, bool useCenterDist=true, bool addValues=false) const;
  void kinematicsContactCost(arr& y, arr& J, const ForceExchange* p, double margin=.0, bool addValues=false) const;
  void kinematicsContactCost(arr& y, arr& J, double margin=.0) const;
  void kinematicsProxyConstraint(arr& g, arr& J, const Proxy& p, double margin=.02) const;
  void kinematicsContactConstraints(arr& y, arr& J) const; //TODO: deprecated?
  void getLimitsMeasure(arr& x, const arr& limits, double margin=.1) const;

  /// @name active set selection
  void setAgent(uint) { NIY }

  /// @name High level (inverse) kinematics
  void inverseKinematicsPos(Frame& frame, const arr& ytarget, const rai::Vector& rel_offset=NoVector, int max_iter=3);

  /// @name dynamics
  void inertia(arr& M);

  /// @name forces and gravity
  void contactsToForces(double hook=.01, double damp=.0003);
  void gravityToForces(double g=-9.81);
  void frictionToForces(double coeff);
  void NewtonEuler_backward();

  /// @name collisions & proxies
  void filterProxiesToContacts(double margin=.01); ///< proxies are returns from a collision engine; contacts stable constraints
  void proxiesToContacts(double margin=.01); ///< proxies are returns from a collision engine; contacts stable constraints
};

} //namespace rai

stdPipes(rai::Configuration)

//===========================================================================
//
// OpenGL static draw functions
//

uintA stringListToShapeIndices(const rai::Array<const char*>& names, const FrameL& shapes);
uintA shapesToShapeIndices(const FrameL& shapes);

//===========================================================================
//
// C-style functions
//

void lib_ors();
void makeConvexHulls(FrameL& frames, bool onlyContactShapes=true);
void computeOptimalSSBoxes(FrameL& frames);
void computeMeshNormals(FrameL& frames, bool force=false);
void computeMeshGraphs(FrameL& frames, bool force=false);
double forceClosureFromProxies(rai::Configuration& C, uint frameIndex,
                               double distanceThreshold=0.01,
                               double mu=.5,     //friction coefficient
                               double discountTorques=1.);  //friction coefficient

void transferQbetweenTwoWorlds(arr& qto, const arr& qfrom, const rai::Configuration& to, const rai::Configuration& from);
void transferQDotbetweenTwoWorlds(arr& qDotTo, const arr& qDotFrom, const rai::Configuration& to, const rai::Configuration& from);
void transferKpBetweenTwoWorlds(arr& KpTo, const arr& KpFrom, const rai::Configuration& to, const rai::Configuration& from);
void transferKdBetweenTwoWorlds(arr& KdTo, const arr& KdFrom, const rai::Configuration& to, const rai::Configuration& from);
void transferU0BetweenTwoWorlds(arr& u0To, const arr& u0From, const rai::Configuration& to, const rai::Configuration& from);
void transferKI_ft_BetweenTwoWorlds(arr& KI_ft_To, const arr& KI_ft_From, const rai::Configuration& to, const rai::Configuration& from);

void displayState(const arr& x, rai::Configuration& G, const char* tag);
void displayTrajectory(const arr& x, int steps, rai::Configuration& G, const KinematicSwitchL& switches, const char* tag, double delay=0., uint dim_z=0, bool copyG=false);
inline void displayTrajectory(const arr& x, int steps, rai::Configuration& G, const char* tag, double delay=0., uint dim_z=0, bool copyG=false) {
  displayTrajectory(x, steps, G, {}, tag, delay, dim_z, copyG);
}
void editConfiguration(const char* orsfile, rai::Configuration& G);
int animateConfiguration(rai::Configuration& G, struct Inotify* ino=nullptr);

void kinVelocity(arr& y, arr& J, uint frameId, const ConfigurationL& Ktuple, double tau);
void kinAngVelocity(arr& y, arr& J, uint frameId, const ConfigurationL& Ktuple, double tau);
