/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "featureSymbols.h"
#include "../Core/array.h"
#include "../Geo/geo.h"
#include "../Geo/mesh.h"

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
struct ForceExchange;
struct Configuration;
struct KinematicSwitch;

struct FclInterface;
struct ConfigurationViewer;

} // namespace rai

//===========================================================================

struct Value {
  arr y, J;
  Value(){}
  Value(const arr& y, const arr& J) : y(y), J(J) {}
  void write(ostream& os) const { os <<"y:" <<y <<" J:" <<J; }
};
stdOutPipe(Value)

extern rai::Configuration& NoConfiguration;

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

/// data structure to store a kinematic/physical situation (lists of frames (with joints, shapes, inertias), forces & proxies)
struct Configuration : GLDrawer {
  unique_ptr<struct sConfiguration> self;

  //-- fundamental structure
  FrameL frames;     ///< list of coordinate frames, with shapes, joints, inertias attached
  ForceExchangeL forces; ///< list of force exchanges between frames
  ProxyA proxies;    ///< list of current collision proximities between frames
  arr q;             ///< the current configuration state (DOF) vector
  arr qInactive;    ///< configuration state of all inactive DOFs

  //-- data structure state (lazy evaluation leave the state structure out of sync)
  JointL activeJoints; //list of currently active joints (computed with ensure_activeSets(); reset with reset_q())
  bool _state_indexedJoints_areGood=false; // the active sets, incl. their topological sorting, are up to date
  bool _state_q_isGood=false; // the q-vector represents the current relative transforms (and force dofs)
  bool _state_proxies_isGood=false; // the proxies have been created for the current state
  //TODO: need a _state for all the plugin engines (SWIFT, PhysX)? To auto-reinitialize them when the config changed structurally?

  //-- format in which Jacobians are returned
  enum JacobianMode { JM_dense, JM_sparse, JM_rowShifted, JM_noArr, JM_emptyShape };
  JacobianMode jacMode = JM_dense;

  static uint setJointStateCount;

  /// @name constructors
  Configuration();
  Configuration(const Configuration& other, bool referenceSwiftOnCopy=false) : Configuration() {  copy(other, referenceSwiftOnCopy);  } ///< same as copy()
  Configuration(const char* filename) : Configuration() {  addFile(filename);  } ///< same as addFile()
  virtual ~Configuration();

  /// @name copy
  void operator=(const Configuration& K) { copy(K); } ///< same as copy()
  void copy(const Configuration& K, bool referenceSwiftOnCopy=false);
  bool operator!() const;

  /// @name initializations, building configurations
  Frame* addFrame(const char* name, const char* parent=nullptr, const char* args=nullptr);
  Frame* addFile(const char* filename);
  void addAssimp(const char* filename);
  void addCopies(const FrameL& F, const ForceExchangeL& _forces);
  void addConfiguration(const Configuration& C){ addCopies(C.frames, C.forces); } ///< same as addCopies() with C.frames and C.forces

  /// @name get frames
  Frame* operator[](const char* name) const { return getFrame(name, true); }  ///< same as getFrame()
  Frame* operator()(int i) const { return frames(i); } ///< same as 'frames.elem(i)'  (the i-th frame)
  Frame* getFrame(const char* name, bool warnIfNotExist=true, bool reverse=false) const;
  FrameL getFrames(const uintA& ids) const;
  FrameL getFrames(const StringA& names) const;
  uintA getFrameIDs(const StringA& names) const;
  StringA getFrameNames() const;
  FrameL getJoints(bool activesOnly=true) const;
  FrameL getJointsSlice(uint t, bool activesOnly=true) const;
  FrameL getJointsSlice(const FrameL& slice, bool activesOnly=true) const;
  uintA getJointIDs() const;
  StringA getJointNames() const;
  uintA getCtrlFramesAndScale(arr& scale=NoArr) const;
  FrameL getRoots() const;
  FrameL getLinks() const;

  /// @name get state
  uint getJointStateDimension() const;
  const arr& getJointState() const;
  arr getJointState(const FrameL& F) const;
  arr getJointState(const uintA& F) const { return getJointState(getFrames(F)); } ///< same as getJointState() with getFrames()
  arr getJointStateSlice(uint t, bool activesOnly=true){  return getJointState(getJointsSlice(t, activesOnly));  }
  arr getFrameState() const { return getFrameState(frames); } ///< same as getFrameState() for all \ref frames
  arr getFrameState(const FrameL& F) const;
  arr getFrameState(const uintA& F) const { return getFrameState(getFrames(F)); } ///< same as getFrameState() with getFrames()

  /// @name set state
  void setJointState(const arr& _q);
  void setJointState(const arr& _q, const FrameL& F);
  void setJointState(const arr& _q, const uintA& F){ setJointState(_q, getFrames(F)); } ///< same as setJointState() with getFrames()
  void setJointStateSlice(const arr& _q, uint t, bool activesOnly=true){  setJointState(_q, getJointsSlice(t, activesOnly));  }
  void setFrameState(const arr& X){ setFrameState(X, frames); } ///< same as setFrameState() for all \ref frames
  void setFrameState(const arr& X, const FrameL& F);
  void setFrameState(const arr& X, const uintA& F){ setFrameState(X, getFrames(F)); } ///< same as setFrameState() with getFrames()
  void setTaus(double tau);

  /// @name active DOFs selection
  void setActiveJoints(const JointL& F);
  void selectJoints(const FrameL& F, bool notThose=false);
  void selectJointsByName(const StringA&, bool notThose=false);
  void selectJointsBySubtrees(const FrameL& roots, bool notThose=false);
  void selectJointsByAtt(const StringA& attNames, bool notThose=false);

  /// @name get other information
  arr getCtrlMetric() const;
  arr getNaturalCtrlMetric(double power=.5) const;               ///< returns diagonal of a natural metric in q-space, depending on tree depth
  arr getLimits() const;
  double getEnergy(const arr& qdot);
  double getTotalPenetration(); ///< proxies are returns from a collision engine; contacts stable constraints
  Graph reportForces();
  bool checkUniqueNames() const;
  FrameL calc_topSort() const;
  bool check_topSort() const;

  /// @name structural operations, changes of configuration
  void clear();
  void reset_q();
  void reconfigureRoot(Frame* newRoot, bool ofLinkOnly);  ///< n becomes the root of the kinematic tree; joints accordingly reversed; lists resorted
  void flipFrames(Frame* a, Frame* b);
  void pruneRigidJoints();        ///< delete rigid joints -> they become just links
  void pruneInactiveJoints();        ///< delete rigid joints -> they become just links
  void reconnectLinksToClosestJoints();        ///< re-connect all links to closest joint
  void pruneUselessFrames(bool pruneNamed=false, bool pruneNonContactNonMarker=false);  ///< delete frames that have no name, joint, and shape
  void optimizeTree(bool _pruneRigidJoints=false, bool pruneNamed=false, bool pruneNonContactNonMarker=false);        ///< call the three above methods in this order
  void sortFrames();
  void makeObjectsFree(const StringA& objects, double H_cost=0.);
  void addTauJoint();
  bool hasTauJoint(Frame* a=0);
  bool checkConsistency() const;
  Joint* attach(Frame* a, Frame* b);
  Joint* attach(const char* a, const char* b);
  uintA getCollisionExcludeIDs(bool verbose=false);
  uintA getCollisionExcludePairIDs(bool verbose=false);
  void prefixNames(bool clear=false);

  /// @name computations on the tree
  void calc_indexedActiveJoints(bool resetActiveJointSet=true); ///< sort of private: count the joint dimensionalities and assign j->q_index
  void calc_Q_from_q();  ///< from q compute the joint's Q transformations
  void calc_q_from_Q();  ///< updates q based on the joint's Q transformations
  arr calc_fwdPropagateVelocities(const arr& qdot);    ///< elementary forward kinematics

  /// @name ensure state consistencies
  void ensure_indexedJoints() {   if(!_state_indexedJoints_areGood) calc_indexedActiveJoints();  }
  void ensure_q() {  if(!_state_q_isGood) calc_q_from_Q();  }
  void ensure_proxies() {  if(!_state_proxies_isGood) stepSwift();  }

  /// @name Jacobians and kinematics (low level)
  void jacobian_pos(arr& J, Frame* a, const Vector& pos_world) const; //usually called internally with kinematicsPos
  void jacobian_angular(arr& J, Frame* a) const; //usually called internally with kinematicsVec or Quat
  void jacobian_tau(arr& J, Frame* a) const;
  void jacobian_zero(arr& J, uint n) const;

  void kinematicsZero(arr& y, arr& J, uint n) const;
  void kinematicsPos(arr& y, arr& J, Frame* a, const Vector& rel=NoVector) const;
  void kinematicsVec(arr& y, arr& J, Frame* a, const Vector& vec=NoVector) const;
  void kinematicsQuat(arr& y, arr& J, Frame* a) const;
  void kinematicsPos_wrtFrame(arr& y, arr& J, Frame* b, const Vector& rel, Frame* self) const;
  void hessianPos(arr& H, Frame* a, Vector* rel=0) const;
  void kinematicsTau(double& tau, arr& J, Frame* a=0) const;

  void kinematicsPenetration(arr& y, arr& J, const Proxy& p, double margin=.0, bool addValues=false) const;
  void kinematicsPenetration(arr& y, arr& J, double margin=.0) const;

  void setJacModeAs(const arr& J);

  /// @name features
  shared_ptr<Feature> feature(FeatureSymbol fs, const StringA& frames= {}) const;
  void evalFeature(arr& y, arr& J, FeatureSymbol fs, const StringA& frames= {}) const;
  template<class T> Value eval(const StringA& frames= {}){ return T().eval(getFrames(frames)); }
  Value eval(FeatureSymbol fs, const StringA& frames= {});

  /// @name high level inverse kinematics
  void inverseKinematicsPos(Frame& frame, const arr& ytarget, const Vector& rel_offset=NoVector, int max_iter=3);

  /// @name dynamics based on the fs() interface
  void equationOfMotion(arr& M, arr& F, const arr& qdot, bool gravity=true);
  void fwdDynamics(arr& qdd, const arr& qd, const arr& tau, bool gravity=true);
  void inverseDynamics(arr& tau, const arr& qd, const arr& qdd, bool gravity=true);

  /// @name collisions & proxies
  void copyProxies(const ProxyA& _proxies);
  void addProxies(const uintA& collisionPairs);

  /// @name extensions on demand
  shared_ptr<ConfigurationViewer>& gl(const char* window_title=nullptr, bool offscreen=false);
  shared_ptr<SwiftInterface> swift();
  shared_ptr<FclInterface> fcl();
  void swiftDelete();
  PhysXInterface& physx();
  OdeInterface& ode();
  FeatherstoneInterface& fs();
  bool hasView();
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

  /// @name I/O
  void write(std::ostream& os) const;
  void write(Graph& G) const;
  void writeURDF(std::ostream& os, const char* robotName="myrobot") const;
  void writeCollada(const char* filename, const char* format="collada") const;
  void writeMeshes(const char* pathPrefix="meshes/") const;
  void read(std::istream& is);
  void glDraw(struct OpenGL&);
  void glDraw_sub(struct OpenGL& gl, const FrameL& F, int drawOpaqueOrTransparanet=0);
  Graph getGraph() const;
  void displayDot();

  //some info
  void report(std::ostream& os=std::cout) const;
  void reportProxies(std::ostream& os=std::cout, double belowMargin=1., bool brief=true) const;

private:
  void readFromGraph(const Graph& G, bool addInsteadOfClear=false);
  friend struct KinematicSwitch;
  friend void editConfiguration(const char* orsfile, Configuration& G);
};

stdPipes(Configuration)

//===========================================================================
//
// OpenGL static draw functions
//

uintA framesToIndices(const FrameL& frames);
uintA jointsToIndices(const JointL& joints);

//===========================================================================
//
// C-style functions
//

void makeConvexHulls(FrameL& frames, bool onlyContactShapes=true);
void computeOptimalSSBoxes(FrameL& frames);
void computeMeshNormals(FrameL& frames, bool force=false);
void computeMeshGraphs(FrameL& frames, bool force=false);

void editConfiguration(const char* orsfile, Configuration& G);
int animateConfiguration(Configuration& G, struct Inotify* ino=nullptr);

} //namespace rai
