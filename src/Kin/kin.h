/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "featureSymbols.h"
#include "../Core/array.h"
#include "../Core/graph.h"
#include "../Geo/geo.h"
#include "../Geo/mesh.h"

struct OpenGL;
struct PhysXInterface;
//struct SwiftInterface;
struct OdeInterface;
struct FeatherstoneInterface;

//===========================================================================

namespace rai {

struct Dof;
struct Joint;
struct Shape;
struct Frame;
struct Proxy;
struct ForceExchangeDof;
struct ForceExchangeDof;
struct Configuration;
struct KinematicSwitch;

struct FclInterface;
struct ConfigurationViewer;

} // namespace rai

//===========================================================================

extern rai::Configuration& NoConfiguration;

typedef rai::Array<rai::Dof*> DofL;
typedef rai::Array<rai::Joint*> JointL;
//typedef rai::Array<rai::Shape*> ShapeL;
typedef rai::Array<rai::Frame*> FrameL;
typedef rai::Array<rai::Proxy*> ProxyL;
typedef rai::Array<rai::Proxy> ProxyA;
typedef rai::Array<rai::ForceExchangeDof*> ForceExchangeL;
typedef rai::Array<rai::KinematicSwitch*> KinematicSwitchL;
typedef rai::Array<rai::Configuration*> ConfigurationL;

namespace rai {

//===========================================================================

/// data structure to store a kinematic/physical situation (lists of frames (with joints, shapes, inertias), forces & proxies)
struct Configuration {
  unique_ptr<struct sConfiguration> self;

  //-- fundamental structure
  FrameL frames;    ///< list of coordinate frames, with shapes, joints, inertias attached
  DofL otherDofs;   ///< list of other degrees of freedom (forces)
  ProxyA proxies;   ///< list of current collision proximities between frames
  arr q;            ///< the current configuration state (DOF) vector
  arr qInactive;    ///< configuration state of all inactive DOFs

  //-- data structure state (lazy evaluation leave the state structure out of sync)
  DofL activeDofs; //list of currently active dofs (computed with ensure_activeSets(); reset with reset_q())
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
  Configuration(const Configuration& other, bool referenceFclOnCopy=false) : Configuration() {  copy(other, referenceFclOnCopy);  } ///< same as copy()
  Configuration(const char* filename) : Configuration() {  addFile(filename);  } ///< same as addFile()
  virtual ~Configuration();

  /// @name copy
  void operator=(const Configuration& K) { copy(K); } ///< same as copy()
  void copy(const Configuration& K, bool referenceFclOnCopy=false);
  bool operator!() const;

  /// @name initializations, building configurations
  Frame* addFrame(const char* name, const char* parent=nullptr, const char* args=nullptr, bool warnDuplicateName=true);
  Frame* addFile(const char* filename, const char* namePrefix=0);
  Frame& addDict(const Graph& G);
  Frame* addAssimp(const char* filename);
  Frame* addH5Object(const char* framename, const char* filename, int verbose);
  Frame* addCopy(const FrameL& F, const DofL& _dofs, const str& prefix= {});
  Frame* addConfigurationCopy(const Configuration& C, const str& prefix= {}, double tau=1.);
  void delFrame(const char* name);
  void delSubtree(const char* name);

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
  uintA getDofIDs() const;
  StringA getJointNames() const;
  DofL getDofs(const FrameL& F, bool actives, bool inactives, bool mimics=false, bool forces=true) const;
  uintA getCtrlFramesAndScale(arr& scale=NoArr, bool jointPairs=true) const;
  FrameL getRoots() const;
  FrameL getParts() const;
  FrameL getLinks() const;
  rai::Array<DofL> getPartsDofs() const;

  /// @name get dof or frame state
  uint getJointStateDimension() const;
  const arr& getJointState() const;
  arr getDofState(const DofL& dofs) const;
  arr getJointState(const FrameL& F) const { return getDofState(getDofs(F, true, true)); }
  arr getJointState(const uintA& F) const { return getJointState(getFrames(F)); } ///< same as getJointState() with getFrames()
  arr getJointStateSlice(uint t, bool activesOnly=true) {  return getJointState(getJointsSlice(t, activesOnly));  }
  arr getDofHomeState(const DofL& dofs) const;
  arr getFrameState() const { return getFrameState(frames); } ///< same as getFrameState() for all \ref frames
  arr getFrameState(const FrameL& F) const;
  arr getFrameState(const uintA& F) const { return getFrameState(getFrames(F)); } ///< same as getFrameState() with getFrames()

  /// @name set state
  void setJointState(const arr& _q);
  void setDofState(const arr& _q, const DofL& dofs, bool mimicsIncludedInQ=false);
  void setJointState(const arr& _q, const FrameL& F) { setDofState(_q, getDofs(F, true, true)); }
  void setJointState(const arr& _q, const uintA& F) { setJointState(_q, getFrames(F)); } ///< same as setJointState() with getFrames()
  void setJointStateSlice(const arr& _q, uint t, bool activesOnly=true) {  setJointState(_q, getJointsSlice(t, activesOnly));  }
  void setFrameState(const arr& X) { setFrameState(X, frames); } ///< same as setFrameState() for all \ref frames
  void setFrameState(const arr& X, const FrameL& F);
  void setFrameState(const arr& X, const uintA& F) { setFrameState(X, getFrames(F)); } ///< same as setFrameState() with getFrames()
  void setTaus(double tau);
  void setTaus(const arr& tau);
  void setRandom(uint timeSlices_d1=0, int verbose=0);
  void setDofBiasesToCurrent();

  /// @name active DOFs selection
  void setActiveDofs(const DofL& dofs);
  void selectJoints(const FrameL& F, bool notThose=false);
  void selectJoints(const DofL& dofs, bool notThose=false);
  void selectJointsByName(const StringA&, bool notThose=false);
  void selectJointsBySubtrees(const FrameL& roots, bool notThose=false);
  void selectJointsByAtt(const StringA& attNames, bool notThose=false);

  /// @name get other information
  arr getCtrlMetric() const;
  arr getNaturalCtrlMetric(double power=.5) const;               ///< returns diagonal of a natural metric in q-space, depending on tree depth
  arr getJointLimits(const DofL& dofs) const;
  arr getJointLimits() const { getJointStateDimension(); return getJointLimits(activeDofs); }
  arr getTorqueLimits(const DofL& dofs, uint index=4) const;
  double getEnergy(const arr& qdot);
  // bool getCollisionFree(); //broken
  Graph reportForces();
  bool checkUniqueNames(bool makeUnique=false);
  FrameL calc_topSort() const;
  bool check_topSort() const;

  /// @name structural operations, changes of configuration
  void clear();
  void reset_q();
  void reconfigureRoot(Frame* newRoot, bool untilPartBreak);  ///< n becomes the root of the kinematic tree; joints accordingly reversed; lists resorted
  void flipFrames(Frame* a, Frame* b);
  void pruneRigidJoints();        ///< delete rigid joints -> they become just links
  void pruneInactiveJoints();        ///< delete rigid joints -> they become just links
  void pruneEmptyShapes();
  void reconnectShapesToParents();
  void reconnectLinksToClosestJoints();        ///< re-connect all links to closest joint
  void pruneUselessFrames(bool pruneNamed=false, bool pruneNonContactShapes=false, bool pruneTransparent=false);  ///< delete frames that have no name, joint, and shape
  void processStructure(bool _pruneRigidJoints=false, bool reconnectToLinks=true, bool pruneNonContactShapes=false, bool pruneTransparent=false);        ///< call the three above methods in this order
  void processInertias(bool recomputeInertias=true, bool transformToDiagInertia=false);
  void sortFrames();
  void makeObjectsFree(const StringA& objects, double H_cost=0.);
  void addTauJoint();
  bool hasTauJoint(Frame* a=0);
  bool checkConsistency() const;
  Joint* attach(Frame* a, Frame* b);
  Joint* attach(const char* a, const char* b);
  uintAA getCollisionExcludePairIDs(int verbose=0);
  FrameL getCollidableShapes();
  FrameL getCollidablePairs();
  void prefixNames(bool clear=false);

  /// @name computations on the tree
  void calc_indexedActiveJoints(bool resetActiveJointSet=true); ///< sort of private: count the joint dimensionalities and assign j->q_index
  void calc_Q_from_q();  ///< from q compute the joint's Q transformations
  void calcDofsFromConfig();  ///< updates q based on the joint's Q transformations
  arr calc_fwdPropagateVelocities(const arr& qdot);    ///< elementary forward kinematics

  /// @name ensure state consistencies
  void ensure_indexedJoints() {   if(!_state_indexedJoints_areGood) calc_indexedActiveJoints();  }
  void ensure_q() {  if(!_state_q_isGood) calcDofsFromConfig();  }
  void ensure_proxies(bool fine=false); //both, broadphase and fine!!

  /// @name Jacobians and kinematics (low level)
  void jacobian_pos(arr& J, Frame* a, const Vector& pos_world) const; //usually called internally with kinematicsPos
  void jacobian_angular(arr& J, Frame* a) const; //usually called internally with kinematicsVec or Quat
  void jacobian_tau(arr& J, Frame* a) const;
  void jacobian_zero(arr& J, uint n) const;

  arr kinematics_pos(Frame* a, const Vector& rel=NoVector) const { arr y, J; kinematicsPos(y, J, a, rel); if(!!J) y.J()=J; return y; }
  arr kinematics_vec(Frame* a, const Vector& vec=NoVector) const { arr y, J; kinematicsVec(y, J, a, vec); if(!!J) y.J()=J; return y; }
  arr kinematics_zero(uint n) const { arr y, J; kinematicsZero(y, J, n); if(!!J) y.J()=J; return y; }

  void kinematicsZero(arr& y, arr& J, uint n) const;
  void kinematicsPos(arr& y, arr& J, Frame* a, const Vector& rel=NoVector) const;
  void kinematicsVec(arr& y, arr& J, Frame* a, const Vector& vec=NoVector) const;
  void kinematicsMat(arr& y, arr& J, Frame* a) const;
  void kinematicsQuat(arr& y, arr& J, Frame* a) const;
  void kinematicsPos_wrtFrame(arr& y, arr& J, Frame* b, const Vector& rel, Frame* self) const;
  void hessianPos(arr& H, Frame* a, Vector* rel=0) const;
  void kinematicsTau(double& tau, arr& J, Frame* a=0) const;

  void kinematicsPenetration(arr& y, arr& J, const Proxy& p, double margin=.0, bool addValues=false) const;
  void kinematicsPenetration(arr& y, arr& J, double margin=.0) const;

  void setJacModeAs(const arr& J);

  /// @name features
  shared_ptr<Feature> feature(FeatureSymbol fs, const StringA& frames= {}, const arr& scale=NoArr, const arr& target=NoArr, int order=-1) const;
  template<class T> arr eval(const StringA& frames= {}) { ensure_q(); return T().eval(getFrames(frames)); }
  arr eval(FeatureSymbol fs, const StringA& frames= {}, const arr& scale=NoArr, const arr& target=NoArr, int order=-1);

  /// @name high level inverse kinematics
  void inverseKinematicsPos(Frame& frame, const arr& ytarget, const Vector& rel_offset=NoVector, int max_iter=3);

  /// @name dynamics following the lecture notes
  void dyn_MF(arr& M, arr& F, const arr& q_dot);
  arr dyn_inverseDyamics(const arr& q_dot, const arr& q_ddot);
  arr dyn_fwdDynamics(const arr& q_dot, const arr& u);
  void dyn_fwdStep_RungeKutta(arr& q_dot, const arr& u, double tau);
  double dyn_energy(const arr& q_dot);
// private: //internal:
  struct FrameDynState{ bool isGood=false; Vector p, v, w, vd, wd; Matrix R; };
  FrameDynState& dyn_ensure(Frame* f, const arr& q_dot, Array<FrameDynState>& buffer);
  arr dyn_inertia(Frame* f);
  arr dyn_M(Frame *f, const arr& I_f);
  arr dyn_J_dot(Frame *f, const arr& q_dot, const arr& Jpos, const arr& Jang);
  arr dyn_C(Frame *f, const arr& q_dot, const arr& I_f, const arr& Jpos, const arr& Jang, Array<FrameDynState>& buffer);
public:

  /// @name dynamics based on the fs() interface
  void equationOfMotion(arr& M, arr& F, const arr& qdot, bool gravity=true);
  arr fwdDynamics(const arr& qd, const arr& tau, bool gravity=true);
  void inverseDynamics(arr& tau, const arr& qd, const arr& qdd, bool gravity=true);

  /// @name collisions & proxies
  void coll_setActiveColliders(const FrameL& colliders);
  void coll_addExcludePair(uint aID, uint bID);

  double coll_totalViolation(); ///< proxies are returns from a collision engine; contacts stable constraints
  bool coll_isCollisionFree();
  void coll_reportProxies(std::ostream& os=cout, double belowMargin=1., bool brief=true) const;
  StringA coll_getProxyPairs(double belowMargin, arr& distances=NoArr);
  std::shared_ptr<FclInterface> coll_fcl(int verbose=0);
  void coll_fclReset();
  void addProxies(const uintA& collisionPairs);

  /// @name extensions on demand
  std::shared_ptr<ConfigurationViewer> get_viewer();
  OpenGL& gl();
  void view_lock(const char* _lockInfo);
  void view_unlock();
  PhysXInterface& physx();
  OdeInterface& ode();
  FeatherstoneInterface& fs();
  bool hasView();
  int view(bool pause=false, const char* txt=nullptr);
  void saveVideoPic(uint& t, const char* pathPrefix="vid/");
  void glAdd(void (*call)(void*, OpenGL&), void* classP);
  int glAnimate();
  void view_close();
  void view_setCameraPose(const arr& pose);
  arr view_getCameraPose();
  void set_viewer(const std::shared_ptr<ConfigurationViewer>& _viewer);
  void coll_stepFcl();
  void stepPhysx(double tau);
  void stepOde(double tau);
  void stepDynamics(arr& qdot, const arr& u_control, double tau, double dynamicNoise = 0.0, bool gravity = true);

  /// @name I/O
  void write(std::ostream& os, bool explicitlySorted=false) const;
  void write(Graph& G) const;
  void writeURDF(std::ostream& os, const char* robotName="myrobot") const;
  void writeCollada(const char* filename, const char* format="collada") const;
  void writeMeshes(str pathPrefix="meshes/", bool copyTextures=false, bool enumerateAssets=false) const;
  void writeMesh(const char* filename="z.ply") const;
  void read(std::istream& is);
  Graph getGraph() const;
  void displayDot();

  void watchFile(const char* filename);
  int animate(struct Inotify* ino=nullptr);
  void animateSpline(uint T=3);

  //some info
  void report(std::ostream& os=cout) const;
  void reportLimits(std::ostream& os=cout) const;

 private:
  friend struct KinematicSwitch;
};

stdPipes(Configuration)

//===========================================================================
//
// OpenGL static draw functions
//

uintA framesToIndices(const FrameL& frames);
FrameL dofsToFrames(const DofL& dofs);
StringA framesToNames(const FrameL& frames);

//===========================================================================
//
// C-style functions
//

void makeConvexHulls(FrameL& frames, bool onlyContactShapes=true);
void computeOptimalSSBoxes(FrameL& frames);
void computeMeshNormals(FrameL& frames, bool force=false);
void computeMeshGraphs(FrameL& frames, bool force=false);

} //namespace rai
