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


#ifndef MLR_kin_h
#define MLR_kin_h

#include <Core/array.h>
#include <Geo/geo.h>

struct OpenGL;
struct PhysXInterface;
struct SwiftInterface;
struct OdeInterface;

//===========================================================================

namespace mlr{

struct Joint;
struct Shape;
struct Frame;
struct Proxy;
struct KinematicWorld;
struct KinematicSwitch;

} // namespace mlr

//===========================================================================

extern mlr::KinematicWorld& NoWorld;

typedef mlr::Array<mlr::Joint*> JointL;
//typedef mlr::Array<mlr::Shape*> ShapeL;
typedef mlr::Array<mlr::Frame*> FrameL;
typedef mlr::Array<mlr::Proxy*> ProxyL;
typedef mlr::Array<mlr::KinematicSwitch*> KinematicSwitchL;
typedef mlr::Array<mlr::KinematicWorld*> WorldL;

//===========================================================================

namespace mlr{

/// data structure to store a whole physical situation (lists of bodies, joints, shapes, proxies)
struct KinematicWorld : GLDrawer{
  struct sKinematicWorld *s;

  //-- fundamental structure
  FrameL frames;

  //-- derived: computed with calc_q(); reset with reset_q()
  arr q, qdot; ///< the current joint configuration vector and velocities
  FrameL fwdActiveSet;
  JointL fwdActiveJoints;

  ProxyL proxies; ///< list of current proximities between bodies

  static uint setJointStateCount;

  //global options
  bool orsDrawJoints=false, orsDrawShapes=true, orsDrawBodies=true, orsDrawProxies=true, orsDrawMarkers=true, orsDrawColors=true, orsDrawIndexColors=false;
  bool orsDrawMeshes=true, orsDrawCores=false, orsDrawZlines=false;
  bool orsDrawBodyNames=false;
  double orsDrawAlpha=1.;
  uint orsDrawLimit=0;
  
  /// @name constructors
  KinematicWorld();
  KinematicWorld(const mlr::KinematicWorld& other);
  KinematicWorld(const char* filename);
  virtual ~KinematicWorld();
  void operator=(const mlr::KinematicWorld& K){ copy(K); }
  void copy(const mlr::KinematicWorld& K, bool referenceMeshesAndSwiftOnCopy=false);
  
  /// @name initializations
  void init(const char* filename);
  void init(const Graph& G);

  /// @name access
  Frame *getFrameByName(const char* name, bool warnIfNotExist=true) const;
//  Link  *getLinkByBodies(const Frame* from, const Frame* to) const;
  Joint *getJointByBodies(const Frame* from, const Frame* to) const;
  Joint *getJointByBodyNames(const char* from, const char* to) const;
  Joint *getJointByBodyIndices(uint ifrom, uint ito) const;
  StringA getJointNames();

  bool checkUniqueNames() const;
  void prefixNames();

  /// @name changes of configuration
  void clear();
  void reset_q();
  void calc_activeSets();
  void calc_q();
  void reconfigureRoot(Frame *root);  ///< n becomes the root of the kinematic tree; joints accordingly reversed; lists resorted
  void pruneRigidJoints(int verbose=0);        ///< delete rigid joints -> they become just links
  void reconnectLinksToClosestJoints();        ///< re-connect all links to closest joint
  void pruneUselessFrames(int verbose=0);      ///< delete frames that have no name, joint, and shape
  void optimizeTree();                         ///< call the three above methods in this order
  bool checkConsistency();

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
  arr naturalQmetric(double power=.5) const;               ///< returns diagonal of a natural metric in q-space, depending on tree depth
  arr getLimits() const;

  /// @name active set selection
  void setActiveJointsByName(const StringA&);

  /// @name set state
  void setJointState(const arr& _q, const arr& _qdot=NoArr);

  /// @name kinematics
  void kinematicsPos (arr& y, arr& J, Frame *a, const Vector& rel=NoVector) const; //TODO: make vector& not vector*
  void kinematicsVec (arr& y, arr& J, Frame *a, const Vector& vec=NoVector) const;
  void kinematicsQuat(arr& y, arr& J, Frame *a) const;
  void hessianPos(arr& H, Frame *a, Vector *rel=0) const;
  void axesMatrix(arr& J, Frame *a) const;
  void kinematicsRelPos (arr& y, arr& J, Frame *a, const Vector& vec1, Frame *b, const Vector& vec2) const;
  void kinematicsRelVec (arr& y, arr& J, Frame *a, const Vector& vec1, Frame *b) const;
  void kinematicsRelRot (arr& y, arr& J, Frame *a, Frame *b) const;

  void kinematicsProxyDist(arr& y, arr& J, Proxy *p, double margin=.02, bool useCenterDist=true, bool addValues=false) const;
  void kinematicsProxyCost(arr& y, arr& J, Proxy *p, double margin=.02, bool useCenterDist=true, bool addValues=false) const;
  void kinematicsProxyCost(arr& y, arr& J, double margin=.02, bool useCenterDist=true) const;
  void kinematicsProxyConstraint(arr& g, arr& J, Proxy *p, double margin=.02) const;
  void kinematicsContactConstraints(arr& y, arr &J) const; //TODO: deprecated?
  void kinematicsPos_wrtFrame(arr& y, arr& J, Frame *b, const mlr::Vector& rel, Frame *s) const;
  void getLimitsMeasure(arr &x, const arr& limits, double margin=.1) const;
  void kinematicsLimitsCost(arr& y, arr& J, const arr& limits, double margin=.1) const;

  /// @name active set selection
  void setAgent(uint){ NIY }

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
  void glDraw_sub(struct OpenGL&);
  Graph getGraph() const;

  //some info
  void report(std::ostream& os=std::cout) const;
  void reportProxies(std::ostream& os=std::cout, double belowMargin=-1., bool brief=true) const;
  void writePlyFile(const char* filename) const; //TODO: move outside

  friend struct KinematicSwitch;
};

} //namespace mlr


stdPipes(mlr::KinematicWorld)


//===========================================================================
//
// OpenGL static draw functions
//

namespace mlr {
void glDrawGraph(void*);
}

uintA stringListToShapeIndices(const mlr::Array<const char*>& names, const FrameL& shapes);
uintA shapesToShapeIndices(const FrameL &shapes);

//===========================================================================
//
// C-style functions
//

void lib_ors();
void makeConvexHulls(FrameL& frames, bool onlyContactShapes=true);
void computeOptimalSSBoxes(FrameL& frames);
void computeMeshNormals(FrameL& frames, bool force=false);
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


void displayState(const arr& x, mlr::KinematicWorld& G, const char *tag);
void displayTrajectory(const arr& x, int steps, mlr::KinematicWorld& G, const KinematicSwitchL& switches, const char *tag, double delay=0., uint dim_z=0, bool copyG=false);
inline void displayTrajectory(const arr& x, int steps, mlr::KinematicWorld& G, const char *tag, double delay=0., uint dim_z=0, bool copyG=false){
  displayTrajectory(x, steps, G, {}, tag, delay, dim_z, copyG);
}
void editConfiguration(const char* orsfile, mlr::KinematicWorld& G);
int animateConfiguration(mlr::KinematicWorld& G, struct Inotify *ino=NULL);




#endif //MLR_ors_h
