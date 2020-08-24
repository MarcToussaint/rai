/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef _HEADER_GUARD_ORS_OLDTASKVARIABLES_H_
#define _HEADER_GUARD_ORS_OLDTASKVARIABLES_H_

struct TaskVariable;

/** Different TVtype of task variables: refer to different ways to
 * compute/access the kinematics and Jacobians.
 *
 * @todo move this to Default task variable?
 */
enum TVtype {
  noneTVT,     ///< undefined
  posTVT,      ///< 3D position of reference, can have 2nd reference, no param
  zoriTVT,     ///< 3D z-axis orientation, no 2nd reference, no param
  zalignTVT,   ///< 1D z-axis alignment, can have 2nd reference, param (optional) determins alternative reference world vector
  qItselfTVT,  ///< q itself as task variable, no param
  qLinearTVT,  ///< k-dim variable linear in q, no references, param: k-times-n matrix
  qSingleTVT,  ///< 1D entry of q, reference-integer=index, no param
  qSquaredTVT, ///< 1D square norm of q, no references, param: n-times-n matrix
  qLimitsTVT,  ///< 1D meassure for joint limit violation, no references, param: n-times-2 matrix with lower and upper limits for each joint
  collTVT,     ///< 1D meassure for collision violation, no references, param: 1D number defining the distance margin
  colConTVT,   ///< 1D meassure collision CONSTRAINT meassure, no references, param: 1D number defining the distance margin
  comTVT,      ///< 2D vector of the horizontal center of mass, no refs, no param
  skinTVT,     ///< vector of skin pressures...
  rotTVT,  //PRELIMINARY OR OBSOLETE
  userTVT      ///< fully user defined: derive from TaskVariable and overload userUpdate(...)
};

enum TargetType { noneTT, directTT, positionGainsTT, pdGainOnRealTT, pdGainOnReferenceTT, trajectoryTT };

//===========================================================================
/** basic task variable */
struct TaskVariable {
  /// @name data fields
  bool active;          ///< active?
  TVtype type;          ///< which type has this variable (arguably: this could be member of DefaultTV -- but useful here)
  TargetType targetType;///< what target type
  rai::String name;      ///< its name

  arr y, y_old, v, v_old, y_target, v_target; ///< current state and final target of this variable
  arr J, Jt;                                  ///< current Jacobian and its transpose
  double y_prec, v_prec;                      ///< precision (=1/variance) associated with this variable
  arr y_trajectory, y_prec_trajectory;        ///< target & precision over a whole trajectory
  arr v_trajectory, v_prec_trajectory;        ///< target & precision over a whole trajectory

  //used for feedback control:
  arr y_ref, v_ref;                           ///< immediate (next step) desired target reference
  double Pgain, Dgain;                        ///< parameters of the PD controller or attractor dynamics

  //a bit obsolete
  double err, derr;

  /// @name initialization
  TaskVariable();
  virtual ~TaskVariable() = 0;
  virtual TaskVariable* newClone() = 0;

  /// @name online target parameters
  void setGains(double Pgain, double Dgain, bool onReal=true);
  void setGainsAsNatural(double decaySteps, double dampingRatio, bool onReal=true);
  void setGainsAsAttractor(double decaySteps, double oscillations=.2, bool onReal=true);

  /// @name trajectory target parameters
  /// @todo REMOVE ALL of the following options:
  void setConstantTargetTrajectory(uint T);
  void setInterpolatedTargetTrajectory(uint T);
  void setPrecisionTrajectoryFinal(uint T, double intermediate_prec, double final_prec);
  void setPrecisionTrajectoryConstant(uint T, double constant_prec);
  void setPrecisionVTrajectoryFinal(uint T, double intermediate_prec, double final_prec);
  void setPrecisionVTrajectoryConstant(uint T, double constant_prec);
  void setIntervalPrecisions(uint T, arr& y_precs, arr& v_precs);
  void setTrajectory(uint T, double funnelsdv=0., double funnelvsdv=0.); //OBSOLETE

  //only keep those:
  void setInterpolatedTargetsEndPrecisions(uint T, double mid_y_prec, double final_y_prec, double mid_v_prec, double final_v_prec);
  void setInterpolatedTargetsConstPrecisions(uint T, double y_prec, double v_prec);
  void setConstTargetsConstPrecisions(uint T, double y_prec, double v_prec);

  void setInterpolatedTargetsEndPrecisions(uint T, double mid_y_prec, double mid_v_prec); //those versions assume y_prec and v_prec were set and use this.
  void setInterpolatedTargetsConstPrecisions(uint T);
  void setConstTargetsConstPrecisions(uint T);
  void appendConstTargetsAndPrecs(uint T);

  void shiftTargets(int offset);

  /// @name updates
  virtual void updateState(const rai::Configuration& ors, double tau=1.) = 0; //updates both, state and Jacobian -> TODO: rename update(..)
  void updateChange(int t=-1, double tau=1.);
  virtual void getHessian(const rai::Configuration& ors, arr& H) { NIY; }

  /// @name I/O
  virtual void write(ostream& os, const rai::Configuration& ors) const;
  void write(ostream& os) const {NIY};
};
stdOutPipe(TaskVariable);

//===========================================================================
/** The default implementation of standard task variables. */
struct DefaultTaskVariable:public TaskVariable {
  /// @name data fields
  int i, j;             ///< which body(-ies) does it refer to?
  rai::Transformation irel, jrel; ///< relative position to the body
  arr params;           ///< parameters of the variable (e.g., liner coefficients, limits, etc)

  /// @name initialization
  DefaultTaskVariable();
  DefaultTaskVariable(
    const char* _name,
    const rai::Configuration& _ors,
    TVtype _type,
    const char* iBodyName, const char* iframe,
    const char* jBodyName, const char* jframe,
    const arr& _params);
  DefaultTaskVariable(
    const char* _name,
    const rai::Configuration& _ors,
    TVtype _type,
    const char* iShapeName,
    const char* jShapeName,
    const arr& _params);
  ~DefaultTaskVariable();
  TaskVariable* newClone() { return new DefaultTaskVariable(*this); }

  void set(
    const char* _name,
    const rai::Configuration& _ors,
    TVtype _type,
    int _i, const rai::Transformation& _irel,
    int _j, const rai::Transformation& _jrel,
    const arr& _params);
  //void set(const char* _name, rai::Configuration& _ors, TVtype _type, const char *iname, const char *jname, const char *reltext);

  /// @name updates
  void updateState(const rai::Configuration& ors, double tau=1.);
  void getHessian(const rai::Configuration& ors, arr& H);

  /// @name virtual user update
  virtual void userUpdate(const rai::Configuration& ors) { NIY; } //updates both, state and Jacobian

  /// @name I/O
  void write(ostream& os, const rai::Configuration& ors) const;
};
//stdOutPipe(DefaultTaskVariable);

//===========================================================================
/** Collision Task Variable. */
enum CTVtype {
  allCTVT,
  allListedCTVT,
  allExceptListedCTVT,
  bipartiteCTVT,
  pairsCTVT,
  allExceptPairsCTVT,
  vectorCTVT
};

/** Proxy task variable */
struct ProxyTaskVariable:public TaskVariable {
  /// @name data fields
  CTVtype type;
  uintA shapes, shapes2;
  double margin;
  bool linear;

  /// @name initialization
  ProxyTaskVariable();
  ProxyTaskVariable(const char* _name,
                    rai::Configuration& _ors,
                    CTVtype _type,
                    uintA _shapes,
                    double _margin=.02,
                    bool _linear=false);
  TaskVariable* newClone() { return new ProxyTaskVariable(*this); }

  /// @name updates
  void updateState(const rai::Configuration& ors, double tau=1.);
};

/** proxy align task variable */
struct ProxyAlignTaskVariable:public TaskVariable {
  /// @name data fields
  CTVtype type;
  uintA shapes, shapes2;
  double margin;
  bool linear;

  /// @name initialization
  ProxyAlignTaskVariable();
  ProxyAlignTaskVariable(const char* _name,
                         rai::Configuration& _ors,
                         CTVtype _type,
                         uintA _shapes,
                         double _margin=3.,
                         bool _linear=true);
  TaskVariable* newClone() { return new ProxyAlignTaskVariable(*this); }

  /// @name updates
  void updateState(const rai::Configuration& ors, double tau=1.);
};

//===========================================================================
/**
 * @name task variable lists
 * @{
 */
typedef rai::Array<TaskVariable*> TaskVariableList;

void reportAll(TaskVariableList& CS, ostream& os, bool onlyActives=true);
void reportState(TaskVariableList& CS, ostream& os, bool onlyActives=true);
void reportErrors(TaskVariableList& CS, ostream& os, bool onlyActives=true, int t=-1);
void reportNames(TaskVariableList& CS, ostream& os, bool onlyActives=true);
void activateAll(TaskVariableList& CS, bool active);
void updateState(TaskVariableList& CS, const rai::Configuration& ors);
void updateChanges(TaskVariableList& CS, int t=-1);
void getJointJacobian(TaskVariableList& CS, arr& J);
void getJointYchange(TaskVariableList& CS, arr& y_change);
void shiftTargets(TaskVariableList& CS, int i);
void bayesianControl(TaskVariableList& CS, arr& dq, const arr& W);

#endif // _HEADER_GUARD_ORS_OLDTASKVARIABLES_H_

