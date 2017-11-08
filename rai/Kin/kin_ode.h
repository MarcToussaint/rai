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


#pragma once

#include "kin.h"

//===========================================================================
/**
 * @defgroup ors_interface_ode ODE interface
 * @{
 */

struct dxWorld;   /* dynamics world */
struct dxSpace;   /* collision space */
struct dxBody;    /* rigid body (dynamics object) */
struct dxGeom;    /* geometry (collision object) */
struct dxJoint;
struct dxJointNode;
struct dxJointGroup;
struct dContactGeom;

typedef struct dxWorld *dWorldID;
typedef struct dxSpace *dSpaceID;
typedef struct dxBody *dBodyID;
typedef struct dxGeom *dGeomID;
typedef struct dxJoint *dJointID;
typedef struct dxJointGroup *dJointGroupID;

/** A trivial interface to the Open Dynamic Engine library.
 *
 * It basically contains a dSpace and dWorld, provides a proximity
 * callback function, and basic stepping function.
 */
struct OdeInterface {
  mlr::KinematicWorld& C;
  double time;
  dxSpace *space;
  dxGeom *plane0, *planex1, *planex2, *planey1, *planey2;
  dxWorld *world;
  dxJointGroup *contactgroup;
  double ERP, CFM; //integration parameters
  double coll_ERP, coll_CFM, coll_bounce, friction; //collision parameter
  bool noGravity, noContactJoints;

  mlr::Array<dxBody*> bodies;
  mlr::Array<dxGeom*> geoms;
  mlr::Array<dxJoint*> joints;
  mlr::Array<dxJoint*> motors;
  mlr::Array<dContactGeom*> conts;

  OdeInterface(mlr::KinematicWorld &_C);
  ~OdeInterface();

  /** @brief reinstantiates a new ODE world (and space) clear of all previous objects */
  void clear();

  /**
   * This function is called from the `dSpaceCollide' routine (in the
   * `step' routine) when two objects get too close.
   *
   * A "collision-joint" is inserted between them that exerts the force
   * of the collision. All of these collision-joints are collected in
   * a group, and they are deleted after the `dWorldStep' by the
   * `dJoinGroupEmpty' routine (in the `step' routine).
   */
  static void staticCallback(void *classP, dxGeom *g1, dxGeom *g2);

  /// sets gravity to zero (or back to -9.81)
  void setForceFree(bool free);

  /** @brief main method: process one time step by calling SpaceCollide and WorldQuickStep */
  void step(double dtime=.01);

  void printInfo(std::ostream& os, dxBody *b);
  void reportContacts();
  void contactForces();
  void penetration(mlr::Vector &p);

  void exportStateToOde();
  void importStateFromOde();
  void exportForcesToOde();
  void addJointForce(mlr::Joint *e, double f1, double f2);
  void addJointForce(arr& f);
  void setMotorVel(const arr& qdot, double maxF);
  uint getJointMotorDimension();
  void setJointMotorPos(arr& x, double maxF=1., double tau=.01);
  void setJointMotorPos(mlr::Joint *e, double x0, double maxF=1., double tau=.01);
  void setJointMotorVel(arr& v, double maxF=1.);
  void setJointMotorVel(mlr::Joint *e, double v0, double maxF=1.);
  void unsetJointMotors();
  void unsetJointMotor(mlr::Joint *e);
  void getJointMotorForce(arr& f);
  void getJointMotorForce(mlr::Joint *e, double& f);
  void pidJointPos(mlr::Joint *e, double x0, double v0, double xGain, double vGain, double iGain=0, double* eInt=0);
  void pidJointVel(mlr::Joint *e, double v0, double vGain);
  void getGroundContact(boolA& cts);
  void importProxiesFromOde();
  void step(arr& force, uint steps=1, double tau=.01);
  void step(uint steps=1, double tau=.01);
  void slGetProxies();
  //void slGetProxyGradient(arr &dx, const arr &x);
  void reportContacts2();
  bool inFloorContacts(mlr::Vector& x);
  void pushPoseForShape(mlr::Shape *s);
};

/** @} */
