/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef MLR_kin_feather_h
#define MLR_kin_feather_h

#include <Geo/geo.h>
#include <Kin/kin.h>

struct F_Link {
  int ID=-1;
  int type=-1;
  int qIndex=-1;
  int parent=-1;
  mlr::Transformation X=0, Q=0;
  mlr::Vector com=0, force=0, torque=0;
  double mass=0.;
  mlr::Matrix inertia=0;
  uint dof();

  arr _h, _Q, _I, _f; //featherstone types

  F_Link(){}
  void setFeatherstones();
  void updateFeatherstones();
  void write(ostream& os) const {
    os <<"*type=" <<type <<" index=" <<qIndex <<" parent=" <<parent <<endl
       <<" XQ,Q=" <<X <<", " <<Q <<endl
       <<" cft=" <<com <<force <<torque <<endl
       <<" mass=" <<mass <<inertia <<endl;
  }
};
stdOutPipe(F_Link)

typedef mlr::Array<F_Link> F_LinkTree;

struct FeatherstoneInterface{
  mlr::KinematicWorld& K;

  mlr::Array<F_Link> tree;

  FeatherstoneInterface(mlr::KinematicWorld& K):K(K){}

  void update();

  void equationOfMotion(arr& M, arr& F,  const arr& qd);
  void fwdDynamics_MF(arr& qdd, const arr& qd, const arr& u);
  void fwdDynamics_aba_nD(arr& qdd, const arr& qd, const arr& tau);
  void fwdDynamics_aba_1D(arr& qdd, const arr& qd, const arr& tau);
  void invDynamics(arr& tau, const arr& qd, const arr& qdd);
};


#endif
