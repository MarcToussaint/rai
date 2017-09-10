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
