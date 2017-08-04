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

namespace mlr {

struct KinematicWorld;

struct F_Link {
  int type;
  int qIndex;
  int parent;
  mlr::Transformation X, A, Q;
  mlr::Vector com, force, torque;
  double mass;
  mlr::Matrix inertia;
  uint dof();

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


typedef Array<mlr::F_Link> F_LinkTree;

void equationOfMotion(arr& M, arr& F, const F_LinkTree& tree,  const arr& qd);
void fwdDynamics_MF(arr& qdd, const F_LinkTree& tree, const arr& qd, const arr& u);
void fwdDynamics_aba_nD(arr& qdd, const F_LinkTree& tree, const arr& qd, const arr& tau);
void fwdDynamics_aba_1D(arr& qdd, const F_LinkTree& tree, const arr& qd, const arr& tau);
void invDynamics(arr& tau, const F_LinkTree& tree, const arr& qd, const arr& qdd);

} //namespace mlr

stdOutPipe(mlr::F_Link)

void GraphToTree(mlr::F_LinkTree& tree, const mlr::KinematicWorld& C);
void updateGraphToTree(mlr::F_LinkTree& tree, const mlr::KinematicWorld& C);

#endif
