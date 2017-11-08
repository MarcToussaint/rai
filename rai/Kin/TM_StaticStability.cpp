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


#include "TM_StaticStability.h"
#include "frame.h"

TM_StaticStability::TM_StaticStability(int iShape, double _margin)
  : i(iShape), margin(_margin){
}

TM_StaticStability::TM_StaticStability(const mlr::KinematicWorld& G, const char* iShapeName, double _margin)
  :i(-1), margin(_margin){
  mlr::Frame *a = iShapeName ? G.getFrameByName(iShapeName):NULL;
  if(a) i=a->ID;
}

FrameL getShapesAbove(mlr::Frame *a){
  FrameL aboves;
  if(a->shape) aboves.append(a);
  for(mlr::Frame *b:a->outLinks) aboves.append(getShapesAbove(b));
  return aboves;
}

void TM_StaticStability::phi(arr& y, arr& J, const mlr::KinematicWorld& K, int t){
  //get shapes above
  mlr::Frame *a = K.frames(i);
  FrameL aboves = getShapesAbove(a);
//  cout <<"ABOVES="<<endl; listWrite(aboves);

  //get average center of all shapes
  arr cog(3) ,J_cog(3, K.getJointStateDimension());
  cog.setZero(); J_cog.setZero();
  double M=0.;
  for(mlr::Frame *b:aboves) if(b!=a){
    double mass=0.;
    if(b->shape) mass=1.;
    if(b->inertia) mass=b->inertia->mass;
    arr y,J;
    K.kinematicsPos(y, J, b);
    cog += mass*y;
    J_cog += mass*J;
    M += mass;
  }
  CHECK(M>0., "");
  cog  /= M;
  J_cog /= M;

  //align avg with object center
  K.kinematicsPos(y, J, a);
  y = (y-cog)({0,1});
  if(&J) J=(J-J_cog)({0,1});

#if 1
  CHECK(a->shape, "");
  CHECK(a->shape->type()==mlr::ST_ssBox, "the supporting shape needs to be a box");
  arr range = { .5*a->shape->size(0)-margin, .5*a->shape->size(1)-margin };
  arr pos=y, posJ=J;

  y.resize(4);
  y(0) =  pos(0) - range(0);
  y(1) = -pos(0) - range(0);
  y(2) =  pos(1) - range(1);
  y(3) = -pos(1) - range(1);
  if(&J){
    J.resize(4, posJ.d1);
    J[0] =  posJ[0];
    J[1] = -posJ[0];
    J[2] =  posJ[1];
    J[3] = -posJ[1];
  }
#endif
}

mlr::String TM_StaticStability::shortTag(const mlr::KinematicWorld &K){
  return STRING("StaticStability:"<<(i<0?"WORLD":K.frames(i)->name));
}
