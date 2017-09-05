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

TM_StaticStability::TM_StaticStability(int iShape, int jShape)
  : i(iShape), j(jShape), margin(.01){
}

TM_StaticStability::TM_StaticStability(const mlr::KinematicWorld& G, const char* iShapeName, const char* jShapeName)
  :i(-1), j(-1), margin(.01){
  mlr::Frame *a = iShapeName ? G.getFrameByName(iShapeName):NULL;
  mlr::Frame *b = jShapeName ? G.getFrameByName(jShapeName):NULL;
  if(a) i=a->ID;
  if(b) j=b->ID;
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
  uint n=0;
  for(mlr::Frame *b:aboves) /*if(b!=a)*/{
    arr y,J;
    K.kinematicsPos(y, J, b);
    cog += y;
    J_cog += J;
    n++;
  }
  cog  /= (double)n;
  J_cog /= (double)n;


  //align avg with object center
  K.kinematicsPos(y, J, a);
  y = (y-cog)({0,1});
  if(&J) J=(J-J_cog)({0,1});
}

mlr::String TM_StaticStability::shortTag(const mlr::KinematicWorld &K){
  return STRING("StaticStability:"<<(i<0?"WORLD":K.frames(i)->name) <<':' <<(j<0?"WORLD":K.frames(j)->name));
}
