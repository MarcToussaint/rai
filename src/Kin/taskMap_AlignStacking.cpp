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


#include "taskMap_AlignStacking.h"

TaskMap_AlignStacking::TaskMap_AlignStacking(int iShape)
  : i(iShape){
}


TaskMap_AlignStacking::TaskMap_AlignStacking(const mlr::KinematicWorld& G, const char* iShapeName)
  :i(-1){
  mlr::Shape *a = iShapeName ? G.getShapeByName(iShapeName):NULL;
  if(a) i=a->index;
}

void TaskMap_AlignStacking::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  mlr::Shape *s=G.shapes(i);
  mlr::Body *b=s->body;

  mlr::Joint *j=b->inLinks.first();
  CHECK(j,"has no support??");

  mlr::Body *b_support=j->from;

#if 0//if there were multiple supporters
  uint n=G.getJointStateDimension();
  //-- compute the center of all supporters (here just one..)
  arr cen = zeros(3), cenJ = zeros(3,n);
  {
    arr y,J;
    G.kinematicsPos(y, J, b_support);
    cen += y;
    cenJ += J;
  }
  cen  /= (double)supporters.N;
  cenJ /= (double)supporters.N;

  //-- max distances to center
  prec=3e-1;
  for(Node *s:supporters){
    b=effKinematics.getBodyByName(s->keys.last());
    effKinematics.kinematicsPos(y, J, b);
    y -= cen;
    double d = length(y);
    arr normal = y/d;
    phi.append( prec*(1.-d) );
    if(&phiJ) phiJ.append( prec*(~normal*(-J+cenJ)) );
    if(&tt) tt.append(OT_sumOfSqr, 1);
  }

  //-- align center with object center
  prec=1e-1;
  b=effKinematics.getBodyByName(obj->keys.last());
  effKinematics.kinematicsPos(y, J, b);
  phi.append( prec*(y-cen) );
  if(&phiJ) phiJ.append( prec*(J-cenJ) );
  if(&tt) tt.append(OT_sumOfSqr, 3);
#else //just one supporter

  arr y1,J1,y2,J2;
//    if(verbose>1){ cout <<"Adding cost term Object" <<*obj <<" below "; listWrite(supporters, cout); cout <<endl; }
  G.kinematicsPos(y1, J1, b);
  G.kinematicsPos(y2, J2, b_support);
  y = (y1-y2)({0,1}); //only x-y-position
  if(&J) J = (J1-J2)({0,1});

#endif
}
