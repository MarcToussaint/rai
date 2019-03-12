/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_StaticStability.h"
#include "frame.h"

TM_StaticStability::TM_StaticStability(int iShape, double _margin)
  : i(iShape), margin(_margin) {
}

TM_StaticStability::TM_StaticStability(const rai::KinematicWorld& G, const char* iShapeName, double _margin)
  :i(-1), margin(_margin) {
  rai::Frame *a = iShapeName ? G.getFrameByName(iShapeName):NULL;
  if(a) i=a->ID;
}

FrameL getShapesAbove(rai::Frame *a) {
  FrameL aboves;
  if(a->shape) aboves.append(a);
  for(rai::Frame *b:a->parentOf) aboves.append(getShapesAbove(b));
  return aboves;
}

void TM_StaticStability::phi(arr& y, arr& J, const rai::KinematicWorld& K) {
  //get shapes above
  rai::Frame *a = K.frames(i);
  FrameL aboves = getShapesAbove(a);
//  cout <<"ABOVES="<<endl; listWrite(aboves);

  //get average center of all shapes
  arr cog(3) ,J_cog(3, K.getJointStateDimension());
  cog.setZero(); J_cog.setZero();
  double M=0.;
  for(rai::Frame *b:aboves) if(b!=a) {
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
  if(!!J) J=(J-J_cog)({0,1});
  
#if 1
  CHECK(a->shape, "");
  CHECK_EQ(a->shape->type(), rai::ST_ssBox, "the supporting shape needs to be a box");
  arr range = { .5*a->shape->size(0)-margin, .5*a->shape->size(1)-margin };
  arr pos=y, posJ=J;
  
  y.resize(4);
  y(0) =  pos(0) - range(0);
  y(1) = -pos(0) - range(0);
  y(2) =  pos(1) - range(1);
  y(3) = -pos(1) - range(1);
  if(!!J) {
    J.resize(4, posJ.d1);
    J[0] =  posJ[0];
    J[1] = -posJ[0];
    J[2] =  posJ[1];
    J[3] = -posJ[1];
  }
#endif
}

rai::String TM_StaticStability::shortTag(const rai::KinematicWorld &K) {
  return STRING("StaticStability:"<<(i<0?"WORLD":K.frames(i)->name));
}
