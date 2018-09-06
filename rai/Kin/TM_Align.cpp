/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_Align.h"
#include "frame.h"

TM_Align::TM_Align(const rai::KinematicWorld& K, const char* iName, const char* jName)
  : i(-1), j(-1) {
  rai::Frame *a = iName ? K.getFrameByName(iName):NULL;
  rai::Frame *b = jName ? K.getFrameByName(jName):NULL;
  if(a) i=a->ID;
  if(b) j=b->ID;
}

void TM_Align::phi(arr& y, arr& J, const rai::KinematicWorld& K) {
  y.resize(3);
  if(!!J) J.resize(3, K.q.N);
  
  rai::Frame* body_i = K.frames(i);
  rai::Frame* body_j = K.frames(j);
  
  arr zi,Ji,zj,Jj;
  
  K.kinematicsVec(zi, Ji, body_i, Vector_z);
  K.kinematicsVec(zj, Jj, body_j, Vector_x);
  y(0) = scalarProduct(zi, zj);
  if(!!J) J[0] = ~zj * Ji + ~zi * Jj;
  
  K.kinematicsVec(zi, Ji, body_i, Vector_z);
  K.kinematicsVec(zj, Jj, body_j, Vector_y);
  y(1) = scalarProduct(zi, zj);
  if(!!J) J[1] = ~zj * Ji + ~zi * Jj;
  
  K.kinematicsVec(zi, Ji, body_i, Vector_y);
  K.kinematicsVec(zj, Jj, body_j, Vector_x);
  y(2) = scalarProduct(zi, zj);
  if(!!J) J[2] = ~zj * Ji + ~zi * Jj;
}

rai::String TM_Align::shortTag(const rai::KinematicWorld &G) {
  return STRING("TM_Align:"<<(i<0?"WORLD":G.frames(i)->name) <<':' <<(j<0?"WORLD":G.frames(j)->name));
}
