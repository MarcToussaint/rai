#include "TM_Align.h"
#include "frame.h"

TM_Align::TM_Align(const mlr::KinematicWorld& K, const char* iName, const char* jName)
  : i(-1), j(-1){
  mlr::Frame *a = iName ? K.getFrameByName(iName):NULL;
  mlr::Frame *b = jName ? K.getFrameByName(jName):NULL;
  if(a) i=a->ID;
  if(b) j=b->ID;
}

void TM_Align::phi(arr& y, arr& J, const mlr::KinematicWorld& K, int t){
  y.resize(3);
  if(&J) J.resize(3, K.q.N);

  mlr::Frame* body_i = K.frames(i);
  mlr::Frame* body_j = K.frames(j);

  arr zi,Ji,zj,Jj;

  K.kinematicsVec(zi, Ji, body_i, Vector_x);
  K.kinematicsVec(zj, Jj, body_j, Vector_x);
  y(0) = scalarProduct(zi, zj);
  if(&J) J[0] = ~zj * Ji + ~zi * Jj;

  K.kinematicsVec(zi, Ji, body_i, Vector_y);
  K.kinematicsVec(zj, Jj, body_j, Vector_y);
  y(1) = scalarProduct(zi, zj);
  if(&J) J[1] = ~zj * Ji + ~zi * Jj;

  K.kinematicsVec(zi, Ji, body_i, Vector_z);
  K.kinematicsVec(zj, Jj, body_j, Vector_z);
  y(2) = scalarProduct(zi, zj);
  if(&J) J[2] = ~zj * Ji + ~zi * Jj;
}

mlr::String TM_Align::shortTag(const mlr::KinematicWorld &G){
  return STRING("TM_Align:"<<(i<0?"WORLD":G.frames(i)->name) <<':' <<(j<0?"WORLD":G.frames(j)->name));
}
