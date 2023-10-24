/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_AlignStacking.h"
#include "frame.h"

TM_AlignStacking::TM_AlignStacking(int iShape)
  : i(iShape) {
}

TM_AlignStacking::TM_AlignStacking(const rai::Configuration& G, const char* iShapeName)
  :i(-1) {
  rai::Frame* a = iShapeName ? G.getFrameByName(iShapeName):nullptr;
  if(a) i=a->ID;
}

void TM_AlignStacking::phi(arr& y, arr& J, const rai::Configuration& G) {
  rai::Frame* b=G.frames(i);

  rai::Joint* j=b->joint;
  CHECK(j, "has no support??");

  rai::Frame* b_support = j->from();

#if 0//if there were multiple supporters
  uint n=G.getJointStateDimension();
  //-- compute the center of all supporters (here just one..)
  arr cen = zeros(3), cenJ = zeros(3, n);
  {
    arr y, J;
    G.kinematicsPos(y, J, b_support);
    cen += y;
    cenJ += J;
  }
  cen  /= (double)supporters.N;
  cenJ /= (double)supporters.N;

  //-- max distances to center
  prec=3e-1;
  for(Node* s:supporters) {
    b=effKinematics.getBodyByName(s->keys.last());
    effKinematics.kinematicsPos(y, J, b);
    y -= cen;
    double d = length(y);
    arr normal = y/d;
    phi.append(prec*(1.-d));
    if(!!phiJ) phiJ.append(prec*(~normal*(-J+cenJ)));
    if(!!tt) tt.append(OT_sos, 1);
  }

  //-- align center with object center
  prec=1e-1;
  b=effKinematics.getBodyByName(obj->keys.last());
  effKinematics.kinematicsPos(y, J, b);
  phi.append(prec*(y-cen));
  if(!!phiJ) phiJ.append(prec*(J-cenJ));
  if(!!tt) tt.append(OT_sos, 3);
#else //just one supporter

  arr y1, J1, y2, J2;
//    if(verbose>1){ cout <<"Adding cost term Object" <<*obj <<" below "; listWrite(supporters, cout); cout <<endl; }
  G.kinematicsPos(y1, J1, b);
  G.kinematicsPos(y2, J2, b_support);
  y = (y1-y2)({0, 1}); //only x-y-position
  if(!!J) J = (J1-J2)({0, 1});

#endif
}

rai::String TM_AlignStacking::shortTag(const rai::Configuration& G) {
  return STRING("AlignStacking:"<<(i<0?"WORLD":G.frames(i)->name));
}
