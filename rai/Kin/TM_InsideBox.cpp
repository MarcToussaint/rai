/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_InsideBox.h"
#include "frame.h"

TM_InsideBox::TM_InsideBox(int iShape, int jShape)
  : i(iShape), j(jShape), margin(.01){
}

TM_InsideBox::TM_InsideBox(const mlr::KinematicWorld& G, const char* iShapeName, const mlr::Vector &_ivec, const char* jShapeName, double _margin)
  :i(-1), j(-1), margin(_margin){
  mlr::Frame *a = iShapeName ? G.getFrameByName(iShapeName):NULL;
  mlr::Frame *b = jShapeName ? G.getFrameByName(jShapeName):NULL;
  if(a) i=a->ID;
  if(b) j=b->ID;
  if(&_ivec) ivec=_ivec; else ivec.setZero();
}

void TM_InsideBox::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  mlr::Shape *pnt=G.frames(i)->shape;
  mlr::Shape *box=G.frames(j)->shape;
  CHECK(pnt && box,"I need shapes!");
  CHECK(box->type()==mlr::ST_ssBox || box->type()==mlr::ST_box,"the 2nd shape needs to be a box"); //s1 should be the board
  arr pos,posJ;
  G.kinematicsRelPos(pos, posJ, &pnt->frame, ivec, &box->frame, NoVector);
  arr range = box->size();
  range *= .5;
  range -= margin;
  for(double& r:range) if(r<.01) r=.01;

  y.resize(6);
  y(0) =  pos(0) - range(0);
  y(1) = -pos(0) - range(0);
  y(2) =  pos(1) - range(1);
  y(3) = -pos(1) - range(1);
  y(4) =  pos(2) - range(2);
  y(5) = -pos(2) - range(2);
  if(&J){
    J.resize(6, posJ.d1);
    J[0] =  posJ[0];
    J[1] = -posJ[0];
    J[2] =  posJ[1];
    J[3] = -posJ[1];
    J[4] =  posJ[2];
    J[5] = -posJ[2];
  }
}

uint TM_InsideBox::dim_phi(const mlr::KinematicWorld &G){ return 6; }

mlr::String TM_InsideBox::shortTag(const mlr::KinematicWorld &G){
    return STRING("InsideBox:"<<(i<0?"WORLD":G.frames(i)->name) <<':' <<(j<0?"WORLD":G.frames(j)->name));
}
