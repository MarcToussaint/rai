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


#include "taskMap_InsideBox.h"
#include "frame.h"

TaskMap_InsideBox::TaskMap_InsideBox(int iShape, int jShape)
  : i(iShape), j(jShape), margin(.01){
}

TaskMap_InsideBox::TaskMap_InsideBox(const mlr::KinematicWorld& G, const char* iShapeName, const mlr::Vector &_ivec, const char* jShapeName, double _margin)
  :i(-1), j(-1), margin(_margin){
  mlr::Frame *a = iShapeName ? G.getFrameByName(iShapeName):NULL;
  mlr::Frame *b = jShapeName ? G.getFrameByName(jShapeName):NULL;
  if(a) i=a->ID;
  if(b) j=b->ID;
  if(&_ivec) ivec=_ivec; else ivec.setZero();
}

void TaskMap_InsideBox::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  mlr::Shape *pnt=G.frames(i)->shape;
  mlr::Shape *box=G.frames(j)->shape;
  CHECK(pnt && box,"I need shapes!");
  CHECK(box->type==mlr::ST_ssBox || box->type==mlr::ST_box,"the 2nd shape needs to be a box"); //s1 should be the board
  arr pos,posJ;
  G.kinematicsRelPos(pos, posJ, pnt->frame, ivec, box->frame, NoVector);
  arr range = box->size;
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

uint TaskMap_InsideBox::dim_phi(const mlr::KinematicWorld &G){ return 6; }

mlr::String TaskMap_InsideBox::shortTag(const mlr::KinematicWorld &G){
    return STRING("InsideBox:"<<(i<0?"WORLD":G.frames(i)->name) <<':' <<(j<0?"WORLD":G.frames(j)->name));
}
