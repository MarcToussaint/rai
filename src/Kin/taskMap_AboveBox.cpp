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


#include "taskMap_AboveBox.h"
#include "frame.h"

TaskMap_AboveBox::TaskMap_AboveBox(int iShape, int jShape)
  : i(iShape), j(jShape){
}


TaskMap_AboveBox::TaskMap_AboveBox(const mlr::KinematicWorld& G, const char* iShapeName, const char* jShapeName)
  :i(-1), j(-1){
  mlr::Frame *a = iShapeName ? G.getFrameByName(iShapeName):NULL;
  mlr::Frame *b = jShapeName ? G.getFrameByName(jShapeName):NULL;
  if(a) i=a->ID;
  if(b) j=b->ID;
}

void TaskMap_AboveBox::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  mlr::Shape *s1=G.frames(i)->shape;
  mlr::Shape *s2=G.frames(j)->shape;
  CHECK(s1 && s2,"I need shapes!");
  if(s2->type!=mlr::ST_ssBox){ //switch roles
    mlr::Shape *z=s1;
    s1=s2; s2=z;
  }
  CHECK(s2->type==mlr::ST_ssBox,"");//s1 should be the board
  arr pos,posJ;
  G.kinematicsRelPos(pos, posJ, s1->frame, NoVector, s2->frame, NoVector);
  arr range(3);
  double d1 = .5*s1->size(0) + s1->size(3);
  d1 =.05; //TODO: fixed! support size/radius of object on top
  double d2 = .5*s2->size(0) + s2->size(3);
  range(0) = fabs(d1 - d2);
  d1 = .5*s1->size(1) + s1->size(3);
  d1 =.05; //TODO: fixed! support size/radius of object on top
  d2 = .5*s2->size(1) + s2->size(3);
  range(1) = fabs(d1 - d2);
  range(2)=0.;
//  if(verbose>2) cout <<pos <<range
//                    <<pos-range <<-pos-range
//                   <<"\n 10=" <<s1->size(0)
//                  <<" 20=" <<s2->size(0)
//                 <<" 11=" <<s1->size(1)
//                <<" 21=" <<s2->size(1)
//               <<endl;
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
}

mlr::String TaskMap_AboveBox::shortTag(const mlr::KinematicWorld &G){
  return STRING("AboveBox:"<<(i<0?"WORLD":G.frames(i)->name) <<':' <<(j<0?"WORLD":G.frames(j)->name));
}
