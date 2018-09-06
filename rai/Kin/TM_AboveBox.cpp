/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_AboveBox.h"
#include "frame.h"

TM_AboveBox::TM_AboveBox(int iShape, int jShape, double _margin)
  : i(iShape), j(jShape), margin(_margin) {
}

TM_AboveBox::TM_AboveBox(const rai::KinematicWorld& K, const char* iShapeName, const char* jShapeName, double _margin)
  :i(-1), j(-1), margin(_margin) {
  rai::Frame *a = iShapeName ? K.getFrameByName(iShapeName):NULL;
  rai::Frame *b = jShapeName ? K.getFrameByName(jShapeName):NULL;
  if(a) i=a->ID;
  if(b) j=b->ID;
}

void TM_AboveBox::phi(arr& y, arr& J, const rai::KinematicWorld& K) {
  rai::Shape *pnt=K.frames(i)->shape;
  rai::Shape *box=K.frames(j)->shape;
  CHECK(pnt && box,"I need shapes!");
//  if(box->type!=rai::ST_ssBox){ //switch roles
//    rai::Shape *z=pnt;
//    pnt=box; box=z;
//  }
  CHECK_EQ(box->type(), rai::ST_ssBox,"the 2nd shape needs to be a box"); //s1 should be the board
  arr pos,posJ;
  K.kinematicsRelPos(pos, posJ, &pnt->frame, NoVector, &box->frame, NoVector);
#if 0
  arr range(3);
  double d1 = .5*pnt->size(0) + pnt->size(3);
  d1 =.05; //TODO: fixed! support size/radius of object on top
  double d2 = .5*box->size(0) + box->size(3);
  range(0) = fabs(d1 - d2);
  d1 = .5*pnt->size(1) + pnt->size(3);
  d1 =.05; //TODO: fixed! support size/radius of object on top
  d2 = .5*box->size(1) + box->size(3);
  range(1) = fabs(d1 - d2);
  range(2)=0.;
#else
  arr range = { .5*box->size(0)-margin, .5*box->size(1)-margin };
#endif
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
  if(!!J) {
    J.resize(4, posJ.d1);
    J[0] =  posJ[0];
    J[1] = -posJ[0];
    J[2] =  posJ[1];
    J[3] = -posJ[1];
  }
}

rai::String TM_AboveBox::shortTag(const rai::KinematicWorld &G) {
    return STRING("AboveBox:"<<(i<0?"WORLD":G.frames(i)->name) <<':' <<(j<0?"WORLD":G.frames(j)->name));
}

Graph TM_AboveBox::getSpec(const rai::KinematicWorld& K){
    return Graph({ {"feature", "above"}, {"o1", K.frames(i)->name}, {"o2", K.frames(j)->name}});
}
