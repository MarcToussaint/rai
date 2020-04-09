/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_geometrics.h"
#include "F_PairCollision.h"
#include "TM_default.h"
#include "frame.h"
//===========================================================================

TM_AboveBox::TM_AboveBox(int iShape, int jShape, double _margin)
  : i(iShape), j(jShape), margin(_margin) {
}

TM_AboveBox::TM_AboveBox(const rai::Configuration& K, const char* iShapeName, const char* jShapeName, double _margin)
  :i(-1), j(-1), margin(_margin) {
  rai::Frame* a = iShapeName ? K.getFrameByName(iShapeName):nullptr;
  rai::Frame* b = jShapeName ? K.getFrameByName(jShapeName):nullptr;
  if(a) i=a->ID;
  if(b) j=b->ID;
}

void TM_AboveBox::phi(arr& y, arr& J, const rai::Configuration& K) {
  rai::Shape* pnt=K.frames(i)->shape;
  rai::Shape* box=K.frames(j)->shape;
  CHECK(pnt && box, "I need shapes!");
//  if(box->type!=rai::ST_ssBox){ //switch roles
//    rai::Shape *z=pnt;
//    pnt=box; box=z;
//  }
  CHECK_EQ(box->type(), rai::ST_ssBox, "the 2nd shape needs to be a box"); //s1 should be the board
  arr pos, posJ;
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

rai::String TM_AboveBox::shortTag(const rai::Configuration& G) {
  return STRING("AboveBox:"<<(i<0?"WORLD":G.frames(i)->name) <<':' <<(j<0?"WORLD":G.frames(j)->name));
}

rai::Graph TM_AboveBox::getSpec(const rai::Configuration& K) {
  return rai::Graph({ {"feature", "above"}, {"o1", K.frames(i)->name}, {"o2", K.frames(j)->name}});
}

//===========================================================================

TM_InsideBox::TM_InsideBox(int iShape, int jShape)
  : i(iShape), j(jShape), margin(.01) {
}

TM_InsideBox::TM_InsideBox(const rai::Configuration& G, const char* iShapeName, const rai::Vector& _ivec, const char* jShapeName, double _margin)
  :i(-1), j(-1), margin(_margin) {
  rai::Frame* a = iShapeName ? G.getFrameByName(iShapeName):nullptr;
  rai::Frame* b = jShapeName ? G.getFrameByName(jShapeName):nullptr;
  if(a) i=a->ID;
  if(b) j=b->ID;
  if(!!_ivec) ivec=_ivec; else ivec.setZero();
}

void TM_InsideBox::phi(arr& y, arr& J, const rai::Configuration& G) {
  rai::Shape* pnt=G.frames(i)->shape;
  rai::Shape* box=G.frames(j)->shape;
  CHECK(pnt && box, "I need shapes!");
  CHECK(box->type()==rai::ST_ssBox || box->type()==rai::ST_box, "the 2nd shape needs to be a box"); //s1 should be the board
  arr pos, posJ;
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
  if(!!J) {
    J.resize(6, posJ.d1);
    J[0] =  posJ[0];
    J[1] = -posJ[0];
    J[2] =  posJ[1];
    J[3] = -posJ[1];
    J[4] =  posJ[2];
    J[5] = -posJ[2];
  }
}

//===========================================================================

void TM_InsideLine::phi(arr& y, arr& J, const rai::Configuration& G) {
  rai::Shape* pnt=G.frames(i)->shape;
  rai::Shape* box=G.frames(j)->shape;
  CHECK(pnt && box, "I need shapes!");
  CHECK(box->type()==rai::ST_capsule, "the 2nd shape needs to be a capsule"); //s1 should be the board
  arr pos, posJ;
  G.kinematicsRelPos(pos, posJ, &pnt->frame, NoVector, &box->frame, NoVector);
  double range = box->size(-2);
  range *= .5;
  range -= margin;
  if(range<.01) range=.01;

  y.resize(2);
  y(0) =  pos(2) - range;
  y(1) = -pos(2) - range;
  if(!!J) {
    J.resize(2, posJ.d1);
    J[0] =  posJ[2];
    J[1] = -posJ[2];
  }
}

//===========================================================================

void F_GraspOppose::phi(arr& y, arr& J, const rai::Configuration& K) {
  Value D1 = F_PairCollision(i, k, F_PairCollision::_vector, true)(K);
  Value D2 = F_PairCollision(j, k, F_PairCollision::_vector, true)(K);

  y = D1.y + D2.y;
  if(!!J) J = D1.J + D2.J;

  if(centering){
    normalizeWithJac(D1.y, D1.J);
    normalizeWithJac(D2.y, D2.J);

    Value P1 = TM_Default(TMT_pos, i)(K);
    Value P2 = TM_Default(TMT_pos, j)(K);


    arr P = 2.*eye(3) - (D1.y*~D1.y) - (D2.y*~D2.y);
    arr p = P2.y - P1.y;
    double scale = 1e-1;
    y.append( scale * (P * p) );
    if(!!J){
      arr Jc = P * (P2.J-P1.J) - (D1.J*scalarProduct(D1.y,p) + D1.y*(~p*D1.J)) - (D2.J*scalarProduct(D2.y,p) + D2.y*(~p*D2.J));
      J.append( scale * Jc );
    }

  }
}

