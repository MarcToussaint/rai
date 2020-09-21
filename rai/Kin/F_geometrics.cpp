/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_geometrics.h"
#include "F_PairCollision.h"
#include "F_pose.h"
#include "TM_default.h"
#include "frame.h"

//===========================================================================

void TM_AboveBox::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 0, "");
  CHECK_EQ(F.N, 2, "");

  rai::Shape* box=F.elem(1)->shape;
  CHECK(box, "I need a shape as second frame!");
  CHECK_EQ(box->type(), rai::ST_ssBox, "the 2nd shape needs to be a box"); //s1 should be the board
  Value pos = F_PositionRel()
              .eval(F);
  arr proj({2,3}, {1,0,0,0,1,0});
  pos.y = proj * pos.y;
  pos.J = proj * pos.J;
  arr range = { .5*box->size(0)-margin, .5*box->size(1)-margin };

  y.setBlockVector(pos.y - range, -pos.y - range);
  J.setBlockMatrix(pos.J, -pos.J);
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
//  arr pos, posJ;
//  G.kinematicsRelPos(pos, posJ, &pnt->frame, ivec, &box->frame, NoVector);
  Value pos = evalFeature<F_PositionRel>({&pnt->frame, &box->frame});
  arr range = box->size();
  range *= .5;
  range -= margin;
  for(double& r:range) if(r<.01) r=.01;

  y.resize(6);
  y(0) =  pos.y(0) - range(0);
  y(1) = -pos.y(0) - range(0);
  y(2) =  pos.y(1) - range(1);
  y(3) = -pos.y(1) - range(1);
  y(4) =  pos.y(2) - range(2);
  y(5) = -pos.y(2) - range(2);
  if(!!J) {
    J.resize(6, pos.J.d1);
    J[0] =  pos.J[0];
    J[1] = -pos.J[0];
    J[2] =  pos.J[1];
    J[3] = -pos.J[1];
    J[4] =  pos.J[2];
    J[5] = -pos.J[2];
  }
}

//===========================================================================

void TM_InsideLine::phi(arr& y, arr& J, const rai::Configuration& G) {
  rai::Shape* pnt=G.frames(i)->shape;
  rai::Shape* box=G.frames(j)->shape;
  CHECK(pnt && box, "I need shapes!");
  CHECK(box->type()==rai::ST_capsule, "the 2nd shape needs to be a capsule"); //s1 should be the board
//  arr pos, posJ;
//  G.kinematicsRelPos(pos, posJ, &pnt->frame, NoVector, &box->frame, NoVector);
  Value pos = evalFeature<F_PositionDiff>({&pnt->frame, &box->frame});
  double range = box->size(-2);
  range *= .5;
  range -= margin;
  if(range<.01) range=.01;

  y.resize(2);
  y(0) =  pos.y(2) - range;
  y(1) = -pos.y(2) - range;
  if(!!J) {
    J.resize(2, pos.J.d1);
    J[0] =  pos.J[2];
    J[1] = -pos.J[2];
  }
}

//===========================================================================

void F_GraspOppose::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 0, "");
  CHECK_EQ(F.N, 3, "");
  Value D1 = F_PairCollision(F_PairCollision::_vector, true)
             .eval({F.elem(0), F.elem(2)});
  Value D2 = F_PairCollision(F_PairCollision::_vector, true)
             .eval({F.elem(1), F.elem(2)});


  if(!centering) {
    y = D1.y + D2.y;
    if(!!J) J = D1.J + D2.J;
  }else{
    normalizeWithJac(D1.y, D1.J);
    normalizeWithJac(D2.y, D2.J);

    Value P1 = F_Position() .eval({F.elem(0)});
    Value P2 = F_Position() .eval({F.elem(1)});

    arr P = 2.*eye(3) - (D1.y*~D1.y) - (D2.y*~D2.y);
    arr p = P2.y - P1.y;
    double scale = 1e-1;

    arr cen, cenJ;
    cen = scale * (P * p);
    if(!!J) cenJ = P * (P2.J-P1.J) - (D1.J*scalarProduct(D1.y, p) + D1.y*(~p*D1.J)) - (D2.J*scalarProduct(D2.y, p) + D2.y*(~p*D2.J));

    y.setBlockVector(D1.y + D2.y, cen);
    J.setBlockMatrix(D1.J + D2.J, cenJ);
  }
}



