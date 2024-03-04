/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_geometrics.h"

#include "F_collisions.h"
#include "F_pose.h"
#include "frame.h"

//===========================================================================

void F_AboveBox::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 0, "");
  CHECK_EQ(F.N, 2, "");

  rai::Frame* pnt=F.elem(0); //object center that is above
  rai::Frame* box=F.elem(1); //box/table that is below
  CHECK(box->shape, "I need a shape as 2nd frame");
  CHECK_EQ(box->shape->type(), rai::ST_ssBox, "the 2nd shape needs to be a box");

  arr pos = F_PositionRel().eval({pnt, box});
  arr proj({2, 3}, {1, 0, 0, 0, 1, 0});
  pos = proj * pos;
  //  pos.J() = proj * pos.J();
  double radMargin = margin + box->shape->radius();
  arr range = { .5*box->shape->size(0)-radMargin, .5*box->shape->size(1)-radMargin };

  y.setBlockVector(pos - range, -pos - range);
  if(!!J) J.setBlockMatrix(pos.J(), -pos.J());
}

//===========================================================================

void F_InsideBox::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(F.N, 2, "");

  rai::Frame* pnt=F.elem(0); //pnt to be inside
  rai::Frame* box=F.elem(1); //box that is around
  CHECK(box->shape, "I need a shape as 2nd frame");
  CHECK_EQ(box->shape->type(), rai::ST_ssBox, "the 2nd shape needs to be a box");

  arr pos = F_PositionRel() .eval({pnt, box});
  arr range = box->shape->size;
  range.resizeCopy(3);
  range *= .5;
  range -= margin;
  for(double& r:range) if(r<.01) r=.01;

  pnt->C.kinematicsZero(y, J, 6);

  y.setBlockVector(pos - range, -pos - range);
  if(!!J) J.setBlockMatrix(pos.J(), -pos.J());
}

//===========================================================================

void TM_InsideLine::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(F.N, 2, "");
  rai::Shape* pnt=F.elem(0)->shape;
  rai::Shape* box=F.elem(1)->shape;
  CHECK(pnt && box, "I need shapes!");
  CHECK(box->type()==rai::ST_capsule, "the 2nd shape needs to be a capsule"); //s1 should be the board
//  arr pos, posJ;
//  G.kinematicsRelPos(pos, posJ, &pnt->frame, NoVector, &box->frame, NoVector);
  arr pos = F_PositionDiff().eval({&pnt->frame, &box->frame});
  double range = box->size(-2);
  range *= .5;
  range -= margin;
  if(range<.01) range=.01;

  y.resize(2);
  y(0) =  pos(2) - range;
  y(1) = -pos(2) - range;
  if(!!J) {
    J.resize(2, pos.J().d1);
    CHECK(!isSpecial(pos.J()), "");
    J[0] =  pos.J()[2];
    J[1] = -pos.J()[2];
  }
}

//===========================================================================

void F_GraspOppose::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 0, "");
  CHECK_EQ(F.N, 3, "");
  arr D1 = F_PairCollision(F_PairCollision::_vector, true)
           .eval({F.elem(0), F.elem(2)});
  arr D2 = F_PairCollision(F_PairCollision::_vector, true)
           .eval({F.elem(1), F.elem(2)});

  if(central<=0.) {
    y = D1 + D2;
    if(!!J) J=y.J_reset();
  } else {

    arr n1 = D1;
    arr n2 = D2;
    op_normalize(n1, 1e-3);
    op_normalize(n2, 1e-3);

    arr P1 = F_Position() .eval({F.elem(0)});
    arr P2 = F_Position() .eval({F.elem(1)});
    arr p = P2 - P1;
    op_normalize(p, 1e-3);

//    arr cen = scale * (2.*p - n1*(~n1*p) - n2*(~n2*p));
    arr cen = central * (2.*p + n1 - n2);
//    if(!!J) cenJ = P * (P2.J()-P1.J()) - (D1.J()*scalarProduct(D1, p) + D1*(~p*D1.J())) - (D2.J()*scalarProduct(D2, p) + D2*(~p*D2.J()));
    y.setBlockVector(D1 + D2, cen);
//    J.setBlockMatrix(D1.J() + D2.J(), cenJ);
    if(!!J) J=y.J_reset();
  }
}

//===========================================================================

arr F_TorusGraspEq::phi(const FrameL& F) {
  arr pos = F_PositionRel().eval(F);
  arr vec = F_VectorRel(Vector_x).eval(F);

  arr vec_z = arr{0., 0., 1.};
  //    arr vec_z = F_Vector(Vector_z).eval({F.elem(1)});
  //    arr posDiff = F_PositionDiff().eval(F);
  arr tangent;
  op_crossProduct(tangent, vec_z, pos);
  arr y1 = ~tangent * vec;

  arr y2 = arr{{1, 3}, {0., 0., 1.}} * pos;

  arr y3 = ~pos * pos;
  y3 -= r1*r1;

  arr y = zeros(3);
  F.elem(0)->C.jacobian_zero(y.J(), y.N);
  y.setVectorBlock(y1, 0);
  y.setVectorBlock(y2, 1);
  y.setVectorBlock(y3, 2);
  return y;
}
