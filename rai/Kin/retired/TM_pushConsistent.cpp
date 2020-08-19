/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_pushConsistent.h"
#include "frame.h"

TM_PushConsistent::TM_PushConsistent(int iShape, int jShape) : i(iShape), j(jShape) {
  order=1;
}

TM_PushConsistent::TM_PushConsistent(const rai::Configuration& G,
                                     const char* iShapeName, const char* jShapeName) : i(-1), j(-1) {
  rai::Frame* a = iShapeName ? G.getFrameByName(iShapeName):nullptr;
  rai::Frame* b = jShapeName ? G.getFrameByName(jShapeName):nullptr;
  if(a) i=a->ID;
  if(b) j=b->ID;
  order=1;
}

//void crossProduct(arr& y, arr& J1, arr& J2, const arr& x1, const arr& x2){
//  CHECK(x1.nd==1 && x2.nd==1, "");
//  CHECK(x1.N==3 && x2.N==3,"cross product only works for 3D vectors!");
//  y = crossProduct(x1,x2);
//  if(!!J1) J1 = skew(x2);
//  if(!!J2) J2 = -skew(x1);
//}

void TM_PushConsistent::phi(arr& y, arr& J, const ConfigurationL& Ktuple) {
  CHECK_GE(Ktuple.N, order+1, "I need at least " <<order+1 <<" configurations to evaluate");

  const rai::Configuration& G2 = *Ktuple.elem(-1);
  const rai::Configuration& G1 = *Ktuple.elem(-2);

  rai::Frame* body_i1 = G1.frames(i);
  rai::Frame* body_i2 = G2.frames(i);
  rai::Frame* body_j2 = G2.frames(j);

  arr yi1, yi2, yj2, Ji1, Ji2, Jj2;
  G1.kinematicsPos(yi1, Ji1, body_i1);
  G2.kinematicsPos(yi2, Ji2, body_i2);
  G2.kinematicsPos(yj2, Jj2, body_j2);

#if 1
//  tau = 1.; //1e-5;
  double tau = Ktuple(-1)->frames(0)->tau; // - Ktuple(-2)->frames(0)->time;

  y = crossProduct((yi2-yi1)/tau, yi2-yj2);
//  cout <<"PC " <<t <<' ' <<(yi2-yi1)/tau <<' ' <<yi2-yj2 <<' ' <<y <<endl;
  if(!!J) {
    uint qidx=0;
    for(uint i=0; i<Ktuple.N; i++) qidx+=Ktuple(i)->q.N;
    J.resize(y.N, qidx).setZero();

    arr J1 = skew(yi2-yj2)*Ji1/tau;
    arr J2 = skew((yi2-yi1)/tau)*(Ji2-Jj2) - skew(yi2-yj2)*Ji2/tau;

    J.setMatrixBlock(J1, 0, qidx-(J1.d1+J2.d1));
    J.setMatrixBlock(J2, 0, qidx-J2.d1);
  }
#else
  arr a = ARR(1., 0, 0);
  y = crossProduct(yi2-yi1, a);
  if(!!J) {
    uint qidx=0;
    for(uint i=0; i<G.N; i++) qidx+=G(i)->q.N;
    J.resize(y.N, qidx).setZero();

    arr J1 = skew(a)*Ji1;
    arr J2 = -skew(a)*Ji2;

    J.setMatrixBlock(J1, 0, qidx-(J1.d1+J2.d1));
    J.setMatrixBlock(J2, 0, qidx-J2.d1);
  }
#endif
}

rai::String TM_PushConsistent::shortTag(const rai::Configuration& G) {
  return STRING("PushConsistent:"<<(i<0?"WORLD":G.frames(i)->name) <<':' <<(j<0?"WORLD":G.frames(j)->name));
}
