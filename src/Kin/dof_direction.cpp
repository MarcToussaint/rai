/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "dof_direction.h"

#include "kin.h"

namespace rai {

DirectionDof::DirectionDof(Frame& a, DirectionDof* copy) {
  frame=&a;
  dim = 3;
  limits = {-1.1,-1.1,-1.1,1.1,1.1,1.1};
  q0 = zeros(3);
  sampleUniform = .0;
  sampleSdv = 1.;
  frame->C.reset_q();
  frame->dirDof=this;
  if(copy) {
    copyParameters(copy);
    vec=copy->vec;
  }
}

DirectionDof::~DirectionDof() {
  frame->C.reset_q();
  frame->dirDof=0;
}

String DirectionDof::name() const {
  return STRING("direction-" <<frame->name <<'.' <<frame->ID);
}

void DirectionDof::setDofs(const arr& q_full, uint qIndex) {
  CHECK_LE(qIndex+dim, q_full.N, "out of range");
  vec.set(q_full.p+qIndex);
  // LOG(0) <<q_full <<vec <<vec.length();
  vec /= (1e-10 + vec.length());
  frame->set_Q()->rot.setDiff(Vector_x, vec);
}

arr DirectionDof::calcDofsFromConfig() const {
  return vec.getArr();
}

void DirectionDof::read(const Graph& ats){
}

void DirectionDof::write(Graph& ats){
  ats.add<String>("joint", "direction");
}

void DirectionDof::kinVec(arr& y, arr& J) const {
  y = vec.getArr();
  arr Jang;
  frame->C.jacobian_angular(Jang, frame);
  J = crossProduct(Jang, y);
  if(active){
    arr q = frame->C.q({qIndex, qIndex+dim-1});
    double q_norm2 = sumOfSqr(q);
    arr Jnorm = eye(3) - (q*~q)/q_norm2;
    Jnorm /= 1e-10+sqrt(q_norm2);
    for(uint i=0; i<3; i++) for(uint j=0; j<3; j++){
        J.elem(i, qIndex+j) = Jnorm(i,j);
      }
  }
}

} //namespace
