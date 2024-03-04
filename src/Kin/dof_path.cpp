/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "dof_path.h"

#include "kin.h"

namespace rai {

//===========================================================================

PathDof::PathDof(Frame& a, PathDof* copy) {
  frame = &a;
  dim = 1;
  limits = {0., 1.};
  frame->C.reset_q();
  frame->pathDof=this;
  if(copy) {
    copyParameters(copy);
    path = copy->path;
  }
}

PathDof::~PathDof() {
  frame->C.reset_q();
  frame->pathDof=0;
}

void PathDof::setDofs(const arr& q_full, uint qIndex) {
  CHECK_LE(qIndex+dim, q_full.N, "out of range");
  q = q_full.elem(qIndex);
  CHECK_GE(q, 0., "out of range");
  CHECK_LE(q, path.d0-1+1e-6, "out of range");
  double a=q*double(path.d0-1), i;
  a = modf(a, &i);
  arr X = (1.-a)*path[i];
  if(i+1<path.d0) X += a*path[i+1];
  frame->set_X()->set(X);
  frame->set_X()->rot.normalize();
}

arr PathDof::calcDofsFromConfig() const {
  return arr{q};
}

void PathDof::read(const Graph& ats) {
  rai::FileToken fil;
  if(ats.get(fil, "path"))     {
    fil.cd_file();
    path.read(fil.getIs());
  }

  ats.get(sampleUniform, "sampleUniform");
}

void PathDof::getJacobians(arr& Jpos, arr& Jang) const {
  //compute 'fwd kin' again:
  double a=q*double(path.d0-1), i;
  a = modf(a, &i);
  arr X = (1.-a)*path[i];
  if(i+1<path.d0) X += a*path[i+1];
  const rai::Transformation fX = frame->get_X();
//  CHECK_ZERO(absMax(X-fX.getArr7d()), 1e-6, "");

  arr d;
  if(i+1<path.d0) d = path[i+1]-path[i];
  else d = path[-1] - path[-2];
  d *= double(path.d0-1);
  Jpos = d({0, 2}).reshape(3, 1);
  Jang = (fX.rot.getJacobian() * d({3, -1})).reshape(3, 1);
  Jang /= sqrt(sumOfSqr(X({3, -1})));  //account for the non-normalization of path or quaternion interpolation
}

String PathDof::name() const {
  return STRING("path-" <<frame->name <<'.' <<frame->ID);
}

} //namespace
