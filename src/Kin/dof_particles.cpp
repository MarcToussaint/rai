/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "dof_particles.h"

#include "kin.h"

namespace rai {

ParticleDofs::ParticleDofs(Frame& a, ParticleDofs* copy) {
  frame=&a;
  CHECK(frame->shape, "only shapes have ParticleDofs");
  CHECK_EQ(frame->shape->type(), ST_mesh, "only mesh shapes have ParticleDofs");
  mesh = &frame->shape->mesh();
  CHECK(mesh->V.d0>0, "mesh has no particles");
  dim = mesh->V.N;
  frame->C.reset_q();
  frame->particleDofs=this;
  if(copy) {
    qIndex=copy->qIndex; dim=copy->dim;
    active=copy->active;
  }
}

ParticleDofs::~ParticleDofs() {
  frame->C.reset_q();
  frame->particleDofs=0;
}

void ParticleDofs::setDofs(const arr& q, uint n) {
  CHECK_LE(n+dim, q.N, "out of range");
  CHECK_EQ(dim, mesh->V.N, "");
  memmove(mesh->V.p, q.p+n, q.sizeT*dim);
}

arr ParticleDofs::calcDofsFromConfig() const {
  arr Vflat = mesh->V;
  Vflat.reshape(-1);
  return Vflat;
}

String ParticleDofs::name() const {
  return STRING("particles-" <<frame->name);
}

} //namespace
