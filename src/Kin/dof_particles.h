/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "frame.h"

//===========================================================================

namespace rai {

struct ParticleDofs : Dof, NonCopyable {
  Mesh* mesh;

  ParticleDofs(Frame& a, ParticleDofs* copy=nullptr);
  ~ParticleDofs();

  virtual void setDofs(const arr& q, uint n);
  virtual arr calcDofsFromConfig() const;
  virtual String name() const;

  void write(ostream& os) const {
    NIY;
  }
};
stdOutPipe(ParticleDofs)

}//namespace
