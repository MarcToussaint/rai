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

struct PathDof : Dof, NonCopyable {
  arr path;
  double q=0.;

  PathDof(Frame& a, PathDof* copy=nullptr);
  ~PathDof();

  virtual String name() const;
  virtual void setDofs(const arr& q_full, uint qIndex=0);
  virtual arr calcDofsFromConfig() const;
  void read(const Graph& ats);

  void getJacobians(arr& Jpos, arr& Jang) const;
};

}
