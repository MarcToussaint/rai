#pragma once

#include "frame.h"

//===========================================================================

namespace rai{

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
stdOutPipe(PathDof)

}
