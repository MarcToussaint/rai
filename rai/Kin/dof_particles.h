#pragma once

#include "frame.h"

//===========================================================================

namespace rai{

struct ParticleDofs : Dof, NonCopyable, GLDrawer {
  Mesh *mesh;

  ParticleDofs(Frame& a, ParticleDofs* copy=nullptr);
  ~ParticleDofs();

  virtual void setDofs(const arr& q, uint n);
  virtual arr calcDofsFromConfig() const;
  virtual String name() const;

  void glDraw(OpenGL&){ NIY; }
  void write(ostream& os) const{
    NIY;
  }
};
stdOutPipe(ParticleDofs)

}//namespace
