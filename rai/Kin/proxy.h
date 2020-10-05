/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Geo/geo.h"
#include "../Geo/pairCollision.h"
#include <memory>

namespace rai {
struct Configuration;
struct Frame;
}

//===========================================================================

namespace rai {

/// a data structure to store proximity information (when two shapes become close) --
/// as return value from external collision libs
struct Proxy : GLDrawer {
  Frame* a=0;      ///< shape A
  Frame* b=0;      ///< shape B
  Vector posA;     ///< contact or closest point position on surface of shape A (in world coordinates)
  Vector posB;     ///< contact or closest point position on surface of shape B (in world coordinates)
  Vector normal;   ///< contact normal, pointing from B to A (proportional to posA-posB)
  double d;        ///< distance (positive) or penetration (negative) between A and B
  uint colorCode = 0;
  shared_ptr<PairCollision> collision;

  void copy(const Configuration& C, const Proxy& p);
  void ensure_coll() { if(!collision) calc_coll(); }
  void calc_coll();
  virtual void glDraw(OpenGL&);
  void write(ostream& os, bool brief=true) const;
};
stdOutPipe(Proxy)

void glDrawProxies(void*);

} //namespace rai
