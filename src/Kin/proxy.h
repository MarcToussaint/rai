/*  -------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Geo/geo.h"
#include "../Geo/pairCollision.h"
#include <memory>

namespace rai {
struct Frame;
}

//===========================================================================

namespace rai {

/// a data structure to store proximity information (when two shapes become close) --
/// as return value from external collision libs
struct Proxy {
  uint A=UINT32_MAX, B=UINT32_MAX;
  Vector posA=0;     ///< contact or closest point position on surface of shape A (in world coordinates)
  Vector posB=0;     ///< contact or closest point position on surface of shape B (in world coordinates)
  Vector normal=0;   ///< contact normal, pointing from B to A (proportional to posA-posB)
  double d=0.;        ///< distance (positive) or penetration (negative) between A and B
  uint colorCode = 0;
  shared_ptr<PairCollision> collision;

  // void set(Frame* _a, Frame* _b,  Vector _posA, Vector _posB, Vector _normal, double _d){ a=_a; b=_b; posA=_posA; posB=_posB; normal=_normal; d=_d; }
  // void copy(const Configuration& C, const Proxy& p);
  PairCollision& ensure_coll(const Array<Frame*>& frames) { if(!collision) calc_coll(frames); return *collision; }
  void calc_coll(const Array<Frame*>& frames);
  void write(ostream& os, bool brief=true) const;
};
stdOutPipe(Proxy)

void glDrawProxies(void*);

} //namespace rai
