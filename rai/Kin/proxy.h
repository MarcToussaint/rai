/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef RAI_proxy_h
#define RAI_proxy_h

#include <Geo/geo.h>
#include <Geo/pairCollision.h>
#include <memory>

namespace rai {
struct KinematicWorld;
struct Frame;
}

//===========================================================================

namespace rai {

/// a data structure to store proximity information (when two shapes become close) --
/// as return value from external collision libs
struct Proxy : GLDrawer {
  Frame *a=0;      ///< shape A
  Frame *b=0;      ///< shape B
  Vector posA;     ///< contact or closest point position on surface of shape A (in world coordinates)
  Vector posB;     ///< contact or closest point position on surface of shape B (in world coordinates)
  Vector normal;   ///< contact normal, pointing from B to A (proportional to posA-posB)
  double d;        ///< distance (positive) or penetration (negative) between A and B
  uint colorCode = 0;
  std::shared_ptr<PairCollision> coll;
  
  Proxy();
  ~Proxy();

  void copy(const KinematicWorld& K, const Proxy& p);
  void calc_coll(const KinematicWorld &K);
  void del_coll() { coll.reset(); }
  void glDraw(OpenGL&);
};

void glDrawProxies(void*);

} //namespace rai

#endif
