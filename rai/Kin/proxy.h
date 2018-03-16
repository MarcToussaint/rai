/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef MLR_proxy_h
#define MLR_proxy_h

#include <Geo/geo.h>
#include <Geo/pairCollision.h>
namespace mlr{
  struct KinematicWorld;
  struct Frame;
}

//===========================================================================

namespace mlr {

/// a data structure to store proximity information (when two shapes become close) --
/// as return value from external collision libs
struct Proxy : GLDrawer {
  Frame *a;              ///< shape A
  Frame *b;              ///< shape B
  Vector posA;  ///< contact or closest point position on surface of shape A (in world coordinates)
  Vector posB;  ///< contact or closest point position on surface of shape B (in world coordinates)
  Vector normal;   ///< contact normal, pointing from B to A (proportional to posA-posB)
  double d;           ///< distance (positive) or penetration (negative) between A and B
  uint colorCode = 0;
  PairCollision *coll=NULL;

  Proxy();
  ~Proxy();

  void calc_coll(const KinematicWorld &K);
  void del_coll(){ if(coll) delete coll; coll=NULL; }
  void glDraw(OpenGL&);
};

void glDrawProxies(void*);

} //namespace mlr

#endif
