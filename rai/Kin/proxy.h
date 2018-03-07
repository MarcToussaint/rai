/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
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
