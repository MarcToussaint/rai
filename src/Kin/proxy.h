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

//===========================================================================

namespace mlr {

/// a data structure to store proximity information (when two shapes become close) --
/// as return value from external collision libs
struct Proxy : GLDrawer {
  //TODO: have a ProxyL& L as above...
  int a;              ///< index of shape A (-1==world) //TODO: would it be easier if this were mlr::Shape* ? YES -> Do it!
  int b;              ///< index of shape B
  Vector posA, cenA;  ///< contact or closest point position on surface of shape A (in world coordinates)
  Vector posB, cenB;  ///< contact or closest point position on surface of shape B (in world coordinates)
  Vector normal, cenN;   ///< contact normal, pointing from B to A (proportional to posA-posB)
  double d, cenD;           ///< distance (positive) or penetration (negative) between A and B
  uint colorCode;
  Proxy();
  void glDraw(OpenGL&);
};

void glDrawProxies(void*);

} //namespace mlr

#endif
