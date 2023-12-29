/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Kin/frame.h"

namespace rai {

struct FclInterface {
  struct FclInterface_self* self=0;
  enum QueryMode { _broadPhaseOnly, _binaryCollisionSingle, _binaryCollisionAll, _distanceCutoff, _fine } mode;
  
  double cutoff=-1.;
  uintAA excludes;
  uintA collisions; //return values!
  arr X_lastQuery;  //memory to check whether an object has moved in consecutive queries

  FclInterface(const Array<Shape*>& geometries, const uintAA& _excludes, QueryMode _mode, double _cutoff=-1.);
  ~FclInterface();

  void step(const arr& X, double _cutoff=-2.);

protected:
  friend FclInterface_self;
  void addCollision(uint a, uint b);
};

}

