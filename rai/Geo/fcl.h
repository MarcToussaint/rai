/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "geoms.h"

namespace rai {

struct CollisionQuery {
  GeomStore& store;
  uintA geoms;
  arr& poses;

};

struct FCL_Interface {
  struct sFCL_Interface* s;

  FCL_Interface(GeomStore& _store = _GeomStore());
  ~FCL_Interface();

  void setCutoff(double _cutoff) { cutoff=_cutoff; }

  void step(rai::Configuration& world, bool dumpReport=false);
  void pushToSwift(const rai::Configuration& world);
  void pullFromSwift(rai::Configuration& world, bool dumpReport);

  void reinitShape(const rai::Frame* s);
//  void close();
  void activate(rai::Frame* s);
  void deactivate(rai::Frame* s);
  void activate(rai::Frame* s1, rai::Frame* s2);
  void deactivate(rai::Frame* s1, rai::Frame* s2);
  void deactivate(const rai::Array<rai::Frame*>& bodies);

  void initActivations(const rai::Configuration& world, uint parentLevelsToDeactivate=1);
  void swiftQueryExactDistance();
};

} //namespace
