/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "geoms.h"

namespace mlr{

struct CollisionQuery{
  GeomStore& store;
  uintA geoms;
  arr& poses;



};

struct FCL_Interface{
  struct sFCL_Interface *s;

  FCL_Interface(GeomStore& _store = _GeomStore());
  ~FCL_Interface();

  void setCutoff(double _cutoff){ cutoff=_cutoff; }

  void step(mlr::KinematicWorld& world, bool dumpReport=false);
  void pushToSwift(const mlr::KinematicWorld& world);
  void pullFromSwift(mlr::KinematicWorld& world, bool dumpReport);

  void reinitShape(const mlr::Frame *s);
//  void close();
  void activate(mlr::Frame *s);
  void deactivate(mlr::Frame *s);
  void activate(mlr::Frame *s1, mlr::Frame *s2);
  void deactivate(mlr::Frame *s1, mlr::Frame *s2);
  void deactivate(const mlr::Array<mlr::Frame*>& bodies);

  void initActivations(const mlr::KinematicWorld& world, uint parentLevelsToDeactivate=1);
  void swiftQueryExactDistance();
};

} //namespace
