/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "kin.h"

//===========================================================================
/**
 * @defgroup ors_interface_SWIFT SWIFT Interface.
 * @{
 */

class SWIFT_Scene;

/// contains all information necessary to communicate with swift
struct SwiftInterface {
  SWIFT_Scene *scene;
  intA INDEXswift2frame, INDEXshape2swift;
  double cutoff;

  SwiftInterface(const mlr::KinematicWorld& world, double _cutoff=.2);
  ~SwiftInterface();

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
  void deactivate(const FrameL& bodies);

  void initActivations(const mlr::KinematicWorld& world, uint parentLevelsToDeactivate=1);
  void swiftQueryExactDistance();
  uint countObjects();
};
/** @} */
