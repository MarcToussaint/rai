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
 * @defgroup rai_interface_SWIFT SWIFT Interface.
 * @{
 */

class SWIFT_Scene;

/// contains all information necessary to communicate with swift
struct SwiftInterface {
  SWIFT_Scene *scene;
  intA INDEXswift2frame, INDEXshape2swift;
  double cutoff;
  
  SwiftInterface(const rai::KinematicWorld& world, double _cutoff=.2);
  ~SwiftInterface();
  
  void setCutoff(double _cutoff) { cutoff=_cutoff; }
  
  void step(rai::KinematicWorld& world, bool dumpReport=false);
  void pushToSwift(const rai::KinematicWorld& world);
  void pullFromSwift(rai::KinematicWorld& world, bool dumpReport);
  
  void reinitShape(const rai::Frame *s);
//  void close();
  void activate(rai::Frame *s);
  void deactivate(rai::Frame *s);
  void activate(rai::Frame *s1, rai::Frame *s2);
  void deactivate(rai::Frame *s1, rai::Frame *s2);
  void deactivate(const FrameL& shapes1, const FrameL& shapes2);
  void deactivate(const FrameL& shapes);
  
  void initActivations(const rai::KinematicWorld& world);
  void swiftQueryExactDistance();
  uint countObjects();
};
/** @} */
