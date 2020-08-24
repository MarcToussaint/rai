/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "kin.h"

class SWIFT_Scene;

/// contains all information necessary to communicate with swift
struct SwiftInterface {
  SWIFT_Scene* scene;
  intA INDEXswift2frame, INDEXshape2swift;
  double cutoff;

  SwiftInterface(const rai::Configuration& world, double _cutoff=.2, int verbose=0);
  ~SwiftInterface();

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
  void deactivate(const FrameL& shapes1, const FrameL& shapes2);
  void deactivate(const FrameL& shapes);

  void initActivations(const rai::Configuration& world);
  void swiftQueryExactDistance();
  uint countObjects();
};
