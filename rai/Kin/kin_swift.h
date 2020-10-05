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
  intA INDEXswift2frame, swiftID;
  double cutoff;
  arr X_lastQuery;  //memory to check whether an object has moved in consecutive queries

  SwiftInterface(const FrameL& frames, double _cutoff=.2, int verbose=0);
  ~SwiftInterface();

  uintA step(const arr& X, bool dumpReport=false);

  void pushToSwift(const arr& X);
  uintA pullFromSwift(bool dumpReport);

  void deactivate(const uintA& collisionExcludeIDs);
  void deactivatePairs(const uintA& collisionExcludePairIDs);

  //-- ** will be deprecated! **
  void reinitShape(const rai::Frame* s);
  void activate(rai::Frame* s);
  void deactivate(rai::Frame* s);
  void activate(rai::Frame* s1, rai::Frame* s2);
  void deactivate(rai::Frame* s1, rai::Frame* s2);
  void deactivate(const FrameL& shapes1, const FrameL& shapes2);
  void deactivate(const FrameL& shapes);
  void initActivations(const FrameL& frames);
  void swiftQueryExactDistance();
  uint countObjects();
};
