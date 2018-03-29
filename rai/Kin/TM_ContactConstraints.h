/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "taskMap.h"


/// this stacks all constacts to one feature vector OF VARIABLE SIZE! (the optimizer has to deal with variable size features; transfering lambdas across steps)
struct TM_ContactConstraints : TaskMap {
  TM_ContactConstraints() {}

  void phi(arr& y, arr& J, const rai::KinematicWorld& K);
  virtual uint dim_phi(const rai::KinematicWorld& K);
  virtual rai::String shortTag(const rai::KinematicWorld& K){ return "ContactConstraints"; }
};
