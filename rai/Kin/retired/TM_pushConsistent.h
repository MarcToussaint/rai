/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once
#include "feature.h"

struct TM_PushConsistent : Feature {
  int i, j;               ///< which shapes does it refer to?
  
  TM_PushConsistent(int iShape=-1, int jShape=-1);
  
  TM_PushConsistent(const rai::KinematicWorld& G,
                    const char* iShapeName=NULL, const char* jShapeName=NULL);
                    
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple);
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G) {  HALT("you shouldn't be here!");  }
  virtual uint dim_phi(const rai::KinematicWorld& G) { return 3; }
  virtual rai::String shortTag(const rai::KinematicWorld& G);
};

