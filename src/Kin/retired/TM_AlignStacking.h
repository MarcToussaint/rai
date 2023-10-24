/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

struct TM_AlignStacking : Feature {
  int i;               ///< which shapes does it refer to?
  TM_AlignStacking(int iShape=-1);

  TM_AlignStacking(const rai::Configuration& G,
                   const char* iShapeName=nullptr);

  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) { return 2; }
  virtual rai::String shortTag(const rai::Configuration& G);
};
