/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

//===========================================================================

struct F_AboveBox : Feature {
  double margin=.0;
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 4; }
};

//===========================================================================

struct F_AlignWithDiff : Feature {
  rai::Vector ref;
  F_AlignWithDiff(const rai::Vector& _ref) : ref(_ref) { setOrder(1); }
  virtual arr phi(const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 3; }
};

//===========================================================================

struct F_InsideBox : Feature {
  rai::Vector ivec;       ///< additional position or vector
  double margin;
  F_InsideBox(double _margin=.01) : margin(_margin) {}
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 6; }
};

//===========================================================================

struct TM_InsideLine : Feature {
  double margin;
  TM_InsideLine(double _margin=.03) : margin(_margin) {}
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 2; }
};

//===========================================================================

struct F_GraspOppose : Feature {
  double central;
  F_GraspOppose(double _central=-1.) : central(_central) {}
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { if(central>0.) return 6; return 3; }
};

//===========================================================================

struct F_TorusGraspEq: Feature {
  double r1, r2;
  F_TorusGraspEq(double _r1, double _r2): r1(_r1), r2(_r2) {}

  uint dim_phi(const FrameL& F) { return 3; }
  arr phi(const FrameL& F);
};
