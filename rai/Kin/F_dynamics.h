/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

struct F_NewtonEuler : Feature {
  double gravity=9.81;

  F_NewtonEuler(bool _transOnly=false) {
    order = 2;
    gravity = rai::getParameter<double>("TM_NewtonEuler/gravity", 9.81);
  }

  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F) { return 6; }
};

//===========================================================================

struct F_NewtonEuler_DampedVelocities : Feature {
  double gravity=9.81;
  bool onlyXYPhi=false;

  F_NewtonEuler_DampedVelocities(double _gravity=-1., bool _onlyXYPhi=false) : onlyXYPhi(_onlyXYPhi) {
    order = 1;
    if(_gravity>=0.) {
      gravity = _gravity;
    } else {
      gravity = rai::getParameter<double>("TM_NewtonEuler/gravity", 9.81);
    }
  }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F) { return 6; }
};

//===========================================================================

struct F_Wrench : Feature {
  rai::Vector vec;
  double gravity=9.81;
  bool torqueOnly=false;
  F_Wrench(const arr& _vec, bool _torqueOnly=false);
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F) {  if(torqueOnly) return 3;  return 6;  }
};

//===========================================================================

struct F_Energy : Feature {
  double gravity=9.81;
  F_Energy();
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F) {  return 1;  }
};

//===========================================================================

struct F_StaticStability : Feature {
  double margin;
  F_StaticStability(double _margin=.01) : margin(_margin) {}
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F) { return 4; }
};
