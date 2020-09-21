/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

struct F_NewtonEuler : Feature {
  int i;               ///< which shapes does it refer to?
  double gravity=9.81;

  F_NewtonEuler(int iShape, bool _transOnly=false) : i(iShape) {
    order = 2;
    gravity = rai::getParameter<double>("TM_NewtonEuler/gravity", 9.81);
  }
  F_NewtonEuler(const rai::Configuration& K, const char* iShapeName, bool _transOnly=false) : F_NewtonEuler(initIdArg(K, iShapeName), _transOnly) {}

  virtual void phi(arr& y, arr& J, const rai::Configuration& K) { HALT("can only be of higher order"); }
  virtual uint dim_phi(const rai::Configuration& K) { HALT("can only be of higher order"); }

  virtual void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  virtual uint dim_phi(const ConfigurationL& Ktuple) { return 6; }

  virtual rai::String shortTag(const rai::Configuration& K) { return STRING("NewtonEuler-" <<K.frames(i)->name); }
};

//===========================================================================

struct F_NewtonEuler_DampedVelocities : Feature {
  int i;               ///< which shapes does it refer to?
  double gravity=9.81;
  bool onlyXYPhi=false;

  F_NewtonEuler_DampedVelocities(int iShape, double _gravity=-1., bool _onlyXYPhi=false) : i(iShape), onlyXYPhi(_onlyXYPhi) {
    order = 1;
    if(_gravity>=0.) {
      gravity = _gravity;
    } else {
      gravity = rai::getParameter<double>("TM_NewtonEuler/gravity", 9.81);
    }
  }
  F_NewtonEuler_DampedVelocities(const rai::Configuration& K, const char* iShapeName, double _gravity=-1., bool _onlyXYPhi=false) : F_NewtonEuler_DampedVelocities(initIdArg(K, iShapeName), _gravity, _onlyXYPhi) {}

  virtual void phi(arr& y, arr& J, const rai::Configuration& K) { HALT("can only be of higher order"); }
  virtual uint dim_phi(const rai::Configuration& K) { HALT("can only be of higher order"); }

  virtual void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  virtual uint dim_phi(const ConfigurationL& Ktuple);

  virtual rai::String shortTag(const rai::Configuration& K) { return STRING("NewtonEuler_DampedVelocities-" <<K.frames(i)->name); }
};

//===========================================================================

struct F_Wrench : Feature {
  int i;                   ///< which shapes does it refer to?
  rai::Vector vec;
  double gravity=9.81;
  bool torqueOnly=false;

  F_Wrench(int iShape, const arr& _vec, bool _torqueOnly=false);
  F_Wrench(const rai::Configuration& K, const char* iShapeName, const arr& _vec, bool _transOnly=false) : F_Wrench(initIdArg(K, iShapeName), _vec, _transOnly) {}

  virtual void phi(arr& y, arr& J, const rai::Configuration& K) { HALT("can only be of higher order"); }
  virtual uint dim_phi(const rai::Configuration& K) { HALT("can only be of higher order"); }

  virtual void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  virtual uint dim_phi(const ConfigurationL& Ktuple);

  virtual rai::String shortTag(const rai::Configuration& K) { return STRING("Wrench-" <<K.frames(i)->name); }
};

//===========================================================================

struct F_Energy : Feature {
  double gravity=9.81;

  F_Energy();

  virtual void phi(arr& y, arr& J, const rai::Configuration& K) { HALT("can only be of higher order"); }
  virtual uint dim_phi(const rai::Configuration& K) { HALT("can only be of higher order"); }

  virtual void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  virtual uint dim_phi(const ConfigurationL& Ktuple);

  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("Energy-"<<order); }
};

//===========================================================================

struct F_StaticStability : Feature {
  int i;               ///< which shapes does it refer to?
  double margin;

  F_StaticStability(int iShape=-1, double _margin=.01);
  F_StaticStability(const rai::Configuration& G,
                    const char* iShapeName=nullptr, double _margin=.01);

  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) { return 4; }
  virtual rai::String shortTag(const rai::Configuration& G);
};
