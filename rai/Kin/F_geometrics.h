/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

//===========================================================================

struct TM_AboveBox : Feature {
  int i, j;               ///< which shapes does it refer to?
  double margin;

  TM_AboveBox(int iShape=-1, int jShape=-1, double _margin=.01);
  TM_AboveBox(const rai::Configuration& G,
              const char* iShapeName=nullptr, const char* jShapeName=nullptr, double _margin=.01);

  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) { return 4; }
  virtual rai::String shortTag(const rai::Configuration& G);
  virtual rai::Graph getSpec(const rai::Configuration& K);
};

//===========================================================================

struct TM_InsideBox : Feature {
  int i, j;               ///< which shapes does it refer to?
  rai::Vector ivec;       ///< additional position or vector
  double margin;

  TM_InsideBox(int iShape=-1, int jShape=-1);
  TM_InsideBox(const rai::Configuration& G,
               const char* iShapeName=nullptr, const rai::Vector& ivec=NoVector, const char* jShapeName=nullptr, double _margin=.03);

  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) { return 6; }
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("InsideBox:"<<(i<0?"WORLD":G.frames(i)->name) <<':' <<(j<0?"WORLD":G.frames(j)->name)); }
};

//===========================================================================

struct TM_InsideLine : Feature {
  int i, j;               ///< which shapes does it refer to?
  double margin;

  TM_InsideLine(int iShape=-1, int jShape=-1, double _margin=.03) : i(iShape), j(jShape), margin(_margin) {}

  TM_InsideLine(const rai::Configuration& G,
                const char* iShapeName=nullptr, const char* jShapeName=nullptr, double _margin=.03)
    : TM_InsideLine(initIdArg(G, iShapeName), initIdArg(G, jShapeName), _margin) {}

  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) { return 2; }
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("InsideBox:"<<(i<0?"WORLD":G.frames(i)->name) <<':' <<(j<0?"WORLD":G.frames(j)->name)); }
};

//===========================================================================

struct F_GraspOppose : Feature {
  int i, j, k;               ///< which shapes does it refer to?
  bool centering=false;

  F_GraspOppose(int iShape=-1, int jShape=-1, int kShape=-1) : i(iShape), j(jShape), k(kShape) {}
  F_GraspOppose(const rai::Configuration& K,
                const char* iShapeName=nullptr, const char* jShapeName=nullptr, const char* kShapeName=nullptr)
    : F_GraspOppose(initIdArg(K, iShapeName), initIdArg(K, jShapeName), initIdArg(K, kShapeName)) {}

  virtual void phi(arr& y, arr& J, const rai::Configuration& K);
  virtual uint dim_phi(const rai::Configuration& G) { if(centering) return 6; return 3; }
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("GraspOppose-" <<G.frames(i)->name <<'-' <<G.frames(j)->name <<'-' <<G.frames(k)->name); }
};

