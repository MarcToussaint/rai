/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

struct TM_InsideBox : Feature {
  int i, j;               ///< which shapes does it refer to?
  rai::Vector ivec;       ///< additional position or vector
  double margin;
  
  TM_InsideBox(int iShape=-1, int jShape=-1);
  TM_InsideBox(const rai::KinematicWorld& G,
               const char* iShapeName=NULL, const rai::Vector& ivec=NoVector, const char* jShapeName=NULL, double _margin=.03);
               
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G);
  virtual uint dim_phi(const rai::KinematicWorld& G){ return 6; }
  virtual rai::String shortTag(const rai::KinematicWorld &G){ return STRING("InsideBox:"<<(i<0?"WORLD":G.frames(i)->name) <<':' <<(j<0?"WORLD":G.frames(j)->name)); }
};

struct TM_InsideLine : Feature {
  int i, j;               ///< which shapes does it refer to?
  double margin;

  TM_InsideLine(int iShape=-1, int jShape=-1, double _margin=.03) : i(iShape), j(jShape), margin(_margin) {}

  TM_InsideLine(const rai::KinematicWorld& G,
               const char* iShapeName=NULL, const char* jShapeName=NULL, double _margin=.03)
    : TM_InsideLine(initIdArg(G, iShapeName), initIdArg(G, jShapeName), _margin) {}

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G);
  virtual uint dim_phi(const rai::KinematicWorld& G){ return 2; }
  virtual rai::String shortTag(const rai::KinematicWorld &G){ return STRING("InsideBox:"<<(i<0?"WORLD":G.frames(i)->name) <<':' <<(j<0?"WORLD":G.frames(j)->name)); }
};
