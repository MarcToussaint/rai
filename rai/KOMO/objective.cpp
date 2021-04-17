/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "objective.h"
#include "../Kin/switch.h"
#include "../Core/graph.h"

//===========================================================================

void Objective::write(std::ostream& os) const {
  os <<"OBJECTIVE '" <<name <<"'";
//  if(configs.N) {
//    if(configs.nd==1) {
//      if(configs.N>4) writeConsecutiveConstant(os, configs);
//      else os <<" ("<<configs <<')';
//    } else os <<" (" <<configs.first() <<".." <<configs.last() <<')';
//  } else os <<" ()";
  os <<"  times:" <<times
     <<"  type:" <<type
     <<"  order:" <<feat->order
     <<"  target:" <<feat->target
     <<"  scale:" <<feat->scale;
}

//===========================================================================
