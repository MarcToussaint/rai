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

ptr<Objective> ObjectiveL::add(const arr& times, const ptr<Feature>& f, ObjectiveType type, const char* name) {
  append(make_shared<Objective>(f, type, name, times) );
  return last();
}

ptr<Objective> ObjectiveL::add(const arr& times, const FeatureSymbol& feat, const rai::Configuration& C, const StringA& frames, ObjectiveType type, const arr& scale, const arr& target, int order, int deltaFromStep, int deltaToStep) {
  auto f = make_feature(feat, frames, C, scale, target, order);
  return add(times, f, type, f->shortTag(C));
}

double ObjectiveL::maxError(const rai::Configuration& C, int verbose) const {
  double maxError = 0.;
  for(const auto& o: *this) {
    if(o->type==OT_ineq || o->type==OT_eq) {
      arr y = o->feat->eval(o->feat->getFrames(C));
      double m=0.;
      for(double& yi : y){
        if(o->type==OT_ineq && yi>m) m=yi;
        if(o->type==OT_eq  && fabs(yi)>m) m=fabs(yi);
      }
      if(verbose>0){
        LOG(0) <<"err: " <<m <<' ' <<o->name <<' ' <<o->feat->shortTag(C);
      }
      if(m>maxError) maxError=m;
    }
  }
  return maxError;
}
