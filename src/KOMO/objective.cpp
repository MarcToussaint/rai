/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "objective.h"
#include "../KOMO/switch.h"
#include "../Kin/feature.h"
#include "../Kin/frame.h"
#include "../Core/graph.h"
#include "../Optim/NLP.h"

//===========================================================================

bool Objective::activeAtTime(double time) {
  if(!times.N) return true; //always
  if(times.N==1) {
    if(times.scalar()==time) return true; else return false;
  }
  CHECK_EQ(times.N, 2, "");
  return time>=times(0) && time<=times(1);
}

void Objective::write(std::ostream& os) const {
  os <<"OBJECTIVE '" <<name <<"'";
//  if(configs.N) {
//    if(configs.nd==1) {
//      if(configs.N>4) writeConsecutiveConstant(os, configs);
//      else os <<" ("<<configs <<')';
//    } else os <<" (" <<configs.first() <<".." <<configs.last() <<')';
//  } else os <<" ()";
  os <<"  times:" <<times
     <<"  type:" <<rai::Enum<ObjectiveType>(type)
     <<"  order:" <<feat->order
     <<"  target:" <<feat->target
     <<"  scale:" <<feat->scale;
}

//===========================================================================

rai::String GroundedObjective::name() { return feat->shortTag(frames.first()->C); }

shared_ptr<Objective> ObjectiveL::add(const arr& times, const shared_ptr<Feature>& f, ObjectiveType type, const char* name) {
  append(make_shared<Objective>(f, type, name, times));
  return last();
}

shared_ptr<Objective> ObjectiveL::add(const arr& times, const shared_ptr<Feature>& f, const rai::Configuration& C, const StringA& frames, ObjectiveType type, const arr& scale, const arr& target, int order, int deltaFromStep, int deltaToStep) {
  f->setup(C, frames, scale, target, order);
  return add(times, f, type, f->shortTag(C));
}

shared_ptr<Objective> ObjectiveL::add(const arr& times, const FeatureSymbol& feat, const rai::Configuration& C, const StringA& frames, ObjectiveType type, const arr& scale, const arr& target, int order, int deltaFromStep, int deltaToStep) {
  auto f = make_feature(feat, frames, C, scale, target, order);
  return add(times, f, type, f->shortTag(C));
}

double ObjectiveL::maxError(const rai::Configuration& C, double time, int verbose) const {
  double maxError = 0.;
  for(const auto& o: *this) {
    if(time<0. || o->activeAtTime(time)) {
      if(o->type==OT_ineq || o->type==OT_eq) {
        if(o->feat->order==0) {
          arr y = o->feat->eval(o->feat->getFrames(C));
          double m=0.;
          for(double& yi : y) {
            if(o->type==OT_ineq && yi>m) m=yi;
            if(o->type==OT_eq  && fabs(yi)>m) m=fabs(yi);
          }
          if(verbose>0) {
            LOG(0) <<"err: " <<m <<' ' <<o->feat->shortTag(C);
          }
          if(m>maxError) maxError=m;
        }
      }
    }
  }
  return maxError;
}
