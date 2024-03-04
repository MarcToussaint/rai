/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "CtrlSet.h"

#include "CtrlTargets.h"

shared_ptr<CtrlObjective> CtrlSet::addObjective(const shared_ptr<Feature>& f, ObjectiveType type, double transientStep) {
  std::shared_ptr<CtrlObjective> t = make_shared<CtrlObjective>();
  t->feat = f;
  t->type = type;
  t->transientStep = transientStep;
  if(t->transientStep>0.) {
    t->setRef(make_shared<CtrlTarget_MaxCarrot>(*t, t->transientStep));
  }
  objectives.append(t);
  return t;
}

shared_ptr<CtrlObjective> CtrlSet::addControlObjective(uint order, double _scale, const rai::Configuration& C) {
  return addObjective(symbols2feature(FS_qControl, {}, C, {_scale}, NoArr, order), OT_sos);
}

void CtrlSet::report(std::ostream& os) const {
  for(auto& o: objectives) {
    o->reportState(os);
  }
}

bool CtrlSet::canBeInitiated(const rai::Configuration& pathConfig) const {
  return isFeasible(*this, pathConfig, true);
}

bool CtrlSet::isConverged(const rai::Configuration& pathConfig) const {
  return isFeasible(*this, pathConfig, false);
}

bool isFeasible(const CtrlSet& CS, const rai::Configuration& pathConfig, bool initOnly, double eqPrecision) {
  bool isFeasible=true;
  for(const auto& o: CS.objectives) {
    if(o->type==OT_ineq || o->type==OT_eq) {
      if(!initOnly && o->transientStep>0. && o->movingTarget->isTransient) { isFeasible=false; break; }
      if(!initOnly || o->transientStep<=0.) {
        arr y = o->feat->eval(o->feat->getFrames(pathConfig));
        if(o->type==OT_ineq) {
          for(double& yi : y) if(yi>eqPrecision) { isFeasible=false; break; }
        }
        if(o->type==OT_eq) {
          for(double& yi : y) if(fabs(yi)>eqPrecision) { isFeasible=false; break; }
        }
      }
    }
    if(!isFeasible) break;
  }
  return isFeasible;
}

CtrlSet operator+(const CtrlSet& A, const CtrlSet& B) {
  CtrlSet CS;
  CS.objectives.resize(A.objectives.N+B.objectives.N);
  for(uint i=0; i<A.objectives.N; i++) CS.objectives.elem(i) = A.objectives.elem(i);
  for(uint i=0; i<B.objectives.N; i++) CS.objectives.elem(A.objectives.N+i) = B.objectives.elem(i);
  return CS;
}
