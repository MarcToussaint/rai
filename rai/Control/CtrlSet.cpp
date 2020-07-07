#include "CtrlSet.h"

#include "CtrlTargets.h"

ptr<CtrlObjective> CtrlSet::addObjective(const ptr<Feature>& f, ObjectiveType type, double transientStep) {
  std::shared_ptr<CtrlObjective> t = make_shared<CtrlObjective>();
  t->feat = f;
  t->type = type;
  t->transientStep = transientStep;
  if(t->transientStep>0.){
    t->setRef(make_shared<CtrlTarget_MaxCarrot>(*t, t->transientStep));
  }
  objectives.append(t);
  return t;
}

void CtrlSet::report(std::ostream& os){
  for(auto& o: objectives){
    o->reportState(os);
  }
}

bool isFeasible(const CtrlSet& CS, const ConfigurationL& Ctuple, bool initOnly, double eqPrecision){
  bool isFeasible=true;
  for(const auto& o: CS.objectives){
    if(o->type==OT_ineq || o->type==OT_eq){
      if(o->transientStep>0. && o->movingTarget->isTransient){ isFeasible=false; break; }
      if(!initOnly || o->transientStep<=0.){
        arr y;
        o->feat->__phi(y, NoArr, Ctuple);
        if(o->type==OT_ineq){
          for(double& yi : y) if(yi>eqPrecision){ isFeasible=false; break; }
        }
        if(o->type==OT_eq){
          for(double& yi : y) if(fabs(yi)>eqPrecision){ isFeasible=false; break; }
        }
      }
    }
    if(!isFeasible) break;
  }
  return isFeasible;
}
