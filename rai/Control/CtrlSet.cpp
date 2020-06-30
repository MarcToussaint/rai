#include "CtrlSet.h"

#include "CtrlTargets.h"

ptr<CtrlObjective> CtrlSet::addObjective(const ptr<Feature>& f, ObjectiveType type, bool transient) {
  ptr<CtrlObjective> t = make_shared<CtrlObjective>();
  t->feat = f;
  objectives.append(t);
  return t;
}

void CtrlSet::report(std::ostream& os){
  for(auto& o: objectives){
    o->reportState(os);
  }
}
