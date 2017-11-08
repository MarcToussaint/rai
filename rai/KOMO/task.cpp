#include "task.h"
#include <Core/graph.h>

//===========================================================================

void Task::setCostSpecs(int fromTime,
                        int toTime,
                        const arr& _target,
                        double _prec){
  if(&_target) target = _target; else target = {0.};
  if(fromTime<0) fromTime=0;
  CHECK(toTime>=fromTime,"");
  prec.resize(toTime+1).setZero();
  for(uint t=fromTime;t<=(uint)toTime;t++) prec(t) = _prec;
}

#define STEP(t) (floor(t*double(stepsPerPhase) + .500001))-1

void Task::setCostSpecs(double fromTime, double toTime, int stepsPerPhase, uint T, const arr& _target, double _prec){
  if(stepsPerPhase<0) stepsPerPhase=T;
  if(STEP(toTime)>T-1){
      LOG(-1) <<"beyond the time!: endTime=" <<toTime <<" phases=" <<double(T)/stepsPerPhase;
  }
  int tFrom = (fromTime<0.?0:STEP(fromTime)+map->order);
  int tTo = (toTime<0.?T-1:STEP(toTime));
  if(tTo<0) tTo=0;
  if(tFrom>tTo && tFrom-tTo<=(int)map->order) tFrom=tTo;

  setCostSpecs(tFrom, tTo, _target, _prec);
}

//===========================================================================

Task* Task::newTask(const Node* specs, const mlr::KinematicWorld& world, int stepsPerPhase, uint T){
  if(specs->parents.N<2) return NULL; //these are not task specs

  //-- check the term type first
  ObjectiveType termType;
  mlr::String& tt=specs->parents(0)->keys.last();
  if(tt=="MinSumOfSqr") termType=OT_sumOfSqr;
  else if(tt=="LowerEqualZero") termType=OT_ineq;
  else if(tt=="EqualZero") termType=OT_eq;
  else return NULL;

  //-- try to crate a map
  TaskMap *map = TaskMap::newTaskMap(specs, world);
  if(!map) return NULL;

  //-- create a task
  Task *task = new Task(map, termType);

  if(specs->keys.N) task->name=specs->keys.last();
  else{
    task->name = map->shortTag(world);
//    for(Node *p:specs->parents) task->name <<'_' <<p->keys.last();
    task ->name<<"_o" <<task->map->order;
  }

  //-- check for additional continuous parameters
  if(specs->isGraph()){
    const Graph& params = specs->graph();
    arr time = params.get<arr>("time",{0.,1.});
    task->setCostSpecs(time(0), time(1), stepsPerPhase, T, params.get<arr>("target", {}), params.get<double>("scale", {1.}));
  }else{
    task->setCostSpecs(0, T-1, {}, 1.);
  }
  return task;
}

//===========================================================================
