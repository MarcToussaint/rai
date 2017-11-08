#pragma once

#include <Optim/optimization.h>
#include <Kin/taskMap.h>

struct Task {
  TaskMap *map;
  const mlr::Enum<ObjectiveType> type;  ///< element of {sumOfSqr, inequality, equality}
  mlr::String name;
  arr target, prec;     ///< optional linear, time-dependent, rescaling (with semantics of target & precision)

  Task(TaskMap *m, const ObjectiveType& type) : map(m), type(type){}
  ~Task(){ if(map) delete map; map=NULL; }

  void setCostSpecs(int fromTime, int toTime,
                    const arr& _target=ARR(0.),
                    double _prec=1.);
  void setCostSpecs(double fromTime, double toTime, int stepsPerPhase, uint T,
                    const arr& _target,
                    double _prec);
  bool isActive(uint t){ return (prec.N>t && prec(t)); }
  void write(std::ostream& os) const{
    os <<"TASK '" <<name <<"'"
      <<"  type=" <<type
      <<"  order=" <<map->order
      <<"  target=[" <<target <<']'
      <<"  prec=";
    writeConsecutiveConstant(os,prec);
  }

  static Task* newTask(const Node* specs, const mlr::KinematicWorld& world, int stepsPerPhase, uint T); ///< create a new Task from specs
};
stdOutPipe(Task)
