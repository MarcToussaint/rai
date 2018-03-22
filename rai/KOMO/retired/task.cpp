//===========================================================================

Task* Task::newTask(const Node* specs, const rai::KinematicWorld& world, int stepsPerPhase, uint T){
  if(specs->parents.N<2) return NULL; //these are not task specs

  //-- check the term type first
  ObjectiveType termType;
  rai::String& tt=specs->parents(0)->keys.last();
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
