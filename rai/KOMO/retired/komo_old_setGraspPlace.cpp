
void KOMO::setGrasp(double time, const char* endeffRef, const char* object){
//#    (EqualZero GJK Hand Obj){ time=[1 1] scale=100 } #touch is not necessary
//  mlr::String& endeffRef = world.getShapeByName(graspRef)->body->inLinks.first()->from->shapes.first()->name;
  mlr::Body *endeff = world.getShapeByName(endeffRef)->body;
  mlr::String& graspRef = endeff->outLinks.last()->to->shapes.scalar()->name;

  setTask(time-.1, time, new TaskMap_Default(vecTMT, world, endeffRef, Vector_z), OT_sumOfSqr, {0.,0.,1.}, 1e3);
  setTask(time-.1, time, new TaskMap_Default(posDiffTMT, world, graspRef, NoVector, object, NoVector), OT_sumOfSqr, NoArr, 1e3);
  setTask(time-.1, time, new TaskMap_Default(quatDiffTMT, world, graspRef, NoVector, object, NoVector), OT_sumOfSqr, NoArr, 1e3);
//#    (MinSumOfSqr posDiff Hand Obj){ time=[.98 1] scale=1e3 }
//#    (MinSumOfSqr quatDiff Hand Obj){ time=[.98 1] scale=1e3 }

  setKinematicSwitch(time, "delete", object, NULL);
  setKinematicSwitch(time, "rigidZero", graspRef, object);
//#    (MakeJoint delete Obj){ time=1 }
//#    (MakeJoint rigidZero Hand Obj){ time=1 }

  if(stepsPerPhase>2){ //otherwise: no velocities
    setTask(time-.2, time-.05, new TaskMap_Default(posTMT, world, object), OT_sumOfSqr, {0.,0.,-.1}, 1e1, 1); //move down
    setTask(time, time+.15, new TaskMap_Default(posTMT, world, object), OT_sumOfSqr, {0.,0.,.1}, 1e1, 1); // move up
//#    (MinSumOfSqr pos Obj){ order=1 scale=1e-1 time=[0 0.15] target=[0 0 .1] } # move up
  }
}

void KOMO::setPlace(double time, const char* endeffRef, const char* object, const char* placeRef){
  mlr::String& graspRef = world.getShapeByName(endeffRef)->body->outLinks.last()->to->shapes.scalar()->name;
  if(stepsPerPhase>2){ //otherwise: no velocities
    setTask(time-.15, time, new TaskMap_Default(posTMT, world, object), OT_sumOfSqr, {0.,0.,-.1}, 1e1, 1);
    setTask(time, time+.15, new TaskMap_Default(posTMT, world, object), OT_sumOfSqr, {0.,0.,.1}, 1e1, 1); // move up
  }

  setTask(time-.02, time, new TaskMap_Default(posDiffTMT, world, object, NoVector, placeRef, NoVector), OT_sumOfSqr, {0.,0.,.1}, 1e3);
//#    (MinSumOfSqr posDiff Obj Onto){ time=[1 1] target=[0 0 .2] scale=1000 } #1/2 metre above the thing

  setTask(time-.02, time, new TaskMap_Default(vecTMT, world, object, Vector_z), OT_sumOfSqr, {0.,0.,1.}, 1e2);
//#    (MinSumOfSqr vec Obj){ time=[1 1] vec1=[0 0 1] target=[0 0 1] scale=100} #upright

  setKinematicSwitch(time, "delete", graspRef, object);
  setKinematicSwitch(time, "rigidAtTo", placeRef, object);
//#    (MakeJoint delete Hand Obj){ time=1 }
//#    (MakeJoint rigidAtTo Onto Obj){ time=1 }
}
