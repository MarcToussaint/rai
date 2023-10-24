/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

void KOMO::setGrasp(double time, const char* endeffRef, const char* object) {
//#    (EqualZero GJK Hand Obj){ time=[1 1] scale=100 } #touch is not necessary
//  rai::String& endeffRef = world.getShapeByName(graspRef)->body->inLinks.first()->from->shapes.first()->name;
  rai::Body* endeff = world.getShapeByName(endeffRef)->body;
  rai::String& graspRef = endeff->children.last()->to->shapes.scalar()->name;

  setTask(time-.1, time, new TM_Default(TMT_vec, world, endeffRef, Vector_z), OT_sos, {0., 0., 1.}, 1e3);
  setTask(time-.1, time, new TM_Default(TMT_posDiff, world, graspRef, NoVector, object, NoVector), OT_sos, NoArr, 1e3);
  setTask(time-.1, time, new TM_Default(TMT_quatDiff, world, graspRef, NoVector, object, NoVector), OT_sos, NoArr, 1e3);
//#    (MinSumOfSqr posDiff Hand Obj){ time=[.98 1] scale=1e3 }
//#    (MinSumOfSqr quatDiff Hand Obj){ time=[.98 1] scale=1e3 }

  setKinematicSwitch(time, "delete", object, nullptr);
  setKinematicSwitch(time, "rigidZero", graspRef, object);
//#    (MakeJoint delete Obj){ time=1 }
//#    (MakeJoint rigidZero Hand Obj){ time=1 }

  if(stepsPerPhase>2) { //otherwise: no velocities
    setTask(time-.2, time-.05, new TM_Default(TMT_pos, world, object), OT_sos, {0., 0., -.1}, 1e1, 1); //move down
    setTask(time, time+.15, new TM_Default(TMT_pos, world, object), OT_sos, {0., 0., .1}, 1e1, 1); // move up
//#    (MinSumOfSqr pos Obj){ order=1 scale=1e-1 time=[0 0.15] target=[0 0 .1] } # move up
  }
}

void KOMO::setPlace(double time, const char* endeffRef, const char* object, const char* placeRef) {
  rai::String& graspRef = world.getShapeByName(endeffRef)->body->children.last()->to->shapes.scalar()->name;
  if(stepsPerPhase>2) { //otherwise: no velocities
    setTask(time-.15, time, new TM_Default(TMT_pos, world, object), OT_sos, {0., 0., -.1}, 1e1, 1);
    setTask(time, time+.15, new TM_Default(TMT_pos, world, object), OT_sos, {0., 0., .1}, 1e1, 1); // move up
  }

  setTask(time-.02, time, new TM_Default(TMT_posDiff, world, object, NoVector, placeRef, NoVector), OT_sos, {0., 0., .1}, 1e3);
//#    (MinSumOfSqr posDiff Obj Onto){ time=[1 1] target=[0 0 .2] scale=1000 } #1/2 metre above the thing

  setTask(time-.02, time, new TM_Default(TMT_vec, world, object, Vector_z), OT_sos, {0., 0., 1.}, 1e2);
//#    (MinSumOfSqr vec Obj){ time=[1 1] vec1=[0 0 1] target=[0 0 1] scale=100} #upright

  setKinematicSwitch(time, "delete", graspRef, object);
  setKinematicSwitch(time, "rigidAtTo", placeRef, object);
//#    (MakeJoint delete Hand Obj){ time=1 }
//#    (MakeJoint rigidAtTo Onto Obj){ time=1 }
}
