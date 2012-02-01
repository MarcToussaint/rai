void MotorPrimitive:step(){

  // query the current next action from actionplan

  //depending on the action type, call.....
    if(goalVar->goalType==FutureMotionGoal::graspGoalT){
      setGraspGoals(*sys, sys->nTime(), goalVar->graspShape);
      //arr q;
      //soc::straightTaskTrajectory(*sys, q, 0);
      //planner.init_trajectory(q);
    } else if(goalVar->goalType==FutureMotionGoal::placeGoalT){
      setPlaceGoals(*sys, sys->nTime(), goalVar->graspShape, goalVar->belowFromShape, goalVar->belowToShape);
      //arr q;
      //soc::straightTaskTrajectory(*sys, q, 0);
      //planner.init_trajectory(q);
    } else if(goalVar->goalType==FutureMotionGoal::homingGoalT){
      setHomingGoals(*sys, sys->nTime(), goalVar->graspShape, goalVar->belowToShape);
      //arr q;
      //soc::straightTaskTrajectory(*sys, q, 1); //task id is q!!!
      //planner.init_trajectory(q);
    }

    //... to set task variables

    //FUTURE: collaps all the task variable stuff to a single Phi
};

