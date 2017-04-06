//===========================================================================

void setTasks(KOMO& MP,
              mlr::Shape &endeff,
              mlr::Shape& target,
              byte whichAxesToAlign,
              uint iterate,
              int timeSteps,
              double duration);

//===========================================================================

arr moveTo(mlr::KinematicWorld& world,
           mlr::Shape &endeff,
           mlr::Shape& target,
           byte whichAxesToAlign,
           uint iterate,
           int timeSteps,
           double duration){

  KOMO MP(world);

  setTasks(MP, endeff, target, whichAxesToAlign, iterate, timeSteps, duration);


  //-- create the Optimization problem (of type kOrderMarkov)
  arr x = MP.getInitialization();
  rndGauss(x,.01,true); //don't initialize at a singular config

//  MP.komo_problem.checkStructure(x);
//  checkJacobianCP(Conv_KOMO_ConstrainedProblem(MP.komo_problem), x, 1e-4);

  //-- optimize
  double colPrec = mlr::getParameter<double>("KOMO/moveTo/collisionPrecision", -1e0);
  mlr::KinematicWorld::setJointStateCount=0;
  for(uint k=0;k<iterate;k++){
    mlr::timerStart();
    if(colPrec<0){
//      optConstrained(x, NoArr, Convert(MP), OPT(verbose=2)); //parameters are set in cfg!!
      optConstrained(x, NoArr, Convert(MP.komo_problem)); //parameters are set in cfg!!
      //verbose=1, stopIters=100, maxStep=.5, stepInc=2./*, nonStrictSteps=(!k?15:5)*/));
    }else{
      optNewton(x, Convert(MP.komo_problem));
    }
    cout <<"** optimization time=" <<mlr::timerRead()
        <<" setJointStateCount=" <<mlr::KinematicWorld::setJointStateCount <<endl;
    //    checkJacobian(Convert(MF), x, 1e-5);
    //MP.costReport();
  }

  return x;
}

//===========================================================================


//===========================================================================

