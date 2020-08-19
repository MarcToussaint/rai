/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

//===========================================================================

void setTasks(KOMO& MP,
              rai::Shape& endeff,
              rai::Shape& target,
              byte whichAxesToAlign,
              uint iterate,
              int timeSteps,
              double duration);

//===========================================================================

arr moveTo(rai::Configuration& world,
           rai::Shape& endeff,
           rai::Shape& target,
           byte whichAxesToAlign,
           uint iterate,
           int timeSteps,
           double duration) {

  KOMO MP(world);

  setTasks(MP, endeff, target, whichAxesToAlign, iterate, timeSteps, duration);

  //-- create the Optimization problem (of type kOrderMarkov)
  arr x = MP.getInitialization();
  rndGauss(x, .01, true); //don't initialize at a singular config

//  MP.komo_problem.checkStructure(x);
//  checkJacobianCP(Conv_KOMO_ConstrainedProblem(MP.komo_problem), x, 1e-4);

  //-- optimize
  double colPrec = rai::getParameter<double>("KOMO/moveTo/collisionPrecision", -1e0);
  rai::Configuration::setJointStateCount=0;
  for(uint k=0; k<iterate; k++) {
    rai::timerStart();
    if(colPrec<0) {
//      optConstrained(x, NoArr, Convert(MP), OPT(verbose=2)); //parameters are set in cfg!!
      optConstrained(x, NoArr, Convert(MP.komo_problem)); //parameters are set in cfg!!
      //verbose=1, stopIters=100, maxStep=.5, stepInc=2./*, nonStrictSteps=(!k?15:5)*/));
    } else {
      optNewton(x, Convert(MP.komo_problem));
    }
    cout <<"** optimization time=" <<rai::timerRead()
         <<" setJointStateCount=" <<rai::Configuration::setJointStateCount <<endl;
    //    checkJacobian(Convert(MF), x, 1e-5);
    //MP.costReport();
  }

  return x;
}

//===========================================================================

//===========================================================================

