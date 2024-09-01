#include <KOMO/komo.h>
#include <KOMO/splined.h>
#include <Optim/NLP_Solver.h>

#include <thread>

//===========================================================================

void createPath(){
  rai::Configuration C("../timeopt/arm.g");
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;
  
  KOMO komo;
  komo.setConfig(C);
  komo.setTiming(1., 100, 1., 2);
  komo.addControlObjective({}, 2, 1.);
//  komo.addQuaternionNorms({}, 1., false);

  //-- set a time optim objective
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e2}, {}, 1); //smooth time evolution
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e1}, {komo.tau}, 0); //prior on timing

  komo.addObjective({1.}, FS_positionDiff, {"endeff", "target"}, OT_eq, {1e2});
//  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e0});
  cout <<komo.report(true, false) <<endl;

//  std::shared_ptr<NLP> skomo = make_shared<SplinedKOMO>(3, 8, komo);
  std::shared_ptr<NLP> skomo = komo.nlp_spline(7, 3);

//  for(uint k=0;k<10;k++){
//    arr x = skomo->getInitializationSample();
//    rndGauss(x, .2, true);
//    skomo->checkJacobian(x, 1e-4, komo.featureNames);
//    arr phi, J;
//    skomo->evaluate(phi, J, x);
//    komo.view(true, "init");
//    while(komo.view_play(true, -.5));
//  }
//  return;

  auto ret = NLP_Solver()
      .setProblem(skomo)
      .solve();
  cout <<*ret <<endl;
  skomo->checkJacobian(ret->x, 1e-4, komo.featureNames);
//  komo.optimize();

  komo.view(true, "result");
  while(komo.view_play(true));

  FILE("z.path") <<komo.getPath_qOrg();
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  rnd.clockSeed();

  createPath();

  return 0;
}

