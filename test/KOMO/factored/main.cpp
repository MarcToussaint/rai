#include <KOMO/komo.h>
#include <Kin/F_collisions.h>
#include <Kin/viewer.h>
#include <Kin/F_pose.h>
#include <Optim/NLP_Solver.h>
#include <Optim/utils.h>
#include <KOMO/skeletonSymbol.h>

//===========================================================================

void testFactored(){
  rai::Configuration C("../switches/model2.g");

  rai::Frame* g = C["gripper"];
  g->ensure_X();
  rai::Frame* g2 = new rai::Frame(C, g);
  g2->name = "gripperDUP";
  g2->setShape(rai::ST_box, {.1,.05,.05}); //usually not!
  g2->setParent(C.frames.first(), true);
  g2->setJoint(rai::JT_free);

  //== define the full problem

  KOMO komo;

  komo.setConfig(C, false);
  komo.setTiming(2.5, 3, 5., 2);
  komo.addControlObjective({}, 2);
  komo.addQuaternionNorms();

  //consistency constraint
  komo.addObjective({}, FS_poseDiff, {"gripper", "gripperDUP"}, OT_eq, {1e2});

  //grasp
//  komo.addSwitch_stable(1., 2., "table", "gripper", "box");
  komo.addModeSwitch({1., 2.}, rai::SY_stable, {"gripper", "box"}, true);
  komo.addObjective({1.}, FS_positionDiff, {"gripperDUP", "box"}, OT_eq, {1e2});
  komo.addObjective({1.}, FS_scalarProductXX, {"gripperDUP", "box"}, OT_eq, {1e2}, {0.});
  komo.addObjective({1.}, FS_vectorZ, {"gripperDUP"}, OT_eq, {1e2}, {0., 0., 1.});

  //slow & down-up
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {}, {}, 1);
  komo.addObjective({.9,1.1}, FS_position, {"gripper"}, OT_eq, {}, {0.,0.,.1}, 2);

  //place
//  komo.addSwitch_stable(2., -1., "gripper", "table", "box", false);
  komo.addModeSwitch({2., -1.}, rai::SY_stable, {"table", "box"}, false);
  komo.addObjective({2.}, FS_positionDiff, {"box", "table"}, OT_eq, {1e2}, {0,0,.08}); //arr({1,3},{0,0,1e2})
  komo.addObjective({2.}, FS_vectorZ, {"gripper"}, OT_eq, {1e2}, {0., 0., 1.});

  //slow & down-up
  komo.addObjective({2.}, FS_qItself, {}, OT_eq, {}, {}, 1);
  komo.addObjective({1.9,2.1}, FS_position, {"gripper"}, OT_eq, {}, {0.,0.,.1}, 2);

  komo.report(true, false);

  //== get info from the factored problem
  {
    std::shared_ptr<NLP_Factored> nlp = komo.nlp_FactoredParts();
    nlp->report(cout, 5);
  }

  //== three equivalent options to solve the full problem:
#if 0
  komo.opt.verbose = 4;
  switch(2){
    case 0: { //old style
      komo.optimize();
    } break;
    case 1: { //generic solver with exact same transcription as old-style
      auto nlp = komo.nlp();
      NLP_Solver()
          .setProblem(nlp)
          .solve();
      nlp->report(cout, 2);
    } break;
    case 2: { //generic solver with factored transcription
      auto nlp = komo.nlp_FactoredParts();
      NLP_Solver()
          .setProblem(nlp)
          .solve();
      nlp->report(cout, 2);
    } break;
  }

  komo.view(true, "optimized motion");
  while(komo.view_play(true));
#endif

  //== testing a partial problem
  std::shared_ptr<NLP_Factored> nlp = komo.nlp_FactoredParts();

  uintA gripperDUP_vars;
  for(uint i=0;i<nlp->variableDimensions.N;i++){
    if(nlp->getVariableName(i).startsWith("gripperDUP")) gripperDUP_vars.append(i);
  }
  cout <<gripperDUP_vars <<endl;

  nlp->subSelect(gripperDUP_vars, {});

  cout <<"======== SUBSELECT ==========" <<endl;
  nlp->report(cout, 5);
  komo.view(true);

  nlp->checkJacobian(komo.x, 1e-6);

  NLP_Solver()
      .setProblem(nlp)
      .solve();

  nlp->report(cout, 3);
  komo.report(true, false);
  komo.view(true);
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  rnd.clockSeed();

  testFactored();

  return 0;
}

