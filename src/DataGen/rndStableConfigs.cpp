#include "rndStableConfigs.h"

#include "../KOMO/komo.h"
#include "../Kin/F_forces.h"
#include "../Kin/viewer.h"
#include "../Optim/NLP_Solver.h"
#include "../Optim/NLP_Sampler.h"

bool RndStableConfigs::getSample(rai::Configuration& C, const StringA& supports, bool forceAll){
  //  FrameL colls = C.getCollidablePairs();
  //  cout <<"COLLIDABLE PAIRS:" <<endl;
  //  for(uint k=0;k<colls.d0;k++) cout <<"  " <<colls(k,0)->name <<' ' <<colls(k,1)->name <<endl;

  C.setRandom(0, 0);

  KOMO komo;
  komo.setConfig(C);
  komo.setTiming(1,1,1,0);
  komo.addControlObjective({}, 0, 1e-1);
  komo.add_jointLimits(true);

  komo.addObjective({}, make_shared<F_TotalForce>(), {"obj"}, OT_eq, {1e1} );
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1} );

  if(opt.verbose>0) komo.set_viewer(C.get_viewer());

  //-- discrete decisions:
  uintA perm = rai::randperm(supports.N);
  uint n_supports = rnd.uni_int(1,3);
  if(forceAll) n_supports = supports.N;
  supp.clear();
  for(uint i=0;i<n_supports;i++){
    str s = supports(perm(i));
    supp.append(s);
    // komo.addContact_stick(0.,-1., "obj", thing, opt.frictionCone_mu);
    komo.addContact_WithPoaFrame(1., "obj", s, opt.frictionCone_mu, .05, .1);
  }
  for(uint i=n_supports;i<supports.N;i++){
    //for all NON-supports, introduce an explicit(!) no-collision
    komo.addObjective({}, FS_negDistance, {"obj", supports(perm(i))}, OT_ineq, {1e1});
  }

  if(opt.verbose>0){
    LOG(0) <<"\n======================\n" <<supp;
  }
  if(opt.verbose>2){
    komo.view(true, STRING("supports = " <<supp));
  }

  for(uint k=0;k<1;k++){
    // komo.initRandom();
    // cout <<komo.pathConfig.reportForces() <<endl;
    // komo.view(true, STRING("init with supports: " <<supp));
    // komo.opt.animateOptimization = 2;
#if 1
    auto ret = rai::NLP_Solver(komo.nlp(), opt.verbose)
                   // .setOptions(rai::OptOptions().set_stopEvals(100))
                   .solve();
    //      komo.nlp()->checkJacobian(ret->x, 1e-4, komo.featureNames);
#else
    auto ret = NLP_Sampler(komo.nlp())
               // .setOptions(NLP_Sampler_Options().set_slackMaxStep(.2). set_downhillMaxSteps(50). set_slackRegLambda(.1) .set_tolerance(1e-4))
               .sample();
#endif


    std::tie(pairs,forces) = komo.pathConfig.getForceArrays();

    ret->setFeasible(1e-2);

    if(opt.verbose>0) LOG(0) <<*ret;
    //    cout <<komo.report() <<endl;
    if(opt.verbose>1){
      cout <<pairs <<forces <<endl;
    }

    totalRuns++;
    if(!ret->feasible){
      if(opt.verbose>2){
        komo.view(opt.verbose>3, STRING("failed" <<*ret));
      }
    }else{
      totalSucc++;
      totalEvals += ret->evals;
      if(opt.verbose>0) komo.view(opt.verbose>1, STRING(supp <<"\n" <<*ret));
      if(savePngs){
        C.get_viewer()->savePng();
      }
      C.setJointState(komo.getPath_qOrg()[0]);
      return true;
    }
  }

  return false;
}

void RndStableConfigs::report(){
  LOG(0) <<"TOTAL SUCC: " <<totalSucc
        <<"  evals/succ: " <<double(totalEvals)/totalSucc
       <<"  succ rate: " <<double(totalSucc)/totalRuns
      <<"  runs: " <<totalRuns;
}
