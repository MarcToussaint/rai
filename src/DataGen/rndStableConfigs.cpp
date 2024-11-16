#include "rndStableConfigs.h"

#include "../KOMO/komo.h"
#include "../Kin/F_forces.h"
#include "../Kin/viewer.h"
#include "../Optim/NLP_Solver.h"
#include "../Optim/NLP_Sampler.h"

bool RndStableConfigs::getSample(rai::Configuration& C, const StringA& supports){
  //  FrameL colls = C.getCollidablePairs();
  //  cout <<"COLLIDABLE PAIRS:" <<endl;
  //  for(uint k=0;k<colls.d0;k++) cout <<"  " <<colls(k,0)->name <<' ' <<colls(k,1)->name <<endl;


  KOMO komo;
  komo.setConfig(C);
  komo.setTiming(1,1,1,0);
  komo.addControlObjective({}, 0, 1e-1);

  komo.addObjective({}, make_shared<F_TotalForce>(), {"obj"}, OT_eq, {1e1} );
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1} );

  if(opt.verbose>0) komo.set_viewer(C.get_viewer());

  //-- discrete decisions:
  str supp="supports:";
  for(const str& thing:supports){
    if(rnd.uni()<.5){
      supp <<' ' <<thing;
      komo.addContact_stick(0.,-1., "obj", thing, opt.frictionCone_mu);
    }
  }
  if(opt.verbose>0){
    LOG(0) <<"\n======================\n" <<supp;
  }

  for(uint k=0;k<10;k++){
    komo.initRandom();
    //komo.view(true, "init");

#if 1
    auto ret = NLP_Solver(komo.nlp())
               .setOptions(rai::OptOptions().set_stopEvals(100))
               .solve();
    //      komo.nlp()->checkJacobian(ret->x, 1e-4, komo.featureNames);
#else
    auto ret = NLP_Sampler(komo.nlp())
               .setOptions(NLP_Sampler_Options().set_slackMaxStep(.2). set_downhillMaxSteps(100). set_slackRegLambda(.1) .set_tolerance(.01))
               .sample();
#endif

    if(opt.verbose>0) LOG(0) <<*ret;
    //    cout <<komo.report() <<endl;
    if(opt.verbose>2){
      cout <<komo.pathConfig.reportForces() <<endl;
    }
    if(!ret->feasible){
      if(opt.verbose>2){
        komo.view(opt.verbose>3, STRING("failed" <<*ret));
      }
    }else{
      totalSucc ++;
      totalEvals += ret->evals;
      if(opt.verbose>0) komo.view(opt.verbose>1, STRING(supp <<"\n" <<*ret));
      if(savePngs){
        C.get_viewer()->savePng();
      }
      return true;
    }
  }

  return false;
}

void RndStableConfigs::report(){
  LOG(0) <<"TOTAL SUCC: " <<totalSucc <<"\n   evals/succ: " <<double(totalEvals)/double(totalSucc) <<endl;
}
