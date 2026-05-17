#include "rndStableConfigs.h"

#include "../KOMO/komo.h"
#include "../Kin/F_forces.h"
#include "../Kin/viewer.h"
#include "../Optim/NLP_Solver.h"
#include "../Optim/NLP_Sampler.h"
#include "../Kin/dof_forceExchange.h"
#include "../Kin/F_collisions.h"
#include "../Kin/F_pose.h"

struct F_Upright : Feature {
  arr S;
  F_Upright(uint i) {
    S = zeros(2,9);
    if(i==0){ S(0,5)=1.; S(1,8)=1.; }
    if(i==1){ S(0,2)=1.; S(1,8)=1.; }
    if(i==2){ S(0,2)=1.; S(1,5)=1.; }
  }
  virtual arr phi(const FrameL& F){
    arr R = F_Matrix().eval(F);
    return S * R;
  }
  virtual uint dim_phi(const FrameL& F) { return 2; }
};

bool RndStableConfigs::getSample(rai::Configuration& C, const StringA& must_supports, const StringA& rnd_supports, const StringA& collides, uint max_n_supports, uint min_n_supports){
  //  FrameL colls = C.getCollidablePairs();
  //  cout <<"COLLIDABLE PAIRS:" <<endl;
  //  for(uint k=0;k<colls.d0;k++) cout <<"  " <<colls(k,0)->name <<' ' <<colls(k,1)->name <<endl;

  C.setRandom(0, 0);
  arr qRnd = C.getJointState();

  // if(opt.verbose>2) C.view(true, "random");

  if(false){ //first make configuration collision free
    KOMO komo(C, 1,1,0, true);
    // komo.addControlObjective({}, 0, 1e-1);
    komo.addObjective({}, FS_qItself, {}, OT_sos, {1e-2}, qRnd);
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1} );
    komo.addQuaternionNorms();
    auto ret = rai::NLP_Solver(komo.nlp(), opt.verbose)
                   .solve();
    C.setJointState(ret->x);
    // if(opt.verbose>2){
    //   cout <<*ret <<endl;
    //   komo.view(false, "no collide");
    //   C.view(true, "no collide");
    // }
    if(!ret->feasible) return false;
  }

  KOMO komo(C, 1,1,0, true);
  // komo.addControlObjective({}, 0, 1e-1);
  komo.addObjective({}, FS_qItself, {}, OT_sos, {1e-2}, qRnd);
  komo.addObjective({}, FS_jointLimits, {}, OT_ineq, {1e0});
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1} );

  komo.addObjective({}, make_shared<F_TotalForce>(), {"obj"}, OT_eq, {1e1} );

  if(opt.verbose>0) komo.set_viewer(C.get_viewer());

  //-- discrete decisions:
  supp.clear();
  uint n_supports = rnd.uni_int(min_n_supports, max_n_supports);
  uint n_rnd_supports = n_supports - must_supports.N;
  if(must_supports.N) supp.append(must_supports.rndElem());
  uintA perm = rai::randperm(rnd_supports.N);
  for(uint i=0;i<n_rnd_supports;i++) supp.append(rnd_supports(perm(i)));

  if(opt.verbose>0){
    LOG(0) <<"\n======================\n" <<supp;
  }

  for(const str& s:supp){
    // komo.addContact_stick(0.,-1., "obj", thing, opt.frictionCone_mu);
    // komo.addContact_WithPoaFrame(1., "obj", s, opt.frictionCone_mu, .05, .2);
    komo.addContactForceFrame({1.}, "obj", s, opt.frictionCone_mu, .1);
  }
  for(uint i=n_rnd_supports;i<rnd_supports.N;i++){
    //for all NON-supports, introduce an explicit(!) no-collision
    komo.addObjective({}, FS_negDistance, {"obj", rnd_supports(perm(i))}, OT_ineq, {1e1});
  }
  for(const str& s: collides){
    komo.addObjective({}, FS_negDistance, {"obj", s}, OT_ineq, {1e1});
  }
  if(supp.N==1 && supp.elem()=="table" && C.getFrame("obj")->shape->type()==rai::ST_ssBox){
    // if(rnd.uni()<.5){
    uint i = rnd.uni_int(0,2);
    if(opt.verbose>0) LOG(0) <<"+++++++++++++++++++ adding orthogonality: dir " <<i <<endl;
    komo.addObjective({}, make_shared<F_Upright>(i), {"obj"}, OT_eq, {1e1});
    // }
  }

  komo.addQuaternionNorms();

  if(opt.verbose>2){
    komo.view(true, STRING("supports = " <<supp));
  }

  for(uint k=0;k<1;k++){
    // komo.initRandom();
    // cout <<komo.pathConfig.reportForces() <<endl;
    // komo.view(true, STRING("init with supports: " <<supp));
    if(opt.verbose>4) komo.opt.animateOptimization = opt.verbose-4;
#if 1
    auto ret = rai::NLP_Solver(komo.nlp(), opt.verbose-1)
                   // .setOptions(rai::OptOptions().set_stopEvals(100))
                   .solve();
         // komo.nlp()->checkJacobian(ret->x, 1e-4, komo.featureNames);
#else
    auto ret = NLP_Sampler(komo.nlp())
               // .setOptions(NLP_Sampler_Options().set_slackMaxStep(.2). set_downhillMaxSteps(50). set_slackRegLambda(.1) .set_tolerance(1e-4))
               .sample();
#endif


    std::tie(pairs,forces) = komo.pathConfig.getForceArrays();

    ret->setFeasible(1e-2);

    if(opt.verbose>0) LOG(0) <<*ret;
    //    cout <<komo.report() <<endl;
    if(opt.verbose>2){
      cout <<"FORCES:\n" <<pairs <<forces <<endl;
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
