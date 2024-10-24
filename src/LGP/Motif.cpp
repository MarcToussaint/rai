#include "Motif.h"

#include "../KOMO/komo_NLP.h"
#include "../KOMO/objective.h"
#include "../Optim/NLP_Solver.h"
#include "../Optim/NLP_Sampler.h"
#include "../Kin/frame.h"
#include "../Kin/feature.h"

bool KOMO_Motif::matches(GroundedObjective* ob, int _timeSlice){
  CHECK(objs.N, "");
  if(_timeSlice != timeSlice) return false;
  FrameL shared = setSection(F, ob->frames); //OPTION! Motifs are time slices? Or shared frames?
  if(!shared.N) return false;
  return true;
}

rai::String KOMO_Motif::getHash(){
  str hash;
  hash <<"#objs" <<objs.N;
  for(GroundedObjective* o:objs){
    hash <<'=' <<o->type <<'-' <<o->feat->shortTag(F.elem(0)->C);
  }

  rai::Configuration& C=F.elem(0)->C;
  DofL dofs;
  dofs = getDofs(C, 0);
  for(rai::Dof *d:dofs){
    hash <<'+' <<d->frame->name;
  }
  return hash;
}

DofL KOMO_Motif::getDofs(rai::Configuration& C, int verbose){
  CHECK_EQ(&C, &F.elem(0)->C, "");

  FrameL selected = F;
  if(verbose>2){ cout <<" selected frames: "; for(rai::Frame* f:selected) cout <<f->name <<'[' <<f->time <<"] "; cout <<endl; }
  //-- all sub frames within komo
  FrameL subFrames;
  for(rai::Frame *f:selected){
    for(rai::Frame *b:f->getPathToRoot()) subFrames.setAppend(b);
  }
  if(verbose>2){ cout <<" all frames: "; for(rai::Frame* f:subFrames) cout <<f->name <<'[' <<f->time <<"] "; cout <<endl; }

  //-- get dofs
  C.ensure_indexedJoints();
  DofL dofs = C.getDofs(subFrames, true, false, true);
  if(verbose>2){ cout <<" all dofs: "; for(rai::Dof* d:dofs) cout <<d->frame->name <<'[' <<d->frame->time <<"] "; cout <<endl; }
  return dofs;
}

std::shared_ptr<SolverReturn> KOMO_Motif::solve(KOMO& komo, str opt_or_sample, int verbose){
#if 0
  //-- selected frames within komo
  FrameL selected;
  for(const auto& ob:pac->objs){
    for(uint i:ob->feat->frameIDs){
      for(int s = 0;s<=int(ob->feat->order);s++){
        selected.setAppend( komo.timeSlices(int(komo.k_order) - s + pac->timeSlice, i) );
      }
    }
  }
  cout <<" selected frames: "; for(rai::Frame* f:selected) cout <<f->name <<'[' <<f->time <<"] "; cout <<endl;
  //-- all sub frames within komo
  FrameL subFrames;
  for(rai::Frame *f:selected){
    for(rai::Frame *b:f->getPathToRoot()){
      subFrames.setAppend(b);
      for(rai::Frame *c:b->children) if(c->shape) subFrames.setAppend(c);
    }
    //    for(rai::Frame *c:f->getUpwardLink()->getSubtree()) subFrames.setAppend(c);
  }
  cout <<" all frames: "; for(rai::Frame* f:subFrames) cout <<f->name <<'[' <<f->time <<"] "; cout <<endl;
  //-- copy those to subC, including DOFs
  rai::Configuration subC;
  subC.addCopy(subFrames, {});

  cout <<" sub problem size: " <<subC.frames.N <<' ' <<subC.getJointStateDimension() <<endl;
  subC.view(true);
  //  subC.animate();
#endif

  //-- get dofs for each pac
  DofL dofs = getDofs(komo.pathConfig, verbose);
  if(!dofs.N){
    if(verbose>0) cout <<"non dof problem -> assuming solved!" <<endl;
    auto ret = make_shared<SolverReturn>();
    ret->feasible=true;
    ret->done=true;
    return ret;
  }

  komo.pathConfig.ensure_indexedJoints();
  DofL orgDofs = komo.pathConfig.activeDofs;

  std::shared_ptr<rai::KOMO_SubNLP> nlp = make_shared<rai::KOMO_SubNLP>(komo, objs, dofs);
//  arr x = nlp->getInitializationSample();
//  nlp->checkJacobian(x, 1e-6, nlp->featureNames);

  std::shared_ptr<SolverReturn> ret;
  if(opt_or_sample=="opt"){
    NLP_Solver sol;
    sol.setProblem(nlp);
    ret = sol.solve(0, verbose);
  }else{
    NLP_Sampler sol(nlp);
    sol.opt.seedMethod=opt_or_sample;
    sol.opt.verbose=verbose;
    sol.opt.downhillMaxSteps=50;
    sol.opt.slackStepAlpha=.5;
    sol.opt.slackMaxStep=.2;
    ret = sol.sample();
  }
//  cout <<*ret <<endl;
  nlp.reset();

  komo.pathConfig.selectJoints(orgDofs); //undo selectJoint that is done internally by KOMO_SubNLP
  //    cout <<"#q=" <<komo.pathConfig.getJointStateDimension() <<endl;
  //  cout <<"**SELECTED:\n" <<komo.report(false, true, false) <<endl;

  return ret;
}
