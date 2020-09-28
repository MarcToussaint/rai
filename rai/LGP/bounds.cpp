/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "bounds.h"
//#include "../Kin/switch.h"
#include "../Kin/F_qFeatures.h"
#include "../Kin/F_forces.h"
#include "../Kin/F_pose.h"

double conv_step2time(int step, uint stepsPerPhase);

template<> const char* rai::Enum<BoundType>::names []= {
  "symbolic",
  "pose",
  "seq",
  "path",
  "seqPath",
  "seqVelPath",
  "poseFromSub",
  "max",
  nullptr
};

rai::Array<SkeletonSymbol> modes = { SY_stable, SY_stableOn, SY_dynamic, SY_dynamicOn, SY_dynamicTrans, };

ptr<ComputeObject> skeleton2Bound(ptr<KOMO>& komo, BoundType boundType, const Skeleton& S,
                                  const rai::Configuration& startKinematics,
                                  bool collisions, const arrA& waypoints) {

  if(boundType==BD_pose)
    return make_shared<PoseBound>(komo, S, startKinematics, collisions);

  if(boundType==BD_seq || boundType==BD_poseFromSeq)
    return make_shared<SeqBound>(komo, S, startKinematics, collisions);

  if(boundType==BD_path)
    return make_shared<PathBound>(komo, S, startKinematics, collisions);

  if(boundType==BD_seqPath)
    return make_shared<SeqPathBound>(komo, S, startKinematics, collisions, waypoints);

  if(boundType==BD_seqVelPath)
    return make_shared<SeqVelPathBound>(komo, S, startKinematics, collisions, waypoints);

  HALT("should not be here!");

  return ptr<ComputeObject>();
}

//===========================================================================

double getMaxPhase(const Skeleton& S) {
  double maxPhase=0;
  for(const SkeletonEntry& s:S) {
    if(s.phase0>maxPhase) maxPhase=s.phase0;
    if(s.phase1>maxPhase) maxPhase=s.phase1;
  }
  return maxPhase;
}

PoseBound::PoseBound(ptr<KOMO>& komo,
                     const Skeleton& S, const rai::Configuration& startKinematics,
                     bool collisions)
  : KOMO_based_bound(komo) {

  double maxPhase = getMaxPhase(S);
  komo->clearObjectives();

  //-- prepare the komo problem
  double optHorizon=maxPhase;
  if(optHorizon<1.) optHorizon=maxPhase=1.;
  if(optHorizon>2.) optHorizon=2.;

  //-- remove non-switches
  Skeleton finalS;
  for(const SkeletonEntry& s:S) {
    if(modes.contains(s.symbol)
        || s.phase0>=maxPhase) {
      SkeletonEntry& fs = finalS.append(s);
      fs.phase0 -= maxPhase-optHorizon;
      fs.phase1 -= maxPhase-optHorizon;
      if(fs.phase0<0.) fs.phase0=0.;
      if(fs.phase1<0.) fs.phase1=0.;
    }
  }
#if 0
  //-- grep only the latest entries in the skeleton
  Skeleton finalS;
  for(const SkeletonEntry& s:S) if(s.phase0>=maxPhase) {
      finalS.append(s);
      finalS.last().phase0 -= maxPhase-1.;
      finalS.last().phase1 -= maxPhase-1.;
    }
#endif

  if(komo->verbose>1) {
    cout <<"POSE skeleton:" <<endl;
    writeSkeleton(cout, finalS, getSwitchesFromSkeleton(finalS));
  }

  komo->setModel(startKinematics, collisions);
  komo->setTiming(optHorizon, 1, 10., 1);

  komo->addSquaredQuaternionNorms();
#if 0
  komo->setHoming(0., -1., 1e-2);
  komo->setSquaredQVelocities(1., -1., 1e-1); //IMPORTANT: do not penalize transitions of from prefix to x_{0} -> x_{0} is 'loose'
#else
  komo->add_qControlObjective({}, 1, 1e-2);
  komo->add_qControlObjective({}, 0, 1e-2);
#endif

  komo->setSkeleton(finalS, false);

  //-- deactivate all velocity objectives except for transition
  //      for(Objective *o:komo->objectives){
  //        if((std::dynamic_pointer_cast<TM_ZeroQVel>(o->feat)
  //           || std::dynamic_pointer_cast<TM_Default>(o->feat))
  //           && o->feat->order==1){
  //          o->vars.clear();
  //        }
  //      }
  for(ptr<Objective>& o:komo->objectives) {
    if(!std::dynamic_pointer_cast<F_qItself>(o->feat)
//        && !std::dynamic_pointer_cast<TM_NoJumpFromParent>(o->feat)
       && !std::dynamic_pointer_cast<F_Pose>(o->feat)
       && !std::dynamic_pointer_cast<F_PoseRel>(o->feat)
       && o->feat->order>0) {
      o->configs.clear();
    }
  }
  for(ptr<GroundedObjective>& o:komo->objs) {
    if(!std::dynamic_pointer_cast<F_qItself>(o->feat)
       && !std::dynamic_pointer_cast<F_Pose>(o->feat)
       && !std::dynamic_pointer_cast<F_PoseRel>(o->feat)
       && o->feat->order>0) {
      o->feat.reset();
    }
  }
  for(uint i=komo->objs.N;i--;) if(!komo->objs(i)->feat){
    komo->objs.remove(i);
  }


  if(collisions) komo->add_collision(false);

  komo->run_prepare(.01);
  //      komo->setPairedTimes();
}

SeqBound::SeqBound(ptr<KOMO>& komo,
                   const Skeleton& S, const rai::Configuration& startKinematics,
                   bool collisions)
  : KOMO_based_bound(komo) {

  double maxPhase = getMaxPhase(S);
  komo->clearObjectives();

  komo->setModel(startKinematics, collisions);
  komo->setTiming(maxPhase+1., 1, 5., 1);
//  komo->solver=rai::KS_sparse; //sparseOptimization = true;
  komo->animateOptimization = 0;

  komo->addSquaredQuaternionNorms();
#if 0
  komo->setHoming(0., -1., 1e-2);
  komo->setSquaredQVelocities(0., -1., 1e-2);
#else
  komo->add_qControlObjective({}, 1, 1e-2);
  komo->add_qControlObjective({}, 0, 1e-2);
#endif
  komo->setSkeleton(S);

  if(collisions) komo->add_collision(true);

  komo->run_prepare(.01);
//      komo->setPairedTimes();
  //      cout <<komo->getPath_times() <<endl;

}

PathBound::PathBound(ptr<KOMO>& komo,
                     const Skeleton& S, const rai::Configuration& startKinematics,
                     bool collisions)
  : KOMO_based_bound(komo) {

  double maxPhase = getMaxPhase(S);
  komo->clearObjectives();

  komo->setModel(startKinematics, collisions);
  uint stepsPerPhase = rai::getParameter<uint>("LGP/stepsPerPhase", 10);
  uint pathOrder = rai::getParameter<uint>("LGP/pathOrder", 2);
  komo->setTiming(maxPhase+.5, stepsPerPhase, 10., pathOrder);
  komo->animateOptimization = 0;

  komo->addSquaredQuaternionNorms();
#if 0
  komo->setHoming(0., -1., 1e-2);
  if(pathOrder==1) komo->setSquaredQVelocities();
  else komo->setSquaredQAccelerations();
#else
  komo->add_qControlObjective({}, 2, 1.);
  komo->add_qControlObjective({}, 0, 1e-2);
#endif

  komo->setSkeleton(S);

  if(collisions) komo->add_collision(true, 0., 1e1);

  komo->run_prepare(.01);
  //      cout <<komo->getPath_times() <<endl;

}

SeqPathBound::SeqPathBound(ptr<KOMO>& komo,
                           const Skeleton& S, const rai::Configuration& startKinematics,
                           bool collisions, const arrA& waypoints)
  : KOMO_based_bound(komo) {

  double maxPhase = getMaxPhase(S);
  komo->clearObjectives();

  komo->setModel(startKinematics, collisions);
  uint stepsPerPhase = rai::getParameter<uint>("LGP/stepsPerPhase", 10);
  uint pathOrder = rai::getParameter<uint>("LGP/pathOrder", 2);
  komo->setTiming(maxPhase+.5, stepsPerPhase, 10., pathOrder);
  komo->animateOptimization = 0;

  komo->addSquaredQuaternionNorms();
#if 0
  komo->setHoming(0., -1., 1e-2);
  if(pathOrder==1) komo->setSquaredQVelocities();
  else komo->setSquaredQAccelerations();
#else
  komo->add_qControlObjective({}, 2, 1.);
  komo->add_qControlObjective({}, 0, 1e-2);
#endif

  uint T = floor(maxPhase+.5);
  uint waypointsStepsPerPhase = waypoints.N/(T+1);
  CHECK_EQ(waypoints.N, waypointsStepsPerPhase * (T+1), "waypoint steps not clear");
#if 0 //impose waypoint costs?
  for(uint i=0; i<waypoints.N-1; i++) {
    komo->addObjective(ARR(conv_step2time(i, waypointsStepsPerPhase)), FS_qItself, {}, OT_sos, {1e-1}, waypoints(i));
  }
#endif

  komo->setSkeleton(S);
  //delete all added objectives! -> only keep switches
  //      uint O = komo->objectives.N;
  //      for(uint i=O; i<komo->objectives.N; i++) delete komo->objectives(i);
  //      komo->objectives.resizeCopy(O);

  if(collisions) komo->add_collision(true, 0., 1e1);

  komo->run_prepare(.01);
  komo->initWithWaypoints(waypoints, waypointsStepsPerPhase);
  //      cout <<komo->getPath_times() <<endl;

}

SeqVelPathBound::SeqVelPathBound(ptr<KOMO>& komo,
                                 const Skeleton& S, const rai::Configuration& startKinematics,
                                 bool collisions, const arrA& waypoints)
  : KOMO_based_bound(komo) {

  double maxPhase = getMaxPhase(S);
  komo->clearObjectives();

  komo->setModel(startKinematics, collisions);
  uint stepsPerPhase = rai::getParameter<uint>("LGP/stepsPerPhase", 10);
  komo->setTiming(maxPhase+.5, stepsPerPhase, 10., 1);

  komo->add_qControlObjective({}, 1, 1.);
  komo->add_qControlObjective({}, 0, 1e-2);
  komo->addSquaredQuaternionNorms();

  CHECK_EQ(waypoints.N-1, floor(maxPhase+.5), "");
  for(uint i=0; i<waypoints.N-1; i++) {
    komo->addObjective(ARR(double(i+1)), FS_qItself, {}, OT_sos, {1e-1}, waypoints(i));
//        komo->addObjective(ARR(double(i+1)), FS_qItself, {}, OT_eq, {1e0}, waypoints(i));
  }
//      uint O = komo->objectives.N;

  komo->setSkeleton(S);
  //delete all added objectives! -> only keep switches
//      for(uint i=O; i<komo->objectives.N; i++) delete komo->objectives(i);
//      komo->objectives.resizeCopy(O);

  if(collisions) komo->add_collision(true, 0, 1e1);

  komo->run_prepare(.01);
  komo->initWithWaypoints(waypoints, false);
  //      cout <<komo->getPath_times() <<endl;

}

//===========================================================================

ptr<CG> skeleton2CGO(const Skeleton& S, const rai::Configuration& startKinematics, bool collisions) {
  cout <<"*** " <<RAI_HERE <<endl;
  writeSkeleton(cout, S);

  double maxPhase=0;
  for(const SkeletonEntry& s:S) {
    if(s.phase0>maxPhase) maxPhase=s.phase0;
    if(s.phase1>maxPhase) maxPhase=s.phase1;
  }
  cout <<"maxPhase:" <<maxPhase <<endl;

  StringA frames;
  for(const SkeletonEntry& s:S) {
    frames.append(s.frames);
  }
  frames.sort().removeDoublesInSorted();
  cout <<"frames: " <<frames <<endl;

  //-- create graph vertices
  ptr<CG> cg = make_shared<CG>();
  for(int t=0; t<=maxPhase; t++) {
    for(rai::String& f:frames) {
      cg->G.newNode<int>({STRING(f <<'_' <<t)}, {}, t);
    }
  }

  //-- add graph constraints
  for(const SkeletonEntry& s:S) {
    int s_end=s.phase1;
    if(s_end<0) s_end = maxPhase;
    for(int t=s.phase0; t<=s_end; t++) {
      rai::NodeL parents;
      for(const rai::String& f:s.frames) parents.append(cg->G.getNode(STRING(f <<'_' <<t)));
      cg->G.newNode<rai::Enum<SkeletonSymbol>>({}, parents, s.symbol);
      if(s.symbol==SY_stable || s.symbol==SY_stableOn) {
        for(int t2=t+1; t2<=s_end+1 && t2<=maxPhase; t2++) {
          rai::NodeL parents2=parents;
          for(const rai::String& f:s.frames) parents2.append(cg->G.getNode(STRING(f <<'_' <<t2)));
          cg->G.newNode<rai::Enum<SkeletonSymbol>>({}, parents2, s.symbol);
        }
      }
    }
  }
//  cout <<"initial CGO:" <<cg->G <<endl;

  //-- add implicit identical and collision constraints
  for(rai::String& f:frames) {
    for(uint t=0; t<=maxPhase; t++) {
      bool stop=false;
      rai::Node* n = cg->G.getNode(STRING(f <<'_' <<t));
      for(rai::Node* c:n->children) {
        SkeletonSymbol s = c->getValue<rai::Enum<SkeletonSymbol>>()->x;
        if(s==SY_stable
            || s==SY_stableOn
            || s==SY_dynamic
            || s==SY_dynamicOn
            || s==SY_dynamicTrans) stop=true;
        if(stop) break;
      }
      if(t>0) {
        cg->G.newNode<rai::Enum<SkeletonSymbol>>({}, {cg->G.getNode(STRING(f <<'_' <<t-1)), n}, rai::Enum<SkeletonSymbol>(SY_identical));
      }
      if(stop) break;
    }
  }
  for(rai::String& f0:frames) if(f0.startsWith("obj")) {
      for(rai::String& f1:frames) if(f1!=f0 && f0<=f1 && f1.startsWith("obj")) {
          for(uint t=0; t<=maxPhase; t++) {
            cg->G.newNode<rai::Enum<SkeletonSymbol>>({},
            {cg->G.getNode(STRING(f0 <<'_' <<t)), cg->G.getNode(STRING(f1 <<'_' <<t))},
            rai::Enum<SkeletonSymbol>(SY_noCollision));
          }
        }
    }
//  cout <<"CGO with identicals:" <<cg->G <<endl;

  //-- collapse identicals
  rai::NodeL ids;
  for(rai::Node* c:cg->G) {
    if(c->isOfType<rai::Enum<SkeletonSymbol>>()
        && c->getValue<rai::Enum<SkeletonSymbol>>()->x == SY_identical) ids.append(c);
  }
  for(rai::Node* c:ids) {
    rai::Node* a=c->parents(0), *b=c->parents(1);
    cg->G.collapse(a, b);
#if 1
    //delete duplicate children
    rai::NodeL dels;
    for(uint i=0; i<a->children.N; i++) for(uint j=i+1; j<a->children.N; j++) {
        rai::Node* ch1 = a->children.elem(i);
        rai::Node* ch2 = a->children.elem(j);
        if(ch1!=ch2 && ch1->parents==ch2->parents && ch1->hasEqualValue(ch2)) dels.append(ch2);
      }
    for(rai::Node* d:dels) delete d;
#endif
  }

  cout <<"CGO with identicals removed:\n" <<cg->G <<endl;

//  cg->G.displayDot();

  //-- generate list of subproblems

  //size 1
#if 1
  StringA noSeeds = {"initial", "stable", "stableOn", "noCollision"};
  for(rai::Node* c:cg->G) {
    if(c->isOfType<rai::Enum<SkeletonSymbol>>()
        && !noSeeds.contains(c->getValue<rai::Enum<SkeletonSymbol>>()->name())) {

      auto sp = make_shared<SubCG>();
      cg->subproblems.append(sp);

      sp->maxT = (uint)maxPhase;
      //frames
      for(rai::Node* p:c->parents) sp->frames.append(p);
      sp->frames.sort();
      //constraints
      for(rai::Node* c2:cg->G) {
        if(c2->isOfType<rai::Enum<SkeletonSymbol>>()
            && sp->frames.contains(c2->parents)) {
          sp->constraints.append(c2);
        }
      }
    }
  }
#else
  for(Node* v:cg->G) {
    if(v->isOfType<int>()) {
      auto sp = make_shared<SubCG>();
      cg->subproblems.append(sp);

      sp->maxT = (uint)maxPhase;
      //frames
      sp->frames.append(v);
      //constraints
      for(Node* c2:cg->G) {
        if(c2->isOfType<rai::Enum<SkeletonSymbol>>()
            && sp->frames.contains(c2->parents)) {
          sp->constraints.append(c2);
        }
      }
    }
  }
#endif

//  cout <<"PROBLEMS size 1:" <<cg->subproblems;
  return cg;

  //size 2
  for(uint lev=1;; lev++) {
    //compute distances between subproblems
    arr D(cg->subproblems.N, cg->subproblems.N);
    D = -1;
    double mind=-1;
    int mini=-1, minj=-1;
    for(uint i=0; i<D.d0; i++) for(uint j=i+1; j<D.d1; j++) {
        rai::NodeL join = cat(cg->subproblems(i)->frames, cg->subproblems(j)->frames).sort().removeDoublesInSorted();
        bool exists=false;
        for(auto& sp:cg->subproblems) if(join==sp->frames) { exists=true; break; }
        if(exists) continue;

        double d = distance(cg->subproblems(i)->frames, cg->subproblems(j)->frames);
        if(d>=0.) d += 0.01 * join.N;
        D(i, j) = D(j, i) = d;
        if(d>=0. && (mind<0 || d<mind)) { mind=d; mini=i; minj=j; }
      }
    cout <<"SP distance:\n" <<D <<endl;
    rai::wait();
    if(mind==-1.) break;

    //merge the pair with minimal distance
    auto sp = make_shared<SubCG>();
    cg->subproblems.append(sp);
    sp->frames = cat(cg->subproblems(mini)->frames, cg->subproblems(minj)->frames);
    sp->frames.sort();
    sp->frames.removeDoublesInSorted();
    for(rai::Node* c:cg->G) {
      if(c->isOfType<rai::Enum<SkeletonSymbol>>()
          && sp->frames.contains(c->parents)) {
        sp->constraints.append(c);
      }
    }
    cg->subproblems(mini)->merged = minj;
    cg->subproblems(minj)->merged = mini;
    cout <<"NEW PROBLEM:\n" <<*sp;
  }

  return cg;
}

void CG2komo(KOMO& komo, const SubCG& scg, const rai::Configuration& C, bool collisions) {
  //C is the template; pick only the relevant frames
  rai::Configuration* c = komo.configurations.append(new rai::Configuration);
  rai::Array<FrameL> framesPerT(scg.maxT+1);
  for(rai::Node* f:scg.frames) {
    rai::String obj=f->key;
    uint t=0;
    obj.getLastN(1) >>t;
    obj = obj.getFirstN(obj.N-2);
    rai::Frame* f_C = C.getFrameByName(obj);
    CHECK(f_C, "");
    framesPerT(t).append(f_C->getPathToRoot());
  }
  for(uint t=0; t<=scg.maxT; t++) {
    framesPerT(t).sort().removeDoublesInSorted();
    c->addFramesCopy(framesPerT(t));
//    if(!t) for(rai::Frame *f:c->frames) if(f->joint) delete f->joint; //kill all DOFs at t=0
  }
  //delete joints without parents
//  for(rai::Frame *f:c->frames)
//    if(!f->parent && f->joint) delete f->joint;
  c->checkConsistency();

  c->ensure_q();
  cout <<*c <<endl;
  c->report(cout);
  c->watch(true);
}

//===========================================================================
