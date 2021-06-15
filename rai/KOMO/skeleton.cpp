#include "skeleton.h"

#include "komo.h"

#include "../Kin/F_pose.h"
#include "../Kin/F_forces.h"
#include "../Kin/F_qFeatures.h"
#include "../Optim/solver.h"
#include "../Logic/fol.h"


double shapeSize(const rai::Configuration& K, const char* name, uint i=2);

namespace rai {

Array<SkeletonSymbol> skeletonModes = { SY_stable, SY_stableOn, SY_dynamic, SY_dynamicOn, SY_dynamicTrans, SY_quasiStatic, SY_quasiStaticOn, SY_magicTrans };

void SkeletonEntry::write(std::ostream& os) const {
  os <<"[" <<phase0 <<", " <<phase1 <<"] " <<symbol <<' ';
  frames.write(os, " ", nullptr, "()");
}

void Skeleton::ensure_komo() {
  if(!komo) {
  }
}

void Skeleton::write(ostream& os, const intA& switches) const {
  os <<"SKELETON:";
  for(auto& s:S) os <<"\n  " <<s;
  if(switches.N) {
    os <<"SWITCHES:" <<endl;
    for(uint i=0; i<switches.d0; i++) {
      int j = switches(i, 0);
      if(j<0)
        os <<"  START  -->  " <<S(switches(i, 1)) <<endl;
      else
        os <<"  " <<S(j) <<"  -->  " <<S(switches(i, 1)) <<endl;
    }
  }
}

void Skeleton::setFromStateSequence(Array<Graph*>& states, const arr& times){
  //setup a done marker array: which literal in each state is DONE
  uint maxLen=0;
  for(Graph* s:states) if(s->N>maxLen) maxLen = s->N;
  boolA done(states.N, maxLen);
  done = false;

  for(uint k=0; k<states.N; k++) {
    Graph& G = *states(k);
//    cout <<G <<endl;
    for(uint i=0; i<G.N; i++) {
      if(!done(k, i)) {
        Node* n = G(i);
        if(n->isGraph() && n->graph().findNode("%decision")) continue; //don't pickup decision literals
        StringA symbols;
        for(Node* p:n->parents) symbols.append(p->key);

        //check if there is a predicate
        if(!symbols.N) continue;

        //check if predicate is a SkeletonSymbol
        if(!Enum<SkeletonSymbol>::contains(symbols.first())) continue;

        //trace into the future
        uint k_end=k+1;
        for(; k_end<states.N; k_end++) {
          Node* persists = getEqualFactInList(n, *states(k_end), true);
          if(!persists) break;
          done(k_end, persists->index) = true;
        }
        k_end--;

        Enum<SkeletonSymbol> sym(symbols.first());
        if(k_end==states.N-1) {
          S.append(SkeletonEntry({times(k), times.last(), sym, symbols({1, -1})}));
        } else {
          S.append(SkeletonEntry({times(k), times(k_end), sym, symbols({1, -1})}));
        }
      }
    }
  }

  for(uint i=0; i<S.N; i++) {
    SkeletonEntry& se =  S.elem(i);
    if(skeletonModes.contains(se.symbol)){ //S(i) is about a switch
      if(se.phase1<times.last()){
        se.phase1 += 1.; //*** MODES EXTEND TO THE /NEXT/ TIME SLICE ***
      }else{
        se.phase1 = -1.;
      }
    }
  }

}

double Skeleton::getMaxPhase() const {
  double maxPhase=0;
  for(const SkeletonEntry& s:S) {
    if(s.phase0>maxPhase) maxPhase=s.phase0;
    if(s.phase1>maxPhase) maxPhase=s.phase1;
  }
  return maxPhase;
}

intA Skeleton::getSwitches(const rai::Configuration& C) const {
  intA ret;
  for(int i=0; i<(int)S.N; i++) {
    if(skeletonModes.contains(S.elem(i).symbol)) { //S(i) is about a switch
      int j=i-1;
      rai::Frame* toBeSwitched = C[S.elem(i).frames(1)];
      CHECK(toBeSwitched,"");
      rai::Frame* rootOfSwitch = toBeSwitched->getUpwardLink(NoTransformation, true);
      rai::Frame* childOfSwitch = toBeSwitched->getDownwardLink(true);
      for(; j>=0; j--) {
        if(skeletonModes.contains(S.elem(j).symbol)) { //S(j) is about a switch
          const rai::String& prevSwitched = S.elem(j).frames(1);
          if(prevSwitched==toBeSwitched->name
              || prevSwitched==rootOfSwitch->name
              || prevSwitched==childOfSwitch->name)
            break;
        }
      }
      //j=-1 if not previously switched, otherwise the index of the previous switch
      ret.append({j, i});
    }
  }
  ret.reshape(ret.N/2, 2);

  return ret;
}

void Skeleton::solve() {
  CHECK(C, "");
  komo.reset();
  komo=make_shared<KOMO>();
  komo->setModel(*C, false);
  setKOMO(*komo, rai::_sequence);
  komo->optimize();
  //  komo->checkGradients();

  komo->getReport(true);
  komo->view(true, "optimized motion");
  while(komo->view_play(true));
}

shared_ptr<SolverReturn> Skeleton::solve2(){
  auto trans = this->mp();

  NLP_Solver sol;
  sol.setProblem(*trans.mp);

  auto ret = sol.solve();
  trans.mp->report(cout, 10);
  sol.gnuplot_costs();
  return ret;
}

SkeletonTranscription Skeleton::mp(){
  SkeletonTranscription ret;
  ret.komo=make_shared<KOMO>();
  ret.komo->verbose=verbose;
#if 0
  ret.komo->solver = rai::KS_sparse;
  ret.komo->setModel(*C, collisions);
  setKOMO(*ret.komo, rai::_sequence);
  ret.komo->run_prepare(0.);
#else
  ptr<KOMO> komo = ret.komo;
  double maxPhase = getMaxPhase();
  komo->clearObjectives();

  komo->setModel(*C, collisions);
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
  setKOMO(*komo);

  if(collisions) komo->add_collision(true);

  komo->run_prepare(.01);
#endif
  ret.mp=ret.komo->nlp_SparseNonFactored();
  return ret;
}

SkeletonTranscription Skeleton::mp_finalSlice(){
  SkeletonTranscription ret;
  ret.komo=make_shared<KOMO>();
  ret.komo->verbose=verbose;
  ptr<KOMO> komo = ret.komo;

  double maxPhase = getMaxPhase();
  komo->clearObjectives();

  //-- prepare the komo problem
  double optHorizon=maxPhase;
  if(optHorizon<1.) optHorizon=maxPhase=1.;
  if(optHorizon>2.) optHorizon=2.;

  //-- remove non-switches
  rai::Skeleton finalS;
  for(const rai::SkeletonEntry& s:S) {
    if(rai::skeletonModes.contains(s.symbol)
        || s.phase0>=maxPhase) {
      rai::SkeletonEntry& fs = finalS.S.append(s);
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
    finalS.write(cout, finalS.getSwitches(*C));
  }

  komo->setModel(*C, collisions);
  komo->setTiming(optHorizon, 1, 10., 1);

  komo->addSquaredQuaternionNorms();
#if 0
  komo->setHoming(0., -1., 1e-2);
  komo->setSquaredQVelocities(1., -1., 1e-1); //IMPORTANT: do not penalize transitions of from prefix to x_{0} -> x_{0} is 'loose'
#else
  komo->add_qControlObjective({}, 1, 1e-2);
  komo->add_qControlObjective({}, 0, 1e-2);
#endif

  finalS.setKOMO(*komo);

  //-- deactivate all velocity objectives except for transition
  for(ptr<Objective>& o:komo->objectives) {
    if(o->feat->order>0
       && !std::dynamic_pointer_cast<F_qItself>(o->feat)
       && !std::dynamic_pointer_cast<F_Pose>(o->feat)
       && !std::dynamic_pointer_cast<F_PoseRel>(o->feat)) {
      o->times={1e6};
    }
  }
  for(ptr<GroundedObjective>& o:komo->objs) {
    if(o->feat->order>0
       && !std::dynamic_pointer_cast<F_qItself>(o->feat)
       && !std::dynamic_pointer_cast<F_Pose>(o->feat)
       && !std::dynamic_pointer_cast<F_PoseRel>(o->feat)) {
      o->feat.reset();
    }
  }
  for(uint i=komo->objs.N;i--;) if(!komo->objs(i)->feat){
    komo->objs.remove(i);
  }

  if(collisions) komo->add_collision(false);

  komo->run_prepare(.01);
  //      komo->setPairedTimes();

  ret.mp=ret.komo->nlp_SparseNonFactored();
  return ret;
}

SkeletonTranscription Skeleton::mp_path(const arrA& waypoints){
  SkeletonTranscription ret;
  ret.komo=make_shared<KOMO>();
  ret.komo->verbose=verbose;
#if 0
  ret.komo->solver = rai::KS_sparse;
  ret.komo->setModel(*C, collisions);
  setKOMO(*ret.komo, rai::_path);
  ret.komo->run_prepare(0.);
#else
  ptr<KOMO> komo = ret.komo;
  double maxPhase = getMaxPhase();
  komo->clearObjectives();

  komo->setModel(*C, collisions);
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

  setKOMO(*komo);

  if(collisions) komo->add_collision(true, 0., 1e1);

  komo->run_prepare(.01);
#endif
  ret.mp=ret.komo->nlp_SparseNonFactored();
  return ret;

}

void Skeleton::setKOMO(KOMO& komo, ArgWord sequenceOrPath) const {
  //  if(sequenceOrPath==rai::_sequence){
  //    solver = rai::KS_dense;
  //  }else{
  //    solver = rai::KS_sparse;
  //  }

  double maxPhase = getMaxPhase();
  if(sequenceOrPath==rai::_sequence) {
    komo.setTiming(maxPhase, 1, 2., 1);
//    komo.setTiming(maxPhase+1., 1, 5., 1); //as defined in bounds.cpp
    komo.add_qControlObjective({}, 1, 1e-2);
    komo.add_qControlObjective({}, 0, 1e-2);
  } else {
    komo.setTiming(maxPhase, 30, 2., 2);
//    komo->setTiming(maxPhase+.5, 10, 10., 2); //as defined in bounds.cpp
    komo.add_qControlObjective({}, 2, 1.);
    komo.add_qControlObjective({}, 0, 1e-2);
  }
  komo.addSquaredQuaternionNorms();

  if(collisions) komo.add_collision(true);

  setKOMO(komo);

  komo.run_prepare(.01);
}

void Skeleton::setKOMO(KOMO& komo) const {
  //-- add objectives for mode switches
  intA switches = getSwitches(komo.world);
  for(uint i=0; i<switches.d0; i++) {
    int j = switches(i, 0);
    int k = switches(i, 1);
    komo.addModeSwitch({S(k).phase0, S(k).phase1}, S(k).symbol, S(k).frames, j<0);
  }
  //-- add objectives for rest
  for(const SkeletonEntry& s:S) {
    switch(s.symbol) {
      case SY_none:       HALT("should not be here");  break;
      case SY_end: break; //explicit redundant symbol, e.g. to mark the end of a skeleton
      case SY_touch:      komo.addObjective({s.phase0, s.phase1}, FS_distance, {s.frames(0), s.frames(1)}, OT_eq, {1e2});  break;
      case SY_above:      komo.addObjective({s.phase0, s.phase1}, FS_aboveBox, {s.frames(0), s.frames(1)}, OT_ineq, {1e1});  break;
      case SY_inside:     komo.addObjective({s.phase0, s.phase1}, FS_insideBox, {s.frames(0), s.frames(1)}, OT_ineq, {1e1});  break;
      //      case SY_inside:     komo.addObjective({s.phase0, s.phase1}, make_shared<TM_InsideLine>(world, s.frames(0), s.frames(1)), OT_ineq, {1e1});  break;
      case SY_oppose:     komo.addObjective({s.phase0, s.phase1}, FS_oppose, s.frames, OT_eq, {1e1});  break;

      case SY_topBoxGrasp: {
        komo.addObjective({s.phase0}, FS_positionDiff, s.frames, OT_eq, {1e2});
        komo.addObjective({s.phase0}, FS_scalarProductXX, s.frames, OT_eq, {1e2}, {0.});
        komo.addObjective({s.phase0}, FS_vectorZ, {s.frames(0)}, OT_eq, {1e2}, {0., 0., 1.});
        //slow - down - up
        if(komo.k_order>=2) {
          komo.addObjective({s.phase0}, FS_qItself, {}, OT_eq, {}, {}, 1);
          komo.addObjective({s.phase0-.1, s.phase0+.1}, FS_position, {s.frames(0)}, OT_eq, {}, {0., 0., .1}, 2);
        }
        break;
      }
      case SY_topBoxPlace: {
        komo.addObjective({s.phase0}, FS_positionDiff, {s.frames(1), s.frames(2)}, OT_eq, {1e2}, {0, 0, .08}); //arr({1,3},{0,0,1e2})
        komo.addObjective({s.phase0}, FS_vectorZ, {s.frames(0)}, OT_eq, {1e2}, {0., 0., 1.});
        //slow - down - up
        if(komo.k_order>=2) {
          komo.addObjective({s.phase0}, FS_qItself, {}, OT_eq, {}, {}, 1);
          komo.addObjective({s.phase0-.1, s.phase0+.1}, FS_position, {s.frames(0)}, OT_eq, {}, {0., 0., .1}, 2);
        }
        break;
      }

      case SY_touchBoxNormalX: {
        //        rai::Frame* box = world.getFrame(s.frames(1));
        //        CHECK(box, "");
        //        CHECK(box->shape && box->shape->type()==rai::ST_ssBox, "");
        double boxSize = shapeSize(komo.world, s.frames(1), 0);
        komo.addObjective({s.phase0}, FS_positionDiff, {s.frames(0), s.frames(1)}, OT_eq, {{1, 3}, {1e2, .0, .0}}, {.5*boxSize, 0., 0.}); //arr({1,3},{0,0,1e2})
        komo.addObjective({s.phase0}, FS_scalarProductXZ, {s.frames(1), s.frames(0)}, OT_eq, {1e2}, {1.});
        //        komo.addObjective({s.phase0}, FS_scalarProductYZ, {s.frames(1), s.frames(0)}, OT_eq, {1e2});
        break;
      }
      case SY_touchBoxNormalY: {
        //rai::Frame* box = world.getFrame(s.frames(1));
        //        CHECK(box, "");
        //        CHECK(box->shape && box->shape->type()==rai::ST_ssBox, "");
        double boxSize = shapeSize(komo.world, s.frames(1), 1);
        komo.addObjective({s.phase0}, FS_positionDiff, {s.frames(0), s.frames(1)}, OT_eq, {{1, 3}, {1e2, .0, .0}}, {.5*boxSize, 0., 0.}); //arr({1,3},{0,0,1e2})
        komo.addObjective({s.phase0}, FS_scalarProductYZ, {s.frames(1), s.frames(0)}, OT_eq, {1e2}, {1.});
        //        komo.addObjective({s.phase0}, FS_scalarProductYZ, {s.frames(1), s.frames(0)}, OT_eq, {1e2});
        break;
      }
      case SY_touchBoxNormalZ: {
        rai::Frame* box = komo.world.getFrame(s.frames(1));
        CHECK(box, "");
        CHECK(box->shape, "");
        double boxSize = 0.;
        if(box->shape->type()==rai::ST_ssBox) {
          boxSize = shapeSize(komo.world, s.frames(1), 2);
        } else if(box->shape->type()==rai::ST_cylinder) {
          boxSize = shapeSize(komo.world, s.frames(1), 1);
        } else HALT("");
        komo.addObjective({s.phase0}, FS_positionDiff, {s.frames(0), s.frames(1)}, OT_eq, {{1, 3}, {0., 0., 1e2}}, {0, 0, .5*boxSize}); //arr({1,3},{0,0,1e2})
        komo.addObjective({s.phase0}, FS_scalarProductZZ, {s.frames(1), s.frames(0)}, OT_eq, {1e2}, {1.});
        //        komo.addObjective({s.phase0}, FS_vectorZDiff, {s.frames(0), s.frames(1)}, OT_eq, {1e2});
        break;
      }

      case SY_makeFree:   komo.world.makeObjectsFree(s.frames);  break;
      case SY_stableRelPose: komo.addObjective({s.phase0, s.phase1+1.}, FS_poseRel, s.frames, OT_eq, {1e2}, {}, 1);  break;
      case SY_stablePose:  komo.addObjective({s.phase0, s.phase1+1.}, FS_pose, s.frames, OT_eq, {1e2}, {}, 1);  break;
      case SY_poseEq: komo.addObjective({s.phase0, s.phase1}, FS_poseDiff, s.frames, OT_eq, {1e2});  break;

      case SY_downUp: {
        if(komo.k_order>=2) {
          komo.addObjective({s.phase0, s.phase1}, FS_position, {s.frames(0)}, OT_eq, {}, {0., 0., .1}, 2, +1, +1);
        }
        break;
      }

      case SY_break:      NIY; //komo.addObjective({s.phase0, s.phase1}, make_shared<F_NoJumpFromParent_OBSOLETE>(), {s.frames(0)}, OT_eq, {1e2}, NoArr, 1, 0, 0);  break;

      case SY_contact:    komo.addContact_slide(s.phase0, s.phase1, s.frames(0), s.frames(1));  break;
      case SY_contactStick:    komo.addContact_stick(s.phase0, s.phase1, s.frames(0), s.frames(1));  break;
      case SY_contactComplementary: komo.addContact_ComplementarySlide(s.phase0, s.phase1, s.frames(0), s.frames(1));  break;
      case SY_bounce:     komo.addContact_elasticBounce(s.phase0, s.frames(0), s.frames(1), .9);  break;
      //case SY_contactComplementary:     addContact_Complementary(s.phase0, s.phase1, s.frames(0), s.frames(1));  break;

      case SY_dampMotion: {
        double sqrAccCost=1e-2, sqrVelCost=1e-2;
        if(sqrVelCost>0. && komo.k_order>=1) {
          komo.addObjective({s.phase0, s.phase1}, make_shared<F_LinAngVel>(), {s.frames(0)}, OT_sos, {sqrVelCost}, NoArr, 1);
        }
        if(sqrAccCost>0. && komo.k_order>=2) {
          komo.addObjective({s.phase0, s.phase1}, make_shared<F_LinAngVel>(), {s.frames(0)}, OT_sos, {sqrAccCost}, NoArr, 2);
        }
      } break;
      case SY_alignByInt: {
        komo.addObjective({s.phase0, s.phase1}, FS_scalarProductXX, s.frames, OT_sos);  break;
        cout <<"THE INTEGER IS: " <<s.frames(2) <<endl;
      } break;

      case SY_forceBalance: {
        komo.addObjective({s.phase0, s.phase1}, make_shared<F_TotalForce>(), {s.frames(0)}, OT_eq, {1e2});
      } break;

      //switches are handled above now
      case SY_stable:      //if(!ignoreSwitches) addSwitch_stable(s.phase0, s.phase1+1., s.frames(0), s.frames(1));  break;
      case SY_stableOn:    //if(!ignoreSwitches) addSwitch_stableOn(s.phase0, s.phase1+1., s.frames(0), s.frames(1));  break;
      case SY_dynamic:     //if(!ignoreSwitches) addSwitch_dynamic(s.phase0, s.phase1+1., "base", s.frames(0));  break;
      case SY_dynamicOn:   //if(!ignoreSwitches) addSwitch_dynamicOn(s.phase0, s.phase1+1., s.frames(0), s.frames(1));  break;
      case SY_dynamicTrans:   //if(!ignoreSwitches) addSwitch_dynamicTrans(s.phase0, s.phase1+1., "base", s.frames(0));  break;
      case SY_quasiStatic:
      case SY_quasiStaticOn:
        break;
      case SY_magicTrans: //addSwitch_magicTrans(s.phase0, s.phase1, world.frames.first()->name, s.frames(0), 0.);  break;
      case SY_magic:      komo.addSwitch_magic(s.phase0, s.phase1, komo.world.frames.first()->name, s.frames(0), 0., 0.);  break;
      default: HALT("undefined symbol: " <<s.symbol);
    }
  }
}

void Skeleton::read_old(std::istream& is) {
  rai::Graph G(is);
  for(rai::Node* n:G) {
    cout <<"ENTRY: " << *n <<endl;
    rai::Graph& entry = n->graph();
    arr& when = entry.elem(0)->get<arr>();
    CHECK(when.N<=2, "Skeleton error entry " <<n->index <<" time interval: interval needs no, 1, or 2 elements");
    if(when.N==0) when= {0., -1.};
    if(when.N==1) when= {when.scalar(), when.scalar()};
    rai::Enum<SkeletonSymbol> symbol;
    try {
      symbol = entry.elem(1)->key;
    } catch(std::runtime_error& err) {
      LOG(-1) <<"Skeleton error line " <<n->index <<" symbol: " <<err.what() <<endl;
    }
    StringA frames;
    try {
      if(entry.elem(2)->isOfType<arr>()) {
        CHECK(!entry.elem(2)->get<arr>().N, "");
      } else {
        frames = entry.elem(2)->get<StringA>();
      }
    } catch(std::runtime_error& err) {
      LOG(-1) <<"Skeleton error line " <<n->index <<" frames: " <<err.what() <<endl;
    }
    S.append(SkeletonEntry(when(0), when(1), symbol, frames));
  }
}

void Skeleton::read(std::istream& is) {
  //-- first get a PRE-skeleton
  rai::Graph G(is);
  double phase0=1.;
  for(rai::Node* step:G) {
    rai::Graph& stepG = step->graph();
    for(rai::Node* lit:stepG) {
      StringA frames;
      try {
        frames = lit->get<StringA>();
      } catch(std::runtime_error& err) {
        LOG(-1) <<"Skeleton error step" <<phase0 <<" literal: " <<*lit <<" err: " <<err.what() <<endl;
      }

      rai::Enum<SkeletonSymbol> symbol;
      rai::String& symb = frames.first();
      double phase1 = phase0;
      if(symb(-1)=='_') {
        phase1=-1.;
        symb.resize(symb.N-1, true);
      }
      try {
        symbol = symb;
      } catch(std::runtime_error& err) {
        LOG(-1) <<"Skeleton error line " <<phase0 <<" literal: " <<*lit <<" err: " <<err.what() <<endl;
      }

      S.append(SkeletonEntry(phase0, phase1, symbol, frames({1, -1})));
    }
    phase0 += 1.;
  }

  cout <<"PRE_skeleton: " <<endl;
  write(cout);

  //-- fill in the missing phase1!
  for(uint i=0; i<S.N; i++) {
    SkeletonEntry& si = S(i);
    if(si.phase1==-1 && si.frames.N) {
      for(uint j=i+1; j<S.N; j++) {
        SkeletonEntry& sj = S(j);
        if(sj.phase1==-1. && sj.frames.N && sj.frames.last()==si.frames.last()) { //this is also a mode symbol (due to phase1==-1.)
          si.phase1 = sj.phase0;
          break;
        }
      }
    }
  }

  cout <<"TIMED_skeleton: " <<endl;
  write(cout);
}

} //namespace

