/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "skeleton.h"

#include "komo.h"
#include "manipTools.h"

#include "../Kin/F_pose.h"
#include "../Kin/F_forces.h"
#include "../Kin/F_qFeatures.h"
#include "../KOMO/switch.h"
#include "../Optim/NLP_Solver.h"
#include "../Logic/fol.h"

void addBoxPickObjectives(KOMO& komo, double time, rai::ArgWord dir,
                          const char* boxName, const arr& boxSize,
                          const char* gripperName, const char* palmName, const char* tableName, bool pre=false);

void addBoxPlaceObjectives(KOMO& komo, double time,
                           rai::ArgWord dir, const char* boxName, const arr& boxSize, const char* tableName,
                           const char* gripperName, const char* palmName, double margin=-.02, bool pre=false);

namespace rai {

Array<SkeletonSymbol> skeletonModes = { SY_stable, SY_stableOn, SY_stableYPhi, SY_stableZero, SY_dynamic, SY_dynamicOn, SY_dynamicTrans, SY_quasiStatic, SY_quasiStaticOn,
                                        //SY_magicTrans,
                                        SY_stableOnX, SY_stableOnY
                                      };

void SkeletonEntry::write(std::ostream& os) const {
  os <<"[" <<phase0 <<", " <<phase1 <<"] " <<symbol <<' ';
  frames.write(os, " ", nullptr, "()");
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

void Skeleton::setFromStateSequence(const Array<Graph*>& states, const arr& times) {
  //setup a done marker array: which literal in each state is DONE
  uint maxLen=0;
  for(Graph* s:states) if(s->N>maxLen) maxLen = s->N;
  boolA done(states.N, maxLen);
  done = false;

  for(uint k=0; k<states.N; k++) {
    const Graph& G = *states(k);
//    cout <<G <<endl;
    for(uint i=0; i<G.N; i++) {
      if(!done(k, i)) {
        Node* n = G(i);
        if(n->is<Graph>() && n->graph().findNode("%decision")) continue; //don't pickup decision literals
        StringA symbols;
        for(Node* p:n->parents) symbols.append(p->key);

        //check if there is a predicate
        if(!symbols.N) continue;

        //if logic symbol ends with _, extend one time step further
        rai::String& symstr = symbols.first();
        bool extend=false;
        if(symstr(-1)=='_') {
          extend=true;
          symstr.resize(symstr.N-1, true);
        }

        //check if predicate is a SkeletonSymbol
        if(!Enum<SkeletonSymbol>::contains(symstr)) continue;

        //trace into the future
        uint k_end=k+1;
        for(; k_end<states.N; k_end++) {
          Node* persists = getEqualFactInList(n, *states(k_end), true);
          if(!persists) break;
          done(k_end, persists->index) = true;
        }
        k_end--;
        if(extend) k_end++;

        Enum<SkeletonSymbol> sym(symstr);
        if(k_end>=states.N-1) {
          S.append(SkeletonEntry({times(k), times.last(), sym, symbols({1, -1})}));
        } else {
          S.append(SkeletonEntry({times(k), times(k_end), sym, symbols({1, -1})}));
        }
      }
    }
  }

  for(uint i=0; i<S.N; i++) {
    SkeletonEntry& se =  S.elem(i);
    if(skeletonModes.contains(se.symbol)) { //S(i) is about a switch
      if(se.phase1<times.last()) {
        se.phase1 += 1.; //*** MODES EXTEND TO THE /NEXT/ TIME SLICE ***
      } else {
        se.phase1 = -1.;
      }
    }
  }

}

void Skeleton::fillInEndPhaseOfModes() {
  //double maxPhase = getMaxPhase();
  for(uint i=0; i<S.N; i++) {
    SkeletonEntry& si = S(i);
    if(si.phase1==-1 && si.frames.N) {
//      si.phase1=maxPhase;
      for(uint j=i+1; j<S.N; j++) {
        SkeletonEntry& sj = S(j);
        if(sj.phase0>si.phase0 &&      //needs to be in the future
            sj.phase1==-1. &&  //is also a '_' mode symbol
            sj.frames.N &&
            sj.frames.last()==si.frames.last() //has the same last frame
          ) {
          si.phase1 = sj.phase0;
          break;
        }
      }
    }
  }
}

void Skeleton::addExplicitCollisions(const StringA& collisions) {
  explicitCollisions = collisions;
  for(uint i=0; i<collisions.N; i+=2) {
    if(komoWaypoints) komoWaypoints->addObjective({}, FS_distance, {collisions.elem(i), collisions.elem(i+1)}, OT_ineq, {1e1});
    if(komoPath) komoPath->addObjective({}, FS_distance, {collisions.elem(i), collisions.elem(i+1)}, OT_ineq, {1e1});
  }
}

void Skeleton::addLiftPriors(const StringA& lift) {
  explicitLiftPriors = lift;
  if(komoPath) {
    for(uint i=0; i<lift.N; i++) if(komoPath->world[lift(i)]) {
        for(uint t=0; t<getMaxPhase(); t++) {
          komoPath->addObjective({.3+t, .7+t}, FS_position, {lift(i)}, OT_ineqP, {{1, 3}, {0, 0, -1e1}}, {0., 0., .6});
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
      CHECK_GE(S.elem(i).frames.N, 2, "switch symbols need at least 2 frames (e.g. {world, obj})")
      rai::Frame* toBeSwitched = C[S.elem(i).frames(1)];
      CHECK(toBeSwitched, "");
      rai::Frame* rootOfSwitch = toBeSwitched->getUpwardLink(NoTransformation, true);
      rai::Frame* childOfSwitch = toBeSwitched->getDownwardLink(true);
      for(; j>=0; j--) {
        if(skeletonModes.contains(S.elem(j).symbol)) { //S(j) is about a switch
          const rai::String& prevSwitched = S.elem(j).frames(1);
          if(S.elem(i).frames(-1)==S.elem(j).frames(-1)
              || prevSwitched==toBeSwitched->name
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

arr Skeleton::solve(const rai::Configuration& C, ArgWord sequenceOrPath, int verbose) {
  komoPath = getKomo_path(C, sequenceOrPath);
  komoPath->optimize();
  //  komo->checkGradients();

  if(verbose>0) komoPath->report(false, true, true);
  if(verbose>1) komoPath->view(true, "optimized motion");
  if(verbose>2) {
    while(komoPath->view_play(true));
    komoPath->view_play(false, 0, .1, "z.vid/");
  }

  return komoPath->getPath_X();
}

shared_ptr<SolverReturn> Skeleton::solve2(const rai::Configuration& C, int verbose) {
  auto T = this->nlp_waypoints(C);

  NLP_Solver sol;
  sol.setProblem(T.nlp);
  sol.setInitialization(T.komo->x); //to avoid adding noise again
  sol.setOptions(OptOptions().set_verbose(verbose));
  auto ret = sol.solve();
  if(verbose>0) T.nlp->report(cout, verbose);
  if(verbose>1) sol.gnuplot_costs();
  return ret;
}

shared_ptr<SolverReturn> Skeleton::solve3(const rai::Configuration& C, bool useKeyframes, int verbose) {
  auto waypointsT = this->nlp_waypoints(C);

  shared_ptr<SolverReturn> ret;
  {
    NLP_Solver sol;
    sol.setProblem(waypointsT.nlp);
    sol.setInitialization(waypointsT.komo->x); //to avoid adding noise again
    ret = sol.solve();
    waypointsT.nlp->report(cout, verbose);
//    sol.gnuplot_costs();
  }

#if 0
  arr X = keyframes.komo->getPath_X();
  arrA waypoints(X.d0);
  for(uint i=0; i<waypoints.N; i++) waypoints(i) = X[i];
#else
  arrA waypoints = waypointsT.komo->getPath_qAll();
#endif

  auto pathT = this->nlp_path(C, (useKeyframes?waypoints:arrA()));

  {
    NLP_Solver sol;
    sol.setProblem(pathT.nlp);
    sol.setInitialization(pathT.komo->x); //to avoid adding noise again
//    path.komo->opt.animateOptimization = 1;
    ret = sol.solve();
    pathT.nlp->report(cout, verbose);
//    sol.gnuplot_costs();
  }

  return ret;
}

void Skeleton::getKeyframeConfiguration(Configuration& C, int step, int verbose) {
  DEPR;
  //note: the alternative would be to copy the frames komo.timeSlices[step] into a new config
  CHECK(komoPath, "");
  CHECK_EQ(komoPath->k_order, 1, "");
  C.copy(komoPath->world);
  for(shared_ptr<KinematicSwitch>& sw:komoPath->switches) {
    int s = sw->timeOfApplication;
    if(s<=step) {
      if(verbose) { LOG(0) <<"applying switch:"; sw->write(cout, C.frames); cout <<endl; }
      sw->apply(C.frames);
    }
  }
}

void Skeleton::getTwoWaypointProblem(int t2, Configuration& C, arr& q1, arr& q2, KOMO& komoWays) {
//  FILE("z.keys") <<komoWays.pathConfig;
  komoWays.getConfiguration_full(C, t2-1, 0);
//  FILE("z.sub") <<C;
  C.ensure_indexedJoints();
  if(!t2) C.selectJoints(DofL{}, true);
  DofL acts = C.activeDofs;
  for(Dof* d:acts) {
#if 0
    rai::Frame* orgF = komoWays.world.frames(d->frame->ID);
    if(!orgF->joint || !d->joint() || !orgF->joint->active || orgF->joint->type!=d->joint()->type) {
      d->setActive(false);
    }
#else
    if(!d->joint() || d->isStable) {
      d->setActive(false);
    }
#endif
  }
  //  if(opt.verbose>1) LOG(0) <<C.getJointNames();
  q1 = C.getJointState();
  //  FILE("z.g") <<C <<endl;  C.view(true, "JETZT!");
  C.setFrameState(komoWays.getConfiguration_X(t2), C.frames({0, komoWays.world.frames.N-1}));
  q2 = C.getJointState();
  //  C.view(true);
}

shared_ptr<KOMO> Skeleton::getKomo_path(const rai::Configuration& C, uint stepsPerPhase, double accScale, double lenScale, double homingScale, double collScale) {
  shared_ptr<KOMO> komo=make_shared<KOMO>();
  komo->opt.verbose = verbose-2;
  komo->setConfig(C, collisions);

  double maxPhase = getMaxPhase();
  komo->setTiming(maxPhase, stepsPerPhase, 5., 2);
  if(accScale>0.) komo->addControlObjective({}, 2, accScale);
  if(homingScale>0.) komo->addControlObjective({}, 0, homingScale);
  komo->addQuaternionNorms();
  if(collisions) komo->add_collision(true);

  addObjectives(*komo);

  for(uint i=0; i<explicitCollisions.N; i+=2) {
    komo->addObjective({}, FS_distance, {explicitCollisions.elem(i), explicitCollisions.elem(i+1)}, OT_ineq, {collScale});
  }
  for(uint i=0; i<explicitLiftPriors.N; i++) if(komo->world[explicitLiftPriors(i)]) {
      for(uint t=0; t<maxPhase; t++) {
        komo->addObjective({.3+t, .7+t}, FS_position, {explicitLiftPriors(i)}, OT_ineqP, {{1, 3}, {0, 0, -1e1}}, {0., 0., .6});
      }
    }

  komo->run_prepare(0.);
  komoPath = komo;
  return komo;
}

shared_ptr<KOMO> Skeleton::getKomo_waypoints(const Configuration& C, double lenScale, double homingScale, double collScale) {
  shared_ptr<KOMO> komo=make_shared<KOMO>();
  komo->opt.verbose = verbose-2;
  komo->setConfig(C, collisions);

  double maxPhase = getMaxPhase();
  komo->setTiming(maxPhase, 1, 5., 1);
  if(lenScale>0.) komo->addControlObjective({}, 1, lenScale);
  if(homingScale>0.) komo->addControlObjective({}, 0, homingScale);
  komo->addQuaternionNorms();
  if(collisions) komo->add_collision(true);

  addObjectives(*komo);

  for(uint i=0; i<explicitCollisions.N; i+=2) {
    komo->addObjective({}, FS_distance, {explicitCollisions.elem(i), explicitCollisions.elem(i+1)}, OT_ineq, {collScale});
  }

  komo->run_prepare(0.);
  komoWaypoints = komo;
  return komo;
}

shared_ptr<KOMO> Skeleton::getKomo_finalSlice(const rai::Configuration& C, double lenScale, double homingScale, double collScale) {
  shared_ptr<KOMO> komo=make_shared<KOMO>();
  komo->opt.verbose = verbose-2;

  //-- prepare the komo problem
  double maxPhase = getMaxPhase();
  double optHorizon=2.;
  if(maxPhase<1.) optHorizon=maxPhase=1.;

  //-- create another skeleton, with non-switches removed
  rai::Skeleton finalS;
  for(const rai::SkeletonEntry& s:S) {
    if(rai::skeletonModes.contains(s.symbol)
        || s.phase0>=maxPhase) {
      finalS.S.append(s);
      rai::SkeletonEntry& fs = finalS.S(-1);
      fs.phase0 -= maxPhase-optHorizon;
      if(fs.phase0<0.) fs.phase0=0.;
      if(fs.phase1!=-1) {
        fs.phase1 -= maxPhase-optHorizon;
        if(fs.phase1<0.) fs.phase1=0.;
      }
    }
  }

  if(komo->opt.verbose>1) {
    cout <<"POSE skeleton:" <<endl;
    finalS.write(cout, finalS.getSwitches(C));
  }

  komo->setConfig(C, collisions);
  komo->setTiming(optHorizon, 1, 10., 1);
  if(lenScale>0.) komo->addControlObjective({}, 1, lenScale);
  if(homingScale>0.) komo->addControlObjective({}, 0, homingScale);
  komo->addQuaternionNorms();
  if(collisions) komo->add_collision(false);

  finalS.addObjectives(*komo);

  //-- deactivate all velocity objectives except for transition
  for(shared_ptr<Objective>& o:komo->objectives) {
    if(o->feat->order>0
        && !std::dynamic_pointer_cast<F_qItself>(o->feat)
        && !std::dynamic_pointer_cast<F_Pose>(o->feat)
        && !std::dynamic_pointer_cast<F_PoseRel>(o->feat)) {
      o->times= {1e6};
    }
  }
  for(shared_ptr<GroundedObjective>& o:komo->objs) {
    if(o->feat->order>0
        && !std::dynamic_pointer_cast<F_qItself>(o->feat)
        && !std::dynamic_pointer_cast<F_Pose>(o->feat)
        && !std::dynamic_pointer_cast<F_PoseRel>(o->feat)) {
      o->feat.reset();
    }
  }
  for(uint i=komo->objs.N; i--;) if(!komo->objs(i)->feat) {
      komo->objs.remove(i);
    }

  for(uint i=0; i<explicitCollisions.N; i+=2) {
    komo->addObjective({}, FS_distance, {explicitCollisions.elem(i), explicitCollisions.elem(i+1)}, OT_ineq, {collScale});
  }

  komo->run_prepare(.01);
  komoFinal = komo;
  return komo;
}

SkeletonTranscription Skeleton::nlp_waypoints(const rai::Configuration& C) {
  SkeletonTranscription T;
  T.komo = getKomo_waypoints(C, 1e-2, 1e-2);
  T.nlp = T.komo->nlp();
  return T;
}

SkeletonTranscription Skeleton::nlp_path(const rai::Configuration& C, const arrA& initWaypoints) {
  SkeletonTranscription T;
  uint stepsPerPhase = rai::getParameter<uint>("LGP/stepsPerPhase", 10);
  T.komo = getKomo_path(C, stepsPerPhase, 3e-1, -1e-2, 1e-2);

  if(initWaypoints.N) {
    T.komo->initWithWaypoints(initWaypoints, 1, true);
//    komo->opt.animateOptimization = 2;
  }

  T.nlp = T.komo->nlp();
  return T;
}

SkeletonTranscription Skeleton::nlp_finalSlice(const rai::Configuration& C) {
  SkeletonTranscription T;
  T.komo = getKomo_finalSlice(C, 1e-2, 1e-2);
  T.nlp = T.komo->nlp();
  return T;
}

void Skeleton::addObjectives(KOMO& komo) const {
  //-- add objectives for mode switches
  intA switches = getSwitches(komo.world);
  for(uint i=0; i<switches.d0; i++) {
    int j = switches(i, 0);
    int k = switches(i, 1);
    komo.addModeSwitch({S(k).phase0, S(k).phase1}, S(k).symbol, S(k).frames, j<0);
    if(S(k).phase1!=-1.
        && S(k).phase0>=S(k).phase1
        && !(S(k).phase0==0. && S(k).phase1==0.)) { //this case happens in final slice skeletons, where several switches happen at step=0 to create effective dofs
//      cout <<*this <<endl;
      LOG(-1) <<"are you sure this is only a single timeslice mode??:" <<S(k);
    }
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
      case SY_lift:       if(komo.k_order>=2) komo.addObjective({s.phase0+.3, s.phase0+.7}, FS_position, {s.frames(0)}, OT_ineqP, {{1, 3}, {0, 0, -1e1}}, {0., 0., .4});  break;

      case SY_relPosY:    komo.addObjective({s.phase0, s.phase1}, FS_positionRel, {s.frames(0), s.frames(1)}, OT_eq, {{1, 3}, {0, 1e2, 0}});  break;
      case SY_restingOn: {
        rai::Frame* table = komo.world.getFrame(s.frames(0));
        rai::Frame* obj = komo.world.getFrame(s.frames(1));
        if(obj->shape && obj->shape->type()==ST_capsule) {
          double height = .5*table->getSize()(2) + obj->getSize()(1);
          komo.addObjective({s.phase0, s.phase1}, FS_positionRel, {s.frames(1), s.frames(0)}, OT_eq, {{1, 3}, {0, 0, 1e1}}, {0, 0, height});
          komo.addObjective({s.phase0, s.phase1}, FS_scalarProductZZ, s.frames, OT_eq, {1e1});
        } else {
          NIY;
        }

      } break;
      case SY_topBoxGrasp: {
        komo.addObjective({s.phase0}, FS_positionDiff, s.frames, OT_eq, {1e2});
        komo.addObjective({s.phase0}, FS_scalarProductXY, s.frames, OT_eq, {1e2}, {0.});
        komo.addObjective({s.phase0}, FS_vectorZDiff, s.frames, OT_eq, {1e2}, {0., 0., 1.});
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
        double boxSize = shapeSize(komo.world.getFrame(s.frames(1)), 0);
        komo.addObjective({s.phase0}, FS_positionDiff, {s.frames(0), s.frames(1)}, OT_eq, {{1, 3}, {1e2, .0, .0}}, {.5*boxSize, 0., 0.}); //arr({1,3},{0,0,1e2})
        komo.addObjective({s.phase0}, FS_scalarProductXZ, {s.frames(1), s.frames(0)}, OT_eq, {1e2}, {1.});
        //        komo.addObjective({s.phase0}, FS_scalarProductYZ, {s.frames(1), s.frames(0)}, OT_eq, {1e2});
        komo.addObjective({s.phase0, s.phase1}, FS_insideBox, {s.frames(0), s.frames(1)}, OT_ineq, {0, 1e1, 1e1, 0, 1e1, 1e1});
        break;
      }
      case SY_touchBoxNormalY: {
        //rai::Frame* box = world.getFrame(s.frames(1));
        //        CHECK(box, "");
        //        CHECK(box->shape && box->shape->type()==rai::ST_ssBox, "");
        double boxSize = shapeSize(komo.world.getFrame(s.frames(1)), 1);
        komo.addObjective({s.phase0}, FS_positionDiff, {s.frames(0), s.frames(1)}, OT_eq, {{1, 3}, {0., 1e2, .0}}, {0, .5*boxSize, 0.});
        komo.addObjective({s.phase0}, FS_scalarProductYZ, {s.frames(1), s.frames(0)}, OT_eq, {1e2}, {1.});
        //        komo.addObjective({s.phase0}, FS_scalarProductYZ, {s.frames(1), s.frames(0)}, OT_eq, {1e2});
        komo.addObjective({s.phase0, s.phase1}, FS_insideBox, {s.frames(0), s.frames(1)}, OT_ineq, {1e1, 0, 1e1, 1e1, 0, 1e1});
        break;
      }
      case SY_touchBoxNormalZ: {
        rai::Frame* box = komo.world.getFrame(s.frames(1));
        CHECK(box, "");
        CHECK(box->shape, "");
        double boxSize = 0.;
        if(box->shape->type()==rai::ST_ssBox) {
          boxSize = shapeSize(komo.world.getFrame(s.frames(1)), 2);
        } else if(box->shape->type()==rai::ST_cylinder) {
          boxSize = shapeSize(komo.world.getFrame(s.frames(1)), 1);
        } else HALT("");
        komo.addObjective({s.phase0}, FS_positionDiff, {s.frames(0), s.frames(1)}, OT_eq, {{1, 3}, {0., 0., 1e2}}, {0, 0, .5*boxSize}); //arr({1,3},{0,0,1e2})
        komo.addObjective({s.phase0}, FS_scalarProductZZ, {s.frames(1), s.frames(0)}, OT_eq, {1e2}, {1.});
        //        komo.addObjective({s.phase0}, FS_vectorZDiff, {s.frames(0), s.frames(1)}, OT_eq, {1e2});
        komo.addObjective({s.phase0, s.phase1}, FS_insideBox, {s.frames(0), s.frames(1)}, OT_ineq, {1e1, 1e1, 0, 1e1, 1e1, 0});
        break;
      }

      case SY_boxGraspX: {
        rai::Frame* box = komo.world.getFrame(s.frames(1));
        arr size = box->getSize();
        size.resizeCopy(3);
        addBoxPickObjectives(komo, {s.phase0},
                             rai::_xAxis,
                             s.frames(1),
                             size,
                             s.frames(0),
                             "r_palm", 0);
      } break;

      case SY_makeFree:   komo.world.makeObjectsFree(s.frames);  break;
      case SY_stableRelPose: komo.addObjective({s.phase0, s.phase1+1.}, FS_poseRel, s.frames, OT_eq, {1e2}, {}, 1);  break;
      case SY_stablePose:  komo.addObjective({s.phase0, s.phase1+1.}, FS_pose, s.frames, OT_eq, {1e2}, {}, 1);  break;
      case SY_poseEq: komo.addObjective({s.phase0, s.phase1}, FS_poseDiff, s.frames, OT_eq, {1e2});  break;
      case SY_positionEq: komo.addObjective({s.phase0, s.phase1}, FS_positionDiff, s.frames, OT_eq, {1e2});  break;

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
      case SY_push: {
        komo.addContact_slide(s.phase0, s.phase1, s.frames(0), s.frames(1));
        //prior on contact point!
        if(s.phase1>=s.phase0+.8) {
          rai::Frame* obj = komo.world.getFrame(s.frames(1));
          if(obj->children.N && !(obj->shape && (obj->shape->type()==ST_sphere || obj->shape->type()==ST_capsule))) {
            obj = obj->children.last();
          }
          if(obj->shape && (obj->shape->type()==ST_sphere || obj->shape->type()==ST_capsule)) {
            double rad = obj->shape->radius();
            arr times = {s.phase0+.2, s.phase1-.2};
            if(komo.k_order==1) times = {s.phase0, s.phase1};
            komo.addObjective(times, make_shared<F_PushRadiusPrior>(rad), s.frames, OT_sos, {1e0}, NoArr, 1, +1, 0);
          }
        }
        //motion/control costs on the object
        if(komo.k_order>1) {
          komo.addObjective({s.phase0, s.phase1}, FS_position, {s.frames(1)}, OT_sos, {3e-1}, {}, 2); //smooth obj motion
          komo.addObjective({s.phase1}, FS_pose, {s.frames(0)}, OT_eq, {1e0}, {}, 1); //zero vel at start
          komo.addObjective({s.phase1}, FS_pose, {s.frames(1)}, OT_eq, {1e0}, {}, 1); //zero vel at end
        }
      } break;
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
      case SY_stableYPhi:    //if(!ignoreSwitches) addSwitch_stableOn(s.phase0, s.phase1+1., s.frames(0), s.frames(1));  break;
      case SY_stableOnX:    //if(!ignoreSwitches) addSwitch_stableOn(s.phase0, s.phase1+1., s.frames(0), s.frames(1));  break;
      case SY_stableOnY:    //if(!ignoreSwitches) addSwitch_stableOn(s.phase0, s.phase1+1., s.frames(0), s.frames(1));  break;
      case SY_stableZero:
      case SY_dynamic:     //if(!ignoreSwitches) addSwitch_dynamic(s.phase0, s.phase1+1., "base", s.frames(0));  break;
      case SY_dynamicOn:   //if(!ignoreSwitches) addSwitch_dynamicOn(s.phase0, s.phase1+1., s.frames(0), s.frames(1));  break;
      case SY_dynamicTrans:   //if(!ignoreSwitches) addSwitch_dynamicTrans(s.phase0, s.phase1+1., "base", s.frames(0));  break;
      case SY_quasiStatic:
      case SY_quasiStaticOn:
        break;
      case SY_magicTrans: {
        komo.addSwitch({s.phase0}, true, false, JT_transXY, SWInit_copy, s.frames(0), s.frames(1));
        break;
      }
      case SY_magic: {
        komo.addSwitch({s.phase0}, true, false, JT_free, SWInit_copy, komo.world.frames.first()->name, s.frames(0));
//            komo.addSwitch_magic(s.phase0, s.phase1, komo.world.frames.first()->name, s.frames(0), 0., 0.);  break;
      } break;
      case SY_follow: {
        komo.addObjective({s.phase0, s.phase1}, FS_positionDiff, s.frames, OT_sos, {3.});
        komo.addObjective({s.phase0, s.phase1}, FS_qItself, {s.frames(1)}, OT_f, {-3.});
      } break;
      default: HALT("undefined symbol: " <<s.symbol);
    }
  }
}

void Skeleton::read_old(std::istream& is) {
  rai::Graph G(is);
  for(rai::Node* n:G) {
    cout <<"ENTRY: " << *n <<endl;
    rai::Graph& entry = n->graph();
    arr& when = entry.elem(0)->as<arr>();
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
      if(entry.elem(2)->is<arr>()) {
        CHECK(!entry.elem(2)->as<arr>().N, "");
      } else {
        frames = entry.elem(2)->as<StringA>();
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
  //double maxPhase=0.;
  for(rai::Node* step:G) {
    rai::Graph& stepG = step->graph();
    for(rai::Node* lit:stepG) {
      StringA frames;
      try {
        frames = lit->as<StringA>();
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
      //maxPhase=phase0;
    }
    phase0 += 1.;
  }

//  cout <<"PRE_skeleton: " <<endl;
//  write(cout);

  //-- fill in the missing phase1!
  fillInEndPhaseOfModes();
//  cout <<"TIMED_skeleton: " <<endl;
//  write(cout);
}

} //namespace


void addBoxPickObjectives(KOMO& komo, double time, rai::ArgWord dir, const char* boxName, const arr& boxSize, const char* gripperName, const char* palmName, const char* tableName, bool pre) {
  arr xLine, yzPlane;
  FeatureSymbol xyScalarProduct=FS_none, xzScalarProduct=FS_none;
  if(dir==rai::_xAxis) {
    xLine = arr{{1, 3}, {1, 0, 0}};
    yzPlane = arr{{2, 3}, {0, 1, 0, 0, 0, 1}};
    xyScalarProduct = FS_scalarProductXY;
    xzScalarProduct = FS_scalarProductXZ;
  } else if(dir==rai::_yAxis) {
    xLine = arr{{1, 3}, {0, 1, 0}};
    yzPlane = arr{{2, 3}, {1, 0, 0, 0, 0, 1}};
    xyScalarProduct = FS_scalarProductXX;
    xzScalarProduct = FS_scalarProductXZ;
  } else if(dir==rai::_zAxis) {
    xLine = arr{{1, 3}, {0, 0, 1}};
    yzPlane = arr{{2, 3}, {1, 0, 0, 0, 1, 0}};
    xyScalarProduct = FS_scalarProductXX;
    xzScalarProduct = FS_scalarProductXY;
  }

  double margin=.02;

  //position: center in inner target plane; X-specific
  if(!pre) {
    komo.addObjective({time}, FS_positionRel, {gripperName, boxName}, OT_eq, xLine*1e1, {});
    komo.addObjective({time}, FS_positionRel, {gripperName, boxName}, OT_ineq, yzPlane*1e0, (boxSize/2.-margin));
    komo.addObjective({time}, FS_positionRel, {gripperName, boxName}, OT_ineq, yzPlane*(-1e0), -(boxSize/2.-margin));
  } else {
    komo.addObjective({time, time+1.}, FS_positionRel, {gripperName, boxName}, OT_eq, xLine*1e1, {});
  }

  //orientation: grasp axis orthoginal to target plane; X-specific
  komo.addObjective({time-.2, time}, xyScalarProduct, {gripperName, boxName}, OT_eq, {1e0}, {});
  komo.addObjective({time-.2, time}, xzScalarProduct, {gripperName, boxName}, OT_eq, {1e0}, {});

  //no collision with palm
  if(!pre) {
    komo.addObjective({time-.3, time}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.001});
  } else {
    komo.addObjective({time-.3, time}, FS_distance, {palmName, boxName}, OT_eq, {1e1}, {-.07});
  }

  //approach: only longitudial velocity, min distance before and at grasp
  if(komo.k_order>1) komo.addObjective({time-.3, time}, FS_positionRel, {boxName, gripperName}, OT_eq, arr{{2, 3}, {1, 0, 0, 0, 1, 0}}*1e2, {}, 1);
  if(komo.k_order>1) komo.addObjective({time-.5, time-.3}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.1});

  //zero vel
  if(komo.k_order>1) komo.addObjective({time}, FS_qItself, {}, OT_eq, {}, {}, 1);
}

void addBoxPlaceObjectives(KOMO& komo, double time,
                           rai::ArgWord dir, const char* boxName, const arr& boxSize,
                           const char* tableName,
                           const char* gripperName, const char* palmName,
                           double margin, bool pre) {
  double relPos=0.;
  FeatureSymbol zVector = FS_none;
  arr zVectorTarget = {0., 0., 1.};
  if(dir==rai::_xAxis) {
    relPos = .5*boxSize(0)+.03;
    zVector = FS_vectorX;
  } else if(dir==rai::_yAxis) {
    relPos = .5*boxSize(1)+.03;
    zVector = FS_vectorY;
  } else if(dir==rai::_zAxis) {
    relPos = .5*boxSize(2)+.03;
    zVector = FS_vectorZ;
  } else if(dir==rai::_xNegAxis) {
    relPos = .5*boxSize(0)+.03;
    zVector = FS_vectorX;
    zVectorTarget *= -1.;
  } else if(dir==rai::_yNegAxis) {
    relPos = .5*boxSize(1)+.03;
    zVector = FS_vectorY;
    zVectorTarget *= -1.;
  } else if(dir==rai::_zNegAxis) {
    relPos = .5*boxSize(2)+.03;
    zVector = FS_vectorZ;
    zVectorTarget *= -1.;
  }

  //z-position: fixed
  if(!pre) {
    komo.addObjective({time}, FS_positionDiff, {boxName, tableName}, OT_eq, 1e1*arr({1, 3}, {0, 0, 1}), {.0, .0, relPos});
  } else {
    komo.addObjective({time}, FS_positionDiff, {boxName, tableName}, OT_eq, 1e1*arr({1, 3}, {0, 0, 1}), {.0, .0, relPos+.04});
  }

  //xy-position: above table
  if(!pre) {
    komo.addObjective({time}, FS_positionDiff, {boxName, tableName}, OT_eq, 1e1*arr({2, 3}, {1, 0, 0, 0, 1, 0}));
    //komo.addObjective({time}, FS_aboveBox, {boxName, tableName}, OT_ineq, {3e0}, {margin});
  } else {
    komo.addObjective({time}, FS_positionDiff, {boxName, tableName}, OT_eq, 1e1*arr({2, 3}, {1, 0, 0, 0, 1, 0}));
    //komo.addObjective({time, time+1.}, FS_aboveBox, {boxName, tableName}, OT_ineq, {3e0}, {margin});
  }

  //orientation: Y-up
  komo.addObjective({time-.2, time}, zVector, {boxName}, OT_eq, {0.5}, zVectorTarget);

  //retract: only longitudial velocity, min distance after grasp
  if(komo.k_order>1) komo.addObjective({time, time+.3}, FS_positionRel, {boxName, gripperName}, OT_eq, arr{{2, 3}, {1, 0, 0, 0, 1, 0}}*1e2, {}, 1);
  if(komo.k_order>1) komo.addObjective({time+.3, time+.5}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.1});

  //zero vel
  if(komo.k_order>1) komo.addObjective({time}, FS_qItself, {}, OT_eq, {}, {}, 1);
}
