ptr<KOMO> LGP_Node::optSubCG(const SubCG& scg, bool collisions, int verbose) {
  ptr<KOMO> komo = make_shared<KOMO>();

  komo->verbose = rai::MAX(verbose, 0);

  if(komo->verbose>0) {
    cout <<"########## OPTIM SubCG: " <<scg <<endl;
  }

//  komo->fil = new ofstream(OptLGPDataPath + STRING("komo-" <<id <<'-' <<step <<'-' <<bound));

  CG2komo(*komo, scg, startKinematics, collisions);
  return komo;

  if(komo->logFile) {
    komo->reportProblem(*komo->logFile);
    (*komo->logFile) <<komo->getProblemGraph(false);
  }

//  if(level==BD_seq) komo->denseOptimization=true;

  //-- optimize
  DEBUG(FILE("z.fol") <<fol;);
  DEBUG(komo->getReport(false, 1, FILE("z.problem")););
  if(komo->verbose>1) komo->reportProblem();
//  if(komo->verbose>5) komo->animateOptimization = komo->verbose-5;

  try {
    komo->run();
  } catch(std::runtime_error& err) {
    cout <<"KOMO CRASHED: " <<err.what() <<endl;
    komo->reset();
    return komo;
  }
  COUNT_evals += komo->opt->newton.evals;
  COUNT_kin += rai::Configuration::setJointStateCount;
  COUNT_time += komo->runTime;

  DEBUG(komo->getReport(false, 1, FILE("z.problem")););
//  cout <<komo->getReport(true) <<endl;
//  komo->reportProxies(cout, 0.);
//  komo->checkGradients();

  Graph result = komo->getReport(komo->verbose>0);
  DEBUG(FILE("z.problem.cost") <<result;);

  //double cost_here = result.get<double>({"total","sqrCosts"});
  //double constraints_here = result.get<double>({"total","constraints"});
  //bool feas = (constraints_here<1.);

  return komo;
}

ptr<CG> LGP_Node::getCGO(bool collisions, int verbose) {
  Skeleton S = getSkeleton();

  if(verbose>1) {
    writeSkeleton(cout, S, getSwitchesFromSkeleton(S));
  }

  return skeleton2CGO(S,
                      startKinematics,
                      collisions);
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
    c->addConfigurationCopy(framesPerT(t));
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
