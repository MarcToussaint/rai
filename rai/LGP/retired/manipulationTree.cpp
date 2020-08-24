/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

void MNode::recomputeAllMCStats(bool excludeLeafs) {
  if(!mcStats) return;
  if(!isTerminal) {
    if(children.N || !excludeLeafs || isInfeasible)
      mcStats->clear();
  }
  for(MNode* ch:children) {
    ch->recomputeAllMCStats(excludeLeafs);
    for(double x:ch->mcStats->X) mcStats->add(x);
  }
  if(mcStats->n)
    mcCost = - mcStats->X.first();
  else
    mcCost = 100.;
}

arr MNode::generateRootMCRollouts(uint num, int stepAbort, const rai::Array<MCTS_Environment::Handle>& prefixDecisions) {
  CHECK(!parent, "generating rollouts needs to be done by the root only");

  fol.reset_state();
  //  cout <<"********\n *** MC from STATE=" <<*fol.state->isNodeOfGraph <<endl;
  if(!rootMC) {
    rootMC = new PlainMC(fol);
    rootMC->verbose = 0;
  }

  arr R;

  for(uint k=0; k<num; k++) {
    rootMC->initRollout(prefixDecisions);
    fol.setState(folState);
    double r = rootMC->finishRollout(stepAbort);
    R.append(r);
  }

  return R;
}

void MNode::addMCRollouts(uint num, int stepAbort) {
  //-- collect decision path
  MNodeL treepath = getTreePath();
  rai::Array<MCTS_Environment::Handle> prefixDecisions(treepath.N-1);
  for(uint i=1; i<treepath.N; i++)
    prefixDecisions(i-1) = treepath(i)->decision;

  //  cout <<"DECISION PATH = "; listWrite(prefixDecisions); cout <<endl;

#if 0
  arr R = generateRootMCRollouts(num, stepAbort, prefixDecisions);
#else
  arr R;
  for(uint k=0; k<num; k++) {
    rootMC->initRollout(prefixDecisions);
    fol.setState(folState);
    double r = rootMC->finishRollout(stepAbort);
    R.append(r);
  }
#endif

  for(MNode* n:treepath) {
    if(!n->mcStats) n->mcStats = new MCStatistics;
    for(auto& r:R) {
      n->mcStats->add(r);
      n->mcCost = - n->mcStats->X.first();
    }
  }

  mcCount += num;
  //  mcStats->report();
  //  auto a = rootMC->getBestAction();
  //  cout <<"******** BEST ACTION " <<*a <<endl;
}

void MNode::solvePoseProblem() {
  //reset the effective kinematics:
  CHECK(!parent || parent->hasEffKinematics, "parent needs to have computed the pose first!");
  if(parent) effKinematics = parent->effKinematics;

#if 0
  if(true || !poseProblem) { //create the pose problem
    Node* n = fol.KB.newSubgraph({"PoseProblem"}, {folState->isNodeOfGraph});
    poseProblemSpecs = &n->graph();
    poseProblemSpecs->copy(*folState, &fol.KB);
    NodeL komoRules = fol.KB.getNodes("PoseProblemRule");  //  listWrite(komoRules, cout, "\n"); cout <<endl;
    forwardChaining_FOL(*poseProblemSpecs, komoRules/*, nullptr, NoGraph, 5*/);
    cout <<"POSE PROBLEM:" <<*poseProblemSpecs <<endl;

    poseProblem = new KOMO(effKinematics, true);
    poseProblem->setTiming(0, 1.);
    poseProblem->k_order=0;
    poseProblem->parseTasks(*poseProblemSpecs);
    //    Problem->reportFeatures();
  }

  for(rai::KinematicSwitch* sw: poseProblem->switches)
    if(sw->timeOfApplication==0) sw->apply(effKinematics);

  arr newPose=poseProblem->getInitialization();
  rndGauss(newPose, .1, true);
  OptConstrained opt(newPose, NoArr, poseProblem->InvKinProblem(), OPT(verbose=2));
  opt.run();
  //  poseProblem->reportFeatures();
  poseProblem->costReport(false);
  //    Problem->world.watch(true);

  if(!pose.N || opt.newton.fx<poseCost) {
    poseCost = opt.newton.fx;
    pose = newPose;
  }
#else
  poseProblem = new KOMO();
  KOMO& komo(*poseProblem);
  komo.setModel(effKinematics);
  komo.setTiming(1, 1, 5., 1);

  komo.setSquaredQVelocities();
  cout <<"  ** PoseProblem for state" <<*folState <<endl;
  komo.setAbstractTask(0, *folState);

  komo.run_prepare();
  komo.MP->reportFeatures(true, FILE("z.problem"));
  komo.run();

  Graph result = komo.getReport();
  double cost = result.get<double>({"total", "sqrCosts"});
  if(!pose.N || cost<poseCost) {
    poseCost = cost;
    pose = komo.x;
  }
//  komo.displayTrajectory(-1.);
#endif

  effKinematics.setJointState(pose);

  for(rai::KinematicSwitch* sw: poseProblem->MP->switches)
    if(sw->timeOfApplication==1) sw->apply(effKinematics);
  effKinematics.topSort();
  effKinematics.checkConsistency();
  effKinematics.getJointState();
  hasEffKinematics = true;
}

void MNode::solveSeqProblem(int verbose) {
  if(!s) return;

  //-- create new problem declaration (within the KB)
  Node* seqProblemNode = fol.KB.newSubgraph({"SeqProblem"}, {folState->isNodeOfGraph});
  seqProblemSpecs = &seqProblemNode->graph();

  //-- collect 'path nodes'
  MNodeL treepath = getTreePath();

#if 0
  //-- add decisions to the seq pose problem description
  seqProblem = new KOMO(startKinematics, true);
  seqProblem->setTiming(s-1, 5.*s); //T=0 means one pose is optimized!!
  seqProblem->k_order=1;
  NodeL komoRules = fol.KB.getNodes("SeqProblemRule");
//  listWrite(komoRules, cout, "\n"); cout <<endl;
  for(MNode* node:treepath) if(node->folDecision) { //(e.g. the root may not have a decision)
      CHECK(node->s > 0, ""); //don't add anything for the root
      Graph tmp(*node->folState);
      Graph& changes = fol.KB.newSubgraph({}, {})->graph();
      forwardChaining_FOL(tmp, komoRules, nullptr, changes); //use the rules to add to the specs
      changes.checkConsistency();
      for(Node* n:changes) {
        Graph* p;
        arr* t;
        double* tt;
        if((p=&n->graph())) {
          if((t=p->find<arr>("time"))) *t += (double)(node->s)-2.;
          if((tt=p->find<double>("time"))) *tt += (double)(node->s)-2.;
        }
      }
      seqProblemSpecs->copy(changes, true);
      delete changes.isNodeOfGraph;
//    cout <<"SEQ PROBLEM: (s=" <<node->s <<")\n" <<*seqProblemSpecs <<endl;
    }

  cout <<"SEQ PROBLEM symbolic:\n" <<*seqProblemSpecs <<endl;
  seqProblem->parseTasks(*seqProblemSpecs, 1, 0);
  arr newSeq = seqProblem->getInitialization();
  cout <<"SEQ PROBLEM motion problem:\n";
  seqProblem->reportFeatures(true);
  rndGauss(newSeq, .1, true);

  Convert cvt(*seqProblem);

//  checkJacobianCP(cvt, seq, 1e-4);
//  checkHessianCP(cvt, seq, 1e-4);
//  exit(0);

  double newCost;
  if(!seqProblem->dim_g_h()) {
    OptNewton opt(newSeq, cvt, OPT(verbose=2));
    opt.run();
    newCost = opt.fx;
  } else {
    OptConstrained opt(newSeq, NoArr, cvt, OPT(verbose=0));
    opt.run();
    newCost = opt.newton.fx;
  }

  if(!seq.N || newCost < seqCost) {
    seqCost = newCost;
    seq=newSeq;
  }

//  seqProblem->reportFeatures(true);
  seqProblem->costReport(verbose>1);
  if(verbose>1) seqProblem->displayTrajectory(1, "SeqProblem", -.01);
#else
  seqProblem = new KOMO();
  KOMO& komo(*seqProblem);
  komo.setModel(startKinematics);
  komo.setTiming(time, 2, 5., 1, false);

  komo.setHoming(-1., -1., 1e-1); //gradient bug??
  komo.setSquaredQVelocities();
  komo.setFixEffectiveJoints(-1., -1., 1e3);
  komo.setFixSwitchedObjects(-1., -1., 1e3);

  for(MNode* node:treepath) {
    komo.setAbstractTask(node->time, *node->folState);
  }

  komo.reset();
//  komo.MP->reportFeatures(true, FILE("z.problem"));
  komo.run();
  komo.MP->reportFeatures(true, FILE("z.problem"));
//  komo.checkGradients();

  Graph result = komo.getReport();
  double cost = result.get<double>({"total", "sqrCosts"});
  if(!pose.N || cost<poseCost) {
    poseCost = cost;
    pose = komo.x;
  }
#endif
}

void MNode::solvePathProblem(uint microSteps, int verbose) {
  Node* pathProblemNode = fol.KB.newSubgraph({"PathProblem"}, {folState->isNodeOfGraph});
  pathProblemSpecs = &pathProblemNode->graph();

  //-- collect 'path nodes'
  MNodeL treepath = getTreePath();

#if 0
  //-- add decisions to the path problem description
  pathProblem = new KOMO(startKinematics, true);
  pathProblem->setTiming(s*microSteps, 5.*s);
  pathProblem->k_order=2;
  NodeL komoRules = fol.KB.getNodes("PathProblemRule");
//  listWrite(komoRules, cout, "\n"); cout <<endl;
  for(MNode* node:treepath) if(node->folDecision) {
      CHECK(node->s > 0, "");
      node->folDecision->newClone(*pathProblemSpecs);
      forwardChaining_FOL(*pathProblemSpecs, komoRules); //, nullptr, NoGraph, 4);
      pathProblem->parseTasks(*pathProblemSpecs, microSteps, (node->s-1)*microSteps);
      cout <<"PATH PROBLEM: (s=" <<node->s <<")\n" <<*pathProblemSpecs <<endl;
      pathProblemSpecs->clear();
    }

  path = pathProblem->getInitialization();
  pathProblem->reportFeatures(true);
  rndGauss(path, .1, true);

  Convert cvt(*pathProblem);

  if(!pathProblem->dim_g_h()) {
//    optNewton(path, cvt, OPT(verbose=2, stopIters=100, maxStep=.1, stepInc=1.1, stepDec=0.7 , damping=1., allowOverstep=true));
    OptNewton opt(path, cvt, OPT(verbose=2));
    opt.run();
    pathCost = opt.fx;
  } else {
    OptConstrained opt(path, NoArr, cvt, OPT(verbose=0));
    opt.run();
    pathCost = opt.newton.fx;
  }

//  pathProblem->reportFeatures(true);
  pathProblem->costReport(verbose>1);
  if(verbose>1) pathProblem->displayTrajectory(1, "PathProblem", -.01);
#else
  pathProblem = new KOMO();
  KOMO& komo(*pathProblem);
  komo.setModel(startKinematics);
  komo.setTiming(time, microSteps, 5., 2);

  komo.setSquaredQAccelerations();
  komo.setFixEffectiveJoints(-1., -1., 1e3);

  for(MNode* node:treepath) { /*if(node->folDecision)*/
    komo.setAbstractTask(node->time, *node->folState);
  }

  komo.reset();
  komo.MP->reportFeatures(true, FILE("z.problem"));
  komo.run();

  Graph result = komo.getReport();
  double cost = result.get<double>({"total", "sqrCosts"});
  if(!pose.N || cost<poseCost) {
    poseCost = cost;
    pose = komo.x;
  }
//  komo.displayTrajectory(-1.);
#endif
}

#if 1
Skeleton S = getSkeleton({"touch", "above", "inside", "impulse",
                          "stable", "stableOn", "dynamic", "dynamicOn",
                          "push", "graspSlide", "liftDownUp"
                         });
if(level==1 && parent) CHECK(parent->effKinematics.q.N, "I can't compute a pose when no pose was comp. for parent (I need the effKin)");
skeleton2Bound(komo, BoundType(level), S, startKinematics, (parent?parent->effKinematics:startKinematics));
#else
//-- prepare the komo problem
switch(level) {
case 1: {
  //pose: propagate eff kinematics
  if(!parent) effKinematics = startKinematics;
  else {
    if(!parent->effKinematics.q.N) {
      LOG(-1) <<"I can't compute a pose when no pose was comp. for parent (I need the effKin)";
      return;
    }
    effKinematics = parent->effKinematics;
  }

  komo.setModel(effKinematics, false);
  komo.setTiming(1., 2, 5., 1);

  if(LGP_useHoming) komo.setHoming(-1., -1., 1e-2);
  komo.setSquaredQVelocities(.5, -1., 1.); //IMPORTANT: do not penalize transitions of from prefix to x_{0} -> x_{0} is 'loose'
  //komo.setFixEffectiveJoints(-1., -1., 1e2); //IMPORTANT: assume ALL eff to be articulated; problem: no constraints (touch)
  komo.setFixSwitchedObjects(-1., -1., 1e2);
  komo.setSquaredQuaternionNorms();

#if 1
  Skeleton S = getSkeleton({"touch", "above", "inside", "impulse",
                            "stable", "stableOn", "dynamic", "dynamicOn",
                            "push", "graspSlide"
                           }, true);
  komo.setSkeleton(S);
#else
  komo.setAbstractTask(0., *folState);
#endif

  komo.reset();
  komo.setPairedTimes();
//      cout <<komo.getPath_times() <<endl;
} break;
//  case 1:{
//    //pose: propagate eff kinematics
//    if(!parent) effKinematics = startKinematics;
//    else effKinematics = parent->effKinematics;

//    if(!parent || !parent->parent){
//      komo.setModel(startKinematics, false);
//    }else{
//      komo.setModel(parent->parent->effKinematics, false);
//    }
//    komo.setTiming(2.+.5, 2, 5., 1);

//    if(LGP_useHoming) komo.setHoming(-1., -1., 1e-2);
//    komo.setSquaredQVelocities(1.1, -1., 1.); //IMPORTANT: do not penalize transitions of from prefix to x_{0} -> x_{0} is 'loose'
//    komo.setFixEffectiveJoints(.5, -1., 1e2); //IMPORTANT: assume ALL eff to be articulated; problem: no constraints (touch)
//    komo.setFixSwitchedObjects(.5, -1., 1e2);
//    komo.setSquaredQuaternionNorms();

//    if(!parent){
//      komo.setAbstractTask(0., *folState);
//    }else{
//      komo.setAbstractTask(0., *parent->folState);
//      komo.setAbstractTask(1., *folState);
//    }

//    komo.reset();
//    komo.setPairedTimes();
//  } break;
case 2: {
  komo.setModel(startKinematics, false);
  if(time>1e-2) komo.setTiming(time, 2, 5., 1);
  else  komo.setTiming(1., 2, 5., 1);

  if(LGP_useHoming) komo.setHoming(-1., -1., 1e-2);
  komo.setSquaredQVelocities();
  komo.setFixEffectiveJoints(-1., -1., 1e2);
  komo.setFixSwitchedObjects(-1., -1., 1e2);
  komo.setSquaredQuaternionNorms();

#if 1
  Skeleton S = getSkeleton({"touch", "above", "inside", "impulse",
                            "stable", "stableOn", "dynamic", "dynamicOn",
                            "push", "graspSlide"
                           });
  komo.setSkeleton(S);
#else
  for(MNode* node:getTreePath()) {
    komo.setAbstractTask((node->parent?node->parent->time:0.), *node->folState);
  }
#endif

  komo.reset();
  komo.setPairedTimes();
//      cout <<komo.getPath_times() <<endl;
} break;
case 3: {
  komo.setModel(startKinematics, collisions);
  uint stepsPerPhase = rai::getParameter<uint>("LGP/stepsPerPhase", 10);
  uint pathOrder = rai::getParameter<uint>("LGP/pathOrder", 2);
  komo.setTiming(time+.5, stepsPerPhase, 5., pathOrder);

  if(LGP_useHoming) komo.setHoming(-1., -1., 1e-2);
  if(pathOrder==1) komo.setSquaredQVelocities();
  else komo.setSquaredQAccelerations();
  komo.setFixEffectiveJoints(-1., -1., 1e2);
  komo.setFixSwitchedObjects(-1., -1., 1e2);
  komo.setSquaredQuaternionNorms();

#if 1
  Skeleton S = getSkeleton({"touch", "above", "inside", "impulse",
                            "stable", "stableOn", "dynamic", "dynamicOn",
                            "push", "graspSlide", "liftDownUp"
                           });
  komo.setSkeleton(S);
#else
  if(collisions) komo.setCollisions(false);
  for(MNode* node:getTreePath()) {
    komo.setAbstractTask((node->parent?node->parent->time:0.), *node->folState);
  }
#endif

  komo.reset();
//      cout <<komo.getPath_times() <<endl;
} break;
}
#endif

//void MNode::createEffKinematics(){
//  KOMO komo;

//  CHECK(!effKinematics.q.N, "has been created before");

//  if(!parent) effKinematics = startKinematics;
//  else{
//      if(!parent->effKinematics.q.N){
//          LOG(-1) <<"I can't compute a pose when no pose was comp. for parent (I need the effKin)";
//          return;
//      }
//      effKinematics = parent->effKinematics;
//  }

//  komo.setModel(effKinematics);
//  komo.setAbstractTask(0., *folState);

//  effKinematics = *komo.configurations.last();

//  for(uint t=0;t<komo.T;t++){
//      for(rai::KinematicSwitch *sw: komo.switches){
//          if(sw->timeOfApplication==t) sw->apply(effKinematics);
//      }
//  }
//  effKinematics.topSort();
//  DEBUG( effKinematics.checkConsistency(); )
//          effKinematics.getJointState();
//}

#ifdef OLD
void MNode::solvePoseProblem() {
  uint level=1;

  //reset the effective kinematics:
  if(parent && !parent->effKinematics.q.N) {
    RAI_MSG("parent needs to have computed the pose first!");
    return;
  }
  if(!parent) effKinematics = startKinematics;
  else effKinematics = parent->effKinematics;

  //-- collect 'path nodes'
  MNodeL treepath = getTreePath();

  poseProblem = new KOMO();
  KOMO& komo(*poseProblem);
  komo.setModel(effKinematics);
  komo.setTiming(1., 2, 5., 1, false);

  if(LGP_useHoming) komo.setHoming(-1., -1., 1e-1); //gradient bug??
  komo.setSquaredQVelocities();
  //  komo.setFixEffectiveJoints(-1., -1., 1e3);
  komo.setFixSwitchedObjects(-1., -1., 1e3);

  komo.setAbstractTask(0., *folState);
  //  for(rai::KinematicSwitch *sw: poseProblem->switches){
  //    sw->timeOfApplication=2;
  //  }

  DEBUG(FILE("z.fol") <<fol;)
  DEBUG(komo.getReport(false, 1, FILE("z.problem"));)
  komo.reset();
  try {
    komo.run();
  } catch(const char* msg) {
    cout <<"KOMO FAILED: " <<msg <<endl;
  }
  COUNT_evals += komo.opt->newton.evals;
  COUNT_kin += rai::Configuration::setJointStateCount;
  COUNT_opt(level)++;
  count(level)++;

  DEBUG(komo.getReport(false, 1, FILE("z.problem"));)

  Graph result = komo.getReport();
  DEBUG(FILE("z.problem.cost") <<result;)
  double cost_here = result.get<double>({"total", "sqrCosts"});
  double constraints_here = result.get<double>({"total", "constraints"});
  bool feas = (constraints_here<.5);

  cost_here -= 0.1*ret.reward; //account for the symbolic costs
  if(parent) cost_here += parent->cost(level); //this is sequentially additive cost

  //update the bound
  if(feas) {
    if(count(level)==1/*&& count({2,-1})==0 (also no higher levels)*/ || cost_here<bound) bound=cost_here;
  }

  if(count(level)==1 || cost_here<cost(level)) {
    cost(level) = cost_here;
    constraints(level) = constraints_here;
    feasible(level) = feas;
    pose = komo.x;
  }

  if(!feasible(level))
    labelInfeasible();

  effKinematics = *komo.configurations.last();

  for(rai::KinematicSwitch* sw: komo.switches) {
    //    CHECK_EQ(sw->timeOfApplication, 1, "need to do this before the optimization..");
    if(sw->timeOfApplication>=2) sw->apply(effKinematics);
  }
  effKinematics.topSort();
  DEBUG(effKinematics.checkConsistency();)
  effKinematics.getJointState();
}

void MNode::solveSeqProblem(int verbose) {
  uint level=2;

  if(!step) { feasible(level)=true; return; } //there is no sequence to compute

  //-- collect 'path nodes'
  MNodeL treepath = getTreePath();

  seqProblem = new KOMO();
  KOMO& komo(*seqProblem);
  komo.setModel(startKinematics);
  komo.setTiming(time, 2, 5., 1, false);

  if(LGP_useHoming) komo.setHoming(-1., -1., 1e-1); //gradient bug??
  komo.setSquaredQVelocities();
  komo.setFixEffectiveJoints(-1., -1., 1e3);
  komo.setFixSwitchedObjects(-1., -1., 1e3);

  for(MNode* node:treepath) {
    komo.setAbstractTask((node->parent?node->parent->time:0.), *node->folState);
  }

  DEBUG(FILE("z.fol") <<fol;)
  DEBUG(komo.getReport(false, 1, FILE("z.problem"));)
  komo.reset();
  try {
    komo.run();
  } catch(const char* msg) {
    cout <<"KOMO FAILED: " <<msg <<endl;
  }
  COUNT_evals += komo.opt->newton.evals;
  COUNT_kin += rai::Configuration::setJointStateCount;
  COUNT_opt(level)++;
  count(level)++;

  DEBUG(komo.getReport(false, 1, FILE("z.problem"));)
  //  komo.checkGradients();

  Graph result = komo.getReport();
  DEBUG(FILE("z.problem.cost") <<result;)
  double cost_here = result.get<double>({"total", "sqrCosts"});
  double constraints_here = result.get<double>({"total", "constraints"});
  bool feas = (constraints_here<.5);

  cost_here += cost(l_symbolic); //account for the symbolic costs

  //update the bound
  if(feas) {
    if(!count(level)/*actually !count({1,-1}) (also no higher levels)*/ || cost_here<bound) bound=cost_here;
  }

  if(!seq.N || cost_here<cost(level)) {
    cost(level) = cost_here;
    constraints(level) = constraints_here;
    feasible(level) = (constraints_here<.5);
    seq = komo.x;
  }

  if(!feasible(level))
    labelInfeasible();
}

void MNode::solvePathProblem(uint microSteps, int verbose) {
  uint level=3;

  if(!step) { feasible(level)=true; return; } //there is no path to compute

  //-- collect 'path nodes'
  MNodeL treepath = getTreePath();

  pathProblem = new KOMO();
  KOMO& komo(*pathProblem);
  komo.setModel(startKinematics);
  komo.setTiming(time, microSteps, 5., 2, false);

  if(LGP_useHoming) komo.setHoming(-1., -1., 1e-2); //gradient bug??
  komo.setSquaredQAccelerations();
  komo.setFixEffectiveJoints(-1., -1., 1e3);
  komo.setFixSwitchedObjects(-1., -1., 1e3);

  for(MNode* node:treepath) {
    komo.setAbstractTask((node->parent?node->parent->time:0.), *node->folState);
  }

  DEBUG(FILE("z.fol") <<fol;)
  DEBUG(komo.getReport(false, 1, FILE("z.problem"));)
  komo.reset();
  try {
    komo.run();
  } catch(const char* msg) {
    cout <<"KOMO FAILED: " <<msg <<endl;
  }
  COUNT_evals += komo.opt->newton.evals;
  COUNT_kin += rai::Configuration::setJointStateCount;
  COUNT_opt(level)++;
  count(level)++;

  DEBUG(komo.getReport(false, 1, FILE("z.problem"));)
  //  komo.checkGradients();

  Graph result = komo.getReport();
  DEBUG(FILE("z.problem.cost") <<result;)
  double cost_here = result.get<double>({"total", "sqrCosts"});
  double constraints_here = result.get<double>({"total", "constraints"});
  bool feas = (constraints_here<.5);

  cost_here += cost(l_symbolic); //account for the symbolic costs

  //update the bound
  if(feas) {
    if(!count(level)/*actually !count({1,-1}) (also no higher levels)*/ || cost_here<bound) bound=cost_here;
  }

  if(!path.N || cost_here<cost(level)) {
    cost(level) = cost_here;
    constraints(level) = constraints_here;
    feasible(level) = (constraints_here<.5);
    path = komo.x;
  }

  if(!feasible(level))
    labelInfeasible();
}
#endif
