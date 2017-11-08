void ManipulationTree_Node::solvePoseProblem(){
  //reset the effective kinematics:
  CHECK(!parent || parent->hasEffKinematics,"parent needs to have computed the pose first!");
  if(parent) effKinematics = parent->effKinematics;

#if 0
  if(true || !poseProblem){ //create the pose problem
    Node *n = fol.KB.newSubgraph({"PoseProblem"}, {folState->isNodeOfGraph});
    poseProblemSpecs = &n->graph();
    poseProblemSpecs->copy(*folState, &fol.KB);
    NodeL komoRules = fol.KB.getNodes("PoseProblemRule");  //  listWrite(komoRules, cout, "\n"); cout <<endl;
    forwardChaining_FOL(*poseProblemSpecs, komoRules/*, NULL, NoGraph, 5*/);
    cout <<"POSE PROBLEM:" <<*poseProblemSpecs <<endl;

    poseProblem = new KOMO(effKinematics, true);
    poseProblem->setTiming(0, 1.);
    poseProblem->k_order=0;
    poseProblem->parseTasks(*poseProblemSpecs);
    //    Problem->reportFeatures();
  }

  for(mlr::KinematicSwitch *sw: poseProblem->switches)
    if(sw->timeOfApplication==0) sw->apply(effKinematics);

  arr newPose=poseProblem->getInitialization();
  rndGauss(newPose, .1, true);
  OptConstrained opt(newPose, NoArr, poseProblem->InvKinProblem(), OPT(verbose=2));
  opt.run();
  //  poseProblem->reportFeatures();
  poseProblem->costReport(false);
  //    Problem->world.gl().watch();

  if(!pose.N || opt.newton.fx<poseCost){
    poseCost = opt.newton.fx;
    pose = newPose;
  }
#else
  poseProblem = new KOMO();
  KOMO& komo(*poseProblem);
  komo.setModel(effKinematics);
  komo.setTiming(1,1,5.,1);

  komo.setSquaredQVelocities();
  cout <<"  ** PoseProblem for state" <<*folState <<endl;
  komo.setAbstractTask(0, *folState);

  komo.reset();
  komo.MP->reportFeatures(true, FILE("z.problem"));
  komo.run();

  Graph result = komo.getReport();
  double cost = result.get<double>({"total","sqrCosts"});
  if(!pose.N || cost<poseCost){
    poseCost = cost;
    pose = komo.x;
  }
//  komo.displayTrajectory(-1.);
#endif


  effKinematics.setJointState(pose);

  for(mlr::KinematicSwitch *sw: poseProblem->MP->switches)
    if(sw->timeOfApplication==1) sw->apply(effKinematics);
  effKinematics.topSort();
  effKinematics.checkConsistency();
  effKinematics.getJointState();
  hasEffKinematics = true;
}

void ManipulationTree_Node::solveSeqProblem(int verbose){
  if(!s) return;

  //-- create new problem declaration (within the KB)
  Node *seqProblemNode = fol.KB.newSubgraph({"SeqProblem"}, {folState->isNodeOfGraph});
  seqProblemSpecs = &seqProblemNode->graph();

  //-- collect 'path nodes'
  ManipulationTree_NodeL treepath = getTreePath();

#if 0
  //-- add decisions to the seq pose problem description
  seqProblem = new KOMO(startKinematics, true);
  seqProblem->setTiming(s-1, 5.*s); //T=0 means one pose is optimized!!
  seqProblem->k_order=1;
  NodeL komoRules = fol.KB.getNodes("SeqProblemRule");
//  listWrite(komoRules, cout, "\n"); cout <<endl;
  for(ManipulationTree_Node *node:treepath) if(node->folDecision){ //(e.g. the root may not have a decision)
    CHECK(node->s > 0,""); //don't add anything for the root
    Graph tmp(*node->folState);
    Graph& changes = fol.KB.newSubgraph({}, {})->graph();
    forwardChaining_FOL(tmp, komoRules, NULL, changes); //use the rules to add to the specs
    changes.checkConsistency();
    for(Node *n:changes){
      Graph *p;
      arr *t;
      double *tt;
      if((p=&n->graph())){
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
  if(!seqProblem->dim_g_h()){
    OptNewton opt(newSeq, cvt, OPT(verbose=2));
    opt.run();
    newCost = opt.fx;
  }else{
    OptConstrained opt(newSeq, NoArr, cvt, OPT(verbose=0));
    opt.run();
    newCost = opt.newton.fx;
  }

  if(!seq.N || newCost < seqCost){
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

  for(ManipulationTree_Node *node:treepath){
    komo.setAbstractTask(node->time, *node->folState);
  }

  komo.reset();
//  komo.MP->reportFeatures(true, FILE("z.problem"));
  komo.run();
  komo.MP->reportFeatures(true, FILE("z.problem"));
//  komo.checkGradients();

  Graph result = komo.getReport();
  double cost = result.get<double>({"total","sqrCosts"});
  if(!pose.N || cost<poseCost){
    poseCost = cost;
    pose = komo.x;
  }
#endif
}

void ManipulationTree_Node::solvePathProblem(uint microSteps, int verbose){
  Node *pathProblemNode = fol.KB.newSubgraph({"PathProblem"}, {folState->isNodeOfGraph});
  pathProblemSpecs = &pathProblemNode->graph();

  //-- collect 'path nodes'
  ManipulationTree_NodeL treepath = getTreePath();

#if 0
  //-- add decisions to the path problem description
  pathProblem = new KOMO(startKinematics, true);
  pathProblem->setTiming(s*microSteps, 5.*s);
  pathProblem->k_order=2;
  NodeL komoRules = fol.KB.getNodes("PathProblemRule");
//  listWrite(komoRules, cout, "\n"); cout <<endl;
  for(ManipulationTree_Node *node:treepath) if(node->folDecision){
    CHECK(node->s > 0,"");
    node->folDecision->newClone(*pathProblemSpecs);
    forwardChaining_FOL(*pathProblemSpecs, komoRules); //, NULL, NoGraph, 4);
    pathProblem->parseTasks(*pathProblemSpecs, microSteps, (node->s-1)*microSteps);
    cout <<"PATH PROBLEM: (s=" <<node->s <<")\n" <<*pathProblemSpecs <<endl;
    pathProblemSpecs->clear();
  }

  path = pathProblem->getInitialization();
  pathProblem->reportFeatures(true);
  rndGauss(path, .1, true);

  Convert cvt(*pathProblem);

  if(!pathProblem->dim_g_h()){
//    optNewton(path, cvt, OPT(verbose=2, stopIters=100, maxStep=.1, stepInc=1.1, stepDec=0.7 , damping=1., allowOverstep=true));
    OptNewton opt(path, cvt, OPT(verbose=2));
    opt.run();
    pathCost = opt.fx;
  }else{
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

  for(ManipulationTree_Node *node:treepath) /*if(node->folDecision)*/{
    komo.setAbstractTask(node->time, *node->folState);
  }

  komo.reset();
  komo.MP->reportFeatures(true, FILE("z.problem"));
  komo.run();

  Graph result = komo.getReport();
  double cost = result.get<double>({"total","sqrCosts"});
  if(!pose.N || cost<poseCost){
    poseCost = cost;
    pose = komo.x;
  }
//  komo.displayTrajectory(-1.);
#endif
}
