void KOMO::setSpline(uint splineT) {
  Spline S;
  S.setUniformNonperiodicBasis(T-1, splineT, 2);
  uint n=dim_x(0);
  splineB = zeros(S.basis.d0*n, S.basis.d1*n);
  for(uint i=0; i<S.basis.d0; i++) for(uint j=0; j<S.basis.d1; j++)
      splineB.setMatrixBlock(S.basis(i, j)*eye(n, n), i*n, j*n);
  z = pseudoInverse(splineB) * x;
}

void KOMO::setPairedTimes() {
  CHECK_EQ(k_order, 1, "NIY");
  for(uint s=1; s<T; s+=2) {
    configurations(s)  ->setTimes(1.98*tau); //(tau*(.98+int(s+1)-int(k_order)));
    configurations(s+1)->setTimes(0.02*tau); //(tau*(int(s)-int(k_order)));
  }
}

void KOMO::setStartConfigurations(const arr& q) {
  if(!configurations.N) setupConfigurations();
  for(uint s=0; s<k_order; s++) {
    configurations(s)->setJointState(q);
  }
}

void KOMO_ext::getPhysicsReference(uint subSteps, int display) {
  x.resize(T, world.getJointStateDimension());
  PhysXInterface& px = world.physx();
  px.pushFullState(world.frames);
  for(uint t=0; t<T; t++) {
    for(uint s=0; s<subSteps; s++) {
      px.step(tau/subSteps);
      if(display) px.watch((display<0), STRING("t="<<t<<";"<<s));
    }
    x[t] = world.q;
//      K.calc_fwdPropagateFrames();
//    K.watch();
  }
//  K.watch(true);
  world.setJointState(x[0]);
  if(configurations.N) {
    for(uint s=0; s<k_order; s++) {
      configurations(s)->setJointState(x[0]);
    }
  }
}

void KOMO_ext::playInPhysics(uint subSteps, bool display) {
  arr vels;
  PhysXInterface& px = world.physx();
  for(uint t=0; t<T; t++) {
    NIY; //get the velocity from consequtive frames?
    px.pushFullState(configurations(k_order+t)->frames, NoArr, true);
    for(uint s=0; s<subSteps; s++) {
      if(display) px.watch(false, STRING("t="<<t<<";"<<s));
      world.physx().step(tau/subSteps);
    }
    px.pullDynamicStates(configurations(k_order+t)->frames, vels);
  }
  //  for(uint i=0;i<vels.d0;i++) if(i<world.frames.N) cout <<world.frames(i)->name <<" v=" <<vels[i] <<endl;
}

void KOMO::selectJointsBySubtrees(const StringA& roots, const arr& times, bool notThose) {
  if(!configurations.N) setupConfigurations();

  world.selectJointsBySubtrees(roots, notThose);

  if(!times.N) {
    for(Configuration* C:configurations) {
      C->selectJointsBySubtrees(roots, notThose);
      C->ensure_q();
      C->checkConsistency();
    }
  } else {
    int tfrom = conv_time2step(times(0), stepsPerPhase);
    int tto   = conv_time2step(times(1), stepsPerPhase);
    for(uint s=0; s<configurations.N; s++) {
      Configuration* C = configurations(s);
      int t = int(s) - k_order;
      if(t<=tfrom || t>tto) C->selectJointsBySubtrees({});
      else C->selectJointsBySubtrees(roots, notThose);
      C->ensure_q();
      C->checkConsistency();
    }
  }
}

void KOMO::setupConfigurations(const arr& q_init, const uintA& q_initJoints) {

  //IMPORTANT: The configurations need to include the k prefix configurations!
  //Therefore configurations(0) is for time=-k and configurations(k+t) is for time=t
  CHECK(configurations.N != k_order+T, "why setup again?");

  int xIndexCount=0;
  if(!configurations.N) { //add the initial configuration (with index -k_order )
    computeMeshNormals(world.frames, true);
    computeMeshGraphs(world.frames, true);

    rai::Configuration* C = configurations.append(new Configuration());
    C->copy(world, true);
    C->setTimes(tau);
    for(KinematicSwitch* sw:switches) { //apply potential switches
      if(sw->timeOfApplication+(int)k_order<=0) {
        sw->apply(C->frames);
      }
    }
    if(computeCollisions) {
#ifndef FCLmode
      C->stepSwift();
#else
      C->stepFcl();
#endif
    }
    C->ensure_q();
    C->checkConsistency();

    xIndexCount = -k_order*C->getJointStateDimension();
    xIndexCount += C->getJointStateDimension();
  }

  while(configurations.N<k_order+T) { //add further configurations
    uint s = configurations.N;
    rai::Configuration* C = configurations.append(new Configuration());
    C->copy(*configurations(s-1), true);
    CHECK_EQ(configurations(s), configurations.last(), "");
    C->setTimes(tau);
    if(!!q_init && s>k_order) C->setJointState(q_init, q_initJoints);
    for(KinematicSwitch* sw:switches) { //apply potential switches
      if(sw->timeOfApplication+k_order==s) {
        sw->apply(C->frames);
      }
    }
    if(computeCollisions) {
#ifndef FCLmode
      C->stepSwift();
#else
      C->stepFcl();
#endif
    }
    C->ensure_q();
    C->checkConsistency();

    xIndexCount += C->getJointStateDimension();
  }
}

void KOMO::retrospectApplySwitches(rai::Array<KinematicSwitch*>& _switches) {
  for(KinematicSwitch* sw:_switches) {
    uint s = sw->timeOfApplication+k_order;
    rai::Configuration* C = configurations.elem(s);
    rai::Frame* f = sw->apply(C->frames);
    s++;
    //apply the same switch on all following configurations!
    for(; s<k_order+T; s++) {
      rai::Configuration* C1 = configurations.elem(s);
      rai::Frame* f1 = sw->apply(C1->frames);
      if(f && f1) {
        f1->set_Q() = f->get_Q(); //copy the relative pose (switch joint initialization) from the first application
      }
    }
  }
}

void KOMO::retrospectChangeJointType(int startStep, int endStep, uint frameID, JointType newJointType) {
  uint s = startStep+k_order;
  //apply the same switch on all following configurations!
  for(; s<endStep+k_order; s++) {
    rai::Configuration* C = configurations.elem(s);
    rai::Frame* f = C->frames(frameID);
    f->setJoint(newJointType);
  }
}

void KOMO::set_x(const arr& x, const uintA& selectedConfigurationsOnly) {
  set_xCount++;
#ifdef KOMO_PATH_CONFIG
  set_x2(x, selectedConfigurationsOnly);
  return;
#endif

  if(!configurations.N) setupConfigurations();
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");

  uintA configs;
  if(!!selectedConfigurationsOnly) { //we only set some configurations: those listed in selectedConfigurationsOnly
    configs = selectedConfigurationsOnly;
  } else {
    configs.setStraightPerm(T); //by default, we loop straight through all configurations
  }

  //-- set the configurations' states
  uint x_count=0;
  for(uint t:configs) {
    uint s = t+k_order;
    uint x_dim = dim_x(t);
    if(x_dim) {
      rai::timerRead(true);
      if(x.nd==1)  configurations(s)->setJointState(x({x_count, x_count+x_dim-1}));
      else         configurations(s)->setJointState(x[t]);
      timeKinematics += rai::timerRead(true);
      if(computeCollisions) {
#ifndef FCLmode
        configurations(s)->stepSwift();
#else
        configurations(s)->stepFcl();
#endif
      }
      timeCollisions += rai::timerRead(true);
      x_count += x_dim;
    }
//    configurations(s)->checkConsistency();
  }
  CHECK_EQ(x_count, x.N, "");
}

void KOMO::reportProxies(std::ostream& os, double belowMargin) {
  int s=0;
  for(auto& K:configurations) {
    os <<" **** KOMO PROXY REPORT t=" <<s-(int)k_order <<endl;
    if(K->_state_proxies_isGood) {
      K->reportProxies(os, belowMargin);
    } else {
      os <<"  [not evaluated]" <<endl;
    }
    s++;
  }
}

rai::Graph KOMO::getContacts() {
  rai::Graph G;
  int s=0;
  for(auto& K:configurations) {
    for(rai::ForceExchange* con:K->forces) {
      Graph& g = G.newSubgraph();
      g.newNode<int>({"at"}, {}, s-(int)k_order);
      g.newNode<rai::String>({"from"}, {}, con->a.name);
      g.newNode<rai::String>({"to"}, {}, con->b.name);
      g.newNode<arr>({"force"}, {}, con->force);
      g.newNode<arr>({"poa"}, {}, con->poa);
    }
    s++;
  }
  return G;
}

bool KOMO::displayTrajectory(double delay, bool watch, bool overlayPaths, const char* saveVideoPath, const char* addText) {
  const char* tag = "KOMO planned trajectory";
  rai::String timetag;
  if(!gl) {
    gl = make_shared<OpenGL>("KOMO display");
    gl->camera.setDefault();
  }

  if(saveVideoPath) {
    rai::system(STRING("mkdir -p " <<saveVideoPath));
    rai::system(STRING("rm -f " <<saveVideoPath <<"*.ppm"));
  }

  uintA allFrames;
  NIY//  allFrames.setStraightPerm(configurations.first()->frames.N);
  arr X = getPath_frames(allFrames);
  DrawPaths drawX(X);

  for(int t=-(int)k_order; t<(int)T; t++) {
    rai::Configuration& K = *configurations(t+k_order);
    timetag.clear() <<tag <<" (config:" <<t <<'/' <<T <<"  s:" <<conv_step2time(t, stepsPerPhase) <<" tau:" <<K.frames.first()->tau <<')';
    if(addText) timetag <<addText;
//    K.reportProxies();
    K.orsDrawProxies=false;
    gl->clear();
    gl->add(glStandardScene, 0);
    gl->add(K);
    if(overlayPaths) gl->add(drawX);
    if(delay<0.) {
      if(delay<-10.) FILE("z.graph") <<K;
      gl->watch(timetag.p);
    } else {
      gl->update(timetag.p, true);
      if(delay) rai::wait(delay * K.frames.first()->tau);
    }
    if(saveVideoPath) write_ppm(gl->captureImage, STRING(saveVideoPath<<std::setw(4)<<std::setfill('0')<<t<<".ppm"));
  }
  if(watch) {
    int key = gl->watch(timetag.p);
    return !(key==27 || key=='q');
  }
  gl->clear();
  return false;
}

struct Conv_KOMO_KOMOProblem_toBeRetired : KOMO_Problem {
  KOMO& komo;
  uint dimPhi;
  uintA phiIndex, phiDim;
  StringA featureNames;

  Conv_KOMO_KOMOProblem_toBeRetired(KOMO& _komo) : komo(_komo) {}
  void clear() { dimPhi=0; phiIndex.clear(); phiDim.clear(); featureNames.clear(); }

  virtual uint get_k() { return komo.k_order; }
  virtual void getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes);
  virtual void phi(arr& phi, arrA& J, arrA& H, uintA& featureTimes, ObjectiveTypeA& tt, const arr& x);
} komo_problem;

void KOMO::Conv_KOMO_KOMOProblem_toBeRetired::getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes) {
  CHECK_EQ(komo.configurations.N, komo.k_order+komo.T, "configurations are not setup yet: use komo.reset()");

  if(!!variableDimensions) {
    variableDimensions.resize(komo.T);
    for(uint t=0; t<komo.T; t++) variableDimensions(t) = komo.configurations(t+komo.k_order)->getJointStateDimension();
  }

  if(!!featureTimes) featureTimes.clear();
  if(!!featureTypes) featureTypes.clear();
  featureNames.clear();
  uint M=0;
  phiIndex.resize(komo.T, komo.objectives.N); phiIndex.setZero();
  phiDim.resize(komo.T, komo.objectives.N);   phiDim.setZero();
  for(uint t=0; t<komo.T; t++) {
    for(uint i=0; i<komo.objectives.N; i++) {
      ptr<Objective> task = komo.objectives.elem(i);
      if(task->isActive(t)) {
        uint m = task->feat->__dim_phi(komo.configurations({t, t+komo.k_order})); //dimensionality of this task

        if(!!featureTimes) featureTimes.append(t, m); //consts<uint>(t, m));
        if(!!featureTypes) featureTypes.append(task->type, m); //consts<ObjectiveType>(task->type, m));
        for(uint j=0; j<m; j++)  featureNames.append(STRING(task->name <<'_'<<j));

        //store indexing phi <-> tasks
        phiIndex(t, i) = M;
        phiDim(t, i) = m;
        M += m;
      }
    }
  }
  dimPhi = M;
  CHECK_EQ(M, sum(phiDim), "");
}

bool WARN_FIRST_TIME=true;

void KOMO::Conv_KOMO_KOMOProblem_toBeRetired::phi(arr& phi, arrA& J, arrA& H, uintA& featureTimes, ObjectiveTypeA& tt, const arr& x) {
  const uintA prevPhiIndex=phiIndex, prevPhiDim=phiDim;

  //-- set the trajectory
  komo.set_x(x);

  if(!dimPhi) {
    LOG(-1) <<"no objectives! did you call getStructure first?";
  }
  //  getStructure(NoUintA, featureTimes, tt);
  //  if(WARN_FIRST_TIME){ LOG(-1)<<"calling inefficient getStructure"; WARN_FIRST_TIME=false; }
  phi.resize(dimPhi);
  if(!!tt) tt.resize(dimPhi);
  if(!!J) J.resize(dimPhi);

  arr y, Jy;
  uint M=0;
  for(uint t=0; t<komo.T; t++) {
    //build the Ktuple with order given by map
    ConfigurationL Ktuple = komo.configurations({t, t+komo.k_order});
    uintA Ktuple_dim = getKtupleDim(Ktuple);

    for(uint i=0; i<komo.objectives.N; i++) {
      ptr<Objective> task = komo.objectives.elem(i);
      if(task->isActive(t)) {
        //query the task map and check dimensionalities of returns
        task->feat->__phi(y, Jy, Ktuple);
//        uint m = task->feat->__dim_phi(Ktuple);
//        CHECK_EQ(m,y.N,"");
        if(!!J) CHECK_EQ(y.N, Jy.d0, "");
        if(!!J) CHECK_EQ(Jy.nd, 2, "");
        if(!!J) CHECK_EQ(Jy.d1, Ktuple_dim.last(), "");
        if(!y.N) continue;
        if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

        //write into phi and J
        phi.setVectorBlock(y, M);
        if(!!J) {
          if(isSpecial(Jy)) Jy = unpack(Jy);
//          Jy *= task->prec(t);
          if(t<komo.k_order) Jy.delColumns(0, Ktuple_dim(komo.k_order-t-1)); //delete the columns that correspond to the prefix!!
//          if(t<komo.k_order) Jy.delColumns(0,(komo.k_order-t)*komo.configurations(0)->q.N); //delete the columns that correspond to the prefix!!
          for(uint i=0; i<y.N; i++) J(M+i) = Jy[i]; //copy it to J(M+i); which is the Jacobian of the M+i'th feature w.r.t. its variables
        }
        if(!!tt) for(uint i=0; i<y.N; i++) tt(M+i) = task->type;

//        //store indexing phi <-> tasks
//        phiIndex(t, i) = M;
//        phiDim(t, i) = y.N;

        //counter for features phi
        M += y.N;
      }
    }
  }

  CHECK_EQ(M, dimPhi, "");
//  if(!!lambda) CHECK_EQ(prevLambda, lambda, ""); //this ASSERT only holds is none of the tasks is variable dim!
  komo.featureValues = phi;
  if(!!J) komo.featureJacobians = J;
  if(!!tt) komo.featureTypes = tt;

  reportAfterPhiComputation(komo);
}

struct Conv_KOMO_GraphProblem_toBeRetired : GraphProblem {
  KOMO& komo;
  uint dimPhi=0;

  Conv_KOMO_GraphProblem_toBeRetired(KOMO& _komo) : komo(_komo) {}
  void clear() { dimPhi=0; }

  virtual void getStructure(uintA& variableDimensions, intAA& featureVariables, ObjectiveTypeA& featureTypes);
  virtual void phi(arr& phi, arrA& J, arrA& H, const arr& x);

  virtual void setPartialX(const uintA& whichX, const arr& x);
  virtual void getPartialPhi(arr& phi, arrA& J, arrA& H, const uintA& whichPhi);
  virtual void getSemantics(StringA& varNames, StringA& phiNames);
};

void KOMO::Conv_KOMO_GraphProblem_toBeRetired::getStructure(uintA& variableDimensions, intAA& featureVariables, ObjectiveTypeA& featureTypes) {
  CHECK_EQ(komo.configurations.N, komo.k_order+komo.T, "configurations are not setup yet: use komo.reset()");
  if(!!variableDimensions) {
#if 1
    variableDimensions.resize(komo.T);
    for(uint t=0; t<komo.T; t++) variableDimensions(t) = komo.configurations(t+komo.k_order)->getJointStateDimension();
#else
    variableDimensions.clear();
    for(uint t=0; t<komo.T; t++) {
      uint n=komo.configurations(t+komo.k_order)->vars_getNum();
      for(uint i=0; i<n; i++) {
        variableDimensions.append() = komo.configurations(t+komo.k_order)->vars_getDim(i);
      }
    }
#endif
  }

  if(!!featureVariables) featureVariables.clear();
  if(!!featureTypes) featureTypes.clear();
  uint M=0;
  for(ptr<Objective>& ob:komo.objectives) {
    CHECK_EQ(ob->configs.nd, 2, "in sparse mode, vars need to be tuples of variables");
    for(uint l=0; l<ob->configs.d0; l++) {
      ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      uint m = ob->feat->__dim_phi(Ktuple); //dimensionality of this task
      if(!!featureVariables) featureVariables.append(ob->configs[l], m);
      if(!!featureTypes) featureTypes.append(ob->type, m);
      M += m;
    }
  }

  if(!!featureTypes) komo.featureTypes = featureTypes;

  dimPhi = M;
}

void KOMO::Conv_KOMO_GraphProblem_toBeRetired::getSemantics(StringA& varNames, StringA& phiNames) {
#if 1
  varNames.resize(komo.T);
  for(uint t=0; t<komo.T; t++) varNames(t) <<"config_" <<t;
#else
  varNames.clear();
  for(uint t=0; t<komo.T; t++) {
    uint n=komo.configurations(t+komo.k_order)->vars_getNum();
    for(uint i=0; i<n; i++) {
      varNames.append(STRING("config_" <<t <<"_var_" <<i));
    }
  }
#endif

  phiNames.clear();
  uint M=0;
  for(ptr<Objective>& ob:komo.objectives) {
    CHECK_EQ(ob->configs.nd, 2, "in sparse mode, vars need to be tuples of variables");
    for(uint l=0; l<ob->configs.d0; l++) {
      ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      uint m = ob->feat->__dim_phi(Ktuple); //dimensionality of this task
      phiNames.append(ob->name, m);
      M += m;
    }
  }
}

void KOMO::Conv_KOMO_GraphProblem_toBeRetired::phi(arr& phi, arrA& J, arrA& H, const arr& x) {
  //-- set the trajectory
  komo.set_x(x);

//  if(!dimPhi) getStructure();
  CHECK(dimPhi, "getStructure must be called first");
  phi.resize(dimPhi);
  if(!!J) J.resize(dimPhi);

  rai::timerStart();
  arr y, Jy;
//  Jy.sparse();
  uint M=0;
  for(ptr<Objective>& ob:komo.objectives) {
    CHECK_EQ(ob->configs.nd, 2, "in sparse mode, vars need to be tuples of variables");
    for(uint l=0; l<ob->configs.d0; l++) {
      ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      uintA kdim = getKtupleDim(Ktuple);
      kdim.prepend(0);

      //query the task map and check dimensionalities of returns
      ob->feat->__phi(y, Jy, Ktuple);
      if(!!J) CHECK_EQ(y.N, Jy.d0, "");
      if(!!J) CHECK_EQ(Jy.nd, 2, "");
      if(!!J) CHECK_EQ(Jy.d1, kdim.last(), "");
      if(!y.N) continue;
      if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

//      if(!!Jy) CHECK(isSparseMatrix(J), "");

      //write into phi and J
      phi.setVectorBlock(y, M);

      if(!!J) {
        for(uint j=ob->configs.d1; j--;) {
          if(ob->configs(l, j)<0) {
            if(isSpecial(Jy)) Jy = unpack(Jy);
            Jy.delColumns(kdim(j), kdim(j+1)-kdim(j)); //delete the columns that correspond to the prefix!!
          }
        }
        if(!isSparseMatrix(Jy)) {
          for(uint i=0; i<y.N; i++) J(M+i) = Jy[i];
        } else {
          Jy.sparse().setupRowsCols();
          for(uint i=0; i<y.N; i++) J(M+i) = Jy.sparse().getSparseRow(i);
        }
      }

      //counter for features phi
      M += y.N;
    }
  }
  komo.timeFeatures += rai::timerRead(true);

  CHECK_EQ(M, dimPhi, "");
  komo.featureValues = phi;

  reportAfterPhiComputation(komo);
}

void KOMO::Conv_KOMO_GraphProblem_toBeRetired::setPartialX(const uintA& whichX, const arr& x) {
  komo.set_x(x, whichX);
}
void KOMO::Conv_KOMO_GraphProblem_toBeRetired::getPartialPhi(arr& phi, arrA& J, arrA& H, const uintA& whichPhi) {
  //NON EFFICIENT

  {
    //copy and past from full phi!
    CHECK(dimPhi, "getStructure must be called first");
    if(!!phi) phi.resize(dimPhi);
    if(!!J) J.resize(dimPhi);

//    uintA x_index = getKtupleDim(komo.configurations({komo.k_order,-1}));
//    x_index.prepend(0);

    arr y, Jy;
    uint M=0;
    for(ptr<Objective>& ob:komo.objectives) {
      CHECK_EQ(ob->configs.nd, 2, "in sparse mode, vars need to be tuples of variables");
      for(uint l=0; l<ob->configs.d0; l++) {
        ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
        uintA kdim = getKtupleDim(Ktuple);
        kdim.prepend(0);

        //query the task map and check dimensionalities of returns
        ob->feat->__phi(y, Jy, Ktuple);
        if(!!J && isSpecial(Jy)) Jy = unpack(Jy);

        if(!!J) CHECK_EQ(y.N, Jy.d0, "");
        if(!!J) CHECK_EQ(Jy.nd, 2, "");
        if(!!J) CHECK_EQ(Jy.d1, kdim.last(), "");
        if(!y.N) continue;
        if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

        //write into phi and J
        if(!!phi) phi.setVectorBlock(y, M);

        if(!!J) {
          for(uint j=ob->configs.d1; j--;) {
            if(ob->configs(l, j)<0) {
              Jy.delColumns(kdim(j), kdim(j+1)-kdim(j)); //delete the columns that correspond to the prefix!!
            }
          }
          for(uint i=0; i<y.N; i++) J(M+i) = Jy[i];
        }

        //counter for features phi
        M += y.N;
      }
    }

    CHECK_EQ(M, dimPhi, "");
    //  if(!!lambda) CHECK_EQ(prevLambda, lambda, ""); //this ASSERT only holds is none of the tasks is variable dim!
    if(!!phi) komo.featureValues = phi;

    reportAfterPhiComputation(komo);
  }

  //now subselect features
  if(!!phi) phi = phi.sub(whichPhi);
  if(!!J) J = J.sub(whichPhi);
}

void KOMO::run_sub(const uintA& X, const uintA& Y) {
  Configuration::setJointStateCount=0;
  double timeZero = timerStart();
  if(opt) delete opt;

  {
    Conv_KOMO_GraphProblem_toBeRetired graph_problem(*this);
    GraphProblem_Structure Gstruct(graph_problem);
    //evaluate once with full parameters to adopt initialization
    {
      uintA X;
      X.setStraightPerm(Gstruct.V.N);
      SubGraphProblem G_X(Gstruct, X, {});
      G_X.phi(NoArr, NoArrA, NoArrA, x);
    }

    SubGraphProblem G_XY(Gstruct, X, Y);
    G_XY.optim(rai::MAX(verbose-2, 0));
    sos = G_XY.sos; eq = G_XY.eq; ineq = G_XY.ineq;
  }

  runTime = timerRead(true, timeZero);
  if(verbose>0) {
    cout <<"** optimization time=" <<runTime
         <<" (kin:" <<timeKinematics <<" coll:" <<timeCollisions <<" feat:" <<timeFeatures <<" newton: " <<timeNewton <<")"
         <<" setJointStateCount=" <<Configuration::setJointStateCount <<endl;
  }
  if(verbose>0) cout <<getReport(verbose>1) <<endl;
}

rai::Configuration& KOMO::getConfiguration(double phase) {
#ifdef KOMO_PATH_CONFIG
  HALT("can't use this anymore")
#endif
  if(!configurations.N) setupConfigurations();
  uint s = k_order + conv_time2step(phase, stepsPerPhase);
  return *configurations(s);
}

Configuration& KOMO::getConfiguration_t(int t) {
#ifdef KOMO_PATH_CONFIG
  HALT("can't use this anymore")
#endif
  if(!configurations.N) setupConfigurations();
  if(t<0) CHECK_LE(-t, (int)k_order, "");
  return *configurations(t+k_order);
}

uint KOMO::getPath_totalDofs() {
#ifdef KOMO_PATH_CONFIG
  HALT("can't use this anymore")
#endif
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");
  uint n=0;
  for(uint t=0; t<T; t++) n +=configurations(t+k_order)->getJointStateDimension();
  return n;
}

arr KOMO::getPath_decisionVariable() {
#ifndef KOMO_PATH_CONFIG
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");
  arr x;
  for(uint t=0; t<T; t++) x.append(configurations(t+k_order)->getJointState());
  return x;
#else
  return pathConfig.getJointState();
#endif
}

arr KOMO::getPath(const uintA& joints) {
#ifdef KOMO_PATH_CONFIG
  HALT("can't use this anymore")
#endif
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");
  uint n = joints.N;
  if(!n) n = world.getJointStateDimension();
  arr X(T, n);
  for(uint t=0; t<T; t++) {
    if(joints.N)
      X[t] = configurations(t+k_order)->getJointState(joints);
    else
      X[t] = configurations(t+k_order)->getJointState();
  }
  return X;
}

struct EffJointInfo {
  rai::Joint* j;
  rai::Transformation Q=0;
  int t, t_start=0, t_end=0;
  double accum=0.;
  EffJointInfo(rai::Joint* j, uint t): j(j), t(t) {}
  void write(ostream& os) const {
    os <<"EffInfo " <<j->frame->parent->name <<"->" <<j->frame->name <<" \t" <<j->type <<" \tt=" <<t_start <<':' <<t_end <<" \tQ=" <<Q;
  }
};
stdOutPipe(EffJointInfo)
bool operator==(const EffJointInfo&, const EffJointInfo&) { return false; }

rai::Array<rai::Transformation> KOMO::reportEffectiveJoints(std::ostream& os) {
  os <<"**** KOMO EFFECTIVE JOINTS" <<endl;
  Graph G;
  std::map<rai::Joint*, Node*> map;
  for(uint s=k_order+1; s<T+k_order; s++) {
    JointL matches = getMatchingJoints({configurations(s-1), configurations(s)}, true);
    for(uint i=0; i<matches.d0; i++) {
      JointL match = matches[i];
      auto* n = new Node_typed<EffJointInfo>(G, {match(1)->frame->name}, {}, EffJointInfo(match(1), s-k_order));
      map[match(1)] = n;
      if(map.find(match(0))==map.end()) map[match(0)] = new Node_typed<EffJointInfo>(G, {match(0)->frame->name}, {}, EffJointInfo(match(0), s-k_order-1));
      Node* other=map[match(0)];
      n->addParent(other);
    }
  }

//  for(uint t=0;t<T+k_order;t++){
//    rai::Configuration *K = configurations(t);
//    for(rai::Frame *f:K->frames){
//      if(f->joint && f->joint->constrainToZeroVel)
//        os <<" t=" <<t-k_order <<'\t' <<f->name <<" \t" <<f->joint->type <<" \tq=" <<f->joint->getQ() <<" \tQ=" <<f->Q <<endl;
//    }
//  }

//  G.displayDot();

  for(Node* n:G) {
    if(!n->parents.N) { //a root node -> accumulate all info
      EffJointInfo& info = n->get<EffJointInfo>();
      info.t_start = info.t_end = info.t;
      info.Q = info.j->frame->get_Q();
      info.accum += 1.;
      Node* c=n;
      for(;;) {
        if(!c->children.N) break;
        c = c->children.scalar();
        EffJointInfo& cinfo = c->get<EffJointInfo>();
        if(info.t_end<cinfo.t) info.t_end=cinfo.t;
        info.Q.rot.add(cinfo.j->frame->get_Q().rot);
        info.Q.pos += cinfo.j->frame->get_Q().pos;
        info.accum += 1.;
//        cout <<" t=" <<cinfo.t <<'\t' <<c->keys <<" \t" <<cinfo.j->type <<" \tq=" <<cinfo.j->getQ() <<" \tQ=" <<cinfo.j->frame->Q <<endl;
      }
      info.Q.pos /= info.accum;
      info.Q.rot.normalize();
      cout <<info <<endl;
    }
  }

  //-- align this with the switches and return the transforms
  uint s=0;
  rai::Array<rai::Transformation> Qs(switches.N);
  for(Node* n:G) {
    if(!n->parents.N) {
      EffJointInfo& info = n->get<EffJointInfo>();
#ifndef RAI_NOCHECK
      rai::KinematicSwitch* sw = switches(s);
      CHECK_EQ(info.t_start, sw->timeOfApplication, "");
      CHECK_EQ(info.j->type, sw->jointType, "");
//      CHECK_EQ(info.j->frame->parent->ID, sw->fromId, "");
//      CHECK_EQ(info.j->frame->ID, sw->toId, "");
#endif

      Qs(s) = info.Q;

      s++;
    }
  }

  cout <<Qs <<endl;

  return Qs;
}

