//===========================================================================
//
// deprecated
//

void deprecated_reportProblem(ostream& os=std::cout);
rai::Graph deprecated_getReport(bool plotOverTime=false, int reportFeatures=0, ostream& featuresOs=std::cout); ///< return a 'dictionary' summarizing the optimization results (optional: gnuplot objective costs; output detailed cost features per time slice)
rai::Graph deprecated_getProblemGraph(bool includeValues, bool includeSolution=true);

void addSquaredQuaternionNorms(const arr& times=NoArr, double scale=3e0) { DEPR; addQuaternionNorms(times, scale); }

bool displayTrajectory(double delay=1., bool watch=true, bool overlayPaths=true, const char* saveVideoPath=nullptr, const char* addText=nullptr) {
  DEPR; return view_play(watch, delay, saveVideoPath);
}
bool displayPath(const char* txt, bool watch=true, bool full=true) {
  DEPR; return view(watch, txt);
}
rai::Camera& displayCamera();

void add_StableRelativePose(const std::vector<int>& confs, const char* gripper, const char* object) {
  DEPR;
  for(uint i=1; i<confs.size(); i++)
    addObjective(arr{(double)confs[0], (double)confs[i]}, FS_poseRel, {gripper, object}, OT_eq);
  world.makeObjectsFree({object});
}
void add_StablePose(const std::vector<int>& confs, const char* object) {
  DEPR;
  for(uint i=1; i<confs.size(); i++)
    addObjective(arr{(double)confs[0], (double)confs[i]}, FS_pose, {object}, OT_eq);
  world.makeObjectsFree({object});
}
void add_grasp(int conf, const char* gripper, const char* object) {
  DEPR;
  addObjective(arr{(double)conf}, FS_distance, {gripper, object}, OT_eq);
}
void add_place(int conf, const char* object, const char* table) {
  DEPR;
  addObjective(arr{(double)conf}, FS_aboveBox, {table, object}, OT_ineq);
  addObjective(arr{(double)conf}, FS_standingAbove, {table, object}, OT_eq);
  addObjective(arr{(double)conf}, FS_vectorZ, {object}, OT_sos, {}, {0., 0., 1.});
}
void add_resting(int conf1, int conf2, const char* object) {
  DEPR;
  addObjective(arr{(double)conf1, (double)conf2}, FS_pose, {object}, OT_eq);
}
void add_restingRelative(int conf1, int conf2, const char* object, const char* tableOrGripper) {
  DEPR;
  addObjective(arr{(double)conf1, (double)conf2}, FS_poseRel, {tableOrGripper, object}, OT_eq);
}
void activateCollisions(const char* s1, const char* s2) { DEPR; HALT("see komo-21-03-06"); }
void deactivateCollisions(const char* s1, const char* s2);
//arr getFrameStateX(int t){ DEPR; return getConfiguration_X(t); }
//arr getPath_qAll(int t){ DEPR; return getConfiguration_qOrg(t); }
//arr getConfiguration_q(int t) { DEPR; return getConfiguration_qAll(t); }
//arr getPath_qOrg(uintA joints, const bool activesOnly){ DEPR; return getPath_qOrg(); }

//===========================================================================

void KOMO::deprecated_reportProblem(std::ostream& os) {
  os <<"KOMO Problem:" <<endl;
  os <<"  x-dim:" <<x.N <<"  dual-dim:" <<dual.N <<endl;
  os <<"  T:" <<T <<" k:" <<k_order <<" phases:" <<double(T)/stepsPerPhase <<" stepsPerPhase:" <<stepsPerPhase <<" tau:" <<tau <<endl;
  os <<"  #timeSlices:" <<timeSlices.d0 <<" #totalDOFs:" <<pathConfig.getJointStateDimension() <<" #frames:" <<pathConfig.frames.N;
  os <<"  #pathQueries:" <<pathConfig.setJointStateCount;
  os <<endl;

  arr times = getPath_times();
  if(times.N>10) times.resizeCopy(10);
  os <<"    times:" <<times <<endl;

  os <<"  computeCollisions:" <<computeCollisions <<endl;
  for(shared_ptr<Objective>& t:objectives) {
    os <<"    " <<*t;
    if(t->times.N && t->times.elem(0)==-10.) {
      os <<"  timeSlices: [" <<t->times <<"]" <<endl;
    } else {
      int fromStep, toStep;
      conv_times2steps(fromStep, toStep, t->times, stepsPerPhase, T, +0, +0);
      os <<"  timeSlices: [" <<fromStep <<".." <<toStep <<"]" <<endl;
    }
  }
  for(std::shared_ptr<KinematicSwitch>& sw:switches) {
    os <<"    ";
    if(sw->timeOfApplication+k_order >= timeSlices.d0) {
//      LOG(-1) <<"switch time " <<sw->timeOfApplication <<" is beyond time horizon " <<T;
      sw->write(os, {});
    } else {
      sw->write(os, timeSlices[sw->timeOfApplication+k_order]);
    }
    os <<endl;
  }

  if(opt.verbose>6) {
    os <<"  INITIAL STATE" <<endl;
    for(rai::Frame* f:pathConfig.frames) {
      if(f->joint && f->joint->dim) os <<"    " <<f->name <<" [" <<f->joint->type <<"] : " <<f->joint->calcDofsFromConfig() /*<<" - " <<pathConfig.q.elem(f->joint->qIndex)*/ <<endl;
      for(auto* ex:f->forces) os <<"    " <<f->name <<" [force " <<ex->a.name <<'-' <<ex->b.name <<"] : " <<ex->force /*<<' ' <<ex->torque*/ <<' ' <<ex->poa <<endl;
    }
  }
}

Camera& KOMO::displayCamera() { DEPR; return pathConfig.get_viewer()->displayCamera(); }

rai::Graph KOMO::deprecated_getReport(bool plotOverTime, int reportFeatures, std::ostream& featuresOs) {
  //-- collect all task costs and constraints
  StringA name; name.resize(objectives.N);
  arr err=zeros(T, objectives.N);
  arr dualSolution; if(dual.N) dualSolution=zeros(T, objectives.N);
  arr taskC=zeros(objectives.N);
  arr taskG=zeros(objectives.N);
  arr taskH=zeros(objectives.N);
  arr taskF=zeros(objectives.N);
  uint M=0;
  for(shared_ptr<GroundedObjective>& ob:objs) {
    uint d = ob->feat->dim(ob->frames);
    int i = ob->objId;
    CHECK_GE(i, 0, "");
    uint time = ob->timeSlices.last();
    //          for(uint j=0; j<d; j++) CHECK_EQ(featureTypes(M+j), ob->type, "");
    if(d) {
      if(ob->type==OT_sos) {
        for(uint j=0; j<d; j++) err(time, i) += sqr(featureValues(M+j));
        taskC(i) += err(time, i);
      }
      if(ob->type==OT_ineq) {
        for(uint j=0; j<d; j++) err(time, i) += MAX(0., featureValues(M+j));
        taskG(i) += err(time, i);
      }
      if(ob->type==OT_eq) {
        for(uint j=0; j<d; j++) err(time, i) += fabs(featureValues(M+j));
        taskH(i) += err(time, i);
      }
      if(ob->type==OT_f) {
        for(uint j=0; j<d; j++) err(time, i) += featureValues(M+j);
        taskF(i) += err(time, i);
      }
      M += d;
    }
    //        }
    if(reportFeatures==1) {
      featuresOs <<std::setw(4) <<time <<' ' <<std::setw(2) <<i <<' ' <<std::setw(2) <<d <<ob->timeSlices
                 <<' ' <<std::setw(40) <<typeid(*ob->feat).name()
                 <<" k=" <<ob->feat->order <<" ot=" <<ob->type <<" prec=" <<std::setw(4) <<ob->feat->scale;
      if(ob->feat->target.N<5) featuresOs <<" y*=[" <<ob->feat->target <<']'; else featuresOs<<"y*=[..]";
      featuresOs <<" y^2=" <<err(time, i) <<endl;
    }
  }
  CHECK_EQ(M, featureValues.N, "");

  //-- generate a report graph
  rai::Graph report;
  double totalC=0., totalG=0., totalH=0., totalF=0.;
  for(uint i=0; i<objectives.N; i++) {
    shared_ptr<Objective> c = objectives(i);
    Graph& g = report.addSubgraph(STRING('o' <<i));
    g.add<String>("name", c->feat->typeString());
    uintA frameIDs = c->feat->frameIDs;
    if(frameIDs.N<=3) {
      g.add<StringA>("frames", framesToNames(world.getFrames(frameIDs)));
    } else {
      g.add<StringA>("frames", {STRING("#" <<frameIDs.N)});
    }
    g.add<double>("order", c->feat->order);
    g.add<String>("type", Enum<ObjectiveType>(c->type).name());
    if(taskC(i)) g.add<double>("sos", taskC(i));
    if(taskG(i)) g.add<double>("ineq", taskG(i));
    if(taskH(i)) g.add<double>("eq", taskH(i));
    if(taskF(i)) g.add<double>("f", taskF(i));
    totalC += taskC(i);
    totalG += taskG(i);
    totalH += taskH(i);
    totalF += taskF(i);
  }
  report.add<double>("sos", totalC);
  report.add<double>("ineq", totalG);
  report.add<double>("eq", totalH);
  report.add<double>("f", totalF);

  if(plotOverTime) {
    //-- write a nice gnuplot file
    ofstream fil("z.costReport");
    //first line: legend
    for(auto c:objectives) fil <<c->name <<' ';
    for(auto c:objectives) if(c->type==OT_ineq && dualSolution.N) fil <<c->name <<"_dual ";
    fil <<endl;

    //rest: just the matrix
    if(true) { // && !dualSolution.N) {
      err.write(fil, nullptr, nullptr, "  ");
    } else {
      dualSolution.reshape(T, dualSolution.N/(T));
      catCol(err, dualSolution).write(fil, nullptr, nullptr, "  ");
    }
    fil.close();

    ofstream fil2("z.costReport.plt");
    fil2 <<"set key autotitle columnheader" <<endl;
    fil2 <<"set title 'costReport ( plotting sqrt(costs) )'" <<endl;
    fil2 <<"plot 'z.costReport' \\" <<endl;
    for(uint i=1; i<=objectives.N; i++) fil2 <<(i>1?"  ,''":"     ") <<" u (($0+1)/" <<stepsPerPhase <<"):"<<i<<" w l lw 3 lc " <<i <<" lt " <<1-((i/10)%2) <<" \\" <<endl;
    if(dualSolution.N) for(uint i=0; i<objectives.N; i++) fil2 <<"  ,'' u (($0+1)/" <<stepsPerPhase <<"):"<<1+objectives.N+i<<" w l \\" <<endl;
    fil2 <<endl;
    fil2.close();

    if(plotOverTime) {
//      cout <<"KOMO Report\n" <<report <<endl;
      gnuplot("load 'z.costReport.plt'");
    }
  }

  return report;
}

/// output the defined problem as a generic graph, that can also be displayed, saved and loaded
rai::Graph KOMO::deprecated_getProblemGraph(bool includeValues, bool includeSolution) {
  rai::Graph K;
  //header
#if 1
  Graph& g = K.addSubgraph("specs");
  g.add<uint>("x_dim", x.N);
  g.add<uint>("T", T);
  g.add<uint>("k_order", k_order);
  g.add<double>("phases", double(T)/stepsPerPhase);
  g.add<uint>("stepsPerPhase", stepsPerPhase);
  g.add<double>("tau", tau);

  g.add<uint>("#timeSlices", timeSlices.d0);
  g.add<uint>("#totalDOFs", pathConfig.getJointStateDimension());
  g.add<uint>("#frames", pathConfig.frames.N);

  //  uintA dims(configurations.N);
  //  for(uint i=0; i<configurations.N; i++) dims(i)=configurations(i)->q.N;
  //  g.newNode<uintA>({"q_dims"}, dims);
  //  arr times(configurations.N);
  //  for(uint i=0; i<configurations.N; i++) times(i)=configurations(i)->frames.first()->time;
  //  g.newNode<double>({"times"}, times);
  g.add<bool>("computeCollisions", computeCollisions);
#endif

  if(includeSolution) {
    //full configuration paths
    g.add<arr>("X", getPath_X());
    g.add<arrA>("x", getPath_qAll());
    g.add<arr>("dual", dual);
  }

  //objectives
  for(shared_ptr<GroundedObjective>& ob:objs) {

    Graph& g = K.addSubgraph(ob->feat->shortTag(pathConfig));
    g.add<double>("order", ob->feat->order);
    g.add<String>("type", STRING(ob->type));
    g.add<String>("feature", ob->feat->shortTag(pathConfig));
    if(ob->timeSlices.N) g.add<intA>("vars", ob->timeSlices);
//    g.copy(task->feat->getSpec(world), true);
    if(includeValues) {
      arr y;
      arrA V, J;
      for(uint l=0; l<ob->timeSlices.d0; l++) {
//        ConfigurationL Ktuple = configurations.sub(convert<uint, int>(ob->timeSlices[l]+(int)k_order));
//        ob->feat->eval(y, Jy, Ktuple);
        y = ob->feat->eval(ob->frames);
        if(isSpecial(y.J())) y.J() = unpack(y.J());

        V.append(y);
        J.append(y.J());
      }
      g.add<arrA>("y", V);
      g.add<arrA>("J", J);

      arr Vflat;
      for(arr& v: V) Vflat.append(v);

      if(ob->type==OT_sos) {
        g.add<double>("sos_sumOfSqr", sumOfSqr(Vflat));
      } else if(ob->type==OT_eq) {
        g.add<double>("eq_sumOfAbs", sumOfAbs(Vflat));
      } else if(ob->type==OT_ineq) {
        double c=0.;
        for(double& v:Vflat) if(v>0) c+=v;
        g.add<double>("inEq_sumOfPos", c);
      }
    }
  }

  return K;
}

void KOMO::deprecated_run(OptOptions options) {
  Configuration::setJointStateCount=0;
  if(opt.verbose>0) {
    cout <<"** KOMO::run "
         <<" collisions:" <<computeCollisions
         <<" x-dim:" <<x.N
         <<" T:" <<T <<" k:" <<k_order <<" phases:" <<double(T)/stepsPerPhase <<" stepsPerPhase:" <<stepsPerPhase <<" tau:" <<tau;
    cout <<"  #timeSlices:" <<timeSlices.d0 <<" #totalDOFs:" <<pathConfig.getJointStateDimension() <<" #frames:" <<pathConfig.frames.N;
    cout <<endl;
  }

  options.verbose = rai::MAX(opt.verbose-2, 0);
  timeTotal -= rai::cpuTime();
  CHECK(T, "");
  if(logFile)(*logFile) <<"KOMO_run_log: [" <<endl;

  {
    OptConstrained _opt(x, dual, nlp(), options, logFile);
    _opt.run();
    timeNewton += _opt.newton.timeNewton;
  }

  timeTotal += rai::cpuTime();

  if(logFile)(*logFile) <<"\n] #end of KOMO_run_log" <<endl;
  if(opt.verbose>0) {
    cout <<"** optimization time:" <<timeTotal
         <<" (kin:" <<timeKinematics <<" coll:" <<timeCollisions <<" feat:" <<timeFeatures <<" newton: " <<timeNewton <<")"
         <<" setJointStateCount:" <<Configuration::setJointStateCount
         <<"\n   sos:" <<sos <<" ineq:" <<ineq <<" eq:" <<eq <<endl;
  }
  if(opt.verbose>1) cout <<report(false, true, opt.verbose>2) <<endl;
}

void KOMO::retrospectApplySwitches() {
  DEPR; //switches are applied immediately on addSwitch
  for(std::shared_ptr<KinematicSwitch>& sw:switches) applySwitch(*sw);
}

void KOMO::checkBounds(const arr& x) {
  DEPR; //should not be necessary, I think!, or move to kin!

  arr bounds = getBounds();
  CHECK_EQ(x.N, bounds.d1, "");
  boundCheck(x, bounds);
}

void KOMO::retrospectChangeJointType(int startStep, int endStep, uint frameID, JointType newJointType) {
  uint s = startStep+k_order;
  //apply the same switch on all following configurations!
  for(; s<endStep+k_order; s++) {
    rai::Frame* f = timeSlices(s, frameID);
    f->setJoint(newJointType);
  }
}

arr KOMO::getActiveConstraintJacobian() {
  uint n=0;
  for(uint i=0; i<dual.N; i++) if(dual.elem(i)>0.) n++;

  arr J(n, x.N);

  n=0;
  for(uint i=0; i<dual.N; i++) {
    if(dual.elem(i)>0.) {
      J[n] = featureJacobians.scalar()[i];
      n++;
    }
  }
  CHECK_EQ(n, J.d0, "");

  return J;
}

double KOMO::getConstraintViolations() {
  Graph R = report();
  return R.get<Graph>("totals").get<double>("ineq") + R.get<Graph>("totals").get<double>("eq");
}

double KOMO::getCosts() {
  Graph R = report();
  return R.get<Graph>("totals").get<double>("sos");
}
