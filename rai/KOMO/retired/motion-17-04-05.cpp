/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "motion.h"
#include "taskMaps.h"
#include "../Gui/opengl.h"
#include "../Kin/kin_swift.h"
#include <climits>
#include <iomanip>

//===========================================================================
//#define T T //(T+1)
//#define t<T (t<T) //(t<=T)
//===========================================================================

void Task::setCostSpecs(int fromTime,
                        int toTime,
                        const arr& _target,
                        double _prec) {
  if(!!_target) target = _target; else target = {0.};
  if(fromTime<0) fromTime=0;
  CHECK_GE(toTime, fromTime, "");
  prec.resize(toTime+1).setZero();
  for(uint t=fromTime; t<=(uint)toTime; t++) prec(t) = _prec;
}

#define STEP(t) (floor(t*double(stepsPerPhase) + .500001))-1

void Task::setCostSpecs(double fromTime, double toTime, int stepsPerPhase, uint T, const arr& _target, double _prec) {
  if(stepsPerPhase<0) stepsPerPhase=T;
  if(STEP(toTime)>T-1) LOG(-1) <<"beyond the time!: endTime=" <<toTime <<" phases=" <<double(T)/stepsPerPhase;
  int tFrom = (fromTime<0.?0:STEP(fromTime)+map->order);
  int tTo = (toTime<0.?T-1:STEP(toTime));
  if(tTo<0) tTo=0;
  if(tFrom>tTo && tFrom-tTo<=(int)map->order) tFrom=tTo;

  setCostSpecs(tFrom, tTo, _target, _prec);
}

//===========================================================================

Task* Task::newTask(const Node* specs, const rai::Configuration& world, int stepsPerPhase, uint T) {
  if(specs->parents.N<2) return nullptr; //these are not task specs

  //-- check the term type first
  ObjectiveType termType;
  rai::String& tt=specs->parents(0)->keys.last();
  if(tt=="MinSumOfSqr") termType=OT_sos;
  else if(tt=="LowerEqualZero") termType=OT_ineq;
  else if(tt=="EqualZero") termType=OT_eq;
  else return nullptr;

  //-- try to crate a map
  Feature* map = Feature::newTaskMap(specs, world);
  if(!map) return nullptr;

  //-- create a task
  Task* task = new Task(map, termType);

  if(specs->keys.N) task->name=specs->keys.last();
  else {
    task->name = map->shortTag(world);
//    for(Node *p:specs->parents) task->name <<'_' <<p->keys.last();
    task ->name<<"_o" <<task->feat->order;
  }

  //-- check for additional continuous parameters
  if(specs->isGraph()) {
    const Graph& params = specs->graph();
    arr time = params.get<arr>("time", {0., 1.});
    task->setCostSpecs(time(0), time(1), stepsPerPhase, T, params.get<arr>("target", {}), params.get<double>("scale", {1.}));
  } else {
    task->setCostSpecs(0, T-1, {}, 1.);
  }
  return task;
}

//===========================================================================

KOMO::KOMO(rai::Configuration& originalWorld, bool useSwift)
  : world(originalWorld), useSwift(useSwift), T(0), tau(0.), k_order(2), gl(nullptr), invKin_problem(*this), komo_problem(*this) {
  if(useSwift) {
    makeConvexHulls(originalWorld.shapes);
    originalWorld.swift().setCutoff(2.*rai::getParameter<double>("swiftCutoff", 0.11));
  }
  computeMeshNormals(originalWorld.shapes);
  setTiming(rai::getParameter<uint>("timeSteps", 50), rai::getParameter<double>("duration", 5.));
}

KOMO::~KOMO() {
  if(gl) delete gl;
  listDelete(configurations);
  listDelete(tasks);
  listDelete(switches);
}

KOMO& KOMO::operator=(const KOMO& other) {
  HALT("does the following work and make sense?");
  world = other.world; //const_cast<const rai::Configuration&>(other.world);
  useSwift = other.useSwift;
  tasks = other.tasks;
  T = other.T;
  tau = other.tau;
  k_order = other.k_order;
  featureValues = other.featureValues;
  dualSolution = other.dualSolution;
  featureTypes = other.featureTypes;
  return *this;
}

void KOMO::setTiming(uint steps, double duration) {
  T = steps;
  CHECK(T, "using T=0 to indicate inverse kinematics is deprecated.");
  if(T) tau = duration/T; else tau=duration;
}

Task* KOMO::addTask(const char* name, Feature* m, const ObjectiveType& termType) {
  Task* t = new Task(m, termType);
  t->name=name;
  tasks.append(t);
  return t;
}

bool KOMO::parseTask(const Node* n, int stepsPerPhase) {
  if(stepsPerPhase==-1) stepsPerPhase=T;
  //-- task?
  Task* task = Task::newTask(n, world, stepsPerPhase, T);
  if(task) {
    tasks.append(task);
    return true;
  }
  //-- switch?
  rai::KinematicSwitch* sw = rai::KinematicSwitch::newSwitch(n, world, stepsPerPhase, T);
  if(sw) {
    switches.append(sw);
    return true;
  }
//  LOG(-1) <<"task spec '" <<*n <<"' could not be parsed";
  return false;
}

void KOMO::parseTasks(const Graph& specs, int stepsPerPhase) {
  for(Node* n:specs) parseTask(n, stepsPerPhase);

  //-- add TransitionTask for InvKinematics
  if(!T) {
    Feature* map = new F_qItself();
    map->order = 0;
    Task* task = new Task(map, OT_sos);
    task->name="InvKinTransition";
    task->setCostSpecs(0, 0, world.q, 1./(tau*tau));
    tasks.append(task);
  }
}

#if 0
uint KOMO::dim_phi(uint t) {
  uint m=0;
  for(Task* c: tasks) {
//        CHECK_LE(c->prec.N, T,"");
    if(c->prec.N>t && c->prec(t))
      m += c->feat->dim_phi(configurations({t, t+k_order}), t); //counts also constraints
  }
  return m;
}

uint KOMO::dim_g(uint t) {
  uint m=0;
  for(Task* c: tasks) {
    if(c->type==OT_ineq && c->prec.N>t && c->prec(t))
      m += c->feat->dim_phi(configurations({t, t+k_order}), t);
  }
  return m;
}

uint KOMO::dim_h(uint t) {
  uint m=0;
  for(Task* c: tasks) {
    if(c->type==OT_eq && c->prec.N>t && c->prec(t))
      m += c->feat->dim_phi(configurations({t, t+k_order}), t);
  }
  return m;
}
#endif

void KOMO::setupConfigurations() {

  //IMPORTANT: The configurations need to include the k prefix configurations!
  //Therefore configurations(0) is for time=-k and configurations(k+t) is for time=t
  CHECK(!configurations.N, "why setup again?");
//    listDelete(configurations);

  configurations.append(new rai::Configuration())->copy(world, true);
  for(uint s=1; s<k_order+T; s++) {
    configurations.append(new rai::Configuration())->copy(*configurations(s-1), true);
    CHECK_EQ(configurations(s), configurations.last(), "");
    //apply potential graph switches
    for(rai::KinematicSwitch* sw:switches) {
      if(sw->timeOfApplication+k_order==s) {
        sw->apply(*configurations(s));
        //          if(MP.useSwift) configurations(t)->swift().initActivations(*configurations(t));
      }
    }
  }
}

void KOMO::set_x(const arr& x) {
  if(!configurations.N) setupConfigurations();
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");

  //-- set the configurations' states
  uint x_count=0;
  for(uint t=0; t<T; t++) {
    uint s = t+k_order;
    uint x_dim = dim_x(t); //configurations(s)->getJointStateDimension();
//    temporallyAlignKinematicSwitchesInConfiguration(t); //this breaks the jacobian check
    if(x_dim) {
      if(x.nd==1) configurations(s)->setJointState(x({x_count, x_count+x_dim-1}));
      else        configurations(s)->setJointState(x[t]);
      if(useSwift) configurations(s)->stepSwift();
      x_count += x_dim;
    }
  }
  CHECK_EQ(x_count, x.N, "");
}

/// this sets the t'th configuration and then redefines all joints as fixed -> no DOFs anymore in this time slice
void KOMO::set_fixConfiguration(const arr& x, uint t) {
  if(!configurations.N) setupConfigurations();
  CHECK(t<T, "");
  rai::Configuration* W=configurations(t+k_order);
  W->setJointState(x);
  if(useSwift) W->stepSwift();
  W->zeroGaugeJoints();
  for(rai::Joint* j:W->joints) j->type = rai::JT_rigid;
  W->meldFixedJoints();
}

bool KOMO::displayTrajectory(int steps, const char* tag, double delay) {
  if(!gl) {
    gl = new OpenGL("KOMO display");
    gl->camera.setDefault();
  }

  bool watch = (steps==-1);
  if(steps==1 || steps==-1) {
    steps=1;
  } else {
    steps = T/steps;
  }

  for(uint t=0; t<T; t+=steps) {
    gl->clear();
    gl->add(glStandardScene, 0);
    gl->addDrawer(configurations(t+k_order));
    if(delay<0.) {
      if(delay<-10.) FILE("z.graph") <<*configurations(t+k_order);
      gl->watch(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
    } else {
      gl->update(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
      if(delay) rai::wait(delay);
    }
  }
  if(watch) {
    int key = gl->watch(STRING(tag <<" (time " <<std::setw(3) <<T <<'/' <<T <<')').p);
    return !(key==27 || key=='q');
  } else
    return false;
}

#if 0
void KOMO::phi_t(arr& phi, arr& J, ObjectiveTypeA& tt, uint t) {
#if 0
  phi.clear();
  if(!!tt) tt.clear();
  if(!!J) J.clear();
#endif
  arr y, Jy, Jtmp;
  uint dimPhi_t=0;
  for(Task* task: tasks) if(task->prec.N>t && task->prec(t)) {
      task->feat->phi(y, Jy, configurations({t, t+k_order}), tau, t);
      if(!y.N) continue;
      dimPhi_t += y.N;
      if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

      //linear transform (target shift)
      if(task->target.N==1) y -= task->target.elem(0);
      else if(task->target.nd==1) y -= task->target;
      else if(task->target.nd==2) y -= task->target[t];
      y *= sqrt(task->prec(t));
      phi.append(y);

      if(!!J) {
        Jy *= sqrt(task->prec(t));
        Jtmp.append(Jy);
      }

      if(!!tt) for(uint i=0; i<y.N; i++) tt.append(task->type);
    }
  if(!!J && dimPhi_t) {
    Jtmp.reshape(dimPhi_t, Jtmp.N/dimPhi_t);
    if(t<k_order) Jtmp.delColumns(0, (k_order-t)*configurations(0)->q.N); //delete the columns that correspond to the prefix!!
    J.append(Jtmp);
  }

  CHECK_EQ(dimPhi_t, dim_phi(t), "");

  //memorize for report
  if(!phiMatrix.N) phiMatrix.resize(T);
  phiMatrix(t) = phi;
  if(!!tt) {
    if(!ttMatrix.N) ttMatrix.resize(T);
    ttMatrix(t) = tt;
  }
}

StringA KOMO::getPhiNames(uint t) {
  StringA names(dim_phi(t));
  uint m=0;
  for(Task* c: tasks) if(c->prec.N>t && c->prec(t)) {
      if(c->type==OT_sos) {
        uint d = c->feat->dim_phi(configurations({t, t+k_order}), t); //counts also constraints
        for(uint i=0; i<d; i++) {
          names(m+i)=c->name;
          names(m+i) <<"_f" <<i;
        }
        m+=d;
      }
    }
  for(Task* c: tasks) if(c->prec.N>t && c->prec(t)) {
      if(c->type==OT_ineq) {
        uint d = c->feat->dim_phi(configurations({t, t+k_order}), t); //counts also constraints
        for(uint i=0; i<d; i++) {
          names(m+i)=c->name;
          names(m+i) <<"_g" <<i;
        }
        m+=d;
      }
    }
  CHECK_EQ(m, names.N, "");
  return names;
}
#endif

void KOMO::reportFeatures(bool brief, ostream& os) {
  os <<"*** KOMO -- FeatureReport " <<endl;

  os <<"  useSwift=" <<useSwift <<endl;
  os <<"  T=" <<T <<endl;
  os <<"  tau=" <<tau <<endl;
  os <<"  k_order=" <<k_order <<endl;

  if(!configurations.N) setupConfigurations();

  os <<"  TASKS (as list): " <<tasks.N <<endl;
  for(Task* t:tasks) {
    os <<"  " <<*t <<endl;
  }

  os <<"  SWITCHES: " <<switches.N <<endl;
  for(rai::KinematicSwitch* sw:switches) {
    if(sw->timeOfApplication+k_order<configurations.N)
      os <<sw->shortTag(configurations(sw->timeOfApplication+k_order));
    else
      os <<sw->shortTag(nullptr);
  }

  //-- collect all task costs and constraints
  os <<"  TASKS (time idx dim name order type target scale ttMatrix phiMatrix):" <<endl;
  uint M=0;
  for(uint t=0; t<T; t++) {
    for(uint i=0; i<tasks.N; i++) {
      Task* task = tasks(i);
      if(!task->isActive(t)) continue;
      uint d=task->feat->dim_phi(configurations({t, t+k_order}), t);
      if(brief) {
        if(d) {
          os <<"  " <<t <<' ' <<i <<' ' <<d
             <<' ' <<std::setw(10) <<task->name
             <<' ' <<task->feat->order <<' ' <<task->type <<' ';
          if(task->target.N<5) os <<'[' <<task->target <<']'; else os<<"[..]";
          os <<' ' <<task->prec(t);
          if(featureTypes.N) {
            os <<' ' <<featureTypes.scalar().elem(M)
               <<' ' <<sumOfSqr(featureValues.scalar()({M, M+d-1}));
          }
          os <<endl;
        }
      } else {
        for(uint i=0; i<d; i++) {
          os <<"  " <<t <<' ' <<i
             <<' ' <<std::setw(10) <<task->name
             <<' ' <<task->feat->order <<' ' <<task->type <<' ';
          if(task->target.N==1) os <<task->target.elem(0);
          else if(task->target.nd==1) os <<task->target(i);
          else if(task->target.nd==2) os <<task->target(t, i);
          else os <<"00";
          os <<' ' <<task->prec(t);
          if(featureTypes.N) {
            os <<' ' <<featureTypes(t)(M+i)
               <<' ' <<featureValues(t)(M+i);
          }
          os <<endl;
        }
      }

      M += d;
    }
  }
  if(featureValues.N) CHECK_EQ(M, featureValues.scalar().N, "");
}

void KOMO::reportProxies(std::ostream& os) {
  int t=0;
  for(auto& K:configurations) {
    os <<" **** KOMO PROXY REPORT t=" <<t-k_order <<endl;
    K->reportProxies(os);
    t++;
  }
}

Graph KOMO::getReport(bool gnuplt, int reportFeatures) {
  if(featureValues.N>1) { //old optimizer -> remove some time..
    arr tmp;
    for(auto& p:featureValues) tmp.append(p);
    featureValues = ARRAY<arr>(tmp);

    ObjectiveTypeA ttmp;
    for(auto& p:featureTypes) ttmp.append(p);
    featureTypes = ARRAY<ObjectiveTypeA>(ttmp);
  }

  const arr& phi = featureValues.scalar();
  const ObjectiveTypeA& tt = featureTypes.scalar();

  //-- collect all task costs and constraints
  StringA name; name.resize(tasks.N);
  arr err=zeros(T, tasks.N);
  arr taskC=zeros(tasks.N);
  arr taskG=zeros(tasks.N);
  uint M=0;
  for(uint t=0; t<T; t++) {
    for(uint i=0; i<tasks.N; i++) {
      Task* task = tasks(i);
      if(task->prec.N>t && task->prec(t)) {
        uint d=task->feat->dim_phi(configurations({t, t+k_order}), t);
        for(uint j=0; j<d; j++) CHECK_EQ(tt(M+j), task->type, "");
        if(d) {
          if(task->type==OT_sos) {
            for(uint j=0; j<d; j++) err(t, i) += rai::sqr(phi(M+j)); //sumOfSqr(phi.sub(M,M+d-1));
            taskC(i) += err(t, i);
          }
          if(task->type==OT_ineq) {
            for(uint j=0; j<d; j++) err(t, i) += rai::MAX(0., phi(M+j));
            taskG(i) += err(t, i);
          }
          if(task->type==OT_eq) {
            for(uint j=0; j<d; j++) err(t, i) += fabs(phi(M+j));
            taskG(i) += err(t, i);
          }
          M += d;
        }
        if(reportFeatures==1) {
          cout <<std::setw(4) <<t <<' ' <<std::setw(2) <<i <<' ' <<std::setw(2) <<d
               <<' ' <<std::setw(40) <<task->name
               <<" k=" <<task->feat->order <<" ot=" <<task->type <<" prec=" <<std::setw(4) <<task->prec(t);
          if(task->target.N<5) cout <<" y*=[" <<task->target <<']'; else cout<<"y*=[..]";
          cout <<" y^2=" <<err(t, i) <<endl;
        }
      }
    }
  }
  CHECK_EQ(M, phi.N, "");

  //-- generate a report graph
  Graph report;
  double totalC=0., totalG=0.;
  for(uint i=0; i<tasks.N; i++) {
    Task* c = tasks(i);
    Graph* g = &report.newSubgraph({c->name}, {})->value;
    g->newNode<double>({"order"}, {}, c->feat->order);
    g->newNode<rai::String>({"type"}, {}, STRING(ObjectiveTypeString[c->type]));
    g->newNode<double>({"sqrCosts"}, {}, taskC(i));
    g->newNode<double>({"constraints"}, {}, taskG(i));
    totalC += taskC(i);
    totalG += taskG(i);
  }
  report.newNode<double>({"total", "sqrCosts"}, {}, totalC);
  report.newNode<double>({"total", "constraints"}, {}, totalG);

  //-- write a nice gnuplot file
  ofstream fil("z.costReport");
  //first line: legend
  for(auto c:tasks) fil <<c->name <<' ';
  for(auto c:tasks) if(c->type==OT_ineq && dualSolution.N) fil <<c->name <<"_dual ";
  fil <<endl;

  //rest: just the matrix
  if(!dualSolution.N) {
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
  for(uint i=1; i<=tasks.N; i++) fil2 <<(i>1?"  ,''":"     ") <<" u 0:"<<i<<" w l lw 3 lc " <<i <<" lt " <<1-((i/10)%2) <<" \\" <<endl;
  if(dualSolution.N) for(uint i=0; i<tasks.N; i++) fil2 <<"  ,'' u 0:"<<1+tasks.N+i<<" w l \\" <<endl;
  fil2 <<endl;
  fil2.close();

  if(gnuplt) {
    cout <<"KOMO Report\n" <<report <<endl;
    gnuplot("load 'z.costReport.plt'");
  }

  return report;
}

arr KOMO::getInitialization() {
  if(!configurations.N) setupConfigurations();
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");
  arr x;
  for(uint t=0; t<T; t++) x.append(configurations(t+k_order)->getJointState());
  return x;
}

void KOMO::inverseKinematics(arr& y, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x) {
  CHECK(!T, "");
  y.clear();
  if(!!J) J.clear();
  if(!!H) H.clear();
  if(!!tt) tt.clear();
  arrA Ja, Ha;
  komo_problem.phi(y, (!!J?Ja:NoArrA), NoArrA, tt, x);
  if(!!J) J = Ja.scalar();
//  set_x(x);
//  phi_t(y, J, tt, 0);
}

void KOMO::Conv_MotionProblem_KOMO_Problem::getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes) {
  variableDimensions.resize(MP.T);
  for(uint t=0; t<MP.T; t++) variableDimensions(t) = MP.configurations(t+MP.k_order)->getJointStateDimension();

  featureTimes.clear();
  featureTypes.clear();
  for(uint t=0; t<MP.T; t++) {
    for(Task* task: MP.tasks) if(task->prec.N>t && task->prec(t)) {
//      CHECK_LE(task->prec.N, MP.T,"");
        uint m = task->feat->dim_phi(MP.configurations({t, t+MP.k_order}), t); //dimensionality of this task
        featureTimes.append(consts<uint>(t, m));
        featureTypes.append(consts<ObjectiveType>(task->type, m));
      }
  }
  dimPhi = featureTimes.N;
}

void KOMO::Conv_MotionProblem_KOMO_Problem::phi(arr& phi, arrA& J, arrA& H, ObjectiveTypeA& tt, const arr& x) {
  //-- set the trajectory
  MP.set_x(x);

  CHECK(dimPhi, "getStructure must be called first");
  phi.resize(dimPhi);
  if(!!tt) tt.resize(dimPhi);
  if(!!J) J.resize(dimPhi);

  arr y, Jy;
  uint M=0;
  for(uint t=0; t<MP.T; t++) {
    for(Task* task: MP.tasks) if(task->prec.N>t && task->prec(t)) {
        task->feat->phi(y, Jy, MP.configurations({t, t+MP.k_order}), MP.tau, t);
        if(!y.N) continue;
        if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

        //linear transform (target shift)
        if(task->target.N==1) y -= task->target.elem(0);
        else if(task->target.nd==1) y -= task->target;
        else if(task->target.nd==2) y -= task->target[t];
        y *= sqrt(task->prec(t));

        //write into phi and J
        phi.setVectorBlock(y, M);
        if(!!J) {
          Jy *= sqrt(task->prec(t));
          if(t<MP.k_order) Jy.delColumns(0, (MP.k_order-t)*MP.configurations(0)->q.N); //delete the columns that correspond to the prefix!!
          for(uint i=0; i<y.N; i++) J(M+i) = Jy[i]; //copy it to J(M+i); which is the Jacobian of the M+i'th feature w.r.t. its variables
        }
        if(!!tt) for(uint i=0; i<y.N; i++) tt(M+i) = task->type;

        //counter for features phi
        M += y.N;
      }
  }

  CHECK_EQ(M, dimPhi, "");
  MP.featureValues = ARRAY<arr>(phi);
  if(!!tt) MP.featureTypes = ARRAY<ObjectiveTypeA>(tt);
}

//===========================================================================

arr getH_rate_diag(const rai::Configuration& world) {
  //transition cost metric
  arr W_diag;
  if(rai::checkParameter<arr>("Wdiag")) {
    W_diag = rai::getParameter<arr>("Wdiag");
  } else {
    W_diag = world.naturalQmetric();
  }
  return rai::getParameter<double>("Hrate", 1.)*W_diag;
}

void sineProfile(arr& q, const arr& q0, const arr& qT, uint T) {
  q.resize(T+1, q0.N);
  for(uint t=0; t<=T; t++) q[t] = q0 + .5 * (1.-cos(RAI_PI*t/T)) * (qT-q0);
}

arr reversePath(const arr& q) {
  uint T=q.d0-1;
  arr r(T+1, q.d1);
  for(uint t=0; t<=T; t++) r[T-t] = q[t];
  return r;
}

void getVel(arr& v, const arr& q, double tau) {
  uint T=q.d0-1;
  v.resizeAs(q);
  for(uint t=1; t<T; t++)  v[t] = (q[t+1] - q[t-1])/(2.*tau);
  v[0] = (q[1] - q[0])/tau;
  v[T] = (q[T] - q[T-1])/tau;
}

void getAcc(arr& a, const arr& q, double tau) {
  uint T=q.d0-1;
  a.resizeAs(q);
  for(uint t=1; t<T; t++)  a[t] = (q[t+1] + q[t-1] - 2.*q[t])/(tau*tau);
  a[0] = a[1]/2.;
  a[T] = a[T-1]/2.;
}

RUN_ON_INIT_BEGIN(motion)
rai::Array<rai::Configuration*>::memMove=true;
RUN_ON_INIT_END(motion)
