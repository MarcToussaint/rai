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

void Feature::phi(arr& y, arr& J, const ConfigurationL& G, double tau, int t) {
  CHECK_GE(G.N, order+1, "I need at least " <<order+1 <<" configurations to evaluate");
  uint k=order;
  if(k==0) { // basic case: order=0
    arr J_bar;
    phi(y, J_bar, *G.last(), t);
    if(!!J) {
      J = zeros(G.N, y.N, J_bar.d1);
      J[G.N-1]() = J_bar;
      arr tmp(J);
      tensorPermutation(J, tmp, TUP(1u, 0u, 2u));
      J.reshape(y.N, G.N*J_bar.d1);
    }
    return;
  }
  arrA y_bar, J_bar;
  double tau2=tau*tau, tau3=tau2*tau;
  y_bar.resize(k+1);
  J_bar.resize(k+1);
  //-- read out the task variable from the k+1 configurations
  for(uint i=0; i<=k; i++)
    phi(y_bar(i), J_bar(i), *G(G.N-1-i), t-i);
  if(k==1)  y = (y_bar(0)-y_bar(1))/tau; //penalize velocity
  if(k==2)  y = (y_bar(0)-2.*y_bar(1)+y_bar(2))/tau2; //penalize acceleration
  if(k==3)  y = (y_bar(0)-3.*y_bar(1)+3.*y_bar(2)-y_bar(3))/tau3; //penalize jerk
  if(!!J) {
    J = zeros(G.N, y.N, J_bar(0).d1);
    if(k==1) { J[G.N-1-1]() = -J_bar(1);  J[G.N-1-0]() = J_bar(0);  J/=tau; }
    if(k==2) { J[G.N-1-2]() =  J_bar(2);  J[G.N-1-1]() = -2.*J_bar(1);  J[G.N-1-0]() = J_bar(0);  J/=tau2; }
    if(k==3) { J[G.N-1-3]() = -J_bar(3);  J[G.N-1-2]() =  3.*J_bar(2);  J[G.N-1-1]() = -3.*J_bar(1);  J[G.N-1-0]() = J_bar(0);  J/=tau3; }
    arr tmp(J);
    tensorPermutation(J, tmp, TUP(1u, 0u, 2u));
    J.reshape(y.N, G.N*J_bar(0).d1);
  }
}

//===========================================================================

void Task::setCostSpecs(uint fromTime,
                        uint toTime,
                        const arr& _target,
                        double _prec) {
  if(!!_target) target = _target; else target = {0.};
  CHECK_GE(toTime, fromTime, "");
  prec.resize(toTime+1).setZero();
  for(uint t=fromTime; t<=toTime; t++) prec(t) = _prec;
}

//===========================================================================

Feature* newTaskMap(const Node* specs, const rai::Configuration& world) {
  if(specs->parents.N<2) return nullptr;

  //-- get tags
  rai::String& tt=specs->parents(0)->keys.last();
  rai::String& type=specs->parents(1)->keys.last();
  const char* ref1=nullptr, *ref2=nullptr;
  if(specs->parents.N>2) ref1=specs->parents(2)->keys.last().p;
  if(specs->parents.N>3) ref2=specs->parents(3)->keys.last().p;

  //-- check the term type
  ObjectiveType termType;
  if(tt=="MinSumOfSqr") termType=OT_sos;
  else if(tt=="LowerEqualZero") termType=OT_ineq;
  else if(tt=="EqualZero") termType=OT_eq;
  else return nullptr;

  //-- create a task map
  Feature* map;
  const Graph& params = specs->graph();
//  rai::String type = specs.get<rai::String>("type", "pos");
  if(type=="wheels") {
    map = new F_qItself(world, "worldTranslationRotation");
  } else if(type=="collisionIneq") {
    map = new CollisionConstraint((params?params->get<double>("margin", 0.1):0.1));
  } else if(type=="collisionPairs") {
    uintA shapes;
    for(uint i=2; i<specs->parents.N; i++) {
      rai::Shape* s = world.getShapeByName(specs->parents(i)->keys.last());
      CHECK(s, "No Shape '" <<specs->parents(i)->keys.last() <<"'");
      shapes.append(s->index);
    }
    map = new ProxyConstraint(TMT_pairsP, shapes, (params?params->get<double>("margin", 0.1):0.1));
  } else if(type=="collisionExceptPairs") {
    uintA shapes;
    for(uint i=2; i<specs->parents.N; i++) {
      rai::Shape* s = world.getShapeByName(specs->parents(i)->keys.last());
      CHECK(s, "No Shape '" <<specs->parents(i)->keys.last() <<"'");
      shapes.append(s->index);
    }
    map = new ProxyConstraint(TMT_allExceptPairsP, shapes, (params?params->get<double>("margin", 0.1):0.1));
  } else if(type=="proxy") {
    map = new TM_Proxy(TMT_allP, {0u}, (params?params->get<double>("margin", 0.1):0.1));
  } else if(type=="qItself") {
    if(ref1) map = new F_qItself(world, ref1);
    else if(params && params->getNode("Hmetric")) map = new F_qItself(params->getNode("Hmetric")->get<double>()*world.getHmetric()); //world.naturalQmetric()); //
    else map = new F_qItself();
  } else if(type=="GJK") {
    map = new TM_GJK(world, ref1, ref2, true);
  } else {
    map = new TM_Default(specs, world);
  }
  map->type=termType;

  //-- check additional real-valued parameters: order
  if(specs->isGraph()) {
    const Graph& params = specs->graph();
    map->order = params->get<double>("order", 0);
  }
  return map;
}

//===========================================================================

Task* newTask(const Node* specs, const rai::Configuration& world, uint Tinterval, uint Tzero) {
  //-- try to crate a map
  Feature* map = newTaskMap(specs, world);
  if(!map) return nullptr;
  //-- create a task
  Task* task = new Task(map);
  //-- check for additional continuous parameters
  if(specs->isGraph()) {
    const Graph& params = specs->graph();
    arr time = params->get<arr>("time", {0., 1.});
    task->setCostSpecs(Tzero + time(0)*Tinterval, Tzero + time(1)*Tinterval, params->get<arr>("target", {}), params->get<double>("scale", {1.}));
  } else {
    task->setCostSpecs(Tzero, Tzero+Tinterval, {}, 1.);
  }
  return task;
}

//===========================================================================

rai::KinematicSwitch* newSwitch(const Node* specs, const rai::Configuration& world, uint Tinterval, uint Tzero=0) {
  if(specs->parents.N<2) return nullptr;

  //-- get tags
  rai::String& tt=specs->parents(0)->keys.last();
  rai::String& type=specs->parents(1)->keys.last();
  const char* ref1=nullptr, *ref2=nullptr;
  if(specs->parents.N>2) ref1=specs->parents(2)->keys.last().p;
  if(specs->parents.N>3) ref2=specs->parents(3)->keys.last().p;

  if(tt!="MakeJoint") return nullptr;

  //-- create switch
  rai::KinematicSwitch* sw= new rai::KinematicSwitch();
  if(type=="addRigid") { sw->symbol=rai::SW_effJoint; sw->jointType=rai::JT_rigid; }
//  else if(type=="addRigidRel"){ sw->symbol = rai::addJointAtTo; sw->jointType=rai::JT_rigid; }
  else if(type=="rigid") { sw->symbol = rai::addJointAtTo; sw->jointType=rai::JT_rigid; }
  else if(type=="rigidZero") { sw->symbol = rai::SW_effJoint; sw->jointType=rai::JT_rigid; }
  else if(type=="transXYPhi") { sw->symbol = rai::addJointAtFrom; sw->jointType=rai::JT_transXYPhi; }
  else if(type=="free") { sw->symbol = rai::addJointAtTo; sw->jointType=rai::JT_free; }
  else if(type=="delete") { sw->symbol = rai::deleteJoint; }
  else HALT("unknown type: "<< type);
  sw->fromId = world.getShapeByName(ref1)->index;
  if(!ref2) {
    CHECK_EQ(sw->symbol, rai::deleteJoint, "");
    rai::Body* b = world.shapes(sw->fromId)->body;
    if(b->inLinks.N==1) {
//      CHECK_EQ(b->children.N, 0, "");
      sw->toId = sw->fromId;
      sw->fromId = b->inLinks(0)->from->shapes.first()->index;
    } else if(b->children.N==1) {
      CHECK_EQ(b->inLinks.N, 0, "");
      sw->toId = b->children(0)->from->shapes.first()->index;
    } else if(b->inLinks.N==0 && b->children.N==0) {
      RAI_MSG("No link to delete for shape '" <<ref1 <<"'");
      delete sw;
      return nullptr;
    } else HALT("that's ambiguous");
  } else {
    sw->toId = world.getShapeByName(ref2)->index;
  }
  sw->timeOfApplication = Tzero + Tinterval + 1;
  if(specs->isGraph()) {
    const Graph& params = specs->graph();
    sw->timeOfApplication = Tzero + params->get<double>("time", 1.)*Tinterval + 1;
  }
  return sw;
}

//===========================================================================

KOMO::KOMO(rai::Configuration& _world, bool useSwift)
  : world(_world), useSwift(useSwift), T(0), tau(0.), k_order(2) {
  if(useSwift) {
    makeConvexHulls(world.shapes);
    world.swift().setCutoff(2.*rai::getParameter<double>("swiftCutoff", 0.11));
  }
  world.getJointState(x0, v0);
  if(!v0.N) { v0.resizeAs(x0).setZero(); world.setJointState(x0, v0); }
  setTiming(rai::getParameter<uint>("timeSteps", 50), rai::getParameter<double>("duration", 5.));
}

KOMO& KOMO::operator=(const KOMO& other) {
  world = const_cast<rai::Configuration&>(other.world);
  useSwift = other.useSwift;
  tasks = other.tasks;
  T = other.T;
  tau = other.tau;
  k_order = other.k_order;
  x0 = other.x0;
  v0 = other.v0;
  prefix = other.prefix;
  phiMatrix = other.phiMatrix;
  dualMatrix = other.dualMatrix;
  ttMatrix = other.ttMatrix;
  return *this;
}

void KOMO::setTiming(uint timeSteps, double duration) {
  T = timeSteps;
  if(T) tau = duration/T; else tau=duration;
}

arr KOMO::getH_rate_diag() {
  //transition cost metric
  arr W_diag;
  if(rai::checkParameter<arr>("Wdiag")) {
    W_diag = rai::getParameter<arr>("Wdiag");
  } else {
    W_diag = world.naturalQmetric();
  }
  return rai::getParameter<double>("Hrate", 1.)*W_diag;
}

Task* KOMO::addTask(const char* name, Feature* m) {
  Task* t = new Task(m);
  t->name=name;
  tasks.append(t);
  return t;
}

bool KOMO::parseTask(const Node* n, int Tinterval, uint Tzero) {
  if(Tinterval==-1) Tinterval=T;
  //-- task?
  Task* task = newTask(n, world, Tinterval, Tzero);
  if(task) {
    if(n->keys.N) task->name=n->keys.last(); else {
      for(Node* p:n->parents) task->name <<'_' <<p->keys.last();
    }
    tasks.append(task);
    return true;
  }
  //-- switch?
  rai::KinematicSwitch* sw = newSwitch(n, world, Tinterval, Tzero);
  if(sw) {
    switches.append(sw);
    return true;
  }
  return false;
}

void KOMO::parseTasks(const Graph& specs, int Tinterval, uint Tzero) {
  for(Node* n:specs) parseTask(n, Tinterval, Tzero);

  //-- add TransitionTask for InvKinematics
  if(!T) {
    Feature* map = new F_qItself();
    map->order = 0;
    map->type=OT_sos;
    Task* task = new Task(map);
    task->name="InvKinTransition";
    task->setCostSpecs(0, 0, x0, 1./(tau*tau));
    tasks.append(task);
  }
}

#if 0
void KOMO::setInterpolatingCosts(
  Task* c,
  TaskCostInterpolationType inType,
  const arr& y_finalTarget, double y_finalPrec, const arr& y_midTarget, double y_midPrec, double earlyFraction) {
  uint m=c->feat.dim_phi(world);
  setState(x0, v0);
  arr y0;
  c->feat.phi(y0, NoArr, world);
  arr midTarget=zeros(m), finTarget=zeros(m);
  if(!!y_finalTarget) { if(y_finalTarget.N==1) finTarget = y_finalTarget(0); else finTarget=y_finalTarget; }
  if(!!y_midTarget) {   if(y_midTarget.N==1)   midTarget = y_midTarget(0);   else midTarget=y_midTarget; }
  switch(inType) {
    case constant: {
      c->target = replicate(finTarget, T+1);
      c->prec.resize(T+1) = y_finalPrec;
    } break;
    case finalOnly: {
      c->target.resize(T+1, m).setZero();
      c->target[T]() = finTarget;
      c->prec.resize(T+1).setZero();
      c->prec(T) = y_finalPrec;
    } break;
    case final_restConst: {
      c->target = replicate(midTarget, T+1);
      c->target[T]() = finTarget;
      c->prec.resize(T+1) = y_midPrec<=0. ? 0. : y_midPrec;
      c->prec(T) = y_finalPrec;
    } break;
    case final_restLinInterpolated: {
      c->target.resize(T+1, m).setZero();
      for(uint t=0; t<=T; t++) {
        double a = (double)t/T;
        c->target[t]() = ((double)1.-a)*y0 + a*finTarget;
      }
      c->prec.resize(T+1) = y_midPrec<0. ? y_finalPrec : y_midPrec;
      c->prec(T) = y_finalPrec;
    } break;
    case early_restConst: {
      uint t;
      CHECK(earlyFraction>=0. && earlyFraction<=1., "");
      uint Tearly=earlyFraction*T;
      c->target.resize(T+1, m).setZero();
      for(t=0; t<Tearly; t++) c->target[t]() = midTarget;
      for(t=Tearly; t<=T; t++) c->target[t]() = finTarget;
      c->prec.resize(T+1).setZero();
      for(t=0; t<Tearly; t++) c->prec(t) = y_midPrec<=0. ? 0. : y_midPrec;
      for(t=Tearly; t<=T; t++) c->prec(t) = y_finalPrec;
    } break;
  }
}
#endif

void KOMO::setState(const arr& q, const arr& v) {
  world.setJointState(q, v);
  if(useSwift) world.stepSwift();
}

uint KOMO::dim_phi(const rai::Configuration& G, uint t) {
  uint m=0;
  for(Task* c: tasks) {
    if(c->active && c->prec.N>t && c->prec(t)) m += c->dim_phi(G, t); //counts also constraints
  }
  return m;
}

uint KOMO::dim_g(const rai::Configuration& G, uint t) {
  uint m=0;
  for(Task* c: tasks) {
    if(c->feat.type==OT_ineq && c->active && c->prec.N>t && c->prec(t))  m += c->feat.dim_phi(G);
  }
  return m;
}

uint KOMO::dim_h(const rai::Configuration& G, uint t) {
  uint m=0;
  for(Task* c: tasks) {
    if(c->feat.type==OT_eq && c->active && c->prec.N>t && c->prec(t))  m += c->feat.dim_phi(G);
  }
  return m;
}

void KOMO::setConfigurationStates() {
  //IMPORTANT: The configurations need to include the k prefix configurations!
  //Therefore configurations(0) is for time=-k and configurations(k+t) is for time=t
  if(configurations.N!=k_order+T+1) {
    listDelete(configurations);
    configurations.append(new rai::Configuration())->copy(world, true);
    for(uint t=1; t<=k_order+T; t++) {
      configurations.append(new rai::Configuration())->copy(*configurations(t-1), true);
      CHECK_EQ(configurations(t), configurations.last(), "");
      //apply potential graph switches
      for(rai::KinematicSwitch* sw:switches) {
        if(sw->timeOfApplication==t-k_order) {
          sw->apply(*configurations(t));
//          if(MP.useSwift) configurations(t)->swift().initActivations(*configurations(t));
        }
      }
    }
  }
}

void KOMO::temporallyAlignKinematicSwitchesInConfiguration(uint t) {
  for(rai::KinematicSwitch* sw:switches) if(sw->timeOfApplication<=t) {
      sw->temporallyAlign(*configurations(t+k_order-1), *configurations(t+k_order));
    }
}

void KOMO::displayTrajectory(int steps, const char* tag, double delay) {
  OpenGL gl;

  uint num;
  if(steps==1 || steps==-1) num=T; else num=steps;
  for(uint k=0; k<=(uint)num; k++) {
    uint t = (T?(k*T/num):0);

    gl.clear();
    gl.add(glStandardScene, 0);
    gl.addDrawer(configurations(t+k_order));
    if(delay<0.) {
      if(delay<-10.) FILE("z.graph") <<*configurations(t+k_order);
      gl.watch(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
    } else {
      gl.update(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
      if(delay) rai::wait(delay);
    }
  }
  if(steps==1)
    gl.watch(STRING(tag <<" (time " <<std::setw(3) <<T <<'/' <<T <<')').p);
}

bool KOMO::getPhi(arr& phi, arr& J, ObjectiveTypeA& tt, uint t, const ConfigurationL& G, double tau) {
  phi.clear();
  if(!!tt) tt.clear();
  if(!!J) J.clear();
  arr y, Jy;
  bool ineqHold=true;
  for(Task* c: tasks) if(c->active && c->prec.N>t && c->prec(t)) {
      c->feat.phi(y, Jy, G, tau, t);
      if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);
      //linear transform (target shift)
      if(true) {
        if(c->target.N==1) y -= c->target.elem(0);
        else if(c->target.nd==1) y -= c->target;
        else if(c->target.nd==2) y -= c->target[t];
        y *= sqrt(c->prec(t));
        if(!!J) Jy *= sqrt(c->prec(t));
      }
      phi.append(y);
      if(!!tt) for(uint i=0; i<y.N; i++) tt.append(c->feat.type);
      if(!!J) J.append(Jy);
      if(c->feat.type==OT_ineq && max(y)>0.) ineqHold=false;
    }
  if(!!J) J.reshape(phi.N, G.N*G.last()->getJointStateDimension());

  CHECK_EQ(phi.N, dim_phi(*G.last(), t), "");

  //memorize for report
  if(!phiMatrix.N) phiMatrix.resize(T+1);
  phiMatrix(t) = phi;
  if(!!tt) {
    if(!ttMatrix.N) ttMatrix.resize(T+1);
    ttMatrix(t) = tt;
  }

  return ineqHold;
}

StringA KOMO::getPhiNames(const rai::Configuration& G, uint t) {
  StringA names(dim_phi(G, t));
  uint m=0;
  for(Task* c: tasks) if(c->active && c->prec.N>t && c->prec(t)) {
      if(c->feat.type==OT_sos) {
        uint d = c->dim_phi(G, t); //counts also constraints
        for(uint i=0; i<d; i++) {
          names(m+i)=c->name;
          names(m+i) <<"_f" <<i;
        }
        m+=d;
      }
    }
  for(Task* c: tasks) if(c->active && c->prec.N>t && c->prec(t)) {
      if(c->feat.type==OT_ineq) {
        uint d = c->dim_phi(G, t); //counts also constraints
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

void KOMO::activateAllTaskCosts(bool active) {
  for(Task* c: tasks) c->active=active;
}

void KOMO::reportFeatures(bool brief) {
  cout <<"*** KOMO -- FeatureReport " <<endl;

  cout <<"  useSwift=" <<useSwift <<endl;
  cout <<"  T=" <<T <<endl;
  cout <<"  tau=" <<tau <<endl;
  cout <<"  k_order=" <<k_order <<endl;
  cout <<"  x0=" <<x0 <<endl;
  cout <<"  v0=" <<v0 <<endl;
  cout <<"  prefix=" <<prefix <<endl;
  cout <<"  TASKS (time idx name order type target scale ttMatrix phiMatrix):" <<endl;

  //-- collect all task costs and constraints
  for(uint t=0; t<=T; t++) {
    uint m=0;
    for(uint i=0; i<tasks.N; i++) {
      Task* c = tasks(i);
      uint d=c->dim_phi(world, t);
      if(brief) {
        if(d) {
          cout <<"  " <<t <<' ' <<d
               <<' ' <<std::setw(10) <<c->name
               <<' ' <<c->feat.order <<' ' <<c->feat.type <<' ';
          cout <<"xx";
          cout <<' ' <<c->prec(t);
          if(ttMatrix.N) {
            cout <<' ' <<ttMatrix(t).elem(m)
                 <<' ' <<sumOfSqr(phiMatrix(t)({m, m+d-1}));
          }
          cout <<endl;
        }
      } else {
        for(uint i=0; i<d; i++) {
          cout <<"  " <<t <<' ' <<i
               <<' ' <<std::setw(10) <<c->name
               <<' ' <<c->feat.order <<' ' <<c->feat.type <<' ';
          if(c->target.N==1) cout <<c->target.elem(0);
          else if(c->target.nd==1) cout <<c->target(i);
          else if(c->target.nd==2) cout <<c->target(t, i);
          else cout <<"00";
          cout <<' ' <<c->prec(t);
          if(ttMatrix.N) {
            cout <<' ' <<ttMatrix(t)(m+i)
                 <<' ' <<phiMatrix(t)(m+i);
          }
          cout <<endl;
        }
      }
      m += d;
    }
    if(phiMatrix.N) CHECK_EQ(m, phiMatrix(t).N, "");
  }

  cout <<"  SWITCHES: " <<switches.N <<endl;
  for(rai::KinematicSwitch* sw:switches) {
    cout <<*sw <<endl;
  }

}

void KOMO::costReport(bool gnuplt) {
  cout <<"*** KOMO -- CostReport" <<endl;
  if(phiMatrix.N!=T+1) {
    CHECK_EQ(phiMatrix.N, 0, "");
    phiMatrix.resize(T+1);
  }

  arr plotData(T+1, tasks.N); plotData.setZero();

  //-- collect all task costs and constraints
  double a;
  arr taskC(tasks.N); taskC.setZero();
  arr taskG(tasks.N); taskG.setZero();
  for(uint t=0; t<=T; t++) {
    uint m=0;
    for(uint i=0; i<tasks.N; i++) {
      Task* c = tasks(i);
      uint d=c->dim_phi(world, t);
      if(ttMatrix.N) for(uint i=0; i<d; i++) CHECK_EQ(ttMatrix(t)(m+i), c->feat.type, "");
      if(d) {
        if(c->feat.type==OT_sos) {
          taskC(i) += a = sumOfSqr(phiMatrix(t).sub(m, m+d-1));
          plotData(t, i) = a;
        }
        if(c->feat.type==OT_ineq) {
          double gpos=0., gall=0.;
          for(uint j=0; j<d; j++) {
            double g=phiMatrix(t)(m+j);
            if(g>0.) gpos+=g;
            gall += g;
          }
          taskG(i) += gpos;
          plotData(t, i) = gall;
        }
        if(c->feat.type==OT_eq) {
          double gpos=0., gall=0.;
          for(uint j=0; j<d; j++) {
            double h=phiMatrix(t)(m+j);
            gpos+=fabs(h);
            gall += h;
          }
          taskG(i) += gpos;
          plotData(t, i) = gall;
        }
        m += d;
      }
    }
    CHECK_EQ(m, phiMatrix(t).N, "");
  }

  //-- generate output
  cout <<" * task costs:" <<endl;
  double totalC=0., totalG=0.;
  for(uint i=0; i<tasks.N; i++) {
    Task* c = tasks(i);
    cout <<"\t '" <<c->name <<"' order=" <<c->feat.order <<" type=" <<c->feat.type;
    cout <<" \tcosts=" <<taskC(i) <<" \tconstraints=" <<taskG(i) <<endl;
    totalC += taskC(i);
    totalG += taskG(i);
  }

  cout <<"\t total task        = " <<totalC <<endl;
  cout <<"\t total constraints = " <<totalG <<endl;

  //-- write a nice gnuplot file
  ofstream fil("z.costReport");
  //first line: legend
  for(auto c:tasks) {
    uint d=c->feat.dim_phi(world);
    fil <<c->name <<'[' <<d <<"] ";
  }
  for(auto c:tasks) {
    if(c->feat.type==OT_ineq && dualMatrix.N) {
      fil <<c->name <<"_dual";
    }
  }
  fil <<endl;
  //rest: just the matrix?
  if(!dualMatrix.N) {
    plotData.write(fil, nullptr, nullptr, "  ");
  } else {
    dualMatrix.reshape(T+1, dualMatrix.N/(T+1));
    catCol(plotData, dualMatrix).write(fil, nullptr, nullptr, "  ");
  }
  fil.close();

  ofstream fil2("z.costReport.plt");
  fil2 <<"set key autotitle columnheader" <<endl;
  fil2 <<"set title 'costReport ( plotting sqrt(costs) )'" <<endl;
  fil2 <<"plot 'z.costReport' \\" <<endl;
  for(uint i=1; i<=tasks.N; i++) fil2 <<(i>1?"  ,''":"     ") <<" u 0:"<<i<<" w l \\" <<endl;
  if(dualMatrix.N) for(uint i=0; i<tasks.N; i++) fil2 <<"  ,'' u 0:"<<1+tasks.N+i<<" w l \\" <<endl;
  fil2 <<endl;
  fil2.close();

  if(gnuplt) gnuplot("load 'z.costReport.plt'");
}

Graph KOMO::getReport() {
  if(phiMatrix.N!=T+1) {
    CHECK_EQ(phiMatrix.N, 0, "");
    phiMatrix.resize(T+1);
  }

  //-- collect all task costs and constraints
  arr taskC(tasks.N); taskC.setZero();
  arr taskG(tasks.N); taskG.setZero();
  for(uint t=0; t<=T; t++) {
    uint m=0;
    for(uint i=0; i<tasks.N; i++) {
      Task* c = tasks(i);
      uint d=c->dim_phi(world, t);
      for(uint i=0; i<d; i++) CHECK_EQ(ttMatrix(t)(m+i), c->feat.type, "");
      if(d) {
        if(c->feat.type==OT_sos) taskC(i) += sumOfSqr(phiMatrix(t).sub(m, m+d-1));
        if(c->feat.type==OT_ineq) {
          for(uint j=0; j<d; j++) {
            double g=phiMatrix(t)(m+j);
            if(g>0.) taskG(i) += g;
          }
        }
        if(c->feat.type==OT_eq) {
          for(uint j=0; j<d; j++) taskG(i) += fabs(phiMatrix(t)(m+j));
        }
        m += d;
      }
    }
    CHECK_EQ(m, phiMatrix(t).N, "");
  }

  Graph report;
  double totalC=0., totalG=0.;
  for(uint i=0; i<tasks.N; i++) {
    Task* c = tasks(i);
    Graph* g = &newSupGraph(report, {c->name}, {})->value;
    g->newNode<double>({"order"}, {}, c->feat.order);
    g->newNode<rai::String>({"type"}, {}, STRING(ObjectiveTypeString[c->feat.type]));
    g->newNode<double>({"sqrCosts"}, {}, taskC(i));
    g->newNode<double>({"constraints"}, {}, taskG(i));
    totalC += taskC(i);
    totalG += taskG(i);
  }
  report.newNode<double>({"total", "sqrCosts"}, {}, totalC);
  report.newNode<double>({"total", "constraints"}, {}, totalG);

  return report;
}

arr KOMO::getInitialization() {
  return replicate(x0, T+1);
}

void KOMO::inverseKinematics(arr& y, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x) {
  CHECK(!T, "");
//  CHECK(!k_order,"");
//  CHECK(!switches.N,"");

  setState(x);
  getPhi(y, J, tt, 0, {&world}, tau);
  if(!!H) H.clear();
//  double h=1./sqrt(tau);
//  y.append(h*(x-x0));
//  if(!!J) J.append(h*eye(x.N));
//  if(!!tt) tt.append(consts(OT_sos, x.N));

//  phiMatrix(0).append(h*(x-x0));
//  ttMatrix(0).append(consts(OT_sos, x.N));
}

//===========================================================================

arr MotionProblemFunction::get_prefix() {
  if(!MP.prefix.N) {
    MP.prefix.resize(get_k(), dim_x());
    for(uint i=0; i<MP.prefix.d0; i++) MP.prefix[i]() = MP.x0;
  }
  CHECK(MP.prefix.d0==get_k() && MP.prefix.d1==dim_x(), "the prefix you set has wrong dimension");
  return MP.prefix;
}

arr MotionProblemFunction::get_postfix() {
  if(!MP.postfix.N) return arr();
  CHECK(MP.postfix.d0==get_k() && MP.postfix.d1==dim_x(), "the postfix you set has wrong dimension");
  return MP.postfix;
}

void MotionProblemFunction::phi_t(arr& phi, arr& J, ObjectiveTypeA& tt, uint t, const arr& x_bar) {
  uint T=get_T(), n=dim_x()+dim_z(), k=get_k();

  //assert some dimensions
  CHECK_EQ(x_bar.d0, k+1, "");
  CHECK_EQ(x_bar.d1, n, "");
  CHECK_LE(t, T, "");

#define NEWCODE
#ifdef NEWCODE
  MP.setConfigurationStates();

  //set states
  for(uint i=0; i<=k; i++) {
    if(x_bar[i]!=MP.configurations(t+i)->q) {
//    if(configurations(t+i)->q.N!=x_bar.d1 || maxDiff(x_bar[i],configurations(t+i)->q)>1e-6){
      MP.configurations(t+i)->setJointState(x_bar[i]);
      if(MP.useSwift) MP.configurations(t+i)->stepSwift();
    }
    if(t+i>=k) MP.temporallyAlignKinematicSwitchesInConfiguration(t+i-k);
  }
#else // old way: have only k+1 configurations and 'move' them on the fly
  //-- manage configurations and set x_bar states
  if(configurations.N!=k+1 || (MP.switches.N && t==0)) {
    listDelete(configurations);
    for(uint i=0; i<=k; i++) configurations.append(new rai::Configuration())->copy(MP.world, true);
  }
#if 0
  //find matches
  if(!MP.switches.N) { //this efficiency gain only works without switches yet...
    uintA match(k+1); match=UINT_MAX;
    boolA used(k+1); used=false;
    uintA unused;
    for(uint i=0; i<=k; i++) for(uint j=0; j<=k; j++) {
        if(!used(j) && x_bar[i]==configurations(j)->q) { //we've found a match
          match(i)=j;
          used(j)=true;
          j=k;
        }
      }
    for(uint i=0; i<=k; i++) if(!used(i)) unused.append(i);
    for(uint i=0; i<=k; i++) if(match(i)==UINT_MAX) match(i)=unused.popFirst();
    configurations.permute(match);
  }
#endif
  //apply potential graph switches
  for(rai::KinematicSwitch* sw:MP.switches) {
    for(uint i=0; i<=k; i++) {
      if(t+i>=k && sw->timeOfApplication==t-k+i) {
        sw->apply(*configurations(i));
        if(MP.useSwift) configurations(i)->swift().initActivations(*configurations(i));
      }
    }
  }
  //set states
  for(uint i=0; i<=k; i++) {
    if(x_bar[i]!=configurations(i)->q) {
      configurations(i)->setJointState(x_bar[i]);
      if(MP.useSwift) configurations(i)->stepSwift();
    }
  }
#endif

  //-- task cost (which are taken w.r.t. x_bar[k])
  arr _phi, _J;
  ObjectiveTypeA _tt;
#ifdef NEWCODE
  MP.getPhi(_phi, _J, (!!tt?_tt:NoTermTypeA), t, MP.configurations({t, t+k}), MP.tau);
#else
  MP.getPhi(_phi, _J, (!!tt?_tt:NoTermTypeA), t, MP.configurations, MP.tau);
#endif
  phi.append(_phi);
  if(!!tt) tt.append(_tt);
  if(!!J)  J.append(_J);
//  if(!!J_z){
//    for(auto& c:configurations) c->setAgent(1);
//    MP.getTaskCosts2(_phi, J_z, t, configurations, MP.tau);
//    for(auto& c:configurations) c->setAgent(0);
//  }

  if(!!tt) CHECK_EQ(tt.N, phi.N, "");
  if(!!J) CHECK_EQ(J.d0, phi.N, "");
//  if(!!J_z) CHECK_EQ(J.d0,phi.N,"");

}

StringA MotionProblemFunction::getPhiNames(uint t) {
  return MP.getPhiNames(*MP.configurations.last(), t);
}

//===========================================================================

void MotionProblem_EndPoseFunction::fv(arr& phi, arr& J, const arr& x) {
  //-- transition costs
  NIY;
//  arr h = MP.H_rate_diag;
//  if(MP.transitionType==KOMO::kinematic){
//    h *= MP.tau/double(MP.T);
//    h=sqrt(h);
//  } else {
//    double D = MP.tau*MP.T;
//    h *= 16./D/D/D;
//    h=sqrt(h);
//  }
//  phi = h%(x-MP.x0);
//  if(!!J) J.setDiag(h);

  //-- task costs
  arr _phi, J_x;
  MP.setState(x, zeros(x.N));
  MP.getPhi(_phi, J_x, NoTermTypeA, MP.T, LIST(MP.world), MP.tau);
  phi.append(_phi);
  if(!!J && _phi.N) {
    J.append(J_x);
  }

  if(absMax(phi)>1e10) {
    RAI_MSG("\nx=" <<x <<"\nphi=" <<phi <<"\nJ=" <<J);
    MP.setState(x, NoArr);
    MP.getPhi(_phi, J_x, NoTermTypeA, MP.T, LIST(MP.world), MP.tau);
  }

  if(!!J) CHECK_EQ(J.d0, phi.N, "");

  //store in CostMatrix
  MP.phiMatrix = phi;
}

//===========================================================================

MotionProblem_EndPoseFunction::MotionProblem_EndPoseFunction(KOMO& _MP)
  : MP(_MP) {
//  ConstrainedProblem::operator=( [this](arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x) -> void {
//    this->Phi(phi, J, H, tt, x);
//  } );
}

//===========================================================================

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
