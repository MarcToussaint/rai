/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "taskControl.h"
#include "../Kin/kin_swift.h"
#include "../KOMO/komo.h"
#include "../Kin/frame.h"
#include "../Kin/taskMaps.h"
#include "../Core/graph.h"

//===========================================================================

CT_Status CtrlReference_Const::update(arr& yRef, arr& ydotRef, double tau, const arr& y, const arr& ydot) {
  if(flipTargetSignOnNegScalarProduct && scalarProduct(y_target, y) < 0) {
    y_target = -y_target;
  }
  yRef = y_target;
  ydotRef = zeros(y.N);
  return CT_running;
}

//===========================================================================

CT_Status CtrlReference_Sine::update(arr& yRef, arr& ydotRef, double tau, const arr& y, const arr& ydot) {
  t+=tau;
  if(t>T) t=T;
  if(y_start.N!=y.N) y_start=y; //initialization
  if(y_target.N!=y.N) y_target = zeros(y.N);
  yRef = y_start + (.5*(1.-cos(RAI_PI*t/T))) * (y_target - y_start);
  ydotRef = zeros(y.N);
  y_err = yRef - y;
  if(t>=T-1e-6/* && length(y_err)<1e-3*/) return CT_done;
  return CT_running;
}

bool CtrlReference_Sine::isDone() {
  NIY;
  return t>=T && length(y_err)<1e-3;
}

//===========================================================================

CtrlReference_PD::CtrlReference_PD()
  : kp(0.), kd(0.), maxVel(0.), maxAcc(0.), flipTargetSignOnNegScalarProduct(false), makeTargetModulo2PI(false), tolerance(1e-3) {}

CtrlReference_PD::CtrlReference_PD(const arr& _y_target, double decayTime, double dampingRatio, double maxVel, double maxAcc)
  : CtrlReference_PD() {
  y_target = _y_target;
  //    CHECK(decayTime>0. && dampingRatio>0., "this does not define proper gains!");
  //    double lambda = -decayTime*dampingRatio/log(.1);
  //    kp = rai::sqr(1./lambda);
  //    kd = 2.*dampingRatio/lambda;
  setGainsAsNatural(decayTime, dampingRatio);
}

CtrlReference_PD::CtrlReference_PD(const Graph& params)
  : CtrlReference_PD() {
  Node* it;
  if((it=params["PD"])) {
    arr pd=it->get<arr>();
    setGainsAsNatural(pd(0), pd(1));
    maxVel = pd(2);
    maxAcc = pd(3);
  }
  if((it=params["target"])) y_ref = it->get<arr>();
}

void CtrlReference_PD::setTarget(const arr& ytarget, const arr& vtarget) {
  y_target = ytarget;
  if(!!vtarget) v_target=vtarget; else v_target.resizeAs(y_target).setZero();
  y_ref.clear(); v_ref.clear(); //resets the current reference
}

void CtrlReference_PD::setGains(double _kp, double _kd) {
  kp = _kp;
  kd = _kd;
}

void CtrlReference_PD::setGainsAsNatural(double decayTime, double dampingRatio) {
  CHECK(decayTime>0. && dampingRatio>0., "this does not define proper gains!");
  double lambda = -decayTime*dampingRatio/log(.1);
  setGains(rai::sqr(1./lambda), 2.*dampingRatio/lambda);
}

CT_Status CtrlReference_PD::update(arr& yRef, arr& vRef, double tau, const arr& y, const arr& ydot) {
  //only on initialization the true state is used; otherwise ignored!
  if(y_ref.N!=y.N) { y_ref=y; v_ref=ydot; }
  if(y_target.N!=y_ref.N) { y_target=y_ref; v_target=v_ref; }

  if(flipTargetSignOnNegScalarProduct && scalarProduct(y_target, y_ref) < 0) {
    y_target = -y_target;
  }
  if(makeTargetModulo2PI) for(uint i=0; i<y_ref.N; i++) {
      while(y_target(i) < y_ref(i)-RAI_PI) y_target(i)+=RAI_2PI;
      while(y_target(i) > y_ref(i)+RAI_PI) y_target(i)-=RAI_2PI;
    }

  arr a = getDesiredAcceleration();

  y_ref += tau*v_ref + (.5*tau*tau)*a;
  v_ref += tau*a;

  yRef = y_ref;
  vRef = v_ref;

  if(isConverged(-1.)) return CT_conv;
  return CT_running;
}

arr CtrlReference_PD::getDesiredAcceleration() {
  arr a = kp*(y_target-y_ref) + kd*(v_target-v_ref);

  //check vel/acc limits
  double accNorm = length(a);
  if(accNorm>1e-4) {
    if(maxAcc>0. && accNorm>maxAcc) a *= maxAcc/accNorm;
    if(maxVel>0.) {
      double velRatio = scalarProduct(v_ref, a/accNorm)/maxVel;
      if(velRatio>1.) a.setZero();
      else if(velRatio>.9) a *= 1.-10.*(velRatio-.9);
    }
  }
  return a;
}

//arr CtrlReference_PD::getDesiredAcceleration(){
//  arr Kp_y = Kp;
//  arr Kd_y = Kd;
//  makeGainsMatrices(Kp_y, Kd_y, y.N);
//  arr a = Kp_y*(get_y_ref()-y) + Kd_y*(get_ydot_ref()-v);

//  //check vel/acc limits
//  double accNorm = length(a);
//  if(accNorm<1e-4) return a;
//  if(maxAcc>0. && accNorm>maxAcc) a *= maxAcc/accNorm;
//  if(!maxVel) return a;
//  double velRatio = scalarProduct(v, a/accNorm)/maxVel;
//  if(velRatio>1.) a.setZero();
//  else if(velRatio>.9) a *= 1.-10.*(velRatio-.9);
//  return a;
//}

void CtrlReference_PD::getDesiredLinAccLaw(arr& Kp_y, arr& Kd_y, arr& a0_y) {
  //this one doesn't depend on the current state...
  Kp_y = diag(kp, y_ref.N);
  Kd_y = diag(kd, y_ref.N);
  a0_y = Kp_y*y_target + Kd_y*v_target;
//  arr a = a0_y - Kp_y*y - Kd_y*v; //linear law
}

double CtrlReference_PD::error() {
  if(!(y_ref.N && y_ref.N==y_target.N && v_ref.N==v_target.N)) return -1.;
  return maxDiff(y_ref, y_target) + maxDiff(v_ref, v_target);
}

bool CtrlReference_PD::isConverged(double _tolerance) {
  if(_tolerance<0.) _tolerance=tolerance;
  return (y_ref.N && y_ref.N==y_target.N && v_ref.N==v_target.N
          && maxDiff(y_ref, y_target)<_tolerance
          && maxDiff(v_ref, v_target)<_tolerance); //TODO what if Kp = 0, then it should not count?!?
}

//===========================================================================

CtrlReference_Path::CtrlReference_Path(const arr& path, double executionTime) : executionTime(executionTime), phase(0.) {
  CHECK_EQ(path.nd, 2, "need a properly shaped path!");
  spline.points = path;
  spline.setUniformNonperiodicBasis();
}

CT_Status CtrlReference_Path::update(arr& yRef, arr& ydotRef, double tau, const arr& y, const arr& ydot) {
  phase += tau/executionTime;
  if(phase > 1.) phase=1.;
  yRef    = spline.eval(phase);
  ydotRef = spline.eval(phase, 1)/executionTime;
  if(phase>=1.) return CT_done;
  return CT_running;
}

//===========================================================================

CtrlObjective::CtrlObjective(const char* name, Feature* map)
  : map(map), name(name), active(true), status(CT_init), ref(nullptr), prec(ARR(1.)), hierarchy(1) {
  //  ref = new CtrlReference_PD();
}

CtrlObjective::CtrlObjective(const char* name, Feature* map, double decayTime, double dampingRatio, double maxVel, double maxAcc)
  : CtrlObjective(name, map) {
  ref = new CtrlReference_PD({}, decayTime, dampingRatio, maxVel, maxAcc);
}

CtrlObjective::CtrlObjective(const char* name, Feature* map, const Graph& params)
  : CtrlObjective(name, map) {
  ref = new CtrlReference_PD(params);
  Node* n;
  if((n=params["prec"])) prec = n->get<arr>();
}

CtrlObjective::~CtrlObjective() {
  if(map) delete map; map=nullptr;
  if(ref) delete ref; ref=nullptr;
}

CT_Status CtrlObjective::update(double tau, const rai::Configuration& world) {
  map->phi(y, J_y, world);
  if(world.qdot.N) v = J_y*world.qdot; else v.resize(y.N).setZero();
  CT_Status s=status;
  if(ref) s = ref->update(y_ref, v_ref, tau, y, v);
  if(s!=status) { //new status
    status=s;
    for(auto& c:callbacks) c(this, status);
  }
  return status;
}

CtrlReference_PD& CtrlObjective::PD() {
  if(!ref) ref = new CtrlReference_PD();
  CtrlReference_PD* pd = dynamic_cast<CtrlReference_PD*>(ref);
  if(!pd) {
    LOG(-1) <<"you've created a non-PD ref for this before, of type " <<typeid(*ref).name();
    delete ref;
    ref = new CtrlReference_PD();
    pd = dynamic_cast<CtrlReference_PD*>(ref);
  }
  return *pd;
}

void CtrlObjective::setRef(CtrlReference* _ref) {
  CHECK(!ref, "ref is already set");
  ref = _ref;
}

void CtrlObjective::setTarget(const arr& y_target) {
  CHECK(ref, "need a ref to set target");
  ref->setTarget(y_target);
  ref->resetState();
}

arr CtrlObjective::getPrec() {
  uint n=y_ref.N;
  if(prec.N==1) return diag(rai::sqr(prec.scalar()), n);
  if(prec.nd==1) return diag(prec%prec);
  return comp_At_A(prec);
}

void CtrlObjective::getForceControlCoeffs(arr& f_des, arr& u_bias, arr& K_I, arr& J_ft_inv, const rai::Configuration& world) {
  //-- get necessary Jacobians
  TM_Default* m = dynamic_cast<TM_Default*>(map);
  CHECK(m, "this only works for the default position task map");
  CHECK_EQ(m->type, TMT_pos, "this only works for the default positioni task map");
  CHECK_GE(m->i, 0, "this only works for the default position task map");
  rai::Frame* body = world.frames(m->i);
  rai::Frame* l_ft_sensor = world.getFrameByName("l_ft_sensor");
  arr J_ft, J;
  world.kinematicsPos(NoArr, J,   body, m->ivec);
  world.kinematicsPos_wrtFrame(NoArr, J_ft, body, m->ivec, l_ft_sensor);

  //-- compute the control coefficients
  u_bias = ~J*f_ref;
  f_des = f_ref;
  J_ft_inv = inverse_SymPosDef(J_ft*~J_ft)*J_ft;
  K_I = f_alpha*~J;
}

void CtrlObjective::reportState(ostream& os) {
  os <<"  CtrlObjective " <<name;
  if(!active) cout <<" INACTIVE";
  if(y_ref.N==y.N && v_ref.N==v.N) {
    os <<": "/*y_target=" <<PD().y_target */<<" \ty_ref=" <<y_ref <<" \ty=" <<y
       <<"  y-err=" <<length(y_ref-y)
       <<"  v-err=" <<length(v_ref-v)
       <<endl;
  } else {
    os <<" -- y_ref.N!=y.N or ydot_ref.N!=v.N -- not initialized? -- " <<endl;
  }
}

//===========================================================================

TaskControlMethods::TaskControlMethods(const rai::Configuration& world)
  : Hmetric(world.getHmetric()), qNullCostRef("qNullPD", new F_qItself()) {
//  qNullCostRef.PD().setGains(0.,1.);
//  qNullCostRef.prec = ::sqrt(rai::getParameter<double>("Hrate", .1)*Hmetric);
//  qNullCostRef.PD().setTarget(world.q);
}

void TaskControlMethods::updateCtrlObjectives(double tau, const rai::Configuration& world) {
  qNullCostRef.update(tau, world);
  for(CtrlObjective* t: tasks) t->update(tau, world);
}

void TaskControlMethods::resetCtrlObjectivesState() {
  qNullCostRef.resetState();
  for(CtrlObjective* t: tasks) t->resetState();
}

CtrlObjective* TaskControlMethods::addPDTask(const char* name, double decayTime, double dampingRatio, Feature* map) {
  return tasks.append(new CtrlObjective(name, map, decayTime, dampingRatio, 1., 1.));
}

//CtrlObjective* TaskControlMethods::addPDTask(const char* name,
//                                         double decayTime, double dampingRatio,
//                                         TM_DefaultType type,
//                                         const char* iShapeName, const rai::Vector& ivec,
//                                         const char* jShapeName, const rai::Vector& jvec){
//  return tasks.append(new CtrlObjective(name, new TM_Default(type, world, iShapeName, ivec, jShapeName, jvec),
//                                   decayTime, dampingRatio, 1., 1.));
//}

//ConstraintForceTask* TaskControlMethods::addConstraintForceTask(const char* name, Feature *map){
//  ConstraintForceTask *t = new ConstraintForceTask(map);
//  t->name=name;
//  t->desiredApproach.name=STRING(name <<"_PD");
//  t->desiredApproach.active=false;
//  forceTasks.append(t);
//  tasks.append(&t->desiredApproach);
//  return t;
//}

void TaskControlMethods::lockJointGroup(const char* groupname, rai::Configuration& world, bool lockThem) {
  if(!groupname) {
    if(lockThem) {
      lockJoints = consts<byte>(true, world.q.N);
      world.qdot.setZero();
    } else lockJoints.clear();
    return;
  }
  if(!lockJoints.N) lockJoints = consts<byte>(false, world.q.N);
  rai::Joint* j;
  for(rai::Frame* f : world.frames) if((j=f->joint)) {
      if(f->ats[groupname]) {
        for(uint i=0; i<j->qDim(); i++) {
          lockJoints(j->qIndex+i) = lockThem;
          if(lockThem && world.qdot.N) world.qdot(j->qIndex+i) = 0.;
        }
      }
    }
}

double TaskControlMethods::getIKCosts(const arr& q, const arr& q0, arr& g, arr& H) {
  double c=0.;
  arr y, J;
  if(!!g) { CHECK(&q, ""); g = zeros(q.N); }
  if(!!H) { CHECK(&q, ""); H = zeros(q.N, q.N); }
  for(CtrlObjective* t: tasks) {
    if(t->active && t->ref) {
      y = t->prec%(t->y_ref - t->y);
      J = t->prec%(t->J_y);
      c += sumOfSqr(y);
      if(!!g) g -= 2.*~(~y*J);
      if(!!H) H += 2.*comp_At_A(J);
    }
  }

  if(!!q && !!q0) {
    arr dq = q-q0;
    c += sum(dq%Hmetric%dq);
    if(!!g) g += 2.*dq%Hmetric;
    if(!!H) H += 2.*diag(Hmetric);
  }
  return c;
}

#if 0 //methods taken from TaskControlThread -> could be integrated here
arr q_now = q_model;
double alpha = 1.;
double cost2;
arr dq = alpha*taskController->inverseKinematics(qdot_model, q_now-q_model, &cost);
for(uint k=0; k<10; k++) {
  q_model += dq;
  modelWorld().setJointState(q_model, qdot_model);
  modelWorld().stepSwift();
  taskController->updateCtrlObjectives(.0, modelWorld());
  arr dq2 = alpha*taskController->inverseKinematics(qdot_model, q_now-q_model, &cost2);
  if(cost2<cost) { //accept
    cost=cost2;
    dq=dq2;
    alpha *= 1.2;
    if(alpha>1.) alpha=1.;
  } else { //reject
    q_model -= dq; //undo the step
    dq *= .5;
    alpha *= .5;
    cout <<"reject-" <<std::flush;
  }
  if(absMax(dq)<1e-5*alpha) {
    cout <<"STOP k=" <<k <<endl;
    break;
  }
}
//#endif
if(true || cost>10.) { //calling an optimizer!
  cout <<"HIGH COST IK! " <<cost <<" -> calling newton..." <<std::flush;
  auto f = [this, &q_base](arr& g, arr& H, const arr& x)->double{
    this->modelWorld().setJointState(x);
    this->modelWorld().stepSwift();
    this->taskController->updateCtrlObjectives(0., this->modelWorld());
    double c = this->taskController->getIKCosts(x, q_base, g, H);
    return c;
  };
  q_model = q_base;
  optNewton(q_model, f, OPT(stopTolerance=1e-8, maxStep=1e-1, damping=1e1));
//    checkGradient(f, q_base, 1e-4);
  modelWorld().setJointState(q_model, qdot_model);
  modelWorld().stepSwift();
  taskController->updateCtrlObjectives(0., modelWorld());
  cost = taskController->getIKCosts(q_model, q_base);
  cout <<" cost=" <<cost <<endl;
}
#endif

arr TaskControlMethods::inverseKinematics(arr& qdot, const arr& nullRef, double* cost) {
  arr y, v, J;
  for(CtrlObjective* t: tasks) {
    if(t->active && t->ref) {
      y.append(t->prec%(t->y_ref - t->y));
      J.append(t->prec%(t->J_y));
      if(!!qdot) v.append(t->prec%(t->v_ref));
    }
  }
  if(!y.N) return zeros(Hmetric.d0);
  J.reshape(y.N, J.N/y.N);

  arr Winv = oneover(Hmetric);
  if(lockJoints.N) {
    uint n=Winv.N;
    CHECK_EQ(lockJoints.N, n, "");
    for(uint i=0; i<n; i++) if(lockJoints(i)) Winv(i) = 0.;
  }

  if(J.d1 > Winv.N) Winv.append(1e6, Winv.N-J.d1); //append high costs for joints not represented in Hmetric

  arr Jinv = pseudoInverse(J, Winv, 1e-1);
  checkNan(Jinv);
  checkNan(y);
  if(!!qdot) qdot = Jinv*v;
  arr dq = Jinv*y;
  if(!!nullRef) dq += nullRef - Jinv*(J*nullRef);
  if(cost) {
    *cost = sumOfSqr(y);
    if(!!nullRef) *cost += sum(nullRef%Hmetric%nullRef);
  }
  return dq;
}

arr TaskControlMethods::inverseKinematics_hierarchical() {
  uint maxHierarchy=0;
  uint n=0;
  for(CtrlObjective* t: tasks) if(t->active && t->ref) {
      if(t->hierarchy>maxHierarchy) maxHierarchy=t->hierarchy;
      if(!n) n=t->J_y.d1; else CHECK_EQ(n, t->J_y.d1, "");
    }

  arr Winv = oneover(Hmetric);
  if(lockJoints.N) {
    CHECK_EQ(lockJoints.N, n, "");
    for(uint i=0; i<n; i++) if(lockJoints(i)) Winv(i) = 0.;
  }

  arr dq = zeros(n);
  for(uint h=0; h<=maxHierarchy; h++) { //start with lowest priorities; end with highest
    arr y, J;
    for(CtrlObjective* t: tasks) if(t->active && t->ref && t->hierarchy==h) {
        y.append(t->prec%(t->y_ref - t->y));
        J.append(t->prec%(t->J_y));
      }
    if(!y.N) continue;
    J.reshape(y.N, J.N/y.N);
    arr Jinv = pseudoInverse(J, Winv, 1e2);
    dq = (eye(n) - Jinv*J)*dq; //projection into the null space
    dq += Jinv*y;
  }

  return dq;
}

arr TaskControlMethods::getComplianceProjection() {
  arr P;
  uint count=0;
  for(CtrlObjective* t: tasks) {
    if(t->active && t->complianceDirection.N) {
      if(!P.N) P = eye(t->J_y.d1);

      CHECK(!count, "only implemented for ONE compliance task yet -> subtract more dimensions?");
      CHECK_EQ(t->complianceDirection.N, t->y.N, "compliance direction has wrong dim");
      double factor = length(t->complianceDirection);
      CHECK(factor>0 && factor<=1., "compliance direction needs length in (0,1]");

      arr J = t->J_y;

      if(lockJoints.N) {
        uint n=J.d1;
        CHECK_EQ(lockJoints.N, n, "");
        for(uint i=0; i<n; i++) if(lockJoints(i)) for(uint j=0; j<J.d0; j++) J(j, i) = 0.; //zeroing all locked joint Jacobians;
      }

#if 1
      arr Winv = oneover(Hmetric);
      if(lockJoints.N) {
        uint n=J.d1;
        CHECK_EQ(lockJoints.N, n, "");
        for(uint i=0; i<n; i++) if(lockJoints(i)) Winv(i) = 0.;
      }
      arr Jinv = pseudoInverse(J, Winv, 1e-1);
      arr d = Jinv * t->complianceDirection;
#else
      arr d = ~J * t->complianceDirection;
#endif
      P -= factor*d*~d/(sumOfSqr(d)+1e-6);

      count++;
    }
  }
  return P;
}

void TaskControlMethods::reportCurrentState() {
  cout <<"** TaskControlMethods" <<endl;
  for(CtrlObjective* t: tasks) t->reportState(cout);
}

//void TaskControlMethods::updateConstraintControllers(){
//  arr y;
//  for(ConstraintForceTask* t: forceTasks){
//    if(t->active){
//      t->feat->phi(y, NoArr, world);
//      t->updateConstraintControl(y, t->desiredForce);
//    }
//  }
//}

//arr TaskControlMethods::getDesiredConstraintForces(){
//  arr Jl(world.q.N, 1);
//  Jl.setZero();
//  arr y, J_y;
//  for(ConstraintForceTask* t: forceTasks){
//    if(t->active) {
//      t->feat->phi(y, J_y, world);
//      CHECK_EQ(y.N,1," can only handle 1D constraints for now");
//      Jl += ~J_y * t->desiredForce;
//    }
//  }
//  Jl.reshape(Jl.N);
//  return Jl;
//}

arr TaskControlMethods::operationalSpaceControl() {
  //-- get the stacked task coefficient ($J_\phi$ and $c$ in the reference (they include C^{1/2}))
  arr yddot_des, J;
  for(CtrlObjective* t: tasks) {
    if(t->active && !t->f_ref.N) {
      arr a_des = t->PD().getDesiredAcceleration();
      yddot_des.append(t->prec%(a_des /*-Jdot*qdot*/));
      J.append(t->prec%t->J_y);
    }
  }
  if(yddot_des.N) J.reshape(yddot_des.N, J.N/yddot_des.N);
  if(!yddot_des.N && !qNullCostRef.active) return zeros(J.d1);

  //regularization: null-cost-behavior
  arr A = qNullCostRef.getPrec();
  arr a = zeros(A.d0);
  if(qNullCostRef.active) {
    a += qNullCostRef.getPrec() * qNullCostRef.PD().getDesiredAcceleration();
  }
  //all the tasks
  if(yddot_des.N) {
    A += comp_At_A(J);
    a += comp_At_x(J, yddot_des);
  }
  if(lockJoints.N) {
    CHECK_EQ(lockJoints.N, a.N, "");
    for(uint i=0; i<a.N; i++) if(lockJoints(i)) {
        a(i)=0.;
        for(uint j=0; j<a.N; j++) A(i, j) = A(j, i) = 0.;
        A(i, i)=1.;
      }
  }
  arr q_ddot = lapack_Ainv_b_sym(A, a); // inverse_SymPosDef(A) * a;
  return q_ddot;
}

arr TaskControlMethods::getDesiredLinAccLaw(arr& Kp, arr& Kd, arr& k, const arr& q, const arr& qdot) {
  arr Kp_y, Kd_y, k_y, C_y;
  qNullCostRef.PD().getDesiredLinAccLaw(Kp_y, Kd_y, k_y);
  arr H = qNullCostRef.getPrec();

  Kp = H * Kp_y;
  Kd = H * Kd_y;
  k  = H * k_y;

  arr JCJ = zeros(q.N, q.N);

  for(CtrlObjective* task : tasks) if(task->active) {
      arr J_y;
      task->PD().getDesiredLinAccLaw(Kp_y, Kd_y, k_y);
      C_y = task->getPrec();

      arr JtC_y = ~J_y*C_y;

      JCJ += JtC_y*J_y;

      Kp += JtC_y*Kp_y*J_y;
      Kd += JtC_y*Kd_y*J_y;
      k  += JtC_y*(k_y + Kp_y*(J_y*q - task->y));
    }
  arr invA = inverse_SymPosDef(H + JCJ);

  /*arr E = zeros(4,world.q.N);
  E(0,0) = 1; //Fix Base
  E(1,1) = 1; //Fix Base
  E(2,2) = 1; //Fix Base

  E(3,3) = 1; //Fix Torso

  invA = invA*(eye(world.q.N)-~E*inverse_SymPosDef(E*invA*~E)*E*invA);*/

  Kp = invA*Kp;
  Kd = invA*Kd;
  k  = invA*k;

  return k - Kp*q - Kd*qdot;
}

arr TaskControlMethods::calcOptimalControlProjected(arr& Kp, arr& Kd, arr& u0, const arr& q, const arr& qdot, const arr& M, const arr& F) {
  uint n=F.N;

//  arr q0, q, qDot;
//  world.getJointState(q,qDot);

  arr H = inverse(M); //TODO: Other metrics (have significant influence)

  arr A = ~M*H*M; //TODO: The M matrix is symmetric, isn't it? And also symmetric? Furthermore, if H = M^{-1}, this should be calculated more efficiently
  arr a = zeros(n); //TODO M*eye(world.getJointStateDimension())*5.0*(-qDot);// //TODO: other a possible
  u0 = ~M*H*(a-F);
  arr y, J_y, Kp_y, Kd_y, a0_y;
  arr tempJPrec, tempKp;

//  q0 = q;
  Kp = zeros(n, n);
  Kd = zeros(n, n);
  //TODO: add qitselfPD!!
//  if(qitselfPD.active){
//    a += H_rate_diag % qitselfPD.getDesiredAcceleration(world.q, world.qdot);
//  }
  for(CtrlObjective* law : tasks) if(law->active) {
      tempJPrec = ~J_y*law->getPrec();
      A += tempJPrec*J_y;

      law->PD().getDesiredLinAccLaw(Kp_y, Kd_y, a0_y);

      u0 += tempJPrec*a0_y;

      tempKp = tempJPrec*Kp_y;

      u0 += tempKp*(-y + J_y*q);

      //u0 += ~J*law->getC()*law->getDDotRef(); //TODO: add ydd_ref

      Kp += tempKp*J_y;
      Kd += tempJPrec*Kd_y*J_y;
    }
  arr invA = inverse(A); //TODO: SymPosDef?
  Kp = M*invA*Kp;
  Kd = M*invA*Kd;
  u0 = M*invA*u0 + F;

  return u0 + Kp*q + Kd*qdot;
}

void fwdSimulateControlLaw(arr& Kp, arr& Kd, arr& u0, rai::Configuration& world) {
  arr M, F;
  world.equationOfMotion(M, F, false);

  arr u = u0 - Kp*world.q - Kd*world.qdot;
  arr qdd;
  world.fwdDynamics(qdd, world.qdot, u);

  for(uint tt=0; tt<10; tt++) {
    world.qdot += .001*qdd;
    world.q += .001*world.qdot;
    world.setJointState(world.q, world.qdot);
  }
}

void TaskControlMethods::calcForceControl(arr& K_ft, arr& J_ft_inv, arr& fRef, double& gamma, const rai::Configuration& world) {
  uint nForceTasks=0;
  for(CtrlObjective* task : this->tasks) if(task->active && task->f_ref.N) {
      nForceTasks++;
      TM_Default* map = dynamic_cast<TM_Default*>(task->feat);
      rai::Frame* body = world.frames(map->i);
      rai::Frame* lFtSensor = world.getFrameByName("r_ft_sensor");
      arr y, J, J_ft;
      task->feat->phi(y, J, world);
      world.kinematicsPos_wrtFrame(NoArr, J_ft, body, map->ivec, lFtSensor);
      J_ft_inv = -~conv_vec2arr(map->ivec)*inverse_SymPosDef(J_ft*~J_ft)*J_ft;
      K_ft = -~J*task->f_alpha;
      fRef = task->f_ref;
      gamma = task->f_gamma;
    }

  CHECK_LE(nForceTasks, 1, "Multiple force laws not allowed at the moment");
  if(!nForceTasks) {
    K_ft = zeros(world.getJointStateDimension());
    fRef = ARR(0.0);
    J_ft_inv = zeros(1, 6);
    gamma = 0.0;
  }

}

RUN_ON_INIT_BEGIN(CtrlObjective)
rai::Array<CtrlObjective*>::memMove=true;
RUN_ON_INIT_END(CtrlObjective)
