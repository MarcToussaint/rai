/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "taskControl.h"
#include "../KOMO/komo.h"
#include "../Core/graph.h"
#include "../Kin/frame.h"
#include "../Kin/kin_swift.h"
#include "../Kin/TM_default.h"
#include "../Kin/F_qFeatures.h"

//===========================================================================

void naturalGains(double& Kp, double& Kd, double decayTime, double dampingRatio) {
  CHECK(decayTime>0. && dampingRatio>=0., "this does not define proper gains!");
  double lambda = decayTime*dampingRatio/(-log(.1));
//  double lambda = decayTime/(-log(.1)); //assume the damping ratio always 1. -- just so that setting ratio to zero still gives a reasonable value
  double freq = 1./lambda;
  Kp = freq*freq;
  Kd = 2.*dampingRatio*freq;
}

//===========================================================================

ActStatus MotionProfile_Const::update(arr& yRef, arr& ydotRef, double tau, const arr& y, const arr& ydot) {
  if(y_target.N!=y.N) y_target = y;
  if(flipTargetSignOnNegScalarProduct && scalarProduct(y_target, y) < 0) {
    y_target = -y_target;
  }
  yRef = y_target;
  ydotRef = zeros(y.N);
  return AS_running;
}

//===========================================================================

ActStatus MotionProfile_ConstVel::update(arr& yRef, arr& ydotRef, double tau, const arr& y, const arr& ydot) {
  if(v_target.N!=y.N) v_target = zeros(y.N);
  yRef.clear();
  ydotRef = v_target;
  return AS_running;
}

//===========================================================================

ActStatus MotionProfile_Sine::update(arr& yRef, arr& ydotRef, double tau, const arr& y, const arr& ydot) {
  t+=tau;
  if(t>T) t=T;
  if(y_start.N!=y.N) y_start=y; //initialization
  if(y_target.N!=y.N) y_target = y_start;
  yRef = y_start + (.5*(1.-cos(RAI_PI*t/T))) * (y_target - y_start);
  ydotRef = zeros(y.N);
  y_err = yRef - y;
  if(t>=T-1e-6/* && length(y_err)<1e-3*/) return AS_done;
  return AS_running;
}

void MotionProfile_Sine::setTarget(const arr& ytarget, const arr& vtarget) {
  y_target = ytarget;
  resetState();
}

bool MotionProfile_Sine::isDone() {
  NIY;
  return t>=T && length(y_err)<1e-3;
}

//===========================================================================

MotionProfile_Bang::MotionProfile_Bang(const arr& _y_target, double _maxVel)
  : y_target(_y_target), maxVel(_maxVel), tolerance(1e-3) {
}

void MotionProfile_Bang::setTarget(const arr& ytarget, const arr& vtarget) {
  y_target = ytarget;
  resetState(); //resets the current reference
}

void getAcc_bang(double& x, double& v, double maxVel, double tau) {
  double bang = maxVel/.1;

  if(fabs(x)<.1*maxVel) {
    v=0.;
    x=0.;
    return;
  }

  if(v<0.) {
    x*=-1.; v*=-1;
    getAcc_bang(x, v, maxVel, tau);
    x*=-1.; v*=-1;
    return;
  }

  if(x>0.) {
    v -= tau*bang;
    x += tau*v;
    if(x<0.) { x=0.; v=0.; }
    return;
  }

  double ahit = -0.5*(v*v)/x;

  //accelerate
  if(ahit<bang) {
    v += tau*bang;
    if(v>maxVel) v=maxVel;
    x += tau*v;
    return;
  }

  //deccelerate
  v -= tau*ahit;
  if(v<0.) v=0.;
}

void getVel_bang(double& x, double& v, double maxVel, double tau) {
  double sign=rai::sign(x); //-1=left
  v = -sign*maxVel;
  if((x<0. && x+tau*v > 0.) ||
      (x>0. && x+tau*v < 0.)) {
    v=0.; x=0;
  } else {
    x=x+tau*v;
  }
}

ActStatus MotionProfile_Bang::update(arr& yRef, arr& ydotRef, double tau, const arr& y, const arr& ydot) {
  //only on initialization the true state is used; otherwise ignored!
  if(y_target.N!=y.N) { y_target=y; }

#if 1
  yRef = y - y_target;
  ydotRef = ydot;
  for(uint i=0; i<y.N; i++) {
    getAcc_bang(yRef(i), ydotRef(i), maxVel, tau);
    if(i==2) {
      cout <<y(i) <<' ' <<ydot(i) <<' ' <<ydotRef(i) <<endl;
    }
  }
  yRef += y_target;
#else
  arr yDelta = y - y_target;
  ydotRef.resizeAs(y).setZero();
  for(uint i=0; i<y.N; i++) getVel_bang(yDelta(i), ydotRef(i), maxVel, tau);
  yRef = y_target + yDelta;
#endif

  if(maxDiff(y, y_target)<tolerance
      && absMax(ydot)<tolerance) {
    return AS_converged;
  }
  return AS_running;
}

//===========================================================================

MotionProfile_PD::MotionProfile_PD()
  : kp(0.), kd(0.), maxVel(-1.), maxAcc(-1.), flipTargetSignOnNegScalarProduct(false), makeTargetModulo2PI(false), tolerance(1e-3) {}

MotionProfile_PD::MotionProfile_PD(const arr& _y_target, double decayTime, double dampingRatio, double maxVel, double maxAcc)
  : MotionProfile_PD() {
  y_target = _y_target;
  //    CHECK(decayTime>0. && dampingRatio>0., "this does not define proper gains!");
  //    double lambda = -decayTime*dampingRatio/log(.1);
  //    kp = rai::sqr(1./lambda);
  //    kd = 2.*dampingRatio/lambda;
  setGainsAsNatural(decayTime, dampingRatio);
}

MotionProfile_PD::MotionProfile_PD(const rai::Graph& params)
  : MotionProfile_PD() {
  rai::Node* it;
  if((it=params["PD"])) {
    arr pd=it->get<arr>();
    setGainsAsNatural(pd(0), pd(1));
    maxVel = pd(2);
    maxAcc = pd(3);
  }
  if((it=params["target"])) y_ref = it->get<arr>();
}

void MotionProfile_PD::setTarget(const arr& ytarget, const arr& vtarget) {
  y_target = ytarget;
  if(!!vtarget) v_target=vtarget; else v_target.resizeAs(y_target).setZero();
  resetState(); //resets the current reference
}

void MotionProfile_PD::setGains(double _kp, double _kd) {
  kp = _kp;
  kd = _kd;
}

void MotionProfile_PD::setGainsAsNatural(double decayTime, double dampingRatio) {
  naturalGains(kp, kd, decayTime, dampingRatio);
//  CHECK(decayTime>0. && dampingRatio>0., "this does not define proper gains!");
//  double lambda = -decayTime*dampingRatio/log(.1);
//  setGains(rai::sqr(1./lambda), 2.*dampingRatio/lambda);
}

ActStatus MotionProfile_PD::update(arr& yRef, arr& vRef, double tau, const arr& y, const arr& ydot) {
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

  if(isConverged(-1.)) return AS_converged;
  return AS_running;
}

arr MotionProfile_PD::getDesiredAcceleration() {
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

//arr MotionProfile_PD::getDesiredAcceleration(){
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

void MotionProfile_PD::getDesiredLinAccLaw(arr& Kp_y, arr& Kd_y, arr& a0_y) {
  //this one doesn't depend on the current state...
  Kp_y = diag(kp, y_ref.N);
  Kd_y = diag(kd, y_ref.N);
  a0_y = Kp_y*y_target + Kd_y*v_target;
//  arr a = a0_y - Kp_y*y - Kd_y*v; //linear law
}

double MotionProfile_PD::error() {
  if(!(y_ref.N && y_ref.N==y_target.N && v_ref.N==v_target.N)) return -1.;
  return maxDiff(y_ref, y_target) + maxDiff(v_ref, v_target);
}

bool MotionProfile_PD::isConverged(double _tolerance) {
  if(_tolerance<0.) _tolerance=tolerance;
  return (y_ref.N && y_ref.N==y_target.N && v_ref.N==v_target.N
          && maxDiff(y_ref, y_target)<_tolerance
          && maxDiff(v_ref, v_target)<_tolerance); //TODO what if Kp = 0, then it should not count?!?
}

//===========================================================================

MotionProfile_Path::MotionProfile_Path(const arr& path, double executionTime)
  : endTime(executionTime), time(0.) {
  CHECK_EQ(path.nd, 2, "need a properly shaped path!");
  arr times(path.d0);
  for(uint i=0; i<path.d0; i++) times.elem(i) = endTime*double(i)/double(times.N-1);
  spline.set(2, path, times);
}

MotionProfile_Path::MotionProfile_Path(const arr& path, const arr& times)
  : endTime(times.last()), time(0.) {
  spline.set(2, path, times);
}

ActStatus MotionProfile_Path::update(arr& yRef, arr& ydotRef, double tau, const arr& y, const arr& ydot) {
  time += tau;
  if(time > endTime) time=endTime;
  yRef    = spline.eval(time);
  ydotRef = spline.eval(time, 1);
  if(time>=endTime) return AS_done;
  return AS_running;
}

//===========================================================================

CtrlTask::CtrlTask(const char* name, const ptr<Feature>& _feat)
  : name(name), active(true), feat(_feat), scale(1.), hierarchy(1) {
  status.set() = AS_init;
  //  ref = new MotionProfile_PD();
}

CtrlTask::CtrlTask(const char* name, const ptr<Feature>& _feat, const ptr<MotionProfile>& _ref)
  : CtrlTask(name, _feat) {
  ref = _ref;
  status.set() = AS_init;
}

CtrlTask::CtrlTask(const char* name, const ptr<Feature>& _feat, double maxVel)
  : CtrlTask(name, _feat) {
  ref = make_shared<MotionProfile_Bang>(arr(), maxVel);
}

CtrlTask::CtrlTask(const char* name, const ptr<Feature>& _feat, double decayTime, double dampingRatio, double maxVel, double maxAcc)
  : CtrlTask(name, _feat) {
  if(dampingRatio<0.) {
    ref = make_shared<MotionProfile_Sine>(arr(), decayTime);
  } else {
    ref = make_shared<MotionProfile_PD>(arr(), decayTime, dampingRatio, maxVel, maxAcc);
  }
}

CtrlTask::CtrlTask(const char* name, const ptr<Feature>& _feat, const rai::Graph& params)
  : CtrlTask(name, _feat) {
  ref = make_shared<MotionProfile_PD>(params);
  rai::Node* n;
  if((n=params["scale"])) scale = n->get<double>();
}

CtrlTask::~CtrlTask() {
  if(ctrlTasks) {
    ctrlTasks->set()->removeValue(this, true);
    ctrlTasks=0;
  }
}

ActStatus CtrlTask::update(double tau, const rai::Configuration& world) {
  feat->__phi(y, J_y, world);
  //if(world.qdot.N) v = J_y*world.qdot; else v.resize(y.N).setZero();
  ActStatus s_old = status.get();
  ActStatus s_new = s_old;
  if(ref) s_new = ref->update(y_ref, v_ref, tau, y, v);
  if(s_new!=s_old) status.set()=s_new;
  return s_new;
}

MotionProfile_PD& CtrlTask::PD() {
  if(!ref) ref = make_shared<MotionProfile_PD>();
  ptr<MotionProfile_PD> pd = std::dynamic_pointer_cast<MotionProfile_PD>(ref);
  if(!pd) {
    LOG(-1) <<"you've created a non-PD ref for this before, of type " <<typeid(*ref).name();
    ref = make_shared<MotionProfile_PD>();
    pd = std::dynamic_pointer_cast<MotionProfile_PD>(ref);
  }
  return *pd;
}

void CtrlTask::setRef(ptr<MotionProfile> _ref) {
  CHECK(!ref, "ref is already set");
  ref = _ref;
}

void CtrlTask::setTarget(const arr& y_target) {
  CHECK(ref, "need a ref to set target");
  ref->setTarget(y_target);
  ref->resetState();
}

void CtrlTask::getForceControlCoeffs(arr& f_des, arr& u_bias, arr& K_I, arr& J_ft_inv, const rai::Configuration& world) {
  //-- get necessary Jacobians
  ptr<TM_Default> m = std::dynamic_pointer_cast<TM_Default>(feat);
  CHECK(m, "this only works for the default position task feat");
  CHECK_EQ(m->type, TMT_pos, "this only works for the default positioni task feat");
  CHECK_GE(m->i, 0, "this only works for the default position task feat");
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

void CtrlTask::reportState(ostream& os) {
  os <<"  CtrlTask " <<name;
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

TaskControlMethods::TaskControlMethods(const arr& _Hmetric)
  : Hmetric(_Hmetric) { //rai::getParameter<double>("Hrate", .1)*world.getHmetric()) {
}

CtrlTask* TaskControlMethods::addPDTask(CtrlTaskL& tasks, const char* name, double decayTime, double dampingRatio, ptr<Feature> feat) {
  return tasks.append(new CtrlTask(name, feat, decayTime, dampingRatio, 1., 1.));
}

//ptr<CtrlTask> TaskControlMethods::addPDTask(const char* name,
//                                         double decayTime, double dampingRatio,
//                                         TM_DefaultType type,
//                                         const char* iShapeName, const rai::Vector& ivec,
//                                         const char* jShapeName, const rai::Vector& jvec){
//  return tasks.append(new CtrlTask(name, new TM_Default(type, world, iShapeName, ivec, jShapeName, jvec),
//                                   decayTime, dampingRatio, 1., 1.));
//}

//ConstraintForceTask* TaskControlMethods::addConstraintForceTask(const char* name, Feature *feat){
//  ConstraintForceTask *t = new ConstraintForceTask(feat);
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
    } else lockJoints.clear();
    return;
  }
  if(!lockJoints.N) lockJoints = consts<byte>(false, world.q.N);
  rai::Joint* j;
  for(rai::Frame* f : world.frames) if((j=f->joint)) {
      if(f->ats[groupname]) {
        for(uint i=0; i<j->qDim(); i++) {
          lockJoints(j->qIndex+i) = lockThem;
          //if(lockThem && world.qdot.N) world.qdot(j->qIndex+i) = 0.;
        }
      }
    }
}

double TaskControlMethods::getIKCosts(CtrlTaskL& tasks, const arr& q, const arr& q0, arr& g, arr& H) {
  double c=0.;
  arr y, J;
  if(!!g) { CHECK(!!q, ""); g = zeros(q.N); }
  if(!!H) { CHECK(!!q, ""); H = zeros(q.N, q.N); }
  for(CtrlTask* t: tasks) {
    if(t->active && t->ref) {
      y = t->scale*(t->y_ref - t->y);
      J = t->scale*(t->J_y);
      c += sumOfSqr(y);
      if(!!g) g -= 2.*t->scale*~(~y*J);
      if(!!H) H += 2.*t->scale*comp_At_A(J);
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
  taskController->updateCtrlTasks(.0, modelWorld());
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
    this->taskController->updateCtrlTasks(0., this->modelWorld());
    double c = this->taskController->getIKCosts(x, q_base, g, H);
    return c;
  };
  q_model = q_base;
  optNewton(q_model, f, OPT(stopTolerance=1e-8, maxStep=1e-1, damping=1e1));
//    checkGradient(f, q_base, 1e-4);
  modelWorld().setJointState(q_model, qdot_model);
  modelWorld().stepSwift();
  taskController->updateCtrlTasks(0., modelWorld());
  cost = taskController->getIKCosts(q_model, q_base);
  cout <<" cost=" <<cost <<endl;
}
#endif

arr TaskControlMethods::inverseKinematics(CtrlTaskL& tasks, arr& qdot, const arr& P_compliance, const arr& nullRef, double* cost) {
  arr y, v, J, J_vel; //separate J only for velocity tasks
  for(CtrlTask* t: tasks) {
    if(t->active && t->ref) {
      if(t->y_ref.N) {
        y.append(t->scale*(t->y_ref - t->y));
        J.append(t->scale*(t->J_y));
      }
      if((!!qdot) && t->v_ref.N) {
        v.append(t->scale*(t->v_ref));
        J_vel.append(t->scale*(t->J_y));
      }
    }
  }

  arr Winv = oneover(Hmetric);
  if(lockJoints.N) {
    uint n=Winv.N;
    CHECK_EQ(lockJoints.N, n, "");
    for(uint i=0; i<n; i++) if(lockJoints(i)) Winv(i) = 0.;
  }

  //compute the qdot reference: only velocity tasks, special J_vec Jacobian, and not accounting for compliance
  if(!!qdot) {
    if(v.N) {
      J_vel.reshape(v.N, J_vel.N/v.N);
      qdot = pseudoInverse(J_vel, Winv, 1e-1)*v;
    } else {
      qdot.setZero();
    }
  }

  if(!y.N) return zeros(Hmetric.d0);
  J.reshape(y.N, J.N/y.N);

#if 0
  //integrate compliance in regularization metric
  if(!!P_compliance && P_compliance.N) {
    CHECK_EQ(P_compliance.d0, Winv.d0, "");
    if(Winv.nd==1) {
      Winv = P_compliance * (Winv % P_compliance);
    } else {
      Winv = P_compliance * Winv * P_compliance;
    }
  }
#endif

  if(J.d1 > Winv.N) Winv.append(1e6, Winv.N-J.d1); //append high costs for joints not represented in Hmetric

  arr Jinv = pseudoInverse(J, Winv, 1e-1);
  checkNan(Jinv);
  checkNan(y);
  arr dq = Jinv*y;
  if(!!nullRef) dq += nullRef - Jinv*(J*nullRef);
  if(cost) {
    *cost = sumOfSqr(y);
    if(!!nullRef) *cost += sum(nullRef%Hmetric%nullRef);
  }
  return dq;
}

arr TaskControlMethods::inverseKinematics_hierarchical(CtrlTaskL& tasks) {
  uint maxHierarchy=0;
  uint n=0;
  for(CtrlTask* t: tasks) if(t->active && t->ref) {
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
    for(CtrlTask* t: tasks) if(t->active && t->ref && t->hierarchy==h) {
        y.append(t->scale*(t->y_ref - t->y));
        J.append(t->scale*(t->J_y));
      }
    if(!y.N) continue;
    J.reshape(y.N, J.N/y.N);
    arr Jinv = pseudoInverse(J, Winv, 1e2);
    dq = (eye(n) - Jinv*J)*dq; //projection into the null space
    dq += Jinv*y;
  }

  return dq;
}

arr TaskControlMethods::getComplianceProjection(CtrlTaskL& tasks) {
  arr P;
  uint count=0;
  for(CtrlTask* t: tasks) {
    if(t->active && t->compliance.N) {
      if(!P.N) P = eye(t->J_y.d1);

      //special case! qItself feature!
      if(t->compliance.N==1 && std::dynamic_pointer_cast<F_qItself>(t->feat)) {
        double compliance = t->compliance.scalar();
        CHECK_GE(compliance, 0., "");
        CHECK_LE(compliance, 1., "");
        P = diag(1.-compliance, P.d0);
        return P;
      }

      CHECK(!count, "only implemented for ONE compliance task yet -> subtract more dimensions?");
      CHECK_EQ(t->compliance.N, t->y.N, "compliance direction has wrong dim");
      double factor = length(t->compliance);
      CHECK(factor>0 && factor<=1., "compliance direction needs length in (0,1] (1 means full compliance in this direction)");

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
      arr d = Jinv * t->compliance;
#else
      arr d = ~J * t->compliance;
#endif
      P -= factor*d*~d/(sumOfSqr(d)+1e-6);

      count++;
    }
  }
  return P;
}

void TaskControlMethods::reportCurrentState(CtrlTaskL& tasks) {
  cout <<"** TaskControlMethods" <<endl;
  for(CtrlTask* t: tasks) t->reportState(cout);
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

arr TaskControlMethods::operationalSpaceControl(CtrlTaskL& tasks) {
  //-- get the stacked task coefficient ($J_\phi$ and $c$ in the reference (they include C^{1/2}))
  arr yddot_des, J;
  for(CtrlTask* t: tasks) {
    if(t->active && !t->f_ref.N) {
      arr a_des = t->PD().getDesiredAcceleration();
      yddot_des.append(t->scale*(a_des /*-Jdot*qdot*/));
      J.append(t->scale*t->J_y);
    }
  }
  if(yddot_des.N) J.reshape(yddot_des.N, J.N/yddot_des.N);
  if(!yddot_des.N) return zeros(J.d1);

  //regularization: null-cost-behavior
  arr A = diag(Hmetric);
  arr a = zeros(A.d0);
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

arr TaskControlMethods::getDesiredLinAccLaw(CtrlTaskL& tasks, arr& Kp, arr& Kd, arr& k, const arr& q, const arr& qdot) {
  arr Kp_y, Kd_y, k_y, H;
  NIY;
//  qNullCostRef.PD().getDesiredLinAccLaw(Kp_y, Kd_y, k_y);
//  arr H = qNullCostRef.getscale();

  Kp = H * Kp_y;
  Kd = H * Kd_y;
  k  = H * k_y;

  arr JCJ = zeros(q.N, q.N);

  for(CtrlTask* task : tasks) if(task->active) {
      arr J_y;
      task->PD().getDesiredLinAccLaw(Kp_y, Kd_y, k_y);

      arr JtC_y = ~J_y*task->scale;

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

arr TaskControlMethods::calcOptimalControlProjected(CtrlTaskL& tasks, arr& Kp, arr& Kd, arr& u0, const arr& q, const arr& qdot, const arr& M, const arr& F) {
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
  for(CtrlTask* law : tasks) if(law->active) {
      tempJPrec = ~J_y*law->scale;
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
  arr qDot = zeros(world.q.N); HALT("WARNING: qDot should be maintained outside world!");

  arr M, F;
  world.equationOfMotion(M, F, qDot, false);

  arr u = u0 - Kp*world.q - Kd*qDot;
  arr qdd;
  //  world.fwdDynamics(qdd, qDot, u);
  HALT("why compute M and F, but then call fwd dynamics??");

  for(uint tt=0; tt<10; tt++) {
    qDot += .001*qdd;
    world.q += .001*qDot;
    world.setJointState(world.q);
  }
}

void TaskControlMethods::calcForceControl(CtrlTaskL& tasks, arr& K_ft, arr& J_ft_inv, arr& fRef, double& gamma, const rai::Configuration& world) {
  uint nForceTasks=0;
  for(CtrlTask* task : tasks) if(task->active && task->f_ref.N) {
      nForceTasks++;
      ptr<TM_Default> feat = std::dynamic_pointer_cast<TM_Default>(task->feat);
      rai::Frame* body = world.frames(feat->i);
      rai::Frame* lFtSensor = world.getFrameByName("r_ft_sensor");
      arr y, J, J_ft;
      task->feat->__phi(y, J, world);
      world.kinematicsPos_wrtFrame(NoArr, J_ft, body, feat->ivec, lFtSensor);
      J_ft_inv = -~conv_vec2arr(feat->ivec)*inverse_SymPosDef(J_ft*~J_ft)*J_ft;
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
