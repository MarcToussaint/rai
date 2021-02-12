/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "CtrlSolvers.h"
#include "CtrlSolver.h"

#include "../Kin/feature.h"
#include "../Optim/constrained.h"

#if 0

TaskControlMethods::TaskControlMethods(const arr& _Hmetric)
  : Hmetric(_Hmetric) { //rai::getParameter<double>("Hrate", .1)*world.getHmetric()) {
}

CtrlObjective* TaskControlMethods::addPDTask(CtrlObjectiveL& tasks, const char* name, double decayTime, double dampingRatio, ptr<Feature> feat) {
  return tasks.append(new CtrlObjective(name, feat, decayTime, dampingRatio, 1., 1.));
}

//ptr<CtrlObjective> TaskControlMethods::addPDTask(const char* name,
//                                         double decayTime, double dampingRatio,
//                                         TM_DefaultType type,
//                                         const char* iShapeName, const rai::Vector& ivec,
//                                         const char* jShapeName, const rai::Vector& jvec){
//  return tasks.append(new CtrlObjective(name, new TM_Default(type, world, iShapeName, ivec, jShapeName, jvec),
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

double TaskControlMethods::getIKCosts(CtrlObjectiveL& tasks, const arr& q, const arr& q0, arr& g, arr& H) {
  double c=0.;
  arr y, J;
  if(!!g) { CHECK(!!q, ""); g = zeros(q.N); }
  if(!!H) { CHECK(!!q, ""); H = zeros(q.N, q.N); }
  for(CtrlObjective* t: tasks) {
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

arr TaskControlMethods::inverseKinematics_hierarchical(CtrlObjectiveL& tasks) {
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

arr TaskControlMethods::getComplianceProjection(CtrlObjectiveL& tasks) {
  arr P;
  uint count=0;
  for(CtrlObjective* t: tasks) {
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

void TaskControlMethods::reportCurrentState(CtrlObjectiveL& tasks) {
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

arr TaskControlMethods::operationalSpaceControl(CtrlObjectiveL& tasks) {
  //-- get the stacked task coefficient ($J_\phi$ and $c$ in the reference (they include C^{1/2}))
  arr yddot_des, J;
  for(CtrlObjective* t: tasks) {
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

arr TaskControlMethods::getDesiredLinAccLaw(CtrlObjectiveL& tasks, arr& Kp, arr& Kd, arr& k, const arr& q, const arr& qdot) {
  arr Kp_y, Kd_y, k_y, H;
  NIY;
//  qNullCostRef.PD().getDesiredLinAccLaw(Kp_y, Kd_y, k_y);
//  arr H = qNullCostRef.getscale();

  Kp = H * Kp_y;
  Kd = H * Kd_y;
  k  = H * k_y;

  arr JCJ = zeros(q.N, q.N);

  for(CtrlObjective* task : tasks) if(task->active) {
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

arr TaskControlMethods::calcOptimalControlProjected(CtrlObjectiveL& tasks, arr& Kp, arr& Kd, arr& u0, const arr& q, const arr& qdot, const arr& M, const arr& F) {
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

void TaskControlMethods::calcForceControl(CtrlObjectiveL& tasks, arr& K_ft, arr& J_ft_inv, arr& fRef, double& gamma, const rai::Configuration& world) {
  uint nForceTasks=0;
  for(CtrlObjective* task : tasks) if(task->active && task->f_ref.N) {
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

#else

//===========================================================================

TaskControlMethods::TaskControlMethods(const arr& _Hmetric)
  : Hmetric(_Hmetric) { //rai::getParameter<double>("Hrate", .1)*world.getHmetric()) {
}

CtrlObjective* TaskControlMethods::addPDTask(CtrlObjectiveL& tasks, const char* name, double decayTime, double dampingRatio, ptr<Feature> feat) {
  NIY
}

void TaskControlMethods::lockJointGroup(const char* groupname, rai::Configuration& world, bool lockThem) {
  NIY
}

double TaskControlMethods::getIKCosts(CtrlObjectiveL& tasks, const arr& q, const arr& q0, arr& g, arr& H) {
  NIY
}

arr TaskControlMethods::inverseKinematics(const ConfigurationL& Ctuple, CtrlObjectiveL& tasks, arr& qdot, const arr& P_compliance, const arr& nullRef, double* cost) {
  arr y, v, J, J_vel; //separate J only for velocity tasks
  arr t_y, t_J;
  for(auto& t: tasks) {
    if(t->active) {
//      if(t->ref->y_ref.N) {
      t->feat->__phi(t_y, t_J, Ctuple);
      y.append(-t_y);
      J.append(t_J);
//      }
//      if((!!qdot) && t->ref->v_ref.N) {
//        v.append(t->scale*(t->ref->v_ref));
//        J_vel.append(t->scale*(t->J_y));
//      }
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

arr TaskControlMethods::inverseKinematics_hierarchical(CtrlObjectiveL& tasks) {
  NIY
}

arr TaskControlMethods::getComplianceProjection(CtrlObjectiveL& tasks) {
  NIY
}

void TaskControlMethods::reportCurrentState(CtrlObjectiveL& tasks) {
  NIY
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

arr TaskControlMethods::operationalSpaceControl(CtrlObjectiveL& tasks) {
  NIY
}

arr TaskControlMethods::getDesiredLinAccLaw(CtrlObjectiveL& tasks, arr& Kp, arr& Kd, arr& k, const arr& q, const arr& qdot) {
  NIY
}

arr TaskControlMethods::calcOptimalControlProjected(CtrlObjectiveL& tasks, arr& Kp, arr& Kd, arr& u0, const arr& q, const arr& qdot, const arr& M, const arr& F) {
  NIY
}

void fwdSimulateControlLaw(arr& Kp, arr& Kd, arr& u0, rai::Configuration& world) {
  NIY
}

void TaskControlMethods::calcForceControl(CtrlObjectiveL& tasks, arr& K_ft, arr& J_ft_inv, arr& fRef, double& gamma, const rai::Configuration& world) {
  NIY
}

#endif

CtrlProblem_MathematicalProgram::CtrlProblem_MathematicalProgram(CtrlSolver& _CP)
  : CP(_CP) {
  for(uint k=0; k<2; k++) {
    rai::Configuration* C = Ctuple.append(new rai::Configuration());
    C->copy(CP.komo.world, true);
    C->setTaus(CP.tau);
    C->ensure_q();
    C->checkConsistency();
  }
}

uint CtrlProblem_MathematicalProgram::getDimension() {
  return CP.komo.world.getJointStateDimension();
}

void CtrlProblem_MathematicalProgram::getBounds(arr& bounds_lo, arr& bounds_up) {
  arr limits = ~CP.komo.world.getLimits();
  bounds_lo = limits[0];
  bounds_up = limits[1];

  //velocity bounds
  arr q_1 = Ctuple(-2)->getJointState();
  bounds_lo = elemWiseMax(bounds_lo, q_1 - CP.maxVel*CP.tau);
  bounds_up = elemWiseMin(bounds_up, q_1 + CP.maxVel*CP.tau);

  //acceleration bounds
  if(Ctuple.N>=3) {
    arr q_2 = Ctuple(-3)->getJointState();
    bounds_lo = elemWiseMax(bounds_lo, 2.*q_1 - q_2 - (CP.maxAcc*CP.tau*CP.tau));
    bounds_up = elemWiseMin(bounds_up, 2.*q_1 - q_2 + (CP.maxAcc*CP.tau*CP.tau));
  }
}

void CtrlProblem_MathematicalProgram::getFeatureTypes(ObjectiveTypeA& featureTypes) {
  for(auto& o: CP.objectives) if(o->active) {
    uint d = o->feat->__dim_phi(CP.komo.world);
    featureTypes.append(consts<ObjectiveType>(o->type, d));
  }
  dimPhi = featureTypes.N;
}

void CtrlProblem_MathematicalProgram::getNames(StringA& variableNames, StringA& featureNames) {
  variableNames = CP.komo.world.getJointNames();
  for(auto& o: CP.objectives) if(o->active) {
    uint d = o->feat->__dim_phi(CP.komo.world);
    featureNames.append(consts<rai::String>(o->name, d));
  }
}

arr CtrlProblem_MathematicalProgram::getInitializationSample(const arr& previousOptima) {
  NIY;
}

void CtrlProblem_MathematicalProgram::evaluate(arr& phi, arr& J, const arr& x) {
  Ctuple(-1)->setJointState(x);
  Ctuple(-1)->stepSwift();

  if(!dimPhi) {
    ObjectiveTypeA featureTypes;
    getFeatureTypes(featureTypes);
  }
  phi.resize(dimPhi);
  if(!!J) {
    bool SPARSE_JACOBIANS = false;
    if(!SPARSE_JACOBIANS) {
      J.resize(dimPhi, x.N).setZero();
    } else {
      J.sparse().resize(dimPhi, x.N, 0);
    }
  }

  arr y, Jy;
  uint M=0;
  for(auto& ob: CP.objectives) if(ob->active) {
    uintA kdim = getKtupleDim(Ctuple);
    kdim.prepend(0);

    //query the task map and check dimensionalities of returns
    ob->feat->__phi(y, Jy, Ctuple);
    if(!!J) CHECK_EQ(y.N, Jy.d0, "");
    if(!!J) CHECK_EQ(Jy.nd, 2, "");
    if(!!J) CHECK_EQ(Jy.d1, kdim.last(), "");
    if(!y.N) continue;
    if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

    //write into phi and J
    phi.setVectorBlock(y, M);

    if(!!J) {
      if(!isSpecial(Jy)) {
        J.setMatrixBlock(Jy.sub(0, -1, kdim(-2), kdim(-1)-1), M, 0);
      } else {
        Jy.sparse().reshape(J.d0, J.d1);
        Jy.sparse().colShift(M);
        Jy.sparse().rowShift(-kdim(-2));
        J += Jy;
      }
    }

    //counter for features phi
    M += y.N;
  }

  CHECK_EQ(M, dimPhi, "");
  store_phi = phi;
  if(!!J) store_J = J;

//  reportAfterPhiComputation(komo);
}

arr solve_optim(CtrlSolver& CP) {
  auto MP = make_shared<CtrlProblem_MathematicalProgram>(CP);

  arr x = CP.komo.world.getJointState();
  OptOptions opt;
  opt.stopTolerance = 1e-4;
  opt.stopGTolerance = 1e-4;
  opt.stopIters = 10;
//  opt.nonStrictSteps=-1;
  OptConstrained O(x, NoArr, *MP, opt);
  MP->getBounds(O.newton.bounds_lo, O.newton.bounds_up);
  O.run();
  return x;
}
