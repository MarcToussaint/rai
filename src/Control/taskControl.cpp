/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */
#include "taskControl.h"
#include <Kin/kin_swift.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>

//===========================================================================

void MotionProfile_Sine::update(arr& yRef, arr& ydotRef, double tau, const arr& y, const arr& ydot){
  t+=tau;
  if(y_init.N!=y.N) y_init=y; //initialization
  yRef = y_init + (.5*(1.-cos(MLR_PI*t/T))) * (y_target - y_init);
  ydotRef = zeros(y.N);
}

//===========================================================================

MotionProfile_PD::MotionProfile_PD()
  : kp(0.), kd(0.), maxVel(0.), maxAcc(0.), flipTargetSignOnNegScalarProduct(false), makeTargetModulo2PI(false), tolerance(1e-3){}

MotionProfile_PD::MotionProfile_PD(const arr& _y_target, double decayTime, double dampingRatio, double maxVel, double maxAcc)
  : MotionProfile_PD() {
  y_target = _y_target;
  //    CHECK(decayTime>0. && dampingRatio>0., "this does not define proper gains!");
  //    double lambda = -decayTime*dampingRatio/log(.1);
  //    kp = mlr::sqr(1./lambda);
  //    kd = 2.*dampingRatio/lambda;
  setGainsAsNatural(decayTime, dampingRatio);
}

MotionProfile_PD::MotionProfile_PD(const Graph& params)
  : MotionProfile_PD(){
  Node *it;
  if((it=params["PD"])){
    arr pd=it->get<arr>();
    setGainsAsNatural(pd(0), pd(1));
    maxVel = pd(2);
    maxAcc = pd(3);
  }
  if((it=params["target"])) y_ref = it->get<arr>();
}

void MotionProfile_PD::setTarget(const arr& ytarget, const arr& vtarget){
  y_target = ytarget;
  if(&vtarget) v_target=vtarget; else v_target.resizeAs(y_target).setZero();
  y_ref.clear(); v_ref.clear(); //resets the current reference
}

void MotionProfile_PD::setGains(double _kp, double _kd) {
  kp = _kp;
  kd = _kd;
}

void MotionProfile_PD::setGainsAsNatural(double decayTime, double dampingRatio) {
  CHECK(decayTime>0. && dampingRatio>0., "this does not define proper gains!");
  double lambda = -decayTime*dampingRatio/log(.1);
  setGains(mlr::sqr(1./lambda), 2.*dampingRatio/lambda);
}

void MotionProfile_PD::update(arr& yRef, arr& vRef, double tau, const arr& y, const arr& ydot){
  //only on initialization the true state is used; otherwise ignored!
  if(y_ref.N!=y.N){ y_ref=y; v_ref=ydot; }
//   y_ref=y; v_ref=ydot;//TODO: exactly DONT DO THAT!
  if(y_target.N!=y_ref.N) y_target = zeros(y_ref.N);
  if(v_target.N!=v_ref.N) v_target = zeros(v_ref.N);

  if(flipTargetSignOnNegScalarProduct && scalarProduct(y_target, y_ref) < 0){
    y_target = -y_target;
  }
  if(makeTargetModulo2PI) for(uint i=0;i<y_ref.N;i++){
    while(y_target(i) < y_ref(i)-MLR_PI) y_target(i)+=MLR_2PI;
    while(y_target(i) > y_ref(i)+MLR_PI) y_target(i)-=MLR_2PI;
  }

  arr a = getDesiredAcceleration();

  y_ref += tau*v_ref + (.5*tau*tau)*a;
  v_ref += tau*a;

  yRef = y_ref;
  vRef = v_ref;
}

arr MotionProfile_PD::getDesiredAcceleration(){
  arr a = kp*(y_target-y_ref) + kd*(v_target-v_ref);

  //check vel/acc limits
  double accNorm = length(a);
  if(accNorm>1e-4){
    if(maxAcc>0. && accNorm>maxAcc) a *= maxAcc/accNorm;
    if(maxVel>0.){
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

void MotionProfile_PD::getDesiredLinAccLaw(arr& Kp_y, arr& Kd_y, arr& a0_y){
  //this one doesn't depend on the current state...
  Kp_y = diag(kp, y_ref.N);
  Kd_y = diag(kd, y_ref.N);
  a0_y = Kp_y*y_target + Kd_y*v_target;
//  arr a = a0_y - Kp_y*y - Kd_y*v; //linear law
}

double MotionProfile_PD::error(){
  if(!(y_ref.N && y_ref.N==y_target.N && v_ref.N==v_target.N)) return -1.;
  return maxDiff(y_ref, y_target) + maxDiff(v_ref, v_target);
}

bool MotionProfile_PD::isConverged(double _tolerance){
  if(_tolerance<0.) _tolerance=tolerance;
  return (y_ref.N && y_ref.N==y_target.N && v_ref.N==v_target.N
          && maxDiff(y_ref, y_target)<_tolerance
          && maxDiff(v_ref, v_target)<_tolerance); //TODO what if Kp = 0, then it should not count?!?
}

//===========================================================================

MotionProfile_Path::MotionProfile_Path(const arr& path, double executionTime) : executionTime(executionTime), phase(0.){
  CHECK(path.nd==2,"need a properly shaped path!");
  spline.points = path;
  spline.setUniformNonperiodicBasis();
}

void MotionProfile_Path::update(arr& yRef, arr& ydotRef, double tau, const arr& y, const arr& ydot){
  phase += tau/executionTime;
  if(phase > 1.) phase=1.;
  yRef    = spline.eval(phase);
  ydotRef = spline.eval(phase, 1)/executionTime;
}

//===========================================================================

CtrlTask::CtrlTask(const char* name, TaskMap* map)
  : map(map), name(name), active(true), ref(NULL), prec(ARR(10.)){
  //  ref = new MotionProfile_PD();
}

CtrlTask::CtrlTask(const char* name, TaskMap* map, double decayTime, double dampingRatio, double maxVel, double maxAcc)
  : CtrlTask(name, map){
  ref = new MotionProfile_PD({}, decayTime, dampingRatio, maxVel, maxAcc);
}

CtrlTask::CtrlTask(const char* name, TaskMap* map, const Graph& params)
  : CtrlTask(name, map){
  ref = new MotionProfile_PD(params);
  Node *n;
  if((n=params["prec"])) prec = n->get<arr>();
}

CtrlTask::~CtrlTask(){
  if(map) delete map; map=NULL;
  if(ref) delete ref; ref=NULL;
}

void CtrlTask::update(double tau, const mlr::KinematicWorld& world){
  map->phi(y, J_y, world);
  if(world.qdot.N) v = J_y*world.qdot; else v.resize(y.N).setZero();
  if(ref) ref->update(y_ref, v_ref, tau, y, v);
}

MotionProfile_PD& CtrlTask::PD(){
  if(!ref) ref = new MotionProfile_PD();
  MotionProfile_PD *pd = dynamic_cast<MotionProfile_PD*>(ref);
  CHECK(pd, "");
  return *pd;
}

arr CtrlTask::getPrec(){
  uint n=y_ref.N;
  if(prec.N==1) return diag(mlr::sqr(prec.scalar()), n);
  if(prec.nd==1) return diag(prec%prec);
  return comp_At_A(prec);
}

void CtrlTask::getForceControlCoeffs(arr& f_des, arr& u_bias, arr& K_I, arr& J_ft_inv, const mlr::KinematicWorld& world){
  //-- get necessary Jacobians
  TaskMap_Default *m = dynamic_cast<TaskMap_Default*>(map);
  CHECK(m,"this only works for the default position task map");
  CHECK(m->type==posTMT,"this only works for the default positioni task map");
  CHECK(m->i>=0,"this only works for the default position task map");
  mlr::Body *body = world.shapes(m->i)->body;
  mlr::Vector vec = world.shapes(m->i)->rel*m->ivec;
  mlr::Shape* l_ft_sensor = world.getShapeByName("l_ft_sensor");
  arr J_ft, J;
  world.kinematicsPos         (NoArr, J,   body, vec);
  world.kinematicsPos_wrtFrame(NoArr, J_ft,body, vec, l_ft_sensor);

  //-- compute the control coefficients
  u_bias = ~J*f_ref;
  f_des = f_ref;
  J_ft_inv = inverse_SymPosDef(J_ft*~J_ft)*J_ft;
  K_I = f_alpha*~J;
}


void CtrlTask::reportState(ostream& os){
  os <<"  CtrlTask " <<name;
  if(!active) cout <<" INACTIVE";
  if(y_ref.N==y.N && v_ref.N==v.N){
    os <<":  y_target=" <<PD().y_target <<" \ty_ref=" <<y_ref <<" \ty=" <<y
      <<"  y-err=" <<length(y_ref-y)
     <<"  v-err=" <<length(v_ref-v)
    <<endl;
  }else{
    os <<" -- y_ref.N!=y.N or ydot_ref.N!=v.N -- not initialized? -- " <<endl;
  }
}

//===========================================================================

TaskControlMethods::TaskControlMethods(const mlr::KinematicWorld& world)
  : Hmetric(world.getHmetric()), qNullCostRef("qNullPD", new TaskMap_qItself()) {
  qNullCostRef.PD().setGains(0.,1.);
  qNullCostRef.prec = ::sqrt(mlr::getParameter<double>("Hrate", .1)*Hmetric);
  qNullCostRef.PD().setTarget( world.q );
}

void TaskControlMethods::updateCtrlTasks(double tau, const mlr::KinematicWorld& world){
  qNullCostRef.update(tau, world);
  for(CtrlTask* t: tasks) t->update(tau, world);
}

CtrlTask* TaskControlMethods::addPDTask(const char* name, double decayTime, double dampingRatio, TaskMap *map){
  return tasks.append(new CtrlTask(name, map, decayTime, dampingRatio, 1., 1.));
}

//CtrlTask* TaskControlMethods::addPDTask(const char* name,
//                                         double decayTime, double dampingRatio,
//                                         TaskMap_DefaultType type,
//                                         const char* iShapeName, const mlr::Vector& ivec,
//                                         const char* jShapeName, const mlr::Vector& jvec){
//  return tasks.append(new CtrlTask(name, new TaskMap_Default(type, world, iShapeName, ivec, jShapeName, jvec),
//                                   decayTime, dampingRatio, 1., 1.));
//}

//ConstraintForceTask* TaskControlMethods::addConstraintForceTask(const char* name, TaskMap *map){
//  ConstraintForceTask *t = new ConstraintForceTask(map);
//  t->name=name;
//  t->desiredApproach.name=STRING(name <<"_PD");
//  t->desiredApproach.active=false;
//  forceTasks.append(t);
//  tasks.append(&t->desiredApproach);
//  return t;
//}

void TaskControlMethods::lockJointGroup(const char* groupname, mlr::KinematicWorld& world, bool lockThem){
  if(!groupname){
    if(lockThem){
      lockJoints = consts<bool>(true, world.q.N);
      world.qdot.setZero();
    }else lockJoints.clear();
    return;
  }
  if(!lockJoints.N) lockJoints = consts<bool>(false, world.q.N);
  for(mlr::Joint *j:world.joints){
    if(j->ats.getNode(groupname)){
      for(uint i=0;i<j->qDim();i++){
        lockJoints(j->qIndex+i) = lockThem;
        if(lockThem && world.qdot.N) world.qdot(j->qIndex+i) = 0.;
      }
    }
  }
}

arr TaskControlMethods::inverseKinematics(arr& qdot){
  arr y;
  arr v;
  arr J;
  for(CtrlTask* t: tasks) {
    if(t->active && t->ref) {
      y.append(t->prec%(t->y_ref - t->y));
      J.append(t->prec%(t->J_y));
      if(&qdot) v.append(t->prec%(t->v_ref));
    }
  }
  if(!y.N) return zeros(Hmetric.d0);
  J.reshape(y.N, J.N/y.N);

  arr Winv = oneover(Hmetric);
  if(lockJoints.N){
    uint n=J.d1;
    CHECK_EQ(lockJoints.N, n, "");
    for(uint i=0;i<n;i++) if(lockJoints(i)) Winv(i) = 0.;
  }

  arr Jinv = pseudoInverse(J, Winv, 1e-8);
  if(&qdot) qdot = Jinv*v;
  return Jinv*y;
}

void TaskControlMethods::reportCurrentState(){
  cout <<"** TaskControlMethods" <<endl;
  for(CtrlTask* t: tasks) t->reportState(cout);
}

//void TaskControlMethods::updateConstraintControllers(){
//  arr y;
//  for(ConstraintForceTask* t: forceTasks){
//    if(t->active){
//      t->map->phi(y, NoArr, world);
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
//      t->map->phi(y, J_y, world);
//      CHECK_EQ(y.N,1," can only handle 1D constraints for now");
//      Jl += ~J_y * t->desiredForce;
//    }
//  }
//  Jl.reshape(Jl.N);
//  return Jl;
//}

arr TaskControlMethods::operationalSpaceControl(){
  //-- get the stacked task coefficient ($J_\phi$ and $c$ in the reference (they include C^{1/2}))
  arr yddot_des, J;
  for(CtrlTask* t: tasks) {
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
  if(qNullCostRef.active){
    a += qNullCostRef.getPrec() * qNullCostRef.PD().getDesiredAcceleration();
  }
  //all the tasks
  if(yddot_des.N){
    A += comp_At_A(J);
    a += comp_At_x(J, yddot_des);
  }
  if(lockJoints.N){
    CHECK_EQ(lockJoints.N, a.N, "");
    for(uint i=0;i<a.N;i++) if(lockJoints(i)){
      a(i)=0.;
      for(uint j=0;j<a.N;j++) A(i,j) = A(j,i) = 0.;
      A(i,i)=1.;
    }
  }
  arr q_ddot = lapack_Ainv_b_sym(A,a); // inverse_SymPosDef(A) * a;
  return q_ddot;
}

arr TaskControlMethods::getDesiredLinAccLaw(arr &Kp, arr &Kd, arr &k, const arr& q, const arr& qdot) {
  arr Kp_y, Kd_y, k_y, C_y;
  qNullCostRef.PD().getDesiredLinAccLaw(Kp_y, Kd_y, k_y);
  arr H = qNullCostRef.getPrec();

  Kp = H * Kp_y;
  Kd = H * Kd_y;
  k  = H * k_y;

  arr JCJ = zeros(q.N, q.N);

  for(CtrlTask* task : tasks) if(task->active){
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


arr TaskControlMethods::calcOptimalControlProjected(arr &Kp, arr &Kd, arr &u0, const arr& q, const arr& qdot, const arr& M, const arr& F) {
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
  for(CtrlTask* law : tasks) if(law->active){
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

void fwdSimulateControlLaw(arr& Kp, arr& Kd, arr& u0, mlr::KinematicWorld& world){
  arr M, F;
  world.equationOfMotion(M, F, false);

  arr u = u0 - Kp*world.q - Kd*world.qdot;
  arr qdd;
  world.fwdDynamics(qdd, world.qdot, u);

  for(uint tt=0;tt<10;tt++){
    world.qdot += .001*qdd;
    world.q += .001*world.qdot;
    world.setJointState(world.q, world.qdot);
  }
}

void TaskControlMethods::calcForceControl(arr& K_ft, arr& J_ft_inv, arr& fRef, double& gamma, const mlr::KinematicWorld& world) {
  uint nForceTasks=0;
  for(CtrlTask* task : this->tasks) if(task->active && task->f_ref.N){
    nForceTasks++;
    TaskMap_Default* map = dynamic_cast<TaskMap_Default*>(task->map);
    mlr::Body* body = world.shapes(map->i)->body;
    mlr::Vector vec = world.shapes(map->i)->rel.pos;
    mlr::Shape* lFtSensor = world.getShapeByName("r_ft_sensor");
    arr y, J, J_ft;
    task->map->phi(y, J, world);
    world.kinematicsPos_wrtFrame(NoArr, J_ft, body, vec, lFtSensor);
    J_ft_inv = -~conv_vec2arr(map->ivec)*inverse_SymPosDef(J_ft*~J_ft)*J_ft;
    K_ft = -~J*task->f_alpha;
    fRef = task->f_ref;
    gamma = task->f_gamma;
  }

  CHECK(nForceTasks<=1, "Multiple force laws not allowed at the moment");
  if(!nForceTasks){
    K_ft = zeros(world.getJointStateDimension());
    fRef = ARR(0.0);
    J_ft_inv = zeros(1,6);
    gamma = 0.0;
  }

}

RUN_ON_INIT_BEGIN(CtrlTask)
mlr::Array<CtrlTask*>::memMove=true;
RUN_ON_INIT_END(CtrlTask)
