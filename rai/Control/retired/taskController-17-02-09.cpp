/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "taskControl.h"
#include "../Kin/kin_swift.h"
#include "../KOMO/komo.h"
#include "../Kin/taskMaps.h"

//===========================================================================

CtrlObjective::CtrlObjective(const char* name, Feature* map)
  : map(*map), name(name), active(true), prec(ARR(100.)), maxVel(0.), maxAcc(0.), f_alpha(0.), f_gamma(0.),
    flipTargetSignOnNegScalarProduct(false), makeTargetModulo2PI(false) {
}

CtrlObjective::CtrlObjective(const char* name, Feature* map, double decayTime, double dampingRatio, double maxVel, double maxAcc)
  : map(*map), name(name), active(true), prec(ARR(100.)), maxVel(maxVel), maxAcc(maxAcc), f_alpha(0.), f_gamma(0.),
    flipTargetSignOnNegScalarProduct(false), makeTargetModulo2PI(false) {
  setGainsAsNatural(decayTime, dampingRatio);
}

CtrlObjective::CtrlObjective(const char* name, Feature* map, const Graph& params)
  : map(*map), name(name), active(true), prec(ARR(100.)), maxVel(0.), maxAcc(0.), f_alpha(0.), f_gamma(0.),
    flipTargetSignOnNegScalarProduct(false), makeTargetModulo2PI(false) {
  if(!params["PD"]) setGainsAsNatural(3., .7);
  set(params);
}

void CtrlObjective::set(const Graph& params) {
  Node* it;
  if((it=params["PD"])) {
    arr pd=it->get<arr>();
    setGainsAsNatural(pd(0), pd(1));
    maxVel = pd(2);
    maxAcc = pd(3);
  }
  if((it=params["prec"])) prec = it->get<arr>();
  if((it=params["target"])) y_ref = it->get<arr>();
}

void CtrlObjective::setTarget(const arr& yref, const arr& vref) {
  y_ref = yref;
  if(!!vref) v_ref=vref; else v_ref.resizeAs(y_ref).setZero();
}

void CtrlObjective::setTargetToCurrent() {
  y_ref = y;
}

void CtrlObjective::setGains(const arr& _Kp, const arr& _Kd) {
  //active=true; //TODO
  Kp = _Kp;
  Kd = _Kd;
  if(!prec.N) prec=ARR(100.);
}

void CtrlObjective::setGains(double pgain, double dgain) {
  //active=true; //TODO
  Kp = ARR(pgain);
  Kd = ARR(dgain);
  if(!prec.N) prec=ARR(100.);
}

void CtrlObjective::setGainsAsNatural(double decayTime, double dampingRatio) {
  CHECK(decayTime>0. && dampingRatio>0., "this does not define proper gains!");
  double lambda = -decayTime*dampingRatio/log(.1);
  setGains(rai::sqr(1./lambda), 2.*dampingRatio/lambda);
}

void CtrlObjective::setC(const arr& C) {
  prec = C;
}

void makeGainsMatrices(arr& Kp, arr& Kd, uint n) {
  if(Kp.N==1) Kp = diag(Kp.scalar(), n);
  if(Kd.N==1) Kd = diag(Kd.scalar(), n);
  if(Kp.nd==1) Kp = diag(Kp);
  if(Kd.nd==1) Kd = diag(Kd);
  CHECK(Kp.nd==2 && Kp.d0==n && Kp.d1==n, "");
  CHECK(Kd.nd==2 && Kd.d0==n && Kd.d1==n, "");
}

arr CtrlObjective::get_y_ref() {
  if(y_ref.N!=y.N) {
    if(!y_ref.N) y_ref = zeros(y.N); //by convention: no-references = zero references
    if(y_ref.N==1) y_ref = consts<double>(y_ref.scalar(), y.N); //by convention: scalar references = const vector references
  }
  if(flipTargetSignOnNegScalarProduct && scalarProduct(y, y_ref) < 0)
    y_ref = -y_ref;
  if(makeTargetModulo2PI) for(uint i=0; i<y.N; i++) {
      while(y_ref(i) < y(i)-RAI_PI) y_ref(i)+=RAI_2PI;
      while(y_ref(i) > y(i)+RAI_PI) y_ref(i)-=RAI_2PI;
    }
  return y_ref;
}

arr CtrlObjective::get_ydot_ref() {
  if(v_ref.N!=v.N) {
    if(!v_ref.N) v_ref = zeros(v.N);
    if(v_ref.N==1) v_ref = consts<double>(v_ref.scalar(), v.N);
  }
  return v_ref;
}

arr CtrlObjective::getC() {
  uint n=y_ref.N;
  if(prec.N==1) return diag(prec.scalar(), n);
  if(prec.nd==1) return diag(prec);
  return prec;
}

arr CtrlObjective::getDesiredAcceleration() {
  arr Kp_y = Kp;
  arr Kd_y = Kd;
  makeGainsMatrices(Kp_y, Kd_y, y.N);
  arr a = Kp_y*(get_y_ref()-y) + Kd_y*(get_ydot_ref()-v);

  //check vel/acc limits
  double accNorm = length(a);
  if(accNorm<1e-4) return a;
  if(maxAcc>0. && accNorm>maxAcc) a *= maxAcc/accNorm;
  if(!maxVel) return a;
  double velRatio = scalarProduct(v, a/accNorm)/maxVel;
  if(velRatio>1.) a.setZero();
  else if(velRatio>.9) a *= 1.-10.*(velRatio-.9);
  return a;
}

void CtrlObjective::getDesiredLinAccLaw(arr& Kp_y, arr& Kd_y, arr& a0_y) {
  Kp_y = Kp;
  Kd_y = Kd;
  makeGainsMatrices(Kp_y, Kd_y, y.N);

  arr y_delta = get_y_ref() - y;
  double y_delta_length = length(y_delta);
  if(maxVel && y_delta_length>maxVel)
    y_delta *= maxVel/y_delta_length;

  a0_y = Kp_y*(y+y_delta) + Kd_y*get_ydot_ref();
  arr a = a0_y - Kp_y*y - Kd_y*v; //linear law
  double accNorm = length(a);

  return;

  //check vel limit -> change a0, no change in gains
  if(maxVel) {
    double velRatio = scalarProduct(v, a/accNorm)/maxVel;
    if(velRatio>1.) a0_y -= a; //a becomes zero
    else if(velRatio>.9) a0_y -= a*(10.*(velRatio-.9));
  }

  //check acc limits -> change all
  if(maxAcc>1e-4 && accNorm>maxAcc) {
    double scale = maxAcc/accNorm;
    a0_y *= scale;
    Kp_y *= scale;
    Kd_y *= scale;
  }
}

void CtrlObjective::getForceControlCoeffs(arr& f_des, arr& u_bias, arr& K_I, arr& J_ft_inv, const rai::Configuration& world) {
  //-- get necessary Jacobians
  TM_Default* m = dynamic_cast<TM_Default*>(&map);
  CHECK(m, "this only works for the default position task map");
  CHECK_EQ(m->type, TMT_pos, "this only works for the default positioni task map");
  CHECK_GE(m->i, 0, "this only works for the default position task map");
  rai::Body* body = world.shapes(m->i)->body;
  rai::Vector vec = world.shapes(m->i)->rel*m->ivec;
  rai::Shape* l_ft_sensor = world.getShapeByName("l_ft_sensor");
  arr J_ft, J;
  world.kinematicsPos(NoArr, J,   body, vec);
  world.kinematicsPos_wrtFrame(NoArr, J_ft, body, vec, l_ft_sensor);

  //-- compute the control coefficients
  u_bias = ~J*f_ref;
  f_des = f_ref;
  J_ft_inv = inverse_SymPosDef(J_ft*~J_ft)*J_ft;
  K_I = f_alpha*~J;
}

double CtrlObjective::error() {
  if(!(y.N && y.N==y_ref.N && v.N==v_ref.N)) return -1.;
  return maxDiff(y, y_ref) + maxDiff(v, v_ref);
}

bool CtrlObjective::isConverged(double tolerance) {
  return (y.N && y.N==y_ref.N && v.N==v_ref.N
          && maxDiff(y, y_ref)<tolerance
          && maxDiff(v, v_ref)<tolerance); //TODO what if Kp = 0, then it should not count?!?
}

void CtrlObjective::reportState(ostream& os) {
  os <<"  CtrlObjective " <<name;
  if(!active) cout <<" INACTIVE";
  if(y_ref.N==y.N && v_ref.N==v.N) {
    os <<":  y_ref=" <<y_ref <<" \ty=" <<y
       <<"  Pterm=(" <<Kp <<'*' <<length(y_ref-y)
       <<")   Dterm=(" <<Kd <<'*' <<length(v_ref-v) <<')'
       <<endl;
  } else {
    os <<" -- y_ref.N!=y.N or v_ref.N!=v.N -- not initialized? -- "
       <<" Pgain=" <<Kp
       <<" Dgain=" <<Kd <<endl;
  }
}

//===========================================================================

void ConstraintForceTask::updateConstraintControl(const arr& _g, const double& lambda_desired) {
  CHECK_EQ(_g.N, 1, "can handle only 1D constraints so far");
  double g=_g(0);
  CHECK_GE(lambda_desired, 0., "lambda must be positive or zero");

  if(g<0 && lambda_desired>0.) { //steer towards constraint
    desiredApproach.y_ref=ARR(.05); //set goal to overshoot!
    desiredApproach.setGainsAsNatural(.3, 1.);
    desiredApproach.prec=ARR(1e4);
  }

  if(g>-1e-2 && lambda_desired>0.) { //stay in constraint -> constrain dynamics
    desiredApproach.y_ref=ARR(0.);
    desiredApproach.setGainsAsNatural(.05, .7);
    desiredApproach.prec=ARR(1e6);
  }

  if(g>-0.02 && lambda_desired==0.) { //release constraint -> softly push out
    desiredApproach.y_ref=ARR(-0.04);
    desiredApproach.setGainsAsNatural(.3, 1.);
    desiredApproach.prec=ARR(1e4);
  }

  if(g<=-0.02 && lambda_desired==0.) { //stay out of contact -> constrain dynamics
    desiredApproach.active=false;
  }
}

//===========================================================================

TaskControlMethods::TaskControlMethods(rai::Configuration& _world, bool _useSwift)
  : world(_world), qNullCostRef(nullptr, nullptr), useSwift(_useSwift) {
  computeMeshNormals(world.shapes);
  if(useSwift) {
    makeConvexHulls(world.shapes);
    world.swift().setCutoff(2.*rai::getParameter<double>("swiftCutoff", 0.11));
  }
  qNullCostRef.name="qitselfPD";
  qNullCostRef.setGains(0., 100.);
  qNullCostRef.prec = getH_rate_diag(world);
  qNullCostRef.setTarget(world.q);
}

CtrlObjective* TaskControlMethods::addPDTask(const char* name, double decayTime, double dampingRatio, Feature* map) {
  return tasks.append(new CtrlObjective(name, map, decayTime, dampingRatio, 1., 1.));
}

CtrlObjective* TaskControlMethods::addPDTask(const char* name,
    double decayTime, double dampingRatio,
    TM_DefaultType type,
    const char* iShapeName, const rai::Vector& ivec,
    const char* jShapeName, const rai::Vector& jvec) {
  return tasks.append(new CtrlObjective(name, new TM_Default(type, world, iShapeName, ivec, jShapeName, jvec),
                                        decayTime, dampingRatio, 1., 1.));
}

ConstraintForceTask* TaskControlMethods::addConstraintForceTask(const char* name, Feature* map) {
  ConstraintForceTask* t = new ConstraintForceTask(map);
  t->name=name;
  t->desiredApproach.name=STRING(name <<"_PD");
  t->desiredApproach.active=false;
  forceTasks.append(t);
  tasks.append(&t->desiredApproach);
  return t;
}

void TaskControlMethods::lockJointGroup(const char* groupname, bool lockThem) {
  if(!groupname) {
    if(lockThem) {
      lockJoints = consts<bool>(true, world.q.N);
      world.qdot.setZero();
    } else lockJoints.clear();
    return;
  }
  if(!lockJoints.N) lockJoints = consts<bool>(false, world.q.N);
  for(rai::Joint* j:world.joints) {
    if(j->ats.getNode(groupname)) {
      for(uint i=0; i<j->qDim(); i++) {
        lockJoints(j->qIndex+i) = lockThem;
        if(lockThem) world.qdot(j->qIndex+i) = 0.;
      }
    }
  }
}

void TaskControlMethods::getTaskCoeffs(arr& yddot_des, arr& J) {
  yddot_des.clear();
  if(!!J) J.clear();
  arr J_y, a_des;
  for(CtrlObjective* t: tasks) {
    t->feat.phi(t->y, J_y, world);
    t->v = J_y*world.qdot;
    if(t->active && !t->f_ref.N) {
      a_des = t->getDesiredAcceleration();
      yddot_des.append(::sqrt(t->prec)%(a_des /*-Jdot*qdot*/));
      if(!!J) J.append(::sqrt(t->prec)%J_y);
    }
  }
  if(!!J) J.reshape(yddot_des.N, world.q.N);
}

void TaskControlMethods::reportCurrentState() {
  cout <<"** TaskControlMethods" <<endl;
  for(CtrlObjective* t: tasks) t->reportState(cout);
}

void TaskControlMethods::setState(const arr& q, const arr& qdot) {
  world.setJointState(q, qdot);
  if(useSwift) world.stepSwift();
}

void TaskControlMethods::updateConstraintControllers() {
  arr y;
  for(ConstraintForceTask* t: forceTasks) {
    if(t->active) {
      t->feat.phi(y, NoArr, world);
      t->updateConstraintControl(y, t->desiredForce);
    }
  }
}

arr TaskControlMethods::getDesiredConstraintForces() {
  arr Jl(world.q.N, 1);
  Jl.setZero();
  arr y, J_y;
  for(ConstraintForceTask* t: forceTasks) {
    if(t->active) {
      t->feat.phi(y, J_y, world);
      CHECK_EQ(y.N, 1, " can only handle 1D constraints for now");
      Jl += ~J_y * t->desiredForce;
    }
  }
  Jl.reshape(Jl.N);
  return Jl;
}

arr TaskControlMethods::operationalSpaceControl() {
  arr yddot_des, J;
  getTaskCoeffs(yddot_des, J); //this corresponds to $J_\phi$ and $c$ in the reference (they include C^{1/2})
  if(!yddot_des.N && !qNullCostRef.active) return zeros(world.q.N, 1).reshape(world.q.N);
  arr A = qNullCostRef.getC();
  arr a = zeros(A.d0);
  qNullCostRef.y=world.q;
  qNullCostRef.v=world.qdot;
  if(qNullCostRef.active) {
    a += qNullCostRef.getC() * qNullCostRef.getDesiredAcceleration();
  }
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

arr TaskControlMethods::getDesiredLinAccLaw(arr& Kp, arr& Kd, arr& k) {
  arr Kp_y, Kd_y, k_y, C_y;
  qNullCostRef.y=world.q;
  qNullCostRef.v=world.qdot;
  qNullCostRef.getDesiredLinAccLaw(Kp_y, Kd_y, k_y);
  arr H = qNullCostRef.getC();

  Kp = H * Kp_y;
  Kd = H * Kd_y;
  k  = H * k_y;

  arr JCJ = zeros(world.q.N, world.q.N);

  for(CtrlObjective* task : tasks) if(task->active) {
      arr J_y;
      task->feat.phi(task->y, J_y, world);
      task->v = J_y*world.qdot;
      task->getDesiredLinAccLaw(Kp_y, Kd_y, k_y);
      C_y = task->getC();

      arr JtC_y = ~J_y*C_y;

      JCJ += JtC_y*J_y;

      Kp += JtC_y*Kp_y*J_y;
      Kd += JtC_y*Kd_y*J_y;
      k  += JtC_y*(k_y + Kp_y*(J_y*world.q - task->y));
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

  return k - Kp*world.q - Kd*world.qdot;
}

arr TaskControlMethods::calcOptimalControlProjected(arr& Kp, arr& Kd, arr& u0, const arr& M, const arr& F) {
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
      law->feat.phi(law->y, J_y, world);
      law->v = J_y*world.qdot;
      tempJPrec = ~J_y*law->getC();
      A += tempJPrec*J_y;

      law->getDesiredLinAccLaw(Kp_y, Kd_y, a0_y);

      u0 += tempJPrec*a0_y;

      tempKp = tempJPrec*Kp_y;

      u0 += tempKp*(-y + J_y*world.q);

      //u0 += ~J*law->getC()*law->getDDotRef(); //TODO: add ydd_ref

      Kp += tempKp*J_y;
      Kd += tempJPrec*Kd_y*J_y;
    }
  arr invA = inverse(A); //TODO: SymPosDef?
  Kp = M*invA*Kp;
  Kd = M*invA*Kd;
  u0 = M*invA*u0 + F;

  return u0 + Kp*world.q + Kd*world.qdot;
}

void TaskControlMethods::fwdSimulateControlLaw(arr& Kp, arr& Kd, arr& u0) {
  arr M, F;
  world.equationOfMotion(M, F, false);

  arr u = u0 - Kp*world.q - Kd*world.qdot;
  arr qdd;
  world.fwdDynamics(qdd, world.qdot, u);

  for(uint tt=0; tt<10; tt++) {
    world.qdot += .001*qdd;
    world.q += .001*world.qdot;
    setState(world.q, world.qdot);
  }
}

void TaskControlMethods::calcForceControl(arr& K_ft, arr& J_ft_inv, arr& fRef, double& gamma) {
  uint nForceTasks=0;
  for(CtrlObjective* law : this->tasks) if(law->active && law->f_ref.N) {
      nForceTasks++;
      TM_Default& map = dynamic_cast<TM_Default&>(law->feat);
      rai::Body* body = world.shapes(map.i)->body;
      rai::Vector vec = world.shapes(map.i)->rel.pos;
      rai::Shape* lFtSensor = world.getShapeByName("r_ft_sensor");
      arr y, J, J_ft;
      law->feat.phi(y, J, world);
      world.kinematicsPos_wrtFrame(NoArr, J_ft, body, vec, lFtSensor);
      J_ft_inv = -~conv_vec2arr(map.ivec)*inverse_SymPosDef(J_ft*~J_ft)*J_ft;
      K_ft = -~J*law->f_alpha;
      fRef = law->f_ref;
      gamma = law->f_gamma;
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
