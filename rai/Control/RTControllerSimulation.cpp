/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "RTControllerSimulation.h"
#include "../Kin/proxy.h"
#include "../Kin/frame.h"

void force(rai::Configuration* world, arr& fR) {
  world->stepSwift();
  //world->contactsToForces(100.0);

  for(const rai::Proxy& p : world->proxies) {
    if(p.a->name == "endeffR" && p.b->name == "b") {
      if(p.d <= 0.0) {
        rai::Vector trans = p.posB - p.posA;
        rai::Vector force = 100.0*trans;
        rai::Vector torque = (p.posA - p.a->ensure_X().pos) ^ force;
        fR(0) = force(0);
        fR(1) = force(1);
        fR(2) = force(2);
        fR(3) = torque(0);
        fR(4) = torque(1);
        fR(5) = torque(2);
        cout << fR(2) << endl;
      }
    }
  }
}

void forceSimulateContactOnly(rai::Configuration* world, arr& fR) {
  world->stepSwift();
  for(const rai::Proxy& p : world->proxies) {
    if(p.a->name == "endeffR" && p.b->name == "b") {
      if(p.d <= 0.02) {
        fR(2) = -4.0;
      }
    }
  }
}

void calcFTintegral(arr& f_errIntegral, const arr& f_ref, const arr& f_obs, const arr& J_ft_inv, const double& f_gamma) {
  // check if f_err has same dimension as f_ref, otherwise reset to zero
  if(f_errIntegral.N != f_ref.N) {
    f_errIntegral = zeros(f_ref.N);
  }

  f_errIntegral *= f_gamma;
  arr f_task = J_ft_inv*f_obs;

  for(uint i=0; i<f_task.N; i++) {
    if(f_ref(i) < 0) {
      if(f_task(i) < f_ref(i)) {
        f_errIntegral(i) += f_ref(i) - f_task(i);
      }
    } else {
      if(f_task(i) > f_ref(i)) {
        f_errIntegral(i) += f_ref(i) - f_task(i);
      }
    }
  }
}

void RTControlStep(
  arr& u,
  arr& base_v,
  arr& I_term, arr& fL_errIntegral, arr& fR_errIntegral,
  const arr& q, const arr& qd,
  const arr& fL_obs, const arr& fR_obs,
  const CtrlMsg& cmd,
  const arr& Kp_base, const arr& Kd_base,
  const arr& limits,
  rai::Joint* j_baseTranslationRotation
) {

  //-- PD terms
  u = cmd.u_bias;
  if(cmd.Kp.N==1 && cmd.Kd.N==1) { //both just scalars -> use *_base (defined in cfg file)
    u += Kp_base % (cmd.Kp.scalar() * (cmd.q - q));
    u += Kd_base % (cmd.Kd.scalar() * (cmd.qdot - qd));
  } else if(cmd.Kp.d0==q.N && cmd.Kp.d1==q.N && cmd.Kd.N==1) { //Kd is scalar -> use *_base (defined in cfg file)
    u += Kp_base % (cmd.Kp * (cmd.q - q));
    u += Kd_base % (cmd.Kd.scalar() * (cmd.qdot - qd));
  } else if(cmd.Kp.d0==q.N && cmd.Kp.d1==q.N && cmd.Kd.d0==q.N && cmd.Kd.d1==q.N) { //Kp and Kd are proper matrices -> assume they define desired linear acceleration laws
    u += (cmd.Kp * (cmd.q - q));
    u += (cmd.Kd * (cmd.qdot - qd));
  }

  //-- I term
  if(cmd.Ki.N==1) {
    I_term += Kp_base % (cmd.Ki.scalar() *0.01 * (cmd.q - q));
    //limits: [q_lo, q_hi, vel_limit, u_limit, int_limit]
    for(uint i=0; i<q.N; i++) clip(I_term(i), -cmd.intLimitRatio*limits(i, 4), cmd.intLimitRatio*limits(i, 4));
    u += I_term;
  }

  //-- F/T sensor error
  //TODO: How to allow multiple Tasks? Upper AND Lower bounds simultaneously?
  //TODO(mt): don't distinguish between L and R -- all is just matrix equations..
  if(!!fL_errIntegral) {
    if(!cmd.KiFTL.N) {    // no contact or Ki gain -> don't use the integral term
      fL_errIntegral = fL_errIntegral*0.;              // reset integral error
    } else {
      calcFTintegral(fL_errIntegral, cmd.fL, fL_obs, cmd.J_ft_invL, cmd.fL_gamma);
      u += cmd.KiFTL * fL_errIntegral;
    }
  }
  if(!!fR_errIntegral) {
    if(!cmd.KiFTR.N) {    // no contact or Ki gain -> don't use the integral term
      fR_errIntegral = fR_errIntegral*0.;              // reset integral error
    } else {
      calcFTintegral(fR_errIntegral, cmd.fR, fR_obs, cmd.J_ft_invR, cmd.fR_gamma);
      u += cmd.KiFTR * fR_errIntegral;
    }
  }

  //-- base velocities
  if(j_baseTranslationRotation && j_baseTranslationRotation->qDim()==3) {
    double phi = cmd.q(j_baseTranslationRotation->qIndex+2);
    double vx  = cmd.qdot(j_baseTranslationRotation->qIndex+0);
    double vy  = cmd.qdot(j_baseTranslationRotation->qIndex+1);
    double co  = cos(phi), si = -sin(phi);
    base_v.resize(3);
    base_v(0) = co*vx - si*vy; //x
    base_v(1) = si*vx + co*vy; //y
    base_v(2) = cmd.qdot(j_baseTranslationRotation->qIndex+2); //phi
  } else {
    base_v.clear();
  }

  //-- clip torques
  for(uint i=0; i<q.N; i++) {
    /*double velM = marginMap(qd(i), -velLimitRatio*limits(i,2), velLimitRatio*limits(i,2), .1);
        //clip(velM, -1., 1.)
    if(velM<0. && u(i)<0.) u(i)*=(1.+velM); //decrease effort close to velocity margin
    if(velM>0. && u(i)>0.) u(i)*=(1.-velM); //decrease effort close to velocity margin
      */
    //clip(u(i), -cmd.effLimitRatio*limits(i,3), cmd.effLimitRatio*limits(i,3));
  }

}

RTControllerSimulation::RTControllerSimulation(const rai::Configuration& realWorld, const Var<CtrlMsg>& _ctrl_ref, const Var<CtrlMsg>& _ctrl_obs, double tau, bool gravity, double _systematicErrorSdv)
  : Thread("DynmSim", -1.)
  , ctrl_ref(this, _ctrl_ref, true)
  , ctrl_obs(this, _ctrl_obs)
    //, modelWorld(this, "modelWorld")
  , tau(tau)
  , gravity(gravity)
  , stepCount(0)
  , systematicErrorSdv(_systematicErrorSdv) {
  //world = new rai::Configuration(realWorld);
  world = new rai::Configuration(rai::raiPath("data/pr2_model/pr2_model.g"));

  //Object o(*world);
  //o.generateObject("b", 0.16, 0.16, 0.1, 0.55, -0.1, 0.55); //0.5 for x

  //Object o(*world);
  //o.generateObject("b", 0.16, 0.16, 0.1, 0.55, -0.1, 0.55); //0.5 for x
  //Object ob(*world);
  //ob.generateObject("trueShape", 0.17, 0.17, 0.12, 0.55, -0.1, 0.55, false);
}

void RTControllerSimulation::open() {
  //world = new rai::Configuration;
  //world->copy(modelWorld.get()());
  //world = new rai::Configuration(modelWorld.get());
  //world = new rai::Configuration(rai::raiPath("data/pr2_model/pr2_model.g"));

  makeConvexHulls(world->frames);
  arr q = world->getJointState();
  arr qDot = zeros(q.N);

  //makeConvexHulls(world->shapes);

  I_term = zeros(q.N);

  // read ctrl parameters from dfg file:
  Kp_base.resize(world->q.N).setZero();
  Kd_base.resize(world->q.N).setZero();
  limits.resize(world->q.N, 5).setZero();
  rai::Joint* j;
  for(rai::Frame* f: world->frames) if((j=f->joint) && j->qDim()>0) {
      arr* info;
      info = f->ats.find<arr>("gains");  if(info) {
        for(uint i=0; i<j->qDim(); i++) { Kp_base(j->qIndex+i)=info->elem(0); Kd_base(j->qIndex+i)=info->elem(1); }
      }
      info = f->ats.find<arr>("limits");  if(info) {
        for(uint i=0; i<j->qDim(); i++) { limits(j->qIndex+i, 0)=info->elem(0); limits(j->qIndex+i, 1)=info->elem(1); }
      }
      info = f->ats.find<arr>("ctrl_limits");  if(info) {
        for(uint i=0; i<j->qDim(); i++) { limits(j->qIndex+i, 2)=info->elem(0); limits(j->qIndex+i, 3)=info->elem(1); limits(j->qIndex+i, 4)=info->elem(2); }
      }
    }

  this->ctrl_obs.writeAccess();
  this->ctrl_obs().q = q;
  this->ctrl_obs().qdot = qDot;
  this->ctrl_obs().fL = zeros(6);
  this->ctrl_obs().fR = zeros(6);
  this->ctrl_obs().u_bias = zeros(q.d0);
  this->ctrl_obs.deAccess();

  j_baseTranslationRotation = world->getFrame("worldTranslationRotation")->joint;
}

void RTControllerSimulation::step() {
  stepCount++;

  CtrlMsg cmd = ctrl_ref.get();

  arr u, base_v;
  arr q = world->getJointState();
  arr qDot = zeros(q.N); HALT("WARNING: qDot should be maintained outside world!");

  if(!(stepCount%200) && systematicErrorSdv>0.) {
    systematicError.resize(q.N);
    rndGauss(systematicError, systematicErrorSdv, false);
  }
  arr fR = zeros(6);
  if(cmd.q.N==q.N) {
#if 0
    //TODO: use exactly same conditions as in RT controller
    //TODO: the real RT controller does a lot more: checks ctrl limits, etc. This should be simulated as well
    u = cmd.u_bias + cmd.Kp*(cmd.q - q) + cmd.Kd*(cmd.qdot - qDot);
#else
    RTControlStep(u, base_v, I_term, NoArr, NoArr, q, qDot, NoArr, NoArr, cmd, Kp_base, Kd_base, limits, j_baseTranslationRotation);
    if(systematicError.N) u += systematicError;
#endif

    //force(world, fR);
    forceSimulateContactOnly(world, fR);
    //u(3) = 0.0;
    world->stepDynamics(qDot, u, tau, 0., this->gravity);

  }

  //cout << fR(2) << endl;

  checkNan(q);
  checkNan(qDot);
  checkNan(u);

  this->ctrl_obs.writeAccess();
  /*for(uint i = 0; i < q.N; i++) {
    q(i) = round(q(i)*1000)/1000.0;
  }*/
  this->ctrl_obs().q = q;
  this->ctrl_obs().qdot = qDot;
  this->ctrl_obs().u_bias = u;
  this->ctrl_obs().fR = fR;
  this->ctrl_obs.deAccess();

  rai::wait(tau); //TODO: why does this change something??? FISHY!
}

