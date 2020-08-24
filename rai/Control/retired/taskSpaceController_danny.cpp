/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "taskSpaceController.h"
#include "../Algo/spline.h"

LinTaskSpaceAccLaw::LinTaskSpaceAccLaw(Feature* map, rai::Configuration* world, rai::String name) : map(map), world(world), name(name) {
  this->setRef(); //TODO: is this the best way?
  uint dim = this->getPhiDim();
  this->setC(zeros(dim, dim));
  this->setGains(zeros(dim, dim), zeros(dim, dim));
}

// TODO: enable to set ref and generate trajectory out of it

void LinTaskSpaceAccLaw::setRef(const arr& yRef, const arr& yDotRef, const arr& yDDotRef) {
  if(!!yRef) {
    this->yRef = yRef;
  } else if(!this->yRef.N) {
    this->yRef = this->getPhi();//zeros(this->getPhiDim()); // TODO: is this the best way?
  }
  if(!!yDotRef) {
    this->yDotRef = yDotRef;
  } else if(!this->yDotRef.N) {
    this->yDotRef = zeros(this->getPhiDim());
  }
  if(!!yDDotRef) {
    this->yDDotRef = yDDotRef;
  } else if(!this->yDDotRef.N) {
    this->yDDotRef = zeros(this->getPhiDim());
  }
}

void LinTaskSpaceAccLaw::setGains(arr Kp, arr Kd) {
  this->Kp = Kp;
  this->Kd = Kd;
}

void LinTaskSpaceAccLaw::setC(arr C) {
  this->C = C;
}

void LinTaskSpaceAccLaw::setSpline(rai::Spline* yS, rai::Spline* yDotS, rai::Spline* yDDotS) {
  this->trajectorySpline = yS;
  this->trajectoryDotSpline = yDotS;
  this->trajectoryDDotSpline = yDDotS;
}

void LinTaskSpaceAccLaw::setTargetEvalSpline(double s) {
  //TODO: here I tried to use velocities and accerlerations of the spline to feed it into the controller.
  //this->setRef(this->trajectorySpline->eval(s), this->trajectorySpline->eval(s, 1),this->trajectorySpline->eval(s, 2));
  //this->setRef(this->trajectorySpline->eval(s), this->trajectoryDotSpline->eval(s));//, this->trajectorySpline->eval(s, 2)/20.0);
  //this->setRef(this->trajectorySpline->eval(s), this->trajectorySpline->eval(s,1)/10.0, this->trajectorySpline->eval(s,2)/10.0);
  this->setRef(this->trajectorySpline->eval(s), this->trajectoryDotSpline->eval(s), this->trajectoryDDotSpline->eval(s));
}

void LinTaskSpaceAccLaw::setTrajectory(uint trajLength, const arr& traj, const arr& trajDot, const arr& trajDDot) {
  if(!!traj) {
    this->trajectory = traj;
  } else if(!this->trajectory.N) {
    this->trajectory = zeros(trajLength, this->getPhiDim()); //TODO: same as setRef: getPhi
  }
  if(!!trajDot) {
    this->trajectoryDot = trajDot;
  } else if(!this->trajectoryDot.N) {
    this->trajectoryDot = zeros(trajLength, this->getPhiDim());
  }
  if(!!trajDDot) {
    this->trajectoryDDot = trajDDot;
  } else if(!this->trajectoryDDot.N) {
    this->trajectoryDDot = zeros(trajLength, this->getPhiDim());
  }
}

void LinTaskSpaceAccLaw::setTrajectoryActive(bool active) {
  this->trajectoryActive = active;
}

arr LinTaskSpaceAccLaw::getPhi() {
  arr y;
  this->feat->phi(y, NoArr, *this->world);
  return y;
}

void LinTaskSpaceAccLaw::getPhi(arr& y, arr& J) {
  this->feat->phi(y, J, *this->world);
}

uint LinTaskSpaceAccLaw::getPhiDim() {
  return this->feat->dim_phi(*this->world);
}

arr LinTaskSpaceAccLaw::getC() {
  return this->C;
}

arr LinTaskSpaceAccLaw::getKp() {
  return this->Kp;
}

arr LinTaskSpaceAccLaw::getKd() {
  return this->Kd;
}

arr LinTaskSpaceAccLaw::getRef() {
  return this->yRef;
}

arr LinTaskSpaceAccLaw::getDotRef() {
  return this->yDotRef;
}

arr LinTaskSpaceAccLaw::getDDotRef() {
  return this->yDDotRef;
}

bool LinTaskSpaceAccLaw::getTrajectoryActive() {
  return this->trajectoryActive;
}

double LinTaskSpaceAccLaw::getCosts() {
  arr yAct;
  this->feat->phi(yAct, NoArr, *this->world);
  NIY;
  return ~yAct*this->C*yAct; //this is murks!!!!!!!!!!!!!! TODO
}

void ConstrainedTaskLaw::setForce(arr force) {
  this->force = force;
}

arr ConstrainedTaskLaw::getForce() {
  return this->force;
}

void ConstrainedTaskLaw::setAlpha(arr alpha) {
  this->alpha = alpha;
}

arr ConstrainedTaskLaw::getAlpha() {
  return this->alpha;
}

void ConstrainedTaskLaw::setGamma(double gamma) {
  this->gamma = gamma;
}

double ConstrainedTaskLaw::getGamma() {
  return this->gamma;
}

void TaskSpaceController::addLinTaskSpaceAccLaw(LinTaskSpaceAccLaw* law) {
  this->taskSpaceAccLaws.append(law);
}

void TaskSpaceController::addConstrainedTaskLaw(ConstrainedTaskLaw* law) {
  this->constrainedTaskLaws.append(law);
  this->addLinTaskSpaceAccLaw(law);
}

void TaskSpaceController::calcOptimalControlProjected(arr& Kp, arr& Kd, arr& u0) {
  arr M, F;
  world->equationOfMotion(M, F, this->gravity);

  arr q0, q, qDot;
  world->getJointState(q, qDot);

  //arr H = /*diag(this->world->getHmetric());//*/0.1*eye(this->world->getJointStateDimension());
  //M = H;
  //F = zeros(this->world->getJointStateDimension());

  arr H = inverse(M); //TODO: Other metrics (have significant influence)

  arr A = ~M*H*M; //TODO: The M matrix is symmetric, isn't it? And also symmetric? Furthermore, if H = M^{-1}, this should be calculated more efficiently
  arr a = zeros(this->world->getJointStateDimension());//M*eye(this->world->getJointStateDimension())*5.0*(-qDot);// //TODO: other a possible
  u0 = ~M*H*(a-F);
  arr y, J;
  arr tempKp, tempKd;

  q0 = q;
  Kp = zeros(this->world->getJointStateDimension(), this->world->getJointStateDimension());
  Kd = zeros(this->world->getJointStateDimension(), this->world->getJointStateDimension());
  for(LinTaskSpaceAccLaw* laws : this->taskSpaceAccLaws) {
    laws->getPhi(y, J);
    A += ~J*laws->getC()*J;
    tempKp = ~J*laws->getC()*laws->getKp();
    tempKd = ~J*laws->getC()*laws->getKd();
    u0 += tempKp*(laws->getRef() - y + J*q0);
    u0 += tempKd*laws->getDotRef();
    u0 += ~J*laws->getC()*laws->getDDotRef();
    Kp += tempKp*J;
    Kd += tempKd*J;
  }
  arr invA = inverse(A); //TODO: SymPosDef?
  Kp = M*invA*Kp;
  Kd = M*invA*Kd;
  u0 = M*invA*u0 + F;
}

void TaskSpaceController::calcForceControl(arr& K_ft, arr& J_ft_inv, arr& fRef, double& gamma) {
  if(this->constrainedTaskLaws.N > 0) {
    CHECK_EQ(this->constrainedTaskLaws.N,  1, "Multiple force laws not allowed at the moment");
    for(ConstrainedTaskLaw* law : this->constrainedTaskLaws) {
      TM_Default* m = dynamic_cast<TM_Default*>(law->feat);
      rai::Body* body = this->world->shapes(m->i)->body;
      rai::Vector vec = this->world->shapes(m->i)->rel.pos;
      rai::Shape* lFtSensor = this->world->getShapeByName("l_ft_sensor");
      arr y, J, J_ft;
      law->getPhi(y, J);
      this->world->kinematicsPos_wrtFrame(NoArr, J_ft, body, vec, lFtSensor);
      J_ft_inv = -~conv_vec2arr(m->ivec)*inverse_SymPosDef(J_ft*~J_ft)*J_ft;
      K_ft = -~J*law->getAlpha();
      fRef = law->getForce();
      gamma = law->getGamma();
    }
  } else {
    K_ft = zeros(this->world->getJointStateDimension());
    fRef = ARR(0.0);
    J_ft_inv = zeros(1, 6);
    gamma = 0.0;
  }
}

#if 0
void TaskSpaceController::generateTaskSpaceTrajectoryFromJointSpace(arr jointSpaceTrajectory) {
  uint trajSteps = jointSpaceTrajectory.d0;
  arr qOld, qDotOld, y;
  this->world->getJointState(qOld, qDotOld);
  for(uint i = 0; i < trajSteps; i++) {
    this->world->setJointState(jointSpaceTrajectory[i]);
    for(LinTaskSpaceAccLaw* law : this->taskSpaceAccLaws) {
      if(law->getTrajectoryActive()) {
        law->getPhi(y, NoArr);
      } else {
        y = law->getRef();
      }
      law->trajectory.append(y);
    }
  }
  for(LinTaskSpaceAccLaw* law : this->taskSpaceAccLaws) {
    law->trajectory.reshape(trajSteps, law->getPhiDim());
  }
  this->world->setJointState(qOld, qDotOld);
}
#elif 0
void TaskSpaceController::generateTaskSpaceTrajectoryFromJointSpace(arr jointSpaceTrajectory) {
  uint trajSteps = jointSpaceTrajectory.d0;
  arr qOld, qDotOld, y, yDot, y0, y1;
  this->world->getJointState(qOld, qDotOld);
  for(LinTaskSpaceAccLaw* law : this->taskSpaceAccLaws) {
    for(uint i = 0; i < trajSteps; i++) {
      this->world->setJointState(jointSpaceTrajectory[i]);
      if(law->getTrajectoryActive()) {
        law->getPhi(y, NoArr);
      } else {
        y = law->getRef();
      }
      law->trajectory.append(y);
    }

    if(law->getTrajectoryActive()) {
      this->world->setJointState(jointSpaceTrajectory[0]);
      law->getPhi(y0, NoArr);
      this->world->setJointState(jointSpaceTrajectory[1]);
      law->getPhi(y1, NoArr);
      yDot = (y1-y0)/0.1;
    } else {
      yDot = law->getDotRef();
    }
    law->trajectoryDot.append(yDot);

    for(uint i = 1; i < trajSteps-1; i++) {
      if(law->getTrajectoryActive()) {
        this->world->setJointState(jointSpaceTrajectory[i-1]);
        law->getPhi(y0, NoArr);
        this->world->setJointState(jointSpaceTrajectory[i+1]);
        law->getPhi(y1, NoArr);
        yDot = (y1-y0)/(2*0.1);
      } else {
        yDot = law->getDotRef();
      }
      law->trajectoryDot.append(yDot);
    }

    if(law->getTrajectoryActive()) {
      this->world->setJointState(jointSpaceTrajectory[jointSpaceTrajectory.d0-2]);
      law->getPhi(y0, NoArr);
      this->world->setJointState(jointSpaceTrajectory[jointSpaceTrajectory.d0-1]);
      law->getPhi(y1, NoArr);

      yDot = (y1-y0)/0.1;
    } else {
      yDot = law->getDotRef();
    }
    law->trajectoryDot.append(yDot);
  }
  for(LinTaskSpaceAccLaw* law : this->taskSpaceAccLaws) {
    law->trajectory.reshape(trajSteps, law->getPhiDim());
    law->trajectoryDot.reshape(trajSteps, law->getPhiDim());
  }
  this->world->setJointState(qOld, qDotOld);
}
#else
void TaskSpaceController::generateTaskSpaceTrajectoryFromJointSpace(const arr& jointSpaceTrajectory, const arr& jointSpaceTrajectoryDot, const arr& jointSpaceTrajectoryDDot) {
  uint trajSteps = jointSpaceTrajectory.d0;
  arr qOld, qDotOld, y, yDot, yDDot;
  this->world->getJointState(qOld, qDotOld);

  for(uint i = 0; i < trajSteps; i++) {
    this->world->setJointState(jointSpaceTrajectory[i]);
    for(LinTaskSpaceAccLaw* law : this->taskSpaceAccLaws) {
      if(law->getTrajectoryActive()) {
        law->getPhi(y, NoArr);
      } else {
        y = law->getRef();
      }
      law->trajectory.append(y);
    }
  }
  for(LinTaskSpaceAccLaw* law : this->taskSpaceAccLaws) {
    law->trajectory.reshape(trajSteps, law->getPhiDim());
    law->trajectoryDot = zeros(trajSteps, law->getPhiDim());
    law->trajectoryDDot = zeros(trajSteps, law->getPhiDim());
  }
  this->world->setJointState(qOld, qDotOld);
}
#endif

void TaskSpaceController::generateTaskSpaceSplines() {
  for(LinTaskSpaceAccLaw* law : this->taskSpaceAccLaws) {
    CHECK_GE(law->trajectory.d0,  3, "The trajectory must consists of at least 3 states for the spline to work");
    CHECK_GE(law->trajectoryDot.d0,  3, "The trajectoryDot must consists of at least 3 states for the spline to work");
    CHECK_GE(law->trajectoryDDot.d0,  3, "The trajectoryDDot must consists of at least 3 states for the spline to work");
    //law->setSpline(new rai::Spline(law->trajectory.d0, law->trajectory), new rai::Spline(law->trajectory.d0, law->trajectoryDot));
    //law->setSpline(new rai::Spline(law->trajectory.d0, law->trajectory));
    law->setSpline(new rai::Spline(law->trajectory.d0, law->trajectory), new rai::Spline(law->trajectoryDot.d0, law->trajectoryDot), new rai::Spline(law->trajectoryDDot.d0, law->trajectoryDDot));
  }
}

void TaskSpaceController::setGravity(bool gravity) {
  this->gravity = gravity;
  cout << gravity << endl;
}

