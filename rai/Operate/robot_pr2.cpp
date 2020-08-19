/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_ROS

#include "robot_pr2.h"
#include "splineRunner.h"
#include "../Algo/spline.h"
#include "../Control/ctrlMsg.h"
#include "../Kin/frame.h"
#include "../RosCom/roscom.h"

struct Robot_PR2_PathThread : Thread {
  Mutex threadLock;
  RosCom ROS;
  rai::Configuration K;
  arr q0, q_real, qdot_real; //< real state
  arr Kp_base, Kd_base; //< Kp, Kd parameters defined in the model file
  double kp_factor, kd_factor, ki_factor;

  Var <CtrlMsg> ctrl_ref; //< the message send to the RTController
  Var <CtrlMsg> ctrl_obs; //< the message received from the RTController
  Var <arr> pr2_odom;

  std::shared_ptr<SubscriberConvNoHeader<rai_msgs::JointState, CtrlMsg, &conv_JointState2CtrlMsg>> sub_obs; //(ctrl_obs, "/marc_rt_controller/jointState");
  std::shared_ptr<PublisherConv<rai_msgs::JointState, CtrlMsg, &conv_CtrlMsg2JointState>>          pub_ref;//(ctrl_ref, "/marc_rt_controller/jointReference");
  std::shared_ptr<SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>>  sub_odom; //(pr2_odom, "/robot_pose_ekf/odom_combined");

  StringA currentlyUsedJoints; //the joints that the spline refers to
  SplineRunner spline;
  double dt=.01; // time stepping interval
  uint stepCount=0; // number of simulation steps

  Robot_PR2_PathThread(const rai::Configuration& _K);
  ~Robot_PR2_PathThread();

  void open() {}
  void step();
  void close() {}
};

Robot_PR2_PathThread::Robot_PR2_PathThread(const rai::Configuration& _K)
  : Thread("Robot_PR2_PathThread", .01),
    K(_K),
    ctrl_ref(this, "/marc_rt_controller/jointReference"),
    ctrl_obs(this, "/marc_rt_controller/jointState"),
    pr2_odom(this, "/robot_pose_ekf/odom_combined") {

  kp_factor = rai::getParameter<double>("controller_kp_factor", 1.);
  kd_factor = rai::getParameter<double>("controller_kd_factor", 1.);
  ki_factor = rai::getParameter<double>("controller_ki_factor", .2);

  q0 = K.getJointState();

  Kp_base = zeros(q0.N);
  Kd_base = zeros(q0.N);
  for(rai::Joint* j: K.activeJoints) if(j->qDim()>0) {
      arr* gains = j->frame->ats.find<arr>("gains");
      if(gains) {
        for(uint i=0; i<j->qDim(); i++) {
          Kp_base(j->qIndex+i)=gains->elem(0);
          Kd_base(j->qIndex+i)=gains->elem(1);
        }
      }
    }

  ROS.subscribe(sub_obs, ctrl_obs, false);
  ROS.subscribe(sub_odom, pr2_odom, false);
  ROS.publish(pub_ref, ctrl_ref, false);

  threadLoop();
}

Robot_PR2_PathThread::~Robot_PR2_PathThread() {
  threadClose();
}

void Robot_PR2_PathThread::step() {
  stepCount++;

  auto lock = threadLock(RAI_HERE);

  rai::Frame* transF = K.getFrameByName("worldTranslationRotation", false);
  rai::Joint* trans = (transF?transF->joint:nullptr);

  //-- read real state
  if(ctrl_obs.last_read_revision<10) {
    LOG(0) <<"waiting for first 10 strl message...";
    ctrl_obs.waitForRevisionGreaterThan(10);
    LOG(0) <<"...done";
  }
//    pr2_odom.waitForRevisionGreaterThan(10);

  //update q_read from both, ctrl_obs and pr2_odom
  q_real = ctrl_obs.get()->q;
  qdot_real = ctrl_obs.get()->qdot;
  arr pr2odom = pr2_odom.get();
  if(q_real.N!=K.getJointStateDimension()) {
    LOG(-1) <<"waiting for PR2 ctrl_obs message with right dimension (" <<q_real.N <<" != " <<K.getJointStateDimension() <<")";
    return;
  }

  if(pr2odom.N!=3) {
//        LOG(-1) <<"PR2 odom message of wrong dimension: (" <<pr2odom.N <<" != 3 )";
  } else {
    q_real({trans->qIndex, trans->qIndex+2}) = pr2odom;
  }

  arr qref_dot;
  arr q_ref = spline.run(dt/*, qref_dot*/);
  if(!q_ref.N) return;

  //-- determine gains
  //project gains if there are compliance tasks
  arr Kp = diag(Kp_base);  Kp *= kp_factor;
  arr Kd = diag(Kd_base);  Kd *= kd_factor;
  //  arr Ki = diag(Kp_base);  Ki *= ki_factor;
  arr Ki = ARR(ki_factor);

  //  Kp = M*Kp; DANGER!!
  //  Kd = M*Kd;

//    if(CompProj.N) {
//      Kp = CompProj*Kp*CompProj;
//      Kd = CompProj*Kd*CompProj;
//      Ki = CompProj*diag(Kp_base*ki_factor)*CompProj;
//    }

  {
    CtrlMsg refs;
    refs.q =  q_ref;
    if(qref_dot.N) refs.qdot = qref_dot;
    else refs.qdot = zeros(q_ref.N);
    refs.fL_gamma = 1.;
    refs.Kp = Kp; //ARR(1.);
    refs.Kd = Kd; //ARR(1.);
    refs.Ki = Ki; //ARR(.2);
    refs.fL = zeros(6);
    refs.fR = zeros(6);
    refs.KiFTL.clear();
    refs.J_ft_invL.clear();
    refs.u_bias = zeros(q_real.N);
    refs.intLimitRatio = 1.;
    refs.qd_filt = .99;

    //-- set base motion command as velocities
//      if(!fixBase.get() && trans && trans->qDim()==3) {
//        refs.qdot(trans->qIndex+0) = qdot_model(trans->qIndex+0);
//        refs.qdot(trans->qIndex+1) = qdot_model(trans->qIndex+1);
//        refs.qdot(trans->qIndex+2) = qdot_model(trans->qIndex+2);
//      }

    //-- send the computed movement to the robot
    ctrl_ref.set() = refs;
  }
}

Robot_PR2::Robot_PR2(const rai::Configuration& _K) {
  self = new Robot_PR2_PathThread(_K);
}

Robot_PR2::~Robot_PR2() {
  delete self;
}

bool Robot_PR2::executeMotion(const StringA& joints, const arr& path, const arr& times, double timeScale, bool append) {
  auto lock = self->threadLock(RAI_HERE);

  if(self->currentlyUsedJoints!=joints) {
    if(self->spline.refPoints.N) {
      LOG(-1) <<"you changed the robot joints before the spline was done -- killing spline execution";
      self->spline.stop();
    }
    self->currentlyUsedJoints=joints;
  }

  if(path.d1 != self->currentlyUsedJoints.N) {
    LOG(-1) <<"you're sending me a motion reference of wrong dimension!"
            <<"\n  I'm ignoring this"
            <<"\n  my dimension=" <<self->currentlyUsedJoints.N <<"  your message=" <<path.d1
            <<"\n  my joints=" <<self->currentlyUsedJoints;
    return false;
  }

  self->spline.set(path, times*timeScale, self->q_real, append);

  return true;
}

void Robot_PR2::execGripper(const rai::String& gripper, double position, double force) {
  NIY
}

arr Robot_PR2::getJointPositions(const StringA& joints) {
  self->ctrl_obs.waitForRevisionGreaterThan(10);

  auto lock = self->threadLock(RAI_HERE);

  if(joints.N) {
    NIY
  } else {
    return self->q_real;
  }
}

StringA Robot_PR2::getJointNames() {
  return self->K.getJointNames();
}

double Robot_PR2::timeToGo() {
  auto lock = self->threadLock(RAI_HERE);

  return self->spline.timeToGo();
}

arr Robot_PR2::getHomePose() {
  return self->q0;
}

#endif
