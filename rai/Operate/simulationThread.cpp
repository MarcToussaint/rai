#include "simulationThread.h"
#include <RosCom/roscom.h>
//#include "simulationIO_self.h"

struct SimulationIO_self{
  RosCom ROS;

  //inputs (reactive variables / messages)
  Var<rai_msgs::MotionReference> ref; // communication variable with ROS
  Var<rai_msgs::StringA> command;

  //outputs
  Var<rai_msgs::arr> currentQ;
  Var<rai_msgs::arr> objectPoses;
  Var<rai_msgs::StringA> objectNames;

  std::shared_ptr<Subscriber<rai_msgs::MotionReference>> sub_ref; //subscriber
  std::shared_ptr<Subscriber<rai_msgs::StringA>> sub_command; //subscriber
  std::shared_ptr<Publisher<rai_msgs::arr>> pub_currentQ;
  std::shared_ptr<Publisher<rai_msgs::arr>> pub_objectPoses;
  std::shared_ptr<Publisher<rai_msgs::StringA>> pub_objectNames;
  std::shared_ptr<PublisherConv<std_msgs::Float64, double, conv_double2Float64>> pub_timeToGo;

  SimulationIO_self();
};

SimulationIO_self::SimulationIO_self()
    : ROS("simulator"),
      ref("MotionReference"),
      command("command"),
      currentQ("currentQ"),
      objectPoses("objectPoses"),
      objectNames("objectNames"){

    //setup ros communication
    sub_ref = ROS.subscribe(ref);
    sub_command = ROS.subscribe(command);

    pub_objectPoses = ROS.publish(objectPoses);
    pub_currentQ = ROS.publish(currentQ);
}

SimulationThread::SimulationThread(const rai::KinematicWorld& K, double dt, bool _pubSubToROS)
  : Thread("SimulationIO", dt),
    SIM(K, dt),
    pubSubToROS(_pubSubToROS),
    timeToGo("timeToGo"){

    if(pubSubToROS){
        self = new SimulationIO_self();
        self->ROS.publish(self->pub_timeToGo, timeToGo);
    }

    threadLoop();
}

SimulationThread::~SimulationThread(){
  threadClose();
  if(self) delete self;
}

void SimulationThread::step(){
  //check for inputs:
  if(pubSubToROS && self->ref.hasNewRevision()){
    self->ref.readAccess();
    StringA joints = conv_stdStringVec2StringA(self->ref->joints);
    arr t = conv_arr2arr(self->ref->t);
    arr x = conv_arr2arr(self->ref->x);
    bool append = self->ref->append;
    cout <<"MotionReference revision=" <<self->ref.getRevision() <<' ' <<self->ref->revision <<endl;
    self->ref.deAccess();

    SIM.setUsedRobotJoints(joints);
    SIM.exec(x, t, append);
  }

  if(pubSubToROS && self->command.hasNewRevision()){
    StringA cmd = conv_StringA2StringA( self->command.get() );
    SIM.exec(cmd);
  }

  SIM.stepKin();

  timeToGo.set() = SIM.getTimeToGo();

  //publish:
  if(pubSubToROS){
      self->currentQ.set() = conv_arr2arr( SIM.getJointState() );
      self->objectNames.set() = conv_StringA2StringA( SIM.getObjectNames() );
      self->objectPoses.set() = conv_arr2arr( SIM.getObjectPoses() );
  }
}

void SimulationThread::loop(){
  //loop
  Metronome tictac(dt);
  for(;;){
    tictac.waitForTic();
    step();
  }
}

