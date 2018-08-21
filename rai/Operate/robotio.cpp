#include "robotio.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <RosCom/roscom.h>
#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <RosCom/rai_msgs/SendJointTrajectory.h>
#include <RosCom/rai_msgs/StringString.h>

#include "simulationThread.h"

struct RobotAbstraction_KukaWSG : RobotAbstraction{
    RosCom ROS;

    Var<sensor_msgs::JointState> jointState;
    Var<tf::tfMessage> tfMessages;
    Var<rai_msgs::WSG_50_command> gripperCommand;
    std::map<String, Var<geometry_msgs::PoseStamped>> objectStates;

    std::shared_ptr<Subscriber<sensor_msgs::JointState>> sub_jointState; //subscriber
    std::shared_ptr<Subscriber<tf::tfMessage>> sub_tfMessages; //subscriber
    std::shared_ptr<Publisher<rai_msgs::WSG_50_command>> pub_gripperCommand; //subscriber
    std::map<String, std::shared_ptr<Subscriber<geometry_msgs::PoseStamped>>> sub_objectStates; //subscribers


    uint gripperCommandCounter=0;

    RobotAbstraction_KukaWSG(const rai::KinematicWorld& _K)
        : jointState("/joint_states"),
          tfMessages("/tf"),
          gripperCommand("/schunk_driver/schunk_wsg_command"){

        sub_jointState = ROS.subscribe(jointState);
        sub_tfMessages = ROS.subscribe(tfMessages);
        pub_gripperCommand = ROS.publish(gripperCommand);
        sub_jointState = ROS.subscribe(jointState);
        sub_tfMessages = ROS.subscribe(tfMessages);

    }

    virtual bool executeMotion(const StringA& joints, const arr& path, const arr& times, double timeScale=1., bool append=false);
    virtual void execGripper(const rai::String& gripper, double position, double force=40.);
    virtual arr getJointPositions(const StringA& joints);
    virtual  StringA getJointNames();

};

struct RobotAbstraction_SimulationThread : RobotAbstraction {
    SimulationThread S;

    RobotAbstraction_SimulationThread(const rai::KinematicWorld& _K)
        : S(_K, .01, false){
    }

    ~RobotAbstraction_SimulationThread(){}

    virtual bool executeMotion(const StringA& joints, const arr& path, const arr& times, double timeScale=1., bool append=false){
        S.stepMutex.lock();
        S.SIM.setUsedRobotJoints(joints);
        S.SIM.exec(path, times*timeScale, append);
        S.stepMutex.unlock();
        return true;
    }
    virtual void execGripper(const rai::String& gripper, double position, double force=40.){
        if(gripper=="pr2R"){
            //  komo->addObjective(0., 0., OT_eq, FS_accumulatedCollisions, {}, 1e0);
            //open gripper
            //  komo->addObjective(0.,0., OT_sos, FS_qItself, {"r_gripper_joint"}, 1e1, {.08} );
            //  komo->addObjective(0.,0., OT_sos, FS_qItself, {"r_gripper_l_finger_joint"}, 1e1, {.8} );

            S.stepMutex.lock();
            S.SIM.setUsedRobotJoints({"r_gripper_joint", "r_gripper_l_finger_joint"});
            S.SIM.exec({1,2, {position, position*10.}}, {1.}, true);
            S.stepMutex.unlock();
            return;
        }
        NIY
    }
    virtual arr getJointPositions(const StringA& joints){
        S.stepMutex.lock();
        S.SIM.setUsedRobotJoints(joints);
        arr q = S.SIM.getJointState();
        S.stepMutex.unlock();
        return q;
    }
    virtual StringA getJointNames(){
        S.stepMutex.lock();
        StringA joints = S.SIM.getJointNames();
        S.stepMutex.unlock();
        return joints;
    }
    virtual void attach(const char *a, const char *b){
        S.stepMutex.lock();
        S.SIM.exec({"attach", a, b});
        S.stepMutex.unlock();
    }
};

RobotIO::RobotIO(const rai::KinematicWorld& _K, RobotType _type)
    : type(_type),
      timeToGo("timeToGo"){

    switch(type){
        case ROB_kukaWSG: self = std::make_shared<RobotAbstraction_KukaWSG>(_K); break;
        case ROB_sim: self = std::make_shared<RobotAbstraction_SimulationThread>(_K); break;
        default: NIY;
    }
}

RobotIO::~RobotIO(){
    self.reset();
}

bool RobotAbstraction_KukaWSG::executeMotion(const StringA &joints, const arr &path, const arr &times, double timeScale, bool append){



    trajectory_msgs::JointTrajectory msg;

    //msg.header.stamp = ros::Time::now();

    for(const rai::String& str:joints){
        msg.joint_names.push_back(str.p);
    }

    CHECK_EQ(path.d1, joints.N, "");
    CHECK_EQ(path.d0, times.N, "");
    CHECK_GE(timeScale, 1., "need a timeScale >=1");

    for(uint t=0;t<path.d0;t++){
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = conv_arr2stdvec(path[t]);
        point.velocities = conv_arr2stdvec(zeros(path.d1));
        point.accelerations = conv_arr2stdvec(zeros(path.d1));
        point.effort = conv_arr2stdvec(zeros(path.d1));
        point.time_from_start = ros::Duration(times(t)*timeScale);
        msg.points.push_back(point);
    }

    rai_msgs::SendJointTrajectory com;
    com.request.trajectory = msg;
    LOG(0) <<"SERVICE CALL: send trajectory";
    if(ros::service::call("/robot_control/SendJointTrajectory", com)){
        LOG(0) <<"SERVICE CALL: return = " <<(com.response.success?"TRUE":"FALSE");
    }else{
        LOG(0) <<"SERVICE CALL: FAILED!";
        return false;
    }
    return true;
}

void RobotAbstraction_KukaWSG::execGripper(const rai::String& gripper, double position, double force){
    CHECK_EQ(gripper, "dontKnow", "");

    if(position<0 || position>100.){
        LOG(-1) <<"commanded gripper position '" <<position <<"' out of range [0, 100]";
        return;
    }


    rai_msgs::WSG_50_command cmd;
    cmd.position_mm = position;
    cmd.force = 60.0;

    gripperCommand.set() = cmd;
}

arr RobotAbstraction_KukaWSG::getJointPositions(const StringA &joints){
    jointState.waitForRevisionGreaterThan(10);
    arr X(joints.N);
    jointState.readAccess();
    for(uint i=0;i<joints.N;i++){
        for(uint j=0;j<jointState->name.size();j++){
            if(joints(i)==jointState->name[j].c_str()){
                X(i) = jointState->position[j];
            }
        }
    }
    jointState.deAccess();
    return X;
}

//arr RobotIO::getObjectPoses(const StringA& objs){
//    if(type==ROB_kukaWSG){
//        rai::Transformation OPTI=0;
//        OPTI.addRelativeRotationDeg(-90, 1,0,0);
//        OPTI.addRelativeRotationDeg(180, 0,0,1);
//        arr poses(objs.N,7); // point x,y,z quaternion x,y,z,w
//        for(uint i=0; i<objs.N; i++){
//            auto obName = objs(i);
//            if(objectStates.count(obName)){
//                objectStates[obName].waitForRevisionGreaterThan(10);
//                objectStates[obName].readAccess();
//                auto pose = objectStates[obName]->pose;
//                poses(i,0) = pose.position.x;
//                poses(i,1) = pose.position.y;
//                poses(i,2) = pose.position.z;
//                poses(i,3) = pose.orientation.w;
//                poses(i,4) = pose.orientation.x;
//                poses(i,5) = pose.orientation.y;
//                poses(i,6) = pose.orientation.z;

//                rai::Transformation tmp;
//                tmp.set(poses[i]);
//                poses[i] = ((-OPTI)*tmp*OPTI).getArr7d();

//                objectStates[obName].deAccess();
//            }
//        }

//        return poses;
//    }
//    NIY;
//    return {};
//}

StringA RobotAbstraction_KukaWSG::getJointNames() {
    jointState.waitForRevisionGreaterThan(10);
    StringA joints;
    jointState.readAccess();
    joints.resize(jointState->name.size());
    for(uint j=0;j<jointState->name.size();j++){
        joints(j) = jointState->name[j];
    }
    jointState.deAccess();
    return joints;
}

//StringA RobotIO::getObjectNames() {
//    if(type==ROB_kukaWSG){
//        StringA objects;
//        ros::master::V_TopicInfo master_topics;
//        ros::master::getTopics(master_topics);
//        std::string objectTopicPrefix = "/vrpn_client_node/";
//        uint numObjects = 0;

//        for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
//            const ros::master::TopicInfo& info = *it;
//            //cout << "ROSTOPIC: " << info.name << endl;
//            if(std::equal(
//                        info.name.begin(),
//                        info.name.begin() + std::min( info.name.size(), objectTopicPrefix.size() ),
//                        objectTopicPrefix.begin() )){
//                numObjects++;
//            }
//        }

//        objects.resize(numObjects);
//        numObjects = 0;

//        for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
//            const ros::master::TopicInfo& info = *it;
//            if(std::equal(
//                        info.name.begin(),
//                        info.name.begin() + std::min( info.name.size(), objectTopicPrefix.size() ),
//                        objectTopicPrefix.begin() )){
//                objects(numObjects) = info.name.substr(objectTopicPrefix.size(),info.name.size() - objectTopicPrefix.size() - 5);
//                numObjects++;
//            }
//        }

//        // add new topics to subscriber map
//        for (uint i= 0; i < objects.size(); i++){
//            auto obName = objects(i);
//            if (objectStates.count(obName)){
//                // already added
//            } else {
//                // new topic
//                auto obTopic = STRING(objectTopicPrefix << obName << "/pose");
//                auto obState = new Var<geometry_msgs::PoseStamped>(obTopic);
//                objectStates[obName] = *obState;
//                sub_objectStates[obName] = ROS.subscribe(objectStates[obName]);
//            }
//        }

//        return objects;
//    }
//    NIY;
//    return {};
//}


