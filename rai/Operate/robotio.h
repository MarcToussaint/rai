#pragma once

#include <Core/thread.h>
#include <Kin/kin.h>

enum RobotType { ROB_sim=0, ROB_pr2, ROB_baxter, ROB_kukaWSG };

struct RobotAbstraction{
    virtual ~RobotAbstraction(){}
    virtual bool executeMotion(const StringA& joints, const arr& path, const arr& times, double timeScale=1., bool append=false) = 0;
    virtual void execGripper(const rai::String& gripper, double position, double force=40.) = 0;
    virtual arr getJointPositions(const StringA& joints) = 0;
    virtual StringA getJointNames() = 0;
    virtual void attach(const char *a, const char *b){}
};

struct RobotIO{
    std::shared_ptr<RobotAbstraction> self;
    rai::Enum<RobotType> type;
    Var<double> timeToGo;

    RobotIO(const rai::KinematicWorld& _K, RobotType type);
    ~RobotIO();

    //-- just call virtuals
    bool executeMotion(const StringA& joints, const arr& path, const arr& times, double timeScale=1., bool append=false){
        return self->executeMotion(joints, path, times, timeScale, append); }
    void execGripper(const rai::String& gripper, double position, double force=40.){
        return self->execGripper(gripper, position, force); }
    arr getJointPositions(const StringA& joints){
        return self->getJointPositions(joints); }
    StringA getJointNames(){
        return self->getJointNames(); }
    void attach(const char *a, const char *b){
        return self->attach(a,b);
    }

    void waitForCompletion(){
        timeToGo.get();
        timeToGo.waitForNextRevision(10);
        for(;;){
            if(timeToGo.get()<=0.) break;
            timeToGo.waitForNextRevision(10);
            //      cout <<"ttg=" <<R.timeToGo.get() <<endl;
        }
    }

};

