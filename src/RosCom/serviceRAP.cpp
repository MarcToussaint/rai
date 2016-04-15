#include "serviceRAP.h"

#ifdef MLR_ROS
#include <mlr_srv/StringString.h>

#include <ros/ros.h>
struct sServiceRAP{
  ros::NodeHandle nh;
  ros::ServiceServer service;
  bool cb_service(mlr_srv::StringString::Request& _request, mlr_srv::StringString::Response& response);
};

ServiceRAP::ServiceRAP()
  : RM(NULL, "RM"), s(NULL){
  if(mlr::getParameter<bool>("useRos")){
    cout <<"*** Starting ROS Service RAP" <<endl;
    s = new sServiceRAP;
    s->service = s->nh.advertiseService("/RAP/service", &sServiceRAP::cb_service, s);
  }
}

ServiceRAP::~ServiceRAP(){
  if(s) delete s;
 }

bool sServiceRAP::cb_service(mlr_srv::StringString::Request& _request, mlr_srv::StringString::Response& response){
  mlr::String request = _request.str.c_str();
  if(request=="getState"){
    mlr::String str = RM.get()->getState();
    response.str = str.p;
    return true;
  }
  if(request=="getSymbols"){
    mlr::String str;
    str <<RM.get()->getSymbols();
    response.str = str.p;
    return true;
  }

  cout <<"received new effect '" <<request <<"'" <<endl;
  if(!request.N) return false;
  RM.writeAccess();
  RM().applyEffect(request);
  RM().fwdChainRules();
  mlr::String str;
  RM().tmp->write(str," ");
  RM.deAccess();
  if(str.N)
    response.str = str.p;
  else
    response.str = "<NO RESPONSE>";
  return true;
}

#else

struct sServiceRAP{};
ServiceRAP::ServiceRAP() : RM(NULL, "RM"){}
ServiceRAP::~ServiceRAP(){}

#endif
