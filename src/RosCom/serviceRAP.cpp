#include "serviceRAP.h"

ServiceRAP::ServiceRAP()
  : RM(NULL, "RM"), nh(NULL){
  if(mlr::getParameter<bool>("useRos")){
    cout <<"*** Starting ROS Service RAP" <<endl;
    nh = new ros::NodeHandle;
    service = nh->advertiseService("/RAP/service", &ServiceRAP::cb_service, this);
  }
}

ServiceRAP::~ServiceRAP(){
  if(nh) delete nh;
 }

bool ServiceRAP::cb_service(mlr_srv::StringString::Request& _request, mlr_srv::StringString::Response& response){
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
