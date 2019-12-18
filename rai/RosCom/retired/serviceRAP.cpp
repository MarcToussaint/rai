/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "serviceRAP.h"

#ifdef RAI_ROS
#include <rai_msgs/StringString.h>

#include <ros/ros.h>
struct sServiceRAP {
  Var<RelationalMachine> RM;
  ros::NodeHandle nh;
  ros::ServiceServer service;
  bool cb_service(rai_msgs::StringString::Request& _request, rai_msgs::StringString::Response& response);

  sServiceRAP() {}
};

ServiceRAP::ServiceRAP() : s(nullptr) {
  if(rai::getParameter<bool>("useRos")) {
    cout <<"*** Starting ROS Service RAP" <<endl;
    s = new sServiceRAP;
    s->service = s->nh.advertiseService("/RAP/service", &sServiceRAP::cb_service, s);
  }
}

ServiceRAP::~ServiceRAP() {
  if(s) delete s;
}

bool sServiceRAP::cb_service(rai_msgs::StringString::Request& _request, rai_msgs::StringString::Response& response) {
  rai::String request = _request.str.c_str();
  if(request=="getState") {
    rai::String str = RM.get()->getState();
    response.str = str.p;
    return true;
  }
  if(request=="getSymbols") {
    rai::String str;
    str <<RM.get()->getSymbols();
    response.str = str.p;
    return true;
  }

  cout <<"received new effect '" <<request <<"'" <<endl;
  if(!request.N) return false;
  RM.writeAccess();
  RM().applyEffect(request);
  RM().fwdChainRules();
  rai::String str;
  RM().tmp->write(str, " ");
  RM.deAccess();
  if(str.N)
    response.str = str.p;
  else
    response.str = "<NO RESPONSE>";
  return true;
}

#else

struct sServiceRAP {};
ServiceRAP::ServiceRAP() : s(nullptr) {}
ServiceRAP::~ServiceRAP() {}

#endif
