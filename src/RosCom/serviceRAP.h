#include <FOL/relationalMachine.h>
#include <Core/module.h>
#include <mlr_srv/StringString.h>

struct ServiceRAP{
  Access_typed<RelationalMachine> RM;

  ros::NodeHandle *nh;
  ros::ServiceServer service;

  ServiceRAP();
  ~ServiceRAP();

  bool cb_service(mlr_srv::StringString::Request& _request, mlr_srv::StringString::Response& response);
};
