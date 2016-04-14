#include <FOL/relationalMachine.h>
#include <Core/module.h>
#include <mlr_srv/StringString.h>

struct ServiceRAP{
  Access_typed<RelationalMachine> RM;
  struct sServiceRAP *s;

  ServiceRAP();
  ~ServiceRAP();

  bool cb_service(mlr_srv::StringString::Request& _request, mlr_srv::StringString::Response& response);
};
