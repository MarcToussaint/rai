#include <FOL/relationalMachine.h>
#include <Core/module.h>

struct ServiceRAP{
  Access_typed<RelationalMachine> RM;
  struct sServiceRAP *s;

  ServiceRAP();
  ~ServiceRAP();
};
