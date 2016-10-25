#include <FOL/relationalMachine.h>
#include <Core/thread.h>

struct ServiceRAP{
  struct sServiceRAP *s;

  ServiceRAP();
  ~ServiceRAP();
};
