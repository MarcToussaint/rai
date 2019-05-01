/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include <Logic/relationalMachine.h>
#include <Core/thread.h>

struct ServiceRAP {
  struct sServiceRAP *s;
  
  ServiceRAP();
  ~ServiceRAP();
};
