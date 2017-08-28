#pragma once

#include <Kin/frame.h>

namespace mlr{

struct Uncertainty{
  Joint *joint;
  arr sigma;

  Uncertainty(Joint *j, Uncertainty *copyUncertainty=NULL);

  void read(const Graph &ats);
};

}//namespace mlr
