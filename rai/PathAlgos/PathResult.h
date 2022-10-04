#pragma once

#include "../Core/array.h"

#include <iostream>

struct PathResult{
  arr path;
  arr Xpath;
  int feasible=-1;
  double duration=-1.;
  double cost=-1., ineq=-1., eq=-1.;
  bool ineqFeasible(double eps=1e-3){ return ineq<=eps && eq<=eps; }

  PathResult() {}
  PathResult(bool infeasible) : feasible(0) { CHECK(infeasible==false, ""); }
  PathResult(const arr& _path) : path(_path), feasible(1) { if(!path.N) feasible=0; }

  void write(std::ostream& os) const{
    if(path.N) os <<" path-dim:" <<path.dim();
    if(feasible>=0) os <<" feasible:" <<feasible;
    if(duration>=0) os <<" duration:" <<duration;
    if(cost>=0) os <<" cost:" <<cost <<" ineq:" <<ineq <<" eq:" <<eq;
  }
};
stdOutPipe(PathResult)
