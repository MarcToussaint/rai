#ifndef _HEADER_GUARD_RRT_H_
#define _HEADER_GUARD_RRT_H_

#include <Core/array_t.h>

struct RRT{
private:
  struct sRRT* s;
  
public:
  RRT(const arr& q0, double _stepsize);
  double getProposalTowards(arr& q);
  void add(const arr& q);

  //some access routines
  double getStepsize();
  uint getNearest();
  uint getParent(uint i);
  uint getNumberNodes();
  arr getNode(uint i);
  void getRandomNode(arr& q);
  arr getRandomNode();
};


#endif // _HEADER_GUARD_RRT_H_

