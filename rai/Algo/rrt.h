/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#ifndef _HEADER_GUARD_RRT_H_
#define _HEADER_GUARD_RRT_H_

#include <Core/array.tpp>

struct RRT{
private:
  struct sRRT* s;
  
public:
  RRT(const arr& q0, double _stepsize);
  double getProposalTowards(arr& proposal, const arr& q);
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

