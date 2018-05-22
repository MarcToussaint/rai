/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "rrt_planner.h"

#include <Kin/kin.h>
#include <Algo/rrt.h>
#include <KOMO/komo.h>

#include <Gui/opengl.h>
#include <Plot/plot.h>

namespace rai {
struct sRRTPlanner {
  RRTPlanner *p;
  RRT rrt;
  
  sRRTPlanner(RRTPlanner *p, RRT rrt, bool verbose) : p(p), rrt(rrt), verbose(verbose) { };
  
  bool growTowards(RRT& growing, RRT& passive);
  
  bool isFeasible(const arr& q);
  
  uint success_growing;
  uint success_passive;
  
  bool verbose;
};
}

bool rai::sRRTPlanner::isFeasible(const arr& q) {
  arr phi;
  ObjectiveTypeA tt;
  p->problem.configurations(0)->setJointState(q, NoArr);
  p->problem.phi_t(phi, NoArr, tt, 0);
  for(uint i=0; i<phi.N; i++)
    if(tt(i)==OT_ineq && phi(i)>0.) return false;
  return true;
}

bool rai::sRRTPlanner::growTowards(RRT& growing, RRT& passive) {
  arr q;
  if(rnd.uni()<.5) {
    q = p->joint_min + rand(p->problem.world.getJointStateDimension(), 1) % (p->joint_max - p->joint_min);
    q.reshape(q.d0);
  } else {
    q = passive.getRandomNode();
  }
  arr proposal;
  growing.getProposalTowards(proposal, q);
  
  bool feasible = isFeasible(proposal);
  if(feasible) {
    growing.add(proposal);
    arr tmp_prop;
    double d = passive.getProposalTowards(tmp_prop, proposal);
    
    if(d < growing.getStepsize()) {
      growing.getProposalTowards(tmp_prop, proposal); // to actually get the latest point
      success_growing = growing.getNearest();
      success_passive = passive.getNearest();
      return true;
    }
  }
  return false;
}

arr buildTrajectory(RRT& rrt, uint node, bool forward) {
  arr q;
  uint N = rrt.getNode(node).N;
  uint i = 1; // this is not 0, because we do "do...while"
  do {
    q.append(rrt.getNode(node));
    node = rrt.getParent(node);
    
    ++i;
  } while(node);
  // append the root node
  q.append(rrt.getNode(0));
  
  q.reshape(i, N);
  if(forward) {
    q.reverseRows();
  }
  
  return q;
}

rai::RRTPlanner::RRTPlanner(rai::KinematicWorld *G, KOMO &problem, double stepsize, bool verbose) :
  G(G), problem(problem) {
  arr q; G->getJointState(q);
  s = new rai::sRRTPlanner(this, RRT(q, stepsize), verbose);
  joint_min = zeros(G->getJointStateDimension());
  joint_max = ones(G->getJointStateDimension());
}

void drawRRT(RRT rrt) {
  for(uint i=1; i < rrt.getNumberNodes(); ++i) {
    arr line;
    line.append(rrt.getNode(i)); line.reshape(1, line.N);
    line.append(rrt.getNode(rrt.getParent(i)));
    plotLine(line);
  }
}

arr rai::RRTPlanner::getTrajectoryTo(const arr& target, int max_iter) {
  arr q;
  
  if(!s->isFeasible(target))
    return arr(0);
    
  RRT target_rrt(target, s->rrt.getStepsize());
  
  bool found = false;
  uint node0 = 0, node1 = 0;
  
  int iter = 0;
  while(!found) {
    found = s->growTowards(s->rrt, target_rrt);
    if(found) {
      node0 = s->success_growing;
      node1 = s->success_passive;
      break;
    }
    
    found = s->growTowards(target_rrt, s->rrt);
    if(found) {
      node0 = s->success_passive;
      node1 = s->success_growing;
      break;
    }
    if(s->verbose && iter % 20 == 0) std::cout << "." << std::flush;
    if(max_iter && iter >= max_iter) return arr(0);
    iter++;
  }
  if(s->verbose) std::cout << std::endl;
  
  arr q0 = buildTrajectory(s->rrt, node0, true);
  arr q1 = buildTrajectory(target_rrt, node1, false);
  
  // add trajectories
  q.append(q0);
  q.append(q1);
  q.reshape(q0.d0 + q1.d0, q0.d1);
  
  return q;
}

