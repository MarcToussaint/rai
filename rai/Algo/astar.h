/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once
#include <Core/graph.h>
#include <Algo/priorityQueue.h>

//===========================================================================

template<class NodeType>
struct AStarOnGraph {
  Graph& G;
  NodeType* start, *goal;
  PriorityQueue<NodeType*> queue;
  rai::Array<NodeType*> solutions;
  uint iters=0;

  AStarOnGraph(Graph& _G, NodeType* _start, NodeType* _goal)
    :G(_G), start(_start), goal(_goal) {
    rai::Array<NodeType*>::memMove = true;
    start->astar_g = 0;
    double f = start->astar_heuristic(goal);
    queue.add(f, start);
  }

  bool step() {
    if(!queue.N) {
      LOG(-1) <<"AStarOnGraph: queue is empty -> failure?";
      return false;
    }
    //pop
    NodeType* node =  queue.pop();
    //loop/graph check
    if(!node->astar_isClosed) {
      //goal check
      if(node==goal) return true;
      //expand
      rai::Array<NodeType*> N = getNeighbors(node);
      for(NodeType* child:N) {
        double cost = node->astar_g + child->astar_cost(node);
        if(child->isFeasible || child==goal) {
          if(cost < child->astar_g) {
            child->astar_g = cost;
            child->astar_parent = node;
            double f = child->astar_g + child->astar_heuristic(goal);
            queue.add(f, child, true);
          }
        }
      }
    }
    return false;
  }

  void run() {
    for(;;) {
      if(step()) break;
      iters++;
    }
    NodeType* n=goal;
    cout <<"ASTAR costs:" <<n->astar_g;
    uint steps=0;
    for(;;) {
      n->astar_isOnPath = true;
      if(n==start) break;
      n = n->astar_parent;
      CHECK(n, "something is wrong");
      CHECK(n->isFeasible, "");
      steps++;
    }
    cout <<" steps:" <<steps <<" iterations:" <<iters <<endl;
  }

  void reportQueue();
};

//===========================================================================

