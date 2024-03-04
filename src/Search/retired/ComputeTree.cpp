/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "ComputeTree.h"

#include <math.h>
#include "../Core/util.h"
#include "../Core/graph.h"

static uint CT_Node_ID=0;

template<> const char* rai::Enum<ComputeTree_SolverOptions::SolverMethod>::names [] = {
  "noMethod", "SCE_Thresholded", "SCE_RoundRobin", "SCE_IterativeLimited", nullptr
};

CT_Node::CT_Node(CT_Node* _parent, shared_ptr<rai::TreeSearchNode> _comp)
  : parent(_parent), comp(std::dynamic_pointer_cast<rai::ComputeNode>(_comp)) {
  comp->ID = CT_Node_ID++;
  comp->name.prepend(STRING('#' <<comp->ID <<'_'));
}

void CT_Node::write(std::ostream& os) const {
  os <<comp->name <<" n=" <<c_children <<" R=" <<R;
//  if(comp->isTerminal) os <<" data: " <<D;
}

//===========================================================================

ComputeTree_Solver::ComputeTree_Solver(const shared_ptr<rai::ComputeNode>& _root) : root(0, _root) {
  root.comp->isComplete=true;
  root.comp->isTerminal=false;
  all.append(&root);
  nonTerminals.append(&root);
}

void ComputeTree_Solver::query(CT_Node* n) {
  if(!n) return;

  if(opt.verbose>0) LOG(0) <<"querying " <<n->comp->name;

  //== complete & non-terminal -> create new child, then query it directly
  if(n->comp->isComplete && !n->comp->isTerminal) {
    shared_ptr<CT_Node> child = make_shared<CT_Node>(n, n->comp->transition(n->R));
    child->c_soFar = n->c_soFar+n->comp->c;
    n->R++;
    n->children.append(child);
    all.append(child.get());
    rr_computeFifo.append(child.get());
    if(opt.verbose>0) LOG(0) <<"created new child ID:" <<child->comp->ID <<" of type '" <<rai::niceTypeidName(typeid(*child->comp)) <<"'";
    query(child.get());
    return;
  }

  //== incomplete -> compute
  if(!n->comp->isComplete) {

    n->comp->compute();
    //c_now = time;
    //n->comp->c += time;
    n->comp_n += 1.;
    n->parent->c_children++;

    if(n->comp->isComplete) {
      if(opt.verbose>0) LOG(0) <<"computed " <<n->comp->name <<" -> complete with c=" <<n->comp->c <<" l=" <<n->comp->l;
      CHECK_GE(n->comp->l, 0., "lower bound was not computed");
      if(n->comp->isTerminal) {
        terminals.append(n);
      } else {
        if(n->comp->l<1e10) {
          nonTerminals.append(n);
        } else {
          n->childrenComplete = true;
          n->branchComplete = true;
          CT_Node* p=n;
          while(p) {
            p->y_tot += -1.;
            p->y_num += 1.;
            p=p->parent;
          }
        }
      }
    } else {
      if(opt.verbose>0) LOG(0) <<"computed " <<n->comp->name <<" -> still incomplete with c=" <<n->comp->c;
    }

    //backup compute costs
    CT_Node* p=n->parent;
    while(p) {
      p->c_tot += n->comp->c_now;
      p=p->parent;
    }

    //backup completeness
    if(n->comp->isComplete) {
      CT_Node* p=n->parent;
      bool allComplete=true;
      if(p->comp->getNumDecisions()<0 || (int)p->R<p->comp->getNumDecisions()) {
        allComplete=false; //not all possible children expanded
      } else {
        for(auto& ch:p->children) if(!ch->comp->isComplete) { allComplete=false; break; } //not all children closed
      }
      if(allComplete) p->childrenComplete=true;
    }

    //backup closedness
    if(n->comp->isComplete && n->comp->isTerminal) n->branchComplete=true;
    if(n->branchComplete) {
      CT_Node* p=n->parent;
      while(p) {
        if(p->comp->getNumDecisions()<0 || (int)p->R<p->comp->getNumDecisions()) break; //not all possible children expanded
        for(auto& ch:p->children) if(!ch->branchComplete) { p=0; break; } //not all children closed
        if(!p) break;
        p->branchComplete=true;
        p=p->parent;
      }
    }
  }

  //== complete & terminal -> sample
  if(n->comp->isComplete && n->comp->isTerminal) {
    CHECK(n->comp->isTerminal, "");

    double y = n->comp->sample();
    y_now=y;
    if(y_baseline<0. || y>y_baseline) y_baseline=y;
    if(opt.verbose>0) LOG(0) <<"sampled " <<n->comp->name <<" -> return " <<y;

    while(n) {
      n->y_tot += y;
      n->y_num += 1.;
      n=n->parent;
    }
  }
}

void ComputeTree_Solver::step() {
  y_now=-1.;
  c_now=-1.;
  clearScores();
  CT_Node* n = 0;
  if(opt.method1==ComputeTree_SolverOptions::SCE_Thresholded) { while(!n) n=select_Thresholded(); }
  else if(opt.method1==ComputeTree_SolverOptions::SCE_RoundRobin) { while(!n) n=select_RoundRobin(); }
  else if(opt.method1==ComputeTree_SolverOptions::SCE_IterativeLimited) { while(!n) n=selectBestCompute_IterativeLimited(); }
  else NIY;
  CHECK(n, "");
  n->isSelected = true;
  query(n);
  report();
  steps++;
}

void ComputeTree_Solver::runTrivial(uint k, double maxEffortPerCompute) {
  CT_Node* n = &root;

  for(uint i=0; i<k; i++) {
    query(n);
    report();
    steps++;

    if(n->comp->isComplete) {
      if(n->comp->l>=1e10) {
        if(opt.verbose>0) LOG(0) <<"compute " <<n->comp->name <<" -> *** infeasible with c=" <<n->comp->c;
        n=&root;
      } else {
        if(n->comp->isTerminal) n=&root;
        else n=getCheapestIncompleteChild(n);
      }
    } else {
      if(n->comp->c > maxEffortPerCompute) {
        if(opt.verbose>0) LOG(0) <<"compute " <<n->comp->name <<" -> *** aborted with c=" <<n->comp->c;
        n=&root;
      }
    }

//    if(n==&root){ printTree(cout, root); rai::wait(); }
  }
}

CT_Node* ComputeTree_Solver::getBestSample_Flat() {
  if(!terminals.N) return 0;

  //-- compute UCB1 score for all terminals
  arr score(terminals.N);
  uint i=0;
  for(CT_Node* n:terminals) {
    if(n->y_num>0.) {
//      double parent_num=0;
//      for(auto& ch: n->parent->children) parent_num += ch->y_num;
      n->y_ucb = n->y_tot/n->y_num + opt.beta * ::sqrt(2.*::log(root.y_num) / n->y_num);
    } else {
      n->y_ucb = double(1<<10);
    }
    score(i++) = n->y_ucb;
  }
  if(opt.verbose>0) LOG(0) <<"terminal's data scores: " <<score;

  return terminals(argmax(score));
}

CT_Node* ComputeTree_Solver::getBestSample_UCT() {
  if(!terminals.N) return 0;

  double bestMean=0.;
  for(CT_Node* n:terminals) {
    double y = n->y_tot/n->y_num;
    if(y>bestMean) bestMean=y;
  }
  y_baseline = bestMean;

  //-- select terminal node using tree policy
  CT_Node* n = &root;
  while(!n->comp->isTerminal) {
    //for all children with data compute UCB1 score
    CT_Node* best=0;
    for(auto& n:n->children) {
      if(n->y_num>0.) {
        n->y_ucb = n->y_tot/n->y_num + opt.beta * ::sqrt(2.*::log(n->parent->y_num) / n->y_num);
      } else {
        n->y_ucb = -1.;
      }
      if(!best || n->y_ucb>=best->y_ucb) best=n.get();
      if(n->comp->isTerminal) n->score = n->y_ucb - y_baseline;
    }

    // pick the child with highest
    n = best;
    if(!n) break;
  }
  return n;
}

CT_Node* ComputeTree_Solver::getCheapestIncompleteChild(CT_Node* r) {
  CT_Node* m=0;
  for(auto& ch:r->children) if(!ch->comp->isComplete && (!m || ch->comp->c < m->comp->c)) m=ch.get();
  return m;
}

void ComputeTree_Solver::clearScores() {
  for(CT_Node* n:all) {
    n->score = -1.;
    n->isSelected = false;
    n->isBest = false;
  }
}

CT_Node* ComputeTree_Solver::getBestCompute() {
  CT_Node* best=0;
  for(CT_Node* n:all) {
    if(!n->comp->isComplete) {
      n->score = 1./(n->comp->c + n->comp->effortHeuristic());
      if(!best || n->score>=best->score) best=n;
    }
  }
  if(best) best->isBest=true;
  return best;
}

CT_Node* ComputeTree_Solver::getBestExpand() {
  CT_Node* best=0;
  for(CT_Node* n:all) {
    int Rmax = n->comp->getNumDecisions();
    if(n->comp->isComplete && !n->comp->isTerminal && n->comp->l<1e9 && (Rmax<0 || (int)n->R<Rmax)) {
      n->score = 1./(sqrt(n->y_num+1.) * (n->R+1.) / n->comp->branchingHeuristic());   // * (n->comp->effortHeuristic());
      if(!best || n->score>=best->score) best=n;
    }
  }
  if(best) best->isBest=true;
  return best;
}

CT_Node* ComputeTree_Solver::select_Thresholded() {
  //sample?
  CT_Node* s = getBestSample_UCT();
  //select and threshold
  if(s && s->score>opt.theta) return s;

  if(opt.method2==ComputeTree_SolverOptions::SCE_IterativeLimited) {
    return selectBestCompute_IterativeLimited();
  } else if(opt.method2==ComputeTree_SolverOptions::SCE_RoundRobin) {
    return selectBestCompute_RoundRobin();
  }//else...

  //compute?
  CT_Node* c = getBestCompute();
  if(c && c->comp->c < opt.gamma*sqrt(root.c_tot)) return c; //v1
//  if(c && c->comp->c < gamma*all.N) return c; //v2
//  if(c && root.c_tot < gamma*all.N*all.N) return c; //v3

  //expand?
  CT_Node* e = getBestExpand();
  return e;
}

CT_Node* ComputeTree_Solver::select_RoundRobin() {

  if(rr_sample < opt.rr_sampleFreq*rr_compute) {
    rr_sample++;
    return getBestSample_UCT();
  } else {
    rr_compute++;
    if(opt.method2==ComputeTree_SolverOptions::SCE_IterativeLimited) {
      return selectBestCompute_IterativeLimited();
    } else if(opt.method2==ComputeTree_SolverOptions::SCE_RoundRobin) {
      return selectBestCompute_RoundRobin();
    } else if(opt.method2==ComputeTree_SolverOptions::SCE_Thresholded) {
      //compute?
      CT_Node* c = getBestCompute();
      if(c && c->comp->c < opt.gamma*sqrt(root.c_tot)) return c; //v1
      //  if(c && c->comp->c < gamma*all.N) return c; //v2
      //  if(c && root.c_tot < gamma*all.N*all.N) return c; //v3

      //expand?
      CT_Node* e = getBestExpand();
      return e;

    } else NIY;
  }

  return 0;
}

CT_Node* ComputeTree_Solver::selectBestCompute_IterativeLimited() {
  if(!lifo.N) {
    lifo.memMove=true;
    limit_R++;
    limit_c = opt.gamma*limit_R; //*limit_R;
    lifo.prepend(&root);
    if(opt.verbose>0) LOG(0) <<"expanding to limits R:" <<limit_R <<" c: " <<limit_c;
  }

  //depth first search
  for(; lifo.N;) {
    CT_Node* n = lifo(0);
    if(n->comp->isTerminal && n->comp->isComplete) { lifo.popFirst(); return 0; } //(don't) sample
    if(!n->comp->isComplete) {
      if(n->c_soFar+n->comp->c<limit_c) { //compute
        return n;
      } else {
        lifo.popFirst();
      }
    }
    if(n->comp->isComplete && n->comp->l>=1e9) { //infeasible branch
      n = lifo.popFirst();
      continue;
    }
    int Rmax = n->comp->getNumDecisions();
    if(Rmax<0 || Rmax>double(limit_R)*n->comp->branchingHeuristic()) Rmax = double(limit_R)*n->comp->branchingHeuristic();
    if(n->comp->isComplete && n->R<(uint)Rmax) { //expand
      return n;
    }
    if(n->comp->isComplete) { //go deeper
      n = lifo.popFirst();
      for(uint i=n->children.N; i--;) {
        lifo.prepend(n->children(i).get());
      }
    }
  }

  return 0;
}

CT_Node* ComputeTree_Solver::selectBestCompute_RoundRobin() {
  rr_computeFifo.memMove=true;

  if(rr_computeFifo.N && rr_compComp < opt.rr_computeFreq*rr_compExp) {
    CT_Node* n = 0;
    for(; rr_computeFifo.N;) {
      n = rr_computeFifo(0);
      if(!n->comp->isComplete) break;
      rr_computeFifo.popFirst();
      n=0;
    }
    if(n) {
      CHECK(!n->comp->isComplete, "");
      rr_compComp++;
      rr_computeFifo.shift(-1, true);
      return n;
    }
  }

  rr_compExp++;
  return getBestExpand();
}

void ComputeTree_Solver::report() {
  double bestMean=0.;
  for(CT_Node* n:terminals) {
    double y = n->y_tot/n->y_num;
    if(y>bestMean) bestMean=y;
  }
  if(fil) {
    if(int(totalCost()/10) > int(filLast/10)) {
      (*fil) <<steps <<' ' <<totalCost() <<' ' <<y_now <<' ' <<(root.y_num>0.?root.y_tot/root.y_num:0.) <<' ' <<root.y_num <<' ' <<root.y_tot <<' ' <<bestMean <<' ' <<terminals.N <<' ' <<nonTerminals.N <<endl;
      filLast = totalCost();
    }
  }
}

rai::Array<CT_Node*> getAllNodes(CT_Node& root) {
  rai::Array<CT_Node*> queue;
  queue.append(&root);

  uint i=0;
  while(i<queue.N) {
    CT_Node* n = queue(i);
    n->comp->ID=i;
    i++;

    if(n->parent) CHECK_EQ(queue(n->parent->comp->ID), n->parent, "");

    for(uint j=0; j<n->children.N; j++) {
      queue.append(n->children(j).get());
    }
  }
  return queue;
}

void printTree(std::ostream& os, CT_Node& root) {

  rai::Array<CT_Node*> T = getAllNodes(root);

  rai::Graph G;
  for(uint i=0; i<T.N; i++) {
    CT_Node* n = T(i);
    rai::NodeL par;
    if(n->parent) par.append(G.elem(n->parent->comp->ID));
    rai::Graph& sub = G.addSubgraph(n->comp->name, par);

    sub.add<double>("score", n->score);
    sub.add<double>("c", n->comp->c);
    sub.add<double>("comp_n", n->comp_n);
//    sub.newNode<double>("c_children", {}, n->c_children);
    if(n->comp->l>=0.) {
      sub.add<double>("l", n->comp->l);
    }
//    sub.newNode<double>("R", {}, n->R);
    if(n->comp->l<1e9) {
      if(n->y_num) {
        sub.add<double>("y_mean", n->y_tot/n->y_num);
        sub.add<double>("y_num", n->y_num);
      }
      //    sub.newNode<double>("y_ucb", {}, n->y_ucb);
      //    sub.newNode<double>("C_ucb", {}, n->mean_ucb);
      //    sub.newNode<double>("C_eff", {}, n->eff);
      if(n->c_tot) {
        sub.add<double>("c_tot", n->c_tot);
      }
      if(n->childrenComplete) sub.add<bool>("childrenCpl", true);
      if(n->branchComplete) sub.add<bool>("branchCpl", true);
    }

//    if(n->D.N) sub.newNode<arr>("data", {}, n->D);
    if(!n->comp->isComplete) G.getRenderingInfo(sub.isNodeOfGraph).dotstyle <<", shape=box, style=dashed";
    else if(n->comp->isTerminal) G.getRenderingInfo(sub.isNodeOfGraph).dotstyle <<", shape=box, style=rounded";
    if(n->isSelected) G.getRenderingInfo(sub.isNodeOfGraph).dotstyle <<", color=red";
    else if(n->isBest) G.getRenderingInfo(sub.isNodeOfGraph).dotstyle <<", color=orange";
  }

  G.checkConsistency();
  G.write(FILE("z.tree"));
  G.writeDot(FILE("z.dot"));
  rai::system("dot -Tpdf z.dot > z.pdf");
}

