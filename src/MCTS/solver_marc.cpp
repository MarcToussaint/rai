#include "solver_marc.h"

void MCTS::addRollout(int stepAbort){
  int step=0;
  MCTS_Node *n = &root;
  world.reset_state();

  //-- tree policy
  while(!world.is_terminal_state() && (stepAbort<0 || step++<stepAbort)){
    if(!n->children.N && !n->N) break; //freshmen -> do not expand
    if(!n->children.N && n->N){ //expand: compute new decisions and add corresponding nodes
      if(verbose>2) cout <<"****************** MCTS: expanding: computing all decisions for current node and adding them as freshmen nodes" <<endl;
      for(const MCTS_Environment::Handle& d:world.get_actions()) new MCTS_Node(n, d); //this adds a bunch of freshmen for all possible decisions
      n->children.permuteRandomly();
    }else{
      if(verbose>2) cout <<"****************** MCTS: decisions in current node already known" <<endl;
    }
    n = treePolicy(n);
    if(verbose>1) cout <<"****************** MCTS: made tree policy decision" <<endl;
    n->r = world.transition(n->decision).second;
  }

  //-- rollout
  double Return=0.;
  while(!world.is_terminal_state() && (stepAbort<0 || step++<stepAbort)){
    if(verbose>1) cout <<"****************** MCTS: random decision" <<endl;
    Return += world.transition_randomly().second;
  }


  double r = world.get_terminal_reward();
  Return += r;
  if(step>=stepAbort) Return -= 100.;
  if(verbose>0) cout <<"****************** MCTS: terminal state reached; step=" <<step <<" terminal r=" <<r <<" Return=" <<Return <<endl;

  //-- backup
  for(;;){
    if(!n) break;
    n->N++;
    n->R += n->r;   //total immediate reward
    Return += n->r; //add up total return from n to terminal
    n->Q += Return;
    if(n->children.N && n->N>n->children.N){ //propagate bounds
      n->Qup = max( Qfunction(n, +1) );
      n->Qme = max( Qfunction(n,  0) );
      n->Qlo = max( Qfunction(n, -1) );
    }
    n = n->parent;
  }
}

MCTS_Node* MCTS::treePolicy(MCTS_Node* n){
  CHECK(n->children.N, "you should have children!");
  CHECK(n->N, "you should not be a freshman!");
  if(n->N>n->children.N){ //we've visited each child at least once
    arr Q = Qfunction(n, +1);      //optimistic Qfunction
    rndUniform(Q, 0., 1e-3, true); //add noise
    return n->children( argmax( Q ) );
  }
  return n->children( n->N-1 ); //else: visit children by their order
}

arr MCTS::Qfunction(MCTS_Node* n, int optimistic){
  if(!n) n=&root;
  if(!n->children.N) return arr();
  arr Q(n->children.N);
  uint i=0;
  for(MCTS_Node *ch:n->children){ Q(i) = Qvalue(ch, optimistic); i++; }
  return Q;
}

arr MCTS::Qvariance(Node* n){
  if(!n) n=&root;
  if(!n->children.N) return arr();
  arr QV(n->children.N);
  uint i=0;
  for(Node *ch:n->children){ QV(i) = ch->Qup - ch->Qlo; i++; }
  return QV;
}

void MCTS::reportQ(ostream& os, Node* n){
  if(!n) n=&root;
  if(!n->children.N) return;
  uint i=0;
  for(Node *ch:n->children){
    os <<'t' <<ch->t <<'N' <<ch->N <<'[' <<ch->Qlo <<',' <<ch->Qme <<',' <<ch->Qup <<']' <<endl;
    i++;
  }
}

uint MCTS::Nnodes(Node *n, bool subTree){
  if(!n) n=&root;
  if(!subTree) return n->children.N;
  uint i=1;
  for(Node *ch:n->children) i += Nnodes(ch, true);
  return i;
}

double MCTS::Qvalue(Node* n, int optimistic){
  if(false && n->children.N && n->N>n->children.N){ //the child is mature and has children itself
    if(optimistic==+1) return n->Qup;
    if(optimistic== 0) return n->Qme;
    if(optimistic==-1) return n->Qlo;
  }else{
    //the child is premature -> use its on-policy return estimates (and UCB)
    double c = beta*sqrt(2.*::log(n->parent?n->parent->N:n->N));
    if(optimistic==+1) return n->Q/n->N + c/sqrt(n->N);
    if(optimistic== 0) return n->Q/n->N;
    if(optimistic==-1) return n->Q/n->N - c/sqrt(n->N);
  }
  HALT("");
  return 0.;
}

void MCTS::writeToGraph(Graph& G, MCTS_Node* n){
  NodeL par;
  if(!n) n=&root; else par.append((Node*)(n->parent->data));
  double q=-10.;  if(n->N) q=n->Q/n->N;
  n->data = new Node_typed<double>(G, {STRING("t"<<n->t <<'N' <<n->N <<'[' <<n->Qlo <<',' <<n->Qme <<',' <<n->Qup <<']')}, par, new double(q), true);
  for(MCTS_Node *c:n->children) writeToGraph(G, c);
}
