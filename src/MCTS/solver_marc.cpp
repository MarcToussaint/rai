#include "solver_marc.h"

void MCTS::addRollout(){
  Node *n = &root;
  world.reset_state();

  //-- tree policy
  while(!world.is_terminal_state()){
    if(!n->children.N && !n->N) break; //freshmen -> do not expand
    if(!n->children.N && n->N){ //expand: compute new decisions and add corresponding nodes
      for(const MCTS_Environment::Handle& d:world.get_actions()) new Node(n, d); //this adds a bunch of freshmen for all possible decisions
    }
    n = treePolicy(n);
    n->r = world.transition(n->decision).second;
  }

  //-- rollout
  double Return=0.;
  while(!world.is_terminal_state())  Return += world.transition_randomly().second;

  Return += world.get_terminal_reward();

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

Node* MCTS::treePolicy(Node* n){
  CHECK(n->children.N, "you should have children!");
  CHECK(n->N, "you should not be a freshman!");
  if(n->N>n->children.N){ //we've visited each child at least once
    arr Q = Qfunction(n, +1);      //optimistic Qfunction
    rndUniform(Q, 0., 1e-3, true); //add noise
    return n->children( argmax( Q ) );
  }
  return n->children( n->N-1 ); //else: visit children by their order
}

arr MCTS::Qfunction(Node* n, int optimistic){
  if(!n) n=&root;
  if(!n->children.N) return arr();
  arr Q(n->children.N);
  uint i=0;
  for(Node *ch:n->children){ Q(i) = Qvalue(ch, optimistic); i++; }
  return Q;
}

double MCTS::Qvalue(Node* n, int optimistic){
  if(n->children.N && n->N>n->children.N){ //the child is mature and has children itself
    if(optimistic==+1) return n->Qup;
    if(optimistic== 0) return n->Qme;
    if(optimistic==-1) return n->Qlo;
  }else{
    //the child is premature -> use its on-policy return estimates (and UCB)
    double beta = 2.*sqrt(2.*::log(n->parent?n->parent->N:n->N));
    if(optimistic==+1) return n->Q/n->N + beta/sqrt(n->N);
    if(optimistic== 0) return n->Q/n->N;
    if(optimistic==-1) return n->Q/n->N - beta/sqrt(n->N);
  }
  HALT("");
  return 0.;
}

void MCTS::writeToGraph(Graph& G, Node* n){
  ItemL par;
  if(!n) n=&root; else par.append((Item*)(n->parent->data));
  double q=-10.;  if(n->N) q=n->Q/n->N;
  n->data = new Item_typed<double>(G, {STRING("t"<<n->t <<'N' <<n->N <<'[' <<n->Qlo <<',' <<n->Qme <<',' <<n->Qup <<']')}, par, new double(q), true);
  for(Node *c:n->children) writeToGraph(G, c);
}
