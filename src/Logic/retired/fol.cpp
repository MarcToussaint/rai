/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

/// extracts the preconditions of the rule, then returns substitutions
NodeL getRuleSubstitutions(Graph& facts, Node* rule, NodeL& domain, int verbose) {
  //-- extract precondition
  if(verbose>1) { cout <<"Substitutions for rule " <<*rule <<endl; }
  Graph& Rule=rule->graph();
  return getSubstitutions(facts, getFirstNonSymbolOfScope(Rule)->graph(), domain, verbose);
}

NodeL getSubstitutions(Graph& facts, NodeL& literals, NodeL& domain, int verbose) {
  CHECK(literals.N, "");
  Graph& varScope = literals(0)->container.isNodeOfGraph->container; //this is usually a rule (scope = subGraph in which we'll use the indexing)

  Node* EQ = facts["EQ"];
  NodeL vars = getSymbolsOfScope(varScope);

//  if(!vars.N){
//    cout <<"Substitutions for literals "; listWrite(literals, cout); cout <<" WITHOUT variables!" <<endl;
//    NodeL subs(1u,0u);
//    return subs;
//  }

  if(verbose>2) {
    cout <<"Substitutions for literals "; listWrite(literals, cout); cout <<" with variables '"; listWrite(vars, cout); cout <<'\'' <<endl;
//    cout <<"   with facts " <<facts <<" and domain "; listWrite(domain, cout); cout <<'\'' <<endl;
  }

  //-- initialize potential domains for each variable
  rai::Array<NodeL> domainOf(vars.N);
//  constants.sort(NodeComp);
  for(Node* v:vars) domainOf(v->index) = domain;

  if(verbose>3) cout <<"domains before 'constraint propagation':" <<endl;
  if(verbose>3) for(Node* var:vars) { cout <<"'" <<*var <<"' {"; listWrite(domainOf(var->index), cout); cout <<" }" <<endl; }

  //-- grab open variables for each literal
  uintA lit_numVars(literals(0)->container.N);
  for(Node* literal:literals) lit_numVars(literal->index) = getNumOfVariables(literal, &varScope);

  //-- first pick out all precondition predicates with just one open variable and reduce domains directly
  for(Node* literal:literals) {
    if(lit_numVars(literal->index)==1) {
      Node* var = getFirstVariable(literal, &varScope);
      if(verbose>3) cout <<"checking literal '" <<*literal <<"'" <<flush;
      removeInfeasibleSymbolsFromDomain(facts, domainOf(var->index), literal, &varScope);
      if(verbose>3) { cout <<" gives remaining domain for '" <<*var <<"' {"; listWrite(domainOf(var->index), cout); cout <<" }" <<endl; }
      if(domainOf(var->index).N==0) {
        if(verbose>2) cout <<"NO POSSIBLE SUBSTITUTIONS" <<endl;
        return NodeL(); //early failure
      }
    }
  }

  if(verbose>2) cout <<"domains after 'constraint propagation':" <<endl;
  if(verbose>2) for(Node* var:vars) { cout <<"'" <<*var <<"' {"; listWrite(domainOf(var->index), cout); cout <<" }" <<endl; }

  //-- for the others, create constraints
  NodeL constraints;
  for(Node* literal:literals) {
    if(lit_numVars(literal->index)!=1 || (literal->parents.N && literal->parents(0)==EQ)) {
      constraints.append(literal);
    }
  }

  if(verbose>2) { cout <<"remaining constraint literals:" <<endl; listWrite(constraints, cout); cout <<endl; }

  //-- naive CSP: loop through everything
  uint subN=0;
  NodeL substitutions;
  NodeL values(vars.N); values.setZero();
  {
    //-- using 'getIndexTuple' we can linearly enumerate all configurations of all variables
    uintA domainN(vars.N);
    for(uint i=0; i<vars.N; i++) domainN(i) = domainOf(i).N; //collect dims/cardinalities of domains
    uint configurationsN = product(domainN); //number of all possible configurations
    for(uint config=0; config<configurationsN; config++) { //loop through all possible configurations
      uintA valueIndex = getIndexTuple(config, domainN);
      bool feasible=true;
      for(uint i=0; i<vars.N; i++) values(vars(i)->index) = domainOf(i)(valueIndex(i)); //assign the configuration
      //only allow for disjoint assignments
      for(uint i=0; i<values.N && feasible; i++) for(uint j=i+1; j<values.N && feasible; j++) {
          if(values(i)==values(j)) feasible=false;
        }
      if(!feasible) continue;
      for(Node* literal:constraints) { //loop through all constraints
        if(literal->parents.N && literal->parents(0)==EQ) { //check equality of subsequent literals
          Node* it1 = literal->container(literal->index+1);
          Node* it2 = literal->container(literal->index+2);
          feasible = matchingFactsAreEqual(facts, it1, it2, values, &varScope);
        } else {
          feasible = getEqualFactInKB(facts, literal, values, &varScope);
          if(!feasible) { //when literal is a negative boolean literal and we don't find a match, we interpret this as feasible!
            if(literal->isBoolAndFalse())
              feasible=true;
          }
        }
        if(verbose>3) { cout <<"checking literal '" <<*literal <<"' with args "; listWrite(values, cout); cout <<(feasible?" -- good":" -- failed") <<endl; }
        if(!feasible) break;
      }
      if(feasible) {
        if(verbose>3) { cout <<"adding feasible substitution "; listWrite(values, cout); cout <<endl; }
        substitutions.append(values);
        subN++;
      }
    }
  }
  substitutions.reshape(subN, vars.N);

  if(verbose>1) {
    cout <<"POSSIBLE SUBSTITUTIONS: " <<substitutions.d0 <<endl;
    for(uint s=0; s<substitutions.d0; s++) {
      for(uint i=0; i<substitutions.d1; i++) if(substitutions(s, i)) {
          cout <<varScope(i)->keys(0) <<" -> " <<substitutions(s, i)->keys(1) <<", ";
        }
      cout <<endl;
    }
    if(!substitutions.d0) cout <<"NO POSSIBLE SUBSTITUTIONS" <<endl;
  }
  return substitutions;
}

//TODO: it*->literal*
bool matchingFactsAreEqual(Graph& facts, Node* it1, Node* it2, const NodeL& subst, Graph* subst_scope) {
  CHECK(&it1->container==&it2->container, "");
  if(it1->type!=it2->type) return false;
  if(it1->parents(0)!=it2->parents(0)) return false;

  Node* m1=getEqualFactInKB(facts, it1, subst, subst_scope, false);
  if(!m1) return false;
  Node* m2=getEqualFactInKB(facts, it2, subst, subst_scope, false);
  if(!m2) return false;

  if(m1==m2) return true;
  return m1->hasEqualValue(m2);
}
