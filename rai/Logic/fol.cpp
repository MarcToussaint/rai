/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "fol.h"

#define DEBUG(x) //x

//inline bool NodeComp(Node* const& a, Node* const& b){ //TODO: why?
//  return a < b;
//}

/// any node that has a key, no parents, and is bool, is a symbol declaration
bool isSymbol(rai::Node* n) {
  return n->key.N>0 && n->parents.N==0 && n->isOfType<bool>() && n->key(0)!='%';
}

/// given a scope (a subGraph, e.g. the full KB, or a rule or so), return all literals (defined by degree>0, keys.N=0)
NodeL getLiteralsOfScope(Graph& KB) {
  NodeL state;
  state.reserveMEM(KB.N);
  for(Node* i:KB) if(i->parents.N>0) state.append(i);
  return state;
}

/// return all variables (defined by keys.N>0, degree=0, isBoolean)
NodeL getSymbolsOfScope(const Graph& KB) {
  NodeL vars;
  vars.reserveMEM(KB.N);
  for(Node* i:KB) if(isSymbol(i)) vars.append(i);
  return vars;
}

//typically: the preconditions of a rule!
Node* getFirstNonSymbolOfScope(Graph& KB) {
  for(Node* i:KB) if(!isSymbol(i)) return i;
  return nullptr;
}

//typically: the preconditions of a rule!
Node* getSecondNonSymbolOfScope(Graph& KB) {
  uint count=2;
  for(Node* i:KB) if(!isSymbol(i)) {
      count--;
      if(!count) return i;
    }
  return nullptr;
}

/// returns all variables of the literal
NodeL getVariables(Node* literal, Graph* varScope) {
  NodeL vars;
  for(Node* i:literal->parents) if(&i->container==varScope) {
      CHECK(isSymbol(i), "");
      vars.append(i);
    }
  return vars;
}

uint getNumOfVariables(Node* literal, Graph* varScope) {
  uint v=0;
  for(Node* i:literal->parents) if(&i->container==varScope) {
      CHECK(isSymbol(i), "");
      v++;
    }
  return v;
}

Node* getFirstVariable(Node* literal, Graph* varScope) {
  for(Node* i:literal->parents) if(&i->container==varScope) {
      CHECK(isSymbol(i), "");
      return i;
    }
  return nullptr;
}

/// check if these are literally equal (all arguments are identical, be they vars or consts) -- fact1 is only a tuple, not an node of the graph
bool tuplesAreEqual(NodeL& tuple0, NodeL& tuple1) {
  if(tuple0.N!=tuple1.N) return false;
  for(uint i=0; i<tuple0.N; i++) {
    if(tuple0.elem(i)->key=="ANY") continue;
    if(tuple0.elem(i) != tuple1.elem(i)) return false;
  }
  return true;
}

bool valuesAreEqual(Node* fact0, Node* fact1, bool booleanMeansExistance) {
  if(booleanMeansExistance) {
    if(fact0->isBoolAndTrue() && !fact1->isOfType<bool>()) return true;
    if(fact1->isBoolAndTrue() && !fact0->isOfType<bool>()) return true;
  }
  if(fact0->type!=fact1->type) return false;
  if(!fact0->hasEqualValue(fact1)) return false;
  return true;
}

/// two facts are exactly equal (tuplesAreEqual (vars or consts), keys are equal, valuesAreEqual)
bool factsAreEqual(Node* fact0, Node* fact1, bool checkAlsoValue) {
  if(!tuplesAreEqual(fact0->parents, fact1->parents)) return false;
  if(fact0->key!=fact1->key) return false;
  if(checkAlsoValue) return valuesAreEqual(fact0, fact1, true);
  return true;
}

/// after substituting subst in literal it becomes equal to fact [ignoreSubst ignores all variables in literal]
bool factsAreEqual(Node* fact, Node* literal, const NodeL& subst, const Graph* subst_scope, bool checkAlsoValue, bool ignoreSubst) {
  if(fact->parents.N!=literal->parents.N) return false;
  if(fact->key!=literal->key) return false;
  for(uint i=0; i<fact->parents.N; i++) {
    Node* fact_arg = fact->parents.elem(i);
    Node* lit_arg = literal->parents.elem(i);
    if(lit_arg->key=="ANY") {
    } else if(&lit_arg->container==subst_scope) { //lit_arg is a variable -> check match of substitution
      if(!ignoreSubst && subst(lit_arg->index)!=fact_arg) return false;
    } else if(lit_arg!=fact_arg) return false;
  }
  if(checkAlsoValue) return valuesAreEqual(fact, literal, true);
  return true;
}

/// try to find a fact within 'KB' that is exactly equal to 'fact'
bool getEqualFactInKB(Graph& KB, Node* fact, bool checkAlsoValue) {
  if(!fact->parents.N) {
    CHECK(fact->isGraph(), "special literals need Graph type");
    Graph& graph=fact->graph();
    //assume this is a special parent!
    if(fact->key=="aggregate") {
      NodeL subs = getRuleSubstitutions2(KB, fact->graph(), 0);
      if(graph.last()->key=="count") {
        if(subs.d0 == graph.last()->get<double>()) return true;
        else return false;
      } else HALT("unknown aggregate mode '" <<graph.last()->key <<"'");
    } else HALT("unknown special literal key'" <<fact->key <<"'");
  }
#if 0
  //first find the section of all facts that derive from the same symbols
  NodeL candidates=fact->parents(0)->children;
  for(uint p=1; p<fact->parents.N; p++)
    candidates = setSection(candidates, fact->parents(p)->children);
#else
  NodeL& candidates = KB;
#endif
  //now check only these candidates
  for(Node* fact1:candidates) if(&fact1->container==&KB && fact1!=fact) {
      if(factsAreEqual(fact, fact1, checkAlsoValue)) return true;
    }
  return false;
}

/// return the subset of 'literals' that matches with a fact (calling match(lit0, lit1))
Node* getEqualFactInList(Node* fact, NodeL& facts, bool checkAlsoValue) {
//  NodeL candidates=facts;
//  for(Node *p:fact->parents) candidates = setSection(candidates, p->children);
  for(Node* fact1:facts) if(factsAreEqual(fact, fact1, checkAlsoValue)) return fact1; //matches.append(fact1);
  return nullptr;
}

///// try to find a fact within 'facts' that is exactly equal to 'literal'
//bool getEqualFactInKB(Graph& facts, NodeL& fact, bool checkAlsoValue){
//  //first find the section of all facts that derive from the same symbols
//  NodeL candidates=fact(0)->children;
////  for(uint p=1;p<fact.N;p++)
////    candidates = setSection(candidates, fact->parents(p)->children);
//  //now check only these candidates
//  for(Node *fact1:candidates) if(&fact1->container==&facts){
//    if(factsAreEqual(fact1, fact, checkAlsoValue)) return true;
//  }
//  return false;
//}

/// check if subst is a feasible substitution for a literal (by checking with all facts that have same predicate)
bool getEqualFactInKB(Graph& KB, Node* literal, const NodeL& subst, const Graph* subst_scope, bool checkAlsoValue) {
#if 0
  //TODO: this should construct the tuple, call the above method, then additionally checkForValue -> write a method that checks value only
  NodeL candidates = literal->parents(0)->children;
  NodeL candidates = getLiteralsOfScope(KB);
#else
  NodeL& candidates = KB;
#endif
  for(Node* fact:candidates) if(&fact->container==&KB && fact!=literal) {
      if(factsAreEqual(fact, literal, subst, subst_scope, checkAlsoValue)) return true;
    }
  return false;
}

/// find, modulo ignoring variables (i.e., for all possible subst), all facts that match the tuple (from those, possible substitutions can be found)
NodeL getPotentiallyEqualFactsInKB(Graph& KB, Node* tuple, const Graph& varScope, bool checkAlsoValue) {
#if 0
  Node* rarestSymbol=nullptr;
  uint rarestSymbolN=0;
  for(Node* sym:tuple->parents) if(&sym->container!=&varScope) { //loop through all grounded symbols, not variables
      if(!rarestSymbol || sym->children.N<rarestSymbolN) {
        rarestSymbol = sym;
        rarestSymbolN = sym->children.N;
      }
    }
  const NodeL& candidates = rarestSymbol->children;//that totally doesn't scale!!!
#else
  const NodeL& candidates = KB;
#endif
  NodeL matches;
  for(Node* fact:candidates) if(&fact->container==&KB && fact!=tuple) {
      if(factsAreEqual(fact, tuple, NoNodeL, &varScope, checkAlsoValue, true))
        matches.append(fact);
    }
  return matches;
}

/// check if all facts can be matched with one in scope
bool allFactsHaveEqualsInKB(Graph& KB, NodeL& facts, bool checkAlsoValue) {
  for(Node* fact:facts) {
    if(!fact->isOfType<bool>() || fact->get<bool>()==true) { //normal
      if(!getEqualFactInKB(KB, fact, checkAlsoValue)) return false;
    } else { //negated, return false if such a fact exists, independent of its value
      if(getEqualFactInKB(KB, fact, false)) return false;
    }
  }
  return true;
}

/// check if all facts can be matched with one in scope
bool allFactsHaveEqualsInKB(Graph& KB, NodeL& literals, const NodeL& subst, const Graph* subst_scope, bool checkAlsoValue) {
  for(Node* literal:literals) {
    if(!literal->isOfType<bool>() || literal->get<bool>()==true) { //normal
      if(!getEqualFactInKB(KB, literal, subst, subst_scope, checkAlsoValue)) return false;
    } else { //negated, return false if such a fact exists, independent of its value
      if(getEqualFactInKB(KB, literal, subst, subst_scope, false)) return false;
    }
  }
  return true;
}

/// ONLY for a literal with one free variable: remove all infeasible values from the domain
/// this is meant to be used as basic 'constraint propagation' for order-1 constraints
void removeInfeasibleSymbolsFromDomain(Graph& facts, NodeL& domain, Node* literal, Graph* varScope) {
  CHECK_EQ(getNumOfVariables(literal, varScope), 1, " remove Infeasible works only for literals with one open variable!");
  Node* var = getFirstVariable(literal, varScope);
//  Node *predicate = literal->parents(0);

  NodeL dom;
  dom.reserveMEM(domain.N);
  for(Node* fact:facts) { //for(Node *fact:predicate->children) if(&fact->container==&facts){
    //-- check that all arguments are the same, except for var!
    bool match=true;
    Node* value=nullptr;
    if(literal->parents.N != fact->parents.N) {
      match = false;
    }
    if(match) {
      for(uint i=0; i<literal->parents.N; i++) {
        Node* lit_arg = literal->parents(i);
        Node* fact_arg = fact->parents(i);
        if(lit_arg==var) value = fact_arg;
        else if(lit_arg!=fact_arg) { match=false; break; }
      }
    }
    if(match && !literal->isOfType<bool>()) { //if the literal is boolean, we don't YET check the value (see below)
      if(fact->type!=literal->type) match=false;
      match = fact->hasEqualValue(literal);
    }
    if(match) {
      CHECK(value && &value->container==&facts.isNodeOfGraph->container, ""); //the value should be a constant!
//      dom.NodeL::setAppendInSorted(value, NodeComp);
      dom.NodeL::append(value);
    }
  }

  //for a negative boolean literal, we REMOVE the matches instead of allowing for them
  if(literal->isBoolAndFalse()) {
    setMinus(domain, dom);
  } else {
    domain = setSection(domain, dom);
  }
}

/// directly create a new fact
Node* createNewFact(Graph& facts, const NodeL& symbols) {
  return facts.newNode<bool>({}, symbols, true);
}

/// create a new fact by substituting all variables with subst(var->index) (if non-nullptr)
/// add the new literal to KB
Node* createNewSubstitutedLiteral(Graph& facts, Node* literal, const NodeL& subst, Graph* subst_scope) {
  Node* fact = literal->newClone(facts);
  for(uint i=0; i<fact->parents.N; i++) {
    Node* arg=fact->parents(i);
    CHECK(&arg->container==subst_scope || &arg->container==&facts.isNodeOfGraph->container, "the literal argument should be a constant (KB scope) or variable (1st level local scope)");
    if(&arg->container==subst_scope) { //is a variable, and subst exists
      CHECK(subst(arg->index)!=nullptr, "a variable (=argument in local scope) requires a substitution, no?");
      //CHECK_EQ(arg->container.N, subst.N, "somehow the substitution does not fit the container of literal arguments");
//      fact->parents(i) = subst(arg->index);
//arg->numChildren--;//      arg->children.removeValue(fact);
//fact->parents(i)->numChildren++;//      fact->parents(i)->children.append(fact);
      fact->swapParent(i, subst(arg->index));
    }
  }
//  cout <<*fact <<endl;
  return fact;
}

bool applySubstitutedLiteral(Graph& facts, Node* literal, const NodeL& subst, Graph* subst_scope, Graph& changes) {
  if(!literal->parents.N) {
    LOG(-1) <<"trying to apply () literal. aborting" <<endl;
    return false;
  }

  bool trueValue=true; //check if the literal is negated
  if(literal->isBoolAndFalse()) trueValue = false;

  bool hasEffects=false;

  //first collect tuple matches
  NodeL matches;
  for(Node* fact:facts) { //for(Node *fact:literal->parents(0)->children) if(&fact->container==&facts){
    if(factsAreEqual(fact, literal, subst, subst_scope, false)) matches.append(fact);
  }

  if(trueValue) {
    if(!matches.N) {
      Node* newNode = createNewSubstitutedLiteral(facts, literal, subst, subst_scope);
      hasEffects=true;
      if(!!changes) newNode->newClone(changes);
    } else {
      for(Node* m:matches) {
#if 0
        if(m->isOfType<double>()) { //TODO: very special HACK: double add up instead of being assigned
          m->get<double>() += literal->get<double>();
          hasEffects=true;
          if(!!changes) m->newClone(changes);
        } else
#endif
          if(!m->hasEqualValue(literal)) {
            m->copyValue(literal);
            hasEffects=true;
            if(!!changes) m->newClone(changes);
          }
      }
    }
    //TODO: remove double!
  } else {
    //delete all matching facts!
    for(Node* fact:matches) {
      hasEffects=true;
      if(!!changes) { Node* it=fact->newClone(changes); if(it->isOfType<bool>()) it->get<bool>()=false; }
      delete fact;
    }
  }
  return hasEffects;
}

bool applyEffectLiterals(Graph& facts, NodeL& effects, const NodeL& subst, Graph* subst_scope, Graph& changes) {
  bool hasEffects=false;
  for(Node* lit:effects) {
    bool e = applySubstitutedLiteral(facts, lit, subst, subst_scope, changes);
    hasEffects = hasEffects || e;
  }
  return hasEffects;
}

/// extracts the preconditions of the rule, then returns substitutions
NodeL getRuleSubstitutions2(Graph& KB, Graph& rule, int verbose) {
  //-- extract precondition
  if(verbose>1) { cout <<"Substitutions for rule " <<rule <<endl; }
  Graph& preconditions = getFirstNonSymbolOfScope(rule)->graph();
  if(!preconditions.N) return {};
  return getSubstitutions2(KB, preconditions, verbose);
}

/// check whether the precondition of a rule with substitution holds in the KB
bool substitutedRulePreconditionHolds(Graph& KB, Node* rule, const NodeL& subst, int verbose) {
  //-- extract precondition
  if(verbose>1) { cout <<"\n** precondition check for rule " <<*rule <<"\nwith substitution: "; listWrite(subst); cout <<endl; }
  Graph& Rule=rule->graph();
  Graph& precondition = getFirstNonSymbolOfScope(Rule)->graph();
  bool holds=allFactsHaveEqualsInKB(KB, precondition, subst, &Rule, true);
  if(verbose>1) {
    if(!holds) cout <<"precondition does NOT hold in the KB\n" <<KB <<endl;
    if(holds) cout <<"precondition does HOLDS in the KB\n" <<KB <<endl;
  }
  return holds;
}

/// the list of literals is a conjunctive clause (e.g. precondition)
/// all literals must be in the same scope (element of the same subGraph)
/// we return all feasible substitutions of the literal's variables by constants
/// the return value is an array: for every item of the literal's scope:
/// if item=variable the array contains a pointer to the constant
/// if item=non-variable the arrach contains a nullptr pointer

NodeL getSubstitutions2(Graph& KB, NodeL& relations, int verbose) {
  CHECK(relations.N, "");
  Graph& varScope = relations(0)->container.isNodeOfGraph->container; //this is usually a rule (scope = subGraph in which we'll use the indexing)

  NodeL vars = getSymbolsOfScope(varScope);

  if(verbose>2) {
    cout <<"Substitutions for literals "; listWrite(relations, cout); cout <<" with variables '"; listWrite(vars, cout); cout <<'\'' <<endl;
  }

  //-- collect #free variables of relations
  uintA nFreeVars(relations.N);
  for(Node* rel:relations) nFreeVars(rel->index) = getNumOfVariables(rel, &varScope);

  //-- for relations with 0 free variable, simply check
  for(Node* rel:relations) if(nFreeVars(rel->index)==0) {
      if(!rel->isOfType<bool>() || rel->get<bool>()==true) { //normal
        if(!getEqualFactInKB(KB, rel)) {
          if(verbose>2) cout <<"NO POSSIBLE SUBSTITUTIONS (" <<*rel <<" not true)" <<endl;
          return NodeL(); //early failure
        }
      } else { //negated boolean
        bool neg = getEqualFactInKB(KB, rel, false);
        if(neg) {
          if(verbose>2) cout <<"NO POSSIBLE SUBSTITUTIONS (" <<*rel <<" not true)" <<endl;
          return NodeL(); //early failure
        }
      }
    }

  //-- early success if no vars:
  if(!vars.N) return NodeL(1u, 0u);

  //-- collect domains for each variable by checking (marginally) for potentially matching facts
  rai::Array<NodeL> domainOf(vars.N);
  rai::Array<unsigned char> domainIsConstrained(vars.N);
  rai::Array<NodeL> domainsForThisRel(vars.N);
  if(vars.N) domainIsConstrained = false;

  for(Node* rel:relations) if(nFreeVars(rel->index)>0) { //first go through all (non-negated) relations...
      if(!rel->isOfType<bool>() || rel->get<bool>()==true) { //normal (not negated boolean)
        for(auto& d:domainsForThisRel) d.clear();
        NodeL matches = getPotentiallyEqualFactsInKB(KB, rel, varScope, true);
        if(!matches.N) {
          if(verbose>1) cout <<"Relation " <<*rel <<" has no match -> no subst" <<endl;
          return NodeL(); //early failure
        }
        for(uint i=0; i<rel->parents.N; i++) { //add the symbols to the domain
          Node* var = rel->parents(i);
          if(&var->container==&varScope) { //this is a var
            CHECK(var->index<vars.N, "relation '" <<*rel <<"' has variable '" <<var->key <<"' that is not in the scope");
            for(Node* m:matches) domainsForThisRel(var->index).setAppend(m->parents(i)); //setAppend not necessary
          }
        }
        if(verbose>3) {
          cout <<"Relation " <<*rel <<" allows for domains:" <<endl;
          for(Node* var:rel->parents) if(&var->container==&varScope) {
              cout <<"'" <<*var <<"' {"; listWrite(domainsForThisRel(var->index), cout); cout <<" }" <<endl;
            }
        }
        for(uint i=0; i<vars.N; i++) if(domainsForThisRel(i).N) {
            if(domainIsConstrained(i)) {
              domainOf(i) = setSection(domainOf(i), domainsForThisRel(i));
            } else {
              domainOf(i) = domainsForThisRel(i);
              domainIsConstrained(i)=true;
            }
            if(verbose>3) cout <<"domains after 'marginal domain collection':" <<endl;
            if(verbose>3) for(Node* var:vars) { cout <<"'" <<*var <<"' constrained?" <<domainIsConstrained(var->index) <<" {"; listWrite(domainOf(var->index), cout); cout <<" }" <<endl; }
          }
      }
    }

  if(verbose>2) cout <<"final domains after 'marginal domain collection':" <<endl;
  if(verbose>2) for(Node* var:vars) { cout <<"'" <<*var <<"' {"; listWrite(domainOf(var->index), cout); cout <<" }" <<endl; }

  //-- check that every domain is constrained (that is, mentioned by at least SOME relation)
  for(uint i=0; i<vars.N; i++) if(!domainIsConstrained(i)) {
      cout <<"PRECOND:" <<endl;
      listWrite(relations, cout);
      HALT("The domain of variable " <<*vars(i) <<" is unconstrained (infinite, never mentioned,..)");
    }

  //-- for negative relations with 1 variable, delete from the domain
  for(Node* rel:relations) {
    if(nFreeVars(rel->index)==1 && rel->isOfType<bool>() && rel->get<bool>()==false) {
      Node* var = getFirstVariable(rel, &varScope);
      if(verbose>3) cout <<"checking literal '" <<*rel <<"'" <<flush;
      removeInfeasibleSymbolsFromDomain(KB, domainOf(var->index), rel, &varScope);
      if(verbose>3) { cout <<" gives remaining domain for '" <<*var <<"' {"; listWrite(domainOf(var->index), cout); cout <<" }" <<endl; }
      if(domainOf(var->index).N==0) {
        if(verbose>2) cout <<"NO POSSIBLE SUBSTITUTIONS" <<endl;
        return NodeL(); //early failure
      }
    }
  }

  //-- for relations with more than 1 variable, create joint constraints
  NodeL constraints;
  for(Node* rel:relations) if(nFreeVars(rel->index)>1) { // || (literal->parents.N && literal->parents(0)==EQ)){
      constraints.append(rel);
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
      //only allow for disjoint assignments -- DISABLED!
//        for(uint i=0; i<values.N && feasible; i++) for(uint j=i+1; j<values.N && feasible; j++) {
//          if(values(i)==values(j)) feasible=false;
//        }
      if(!feasible) continue;
      for(Node* literal:constraints) { //loop through all constraints
        //         if(literal->parents.N && literal->parents(0)==EQ){ //check equality of subsequent literals
        //           Node *it1 = literal->container(literal->index+1);
        //           Node *it2 = literal->container(literal->index+2);
        //           feasible = matchingFactsAreEqual(facts, it1, it2, values, &varScope);
        //         }else{
        if(literal->isBoolAndFalse()) { //deal differently with false literals
          feasible = getEqualFactInKB(KB, literal, values, &varScope, false); //check match ignoring value
          feasible = !feasible; //invert result
        } else { //normal
          feasible = getEqualFactInKB(KB, literal, values, &varScope);
        }
        //         }
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
          cout <<varScope(i)->key <<" -> " <<substitutions(s, i)->key <<", ";
        }
      cout <<endl;
    }
    if(!substitutions.d0) cout <<"NO POSSIBLE SUBSTITUTIONS" <<endl;
  }
  return substitutions;
}

bool forwardChaining_FOL(Graph& KB, Graph& state, Node* query, Graph& changes, int verbose, int* samplingObservation) {
  NodeL rules = KB.getNodes("Rule");
//  NodeL constants = KB.getNodes("Constant");
  CHECK(state.isNodeOfGraph && &state.isNodeOfGraph->container==&KB, "state must be a node of the KB");
//  Graph& state = KB.get<Graph>("STATE");
  return forwardChaining_FOL(state, rules, query, changes, verbose, samplingObservation);
}

bool forwardChaining_FOL(Graph& state, NodeL& rules, Node* query, Graph& changes, int verbose, int* samplingObservation) {

  for(;;) {
    DEBUG(state.isNodeOfGraph->container.checkConsistency();)
    bool newFacts=false;
    for(Node* rule:rules) {
      if(verbose>1) cout <<"Testing Rule " <<*rule <<endl;
      NodeL subs = getRuleSubstitutions2(state, rule->graph(), verbose);
      for(uint s=0; s<subs.d0; s++) {
        Node* effect = getSecondNonSymbolOfScope(rule->graph());
        Node* probabilities = rule->graph().last();

        if(probabilities->isOfType<arr>()) { //TODO: THIS IS SAMPLING!!! SOMEHOW MAKE THIS CLEAR/transparent/optional or so
          arr p = probabilities->get<arr>();
          uint r = sampleMultinomial(p);
          if(samplingObservation) *samplingObservation = (*samplingObservation)*p.N + r; //raise previous samplings to the factor p.N and add current sampling
          //TODO: also return sampleProbability?
          effect = rule->graph().elem(-1-p.N+r);
        }
        if(verbose>0) { cout <<"*** applying" <<*effect <<" SUBS"; listWrite(subs[s], cout); cout <<endl; }
        bool e = applyEffectLiterals(state, effect->graph(), subs[s], &rule->graph(), changes);
        if(verbose>1) {
          if(e) {
            cout <<"NEW STATE = " <<state <<endl;
            if(!!changes) cout <<"CHANGES = " <<changes <<endl;
          } else cout <<"DID NOT CHANGE STATE" <<endl;
        }
        newFacts |= e;
        if(e && query) {
          if(getEqualFactInKB(state, query)) {
            cout <<"SUCCESS!" <<endl;
            return true;
          }
        }
      }
      if(!subs.d0) {
        if(verbose>1) cout <<"NO NEW STATE for this rule" <<endl;
      }
    }
    if(!newFacts) break;
  }
  if(query) cout <<"FAILED" <<endl;
  return false;
}

/// actually propositional logic:
bool forwardChaining_propositional(Graph& KB, Node* q) {
  DEBUG(KB.checkConsistency();)
  uintA count(KB.N);     count=0;
  boolA inferred(KB.N);  inferred=false;
  NodeL clauses = KB.getNodes("Clause");
  NodeL agenda;
  for(Node* clause:clauses) {
    count(clause->index) = clause->graph().N;
    if(!count(clause->index)) { //no preconditions -> facts -> add to 'agenda'
      agenda.append(clause->parents(0));
    }
  }
  cout <<count <<endl;

  while(agenda.N) {
    Node* s = agenda.popFirst();
    if(!inferred(s->index)) {
      inferred(s->index) = true;
      for(Node* child : s->children) { //all objects that involve 's'
        const Node* clause = child->container.isNodeOfGraph; //check if child is a literal in a clause
        if(clause) { //yes: 's' is a literal in a clause
          CHECK(count(clause->index)>0, "");
          //          if(count(clause->index)>0){ //I think this is always true...
          count(clause->index)--;
          if(!count(clause->index)) { //are all the preconditions fulfilled?
            Node* newFact = clause->parents(0);
            if(newFact==q) return true;
            agenda.append(newFact);
          }
        }
        cout <<count <<endl;
      }
    }
  }
  return false;
}

double evaluateFunction(Graph& func, Graph& state, int verbose) {
  double f=0.;
  for(Node* tree:func) { //trees are additive; within a tree, only one leaf contributes
    if(!tree->isGraph()) continue; //just some annotation
    double ftree=0.;
    Graph& treeG = tree->graph();
    for(Node* leaf:treeG) { //every tree is a list of leafs (terms), each leaf a graph
      if(leaf==treeG.last()) break;
      Graph& leafG = leaf->graph();
      if(verbose>2) LOG(0) <<"testing tree leaf " <<leafG <<endl;
      NodeL subs = getRuleSubstitutions2(state, leafG, 0); //a leaf is like a rule -> can be tested for substitutions
      if(subs.d0) {
//        cout <<"STATE=" <<state <<endl;
        CHECK(leafG.last()->isOfType<double>(), "");
        double fleaf = leafG.last()->get<double>(); //the last attribute is the reward
        ftree += fleaf;
        if(verbose>0) LOG(0) <<"tree leaf HIT " <<leafG <<" with f-value " <<fleaf <<endl;
        break; //if a leaf evaluates true, no further leafs are considered
      }
    }
    CHECK(treeG.last()->isOfType<double>(), "");
    f += treeG.last()->get<double>() * ftree; //the last attribute is a weight of the whole tree
  }
  return f;
}

