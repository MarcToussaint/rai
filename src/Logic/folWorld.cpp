/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "folWorld.h"

#include "fol.h"

#define DEBUG(x) //x

namespace rai {

NodeL FOL_World::Decision::getTuple() const {
  NodeL t;
  if(rule) t.append(rule);
  for(uint i=0; i<substitution.N; i++) t.append(substitution.elem(i));
  return t;
}

void FOL_World::Decision::write(ostream& os) const {
  if(waitDecision) {
    os <<"(WAIT)";
  } else {
#if 0
    os <<"RULE '" <<rule->keys(1) <<"' SUB ";
    Graph& r=rule->graph();
    for(uint i=0; i<substitution.N; i++) {
      os <<r.elem(i)->key <<'/' <<substitution.elem(i)->key <<' ';
    }
#else
    os <<'(' <<rule->key;
    for(uint i=0; i<substitution.N; i++) { os <<' ' <<substitution.elem(i)->key; }
    os <<')' <<std::flush;
#endif
  }
}

FOL_World::FOL_World()
  : hasWait(false), gamma(1.), stepCost(1.), timeCost(1.), deadEndCost(100.), maxHorizon(100),
    state(nullptr), lastDecisionInState(nullptr), verbose(0), verbFil(0),
    lastStepReward(0.), lastStepDuration(0.), lastStepProbability(1.), lastStepObservation(0), count(0) {
  KB.isDoubleLinked=false;
}

FOL_World::FOL_World(const char* filename) : FOL_World() {
  init(filename);
}

void FOL_World::init(const char* filename) {
  FileToken file(filename, true);
  init(Graph(file));
  file.cd_start();
}

void FOL_World::init(const Graph& _KB) {
  KB = _KB;
  KB.checkConsistency();

  start_state = KB.find<Graph>("START_STATE"); if(!start_state) start_state = &KB.add<Graph>("START_STATE")->value;
  rewardFct = KB.find<Graph>("REWARD");
  worldRules = KB.getNodesWithTag("%Rule");
  decisionRules = KB.getNodesWithTag("%DecisionRule");
  Terminate_keyword = KB["Terminate"];  if(!Terminate_keyword) Terminate_keyword = KB.add<bool>("Terminate"); CHECK(Terminate_keyword, "You need to declare the Terminate keyword");
  Quit_keyword = KB["QUIT"];            if(!Quit_keyword) Quit_keyword = KB.add<bool>("QUIT");   CHECK(Quit_keyword, "You need to declare the QUIT keyword");
  Wait_keyword = KB["WAIT"];            //CHECK(Wait_keyword, "You need to declare the WAIT keyword");
  Subgoal_keyword = KB["SubgoalDone"];            //CHECK(Wait_keyword, "You need to declare the WAIT keyword");
  Quit_literal = KB.add<bool>(0, true, {Quit_keyword});
  if(Subgoal_keyword) {
    Subgoal_literal = KB.add<bool>("tmp", true, {Subgoal_keyword});
  }

  Graph* params = KB.find<Graph>("FOL_World");
  if(params) {
    hasWait = params->get<bool>("hasWait", hasWait);
    gamma = params->get<double>("gamma", gamma);
    stepCost = params->get<double>("stepCost", stepCost);
    timeCost = params->get<double>("timeCost", timeCost);
    deadEndCost = params->get<double>("deadEndCost", deadEndCost);
    maxHorizon = (uint)params->get<double>("maxHorizon", maxHorizon);
  }

  if(verbose>1) {
    cout <<"****************** FOL_World: creation info:" <<endl;
    cout <<"*** start_state=" <<*start_state <<endl;
    cout <<"*** reward fct=" <<*rewardFct <<endl;
    cout <<"*** worldRules = "; rai::listWrite(worldRules, cout); cout <<endl;
    cout <<"*** decisionRules = "; rai::listWrite(decisionRules, cout, "\n"); cout <<endl;
  }

  if(verbFil) {
    open(fil, "z.FOL_World");
  }

  start_T_step=0;
  start_T_real=0.;
//  reset_state();
}

FOL_World::~FOL_World() {
}

TreeSearchDomain::TransitionReturn FOL_World::transition(const Handle& action) {
  lastStepReward = -stepCost;
  lastStepDuration = 0.;
  lastStepProbability = 1.;
  lastStepObservation = 0;

  T_step++;

  CHECK(!hasWait || Wait_keyword, "if the FOL uses wait, the WAIT keyword needs to be declared");

  if(verbose>2) cout <<"****************** FOL_World: step " <<T_step <<endl;
  if(verbose>2) { cout <<"*** pre-state = "; state->write(cout, " "); cout <<endl; }

  //-- get the decision
  const Decision* d = std::dynamic_pointer_cast<const Decision>(action).get();
  if(verbose>2) { cout <<"*** decision = ";  d->write(cout); cout <<endl; }

  //-- remove state annotations from state, if exists
  for(uint i=state->N; i--;) {
    Node* n=state->elem(i);
    if(n->key.N) delete n;
  }

  //-- add the decision as a fact
  if(!d->waitDecision) {
    NodeL decisionTuple = {d->rule};
    decisionTuple.append(d->substitution);
    lastDecisionInState = createNewFact(*state, decisionTuple);
    lastDecisionInState->key = "decision";
  } else {
    lastDecisionInState = createNewFact(*state, {Wait_keyword});
    lastDecisionInState->key = "decision";
  }

  //-- apply effects of decision
  if(d->waitDecision) {
    CHECK(hasWait, "");

    //-- find minimal wait time
    double w=1e10;
    for(Node* i:*state) {
      if(i->is<double>()) {
        double wi = i->as<double>();
        if(w>wi) w=wi;
      }
    }
    if(verbose>2) cout <<"*** real time progress = " <<w <<endl;

    if(w==1e10) {
      if(verbose>2) cout <<"*** NOTHING TO WAIT FOR!" <<endl;
      lastStepDuration = 1.;
    } else {
      //-- subtract w from all times and collect all activities with minimal wait time
      NodeL terminatingActivities;
      for(Node* i:*state) {
        if(i->is<double>()) {
          double& wi = i->as<double>(); //this is a double reference!
          wi -= w;
          if(fabs(wi)<1e-10) terminatingActivities.append(i);
        }
      }

      //-- for all these activities call the terminate operator
      for(Node* act:terminatingActivities) {
        NodeL symbols;
        symbols.append(Terminate_keyword);
        symbols.append(act->parents);
        createNewFact(*state, symbols);
      }

      lastStepDuration = w;
    }
  } else { //normal decision
    //first check if probabilistic
    Node* effect = getSecondNonSymbolOfScope(d->rule->graph());
    Node* probabilities = d->rule->graph().last();

    if(probabilities->is<arr>()) {
      HALT("probs in decision rules not properly implemented (observation id is not...)");
      arr p = probabilities->as<arr>();
      uint r = sampleMultinomial(p);
      lastStepProbability = p(r);
      lastStepObservation = lastStepObservation*p.N + r; //raise previous observations to the factor p.N and add current decision
      effect = d->rule->graph().elem(-1-p.N+r);
    } else {
      lastStepProbability = 1.;
    }
    if(verbose>2) { cout <<"*** effect =" <<*effect <<" SUB"; rai::listWrite(d->substitution, cout); cout <<endl; }
    applyEffectLiterals(*state, effect->graph(), d->substitution, &d->rule->graph());

    if(!hasWait) lastStepDuration = 1.;
  }

  T_real += lastStepDuration;
  lastStepReward -= lastStepDuration*timeCost; //cost per real time

  //-- generic world transitioning
  forwardChaining_FOL(*state, worldRules, nullptr, NoGraph, verbose-3, &lastStepObservation);

  //-- check for QUIT
  successEnd = getEqualFactInKB(*state, Quit_literal);
  deadEnd = (T_step>maxHorizon);

  //-- check for rewards
  if(rewardFct) {
    lastStepReward += evaluateFunction(*rewardFct, *state, verbose-3);
  } else {
    if(successEnd) lastStepReward += 100.;
  }

  if(deadEnd) lastStepReward -= deadEndCost;

  if(verbose>2) { cout <<"*** post-state = "; state->write(cout, " "); cout <<endl; }
  if(verbFil) {
    fil <<"--\n  T_step=" <<T_step;
    fil <<"\n  decision="; d->write(fil);
    fil <<"\n  T_real=" <<T_real;
    fil <<"\n  observation=" <<lastStepObservation;
    fil <<"\n  reward=" <<lastStepReward;
    fil <<"\n  state="; state->write(fil, " ", "{}"); fil <<endl;
  }

  R_total += lastStepReward;

  return { Handle(new Observation(lastStepObservation)), lastStepReward, lastStepDuration };
}

const Array<TreeSearchDomain::Handle> FOL_World::get_actions() {
  CHECK(state, "you need to set the state first! (e.g., reset_state)");
  if(verbose>2) cout <<"****************** FOL_World: Computing possible decisions" <<std::flush;
  Array<Handle> decisions; //tuples of rule and substitution
  if(hasWait) {
    decisions.append(Handle(new Decision(true, nullptr, {}, decisions.N))); //the wait decision (true as first argument, no rule, no substitution)
  }
  for(Node* rule:decisionRules) {
    if(verbose>3) cout <<"\n-- # checking rule " <<*rule <<endl;
    NodeL subs = getRuleSubstitutions2(*state, rule->graph(), verbose-3);
    for(uint s=0; s<subs.d0; s++) {
      decisions.append(Handle(new Decision(false, rule, subs[s], decisions.N))); //a grounded rule decision (abstract rule with substution)
    }
  }
  if(verbose>2) cout <<"-- # possible decisions: " <<decisions.N <<endl;
  if(verbose>3) for(Handle& d:decisions) { d.get()->write(cout); cout <<endl; }
//    cout <<"rule " <<d.first->keys(1) <<" SUB "; listWrite(d.second, cout); cout <<endl;
  return decisions;
}

bool FOL_World::is_feasible_action(const TreeSearchDomain::Handle& action) {
  const Decision* d = std::dynamic_pointer_cast<const Decision>(action).get();
  return substitutedRulePreconditionHolds(*state, d->rule, d->substitution);
}

const TreeSearchDomain::Handle FOL_World::get_stateCopy() {
  return std::make_shared<const State>(createStateCopy(), *this);
}

void FOL_World::set_state(const TreeSearchDomain::Handle& _state) {
  const State* s = std::dynamic_pointer_cast<const State>(_state).get();
  CHECK(s, "the given handle was not a FOL_World::State handle");
  setState(s->state, s->T_step);
  T_real = s->T_real;
}

bool FOL_World::is_terminal_state() const {
  if(deadEnd) {
    if(verbose>0) cout <<"************* FOL_World: DEAD END STATE (T_steps=" <<T_step <<", T_real="<<T_real <<") ************" <<endl;
    if(verbose>1) { cout <<"*** FINAL STATE = "; state->write(cout, " "); cout <<endl; }
    if(verbFil) {
      (*((ofstream*)&fil)) <<"--\n  DEAD END STATE";
      (*((ofstream*)&fil)) <<"\n  total reward=" <<R_total <<endl;
    }
    return true;
  }
  //-- test the terminal state
  if(successEnd) {
    if(verbose>0) cout <<"************* FOL_World: SUCCESS STATE FOUND (T_steps=" <<T_step <<", T_real="<<T_real <<") ************" <<endl;
    if(verbose>1) { cout <<"*** FINAL STATE = "; state->write(cout, " "); cout <<endl; }
    if(verbFil) {
      (*((ofstream*)&fil)) <<"--\n  SUCCESS STATE";
      (*((ofstream*)&fil)) <<"\n  total reward=" <<R_total <<endl;
    }
    return true;
  }
  return false;
}

void FOL_World::make_current_state_new_start() {
  if(!start_state) start_state = &KB.addSubgraph("START_STATE", state->isNodeOfGraph->parents);
  state->index();
  start_state->copy(*state);
  start_state->isNodeOfGraph->key="START_STATE";
  start_T_step = T_step;
  start_T_real = T_real;
  DEBUG(KB.checkConsistency();)
  if(verbose>1) cout <<"****************** FOL_World: reassign start state" <<endl;
  if(verbose>1) { cout <<"*** start_state = "; start_state->write(cout, " "); cout <<endl; }
  if(verbFil) {
    fil <<"*** reassign start state ***" <<endl;
    fil <<"  start_state="; start_state->write(fil, " ", "{}"); fil <<endl;
  }
}

void FOL_World::reset_state() {
  DEBUG(FILE("z.before") <<KB;)
  T_step=start_T_step;
  T_real=start_T_real;
  R_total=0.;
  deadEnd=false;
  successEnd=false;

#if 1
  setState(start_state);
#else
  if(!state) state = &KB.newSubgraph({"STATE"}, {start_state->isNodeOfGraph})->value;
  state->copy(*start_state);
  DEBUG(KB.checkConsistency();)
#endif

  DEBUG(KB.checkConsistency();)
  DEBUG(FILE("z.after") <<KB;)

  //-- forward chain rules
  forwardChaining_FOL(KB, KB.get<Graph>("STATE"), nullptr, NoGraph, verbose-3); //, &decisionObservation);

  //-- check for terminal
//  successEnd = allFactsHaveEqualsInKB(*state, *terminal);
  successEnd = getEqualFactInKB(*state, Quit_literal);

  if(verbose>1) cout <<"****************** FOL_World: reset_state" <<endl;
  if(verbose>1) { cout <<"*** state = "; state->write(cout, " "); cout <<endl; }

  if(verbFil) {
    fil <<"*** reset ***" <<endl;
    fil <<"  T_step=" <<T_step <<"\n  T_real=" <<T_real <<endl;
    fil <<"  state="; state->write(fil, " ", "{}"); fil <<endl;
  }
}

bool FOL_World::get_info(InfoTag tag) const {
  switch(tag) {
    case hasTerminal: return true;
    case isDeterministic: return true;
    case hasMaxReward: return true;
    case hasMinReward: return true;
    case isMarkov: return true;
    case writeState: {
      cout <<"INFO: deadEnd=" <<deadEnd <<" successEnd=" <<successEnd <<" T_step=" <<T_step <<" T_real=" <<T_real <<" R_total=" <<R_total <<" state=" <<endl;
      state->write(cout, " ", "{}");
      return true;
    }
    default: HALT("unknown tag" <<tag);
  }
}

double FOL_World::get_info_value(InfoTag tag) const {
  switch(tag) {
    case getGamma: return gamma;
    case getMaxReward: return 100.;
    case getMinReward: return -deadEndCost;
    default: HALT("unknown tag" <<tag);
  }
}

void FOL_World::write_state(ostream& os) {
  state->write(os, " ", "{}");
}

void FOL_World::set_state(String& s) {
  state->clear();
  s >>PARSE("{");
  state->read(s);
}

Graph* FOL_World::getState() {
  return state;
}

void FOL_World::setState(Graph* s, int setT_step) {
  CHECK(s, "can't set state to nullptr graph");
  if(state) {
    CHECK(s->isNodeOfGraph != state->isNodeOfGraph, "you are setting the state to itself");
  }
  if(!state) state = &KB.addSubgraph("STATE", {s->isNodeOfGraph});
  state->copy(*s);
  DEBUG(KB.checkConsistency();) {
    //the old state hat a parent: its predecessor; this was copied to the new state
    //but the new state needs the old state as parent -> swap parents
    Node* n=state->isNodeOfGraph;
    CHECK_EQ(n->parents.N, 1, "");
    n->swapParent(0, s->isNodeOfGraph);
  }
  if(setT_step>=0) T_step = setT_step;
  DEBUG(KB.checkConsistency();)
  CHECK(state->isNodeOfGraph && &state->isNodeOfGraph->container==&KB, "");
  deadEnd=successEnd=false;
}

Graph* FOL_World::createStateCopy() {
  CHECK(state, "state is not initialized");
  Graph* new_state = &KB.addSubgraph(STRING("STATE_"<<count++), state->isNodeOfGraph->parents);
  state->index();
  new_state->copy(*state);
  return new_state;
}

void FOL_World::writePDDLdomain(std::ostream& os, const char* domainName) const {
  os <<"(define (domain " <<domainName;

  //collect all predicate symbols used in the rules
  CHECK(KB.isIndexed, "");
  boolA used(KB.N);
  used=false;
  NodeL predicates;
  for(Node* rule:decisionRules) {
    Graph& Rule = rule->graph();
    Graph& precond = getFirstNonSymbolOfScope(Rule)->graph(); //Rule.elem(-2)->graph();
    Graph& effect = getSecondNonSymbolOfScope(Rule)->graph(); //Rule.elem(-1)->graph();
    for(Node* n:precond) {
      if(n->key.N) continue; //no temporary facts
      uint ID = n->parents(0)->index;
      if(!used(ID)) { predicates.setAppend(n); used(ID)=true; }
    }
    for(Node* n:effect) {
      if(n->key.N) continue; //no temporary facts
      uint ID = n->parents(0)->index;
      if(!used(ID)) { predicates.setAppend(n); used(ID)=true; }
    }
  }
  //output them
  os <<")\n   (:predicates";
  for(Node* n:predicates) {
    os <<" (" <<n->parents(0)->key;
    for(uint i=1; i<n->parents.N; i++) os <<" ?" <<n->parents(i)->key;
    os <<')';
  }

  for(Node* rule:decisionRules) {
    os <<")\n   (:action " <<rule->key;

    Graph& Rule = rule->graph();
    Graph& precond = getFirstNonSymbolOfScope(Rule)->graph(); //Rule.elem(-2)->graph();
    Graph& effect = getSecondNonSymbolOfScope(Rule)->graph(); //Rule.elem(-1)->graph();

    os <<"\n      :parameters (";
    for(Node* n:Rule) if(isSymbol(n)) os <<" ?" <<n->key;

    os <<")\n      :precondition (and";
    for(Node* n:precond) {
      if(n->key.N) continue; //no temporary facts
      bool neg = n->is<bool>() && !n->as<bool>();
      if(neg) os <<" (not";
      os <<" (" <<n->parents(0)->key;
      for(uint i=1; i<n->parents.N; i++) os <<" ?" <<n->parents(i)->key;
      os <<')';
      if(neg) os <<')';
    }

    os <<")\n      :effect (and";
    for(Node* n:effect) {
      if(n->key.N) continue; //no temporary facts
      bool neg = n->is<bool>() && !n->as<bool>();
      if(neg) os <<" (not";
      os <<" (" <<n->parents(0)->key;
      for(uint i=1; i<n->parents.N; i++) os <<" ?" <<n->parents(i)->key;
      os <<')';
      if(neg) os <<')';
    }

    os <<')';
  }
  os <<")\n)" <<endl;
}

void FOL_World::writePDDLproblem(std::ostream& os, const char* domainName, const char* problemName) const {
  os <<"(define (problem " <<problemName;

  os <<")\n   (:domain " <<domainName;

  //collect all constants used in the start state
  CHECK(KB.isIndexed, "");
  boolA used(KB.N);
  used=false;
  NodeL constants;
  {
    for(Node* n:*start_state) {
      if(n->key.N) continue; //no temporary facts
      for(uint i=1; i<n->parents.N; i++) {
        uint ID = n->parents(i)->index;
        if(!used(ID)) { constants.setAppend(n->parents(i)); used(ID)=true; }
      }
    }
  }
  //output them
  os <<")\n   (:objects";
  for(Node* n:constants) os <<' ' <<n->key;

  //-- start state
  os <<")\n   (:init";
  for(Node* n:*start_state) os <<' ' <<*n;

  //-- terminal rules
  os <<")\n   (:goal"; // add an " (or" here if PDDL would support disjunction of goals
  uint numGoals=0;
  for(Node* rule:worldRules) {
    Graph& Rule = rule->graph();
    if(Rule.elem(-1)->is<arr>()) continue; //this is a probabilistic rule!
    Graph& precond = getFirstNonSymbolOfScope(Rule)->graph(); //Rule.elem(-2)->graph();
    Graph& effect = getSecondNonSymbolOfScope(Rule)->graph(); //Rule.elem(-1)->graph();

    if(effect.N==1 && effect(0)->parents.N==1 && effect(0)->parents(0)==Quit_keyword) { //this is a termination rule
      os <<" (and";
      CHECK(!numGoals, "downward (in standard config) doesnt work for multiple goals!");
      numGoals++;
      for(Node* n:precond) {
        bool neg = n->is<bool>() && !n->as<bool>();
        if(neg) os <<" (not";
        os <<' ' <<*n;
//        os <<" (";
//        for(uint i=0;i<n->parents.N;i++) os <<n->parents(i)->key;
//        os <<')';
        if(neg) os <<')';
      }
      os <<')';
    }
  }
  os <<")\n)" <<endl;
}

void FOL_World::writePDDLfiles(const String& name) {
  ofstream f1(name+".domain.pddl");
  ofstream f2(name+".problem.pddl");
  writePDDLdomain(f1, name+"-domain");
  writePDDLproblem(f2, name+"-domain", name+"-problem");
}

String FOL_World::callPDDLsolver() {
  writePDDLfiles("z");

  String cmd = "~/git/downward/fast-downward.py";
  cmd <<" --plan-file z.sas_plan";
  cmd <<" z.domain.pddl z.problem.pddl";
  cmd <<" --landmarks 'lm=lm_hm(m=2)' --search 'astar(ff(transform=no_transform(), cache_estimates=true))'";

  rai::system(cmd);

  String plan(FILE("z.sas_plan"));

  //cut the last line comment with ';'
  uint i=plan.N;
  for(; i--;) if(plan(i)==';') break;
  plan.resize(i, true);

  cout <<"FOUND PLAN: " <<plan << endl;

  return plan;
}

void FOL_World::report(std::ostream& os) const {
  os <<"FOL_World info:"
     <<"\n  decisions: ";
  for(Node* n:decisionRules) os <<n->key <<", ";
  os <<"\n  rules: ";
  for(Node* n:worldRules) os <<n->key <<", ";
  os <<"\n  start state: ";
  start_state->write(os, " ");
  os <<endl;
}

Node* FOL_World::addSymbol(const char* name) {
  return KB.add<bool>(name, true);
}

void FOL_World::addFact(const StringA& symbols) {
  NodeL parents;
  for(const String& s:symbols) {
    Node* sym = KB[s];
    if(!sym) sym=addSymbol(s);
    parents.append(sym);
    CHECK(parents.last(), "Node '" <<s <<"' was not declared");
  }
  start_state->add<bool>(0, true, parents);
}

void FOL_World::addAgent(const char* name) {
  addFact({"agent", name});
  addFact({"free", name});
}

void FOL_World::addObject(const char* name) {
  addFact({"object", name});
}

void FOL_World::addTerminalRule(const char* literals) {
  //first create a new rule
  Graph& rule = KB.addSubgraph("Rule");
  worldRules.append(rule.isNodeOfGraph);
  Graph& preconditions = rule.addSubgraph();
  Graph& effect = rule.addSubgraph();
  effect.add<bool>(0, true, {Quit_keyword}); //adds the (QUIT) to the effect

  preconditions.read(STRING(literals));
//  LOG(0) <<"CREATED TERMINATION RULE:" <<*rule.isNodeOfGraph;
}

void FOL_World::addTerminalRule(const StringAA& literals) {
  //first create a new rule
  Graph& rule = KB.addSubgraph("Rule");
  worldRules.append(rule.isNodeOfGraph);
  Graph& preconditions = rule.addSubgraph();
  Graph& effect = rule.addSubgraph();
  effect.add<bool>(0, true, {Quit_keyword}); //adds the (QUIT) to the effect

  for(const StringA& lit:literals) {
    NodeL parents;
    for(const String& s:lit) parents.append(KB[s]);
    preconditions.add<bool>(0, true, parents);
  }

  cout <<"CREATED RULE NODE:" <<*rule.isNodeOfGraph <<endl;
}

void FOL_World::addDecisionSequence(std::istream& is) {
  Graph& seq = KB.addSubgraph("Decisions");
  seq.read(is);
  cout <<"CREATED DECISION SEQUENCE:" <<*seq.isNodeOfGraph <<endl;
}

FOL_World_State::FOL_World_State(FOL_World& L, TreeSearchNode* _parent, bool _isTerminal)
  : TreeSearchNode(_parent), L(L), state(L.createStateCopy()), T_step(L.T_step), T_real(L.T_real), R_total(L.R_total) {
  isTerminal = _isTerminal;
  isFeasible = true;
  isComplete = true;
  if(!isTerminal) {
    actions = L.get_actions();
  }
}

std::shared_ptr<TreeSearchNode> FOL_World_State::transition(int action) {
  CHECK_GE(action, 0, "");
  if(action<(int)children.N && children(action)) HALT("duplicate transition call");
  if(L.state!=state) L.setState(state, T_step);
  L.T_real = T_real;
  CHECK_LE(1+(uint)action, actions.N, "that action doesn't exist");
  TreeSearchDomain::TransitionReturn ret = L.transition(actions(action));
  CHECK(L.state!=state, "");
  std::shared_ptr<FOL_World_State> s = make_shared<FOL_World_State>(L, this, L.is_terminal_state());
  s->folDecision = s->state->getNode("decision");
  s->f_prio = L.T_step;
  if(!s->isTerminal) s->f_prio += .9;
  s->name <<L.T_step <<'.' <<action <<' ' <<*actions(action);
  while(action>=(int)children.N) children.append(0);
  children(action) = s.get();
  return s;
}

NodeL FOL_World_State::getDecisionSequence(String& string) {
  Array<FOL_World_State*> folStates;
  folStates.memMove=1;
  FOL_World_State* s = this;
  while(s) { folStates.prepend(s); s = dynamic_cast<FOL_World_State*>(s->parent); }
  NodeL decisions;
  for(FOL_World_State* s:folStates) if(s->folDecision) {
      decisions.append(s->folDecision);
      s->folDecision->key = " ";
      string <<*s->folDecision;
      s->folDecision->key = "decision";
    }
  return decisions;
}

void FOL_World_State::getStateSequence(Array<Graph*>& states, arr& times, String& planString) {
  Array<FOL_World_State*> folStates;
  folStates.memMove=states.memMove=1;
  FOL_World_State* s = this;
  while(s) { folStates.prepend(s); s = dynamic_cast<FOL_World_State*>(s->parent); }
  for(FOL_World_State* s:folStates) {
    if(s->name.N) planString <<'\n' <<s->name;
    states.append(s->state);
    times.append(s->T_real);
  }
}

FOL_World_State* FOL_World_State::getChildByAction(Node* folDecision) {
  CHECK(children.N, "node is not expanded");
  for(TreeSearchNode* _ch:children) {
    FOL_World_State* ch = dynamic_cast<FOL_World_State*>(_ch);
    if(tuplesAreEqual(ch->folDecision->parents, folDecision->parents)) return ch;
  }
  LOG(-1) <<"a child with action '" <<*folDecision <<"' does not exist";
  return nullptr;
}

void FOL_World_State::write(std::ostream& os) const { os <<'#' <<ID <<'_' <<name <<' '; }

void FOL_World_State::report(std::ostream& os, int verbose) const { os <<'#' <<ID <<'_' <<name <<' '; }

void FOL_World_State::data(Graph& g) const {
  g.add("a", name); //STRING(*folDecision));
}

} //namespace
