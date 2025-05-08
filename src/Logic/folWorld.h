/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "treeSearchDomain.h"
#include "../Search/TreeSearchNode.h"
#include "../Core/array.h"
#include "../Core/graph.h"

namespace rai {

struct FOL_World : TreeSearchDomain {

  struct Decision : SAO {
    bool waitDecision;
    Node* rule;
    NodeL substitution;
    int id;
    Decision(bool waitDecision, Node* rule, const NodeL& substitution, int id)
      : waitDecision(waitDecision), rule(rule), substitution(substitution), id(id) {}
    virtual bool operator==(const SAO& other) const {
      auto decision = dynamic_cast<const Decision*>(&other);
      if(decision==nullptr) return false;
      if(decision->waitDecision!=waitDecision) return false;
      if(decision->rule!=rule) return false;
      if(decision->substitution!=substitution) return false;
      return true;
    }
    NodeL getTuple() const;
    void write(ostream&) const;
    virtual size_t get_hash() const {
      return std::hash<int>()(id);
    }
  };

  struct Observation:SAO {
    int id;
    Observation(int id)
      : id(id) {}
    virtual bool operator==(const SAO& other) const {
      auto ob = dynamic_cast<const Observation*>(&other);
      return ob!=nullptr && ob->id==id;
    }
    void write(ostream& os) const { os <<id; }
    virtual size_t get_hash() const {
      return std::hash<int>()(id);
    }
  };

  struct State:SAO {
    Graph* state;
    uint T_step;
    double T_real;
    double R_total;

    State(Graph* state, FOL_World& fol_state)
      : state(state), T_step(fol_state.T_step), T_real(fol_state.T_real), R_total(fol_state.R_total) {}
    virtual bool operator==(const SAO& other) const {
      auto ob = dynamic_cast<const State*>(&other);
      return ob!=nullptr && ob->state==state;
    }
    void write(ostream& os) const { os <<*state; }
  };

  //-- parameters
  bool hasWait;
  double gamma, stepCost, timeCost, deadEndCost;
  uint maxHorizon;

  //-- internal state
  uint T_step, start_T_step; ///< discrete "time": decision steps so far
  double T_real, start_T_real;///< real time so far;
  double R_total;

  // the logic state is fully described by the KB; all other variables just point into the KB
  bool deadEnd, successEnd;
  Graph KB;     ///< current knowledge base
  Graph* start_state=0; ///< the start-state within the KB (is a subgraph item of KB)
  Graph* state=0; ///< the dynamic/fluent state within the KB (is a subgraph item of KB, created within the constructor)
  NodeL worldRules;     ///< rules within the KB (each is a subgraph item of the KB)
  NodeL decisionRules;  ///< rules within the KB (each is a subgraph item of the KB)
  Node* lastDecisionInState=0; ///< the literal that represents the last decision in the state
  Graph* rewardFct; ///< the reward function within the KB (is a subgraph item of KB)
  Node* Terminate_keyword=0, *Wait_keyword=0, *Quit_keyword=0, *Quit_literal=0, *Subgoal_keyword=0, *Subgoal_literal=0;
  Graph* subgoals=0;

  int verbose;
  int verbFil;
  ofstream fil;

  double lastStepReward;
  double lastStepDuration;
  double lastStepProbability;
  int lastStepObservation;
  long count;

  FOL_World();
  FOL_World(const char* filename);
  virtual ~FOL_World();
  void init(const Graph& _KB);
  void init(const char* filename);
  void copy(const FOL_World& fol) { init(fol.KB); }

  virtual TransitionReturn transition(const Handle& action); //returns (observation, reward)
  virtual const Array<Handle> get_actions();
  virtual bool is_feasible_action(const Handle& action);
  virtual const Handle get_stateCopy();
  virtual void set_state(const Handle& _state);

  virtual bool is_terminal_state() const;
  virtual void make_current_state_new_start();
  virtual void reset_state();

  virtual bool get_info(InfoTag tag) const;
  virtual double get_info_value(InfoTag tag) const;
  void write_state(ostream&);
  void set_state(String&);

  //-- helpers to modify the problem
  Node* addSymbol(const char* name);
  void addFact(const StringA& symbols);
  void addAgent(const char* name);
  void addObject(const char* name);
  template<class T> void addValuedFact(const StringA& symbols, const T& x) {
    NodeL parents;
    for(const String& s:symbols) parents.append(KB.findNode(s));
    start_state->add<T>(0,  x, parents);
  }
  void addTerminalRule(const char* literals);
  void addTerminalRule(const StringAA& literals);
  void addDecisionSequence(std::istream& is);

  //-- internal access
  Graph* getState();
  void setState(Graph*, int setT_step=-1);
  Graph* createStateCopy();

  void write(std::ostream& os) const { os <<KB; }
  void writePDDLdomain(std::ostream& os, const char* domainName="raiFolDomain") const;
  void writePDDLproblem(std::ostream& os, const char* domainName="raiFolDomain", const char* problemName="raiFolProblem") const;
  void writePDDLfiles(const String& name);
  String callPDDLsolver();

  void report(ostream& os) const;
};
stdOutPipe(FOL_World)

struct FOL_World_State : TreeSearchNode {
  FOL_World& L;
  Graph* state=0;
  uint T_step=0;
  double T_real=0;
  double R_total=0;
  Node* folDecision=0;
  Array<FOL_World::Handle> actions;
  rai::String name;

  FOL_World_State(FOL_World& L, TreeSearchNode* _parent, bool _isTerminal);

  //compute
  virtual void compute() { HALT("shouldn't be here"); }

  //transition
  virtual int getNumDecisions() { return actions.N; }
  virtual std::shared_ptr<TreeSearchNode> transition(int action);

  //helpers
  void getStateSequence(Array<Graph*>& states, arr& times, String& skeletonString);
  NodeL getDecisionSequence(String& string);
  FOL_World_State* getChildByAction(Node* folDecision);

  //I/O
  virtual void write(std::ostream& os) const;
  virtual void report(std::ostream& os, int verbose) const;
  virtual void data(Graph& g) const;
};

} //namespace
