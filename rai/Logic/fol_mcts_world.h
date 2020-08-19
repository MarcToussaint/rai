/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../MCTS/environment.h"
#include "../Core/array.h"
#include "../Core/graph.h"

struct FOL_World : MCTS_Environment {

  struct Decision : SAO {
    bool waitDecision;
    rai::Node* rule;
    rai::NodeL substitution;
    int id;
    Decision(bool waitDecision, rai::Node* rule, const rai::NodeL& substitution, int id)
      : waitDecision(waitDecision), rule(rule), substitution(substitution), id(id) {}
    virtual bool operator==(const SAO& other) const {
      auto decision = dynamic_cast<const Decision*>(&other);
      if(decision==nullptr) return false;
      if(decision->waitDecision!=waitDecision) return false;
      if(decision->rule!=rule) return false;
      if(decision->substitution!=substitution) return false;
      return true;
    }
    rai::NodeL getTuple() const;
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
    rai::Graph* state;
    uint T_step;
    double T_real;
    double R_total;

    State(rai::Graph* state, FOL_World& fol_state)
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
  rai::Graph KB;     ///< current knowledge base
  rai::Graph* start_state=0; ///< the start-state within the KB (is a subgraph item of KB)
  rai::Graph* state=0; ///< the dynamic/fluent state within the KB (is a subgraph item of KB, created within the constructor)
  rai::NodeL worldRules;     ///< rules within the KB (each is a subgraph item of the KB)
  rai::NodeL decisionRules;  ///< rules within the KB (each is a subgraph item of the KB)
  rai::Node* lastDecisionInState=0; ///< the literal that represents the last decision in the state
  rai::Graph* rewardFct; ///< the reward function within the KB (is a subgraph item of KB)
  rai::Node* Terminate_keyword=0, *Wait_keyword=0, *Quit_keyword=0, *Quit_literal=0, *Subgoal_keyword=0, *Subgoal_literal=0;
  rai::Graph* subgoals=0;

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
  void init(const rai::Graph& _KB);
  void init(const char* filename);
  void copy(const FOL_World& fol) { init(fol.KB); }

  virtual TransitionReturn transition(const Handle& action); //returns (observation, reward)
  virtual const std::vector<Handle> get_actions();
  virtual bool is_feasible_action(const Handle& action);
  virtual const Handle get_stateCopy();
  virtual void set_state(const Handle& _state);

  virtual bool is_terminal_state() const;
  virtual void make_current_state_new_start();
  virtual void reset_state();

  virtual bool get_info(InfoTag tag) const;
  virtual double get_info_value(InfoTag tag) const;
  void write_state(ostream&);
  void set_state(rai::String&);

  //-- helpers to modify the problem
  rai::Node* addSymbol(const char* name);
  void addFact(const StringA& symbols);
  void addAgent(const char* name);
  void addObject(const char* name);
  template<class T> void addValuedFact(const StringA& symbols, const T& x) {
    rai::NodeL parents;
    for(const rai::String& s:symbols) parents.append(KB[s]);
    start_state->newNode<T>({}, parents, x);
  }
  void addTerminalRule(const char* literals);
  void addTerminalRule(const StringAA& literals);
  void addDecisionSequence(std::istream& is);

  //-- internal access
  rai::Graph* getState();
  void setState(rai::Graph*, int setT_step=-1);
  rai::Graph* createStateCopy();

  void write(std::ostream& os) const { os <<KB; }
  void writePDDLdomain(std::ostream& os, const char* domainName="raiFolDomain") const;
  void writePDDLproblem(std::ostream& os, const char* domainName="raiFolDomain", const char* problemName="raiFolProblem") const;
  void writePDDLfiles(rai::String name);
  rai::String callPDDLsolver();

};
stdOutPipe(FOL_World)

