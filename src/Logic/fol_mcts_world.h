/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#pragma once

#include <MCTS/environment.h>
#include <Core/array.h>
#include <Core/graph.h>

struct FOL_World:MCTS_Environment{

  struct Decision:SAO{
    bool waitDecision;
    Node *rule;
    NodeL substitution;
    int id;
    Decision(bool waitDecision, Node *rule, const NodeL& substitution, int id)
      : waitDecision(waitDecision), rule(rule), substitution(substitution), id(id) {}
    virtual bool operator==(const SAO & other) const {
      auto decision = dynamic_cast<const Decision *>(&other);
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

  struct Observation:SAO{
    int id;
    Observation(int id)
      : id(id) {}
    virtual bool operator==(const SAO & other) const {
      auto ob = dynamic_cast<const Observation *>(&other);
      return ob!=nullptr && ob->id==id;
    }
    void write(ostream& os) const { os <<id; }
    virtual size_t get_hash() const {
      return std::hash<int>()(id);
    }
  };

  struct State:SAO {
    Graph *state;
    uint T_step;
    double T_real;
    double R_total;

    State(Graph* state, FOL_World& fol_state)
      : state(state), T_step(fol_state.T_step), T_real(fol_state.T_real), R_total(fol_state.R_total) {}
    virtual bool operator==(const SAO & other) const {
      auto ob = dynamic_cast<const State*>(&other);
      return ob!=nullptr && ob->state==state;
    }
    void write(ostream& os) const { os <<*state; }
  };

  uint T_step, start_T_step; ///< discrete "time": decision steps so far
  double T_real, start_T_real;///< real time so far;
  double R_total;

  //-- parameters
  bool hasWait;
  double gamma, stepCost, timeCost, deadEndCost;
  uint maxHorizon;

  bool deadEnd, successEnd;
  Graph KB;     ///< current knowledge base
  Graph *start_state; ///< the start-state within the KB (is a subgraph item of KB)
  Graph *state; ///< the dynamic/fluent state within the KB (is a subgraph item of KB, created within the constructor)
  NodeL worldRules;     ///< rules within the KB (each is a subgraph item of the KB)
  NodeL decisionRules;  ///< rules within the KB (each is a subgraph item of the KB)
  Node *lastDecisionInState; ///< the literal that represents the last decision in the state
  Graph *rewardFct; ///< the reward function within the KB (is a subgraph item of KB)
  Node *Terminate_keyword, *Quit_keyword, *Wait_keyword, *Quit_literal;
  int verbose;
  int verbFil;
  ofstream fil;

  double lastStepReward;
  double lastStepDuration;
  double lastStepProbability;
  int lastStepObservation;
  long count;

  FOL_World();
  FOL_World(istream& fil);
  virtual ~FOL_World();
  void init(istream& fil);
  void init(const char* filename){ init(mlr::FileToken(filename)); }

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
  void set_state(mlr::String&);

  //-- helpers
  void addFact(const StringA& symbols);
  void addAgent(const char* name);
  void addObject(const char* name);
  template<class T> void addValuedFact(const StringA& symbols, const T& x){
    NodeL parents;
    for(const mlr::String& s:symbols) parents.append(KB[s]);
    start_state->newNode<T>({}, parents, x);
  }

  //-- internal access
  Graph* getState();
  void setState(Graph*, int setT_step=-1);
  Graph* createStateCopy();

  void write(std::ostream& os) const{ os <<KB; }
};
stdOutPipe(FOL_World)
