/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "fol.h"

/** The RelationalMachine maintains a knowledge base, which stores both, a state (=set/conjunction of facts) and a set
 * of rules (classically clauses, here 1st-order rules which may represent a stationary policy, script, or policy tree).
 *
 * Two situations change the state:
 * 1) An external process (sensor, action feedback, or planner) explicitly adds a fact via 'applyEffect'
 * 2) Within 'fwdChainRules', all rules are checked for their condition and if they hold, their effects are applied to the state
 *
 * The standard usage is nothing but alternating 'applyEffect' (triggered from external processes) and 'fwdChainRules'/
 *
 */
struct RelationalMachine {
  Graph KB;     ///< knowledge base
  Graph* state; ///< the state within the KB (is a subgraph item of KB)
  Graph* tmp;   ///< a tmp subgraph of the KB (private)
  rai::LogObject _log;

  RelationalMachine();
  RelationalMachine(const char* filename);
  void init(const char* filename);

  rai::Node* getFact(rai::String query) const;
  rai::Node* addFact(rai::String query) const;
  bool queryCondition(rai::String query) const; ///< return indicates coverage of the condition
  NodeL querySubstitutions(rai::String query) const;
  bool applyEffect(rai::String effect, bool fwdChain=false);   ///< return indicates change of state
//  bool applyEffects(rai::String effects, const NodeL& substitutions, bool fwdChain=false);
  bool applyEffect(Node* literal, bool fwdChain=false);
  void delFact(rai::Node* fact);
  NodeL fwdChainRules();                 ///< progresses the state by applying all rules until convergence

  Node* declareNewSymbol(rai::String symbolStr);
  rai::String getKB();
  rai::String getState() const;
  rai::String getRules() const;
  StringA getSymbols() const;
};

inline RelationalMachine& operator<<(RelationalMachine& RM, const char* effect) {
  RM.applyEffect(rai::String(effect));
  return RM;
}

inline std::ostream& operator<<(std::ostream& os, RelationalMachine& RM) {
  os <<RM.getState();
  return os;
}
