/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "relationalMachine.h"

RelationalMachine::RelationalMachine()
  : state(nullptr), tmp(nullptr), _log("RelationalMachine", 1, 0) {
}

RelationalMachine::RelationalMachine(const char* filename)
  : state(nullptr), tmp(nullptr), _log("RelationalMachine", 1, 0) {
  init(filename);
}

void RelationalMachine::init(const char* filename) {
  rai::FileToken fil(filename);
  if(fil.exists()) {
    fil >>KB;
    KB.checkConsistency();
  }
  if(!KB["TMP"])   KB.newSubgraph({"TMP"}, {});
  if(!KB["STATE"]) KB.newSubgraph({"STATE"}, {});
  state = &KB["STATE"]->graph();
  tmp   = &KB["TMP"]->graph();
}

rai::Node* RelationalMachine::getFact(rai::String query) const {
  tmp->clear();
  try {
    query >>*tmp;
    tmp->checkConsistency();
    CHECK_EQ(tmp->N, 1, "only single fact!")
    return getEqualFactInList(tmp->scalar(), *state);
  } catch(...) {
    LOG(-1) <<"queryCondition "<<query <<" -- syntax error of query" ;
  }
  return 0;
}

rai::Node* RelationalMachine::addFact(rai::String query) const {
  tmp->clear();
  try {
    query >>*tmp;
    tmp->checkConsistency();
    CHECK_EQ(tmp->N, 1, "only single fact!");
    return createNewFact(*state, tmp->scalar()->parents);
  } catch(...) {
    LOG(-1) <<"queryCondition "<<query <<" -- syntax error of query" ;
  }
  return 0;
}

bool RelationalMachine::queryCondition(rai::String query) const {
  tmp->clear();
  bool q=false;
  try {
    query >>*tmp;
    tmp->checkConsistency();
    q=allFactsHaveEqualsInKB(*state, *tmp);
  } catch(...) {
    LOG(-1) <<"queryCondition "<<query <<" -- syntax error of query:" ;
    return false;
  }
  LOG(2) <<"  query=" <<*tmp <<"  outcome=" <<(q?"TRUE":"FALSE");
  return q;
}

rai::NodeL RelationalMachine::querySubstitutions(rai::String query) const {
  tmp->clear();
  rai::NodeL substitutions;
  try {
    query >>*tmp;
    tmp->checkConsistency();
    substitutions = getRuleSubstitutions2(*state, *tmp);
  } catch(...) {
    LOG(-1) <<"queryCondition "<<query <<" -- syntax error of query:" ;
    return substitutions;
  }
  LOG(2) <<"  query=" <<*tmp <<"  outcome=" <<substitutions;
  return substitutions;
}

bool RelationalMachine::applyEffect(rai::String effect, bool fwdChain) {
  tmp->clear();
  bool e=false;
  try {
    effect >>*tmp;
    tmp->checkConsistency();
    e = applyEffectLiterals(*state, *tmp, {}, nullptr);
  } catch(...) {
    LOG(-1) <<"applyEffect "<<effect <<" -- syntax error of query";
//    return false;
  }
  LOG(1) <<"  effects=" <<*tmp;
  LOG(2) <<"  new state=\n  " <<getState();
  if(fwdChain) fwdChainRules();
  return e;
}

/*
bool RelationalMachine::applyEffects(rai::String effects, const NodeL& substitutions, bool fwdChain) {
  tmp->clear();
  bool e=false;
  try {
    effects >>*tmp;
    tmp->checkConsistency();
    e = applyEffectLiterals(*state, tmp->last()->graph(), substitutions, nullptr);
  } catch(...) {
    LOG(-1) <<"applyEffect "<<effects <<" -- syntax error of query";
//    return false;
  }
  LOG(1) <<"  effects=" <<*tmp;
  LOG(2) <<"  new state=\n  " <<getState();
  if(fwdChain) fwdChainRules();
  return e;
}
*/

bool RelationalMachine::applyEffect(Node* literal, bool fwdChain) {
  bool e = applySubstitutedLiteral(*state, literal, {}, nullptr);
  LOG(1) <<"  effects=" <<*literal;
  LOG(2) <<"  new state=\n  " <<getState();
  if(fwdChain) fwdChainRules();
  return e;
}

void RelationalMachine::delFact(rai::Node* fact) {
  CHECK_EQ(&fact->container, state, "");
  state->delNode(fact);
}

NodeL RelationalMachine::fwdChainRules() {
  tmp->clear();
  forwardChaining_FOL(KB, KB.get<Graph>("STATE"), nullptr, *tmp, false);
  LOG(2) <<"  changes=" <<*tmp;
  LOG(2) <<"  new state=\n  " <<getState();
  return *tmp;
}

Node* RelationalMachine::declareNewSymbol(rai::String symbolStr) {
  StringA tags;
  Node* it = KB.readNode(symbolStr, tags, 0, false, false);
  return it;
}

rai::String RelationalMachine::getKB() {
  rai::String str;
  KB.write(str, "\n  ");
  return str;
}

rai::String RelationalMachine::getState() const {
  rai::String str;
  state->write(str, "\n  ");
  return str;
}

rai::String RelationalMachine::getRules() const {
  NodeL rules = KB.getNodes("Rule");
  rai::String str;
  listWrite(rules, str, "\n  ", "[]");
  return str;
}

StringA RelationalMachine::getSymbols() const {
  NodeL symbols = getSymbolsOfScope(KB);
  StringA strs(symbols.N);
  for(uint i=0; i<symbols.N; i++) {
    strs(i) <<*symbols(i);
  }
  return strs;
}
