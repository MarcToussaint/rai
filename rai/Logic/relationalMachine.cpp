/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "relationalMachine.h"

RelationalMachine::RelationalMachine()
  : state(NULL), tmp(NULL), _log("RelationalMachine", 1, 0) {
}

RelationalMachine::RelationalMachine(const char* filename)
  : state(NULL), tmp(NULL), _log("RelationalMachine", 1, 0) {
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

bool RelationalMachine::applyEffect(rai::String effect, bool fwdChain) {
  tmp->clear();
  bool e=false;
  try {
    effect >>*tmp;
    tmp->checkConsistency();
    e = applyEffectLiterals(*state, *tmp, {}, NULL);
  } catch(...) {
    LOG(-1) <<"applyEffect "<<effect <<" -- syntax error of query";
//    return false;
  }
  LOG(1) <<"  effects=" <<*tmp;
  LOG(2) <<"  new state=\n  " <<getState();
  if(fwdChain) fwdChainRules();
  return e;
}

bool RelationalMachine::applyEffect(Node* literal, bool fwdChain) {
  bool e = applySubstitutedLiteral(*state, literal, {}, NULL);
  LOG(1) <<"  effects=" <<*literal;
  LOG(2) <<"  new state=\n  " <<getState();
  if(fwdChain) fwdChainRules();
  return e;
}

NodeL RelationalMachine::fwdChainRules() {
  tmp->clear();
  forwardChaining_FOL(KB, KB.get<Graph>("STATE"), NULL, *tmp, false);
  LOG(2) <<"  changes=" <<*tmp;
  LOG(2) <<"  new state=\n  " <<getState();
  return *tmp;
}

Node* RelationalMachine::declareNewSymbol(rai::String symbolStr) {
  Node *it = KB.readNode(symbolStr);
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
