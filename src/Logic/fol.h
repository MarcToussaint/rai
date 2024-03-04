/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/graph.h"

/* WORDING:

 a fact is a grounded literal (no variables)

 [a literal generally is a predicate with value, like true/false, number]

 a literal here typically a literal with open variables

 a substution lists values of all variables in a substitution_scope

 ['all variables in a substitution_scope' are all the declared symbols in that graph]

 */

namespace rai {

bool isSymbol(Node* n);
NodeL getLiteralsOfScope(Graph& KB);
NodeL getSymbolsOfScope(const Graph& KB);
NodeL getVariables(Node* literal, Graph* varScope);
uint getNumOfVariables(Node* literal, Graph* varScope);
Node* getFirstVariable(Node* literal, Graph* varScope);
//Node *getFirstSymbol(Node* literal, Graph* varScope);
Node* getFirstNonSymbolOfScope(Graph& KB);
Node* getSecondNonSymbolOfScope(Graph& KB);

//---------- checking equality of two single facts, or fact and literal, or find facts in a KB that match a fact or literal

bool tuplesAreEqual(NodeL& tuple0, NodeL& tuple1);
bool valuesAreEqual(Node* fact0, Node* fact1, bool booleanMeansExistance);
bool factsAreEqual(Node* fact0, Node* fact1, bool checkAlsoValue);
bool factsAreEqual(Node* fact, Node* literal, const NodeL& subst, const Graph* subst_scope, bool checkAlsoValue, bool ignoreSubst=false);
bool getEqualFactInKB(Graph& KB, Node* fact, bool checkAlsoValue=true);
bool getEqualFactInKB(Graph& KB, Node* literal, const NodeL& subst, const Graph* subst_scope, bool checkAlsoValue=true);
NodeL getPotentiallyEqualFactsInKB(Graph& KB, Node* tuple, const Graph& varScope, bool checkAlsoValue=true);
Node* getEqualFactInList(Node* fact, NodeL& facts, bool checkAlsoValue=true);

bool allFactsHaveEqualsInKB(Graph& KB, NodeL& facts, bool checkAlsoValue=true);
bool allFactsHaveEqualsInKB(Graph& KB, NodeL& literals, const NodeL& subst, const Graph* subst_scope, bool checkAlsoValue);

bool matchingFactsAreEqual(Graph& facts, Node* it1, Node* it2, const NodeL& subst, Graph* subst_scope);

//---------- finding possible variable substitutions

void removeInfeasibleSymbolsFromDomain(Graph& facts, NodeL& domain, Node* literal, Graph* varScope);
NodeL getSubstitutions2(Graph& KB, NodeL& relations, int verbose=0);
NodeL getRuleSubstitutions2(Graph& KB, rai::Graph& rule, int verbose=0);
bool substitutedRulePreconditionHolds(Graph& KB, Node* rule, const NodeL& subst, int verbose=0);

//----------- adding facts

Node* createNewFact(Graph& facts, const NodeL& symbols);
Node* createNewSubstitutedLiteral(Graph& facts, Node* literal, const NodeL& subst, Graph* subst_scope);
bool applySubstitutedLiteral(Graph& facts, Node*  literal, const NodeL& subst, Graph* subst_scope, Graph& changes=NoGraph);
bool applyEffectLiterals(Graph& facts, NodeL& effects, const NodeL& subst, Graph* subst_scope, Graph& changes=NoGraph);

//------------ fwd chaining

bool forwardChaining_FOL(Graph& state, NodeL& rules, Node* query=nullptr, Graph& changes=NoGraph, int verbose=0, int* samplingObservation=nullptr);
bool forwardChaining_FOL(Graph& KB, Graph& state, Node* query, Graph& changes=NoGraph, int verbose=0, int* samplingObservation=nullptr);
bool forwardChaining_propositional(Graph& KB, Node* q);

//------------ functions

double evaluateFunction(Graph& func, Graph& state, int verbose=0);

} //namespace
