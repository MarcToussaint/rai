/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

//#include "../RosCom/actionMachine.h"
//#include "manipSim.h"
#include "../Kin/kin.h"
#include "../Logic/fol.h"

void runMonteCarlo(Graph& G) {
//  rai::rnd.seed(3);
  uint verbose=0;

  G.checkConsistency();
  //    Node *Terminate_keyword = G["Terminate"];
  NodeL rules = G.getNodes("Rule");
  NodeL constants = G.getNodes("Object");
  Graph& actionSequence = G["actionSequence"]->graph();
  Node* papSymbol = G["pap"];
  Node* depthSymbol = G["depth"];
  Graph& state = G["STATE"]->graph();
  //    Graph& terminal = G.get<Graph>("terminal");

  for(uint h=0; h<100; h++) {
    if(verbose>2) cout <<"****************** MonteCarlo rollout step " <<h <<endl;

    if(verbose>2) { cout <<"*** state = "; state.write(cout, " ", "{}"); cout<<endl; }
    if(verbose>2) { cout <<"*** actionSequence = "; actionSequence.write(cout, " ", "{}"); cout<<endl; }

    {
      //-- get all possible decisions
      rai::Array<std::pair<Node*, NodeL>> decisions; //tuples of rule and substitution
      for(Node* rule:rules) {
        //      cout <<"*** RULE: " <<*rule <<endl;
        //      cout <<  "Substitutions:" <<endl;
        NodeL subs = getRuleSubstitutions2(state, rule->graph(), (verbose-3));
        for(uint s=0; s<subs.d0; s++) {
          decisions.append(std::pair<Node*, NodeL>(rule, subs[s]));
        }
      }

      if(verbose>2) cout <<"*** # possible decisions: " <<decisions.N <<endl;
      if(verbose>3) for(auto d:decisions) {
          cout <<"rule " <<d.first->keys(1) <<" SUBS "; listWrite(d.second, cout); cout <<endl;
        }

      if(!decisions.N) {
        if(verbose>1) cout <<"*** NO DECISIONS LEFT" <<endl;
        return;
      } else {
        //-- pick a random decision
        uint deci = rnd(decisions.N);
        std::pair<Node*, NodeL>& d = decisions(deci);
        if(verbose>2) { cout <<"*** decision = " <<deci <<':' <<d.first->keys(1) <<" SUBS "; listWrite(d.second, cout); cout <<endl; }

        Node* effect = d.first->graph().last();
        if(verbose>2) { cout <<"*** applying" <<*effect <<" SUBS"; listWrite(d.second, cout); cout <<endl; }
        applyEffectLiterals(state, effect->graph(), d.second, &d.first->graph());

        //hack: apply depth effect:
        Node* depth0=nullptr, *depth1=nullptr;
        for(Node* fact:d.second(0)->children) if(&fact->container==&state && fact->parents(0)==depthSymbol) {
            depth0=fact; break;
          }
        for(Node* fact:d.second(1)->children) if(&fact->container==&state && fact->parents(0)==depthSymbol) {
            depth1=fact; break;
          }
        if(depth0 && depth1) {
          depth0->get<double>() = depth1->get<double>() + 1.;
        }

        //-- append it to store the decision
        actionSequence.newNode({d.first->keys(1)}, cat({papSymbol}, d.second), true);
      }
    }

    //-- test the terminal state
    //      if(checkAllMatchesInScope(terminal, &G)){
    //        if(verbose>0) cout <<"************* TERMINAL STATE FOUND (h=" <<h <<") ************" <<endl;
    //        state = getLiteralsOfScope(G);
    //        if(verbose>1){ cout <<"*** FINAL STATE = "; listWrite(state, cout); cout<<endl; }
    //        break;
    //      }
  }
}
