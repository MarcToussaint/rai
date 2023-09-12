#include <Logic/fol.h>
//#include <Gui/graphview.h>

//===========================================================================

void testPolFwdChaining(){
  rai::Graph G;
  FILE("pol.g") >>G;

  cout <<G <<endl;

  cout <<"Q=" <<(rai::forwardChaining_propositional(G, G["Q"]) ?"true":"false") <<endl;
}

//===========================================================================

void testFolLoadFile(){
  rai::Graph G;
  G.checkConsistency();
  FILE("fol0.g") >>G;
  G.checkConsistency();

//  cout <<"\n-----------------\n" <<G <<"\n-----------------\n" <<endl;

  rai::NodeL consts = G.getNodesOfType<bool>();
  rai::Node *s = G["STATE"];
  rai::Node *r = G["cruiseto"]->graph()["precond"];
  rai::NodeL vars = G["cruiseto"]->graph().getNodes("Var");

  cout <<"state = " <<*s <<"\nrule=" <<*r <<endl;

  G.checkConsistency();
  rai::Graph& sub = G.addSubgraph();
  G.checkConsistency();
  sub.add<bool>(0, true, {s, consts(0)});
  G.checkConsistency();
  sub.add<bool>(0, true, {s, consts(2)});
  G.checkConsistency();
}

//===========================================================================

void testLoadAndDot(const char* filename="fol.g"){
  rai::Graph G(filename);
  G.checkConsistency();

  G.writeDot(FILE("z.dot"), false, false, 0);
  int r = system("dot -Tpdf z.dot > z.pdf; evince z.pdf &");
  if(r) LOG(-1) <<"could not startup dot or evince";
}

//===========================================================================

void testFolFwdChaining(){
  rai::Graph G;

  FILE("fol.g") >>G;

  rai::Graph& state = G.get<rai::Graph>("STATE");

  cout <<"INIT STATE = " <<state <<endl;

  rai::Node *query=G["Query"]->graph()(0);
  rai::forwardChaining_FOL(G, state, query);
//  cout <<"FINAL STATE = " <<state <<endl;
}

//===========================================================================

void testFolDisplay(){
  rai::Graph G;
  FILE("fol.g") >>G;

  G.displayDot();
}

//===========================================================================

void testFolSubstitution(){
  rai::Graph KB;

//  FILE("boxes.g") >>G;
  FILE("substTest.g") >>KB;

  rai::NodeL rules = KB.getNodes("Rule");
  rai::NodeL constants = KB.getNodes("Constant");
  rai::Graph& state = KB.get<rai::Graph>("STATE");

  for(rai::Node* rule:rules){
    cout <<"*** RULE: " <<*rule <<endl;
    cout <<  "Substitutions:" <<endl;
    rai::NodeL subs = getRuleSubstitutions2(state, rule->graph(), 2);
    cout <<"BEFORE state="; state.write(cout, " "); cout <<endl;
    for(uint s=0;s<subs.d0;s++){
      rai::Node *effect = rule->graph().last();
      { cout <<"*** applying" <<*effect <<" SUBS"; listWrite(subs[s], cout); cout <<endl; }
      applyEffectLiterals(state, effect->graph(), subs[s], &rule->graph());
      cout <<"AFTER state="; state.write(cout, " "); cout <<endl;
    }
  }
}

//===========================================================================

void testFolFunction(){
  rai::Graph KB(FILE("functionTest.g"));

  rai::Graph& state = KB.get<rai::Graph>("STATE");
  rai::Graph& func = KB.get<rai::Graph>("Qfunction");

  cout <<"f=" <<rai::evaluateFunction(func, state, 3) <<endl;
}

//===========================================================================

using rai::Node;
using rai::NodeL;

void testMonteCarlo(){
  rai::Graph Gorig;
  FILE("boxes.g") >>Gorig;
  rnd.seed(3);
  int verbose=2;

  for(uint k=0;k<10;k++){
    rai::Graph KB = Gorig;
    KB.checkConsistency();
    rai::Node *Terminate_keyword = KB["Terminate"];
    rai::Graph& state = KB.get<rai::Graph>("STATE");
    rai::NodeL rules = KB.getNodes("Rule");
    rai::Graph& terminal = KB.get<rai::Graph>("terminal");

    for(uint h=0;h<100;h++){
      if(verbose>2) cout <<"****************** " <<k <<" MonteCarlo rollout step " <<h <<endl;

      if(verbose>2){ cout <<"*** state = "; state.write(cout, " "); cout <<endl; }

      bool forceWait=false, decideWait=false;
      if(rnd.uni()<.8){ //normal rule decision
        //-- get all possible decisions
        rai::Array<std::pair<Node*, NodeL> > decisions; //tuples of rule and substitution
        for(Node* rule:rules){
          NodeL subs = rai::getRuleSubstitutions2(state, rule->graph(), verbose-2 );
//          NodeL subs = getRuleSubstitutions2(state, rule, verbose-2 );
          for(uint s=0;s<subs.d0;s++){
            decisions.append(std::pair<Node*, NodeL>(rule, subs[s]));
          }
        }

        if(verbose>2) cout <<"*** # possible decisions: " <<decisions.N <<endl;
        if(verbose>3) for(auto d:decisions){
          cout <<"rule " <<d.first->key <<" SUBS "; listWrite(d.second, cout); cout <<endl;
        }

        if(!decisions.N){
          forceWait=true;
        }else{
          //-- pick a random decision
          uint deci = rnd(decisions.N);
          std::pair<Node*, NodeL>& d = decisions(deci);
          if(verbose>2){ cout <<"*** decision = " <<deci <<':' <<d.first->key <<" SUBS "; listWrite(d.second, cout); cout <<endl; }

          Node *effect = d.first->graph().last();
          if(verbose>2){ cout <<"*** applying" <<*effect <<" SUBS"; listWrite(d.second, cout); cout <<endl; }
          applyEffectLiterals(state, effect->graph(), d.second, &d.first->graph());
        }
      }else{
        decideWait=true;
      }

      if(forceWait || decideWait){
        if(verbose>2){ cout <<"*** WAIT decision " <<endl; }

        //-- find minimal wait time
        double w=1e10;
        for(Node *i:state){
          if(i->is<double>()){
            double wi = i->as<double>();
            if(w>wi) w=wi;
          }
        }

        if(w==1e10){
          if(verbose>2) cout <<"*** not applicable" <<endl;
          if(forceWait){ cout <<"*** STUCK - NO FEASIBLE SOLUTION FOUND" <<endl;  break; }
        }else{
          //-- subtract w from all times and collect all activities with minimal wait time
          NodeL activities;
          for(Node *i:state){
            if(i->is<double>()){
              double &wi = i->as<double>();
              wi -= w;
              if(fabs(wi)<1e-10) activities.append(i);
            }
          }

          //-- for all these activities call the terminate operator
          for(Node *act:activities){
            Node *predicate = act->parents(0);
            Node *rule = KB.getEdge(Terminate_keyword, predicate);
            if(!rule) HALT("No termination rule for '" <<*predicate <<"'");
            Node *effect = rule->graph().last();
            NodeL vars = getSymbolsOfScope(rule->graph());
            NodeL subs(vars.N); subs.setZero();
            CHECK_EQ(vars.N, act->parents.N-1,"");
            for(uint i=0;i<vars.N;i++) subs(i) = act->parents(i+1);

            if(verbose>2){ cout <<"*** applying" <<*effect <<" SUBS"; listWrite(subs, cout); cout <<endl; }
            applyEffectLiterals(state, effect->graph(), subs, &rule->graph());
          }
        }
      }

      //-- test the terminal state
      if(allFactsHaveEqualsInKB(state, terminal)){
        if(verbose>0) cout <<"************* TERMINAL STATE FOUND (h=" <<h <<") ************" <<endl;
        if(verbose>1){ cout <<"*** FINAL STATE = "; state.write(cout, " "); cout <<endl; }
        break;
      }
    }
  }
}

//===========================================================================

int main(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  if(argc>1 && argv[1][0]!='-'){
    testLoadAndDot(argv[1]);
    return 0;
  }

  testPolFwdChaining();
  testFolLoadFile();
  testFolFwdChaining();
  testFolDisplay();
  testFolSubstitution();
  testFolFunction();
//  testMonteCarlo();

  cout <<"BYE BYE" <<endl;

  return 0;
}
