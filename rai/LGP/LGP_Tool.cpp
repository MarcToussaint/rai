/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "LGP_Tool.h"

#include <KOMO/komo.h>
#include <KOMO/skeleton.h>
#include "initFol.h"

namespace rai {

  LGP_Tool::LGP_Tool(const char* file){
    fileBase = file;
    if(fileBase.endsWith(".lgp")) fileBase.resize(fileBase.N-4, true);

    String lgpFile = fileBase + ".lgp";
    String confFile = fileBase + ".g";
    LOG(0) <<"using lgpFile: '" <<lgpFile <<"'";
    LOG(0) <<"using confFile: '" <<confFile <<"'";

    Graph lgpConfig(lgpFile.p);
    C.addFile(confFile);

    //setup FolWorld
    FileToken folFile = lgpConfig.get<FileToken>("fol");
    LOG(0) <<"using folFile '" <<folFile.fullPath() <<"'";
    L.init(folFile.fullPath());
    initFolStateFromKin(L, C);
    L.addTerminalRule(lgpConfig.get<String>("terminal")); //"(on gripper ball)");

    //setup lgproot
    lgproot = make_shared<LGPComp_root>(L, C,
                                             lgpConfig.get<bool>("genericCollisions"),
                                             lgpConfig.get<StringA>("coll"),
                                             lgpConfig.get<StringA>("lifts"));

  }

  LGP_Tool::LGP_Tool(FOL_World& L, Configuration& C, bool genericCollisions, const StringA& explicitCollisions, const StringA& explicitLift) {

    lgproot = make_shared<LGPComp_root>(L, C, genericCollisions, explicitCollisions, explicitLift);
    //info = make_shared<LGP_DomainInfo>();
    //fol.reset_state();
    //root = make_shared<FOL_World_State>(fol, nullptr, false);
    //  mem.append(lgproot->root);
    focusNode = dynamic_cast<FOL_World_State*>(lgproot->fol_astar->root.get());
  }

  LGP_Tool::~LGP_Tool() {
  }

  void LGP_Tool::viewConfig(){
    lgproot->C.checkConsistency();
    lgproot->C.ensure_proxies(true);
    lgproot->C.reportProxies(cout, .1, true);
    editConfiguration(fileBase + ".g", lgproot->C);
  }


//-- extract skeleton
void LGP_Tool::getSkeleton(Skeleton& skeleton, String& skeletonString){
  skeleton.collisions=lgproot->genericCollisions;
  skeleton.addExplicitCollisions(lgproot->explicitCollisions);
  skeleton.addLiftPriors(lgproot->explicitLift);

  Array<Graph*> states;
  arr times;
  focusNode->getStateSequence(states, times, skeletonString);
  skeleton.setFromStateSequence(states, times);
}

void LGP_Tool::optFinalSlice(Skeleton& skeleton, const String& skeletonString){
  shared_ptr<KOMO> komoFinalSlice = skeleton.getKOMO_finalSlice(lgproot->C, 1e-2, -1e-2, lgproot->info->collScale);

  //    rnd.seed(rndSeed);
  komoFinalSlice->initRandom(0);
  komoFinalSlice->view(true, STRING(skeletonString <<" - init"));

  NLP_Solver sol;
  sol.setProblem(komoFinalSlice->nlp());
  sol.setOptions(OptOptions().set_stopEvals(lgproot->info->waypointStopEvals));
  sol.setInitialization(komoFinalSlice->x);

  auto ret = sol.solve();

  komoFinalSlice->view(true, STRING(skeletonString <<" - waypoints\n" <<*ret));
  komoFinalSlice->view_play(true);
}

void LGP_Tool::optWaypoints(Skeleton& skeleton, const String& skeletonString){
  shared_ptr<KOMO> komoWaypoints = skeleton.getKomo_waypoints(lgproot->C, 1e-2, -1e-2, lgproot->info->collScale);

  //    rnd.seed(rndSeed);
  komoWaypoints->initRandom(0);
  komoWaypoints->view(true, STRING(skeletonString <<" - init"));

  NLP_Solver sol;
  sol.setProblem(komoWaypoints->nlp());
  sol.setOptions(OptOptions().set_stopEvals(lgproot->info->waypointStopEvals));
  sol.setInitialization(komoWaypoints->x);

  auto ret = sol.solve();

  komoWaypoints->view(true, STRING(skeletonString <<" - waypoints\n" <<*ret));
  komoWaypoints->view_play(true);
}

void LGP_Tool::optPath(Skeleton& skeleton, const String& skeletonString){
  shared_ptr<KOMO> komoPath = skeleton.getKomo_path(lgproot->C, 30, lgproot->info->pathCtrlCosts, -1e-2, -1e-2, lgproot->info->collScale);

    //    rnd.seed(rndSeed);
  komoPath->initRandom(0);
  komoPath->view(true, STRING(skeletonString <<" - init"));

  NLP_Solver sol;
  sol.setProblem(komoPath->nlp());
  sol.setOptions(OptOptions().set_stopEvals(lgproot->info->waypointStopEvals));
  sol.setInitialization(komoPath->x);

  auto ret = sol.solve();

  komoPath->view(true, STRING(skeletonString <<" - waypoints\n" <<*ret));
  komoPath->view_play(true);
}

void LGP_Tool::solve_Decisions(const String& seq) {
  walkToNode(seq);
  String skeletonString;
  Skeleton skeleton;
  getSkeleton(skeleton, skeletonString);
  //cout <<skeleton <<endl;
  skeleton.write(cout, skeleton.getSwitches(lgproot->C));

  optFinalSlice(skeleton, skeletonString);
  optWaypoints(skeleton, skeletonString);
  optPath(skeleton, skeletonString);
}

void LGP_Tool::player() {

//  bool interactive = getParameter<bool>("interact", false);
//  bool random = getParameter<bool>("random", false);
  StringA cmds = getParameter<StringA>("playerCmds", {});

  focusNode = dynamic_cast<FOL_World_State*>(lgproot->fol_astar->root.get());
  expand(focusNode);

  for(uint s=0;; s++) {
    {
      //print
      cout <<"********************" <<endl;
      cout <<*focusNode <<endl;
      cout <<"--------------------" <<endl;
      cout <<"\nCHOICES:" <<endl;
      cout <<"(q) quit" <<endl;
      cout <<"(u) up" <<endl;
      cout <<"(e) expand node" <<endl;
      cout <<"(k) print skeleton" <<endl;
      cout <<"(p) pose optim" <<endl;
      cout <<"(s) sequence optim" <<endl;
      cout <<"(x) path optim" <<endl;
      uint c=0;
      for(FOL_World_State* a:focusNode->children) {
        cout <<"(" <<c++ <<") " <<*a->folDecision <<endl;
      }
    }

    {
      //query
      String cmd;
      if(cmds.N){
        cmd = cmds.popFirst();
      }else{
        std::string tmp;
        getline(std::cin, tmp);
        cmd=tmp.c_str();
      }
      cout <<"COMMAND: '" <<cmd <<"'" <<endl;

      //exec
      if(cmd=="q") break;
      else if(cmd=="e") { expand(focusNode); }
      else if(cmd=="u") { if(focusNode->parent) focusNode = dynamic_cast<FOL_World_State*>(focusNode->parent); }
      else if(cmd=="k"){ Skeleton S; String s; getSkeleton(S,s); cout <<"SKELETON: " <<S <<endl; }
      else if(cmd=="p"){ optFinalSlice(); }
      else if(cmd=="s"){ optWaypoints(); }
      else if(cmd=="x"){ optPath(); }
      else {
        int choice=-1;
        cmd >>choice;
        cout <<"CHOICE=" <<choice <<endl;
        if(choice<0 || choice>=(int)focusNode->children.N) {
          cout <<"--- there is no such choice" <<endl;
        } else {
          focusNode = focusNode->children(choice); //choose a decision
          expand(focusNode);
        }
      }
    }
  }
}

void LGP_Tool::step_folPlan(){
  lgproot->fol_astar->run();
  FOL_World_State* s = dynamic_cast<FOL_World_State*>(lgproot->fol_astar->solutions(-1));
  String str;
  s->getDecisionSequence(str);
  cout <<"FOL solution #" <<lgproot->fol_astar->solutions.N-1 <<": " <<str <<endl;
}

void LGP_Tool::compute(const intA& branches){
  Array<std::shared_ptr<TreeSearchNode>> mem;
  mem.append(lgproot);

  TreeSearchNode* node =  lgproot.get();
  for(uint i=0;i<branches.N;i++){
    auto child = node->transition(branches(i));
    mem.append(child);
    node = child.get();
    while(!node->isComplete) node->compute();
  }

  printTree(mem);
  system("evince z.pdf &");
}

void LGP_Tool::solve(const std::shared_ptr<TreeSearchNode>& root){
  AStar compute_astar(root);
  printTree(compute_astar.mem);
  system("evince z.pdf &");
  //  tree.runTrivial(1000, 100.);
  for(uint k=0;k<1000;k++){
    for(uint i=0;i<20;i++) compute_astar.step();
    //astar.report();
    printTree(compute_astar.mem);
    uint solutions=0;
    for(TreeSearchNode *n:compute_astar.solutions){
      if(n->isFeasible) solutions++;
      cout <<"=== SOLUTIONS: " <<solutions <<endl;
    }
    if(solutions>=12) break;
    //wait();
  }
}

void LGP_Tool::report(std::ostream& os) const{
  os <<"=== LGP info ===" <<endl;
  lgproot->L.report(os);
  lgproot->C.report(os);
  os <<"genericCollisions: " <<lgproot->genericCollisions <<endl;
  os <<"#explicitCollisions: " <<lgproot->explicitCollisions.d0 <<endl;
  os <<"#explicitLift: " <<lgproot->explicitLift.d0 <<endl;
  os <<"================" <<endl;
}

void LGP_Tool::expand(FOL_World_State* node){
  if(node->getNumDecisions()!=(int)node->children.N){ //expand
    int n = node->getNumDecisions();
    for(int i=0;i<n;i++) lgproot->fol_astar->mem.append(node->transition(i));
  }
}

void LGP_Tool::walkToNode(const String& seq) {
  FOL_World_State *root = dynamic_cast<FOL_World_State*>(lgproot->fol_astar->root.get());
  Graph& tmp = root->L.KB.addSubgraph("TMP");
  String tmpseq(seq);
  tmp.read(tmpseq);
  cout <<"decision sequence:" <<*tmp.isNodeOfGraph <<endl;

  //first walk to the node that corresponds to seq
  FOL_World_State* node = root;
  for(Node* actionLiteral:tmp) {
    expand(node);
    FOL_World_State* next = node->getChildByAction(actionLiteral);
    if(!next) LOG(-2) <<"action '" <<*actionLiteral <<"' is not a child of '" <<*node <<"'";
    node = next;
  }

  focusNode = node;
}

void LGP_Tool::buildTree(uint depth) {
  if(lgproot->info->verbose>0) {
    LOG(0) <<"BULDING TREE to depth " <<depth <<endl;
  }

  FOL_World_State *root = dynamic_cast<FOL_World_State*>(lgproot->fol_astar->root.get());
  Array<FOL_World_State*> fifo;
  fifo.append(root);

  while(fifo.N){
      FOL_World_State* n = fifo.popFirst();
      if(n->T_step<depth){
          expand(n);
          for(FOL_World_State* c:n->children) fifo.append(c);
      }
  }

  displayTreeUsingDot();
  system("evince z.pdf &");
}

void LGP_Tool::displayTreeUsingDot() {
  printTree(lgproot->fol_astar->mem);
}

} //namespace
