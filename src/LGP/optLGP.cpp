#include "optLGP.h"

#include <Kin/kinViewer.h>
#include <KOMO/komo.h>

OptLGP::OptLGP(mlr::KinematicWorld &kin, FOL_World &fol)
    : verbose(3), numSteps(0), fil("z.optLGP.dat"){
  root = new MNode(kin, fol, 4);
  displayFocus = root;
//  threadOpenModules(true);
}

OptLGP::~OptLGP(){
    views.clear();
}

void OptLGP::initDisplay(){
    if(!views.N){
        views.resize(4);
        views(1) = make_shared<OrsPathViewer>("pose", 1., -0);
        views(2) = make_shared<OrsPathViewer>("sequence", 1., -0);
        views(3) = make_shared<OrsPathViewer>("path", .1, -1);
        int r=system("evince z.pdf &");
        if(r) LOG(-1) <<"could not startup evince";
        for(auto& v:views) if(v) v->copy.orsDrawJoints=v->copy.orsDrawMarkers=v->copy.orsDrawProxies=false;
    }
}

void OptLGP::renderToFile(uint i, const char* filePrefix){
    CHECK(displayFocus->komoProblem(i) && displayFocus->komoProblem(i)->configurations.N, "level " <<i <<" has not been computed for the current 'displayFocus'");
    renderConfigurations(displayFocus->komoProblem(i)->configurations, filePrefix, -2, 600, 600);
}

void OptLGP::updateDisplay(){
  for(uint i=1;i<views.N;i++){
      if(displayFocus->komoProblem(i) && displayFocus->komoProblem(i)->configurations.N)
          views(i)->setConfigurations(displayFocus->komoProblem(i)->configurations);
      else views(i)->clear();
  }
//  if(node->komoProblem(2) && node->komoProblem(2)->configurations.N)
//    seqView.setConfigurations(node->komoProblem(2)->configurations);
//  else seqView.clear();
//  if(node->komoProblem(3) && node->komoProblem(3)->configurations.N)
//    pathView.setConfigurations(node->komoProblem(3)->configurations);
//  else pathView.clear();

  //generate the tree pdf
  MNodeL all = root->getAll();
  for(auto& n:all) n->note.clear();

  for(auto& n:all) if(n->isInfeasible) n->note <<"INFEASIBLE ";
  for(auto& n:fringe_expand)      n->note <<"EXPAND ";
  for(auto& n:terminals) n->note <<"TERMINAL ";
  for(auto& n:fringe_pose)  n->note <<"POSE ";
  for(auto& n:fringe_pose2) n->note <<"POSE2 ";
  for(auto& n:fringe_seq)  n->note <<"SEQ ";
  for(auto& n:fringe_path)  n->note <<"PATH ";
  for(auto& n:fringe_done) n->note <<"DONE";

  Graph dot=root->getGraph();
  dot.writeDot(FILE("z.dot"));
  int r = system("dot -Tpdf z.dot > z.pdf");
  if(r) LOG(-1) <<"could not startup dot";
}

void OptLGP::printChoices(){
  //-- query UI
  cout <<"********************" <<endl;
  cout <<"NODE:\n" <<*displayFocus <<endl;
  cout <<"--------------------" <<endl;
  cout <<"\nCHOICES:" <<endl;
  cout <<"(q) quit" <<endl;
  cout <<"(u) up" <<endl;
  cout <<"(e) expand node" <<endl;
  cout <<"(p) pose optim" <<endl;
  cout <<"(s) sequence optim" <<endl;
  cout <<"(x) path optim" <<endl;
  cout <<"(m) MC planning" <<endl;
  uint c=0;
  for(MNode* a:displayFocus->children){
    cout <<"(" <<c++ <<") DECISION: " <<*a->decision <<endl;
  }
}

mlr::String OptLGP::queryForChoice(){
  mlr::String cmd;
  std::string tmp;
  getline(std::cin, tmp);
  cmd=tmp.c_str();
  return cmd;
}

bool OptLGP::execRandomChoice(){
  mlr::String cmd;
  if(rnd.uni()<.5){
    switch(rnd.num(5)){
      case 0: cmd="u"; break;
      case 1: cmd="p"; break;
      case 2: cmd="s"; break;
      case 3: cmd="x"; break;
      case 4: cmd="m"; break;
    }
  }else{
    cmd <<rnd(displayFocus->children.N);
  }
  return execChoice(cmd);
}

void OptLGP::player(StringA cmds){
    bool interactive = mlr::getParameter<bool>("interact", false);
    bool random = mlr::getParameter<bool>("random", false);

    root->expand(5);

    initDisplay();

    for(uint s=0;;s++){
        updateDisplay();
        printChoices();

        if(random){
            if(!execRandomChoice()) break;
        }else{
            if(!interactive && s<cmds.N){
                if(s>=cmds.N) break;
                if(!execChoice(cmds(s))) break;
            }else{
                mlr::String cmd = queryForChoice();
                if(!execChoice(cmd)) break;
            }
        }
    }
}

void OptLGP::optFixedSequence(const mlr::String& seq){
    Graph& tmp = root->fol.KB.newSubgraph({"TMP"},{})->value;
    mlr::String tmpseq(seq);
    tmp.read(tmpseq);

    cout <<"TMP:" <<*tmp.isNodeOfGraph <<endl;

    initDisplay();

    MNode *node = root;

    for(Node *actionLiteral:tmp){
        node->optLevel(1); //optimize poses along the path
        node->expand();
        MNode *next = node->getChildByAction(actionLiteral);
        if(!next) LOG(-2) <<"action '" <<*actionLiteral <<"' is not a child of '" <<*node <<"'";
        displayFocus = node;
        updateDisplay();
        node = next;
    }

    node->optLevel(1);
    node->optLevel(2);
    node->optLevel(3);

    displayFocus = node;
    updateDisplay();
}

bool OptLGP::execChoice(mlr::String cmd){
    cout <<"COMMAND: '" <<cmd <<"'" <<endl;

    if(cmd=="q") return false;
    else if(cmd=="u"){ if(displayFocus->parent) displayFocus = displayFocus->parent; }
    else if(cmd=="e") displayFocus->expand();
    else if(cmd=="p") displayFocus->optLevel(1);
    else if(cmd=="s") displayFocus->optLevel(2);
    else if(cmd=="x") displayFocus->optLevel(3);
    //  else if(cmd=="m") node->addMCRollouts(100,10);
    else{
        int choice=-1;
        cmd >>choice;
        cout <<"CHOICE=" <<choice <<endl;
        if(choice<0 || choice>=(int)displayFocus->children.N){
            cout <<"--- there is no such choice" <<endl;
        }else{
            displayFocus = displayFocus->children(choice);
            if(!displayFocus->isExpanded) displayFocus->expand();
        }
    }
    return true;
}

MNode *OptLGP::getBest(MNodeL &fringe, uint level){
    if(!fringe.N) return NULL;
    MNode* best=NULL;
    for(MNode* n:fringe){
        if(n->isInfeasible || !n->count(level)) continue;
        if(!best || (n->feasible(level) && n->cost(level)<best->cost(level))) best=n;
    }
    return best;
}

MNode *OptLGP::popBest(MNodeL &fringe, uint level){
    if(!fringe.N) return NULL;
    MNode* best=getBest(fringe, level);
    if(!best) return NULL;
    fringe.removeValue(best);
    return best;
}

void OptLGP::expandBest(){ //expand
    MNode *n =  popBest(fringe_expand, 0);
    CHECK(n,"");
    n->expand();
    for(MNode* ch:n->children){
        if(ch->isTerminal){
            terminals.append(ch);
            MNodeL path = ch->getTreePath();
            for(MNode *n:path) if(!n->count(1)) fringe_pose2.append(n); //pose2 is a FIFO
        }else{
            fringe_expand.append(ch);
        }
        if(n->count(1)) fringe_pose.append(ch);
    }
}

void OptLGP::optBestOnLevel(int level, MNodeL &fringe, MNodeL *addIfTerminal, MNodeL *addChildren){ //optimize a seq
    if(!fringe.N) return;
    MNode* n = popBest(fringe, level-1);
    if(n && !n->count(level)){
        try{
            n->optLevel(level);
        }catch(const char* err){
            LOG(-1) <<"opt(level=" <<level <<") has failed for the following node:";
            n->write(cout, false, true);
            LOG(-3) <<"node optimization failed";
        }

        if(n->feasible(level)){
            if(addIfTerminal && n->isTerminal) addIfTerminal->append(n);
            if(addChildren) for(MNode* c:n->children) addChildren->append(c);
        }
        displayFocus = n;
    }
}

void OptLGP::optFirstOnLevel(int level, MNodeL &fringe, MNodeL *addIfTerminal){
    if(!fringe.N) return;
    MNode *n =  fringe.popFirst();
    if(n && !n->count(level)){
       try{
            n->optLevel(level);
        }catch(const char* err){
            LOG(-1) <<"opt(level=" <<level <<") has failed for the following node:";
            n->write(cout, false, true);
            LOG(-3) <<"node optimization failed";
        }

        if(n->feasible(level)){
            if(addIfTerminal && n->isTerminal) addIfTerminal->append(n);
        }
        displayFocus = n;
    }
}

void OptLGP::clearFromInfeasibles(MNodeL &fringe){
    for(uint i=fringe.N;i--;)
        if(fringe.elem(i)->isInfeasible) fringe.remove(i);
}

uint OptLGP::numFoundSolutions(){
    return fringe_done.N;
}

mlr::String OptLGP::report(bool detailed){
    MNode *bpose = getBest(terminals, 1);
    MNode *bseq  = getBest(terminals, 2);
    MNode *bpath = getBest(fringe_done, 3);

    mlr::String out;
    out <<"TIME= " <<mlr::cpuTime() <<" KIN= " <<COUNT_kin <<" EVALS= " <<COUNT_evals
       <<" POSE= " <<COUNT_opt(1) <<" SEQ= " <<COUNT_opt(2) <<" PATH= " <<COUNT_opt(3)
      <<" bestPose= " <<(bpose?bpose->cost(1):100.)
     <<" bestSeq = " <<(bseq ?bseq ->cost(2):100.)
    <<" pathPath= " <<(bpath?bpath->cost(3):100.)
    <<" #solutions= " <<fringe_done.N;

    if(bseq) displayFocus=bseq;
    if(bpath) displayFocus=bpath;

    if(detailed){
      out <<"\n*** found solutions:" <<endl;
      for(MNode *n:fringe_done) n->write(out, false, true);
    }

    return out;
}

void OptLGP::step(){
    expandBest();

    if(rnd.uni()<.5) optBestOnLevel(l_pose, fringe_pose, &fringe_seq, &fringe_pose);
    optFirstOnLevel(l_pose, fringe_pose2, &fringe_seq);
    optBestOnLevel(l_seq, fringe_seq, &fringe_path, NULL);
    optBestOnLevel(l_path, fringe_path, &fringe_done, NULL);

    //-- update queues (if something got infeasible)
    clearFromInfeasibles(fringe_expand);
    clearFromInfeasibles(fringe_pose);
    clearFromInfeasibles(fringe_pose2);
    clearFromInfeasibles(fringe_seq);
    clearFromInfeasibles(fringe_path);
    clearFromInfeasibles(terminals);

    if(verbose>0){
        mlr::String out=report();
        fil <<out <<endl;
        if(verbose>1) cout <<out <<endl;
        if(verbose>2 && !(numSteps%10)) updateDisplay();
    }
    numSteps++;
}

void OptLGP::init(){
    fringe_expand.append(root);
    fringe_pose.append(root);
    if(verbose>2){
        initDisplay();
        updateDisplay();
    }
}

void OptLGP::run(uint steps){
    init();

    for(uint k=0;k<steps;k++){
        step();

        if(fringe_done.N>10) break;
    }

    if(verbose>0) report(true);

    //this generates the movie!
    if(verbose>2) renderToFile();

    if(verbose>2) views.clear();
}
