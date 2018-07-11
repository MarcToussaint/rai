/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "optLGP.h"

#include <Kin/kinViewer.h>
#include <KOMO/komo.h>
#include <Gui/opengl.h>
#ifdef RAI_GL
#  include <GL/gl.h>
#  include <GL/glu.h>
#endif
#include <iomanip>

uint displaySize=350;

void _system(const char* cmd) {
  cout <<"SYSTEM CMD: " <<cmd <<endl;
  int r = system(cmd);
  rai::wait(.1);
  if(r) HALT("system return error " <<r);
}

bool sortComp(const MNode* a, const MNode* b) {
  if(!a->isInfeasible && b->isInfeasible) return true;
  if(a->isInfeasible && !b->isInfeasible) return false;
  return a->cost.last() < b->cost.last();
}

typedef OptLGP_SolutionData* OptLGP_SolutionDataPtr;
bool sortComp2(const OptLGP_SolutionDataPtr& a, const OptLGP_SolutionDataPtr& b) {
  return sortComp(a->node, b->node);
}

struct DisplayThread : MiniThread {
  OptLGP* lgp;
  OpenGL gl;
  uint t=0;
  bool saveVideo=false;
  DisplayThread(OptLGP* lgp) : MiniThread("OptLGP_Display"), lgp(lgp), gl("OptLGP", 3*displaySize, 2*displaySize) {}
  ~DisplayThread() { threadClose(); }
  void resetSteppings() {
    lgp->solutions.writeAccess();
    for(uint i=0; i<lgp->solutions().N; i++) {
      lgp->solutions()(i)->displayStep=0;
    }
    lgp->solutions.deAccess();
  }
  
  void main() {
//    Metronome tic(.1);
    for(;;) {
      if(getStatus()<0) break;
//      tic.waitForTic();
      rai::wait(.1);
      lgp->solutions.writeAccess();
      for(uint i=0; i<lgp->solutions().N; i++) {
        lgp->solutions()(i)->displayStep++;
        if(gl.views.N>i)
          gl.views(i).text.clear() <<i <<':' <<lgp->solutions()(i)->displayStep <<": "
                                   <<lgp->solutions()(i)->node->cost <<"|  " <<lgp->solutions()(i)->node->constraints.last() <<'\n'
                                   <<lgp->solutions()(i)->decisions;
      }
      lgp->solutions.deAccess();
      if(saveVideo) gl.doCaptureImage=true;
      gl.update(NULL, false, false, false);
      if(saveVideo) write_ppm(gl.captureImage, STRING(OptLGPDataPath <<"vid/" <<std::setw(3)<<std::setfill('0')<<t++<<".ppm"));
    }
  }
};

void initFolStateFromKin(FOL_World& L, const rai::KinematicWorld& K) {
  for(rai::Frame *a:K.frames) if(a->ats["logical"]) {
      const Graph& G = a->ats["logical"]->graph();
      for(Node *n:G) L.addFact({n->keys.last(), a->name});
    }
  for(rai::Frame *a:K.frames) if(a->shape && a->ats["logical"]) {
      rai::Frame *p = a->getUpwardLink();
      if(!p) continue;
      FrameL F;
      p->getRigidSubFrames(F);
      for(rai::Frame *b:F) if(b!=a && b->shape && b->ats["logical"]) {
          L.addFact({"partOf", a->name, b->name});
        }
    }
  for(rai::Frame *a:K.frames) if(a->shape && a->ats["logical"]) {
      rai::Frame *p = a;
      while(p && !p->joint) p=p->parent;
      if(!p) continue;
      p = p->parent;
      FrameL F;
      while(p) {
        F.append(p);
        if(p->joint) break;
        p=p->parent;
      }
      for(rai::Frame *b:F) if(b!=a && b->shape && b->ats["logical"]) {
          L.addFact({"on", a->name, b->name});
        }
    }
}

OptLGP::OptLGP()
    : verbose(3), numSteps(0),
      solutions("OptLGPsolutions"){
    dataPath <<"z." <<rai::date2() <<"/";
    dataPath = rai::getParameter<rai::String>("LGP_dataPath", dataPath);
    _system(STRING("mkdir -p " <<dataPath));
    _system(STRING("rm -Rf " <<dataPath <<"vid  &&  rm -f " <<dataPath <<"*"));

    OptLGPDataPath = dataPath;
    if(!filNodes) filNodes = new ofstream(dataPath + "nodes");

    verbose = rai::getParameter<int>("LGP/vebose", 3);
    if(verbose>0) fil.open(dataPath + "optLGP.dat"); //STRING("z.optLGP." <<rai::date() <<".dat"));

}

OptLGP::OptLGP(rai::KinematicWorld &kin, FOL_World &fol) : OptLGP() {
    init(kin, fol);
}

void OptLGP::init(rai::KinematicWorld &kin, FOL_World &fol) {
    CHECK(!root,"");
  root = new MNode(kin, fol, 4);
  displayFocus = root;
  //  threadOpenModules(true);
}

OptLGP::~OptLGP() {
  views.clear();
  if(dth) delete dth;
  delete root;
  root=NULL;
  if(filNodes) { delete filNodes; filNodes=NULL; }
}

void OptLGP::initDisplay() {
  if(!views.N) {
    views.resize(4);
    views(1) = make_shared<OrsPathViewer>("pose", .2, -1);
    views(2) = make_shared<OrsPathViewer>("sequence", .2, -1);
    views(3) = make_shared<OrsPathViewer>("path", .05, -2);
    if(rai::getParameter<bool>("LGP/displayTree", 1)) {
      _system("evince z.pdf &");
      displayTree = true;
    } else {
      displayTree = false;
    }
    for(auto& v:views) if(v) v->copy.orsDrawJoints=v->copy.orsDrawMarkers=v->copy.orsDrawProxies=false;
  }
  if(!dth) dth = new DisplayThread(this);
}

void OptLGP::renderToVideo(uint level, const char* filePrefix) {
  CHECK(displayFocus->komoProblem(level) && displayFocus->komoProblem(level)->configurations.N, "level " <<level <<" has not been computed for the current 'displayFocus'");
  renderConfigurations(displayFocus->komoProblem(level)->configurations, filePrefix, -2, 600, 600, &views(3)->copy.gl().camera);
}

void OptLGP::updateDisplay() {
  if(fringe_solved.N) displayFocus = fringe_solved.last();
  
  rai::String decisions = displayFocus->getTreePathString('\n');
  for(uint i=1; i<views.N; i++) {
    if(displayFocus->komoProblem(i) && displayFocus->komoProblem(i)->configurations.N) {
      views(i)->setConfigurations(displayFocus->komoProblem(i)->configurations);
      views(i)->text.clear() <<displayFocus->cost <<"|  " <<displayFocus->constraints.last() <<'\n' <<decisions;
    } else views(i)->clear();
  }
  //  if(node->komoProblem(2) && node->komoProblem(2)->configurations.N)
  //    seqView.setConfigurations(node->komoProblem(2)->configurations);
  //  else seqView.clear();
  //  if(node->komoProblem(3) && node->komoProblem(3)->configurations.N)
  //    pathView.setConfigurations(node->komoProblem(3)->configurations);
  //  else pathView.clear();
  
  solutions.writeAccess();
  for(uint i=0; i<solutions().N && i<6; i++) {
    if(dth->gl.views.N<=i || !dth->gl.views(i).drawers.N) {
      dth->gl.addSubView(i, glStandardScene, NULL);
      dth->gl.addSubView(i, *solutions()(i));
//    dth->gl.views(i).drawers.last() = solutions()(i);
//    dth->gl.addSubView(i, glStandardScene, NULL);
//    dth->gl.addSubView(i, *solutions()(i));
      dth->gl.views(i).camera.setDefault();
//      dth->gl.views(i).camera.focus(.9, 0., 1.3);
    }
    dth->gl.views(i).drawers.last() = solutions()(i);
    dth->gl.views(i).text.clear() <<solutions()(i)->node->cost <<'\n' <<solutions()(i)->decisions;
  }
  dth->gl.setSubViewTiles(3,2);
  solutions.deAccess();
//  gl->update();

//  solutions.writeAccess();
//  if(solutions().N){
//    cout <<"SOLUTIONS: " <<solutions().N <<endl;
//    for(uint i=0;i<solutions().N;i++){
//      solutions()(i)->write(cout);
//    }
//  }
//  solutions.deAccess();

  if(displayTree) {
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
    for(auto& n:fringe_solved) n->note <<"DONE";
    
    Graph dot=root->getGraph(true);
    dot.writeDot(FILE("z.dot"));
    _system("dot -Tpdf z.dot > z.pdf");
  }
}

void OptLGP::printChoices() {
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
  for(MNode* a:displayFocus->children) {
    cout <<"(" <<c++ <<") DECISION: " <<*a->decision <<endl;
  }
}

rai::String OptLGP::queryForChoice() {
  rai::String cmd;
  std::string tmp;
  getline(std::cin, tmp);
  cmd=tmp.c_str();
  return cmd;
}

bool OptLGP::execRandomChoice() {
  rai::String cmd;
  if(rnd.uni()<.5) {
    switch(rnd.num(5)) {
      case 0: cmd="u"; break;
      case 1: cmd="p"; break;
      case 2: cmd="s"; break;
      case 3: cmd="x"; break;
      case 4: cmd="m"; break;
    }
  } else {
    cmd <<rnd(displayFocus->children.N);
  }
  return execChoice(cmd);
}

void OptLGP::player(StringA cmds) {
  bool interactive = rai::getParameter<bool>("interact", false);
  bool random = rai::getParameter<bool>("random", false);
  
  root->expand(5);
  
  initDisplay();
  
  for(uint s=0;; s++) {
    updateDisplay();
    printChoices();
    
    if(random) {
      if(!execRandomChoice()) break;
    } else {
      if(!interactive && s<cmds.N) {
        if(s>=cmds.N) break;
        if(!execChoice(cmds(s))) break;
      } else {
        rai::String cmd = queryForChoice();
        if(!execChoice(cmd)) break;
      }
    }
  }
}

void OptLGP::optFixedSequence(const rai::String& seq, int specificLevel, bool collisions) {
  Graph& tmp = root->fol.KB.newSubgraph({"TMP"}, {})->value;
  rai::String tmpseq(seq);
  tmp.read(tmpseq);
  
  cout <<"TMP:" <<*tmp.isNodeOfGraph <<endl;
  
  initDisplay();
  
  MNode *node = root;
  
  for(Node *actionLiteral:tmp) {
    if(specificLevel==-1 || specificLevel==1) node->optLevel(1); //optimize poses along the path
    node->expand();
    MNode *next = node->getChildByAction(actionLiteral);
    if(!next) LOG(-2) <<"action '" <<*actionLiteral <<"' is not a child of '" <<*node <<"'";
    displayFocus = node;
    updateDisplay();
    node = next;
  }
  
  if(specificLevel==-1 || specificLevel==1) node->optLevel(1);
  if(specificLevel==-1 || specificLevel==2) node->optLevel(2);
  if(specificLevel==-1 || specificLevel==3) node->optLevel(3, collisions);
  
  displayFocus = node;
  
  solutions.set()->append(new OptLGP_SolutionData(node));
  solutions.set()->sort(sortComp2);
  
  updateDisplay();
}

void OptLGP::optMultiple(const StringA& seqs) {
  for(const rai::String& seq:seqs) optFixedSequence(seq);
  
  _system(STRING("mkdir -p " <<OptLGPDataPath <<"vid"));
  _system(STRING("rm -f " <<OptLGPDataPath <<"vid/*.ppm"));
  dth->resetSteppings();
  dth->saveVideo = true;
  rai::wait(20.);
}

void OptLGP::writeNodeList(std::ostream &os) {
  os <<"id step cost= C0 C1 C2 C3 constr= R0 R1 R2 R3 fea= F0 F1 F2 F3 time= T0 T1 T2 T3 skeleton" <<endl;
  MNodeL ALL = root->getAll();
  for(MNode *n : ALL) {
//    if(n->count(l_pose)){
    os <<n->id <<' ' <<n->step
       <<" cost= " <<n->cost <<" constr= " <<n->constraints <<" fea= " <<convert<int>(n->feasible) <<" time= " <<n->computeTime <<" \"" <<n->getTreePathString() <<"\"" <<endl;
//    }
  }
}

void OptLGP::glDraw(OpenGL &gl) {
}

bool OptLGP::execChoice(rai::String cmd) {
  cout <<"COMMAND: '" <<cmd <<"'" <<endl;
  
  if(cmd=="q") return false;
  else if(cmd=="u") { if(displayFocus->parent) displayFocus = displayFocus->parent; }
  else if(cmd=="e") displayFocus->expand();
  else if(cmd=="p") displayFocus->optLevel(1);
  else if(cmd=="s") displayFocus->optLevel(2);
  else if(cmd=="x") displayFocus->optLevel(3);
  //  else if(cmd=="m") node->addMCRollouts(100,10);
  else {
    int choice=-1;
    cmd >>choice;
    cout <<"CHOICE=" <<choice <<endl;
    if(choice<0 || choice>=(int)displayFocus->children.N) {
      cout <<"--- there is no such choice" <<endl;
    } else {
      displayFocus = displayFocus->children(choice);
      if(!displayFocus->isExpanded) displayFocus->expand();
    }
  }
  return true;
}

MNode *OptLGP::getBest(MNodeL &fringe, uint level) {
  if(!fringe.N) return NULL;
  MNode* best=NULL;
  for(MNode* n:fringe) {
    if(n->isInfeasible || !n->count(level)) continue;
    if(!best || (n->feasible(level) && n->cost(level)<best->cost(level))) best=n;
  }
  return best;
}

MNode *OptLGP::popBest(MNodeL &fringe, uint level) {
  if(!fringe.N) return NULL;
  MNode* best=getBest(fringe, level);
  if(!best) return NULL;
  fringe.removeValue(best);
  return best;
}

MNode *OptLGP::expandBest(int stopOnDepth) { //expand
  //    MNode *n =  popBest(fringe_expand, 0);
  if(!fringe_expand.N) HALT("the tree is dead!");
  MNode *n =  fringe_expand.popFirst();
  
  CHECK(n,"");
  if(stopOnDepth>0 && n->step>=(uint)stopOnDepth) return NULL;
  n->expand();
  for(MNode* ch:n->children) {
    if(ch->isTerminal) {
      terminals.append(ch);
      MNodeL path = ch->getTreePath();
      for(MNode *n:path) if(!n->count(1)) fringe_pose2.setAppend(n); //pose2 is a FIFO
    } else {
      fringe_expand.append(ch);
    }
    if(n->count(1)) fringe_pose.append(ch);
  }
  return n;
}

void OptLGP::optBestOnLevel(int level, MNodeL &fringe, MNodeL *addIfTerminal, MNodeL *addChildren) { //optimize a seq
  if(!fringe.N) return;
  MNode* n = popBest(fringe, level-1);
  if(n && !n->count(level)) {
    try {
      n->optLevel(level);
    } catch(const char* err) {
      LOG(-1) <<"opt(level=" <<level <<") has failed for the following node:";
      n->write(cout, false, true);
      LOG(-3) <<"node optimization failed";
    }
    
    if(n->feasible(level)) {
      if(addIfTerminal && n->isTerminal) addIfTerminal->append(n);
      if(addChildren) for(MNode* c:n->children) addChildren->append(c);
    }
    displayFocus = n;
  }
}

void OptLGP::optFirstOnLevel(int level, MNodeL &fringe, MNodeL *addIfTerminal) {
  if(!fringe.N) return;
  MNode *n =  fringe.popFirst();
  if(n && !n->count(level)) {
    try {
      n->optLevel(level);
    } catch(const char* err) {
      LOG(-1) <<"opt(level=" <<level <<") has failed for the following node:";
      n->write(cout, false, true);
      LOG(-3) <<"node optimization failed";
    }
    
    if(n->feasible(level)) {
      if(addIfTerminal && n->isTerminal) addIfTerminal->append(n);
    }
    displayFocus = n;
  }
}

void OptLGP::clearFromInfeasibles(MNodeL &fringe) {
  for(uint i=fringe.N; i--;)
    if(fringe.elem(i)->isInfeasible) fringe.remove(i);
}

uint OptLGP::numFoundSolutions() {
  return fringe_solved.N;
}

rai::String OptLGP::report(bool detailed) {
  MNode *bpose = getBest(terminals, 1);
  MNode *bseq  = getBest(terminals, 2);
  MNode *bpath = getBest(fringe_solved, 3);
  
  rai::String out;
  out <<"TIME= " <<rai::cpuTime() <<" TIME= " <<COUNT_time <<" KIN= " <<COUNT_kin <<" EVALS= " <<COUNT_evals
      <<" POSE= " <<COUNT_opt(1) <<" SEQ= " <<COUNT_opt(2) <<" PATH= " <<COUNT_opt(3)
      <<" bestPose= " <<(bpose?bpose->cost(1):100.)
      <<" bestSeq= " <<(bseq ?bseq ->cost(2):100.)
      <<" bestPath= " <<(bpath?bpath->cost(3):100.)
      <<" #solutions= " <<fringe_solved.N;
      
//  if(bseq) displayFocus=bseq;
//  if(bpath) displayFocus=bpath;

  if(detailed) {
    out <<"\n*** found solutions:" <<endl;
    for(MNode *n:fringe_solved) n->write(out, false, true);
  }
  
  return out;
}

void OptLGP::reportEffectiveJoints() {
//  MNode *best = getBest();
  if(!displayFocus->komoProblem.last()) return;
  displayFocus->komoProblem.last()->reportProblem();
  displayFocus->komoProblem.last()->reportEffectiveJoints();
}

void OptLGP::step() {
  expandBest();
  
  uint numSol = fringe_solved.N;
  
  if(rnd.uni()<.5) optBestOnLevel(l_pose, fringe_pose, &fringe_seq, &fringe_pose);
  optFirstOnLevel(l_pose, fringe_pose2, &fringe_seq);
  optBestOnLevel(l_seq, fringe_seq, &fringe_path, NULL);
  if(verbose>0 && fringe_path.N) cout <<"EVALUATING PATH " <<fringe_path.last()->getTreePathString() <<endl;
  optBestOnLevel(l_path, fringe_path, &fringe_solved, NULL);
  
  if(fringe_solved.N>numSol) {
    if(verbose>0) cout <<"NEW SOLUTION FOUND! " <<fringe_solved.last()->getTreePathString() <<endl;
    solutions.set()->append(new OptLGP_SolutionData(fringe_solved.last()));
    solutions.set()->sort(sortComp2);
  }
  
  //-- update queues (if something got infeasible)
  clearFromInfeasibles(fringe_expand);
  clearFromInfeasibles(fringe_pose);
  clearFromInfeasibles(fringe_pose2);
  clearFromInfeasibles(fringe_seq);
  clearFromInfeasibles(fringe_path);
  clearFromInfeasibles(terminals);
  
  if(verbose>0) {
    rai::String out=report();
    fil <<out <<endl;
    if(verbose>1) cout <<out <<endl;
    if(verbose>2 && !(numSteps%1)) updateDisplay();
  }
  numSteps++;
}

void OptLGP::buildTree(uint depth) {
  init();
  
  if(verbose>0) {
    cout <<"BULDING TREE to depth " <<depth <<endl;
  }
  
  rai::timerRead(true);
  for(uint k=0;; k++) {
    MNode *b = expandBest(depth);
    if(!b) break;
  }
  
  if(verbose>0) {
    rai::String out=report();
    fil <<out <<endl;
    if(verbose>1) cout <<out <<endl;
    if(verbose>2) updateDisplay();
  }
}

void OptLGP::getSymbolicSolutions(uint depth) {
  buildTree(depth);
  uint i=0;
  for(MNode *a:terminals) {
    cout <<"solution " <<i <<": " <<a->getTreePathString() <<endl;
    i++;
  }
}

void OptLGP::init() {
  fringe_expand.append(root);
  fringe_pose.append(root);
  if(verbose>2) {
    initDisplay();
    updateDisplay();
  }
}

void OptLGP::run(uint steps) {
  init();
  
  uint stopSol = rai::getParameter<uint>("stopSol", 12);
  double stopTime = rai::getParameter<double>("stopTime", 400.);
  
  for(uint k=0; k<steps; k++) {
    step();
    
    if(fringe_solved.N>=stopSol) break;
    if(COUNT_time>stopTime) break;
  }
  
  if(verbose>0) report(true);
  
  //basic output
  ofstream output(dataPath+"lgpopt");
  writeNodeList(output);
  output.close();
  
  //this generates the movie!
  if(verbose>2) {
//    renderToVideo();
    _system(STRING("mkdir -p " <<OptLGPDataPath <<"vid"));
    _system(STRING("rm -f " <<OptLGPDataPath <<"vid/*.ppm"));
    dth->resetSteppings();
    dth->saveVideo = true;
    rai::wait(20.);
  }
  
  if(verbose>2) views.clear();
}

OptLGP_SolutionData::OptLGP_SolutionData(MNode *n) : node(n) {
  decisions = n->getTreePathString('\n');
  
  //--init geoms
  const rai::KinematicWorld& K = node->startKinematics;
  uintA frameIDs;
  for(uint f=0; f<K.frames.N; f++) {
    const rai::Frame *a = K.frames(f);
    if(a->shape && a->shape->geom && a->shape->type()!=rai::ST_marker) {
      frameIDs.append(a->ID);
    }
  }
  geomIDs.resizeAs(frameIDs);
  for(uint i=0; i<geomIDs.N; i++) geomIDs(i) = K.frames(frameIDs(i))->shape->geom->ID;
  
  uint L = node->komoProblem.N;
  paths.resize(L);
  for(uint l=0; l<L; l++) {
    KOMO *komo = node->komoProblem(l);
    if(komo && komo->configurations.N) {
      paths(l).resize(komo->configurations.N, frameIDs.N);
      for(uint s=0; s<komo->configurations.N; s++) for(uint i=0; i<frameIDs.N; i++) {
          paths(l)(s, i) = komo->configurations(s)->frames(frameIDs(i))->X;
        }
    }
  }
}

void OptLGP_SolutionData::write(std::ostream &os) const {
  os <<"decisions=" <<decisions
     <<"\t depth=" <<node->step
     <<"\t costs=" <<node->cost
     <<endl;
}

void OptLGP_SolutionData::glDraw(OpenGL &gl) {
#ifdef RAI_GL
  uint l=3;
  rai::Array<rai::Geom*>& geoms = _GeomStore()->geoms;
  
  if(!paths(l).N) return;
  
  for(uint i=0; i<geomIDs.N; i++) {
    if(displayStep >= paths(l).d0) displayStep = 0;
    rai::Transformation &X = paths(l)(displayStep, i);
    double GLmatrix[16];
    X.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    
    geoms(geomIDs(i))->glDraw(gl);
  }
#endif
}
