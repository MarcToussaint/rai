/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "LGP_tree.h"

#include "../Kin/kinViewer.h"
#include "../Kin/viewer.h"
#include "../KOMO/komo.h"
#include "../Gui/opengl.h"
#ifdef RAI_GL
#  include <GL/gl.h>
#  include <GL/glu.h>
#endif
#include <iomanip>

uint displaySize=350;

bool sortComp(const LGP_Node* a, const LGP_Node* b) {
  if(!a->isInfeasible && b->isInfeasible) return true;
  if(a->isInfeasible && !b->isInfeasible) return false;
  return a->cost.last() < b->cost.last();
}

typedef LGP_Tree_SolutionData* OptLGP_SolutionDataPtr;
bool sortComp2(const OptLGP_SolutionDataPtr& a, const OptLGP_SolutionDataPtr& b) {
  return sortComp(a->node, b->node);
}

struct DisplayThread : Thread {
  LGP_Tree* lgp;
  OpenGL gl;
  uint t=0;
  bool saveVideo=false;
  DisplayThread(LGP_Tree* lgp) : Thread("OptLGP_Display"), lgp(lgp), gl("OptLGP", 3*displaySize, 2*displaySize) { threadLoop(); }
  ~DisplayThread() { threadClose(); }
  void resetSteppings() {
    lgp->solutions.writeAccess();
    for(uint i=0; i<lgp->solutions().N; i++) {
      lgp->solutions()(i)->displayStep=0;
    }
    lgp->solutions.deAccess();
  }

  void step(){
    //      tic.waitForTic();
    rai::wait(.1);
    lgp->solutions.writeAccess();
    uint numSolutions = lgp->solutions().N;
    for(uint i=0; i<numSolutions; i++) {
      lgp->solutions()(i)->displayStep++;
      if(gl.views.N>i)
        gl.views(i).text.clear() <<i <<':' <<lgp->solutions()(i)->displayStep <<": "
                                <<lgp->solutions()(i)->node->cost <<"|  " <<lgp->solutions()(i)->node->constraints.last() <<'\n'
                               <<lgp->solutions()(i)->decisions;
    }
    lgp->solutions.deAccess();
    if(numSolutions)
      gl.update();
    if(saveVideo) write_ppm(gl.captureImage, STRING(OptLGPDataPath <<"vid/" <<std::setw(4)<<std::setfill('0')<<t++<<".ppm"));
  }
};

void initFolStateFromKin(FOL_World& L, const rai::Configuration& K) {
  for(rai::Frame* a:K.frames) if(a->ats["logical"]) {
      const Graph& G = a->ats["logical"]->graph();
      for(Node* n:G) L.addFact({n->key, a->name});
//      L.addFact({"initial", a->name}); //*** THE INITIAL FACT WAS INTRODUCED TO SIMPLIFY SKELETONS - OBSOLETE ***
    }
  for(rai::Frame* a:K.frames) if(a->shape && a->ats["logical"]) {
      rai::Frame* p = a->getUpwardLink();
      if(!p) continue;
      FrameL F;
      p->getRigidSubFrames(F);
      for(rai::Frame* b:F) if(b!=a && b->shape && b->ats["logical"]) {
          L.addFact({"partOf", a->name, b->name});
        }
    }
  for(rai::Frame* a:K.frames) if(a->shape && a->ats["logical"]) {
      rai::Frame* p = a;
      while(p && !p->joint) p=p->parent;
      if(!p) continue;
      p = p->parent;
      FrameL F;
      while(p) {
        F.append(p);
        if(p->joint) break;
        p=p->parent;
      }
      for(rai::Frame* b:F) if(b!=a && b->shape && b->ats["logical"]) {
          L.addFact({"on", b->name, a->name});
        }
    }
}

LGP_Tree::LGP_Tree()
  : verbose(2), numSteps(0) {
  dataPath <<"z." <<rai::date(true) <<"/";
  dataPath = rai::getParameter<rai::String>("LGP_dataPath", dataPath);
  rai::system(STRING("mkdir -p " <<dataPath));
  rai::system(STRING("rm -Rf " <<dataPath <<"vid  &&  rm -f " <<dataPath <<"*"));

  OptLGPDataPath = dataPath;
  if(!filNodes) filNodes = new ofstream(dataPath + "nodes");

  collisions = rai::getParameter<bool>("LGP/collisions", true);
  displayTree = rai::getParameter<bool>("LGP/displayTree", false);

  verbose = rai::getParameter<double>("LGP/verbose", 2);
  if(verbose>0) fil.open(dataPath + "optLGP.dat"); //STRING("z.optLGP." <<rai::date() <<".dat"));

  cameraFocus = rai::getParameter<arr>("LGP/cameraFocus", {});
}

LGP_Tree::LGP_Tree(const rai::Configuration& _kin, const char* folFileName) : LGP_Tree() {
  kin.copy(_kin);
  if(collisions) kin.swift(); //initialize swift in root model (SwiftInterface is reference by all child models)
  fol.init(folFileName);
  initFolStateFromKin(fol, kin);
  if(verbose>0) cout <<"INITIAL LOGIC STATE = " <<*fol.start_state <<endl;
  finalGeometryObjectives.setModel(kin);
  finalGeometryObjectives.setTiming(1., 1, 1., 1);
  root = new LGP_Node(this, BD_max);
  focusNode = root;
}

LGP_Tree::LGP_Tree(const rai::Configuration& _kin, const FOL_World& _fol) : LGP_Tree() {
  kin.copy(_kin);
  fol.copy(_fol);
  finalGeometryObjectives.setModel(kin);
  finalGeometryObjectives.setTiming(1., 1, 1., 1);
  root = new LGP_Node(this, BD_max);
  focusNode = root;
  if(verbose>0) cout <<"INITIAL LOGIC STATE = " <<*root->folState <<endl;
}

LGP_Tree::~LGP_Tree() {
  views.clear();
  if(dth) dth.reset();
  delete root;
  root=nullptr;
  if(filNodes) { delete filNodes; filNodes=nullptr; }
  solutions.writeAccess();
  listDelete(solutions());
  solutions.deAccess();
}

void LGP_Tree::initDisplay() {
  if(verbose>2 && !views.N) {
    views.resize(4);
    views(1) = make_shared<KinPathViewer>(Var<ConfigurationL>(), 1.2, -1);
    views(2) = make_shared<KinPathViewer>(Var<ConfigurationL>(), 1.2, -1);
    views(3) = make_shared<KinPathViewer>(Var<ConfigurationL>(), .05, -2);
    for(auto& v:views) if(v) v->copy.orsDrawJoints=v->copy.orsDrawMarkers=v->copy.orsDrawProxies=false;
  }
  if(!dth) dth = make_shared<DisplayThread>(this);
}

void LGP_Tree::renderToVideo(int specificBound, const char* filePrefix) {
  if(specificBound<0) specificBound=displayBound;
  CHECK(focusNode->komoProblem(specificBound) && focusNode->komoProblem(specificBound)->pathConfig.frames.N, "level " <<specificBound <<" has not been computed for the current 'displayFocus'");
  if(specificBound<(int)views.N && views(specificBound)) {
    NIY //renderConfigurations(focusNode->komoProblem(specificBound)->configurations, filePrefix, -2, 600, 600, &views(specificBound)->copy.gl()->displayCamera());
  } else {
    NIY //renderConfigurations(focusNode->komoProblem(specificBound)->configurations, filePrefix, -2, 600, 600);
  }
}

void LGP_Tree::displayTreeUsingDot() {
  LGP_NodeL all = root->getAll();
  for(auto& n:all) n->note.clear();

  for(auto& n:all) if(n->isInfeasible) n->note <<"INFEASIBLE ";
  for(auto& n:fringe_expand)      n->note <<"EXPAND ";
  for(auto& n:terminals) n->note <<"TERMINAL ";
  for(auto& n:fringe_pose)  n->note <<"POSE ";
  for(auto& n:fringe_poseToGoal) n->note <<"POSE2 ";
  for(auto& n:fringe_seq)  n->note <<"SEQ ";
  for(auto& n:fringe_path)  n->note <<"PATH ";
  for(auto& n:fringe_solved) n->note <<"DONE";

  Graph dot=root->getGraph(false);
  dot.writeDot(FILE("z.dot"));
  rai::system("dot -Tpdf z.dot > z.pdf");
  if(firstTimeDisplayTree) {
    rai::system("evince z.pdf &");
    firstTimeDisplayTree=false;
  }
}

void LGP_Tree::updateDisplay() {
  if(fringe_solved.N) focusNode = fringe_solved.last();

  if(!dth) initDisplay();

  //individual windows to display focusNode
  if(verbose>2) {
    rai::String decisions = focusNode->getTreePathString('\n');
    for(uint i=1; i<views.N; i++) {
      if(focusNode->komoProblem(i) && focusNode->komoProblem(i)->timeSlices.N) {
        NIY//views(i)->setConfigurations(focusNode->komoProblem(i)->configurations);
        views(i)->text.clear() <<focusNode->cost <<"|  " <<focusNode->constraints.last() <<'\n' <<decisions;
      } else views(i)->clear();
    }
  }

  //big windows to display solution buffer
  solutions.writeAccess();
  for(uint i=0; i<solutions().N && i<6; i++) {
    if(dth->gl.views.N<=i || !dth->gl.views(i).drawers.N) {
      dth->gl.addSubView(i, glStandardScene, nullptr);
      dth->gl.addSubView(i, *solutions()(i));
      dth->gl.views(i).camera.setDefault();
      if(cameraFocus.N) dth->gl.views(i).camera.focus(cameraFocus, true);
      //      dth->gl.views(i).camera.focus(.9, 0., 1.3);
    }
    dth->gl.views(i).drawers.last() = solutions()(i);
    dth->gl.views(i).text.clear() <<solutions()(i)->node->cost <<'\n' <<solutions()(i)->decisions;
  }
  dth->gl.setSubViewTiles(3, 2);
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
    displayTreeUsingDot();
  }
}

void LGP_Tree::printChoices() {
  //-- query UI
  cout <<"********************" <<endl;
  cout <<"NODE:\n" <<*focusNode <<endl;
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
  for(LGP_Node* a:focusNode->children) {
    cout <<"(" <<c++ <<") DECISION: " <<*a->decision <<endl;
  }
}

rai::String LGP_Tree::queryForChoice() {
  rai::String cmd;
  std::string tmp;
  getline(std::cin, tmp);
  cmd=tmp.c_str();
  return cmd;
}

bool LGP_Tree::execRandomChoice() {
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
    cmd <<rnd(focusNode->children.N);
  }
  return execChoice(cmd);
}

void LGP_Tree::inspectSequence(const rai::String& seq) {
  LGP_Node* node = walkToNode(seq);
  LGP_NodeL path = node->getTreePath();

  cout <<"### INSPECT SEQUENCE\n  " <<seq <<endl;
  cout <<"  Node Info:\n" <<node->getInfo() <<endl;
  auto S = node->getSkeleton();
  writeSkeleton(cout, S, getSwitchesFromSkeleton(S, kin));

  ptr<OpenGL> gl = make_shared<OpenGL>();
  gl->camera.setDefault();

  //-- first test pose bounds along the path
  BoundType bound = BD_pose;
//  for(LGP_Node* n:path) {
//    n->optBound(bound, true, verbose-2);
//    n->displayBound(gl, bound);
//  }

//  bound = BD_poseFromSeq;
//  for(LGP_Node* n:path) {
//    n->optBound(bound, true, verbose-2);
//    n->displayBound(gl, bound);
//  }

  //-- sequence bound
  bound = BD_seq;
  node->optBound(bound, true, verbose-2);
  node->displayBound(gl, bound);

  //-- path bounds
  bound = BD_seqPath;
  node->optBound(bound, true, verbose-2);
  node->displayBound(gl, bound);

  bound = BD_path;
  node->optBound(bound, true, verbose-2);
  node->displayBound(gl, bound);
}

void LGP_Tree::player(StringA cmds) {
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

LGP_Node* LGP_Tree::walkToNode(const rai::String& seq) {
  init();
  Graph& tmp = root->fol.KB.newSubgraph({"TMP"}, {});
  rai::String tmpseq(seq);
  tmp.read(tmpseq);
  cout <<"decision sequence:" <<*tmp.isNodeOfGraph <<endl;

  //first walk to the node that corresponds to seq
  LGP_Node* node = root;
  for(Node* actionLiteral:tmp) {
//    if(specificBound==BD_all || specificBound==BD_pose) node->optBound(BD_pose, collisions); //optimize poses along the path
    if(!node->isExpanded) node->expand();
    LGP_Node* next = node->getChildByAction(actionLiteral);
    if(!next) LOG(-2) <<"action '" <<*actionLiteral <<"' is not a child of '" <<*node <<"'";
    node = next;
  }

  focusNode = node;
  return node;
}

void LGP_Tree::optFixedSequence(const rai::String& seq, BoundType specificBound, bool collisions) {
  initDisplay();

  //parse the string to decision predicates by making it a node of the logic graph
  LGP_Node* node = walkToNode(seq);

  updateDisplay();

  //then compute the desired bound
  if(specificBound==BD_all || specificBound==BD_pose) node->optBound(BD_pose, collisions, verbose-2);
  if(specificBound==BD_all || specificBound==BD_seq)  node->optBound(BD_seq, collisions, verbose-2);
  if(specificBound==BD_all || specificBound==BD_path) node->optBound(BD_path, collisions, verbose-2);
  if(specificBound==BD_all || specificBound==BD_seqPath) node->optBound(BD_seqPath, collisions, verbose-2);

  focusNode = node;

  solutions.set()->append(new LGP_Tree_SolutionData(*this, node));
  solutions.set()->sort(sortComp2);

  updateDisplay();
}

void LGP_Tree::optMultiple(const StringA& seqs) {
  for(const rai::String& seq:seqs) optFixedSequence(seq);

  rai::system(STRING("mkdir -p " <<OptLGPDataPath <<"vid"));
  rai::system(STRING("rm -f " <<OptLGPDataPath <<"vid/*.ppm"));
  dth->resetSteppings();
  dth->saveVideo = true;
  rai::wait(20.);
}

void LGP_Tree::writeNodeList(std::ostream& os) {
  os <<"id step cost= C0 C1 C2 C3 constr= R0 R1 R2 R3 fea= F0 F1 F2 F3 time= T0 T1 T2 T3 skeleton" <<endl;
  LGP_NodeL ALL = root->getAll();
  for(LGP_Node* n : ALL) {
    //    if(n->count(l_pose)){
    os <<n->id <<' ' <<n->step
       <<" cost= " <<n->cost <<" constr= " <<n->constraints <<" fea= " <<convert<int>(n->feasible) <<" time= " <<n->computeTime <<" \"" <<n->getTreePathString() <<"\"" <<endl;
    //    }
  }
}

void LGP_Tree::glDraw(OpenGL& gl) {
}

bool LGP_Tree::execChoice(rai::String cmd) {
  cout <<"COMMAND: '" <<cmd <<"'" <<endl;

  if(cmd=="q") return false;
  else if(cmd=="u") { if(focusNode->parent) focusNode = focusNode->parent; }
  else if(cmd=="e") focusNode->expand();
  else if(cmd=="p") focusNode->optBound(BD_pose, collisions, verbose-2);
  else if(cmd=="s") focusNode->optBound(BD_seq, collisions, verbose-2);
  else if(cmd=="x") focusNode->optBound(BD_path, collisions, verbose-2);
  //  else if(cmd=="m") node->addMCRollouts(100,10);
  else {
    int choice=-1;
    cmd >>choice;
    cout <<"CHOICE=" <<choice <<endl;
    if(choice<0 || choice>=(int)focusNode->children.N) {
      cout <<"--- there is no such choice" <<endl;
    } else {
      focusNode = focusNode->children(choice);
      if(!focusNode->isExpanded) focusNode->expand();
    }
  }
  return true;
}

LGP_Node* LGP_Tree::getBest(LGP_NodeL& fringe, uint level) {
  if(!fringe.N) return nullptr;
  LGP_Node* best=nullptr;
  for(LGP_Node* n:fringe) {
    if(n->isInfeasible || !n->count(level)) continue;
    if(!best || (n->feasible(level) && n->cost(level)<best->cost(level))) best=n;
  }
  return best;
}

LGP_Node* LGP_Tree::popBest(LGP_NodeL& fringe, uint level) {
  if(!fringe.N) return nullptr;
  LGP_Node* best=getBest(fringe, level);
  if(!best) return nullptr;
  fringe.removeValue(best);
  return best;
}

LGP_Node* LGP_Tree::expandNext(int stopOnDepth, LGP_NodeL* addIfTerminal) { //expand
  //    MNode *n =  popBest(fringe_expand, 0);
  if(!fringe_expand.N) HALT("the tree is dead!");
  LGP_Node* n =  fringe_expand.popFirst();

  CHECK(n, "");
  if(stopOnDepth>0 && n->step>=(uint)stopOnDepth) return nullptr;
  n->expand();
  for(LGP_Node* ch:n->children) {
    if(ch->isTerminal) {
      terminals.append(ch);
      LGP_NodeL path = ch->getTreePath();
      for(LGP_Node* n:path) if(!n->count(1)) fringe_poseToGoal.setAppend(n); //pose2 is a FIFO
    } else {
      fringe_expand.append(ch);
    }
    if(addIfTerminal && ch->isTerminal) addIfTerminal->append(ch);
    if(n->count(1)) fringe_pose.append(ch);
  }
  return n;
}

void LGP_Tree::optBestOnLevel(BoundType bound, LGP_NodeL& drawFringe, BoundType drawFrom, LGP_NodeL* addIfTerminal, LGP_NodeL* addChildren) { //optimize a seq
  if(!drawFringe.N) return;
  LGP_Node* n = popBest(drawFringe, drawFrom);
  if(n && !n->count(bound)) {
    try {
      n->optBound(bound, collisions, verbose-2);
    } catch(const char* err) {
      LOG(-1) <<"opt(level=" <<bound <<") has failed for the following node:";
      n->write(cout, false, true);
      LOG(-3) <<"node optimization failed";
    }

    if(n->feasible(bound)) {
      if(addIfTerminal && n->isTerminal) addIfTerminal->append(n);
      if(addChildren) for(LGP_Node* c:n->children) addChildren->append(c);
    }
    focusNode = n;
  }
}

void LGP_Tree::optFirstOnLevel(BoundType bound, LGP_NodeL& fringe, LGP_NodeL* addIfTerminal) {
  if(!fringe.N) return;
  LGP_Node* n =  fringe.popFirst();
  if(n && !n->count(bound)) {
    try {
      n->optBound(bound, collisions, verbose-2);
    } catch(const char* err) {
      LOG(-1) <<"opt(bound=" <<bound <<") has failed for the following node:";
      n->write(cout, false, true);
      LOG(-3) <<"node optimization failed";
    }

    if(n->feasible(bound)) {
      if(addIfTerminal && n->isTerminal) addIfTerminal->append(n);
    }
    focusNode = n;
  }
}

void LGP_Tree::clearFromInfeasibles(LGP_NodeL& fringe) {
  for(uint i=fringe.N; i--;)
    if(fringe.elem(i)->isInfeasible) fringe.remove(i);
}

uint LGP_Tree::numFoundSolutions() {
  return fringe_solved.N;
}

rai::String LGP_Tree::report(bool detailed) {
  LGP_Node* bpose = getBest(terminals, 1);
  LGP_Node* bseq  = getBest(terminals, 2);
  LGP_Node* bpath = getBest(fringe_solved, 3);

  rai::String out;
  out <<"TIME= " <<rai::cpuTime() <<" TIME= " <<COUNT_time <<" KIN= " <<COUNT_kin <<" EVALS= " <<COUNT_evals << " TREE= " <<COUNT_node
      <<" POSE= " <<COUNT_opt(BD_pose) <<" SEQ= " <<COUNT_opt(BD_seq) <<" PATH= " <<COUNT_opt(BD_path)+COUNT_opt(BD_seqPath)
      <<" bestPose= " <<(bpose?bpose->cost(1):100.)
      <<" bestSeq= " <<(bseq ?bseq ->cost(2):100.)
      <<" bestPath= " <<(bpath?bpath->cost(displayBound):100.)
      <<" #solutions= " <<fringe_solved.N;

  //  if(bseq) displayFocus=bseq;
  //  if(bpath) displayFocus=bpath;

  if(detailed) {
    out <<"\n*** found solutions:" <<endl;
    for(LGP_Node* n:fringe_solved) n->write(out, false, true);
  }

  return out;
}

void LGP_Tree::reportEffectiveJoints() {
  //  MNode *best = getBest();
  if(!focusNode->komoProblem.last()) return;
  focusNode->komoProblem.last()->reportProblem();
  NIY//focusNode->komoProblem.last()->reportEffectiveJoints();
}

void LGP_Tree::step() {
  expandNext();

  uint numSol = fringe_solved.N;

//  if(rnd.uni()<.5) optBestOnLevel(BD_pose, fringe_pose, BD_symbolic, &fringe_seq, &fringe_pose);
  optFirstOnLevel(BD_pose, fringe_poseToGoal, &fringe_seq);
  optBestOnLevel(BD_seq, fringe_seq, BD_pose, &fringe_path, nullptr);
  if(verbose>0 && fringe_path.N) cout <<"EVALUATING PATH " <<fringe_path.last()->getTreePathString() <<endl;
  optBestOnLevel(BD_seqPath, fringe_path, BD_seq, &fringe_solved, nullptr);

  if(fringe_solved.N>numSol) {
    if(verbose>0) cout <<"NEW SOLUTION FOUND! " <<fringe_solved.last()->getTreePathString() <<endl;
    solutions.set()->append(new LGP_Tree_SolutionData(*this, fringe_solved.last()));
    solutions.set()->sort(sortComp2);
  }

  //-- update queues (if something got infeasible)
  clearFromInfeasibles(fringe_expand);
  clearFromInfeasibles(fringe_pose);
  clearFromInfeasibles(fringe_poseToGoal);
  clearFromInfeasibles(fringe_seq);
  clearFromInfeasibles(fringe_path);
  clearFromInfeasibles(terminals);

  if(verbose>0) {
    rai::String out=report();
    fil <<out <<endl;
    cout <<out <<endl;
    if(verbose>1 && !(numSteps%1)) updateDisplay();
  }
  numSteps++;
}

void LGP_Tree::buildTree(uint depth) {
  init();

  if(verbose>0) {
    cout <<"BULDING TREE to depth " <<depth <<endl;
  }

  rai::timerRead(true);
  for(uint k=0;; k++) {
    LGP_Node* b = expandNext(depth);
    if(!b) break;
  }

  if(verbose>0) {
    rai::String out=report();
    fil <<out <<endl;
    cout <<out <<endl;
    if(verbose>1) updateDisplay();
  }
}

void LGP_Tree::getSymbolicSolutions(uint depth) {
  buildTree(depth);
  uint i=0;
  for(LGP_Node* a:terminals) {
    cout <<"solution " <<i <<": " <<a->getTreePathString() <<endl;
    i++;
  }
  if(!terminals.N) cout <<"NO SOLUTIONS up to depth " <<depth <<endl;
}

void LGP_Tree::init() {
  fringe_expand.append(root);
  fringe_pose.append(root);
//  if(verbose>1) {
//    initDisplay();
//    updateDisplay();
//  }
}

void LGP_Tree::run(uint steps) {
  init();

  uint stopSol = rai::getParameter<double>("LGP/stopSol", 12);
  double stopTime = rai::getParameter<double>("LGP/stopTime", 400.);

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
  if(verbose>3) {
    //    renderToVideo();
    rai::system(STRING("mkdir -p " <<OptLGPDataPath <<"vid"));
    rai::system(STRING("rm -f " <<OptLGPDataPath <<"vid/*.ppm"));
    dth->resetSteppings();
    dth->saveVideo = true;
    rai::wait(20.);
  }

  if(verbose>1) views.clear();
}

LGP_Tree_SolutionData::LGP_Tree_SolutionData(LGP_Tree& _tree, LGP_Node* _node) : tree(_tree), node(_node) {
  decisions = node->getTreePathString('\n');

  //--init geoms
  const rai::Configuration& K = node->startKinematics;
  uintA frameIDs;
  for(uint f=0; f<K.frames.N; f++) {
    const rai::Frame* a = K.frames(f);
    if(a->shape && a->shape->_mesh && a->shape->type()!=rai::ST_marker) {
      frameIDs.append(a->ID);
    }
  }
  geoms.resize(frameIDs.N);
  for(uint i=0; i<geoms.N; i++) geoms(i) = K.frames(frameIDs(i))->shape->_mesh;

  uint L = node->komoProblem.N;
  paths.resize(L);
  for(uint l=0; l<L; l++) {
    std::shared_ptr<KOMO> komo = node->komoProblem(l);
    if(komo && komo->timeSlices.N) {
      paths(l).resize(komo->timeSlices.d0, frameIDs.N);
      for(uint s=0; s<komo->timeSlices.d0; s++) for(uint i=0; i<frameIDs.N; i++) {
          paths(l)(s, i) = komo->timeSlices(s, frameIDs(i))->ensure_X();
        }
    }
  }
}

void LGP_Tree_SolutionData::write(std::ostream& os) const {
  os <<"decisions=" <<decisions
     <<"\t depth=" <<node->step
     <<"\t costs=" <<node->cost
     <<endl;
}

void LGP_Tree_SolutionData::glDraw(OpenGL& gl) {
#ifdef RAI_GL
  BoundType displayBound=tree.displayBound;

  if(!paths(displayBound).N) return;

  for(uint i=0; i<geoms.N; i++) {
    if(displayStep >= paths(displayBound).d0) displayStep = 0;
    rai::Transformation& X = paths(displayBound)(displayStep, i);
    double GLmatrix[16];
    X.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);

    geoms(i)->glDraw(gl);
  }
#endif
}
