/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "LGP_node.h"
#include "../Core/thread.h"

struct KinPathViewer;
struct LGP_Tree;
struct DisplayThread;
typedef rai::Array<rai::Transformation> TransformationA;

void initFolStateFromKin(FOL_World& L, const rai::Configuration& K);

struct LGP_Tree_SolutionData : GLDrawer {
  LGP_Tree& tree;
  LGP_Node* node; ///< contains costs, constraints, and solutions for each level
  rai::String decisions;

  rai::Array<ptr<rai::Mesh>> geoms; ///< for display
  rai::Array<TransformationA> paths; ///< for display
  uint displayStep=0;

  LGP_Tree_SolutionData(LGP_Tree& _tree, LGP_Node* _node);

  void write(ostream& os) const;
  void glDraw(struct OpenGL& gl);
};

struct LGP_Tree : GLDrawer {
  int verbose;
  uint numSteps;
  ofstream fil;
  bool displayTree=true;
  BoundType displayBound=BD_seqPath;
  bool collisions=false;
  shared_ptr<DisplayThread> dth;
  rai::String dataPath;
  arr cameraFocus;
  bool firstTimeDisplayTree=true;

  LGP_Node* root=0, *focusNode=0;
  FOL_World fol;
  rai::Configuration kin;

  KOMO finalGeometryObjectives;

  rai::Array<std::shared_ptr<KinPathViewer>> views; //displays for the 3 different levels

  //-- these are lists or queues; I don't maintain them sorted because their evaluation (e.g. f(n)=g(n)+h(n)) changes continuously
  // while new bounds are computed. Therefore, whenever I pop from these lists, I find the minimum w.r.t. a heuristic. The
  // heuristics are defined in main.cpp currently
  LGP_NodeL fringe_expand;//list of nodes to be expanded next
  LGP_NodeL terminals;    //list of found terminals

  LGP_NodeL fringe_pose;  //list of nodes that can be pose tested (parent has been tested)
  LGP_NodeL fringe_poseToGoal; //list of nodes towards a terminal -> scheduled for pose testing
  LGP_NodeL fringe_seq;   //list of terminal nodes that have been pose tested
  LGP_NodeL fringe_path;  //list of terminal nodes that have been seq tested
  LGP_NodeL fringe_solved;  //list of terminal nodes that have been path tested

  Var<rai::Array<LGP_Tree_SolutionData*>> solutions;

  //high-level
  LGP_Tree();
  LGP_Tree(const rai::Configuration& _kin, const char* folFileName="fol.g");
  LGP_Tree(const rai::Configuration& _kin, const FOL_World& _fol);
  ~LGP_Tree();

  //-- methods called in the run loop
 private:
  LGP_Node* getBest(LGP_NodeL& fringe, uint level);
  LGP_Node* popBest(LGP_NodeL& fringe, uint level);
  LGP_Node* getBest() { return getBest(fringe_solved, 3); }
  LGP_Node* expandNext(int stopOnLevel=-1, LGP_NodeL* addIfTerminal=nullptr);

  void optBestOnLevel(BoundType bound, LGP_NodeL& drawFringe, BoundType drawBound, LGP_NodeL* addIfTerminal, LGP_NodeL* addChildren);
  void optFirstOnLevel(BoundType bound, LGP_NodeL& fringe, LGP_NodeL* addIfTerminal);
  void clearFromInfeasibles(LGP_NodeL& fringe);

 public:
  void run(uint steps=10000);
  void init();
  void step();
  void buildTree(uint depth);
  void getSymbolicSolutions(uint depth);
  void optFixedSequence(const rai::String& seq, BoundType specificBound=BD_all, bool collisions=false);
  void optMultiple(const StringA& seqs);

  //-- work directly on the tree
  LGP_Node* walkToNode(const rai::String& seq);

  // output
  uint numFoundSolutions();
  rai::String report(bool detailed=false);
  void reportEffectiveJoints();
  void displayTreeUsingDot();
  void initDisplay();
  void updateDisplay();
  void renderToVideo(int specificBound=-1, const char* filePrefix="vid/");
  void writeNodeList(ostream& os=cout);
  void glDraw(struct OpenGL& gl);

  //-- kind of a gui:
  void printChoices();
  rai::String queryForChoice();
  bool execChoice(rai::String cmd);
  bool execRandomChoice();

  //-- inspection and debugging
  void inspectSequence(const rai::String& seq);
  void player(StringA cmds= {});
};

struct LGP_Tree_Thread : LGP_Tree, Thread {
  LGP_Tree_Thread(const rai::Configuration& _kin, const char* folFileName="fol.g")
    : LGP_Tree(_kin, folFileName), Thread("LGP_Tree", -1) {}

  void open() { LGP_Tree::init(); }
  void step() { LGP_Tree::step(); }
  void close() {}

  //convenience to retrieve solution data
  uint numSolutions() { return solutions.get()->N; }

  const std::shared_ptr<KOMO>& getKOMO(uint i, BoundType bound) {
    const auto& komo = solutions.get()->elem(i)->node->komoProblem(bound);
    CHECK(komo, "solution " <<i <<" has not evaluated the bound " <<bound <<" -- returning KOMO reference to nil");
    return komo;
  }

  Graph getReport(uint i, BoundType bound) {
    const auto& komo = getKOMO(i, bound);
    if(!komo) return Graph();
    return komo->getProblemGraph(true);
  }

};
