/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "LGP_node.h"
#include "../Core/thread.h"

struct ConfigurationViewer;
struct ConfigurationViewerThread;

namespace rai {

struct LGP_Tree;
struct DisplayThread;
typedef Array<Transformation> TransformationA;

struct LGP_Tree_SolutionData {
  LGP_Tree& tree;
  LGP_Node* node; ///< contains costs, constraints, and solutions for each level
  String decisions;

  std::shared_ptr<ConfigurationViewer> viewer;

  LGP_Tree_SolutionData(LGP_Tree& _tree, LGP_Node* _node);

  void write(ostream& os) const;
};

struct LGP_Tree {
  LGP_Node* root=0, *focusNode=0;
  FOL_World fol;
  Configuration kin;

  int verbose;
  uint numSteps;
  ofstream fil;
  unique_ptr<ofstream> filNodes;
  unique_ptr<ofstream> filComputes;
  bool displayTree=true;
  BoundType displayBound=BD_seqPath;
  bool collisions=false;
  shared_ptr<DisplayThread> dth;
  shared_ptr<ConfigurationViewer> singleView;
  String dataPath;
  arr cameraFocus;
  bool firstTimeDisplayTree=true;

  uint COUNT_kin=0;
  uint COUNT_node=0;
  uintA COUNT_opt=consts<uint>(0, BD_max);
  double COUNT_time=0.;
  String OptLGPDataPath;

  Array<std::shared_ptr<ConfigurationViewerThread>> views; //displays for the 3 different levels

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

  Var<Array<LGP_Tree_SolutionData*>> solutions;

  //high-level
  LGP_Tree();
  LGP_Tree(const Configuration& _kin, const char* folFileName="fol.g");
  LGP_Tree(const Configuration& _kin, const FOL_World& _fol);
  ~LGP_Tree();

  //-- methods called in the run loop
 private:
  LGP_Node* getBest(LGP_NodeL& fringe, uint level);
  LGP_Node* popBest(LGP_NodeL& fringe, uint level);
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
  void optFixedSequence(const String& seq, BoundType specificBound=BD_all, bool collisions=false);
  void optMultiple(const StringA& seqs);

  //-- work directly on the tree
  LGP_Node* walkToNode(const String& seq);

  // output
  uint numFoundSolutions() const { return fringe_solved.N; }
  String report(bool detailed=false);
  void displayTreeUsingDot();
  void initDisplay();
  void updateDisplay();
  void renderToVideo(int specificBound=-1, const char* filePrefix="vid/");
  void writeNodeList(ostream& os=cout);

  //-- kind of a gui:
  void printChoices();
  String queryForChoice();
  bool execChoice(String& cmd);
  bool execRandomChoice();

  //-- inspection and debugging
  void inspectSequence(const String& seq);
  void player();
};

} //namespace
