/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "LGP_node.h"
//#include <Geo/geoms.h>
#include <Core/thread.h>

struct KinPathViewer;
typedef rai::Array<rai::Transformation> TransformationA;

void initFolStateFromKin(FOL_World& L, const rai::KinematicWorld& K);

struct LGP_Tree_SolutionData : GLDrawer {
  LGP_Node *node; ///< contains costs, constraints, and solutions for each level
  rai::String decisions;
  
  uintA geomIDs; ///< for display
  rai::Array<TransformationA> paths; ///< for display
  uint displayStep=0;
  
  LGP_Tree_SolutionData(LGP_Node *n);
  
  void write(ostream &os) const;
  void glDraw(struct OpenGL&gl);
};

struct LGP_Tree : GLDrawer {
  int verbose;
  uint numSteps;
  ofstream fil;
  bool displayTree=true;
  bool collisions=false;
  struct DisplayThread *dth=NULL;
  rai::String dataPath;
  arr cameraFocus;
  
  LGP_Node *root=0, *focusNode=0;
  FOL_World *selfCreated=NULL;
  
  rai::Array<std::shared_ptr<KinPathViewer>> views; //displays for the 3 different levels
  
  //-- these are lists or queues; I don't maintain them sorted because their evaluation (e.g. f(n)=g(n)+h(n)) changes continuously
  // while new bounds are computed. Therefore, whenever I pop from these lists, I find the minimum w.r.t. a heuristic. The
  // heuristics are defined in main.cpp currently
  MNodeL fringe_expand;//list of nodes to be expanded next
  MNodeL terminals;    //list of found terminals
  
  MNodeL fringe_pose;  //list of nodes that can be pose tested (parent has been tested)
  MNodeL fringe_poseToGoal; //list of nodes towards a terminal -> scheduled for pose testing
  MNodeL fringe_seq;   //list of terminal nodes that have been pose tested
  MNodeL fringe_path;  //list of terminal nodes that have been seq tested
  MNodeL fringe_solved;  //list of terminal nodes that have been path tested
  
  Var<rai::Array<LGP_Tree_SolutionData*>> solutions;
  
  //high-level
  LGP_Tree();
  LGP_Tree(rai::KinematicWorld& kin, const char *folFileName);
  LGP_Tree(rai::KinematicWorld& kin, FOL_World& fol);
  void init(rai::KinematicWorld &kin, FOL_World &fol);
  ~LGP_Tree();
  
  FOL_World& fol() { return root->fol; }
  const rai::KinematicWorld& kin() { return root->startKinematics; }
  
  //-- methods called in the run loop
private:
  LGP_Node* getBest(MNodeL& fringe, uint level);
  LGP_Node* popBest(MNodeL& fringe, uint level);
  LGP_Node* getBest() { return getBest(fringe_solved, 3); }
  LGP_Node *expandNext(int stopOnLevel=-1, MNodeL* addIfTerminal=NULL);

  void optBestOnLevel(BoundType bound, MNodeL& drawFringe, BoundType drawBound, MNodeL* addIfTerminal, MNodeL* addChildren);
  void optFirstOnLevel(BoundType bound, MNodeL& fringe, MNodeL* addIfTerminal);
  void clearFromInfeasibles(MNodeL& fringe);
  
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
  void initDisplay();
  void updateDisplay();
  void renderToVideo(uint specificBound=3, const char* filePrefix="vid/");
  void writeNodeList(ostream& os=cout);
  void glDraw(struct OpenGL&gl);
  
  //-- kind of a gui:
  void printChoices();
  rai::String queryForChoice();
  bool execChoice(rai::String cmd);
  bool execRandomChoice();
  
  void player(StringA cmds={});
};
