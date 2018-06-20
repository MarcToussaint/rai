/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "manipulationTree.h"
//#include <Geo/geoms.h>
#include <Core/thread.h>

struct OrsPathViewer;
typedef rai::Array<rai::Transformation> TransformationA;

void initFolStateFromKin(FOL_World& L, const rai::KinematicWorld& K);

struct OptLGP_SolutionData : GLDrawer {
  MNode *node; ///< contains costs, constraints, and solutions for each level
  rai::String decisions;
  
  uintA geomIDs; ///< for display
  rai::Array<TransformationA> paths; ///< for display
  uint displayStep=0;
  
  OptLGP_SolutionData(MNode *n);
  
  void write(ostream &os) const;
  void glDraw(struct OpenGL&gl);
};

struct OptLGP : GLDrawer {
  int verbose;
  uint numSteps;
  ofstream fil;
  bool displayTree=true;
  struct DisplayThread *dth=NULL;
  rai::String dataPath;
  
  MNode *root, *displayFocus;
  
  rai::Array<std::shared_ptr<OrsPathViewer>> views; //displays for the 3 different levels
  
  //-- these are lists or queues; I don't maintain them sorted because their evaluation (e.g. f(n)=g(n)+h(n)) changes continuously
  // while new bounds are computed. Therefore, whenever I pop from these lists, I find the minimum w.r.t. a heuristic. The
  // heuristics are defined in main.cpp currently
  MNodeL fringe_expand;//list of nodes to be expanded next
  MNodeL terminals;    //list of found terminals
  
  MNodeL fringe_pose;  //list of nodes that can be pose tested (parent has been tested)
  MNodeL fringe_pose2; //list of nodes towards a terminal -> scheduled for pose testing
  MNodeL fringe_seq;   //list of terminal nodes that have been pose tested
  MNodeL fringe_path;  //list of terminal nodes that have been seq tested
  MNodeL fringe_solved;  //list of terminal nodes that have been path tested
  
  Var<rai::Array<OptLGP_SolutionData*>> solutions;
  
  //high-level
  OptLGP(rai::KinematicWorld& kin, FOL_World& fol);
  ~OptLGP();
  
  FOL_World& fol() { return root->fol; }
  const rai::KinematicWorld& kin() { return root->startKinematics; }
  
  //-- methods called in the run loop
private:
  MNode* getBest(MNodeL& fringe, uint level);
  MNode* popBest(MNodeL& fringe, uint level);
  MNode* getBest() { return getBest(fringe_solved, 3); }
  MNode *expandBest(int stopOnLevel=-1);
  void optBestOnLevel(int level, MNodeL& fringe, MNodeL* addIfTerminal, MNodeL* addChildren);
  void optFirstOnLevel(int level, MNodeL& fringe, MNodeL* addIfTerminal);
  void clearFromInfeasibles(MNodeL& fringe);
  
public:
  void run(uint steps=10000);
  void init();
  void step();
  void buildTree(uint depth);
  void getSymbolicSolutions(uint depth);
  void optFixedSequence(const rai::String& seq, int specificLevel=-1, bool collisions=false);
  void optMultiple(const StringA& seqs);
  
  // output
  uint numFoundSolutions();
  rai::String report(bool detailed=false);
  void reportEffectiveJoints();
  void initDisplay();
  void updateDisplay();
  void renderToVideo(uint level=3, const char* filePrefix="vid/z.");
  void writeNodeList(ostream& os=cout);
  void glDraw(struct OpenGL&gl);
  
  //-- kind of a gui:
  void printChoices();
  rai::String queryForChoice();
  bool execChoice(rai::String cmd);
  bool execRandomChoice();
  
  void player(StringA cmds={});
};
