#include "manipulationTree.h"

typedef ManipulationTree_Node MNode;
typedef ManipulationTree_NodeL MNodeL;
struct OrsPathViewer;


struct OptLGP{
  int verbose;
  uint numSteps;
  ofstream fil;


  MNode *root, *displayFocus;

  mlr::Array<std::shared_ptr<OrsPathViewer>> views; //displays for the 3 different levels

  //-- these are lists or queues; I don't maintain them sorted because their evaluation (e.g. f(n)=g(n)+h(n)) changes continuously
  // while new bounds are computed. Therefore, whenever I pop from these lists, I find the minimum w.r.t. a heuristic. The
  // heuristics are defined in main.cpp currently
  MNodeL fringe_expand;//list of nodes to be expanded next
  MNodeL terminals;    //list of found terminals

  MNodeL fringe_pose;  //list of nodes that can be pose tested (parent has been tested)
  MNodeL fringe_pose2; //list of nodes towards a terminal -> scheduled for pose testing
  MNodeL fringe_seq;   //list of terminal nodes that have been pose tested
  MNodeL fringe_path;  //list of terminal nodes that have been seq tested
  MNodeL fringe_done;  //list of terminal nodes that have been path tested

  //high-level
  OptLGP(mlr::KinematicWorld& kin, FOL_World& fol);
  ~OptLGP();

  FOL_World& fol(){ return root->fol; }
  const mlr::KinematicWorld& kin(){ return root->startKinematics; }


  //-- for methods called in the run loop
private:
  MNode* getBest(MNodeL& fringe, uint level);
  MNode* popBest(MNodeL& fringe, uint level);
  MNode* getBest(){ return getBest(fringe_done, 3); }
  MNode *expandBest(int stopOnLevel=-1);
  void optBestOnLevel(int level, MNodeL& fringe, MNodeL* addIfTerminal, MNodeL* addChildren);
  void optFirstOnLevel(int level, MNodeL& fringe, MNodeL* addIfTerminal);
  void clearFromInfeasibles(MNodeL& fringe);
public:
  void run(uint steps=10000);
  void init();
  void step();
  void buildTree(uint depth);

  // output
  uint numFoundSolutions();
  mlr::String report(bool detailed=false);
  void initDisplay();
  void updateDisplay();
  void renderToVideo(uint level=3, const char* filePrefix="z.vid/z.path.");

  //-- kind of a gui:
  void printChoices();
  mlr::String queryForChoice();
  bool execChoice(mlr::String cmd);
  bool execRandomChoice();

  void player(StringA cmds={});

  void optFixedSequence(const mlr::String& seq, bool fullPathOnly=false, bool collisions=false);
};
