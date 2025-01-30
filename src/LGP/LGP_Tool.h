#pragma once

#include "Motif.h"

#include <LGP/LGP_SkeletonTool.h>
#include <KOMO/manipTools.h>

namespace rai {

//===========================================================================

struct Actions2KOMO_Translator {
  virtual ~Actions2KOMO_Translator() {}
  virtual std::shared_ptr<KOMO> setup_sequence(Configuration& C, uint K) = 0;
  virtual void add_action_constraints(std::shared_ptr<KOMO>& komo, double time, const StringA& action) = 0;
  virtual void add_action_constraints_motion(std::shared_ptr<KOMO>& komo, double time, const StringA& prev_action, const StringA& action, uint actionPhase) = 0;
};

struct TAMP_Provider{
  virtual ~TAMP_Provider() {}
  virtual Array<StringA> getNewPlan() = 0;
  virtual Configuration& getConfig() = 0;
  virtual StringA explicitCollisions() = 0;
};

//===========================================================================

struct ActionNode;
typedef Array<ActionNode*> ActionNodeL;

//===========================================================================

enum JobTag { _solve_ways, _solve_motif, _new_plan };

struct Job{
  uint ID=0;

  double priority=0., succProb=1.;

  //job definition
  ActionNode *a=0;
  KOMO_Motif* motif=0;
  JobTag tag;
  Array<Job*> dependencies;

  //results
  uint n_fail=0, n_succ=0;
  Array<PTR<SolverReturn>> rets;

  //
  Job(double _priority, ActionNode *_a, KOMO_Motif* _m, JobTag _tag);
  static bool compare_priority(const Job* a, const Job* b);
  static bool compare_succProb(const Job* a, const Job* b);
  void write(ostream& os) const;
  void update();
  double getSuccProb();
  str niceMsg();
};
stdOutPipe(Job)

//===========================================================================

struct ActionNode{
  ActionNode* parent;
  Array<ActionNode*> children;
  uint step=0;

  bool isTerminal=false;
  StringA action; //(only for display/debugging) in symbolic form: the grounded action predicate
  Array<StringA> komo_constraints; //in symbolic form: every constraint is a 'grounded literal' of the logic state. The first string: predicate symbol, other strings: frame names to ground the literal

  std::shared_ptr<Job> ways_job;

  ActionNode(ActionNode* _parent, StringA _action);
  ~ActionNode();

  PTR<KOMO>& get_ways(Configuration& C, Actions2KOMO_Translator& trans, const StringA& explicitCollisions);
  Array<PTR<KOMO_Motif>>& getWayMotifs();


  ActionNodeL getTreePath() const;

  ActionNode* descentAndCreate(const Array<StringA>& plan);

  //-- convenience output/debug
  str getPlanString();
  Array<StringA> getPlan();

protected:
  PTR<KOMO> ways;
  Array<PTR<KOMO_Motif>> waysMotifs;

  friend struct LGP_Tool;
};

//===========================================================================

struct LGP_Tool{
  //problem interface
  Configuration& C;
  TAMP_Provider& tamp;
  Actions2KOMO_Translator& trans;
  int verbose=1;

  //internal data structures for action search and job management
  ActionNode* actionTreeRoot;
  PTR<Job> newPlanJob;
  Array<PTR<Job>> motif_jobs;
  std::map<std::string, Job*> motifResults;
  Array<Job*> jobs;
  Array<ActionNode*> open_terminal_nodes;
  ActionNodeL solutions;
  uint step_count=0;

  LGP_Tool(const char* lgp_configfile);
  LGP_Tool(Configuration& _C, TAMP_Provider& _tamp, Actions2KOMO_Translator& _trans);
  ~LGP_Tool();

  void solve_step();

  void solve(int _verbose=-1);
  StringAA getSolvedPlan();
  PTR<KOMO> getSolvedKOMO();
  int view_solved(bool pause=true);
  void view_close();

  arrA solvePiecewiseMotions(int _verbose=1);
  std::shared_ptr<KOMO> solveFullMotion(int _verbose=1);

  std::shared_ptr<KOMO> get_piecewiseMotionProblem(uint phase, bool fixEnd);
  std::shared_ptr<KOMO> get_fullMotionProblem(bool initWithWaypoints);


  int display(PTR<KOMO>& komo, PTR<SolverReturn>& ret, bool pause=true, const char* msg=0, bool play=true);

private:
  //helpers
  ActionNode *addNewOpenPlan();
  PTR<OpenGL> gl;
  PTR<KOMO> gl_komo;
};

//===========================================================================

MotifL analyzeMotifs(KOMO& komo, int verbose=0);
PTR<TAMP_Provider> default_TAMP_Provider(rai::Configuration &C, const char* lgp_configfile);
PTR<Actions2KOMO_Translator> default_Actions2KOMO_Translator();

} //namespace
