#include "LGP_Tool.h"

#include "../KOMO/manipTools.h"
#include "../Gui/opengl.h"
#include "../Kin/viewer.h"
#include "../Kin/frame.h"
#include "../Kin/feature.h"
#include "../Optim/NLP_Sampler.h"

namespace rai{

//===========================================================================

ActionNode::ActionNode(ActionNode* _parent, StringA _action) : parent(_parent), action(_action){
  if(parent){
    step = parent->step+1;
    parent->children.append(this);
  }
}

ActionNode::~ActionNode(){
  ActionNodeL _children = children;
  for(ActionNode *ch: _children) delete ch;

  if(parent){
    parent->children.removeValue(this);
  }
}

PTR<KOMO>& ActionNode::get_ways(Configuration& C, Logic2KOMO_Translator& trans, const StringA& explicitCollisions){
  if(!ways){
    ActionNodeL path = getTreePath();
    //      str planString;
    //      for(ActionNode *a: path){ planString <<"[ "; for(str& s:a->action) planString <<' ' <<s; planString <<']'; }
    //      LOG(0) <<"planString: " <<planString;q

    ways = trans.setup_sequence(C, path.N-1);

    double t = 0.;
    for(ActionNode *a: path){
      trans.add_action_constraints(ways, t, a->action);
      t+=1.;
    }


    for(uint i=0; i<explicitCollisions.N; i+=2) {
      ways->addObjective({}, FS_distance, {explicitCollisions.elem(i), explicitCollisions.elem(i+1)}, OT_ineq, {1e1});
    }
  }

  return ways;
}

Array<PTR<KOMO_Motif>>& ActionNode::getWayMotifs(){
  if(!waysMotifs.N){
    waysMotifs = analyzeMotifs(*ways);
  }
  return waysMotifs;
}

ActionNodeL ActionNode::getTreePath() const {
  ActionNodeL path;
  ActionNode* node=(ActionNode*)this;
  for(; node;) {
    path.prepend(node);
    node = node->parent;
  }
  return path;
}

ActionNode* ActionNode::descentAndCreate(const Array<StringA>& plan){
  ActionNode *a = this;
  for(uint i=0;i<plan.N;i++){
    ActionNode* b=0;
    for(ActionNode* c : a->children){
      if(c->action == plan(i)){ b=c; break; }
    }
    if(b){//we found the action -> descent
      a = b;
    }else{//need to expand and create a new one
      b = new ActionNode(a, plan(i));
      a = b;
    }
  }
  CHECK_EQ(a->step, plan.N, "");
  return a;
}

str ActionNode::getPlanString(){
  ActionNodeL path = getTreePath();
  str planString;
  //    for(ActionNode *a: path) planString <<' ' <<a->action;
  for(ActionNode *a: path) if(a->parent){ planString <<"[ "; for(str& s:a->action) planString <<s <<' '; planString <<']'; }
  return planString;
}

Array<StringA> ActionNode::getPlan(){
  ActionNodeL path = getTreePath();
  Array<StringA> plan;
  for(ActionNode *a: path) if(a->parent) plan.append(a->action);
  return plan;
}

//===========================================================================

template<> const char* Enum<JobTag>::names []= {
  "solve_ways", "solve_motif", "new_plan", nullptr
};

static uint Job_ID=0;

Job::Job(double _priority, ActionNode* _a, KOMO_Motif* _m, JobTag _tag)
  : ID(Job_ID++), priority(_priority), a(_a), motif(_m), tag(_tag) {
}

bool Job::compare_priority(const Job* a, const Job* b){ return a->priority < b->priority; }

bool Job::compare_succProb(const Job* a, const Job* b){ return a->succProb < b->succProb; }

void Job::update() {
  succProb = getSuccProb();
  if(tag==_solve_ways){
    if(n_succ) priority = -100.;
    else priority = -double(n_fail);
    for(Job *d:dependencies) if(!d->n_succ){ priority = -1000.; break; }
  }
  else if(tag==_solve_motif){
    if(n_succ) priority = -10.;
    else priority = -double(n_fail);
  }
  else if(tag==_new_plan){
    priority=-1.5;
  }

}

double Job::getSuccProb(){
  double p = 1.;
//  double p = double(n_succ+1)/double(n_succ+n_fail+1);
  if(!n_succ) p = 1./double(n_fail+1);
  for(Job* d:dependencies){
    p *= d->getSuccProb();
//    if(!d->n_succ) p *= 1./double(d->n_fail+1);
//    p *= double(d->n_succ+1)/double(d->n_succ+d->n_fail+1);
  }
  return p;
}

void Job::write(std::ostream& os) const{
  os <<ID <<':' <<priority <<':' <<succProb <<": " <<Enum<JobTag>(tag) <<' ' <<a->getPlanString();
  if(motif){
    os <<" t:" <<motif->timeSlice;
    for(Frame *f:motif->F) os <<'-' <<f->name;
  }
  os <<" {succ:" <<n_succ <<" fail:" <<n_fail <<'}'; //" last:" <<*rets(-1);
  for(Job* d:dependencies) os <<'<' <<d->ID;
}

str Job::niceMsg(){
  str txt;
  if(tag==_solve_motif){
    txt <<"Motif: timeslice " <<motif->timeSlice <<" action " <<a->getTreePath()(motif->timeSlice+1)->action <<' ';
    txt <<" {succ:" <<n_succ <<" fail:" <<n_fail <<'}';
    txt <<"\nPlan: " <<a->getPlanString();
  }
  else if(tag==_solve_ways){
    txt <<"Waypoints: " <<a->getPlanString() <<' ';
    txt <<" {succ:" <<n_succ <<" fail:" <<n_fail <<'}';
  }
  return txt;
}

//===========================================================================

LGP_Tool::LGP_Tool(Configuration& _C, TAMP_Provider& _tamp, Logic2KOMO_Translator& _trans)
  : C(_C), tamp(_tamp), trans(_trans) {
  actionTreeRoot = new ActionNode(0, {});

  newPlanJob = make_shared<Job>(-1.5, actionTreeRoot, nullptr, _new_plan);
  jobs.append(newPlanJob.get());
}

LGP_Tool::~LGP_Tool(){
  view_close();
  delete actionTreeRoot;
}

ActionNode* LGP_Tool::addNewOpenPlan(){
  Array<StringA> plan = tamp.getNewPlan();
  ActionNode* a = actionTreeRoot->descentAndCreate(plan);
  a->isTerminal = true;
  return a;
}

void LGP_Tool::solve_step(){
  step_count++;

  /*
   * Strategies:
   * Distinguish goal-plans (=action sequnces logically correct) and jobs (all jobs needed to be done before completing a goal-plan
   *
   * Strategy A; First pick a goal-plan, then the first(!) or random of its jobs
   * Score per goal-plan: sum of failurs in all its jobs. Why? should be related to posterior of completing it
   * Why: Given that you only care about a single goal-plan (e.g. conditional to it being the only eventually feasible), it doesn't matter
   * at all which of its jobs to do first. Doesnt matter if some jobs contribute to other goal-plans
   *
   * Strategy B: compute goal-plan scores; make them goal-plan probabilities (softmax?); job score: sum the prob of each goal-plan that it
   *   contributes for; pick the highest job
   * Why: Now we have a mixture of goal-plans; Assuming that only ONE of them is eventually feasible, the strategy picks the job that most likely
   *   contributes to the feasible goal-plan. Does that also work when multiple are feasible? Perhaps yes: plan prob. doesn't have to be normalized:
   *   marginal prob of being feasible.
   */

  for(Job *j:jobs) j->update();

//  std::sort(jobs.p, jobs.p+jobs.N, Job::compare_priority);
  std::sort(open_terminal_nodes.p, open_terminal_nodes.p+open_terminal_nodes.N, [](const ActionNode* a, const ActionNode* b){ return a->ways_job->succProb<b->ways_job->succProb; });

  if(verbose>1){
    cout <<"+++ JOB QUEUE:" <<endl;
    for(uint i=0;i<jobs.N;i++){
      cout <<"  " <<i <<' ' <<*jobs(i) <<endl;
    }
    cout <<"+++ OPEN TERMINALS:" <<endl;
    for(ActionNode *t:open_terminal_nodes){
      cout <<"  " <<' ' <<t->ways_job->getSuccProb() <<' ' <<*t->ways_job <<endl;
    }
    cout <<"+++ SOLUTIONS:" <<endl;
    for(ActionNode *s:solutions){
      cout <<"  " <<*s->ways_job <<endl;
    }
  }

//  wait();

  Job *job = newPlanJob.get();
  if(open_terminal_nodes.N){
    job = open_terminal_nodes.elem(-1)->ways_job.get();
    if(job->succProb<.5) job = newPlanJob.get();
  }
  for(Job* d: job->dependencies) if(!d->n_succ){ job=d; break; }

  if(job->tag==_new_plan){
    ActionNode *new_terminal = addNewOpenPlan();
    open_terminal_nodes.append(new_terminal);

    //add the _solve_ways job for the full path (if not yet exists)
    ActionNodeL path = new_terminal->getTreePath();
    Array<Job*> pathJobs;
//    for(ActionNode *a:path) if(a->parent){
    for(ActionNode *a:path) if(a->isTerminal){ //OPTION! create sub_way_problems? or only motifs?
      if(a->ways_job){ //job exists
        pathJobs.append(a->ways_job.get());
      }else{ //job needs to be created
        a->ways_job = make_shared<Job>(0., a, nullptr, _solve_ways);
        a->ways_job->dependencies = pathJobs;

        if(verbose>2) cout <<"+++ addin job" <<*job <<endl;
        jobs.append(a->ways_job.get());
        pathJobs.append(a->ways_job.get());
#if 1
//        PTR<KOMO>& ways =
        a->get_ways(C, trans, tamp.explicitCollisions());
        Array<PTR<KOMO_Motif>>& ways_motifs= a->getWayMotifs();
        for(PTR<KOMO_Motif>& motif:ways_motifs){
          std::string hash = motif->getHash().p;
          if(verbose>2) cout <<"  -- checking hash '" <<hash <<"'" <<endl;
          Job* motif_job = motifResults[hash];
          if(!motif_job){
            if(verbose>2) cout <<"  -- novel! adding new motif job" <<endl;
            auto job = make_shared<Job>(0., a, motif.get(), _solve_motif);
            motif_jobs.append(job);
            jobs.append(job.get());
            motifResults[hash] = job.get();
            if(verbose>2) cout <<"+++ addin job" <<*job <<endl;
            motif_job = job.get();
          }else{
            if(verbose>2) cout <<"  -- found!" <<endl;
          }
          a->ways_job->dependencies.prepend(motif_job);
        }
#endif
      }
    }
  }
  else if(job->tag == _solve_ways){

    ActionNode *sub = job->a;

    if(verbose>0) cout <<"+++ solving sub-plan " <<sub->getPlanString() <<endl;

    PTR<KOMO>& ways = sub->get_ways(C, trans, tamp.explicitCollisions());

#if 0
    //-- loop through all subNLPs first
    Array<PTR<SubKOMO>>& subs = sub->getSubKOMOs();
    for(PTR<SubKOMO>& sub:subs){
      String hash = sub->getHash();
      SubProblemResults& res = subResults[hash];
      if(res.novel){
        if(verbose>0) cout <<"++ analyzing subKOMOs first.." <<endl;
        motifs.append(sub);

        auto job = make_shared<Job>(0., 0, sub.get(), _solve_motif);
        motif_jobs.append(job);
        jobs.append(job.get());
        return;
      }else{
        CHECK(res.ret, "");
        if(res.ret->feasible){
          sub->initialize(*ways, res.ret->x);
        }else{

          return;
        }
      }
    }
#endif

    //  ways->view(true, STRING("ways init" <<sub->action));
    ways->opt.verbose=0;
#if 1
    auto ret = ways->optimize();
#else
    std::shared_ptr<SolverReturn> ret;
    str opt_or_sample="gauss";
    if(opt_or_sample=="opt"){
      NLP_Solver sol;
      sol.setProblem(ways->nlp());
      ret = sol.solve(0, verbose);
    }else{
      NLP_Sampler sol(ways->nlp());
      sol.opt.seedMethod=opt_or_sample;
      sol.opt.verbose=verbose;
      sol.opt.downhillMaxSteps=250;
      sol.opt.slackMaxStep=.2;
      ret = sol.sample();
    }
#endif

    job->rets.append(ret);
    if(ret->feasible){
      job->n_succ++;
      job->priority = -100.;
    }
    if(!ret->feasible){
      job->n_fail++;
      job->priority -= 1.;
    }

    if(verbose>0) cout <<"++ jobs stats: " <<*job <<endl;

    if(sub->isTerminal && ret->feasible){
      solutions.append(sub);  //we have a feasible plan
      open_terminal_nodes.removeValue(job->a);
    }

    //pause debug
//    ways->view(false, STRING("ways solution " <<sub->action <<"\n" <<*ret));
//    ways->view_close();

    if(verbose>1) display(ways, ret, false, job->niceMsg());

  }else{

    KOMO_Motif* motif=job->motif;
    ActionNode *motif_origin=job->a;

    if(verbose>0) cout <<"+++ solving motif " <<job->niceMsg() <<endl;

    PTR<KOMO>& ways_komo = motif_origin->get_ways(C, trans, tamp.explicitCollisions());
    auto ret = motif->solve(*ways_komo, "gauss", verbose-2);

    job->rets.append(ret);
    if(ret->feasible){
      job->n_succ++;
      job->priority = -100.;
    }
    if(!ret->feasible){
      job->n_fail++;
      job->priority -= 1.;
    }

    if(verbose>0) cout <<"++ jobs stats: " <<*job <<endl;
    if(verbose>1) display(ways_komo, ret, false, job->niceMsg());
  }
}

void LGP_Tool::solve(int _verbose){
  if(_verbose>=0) verbose=_verbose;
  uint n = solutions.N;
  for(;;){
    if(verbose>0) cout <<"--------------------------------------------------------------------" <<endl;
    solve_step();
    if(solutions.N>n){
      if(verbose>0) cout <<"--------------------------------------------------------------------" <<endl;
      if(verbose>0) cout <<"   SOLUTION FOUND:   " <<solutions(-1)->getPlanString() <<endl;
      break;
    }
//    wait();
  }
}

StringAA LGP_Tool::getSolvedPlan(){
  if(!solutions.N) return {};
  return solutions(-1)->getPlan();
}

std::shared_ptr<KOMO> LGP_Tool::getSolvedKOMO(){
  if(!solutions.N) return PTR<KOMO>();
  return solutions(-1)->ways;
}

int LGP_Tool::view_solved(bool pause){
  if(!solutions.N) return 0;
  ActionNode *s = solutions(-1);
  return display(s->ways, s->ways_job->rets(-1), pause, s->ways_job->niceMsg());
}

void LGP_Tool::view_close(){
  if(gl_komo){ gl_komo->view_close();  gl_komo.reset(); }
  gl.reset();
}

arrA LGP_Tool::solvePiecewiseMotions(int _verbose){
  StringAA plan = getSolvedPlan();
  arrA paths;
  for(uint i=0;i<plan.N;i++){
//    cout <<"--- plan " <<i <<' ' <<plan(i) <<endl;
    PTR<KOMO> path = get_piecewiseMotionProblem(i, true);
    NLP_Solver sol;
    sol.setProblem(path->nlp());
    auto ret = sol.solve(0, 0);
//      path->view(true, "solved");
    if(_verbose>0) display(path, ret, _verbose>1);
    paths.append(path->getPath_qOrg());
  }
  return paths;
}

PTR<KOMO> LGP_Tool::solveFullMotion(int _verbose){
  PTR<KOMO> path = get_fullMotionProblem(true);
  //    path->view(true, "init");
  NLP_Solver sol;
  sol.setProblem(path->nlp());
  auto ret = sol.solve(0, 0);
  //    auto ret = path->optimize();
  //    path->view(true, "solved");
  if(_verbose>0) display(path, ret, _verbose>1);
  return path;
}

std::shared_ptr<KOMO> LGP_Tool::get_piecewiseMotionProblem(uint phase, bool fixEnd){ //should only retun a KOMO, I think
  StringAA plan = getSolvedPlan();
  StringA action = plan(phase);
  StringA prev_action = (phase>0?plan(phase-1):StringA());

  ManipulationModelling manip(getSolvedKOMO());
  auto komo = manip.sub_motion(phase, fixEnd)->komo;
  if(!fixEnd) trans.add_action_constraints(komo, 1., action);
  trans.add_action_constraints_motion(komo, 1., prev_action, action, phase);
  return komo;
}

std::shared_ptr<KOMO> LGP_Tool::get_fullMotionProblem(bool initWithWaypoints){
  StringAA path =  getSolvedPlan();

  ManipulationModelling manip;
  manip.setup_motion(C, path.N, 16, -1.);

  for(uint t=0;t<path.N;t++){
    trans.add_action_constraints(manip.komo, double(t)+1., path(t));
  }

  StringA explicitCollisions = tamp.explicitCollisions();
  for(uint i=0; i<explicitCollisions.N; i+=2) {
    manip.komo->addObjective({}, FS_distance, {explicitCollisions.elem(i), explicitCollisions.elem(i+1)}, OT_ineq, {1e1});
  }

  if(initWithWaypoints){
    auto ways = getSolvedKOMO();
    arr stable_q = ways->getConfiguration_qAll(-1);
    manip.komo->setConfiguration_qAll(-2, stable_q);
    arrA waypointsAll = ways->getPath_qAll();
    manip.komo->initWithWaypoints(waypointsAll, 1, true, 0.1);
  }

  for(uint t=0;t<path.N;t++){
    trans.add_action_constraints_motion(manip.komo, double(t)+1., (t>0?path(t-1):StringA()), path(t), t);
  }

//  for(uint k=0;k<waypointsAll.d0;k++){
//    manip.komo->addObjective({double(k)}, FS_qItself, {}, OT_sos, {1e1}, waypointsAll(k));
//  }

  return manip.komo;
}

int LGP_Tool::display(std::shared_ptr<KOMO>& komo, std::shared_ptr<SolverReturn>& ret, bool pause, const char* msg, bool play){
  if(!gl){
    gl = make_shared<OpenGL>("ALGO", 600, 500);
    gl->camera.setDefault();
  }
  if(gl_komo){ gl_komo->view_close();  gl_komo.reset(); }
  gl_komo = komo;
  gl_komo->pathConfig.get_viewer()->gl = gl;
  gl->add(gl_komo->pathConfig.get_viewer().get());

  str text;
  if(ret->feasible) text <<"SOLVED\n";
  else text <<"FAILED\n";
  text <<msg <<"\nsolver: " <<*ret <<"\n[use SHIFT+scroll or arror keys to browse; press key to continue]";
  int key = 0;
  if(play) key = gl_komo->view_play(pause, text);
  else key = gl_komo->view(pause, text);
//  komo->view_close();
  return key;
}

//===========================================================================

MotifL analyzeMotifs(KOMO& komo, int verbose){
  MotifL subs;
  //  cout <<komo.report(true, true, false) <<endl;
  for(shared_ptr<Objective>& ob:komo.objectives) {
    if(ob->feat->frameIDs.N<=2 && ob->feat->order<=1){
      for(GroundedObjective* go:ob->groundings){
        int timeSlice = go->timeSlices.elem(-1);

        if(verbose>1) cout <<"\n** objective " <<*ob <<' ' <<timeSlice <<endl;

        bool wasAdded=false;
        for(auto& sub: subs){
          if(sub->matches(go, timeSlice)){
            if(verbose>1) cout <<" -- matches motif " <<sub->objs.elem(-1)->name() <<' ' <<sub->timeSlice <<endl;
            sub->objs.append(go);
            sub->F.setAppend(go->frames);
            wasAdded=true;
            break;
          }
        }
        if(!wasAdded){
          if(verbose>1) cout <<" -- creating new motif " <<endl;
          auto motif = make_shared<KOMO_Motif>();
          subs.append(motif);
          motif->timeSlice = timeSlice;
          motif->objs.append(go);
          motif->F = go->frames;
        }
      }
    }
  }

  return subs;
}

//===========================================================================

struct Default_KOMO_Translator : Logic2KOMO_Translator{

  ~Default_KOMO_Translator() {}

  virtual std::shared_ptr<KOMO> setup_sequence(Configuration& C, uint K){
    ManipulationModelling manip;
    manip.setup_sequence(C, K);
    return manip.komo;
  }

  virtual void add_action_constraints(std::shared_ptr<KOMO>& komo, double time, const StringA& action){
    if(!action.N) return;

    ManipulationModelling manip(komo);

    if(action(0)=="pick" || action(0)=="handover"){
      str& obj = action(1);
      str& gripper = action(3);
      str palm;
      if(gripper.endsWith("_gripper")){
        palm = gripper.getFirstN(gripper.N-8);
        palm <<"_palm";
      }

      str snapFrame; snapFrame <<"pickPose_" <<gripper <<'_' <<obj <<'_' <<time;
      manip.komo->addFrameDof(snapFrame, gripper, JT_free, true, obj); //a permanent free stable gripper->grasp joint; and a snap grasp->object
      manip.komo->addRigidSwitch(time, {snapFrame, obj});
      if(manip.komo->stepsPerPhase>2) manip.komo->addObjective({time}, FS_poseDiff, {snapFrame, obj}, OT_eq, {1e0}, NoArr, 0, -1, 0);

      manip.grasp_box(time, gripper, obj, palm, "y");
      manip.komo->addObjective({time}, FS_negDistance, {obj, gripper}, OT_ineq, {-1e1});

    }else if(action(0)=="place"){
      str& obj = action(1);
      str& gripper = action(2);
      str& target = action(3);
      str palm;
      if(gripper.endsWith("_gripper")){
        palm = gripper.getFirstN(gripper.N-8);
        palm <<"_palm";
      }

      if(time<manip.komo->T/manip.komo->stepsPerPhase){
        str snapFrame; snapFrame <<"placePose_" <<target <<'_' <<obj <<'_' <<time;
        manip.komo->addFrameDof(snapFrame, target, JT_free, true, obj); //a permanent free stable target->place joint; and a snap place->object
        manip.komo->addRigidSwitch(time, {snapFrame, obj});
        if(manip.komo->stepsPerPhase>2) manip.komo->addObjective({time}, FS_poseDiff, {snapFrame, obj}, OT_eq, {1e0}, NoArr, 0, -1, 0);
      }

      manip.place_box(time, obj, target, palm, "z");
      //gripper center at least inside object
      manip.komo->addObjective({time}, FS_negDistance, {obj, gripper}, OT_ineq, {-1e1});


    }else if(action(0)=="gripper_push"){
      str& obj = action(1);
      str& table = action(2);
      str& gripper = action(3);

      if(time<manip.komo->T/manip.komo->stepsPerPhase){
        str snapFrame; snapFrame <<"pushPose_" <<gripper <<'_' <<obj <<'_' <<time;
        manip.komo->addFrameDof(snapFrame, gripper, rai::JT_free, true, obj);
        manip.komo->addRigidSwitch(time, {snapFrame, obj});
        if(manip.komo->stepsPerPhase>2) manip.komo->addObjective({time}, FS_poseDiff, {snapFrame, obj}, OT_eq, {1e0}, NoArr, 0, -1, 0);
      }

      manip.straight_push({time,time+1}, obj, gripper, table);

    }else if(action(0)=="end_push"){
      //str& gripper = action(1);
      str& obj = action(2);
      //str& floor = action(3);
      str& target = action(4);

      if(time<manip.komo->T/manip.komo->stepsPerPhase){
        NIY;
      }

      manip.place_box(time, obj, target, 0, "z");

    }else{
      HALT("action constraint not implemented: " <<action);
    }
  }

  virtual void add_action_constraints_motion(std::shared_ptr<KOMO>& komo, double time, const StringA& prev_action, const StringA& action, uint actionPhase){
    if(!action.N) return;

    ManipulationModelling manip(komo);

    if(action(0)=="pick" || action(0)=="handover"){
      str& gripper = action(3);
      manip.retract({time-1., time-.8}, gripper);
      manip.approach({time-.2, time}, gripper);
    }
    else if(action(0)=="place"){
      str& gripper = action(2);

      manip.retract({time-1., time-.8}, gripper);
      manip.approach({time-.2, time}, gripper);
    }
    else if(action(0)=="end_push"){
      str& gripper = action(1);
      str& obj = action(2);

      //    komo->addObjective(time_interval, FS_positionRel, {gripper, helperStart}, OT_eq, 1e1*arr{{2, 3}, {1., 0., 0., 0., 0., 1.}});
      str helperEnd = STRING("_straight_pushEnd_" <<gripper <<"_" <<obj <<'_' <<actionPhase+1);

//      komo->addObjective({time-1., time}, FS_positionRel, {obj, helperEnd}, OT_eq, 1e1*arr{{2, 3}, {1., 0., 0., 0., 0., 1.}});
      komo->addObjective({time-1., time}, FS_position, {obj}, OT_eq, 1e1*arr{{1, 3}, {0., 0., 1.}}, {}, 1);
      komo->addObjective({time-1., time}, FS_vectorZ, {obj}, OT_eq, {1e1}, {0., 0., 1.});
//      komo->addObjective({time-1., time}, FS_quaternion, {obj}, OT_eq, {1e1}, {}, 1);
    }

  }
};

//===========================================================================

struct Default_TAMP_Provider : TAMP_Provider{
  LGP_SkeletonTool tool;

  Default_TAMP_Provider(rai::Configuration& C, const char* file) : tool(C, file) {}
  virtual Array<StringA> getNewPlan(){
    FOL_World_State* s = tool.step_folPlan();
  #if 0 //state sequence
    String planString;
    Array<Graph*> states;
    arr times;
    s->getStateSequence(states, times, planString);
    LOG(0) <<states <<times <<planString;
  #else //action sequence (as stringAA)
    str debug;
    NodeL decisions = s->getDecisionSequence(debug);
    Array<StringA> plan(decisions.N);
    for(uint i=0;i<plan.N;i++){
      plan(i).resize(decisions(i)->parents.N);
      for(uint j=0;j<plan(i).N;j++) plan(i)(j) = decisions(i)->parents(j)->key;
    }
    //  LOG(0) <<plan <<endl <<debug;
  #endif
    return plan;
  }
  virtual Configuration& getConfig(){ return tool.lgproot->C; }
  virtual StringA explicitCollisions(){ return tool.lgproot->explicitCollisions; }
};

//===========================================================================

std::shared_ptr<TAMP_Provider> default_TAMP_Provider(rai::Configuration& C, const char* lgp_configfile){ return make_shared<Default_TAMP_Provider>(C, lgp_configfile); }
std::shared_ptr<Logic2KOMO_Translator> default_Logic2KOMO_Translator(){ return make_shared<Default_KOMO_Translator>(); }

}//namespace

RUN_ON_INIT_BEGIN(actionNode)
rai::ActionNodeL::memMove = true;
rai::Array<rai::Job*>::memMove = true;
RUN_ON_INIT_END(actionNode)

