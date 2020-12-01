/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "CtrlSolver.h"
#include "CtrlSolvers.h"

#include "CtrlTargets.h"

CtrlSolver::CtrlSolver(rai::Configuration& _C, double _tau, uint k_order)
  : tau(_tau) {
  komo.setModel(_C, true);
  komo.setTiming(1., 1, _tau, k_order);
  komo.setupConfigurations2();
}

CtrlSolver::~CtrlSolver(){
}

void CtrlSolver::set(const CtrlSet& CS) {
  objectives = CS.objectives;
}

void CtrlSolver::addObjectives(const rai::Array<ptr<CtrlObjective>>& O) {
  objectives.append(O);
}

void CtrlSolver::delObjectives(const rai::Array<ptr<CtrlObjective>>& O) {
  for(auto& o:O) {
    objectives.removeValue(o);
  }
}

#if 0
std::shared_ptr<CtrlObjective> CtrlSolver::add_qControlObjective(uint order, double scale, const arr& target) {
  ptr<Objective> o = komo.add_qControlObjective({}, order, scale, target);
  return addObjective(o->feat, NoStringA, o->type);
}

ptr<CtrlObjective> CtrlSolver::addObjective(const ptr<Feature>& _feat, const StringA& frames, ObjectiveType _type) {
  std::shared_ptr<CtrlObjective> t = make_shared<CtrlObjective>();
  t->feat = _feat;
  t->type = _type;
  if(!!frames && frames.N){
    if(frames.N==1 && frames.scalar()=="ALL") t->feat->frameIDs = framesToIndices(komo.world.frames); //important! this means that, if no explicit selection of frames was made, all frames (of a time slice) are referred to
    else t->feat->frameIDs = komo.world.getFrameIDs(frames);
  }
  addObjectives({t});
  return t;
}

ptr<CtrlObjective> CtrlSolver::addObjective(const FeatureSymbol& feat, const StringA& frames, ObjectiveType type, const arr& scale, const arr& target, int order) {
  return addObjective(symbols2feature(feat, frames, komo.world, scale, target, order), NoStringA, type);
}
#endif

void CtrlSolver::update(rai::Configuration& C) {
  //-- update the KOMO configurations (push one step back, and update current configuration)
  //the joint state:
  arr qold = komo.getConfiguration_q(-1);
  arr q = C.getJointState();
  for(int t=-komo.k_order; t<0; t++) komo.setConfiguration(t, komo.getConfiguration_q(t+1));
  komo.setConfiguration(-1, q);
  komo.setConfiguration(0, q); // + (q-qold));
  //the frame state of roots only:
  uintA roots = framesToIndices(C.getRoots());
  arr X = C.getFrameState(roots);
  for(int t=-komo.k_order; t<0; t++){
    arr Xt = komo.pathConfig.getFrameState(roots+komo.timeSlices(komo.k_order+t+1,0)->ID);
    komo.pathConfig.setFrameState(Xt, roots+komo.timeSlices(komo.k_order+t,0)->ID);
  }
  komo.pathConfig.setFrameState(X, roots+komo.timeSlices(komo.k_order-1,0)->ID);
  komo.pathConfig.setFrameState(X, roots+komo.timeSlices(komo.k_order,0)->ID);

  komo.pathConfig.ensure_q();

  //-- step the moving targets forward, if they have one
  for(shared_ptr<CtrlObjective>& o: objectives) if(o->active){
    if(!o->name.N) o->name = o->feat->shortTag(C);

    if(o->movingTarget) {
      o->y_buffer = o->getValue(*this);
      ActStatus s_new = o->movingTarget->step(tau, o.get(), o->y_buffer);
      if(o->status != s_new) {
        o->status = s_new;
        //callbacks
      }
    } else {
      if(o->status != AS_running) {
        o->status = AS_running;
        //callbacks
      }
    }
  }
}

void CtrlSolver::report(std::ostream& os) {
  os <<"    control objectives:" <<endl;
  for(auto& o: objectives) o->reportState(os);
  os <<"    optimization result:" <<endl;
  os <<optReport <<endl;
}

static int animate=0;

arr CtrlSolver::solve() {
#if 0
  TaskControlMethods M(komo.getConfiguration_t(0).getHmetric());
  arr q = komo.getConfiguration_t(0).getJointState();
  q += M.inverseKinematics(objectives, NoArr, {});
  return q;
#elif 1
  komo.clearObjectives();
  for(auto& o: objectives) if(o->active){
    komo.addObjective({}, o->feat, {}, o->type);
  }
  OptOptions opt;
  opt.stopTolerance = 1e-4;
  opt.stopGTolerance = 1e-4;
  opt.stopIters = 20;
//  opt.nonStrictSteps=-1;
  opt.maxStep = .1; //*tau; //maxVel*tau;
  opt.damping = 1e-3;
//  komo.verbose=4;
  komo.animateOptimization=animate;
  komo.optimize(0., opt);
  optReport = komo.getReport(false);
  if(optReport.get<double>("sos")>.1){
      cout <<optReport <<endl <<"something's wrong?" <<endl;
      rai::wait();
//      animate=2;
  }
//  komo.checkGradients();
//  komo.pathConfig.watch(false, "komo");
  return komo.getPath_q(0);
#else
  return solve_optim(*this);
#endif
}
