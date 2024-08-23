/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "CtrlSolver.h"
#include "CtrlSolvers.h"

#include "CtrlTargets.h"

CtrlSolver::CtrlSolver(const rai::Configuration& _C, double _tau, uint k_order)
  : tau(_tau) {
  komo.setConfig(_C, true);
  komo.setTiming(1., 1, _tau, k_order);
}

CtrlSolver::~CtrlSolver() {
}

void CtrlSolver::set(const CtrlSet& CS) {
  objectives = CS.objectives;
}

void CtrlSolver::addObjectives(const rai::Array<shared_ptr<CtrlObjective>>& O) {
  objectives.append(O);
}

void CtrlSolver::delObjectives(const rai::Array<shared_ptr<CtrlObjective>>& O) {
  for(auto& o:O) {
    objectives.removeValue(o);
  }
}

#if 0
std::shared_ptr<CtrlObjective> CtrlSolver::addControlObjective(uint order, double scale, const arr& target) {
  shared_ptr<Objective> o = komo.addControlObjective({}, order, scale, target);
  return addObjective(o->feat, NoStringA, o->type);
}

shared_ptr<CtrlObjective> CtrlSolver::addObjective(const shared_ptr<Feature>& _feat, const StringA& frames, ObjectiveType _type) {
  std::shared_ptr<CtrlObjective> t = make_shared<CtrlObjective>();
  t->feat = _feat;
  t->type = _type;
  if(!!frames && frames.N) {
    if(frames.N==1 && frames.scalar()=="ALL") t->feat->frameIDs = framesToIndices(komo.world.frames); //important! this means that, if no explicit selection of frames was made, all frames (of a time slice) are referred to
    else t->feat->frameIDs = komo.world.getFrameIDs(frames);
  }
  addObjectives({t});
  return t;
}

shared_ptr<CtrlObjective> CtrlSolver::addObjective(const FeatureSymbol& feat, const StringA& frames, ObjectiveType type, const arr& scale, const arr& target, int order) {
  return addObjective(symbols2feature(feat, frames, komo.world, scale, target, order), NoStringA, type);
}
#endif

void CtrlSolver::update(const arr& q_real, const arr& qDot_real, rai::Configuration& C) {
  //-- pushback frame states of roots (e.g. moving objects)
  uintA roots = framesToIndices(C.getRoots());
  for(int t=-komo.k_order; t<0; t++) {
    arr Xt = komo.pathConfig.getFrameState(roots+komo.timeSlices(komo.k_order+t+1, 0)->ID);
    komo.pathConfig.setFrameState(Xt, roots+komo.timeSlices(komo.k_order+t, 0)->ID);
  }
  //-- if C given, adopt frame states of roots (e.g. moving objects)
  if(!!C) {
    arr X = C.getFrameState(roots);
    komo.pathConfig.setFrameState(X, roots+komo.timeSlices(komo.k_order-1, 0)->ID);
    komo.pathConfig.setFrameState(X, roots+komo.timeSlices(komo.k_order, 0)->ID);
  }
  //-- use the q_real and qDot_real to define the prefix
  //the joint state:
  if(komo.k_order==2) {
    if(qDot_real.N) komo.setConfiguration_qOrg(-2, q_real - tau*qDot_real);
    else komo.setConfiguration_qOrg(-2, komo.getConfiguration_qOrg(-1));
    komo.setConfiguration_qOrg(-1, q_real);
    komo.setConfiguration_qOrg(0, q_real);
  } else if(komo.k_order==1) {
    komo.setConfiguration_qOrg(-1, q_real);
    komo.setConfiguration_qOrg(0, q_real);
  } else NIY;

  komo.pathConfig.ensure_q();

  //-- step the moving targets forward, if they have one
  for(shared_ptr<CtrlObjective>& o: objectives) if(o->active) {
      if(!o->name.N) o->name = o->feat->shortTag(C);

      if(o->movingTarget) {
        o->y_buffer = o->getValue(*this);
        ActStatus s_new = o->movingTarget->step(tau, o.get(), o->y_buffer.noJ());
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
  for(auto& o: objectives) if(o->active) {
      komo.addObjective({}, o->feat, {}, o->type);
    }
  rai::OptOptions opt;
  opt.stopTolerance = 1e-4;
  opt.stopGTolerance = 1e-4;
  opt.stopInners = 20;
//  opt.nonStrictSteps=-1;
  opt.maxStep = .1; //*tau; //maxVel*tau;
  opt.damping = 1e-2;
  komo.opt.verbose=0;
  komo.opt.animateOptimization=animate;
  komo.optimize(0., -1, opt);
  optReport = komo.report().get<rai::Graph>("totals");
  if(optReport.get<double>("sos")>1.1
      || optReport.get<double>("eq")>.1
      || optReport.get<double>("ineq")>.01) {
    cout <<optReport <<endl <<"something's wrong?" <<endl;
    //UNDO OPTIMIZATION:
//      komo.setConfiguration_qOrg(0, komo.getConfiguration_qOrg(-1));
    rai::wait();
//      animate=2;
  }
//  komo.checkGradients();
//  komo.pathConfig.view(false, "komo");
  return komo.getConfiguration_qOrg(0);
#else
  return solve_optim(*this);
#endif
}
