/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "RRT_PathFinder.h"

#include "../Gui/opengl.h"
#include "../Kin/viewer.h"
#include "../KOMO/pathTools.h"

#ifdef RAI_GL
#  include <GL/glew.h>
#  include <GL/gl.h>
#endif

namespace rai {

double corput(uint n, uint base) {
  double q = 0.;
  double bk = 1./double(base);

  while(n > 0) {
    q += (n % base)*bk;
    n /= base;
    bk /= double(base);
  }
  return q;
}

bool checkConnection(ConfigurationProblem& P,
                     const arr& start,
                     const arr& end,
                     const uint num,
                     const bool binary) {
  if(binary) {
    for(uint i=1; i<num; ++i) {
      double ind = corput(i, 2);
      arr p = start + ind * (end-start);

      // TODO: change to check feasibility properly (with path constraints)
      if(!P.query(p)->isFeasible) {
        return false;
      }
    }
  } else {
    for(uint i=1; i<num-1; ++i) {
      arr p = start + 1.0 * i / (num-1) * (end-start);

      // TODO: change to check feasibility properly (with path constraints)
      if(!P.query(p)->isFeasible) {
        return false;
      }
    }
  }
  return true;
}

//===========================================================================

RRT_SingleTree::RRT_SingleTree(const arr& q0, const shared_ptr<QueryResult>& q0_qr) {
//  if(!q0_qr->isFeasible) LOG(0) <<"rooting RRT with infeasible start configuration -- that's likely to fail: query is:\n" <<*q0_qr;
  add(q0, 0, q0_qr);
}

uint RRT_SingleTree::add(const arr& q, uint parentID, const shared_ptr<QueryResult>& _qr) {
  drawMutex.lock(RAI_HERE);
  ann.append(q);
  parent.append(parentID);
  queries.append(_qr);
  disp3d.append(_qr->disp3d);
  disp3d.reshape(-1, 3);

  CHECK_EQ(parent.N, ann.X.d0, "");
  CHECK_EQ(queries.N, ann.X.d0, "");
  //CHECK_EQ(disp3d.d0, ann.X.d0, "");
  drawMutex.unlock();
  return parent.N-1;
}

double RRT_SingleTree::getNearest(const arr& target) {
  //find NN
  nearestID = ann.getNN(target);
  return length(target - ann.X[nearestID]);
}

arr RRT_SingleTree::getProposalTowards(const arr& target, double stepsize) {
  //find NN
  nearestID = ann.getNN(target);

  //compute default step
  arr delta = target - ann.X[nearestID]; //difference vector between q and nearest neighbor
  double dist = length(delta);
  if(dist>stepsize)  delta *= stepsize/dist;

  return getNode(nearestID) + delta;
}

arr RRT_SingleTree::getNewSample(const arr& target, double stepsize) {
  //find NN
  nearestID = ann.getNN(target);
  std::shared_ptr<QueryResult> qr = queries(nearestID);

  //compute default step
  arr delta = target - getNode(nearestID);
  double dist = length(delta);
  if(dist>stepsize) delta *= stepsize/dist;

  return getNode(nearestID) + delta;
}

arr RRT_SingleTree::getPathFromNode(uint fromID) {
  arr path;
  uint node = fromID;
  for(;;) {
    path.append(ann.X[node]);
    if(!node) break;
    node = getParent(node);
  }
  path.reshape(-1, ann.X.d1);
  return path;
}

//===========================================================================

bool RRT_PathFinder::growTreeTowardsRandom(RRT_SingleTree& rrt) {
  const arr start = rrt.ann.X[0];
  arr t(rrt.getNode(0).N);
  rndUniform(t, -RAI_2PI, RAI_2PI, false);
  HALT("DON'T USE 2PI")

  arr q = rrt.getProposalTowards(t, opt.stepsize);

  auto qr = P->query(q);
  if(qr->isFeasible) {
    if(opt.subsamples>0 && !checkConnection(*P, start, q, opt.subsamples, true)) {
      return false;
    }

    rrt.add(q, rrt.nearestID, qr);
    return true;
  }
  return false;
}

void normalizeSphericalCoordinates(arr& x, const uintA& idx){
  arr xsub = x({idx(0), idx(0)+idx(1)});
  op_normalize(xsub);
}

void randomSphericalCoordinates(arr& x, const uintA& idx){
  arr xsub = x({idx(0), idx(0)+idx(1)});
  xsub = randn(xsub.N);
  op_normalize(xsub);
}

void flipSphericalCoordinates(arr& x, const uintA& idx){
  arr xsub = x({idx(0), idx(0)+idx(1)});
  xsub *= -1.;
}

bool RRT_PathFinder::growTreeToTree(RRT_SingleTree& rrt_A, RRT_SingleTree& rrt_B) {
  //decide on a target: forward or random
  arr t;
  if(rnd.uni()<opt.p_connect) {
    t = rrt_B.getRandomNode();
  } else {
    t.resize(rrt_A.getNode(0).N);
    for(uint i=0; i<t.N; i++) {
      double lo=P->limits(0, i), up=P->limits(1, i);
      CHECK_GE(up-lo, 1e-3, "limits are null interval: " <<i <<' ' <<P->C->getJointNames());
      t.elem(i) = lo + rnd.uni()*(up-lo);
    }
    for(uint i=0;i<P->sphericalCoordinates.d0;i++) randomSphericalCoordinates(t, P->sphericalCoordinates[i]);
  }

  //sample configuration towards target, possibly sideStepping
  arr q = rrt_A.getNewSample(t, opt.stepsize);
  for(uint i=0;i<P->sphericalCoordinates.d0;i++) normalizeSphericalCoordinates(t, P->sphericalCoordinates[i]);
  uint parentID = rrt_A.nearestID;

  //special rule: if parent is already in collision, isFeasible = smaller collisions
  shared_ptr<QueryResult>& pr = rrt_A.queries(parentID);
  double org_collisionTolerance = P->collisionTolerance;
  if(pr->totalCollision>P->collisionTolerance) P->collisionTolerance = pr->totalCollision + 1e-6;

  //evaluate the sample
  auto qr = P->query(q);

  //checking subsamples
  if(qr->isFeasible && opt.subsamples>0) {
    const arr start = rrt_A.ann.X[parentID];
    qr->isFeasible = checkConnection(*P, start, q, opt.subsamples, true);
  }

  P->collisionTolerance = org_collisionTolerance;

  //finally adding the new node to the tree
  if(qr->isFeasible){
    rrt_A.add(q, parentID, qr);
    if(P->sphericalCoordinates.N){
      CHECK_LE(P->sphericalCoordinates.d0, 1, "");
      arr q_org = q;
      for(uint i=0;i<P->sphericalCoordinates.d0;i++) {
        flipSphericalCoordinates(q, P->sphericalCoordinates[i]);
        rrt_A.add(q, parentID, qr);
      }
      q = q_org;
    }
    double dist = rrt_B.getNearest(q);
    if(opt.subsamples>0) { if(dist<opt.stepsize/opt.subsamples) return true; }
    else { if(dist<opt.stepsize) return true; }
  }

  return false;
}

//===========================================================================


void RRT_PathFinder::setProblem(shared_ptr<Configuration> C){
  P = make_shared<ConfigurationProblem>(C, opt.useBroadCollisions, opt.collisionTolerance, 1);
  P->verbose=0;
}

void RRT_PathFinder::setStartGoal(const arr& _starts, const arr& _goals){
  arr q0 = _starts;
  arr qT = _goals;
  auto q0ret = P->query(q0);
  auto qTret = P->query(qT);
  if(!q0ret->isFeasible) { LOG(0) <<"initializing with infeasible q0:"; q0ret->writeDetails(std::cout, *P); }
  if(!qTret->isFeasible) { LOG(0) <<"initializing with infeasible qT:"; qTret->writeDetails(std::cout, *P); }
  rrt0 = make_shared<RRT_SingleTree>(q0, q0ret);
  rrtT = make_shared<RRT_SingleTree>(qT, qTret);
}

void RRT_PathFinder::planForward(const arr& q0, const arr& qT) {
  bool success=false;

  for(uint i=0; i<100000; i++) {
    iters++;
    //let rrt0 grow
    bool added = growTreeTowardsRandom(*rrt0);
    if(added) {
      if(length(rrt0->getLast() - qT)<opt.stepsize) success = true;
    }
    if(success) break;

    //some output
    if(opt.verbose>2) {
      if(!(i%100)) {
        DISP.setJointState(rrt0->getLast());
        DISP.view(false);
        std::cout <<"RRT samples=" <<i <<" tree size = " <<rrt0->getNumberNodes() <<std::endl;
      }
    }
  }

  if(!success) return;

  if(opt.verbose>0) {
    std::cout <<"SUCCESS!"
              <<"\n  tested samples=" <<P->evals
              <<"\n  #tree-size=" <<rrt0->getNumberNodes()
              << std::endl;
  }

  arr path = rrt0->getPathFromNode(rrt0->nearestID);
  revertPath(path);

  //display
  if(opt.verbose>1) {
    std::cout << "path-length= " << path.d0 <<std::endl;
    ensure_DISP();
    DISP.proxies.clear();

    for(uint t=0; t<path.d0; t++) {
      DISP.setJointState(path[t]);
      //DISP.view();
      DISP.view(false);
      rai::wait(.1);
    }
  }

  path >>FILE("z.path");
}

void RRT_PathFinder::report(){
  std::cout <<"RRT - queries: " <<P->evals <<" tree sizes: " <<rrt0->getNumberNodes()  <<' ' <<rrtT->getNumberNodes() <<" path length: " <<path.d0 <<std::endl;
}

int RRT_PathFinder::stepConnect() {
  iters++;
  if(iters>(uint)opt.maxIters) return -1;

  bool success = growTreeToTree(*rrt0, *rrtT);
  if(!success) success = growTreeToTree(*rrtT, *rrt0);

  //animation display
  if(opt.verbose>2) {
    if(!(iters%100)) {
      ensure_DISP();
      DISP.setJointState(rrt0->getLast());
      DISP.view(opt.verbose>4, STRING("planConnect evals " <<P->evals));
    }
  }
  if(opt.verbose>1) {
    if(!(iters%100)) {
      report();
    }
  }

  //-- the rest is only on success -> extract the path, display, etc

  if(success) {
    if(opt.verbose>0) {
      std::cout <<"  -- rrt success:";
      report();
//      std::cout <<"  forwardSteps: " <<(100.*n_forwardStepGood/n_forwardStep) <<"%/" <<n_forwardStep;
//      std::cout <<"  backSteps: " <<(100.*n_backStepGood/n_backStep) <<"%/" <<n_backStep;
//      std::cout <<"  rndSteps: " <<(100.*n_rndStepGood/n_rndStep) <<"%/" <<n_rndStep;
//      std::cout <<"  sideSteps: " <<(100.*n_sideStepGood/n_sideStep) <<"%/" <<n_sideStep;
//      std::cout <<std::endl;
    }

    path = rrt0->getPathFromNode(rrt0->nearestID);
    arr pathT = rrtT->getPathFromNode(rrtT->nearestID);

    revertPath(path);
    path.append(pathT);

    //display
    if(opt.verbose>1) {
      std::cout <<"  path-length=" <<path.d0 <<std::endl;
      if(opt.verbose>2) {
        ensure_DISP();
        DISP.proxies.clear();
        for(uint t=0; t<path.d0; t++) {
          DISP.setJointState(path[t]);
          DISP.view(false, STRING("rrt result "<<t));
          rai::wait(.1);
        }
        DISP.view(opt.verbose>3);
        DISP.clear();
      }
    }

    return 1;
  }

  return 0;
}

shared_ptr<SolverReturn> RRT_PathFinder::solve() {
  if(!ret) ret = make_shared<SolverReturn>();
  P->useBroadCollisions = opt.useBroadCollisions;

  ret->time -= rai::cpuTime();
  int r=0;
  while(!r) { r = stepConnect(); }
  if(r==-1) path.clear();
  ret->time += rai::cpuTime();

  ret->done = true; //(r!=0);
  ret->feasible = path.N; //(r==1);
  ret->x = path;
  ret->evals = iters;

  return ret;
}

void revertPath(arr& path) {
  uint N = path.d0;
  arr x;
  for(uint i=0; i<N/2; i++) {
    x = path[i];
    path[i] = path[N-1-i];
    path[N-1-i] = x;
  }
}

void RRT_PathFinder::view(bool pause, const char* txt, bool play){
#if 0
  ensure_DISP();
  DISP.get_viewer() -> updateConfiguration(DISP);
  if(path.N) DISP.get_viewer() -> setMotion(DISP, path);
  // if(play) DISP.get_viewer() -> playVideo();
  DISP.get_viewer() -> view(pause, txt);
#else
  P->C->get_viewer() -> updateConfiguration(*P->C);
  if(path.N) P->C->get_viewer() -> setMotion(*P->C, path);
  P->C->get_viewer() -> view(pause, txt);
#endif
}

void RRT_PathFinder::ensure_DISP(){
  if(DISP.getJointStateDimension() != P->C->getJointStateDimension()){
    DISP.copy(*P->C);
  }
}

void RRT_PathFinder::setExplicitCollisionPairs(const StringA& collisionPairs) {
  CHECK(P, "need to set problem first");
  P->setExplicitCollisionPairs(collisionPairs);
}

arr RRT_PathFinder::get_resampledPath(uint T){ return path_resampleLinear(path, T); }

} //namespace
