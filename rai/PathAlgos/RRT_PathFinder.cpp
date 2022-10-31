#include "RRT_PathFinder.h"

#include "../Gui/opengl.h"
#include "../Kin/viewer.h"
#include "../GL/gl.h"

double corput(uint n, uint base){
  double q=0;
  double bk=(double)1/base;

  while (n > 0) {
    q += (n % base)*bk;
    n /= base;
    bk /= base;
  }
  return q;
}

bool checkConnection(ConfigurationProblem& P,
                     const arr &start,
                     const arr &end,
                     const uint disc,
                     const bool binary){
  if (binary){
    for (uint i=1; i<disc; ++i){
      double ind = corput(i, 2);
      arr p = start + ind * (end-start);

      // TODO: change to check feasibility properly (with path constraints)
      if(!P.query(p)->isFeasible){
        return false;
      }
    }
  }
  else{
    for (uint i=1; i<disc-1; ++i){
      arr p = start + 1.0 * i / (disc-1) * (end-start);

      // TODO: change to check feasibility properly (with path constraints)
      if(!P.query(p)->isFeasible){
        return false;
  }
    }
  }
  return true;
}

//===========================================================================

RRT_SingleTree::RRT_SingleTree(const arr& q0, const shared_ptr<QueryResult>& q0_qr){
  if(!q0_qr->isFeasible) LOG(0) <<"rooting RRT with infeasible start configuration -- that's likely to fail: query is:\n" <<*q0_qr;
  add(q0, 0, q0_qr);
}

uint RRT_SingleTree::add(const arr& q, uint parentID, const shared_ptr<QueryResult>& _qr){
  drawMutex.lock(RAI_HERE);
  ann.append(q);
  parent.append(parentID);
  queries.append(_qr);
  disp3d.append(_qr->disp3d);
  disp3d.reshape(-1,3);

  CHECK_EQ(parent.N, ann.X.d0, "");
  CHECK_EQ(queries.N, ann.X.d0, "");
  CHECK_EQ(disp3d.d0, ann.X.d0, "");
  drawMutex.unlock();
  return parent.N-1;
}

double RRT_SingleTree::getNearest(const arr& target){
  //find NN
  nearestID = ann.getNN(target);
  return length(target - ann.X[nearestID]);
}

arr RRT_SingleTree::getProposalTowards(const arr& target, double stepsize){
  //find NN
  nearestID = ann.getNN(target);

  //compute default step
  arr delta = target - ann.X[nearestID]; //difference vector between q and nearest neighbor
  double dist = length(delta);
  if(dist>stepsize)  delta *= stepsize/dist;

  return getNode(nearestID) + delta;
}

arr RRT_SingleTree::getNewSample(const arr& target, double stepsize, double p_sideStep, bool& isSideStep, const uint recursionDepth){
  //find NN
  nearestID = ann.getNN(target);
  std::shared_ptr<QueryResult> qr = queries(nearestID);

  //compute default step
  arr delta = target - getNode(nearestID);
  double dist = length(delta);
  if(dist>stepsize) delta *= stepsize/dist;

  //without side stepping, we're done
  isSideStep = false;
  if(p_sideStep<=0. || recursionDepth >= 3) return getNode(nearestID) + delta;

  //check whether this is a predicted collision
  bool predictedCollision=false;
  if(qr->coll_y.N){
    arr y = qr->coll_y + qr->coll_J * delta;
    if(min(y)<0.) predictedCollision = true;
  }

  if(predictedCollision && p_sideStep>0. && rnd.uni()<p_sideStep){
    isSideStep=true;

    //compute new target
    arr d = qr->getSideStep();
    d *= rnd.uni(stepsize,2.) / length(d);
    arr targ = getNode(nearestID) + d;
    bool tmp;
    return getNewSample(targ, stepsize, p_sideStep, tmp, recursionDepth + 1);
  }else{
    return getNode(nearestID) + delta;
  }

  HALT("shouldn't be here");
  return NoArr;
}

arr RRT_SingleTree::getPathFromNode(uint fromID){
  arr path;
  uint node = fromID;
  for(;;){
    path.append(ann.X[node]);
    if(!node) break;
    node = getParent(node);
  }
  path.reshape(-1, ann.X.d1);
  return path;
}

void RRT_SingleTree::glDraw(OpenGL& gl){
  glColor(.0, .0, .0);
  glLineWidth(2.f);
  glBegin(GL_LINES);
  drawMutex.lock(RAI_HERE);
  for(uint i=1;i<getNumberNodes();i++){
    glVertex3dv(&disp3d(parent(i),0));
    glVertex3dv(&disp3d(i,0));
  }
  drawMutex.unlock();
  glEnd();
  glLineWidth(1.f);
}

//===========================================================================

bool RRT_PathFinder::growTreeTowardsRandom(RRT_SingleTree& rrt){
  const arr start = rrt.ann.X[0];
  arr t(rrt.getNode(0).N);
  rndUniform(t,-RAI_2PI,RAI_2PI,false);
  HALT("DON'T USE 2PI")

  arr q = rrt.getProposalTowards(t, stepsize);

  auto qr = P.query(q);
  if(qr->isFeasible){
    if (intermediateCheck && !checkConnection(P, start, q, 20, true)){
      return false;
    }

    rrt.add(q, rrt.nearestID, qr);
    return true;
  }
  return false;
}

bool RRT_PathFinder::growTreeToTree(RRT_SingleTree& rrt_A, RRT_SingleTree& rrt_B){
  bool isSideStep, isForwardStep;
  //decide on a target: forward or random
  arr t;
  if(rnd.uni()<p_forwardStep){
    t = rrt_B.getRandomNode();
    isForwardStep = true;
  }else{
#if 1
    t.resize(rrt_A.getNode(0).N);
    for(uint i=0;i<t.N;i++){
      double lo=P.limits(i,0), up=P.limits(i,1);
      CHECK_GE(up-lo, 1e-3,"limits are null interval: " <<i <<' ' <<P.C.getJointNames());
      t.elem(i) = lo + rnd.uni()*(up-lo);
    }
#else
    t.resize(rrt_A.getNode(0).N);
    rndUniform(t,-RAI_2PI,RAI_2PI,false);
#endif
    isForwardStep = false;
  }

  //sample configuration towards target, possibly sideStepping
  arr q = rrt_A.getNewSample(t, stepsize, p_sideStep, isSideStep, 0);

  //evaluate the sample
  auto qr = P.query(q);
  if(isForwardStep){  n_forwardStep++; if(qr->isFeasible) n_forwardStepGood++; }
  if(!isForwardStep){  n_rndStep++; if(qr->isFeasible) n_rndStepGood++; }
  if(isSideStep){  n_sideStep++; if(qr->isFeasible) n_sideStepGood++; }

  //if infeasible, make a backward step from the sample configuration
  if(!qr->isFeasible && p_backwardStep>0. && rnd.uni()<p_backwardStep){
    t = q + qr->getBackwardStep();
    q = rrt_A.getNewSample(t, stepsize, p_sideStep, isSideStep, 0);
    qr = P.query(q);
    n_backStep++; if(qr->isFeasible) n_backStepGood++;
    if(isSideStep){  n_sideStep++; if(qr->isFeasible) n_sideStepGood++; }
  };

  // TODO: add checking motion
  if(qr->isFeasible){
    const arr start = rrt_A.ann.X[rrt_A.nearestID];
    if (intermediateCheck && !checkConnection(P, start, q, 20, true)){
      return false;
    }

    rrt_A.add(q, rrt_A.nearestID, qr);
    double dist = rrt_B.getNearest(q);
    if(dist<stepsize) return true;
  }
  return false;
}

//===========================================================================

RRT_PathFinder::RRT_PathFinder(ConfigurationProblem& _P, const arr& _starts, const arr& _goals, double _stepsize, uint _verbose, bool _intermediateCheck)
  : P(_P),
    stepsize(_stepsize),
    verbose(_verbose),
    intermediateCheck(_intermediateCheck) {
  arr q0 = _starts;
  arr qT = _goals;
  auto q0ret = P.query(q0);
  auto qTret = P.query(qT);
  if(!q0ret->isFeasible){ if(verbose>0) LOG(0) <<"initializing with infeasible q0"; if(verbose>1) q0ret->writeDetails(std::cout, P); }
  if(!qTret->isFeasible){ if(verbose>0) LOG(0) <<"initializing with infeasible qT"; if(verbose>1) qTret->writeDetails(std::cout, P); }
  rrt0 = make_shared<RRT_SingleTree>(q0, q0ret);
  rrtT = make_shared<RRT_SingleTree>(qT, qTret);

  if(verbose>2){
    DISP.clear();
    DISP.copy(P.C);
    DISP.gl()->add(*rrt0);
    DISP.gl()->add(*rrtT);
  }
}

void RRT_PathFinder::planForward(const arr& q0, const arr& qT){
  bool success=false;

  for(uint i=0;i<100000;i++){
    iters++;
    //let rrt0 grow
    bool added = growTreeTowardsRandom(*rrt0);
    if(added){
      if(length(rrt0->getLast() - qT)<stepsize) success = true;
    }
    if(success) break;

    //some output
    if (verbose > 2){
      if(!(i%100)){
        DISP.setJointState(rrt0->getLast());
        DISP.watch(false);
        std::cout <<"RRT samples=" <<i <<" tree size = " <<rrt0->getNumberNodes() <<std::endl;
      }
    }
  }

  if(!success) return;

  if (verbose > 0){
    std::cout <<"SUCCESS!"
              <<"\n  tested samples=" <<P.evals
              <<"\n  #tree-size=" <<rrt0->getNumberNodes()
     << std::endl;
  }

  arr path = rrt0->getPathFromNode(rrt0->nearestID);
  revertPath(path);

  //display
  if(verbose > 1){
    std::cout << "path-length= " << path.d0 <<std::endl;
    DISP.proxies.clear();

    for(uint t=0;t<path.d0;t++){
      DISP.setJointState(path[t]);
      //DISP.watch();
      DISP.watch(false);
      rai::wait(.1);
    }
  }

  path >>FILE("z.path");
}

int RRT_PathFinder::stepConnect(){
  iters++;
  if(iters>maxIters) return -1;

  bool success = growTreeToTree(*rrt0, *rrtT);
  if(!success) success = growTreeToTree(*rrtT, *rrt0);

  //animation display
  if(verbose>2){
    if(!(iters%100)){
      DISP.setJointState(rrt0->getLast());
      DISP.watch(verbose>4, STRING("planConnect evals " <<P.evals));
      std::cout <<"RRT queries=" <<P.evals <<" tree sizes = " <<rrt0->getNumberNodes()  <<' ' <<rrtT->getNumberNodes() <<std::endl;
    }
  }

  //-- the rest is only on success -> extract the path, display, etc

  if(success){
    if(verbose>0){
      std::cout <<"\nSUCCESS!" <<std::endl;
      std::cout <<"  RRT queries=" <<P.evals <<" tree sizes = " <<rrt0->getNumberNodes()  <<' ' <<rrtT->getNumberNodes() <<std::endl;
      std::cout <<"  forwardSteps: " <<(100.*n_forwardStepGood/n_forwardStep) <<"%/" <<n_forwardStep;
      std::cout <<"  backSteps: " <<(100.*n_backStepGood/n_backStep) <<"%/" <<n_backStep;
      std::cout <<"  rndSteps: " <<(100.*n_rndStepGood/n_rndStep) <<"%/" <<n_rndStep;
      std::cout <<"  sideSteps: " <<(100.*n_sideStepGood/n_sideStep) <<"%/" <<n_sideStep;
      std::cout <<std::endl;
    }

    path = rrt0->getPathFromNode(rrt0->nearestID);
    arr pathT = rrtT->getPathFromNode(rrtT->nearestID);

    revertPath(path);
    path.append(pathT);

    //display
    if(verbose>1){
      std::cout <<"  path-length=" <<path.d0 <<std::endl;
      if(verbose>2){
        DISP.proxies.clear();
        for(uint t=0;t<path.d0;t++){
          DISP.setJointState(path[t]);
          DISP.watch(false, STRING("rrt result "<<t));
          rai::wait(.1);
        }
        DISP.watch(true);
        DISP.clear();
      }
    }

    return 1;
  }

  return 0;
}

arr RRT_PathFinder::planConnect(){
  int r=0;
  while(!r){ r = stepConnect(); }
  if(r==-1) return NoArr;
  return path;
}

void revertPath(arr& path){
  uint N = path.d0;
  arr x;
  for(uint i=0;i<N/2;i++){
    x = path[i];
    path[i] = path[N-1-i];
    path[N-1-i] = x;
  }
}

shared_ptr<PathResult> RRT_PathFinder::run(double timeBudget){
  planConnect();
  return make_shared<PathResult>(path);
}

