#include "shapenetGrasps.h"

#include "../Kin/frame.h"
#include "../KOMO/komo.h"
#include "../Optim/NLP_Solver.h"
#include "../Kin/kin_physx.h"
#include "../Core/h5.h"
#include "../Kin/viewer.h"

ShapenetGrasps::ShapenetGrasps(){
  files = rai::fromFile<StringA>(opt.filesPrefix+"files");
  files.reshape(-1);
  if(opt.verbose>0){
    LOG(0) <<"files: " <<files(0) <<" ... " <<files(-1);
  }
}

void ShapenetGrasps::clearScene(){
  C.clear();

  rai::Frame *ref = C.addFrame("ref");
  ref->setShape(rai::ST_marker, {.1});
  ref->setColor({1.,1.,0.});
}

void ShapenetGrasps::resetObjectPose(int idx, bool rndOrientation){
  rai::Frame *obj = C.getFrame(STRING("obj"<<idx));
  if(rndOrientation) obj->set_X()->rot.setRandom();
  obj->setPosition({double(idx),0.,1.});
}

bool ShapenetGrasps::addSceneObject(const char* file, int idx, bool rndOri){
  LOG(0) <<"loading shapenet object " <<file;
  rai::Frame *obj = C.addH5Object(STRING("obj"<<idx), file, 2);
  if(!obj) return false;
  obj->inertia->scaleTo(.1);

  // obj->setShape(rai::ST_marker, {.5});
  resetObjectPose(idx, rndOri);
  cout <<"loaded object inertia: " <<*obj->inertia <<endl;

  return true;
}

arr ShapenetGrasps::sampleGraspPose(){
    return ::sampleGraspCandidate(C, "obj0_pts", "ref", opt.pregraspNormalSdv, opt.verbose);
}

arr sampleGraspCandidate(rai::Configuration& C, const char *ptsFrame, const char* refFrame, double pregraspNormalSdv, int verbose){

  rai::Frame *objPts = C[ptsFrame];
  rai::Frame *ref = C[refFrame];

  for(uint s=0;;s++){
    if(s>100){ //repeated fail
      if(verbose>0){
        LOG(0) <<" FAILED creating candidate";
      }
      return arr{};
    }

    //-- pick random point on the surface
    rai::Mesh& mesh = objPts->shape->mesh();
    uint pt = rnd(mesh.V.d0);
    rai::Vector x = mesh.V[pt];
    rai::Vector n = mesh.Vn[pt];

    //-- move the ref frame to that point, align x to normal but with noise, fully random orientation otherwise
    rai::Transformation X;
    X.pos = x;
    X.rot.setRandom();
    rai::Quaternion rel;
    rel.setDiff(X.rot.getX(), n);
    rai::Quaternion noise;
    noise.setVector(pregraspNormalSdv*randn(3));
    X.rot = rel * X.rot * noise;
    ref->setRelativePose(X);

    if(verbose>0) C.view(verbose>2, "random pick point");

    //-- move the gripper to the ref pose, aligning poses, open gripper to .05
    rai::Transformation pose=ref->ensure_X();
    arr q = C.getJointState();
    q({0,6+1}) = pose.getArr7d();
    q(7) = .04;

    arr bounds = C.getJointLimits();
    bool inBounds = boundCheck(q, bounds);

    C.setJointState(q);
    C.ensure_proxies(true);
    double collisions = C.coll_totalViolation();

    if(verbose>0){
      if(verbose>2){
        C.coll_reportProxies(cout, .0, false);
      }
      C.view(verbose>2, STRING("random pick pose " <<s <<" collisions: " <<collisions <<" bounds: " <<inBounds));
      if(verbose>1) rai::wait(.1);
    }

    //-- reject if too much collision
    if(collisions>=.01)
      continue;

    if(!inBounds)
      continue;

#if 0
    //-- refine using optimization
    KOMO komo;
    komo.setConfig(C, true);
    komo.setTiming(1,1,1,0);
    //    komo.addControlObjective({}, 0, 1e-1);
    q.append(double(q(-1))); //for the mimic joint..
    komo.addObjective({}, FS_qItself, {}, OT_sos, {1e1}, q); //stay close to initialization
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e2});
    komo.addObjective({}, FS_oppose, {"dotA", "dotB", ptsFrame}, OT_sos, {1e1});
    // komo.addObjective({}, FS_distance, {"dotA", ptsFrame}, OT_sos, {1e2}, {-.01});
    // komo.addObjective({}, FS_distance, {"dotB", ptsFrame}, OT_sos, {1e2}, {-.01});

    //komo.view(true, "init");
    //-- solve and display
    komo.opt.animateOptimization=2;
    auto ret = rai::NLP_Solver(komo.nlp(), verbose).setInitialization(C.getJointState()) .solve();
    // komo.nlp()->checkJacobian(ret->x, 1e-4, komo.featureNames);
    C.setJointState(komo.getConfiguration_qOrg(0));

    if(verbose>0){
      cout <<"  refinement: " <<*ret <<endl;
      if(verbose>1){
        cout <<komo.report() <<endl;
        // cout <<komo.pathConfig.reportForces() <<endl;
        //    komo.view(true);
      }
      //C.get_viewer()->nonThreaded=true;
      C.view(verbose>2, "fine tuned pregrasp");
      //C.get_viewer()->savePng();
      if(verbose>1) rai::wait(.1);
    }

    //-- reject if not precise
    if(ret->eq+ret->ineq>=.01)
      continue;
#endif

    //-- success! break the loop
    break;
  }

  if(verbose>0){
    if(verbose>2){
      C.ensure_proxies(true);
      C.coll_reportProxies(std::cout, .01);
    }
    C.view(verbose>1, STRING("proposed grasp"));
  }

  rai::Transformation relGripperPose = C["gripper"]->ensure_X()/C[ptsFrame]->ensure_X();
  return relGripperPose.getArr7d();
}

arr ShapenetGrasps::evaluateGrasp(){
  rai::Frame *obj = C["obj0"];
  rai::Frame *ref = C["ref"];
  //rai::Frame *palm = C["palm"];
  rai::Frame *gripper = C["gripper"];

  if(obj->children.N<2){
    LOG(-1) <<"this object has no meshes (from convex decomp) associated -> can't launch physical simulation";
    rai::wait();
    return {};
  }

  //kill the ball joint before physx simulation!
  C["floatBall"]->setJoint(rai::JT_none);

  //define a control reference
  arr q_ref = C.getJointState();
  arr q_real = q_ref;

  //-- define reference motion directions
  arr dirs;
  arr v = gripper->ensure_X().rot.getX().getArr();
  dirs.append(v);
  dirs.append(-v);
  v = gripper->ensure_X().rot.getY().getArr();
  dirs.append(v);
  dirs.append(-v);
  v = gripper->ensure_X().rot.getZ().getArr();
  dirs.append(v);
  dirs.append(-v);
  dirs.reshape(-1,3);

  //start a sim
  PhysXInterface physx(C, opt.simVerbose, &physxOpt);
  physx.disableGravity(obj, true);

  arr scores(dirs.d0+1, 2);

  evalGripperPoses.resize(dirs.d0+2, 7);

  for(uint phase=0;phase<=dirs.d0;phase++){ //close, x, y, z, gravity
    evalGripperPoses[phase] = (gripper->get_X() / ref->parent->get_X()).getArr7d();

    //set the 'ref' frame to the current grasp center -> diff between 'ref' and 'gripper' indicates in hand motion in each phase
    ref->setPose(gripper->getPose());

    double totalOffset=0.;
    double totalRot=0.;

    uint T=100;
    for(uint t=0;t<T;t++){

      //define control reference: move along 'dir'
      //if(phase==dirs.d0+1) physx.disableGravity(obj, false); //gravity
      if(phase>0 && phase-1<dirs.d0) q_ref({0,2+1}) += opt.moveSpeed*dirs[phase-1]; //move along dirs

      //always close gripper (compliant grasp)
      q_ref(3) -= opt.gripperCloseSpeed;
      rai::clip(q_ref(3), 0., .1);
      rai::clip(q_ref(3), q_real(3)-.01, q_real(3)+.01);

      //step simulation
      C.setJointState(q_ref);
      //physx.pushKinematicStates(C); //not necessary, no kinematic robot involved
      physx.pushMotorTargets(C);
      physx.step(opt.simTau);
      physx.pullDynamicStates(C);
      physx.pullMotorStates(C, NoArr);
      if(physx.opt().verbose>3){
        physx.getDebugConfig().view(false, STRING("Simulation physx debug time: " <<t));
      }
      q_real = C.getJointState();

      //measure in hand motion
      double offset = length(ref->getPosition() - gripper->getPosition());
      double rot = sqrt(quat_sqrDistance(ref->get_X().rot, gripper->get_X().rot));
      totalOffset += offset;
      totalRot += rot;

      //display
      if(opt.verbose>0){
        if(opt.verbose>1) rai::wait(opt.simTau);
        if(!(t%1)){
          //C.get_viewer()->nonThreaded=true;
          C.view(false, STRING("phase: " <<phase <<" t: " <<opt.simTau*t));
          //C.get_viewer()->savePng();
        }
      }
    }
    evalGripperPoses[-1] = (gripper->get_X() / ref->parent->get_X()).getArr7d();

    scores(phase,0) = totalOffset/double(T);
    scores(phase,1) = totalRot/double(T);
    if(opt.verbose>1) cout <<"  phase " <<phase <<" avgOffset: " <<scores(phase,0) <<" avgRotation: " <<scores(phase,1) <<endl;
  }

  scores = scores % arr{ 1./.03, 1./.5 };
  for(double& s:scores){  s = 1.-s; }

  if(opt.verbose>0){
    bool succ = min(scores)>.0;
    cout <<"  eval: " <<(succ?"success":"failure") <<' ' <<scores.reshape(-1) <<endl;
    C.view(opt.verbose>1, STRING("evaluation: " <<(succ?"success":"failure") <<" scores:\n" <<scores));
  }

  C["floatBall"]->setJoint(rai::JT_quatBall);

  return scores;
}

bool ShapenetGrasps::loadObject(uint shape, bool rndOrientation){
  clearScene();
  str file = opt.filesPrefix + files(shape);
  bool succ = addSceneObject(file, 0, rndOrientation);
  if(!succ){
    LOG(0) <<"loading object " <<shape <<" '" <<file <<"' failed";
    return false;
  }
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaFloatingGripper.g"));

  rai::Frame *ref = C.getFrame("ref");
  if(ref->parent) ref->unLink();
  ref->setParent(C.getFrame("obj0_pts"), false);

  return true;
}

arr ShapenetGrasps::getPointCloud(){
  rai::Frame * objPts = C["obj0_pts"];
  return objPts->getMeshPoints();
}

arr ShapenetGrasps::getPointNormals(){
  rai::Frame * objPts = C["obj0_pts"];
  return objPts->shape->mesh().Vn;
}

void ShapenetGrasps::getSamples(arr& X, uintA& shapes, arr& Scores, uint N){
  if(opt.endShape<0) opt.endShape = files.N;

  for(uint n=0;n<N;){
    uint shape = opt.startShape + rnd(opt.endShape-opt.startShape);
    if(opt.verbose>0){
      cout <<"sample " <<n <<", shape " <<shape <<" (" <<files(shape) <<")" <<endl;
    }

    //== CREATE SCENE
    bool succ = loadObject(shape, true);
    if(!succ) continue;
    if(opt.verbose>0) C.view(opt.verbose>2, STRING(shape <<"\nrandom obj pose"));
    //if(!n) C.view(true);

    //== SAMPLE A RANDOM+REJECT+REFINE GRASP POSE
    arr relGripperPose = sampleGraspPose();
    if(!relGripperPose.N) continue;

    //== PHYSICAL SIMULATION
    arr scores = evaluateGrasp();

    if(min(scores)>0.){ //success - store!
      X.append(relGripperPose);
      Scores.append(scores);
      shapes.append(shape);
      n++;
    }else{
    }
  }

  X.reshape(N, 7);
  Scores.reshape(N, -1);
  shapes.reshape(N);
}

arr ShapenetGrasps::evaluateSample(const arr& x, uint shape){
  loadObject(shape, true);

  setGraspPose(x);

  arr scores = evaluateGrasp();

  if(opt.verbose>0) C.view(opt.verbose>1, STRING(shape <<"\nevaluation done - success " <<(min(scores)>.0) <<" scores:\n" <<scores));

  return scores;
}

void ShapenetGrasps::setGraspPose(const arr& pose, const char* objPts){
  rai::Transformation gripperPose = C[objPts]->ensure_X() * rai::Transformation(pose);
  arr q = C.getJointState();
  q({-8,-2+1}) = gripperPose.getArr7d();
  q(7) = .04;
  C.setJointState(q);
}

void ShapenetGrasps::displaySamples(const arr& X, const uintA& shapes, const arr& Scores){
  uint N = X.d0;
  CHECK_EQ(N, shapes.N, "");

  C.get_viewer()->renderUntil=rai::_marker;

  rai::Configuration Cgripper;
  Cgripper.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaFloatingGripper.g"));
  clearScene();

  std::map<uint, uint> shape2place;

  for(uint i=0;i<N;i++){
    arr pose = X[i];
    uint shape = shapes(i);

    uint idx=0;
    if(shape2place.find(shape) == shape2place.end()){
      idx = shape2place.size();
      bool succ = addSceneObject(opt.filesPrefix+files(shape), idx, false);
      CHECK(succ, "");
      shape2place[shape] = idx;
    }else{
      idx = shape2place[shape];
    }

    //add gripper relative to objPts-idx
    C.addCopy(Cgripper.frames, {});
    setGraspPose(pose, STRING("obj"<<idx<<"_pts"));

    if(opt.verbose>0){
      //C.get_viewer()->nonThreaded=true;
      if(Scores.N) C.view(opt.verbose>1, STRING("sample #" <<i << " score:\n" <<Scores[i]));
      else C.view(opt.verbose>1, STRING("sample #" <<i));
      //for(uint k=0;k<20;k++) C.get_viewer()->savePng();
    }
  }
  C.view(true, STRING("display n:" <<N));
}
