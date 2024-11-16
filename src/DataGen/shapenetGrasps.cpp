#include "shapenetGrasps.h"

#include "../Kin/frame.h"
#include "../KOMO/komo.h"
#include "../Optim/NLP_Solver.h"
#include "../Kin/kin_physx.h"
#include "../Core/h5.h"
#include "../Kin/viewer.h"

ShapenetGrasps::ShapenetGrasps(){
  files = fromFile<StringA>(opt.filesPrefix+"files");
  files.reshape(-1);
  if(opt.verbose>0){
    LOG(0) <<"files: " <<files(0) <<" ... " <<files(-1);
  }
}

void ShapenetGrasps::clearScene(){
  C.clear();

  rai::Frame *ref = C.addFrame("ref");
  ref->setShape(rai::ST_marker, {.02});
  ref->setColor({1.,1.,0.});
}

bool ShapenetGrasps::createScene(const char* file, int idx, bool rndPose, bool visual){
  H5_Reader H(file);

  rai::Frame *ref = C.getFrame("ref");
  rai::Frame *obj = C.addFrame(STRING("obj"<<idx));

  {
    arr pts = H.read<double>("points/vertices");
    arr normals = H.read<double>("points/normals");

    rai::Frame *objPts = C.addFrame(STRING("objPts"<<idx));
    objPts->setParent(obj);
    objPts->setPointCloud(pts, {}, normals);
//    objPts->setMesh(pts);
    objPts->setContact(0);
    objPts->setColor({1., 0., 0., .9});

    if(ref->parent) ref->unLink();
    ref->setParent(objPts, false);
  }

  if(!visual){
    arr pts = H.read<double>("decomp/vertices");
    uintA faces = H.read<uint>("decomp/faces");
    byteA colors = H.read<byte>("decomp/colors");
    uintA parts = H.read<uint>("decomp/parts");

    rai::Frame *objMeshes = C.addFrame(STRING("objMeshes"<<idx));
    objMeshes->setParent(obj);
    objMeshes->setMesh(pts, faces, colors, parts);
    objMeshes->setContact(1);
    objMeshes->setMass(.1);

    obj->computeCompoundInertia();
    obj->transformToDiagInertia();

    objMeshes->convertDecomposedShapeToChildFrames();
    if(!objMeshes->children.N){
      //there are no collision shapes
      return false;
    }
  }else{
    arr pts = H.read<double>("mesh/vertices");
    uintA faces = H.read<uint>("mesh/faces");

    rai::Frame *objMesh = C.addFrame(STRING("objMesh"<<idx));
    objMesh->setParent(obj);
    objMesh->setMesh(pts, faces);
  }

  try{ obj->get_X().checkNan(); } catch(...) {
    //obj transform is buggy (typically singular inertia)
    return false;
  }

  if(rndPose) obj->set_X()->setRandom();
  obj->setPosition({double(idx),0.,1.});

  return true;
}

bool ShapenetGrasps::generateRndCandidate(){

  rai::Frame *objPts = C["objPts0"];
  rai::Frame *ref = C["ref"];

  for(uint s=0;;s++){
    if(s>100) return false;

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
    noise.setVec(opt.pregraspNormalSdv*randn(3));
    X.rot = rel * X.rot * noise;
    ref->setRelativePose(X);

    if(opt.verbose>0) C.view(opt.verbose>2, "random pick point");

    //-- move the gripper to the ref pose, aligning poses, open gripper to .05
    rai::Transformation pose=ref->ensure_X();
    arr q = C.getJointState();
    q({0,6}) = pose.getArr7d();
    q(7) = .04;

    arr bounds = C.getJointLimits();
    bool inBounds = boundCheck(q, bounds);

    C.setJointState(q);
    C.ensure_proxies(true);
    double collisions = C.getTotalPenetration();

    if(opt.verbose>0){
      if(opt.verbose>1){
        C.reportProxies(cout, .0, false);
      }
      C.view(opt.verbose>2, STRING("AArandom pick pose " <<s <<" collisions: " <<collisions <<" bounds: " <<inBounds));
      C.view(opt.verbose>2, STRING("BBrandom pick pose " <<s <<" collisions: " <<collisions <<" bounds: " <<inBounds));
      if(opt.verbose>1) rai::wait(.1);
    }

    //-- reject if too much collision
    if(collisions>=.01)
      continue;

    if(!inBounds)
      continue;

    //-- refine using optimization
    KOMO komo;
    komo.setConfig(C, true);
    komo.setTiming(1,1,1,0);
    //    komo.addControlObjective({}, 0, 1e-1);
    q.append(q(-1)); //for the mimic joint..
    komo.addObjective({}, FS_qItself, {}, OT_sos, {1e0}, q);
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e2});
    komo.addObjective({}, FS_oppose, {"dotA", "dotB", "objPts0"}, OT_sos, {1e1});
    //      komo.addObjective({}, FS_distance, {"dotA", "objPts"}, OT_sos, {1e2}, {-.01});
    //      komo.addObjective({}, FS_distance, {"dotB", "objPts"}, OT_sos, {1e2}, {-.01});

    //komo.view(true, "init");
    //-- solve and display
    auto ret = NLP_Solver(komo.nlp(), opt.optVerbose).setInitialization(C.getJointState()) .solve();
    //komo.nlp()->checkJacobian(ret->x, 1e-4, komo.featureNames);
    C.setJointState(komo.getConfiguration_qOrg(0));

    if(opt.verbose>0){
      cout <<"  refinement: " <<*ret <<endl;
      if(opt.verbose>1){
        cout <<komo.report() <<endl;
        cout <<komo.pathConfig.reportForces() <<endl;
        //    komo.view(true);
      }
      C.view(opt.verbose>2, "fine tuned pregrasp");
      if(opt.verbose>1) rai::wait(.1);
    }

    //-- reject if not precise
    if(ret->eq+ret->ineq>=.01)
      continue;

    //-- success! break the loop
    return true;
  }
}

arr ShapenetGrasps::evaluateCandidate(){
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

  for(uint phase=0;phase<=dirs.d0;phase++){ //close, x, y, z, gravity

    //set the 'ref' frame to the current grasp center -> diff between 'ref' and 'gripper' indicates in hand motion in each phase
    ref->setPose(gripper->getPose());

    double totalOffset=0.;
    double totalRot=0.;

    uint T=100;
    for(uint t=0;t<T;t++){

      //define control reference: move along 'dir'
      //if(phase==dirs.d0+1) physx.disableGravity(obj, false); //gravity
      if(phase>0 && phase-1<dirs.d0) q_ref({0,2}) += opt.moveSpeed*dirs[phase-1]; //move along dirs

      //always close gripper (compliant grasp)
      q_ref(3) -= opt.gripperCloseSpeed;
      rai::clip(q_ref(3), 0., .1);
      rai::clip(q_ref(3), q_real(3)-.01, q_real(3)+.01);

      //step simulation
      C.setJointState(q_ref);
      //physx.pushKinematicStates(C); //not necessary, no kinematic robot involved
      physx.pushMotorStates(C);
      physx.step(opt.simTau);
      physx.pullDynamicStates(C);
      q_real = C.getJointState();

      //measure in hand motion
      double offset = length(ref->getPosition() - gripper->getPosition());
      double rot = sqrt(quat_sqrDistance(ref->get_X().rot, gripper->get_X().rot));
      totalOffset += offset;
      totalRot += rot;

      //display
      if(opt.verbose>0){
        if(opt.verbose>1) rai::wait(opt.simTau);
        if(!(t%10)) C.view(false, STRING("phase: " <<phase <<" t: " <<opt.simTau*t));
      }
    }

    scores(phase,0) = totalOffset/double(T);
    scores(phase,1) = totalRot/double(T);
    if(opt.verbose>1) cout <<"  phase " <<phase <<" avgOffset: " <<scores(phase,0) <<" avgRotation: " <<scores(phase,1) <<endl;
  }

  scores = scores % arr{ 1./.03, 1./.5 };
  for(double& s:scores){  s = 1.-s; }

  return scores;
}

void ShapenetGrasps::getSamples(arr& X, arr& Contexts, arr& Scores, uint N){
  if(opt.numShapes<0) opt.numShapes = files.N - opt.startShape;

  for(uint n=0;n<N;){
    uint shape = opt.startShape + rnd(opt.numShapes);

    if(opt.verbose>0){
      cout <<"sample " <<n <<", shape " <<shape <<" (" <<files(shape) <<")" <<endl;
    }

    //== CREATE SCENE
    clearScene();
    bool succ = createScene(opt.filesPrefix+files(shape), 0, true);
    if(!succ) continue;
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaFloatingGripper.g"));
    if(opt.verbose>0) C.view(opt.verbose>2, STRING(shape <<"\nrandom obj pose"));

    //== SAMPLE A RANDOM+REJECT+REFINE GRASP POSE
    succ = generateRndCandidate();
    if(!succ){
      cout <<" FAILED creating candidate";
      continue;
    }
    if(opt.verbose>0){
      if(opt.verbose>2){
        C.ensure_proxies(true);
        C.reportProxies(std::cout, .01);
      }
      C.view(opt.verbose>1, STRING(shape <<"\nproposed grasp"));
    }
    rai::Transformation relGripperPose = C["gripper"]->ensure_X()/C["objPts0"]->ensure_X();

    //== PHYSICAL SIMULATION
    arr scores = evaluateCandidate();
    succ = min(scores)>.0;

    if(opt.verbose>0){
      cout <<"  eval: " <<succ <<' ' <<scores.reshape(-1) <<endl;
      C.view(opt.verbose>1, STRING(shape <<"\nevaluation - success " <<succ <<" scores:\n" <<scores));
    }

    if(succ){ //success - store!
      X.append(relGripperPose.getArr7d());
      Scores.append(scores);
      Contexts.append(double(shape));
      n++;
    }else{
    }
  }

  X.reshape(N, 7);
  Scores.reshape(N, -1);
  Contexts.reshape(N, 1);
}

arr ShapenetGrasps::evaluateSample(const arr& x, const arr& context){
  int shape = int(context.elem());

  clearScene();
  bool succ = createScene(opt.filesPrefix+files(shape), 0,true);
  CHECK(succ, "");
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaFloatingGripper.g"));
  setRelGripperPose(x);

  arr scores = evaluateCandidate();
  succ = min(scores)>.0;

  if(opt.verbose>0) C.view(opt.verbose>1, STRING(shape <<"\nevaluation done - success " <<succ <<" scores:\n" <<scores));

  return scores;
}

void ShapenetGrasps::setRelGripperPose(const arr& pose, const char* objPts){
  rai::Transformation gripperPose = C[objPts]->ensure_X() * rai::Transformation(pose);
  arr q = C.getJointState();
  q({-8,-2}) = gripperPose.getArr7d();
  q(7) = .04;
  C.setJointState(q);
}

void ShapenetGrasps::displaySamples(const arr& X, const arr& Contexts, const arr& Scores){
  uint N = X.d0;
  CHECK_EQ(N, Contexts.d0, "");

  C.get_viewer()->renderUntil=rai::_marker;

  rai::Configuration Cgripper;
  Cgripper.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaFloatingGripper.g"));
  clearScene();

  std::map<int, int> shape2place;

  for(uint i=0;i<N;i++){
    arr pose = X[i];
    int shape = uint(Contexts(i,0));

    int idx=0;
    if(shape2place.find(shape) == shape2place.end()){
      idx = shape2place.size();
      bool succ = createScene(opt.filesPrefix+files(shape), idx, false, true);
      CHECK(succ, "");
      shape2place[shape] = idx;
    }else{
      idx = shape2place[shape];
    }

    //add gripper relative to objPts-idx
    C.addCopy(Cgripper.frames, {});
    setRelGripperPose(pose, STRING("objPts"<<idx));

    if(opt.verbose>0){
      if(Scores.N) C.view(opt.verbose>1, STRING("sample #" <<i << " score:\n" <<Scores[i]));
      else C.view(opt.verbose>1, STRING("sample #" <<i));
    }
  }
  C.view(true, STRING("display n:" <<N));
}
