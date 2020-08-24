/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

KOMO::KOMO(const Graph& specs) : KOMO() {
  init(specs);
//  reset();
//  CHECK(x.N,"");
}

void KOMO::init(const Graph& specs) {
//  specs = _specs;

  Graph& glob = specs.get<Graph>("KOMO");
  stepsPerPhase=glob.get<double>("T");
  double duration=glob.get<double>("duration");
  maxPhase=glob.get<double>("phases", 1.);
  k_order=glob.get<double>("k_order", 2);

  if(glob["model"]) {
    FileToken model = glob.get<FileToken>("model");
    world.read(model);
  } else {
    world.init(specs);
  }

  if(glob["meldFixedJoints"]) {
    world.optimizeTree();
  }

  if(glob["makeConvexHulls"])
    makeConvexHulls(world.frames);

  if(glob["computeOptimalSSBoxes"]) {
    NIY;
    //for(Shape *s: world.shapes) s->mesh.computeOptimalSSBox(s->mesh.V);
    world.watch(true,);
  }

  if(glob["activateAllContacts"])
    for(Frame* a : world.frames) if(a->shape) a->shape->cont=true;

  world.swift().initActivations(world);
  FILE("z.komo.model") <<world;

//  if(MP) delete MP;
//  MP = new KOMO(world);
  if(stepsPerPhase>=0) setTiming(maxPhase, stepsPerPhase, duration);
//  MP->k_order=k_order;

  for(Node* n:specs) parseTask(n, stepsPerPhase);
}

void KOMO::setFact(const char* fact) {
  Graph specs;
  specs.readNode(STRING(fact));
  parseTask(specs.last());
}

bool KOMO::parseTask(const Node* n, int stepsPerPhase) {
  if(stepsPerPhase==-1) stepsPerPhase=T;
  //-- task?
  Task* task = Task::newTask(n, world, stepsPerPhase, T);
  if(task) {
    tasks.append(task);
    return true;
  }
  //-- switch?
  KinematicSwitch* sw = KinematicSwitch::newSwitch(n, world, stepsPerPhase, T);
  if(sw) {
    switches.append(sw);
    return true;
  }
//  LOG(-1) <<"task spec '" <<*n <<"' could not be parsed";
  return false;
}
