#include "bounds.h"
//#include <Kin/switch.h>
#include <Kin/TM_transition.h>

template<> const char* rai::Enum<BoundType>::names []= {
  "symbolic", "pose", "seq", "path", "seqPath", NULL
};

void skeleton2Bound(KOMO& komo, BoundType boundType, const Skeleton& S,
                    const rai::KinematicWorld& startKinematics,
                    const rai::KinematicWorld& effKinematics,
                    bool collisions, const arrA& waypoints){
  double maxPhase=0;
  for(const SkeletonEntry& s:S){
    if(s.phase0>maxPhase) maxPhase=s.phase0;
    if(s.phase1>maxPhase) maxPhase=s.phase1;
  }
  komo.clearObjectives();
  //-- prepare the komo problem
  switch(boundType) {
    case BD_pose: {
      //-- grep only the latest entries in the skeleton
      Skeleton finalS;
      for(const SkeletonEntry& s:S) if(s.phase0>=maxPhase){
        finalS.append(s);
        finalS.last().phase0 -= maxPhase-1.;
        finalS.last().phase1 -= maxPhase-1.;
      }

      komo.setModel(effKinematics, collisions);
      komo.setTiming(1., 1, 10., 1);

      komo.setHoming(0., -1., 1e-2);
      komo.setSquaredQVelocities(1., -1., 1e-1); //IMPORTANT: do not penalize transitions of from prefix to x_{0} -> x_{0} is 'loose'
      komo.setSquaredQuaternionNorms();

      komo.setSkeleton(finalS, false);

      //-- deactivate all velocity objectives except for transition
      for(Objective *o:komo.objectives){
        if(!std::dynamic_pointer_cast<TM_Transition>(o->map) && o->map->order>0){
          o->vars.clear();
        }
      }

      if(collisions) komo.add_collision(false);

      komo.reset();
//      komo.setPairedTimes();
    } break;
    case BD_seq: {
      komo.setModel(startKinematics, collisions);
      komo.setTiming(maxPhase+1., 1, 5., 1);

      komo.setSquaredQuaternionNorms();
      komo.setSquaredQAccelerations_novel(0, -1., 1., 1e-1);
//      komo.setHoming(0., -1., 1e-2);
//      komo.setSquaredQVelocities(0., -1., 1e-2);
//      komo.setFixEffectiveJoints(0., -1., 1e2);
//      komo.setFixSwitchedObjects(0., -1., 1e2);

      komo.setSkeleton(S);

      if(collisions) komo.add_collision(true, 0., 1e1);

      komo.reset();
//      komo.setPairedTimes();
      //      cout <<komo.getPath_times() <<endl;
    } break;
    case BD_path: {
      komo.setModel(startKinematics, collisions);
      uint stepsPerPhase = rai::getParameter<uint>("LGP/stepsPerPhase", 10);
      uint pathOrder = rai::getParameter<uint>("LGP/pathOrder", 2);
      komo.setTiming(maxPhase+.5, stepsPerPhase, 10., pathOrder);

      komo.setSquaredQuaternionNorms(0., -1., 1e1);
#if 0
      komo.setHoming(0., -1., 1e-2);
      if(pathOrder==1) komo.setSquaredQVelocities();
      else komo.setSquaredQAccelerations();
#else
      komo.setSquaredQAccelerations_novel(0, -1., 1., 1e-1);
#endif

      komo.setSkeleton(S);

      if(collisions) komo.add_collision(true, 0, 1e1);

      komo.reset();
      //      cout <<komo.getPath_times() <<endl;
    } break;
    case BD_seqPath: {
      komo.setModel(startKinematics, collisions);
      uint stepsPerPhase = rai::getParameter<uint>("LGP/stepsPerPhase", 10);
      uint pathOrder = rai::getParameter<uint>("LGP/pathOrder", 2);
      komo.setTiming(maxPhase+.5, stepsPerPhase, 10., pathOrder);

      komo.setSquaredQuaternionNorms();
      komo.setSquaredQAccelerations_novel(0, -1., 1., 1e-1);
//      komo.setHoming(0., -1., 1e-2);
//      if(pathOrder==1) komo.setSquaredQVelocities();
//      else komo.setSquaredQAccelerations();

      CHECK_EQ(waypoints.N-1, floor(maxPhase+.5), "");
      for(uint i=0;i<waypoints.N-1;i++){
        komo.addObjective(ARR(double(i+1)), OT_sos, FS_qItself, {}, {1e-1}, waypoints(i));
      }

      komo.setSkeleton(S);
      //delete all added objectives! -> only keep switches
//      uint O = komo.objectives.N;
//      for(uint i=O; i<komo.objectives.N; i++) delete komo.objectives(i);
//      komo.objectives.resizeCopy(O);

      if(collisions) komo.add_collision(true, 0, 1e1);

      komo.reset();
      komo.initWithWaypoints(waypoints);
      //      cout <<komo.getPath_times() <<endl;
    } break;

    case BD_seqVelPath: {
      komo.setModel(startKinematics, collisions);
      uint stepsPerPhase = rai::getParameter<uint>("LGP/stepsPerPhase", 10);
      komo.setTiming(maxPhase+.5, stepsPerPhase, 10., 1);

      komo.setHoming(0., -1., 1e-2);
      komo.setSquaredQVelocities();
      komo.setSquaredQuaternionNorms();

      CHECK_EQ(waypoints.N-1, floor(maxPhase+.5), "");
      for(uint i=0;i<waypoints.N-1;i++){
        komo.addObjective(ARR(double(i+1)), OT_sos, FS_qItself, {}, {1e-1}, waypoints(i));
//        komo.addObjective(ARR(double(i+1)), OT_eq, FS_qItself, {}, {1e0}, waypoints(i));
      }
//      uint O = komo.objectives.N;

      komo.setSkeleton(S);
      //delete all added objectives! -> only keep switches
//      for(uint i=O; i<komo.objectives.N; i++) delete komo.objectives(i);
//      komo.objectives.resizeCopy(O);

      if(collisions) komo.add_collision(true, 0, 1e1);

      komo.reset();
      komo.initWithWaypoints(waypoints, false);
      //      cout <<komo.getPath_times() <<endl;
    } break;

    default: NIY;
  }
}


struct SubCG{
  NodeL frames;
  NodeL constraints;
  int merged=-1;
  void write(ostream& os) const{
    cout <<"*** subproblem (merged:" <<merged <<")" <<endl;
    cout <<"  frames:";
    for(Node *f:frames) cout <<' ' <<*f;
    cout <<"\n  constraints:";
    for(Node *c:constraints) cout <<"\n    " <<*c;
    cout <<endl;
  }
};
stdOutPipe(SubCG)

void skeleton2CGO(const Skeleton& S, const rai::KinematicWorld& startKinematics, bool collisions){
  cout <<"*** " <<RAI_HERE <<endl;
  writeSkeleton(cout, S);

  double maxPhase=0;
  for(const SkeletonEntry& s:S) if(s.phase1>maxPhase) maxPhase=s.phase1;
  cout <<"maxPhase:" <<maxPhase <<endl;

  StringA frames;
  for(const SkeletonEntry& s:S){
    frames.append(s.frames);
  }
  frames.sort();
  frames.removeDoublesInSorted();
  cout <<"frames: " <<frames <<endl;

  //-- create graph vertices
  Graph CGO;
  for(int t=0;t<=maxPhase;t++){
    for(rai::String& f:frames){
      CGO.newNode<int>({STRING(f <<'_' <<t)}, {}, t);
    }
  }

  //-- add graph constraints
  for(const SkeletonEntry& s:S){
    for(uint t=s.phase0; t<=s.phase1;t++){
      NodeL parents;
      for(const rai::String& f:s.frames) parents.append(CGO.getNode(STRING(f <<'_' <<t)));
      CGO.newNode<rai::Enum<SkeletonSymbol>>({}, parents, s.symbol);
      if(s.symbol==SY_stable || s.symbol==SY_stableOn){
        for(uint t2=t+1; t2<=s.phase1+1 && t2<=maxPhase; t2++){
          NodeL parents2=parents;
          for(const rai::String& f:s.frames) parents2.append(CGO.getNode(STRING(f <<'_' <<t2)));
          CGO.newNode<rai::Enum<SkeletonSymbol>>({}, parents2, s.symbol);
        }
      }
    }
  }
  cout <<"initial CGO:" <<CGO <<endl;

  //-- add implicit identical constraints
  for(rai::String& f:frames){
    for(uint t=0;t<=maxPhase;t++){
      bool stop=false;
      Node *n = CGO.getNode(STRING(f <<'_' <<t));
      for(Node *c:n->parentOf){
        SkeletonSymbol s = c->getValue<rai::Enum<SkeletonSymbol>>()->x;
        if(s==SY_stable
           || s==SY_stableOn
           || s==SY_dynamic
           || s==SY_dynamicOn
           || s==SY_dynamicTrans) stop=true;
        if(stop) break;
      }
      if(t>0){
        CGO.newNode<rai::Enum<SkeletonSymbol>>({}, {CGO.getNode(STRING(f <<'_' <<t-1)), n}, rai::Enum<SkeletonSymbol>(SY_identical));
      }
      if(stop) break;
    }
  }
  cout <<"CGO with identicals:" <<CGO <<endl;

  //-- collapse identicals
  NodeL ids;
  for(Node *c:CGO){
    if(c->isOfType<rai::Enum<SkeletonSymbol>>()
       && c->getValue<rai::Enum<SkeletonSymbol>>()->x == SY_identical) ids.append(c);
  }
  for(Node *c:ids){
    Node *a=c->parents(0), *b=c->parents(1);
    CGO.collapse(a,b);
#if 1
    //delete duplicate children
    NodeL dels;
    for(uint i=0;i<a->parentOf.N;i++) for(uint j=i+1;j<a->parentOf.N;j++){
      Node *ch1 = a->parentOf.elem(i);
      Node *ch2 = a->parentOf.elem(j);
      if(ch1!=ch2 && ch1->parents==ch2->parents && ch1->hasEqualValue(ch2)) dels.append(ch2);
    }
    for(Node *d:dels) delete d;
#endif
  }

  cout <<"CGO with identicals removed:" <<CGO <<endl;

//  CGO.displayDot();

  //-- generate list of subproblems
  StringA switches = {"initial", "stable", "stableOn"};

  //size 1
  rai::Array<std::shared_ptr<SubCG>> subproblems;
  for(Node *c:CGO){
    if(c->isOfType<rai::Enum<SkeletonSymbol>>()
       && !switches.contains(c->getValue<rai::Enum<SkeletonSymbol>>()->name())){

      auto sp = make_shared<SubCG>();
      subproblems.append(sp);

      //frames
      for(Node *p:c->parents) sp->frames.append(p);
      sp->frames.sort();
      //constraints
      for(Node *c2:CGO){
        if(c2->isOfType<rai::Enum<SkeletonSymbol>>()
           && sp->frames.contains(c2->parents) ){
          sp->constraints.append(c2);
        }
      }
    }
  }

  cout <<"PROBLEMS size 1:" <<subproblems;

  //size 2
  for(uint lev=1;;lev++){
    //compute distances between subproblems
    arr D(subproblems.N, subproblems.N);
    D = -1;
    double mind=-1;
    int mini=-1, minj=-1;
    for(uint i=0;i<D.d0;i++) for(uint j=i+1;j<D.d1;j++){
      NodeL join = cat(subproblems(i)->frames, subproblems(j)->frames).sort().removeDoublesInSorted();
      bool exists=false;
      for(auto& sp:subproblems) if(join==sp->frames){ exists=true; break; }
      if(exists) continue;

      double d = distance(subproblems(i)->frames, subproblems(j)->frames);
      if(d>=0.) d += 0.01 * join.N;
      D(i,j) = D(j,i) = d;
      if(d>=0. && (mind<0 || d<mind)){ mind=d; mini=i; minj=j; }
    }
    cout <<"SP distance:\n" <<D <<endl;
    rai::wait();
    if(mind==-1.) break;

    //merge the pair with minimal distance
    auto sp = make_shared<SubCG>();
    subproblems.append(sp);
    sp->frames = cat(subproblems(mini)->frames, subproblems(minj)->frames);
    sp->frames.sort();
    sp->frames.removeDoublesInSorted();
    for(Node *c:CGO){
      if(c->isOfType<rai::Enum<SkeletonSymbol>>()
         && sp->frames.contains(c->parents) ){
        sp->constraints.append(c);
      }
    }
    subproblems(mini)->merged = minj;
    subproblems(minj)->merged = mini;
    cout <<"PROBLEMS:\n" <<subproblems;
  }

}
