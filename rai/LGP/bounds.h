#pragma once

#include <KOMO/komo.h>

enum BoundType{ BD_all=-1,
                BD_symbolic=0,
                BD_pose,
                BD_seq,
                BD_path,
                BD_seqPath,
                BD_seqVelPath,
                BD_poseFromSeq,
                BD_max };

void skeleton2Bound(KOMO& komo,
                    BoundType boundType,
                    const Skeleton& S,
                    const rai::Configuration& startKinematics,
                    const rai::Configuration& parentEffKinematics,
                    bool collisions,
                    const arrA& waypoints={}
                    );



struct SubCG{
  NodeL frames;
  NodeL constraints;
  uint maxT;
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
struct CG{
  Graph G;
  rai::Array<std::shared_ptr<SubCG>> subproblems;
};

ptr<CG> skeleton2CGO(const Skeleton& S,
                     const rai::Configuration& startKinematics,
                     bool collisions);

void CG2komo(KOMO& komo,
             const SubCG& scg,
             const rai::Configuration& C,
             bool collisions);
