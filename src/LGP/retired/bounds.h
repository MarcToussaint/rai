/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../KOMO/komo.h"
#include "../KOMO/skeleton.h"

//===========================================================================

struct KOMO_based_bound {
  shared_ptr<KOMO> komo;

  KOMO_based_bound(shared_ptr<KOMO>& komo) : komo(komo) {}
};

//===========================================================================

struct PoseBound : KOMO_based_bound {
  //the constructor creates the komo problem -- absorbes what previously was defined in skeleton2Bound!
  PoseBound(shared_ptr<KOMO>& komo, //TODO: eventually remove this!
            const rai::Skeleton& S,
            const rai::Configuration& startKinematics,
            bool collisions);
};

//===========================================================================

struct SeqBound : KOMO_based_bound {
  SeqBound(shared_ptr<KOMO>& komo, //TODO: eventually remove this!
           const rai::Skeleton& S,
           const rai::Configuration& startKinematics,
           bool collisions);
};

//===========================================================================

struct PathBound : KOMO_based_bound {
  PathBound(shared_ptr<KOMO>& komo, //TODO: eventually remove this!
            const rai::Skeleton& S,
            const rai::Configuration& startKinematics,
            bool collisions);
};

//===========================================================================

struct SeqPathBound : KOMO_based_bound {
  SeqPathBound(shared_ptr<KOMO>& komo, //TODO: eventually remove this!
               const rai::Skeleton& S,
               const rai::Configuration& startKinematics,
               bool collisions, const arrA& waypoints);
};

//===========================================================================

struct SeqVelPathBound : KOMO_based_bound {
  SeqVelPathBound(shared_ptr<KOMO>& komo, //TODO: eventually remove this!
                  const rai::Skeleton& S,
                  const rai::Configuration& startKinematics,
                  bool collisions, const arrA& waypoints);
};

//===========================================================================

enum BoundType { BD_all=-1,
                 BD_symbolic=0,
                 BD_pose,
                 BD_seq,
                 BD_path,
                 BD_seqPath,
                 BD_max
               };

shared_ptr<KOMO_based_bound> skeleton2Bound(shared_ptr<KOMO>& komo,
    BoundType boundType,
    const rai::Skeleton& S,
    const rai::Configuration& startKinematics,
    bool collisions,
    const arrA& waypoints= {}
                                           );

rai::SkeletonTranscription skeleton2Bound2(BoundType boundType, rai::Skeleton& S, const arrA& waypoints= {});

struct SubCG {
  rai::NodeL frames;
  rai::NodeL constraints;
  uint maxT;
  int merged=-1;
  void write(ostream& os) const {
    cout <<"*** subproblem (merged:" <<merged <<")" <<endl;
    cout <<"  frames:";
    for(rai::Node* f:frames) cout <<' ' <<*f;
    cout <<"\n  constraints:";
    for(rai::Node* c:constraints) cout <<"\n    " <<*c;
    cout <<endl;
  }
};
stdOutPipe(SubCG)
struct CG {
  rai::Graph G;
  rai::Array<std::shared_ptr<SubCG>> subproblems;
};
