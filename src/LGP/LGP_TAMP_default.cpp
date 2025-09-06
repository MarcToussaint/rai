#include "LGP_TAMP_Abstraction.h"

#include "../KOMO/manipTools.h"
#include "../Kin/frame.h"
#include "initFol.h"
#include "../Search/AStar.h"

namespace rai {

//===========================================================================

struct Default_LGP_TAMP_Abstraction : LGP_TAMP_Abstraction {
  Configuration& C;
  std::shared_ptr<rai::FOL_World> L;
  std::shared_ptr<rai::AStar> fol_astar;

  ~Default_LGP_TAMP_Abstraction() {}

  virtual Configuration& getConfig(){ return C; }

  Default_LGP_TAMP_Abstraction(rai::Configuration& C, const char* lgpFile) : C(C) {
    LOG(0) <<"using lgpFile: '" <<lgpFile <<"'";

    Graph lgpConfig(lgpFile);

    params()->copy(lgpConfig, true);
    //cout <<"=== ALL PARAMS ===\n" <<params() <<endl;

    //setup FolWorld
    FileToken folFile = lgpConfig.get<FileToken>("fol");
    LOG(0) <<"using folFile '" <<folFile.fullPath() <<"'";
    L = make_shared<rai::FOL_World>();
    L->init(folFile.fullPath());
    initFolStateFromKin(*L, C);
    L->addTerminalRule(lgpConfig.get<String>("terminal")); //"(on gripper ball)");

    // lgpConfig.get<bool>("genericCollisions"),
    // lgpConfig.get<StringA>("coll", {}),
    // lgpConfig.get<StringA>("lifts", {}),
    // lgpConfig.get<String>("terminalSkeleton", {}));
    L->reset_state();

    //setup Astar search within the FOL world
    fol_astar = make_shared<rai::AStar>(make_shared<rai::FOL_World_State>(*L, nullptr, false));
    fol_astar -> verbose = rai::getParameter<int>("LGP/verbose", 1) - 2;

    //other problem options
    explicitCollisions = lgpConfig.get<StringA>("coll", {});
    useBroadCollisions = lgpConfig.get<bool>("genericCollisions");
  }

  virtual Array<StringA> getNewActionSequence(){
    fol_astar->run();
    FOL_World_State* s = dynamic_cast<FOL_World_State*>(fol_astar->solutions(-1));
#if 0 //get state sequence
    String planString;
    Array<Graph*> states;
    arr times;
    s->getStateSequence(states, times, planString);
    LOG(0) <<states <<times <<planString;
#else //get action sequence (as stringAA)
    str debug;
    NodeL decisions = s->getDecisionSequence(debug);
    Array<StringA> actionSequence(decisions.N);
    for(uint i=0;i<actionSequence.N;i++){
      actionSequence(i).resize(decisions(i)->parents.N);
      for(uint j=0;j<actionSequence(i).N;j++) actionSequence(i)(j) = decisions(i)->parents(j)->key;
    }
#endif
    return actionSequence;
  }

  virtual std::shared_ptr<KOMO> setup_sequence(Configuration& C, uint K){
    ManipulationHelper manip;
    manip.setup_sequence(C, K, -1e-2, 1e-2, false, false, true);
    // manip.setup_sequence(C, K,);
    return manip.komo;
  }

  virtual std::shared_ptr<KOMO> setup_motion(Configuration& C, uint K){
    ManipulationHelper manip;
    manip.setup_motion(C, K, 30, -1.);
    return manip.komo;
  }

  virtual void add_action_constraints(std::shared_ptr<KOMO>& komo, double time, const StringA& action){
    if(!action.N) return;

    ManipulationHelper manip(komo);

    if(action(0)=="pick_box" || action(0)=="handover" || action(0)=="pick_touch"){
      str& obj = action(1);
      str& gripper = action(3);
      manip.action_pick(action(0), time, gripper, obj);

    }else if(action(0)=="place_straightOn"){
      str& obj = action(1);
      str& table = action(3);
      manip.action_place_straightOn(action(0), time, obj, table);

    }else if(action(0)=="place_box"){
      str& obj = action(1);
      str& gripper = action(2);
      str& table = action(3);
      manip.action_place_box(action(0), time, obj, table, gripper, "x");

    }else if(action(0)=="poseEq"){
      str& obj = action(1);
      str& target = action(2);
      komo->addObjective({time}, FS_poseDiff, {obj, target}, OT_eq, {1e1});

    }else if(action(0)=="gripper_push"){
      str& obj = action(1);
      str& table = action(2);
      str& gripper = action(3);

      if(time<manip.komo->T/manip.komo->stepsPerPhase){
        str snapFrame; snapFrame <<"pushPose_" <<gripper <<'_' <<obj <<'_' <<time;
        manip.komo->addFrameDof(snapFrame, gripper, rai::JT_free, true, obj);
        manip.komo->addRigidSwitch(time, {snapFrame, obj});
        if(manip.komo->stepsPerPhase>2) manip.komo->addObjective({time}, FS_poseDiff, {snapFrame, obj}, OT_eq, {1e0}, NoArr, 0, -1, 0);
      }

      manip.straight_push({time,time+1}, obj, gripper, table);

    }else if(action(0)=="end_push"){
      //str& gripper = action(1);
      str& obj = action(2);
      //str& floor = action(3);
      str& target = action(4);

      if(time<manip.komo->T/manip.komo->stepsPerPhase){
        NIY;
      }

      manip.place_box(time, obj, target, 0, "z");

    }else{
      HALT("action constraint not implemented: " <<action);
    }
  }

  virtual void add_action_constraints_motion(std::shared_ptr<KOMO>& komo, double time, const StringA& prev_action, const StringA& action, uint actionPhase){
    if(!action.N) return;

    //    ManipulationHelper manip(komo);
    //
    //     if(action(0)=="pick" || action(0)=="handover"){
    //       str& gripper = action(3);
    //       manip.retract({time-1., time-.8}, gripper);
    //       manip.approach({time-.2, time}, gripper);
    //     }
    //     else if(action(0)=="place"){
    //       str& gripper = action(2);

    //       manip.retract({time-1., time-.8}, gripper);
    //       manip.approach({time-.2, time}, gripper);
    //     }
    //     else if(action(0)=="end_push"){
    //       str& gripper = action(1);
    //       str& obj = action(2);

    //       //    komo->addObjective(time_interval, FS_positionRel, {gripper, helperStart}, OT_eq, 1e1*arr{{2, 3}, {1., 0., 0., 0., 0., 1.}});
    //       str helperEnd = STRING("_straight_pushEnd_" <<gripper <<"_" <<obj <<'_' <<actionPhase+1);

    // //      komo->addObjective({time-1., time}, FS_positionRel, {obj, helperEnd}, OT_eq, 1e1*arr{{2, 3}, {1., 0., 0., 0., 0., 1.}});
    //       komo->addObjective({time-1., time}, FS_position, {obj}, OT_eq, 1e1*arr{{1, 3}, {0., 0., 1.}}, {}, 1);
    //       komo->addObjective({time-1., time}, FS_vectorZ, {obj}, OT_eq, {1e1}, {0., 0., 1.});
    // //      komo->addObjective({time-1., time}, FS_quaternion, {obj}, OT_eq, {1e1}, {}, 1);
    //     }

  }
};

//===========================================================================

std::shared_ptr<LGP_TAMP_Abstraction> default_LGP_TAMP_Abstraction(rai::Configuration& C, const char* lgp_configfile){ return make_shared<Default_LGP_TAMP_Abstraction>(C, lgp_configfile); }

}//namespace
