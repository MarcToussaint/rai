/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Logic/treeSearchDomain.h"
#include "AbstractEnvironment.h"

class InterfaceMarc: public AbstractEnvironment {
  //----typedefs/classes----//
 public:
  struct InterfaceMarcAction: public Action {
    InterfaceMarcAction(TreeSearchDomain::Handle action): action(action) {}
    virtual bool operator==(const Action& other) const {
      auto interface_action = dynamic_cast<const InterfaceMarcAction*>(&other);
      return interface_action!=nullptr && *(interface_action->action)==*action;
    }
    virtual size_t get_hash() const {
      return action->get_hash();
    }
    virtual void write(std::ostream& out) const {
      action->write(out);
    }
    TreeSearchDomain::Handle action;
  };
  struct InterfaceMarcObservation: public Observation {
    InterfaceMarcObservation(TreeSearchDomain::Handle observation): observation(observation) {}
    virtual bool operator==(const Observation& other) const {
      auto interface_observation = dynamic_cast<const InterfaceMarcObservation*>(&other);
      return interface_observation!=nullptr && *(interface_observation->observation)==*(observation);
    }
    virtual size_t get_hash() const {
      return observation->get_hash();
    }
    virtual void write(std::ostream& out) const {
      observation->write(out);
    }
    TreeSearchDomain::Handle observation;
  };

  //----members----//
 public:
  std::shared_ptr<TreeSearchDomain> env_marc;

  //----methods----//
 public:
  InterfaceMarc(std::shared_ptr<TreeSearchDomain> env_marc): env_marc(env_marc) {}
  virtual observation_reward_pair_t transition(const action_handle_t& action_handle) {
    auto interface_action = std::dynamic_pointer_cast<const InterfaceMarcAction>(action_handle);
    assert(interface_action!=nullptr);
    auto return_value = env_marc->transition(interface_action->action);
    return observation_reward_pair_t(observation_handle_t(new InterfaceMarcObservation(return_value.first)), return_value.second);
  }
  template<class C>
  static std::shared_ptr<AbstractEnvironment> makeAbstractEnvironment(C* env) {
    auto mcts = dynamic_cast<TreeSearchDomain*>(env);
    assert(mcts!=nullptr);
    return std::shared_ptr<AbstractEnvironment>(
             std::make_shared<InterfaceMarc>(
               std::shared_ptr<TreeSearchDomain>(mcts)));
  }
  virtual action_container_t get_actions() {
    action_container_t action_container;
    for(auto action : env_marc->get_actions()) {
      action_container.push_back(action_handle_t(new InterfaceMarcAction(action)));
    }
    return action_container;
  }
  virtual void make_current_state_new_start() {
    env_marc->make_current_state_new_start();
  }
  virtual void reset_state() {
    env_marc->reset_state();
  }

  virtual bool has_terminal_state() const {
    return env_marc->get_info(TreeSearchDomain::InfoTag::hasTerminal);
  }
  virtual bool is_terminal_state() const {
    return env_marc->is_terminal_state();
  }
  virtual bool is_deterministic() const {
    return env_marc->get_info(TreeSearchDomain::InfoTag::isDeterministic);
  }
  virtual bool has_max_reward() const {
    return env_marc->get_info(TreeSearchDomain::InfoTag::hasMaxReward);
  }
  virtual reward_t max_reward() const {
    return (reward_t)env_marc->get_info_value(TreeSearchDomain::InfoTag::getMaxReward);
  }
  virtual bool has_min_reward() const {
    return env_marc->get_info(TreeSearchDomain::InfoTag::hasMinReward);
  }
  virtual reward_t min_reward() const {
    return (reward_t)env_marc->get_info_value(TreeSearchDomain::InfoTag::getMinReward);
  }
  virtual bool is_markov() const {
    return env_marc->get_info(TreeSearchDomain::InfoTag::isMarkov);
  }
};

