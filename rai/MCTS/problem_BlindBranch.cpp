/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "problem_BlindBranch.h"

BlindBranch::BlindBranch(uint H):H(H) {
  actions = { Handle(new Action(-1)), Handle(new Action(+1)) };
}

void BlindBranch::reset_state() { state=0; T=0; }

MCTS_Environment::TransitionReturn BlindBranch::transition(const MCTS_Environment::Handle& action) {
  state += std::dynamic_pointer_cast<const Action>(action)->d;
  T++;
  double r=0.;
  if(is_terminal_state()) r = (double)state/H;
  return {Handle(nullptr), r, 1.};
}

MCTS_Environment::TransitionReturn BlindBranch::transition_randomly() {
  if(rnd.uni()<.5) return transition(actions(0));
  return transition(actions(1));
}

const std::vector<MCTS_Environment::Handle> BlindBranch::get_actions() {
  return actions.vec();
}

const MCTS_Environment::Handle BlindBranch::get_state() {
  return MCTS_Environment::Handle(new State(state, T));
}

void BlindBranch::set_state(const MCTS_Environment::Handle& _state) {
  auto s = std::dynamic_pointer_cast<const State>(_state);
  state = s->sum;
  T = s->T;
}

bool BlindBranch::is_terminal_state() const { return T>=H; }

bool BlindBranch::get_info(InfoTag tag) const {
  switch(tag) {
    case hasTerminal: return true;
    case isDeterministic: return true;
    case hasMaxReward: return true;
    case hasMinReward: return true;
    case isMarkov: return true;
    default: HALT("unknown tag" <<tag);
  }
}
double BlindBranch::get_info_value(InfoTag tag) const {
  switch(tag) {
    case getMaxReward: return 1.;
    case getMinReward: return 0.;
    default: HALT("unknown tag" <<tag);
  }
}
