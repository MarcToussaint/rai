/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "problem_BlindBranch.h"

BlindBranch::BlindBranch(uint H):H(H){
  actions = { Handle(new Action(-1)), Handle(new Action(+1)) };
}

void BlindBranch::reset_state(){ state=0; T=0; }

MCTS_Environment::TransitionReturn BlindBranch::transition(const MCTS_Environment::Handle& action){
  state += std::dynamic_pointer_cast<const Action>(action)->d;
  T++;
  double r=0.;
  if(is_terminal_state()) r = (double)state/H;
  return {Handle(NULL), r, 1.};
}

MCTS_Environment::TransitionReturn BlindBranch::transition_randomly(){
  if(mlr::rnd.uni()<.5) return transition(actions(0));
  return transition(actions(1));
}

const std::vector<MCTS_Environment::Handle> BlindBranch::get_actions(){
  return conv_arr2stdvec(actions);
}

const MCTS_Environment::Handle BlindBranch::get_state(){
  return MCTS_Environment::Handle(new State(state, T));
}

void BlindBranch::set_state(const MCTS_Environment::Handle& _state){
  auto s = std::dynamic_pointer_cast<const State>(_state);
  state = s->sum;
  T = s->T;
}

bool BlindBranch::is_terminal_state() const{ return T>=H; }


bool BlindBranch::get_info(InfoTag tag) const{
  switch(tag){
    case hasTerminal: return true;
    case isDeterministic: return true;
    case hasMaxReward: return true;
    case hasMinReward: return true;
    case isMarkov: return true;
    default: HALT("unknown tag" <<tag);
  }
}
double BlindBranch::get_info_value(InfoTag tag) const{
  switch(tag){
    case getMaxReward: return 1.;
    case getMinReward: return 0.;
    default: HALT("unknown tag" <<tag);
  }
}
