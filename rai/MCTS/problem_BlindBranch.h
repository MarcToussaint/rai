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


#pragma once

#include "environment.h"
#include <Core/array.h>

struct BlindBranch:MCTS_Environment{
  struct Action:SAO{
    Action(int d):d(d){}
    int d;
    bool operator==(const SAO& other) const{ return d==dynamic_cast<const Action&>(other).d; }
  };

  struct State:SAO{
    State(int sum, uint T):sum(sum), T(T){}
    int sum;
    uint T;
    bool operator==(const SAO& other) const{
      const State& s = dynamic_cast<const State&>(other);
      return sum==s.sum && T==s.T;
    }
  };

  int state; //the state = sum of so-far actions
  int T; //current time (part of the state, actually!)
  int H; //horizon (parameter of the world)
  mlr::Array<Handle> actions; //will contain handles on the -1 and +1 action


  BlindBranch(uint H);
  void reset_state();
  TransitionReturn transition(const Handle& action);
  TransitionReturn transition_randomly();
  const std::vector<Handle> get_actions();
  const Handle get_state();
  void set_state(const Handle& _state);
  bool is_terminal_state() const;

  bool get_info(InfoTag tag) const;
  double get_info_value(InfoTag tag) const;
};
