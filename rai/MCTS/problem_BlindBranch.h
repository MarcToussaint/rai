/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "environment.h"
#include "../Core/array.h"

struct BlindBranch:MCTS_Environment {
  struct Action:SAO {
    Action(int d):d(d) {}
    int d;
    bool operator==(const SAO& other) const { return d==dynamic_cast<const Action&>(other).d; }
  };

  struct State:SAO {
    State(int sum, uint T):sum(sum), T(T) {}
    int sum;
    uint T;
    bool operator==(const SAO& other) const {
      const State& s = dynamic_cast<const State&>(other);
      return sum==s.sum && T==s.T;
    }
  };

  int state; //the state = sum of so-far actions
  int T; //current time (part of the state, actually!)
  int H; //horizon (parameter of the world)
  rai::Array<Handle> actions; //will contain handles on the -1 and +1 action

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
