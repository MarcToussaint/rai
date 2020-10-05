/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "../Core/thread.h"
#include <iomanip>

typedef std::function<int()> Script;

struct Act {
  Var<ActStatus> status;
  double startTime;

  Act() : startTime(rai::realTime()) {
    status.set()=AS_init;
  }
  virtual ~Act() {}

  double time() { return rai::realTime()-startTime; }
  virtual void write(ostream& os) { os <<'<' <<std::setw(14) <<niceTypeidName(typeid(*this)) <<"> @" <<std::setw(12) <<rai::Enum<ActStatus>(status()) <<std::setw(5) <<std::setprecision(3)<<time() <<"s -- "; }

  typedef std::shared_ptr<Act> Ptr;
};

struct Act_Script : Act, Thread {
  Script script;
  Act_Script(const Script& S, double beatIntervalSec=-1.)
    :  Thread("Act_Script", beatIntervalSec), script(S) {
    if(beatIntervalSec<0.) threadStep();
    else threadLoop();
  }

  ~Act_Script() { threadClose(); }

  virtual void open() {}
  virtual void step() { ActStatus r = (ActStatus)script(); status.set()=r; }
  virtual void close() {}
};

template<class T> struct Act_LoopStep : Act, Thread {
  ptr<T> script;
  Act_LoopStep(const ptr<T>& S, double beatIntervalSec=-1.)
    :  Thread("Act_Script", beatIntervalSec), script(S) {
    if(beatIntervalSec<0.) threadStep();
    else threadLoop();
  }

  ~Act_LoopStep() { threadClose(); }

  virtual void open() {}
  virtual void step() { ActStatus r = (ActStatus)script->step(); status.set()=r; }
  virtual void close() {}
};

inline Act::Ptr loop(const std::function<int ()>& script, double beatIntervalSec=.1) {
  return make_shared<Act_Script>(script, beatIntervalSec);
}

template<class T> Act::Ptr loop(const ptr<T>& script, double beatIntervalSec=.1) {
  return make_shared<Act_LoopStep<T>>(script, beatIntervalSec);
}

