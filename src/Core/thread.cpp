/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "thread.h"
#include "graph.h"

#include <exception>
#include <signal.h>
#include <iomanip>

#ifndef RAI_MSVC
#ifndef __CYGWIN__
#  include <sys/syscall.h>
#else
#  include "cygwin_compat.h"
#endif //__CYGWIN __
#  include <unistd.h>
#else
#  define getpid _getpid
#endif
#include <errno.h>

namespace rai {

//===========================================================================

void Var_base::addCallback(const std::function<void (Var_base*)>& call, const void* callbackID) {
  callbacks.append(new Callback<void(Var_base*)>(callbackID, call));
}

int Var_base::read_lock() {
  rwlock.lock_shared();
  return revision;
}

int Var_base::write_lock() {
  rwlock.lock();
  isWriteLocked=true;
  write_time = clockTime();
  return revision+1;
}

int Var_base::read_unlock() {
  int i = revision;
  CHECK(!isWriteLocked, "");
  rwlock.unlock_shared();
  return i;
}

int Var_base::write_unlock() {
  int i = revision++; //before the callbacks and broadcasts, so that they now about the new revision
  for(auto* c:callbacks) c->call()(this);
  cond.notify_all();
  isWriteLocked = false;
  rwlock.unlock();
  return i;
}

//===========================================================================

void updateTimeIndicators(uint i, arr& mean, arr& max, const CycleTimer::timepoint& now, const CycleTimer::timepoint& last, uint step) {
  double dt = (now-last).count();
  if(dt<0.) dt=0.;
  double rate=.01;  if(step<100) rate=1./(1+step);
  while(mean.N<=i) mean.append(0.);
  while(max.N<=i) max.append(0.);
  mean.elem(i) = (1.-rate)*mean.elem(i) + rate*dt;
  if(dt>max.elem(i) || !(step%100)) max.elem(i) = dt;
}

CycleTimer::CycleTimer() {
  reset();
}

CycleTimer::~CycleTimer() {
}

void CycleTimer::reset() {
  steps=0;
  lastTime = std::chrono::high_resolution_clock::now();
  lastZero = lastTime;
}

void CycleTimer::tic(uint i=0) {
  now = std::chrono::high_resolution_clock::now();
  updateTimeIndicators(i, mean, max, now, (i?lastTime:lastZero), steps);
  lastTime=now;
  if(!i){
    lastZero=now;
    steps++;
  }
}

String CycleTimer::report() {
  str s;
  if(!steps){ s <<"cycle report: no steps"; return s; }
  s <<std::setprecision(3);
  s <<"cycle report [" <<steps <<"]: " <<mean(0) <<' ' <<1./mean(0) <<"Hz (max:" <<max(0) <<')';
  for(uint i=1;i<mean.N;i++){
    s <<" tic" <<i <<": " <<mean(i) <<' ' <<mean(i)/mean(0)*100. <<"% (max:" <<max(i) <<')';
  }
  return s;
}

//===========================================================================

#ifdef RAI_QThread
class sThread:QThread {
  Q_OBJECT
 public:
  Thread* th;
  sThread(Thread* _th, const char* name):th(_th) { setObjectName(name); }
  ~sThread() {}
  void open() { start(); }
  void close() { wait(); }
 protected:
  void run() { th->main();  }
};
#endif

Thread::Thread(const char* _name, double beatIntervalSec)
  : status(tsIsClosed),
    name(_name),
    tid(0),
    step_count(0),
    metronome(beatIntervalSec) {
  if(name.N>14) name.resize(14, true);
}

Thread::~Thread() {
  if(thread) {
    cout << "Call 'threadClose()' in the destructor of the DERIVED class! \
           That's because the 'virtual table is destroyed' before calling the destructor ~Thread (google 'call virtual function\
           in destructor') but now the destructor has to call 'threadClose' which triggers a Thread::close(), which is\
           pure virtual while you're trying to call ~Thread.";
    exit(1);
  }
}

void Thread::threadOpen(bool waitForOpened) {
  {
    auto lock = status.set();
    if(thread) return; //this is already open -- or has just beend opened (parallel call to threadOpen)
    thread = std::make_unique<std::thread>(&Thread::main, this);
#ifndef RAI_MSVC
    if(name) pthread_setname_np(thread->native_handle(), name);
#endif
    status.data=tsToOpen;
  }

  if(waitForOpened) status.waitForNotEq(tsToOpen);

  if(metronome.ticInterval>0.) {
    if(metronome.ticInterval>1e-10) {
      status.setValue(tsBEATING);
    } else {
      status.setValue(tsLOOPING);
    }
  }
}

void Thread::threadStep() {
  threadOpen();
  status.setValue(tsToStep);
}

void Thread::threadClose(double timeoutForce) {
  stopListening();
  status.setValue(tsToClose);
  if(!thread) { status.setValue(tsIsClosed); return; }
  status.waitForEq(tsIsClosed);
  thread->join();
  thread.reset();
}

void Thread::threadCancel() {
  stopListening();
  status.setValue(tsToClose);
  if(!thread) return;
#ifndef RAI_MSVC
  int rc;
  rc = pthread_cancel(thread->native_handle());         if(rc) HALT("pthread_cancel failed with err " <<rc <<" '" <<strerror(rc) <<"'");
#endif
  thread->join();
  thread.reset();
  stepMutex.state=-1; //forced destroy in the destructor
}

void Thread::threadLoop(bool waitForOpened) {
  threadOpen(waitForOpened);
  if(metronome.ticInterval>1e-10) {
    status.setValue(tsBEATING);
  } else {
    status.setValue(tsLOOPING);
  }
}

void Thread::threadStop(bool wait) {
  if(thread) {
    status.setValue(tsIDLE);
    if(wait) waitForIdle();
  }
}

void Thread::listenTo(Var_base& v) {
  auto lock = status.set();
  v.rwlock.lock();
  variables.append(&v);
  v.callbacks.append(new Callback<void(Var_base*)>(this, std::bind(&Thread::listeningCallback, this, std::placeholders::_1)));
  v.rwlock.unlock();
}

void Thread::stopListenTo(Var_base& v) {
  v.rwlock.lock();
  auto lock = status.set();
  int i=variables.findValue(&v);
  CHECK_GE(i, 0, "something's wrong");
  variables.remove(i);
  v.callbacks.removeCallback(this);
  v.rwlock.unlock();
}

void Thread::stopListening() {
  while(variables.N) stopListenTo(*variables.elem(-1));
}

void Thread::listeningCallback(Var_base* v) {
  int i = variables.findValue(v);
  CHECK_GE(i, 0, "signaler " <<v <<" was not registered with this event!");
  status.incrementStatus();
}

void Thread::main() {
  tid = getpid();

  {
    auto mux = stepMutex(RAI_HERE);
    try {
      open(); //virtual open routine
    } catch(const std::exception& ex) {
      status.setValue(tsFAILURE);
      cout <<"*** open() of Thread'" <<name <<"'failed: " <<ex.what() <<" -- closing it again" <<endl;
    } catch(...) {
      status.setValue(tsFAILURE);
      cout <<"*** open() of Thread '" <<name <<"' failed! -- closing it again";
      return;
    }
  }

  status.write_lock();
  if(status.data==tsToOpen) status.data=tsIDLE;
  status.write_unlock();

  timer.reset();
  for(;;) {
    //-- wait for a non-idle status
    int s = status.waitForNotEq(tsIDLE);
    if(s<=tsToClose) break;
    if(s==tsBEATING) metronome.waitForTic();
    if(s>0) status.setValue(1);  //multiple single step requests -> single step

    //-- make a step
    timer.tic(0);
    stepMutex.lock(RAI_HERE);
    step(); //virtual step routine
    stepMutex.unlock();
    step_count++;
    // timer.tic(1);

    if(s>0) status.setValue(tsIDLE); //single step -> reset to idle
  };

  stopListening();

  stepMutex.lock(RAI_HERE);
  close(); //virtual close routine
  stepMutex.unlock();

  status.setValue(tsIsClosed);
}

} //namespace

RUN_ON_INIT_BEGIN(thread)
rai::Array<shared_ptr<rai::Var_base>*>::memMove=true;
RUN_ON_INIT_END(thread)
