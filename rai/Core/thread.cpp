/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
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

//===========================================================================

template<> const char* rai::Enum<ActStatus>::names []= {
  "init", "running", "done", "converged", "stalled", "true", "false", "kill", nullptr
};

//===========================================================================
//
// Access RWLock
//

RWLock::RWLock() {
  rwCount=0;
}

RWLock::~RWLock() {
    if (rwCount) {
        std::cerr << "Destroying locked RWLock" << endl;
        exit(1);
    }
}

void RWLock::readLock() {
  rwLock.lock_shared();
  rwCountMutex.lock(RAI_HERE);
  rwCount++;
  rwCountMutex.unlock();
}

void RWLock::writeLock() {
  rwLock.lock();
  rwCountMutex.lock(RAI_HERE);
  rwCount=-1;
  rwCountMutex.unlock();
}

void RWLock::unlock() {
  rwCountMutex.lock(RAI_HERE);
  if(rwCount>0){
    rwCount--;
    rwLock.unlock_shared();
  }else{
    rwCount=0;
    rwLock.unlock();
  }
  rwCountMutex.unlock();
}

bool RWLock::isLocked() {
  return rwCount!=0;
}

bool RWLock::isWriteLocked() {
  return rwCount<0;
}

//===========================================================================
//
// Signaler
//

Signaler::Signaler(int initialStatus)
  : status(initialStatus) {
}

Signaler::~Signaler() {
}

void Signaler::setStatus(int i, Signaler* messenger) {
  auto lock = statusMutex(RAI_HERE);
  status=i;
  broadcast(messenger);
}

int Signaler::incrementStatus(Signaler* messenger) {
  auto lock = statusMutex(RAI_HERE);
  status++;
  broadcast(messenger);
  return status;
}

void Signaler::broadcast(Signaler* messenger) {
  cond.notify_all();
}

void Event::listenTo(Var_base& v) {
  auto lock = statusMutex(RAI_HERE);
  v.readAccess();
  variables.append(&v);
  v.callbacks.append(new Callback<void(Var_base*)>(this, std::bind(&Event::callback, this, std::placeholders::_1)));
  v.deAccess();
}

void Event::stopListenTo(Var_base& v) {
  v.readAccess();
  auto lock = statusMutex(RAI_HERE);
  int i=variables.findValue(&v);
  CHECK_GE(i, 0, "something's wrong");
  variables.remove(i);
  v.callbacks.removeCallback(this);
  v.deAccess();
}

void Event::stopListening() {
  while(variables.N) stopListenTo(*variables.last());
}

void Event::callback(Var_base* v) {
  int i = variables.findValue(v);
  CHECK_GE(i, 0, "signaler " <<v <<" was not registered with this event!");
  if(eventFct) {
    int newEventStatus = eventFct(variables, i);
    //  cout <<"event callback: BOOL=" <<eventStatus <<' ' <<s <<' ' <<status <<" statuses=" <<statuses <<endl;
    setStatus(newEventStatus);
  } else { //we don't have an eventFct, just increment value
    incrementStatus();
  }
}

Event::Event(const rai::Array<Var_base*>& _variables, const EventFunction& _eventFct, int initialState)
  : Signaler(initialState), eventFct(_eventFct) {
  for(Var_base* v:_variables) listenTo(*v);
}

Event::~Event() {
  stopListening();
}

void Signaler::statusLock() {
  statusMutex.lock(RAI_HERE);
}

void Signaler::statusUnlock() {
  statusMutex.unlock();
}

int Signaler::getStatus(Mutex::Token *userHasLocked) const {
  Mutex* m = (Mutex*)&statusMutex; //sorry: to allow for 'const' access
  if(!userHasLocked) m->lock(RAI_HERE); else CHECK_EQ(m->state, getpid(), "user must have locked before calling this!");
  int i=status;
  if(!userHasLocked) m->unlock();
  return i;
}

bool Signaler::waitForSignal(Mutex::Token *userHasLocked, double timeout) {
  bool ret = true;
  if(userHasLocked){
    if(timeout<0.) {
      cond.wait(*userHasLocked);
    } else {
      ret = (cond.wait_for(*userHasLocked, std::chrono::duration<double>(timeout)) == std::cv_status::no_timeout);
    }
  }else{
    auto lk = statusMutex(RAI_HERE);
    if(timeout<0.) {
      cond.wait(lk);
    } else {
      ret = (cond.wait_for(lk, std::chrono::duration<double>(timeout)) == std::cv_status::no_timeout);
    }
  }
  return ret;
}

bool Signaler::waitForEvent(std::function<bool()> f, Mutex::Token *userHasLocked) {
  if(userHasLocked){
    cond.wait(*userHasLocked, f);
  }else{
    auto lk = statusMutex(RAI_HERE);
    cond.wait(lk, f);
  }
  return true;

}

bool Signaler::waitForStatusEq(int i, Mutex::Token *userHasLocked, double timeout) {
  bool ret = true;
  if(userHasLocked){
    while(status!=i) ret = waitForSignal(userHasLocked, timeout);
  }else{
    auto lk = statusMutex(RAI_HERE);
    while(status!=i) ret = waitForSignal(&lk, timeout);
  }
  return ret;
}

int Signaler::waitForStatusNotEq(int i, Mutex::Token *userHasLocked, double timeout) {
  if(userHasLocked){
    while(status==i) waitForSignal(userHasLocked, timeout);
  }else{
    auto lk = statusMutex(RAI_HERE);
    while(status==i) waitForSignal(&lk, timeout);
  }
  return status;
}

int Signaler::waitForStatusGreaterThan(int i, Mutex::Token *userHasLocked, double timeout) {
  if(userHasLocked){
    while(status<=i) waitForSignal(userHasLocked, timeout);
  }else{
    auto lk = statusMutex(RAI_HERE);
    while(status<=i) waitForSignal(&lk, timeout);
  }
  return status;
}

int Signaler::waitForStatusSmallerThan(int i, Mutex::Token* userHasLocked, double timeout) {
  if(userHasLocked){
    while(status>=i) waitForSignal(userHasLocked, timeout);
  }else{
    auto lk = statusMutex(RAI_HERE);
    while(status>=i) waitForSignal(&lk, timeout);
  }
  return status;
}

//===========================================================================
//
// VariableBase
//

Var_base::Var_base(const char* _name) : name(_name) {
}

Var_base::~Var_base() {
}

void Var_base::addCallback(const std::function<void (Var_base*)>& call, const void* callbackID) {
  callbacks.append(new Callback<void(Var_base*)>(callbackID, call));
}

int Var_base::readAccess(Thread* th) {
  rwlock.readLock();
  return revision;
}

int Var_base::writeAccess(Thread* th) {
  rwlock.writeLock();
  write_time = rai::clockTime();
  return revision+1;
}

int Var_base::deAccess(Thread* th) {
  int i;
  if(rwlock.rwCount == -1) { //log a revision after write access
    i = revision++;
    for(auto* c:callbacks) c->call()(this);
  } else {
    i = revision;
  }
  rwlock.unlock();
  return i;
}

//===========================================================================
//
// Metronome
//

Metronome::Metronome(double ticIntervalSec) {
  reset(ticIntervalSec);
}

void Metronome::reset(double ticIntervalSec) {
  ticTime = std::chrono::high_resolution_clock::now();
  tics=0;
  ticInterval = ticIntervalSec;
}

void Metronome::waitForTic() {
  auto interval = std::chrono::duration<double>(ticInterval);
  ticTime += interval;
  std::this_thread::sleep_until(ticTime);
  tics++;
}

double Metronome::getTimeSinceTic() {
  auto now = std::chrono::high_resolution_clock::now();
  return std::chrono::duration<double>(ticTime-now).count();
}

//===========================================================================
//
// CycleTimer
//

void updateTimeIndicators(double& dt, double& dtMean, double& dtMax, const CycleTimer::timepoint& now, const CycleTimer::timepoint& last, uint step) {
  dt = (now-last).count();
  if(dt<0.) dt=0.;
  double rate=.01;  if(step<100) rate=1./(1+step);
  dtMean = (1.-rate)*dtMean    + rate*dt;
  if(dt>dtMax || !(step%100)) dtMax = dt;
}

CycleTimer::CycleTimer(const char* _name) {
  reset();
  name=_name;
}

CycleTimer::~CycleTimer() {
}

void CycleTimer::reset() {
  steps=0;
  busyDt=busyDtMean=busyDtMax=1.;
  cyclDt=cyclDtMean=cyclDtMax=1.;
  lastTime = std::chrono::high_resolution_clock::now();
}

void CycleTimer::cycleStart() {
  now = std::chrono::high_resolution_clock::now();
  updateTimeIndicators(cyclDt, cyclDtMean, cyclDtMax, now, lastTime, steps);
  lastTime=now;
}

void CycleTimer::cycleDone() {
  now = std::chrono::high_resolution_clock::now();
  updateTimeIndicators(busyDt, busyDtMean, busyDtMax, now, lastTime, steps);
  steps++;
}

rai::String CycleTimer::report() {
  rai::String s;
  s.printf("busy=[%5.1f %5.1f] cycle=[%5.1f %5.1f] load=%4.1f%% steps=%i", busyDtMean, busyDtMax, cyclDtMean, cyclDtMax, 100.*busyDtMean/cyclDtMean, steps);
  return s;
}

//=============================================
//
// Thread
//

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
  : event(tsIsClosed),
    name(_name),
    tid(0),
    step_count(0),
    metronome(beatIntervalSec) {
  if(name.N>14) name.resize(14, true);
}

Thread::~Thread() {
    if (thread) {
        std::cerr << "Call 'threadClose()' in the destructor of the DERIVED class! \
           That's because the 'virtual table is destroyed' before calling the destructor ~Thread (google 'call virtual function\
           in destructor') but now the destructor has to call 'threadClose' which triggers a Thread::close(), which is\
           pure virtual while you're trying to call ~Thread.";
        exit(1);
    }
  }

void Thread::threadOpen(bool wait, int priority) {
  {
    auto lock = event.statusMutex(RAI_HERE);
    if(thread) return; //this is already open -- or has just beend opened (parallel call to threadOpen)
    thread = std::make_unique<std::thread>(&Thread::main, this);
#ifndef RAI_MSVC
    if(name) pthread_setname_np(thread->native_handle(), name);
#endif
    event.status=tsToOpen;
  }

  if(wait) event.waitForStatusNotEq(tsToOpen);

  if(metronome.ticInterval>0.) {
    if(metronome.ticInterval>1e-10) {
      event.setStatus(tsBEATING);
    } else {
      event.setStatus(tsLOOPING);
    }
  }
}

void Thread::threadClose(double timeoutForce) {
  event.stopListening();
  event.setStatus(tsToClose);
  if(!thread) { event.setStatus(tsIsClosed); return; }
  for(;;) {
    bool ended = event.waitForStatusEq(tsIsClosed, 0, .2);
    if(ended) break;
    LOG(-1) <<"timeout to end Thread::main of '" <<name <<"'";
//    if(timeoutForce>0.){
//      ended = waitForStatusEq(tsEndOfMain, false, timeoutForce);
//      if(!ended){
//        threadCancel();
//        return;
//      }
//    }
  }
  thread->join();
  thread.reset();
}

void Thread::threadCancel() {
  event.stopListening();
  event.setStatus(tsToClose);
  if(!thread) return;
#ifndef RAI_MSVC
  int rc;
  rc = pthread_cancel(thread->native_handle());         if(rc) HALT("pthread_cancel failed with err " <<rc <<" '" <<strerror(rc) <<"'");
#endif
  thread->join();
  thread.reset();
  stepMutex.state=-1; //forced destroy in the destructor
}

void Thread::threadStep() {
  threadOpen();
  event.setStatus(tsToStep);
}

bool Thread::isIdle() {
  return event.getStatus()==tsIDLE;
}

bool Thread::isClosed() {
  return !thread; //getStatus()==tsIsClosed;
}

void Thread::waitForOpened() {
  event.waitForStatusNotEq(tsIsClosed);
  event.waitForStatusNotEq(tsToOpen);
}

void Thread::waitForIdle() {
  event.waitForStatusEq(tsIDLE);
}

void Thread::threadLoop(bool waitForOpened) {
  threadOpen(waitForOpened);
  if(metronome.ticInterval>1e-10) {
    event.setStatus(tsBEATING);
  } else {
    event.setStatus(tsLOOPING);
  }
}

void Thread::threadStop(bool wait) {
  if(thread) {
    event.setStatus(tsIDLE);
    if(wait) waitForIdle();
  }
}

void Thread::main() {
  tid = getpid();
//  if(verbose>0) cout <<"*** Entering Thread '" <<name <<"'" <<endl;
  //http://linux.die.net/man/3/setpriority
  //if(Thread::threadPriority) setRRscheduling(Thread::threadPriority);
  //if(Thread::threadPriority) setNice(Thread::threadPriority);

  {
    auto mux = stepMutex(RAI_HERE);
    try {
      open(); //virtual open routine
    } catch(const std::exception& ex) {
      event.setStatus(tsFAILURE);
      cerr <<"*** open() of Thread'" <<name <<"'failed: " <<ex.what() <<" -- closing it again" <<endl;
    } catch(...) {
      event.setStatus(tsFAILURE);
      cerr <<"*** open() of Thread '" <<name <<"' failed! -- closing it again";
      return;
    }
  }

  event.statusLock();
  if(event.status==tsToOpen) {
    event.status=tsIDLE;
    event.broadcast();
  }
  //if not =tsOPENING anymore -> the state was set on looping or beating already
  event.statusUnlock();

  timer.reset();
  for(;;) {
    //-- wait for a non-idle state
    int s = event.waitForStatusNotEq(tsIDLE);
    if(s==tsToClose) break;
    if(s==tsBEATING) metronome.waitForTic();
    if(s>0) event.setStatus(tsIDLE); //step command -> reset to idle

    //-- make a step
    timer.cycleStart();
    stepMutex.lock(RAI_HERE);
    step(); //virtual step routine
    stepMutex.unlock();
    step_count++;
    timer.cycleDone();
  };

  stepMutex.lock(RAI_HERE);
  close(); //virtual close routine
  stepMutex.unlock();
//  if(verbose>0) cout <<"*** Exiting Thread '" <<name <<"'" <<endl;

  event.setStatus(tsIsClosed);
}

//===========================================================================
//
// controlling threads
//

Signaler _moduleShutdown;
Signaler* moduleShutdown() { return &_moduleShutdown; }

void signalhandler(int s) {
  int calls = moduleShutdown()->incrementStatus();
  cerr <<"\n*** System received signal " <<s <<" -- count=" <<calls <<endl;
  if(calls==1) {
    LOG(0) <<" -- waiting for main loop to break on moduleShutdown()->getStatus()" <<endl;
  }
  if(calls==2) {
    LOG(0) <<" -- smoothly closing modules directly" <<endl;
//    threadCloseModules(); //might lead to a hangup of the main loop, but processes should close
    LOG(0) <<" -- DONE" <<endl;
  }
  if(calls==3) {
    LOG(0) <<" -- cancelling threads to force closing" <<endl;
//    threadCancelModules();
    LOG(0) <<" -- DONE" <<endl;
  }
  if(calls>3) {
    LOG(3) <<" ** moduleShutdown failed - hard exit!" <<endl;
    exit(1);
  }
}

//===========================================================================
//
// Utils
//

int _allPositive(const VarL& signalers, int whoChanged) {
  bool allPositive=true;
  for(Var_base* s:signalers) {
    Var_data<ActStatus>* a = dynamic_cast<Var_data<ActStatus>*>(s);
    CHECK(a, "this is not an ActStatus!!");
    if(a->rwlock.isLocked() && a->data<=0) allPositive=false;
    if(!a->rwlock.isLocked() && a->data<=0) allPositive=false;
  }
  if(allPositive) return AS_true;
  return AS_false;
}

RUN_ON_INIT_BEGIN(thread)
rai::Array<ptr<Var_base>*>::memMove=true;
ThreadL::memMove=true;
SignalerL::memMove=true;
RUN_ON_INIT_END(thread)
