/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

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
#endif
#include <errno.h>

#ifndef RAI_MSVC

//===========================================================================
//
// Access RWLock
//

RWLock::RWLock() {
  int rc = pthread_rwlock_init(&rwLock, nullptr);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rwCount=0;
}

RWLock::~RWLock() {
  CHECK(!rwCount, "Destroying locked RWLock");
  int rc = pthread_rwlock_destroy(&rwLock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void RWLock::readLock() {
  int rc = pthread_rwlock_rdlock(&rwLock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rwCountMutex.lock(RAI_HERE);
  rwCount++;
  rwCountMutex.unlock();
}

void RWLock::writeLock() {
  int rc = pthread_rwlock_wrlock(&rwLock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rwCountMutex.lock(RAI_HERE);
  rwCount=-1;
  rwCountMutex.unlock();
}

void RWLock::unlock() {
  rwCountMutex.lock(RAI_HERE);
  if(rwCount>0) rwCount--; else rwCount=0;
  rwCountMutex.unlock();
  int rc = pthread_rwlock_unlock(&rwLock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
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
  int rc = pthread_cond_init(&cond, nullptr);    if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

Signaler::~Signaler() {
  int rc = pthread_cond_destroy(&cond);    if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void Signaler::setStatus(int i, Signaler* messenger) {
  statusMutex.lock(RAI_HERE);
  status=i;
  broadcast(messenger);
  statusMutex.unlock();
}

int Signaler::incrementStatus(Signaler* messenger) {
  statusMutex.lock(RAI_HERE);
  status++;
  broadcast(messenger);
  int i=status;
  statusMutex.unlock();
  return i;
}

void Signaler::broadcast(Signaler* messenger) {
  int rc = pthread_cond_signal(&cond);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
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
    auto lock = statusMutex(RAI_HERE);
//    if(this->status!=newEventStatus)
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

int Signaler::getStatus(bool userHasLocked) const {
  Mutex* m = (Mutex*)&statusMutex; //sorry: to allow for 'const' access
  if(!userHasLocked) m->lock(RAI_HERE); else CHECK_EQ(m->state, syscall(SYS_gettid), "user must have locked before calling this!");
  int i=status;
  if(!userHasLocked) m->unlock();
  return i;
}

bool Signaler::waitForSignal(bool userHasLocked, double timeout) {
  if(!userHasLocked) statusMutex.lock(RAI_HERE); // else CHECK_EQ(mutex.state, syscall(SYS_gettid), "user must have locked before calling this!");
  if(timeout<0.) {
    int rc = pthread_cond_wait(&cond, &statusMutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  } else {
    struct timespec ts_timeout;
    clock_gettime(CLOCK_REALTIME, &ts_timeout); //CLOCK_MONOTONIC, &timeout);
    long secs = (long)(floor(timeout));
    timeout -= secs;
    ts_timeout.tv_sec  += secs;
    ts_timeout.tv_nsec += (long)(floor(1e9 * timeout));
    if(ts_timeout.tv_nsec>1000000000l) {
      ts_timeout.tv_sec+=1;
      ts_timeout.tv_nsec-=1000000000l;
    }

    int rc = pthread_cond_timedwait(&cond, &statusMutex.mutex, &ts_timeout);
    if(rc && rc!=ETIMEDOUT) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
    if(rc==ETIMEDOUT) {
      if(!userHasLocked) statusMutex.unlock();
      return false;
    }
  }
  if(!userHasLocked) statusMutex.unlock();
  return true;
}

bool Signaler::waitForEvent(std::function<bool()> f, bool userHasLocked) {
  if(!userHasLocked) statusMutex.lock(RAI_HERE); else CHECK_EQ(statusMutex.state, syscall(SYS_gettid), "user must have locked before calling this!");
  while(!f()) {
    int rc = pthread_cond_wait(&cond, &statusMutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  }
  if(!userHasLocked) statusMutex.unlock();
  return true;

}

bool Signaler::waitForStatusEq(int i, bool userHasLocked, double timeout) {
  if(!userHasLocked) statusMutex.lock(RAI_HERE); else CHECK_EQ(statusMutex.state, syscall(SYS_gettid), "user must have locked before calling this!");
  while(status!=i) {
    bool succ = waitForSignal(true, timeout);
    if(!succ) { if(!userHasLocked) statusMutex.unlock();  return false; }
  }
  if(!userHasLocked) statusMutex.unlock();
  return true;
}

int Signaler::waitForStatusNotEq(int i, bool userHasLocked, double timeout) {
  if(!userHasLocked) statusMutex.lock(RAI_HERE); else CHECK_EQ(statusMutex.state, syscall(SYS_gettid), "user must have locked before calling this!");
  while(status==i) {
    bool succ = waitForSignal(true, timeout);
    if(!succ) { if(!userHasLocked) statusMutex.unlock();  return false; }
  }
  int _status=status;
  if(!userHasLocked) statusMutex.unlock();
  return _status;
}

int Signaler::waitForStatusGreaterThan(int i, bool userHasLocked, double timeout) {
  if(!userHasLocked) statusMutex.lock(RAI_HERE); else CHECK_EQ(statusMutex.state, syscall(SYS_gettid), "user must have locked before calling this!");
  while(status<=i) {
    bool succ = waitForSignal(true, timeout);
    if(!succ) { if(!userHasLocked) statusMutex.unlock();  return false; }
  }
  int _status=status;
  if(!userHasLocked) statusMutex.unlock();
  return _status;
}

int Signaler::waitForStatusSmallerThan(int i, bool userHasLocked, double timeout) {
  if(!userHasLocked) statusMutex.lock(RAI_HERE); else CHECK_EQ(statusMutex.state, syscall(SYS_gettid), "user must have locked before calling this!");
  while(status>=i) {
    bool succ = waitForSignal(true, timeout);
    if(!succ) { if(!userHasLocked) statusMutex.unlock();  return false; }
  }
  int _status=status;
  if(!userHasLocked) statusMutex.unlock();
  return _status;
}

//===========================================================================
//
// VariableBase
//

Var_base::Var_base(const std::type_info& _type, const char* _name) : name(_name) {
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
    for(auto* c:callbacks) {
      //don't call a callback-event for a thread that accessed the variable:
      if(!th || c->id!=&th->event) c->call()(this);
    }
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
  clock_gettime(CLOCK_MONOTONIC, &ticTime);
  tics=0;
  ticInterval = ticIntervalSec;
}

void Metronome::waitForTic() {
  //compute target time
  long secs = (long)(floor(ticInterval));
  ticTime.tv_sec  += secs;
  ticTime.tv_nsec += (long)(floor(1000000000. * (ticInterval-(double)secs)));
  while(ticTime.tv_nsec>1000000000l) {
    ticTime.tv_sec  += 1;
    ticTime.tv_nsec -= 1000000000l;
  }
  //wait for target time
  int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ticTime, nullptr);
  if(rc && errno) RAI_MSG("clock_nanosleep() failed " <<rc <<" errno=" <<errno <<' ' <<strerror(errno));

  tics++;
}

double Metronome::getTimeSinceTic() {
  timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return double(now.tv_sec-ticTime.tv_sec) + 1e-9*(now.tv_nsec-ticTime.tv_nsec);
}

//===========================================================================
//
// CycleTimer
//

void updateTimeIndicators(double& dt, double& dtMean, double& dtMax, const timespec& now, const timespec& last, uint step) {
  dt=double(now.tv_sec-last.tv_sec-1)*1000. +
     double(1000000000l+now.tv_nsec-last.tv_nsec)/1000000.;
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
  clock_gettime(CLOCK_MONOTONIC, &lastTime);
}

void CycleTimer::cycleStart() {
  clock_gettime(CLOCK_MONOTONIC, &now);
  updateTimeIndicators(cyclDt, cyclDtMean, cyclDtMax, now, lastTime, steps);
  lastTime=now;
}

void CycleTimer::cycleDone() {
  clock_gettime(CLOCK_MONOTONIC, &now);
  updateTimeIndicators(busyDt, busyDtMean, busyDtMax, now, lastTime, steps);
  steps++;
}

rai::String CycleTimer::report() {
  rai::String s;
  s.printf("busy=[%5.1f %5.1f] cycle=[%5.1f %5.1f] load=%4.1f%% steps=%i", busyDtMean, busyDtMax, cyclDtMean, cyclDtMax, 100.*busyDtMean/cyclDtMean, steps);
  return s;
}

//===========================================================================
//
// MiniThread
//

void* MiniThread_staticMain(void* _self) {
  MiniThread* th=(MiniThread*)_self;
  th->pthreadMain();
  return nullptr;
}

MiniThread::MiniThread(const char* _name) : Signaler(tsIsClosed), name(_name) {
  if(name.N>14) name.resize(14, true);

  statusLock();

  int rc;
  pthread_attr_t atts;
  rc = pthread_attr_init(&atts); if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rc = pthread_create(&thread, &atts, MiniThread_staticMain, this);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  if(name) pthread_setname_np(thread, name);

  status=0;
  statusUnlock();
}

MiniThread::~MiniThread() {
  if(thread)
    HALT("Call 'threadClose()' in the destructor of the DERIVED class! \
           That's because the 'virtual table is destroyed' before calling the destructor ~Thread (google 'call virtual function\
           in destructor') but now the destructor has to call 'threadClose' which triggers a Thread::close(), which is\
           pure virtual while you're trying to call ~Thread.")
  }

void MiniThread::threadClose(double timeoutForce) {
//  stopListening();
  setStatus(tsToClose);
  if(!thread) { setStatus(tsIsClosed); return; }
  for(;;) {
    bool ended = waitForStatusEq(tsIsClosed, false, .2);
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
  int rc;
  rc = pthread_join(thread, nullptr);     if(rc) HALT("pthread_join failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  thread=0;
}

void MiniThread::threadCancel() {
//  stopListening();
  setStatus(tsToClose);
  if(!thread) { setStatus(tsIsClosed); return; }
  int rc;
  rc = pthread_cancel(thread);         if(rc) HALT("pthread_cancel failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rc = pthread_join(thread, nullptr);     if(rc) HALT("pthread_join failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  thread=0;
}

void MiniThread::pthreadMain() {
  tid = syscall(SYS_gettid);
//  if(verbose>0) cout <<"*** Entering Thread '" <<name <<"'" <<endl;
  //http://linux.die.net/man/3/setpriority
  //if(Thread::threadPriority) setRRscheduling(Thread::threadPriority);
  //if(Thread::threadPriority) setNice(Thread::threadPriority);

  setStatus(1);

  try {
    main();
  } catch(const std::exception& ex) {
    setStatus(tsFAILURE);
    cerr <<"*** main() of Thread'" <<name <<"'failed: " <<ex.what() <<" -- closing it again" <<endl;
  } catch(const char* ex) {
    setStatus(tsFAILURE);
    cerr <<"*** main() of Thread'" <<name <<"'failed: " <<ex <<" -- closing it again" <<endl;
  } catch(...) {
    setStatus(tsFAILURE);
    cerr <<"*** main() of Thread '" <<name <<"' failed! -- closing it again";
  }

  setStatus(tsIsClosed);
}

//=============================================
//
// Thread
//

void* Thread_staticMain(void* _self) {
  Thread* th=(Thread*)_self;
  th->main();
  return nullptr;
}

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
    thread(0),
    tid(0),
    step_count(0),
    metronome(beatIntervalSec) {
  if(name.N>14) name.resize(14, true);
}

Thread::~Thread() {
  if(thread)
    HALT("Call 'threadClose()' in the destructor of the DERIVED class! \
           That's because the 'virtual table is destroyed' before calling the destructor ~Thread (google 'call virtual function\
           in destructor') but now the destructor has to call 'threadClose' which triggers a Thread::close(), which is\
           pure virtual while you're trying to call ~Thread.")
  }

void Thread::threadOpen(bool wait, int priority) {
  {
    auto lock = event.statusMutex(RAI_HERE);
    if(thread) return; //this is already open -- or has just beend opened (parallel call to threadOpen)
#ifndef RAI_QThread
    int rc;
    pthread_attr_t atts;
    rc = pthread_attr_init(&atts); if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
    rc = pthread_create(&thread, &atts, Thread_staticMain, this);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
    /*if(priority){ //doesn't work - but setpriority does work!!
    rc = pthread_attr_setschedpolicy(&atts, SCHED_RR);  if(rc) HALT("pthread failed with err " <<rc <<strerror(rc));
    sched_param  param;
    rc = pthread_attr_getschedparam(&atts, &param);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
    std::cout <<"standard priority = " <<param.sched_priority <<std::endl;
    param.sched_priority += priority;
    std::cout <<"modified priority = " <<param.sched_priority <<std::endl;
    rc = pthread_attr_setschedparam(&atts, &param);  if(rc) HALT("pthread failed with err " <<rc <<strerror(rc));
    }*/
    //prctl(PR_SET_NAME, proc->name.p);
    if(name) pthread_setname_np(thread, name);
#else
    thread = new sThread(this, "hallo");
    thread->open();
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
    bool ended = event.waitForStatusEq(tsIsClosed, false, .2);
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
#ifndef RAI_QThread
  int rc;
  rc = pthread_join(thread, nullptr);     if(rc) HALT("pthread_join failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  thread=0;
#else
  thread->close();
  delete thread;
  thread=nullptr;
#endif
}

void Thread::threadCancel() {
  event.stopListening();
  event.setStatus(tsToClose);
  if(!thread) return;
#ifndef RAI_QThread
  int rc;
  rc = pthread_cancel(thread);         if(rc) HALT("pthread_cancel failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rc = pthread_join(thread, nullptr);     if(rc) HALT("pthread_join failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  thread=0;
#else
  NIY;
#endif
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
  tid = syscall(SYS_gettid);
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

#endif //RAI_MSVC
