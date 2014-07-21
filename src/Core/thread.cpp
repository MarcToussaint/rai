/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

#include "thread.h"
#include <exception>

#ifndef MT_MSVC
#ifndef __CYGWIN__
#  include <sys/syscall.h>
#else
#  include "cygwin_compat.h"
#endif //__CYGWIN __
#  include <unistd.h>
#endif
#ifdef MT_QThread
#  include <QtCore/QThread>
#endif
#include <errno.h>


#ifndef MT_MSVC

//===========================================================================
//
// Access RWLock
//

RWLock::RWLock() {
  int rc = pthread_rwlock_init(&lock, NULL);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  state=0;
}

RWLock::~RWLock() {
  CHECK(!state, "Destroying locked RWLock");
  int rc = pthread_rwlock_destroy(&lock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void RWLock::readLock() {
  int rc = pthread_rwlock_rdlock(&lock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  stateMutex.lock();
  state++;
  stateMutex.unlock();
}

void RWLock::writeLock() {
  int rc = pthread_rwlock_wrlock(&lock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  stateMutex.lock();
  state=-1;
  stateMutex.unlock();
}

void RWLock::unlock() {
  stateMutex.lock();
  if(state>0) state--; else state=0;
  stateMutex.unlock();
  int rc = pthread_rwlock_unlock(&lock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}


//===========================================================================
//
// ConditionVariable
//

ConditionVariable::ConditionVariable(int initialValue) {
  int rc = pthread_cond_init(&cond, NULL);    if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  value=initialValue;
}

ConditionVariable::~ConditionVariable() {
  int rc = pthread_cond_destroy(&cond);    if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void ConditionVariable::setValue(int i, bool signalOnlyFirstInQueue) {
  mutex.lock();
  value=i;
  broadcast(signalOnlyFirstInQueue);
  mutex.unlock();
}

int ConditionVariable::incrementValue(bool signalOnlyFirstInQueue) {
  mutex.lock();
  value++;
  broadcast(signalOnlyFirstInQueue);
  int i=value;
  mutex.unlock();
  return i;
}

void ConditionVariable::broadcast(bool signalOnlyFirstInQueue) {
  if(!signalOnlyFirstInQueue) {
    int rc = pthread_cond_signal(&cond);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  } else {
    int rc = pthread_cond_broadcast(&cond);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  }
}

void ConditionVariable::lock() {
  mutex.lock();
}

void ConditionVariable::unlock() {
  mutex.unlock();
}

int ConditionVariable::getValue(bool userHasLocked) const {
  Mutex *m = (Mutex*)&mutex; //sorry: to allow for 'const' access
  if(!userHasLocked) m->lock(); else CHECK(m->state==syscall(SYS_gettid),"user must have locked before calling this!");
  int i=value;
  if(!userHasLocked) m->unlock();
  return i;
}

void ConditionVariable::waitForSignal(bool userHasLocked) {
  if(!userHasLocked) mutex.lock(); else CHECK(mutex.state==syscall(SYS_gettid),"user must have locked before calling this!");
  int rc = pthread_cond_wait(&cond, &mutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  if(!userHasLocked) mutex.unlock();
}

void ConditionVariable::waitForSignal(double seconds, bool userHasLocked) {
  struct timespec timeout;
  clock_gettime(CLOCK_MONOTONIC, &timeout);
  long secs = (long)(floor(seconds));
  timeout.tv_sec  += secs;
  timeout.tv_nsec += (long)(floor(1000000000. * (seconds-(double)secs)));
  if(timeout.tv_nsec>1000000000l) {
    timeout.tv_sec+=1;
    timeout.tv_nsec-=1000000000l;
  }

  if(!userHasLocked) mutex.lock(); else CHECK(mutex.state==syscall(SYS_gettid),"user must have locked before calling this!");
  int rc = pthread_cond_timedwait(&cond, &mutex.mutex, &timeout);
  if(rc && rc!=ETIMEDOUT) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  if(!userHasLocked) mutex.unlock();
}

void ConditionVariable::waitForValueEq(int i, bool userHasLocked) {
  if(!userHasLocked) mutex.lock(); else CHECK(mutex.state==syscall(SYS_gettid),"user must have locked before calling this!");
  while(value!=i) {
    int rc = pthread_cond_wait(&cond, &mutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  }
  if(!userHasLocked) mutex.unlock();
}

void ConditionVariable::waitForValueNotEq(int i, bool userHasLocked) {
  if(!userHasLocked) mutex.lock(); else CHECK(mutex.state==syscall(SYS_gettid),"user must have locked before calling this!");
  while(value==i) {
    int rc = pthread_cond_wait(&cond, &mutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  }
  if(!userHasLocked) mutex.unlock();
}

void ConditionVariable::waitForValueGreaterThan(int i, bool userHasLocked) {
  if(!userHasLocked) mutex.lock(); else CHECK(mutex.state==syscall(SYS_gettid),"user must have locked before calling this!");
  while(value<=i) {
    int rc = pthread_cond_wait(&cond, &mutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  }
  if(!userHasLocked) mutex.unlock();
}

void ConditionVariable::waitForValueSmallerThan(int i, bool userHasLocked) {
  if(!userHasLocked) mutex.lock(); else CHECK(mutex.state==syscall(SYS_gettid),"user must have locked before calling this!");
  while(value>=i) {
    int rc = pthread_cond_wait(&cond, &mutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  }
  if(!userHasLocked) mutex.unlock();
}


void ConditionVariable::waitUntil(double absTime, bool userHasLocked) {
  NIY;
  /*  int rc;
    timespec ts;
    ts.tv_sec  = tp.tv_sec;
    ts.tv_nsec = tp.tv_usec * 1000;
    ts.tv_sec += WAIT_TIME_SECONDS;

    rc = pthread_mutex_lock(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
    rc = pthread_cond_timedwait(&cond, &mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
    rc = pthread_mutex_unlock(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
    */
}


//===========================================================================
//
// Metronome
//

Metronome::Metronome(const char* _name, double ticIntervalSec) {
  name=_name;
  reset(ticIntervalSec);
}

Metronome::~Metronome() {
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
  int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ticTime, NULL);
  if(rc && errno) MT_MSG("clock_nanosleep() failed " <<rc <<" errno=" <<errno <<' ' <<strerror(errno));

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


//===========================================================================
//
// Thread
//

void* Thread_staticMain(void *_self) {
  Thread *th=(Thread*)_self;
  th->main();
  return NULL;
}

#ifdef MT_QThread
class sThread:QThread {
  Q_OBJECT
public:
  Thread *th;
  sThread(Thread *_th, const char* name):th(_th){ setObjectName(name); }
  ~sThread(){}
  void open(){ start(); }
  void close(){ wait(); }
protected:
  void run(){ th->main();  }
};
#endif

Thread::Thread(const char* _name): name(_name), state(tsCLOSE), tid(0), thread(0), step_count(0), metronome(NULL)  {
  listensTo.memMove=true;
}

Thread::~Thread() {
  if(!isClosed()) threadClose();
}

void Thread::threadOpen(int priority) {
  state.lock();
  if(!isClosed()){ state.unlock(); return; } //this is already open -- or has just beend opened (parallel call to threadOpen)
#ifndef MT_QThread
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
  state.value=tsOPENING;
  state.unlock();
}

void Thread::threadClose() {
  state.setValue(tsCLOSE);
  if(!thread) return;
#ifndef MT_QThread
  int rc;
  rc = pthread_join(thread, NULL);     if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  thread=0;
#else
  thread->close();
  delete thread;
  thread=NULL;
#endif
}

void Thread::threadCancel() {
  if(thread){
#ifndef MT_QThread
    int rc = pthread_cancel(thread);     if(rc) HALT("pthread_cancel failed with err " <<rc <<" '" <<strerror(rc) <<"'");
    rc = pthread_join(thread, NULL);     if(rc) HALT("pthread_join failed with err " <<rc <<" '" <<strerror(rc) <<"'");
#else
    NIY;
#endif
  }
  thread=0;
}

void Thread::threadStep(uint steps, bool wait) {
  if(isClosed()) threadOpen();
  //CHECK(state.value==tsIDLE, "never step while thread is busy!");
  state.setValue(steps);
  if(wait) waitForIdle();
}

void Thread::listenTo(const ConditionVariableL &signalingVars) {
  for_list(ConditionVariable,  v,  signalingVars) listenTo(*v);
}

void Thread::listenTo(ConditionVariable& v) {
  v.mutex.lock(); //don't want to increase revision and broadcast!
  v.listeners.setAppend(this);
  v.mutex.unlock();
  listensTo.setAppend(&v);
}

void Thread::stopListeningTo(ConditionVariable& v){
  v.mutex.lock(); //don't want to increase revision and broadcast!
  v.listeners.removeValue(this);
  v.mutex.unlock();
  listensTo.removeValue(&v);
}

bool Thread::isIdle() {
  return state.getValue()==tsIDLE;
}

//bool Thread::isOpen() {
//  return thread!=0;
//}

bool Thread::isClosed() {
  return state.getValue()==tsCLOSE;
}

void Thread::waitForIdle() {
  state.waitForValueEq(tsIDLE);
}

void Thread::threadLoop() {
  if(isClosed()) threadOpen();
  state.setValue(tsLOOPING);
}

void Thread::threadLoopWithBeat(double sec) {
  if(!metronome)
    metronome=new Metronome("threadTiccer", sec);
  else
    metronome->reset(sec);
  if(isClosed()) threadOpen();
  state.setValue(tsBEATING);
}

void Thread::threadStop() {
  CHECK(!isClosed(), "called stop to closed thread");
  state.setValue(tsIDLE);
}

void Thread::main() {
  tid = syscall(SYS_gettid);
  cout <<"*** Entering Thread '" <<name <<"'" <<endl;
  //http://linux.die.net/man/3/setpriority
  //if(Thread::threadPriority) setRRscheduling(Thread::threadPriority);
  //if(Thread::threadPriority) setNice(Thread::threadPriority);

  try{
    open(); //virtual open routine
  } catch(const std::exception& ex) {
    state.setValue(tsFAILURE);
    cerr << "*** open() of Thread'" << name << "'failed: " << ex.what() << " -- closing it again" << endl;        
  } catch(...) {
    state.setValue(tsFAILURE);
    cerr <<"*** open() of Thread '" <<name <<"' failed! -- closing it again";
    return;
  }

  state.lock();
  if(state.value==tsOPENING){
    state.value=tsIDLE;
    state.broadcast();
  }
  //if not =tsOPENING anymore -> the state was set on looping or beating already
  state.unlock();


  //timer.reset();
  bool waitForTic=false;
  for(;;){
    //-- wait for a non-idle state
    state.lock();
    state.waitForValueNotEq(tsIDLE, true);
    if(state.value==tsCLOSE) { state.unlock();  break; }
    if(state.value==tsBEATING) waitForTic=true; else waitForTic=false;
    if(state.value>0) state.value--; //count down
    state.unlock();

    if(waitForTic) metronome->waitForTic();

    //-- make a step
    //engine().acc->logStepBegin(module);
    step(); //virtual step routine
    step_count++;
    //engine().acc->logStepEnd(module);

    //-- broadcast in case somebody was waiting for a finished step
    state.lock();
    state.broadcast();
    state.unlock();
  };

  close(); //virtual close routine
  cout <<"*** Exiting Thread '" <<name <<"'" <<endl;
}


//===========================================================================
//
// Utils
//

void stop(const ThreadL& P) {
  for_list(Thread,  p,  P) p->threadStop();
}

void wait(const ThreadL& P) {
  for_list(Thread,  p,  P) p->waitForIdle();
}

void close(const ThreadL& P) {
  for_list(Thread,  p,  P) p->threadClose();
}

//===========================================================================
//
// TStream class, for concurrent access to ostreams
//

TStream::TStream(std::ostream &o):out(o) { }

TStream::Access TStream::operator()(const void *obj) {
  return Access(this, obj);
}

TStream::Register TStream::reg(const void *obj) {
  return Register(this, obj);
}

bool TStream::get(const void *obj, char **head) {
  return get_private(obj, head, true);
}

bool TStream::get_private(const void *obj, char **head, bool l) {
  if(l) lock.readLock();
  bool ret = map.count(obj) == 1;
  if(head) *head = ret? (char*)map[obj]: NULL;
  if(l) lock.unlock();
  return ret;
}

void TStream::reg_private(const void *obj, char *p, bool l) {
  if(l) lock.writeLock();
  unreg_private(obj, false);
  map[obj] = p;
  if(l) lock.unlock();
}

void TStream::unreg(const void *obj) {
  unreg_private(obj, true);
}

void TStream::unreg_private(const void *obj, bool l) {
  if(l) lock.writeLock();
  if(get_private(obj, NULL, false)) {
    delete map[obj];
    map.erase(obj);
  }
  if(l) lock.unlock();
}

void TStream::unreg_all() {
  lock.writeLock();
  for(auto it = map.begin(); it != map.end(); )
    unreg_private((it++)->first, false); // always increment before actual deletion
  lock.unlock();
}

TStream::Access::Access(TStream *ts, const void *o):tstream(ts), obj(o) { }
TStream::Access::Access(const Access &a):tstream(a.tstream) { }
TStream::Access::~Access() {
  tstream->mutex.lock();
  char *head;
  tstream->lock.readLock();
  if(tstream->get_private(obj, &head, false))
    tstream->out << head;
  tstream->lock.unlock();
  tstream->out << stream.str();
  tstream->mutex.unlock();
}

TStream::Register::Register(TStream *ts, const void *o):tstream(ts), obj(o) {}
TStream::Register::Register(const Register &r):tstream(r.tstream) {}
TStream::Register::~Register() {
  const char *cstr = stream.str().c_str();
  size_t cstrl = strlen(cstr);
  char *p = new char[cstrl+1];
  memcpy(p, cstr, cstrl+1);

  tstream->reg_private(obj, p, true);
}

#endif //MT_MSVC
