#include "thread.h"

#ifndef MT_MSVC
#  include <sys/syscall.h>
#endif
#ifdef MT_QT
#  include <QtCore/QThread>
#endif
#include <errno.h>


#ifndef MT_MSVC

//===========================================================================
//
// Mutex
//

#define MUTEX_DUMP(x) //x

Mutex::Mutex() {
  pthread_mutexattr_t atts;
  int rc;
  rc = pthread_mutexattr_init(&atts);  if(rc) HALT("pthread failed with err " <<rc <<strerror(rc));
  rc = pthread_mutexattr_settype(&atts, PTHREAD_MUTEX_RECURSIVE_NP);  if(rc) HALT("pthread failed with err " <<rc <<strerror(rc));
  rc = pthread_mutex_init(&mutex, &atts);
  //mutex = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
  state=0;
  recursive=0;
}

Mutex::~Mutex() {
  CHECK(!state, "Mutex destroyed without unlocking first");
  int rc = pthread_mutex_destroy(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void Mutex::lock() {
  int rc = pthread_mutex_lock(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  recursive++;
  state=syscall(SYS_gettid);
  MUTEX_DUMP(cout <<"Mutex-lock: " <<state <<" (rec: " <<recursive << ")" <<endl);
}

void Mutex::unlock() {
  MUTEX_DUMP(cout <<"Mutex-unlock: " <<state <<" (rec: " <<recursive << ")" <<endl);
  if(--recursive == 0)
    state=0;
  int rc = pthread_mutex_unlock(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}


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

#endif //MT_MSVC

//===========================================================================
//
// Thread
//

void* Thread_staticMain(void *_self) {
  Thread *th=(Thread*)_self;
  th->main();
  return NULL;
}

#ifdef MT_QT
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
#ifndef MT_QT
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
#ifndef MT_QT
  int rc;
  rc = pthread_join(thread, NULL);     if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  thread=0;
#else
  thread->close();
  delete thread;
  thread=NULL;
#endif
}

void Thread::threadStep(uint steps, bool wait) {
  if(isClosed()) threadOpen();
  //CHECK(state.value==tsIDLE, "never step while thread is busy!");
  state.setValue(steps);
  if(wait) waitForIdle();
}

void Thread::listenTo(const ConditionVariableL &signalingVars) {
  uint i;  ConditionVariable *v;
  for_list(i, v, signalingVars) listenTo(*v);
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

  //http://linux.die.net/man/3/setpriority
  //if(Thread::threadPriority) setRRscheduling(Thread::threadPriority);
  //if(Thread::threadPriority) setNice(Thread::threadPriority);

  open(); //virtual open routine

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
}


//===========================================================================
//
// Utils
//

void stop(const ThreadL& P) {
  Thread *p; uint i;
  for_list(i, p, P) p->threadStop();
}

void wait(const ThreadL& P) {
  Thread *p; uint i;
  for_list(i, p, P) p->waitForIdle();
}

void close(const ThreadL& P) {
  Thread *p; uint i;
  for_list(i, p, P) p->threadClose();
}

//===========================================================================
//
// throut Utilities
//

#include <map>

/** @namespace throut
 *  @brief throut (threaded-out); Concurrent access to console output.
 *
 * In multi-threaded environments, concurrent accesses to the console output may
 * overlap producing unreadable text. Throut solves this.
 *
 * Instead of using
 * @code
 * cout << "some message" << endl;
 * cout << "value = " << v << endl;
 * @endcode
 * 
 * use
 * @code
 * throut::throut("some message");
 * throut::throut(STRING("value = " << v));
 * @endcode
 *
 * You can "register" a heading with an object pointer, so that referring to
 * that same pointer will always pre-pend the message with that heading. This is
 * useful to identify the object issuing the message.
 *
 * @code
 * MyObject *p = new MyObject();
 * throut::throutRegHeading(p, "Object: "); // registers a heading to p
 * // some code
 * throut::throut(p, "message!"); // prints "Object: message!"
 * // some code
 * throut::throutUnregHeading(p); // unregisters heading related with p
 * @endcode
 *
 * It is more appropriate to register a heading within the constructor of the
 * object which will issue the messages, so as to simply use the key-word
 * 'this'. In this case, unregistering should be handled in the deconstructor.
 *
 * @code
 * using namespace throut;
 * class MyObject {
 *   int ID;
 *   MyObject(int id): ID(id) {
 *     throutRegHeading(this, STRING("Object(" << ID << "): "));
 *   }
 *
 *   ~MyObject() {
 *     throutUnregHeading(this);
 *   }
 *
 *   void process() {
 *     // some code
 *     // outputs "Object(17): message!" (assuming ID = 17);
 *     throut(this, "message!");
 *     // some code
 *   }
 * };
 * @endcode
 * */
namespace throut {
  /**@brief "private" variable, not included in header.
   *
   * - DO NOT USE
   */
  RWLock throutRWLock;
  /**@brief "private" variable, not included in header.
   *
   * - DO NOT USE
   */
  Mutex msgMutex;
  /**@brief "private" variable, not included in header.
   *
   * - DO NOT USE
   */
  std::map<const void*, const char*> throutMap;

  /**@brief "private" method, not included in header.
   *
   * - DO NOT USE
   */
  bool throutContains_private(const void *obj) {
    return throutMap.count(obj) == 1;
  }

  /**@brief "private" method, not included in header.
   *
   * - DO NOT USE
   */
  bool throutGetHeading_private(const void *obj, char **head) {
    bool r = throutContains_private(obj);
    if(head != NULL)
      *head = r? (char*)throutMap[obj]: NULL;
    return r;
  }

  /**@brief "private" method, not included in header.
   *
   * - DO NOT USE
   */
  void throutUnregHeading_private(const void *obj) {
    if(throutContains_private(obj)) {
      delete throutMap[obj];
      throutMap.erase(obj);
    }
  }

  /**@brief registers an object's heading
   *
   * - It is possible to register a heading to the NULL pointer.
   * - Registering a new heading to an already registered object will simply
   * over-write the older heading.
   */
  void throutRegHeading(const void *obj, const MT::String &head) {
    size_t ml = head.N;
    char *p = new char[ml+1];
    memcpy(p, (const char*)head, ml+1);

    throutRWLock.writeLock();
    throutUnregHeading_private(obj);
    throutMap[obj] = p;
    throutRWLock.unlock();
  }

  /**@brief registers an object's heading
   *
   * - It is possible to register a heading to the NULL pointer.
   * - Registering a new heading to an already registered object will simply
   * over-write the older heading.
   */
  void throutRegHeading(const void *obj, const char *head) {
    size_t ml = strlen(head);
    char *p = new char[ml+1];
    memcpy(p, head, ml+1);

    throutRWLock.writeLock();
    throutUnregHeading_private(obj);
    throutMap[obj] = p;
    throutRWLock.unlock();
  }

  /**@brief unregisters an object's heading
   *
   * - Memory leaks will occur if a heading is not unregistered
   */
  void throutUnregHeading(const void *obj) {
    throutRWLock.writeLock();
    throutUnregHeading_private(obj);
    throutRWLock.unlock();
  }

  /**@brief unregisters all headings
   *
   * - Useful to avoid all memory leaks when throut methods are no longer
   *   needed.
   * - Ideally, unregistering should be handled by specific calls to
   *   throut::throutUnregHeading().
   */
  void throutUnregAll() {
    std::map<const void*, const char*>::iterator it;
    throutRWLock.writeLock();
    for(it = throutMap.begin(); it != throutMap.end(); ) {
      throut(STRING("Deleting " << it->first << " " << it->second));
      throutUnregHeading_private((it++)->first);
    }
    throutRWLock.unlock();
  }

  /**@brief checks if an object is currently registered */
  bool throutContains(const void *obj) {
    throutRWLock.readLock();
    bool r = throutContains_private(obj);
    throutRWLock.unlock();
    return r;
  }

  /**@brief prints a message on the console
   * 
   * - NULL messages will be converted to "NULL"
   */
  void throut(const char *m) {
    if(m == NULL) {
      throut("NULL");
      return;
    }
    msgMutex.lock();
    std::cout << m << std::endl;
    msgMutex.unlock();
  }

  /** @brief prints a message on the console */
  void throut(const MT::String &m) {
    throut((const char*)m);
  }

  /** @brief prints a message on the console, pre-pended with an object's heading
   * 
   * - NULL messages will be converted to "NULL".
   * - Messages paired with unregistered objects will be issued pre-pended by
   * "UNREGISTERED_OBJ: ".
   */
  void throut(const void *obj, const char *m) {
    char *head = NULL;
    throutRWLock.readLock();
    if(throutGetHeading_private(obj, &head))
      if(m == NULL)
        throut(STRING(head << "NULL"));
      else
        throut(STRING(head << m));
    else
      if(m == NULL)
        throut("UNREGISTERED_OBJ: NULL");
      else
        throut(STRING("UNREGISTERED_OBJ: " << m));
    throutRWLock.unlock();
  }

  /** @brief prints a message on the console, pre-pended with an object's heading
   * 
   * - Messages paired with unregistered objects will be issued pre-pended by
   * "UNREGISTERED_OBJ: ".
   */
  void throut(const void *obj, const MT::String &m) {
    throut(obj, (const char*)m);
  }
}
