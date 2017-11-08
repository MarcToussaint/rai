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

#include "thread.h"
#include "graph.h"
#include <exception>
#include <signal.h>
#include <iomanip>

#ifndef MLR_MSVC
#ifndef __CYGWIN__
#  include <sys/syscall.h>
#else
#  include "cygwin_compat.h"
#endif //__CYGWIN __
#  include <unistd.h>
#endif
#include <errno.h>


#ifndef MLR_MSVC

//===========================================================================
//
// Access RWLock
//

RWLock::RWLock() {
  int rc = pthread_rwlock_init(&rwLock, NULL);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rwCount=0;
}

RWLock::~RWLock() {
  CHECK(!rwCount, "Destroying locked RWLock");
  int rc = pthread_rwlock_destroy(&rwLock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void RWLock::readLock() {
  int rc = pthread_rwlock_rdlock(&rwLock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rwCountMutex.lock();
  rwCount++;
  rwCountMutex.unlock();
}

void RWLock::writeLock() {
  int rc = pthread_rwlock_wrlock(&rwLock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rwCountMutex.lock();
  rwCount=-1;
  rwCountMutex.unlock();
}

void RWLock::unlock() {
  rwCountMutex.lock();
  if(rwCount>0) rwCount--; else rwCount=0;
  rwCountMutex.unlock();
  int rc = pthread_rwlock_unlock(&rwLock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

bool RWLock::isLocked() {
  return rwCount!=0;
}



//===========================================================================
//
// Signaler
//

Signaler::Signaler(int initialStatus)
  : status(initialStatus), registryNode(NULL){
  int rc = pthread_cond_init(&cond, NULL);    if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

Signaler::~Signaler() {
  for(Signaler *c:listensTo){
    c->statusLock();
    c->listeners.removeValue(this);
    c->statusUnlock();
  }
  for(Signaler *c:listeners){
    c->statusLock();
    c->listensTo.removeValue(this);
    c->messengers.removeValue(this, false);
    c->statusUnlock();
  }
  listDelete(callbacks);
  int rc = pthread_cond_destroy(&cond);    if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void Signaler::setStatus(int i, Signaler* messenger) {
  statusMutex.lock();
  status=i;
  broadcast(messenger);
  statusMutex.unlock();
}

int Signaler::incrementStatus(Signaler* messenger) {
  statusMutex.lock();
  status++;
  broadcast(messenger);
  int i=status;
  statusMutex.unlock();
  return i;
}

void Signaler::broadcast(Signaler* messenger) {
  //remember the messengers:
  if(messenger) messengers.setAppend(messenger);
  //signal to all waiters:
  int rc = pthread_cond_signal(&cond);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  //int rc = pthread_cond_broadcast(&cond);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  //setStatus to all listeners:
  for(Signaler *c:listeners) if(c!=messenger){
    Thread *th = dynamic_cast<Thread*>(c);
    if(th) th->threadStep();
    else c->setStatus(1, this);
  }
  for(auto* c:callbacks) c->call()(this, status);
}

void Signaler::listenTo(Signaler& c){
  statusMutex.lock();
  c.statusLock();
  c.listeners.append(this);
  listensTo.append(&c);
  c.statusUnlock();
  statusMutex.unlock();
}

void Signaler::stopListenTo(Signaler& c){
  statusMutex.lock();
  c.statusLock();
  c.listeners.removeValue(this);
  listensTo.removeValue(&c);
  messengers.removeValue(&c, false);
  c.statusUnlock();
  statusMutex.unlock();
}

void Signaler::stopListening(){
  statusMutex.lock();
  for(Signaler *c:listensTo){
    c->statusLock();
    c->listeners.removeValue(this);
    c->statusUnlock();
  }
  listensTo.clear();
  messengers.clear();
  statusMutex.unlock();
}

void Signaler::statusLock() {
  statusMutex.lock();
}

void Signaler::statusUnlock() {
  statusMutex.unlock();
}

int Signaler::getStatus(bool userHasLocked) const {
  Mutex *m = (Mutex*)&statusMutex; //sorry: to allow for 'const' access
  if(!userHasLocked) m->lock(); else CHECK_EQ(m->state,syscall(SYS_gettid),"user must have locked before calling this!");
  int i=status;
  if(!userHasLocked) m->unlock();
  return i;
}

void Signaler::waitForSignal(bool userHasLocked) {
  if(!userHasLocked) statusMutex.lock(); // else CHECK_EQ(mutex.state, syscall(SYS_gettid), "user must have locked before calling this!");
  int rc = pthread_cond_wait(&cond, &statusMutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  if(!userHasLocked) statusMutex.unlock();
}

bool Signaler::waitForSignal(double seconds, bool userHasLocked) {
  struct timespec timeout;
  clock_gettime(CLOCK_REALTIME, &timeout); //CLOCK_MONOTONIC, &timeout);
  long secs = (long)(floor(seconds));
  seconds -= secs;
  timeout.tv_sec  += secs;
  timeout.tv_nsec += (long)(floor(1e9 * seconds));
  if(timeout.tv_nsec>1000000000l) {
    timeout.tv_sec+=1;
    timeout.tv_nsec-=1000000000l;
  }

  if(!userHasLocked) statusMutex.lock(); // else CHECK_EQ(mutex.state, syscall(SYS_gettid), "user must have locked before calling this!");
  int rc = pthread_cond_timedwait(&cond, &statusMutex.mutex, &timeout);
  if(rc && rc!=ETIMEDOUT) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  if(!userHasLocked) statusMutex.unlock();
  return rc!=ETIMEDOUT;
}

bool Signaler::waitForStatusEq(int i, bool userHasLocked, double seconds) {
  if(!userHasLocked) statusMutex.lock(); else CHECK_EQ(statusMutex.state, syscall(SYS_gettid), "user must have locked before calling this!");
  while(status!=i) {
    if(seconds<0.){
      int rc = pthread_cond_wait(&cond, &statusMutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
    }else{
      bool succ = waitForSignal(seconds, true);
      if(!succ){
        if(!userHasLocked) statusMutex.unlock();
        return false;
      }
    }
  }
  if(!userHasLocked) statusMutex.unlock();
  return true;
}

void Signaler::waitForStatusNotEq(int i, bool userHasLocked) {
  if(!userHasLocked) statusMutex.lock(); else CHECK_EQ(statusMutex.state, syscall(SYS_gettid), "user must have locked before calling this!");
  while(status==i) {
    int rc = pthread_cond_wait(&cond, &statusMutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  }
  if(!userHasLocked) statusMutex.unlock();
}

void Signaler::waitForStatusGreaterThan(int i, bool userHasLocked) {
  if(!userHasLocked) statusMutex.lock(); else CHECK_EQ(statusMutex.state,syscall(SYS_gettid),"user must have locked before calling this!");
  while(status<=i) {
    int rc = pthread_cond_wait(&cond, &statusMutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  }
  if(!userHasLocked) statusMutex.unlock();
}

void Signaler::waitForStatusSmallerThan(int i, bool userHasLocked) {
  if(!userHasLocked) statusMutex.lock(); else CHECK_EQ(statusMutex.state,syscall(SYS_gettid),"user must have locked before calling this!");
  while(status>=i) {
    int rc = pthread_cond_wait(&cond, &statusMutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  }
  if(!userHasLocked) statusMutex.unlock();
}


//===========================================================================
//
// VariableBase
//

//VariableBase::VariableBase(const char *_name):name(_name), revision(0), registryNode(NULL) {
////  registryNode = registry()->newNode<VariableBase* >({"VariableData", name}, {}, this);
//}

VariableBase::~VariableBase() {
//  registry()->delNode(registryNode);
}

//bool VariableBase::hasNewRevision(){
//  return revision.getStatus() > last_revision;
//}

int VariableBase::readAccess(Thread *th) {
//  engine().acc->queryReadAccess(this, p);
  rwlock.readLock();
//  engine().acc->logReadAccess(this, p);
  return getStatus();
}

int VariableBase::writeAccess(Thread *th) {
//  engine().acc->queryWriteAccess(this, p);
  rwlock.writeLock();
  write_time = mlr::clockTime();
//  engine().acc->logWriteAccess(this, p);
  return getStatus()+1;
}

int VariableBase::deAccess(Thread *th) {
//  Module_Thread *p = m?(Module_Thread*) m->thread:NULL;
  int i;
  if(rwlock.rwCount == -1) { //log a revision after write access
    i = incrementStatus(th);
//    engine().acc->logWriteDeAccess(this,p);
  } else {
//    engine().acc->logReadDeAccess(this,p);
    i = getStatus();
  }
  rwlock.unlock();
  return i;
}

//int VariableBase::waitForNextRevision(){
//  revision.statusLock();
//  revision.waitForSignal(true);
//  int rev = revision.status;
//  revision.statusUnlock();
//  return rev;
//}

//int VariableBase::waitForRevisionGreaterThan(int rev) {
//  revision.statusLock();
//  revision.waitForStatusGreaterThan(rev, true);
//  rev = revision.status;
//  revision.statusUnlock();
//  return rev;
//}


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
  int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ticTime, NULL);
  if(rc && errno) MLR_MSG("clock_nanosleep() failed " <<rc <<" errno=" <<errno <<' ' <<strerror(errno));

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

mlr::String CycleTimer::report(){
  mlr::String s;
  s.printf("busy=[%5.1f %5.1f] cycle=[%5.1f %5.1f] load=%4.1f%% steps=%i", busyDtMean, busyDtMax, cyclDtMean, cyclDtMax, 100.*busyDtMean/cyclDtMean, steps);
  return s;
//  fflush(stdout);
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

#ifdef MLR_QThread
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

Thread::Thread(const char* _name, double beatIntervalSec)
  : Signaler(tsIsClosed),
     name(_name),
     thread(0),
     tid(0),
     step_count(0),
     metronome(beatIntervalSec),
     verbose(0) {
  registryNode = registry()->newNode<Thread*>({"Thread", name}, {}, this);
  if(name.N>14) name.resize(14, true);
}

Thread::~Thread() {
  if(thread)
      HALT("Call 'threadClose()' in the destructor of the DERIVED class! \
           That's because the 'virtual table is destroyed' before calling the destructor ~Thread (google 'call virtual function\
           in destructor') but now the destructor has to call 'threadClose' which triggers a Thread::close(), which is\
           pure virtual while you're trying to call ~Thread.")
  registry()->delNode(registryNode);
}

void Thread::threadOpen(bool wait, int priority) {
  statusLock();
  if(thread){ statusUnlock(); return; } //this is already open -- or has just beend opened (parallel call to threadOpen)
#ifndef MLR_QThread
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
  status=tsToOpen;
  statusUnlock();
  if(wait) waitForStatusNotEq(tsToOpen);
  if(metronome.ticInterval>0.){
      if(metronome.ticInterval>1e-10){
          setStatus(tsBEATING);
      }else{
          setStatus(tsLOOPING);
      }
  }
}

void Thread::threadClose(double timeoutForce) {
  stopListening();
  setStatus(tsToClose);
  if(!thread){ setStatus(tsIsClosed); return; }
  for(;;){
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
#ifndef MLR_QThread
  int rc;
  rc = pthread_join(thread, NULL);     if(rc) HALT("pthread_join failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  thread=0;
#else
  thread->close();
  delete thread;
  thread=NULL;
#endif
}

void Thread::threadCancel() {
  stopListening();
  setStatus(tsToClose);
  if(!thread) return;
#ifndef MLR_QThread
  int rc;
  rc = pthread_cancel(thread);         if(rc) HALT("pthread_cancel failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rc = pthread_join(thread, NULL);     if(rc) HALT("pthread_join failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  thread=0;
#else
  NIY;
#endif
  stepMutex.state=-1; //forced destroy in the destructor
}

void Thread::threadStep() {
  threadOpen();
  setStatus(tsToStep);
}

//void Thread::listenTo(VariableBase& var) {
//#if 0
//  var.rwlock.writeLock();  //don't want to increase revision and broadcast!
//  var.listeners.setAppend(this);
//  var.rwlock.unlock();
//  listensTo.setAppend(&var);
//#else
//  listenTo(&var.revision);
//#endif
//}

//void Thread::stopListenTo(VariableBase& var){
//#if 0
//  listensTo.removeValue(&var);
//  var.rwlock.writeLock();
//  var.listeners.removeValue(this);
//  var.rwlock.unlock();
//#else
//  stopListenTo(&var.revision);
//#endif
//}

bool Thread::isIdle() {
  return getStatus()==tsIDLE;
}

bool Thread::isClosed() {
  return !thread; //getStatus()==tsIsClosed;
}

void Thread::waitForOpened() {
  waitForStatusNotEq(tsIsClosed);
  waitForStatusNotEq(tsToOpen);
}

void Thread::waitForIdle() {
  waitForStatusEq(tsIDLE);
}

void Thread::threadLoop(bool waitForOpened) {
  threadOpen(waitForOpened);
  if(metronome.ticInterval>1e-10){
    setStatus(tsBEATING);
  }else{
    setStatus(tsLOOPING);
  }
}

void Thread::threadStop(bool wait) {
  if(thread){
    setStatus(tsIDLE);
    if(wait) waitForIdle();
  }
}

void Thread::main() {
  tid = syscall(SYS_gettid);
  if(verbose>0) cout <<"*** Entering Thread '" <<name <<"'" <<endl;
  //http://linux.die.net/man/3/setpriority
  //if(Thread::threadPriority) setRRscheduling(Thread::threadPriority);
  //if(Thread::threadPriority) setNice(Thread::threadPriority);

  {
    auto mux = stepMutex();
    try{
      open(); //virtual open routine
    } catch(const std::exception& ex) {
      setStatus(tsFAILURE);
      cerr << "*** open() of Thread'" << name << "'failed: " << ex.what() << " -- closing it again" << endl;
    } catch(...) {
      setStatus(tsFAILURE);
      cerr <<"*** open() of Thread '" <<name <<"' failed! -- closing it again";
      return;
    }
  }

  statusLock();
  if(status==tsToOpen){
    status=tsIDLE;
    broadcast();
  }
  //if not =tsOPENING anymore -> the state was set on looping or beating already
  statusUnlock();


  timer.reset();
  bool waitForTic=false;
  for(;;){
    //-- wait for a non-idle state
    statusLock();
    waitForStatusNotEq(tsIDLE, true);
    if(status==tsToClose) { statusUnlock();  break; }
    if(status==tsBEATING) waitForTic=true; else waitForTic=false;
    if(status>0) status=tsIDLE; //step command -> reset to idle
    statusUnlock();

    if(waitForTic) metronome.waitForTic();

    //-- make a step
    //engine().acc->logStepBegin(module);
    timer.cycleStart();
    stepMutex.lock();
    step(); //virtual step routine
    stepMutex.unlock();
    step_count++;
    timer.cycleDone();
    //engine().acc->logStepEnd(module);

//    //-- broadcast in case somebody was waiting for a finished step
//    status.lock();
//    status.broadcast();
//    status.unlock();
  };

  stepMutex.lock();
  close(); //virtual close routine
  stepMutex.unlock();
  if(verbose>0) cout <<"*** Exiting Thread '" <<name <<"'" <<endl;

  setStatus(tsIsClosed);
}


//===========================================================================
//
// controlling threads
//

Singleton<Signaler> moduleShutdown;

void signalhandler(int s){
  int calls = moduleShutdown()->incrementStatus();
  cerr <<"\n*** System received signal " <<s <<" -- count=" <<calls <<endl;
  if(calls==1){
    LOG(0) <<" -- waiting for main loop to break on moduleShutdown()->getStatus()" <<endl;
  }
  if(calls==2){
    LOG(0) <<" -- smoothly closing modules directly" <<endl;
    threadCloseModules(); //might lead to a hangup of the main loop, but processes should close
    LOG(0) <<" -- DONE" <<endl;
  }
  if(calls==3){
    LOG(0) <<" -- cancelling threads to force closing" <<endl;
    threadCancelModules();
    LOG(0) <<" -- DONE" <<endl;
  }
  if(calls>3){
    LOG(3) <<" ** moduleShutdown failed - hard exit!" <<endl;
    exit(1);
  }
}

void openModules(){
  NodeL threads = registry()->getNodesOfType<Thread*>();
  for(Node* th:threads){ th->get<Thread*>()->open(); }
}

void stepModules(){
  NodeL threads = registry()->getNodesOfType<Thread*>();
  for(Node* th:threads){ th->get<Thread*>()->step(); }
}

void closeModules(){
  NodeL threads = registry()->getNodesOfType<Thread*>();
  for(Node* th:threads){ th->get<Thread*>()->close(); }
}

VariableBase::Ptr getVariable(const char* name){
  return registry()->get<VariableBase::Ptr>({"VariableData", name});
}

VariableBaseL getVariables(){
  return registry()->getValuesOfType<VariableBase::Ptr>();
}

void threadOpenModules(bool waitForOpened, bool setSignalHandler){
  if(setSignalHandler) signal(SIGINT, signalhandler);
  NodeL threads = registry()->getNodesOfType<Thread*>();
  for(Node *th: threads) th->get<Thread*>()->threadOpen();
  if(waitForOpened) for(Node *th: threads) th->get<Thread*>()->waitForOpened();
  for(Node *th: threads){
    Thread *mod=th->get<Thread*>();
    if(mod->metronome.ticInterval>=0.) mod->threadLoop();
    //otherwise the module is listening (hopefully)
  }
}

void threadCloseModules(){
  NodeL threads = registry()->getNodesOfType<Thread*>();
  for(Node *th: threads) th->get<Thread*>()->threadClose();
//  threadReportCycleTimes();
}

void threadCancelModules(){
  NodeL threads = registry()->getNodesOfType<Thread*>();
  for(Node *th: threads) th->get<Thread*>()->threadCancel();
}

void threadReportCycleTimes(){
  cout <<"Cycle times for all Threads (msec):" <<endl;
  NodeL threads = registry()->getNodesOfType<Thread*>();
  for(Node *th: threads){
    Thread *thread=th->get<Thread*>();
    cout <<std::setw(30) <<thread->name <<" : " <<thread->timer.report() <<endl;
  }
}


//===========================================================================
//
// Utils
//

//void stop(const ThreadL& P) {
//  for_list(Thread,  p,  P) p->threadStop();
//}

//void wait(const ThreadL& P) {
//  for_list(Thread,  p,  P) p->waitForIdle();
//}

//void close(const ThreadL& P) {
//  for_list(Thread,  p,  P) p->threadClose();
//}

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

RUN_ON_INIT_BEGIN(thread)
VariableBaseL::memMove=true;
ThreadL::memMove=true;
SignalerL::memMove=true;
RUN_ON_INIT_END(thread)

#endif //MLR_MSVC
