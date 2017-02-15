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

#ifndef MLR_thread_h
#define MLR_thread_h

#include "util.h"
#include "array.h"
#include "graph.h"

enum ThreadState { tsIDLE=0, tsCLOSE=-1, tsOPENING=-2, tsLOOPING=-3, tsBEATING=-4, tsFAILURE=-5 }; //positive states indicate steps-to-go
struct ConditionVariable;
struct RevisionedRWLock;
struct Thread;
typedef mlr::Array<ConditionVariable*> ConditionVariableL;
typedef mlr::Array<RevisionedRWLock*> RevisionedRWLockL;
typedef mlr::Array<Thread*> ThreadL;

//void stop(const ThreadL& P);
//void wait(const ThreadL& P);
//void close(const ThreadL& P);

#ifndef MLR_MSVC

//===========================================================================

/// a basic read/write access lock
struct RWLock {
  pthread_rwlock_t lock;
  int state;         ///< -1==write locked, positive=numer of readers, 0=unlocked
  Mutex stateMutex;
  RWLock();
  ~RWLock();
  void readLock();   ///< multiple threads may request 'lock for read'
  void writeLock();  ///< only one thread may request 'lock for write'
  void unlock();     ///< thread must unlock when they're done
  bool isLocked();
};

//===========================================================================

/// a basic condition variable
struct ConditionVariable {
  int status;
  Mutex mutex;
  pthread_cond_t  cond;
  ConditionVariableL listeners;   ///< list of other condition variables that are being signaled on a setStatus access
  ConditionVariableL listensTo;   ///< ...
  ConditionVariableL messengers;  ///< set(!) of condition variables that send signals (via the listen mechanism) - is cleared by the user only
  struct Node* registryNode;      ///< every threading object registers itself globally

  ConditionVariable(int initialStatus=0);
  virtual ~ConditionVariable(); //virtual, to enforce polymorphism

  void setStatus(int i, ConditionVariable* messenger=NULL); ///< sets state and broadcasts
  int  incrementStatus(ConditionVariable* messenger=NULL);   ///< increase value by 1
  void broadcast(ConditionVariable* messenger=NULL);       ///< just broadcast
  void listenTo(ConditionVariable *c);
  void stopListenTo(ConditionVariable *c);

  void statusLock();   //the user can manually lock/unlock, if he needs locked state access for longer -> use userHasLocked=true below!
  void statusUnlock();

  int  getStatus(bool userHasLocked=false) const;
  void waitForSignal(bool userHasLocked=false);
  bool waitForSignal(double seconds, bool userHasLocked=false);
  bool waitForStatusEq(int i, bool userHasLocked=false, double seconds=-1);    ///< return value is the state after the waiting
  void waitForStatusNotEq(int i, bool userHasLocked=false); ///< return value is the state after the waiting
  void waitForStatusGreaterThan(int i, bool userHasLocked=false); ///< return value is the state after the waiting
  void waitForStatusSmallerThan(int i, bool userHasLocked=false); ///< return value is the state after the waiting
};

//===========================================================================
//
// access gated (rwlocked) variables (shared memory)
//

/// This RW lock counts revisions and broadcasts accesses to listeners; who is accessing can be logged; it has a unique name
struct RevisionedRWLock : ConditionVariable{
  RWLock rwlock;              ///< rwLock (usually handled via read/writeAccess)
  double write_time;          ///< clock time of last write access
  double data_time;           ///< time stamp of the original data source

  /// @name c'tor/d'tor
  virtual ~RevisionedRWLock();

  /// @name access control
  /// to be called by a thread before access, returns the revision
  int readAccess(Thread* th=NULL);  //might set the caller to sleep
  int writeAccess(Thread* th=NULL); //might set the caller to sleep
  int deAccess(Thread* th=NULL);
};


//===========================================================================

template<class T>
struct RToken{
  RevisionedRWLock& revLock;
  T *x;
  Thread *th;
  int *last_access_revision;
  RToken(RevisionedRWLock& _revLock, T *var, Thread* th=NULL, int* last_access_revision=NULL, bool isAlreadyLocked=false)
    : revLock(_revLock), x(var), th(th), last_access_revision(last_access_revision){ if(!isAlreadyLocked) revLock.readAccess(th); }
  ~RToken(){ int r = revLock.deAccess(th); if(last_access_revision) *last_access_revision=r; }
  const T* operator->(){ return x; }
  operator const T&(){ return *x; }
  const T& operator()(){ return *x; }
};

template<class T>
struct WToken{
  RevisionedRWLock& revLock;
  T *x;
  Thread *th;
  int *last_access_revision;
  WToken(RevisionedRWLock& _revLock, T *var, Thread* th=NULL, int* last_access_revision=NULL)
    : revLock(_revLock), x(var), th(th), last_access_revision(last_access_revision){ revLock.writeAccess(th); }
  WToken(const double& dataTime, RevisionedRWLock& _revLock, T *var, Thread* th=NULL, int* last_access_revision=NULL)
    : revLock(_revLock), x(var), th(th), last_access_revision(last_access_revision){ revLock.writeAccess(th); revLock.data_time=dataTime; }
  ~WToken(){ int r = revLock.deAccess(th); if(last_access_revision) *last_access_revision=r; }
  void operator=(const T& y){ *x=y; }
  T* operator->(){ return x; }
  operator T&(){ return *x; }
  T& operator()(){ return *x; }
};

//===========================================================================
//
// Timing helpers
//

/// a simple struct to realize a strict tic tac timing (called in step() once in a loop)
struct Metronome {
  double ticInterval;
  timespec ticTime;
  uint tics;

  Metronome(double ticIntervalSec); ///< set tic tac time in micro seconds

  void reset(double ticIntervalSec);
  void waitForTic();              ///< waits until the next tic
  double getTimeSinceTic();       ///< time since last tic
};

/// to meassure cycle and busy times
struct CycleTimer {
  uint steps;
  double busyDt, busyDtMean, busyDtMax;  ///< internal variables to measure step time
  double cyclDt, cyclDtMean, cyclDtMax;  ///< internal variables to measure step time
  timespec now, lastTime;
  const char* name;                    ///< name
  CycleTimer(const char *_name=NULL);
  ~CycleTimer();
  void reset();
  void cycleStart();
  void cycleDone();
  void report();
};


//===========================================================================
/**
 * A Thread does some calculation and shares the result via a AccessData.
 *
 * Inherit from the class Thread to create your own process.
 * You need to implement open(), close(), and step().
 * step() should contain the actual calculation.
 *
 * the condition variable indicates the state of the thread: positive=do steps, otherwise it is a ThreadState
 */
struct Thread : ConditionVariable{
  mlr::String name;
  pid_t tid;                     ///< system thread id
#ifndef MLR_QThread
  pthread_t thread;
#else
  struct sThread *thread;
#endif
  Mutex stepMutex;              ///< This is set whenever the 'main' is in step (or open, or close) --- use this in all service methods callable from outside!!
  uint step_count;              ///< how often the step was called
  Metronome metronome;          ///< used for beat-looping
  CycleTimer timer;             ///< measure how the time spend per cycle, within step, idle
  int verbose;

  /// @name c'tor/d'tor
  /** DON'T open drivers/devices/files or so here in the constructor,
   * but in open(). Sometimes a module might be created only to see
   * which accesses it needs. The default constructure should really
   * do nothing
   *
   * beatIntervalSec=0. indicates full speed looping, beatIntervalSec=-1. indicates no looping (steps triggered by listening)
   */
  Thread(const char* _name, double beatIntervalSec=-1.);
  virtual ~Thread();

  /// @name to be called from `outside' (e.g. the main) to start/step/close the thread
  void threadOpen(bool wait=false, int priority=0);      ///< start the thread (in idle mode) (should be positive for changes)
  void threadClose();                   ///< close the thread (stops looping and waits for idle mode before joining the thread)
  void threadStep();                    ///< trigger (multiple) step (idle -> working mode) (wait until idle? otherwise calling during non-idle -> error)
  void threadLoop();                    ///< loop, either with fixed beat or at full speed
  void threadStop(bool wait=false);     ///< stop looping
  void threadCancel();                  ///< a hard kill (pthread_cancel) of the thread

  void waitForOpened();                 ///< caller waits until opening is done (working -> idle mode)
  void waitForIdle();                   ///< caller waits until step is done (working -> idle mode)
  bool isIdle();                        ///< check if in idle mode
  bool isClosed();                      ///< check if closed

  /// @name listen to a variable
//  void listenTo(RevisionedRWLock& var);
//  void stopListenTo(RevisionedRWLock& var);

  /** use this to open drivers/devices/files and initialize
   *  parameters; this is called within the thread */
  virtual void open() = 0;

  /** The most important method of all of this: step does the actual
   *  computation of the thread. Access
   *  the variables by calling the x.get(), x.set() or
   *  x.[read|write|de]Access(), where ACCESS(TYPE, x) was
   *  declared. */
  virtual void step(){ LOG(-1) <<"you're calling the 'pseudo-pure virtual' step(), which should be overloaded (are you in a destructor?)"; }

  /** use this to close drivers/devices/files; this is called within
   *  the thread */
  virtual void close() = 0;

  void main(); //this is the thread main - should be private!
};

// macro for a most standard declaration of a module
#define BEGIN_MODULE(name) \
  struct name : Thread { \
    struct s##name *s; \
    name(): Thread(#name), s(NULL) {} \
    virtual void open(); \
    virtual void step(); \
    virtual void close();

#define END_MODULE() };


//===========================================================================

/// A variable is an access gated data field of type T
template<class T>
struct AccessData : RevisionedRWLock {
  T value;
  mlr::String name;           ///< name

  AccessData() = default;
  AccessData(const AccessData&){ HALT("not allowed"); }
  void operator=(const AccessData&){ HALT("not allowed"); }
  RToken<T> get(Thread *th=NULL){ return RToken<T>(this, &value, th); } ///< read access to the variable's data
  WToken<T> set(Thread *th=NULL){ return WToken<T>(this, &value, th); } ///< write access to the variable's data
  WToken<T> set(const double& dataTime, Thread *th=NULL){ return WToken<T>(dataTime, this, &value, th); } ///< write access to the variable's data
};

template<class T> bool operator==(const AccessData<T>&,const AccessData<T>&){ return false; }
template<class T> void operator<<(ostream& os, const AccessData<T>& v){ os <<"AccessData '" <<v.name <<'\''; }

//===========================================================================
//
/** When using a thread you may declare which variables the
    thread needs access to (for reading or writing). This is done by
    declaring members as 'Access_typed<TYPE> name;' instead of 'TYPE
    name;'. The macro ACCESS(TYPE, name); makes this simpler. Access
    is the base class of Access_typed.

    This also provides read/write 'tokens' (generated by get()
    and set()) which allow convenient and typed read/write access to
    the variable's content */
template<class T>
struct Access_typed{
  AccessData<T> *data;
  mlr::String name; ///< name; by default the RevLock's name; redefine to a variable's name to autoconnect
  Thread *thread;  ///< which thread is this a member of
  int last_accessed_revision;          ///< last revision that has been accessed (read or write)
  struct Node* registryNode;

  /// A "copy" of acc: An access to the same variable as acc refers to, but now for '_thred'
  Access_typed(Thread* _thread, const Access_typed<T>& acc, bool moduleListens=false)
    : data(NULL), name(acc.name), thread(_thread), last_accessed_revision(0), registryNode(NULL){
    data = acc.data;
    if(thread){
      registryNode = registry().newNode<Access_typed<T>* >({"Access", name}, {thread->registryNode, data->registryNode}, this);
      if(moduleListens) thread->listenTo(data);
    }else{
      registryNode = registry().newNode<Access_typed<T>* >({"Access", name}, {data->registryNode}, this);
    }
  }

  /// searches for globally registrated variable 'name', checks type equivalence, and becomes an access for '_thred'
  Access_typed(Thread* _thread, const char* name, bool moduleListens=false)
    : data(NULL), name(name), thread(_thread), last_accessed_revision(0), registryNode(NULL){
    data = registry().find<AccessData<T> >({"AccessData", name});
    if(!data){ //this is the ONLY place where a variable should be created
      Node_typed<AccessData<T> > *vnode = registry().newNode<AccessData<T> >({"AccessData", name}, {});
      data = &vnode->value;
      data->name = name;
      data->registryNode = vnode;
    }
    if(thread){
      registryNode = registry().newNode<Access_typed<T>* >({"Access", name}, {thread->registryNode, data->registryNode}, this);
      if(moduleListens) thread->listenTo(data);
    }else{
      registryNode = registry().newNode<Access_typed<T>* >({"Access", name}, {data->registryNode}, this);
    }
  }

  ~Access_typed(){ delete registryNode; }
  T& operator()(){ CHECK(data->rwlock.isLocked(),"direct variable access without locking it before");  return data->value; }
  T* operator->(){ CHECK(data->rwlock.isLocked(),"direct variable access without locking it before");  return &(data->value); }
  RToken<T> get(){ return RToken<T>(*data, &data->value, thread, &last_accessed_revision); } ///< read access to the variable's data
  WToken<T> set(){ return WToken<T>(*data, &data->value, thread, &last_accessed_revision); } ///< write access to the variable's data
  WToken<T> set(const double& dataTime){ return WToken<T>(dataTime, *data, &data->value, thread, &last_accessed_revision); } ///< write access to the variable's data

  bool hasNewRevision(){ return data->getStatus()>last_accessed_revision; }
  int readAccess(){  return data->readAccess((Thread*)thread); }
  int writeAccess(){ return data->writeAccess((Thread*)thread); }
  int deAccess(){    last_accessed_revision = data->deAccess((Thread*)thread); return last_accessed_revision; }
  void waitForNextRevision(){ data->waitForStatusGreaterThan(last_accessed_revision); }
  void waitForRevisionGreaterThan(int rev){ data->waitForStatusGreaterThan(rev); }
  void stopListening(){ thread->stopListenTo(data); }
};

#define ACCESS(type, name) Access_typed<type> name = Access_typed<type>(this, #name);
#define ACCESSlisten(type, name) Access_typed<type> name = Access_typed<type>(this, #name, true);
#define ACCESSname(type, name) Access_typed<type> name = Access_typed<type>(NULL, #name);


//===========================================================================
//
// high-level methods to control threads

extern Singleton<ConditionVariable> moduleShutdown;
Node *getVariable(const char* name);
template <class T> T* getVariable(const char* name){  return dynamic_cast<T*>(registry().get<RevisionedRWLock*>({"AccessData",name}));  }
template <class T> T* getThread(const char* name){  return dynamic_cast<T*>(registry().get<Thread*>({"Thread",name}));  }
RevisionedRWLockL getVariables();
void openModules();
void stepModules();
void closeModules();
void threadOpenModules(bool waitForOpened, bool setSignalHandler=true);
void threadCloseModules();
void threadCancelModules();
void modulesReportCycleTimes();


// ================================================
//
// TStream utilities, for concurrent access to ostreams
//

#include <map>

// TODO: share a mutex between different ostreams
class TStream {
  private:
    std::ostream &out;
    Mutex mutex;
    RWLock lock;
    std::map<const void*, const char*> map;

  public:
    class Access;
    class Register;

    TStream(std::ostream &o);

    Access operator()(const void *obj = NULL);
    Register reg(const void *obj = NULL);
    void unreg(const void *obj);
    void unreg_all();
    bool get(const void *obj, char **head);

  private:
    void reg_private(const void *obj, char *head, bool l);
    void unreg_private(const void *obj, bool l);
    bool get_private(const void *obj, char **head, bool l);
};

class TStream::Access: public std::ostream {
  private:
    TStream *tstream;
    std::stringstream stream;
    const void *obj;

  public:
    Access(TStream *ts, const void *o);
    Access(const Access &a);
    ~Access();

    template<class T>
    std::stringstream& operator<<(const T &t);
};

class TStream::Register: public std::ostream {
  private:
    TStream *tstream;
    std::stringstream stream;
    const void *obj;

  public:
    Register(TStream *ts, const void *o);
    Register(const Register &r);
    ~Register();

    template<class T>
    std::stringstream& operator<<(const T &t);
};

template<class T>
std::stringstream& TStream::Access::operator<<(const T &t) {
  stream << t;
  return stream;
}

template<class T>
std::stringstream& TStream::Register::operator<<(const T &t) {
  stream << t;
  return stream;
}

#else //MLR_MSVC

struct ConditionVariable {
  int value;
  ConditionVariable(int initialState=0) {}
  ~ConditionVariable() {}

  void setStatus(int i, bool signalOnlyFirstInQueue=false) { value=i; }
  int  incrementStatus(bool signalOnlyFirstInQueue=false) { value++; }
  void broadcast(bool signalOnlyFirstInQueue=false) {}

  void lock() {}
  void unlock() {}

  int  getStatus(bool userHasLocked=false) const { return value; }
};

#endif //MLR_MSVC

#endif
