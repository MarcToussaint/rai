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

void stop(const ThreadL& P);
void wait(const ThreadL& P);
void close(const ThreadL& P);

#ifndef MLR_MSVC

//===========================================================================

/// a basic read/write access lock
struct RWLock {
  pthread_rwlock_t lock;
  int state; ///< -1==write locked, positive=numer of readers, 0=unlocked
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
  int value;
  Mutex mutex;
  pthread_cond_t  cond;

  ConditionVariable(int initialState=0);
  ~ConditionVariable();

  void setValue(int i, bool signalOnlyFirstInQueue=false); ///< sets state and broadcasts
  int  incrementValue(bool signalOnlyFirstInQueue=false);   ///< increase value by 1
  void broadcast(bool signalOnlyFirstInQueue=false);       ///< just broadcast

  void lock();   //the user can manually lock/unlock, if he needs atomic state access for longer -> use userHasLocked=true below!
  void unlock();

  int  getValue(bool userHasLocked=false) const;
  void waitForSignal(bool userHasLocked=false);
  bool waitForSignal(double seconds, bool userHasLocked=false);
  bool waitForValueEq(int i, bool userHasLocked=false, double seconds=-1);    ///< return value is the state after the waiting
  void waitForValueNotEq(int i, bool userHasLocked=false); ///< return value is the state after the waiting
  void waitForValueGreaterThan(int i, bool userHasLocked=false); ///< return value is the state after the waiting
  void waitForValueSmallerThan(int i, bool userHasLocked=false); ///< return value is the state after the waiting
  void waitUntil(double absTime, bool userHasLocked=false);
};

//===========================================================================
//
// Timing helpers
//

/// a simple struct to realize a strict tic tac timing (call step() once in a loop)
struct Metronome {
  double ticInterval;
  timespec ticTime;
  uint tics;

  Metronome(double ticIntervalSec); ///< set tic tac time in micro seconds

  void reset(double ticIntervalSec);
  void waitForTic();              ///< waits until the next tic
  double getTimeSinceTic();       ///< time since last tic
};

/// a really simple thing to meassure cycle and busy times
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
 */
struct Thread{
  mlr::String name;
  ConditionVariable state;       ///< the condition variable indicates the state of the thread: positive=steps-to-go, otherwise it is a ThreadState
  RevisionedRWLockL listensTo;
  pid_t tid;                     ///< system thread id
#ifndef MLR_QThread
  pthread_t thread;
#else
  struct sThread *thread;
#endif
  uint step_count;
  Metronome metronome;          ///< used for beat-looping
  CycleTimer timer;
  struct Node* registryNode;
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
  void threadOpen(int priority=0);      ///< start the thread (in idle mode) (should be positive for changes)
  void threadClose();                   ///< close the thread (stops looping and waits for idle mode before joining the thread)
  void threadStep(uint steps=1, bool wait=false);     ///< trigger (multiple) step (idle -> working mode) (wait until idle? otherwise calling during non-idle -> error)
  void threadLoop();                    ///< loop, either with fixed beat or at full speed
//  void threadLoopWithBeat(double beatIntervalSec);  ///< loop with a fixed beat (cycle time)
  void threadStop();                    ///< stop looping
  void threadCancel();                  ///< a hard kill (pthread_cancel) of the thread

  void waitForOpened();                 ///< caller waits until opening is done (working -> idle mode)
  void waitForIdle();                   ///< caller waits until step is done (working -> idle mode)
  bool isIdle();                        ///< check if in idle mode
  bool isClosed();                      ///< check if closed

  /// @name listen to a variable
  void listenTo(RevisionedRWLock& var);
  void stopListenTo(RevisionedRWLock& var);

  /** use this to open drivers/devices/files and initialize
   *  parameters; this is called within the thread */
  virtual void open() = 0;

  /** The most important method of all of this: step does the actual
   *  computation of the module. Modules should be state less. Access
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
//
// access gated (rwlocked) variables (shared memory)
//

/// Deriving from this allows to make variables/classes revisioned read-write access gated
struct RevisionedRWLock {
  mlr::String name;           ///< AccessData name
  RWLock rwlock;              ///< rwLock (usually handled via read/writeAccess)
  ConditionVariable revision; ///< revision (= number of write accesses) number
  int last_revision;          ///< last revision that has been accessed (read or write)
  double revision_time;       ///< clock time of last write access
  double data_time;           ///< time stamp of the original data source
  ThreadL listeners;          ///< list of threads that are being signaled a threadStep on write access
  struct Node* registryNode;  ///< these objects are globally registered, so that new Accesses can connect with them

  /// @name c'tor/d'tor
  virtual ~RevisionedRWLock();

  /// @name access control
  /// to be called by a thread before access, returns the revision
  int readAccess(Thread* th=NULL);  //might set the caller to sleep
  int writeAccess(Thread* th=NULL); //might set the caller to sleep
  int deAccess(Thread* th=NULL);

  /// @name syncing via a variable
  bool hasNewRevision();
  /// the caller is set to sleep
  int waitForNextRevision();
  int waitForRevisionGreaterThan(int rev); //returns the revision
  double revisionTime();
  int revisionNumber();
};
inline void operator<<(ostream& os, const RevisionedRWLock& v){ os <<"AccessData '" <<v.name <<'\''; }

//===========================================================================

template<class T>
struct WToken{
  RevisionedRWLock& revLock;
  T *x;
  WToken(RevisionedRWLock& _revLock, T *var) : revLock(_revLock), x(var){ revLock.writeAccess(); }
  ~WToken(){ revLock.deAccess(); }
  void operator=(const T& y){ *x=y; }
  T* operator->(){ return x; }
  operator T&(){ return *x; }
  T& operator()(){ return *x; }
};

//===========================================================================

/// A variable is an access gated data field of type T
template<class T>
struct AccessData : RevisionedRWLock{
  T value;

  AccessData() = default;
  AccessData(const AccessData&){ HALT("not allowed"); }
  void operator=(const AccessData&){ HALT("not allowed"); }
//  ReadToken get(Thread *th=NULL){ return ReadToken(this, th); } ///< read access to the variable's data
//  WriteToken set(Thread *th=NULL){ return WriteToken(this, th); } ///< write access to the variable's data
//  WriteToken set(const double& dataTime, Thread *th=NULL){ return WriteToken(dataTime, this, th); } ///< write access to the variable's data
};

inline bool operator==(const RevisionedRWLock&,const RevisionedRWLock&){ return false; }

//===========================================================================

//-- Token-wise access
template<class T>
struct ReadToken{
  AccessData<T> *v;
  Thread *th;
  ReadToken(AccessData<T> *v, Thread *th):v(v), th(th){ v->readAccess(th); }
  ~ReadToken(){ v->deAccess(th); }
  const T* operator->(){ return &v->value; }
  operator const T&(){ return v->value; }
  const T& operator()(){ return v->value; }
};

template<class T>
struct WriteToken{
  AccessData<T> *v;
  Thread *th;
  WriteToken(AccessData<T> *v, Thread *th):v(v), th(th){ v->writeAccess(th); }
  WriteToken(const double& dataTime, AccessData<T> *v, Thread *th):v(v), th(th){ v->writeAccess(th); v->data_time=dataTime; }
  ~WriteToken(){ v->deAccess(th); }
  WriteToken& operator=(const T& x){ v->value=x; return *this; }
  T* operator->(){ return &v->value; }
  operator T&(){ return v->value; }
  T& operator()(){ return v->value; }
};

//===========================================================================
//
/** When using a thread you may declare which variables the
    thread needs access to (for reading or writing). This is done by
    declaring members as 'Access_typed<TYPE> name;' instead of 'TYPE
    name;'. The macro ACCESS(TYPE, name); makes this simpler. Access
    is the base class of Access_typed */

struct Access{
  mlr::String name; ///< name; by default the access' name; redefine to a variable's name to autoconnect
  Thread *thread;  ///< which module is this a member of
  RevisionedRWLock *revLock;   ///< which variable does it access
  struct Node* registryNode;
  Access(const char* _name, Thread *_thread, RevisionedRWLock *_revLock):name(_name), thread(_thread), revLock(_revLock){}
  virtual ~Access(){}
  bool hasNewRevision(){ return revLock->hasNewRevision(); }
  int readAccess(){  return revLock->readAccess((Thread*)thread); }
  int writeAccess(){ return revLock->writeAccess((Thread*)thread); }
  int deAccess(){    return revLock->deAccess((Thread*)thread); }
  int waitForNextRevision(){ return revLock->waitForNextRevision(); }
  int waitForRevisionGreaterThan(int rev){    return revLock->waitForRevisionGreaterThan(rev); }
//  double& tstamp(){ return _data->data_time; } ///< reference to the data's time. AccessData should be locked while accessing this.
  double& dataTime(){ return revLock->data_time; } ///< reference to the data's time. AccessData should be locked while accessing this.
};


/** See Access. This provides read/write 'tokens' (generated by get()
    and set()) which allow convenient and typed read/write access to
    the variable's content */
template<class T>
struct Access_typed : Access{
  AccessData<T> *data;

  /// A "copy" of acc: An access to the same variable as acc refers to, but now for '_thred'
  Access_typed(Thread* _thread, const Access_typed<T>& acc, bool moduleListens=false)
    : Access(acc.name, _thread, NULL), data(NULL){
    data = acc.data;
    revLock = acc.revLock;
    if(thread){
      registryNode = registry().newNode<Access_typed<T>* >({"Access", name}, {thread->registryNode, data->registryNode}, this);
      if(moduleListens) thread->listenTo(*revLock);
    }else{
      registryNode = registry().newNode<Access_typed<T>* >({"Access", name}, {data->registryNode}, this);
    }
  }

  /// searches for globally registrated variable 'name', checks type equivalence, and becomes an access for '_thred'
  Access_typed(Thread* _thread, const char* name, bool moduleListens=false)
    : Access(name, _thread, NULL), data(NULL){
    data = registry().find<AccessData<T> >({"AccessData", name});
    if(!data){ //this is the ONLY place where a variable should be created
      Node_typed<AccessData<T> > *vnode = registry().newNode<AccessData<T> >({"AccessData", name}, {});
      data = &vnode->value;
      data->name = name;
      data->registryNode = vnode;
    }
    revLock = dynamic_cast<RevisionedRWLock*>(data);
    if(thread){
      registryNode = registry().newNode<Access_typed<T>* >({"Access", name}, {thread->registryNode, data->registryNode}, this);
      if(moduleListens) thread->listenTo(*revLock);
    }else{
      registryNode = registry().newNode<Access_typed<T>* >({"Access", name}, {data->registryNode}, this);
    }
  }

  ~Access_typed(){ delete registryNode; }
  T& operator()(){ CHECK(data->rwlock.isLocked(),"direct variable access without locking it before");  return data->value; }
  T* operator->(){ CHECK(data->rwlock.isLocked(),"direct variable access without locking it before");  return &(data->value); }
  ReadToken<T> get(){ return ReadToken<T>(data, thread); } ///< read access to the variable's data
  WriteToken<T> set(){ return WriteToken<T>(data, thread); } ///< write access to the variable's data
  WriteToken<T> set(const double& dataTime){ return WriteToken<T>(dataTime, data, thread); } ///< write access to the variable's data
};

inline bool operator==(const Access&,const Access&){ return false; }

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

  void setValue(int i, bool signalOnlyFirstInQueue=false) { value=i; }
  int  incrementValue(bool signalOnlyFirstInQueue=false) { value++; }
  void broadcast(bool signalOnlyFirstInQueue=false) {}

  void lock() {}
  void unlock() {}

  int  getValue(bool userHasLocked=false) const { return value; }
};

#endif //MLR_MSVC

#endif
