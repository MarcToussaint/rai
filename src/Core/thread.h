/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "util.h"
#include "array.h"

#include <mutex>
#include <shared_mutex>
#include <condition_variable>
#include <thread>

enum ThreadState { tsIsClosed=-6, tsToOpen=-1, tsLOOPING=-2, tsBEATING=-3, tsIDLE=0, tsToStep=1, tsToClose=-4,  tsFAILURE=-5,  }; //positive states indicate steps-to-go
struct Signaler;
struct Event;
struct Var_base;
struct Thread;
typedef rai::Array<Signaler*> SignalerL;
typedef rai::Array<Var_base*> VarL;
typedef rai::Array<Thread*> ThreadL;

//===========================================================================

template<class F> struct Callback {
  std::function<F> callback;
  const void* id; //only needed to delete callbacks from callback lists!
  Callback(const void* _id) : id(_id) {}
  Callback(const void* _id, const std::function<F>& c) : callback(c), id(_id) {}
  std::function<F>& call() { CHECK(callback, "is not initialized!!"); return callback; }
};

template<class F>
struct CallbackL : rai::Array<Callback<F>*> {
  void removeCallback(const void* id) {
    uint i;
    for(i=0; i<this->N; i++) if(this->elem(i)->id==id) break;
    delete this->elem(i);
    this->remove(i);
  }
};

//===========================================================================

/// a basic read/write access lock
struct RWLock {
  std::shared_timed_mutex rwLock;
  int rwCount=0;     ///< -1==write locked, positive=numer of readers, 0=unlocked
  rai::Mutex rwCountMutex;
  RWLock();
  ~RWLock();
  void readLock();   ///< multiple threads may request 'lock for read'
  void writeLock();  ///< only one thread may request 'lock for write'
  void unlock();     ///< thread must unlock when they're done
  bool isLocked();
  bool isWriteLocked();
};

//===========================================================================
//
// access gated (rwlocked) variables (shared memory)
//

/// This RW lock counts revisions and broadcasts accesses to listeners; who is accessing can be logged; it has a unique name
struct Var_base : rai::NonCopyable {
  RWLock rwlock;               ///< rwLock (handled via read/writeAccess)
  uint revision=0;
  rai::String name;            ///< name
  double write_time=0.;        ///< clock time of last write access
  double data_time=0.;         ///< time stamp of the original data source
  CallbackL<void(Var_base*)> callbacks;

  Var_base(const char* _name=0);
  /// @name c'tor/d'tor
  virtual ~Var_base();

  void addCallback(const std::function<void(Var_base*)>& call, const void* callbackID);

  /// @name access control
  /// to be called by a thread before access, returns the revision
  int readAccess(Thread* th=nullptr);  //might set the caller to sleep
  int writeAccess(Thread* th=nullptr); //might set the caller to sleep
  int deAccess(Thread* th=nullptr);

  int getRevision() { rwlock.readLock(); int r=revision; rwlock.unlock(); return r; }
};

//===========================================================================

template<class T>
struct RToken {
  Var_base* var;
  T* data;
  Thread* th;
  RToken(Var_base& _var, T* _data, Thread* _th=nullptr, int* getRevision=nullptr, bool isAlreadyLocked=false)
    : var(&_var), data(_data), th(_th) {
    if(!isAlreadyLocked) var->readAccess(th);
    if(getRevision) *getRevision=var->revision;
  }
  ~RToken() { var->deAccess(th); }
  const T* operator->() { return data; }
  operator const T& () { return *data; }
  const T& operator()() { return *data; }
};

template<class T>
struct WToken {
  Var_base* var;
  T* data;
  Thread* th;
  WToken(Var_base& _var, T* _data, Thread* _th=nullptr, int* getRevision=nullptr)
    : var(&_var), data(_data), th(_th) {
    var->writeAccess(_th);
    if(getRevision) *getRevision=var->revision+1;
  }
  WToken(const double& dataTime, Var_base& _var, T* _data, Thread* _th=nullptr, int* getRevision=nullptr)
    : var(&_var), data(_data), th(_th) {
    var->writeAccess(th);
    var->data_time=dataTime;
    if(getRevision) *getRevision=var->revision+1;
  }
  ~WToken() { var->deAccess(th); }
  void operator=(const T& y) { *data=y; }
  T* operator->() { return data; }
  operator T& () { return *data; }
  T& operator()() { return *data; }
};

//===========================================================================

/// A variable is an access gated data field of type T
template<class T>
struct Var_data : Var_base {
  T data;

  Var_data(const char* name=0) : Var_base(name), data() {} // default constructor for value always initializes, also primitive types 'bool' or 'int'
  ~Var_data() {
    if(rwlock.isLocked()) { cerr << "can't destroy a variable when it is currently accessed!" <<endl; exit(1); }
  }
};

template<class T> bool operator==(const Var_data<T>&, const Var_data<T>&) { return false; }
template<class T> void operator<<(ostream& os, const Var_data<T>& v) { os <<"VariableData '" <<v.name <<'\''; }

//===========================================================================
//
/** When using a thread you may declare which variables the
    thread needs access to (for reading or writing). This is done by
    declaring members as 'Var<TYPE> name;' instead of 'TYPE
    name;'. The macro VAR(TYPE, name); makes this simpler. Access
    is the base class of Access.

    This also provides read/write 'tokens' (generated by get()
    and set()) which allow convenient and typed read/write access to
    the variable's content */
template<class T>
struct Var {
  shared_ptr<Var_data<T>> data;
  Thread* thread;             ///< which thread is the owner
  int last_read_revision;     ///< last revision that has been accessed (read or write)

  Var();

  Var(const Var<T>& v) : Var(nullptr, v, false) {}

  /// searches for globally registrated variable 'name', checks type equivalence, and becomes an access for '_thread'
  Var(Thread* _thread, bool threadListens=false);

  /// A "copy" of acc: An access to the same variable as v refers to, but now for '_thread'
  Var(Thread* _thread, const Var<T>& v, bool threadListens=false);

  ~Var();

  //only on construction you can make this Var to refer to the data of another Var -- now it is too late; you can of course call
  //the operator= for the data, using var1.set() = var2.get();
  Var& operator=(const Var& v) { HALT("you can't copy Var!") }

  void checkLocked() { if(!data->rwlock.isLocked()) HALT("direct variable access without locking it before"); }
  T& operator()() { CHECK(data->rwlock.isLocked(), "direct variable access without locking it before");  return data->data; }
  T& operator*() {  CHECK(data->rwlock.isLocked(), "direct variable access without locking it before");  return data->data; }
  T* operator->() { CHECK(data->rwlock.isLocked(), "direct variable access without locking it before");  return &(data->data); }
  RToken<T> get() { return RToken<T>(*data, &data->data, thread, &last_read_revision); } ///< read access to the variable's data
  WToken<T> set() { return WToken<T>(*data, &data->data, thread/*, &last_read_revision*/); } ///< write access to the variable's data
  WToken<T> set(const double& dataTime) { return WToken<T>(dataTime, *data, &data->data, thread/*, &last_read_revision*/); } ///< write access to the variable's data
  operator Var_base& () { return *std::dynamic_pointer_cast<Var_base>(data); }

  void reassignTo(const shared_ptr<Var_data<T>>& _data) {
    data.reset();
    data = _data;
  }

  rai::String& name() const { return data->name; }
  int readAccess() {  return last_read_revision = data->readAccess((Thread*)thread); }
  int writeAccess() { return data->writeAccess((Thread*)thread); }
  int deAccess() {    return data->deAccess((Thread*)thread); }
  int getRevision() { data->rwlock.readLock(); int r=data->revision; data->rwlock.unlock(); return r; }
  bool hasNewRevision() { return getRevision()>last_read_revision; }
  void waitForNextRevision(uint multipleRevisions=0) { waitForRevisionGreaterThan(last_read_revision+multipleRevisions); }
  int waitForRevisionGreaterThan(int rev);
  void waitForValueEq(const T& x) {
    data->waitForEvent([this, &x]()->bool {
      return this->data->data==x;
    });
  }
  void stopListening();

  void addCallback(const std::function<void(Var_base*)>& call, const void* callbackID=0) {
    data->addCallback(call, callbackID);
  }

  void write(ostream& os) {
    readAccess();
    os <<"VAR " <<name() <<" [" <<data->getStatus() <<"] " <<data->data <<endl;
    deAccess();
  }
};

template<class T> std::ostream& operator<<(std::ostream& os, Var<T>& x) { x.write(os); return os; }

//===========================================================================

/// a basic condition variable
struct Signaler {
  int status;
  rai::Mutex statusMutex;
  std::condition_variable cond;

  Signaler(int initialStatus=0);
  virtual ~Signaler(); //virtual, to enforce polymorphism

  void setStatus(int i, Signaler* messenger=nullptr); ///< sets status and broadcasts
  int  incrementStatus(Signaler* messenger=nullptr, int delta=+1);  ///< increase status by 1
  void broadcast(Signaler* messenger=nullptr);        ///< wake up waitForSignal callers

  void statusLock();   //the user can manually lock/unlock, if he needs locked state access for longer -> use userHasLocked=true below!
  void statusUnlock();

  int  getStatus(rai::Mutex::Token* userHasLocked=0) const;
  bool waitForSignal(rai::Mutex::Token* userHasLocked=0, double timeout=-1.);
  bool waitForEvent(std::function<bool()> f, rai::Mutex::Token* userHasLocked=0);
  bool waitForStatusEq(int i, rai::Mutex::Token* userHasLocked=0, double timeout=-1.);    ///< return value is the state after the waiting
  int waitForStatusNotEq(int i, rai::Mutex::Token* userHasLocked=0, double timeout=-1.); ///< return value is the state after the waiting
  int waitForStatusGreaterThan(int i, rai::Mutex::Token* userHasLocked=0, double timeout=-1.); ///< return value is the state after the waiting
  int waitForStatusSmallerThan(int i, rai::Mutex::Token* userHasLocked=0, double timeout=-1.); ///< return value is the state after the waiting
};

//===========================================================================

typedef std::function<int(const rai::Array<Var_base*>&, int whoChanged)> EventFunction;

/// a condition variable that auto-changes status according to a given function of variables
struct Event : Signaler {
  rai::Array<Var_base*> variables; /// variables this event depends on
  EventFunction eventFct;          /// int-valued function that computes status based on variables

  Event(int initialState=0) : Signaler(initialState) {}
  Event(const rai::Array<Var_base*>& _variables, const EventFunction& _eventFct, int initialState=0);
  ~Event();

  void listenTo(Var_base& v);
  template<class T> void listenTo(Var<T>& v) { listenTo(*v.data); }
  void stopListening();
  void stopListenTo(Var_base& c);

  void callback(Var_base* v);
};

template<class T> VarL operator+(shared_ptr<T>& p) { return VarL{p->status.data.get()}; }
template<class T> VarL operator+(VarL A, shared_ptr<T>& p) { A.append(p->status.data.get()); return A; }

int _allPositive(const VarL& signalers, int whoChanged);
enum ActStatus { AS_none=-1, AS_init, AS_running, AS_done, AS_converged, AS_stalled, AS_true, AS_false, AS_kill };

inline bool wait(const VarL& acts, double timeout=-1.) {
  return Event(acts, _allPositive).waitForStatusEq(AS_true, 0, timeout);
}

//===========================================================================
//
// Timing helpers
//

/// a simple struct to realize a strict tic tac timing (called in thread::main once each step if looping)
struct Metronome {
  double ticInterval;
  std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::duration<double>> ticTime;
  uint tics;

  Metronome(double ticIntervalSec); ///< set tic tac time in seconds

  void reset(double ticIntervalSec);
  void waitForTic();              ///< waits until the next tic
  double getTimeSinceTic();       ///< time since last tic
};

//===========================================================================

/// to meassure cycle and busy times
struct CycleTimer {
  typedef std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::duration<double>> timepoint;
  uint steps;
  double busyDt, busyDtMean, busyDtMax;  ///< internal variables to measure step time
  double cyclDt, cyclDtMean, cyclDtMax;  ///< internal variables to measure step time
  timepoint now, lastTime;
  const char* name;                      ///< name
  CycleTimer(const char* _name=nullptr);
  ~CycleTimer();
  void reset();
  void cycleStart();
  void cycleDone();
  rai::String report();
};

//===========================================================================
/**
 * A Thread does some calculation and shares the result via a VariableData.
 *
 * Inherit from the class Thread to create your own process.
 * You need to implement open(), close(), and step().
 * step() should contain the actual calculation.
 *
 * the Signaler indicates the state of the thread: positive=do steps, otherwise it is a ThreadState
 */
struct Thread {
  Event event;
  rai::String name;
  std::unique_ptr<std::thread> thread;    ///< the underlying pthread; nullptr iff not opened
  int tid;                    ///< system thread id
  rai::Mutex stepMutex;              ///< This is set whenever the 'main' is in step (or open, or close) --- use this in all service methods callable from outside!!
  uint step_count;              ///< how often the step was called
  Metronome metronome;          ///< used for beat-looping
  CycleTimer timer;             ///< measure how the time spend per cycle, within step, idle

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
  void threadClose(double timeoutForce=-1.);                   ///< close the thread (stops looping and waits for idle mode before joining the thread)
  void threadStep();                    ///< trigger (multiple) step (idle -> working mode) (wait until idle? otherwise calling during non-idle -> error)
  void threadLoop(bool waitForOpened=false);  ///< loop, either with fixed beat or at full speed
  void threadStop(bool wait=false);     ///< stop looping
  void threadCancel();                  ///< a hard kill (pthread_cancel) of the thread

  void waitForOpened();                 ///< caller waits until opening is done (working -> idle mode)
  void waitForIdle();                   ///< caller waits until step is done (working -> idle mode)
  bool isIdle();                        ///< check if in idle mode
  bool isClosed();                      ///< check if closed

  /** use this to open drivers/devices/files and initialize
   *  parameters; this is called within the thread */
  virtual void open() {}

  /** The most important method of all of this: step does the actual
   *  computation of the thread. Access
   *  the variables by calling the x.get(), x.set() or
   *  x.[read|write|de]Access(), where VAR(TYPE, x) was
   *  declared. */
  virtual void step() { LOG(-1) <<"you're calling the 'pseudo-pure virtual' step(), which should be overloaded (are you in a destructor?)"; }

  /** use this to close drivers/devices/files; this is called within
   *  the thread */
  virtual void close() {}

  void main(); //this is the thread main - should be private!
};

//===========================================================================

struct ScriptThread : Thread {
  std::function<int()> script;
  Var<ActStatus> status;
  ScriptThread(const std::function<int()>& S, Var_base& listenTo)
    :  Thread("ScriptThread"), script(S) {
    event.listenTo(listenTo);
    threadOpen();
  }
  ScriptThread(const std::function<int()>& S, double beatIntervalSec=-1.)
    :  Thread("ScriptThread", beatIntervalSec), script(S) {
    if(beatIntervalSec<0.) threadOpen();
    else threadLoop();
  }
  ~ScriptThread() { threadClose(); }

  virtual void step() { ActStatus r = (ActStatus)script(); status.set()=r; }
};

inline shared_ptr<ScriptThread> run(const std::function<int ()>& script, Var_base& listenTo) {
  return make_shared<ScriptThread>(script, listenTo);
}

inline shared_ptr<ScriptThread> run(const std::function<int ()>& script, double beatIntervalSec) {
  return make_shared<ScriptThread>(script, beatIntervalSec);
}

// ================================================
//
// template definitions
//

template<class T>
Var<T>::Var()
  : data(make_shared<Var_data<T>>()), thread(0), last_read_revision(0) {}

template<class T>
Var<T>::Var(Thread* _thread, bool threadListens)
  : data(make_shared<Var_data<T>>()), thread(_thread), last_read_revision(0) {
  if(thread && threadListens) thread->event.listenTo(*data);
}

template<class T>
Var<T>::Var(Thread* _thread, const Var<T>& v, bool threadListens)
  : data(v.data), thread(_thread), last_read_revision(0) {
  if(thread && threadListens) thread->event.listenTo(*data);
}

template<class T>
Var<T>::~Var() {
//  cout <<data.use_count() <<endl;
}

template<class T>
int Var<T>::waitForRevisionGreaterThan(int rev) {
  EventFunction evFct = [&rev](const rai::Array<Var_base*>& vars, int whoChanged) -> int {
    CHECK_EQ(vars.N, 1, ""); //this event only checks the revision for a single var
    if(vars.elem()->revision > (uint)rev) return 1;
    return 0;
  };

  Event ev({data.get()}, evFct, 0);
  ev.waitForStatusEq(1);
  return data->getRevision();
}

template<class T>
void Var<T>::stopListening() { thread->event.stopListenTo(data); }
