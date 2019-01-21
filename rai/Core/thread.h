/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef RAI_thread_h
#define RAI_thread_h

#include "util.h"
#include "array.h"
#include "graph.h"

enum ThreadState { tsIsClosed=-6, tsToOpen=-2, tsLOOPING=-3, tsBEATING=-4, tsIDLE=0, tsToStep=1, tsToClose=-1,  tsFAILURE=-5,  }; //positive states indicate steps-to-go
struct Signaler;
struct Event;
struct Var_base;
struct Thread;
typedef rai::Array<Signaler*> SignalerL;
typedef rai::Array<Var_base*> VarL;
typedef rai::Array<Thread*> ThreadL;

#ifndef RAI_MSVC

//===========================================================================

template<class F> struct Callback {
  const void* id; //needed to delete callbacks from callback lists!
  std::function<F> callback;
  Callback(const void* _id) : id(_id) {}
  Callback(const void* _id, const std::function<F>& c) : id(_id), callback(c) {}
  std::function<F>& call() { CHECK(callback,"is not initialized!!"); return callback; }
};
template<class F> bool operator==(const Callback<F>& a, const Callback<F>& b) { return a.id==b.id; }

template<class F>
struct CallbackL : rai::Array<Callback<F>*> {
  void removeCallback(const void* id) {
    Callback<F>* c = listFindValue(*this, Callback<F>(id));
    CHECK(c,"");
    this->removeValue(c);
    delete c;
  }
};

//===========================================================================

/// a basic read/write access lock
struct RWLock {
  pthread_rwlock_t rwLock;
  int rwCount=0;     ///< -1==write locked, positive=numer of readers, 0=unlocked
  Mutex rwCountMutex;
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
struct Var_base : NonCopyable {
  RWLock rwlock;               ///< rwLock (handled via read/writeAccess)
  uint revision=0;
  const std::type_info& type;  ///< type of the variable
  const void *value_ptr=0;     ///< pointer to variable data
  rai::String name;            ///< name
  double write_time=0.;        ///< clock time of last write access
  double data_time=0.;         ///< time stamp of the original data source
  CallbackL<void(Var_base*)> callbacks;
  struct Node* registryNode=0; ///< every threading object registers itself globally

  Var_base(const std::type_info& _type, void *_value_ptr, const char* _name=0);
  /// @name c'tor/d'tor
  virtual ~Var_base();

  void addCallback(const std::function<void(Var_base*)>& call, const void* callbackID){
    callbacks.append(new Callback<void(Var_base*)>(callbackID, call));
  }

  /// @name access control
  /// to be called by a thread before access, returns the revision
  int readAccess(Thread* th=NULL);  //might set the caller to sleep
  int writeAccess(Thread* th=NULL); //might set the caller to sleep
  int deAccess(Thread* th=NULL);

  int getRevision() { rwlock.readLock(); int r=revision; rwlock.unlock(); return r; }

  typedef std::shared_ptr<Var_base> Ptr;
};

//===========================================================================

template<class T>
struct RToken {
  Var_base *var;
  T *data;
  Thread *th;
  RToken(Var_base& _var, T* _data, Thread* _th=NULL, int* getRevision=NULL, bool isAlreadyLocked=false)
    : var(&_var), data(_data), th(_th) {
    if(!isAlreadyLocked) var->readAccess(th);
    if(getRevision) *getRevision=var->revision;
  }
  ~RToken(){ var->deAccess(th); }
  const T* operator->() { return data; }
  operator const T&() { return *data; }
  const T& operator()() { return *data; }
};

template<class T>
struct WToken {
  Var_base *var;
  T *data;
  Thread *th;
  WToken(Var_base& _var, T* _data, Thread* _th=NULL, int* getRevision=NULL)
    : var(&_var), data(_data), th(_th) {
    var->writeAccess(_th);
    if(getRevision) *getRevision=var->revision+1;
  }
  WToken(const double& dataTime, Var_base& _var, T* _data, Thread* _th=NULL, int* getRevision=NULL)
    : var(&_var), data(_data), th(_th) {
    var->writeAccess(th);
    var->data_time=dataTime;
    if(getRevision) *getRevision=var->revision+1;
  }
  ~WToken(){ var->deAccess(th); }
  void operator=(const T& y) { *data=y; }
  T* operator->() { return data; }
  operator T&() { return *data; }
  T& operator()() { return *data; }
};

//===========================================================================

/// A variable is an access gated data field of type T
template<class T>
struct Var_data : Var_base {
  T data;

  Var_data(const char* name) : Var_base(typeid(T), &data, name), data() {} // default constructor for value always initializes, also primitive types 'bool' or 'int'
  ~Var_data() { CHECK(!rwlock.isLocked(), "can't destroy a variable when it is currently accessed!"); }
};

template<class T> bool operator==(const Var_data<T>&,const Var_data<T>&) { return false; }
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
  ptr<Var_data<T>> data;
  rai::String name; ///< name; by default the RevLock's name; redefine to a variable's name to autoconnect
  Thread *thread;  ///< which thread is this a member of
  int last_read_revision;     ///< last revision that has been accessed (read or write)
  struct Node* registryNode;

  Var() : Var(NULL, NULL, false) {}

  Var(const Var<T>& v) : Var(NULL, v, false) {}

  Var(const char* name) : Var(NULL, name, false) {}

  Var(Thread* _thread) : Var(_thread, NULL, false) {}

  /// searches for globally registrated variable 'name', checks type equivalence, and becomes an access for '_thread'
  Var(Thread* _thread, const char* name, bool threadListens=false);

  /// A "copy" of acc: An access to the same variable as acc refers to, but now for '_thred'
  Var(Thread* _thread, const Var<T>& acc, bool threadListens=false);

  /// A "copy" of acc: An access to the same variable as acc refers to, but now for '_thred'
  Var(Thread* _thread, const Var_base::Ptr& var, bool threadListens=false);

  ~Var();

  //only on construction you can make this Var to refer to the data of another Var -- now it is too late; you can of course call
  //the operator= for the data, using var1.set() = var2.get();
  Var& operator=(const Var& v) = delete;

  void checkLocked(){ if(!data->rwlock.isLocked()) HALT("direct variable access without locking it before"); }
  T& operator()() { CHECK(data->rwlock.isLocked(),"direct variable access without locking it before");  return data->data; }
  T& operator*() {  CHECK(data->rwlock.isLocked(),"direct variable access without locking it before");  return data->data; }
  T* operator->() { CHECK(data->rwlock.isLocked(),"direct variable access without locking it before");  return &(data->data); }
  RToken<T> get() { return RToken<T>(*data, &data->data, thread, &last_read_revision); } ///< read access to the variable's data
  WToken<T> set() { return WToken<T>(*data, &data->data, thread/*, &last_read_revision*/); } ///< write access to the variable's data
  WToken<T> set(const double& dataTime) { return WToken<T>(dataTime, *data, &data->data, thread/*, &last_read_revision*/); } ///< write access to the variable's data

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

  void addCallback(const std::function<void(Var_base*)>& call, const void* callbackID=0){
    data->addCallback(call, callbackID);
  }


  void write(ostream& os) {
    readAccess();
    os <<"VAR " <<name <<" [" <<data->getStatus() <<"] " <<data->data <<endl;
    deAccess();
  }
};

template<class T> std::ostream& operator<<(std::ostream& os, Var<T>& x) { x.write(os); return os; }

#define VAR(type, name) Var<type> name = Var<type>(this, #name);
#define VARlisten(type, name) Var<type> name = Var<type>(this, #name, true);
#define VARname(type, name) Var<type> name = Var<type>(NULL, #name);

//===========================================================================

/// a basic condition variable
struct Signaler {
  int status;
  Mutex statusMutex;
  pthread_cond_t cond;


//  SignalerL listeners;   ///< list of other condition variables that are being signaled on a setStatus access
//  SignalerL listensTo;   ///< ...
//  SignalerL messengers;  ///< set(!) of condition variables that send signals (via the listen mechanism) - is cleared by the user only
  
  Signaler(int initialStatus=0);
  virtual ~Signaler(); //virtual, to enforce polymorphism
  
  void setStatus(int i, Signaler* messenger=NULL); ///< sets state and broadcasts
  int  incrementStatus(Signaler* messenger=NULL);  ///< increase value by 1
  void broadcast(Signaler* messenger=NULL);        ///< wake up listeners and call callbacks with current status
//  void listenTo(Signaler& c);
//  void stopListenTo(Signaler& c);
//  void stopListening();
  
  void statusLock();   //the user can manually lock/unlock, if he needs locked state access for longer -> use userHasLocked=true below!
  void statusUnlock();
  
  int  getStatus(bool userHasLocked=false) const;
  bool waitForSignal(bool userHasLocked=false, double timeout=-1.);
  bool waitForEvent(std::function<bool()> f, bool userHasLocked=false);
  bool waitForStatusEq(int i, bool userHasLocked=false, double timeout=-1.);    ///< return value is the state after the waiting
  int waitForStatusNotEq(int i, bool userHasLocked=false, double timeout=-1.); ///< return value is the state after the waiting
  int waitForStatusGreaterThan(int i, bool userHasLocked=false, double timeout=-1.); ///< return value is the state after the waiting
  int waitForStatusSmallerThan(int i, bool userHasLocked=false, double timeout=-1.); ///< return value is the state after the waiting
};

//===========================================================================

typedef std::function<int(const rai::Array<Var_base*>&, int whoChanged)> EventFunction;

struct Event : Signaler {
  rai::Array<Var_base*> variables;
  EventFunction eventFct;

  Event(int initialState=0) : Signaler(initialState) {}
  Event(const rai::Array<Var_base*>& _variables, const EventFunction& _eventFct, int initialState=0);
  ~Event();

  void listenTo(Var_base& v);
  template<class T> void listenTo(Var<T>& v) { listenTo(*v.data); }
  void stopListening();
  void stopListenTo(Var_base& c);

  void callback(Var_base *v);

  typedef std::shared_ptr<Event> Ptr;
};

template<class T> VarL operator+(ptr<T>& p){ return ARRAY<Var_base*>(p->status.data.get()); }
template<class T> VarL operator+(VarL A, ptr<T>& p){ A.append(p->status.data.get()); return A; }

int _allPositive(const VarL& signalers, int whoChanged);
enum ActStatus { AS_init=-1, AS_running, AS_done, AS_converged, AS_stalled, AS_true, AS_false, AS_kill };

inline bool wait(const VarL& acts, double timeout=-1.){
  return Event(acts, _allPositive).waitForStatusEq(AS_true, false, timeout);
}

//===========================================================================

//
// Timing helpers
//

/// a simple struct to realize a strict tic tac timing (called in thread::main once each step if looping)
struct Metronome {
  double ticInterval;
  timespec ticTime;
  uint tics;
  
  Metronome(double ticIntervalSec); ///< set tic tac time in seconds
  
  void reset(double ticIntervalSec);
  void waitForTic();              ///< waits until the next tic
  double getTimeSinceTic();       ///< time since last tic
};

//===========================================================================

/// to meassure cycle and busy times
struct CycleTimer {
  uint steps;
  double busyDt, busyDtMean, busyDtMax;  ///< internal variables to measure step time
  double cyclDt, cyclDtMean, cyclDtMax;  ///< internal variables to measure step time
  timespec now, lastTime;
  const char* name;                      ///< name
  CycleTimer(const char *_name=NULL);
  ~CycleTimer();
  void reset();
  void cycleStart();
  void cycleDone();
  rai::String report();
};

//===========================================================================
/**
 * MiniThread might replace Thread some time
 */
struct MiniThread : Signaler {
  rai::String name;
  pthread_t thread = 0;             ///< the underlying pthread; NULL iff not opened
  pid_t tid = 0;                    ///< system thread id
  struct Node* registryNode=0; ///< every threading object registers itself globally

  /// @name c'tor/d'tor
  MiniThread(const char* _name);
  virtual ~MiniThread();
  
  /// @name to be called from `outside' (e.g. the main) to start/step/close the thread
  void threadClose(double timeoutForce=-1.);       ///< close the thread (stops looping and waits for idle mode before joining the thread)
  void threadCancel();                             ///< a hard kill (pthread_cancel) of the thread
  
  virtual void main() { LOG(-1) <<"you're calling the 'pseudo-pure virtual' main(), which should be overloaded (are you in a destructor?)"; }
  
  void pthreadMain(); //this is the thread main - should be private!
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
  pthread_t thread;             ///< the underlying pthread; NULL iff not opened
  pid_t tid;                    ///< system thread id
  Mutex stepMutex;              ///< This is set whenever the 'main' is in step (or open, or close) --- use this in all service methods callable from outside!!
  uint step_count;              ///< how often the step was called
  Metronome metronome;          ///< used for beat-looping
  CycleTimer timer;             ///< measure how the time spend per cycle, within step, idle
  int verbose;
  struct Node* registryNode=0; ///< every threading object registers itself globally

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
  virtual void open(){}
  
  /** The most important method of all of this: step does the actual
   *  computation of the thread. Access
   *  the variables by calling the x.get(), x.set() or
   *  x.[read|write|de]Access(), where VAR(TYPE, x) was
   *  declared. */
  virtual void step() { LOG(-1) <<"you're calling the 'pseudo-pure virtual' step(), which should be overloaded (are you in a destructor?)"; }
  
  /** use this to close drivers/devices/files; this is called within
   *  the thread */
  virtual void close(){}
  
  void main(); //this is the thread main - should be private!
};


//===========================================================================
//
// high-level methods to control threads

Signaler* moduleShutdown();
Var_base::Ptr getVariable(const char* name);
template<class T> Var_data<T>& getVariable(const char* name) {
  Var_base::Ptr v = getVariable(name);
  if(!v) HALT("can't find variable of name '" <<name <<"'");
  ptr<Var_data<T>> var = std::dynamic_pointer_cast<Var_data<T>>(v);
  if(!var) HALT("can't convert variable of type '" <<NAME(v->type) <<"' to '" <<NAME(typeid(T)) <<"'");
  return *var;
}

rai::Array<Var_base::Ptr*> getVariables();

template<class T> rai::Array<ptr<Var<T>>> getVariablesOfType() {
  rai::Array<ptr<Var<T>>> ret;
  rai::Array<Var_base::Ptr*> vars = getVariables();
  for(Var_base::Ptr* v : vars) {
    if((*v)->type==typeid(T)) ret.append(std::make_shared<Var<T>>((Thread*)0, *v));
//    ptr<VariableData<T>> var = std::dynamic_pointer_cast<VariableData<T>>(*v);
//    if(var) ret.append(Var<T>(NULL, var));
  }
  return ret;
}

//template <class T> T& getVariable(const char* name){  return registry()->get<T&>({"VariableData",name});  }
template <class T> T* getThread(const char* name) {  return dynamic_cast<T*>(registry()->get<Thread*>({"Thread",name}));  }
void openModules();
void stepModules();
void closeModules();
void threadOpenModules(bool waitForOpened, bool setSignalHandler=true);
void threadCloseModules();
void threadCancelModules();
void threadReportCycleTimes();

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

#else //RAI_MSVC

struct Signaler {
  int value;
  Signaler(int initialState=0) {}
  ~Signaler() {}
  
  void setStatus(int i, bool signalOnlyFirstInQueue=false) { value=i; }
  int  incrementStatus(bool signalOnlyFirstInQueue=false) { value++; }
  void broadcast(bool signalOnlyFirstInQueue=false) {}
  
  void lock() {}
  void unlock() {}
  
  int  getStatus(bool userHasLocked=false) const { return value; }
};

#endif //RAI_MSVC


template<class T>
Var<T>::Var(Thread* _thread, const char* name, bool threadListens)
  : name(name), thread(_thread), last_read_revision(0), registryNode(NULL) {
  Var_base::Ptr *existing = 0;
  if(name) existing = registry()->find<Var_base::Ptr>({"VariableData", name});
  if(existing) {
    data = std::dynamic_pointer_cast<Var_data<T>>(*existing);
    if(!data) HALT("a previous variable '" <<(*existing)->name <<"' with type '" <<NAME((*existing)->type) <<"' != '" <<NAME(typeid(T)) <<" already existed");
  } else {
    //this is the ONLY place where a variable should be created
    data = make_shared<Var_data<T>>(name);
    data->registryNode = registry()->newNode<Var_base::Ptr>({"VariableData", name}, {}, std::dynamic_pointer_cast<Var_base>(data));
  }
  if(thread) {
    registryNode = registry()->newNode<Var<T>* >({"Access", name}, {thread->registryNode, data->registryNode}, this);
    if(threadListens) thread->event.listenTo(*data);
  } else {
    registryNode = registry()->newNode<Var<T>* >({"Access", name}, {data->registryNode}, this);
  }
}

template<class T>
Var<T>::Var(Thread* _thread, const Var<T>& acc, bool threadListens)
  : data(NULL), name(acc.name), thread(_thread), last_read_revision(0), registryNode(NULL) {
  data = acc.data;
  if(thread) {
    registryNode = registry()->newNode<Var<T>* >({"Access", name}, {thread->registryNode, data->registryNode}, this);
    if(threadListens) thread->event.listenTo(*data);
  } else {
    registryNode = registry()->newNode<Var<T>* >({"Access", name}, {data->registryNode}, this);
  }
}

template<class T>
Var<T>::Var(Thread* _thread, const Var_base::Ptr& var, bool threadListens)
  : data(NULL), name(var->name), thread(_thread), last_read_revision(0), registryNode(NULL) {
  if(var->type!=typeid(T)) HALT("types don't match!");
  data = std::dynamic_pointer_cast<Var_data<T>>(var); //dynamic_cast<VariableData<T>*>(&var));
  if(thread) {
    registryNode = registry()->newNode<Var<T>* >({"Access", name}, {thread->registryNode, data->registryNode}, this);
    if(threadListens) thread->event.listenTo(*data);
  } else {
    registryNode = registry()->newNode<Var<T>* >({"Access", name}, {data->registryNode}, this);
  }
}

template<class T>
Var<T>::~Var() {
//  cout <<data.use_count() <<endl;
  if(registryNode) registry()->delNode(registryNode);
}

template<class T>
int Var<T>::waitForRevisionGreaterThan(int rev) {
#if 0
  return data->waitForStatusGreaterThan(rev);
#else
  EventFunction evFct = [&rev](const rai::Array<Var_base*>& vars, int whoChanged) -> int {
    CHECK_EQ(vars.N, 1, ""); //this event only checks the revision for a single var
    if(vars.scalar()->revision > (uint)rev) return 1;
    return 0;
  };

  Event ev({data.get()}, evFct, 0);
  ev.waitForStatusEq(1);
  return data->getRevision();
#endif
}

//template<class T> Var<T>& Var<T>::operator=(const Var& v){
////  if(data) data.reset();
//  if(registryNode) registry()->delNode(registryNode);
//  data = v.data;
//  name = v.name;
//  thread = v.thread;
//  if(thread) {
//    registryNode = registry()->newNode<Var<T>* >({"Access", name}, {thread->registryNode, data->registryNode}, this);
//  } else {
//    registryNode = registry()->newNode<Var<T>* >({"Access", name}, {data->registryNode}, this);
//  }
//  return *this;
//}

template<class T>
void Var<T>::stopListening() { thread->event.stopListenTo(data); }


#endif
