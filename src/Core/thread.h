#ifndef MT_thread_h
#define MT_thread_h

#include "util.h"
#include "array.h"

enum ThreadState { tsIDLE=0, tsCLOSE=-1, tsOPENING=-2, tsLOOPING=-3, tsBEATING=-4 }; //positive states indicate steps-to-go
struct ConditionVariable;
struct Thread;
typedef MT::Array<ConditionVariable*> ConditionVariableL;
typedef MT::Array<Thread*> ThreadL;

void stop(const ThreadL& P);
void wait(const ThreadL& P);
void close(const ThreadL& P);

//===========================================================================
//
// threading: pthread wrappers: Mutex, RWLock, ConditionVariable
//

#ifndef MT_MSVC

/// a basic mutex lock
struct Mutex {
  pthread_mutex_t mutex;
  int state; ///< 0=unlocked, otherwise=syscall(SYS_gettid)
  Mutex();
  ~Mutex();
  void lock();
  void unlock();
};

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
};

/// a basic condition variable
struct ConditionVariable {
  int value;
  Mutex mutex;
  pthread_cond_t  cond;
  MT::Array<Thread*> listeners;

  ConditionVariable(int initialState=0);
  ~ConditionVariable();

  void setValue(int i, bool signalOnlyFirstInQueue=false); ///< sets state and broadcasts
  int  incrementValue(bool signalOnlyFirstInQueue=false);   ///< increase value by 1
  void broadcast(bool signalOnlyFirstInQueue=false);       ///< just broadcast

  void lock();   //the user can manually lock/unlock, if he needs atomic state access for longer -> use userHasLocked=true below!
  void unlock();

  int  getValue(bool userHasLocked=false) const;
  void waitForSignal(bool userHasLocked=false);
  void waitForSignal(double seconds, bool userHasLocked=false);
  void waitForValueEq(int i, bool userHasLocked=false);    ///< return value is the state after the waiting
  void waitForValueNotEq(int i, bool userHasLocked=false); ///< return value is the state after the waiting
  void waitForValueGreaterThan(int i, bool userHasLocked=false); ///< return value is the state after the waiting
  void waitForValueSmallerThan(int i, bool userHasLocked=false); ///< return value is the state after the waiting
  void waitUntil(double absTime, bool userHasLocked=false);
};

/// a generic singleton
template<class T>
struct Singleton {
  struct SingletonFields { //class cannot have own members: everything in the singleton which is created on first demand
    T obj;
    RWLock lock;
  };

  static SingletonFields *singleton;

  SingletonFields& getSingleton() const {
    static bool currentlyCreating=false;
    if(currentlyCreating) return *((SingletonFields*) NULL);
    if(!singleton) {
      static Mutex m;
      m.lock();
      if(!singleton) {
        currentlyCreating=true;
        singleton = new SingletonFields();
        currentlyCreating=false;
      }
      m.unlock();
    }
    return *singleton;
  }

  T& obj() { return getSingleton().obj; }
};
template<class T> typename Singleton<T>::SingletonFields *Singleton<T>::singleton=NULL;


//===========================================================================
//
// Timing helpers
//

/// a simple struct to realize a strict tic tac timing (call step() once in a loop)
struct Metronome {
  double ticInterval;
  timespec ticTime;
  uint tics;
  const char* name;                   ///< name

  Metronome(const char* name, double ticIntervalSec); ///< set tic tac time in micro seconds
  ~Metronome();

  void reset(double ticIntervalSec);
  void waitForTic();              ///< waits until the next tic
  double getTimeSinceTic();       ///< time since last tic
};

/// a really simple thing to meassure cycle and busy times
struct CycleTimer {
  uint steps;
  double cyclDt, cyclDtMean, cyclDtMax;  ///< internal variables to measure step time
  double busyDt, busyDtMean, busyDtMax;  ///< internal variables to measure step time
  timespec now, lastTime;
  const char* name;                    ///< name
  CycleTimer(const char *_name=NULL);
  ~CycleTimer();
  void reset();
  void cycleStart();
  void cycleDone();
};

#else //MT_MSVC

struct Mutex {
  int state;
  Mutex() {};
  ~Mutex() {};
  void lock() { MT_MSG("fake MSVC Mutex"); }
  void unlock() { MT_MSG("fake MSVC Mutex"); }
};

struct ConditionVariable {
  int value;
  ConditionVariable(int initialState=0) {}
  ~ConditionVariable() {}

  void setValue(int i, bool signalOnlyFirstInQueue=false) {} ///< sets state and broadcasts
  int  incrementValue(bool signalOnlyFirstInQueue=false) {}  ///< increase value by 1
  void broadcast(bool signalOnlyFirstInQueue=false) {}      ///< just broadcast

  void lock() {}
  void unlock() {}

};
#endif //MT_MSVC

//===========================================================================
//
// implementations
//

//#if defined MT_IMPLEMENTATION | defined MT_IMPLEMENT_TEMPLATES
#  include "util_t.h"
//#endif


//===========================================================================
/**
 * A Thread does some calculation and shares the result via a Variable.
 *
 * Inherit from the class Thread to create your own variable.
 * You need to implement open(), close(), and step().
 * step() should contain the actual calculation.
 */
struct Thread{
  MT::String name;
  ConditionVariable state;       ///< the condition variable indicates the state of the thread: positive=steps-to-go, otherwise it is a ThreadState
  ConditionVariableL listensTo;
  //ParameterL dependsOn;
  pid_t tid;                     ///< system thread id
#ifndef MT_QT
  pthread_t thread;
#else
  struct sThread *thread;
#endif
  uint step_count;
  Metronome *metronome;          ///< used for beat-looping

  /// @name c'tor/d'tor
  Thread(const char* name);
  virtual ~Thread();

  /// @name to be called from `outside' (e.g. the main) to start/step/close the thread
  void threadOpen(int priority=0);      ///< start the thread (in idle mode) (should be positive for changes)
  void threadClose();                   ///< close the thread (stops looping and waits for idle mode before joining the thread)
  void threadStep(uint steps=1, bool wait=false);     ///< trigger (multiple) step (idle -> working mode) (wait until idle? otherwise calling during non-idle -> error)
  void threadLoop();                    ///< loop, stepping forever
  void threadLoopWithBeat(double sec);  ///< loop with a fixed beat (cycle time)
  void threadStop();                    ///< stop looping

  void waitForIdle();                   ///< caller waits until step is done (working -> idle mode)
  bool isIdle();                        ///< check if in idle mode
  bool isClosed();                      ///< check if closed

  /// @name listen to variable
  void listenTo(ConditionVariable& var);
  void listenTo(const ConditionVariableL &signalingVars);
  void stopListeningTo(ConditionVariable& var);

  virtual void open() = 0;
  virtual void step() = 0;
  virtual void close() = 0;

  void main(); //this is the thread main - should be private!
};

/// a basic thread
/* struct Thread { */
/*   Thread(); */
/*   ~Thread(); */
/*   void open(const char* name=NULL); */
/*   void close(); */
/*   virtual void main() = 0; */
/* }; */

#endif
