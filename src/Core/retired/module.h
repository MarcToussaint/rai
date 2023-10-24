/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef Core_module_h
#define Core_module_h

#include "../Core/array.h"
#include "../Core/graph.h"
#include "../Core/thread.h"

struct Access;
typedef Thread Module;
//struct Module;
typedef rai::Array<Access*> AccessL;
typedef rai::Array<Module*> ModuleL;

//===========================================================================
//
/** This is the core abstraction for users to code modules: derive
    from this class (and perhaps REGISTER_MODULE(...)) to enable the
    engine to instantiate your module and run/schedule it. The
    accesses store all accesses of this module; the engine can
    automatically analyze them and instantiate respective variables if
    necessary */

//struct Module : Thread {
//  Module(const char* name=nullptr, double beatIntervalSec=-1.):Thread(name, beatIntervalSec){
////    registry()->newNode<Module*>({"Module", name}, {}, this);
//  }
//  virtual ~Module(){}
//  virtual void step(){ HALT("you should not run a virtual module"); }
//  virtual void open(){}
//  virtual void close(){}
//};

inline bool operator==(const Module&, const Module&) { return false; }

//===========================================================================
//
/** Instead of declaring 'Var<TYPE> name;' as a module
    member, use the macro VAR(TYPE, name). This is almost
    equivalent, but automatically assigns the name. */

#if 1

#define ACCESSold(type, name)\
  struct __##name##__Access:Var<type>{ \
    __##name##__Access():Var<type>(nullptr, #name){} \
  } name;

#else

#define VAR(type, name) Var<type> name = Var<type>(this, #name);

#endif

#define VAR(type, name) Var<type> name = Var<type>(this, #name);
#define VARlisten(type, name) Var<type> name = Var<type>(this, #name, true);
#define VARname(type, name) Var<type> name = Var<type>(nullptr, #name);

//===========================================================================
//
/** Use this in the cpp-file to register the module. Once it is
    registered it appears in the global registry, no matter if you
    #included its definition. This allows anyone (the engine) to
    instantiate the module just by referring to its string name. */

#define REGISTER_MODULE(name) \
  RUN_ON_INIT_BEGIN(Decl_Module##_##name) \
  registry()->newNode<std::shared_ptr<Type> >({rai::String("Decl_Module"), rai::String(#name)}, NodeL(), make_shared<Type_typed<name, void> >()); \
  RUN_ON_INIT_END(Decl_Module##_##name)

//===========================================================================
//
/** Macros for a most standard declaration of a module */

#define BEGIN_MODULE(name) \
  struct name : Thread { \
    struct s##name *s; \
    name(): Thread(#name), s(nullptr) {} \
    virtual void open(); \
    virtual void step(); \
    virtual void close();

#define END_MODULE() };

#define type name type name;

//===========================================================================
//
// dummy pipe operators
//

inline void operator>>(istream&, Module&) { NIY }
inline void operator<<(ostream& os, const Module& m) { os <<"Module '" <<m.name <<'\''; }

inline void operator>>(istream&, Access&) { NIY }
inline void operator<<(ostream& os, const Access& a) { os <<"Access '" <<a.name <<"' from '" <<(a.thread?a.thread->name:rai::String("NIL")) <<"' to '" << (a.var ? a.data->name : String("??")) <<'\''; }

//===========================================================================
//
// generic recorder module
//

template <class T>
struct Recorder : Thread {
  Var<T> access;
  T buffer;
  ofstream fil;

  Recorder(const char* var_name) : Thread(STRING("Recorder_"<<var_name)), access(this, var_name, true) {}

  void open() {
    rai::open(fil, STRING("z." <<access.name() <<'.' <<rai::getNowString() <<".dat"));
  }
  void step() {
    uint rev = access.readAccess();
    buffer = access();
    double time = access.data->revisionTime();
    access.deAccess();
    rai::String tag;
    tag.resize(30, false);
    sprintf(tag.p, "%6i %13.6f", rev, time);
    fil <<tag <<' ' <<buffer <<endl;
  }
  void close() {
    fil.close();
  }
};

//===========================================================================
//
// file replayer
//

template<class T>
struct FileReplay : Thread {
  Var<T> access;
  T x;
  FileReplay(const char* file_name, const char* var_name, double beatIntervalSec)
    : Thread(STRING("FileReplay_"<<var_name), beatIntervalSec),
      access(this, var_name, false) {
    x <<FILE(file_name);
  }
  void open() {}
  void step() { access.set() = x; }
  void close() {}
};

#endif
