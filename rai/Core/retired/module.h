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

#ifndef Core_module_h
#define Core_module_h

#include <Core/array.h>
#include <Core/graph.h>
#include <Core/thread.h>

struct Access;
typedef Thread Module;
//struct Module;
typedef mlr::Array<Access*> AccessL;
typedef mlr::Array<Module*> ModuleL;

//===========================================================================
//
/** This is the core abstraction for users to code modules: derive
    from this class (and perhaps REGISTER_MODULE(...)) to enable the
    engine to instantiate your module and run/schedule it. The
    accesses store all accesses of this module; the engine can
    automatically analyze them and instantiate respective variables if
    necessary */

//struct Module : Thread {
//  Module(const char* name=NULL, double beatIntervalSec=-1.):Thread(name, beatIntervalSec){
////    registry()->newNode<Module*>({"Module", name}, {}, this);
//  }
//  virtual ~Module(){}
//  virtual void step(){ HALT("you should not run a virtual module"); }
//  virtual void open(){}
//  virtual void close(){}
//};

inline bool operator==(const Module&,const Module&){ return false; }



//===========================================================================
//
/** Instead of declaring 'Var<TYPE> name;' as a module
    member, use the macro VAR(TYPE, name). This is almost
    equivalent, but automatically assigns the name. */

#if 1

#define ACCESSold(type, name)\
struct __##name##__Access:Var<type>{ \
  __##name##__Access():Var<type>(NULL, #name){} \
} name;

#else

#define VAR(type, name) Var<type> name = Var<type>(this, #name);


#endif

#define VAR(type, name) Var<type> name = Var<type>(this, #name);
#define VARlisten(type, name) Var<type> name = Var<type>(this, #name, true);
#define VARname(type, name) Var<type> name = Var<type>(NULL, #name);

//===========================================================================
//
/** Use this in the cpp-file to register the module. Once it is
    registered it appears in the global registry, no matter if you
    #included its definition. This allows anyone (the engine) to
    instantiate the module just by referring to its string name. */

#define REGISTER_MODULE(name) \
  RUN_ON_INIT_BEGIN(Decl_Module##_##name) \
  registry()->newNode<std::shared_ptr<Type> >({mlr::String("Decl_Module"), mlr::String(#name)}, NodeL(), std::make_shared<Type_typed<name, void> >()); \
  RUN_ON_INIT_END(Decl_Module##_##name)



//===========================================================================
//
/** Macros for a most standard declaration of a module */

#define BEGIN_MODULE(name) \
  struct name : Thread { \
    struct s##name *s; \
    name(): Thread(#name), s(NULL) {} \
    virtual void open(); \
    virtual void step(); \
    virtual void close();

#define END_MODULE() };

#define type name type name;

//===========================================================================
//
// dummy pipe operators
//

inline void operator>>(istream&, Module&){ NIY }
inline void operator<<(ostream& os, const Module& m){ os <<"Module '" <<m.name <<'\''; }

inline void operator>>(istream&, Access&){ NIY }
inline void operator<<(ostream& os, const Access& a){ os <<"Access '" <<a.name <<"' from '" <<(a.thread?a.thread->name:mlr::String("NIL")) <<"' to '" << (a.var ? a.data->name : String("??")) <<'\''; }


//===========================================================================
//
// generic recorder module
//

template <class T>
struct Recorder : Thread {
  Var<T> access;
  T buffer;
  ofstream fil;

  Recorder(const char* var_name) : Thread(STRING("Recorder_"<<var_name)), access(this, var_name, true){}

  void open(){
    mlr::open(fil, STRING("z." <<access.name <<'.' <<mlr::getNowString() <<".dat"));
  }
  void step(){
    uint rev = access.readAccess();
    buffer = access();
    double time = access.data->revisionTime();
    access.deAccess();
    mlr::String tag;
    tag.resize(30, false);
    sprintf(tag.p, "%6i %13.6f", rev, time);
    fil <<tag <<' ' <<buffer <<endl;
  }
  void close(){
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
  void open(){}
  void step(){ access.set() = x; }
  void close(){}
};

#endif
