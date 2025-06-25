/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <iosfwd>
#include <typeinfo>
#include <string.h>

//----- if no system flag, I assume Linux
#if !defined RAI_MSVC && !defined RAI_Cygwin && !defined RAI_Linux && !defined RAI_MinGW && !defined RAI_Darwin
#  define RAI_Linux
#endif

//===========================================================================
//
// defines
//

#define RAI_PI 3.14159265358979323846
#define RAI_LN2 0.69314718055994528622676398299518041312694549560546875
#define RAI_2PI 6.283195307179587
#define RAI_LnSqrt2Pi -0.9189385332046727417803296
#define RAI_SQRT2 1.414213562373095049
#define RAI_SQRTPI 1.772453850905516027

#define PTR std::shared_ptr

//===========================================================================
//
// types
//

typedef unsigned char byte;
typedef unsigned int uint;

//===========================================================================
//
// using
//

//using std::cout;
//using std::cerr;
//using std::endl;
using std::ostream;
using std::istream;
using std::ofstream;
using std::ifstream;

namespace rai {

struct String;

//===========================================================================
//
// enums
//

enum ArgWord { _left, _right, _sequence, _path, _xAxis, _yAxis, _zAxis, _xNegAxis, _yNegAxis, _zNegAxis };

//===========================================================================

struct NonCopyable {
  NonCopyable& operator=(const NonCopyable&) = delete;
  NonCopyable(const NonCopyable&) = delete;
  NonCopyable() = default;
};

//===========================================================================
//
// just a hook to make solvers step'able
//

struct Stepper {
  virtual int step() = 0;
  virtual ~Stepper() {}
};

//===========================================================================
//
// logging
//

/// An object that represents a log file and/or cout logging, together with log levels read from a cfg file
struct LogObject {
  std::ofstream* fil=0;
  bool (*callback)(const char*, int) = 0;
  const char* key;
  int logCoutLevel, logFileLevel;
  LogObject(const char* key, int defaultLogCoutLevel=0, int defaultLogFileLevel=0);
  ~LogObject();
  LogObject& getNonConst() const { return *((LogObject*)this); } //ugly... but Logs are often members of classes, and they are accessed in const methods of these classes...
  struct LogToken getToken(int log_level, const char* code_file, const char* code_func, uint code_line);
};

/// A Token to such a Log object which, on destruction, writes into the Log
struct LogToken {
  String* msg=0;
  LogObject& log;
  int log_level;
  const char* code_file, *code_func;
  uint code_line;
  LogToken(LogObject& log, int log_level, const char* code_file, const char* code_func, uint code_line)
    : log(log), log_level(log_level), code_file(code_file), code_func(code_func), code_line(code_line) {}
  ~LogToken(); //that's where the magic happens!
  std::ostream& os();
};

extern LogObject _log;

const char* errString();
String& errStringStream();

//----- parsing strings in a stream
struct PARSE { const char* str; PARSE(const char* _str):str(_str) {} };
std::istream& operator>>(std::istream& is, const PARSE&);

} //namespace

//===========================================================================
//
// macros for LOG and CHECK
//

//----- error handling:
//#define RAI_HERE __FILE__<<':' <<__FUNCTION__ <<':' <<__LINE__ <<' ' //":" <<std::setprecision(5) <<rai::realTime() <<"s "
#define S1(x) #x
#define S2(x) S1(x)
#define __FILENAME__ (::strrchr(__FILE__, '/') ? ::strrchr(__FILE__, '/') + 1 : __FILE__)
#define RAI_HERE __FILE__ ":" S2(__LINE__)
//#define RAI_HERE __FILE__ ## ":" ## #__FUNCTION__ ## ":" ## #__LINE__
#define LOG(log_level) rai::_log.getNonConst().getToken(log_level, __FILENAME__, __func__, __LINE__).os()

#ifndef HALT
#  define RAI_MSG(msg){ LOG(-1) <<msg; }
#  define THROW(msg){ LOG(-2) <<msg; throw std::runtime_error(rai::errString()); }
#  define HALT(msg){ LOG(-2) <<msg; throw std::runtime_error(rai::errString()); }
#  define NIY  { LOG(-2) <<"not implemented yet - HARD EXIT(2)"; exit(2); }
#  define NICO { LOG(-2) <<"not implemented with this compiler options: usually this means that the implementation needs an external library and a corresponding compiler option - see the source code"; throw std::runtime_error(rai::errString()); }
#  define DEPR { LOG(0) <<"this method is deprecated -- please see the code to replace (should be only a rename or one liner)"; }
#endif

//----- check macros:
#ifndef RAI_NOCHECK

#define CHECK(cond, msg) \
  if(!(cond)){ LOG(-2) <<"CHECK failed: '" <<#cond <<"' -- " <<msg;  throw std::runtime_error(rai::errString()); }

#define CHECK_ZERO(expr, tolerance, msg) \
  if((expr)>tolerance || -(expr)>tolerance){ LOG(-2) <<"CHECK_ZERO failed: '" <<#expr<<"'=" <<expr <<" > " <<tolerance <<" -- " <<msg; throw std::runtime_error(rai::errString()); }

#define CHECK_EQ(A, B, msg) \
  if(!(A==B)){ LOG(-2) <<"CHECK_EQ failed: '" <<#A<<"'=" <<A <<" '" <<#B <<"'=" <<B <<" -- " <<msg; throw std::runtime_error(rai::errString()); }

#define CHECK_GE(A, B, msg) \
  if(!(A>=B)){ LOG(-2) <<"CHECK_GE failed: '" <<#A<<"'=" <<A <<" '" <<#B <<"'=" <<B <<" -- " <<msg; throw std::runtime_error(rai::errString()); }

#define CHECK_LE(A, B, msg) \
  if(!(A<=B)){ LOG(-2) <<"CHECK_LE failed: '" <<#A<<"'=" <<A <<" '" <<#B <<"'=" <<B <<" -- " <<msg; throw std::runtime_error(rai::errString()); }

#else
#define CHECK(cond, msg)
#define CHECK_ZERO(expr, tolerance, msg)
#define CHECK_EQ(A, B, msg)
#define CHECK_GE(A, B, msg)
#define CHECK_LE(A, B, msg)
#endif

//===========================================================================
//
// macros to define the standard <<and >>operatos for most classes
//

#define stdInPipe(type)							\
  inline std::istream& operator>>(std::istream& is, type& x){ x.read(is); return is; }
#define stdOutPipe(type)\
  inline std::ostream& operator<<(std::ostream& os, const type& x){ x.write(os); return os; }
#define stdPipes(type)\
  inline std::istream& operator>>(std::istream& is, type& x){ x.read(is); return is; }\
  inline std::ostream& operator<<(std::ostream& os, const type& x){ x.write(os); return os; }
#define inPipe(type)\
  inline type& operator<<(type& x, const char* str){ std::istringstream ss(str); ss >>x; return x; }
#define niyPipes(type)\
  inline std::istream& operator>>(std::istream& is, type& x){ NIY; return is; }\
  inline std::ostream& operator<<(std::ostream& os, const type& x){ NIY; return os; }

ostream& stdCout();
namespace rai {
const char* atomicTypeidName(const std::type_info& type);
}
