/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "defines.h"

#include <iostream>
#include <fstream>
#include <typeinfo>
#include <stdint.h>
#include <string.h>
#include <memory>
#include <climits>
#include <mutex>
#include <functional>
#include <random>

using std::cout;
using std::cerr;
using std::endl;

namespace rai {

//===========================================================================
//
// standard little methods (this needs cleanup)
//

extern int argc;
extern char** argv;
extern std::string startDir;
extern uint lineCount;
struct String;

//----- execute a system command
void system(const char* cmd);

//----- files
void open(std::ofstream& fs, const char* name, const char* errmsg="");
void open(std::ifstream& fs, const char* name, const char* errmsg="");
String raiPath(const char* rel=nullptr);
void setRaiPath(const char* path);

//----- very basic ui
int x11_getKey();

//----- strings and streams
bool contains(const char* s, char c);
char skip(std::istream& is, const char* skipSymbols=" \n\r\t", const char* stopSymbols=nullptr, bool skipCommentLines=true);
void skipRestOfLine(std::istream& is);
void skipOne(std::istream& is);
char getNextChar(std::istream& is, const char* skipSymbols=" \n\r\t", bool skipCommentLines=true);
char peerNextChar(std::istream& is, const char* skipSymbols=" \n\r\t", bool skipCommentLines=true);
bool parse(std::istream& is, const char* str, bool silent=false);
bool skipUntil(std::istream& is, const char* tag);

//----- functions
byte bit(byte* str, uint i);
void flip(byte& b, uint i);
void flip(int& b, uint i);
double MIN(double a, double b);
double MAX(double a, double b);
uint MAX(uint a, uint b);
int MAX(int a, int b);
inline void maxEq(double& x, double a) { if(a>x) x=a; }
double indicate(bool expr);
double modMetric(double x, double y, double mod);
double sign(double x);
double sign0(double x);
double linsig(double x);
void   clip(double& x, double a, double b);
//double phi(double dx, double dy);
//double dphi(double x, double y, double dx, double dy);
double DIV(double x, double y, bool force=false);
double sigmoid11(double x);
double sigmoid(double x);
double dsigmoid(double x);
double approxExp(double x);
double Log(double x);
uint   Log2(uint n);
double sqr(double x);
double sinc(double x);
double cosc(double x);
double erf(double x);
double gaussInt(double x);
double gaussIntExpectation(double x);
double NNsdv(const double& a, const double& b, double sdv);
double NNsdv(double x, double sdv);
double forsyth(double x, double a);

//----- time access
double clockTime(); //(really on the clock)
double realTime(); //(since process start)
double cpuTime();
String date(bool forFileName=false);
void wait(double sec, bool andKeyboard=false);
bool wait(bool useX11=true);

//----- timer functions
double timerStart(bool useRealTime=false);
double timerRead(bool reset=false);
double timerRead(bool reset, double startTime);
double timerPause();
void   timerResume();

//----- memory usage
long mem();

//----- command line handling
void initCmdLine(int _argc, char* _argv[], bool quiet=true);
bool checkCmdLineTag(const char* tag);
char* getCmdLineArgument(const char* tag);

String getcwd_string();
const char* niceTypeidName(const std::type_info& type);

//----- get verbose level
bool getInteractivity();
bool getDisableGui();

//===========================================================================
//
// String class
//

#define STRING(x) (((rai::String&)(rai::String().stream() <<x)))
#define STRINGF(format,...) (rai::String().printf(format, __VA_ARGS__))
#define STREAM(x) (((rai::String&)(rai::String().stream() <<x)).stream())

/** @brief String implements the functionalities of an ostream and an
istream, but also can be send to an ostream or read from an
istream. It is based on a simple streambuf derived from the
rai::Mem class */
struct String : public std::iostream {
 private:
  struct StringBuf : std::streambuf {
    String* string;
    virtual int overflow(int C = traits_type::eof());
    virtual int sync();
    void setIpos(char* p);
    char* getIpos();
  } buffer;
  void init();

 public:
  /// @name data fields
  char* p;    ///< pointer to memory
  uint N;     ///< \# elements (excluding zero)
  uint M;     ///< actual buffer size (in terms of # elements)
  static const char* readSkipSymbols; ///< default argument to read method (also called by operator>>)
  static const char* readStopSymbols; ///< default argument to read method (also called by operator>>)
  static int readEatStopSymbol;       ///< default argument to read method (also called by operator>>)
  void (*flushCallback)(String&);

  /// @name constructors
  String();
  String(const String& s);
  String(const char* s);
  explicit String(const std::string& s);
  explicit String(std::istream& is);
  ~String();

  /// @name access
  operator char* ();
  operator const char* () const;
  char& operator()(int i) const;
  std::iostream& stream();            ///< explicitly returns this as an std::iostream&
  String& operator()();               ///< explicitly return this as a (non-const!) String&
  String getSubString(int start, int end) const;
  String getFirstN(uint n) const;
  String getLastN(uint n) const;

  /// @name setting
  String& operator=(const String& s);
  String& operator=(const std::string& s);
  String& operator=(const char* s);
  void set(const char* s, uint n);
  String& printf(const char* format, ...);
  void resize(uint n, bool copy); //low-level resizing the string buffer - with additinal final 0
  void append(char x);
  void prepend(const String& s);
  void replace(uint i, uint n, const char* xp, uint xN);
  void removePrefix(const char* prefix);
  void removePostfix(const char* postfix);

  String& setRandom();

  /// @name resetting
  String& clear();       //as with Array: resize(0)
  String& clearStream(); //call IOstream::clear();
  String& resetIstream();

  /// @name equality
  bool operator==(const char* s) const;
  bool operator==(const String& s) const;
  bool operator!=(const char* s) const;
  bool operator!=(const String& s) const;
  bool operator<=(const String& s) const;

  /// @name misc
  int find(char c, bool reverse) const;
  bool contains(char c) const;
  bool contains(const String& substring) const;
  bool startsWith(const String& substring) const;
  bool startsWith(const char* substring) const;
  bool endsWith(const String& substring) const;
  bool endsWith(const char* substring) const;

  /// @name I/O
  void write(std::ostream& os) const;
  uint read(std::istream& is, const char* skipSymbols=0, const char* stopSymbols=0, int eatStopSymbol=-1);
};
stdPipes(String)

inline String operator+(const String& a, const char* b) { String s=a; s <<b; return s; }
  //template<class T> String operator+(const String& a, const T& b) { String s=a; s <<b; return s; }


void setLogLevels(int fileLogLevel=3, int consoleLogLevel=2);

//The destructor ~LogToken writes into the log file and
//console. setLogLevel allows to adjust cout verbosity (0 by default),
//and what is written into the log file (1 by default)

//===========================================================================
//
// give names to Enum (for pipe << >> )
//

template<class enum_T>
struct Enum {
  enum_T x;
  static const char* names [];
  Enum():x((enum_T)-1) {}
  Enum(enum_T y):x(y) {}
  explicit Enum(const String& str):Enum() { operator=(str); }
  const enum_T& operator=(enum_T y) { x=y; return x; }
  bool operator==(const enum_T& y) const { return x==y; }
  bool operator!=(const enum_T& y) const { return x!=y; }
  operator enum_T() const { return x; }
  void read(std::istream& is) {
    String str(is);
    operator=(str);
  }
  void operator=(const char* str) {
    operator=(STRING(str));
  }
  void operator=(const String& str) {
    bool good=false;
    for(int i=0; names[i]; i++) {
      const char* n = names[i];
      if(!n) THROW("enum_T " <<typeid(enum_T).name() <<' ' <<str <<" out of range")
        if(str==n) { x=(enum_T)(i); good=true; break; }
    }
    if(!good) {
      String all;
      for(int i=0; names[i]; i++) all <<names[i] <<' ';
      HALT("Enum::read could not find the keyword '" <<str <<"'. Possible Enum keywords: " <<all);
    } else {
      CHECK(str.p && !strcmp(names[x], str.p), "");
    }
  }
  static bool contains(const String& str) {
    for(int i=0; names[i]; i++) {
      if(str==names[i]) return true;
    }
    return false;
  }
  static const char* name(int i) {
    return names[i];
  }
  const char* name() const {
    if(x<0) return "init";
    else return names[x];
  }
  void write(std::ostream& os) const { os <<name(); }
};
template<class T> std::istream& operator>>(std::istream& is, Enum<T>& x) { x.read(is); return is; }
template<class T> std::ostream& operator<<(std::ostream& os, const Enum<T>& x) { x.write(os); return os; }

//===========================================================================
//
// parameters
//

//----- parameter grabbing from command line, config file, or default value
template<class T> T getParameter(const char* tag);
template<class T> T getParameter(const char* tag, const T& Default);
template<class T> void getParameter(T& x, const char* tag, const T& Default);
template<class T> void getParameter(T& x, const char* tag);
template<class T> bool checkParameter(const char* tag);

template<class T> void setParameter(const char* key, const T& x);

template<class Tvar, class Tparam> struct ParameterInit {
  ParameterInit(Tvar& x, const char* tag, const Tparam& Default) { x = (Tvar) getParameter<Tparam>(tag, Default); }
};

#define RAI_PARAM(scope, type, name, Default) \
  type name; \
  auto& set_##name(type _##name){ name=_##name; return *this; } \
  rai::ParameterInit<type, type> __init_##name = {name, scope #name, Default};

#define RAI_PARAMt(scope, Tvar, name, Tparam, Default) \
  Tvar name; \
  auto& set_##name(Tvar _##name){ name=_##name; return *this; } \
  rai::ParameterInit<Tvar, Tparam> __init_##name = {name, scope #name, Default};

template<class T> struct ParameterInitEnum {
  ParameterInitEnum(T& x, const char* tag, const T& Default) {
    String str;
    getParameter<String>(str, tag, "");
    if(str.N) x = Enum<T>(str);
    else x = Default;
  }
};
#define RAI_PARAM_ENUM(scope, type, name, Default) \
  type name; \
  auto& set_##name(type _##name){ name=_##name; return *this; } \
  rai::ParameterInitEnum<type> __init_##name = {name, scope #name, Default};

//===========================================================================
//
// Testing
//

#ifndef RAI_REDEFINE_TESTS
#  define TEST(name) test##name()
#  define MAIN main
#elif defined RAI_GTESTS
#  define GTEST_DONT_DEFINE_TEST 1
#  include <gtest/gtest.h>
#  define TEST(name) test##name(){} GTEST_TEST(examples, name)
#  define MAIN \
  main(int argc, char** argv){               \
    rai::initCmdLine(argc,argv);             \
    testing::InitGoogleTest(&argc, argv);  \
    return RUN_ALL_TESTS();      \
  }                                          \
  inline int obsolete_main //this starts a method declaration
#endif

//===========================================================================
//
// FileToken
//

/** @brief A ostream/istream wrapper that allows easier initialization of objects, like:
arr X = FILE("inname");
X >>FILE("outfile");
etc
*/
struct FileToken {
  String path, name, baseDir;
  std::shared_ptr<std::ofstream> os;
  std::shared_ptr<std::ifstream> is;

  FileToken();
  FileToken(const char* _filename);
  FileToken(const FileToken& ft);
  ~FileToken();
  FileToken& operator()() { return *this; }

  void decomposeFilename();
  void cd_base();
  void cd_file();
  bool exists();
  std::ostream& getOs();
  std::istream& getIs();
  operator std::istream& () { return getIs(); }
  operator std::ostream& () { return getOs(); }

  String autoPath() const;
  String relPath() const;
  String fullPath() const;
};
template<class T> FileToken& operator>>(FileToken& fil, T& x) { fil.getIs() >>x;  return fil; }
template<class T> std::ostream& operator<<(FileToken& fil, const T& x) { fil.getOs() <<x;  return fil.getOs(); }
inline std::ostream& operator<<(std::ostream& os, const FileToken& fil) { return os <<fil.name; }
template<class T> FileToken& operator<<(T& x, FileToken& fil) { fil.getIs() >>x; return fil; }
template<class T> void operator>>(const T& x, FileToken& fil) { fil.getOs() <<x; }
inline bool operator==(const FileToken&, const FileToken&) { return false; }

#define FILE(filename) (rai::FileToken(filename)()) //it needs to return a REFERENCE to a local scope object

//===========================================================================
//
// random number generator
//

/** @brief A random number generator. An global instantiation \c
  rnd of a \c Rnd object is created. Use this one object to get
  random numbers.*/
class Rnd {
 private:
  bool ready;
  int32_t rpoint;     /* Feldindex    */
  int32_t rfield[256];   /* Schieberegisterfeld  */
public:
  std::minstd_rand e1;
  std::random_device r;

 public:
  /// ...
  Rnd() { ready=false; }

 public:/// @name initialization
  /// initialize with a specific seed
   void seed(int s){ e1.seed(s); ready=true; }

  /// use Parameter<uint>("seed") as seed
  void seed(){ e1.seed(getParameter<uint32_t>("seed", 0)); ready=true; }

  /// uses the internal clock to generate a seed
  void seed_random(){ e1.seed(r()); ready=true; }

 public:/// @name access
  int uni_int(int lo, int up) { if(!ready) seed(); std::uniform_int_distribution<int> dist(lo, up); return dist(e1); }
  double uni(double lo=0., double up=1.) { if(!ready) seed(); std::uniform_real_distribution<double> dist(lo, up); return dist(e1); }
  double gauss(double mean=0., double std=1.){ if(!ready) seed(); std::normal_distribution<double> dist(mean, std); return dist(e1); }

  uint operator()(uint up) { return uint(uni_int(0, up-1)); }

};

//===========================================================================
//
/// a little inotify wrapper
//

struct Inotify {
  int fd, wd;
  char* buffer;
  uint buffer_size;
  FileToken* fil;
  Inotify(const char* filename);
  ~Inotify();
  bool poll(bool block=false, bool verbose=false);
};

//===========================================================================
//
/// a basic mutex lock
//

struct Mutex {
  std::mutex mutex;
  int state; ///< 0=unlocked, otherwise=syscall(SYS_gettid)
  uint recursive; ///< number of times it's been locked
  const char* lockInfo;
  Mutex();
  ~Mutex();
  void lock(const char* _lockInfo);
  void unlock();

  typedef std::unique_lock<std::mutex> Token;
  Token operator()(const char* _lockInfo) { lockInfo=_lockInfo; return Token(mutex); }

  template<class T> struct TypedToken : Token {
    T* data;
    TypedToken(Mutex& m, T* data, const char* _lockInfo) : Token(m.mutex), data(data) { m.lockInfo=_lockInfo; }
    T* operator->() { return data; }
    operator T& () { return *data; }
    T& operator()() { return *data; }
  };
  template<class T> TypedToken<T> operator()(T* data, const char* _lockInfo) { return TypedToken<T>(*this, data, _lockInfo); }
};

//===========================================================================
//
/// a generic singleton
//

template<class T>
struct Singleton {

  Mutex& getMutex() const {
    static Mutex mutex;
    return mutex;
  }

  T& getSingleton() const {
    static T singleton;
    return singleton;
  }

  Singleton() {}
  Singleton(Singleton const&) = delete;
  void operator=(Singleton const&) = delete;

  ~Singleton() {}

  Mutex::TypedToken<T> operator()() { return getMutex()(&getSingleton(), RAI_HERE); }
};

//===========================================================================
//
// just a hook to make things gl drawable
//

struct OpenGLDrawOptions {
  bool drawWires=false;
  bool drawColors=true;
  bool drawMode_idColor=false;
  bool drawVisualsOnly=false;

  bool drawShapes=true;
  bool drawProxies=true;
  bool drawJoints=false;
  bool drawFrameNames=false;
  bool drawZlines=false;
  bool enableLighting=true;

  float pclPointSize=-1.;
};

//===========================================================================
//
/// a mutexed cout
//

extern Mutex coutMutex;
struct CoutToken {
  CoutToken() { coutMutex.lock(RAI_HERE); }
  ~CoutToken() { coutMutex.unlock(); }
  std::ostream& getOs() { return cout; }
};
#define COUT (rai::CoutToken().getOs())

//===========================================================================
//
// to register a type
//

struct Node;
struct Graph;

struct Type {
  virtual ~Type() {}
  virtual const std::type_info& typeId() const {NIY}
  virtual struct Node* readIntoNewNode(struct Graph& container, std::istream&) const {NIY}
  virtual void* newInstance() const {NIY}
  void write(std::ostream& os) const {  os <<"Type '" <<typeId().name() <<"' ";  }
  void read(std::istream& is) const {NIY}
};
stdPipes(Type)

template<class T>
struct Type_typed : Type {
  virtual const std::type_info& typeId() const { return typeid(T); }
  virtual void* newInstance() const { return new T(); }
};

inline bool operator!=(Type& t1, Type& t2) { return t1.typeId() != t2.typeId(); }
inline bool operator==(Type& t1, Type& t2) { return t1.typeId() == t2.typeId(); }

//===========================================================================
//
// initialization helpers
//

 template<class T> T fromFile(const char* filename) {
   FileToken file(filename);
   T x;
   file.cd_file();
   x.read(file.getIs());
   file.cd_base();
   return x;
 }

// template<class T> T fromString(const char* str) {
//   std::stringstream stream(str);
//   T x;
//   x.read(stream);
//   return x;
// }

} //namespace

//===========================================================================

typedef rai::String str;
extern rai::Rnd rnd;
std::istream& operator>>(std::istream& is, char* str);

//===========================================================================
//
// shared ptrs
//

template<class T> void _delete(T* ptr){}
template<class T> std::shared_ptr<T> _shared(T& x){ return std::shared_ptr<T>(&x, &_delete<T>); }

//===========================================================================
//
// gnuplot calls
//

void gnuplot(const char* command, bool pause=false, bool persist=false, const char* PDFfile=nullptr);
void gnuplotClose();

//===========================================================================
//
/// running code on init (in cpp files)
//

#define RUN_ON_INIT_BEGIN(key) struct key##_RUN_ON_INIT{ key##_RUN_ON_INIT(){
#define RUN_ON_INIT_END(key)   } } key##_RUN_ON_INIT_dummy;

