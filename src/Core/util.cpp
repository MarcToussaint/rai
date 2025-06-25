/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "util.h"

#include <math.h>
#include <string>
#include <sstream>
#include <signal.h>
#include <stdexcept>
#include <stdarg.h>
#include <iomanip>
#if defined RAI_Linux || defined RAI_Cygwin || defined RAI_Darwin
#include <chrono>
#include <ctime>
#include <thread>
#  include <limits.h>
#  include <sys/resource.h>
#  include <sys/inotify.h>
#  include <sys/stat.h>
#  include <poll.h>
#  include <execinfo.h>
#  include <cxxabi.h>    // for __cxa_demangle
#if defined RAI_X11
#  include <X11/Xlib.h>
#  include <X11/Xutil.h>
#  undef Success
#endif
#endif
#ifdef __CYGWIN__
#include "cygwin_compat.h"
#endif
#if defined RAI_MSVC
#  include <windows.h>
#  include <direct.h>
#  undef min
#  undef max
#  define chdir _chdir
#  define getpid _getpid
#  ifdef RAI_QT
#    undef  NOUNICODE
#    define NOUNICODE
#  endif
#  pragma warning(disable: 4305 4244 4250 4355 4786 4996)
#endif
#if defined RAI_MinGW
#  include <unistd.h>
#endif

#ifdef RAI_QT
#  undef scroll
#  undef Unsorted
#  include <Qt/qmetatype.h>
#  include <Qt/qdatastream.h>
#  include <Qt/qapplication.h>
#  include <QThread>
#endif

#ifndef RAI_ConfigFileName
#  define RAI_ConfigFileName "rai.cfg"
#  define RAI_LogFileName "rai.log"
#endif

#include <errno.h>
#ifndef RAI_MSVC
#  include <unistd.h>
#  include <sys/syscall.h>
#endif


rai::Rnd rnd;

namespace rai {

//===========================================================================

const char* String::readSkipSymbols = " \t";
const char* String::readStopSymbols = ",\n\r";
int   String::readEatStopSymbol     = 1;
Mutex coutMutex;
LogObject _log("global", 2, 3);

//===========================================================================

template<> const char* Enum<ArgWord>::names []= {
  "left", "right", "sequence", "path", "xAxis", "yAxis", "zAxis", "xNegAxis", "yNegAxis", "zNegAxis", 0
};

//===========================================================================
//
// utilities in rai namespace
//

int argc;
char** argv;
bool IOraw=false;
bool noLog=true;
uint lineCount=1;
int verboseLevel=-1;
double timerStartTime=0.;
double timerPauseTime=-1.;
bool timerUseRealTime=false;

#ifdef RAI_QT
QApplication* myApp=nullptr;
#endif

struct ProcessInfo {
  std::string raiPath;
  std::string startDir;
  std::chrono::system_clock::time_point startTime;

  ProcessInfo() {
    raiPath = RAI_ROOT_PATH;
    startDir = getcwd_string();
    timerStartTime = cpuTime();
    startTime = std::chrono::system_clock::now();
  }

  ~ProcessInfo() {
  }
};

Singleton<ProcessInfo> processInfo;

/// running a system command and checking return value
void system(const char* cmd) {
  //cout <<"SYSTEM CMD: " <<cmd <<endl;
  int r = ::system(cmd);
  wait(.1);
  if(r) LOG(-1) <<"system return error " <<r <<" for command '" <<cmd <<"'";
}

/// open an output-file with name '\c name'
void open(std::ofstream& fs, const char* name, const char* errmsg) {
  fs.clear();
  fs.open(name);
  LOG(3) <<"opening output file '" <<name <<"'";
  if(!fs.good()) RAI_MSG("could not open file '" <<name <<"' for output" <<errmsg);
}

/// open an input-file with name '\c name'
void open(std::ifstream& fs, const char* name, const char* errmsg) {
  fs.clear();
  fs.open(name);
  LOG(3) <<"opening input file '" <<name <<"'";
  if(!fs.good()) HALT("could not open file '" <<name <<"' for input" <<errmsg);
}

/// returns true if the (0-terminated) string s contains c
bool contains(const char* s, char c) {
  if(!s) return false;
  for(uint i=0; s[i]; i++) if(s[i]==c) return true;
  return false;
}

/// skips the chars (typically white characters) when parsing from the istream, returns first non-skipped char
char skip(std::istream& is, const char* skipSymbols, const char* stopSymbols, bool skipCommentLines) {
  char c;
  for(;;) {
    c=is.get();
    if(is.eof()) return '\255';
    if(skipCommentLines && c=='#') { skipRestOfLine(is); continue; }
    if(skipSymbols && !contains(skipSymbols, c)) break;
    if(stopSymbols && contains(stopSymbols, c)) break;
    if(c=='\n') lineCount++;
  }
  is.putback(c);
  return c;
}

/// skips a newline character (same as skip(is, "\n");)
void skipRestOfLine(std::istream& is) {
  char c;
  do { c=is.get(); } while(c!='\n' && is.good());
  if(c=='\n') is.putback(c);
}

/// skips the next character
void skipOne(std::istream& is) {
  is.get();
}

/// tell you about the next char (after skip()) but puts it back in the stream
char getNextChar(std::istream& is, const char* skipSymbols, bool skipCommentLines) {
  char c;
  if(skipSymbols) skip(is, skipSymbols, nullptr, skipCommentLines);
  is.get(c);
  if(!is.good()) return 0;
  return c;
}

/// tell you about the next char (after skip()) but puts it back in the stream
char peerNextChar(std::istream& is, const char* skipSymbols, bool skipCommentLines) {
  char c=getNextChar(is, skipSymbols, skipCommentLines);
  if(!is.good()) return 0;
  is.putback(c);
  return c;
}

/// skip throught a stream until the tag is found (which is eaten)
bool skipUntil(std::istream& is, const char* tag) {
  unsigned n=strlen(tag);
  char* buf=new char [n+1];
  memset(buf, 0, n+1);
  while(is.good()) {
    memmove(buf, buf+1, n);
    buf[n-1]=is.get();
    if(buf[n-1]=='\n') lineCount++;
    buf[n]=0;
    if(!strcmp(tag, buf)) { delete[] buf; return true; }
  };
  delete[] buf;
  return false;
}

/// a global operator to scan (parse) strings from a stream
bool parse(std::istream& is, const char* str, bool silent) {
  if(!is.good()) { if(!silent) RAI_MSG("bad stream tag when scanning for '" <<str <<"'"); return false; }  //is.clear(); }
  uint i, n=strlen(str);
  char* buf = new char[n+1];
  buf[n]=0;
  skip(is, " \n\r\t");
  is.read(buf, n);
  if(!is.good() || strcmp(str, buf)) {
    for(i=n; i--;) is.putback(buf[i]);
    is.setstate(std::ios::failbit);
    if(!silent)  RAI_MSG("(LINE=" <<lineCount <<") parsing of constant string '" <<str
                           <<"' failed! (read instead: '" <<buf <<"')");
    delete[] buf;
    return false;
  }
  delete[] buf;
  return true;
}

/// returns the i-th of str
byte bit(byte* str, uint i) { return (str[i>>3] >>(7-(i&7))) & 1; }
//byte bit(byte b, uint i){ return (b >>(7-(i&7))) & 1; }
//void set(byte *state, uint i){ state[i>>3] |= 1 <<(7-(i&7)); }
//void del(byte *state, uint i){ state[i>>3] &= (byte)~(1 <<(7-(i&7))); }
//void flip(byte *str, uint i){ str[i>>3] ^= 1 <<(7-(i&7)); }

/// flips the i-th bit of b
void flip(byte& b, uint i) { b ^= 1 <<(7-(i&7)); }

/// filps the i-th bit of b
void flip(int& b, uint i) { b ^= 1 <<(7-(i&7)); }

double MIN(double a, double b) { return a<b?a:b; }
double MAX(double a, double b) { return a>b?a:b; }
uint MAX(uint a, uint b) { return a>b?a:b; }
int MAX(int a, int b) { return a>b?a:b; }

double indicate(bool expr) { if(expr) return 1.; return 0.; }

/** @brief the distance between x and y w.r.t.\ a circular topology
    (e.g. modMetric(1, 8, 10)=3) */
double modMetric(double x, double y, double mod) {
  double d=fabs(x-y);
  d=fmod(d, mod);
  if(d>mod/2.) d=mod-d;
  return d;
}

/// the sign (+/-1) of x (+1 for zero)
double sign(double x) { if(x<0.) return -1.; return 1.; }

/// the sign (+/-1) of x (0 for zero)
double sign0(double x) { if(x<0.) return -1.; if(!x) return 0.; return 1.; }

/// returns 0 for x<0, 1 for x>1, x for 0<x<1
double linsig(double x) { if(x<0.) return 0.; if(x>1.) return 1.; return x; }

/// x ends up in the interval [a, b]
void clip(double& x, double a, double b) { if(x<a) x=a; else if(x>b) x=b; }

///// the angle of the vector (x, y) in [-pi, pi]
//double phi(double x, double y) {
//  if(x==0. || ::fabs(x)<1e-10) { if(y>0.) return RAI_PI/2.; else return -RAI_PI/2.; }
//  double p=::atan(y/x);
//  if(x<0.) { if(y<0.) p-=RAI_PI; else p+=RAI_PI; }
//  if(p>RAI_PI)  p-=2.*RAI_PI;
//  if(p<-RAI_PI) p+=2.*RAI_PI;
//  return p;
//}

///// the change of angle of the vector (x, y) when displaced by (dx, dy)
//double dphi(double x, double y, double dx, double dy) {
//  //return (dy*x - dx*y)/sqrt(x*x+y*y);
//  if(x==0. || ::fabs(x)<1e-10) { if(y>0.) return -dx/y; else return dx/y; }
//  double f=y/x;
//  return 1./(1.+f*f)*(dy/x - f/x*dx);
//}

/** @brief save division, checks for division by zero; force=true will return
  zero if y=0 */
double DIV(double x, double y, bool force) {
  if(x==0.) return 0.;
  if(force) { if(y==0.) return 0.; } else CHECK(y!=0, "Division by Zero!");
  return x/y;
}

double sigmoid11(double x) {
  return x/(1.+::fabs(x));
}

double sigmoid(double x) {
  return 1./(1.+exp(-x));
}

double dsigmoid(double x) {
  double y=sigmoid(x);
  return y*(1.-y);
}

#define AXETS 1280
#define AXETR 10.
/// approximate exp (sets up a static value table)
double approxExp(double x) {
  static bool initialized=false;
  static double ExpTable [AXETS]; //table ranges from x=-10 to x=10
  int i;
  if(!initialized) {
    for(i=0; i<AXETS; i++) ExpTable[i]=::exp(AXETR*(2*i-AXETS)/AXETS);
    initialized=true;
  }
  x*=.5*AXETS/AXETR;
  i=(int)x;
  x-=(double)i; //x = residual
  i+=AXETS/2; //zero offset
  if(i>=AXETS-1) return ExpTable[AXETS-1];
  if(i<=0) return 0.; //ExpTable[0];
  return (1.-x)*ExpTable[i] + x*ExpTable[i+1];
}

/// ordinary Log, but cutting off for small values
double Log(double x) {
  if(x<.001) x=.001;
  return ::log(x);
}

/// integer log2
uint Log2(uint n) {
  uint l=0;
  n=n>>1;
  while(n) { l++; n=n>>1; }
  return l;
}

/// square of a double
double sqr(double x) { return x*x; }

double sinc(double x) {
  if(fabs(x)<1e-10) return 1.-.167*x*x;
  return ::sin(x)/x;
}

double cosc(double x) {
  if(fabs(x)<1e-10) return 1.-.167*x*x;
  return ::cos(x)/x;
}

#define EXP ::exp //approxExp

double NNsdv(const double& a, const double& b, double sdv) {
  double d=(a-b)/sdv;
  double norm = 1./(::sqrt(RAI_2PI)*sdv);
  return norm*EXP(-.5*d*d);
}

double NNsdv(double x, double sdv) {
  x/=sdv;
  double norm = 1./(::sqrt(RAI_2PI)*sdv);
  return norm*EXP(-.5*x*x);
}

/** @brief double time on the wall clock (probably counted from decades back)
  (probably in micro second resolution) -- Windows checked! */
double clockTime() {
  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  std::chrono::duration<double> duration = now.time_since_epoch();
  return duration.count();
}

/** @brief double time since start of the process in floating-point seconds
  (probably in micro second resolution) -- Windows checked! */
double realTime() {
  std::chrono::duration<double> duration = (std::chrono::system_clock::now() - processInfo()->startTime);
  return duration.count();
}

/** @brief user CPU time of this process in floating-point seconds (pure
  processor time) -- Windows checked! */
double cpuTime() {
  return std::clock() / (double)CLOCKS_PER_SEC;
}

String date(const std::chrono::system_clock::time_point& t, bool forFileName) {
  auto in_time_t = std::chrono::system_clock::to_time_t(t);

  auto msec = std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch());
  msec = msec % 1000000;

  String ss;
  if(!forFileName) {
    ss <<std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X:");
    ss <<std::setfill('0') <<std::setw(3) <<msec.count();
  } else {
    ss << std::put_time(std::localtime(&in_time_t), "%y-%m-%d--%H-%M-%S");
  }
  return ss;
}

/// the absolute double time and date as string
String date(bool forFileName) {
  return date(std::chrono::system_clock::now(), forFileName);
}

/// wait double time
void wait(double sec, bool andKeyboard) {
  std::this_thread::sleep_for(std::chrono::duration<double>(sec));
  if(andKeyboard) wait();
}

/// wait for an ENTER at the console
bool wait(bool useX11) {
  if(!getInteractivity()) {
    wait(.1);
    return true;
  }
  if(!useX11) {
    char c[10];
    cout <<" -- hit a key to continue..." <<std::flush;
    //cbreak(); getch();
    std::cin.getline(c, 10);
    cout <<"\r" <<std::flush;
    if(c[0]==' ') return true;
    else return false;
    return true;
  } else {
    char c = x11_getKey();
    if(c==' ') return true;
    return false;
  }
}

#ifdef RAI_X11
int x11_getKey() {
  wait(.05);
  String txt="PRESS KEY";
  int key=0;

  Display* disp = XOpenDisplay(nullptr);
  CHECK(disp, "Cannot open display");

  Window win = XCreateSimpleWindow(disp, DefaultRootWindow(disp),
                                   10, 10, 80, 50, //24
                                   2, 0x000000, 0x20a0f0);
  XSelectInput(disp, win, KeyPressMask | ExposureMask | ButtonPressMask);
  XMapWindow(disp, win);

  GC gc = XCreateGC(disp, win, 0, nullptr);
  XSetFont(disp, gc,  XLoadFont(disp, "fixed")); //-adobe-courier-bold-r-*-*-*-220-*-*-*-*-*-*"));
  XSetForeground(disp, gc, 0x000000);

  bool quit=false;
  for(; !quit;) {
    XEvent ev;
    XNextEvent(disp, &ev);
    switch(ev.type) {
      case Expose:
        if(ev.xexpose.count == 0) {
          XDrawString(disp, win, gc, 12, 30, txt.p, txt.N);
          XFlush(disp);
        }
        break;
      case KeyPress:
        char string[4];
        XLookupString(&ev.xkey, string, 4, nullptr, nullptr);
        key = string[0];
//        LOG(0) <<"key: " <<key;
        if(key==' ' || key=='q' || key==27 || key==13)
          quit=true;
        break;
      case ButtonPress:
        quit=true;
        break;
    }
  }

  XFreeGC(disp, gc);
  XCloseDisplay(disp);
  return key;
}
#else
int x11_getKey() {
  LOG(-1) <<"fake implementation (no X11)";
  return 13;
}
#endif

/// the integral shared memory size -- not implemented for Windows!
long mem() {
#ifndef RAI_MSVC
  static rusage r; getrusage(RUSAGE_SELF, &r);
  return r.ru_idrss;
#else
  NICO
#endif
}

/// start and reset the timer (user CPU time)
double timerStart(bool useRealTime) {
  if(useRealTime) timerUseRealTime=true; else timerUseRealTime=false;
  timerPauseTime=-1.;
  timerStartTime=(timerUseRealTime?realTime():cpuTime());
  return timerStartTime;
}

/// read the timer and optionally also reset it (user CPU time)
double timerRead(bool reset) {
  double c;
  if(timerPauseTime!=-1.) c=timerPauseTime; else c=(timerUseRealTime?realTime():cpuTime())-timerStartTime;
  if(reset) timerStart(timerUseRealTime);
  return c;
}

/// read the timer relative to a given start time (user CPU time)
double timerRead(bool reset, double startTime) {
  double c=(timerUseRealTime?realTime():cpuTime())-startTime;
  if(reset) timerStart(timerUseRealTime);
  return c;
}

/// read and pause the timer
double timerPause() {
  timerPauseTime=(timerUseRealTime?realTime():cpuTime())-timerStartTime;
  return timerPauseTime;
}

/// resume the timer after a pause, neglecting the time inbetween
void timerResume() {
  timerStartTime=(timerUseRealTime?realTime():cpuTime())-timerPauseTime;
  timerPauseTime=-1.;
}

//COPY & PAST from graph.h
void initParameters(int _argc, char* _argv[], bool forceReload, bool verbose);
String getParamsDump();

/// memorize the command line arguments and open a log file
void initCmdLine(int _argc, char* _argv[], bool quiet) {
  argc=_argc; argv=_argv;
  for(int i=0; i<argc; i++) if(!strcmp(argv[i], "-quiet")) quiet=true;

  if(!quiet) {
    String msg;
    msg <<"** cmd line arguments: '"; for(int i=0; i<argc; i++) msg <<argv[i] <<' ';
    msg <<"'";
    LOG(1) <<msg;
    LOG(1) <<"** run path: '" <<processInfo()->startDir <<"'";
    LOG(1) <<"** rai path: '" <<processInfo()->raiPath <<"'";
  }

  initParameters(argc, argv, false, !quiet);

  if(checkParameter<String>("raiPath")) setRaiPath(getParameter<String>("raiPath"));
}

/// returns true if the tag was found on command line
bool checkCmdLineTag(const char* tag) {
  for(int n=1; n<argc; n++) if(argv[n][0]=='-' && !strcmp(tag, argv[n]+1)) {
      return true;
    }
  return false;
}

/// returns the argument after the cmd-line tag; nullptr if the tag is not found
char* getCmdLineArgument(const char* tag) {
  int n;
  for(n=1; n<argc; n++) if(argv[n][0]=='-' && !strcmp(tag, argv[n]+1)) {
      if(n+1==argc) return (char*)"1";
      return argv[n+1];
    }
  return nullptr;
}

String raiPath(const char* rel) {
  String path(processInfo()->raiPath);
  path <<"/" <<rel;
  return path;
}

void setRaiPath(const char* path) {
  processInfo()->raiPath = path;
}

bool getInteractivity() {
  static int interactivity=-1;
  if(interactivity==-1) interactivity=(checkParameter<bool>("noInteractivity")?0:1);
  return interactivity==1;
}

bool getDisableGui() {
  static int _disableGui = -1;
  if(_disableGui==-1) _disableGui=(checkParameter<bool>("disableGui")?1:0);
  return _disableGui==1;
}

double forsyth(double x, double a) {
  return x*x/(a*a+x*x);
}

//===========================================================================
//
// logging

LogObject::LogObject(const char* key, int defaultLogCoutLevel, int defaultLogFileLevel)
  : key(key), logCoutLevel(defaultLogCoutLevel), logFileLevel(defaultLogFileLevel) {
  processInfo.getSingleton(); //just to ensure it was created
  if(!strcmp(key, "global")) {
    if(!fil) fil=new ofstream;
    (*fil).open("z.log.global");
    (*fil) <<"** compiled at:     " <<__DATE__ <<" " <<__TIME__ <<'\n';
    (*fil) <<"** execution start: " <<date(processInfo()->startTime, false) <<endl;
  } else {
    logCoutLevel = getParameter<int>(STRING("logCoutLevel_"<<key), logCoutLevel);
    logFileLevel = getParameter<int>(STRING("logFileLevel_"<<key), logFileLevel);
  }
}

LogObject::~LogObject() {
  if(!strcmp(key, "global")) {
    (*fil) <<"** execution stop: " <<date()
           <<"\n** real time: " <<realTime()
           <<"sec\n** CPU time: " <<cpuTime() <<endl;
    (*fil) <<"\n** set+queried params:\n" <<getParamsDump();
  }
  if(fil) {
    (*fil).close();
    delete fil;
  }
}

LogToken LogObject::getToken(int log_level, const char* code_file, const char* code_func, uint code_line) {
  return LogToken(*this, log_level, code_file, code_func, code_line);
}

LogToken::~LogToken() {
  auto mut = processInfo(); //keep the mutex
  if(log.logFileLevel>=log_level) {
    if(!log.fil) log.fil = new ofstream;
    if(!log.fil->is_open()) log.fil->open(STRING("z.log."<<log.key));
    (*log.fil) <<code_file <<':' <<code_func <<':' <<code_line <<'(' <<log_level <<") " <<(*msg) <<endl;
  }
  if(log.logCoutLevel>=log_level) {
    errStringStream().clear();
    errStringStream() <<code_file <<':' <<code_func <<':' <<code_line <<'(' <<log_level <<") " <<(*msg);
    if(msg) { delete msg; msg=0; }
    bool useCout=true;
    if(log.callback) useCout = log.callback(errString(), log_level);
    if(log_level>=0) {
      if(useCout) cout <<"-- " <<errString() <<endl;
      return;
    } else {

#ifndef RAI_MSVC
      if(log_level<=-2) {
        void* callstack[10];
        int stack_count = backtrace(callstack, 10);
        char** symbols = backtrace_symbols(callstack, stack_count);
        for(int i=stack_count; i--;)  {
          char* beg = symbols[i];
          while(*beg!='(') beg++;
          beg++;
          char* end=beg;
          while(*end!='+') end++;
          if(beg!=end) {
            *end=0;
            char* demangled = NULL;
            int status;
            demangled = abi::__cxa_demangle(beg, NULL, 0, &status);
            if(demangled) {
              cout <<"STACK" <<i <<' ' <<demangled <<'\n';
              free(demangled);
            } else {
              cout <<"STACK" <<i <<' ' <<symbols[i] <<'\n';
            }
          } else {
            cout <<"STACK" <<i <<' ' <<symbols[i] <<'\n';
          }
        }
        free(symbols);
      }
#endif

      if(log_level==-1) { if(useCout) cout <<"-- WARNING:" <<errString() <<endl; return; }
      else if(log_level==-2) { if(useCout) cerr <<"== ERROR:" <<errString() <<endl; /*throw does not WORK!!! Because this is a destructor. The THROW macro does it inline*/ }
      //INSERT BREAKPOINT HERE (or before and after this line)
      else if(log_level<=-3) { if(useCout) cerr <<"== HARD EXIT! " <<errString() <<endl;  exit(1); }
//      if(log_level<=-3) raise(SIGABRT);
    }
  }
  if(msg) { delete msg; msg=0; }
//  logServer().mutex.unlock();
}

void setLogLevels(int fileLogLevel, int consoleLogLevel) {
  _log.logCoutLevel=consoleLogLevel;
  _log.logFileLevel=fileLogLevel;
}

//===========================================================================
//
// parameters

/// a global operator to scan (parse) strings from a stream
std::istream& operator>>(std::istream& is, const PARSE& x) {
  parse(is, x.str); return is;
}

} //namespace

/// the same global operator for non-const string
std::istream& operator>>(std::istream& is, char* str) {
  rai::parse(is, (const char*)str); return is;
}

namespace rai {

//===========================================================================
//
// String class
//

int String::StringBuf::overflow(int C) {
  string->append(C);
  return C;
}

int String::StringBuf::sync() {
  if(string->flushCallback) string->flushCallback(*string);
  return 0;
}

void String::StringBuf::setIpos(char* p) { setg(string->p, p, string->p+string->N); }

char* String::StringBuf::getIpos() { return gptr(); }

//-- direct memory operations
void String::append(char x) { resize(N+1, true); operator()(N-1)=x; }

void String::prepend(const String& s) {
  uint n=N;
  resize(n+s.N, true);
  memmove(p+s.N, p, n);
  memmove(p, s, s.N);
}

void String::replace(uint i, uint n, const char* xp, uint xN) {
  uint Nold=N;
  if(n==xN) {
    memmove(p+i, xp, (xN));
  } else if(n>xN) {
    memmove(p+i+xN, p+i+n, (Nold-i-n));
    if(i+n<Nold) memmove(p+i, xp, (xN));
    resize(Nold-n+xN, true);
  } else {
    resize(Nold+xN-n, true);
    if(i+n<Nold) memmove(p+i+xN, p+i+n, (Nold-i-n));
    memmove(p+i, xp, (xN));
  }
}

void String::removePostfix(const char* postfix){
  uint n = strlen(postfix);
  CHECK(!strcmp(p+N-n, postfix), "no match between postfix '" <<postfix <<"' and end '" <<p+N-n <<"'");
  resize(N-n, true);
}

String& String::setRandom() {
  resize(rnd.uni_int(2, 6), false);
  for(uint i=0; i<N; i++) operator()(i)=rnd.uni_int('a', 'z');
  return *this;
}

void String::resize(uint n, bool copy) {
  if(N==n && M>N) return;
  char* pold=p;
  uint Mold=M;
  //flexible allocation (more than needed in case of multiple resizes)
  if(M==0) {  //first time
    M=n+1;
  } else if(n+1>M || 10+2*n<M/2) {
    M=11+2*n;
  }
  if(M!=Mold) { //do we actually have to allocate?
    p=new char [M];
    if(!p) HALT("Mem failed memory allocation of " <<M <<"bytes");
    if(copy) memmove(p, pold, N<n?N:n);
    if(Mold) delete[] pold;
  }
  if(!copy) memset(p, ' ', n);
  N=n;
  p[N]=0;
  resetIstream();
}

void String::init() { p=0; N=0; M=0; buffer.string=this; flushCallback=nullptr; }

/// standard constructor
String::String() : std::iostream(&buffer) { init(); clearStream(); }

/// copy constructor
String::String(const String& s) : std::iostream(&buffer) { init(); this->operator=(s); }

/// copy constructor for an ordinary C-string (needs to be 0-terminated)
String::String(const char* s) : std::iostream(&buffer) { init(); if(s) this->operator=(s); }

String::String(const std::string& s) : std::iostream(&buffer) { init(); this->operator=(s.c_str()); }

String::String(std::istream& is) : std::iostream(&buffer) { init(); read(is, "", "", 0); }

String::~String() { if(M) delete[] p; }

/// returns a reference to this
std::iostream& String::stream() { return (std::iostream&)(*this); }

/// returns a reference to this
String& String::operator()() { return *this; }

/** @brief returns the true memory buffer (C-string) of this class (which is
always kept 0-terminated) */
String::operator char* () { return p; }

/// as above but const
String::operator const char* () const { return p; }

/// returns the i-th char
char& String::operator()(int i) const {
  if(i<0) i+=N;
  CHECK_LE((uint)i, N, "String range error (" <<i <<"<=" <<N <<")");
  return p[i];
}

/// return the substring from 'start' to (exclusive) 'end'.
String String::getSubString(int start, int end) const {
  if(start<0) start+=N;
  if(end<0) end+=N;
  CHECK_GE(start, 0, "start < 0");
  CHECK_LE(end, (int)N, "end out of range");
  CHECK_LE(start,  end, "end before start");
  String tmp;
  tmp.set(p+start, 1+end-start);
//  for(int i = start; i < end; i++) tmp.append((*this)(i));
  return tmp;
}

/**
 * @brief Return the last 'n' chars of the string.
 * @param n number of chars to return
 */
String String::getLastN(uint n) const {
  CHECK_LE(n, N, "");
  if(n==N) return *this;
  return getSubString(-int(n), -1);
}

/**
 * @brief Return the first 'n' chars of the string.
 * @param n number of chars to return.
 */
String String::getFirstN(uint n) const {
  return getSubString(0, n-1);
}

/// copy operator
String& String::operator=(const String& s) {
  resize(s.N, false);
  memmove(p, s.p, N);
  return *this;
}

String& String::operator=(const std::string& s) {
  return this->operator=(s.c_str());
}

/// copies from the C-string
String& String::operator=(const char* s) {
  if(!s) {  clear();  return *this;  }
  uint ls = strlen(s);
  if(!ls) {  clear();  return *this;  }
  if(s>=p && s<=p+N) { //s points to a substring within this string!
    memmove(p, s, ls);
    resize(ls, true);
  } else {
    resize(ls, false);
    memmove(p, s, ls);
  }
  return *this;
}

void String::set(const char* s, uint n) { resize(n, false); memmove(p, s, n); }

String& String::printf(const char* format, ...) {
  resize(100, false);
  va_list valist;
  va_start(valist, format);
  int len = vsnprintf(p, 100, format, valist);
  va_end(valist);
  resize(len, true);
  return *this;
}

/// shorthand for the !strcmp command
bool String::operator==(const char* s) const { return p && !strcmp(p, s); }
/// shorthand for the !strcmp command
bool String::operator==(const String& s) const { if(!N && !s.N) return true;  return p && s.p && (!strcmp(p, s.p)); }
bool String::operator!=(const char* s) const { return !operator==(s); }
bool String::operator!=(const String& s) const { return !(operator==(s)); }
bool String::operator<=(const String& s) const { return p && s.p && strcmp(p, s.p)<=0; }

int String::find(char c, bool reverse) const {
  if(!p) return -1;
  if(reverse) {
    for(uint i=N; i--;) if(p[i]==c) return i;
  } else {
    for(uint i=0; i<N; i++) if(p[i]==c) return i;
  }
  return -1;
}

bool String::contains(char c) const {
  if(!p) return false;
  for(uint i=0; i<N; i++) if(p[i]==c) return true;
  return false;
}

bool String::contains(const String& substring) const {
  if(!p && substring.p) return false;
  if(!substring.p && p) return true;
  char* p = strstr(this->p, substring.p);
  return p != nullptr;
}

/// Return true iff the string starts with 'substring'.
bool String::startsWith(const String& substring) const {
  return N>=substring.N && this->getFirstN(substring.N) == substring;
}

/// Return true iff the string starts with 'substring'.
bool String::startsWith(const char* substring) const {
  return this->startsWith(String(substring));
}

/// Return true iff the string ends with 'substring'.
bool String::endsWith(const String& substring) const {
  if(substring.N>N) return false;
  return this->getLastN(substring.N) == substring;
}
/// Return true iff the string ends with 'substring'.
bool String::endsWith(const char* substring) const {
  return this->endsWith(String(substring));
}

/// deletes all memory and resets all stream flags
String& String::clear() { resize(0, false); return *this; }

/// call IOstream::clear();
String& String::clearStream() { std::iostream::clear(); return *this; }

/** @brief when using this String as an istream (to read other variables
  from it), this method resets the reading-pointer to the beginning
  of the string and also clears all flags of the stream */
String& String::resetIstream() { buffer.setIpos(p); clearStream(); return *this; }

/// writes the string into some ostream
void String::write(std::ostream& os) const { if(N) os <<p; }

/** @brief reads the string from some istream: first skip until one of the stopSymbols
is encountered (default: newline symbols) */
uint String::read(std::istream& is, const char* skipSymbols, const char* stopSymbols, int eatStopSymbol) {
  if(!skipSymbols) skipSymbols=readSkipSymbols;
  if(!stopSymbols) stopSymbols=readStopSymbols;
  if(eatStopSymbol==-1) eatStopSymbol=readEatStopSymbol;
  skip(is, skipSymbols);
  clear();
  char c=is.get();
  while(c!=-1 && is.good() && !rai::contains(stopSymbols, c)) {
    append(c);
    c=is.get();
  }
  if(c==-1) is.clear();
  if(c!=-1 && !eatStopSymbol) is.putback(c);
  return N;
}

//===========================================================================
//
// FileToken
//

FileToken::FileToken() {
  baseDir = getcwd_string();
}

FileToken::FileToken(const char* filename) {
  baseDir = getcwd_string();
  name = filename;
//  if(!exists()) HALT("file '" <<filename <<"' does not exist");
}

FileToken::FileToken(const FileToken& ft) {
  name=ft.name;
  path=ft.path;
  baseDir=ft.baseDir;
  is = ft.is;
  os = ft.os;
}

FileToken::~FileToken() {}

/// change to the directory of the given filename
void FileToken::decomposeFilename() {
  path = name;
  int i=path.N;
  for(; i--;) if(path(i)=='/' || path(i)=='\\') break;
  if(i==-1) {
    path=".";
  } else {
    path.resize(i, true);
    name = name+i+1;
  }
}

void FileToken::cd_base() {
  LOG(3) <<"entering path '" <<baseDir<<"'";
  if(chdir(baseDir)) HALT("couldn't change to directory '" <<baseDir <<"'");
}

void FileToken::cd_file() {
  name = fullPath();
  baseDir = getcwd_string();
  decomposeFilename();
  if(path!=".") {
    LOG(3) <<"entering path '" <<path<<"' from '" <<baseDir <<"'";
    if(chdir(path)) HALT("couldn't change to directory '" <<path <<"' from '" <<baseDir <<"'");
  }
}

bool FileToken::exists() {
  struct stat sb;
  int r=stat(name, &sb);
  return r==0;
}

std::ostream& FileToken::getOs() {
  CHECK(!is, "don't use a FileToken both as input and output");
  if(!os) {
    os = std::make_unique<std::ofstream>();
    os->open(name);
    LOG(3) <<"opening output file '" <<name <<"'";
    if(!os->good()) RAI_MSG("could not open file '" <<name <<"' for output from '" <<baseDir <<"./" <<path <<"'");
  }
  return *os;
}

std::istream& FileToken::getIs() {
  CHECK(!os, "don't use a FileToken both as input and output");
  if(!is) {
    is = std::make_unique<std::ifstream>();
    is->open(name);
    LOG(3) <<"opening input file '" <<name <<"'";
    if(!is->good()){
      THROW("could not open file '" <<name <<"' for input from '" <<baseDir <<" / " <<path <<"'");
    }
  }
  return *is;
}

String FileToken::autoPath() const {
  String nowDir;
  nowDir = getcwd_string();
  if(nowDir==baseDir) return relPath();
  return fullPath();
}

String FileToken::relPath() const {
  if(!path.N || name[0]=='/') return name;
  return path+'/'+name;
}

String FileToken::fullPath() const {
  if(name[0]=='/') return name;
  if(path.N && path[0]=='/') return STRING(path <<'/' <<name);
  String str;
  str <<baseDir;
  if(path.N) str <<'/' <<path;
  str <<'/' <<name;
  return str;
}

//===========================================================================
//
// random number generator
//

uint rndInt(uint up){ return uint(rnd.uni_int(0, up)); }


//===========================================================================
//
// Inotify
//

#ifndef RAI_MSVC
Inotify::Inotify(const char* filename): fd(0), wd(0) {
  fd = inotify_init();
  if(fd<0) HALT("Couldn't initialize inotify");
  fil = new FileToken(filename);
  fil->decomposeFilename();
  wd = inotify_add_watch(fd, fil->path,
                         IN_MODIFY | IN_CREATE | IN_DELETE);
  if(wd == -1) HALT("Couldn't add watch to " <<filename);
  buffer_size = 10*(sizeof(struct inotify_event)+64);
  buffer = new char[buffer_size];
}

Inotify::~Inotify() {
  inotify_rm_watch(fd, wd);
  close(fd);
  delete[] buffer;
  delete fil;
}

bool Inotify::poll(bool block, bool verbose) {
  if(!block) {
    struct pollfd fd_poll = {fd, POLLIN, 0};
    int r = ::poll(&fd_poll, 1, 0);
    CHECK_GE(r, 0, "poll failed");
    if(!r) return false;
  }

  int length = read(fd, buffer, buffer_size);
  CHECK_GE(length, 0, "read failed");

  //-- process event list
  for(int i=0; i<length;) {
    struct inotify_event* event = (struct inotify_event*) &buffer[ i ];
    if(verbose) {
      if(event->len) {
        if(event->mask & IN_CREATE)
          cout << "The "
               <<(event->mask&IN_ISDIR?"directory ":"file ")
               <<event->name <<" was created." <<endl;
        if(event->mask & IN_DELETE)
          cout << "The "
               <<(event->mask&IN_ISDIR?"directory ":"file ")
               <<event->name <<" was deleted." <<endl;
        if(event->mask & IN_MODIFY)
          cout << "The "
               <<(event->mask&IN_ISDIR?"directory ":"file ")
               <<event->name <<" was modified." <<endl;
      } else {
        cout <<"event of zero length" <<endl;
      }
    }
    if(event->len
        && (event->mask & (IN_MODIFY|IN_CREATE|IN_DELETE))
        && strncmp(event->name, "z.log", 5) //NOT z.log...
      ) return true; //report modification on specific file
    i += sizeof(struct inotify_event) + event->len;
  }

  return false;
}
#else //RAI_MSVC
Inotify::Inotify(const char* filename) : fd(0), wd(0) { NICO }
Inotify::~Inotify() { NICO }
bool Inotify::poll(bool block, bool verbose) { NICO }
#endif

//===========================================================================
//
// Mutex
//

#define MUTEX_DUMP(x) //x

Mutex::Mutex() {
  state=0;
  recursive=0;
}

Mutex::~Mutex() {
  if(state) {
    cerr << "Mutex destroyed without unlocking first" <<endl;
    exit(1);
  }
}

void Mutex::lock(const char* _lockInfo) {
  mutex.lock();
  int pid = getpid();
  if(!true) {
    cerr <<"could not lock mutex by process " <<pid <<" -- is blocked with info '" <<lockInfo <<"' by process " <<state <<endl;
    exit(1);
  }
  lockInfo = _lockInfo;
  recursive++;
  state = pid;
  MUTEX_DUMP(cout <<"Mutex-lock: " <<state <<" (rec: " <<recursive << ")" <<endl);
}

void Mutex::unlock() {
  MUTEX_DUMP(cout <<"Mutex-unlock: " <<state <<" (rec: " <<recursive << ")" <<endl);
  if(--recursive == 0) state=0;
  mutex.unlock();
}


//===========================================================================
//
// Cumulative probability for the Standard Normal Distribution
//

double erf(double x) {
  double t, z, retval;
  z = fabs(x);
  t = 1.0 / (1.0 + 0.5 * z);
  retval = t * exp(-z * z - 1.26551223 + t *
                   (1.00002368 + t *
                    (0.37409196 + t *
                     (0.09678418 + t *
                      (-0.18628806 + t *
                       (0.27886807 + t *
                        (-1.13520398 + t *
                         (1.48851587 + t *
                          (-0.82215223 + t *
                           0.1708727)))))))));
  if(x < 0.0) return retval - 1.0;
  return 1.0 - retval;
}

/// the integral of N(0, 1) from -infty to x
double gaussInt(double x) {
  return .5*(1.+erf(x/RAI_SQRT2));
}

/// expectation \f$\int_x^\infty {\cal N}(x) x dx\f$ when integrated from -infty to x
double gaussIntExpectation(double x) {
  double norm=gaussInt(x) / (::sqrt(RAI_2PI));
  return - norm*approxExp(-.5*x*x);
}

//===========================================================================
// MISC

/**
 * @brief Return the current working dir as std::string.
 */
String getcwd_string() {
#ifdef RAI_MSVC
#  define PATH_MAX 4096
#endif
  char buff[PATH_MAX];
  char* succ=getcwd(buff, PATH_MAX);
  if(!succ) HALT("could not call getcwd: errno=" <<errno <<' ' <<strerror(errno));
  return String(buff);
}

const char* niceTypeidName(const std::type_info& type) {
  static char buf[256];
  strcpy(buf, type.name());
  for(char* c=buf; *c; c++) if(*c>='0' && *c<='9') *c='_';
  int cut=0;
  while(buf[cut]=='_') cut++;
  return buf+cut;
}

}//namespace rai

//===========================================================================
//
// gnuplot calls
//

struct GnuplotServer {
  FILE* gp;
  GnuplotServer():gp(nullptr) {}
  ~GnuplotServer() {
    if(gp) {
      cout <<"Closing Gnuplot" <<endl;
      //      send("set terminal wxt nopersist close\nexit", false);
      //      fclose(gp);
    }
  }

  void send(const char* cmd, bool persist) {
#ifndef RAI_MSVC
    if(!gp) {
      if(!persist) gp=popen("env gnuplot 2> /dev/null", "w");
      else         gp=popen("env gnuplot -persist 2> /dev/null", "w");
      CHECK(gp, "could not open gnuplot pipe");
    }
    FILE("z.plotcmd") <<cmd; //for debugging..
    fputs(cmd, gp);
    fflush(gp);
#else
    NIY;
#endif
  }
};

rai::Singleton<GnuplotServer> gnuplotServer;

void gnuplot(const char* command, bool pause, bool persist, const char* PDFfile) {
  if(rai::getDisableGui()) return;
  if(!rai::getInteractivity()) {
    pause=false;
    persist=false;
  }

  rai::String cmd;
  cmd <<"set style data lines\n";

// run standard files
#ifndef RAI_MSVC
  if(!access("~/gnuplot.cfg", R_OK)) cmd <<"load '~/gnuplot.cfg'\n";
  if(!access("gnuplot.cfg", R_OK)) cmd <<"load 'gnuplot.cfg'\n";
#endif

  cmd <<"set title '(Gui/plot.h -> gnuplot pipe)'\n"
      <<command <<endl;

  if(PDFfile) {
    cmd <<"set terminal push\n"
        <<"set terminal pdfcairo\n"
        <<"set output '" <<PDFfile <<"'\n"
        <<command <<endl
        <<"\nset terminal pop\n";
  }

  //  if(pauseMouse) cmd <<"\n pause mouse key" <<endl;
  gnuplotServer()->send(cmd.p, persist);

  if(pause) rai::wait(1., true);

  if(!rai::getInteractivity()) {
    rai::wait(.05);
  }
}

//===========================================================================
//
// explicit instantiations
//

#include "util.ipp"

namespace rai{

template void getParameter(int&, const char*);
template void getParameter(int&, const char*, const int&);
template void getParameter(uint&, const char*);
template void getParameter(uint&, const char*, const uint&);
template void getParameter(bool&, const char*, const bool&);
template void getParameter(double&, const char*);
template void getParameter(double&, const char*, const double&);
template void getParameter(String&, const char*, const String&);
template void getParameter(String&, const char*);

template int getParameter<int>(const char*);
template int getParameter<int>(const char*, const int&);
template uint getParameter(const char*);
template uint getParameter<uint>(const char*, const uint&);
template float getParameter<float>(const char*);
template double getParameter<double>(const char*);
template double getParameter<double>(const char*, const double&);
template bool getParameter<bool>(const char*);
template bool getParameter<bool>(const char*, const bool&);
template long getParameter<long>(const char*);
template String getParameter<String>(const char*);
template String getParameter<String>(const char*, const String&);
template StringA getParameter<StringA>(const char*);
template StringA getParameter<StringA>(const char*, const StringA&);
template Graph getParameter<Graph>(const char*);

template void setParameter<double>(const char*, const double&);
template void setParameter<uint>(const char*, const uint&);
template void setParameter<arr>(const char*, const arr&);
template void setParameter<String>(const char*, const String&);

template Enum<ArgWord> getParameter<Enum<ArgWord>>(const char*);
template Enum<ArgWord> getParameter<Enum<ArgWord>>(const char*, const Enum<ArgWord>&);

template bool checkParameter<uint>(const char*);
template bool checkParameter<int>(const char*);
template bool checkParameter<double>(const char*);
template bool checkParameter<bool>(const char*);
template bool checkParameter<String>(const char*);

}

//===========================================================================

RUN_ON_INIT_BEGIN(util)
//processInfo(); //creates the singleton
RUN_ON_INIT_END(util)
