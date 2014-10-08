/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


#include "util.h"
#include <math.h>
#include <string.h>
#if defined MT_Linux || defined MT_Cygwin || defined MT_Darwin
#  include <limits.h>
#  include <sys/time.h>
#  include <sys/times.h>
#  include <sys/resource.h>
#  include <sys/inotify.h>
#  include <poll.h>
#endif
#ifdef __CYGWIN__
#include "cygwin_compat.h"
#endif
#if defined MT_MSVC
#  include <time.h>
#  include <sys/timeb.h>
#  include <windows.h>
#  undef min
#  undef max
#  define MT_TIMEB
#  ifdef MT_QT
#    undef  NOUNICODE
#    define NOUNICODE
#  endif
#  pragma warning(disable: 4305 4244 4250 4355 4786 4996)
#endif
#if defined MT_MinGW
#  include <unistd.h>
#  include <sys/time.h>
#  include <sys/timeb.h>
#  define MT_TIMEB
#endif

#ifdef MT_QT
#  undef scroll
#  undef Unsorted
#  include <Qt/qmetatype.h>
#  include <Qt/qdatastream.h>
#  include <Qt/qapplication.h>
#  include <QThread>
#endif

#ifndef MT_ConfigFileName
#  define MT_ConfigFileName "MT.cfg"
#  define MT_LogFileName "MT.log"
#endif

#include <errno.h>
#ifndef MT_MSVC
#  include <unistd.h>
#  include <sys/syscall.h>
#endif

//===========================================================================
//
// Bag container
//

const char *MT::String::readSkipSymbols = " \t";
const char *MT::String::readStopSymbols = "\n\r";
int   MT::String::readEatStopSymbol     = 1;
MT::String MT::errString;


//===========================================================================
//
// utilities in MT namespace
//

namespace MT {
int argc;
char** argv;
std::ifstream cfgFile;
bool cfgFileOpen=false;
Mutex cfgFileMutex;
bool IOraw=false;
bool noLog=true;
uint lineCount=1;
int verboseLevel=-1;
int interactivity=-1;

#ifndef MT_TIMEB
timeval startTime;
#else
_timeb startTime;
#endif
double timerStartTime=0.;
double timerPauseTime=-1.;
bool timerUseRealTime=false;

#ifdef MT_QT
QApplication *myApp=NULL;
#endif

struct LogFile {
  std::ofstream fil;
  bool isOpen;
  bool noLog;
  
  LogFile():isOpen(false), noLog(false) {
    timerStartTime=MT::cpuTime();
#ifndef MT_TIMEB
    gettimeofday(&startTime, 0);
#else
    _ftime(&startTime);
#endif
  }
  ~LogFile() {
    if(isOpen) {  //don't open a log file anymore in the destructor
      char times[200];
      sprintf(times, "Ellapsed double time:  %.3lfsec\nProcess  user time:   %.3lfsec", realTime(), cpuTime());
      MT::log() <<"Execution stop:      " <<date()
                <<times <<std::endl;
      //"Ellapsed double time:  " <<::dtoa(realTime()) <<"sec\n"
      // <<"Process  user time:   " <<::dtoa(cpuTime()) <<"sec" <<std::endl;
#ifndef MT_TIMEB
      MT::log() <<"Process system time: " <<sysTime() <<"sec" <<std::endl;
#endif
      MT::log().close();
    }
#ifdef MT_QT
    if(myApp) {
      myApp->processEvents();
      myApp->quit();
    }
#endif
  }
  
  std::ofstream& log(const char *name) {
    if(!isOpen && !noLog) {
      fil.open(name);
      if(!fil.good()) MT_MSG("could not open log-file `" <<name <<"' for output");
      isOpen=true;
    }
    return fil;
  }
};

Singleton<LogFile> logFile;

/// access to the log-file
std::ofstream& log(const char *name) { return logFile().log(name); }

/// open an output-file with name '\c name'
void open(std::ofstream& fs, const char *name, const char *errmsg) {
  fs.clear();
  fs.open(name);
  log() <<"opening output file `" <<name <<"'" <<std::endl;
  if(!fs.good()) MT_MSG("could not open file `" <<name <<"' for output" <<errmsg);
}

/// open an input-file with name '\c name'
void open(std::ifstream& fs, const char *name, const char *errmsg) {
  fs.clear();
  fs.open(name);
  log() <<"opening input file `" <<name <<"'" <<std::endl;
  if(!fs.good()) HALT("could not open file `" <<name <<"' for input" <<errmsg);
}

/// returns true if the (0-terminated) string s contains c
bool contains(const char *s, char c) {
  for(uint i=0; s[i]; i++) if(s[i]==c) return true;
  return false;
}

/// skips the chars (typically white characters) when parsing from the istream, returns first non-skipped char
char skip(std::istream& is, const char *skipSymbols, const char *stopSymbols, bool skipCommentLines) {
  char c;
  for(;;) {
    c=is.get();
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
  do { c=is.get(); } while(c!='\n');
  is.putback(c);
}

/// skips the next character
void skipOne(std::istream& is) {
  is.get();
}

/// tell you about the next char (after skip()) but puts it back in the stream
char getNextChar(std::istream& is, const char *skipSymbols, bool skipCommentLines) {
  char c;
  skip(is, skipSymbols, NULL, skipCommentLines);
  is.get(c);
  if(!is.good()) return 0;
  return c;
}

/// tell you about the next char (after skip()) but puts it back in the stream
char peerNextChar(std::istream& is, const char *skipSymbols, bool skipCommentLines) {
  char c=getNextChar(is, skipSymbols, skipCommentLines);
  if(!is.good()) return 0;
  is.putback(c);
  return c;
}

/// skip throught a stream until the tag is found (which is eaten)
bool skipUntil(std::istream& is, const char *tag) {
  unsigned n=strlen(tag);
  char *buf=new char [n+1];
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
bool parse(std::istream& is, const char *str, bool silent) {
  if(!is.good()) { if(!silent) MT_MSG("bad stream tag when scanning for `" <<str <<"'"); return false; }  //is.clear(); }
  uint i, n=strlen(str);
  char *buf=new char [n+1]; buf[n]=0;
  MT::skip(is, " \n\r\t");
  is.read(buf, n);
  if(!is.good() || strcmp(str, buf)) {
    for(i=n; i--;) is.putback(buf[i]);
    is.setstate(std::ios::failbit);
    if(!silent)  MT_MSG("(LINE=" <<MT::lineCount <<") parsing of constant string `" <<str
                          <<"' failed! (read instead: `" <<buf <<"')");
    return false;
  }
  delete[] buf;
  return true;
}


/// returns the i-th of str
byte bit(byte *str, uint i) { return (str[i>>3] >>(7-(i&7))) & 1; }
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

double indicate(bool expr){ if(expr) return 1.; return 0.; }

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
//void clip(double& x, double a, double b) { if(x<a) x=a; if(x>b) x=b; }

/// the angle of the vector (x, y) in [-pi, pi]
double phi(double x, double y) {
  if(x==0. || ::fabs(x)<1e-10) { if(y>0.) return MT_PI/2.; else return -MT_PI/2.; }
  double p=::atan(y/x);
  if(x<0.) { if(y<0.) p-=MT_PI; else p+=MT_PI; }
  if(p>MT_PI)  p-=2.*MT_PI;
  if(p<-MT_PI) p+=2.*MT_PI;
  return p;
}

/// the change of angle of the vector (x, y) when displaced by (dx, dy)
double dphi(double x, double y, double dx, double dy) {
  //return (dy*x - dx*y)/sqrt(x*x+y*y);
  if(x==0. || ::fabs(x)<1e-10) { if(y>0.) return -dx/y; else return dx/y; }
  double f=y/x;
  return 1./(1.+f*f)*(dy/x - f/x*dx);
}

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
  if(x<.001) x=.001; return ::log(x);
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

#define EXP ::exp //MT::approxExp

double NNsdv(const double& a, const double& b, double sdv){
  double d=(a-b)/sdv;
  double norm = 1./(::sqrt(MT_2PI)*sdv);
  return norm*EXP(-.5*d*d);
}

double NNsdv(double x, double sdv){
  x/=sdv;
  double norm = 1./(::sqrt(MT_2PI)*sdv);
  return norm*EXP(-.5*x*x);
}

/* gnuplot:
heavy(x) = (1+sgn(x))/2
eps = 0.1
g(x) = heavy(x-eps)*(x-eps/2) + (1-heavy(x-eps))*x**2/(2*eps)
plot [-.5:.5] g(abs(x))
*/
double POW(double x, double power){ if(power==1.) return x; if(power==2.) return x*x; return pow(x,power); }
double smoothRamp(double x, double eps, double power){
  if(x<0.) return 0.;
  if(power!=1.) return pow(smoothRamp(x,eps,1.),power);
  if(!eps) return x;
  if(x>eps) return x - .5*eps;
  return x*x/(2*eps);
}

double d_smoothRamp(double x, double eps, double power){
  if(x<0.) return 0.;
  if(power!=1.) return power*pow(smoothRamp(x,eps,1.),power-1.)*d_smoothRamp(x,eps,1.);
  if(!eps || x>eps) return 1.;
  return x/eps;
}

/*
heavy(x) = (1+sgn(x))/2
power = 1.5
margin = 1.5
f(x) = heavy(x)*x**power
plot f(x/margin+1), 1
*/
double ineqConstraintCost(double g, double margin, double power){
  if(g<-margin) return 0.;
  double y=g/margin+1.;
  if(power==1.) return y;
  if(power==2.) return y*y;
  return pow(y,power);
}

double d_ineqConstraintCost(double g, double margin, double power){
  if(g<-margin) return 0.;
  double y=g/margin+1.;
  if(power==1.) return 1./margin;
  if(power==2.) return 2.*y/margin;
  return power*pow(y,power-1.)/margin;
}

double eqConstraintCost(double h, double margin, double power){
  double y=h/margin;
  if(power==1.) return fabs(y);
  if(power==2.) return y*y;
  return pow(fabs(y),power);
}

double d_eqConstraintCost(double h, double margin, double power){
  double y=h/margin;
  if(power==1.) return MT::sign(y)/margin;
  if(power==2.) return 2.*y/margin;
  return power*pow(y,power-1.)*MT::sign(y)/margin;
}

/** @brief double time on the clock
  (probably in micro second resolution) -- Windows checked! */
double clockTime() {
#ifndef MT_TIMEB
  static timeval t; gettimeofday(&t, 0);
  return ((double)(t.tv_sec%86400) + //modulo TODAY
          (double)(t.tv_usec)/1000000.);
#else
  static _timeb t; _ftime(&t);
  return ((double)(t.time%86400) + //modulo TODAY
          (double)(t.millitm-startTime.millitm)/1000.);
#endif
}

double toTime(const tm& t) {
    return (double)(mktime(const_cast<tm*>(&t)) % 86400);
}

/** @brief double time since start of the process in floating-point seconds
  (probably in micro second resolution) -- Windows checked! */
double realTime() {
#ifndef MT_TIMEB
  static timeval t; gettimeofday(&t, 0);
  return ((double)(t.tv_sec-startTime.tv_sec-1) +
          (double)((long)1000000+t.tv_usec-startTime.tv_usec)/1000000.);
#else
  static _timeb t; _ftime(&t);
  return ((double)(t.time-startTime.time-1) +
          (double)((unsigned short)1000+t.millitm-startTime.millitm)/1000.);
#endif
}

/** @brief user CPU time of this process in floating-point seconds (pure
  processor time) -- Windows checked! */
double cpuTime() {
#ifndef MT_TIMEB
  static tms t; times(&t);
  return ((double)t.tms_utime)/sysconf(_SC_CLK_TCK);
#else
  return ((double)clock())/CLOCKS_PER_SEC; //MSVC: CLOCKS_PER_SEC=1000
#endif
}

/** @brief system CPU time of this process in floating-point seconds (the
  time spend for file input/output, x-server stuff, etc.)
  -- not implemented for Windows! */
double sysTime() {
#ifndef MT_TIMEB
  static tms t; times(&t);
  return ((double)(t.tms_stime))/sysconf(_SC_CLK_TCK);
#else
  HALT("sysTime() is not implemented for Windows!");
  return 0.;
#endif
}

/** @brief total CPU time of this process in floating-point seconds (same
  as cpuTime + sysTime) -- not implemented for Windows! */
double totalTime() {
#ifndef MT_TIMEB
  static tms t; times(&t);
  return ((double)(t.tms_utime+t.tms_stime))/sysconf(_SC_CLK_TCK);
#else
  HALT("totalTime() is not implemented for Windows!");
  return 0.;
#endif
}

/// the absolute double time and date as string
char *date() { static time_t t; time(&t); return ctime(&t); }

/// wait double time
void wait(double sec, bool msg_on_fail) {
#if defined(MT_Darwin)
  sleep((int)sec);
#elif !defined(MT_MSVC)
  timespec ts;
  ts.tv_sec = (long)(floor(sec));
  sec -= (double)ts.tv_sec;
  ts.tv_nsec = (long)(floor(1000000000. * sec));
  int rc = clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);
  if(rc && msg_on_fail) MT_MSG("clock_nanosleep() failed " <<rc <<' ' <<strerror(rc));
#else
  Sleep((int)(1000.*sec));
  //MsgWaitForMultipleObjects( 0, NULL, FALSE, (int)(1000.*sec), QS_ALLEVENTS);
#endif
  
#if 0
#ifndef MT_TIMEB
  timeval tv;
  tv.tv_sec = (int)(floor(sec));
  sec -= (double)tv.tv_sec;
  tv.tv_usec = (int)(floor(1000000. * sec));
  int r = select(1, NULL, NULL, NULL, &tv);
  if(r==-1) MT_MSG("select() failed");
  /* r=0 time is up
     r!=0 data in NULL stream available (nonsense)
     */
#else
  double t=realTime(); while(realTime()-t<sec);
#  endif
#endif
}

/// wait for an ENTER at the console
bool wait() {
  if(!MT::getInteractivity()) return true;
  char c[10];
  std::cout <<" -- hit a key to continue..." <<std::flush;
  //cbreak(); getch();
  std::cin.getline(c, 10);
  std::cout <<"\r" <<std::flush;
  if(c[0]==' ') return true;
  else return false;
  return true;
}

/// the integral shared memory size -- not implemented for Windows!
long mem() {
#ifndef MT_TIMEB
  static rusage r; getrusage(RUSAGE_SELF, &r);
  return r.ru_idrss;
#else
  HALT("MT::mem() is not implemented for Windows!");
  return 0;
#endif
}

/// start and reset the timer (user CPU time)
void timerStart(bool useRealTime) {
  if(useRealTime) timerUseRealTime=true; else timerUseRealTime=false;
  timerPauseTime=-1.;
  timerStartTime=(timerUseRealTime?realTime():cpuTime());
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

/// memorize the command line arguments and open a log file
void initCmdLine(int _argc, char *_argv[]) {
  argc=_argc; argv=_argv;
  time_t t; time(&t);
  const char *name;
  if(checkCmdLineTag("nolog")) noLog=true; else noLog=false;
  name=getCmdLineArgument("log");
  if(!name) name=MT_LogFileName;
  log(name);
  if(!log().good()) MT_MSG(" -- use `-nolog' or `-log' option to specify the log file");
  log() <<"Compiled at:     " <<__DATE__ <<" " <<__TIME__ <<"\n";
  log() <<"Execution start: " <<ctime(&t);
  log() <<"Program call:    '"; for(int i=0; i<argc; i++) log() <<argv[i] <<" ";
  log() <<"\b'" <<std::endl;
}

/// returns true if the tag was found on command line
bool checkCmdLineTag(const char *tag) {
  for(int n=1; n<argc; n++) if(argv[n][0]=='-' && !strcmp(tag, argv[n]+1)) {
      return true;
    }
  return false;
}

/// returns the argument after the cmd-line tag; NULL if the tag is not found
char *getCmdLineArgument(const char *tag) {
  int n;
  for(n=1; n<argc; n++) if(argv[n][0]=='-' && !strcmp(tag, argv[n]+1)) {
      if(n+1==argc) return (char*)"1";
      return argv[n+1];
    }
  return NULL;
}

/** @brief Open a (possibly new) config file with name '\c name'.<br> If
  \c name is not specified, it searches for a command line-option
  '-cfg' and, if not found, it assumes \c name=MT.cfg */
void openConfigFile(const char *name) {
  log() <<"opening config file ";
  if(!name) name=getCmdLineArgument("cfg");
  if(!name) name=MT_ConfigFileName;
  if(cfgFileOpen) {
    cfgFile.close(); log() <<"(old config file closed) ";
  }
  log() <<"'" <<name <<"'";
  cfgFile.clear();
  cfgFile.open(name);
  cfgFileOpen=true;
  if(!cfgFile.good()) {
    //MT_MSG("couldn't open config file " <<name);
    log() <<" - failed";
  }
  log() <<std::endl;
}

uint getVerboseLevel() {
  if(verboseLevel==-1) verboseLevel=getParameter<int>("verbose", 0);
  return verboseLevel;
}

bool getInteractivity(){
  if(interactivity==-1) interactivity=(checkParameter<bool>("noInteractivity")?0:1);
  return interactivity==1;
}

}

/// a global operator to scan (parse) strings from a stream
std::istream& operator>>(std::istream& is, const PARSE& x) {
  MT::parse(is, x.str); return is;
}

/// the same global operator for non-const string
std::istream& operator>>(std::istream& is, char *str) {
  MT::parse(is, (const char*)str); return is;
}


//===========================================================================
//
// String class
//

int MT::String::StringBuf::overflow(int C) {
  string->append(C);
  return C;
}

int MT::String::StringBuf::sync() {
  if(string->flushCallback) string->flushCallback(*string);
  return 0;
}

void MT::String::StringBuf::setIpos(char *p) { setg(string->p, p, string->p+string->N); }

char *MT::String::StringBuf::getIpos() { return gptr(); }

//-- direct memory operations
void MT::String::append(char x) { resize(N+1, true); operator()(N-1)=x; }

void MT::String::resize(uint n, bool copy) {
  if(N==n && M>N) return;
  char *pold=p;
  uint Mold=M;
  //flexible allocation (more than needed in case of multiple resizes)
  if(M==0) {  //first time
    M=n+1;
  } else if(n+1>M || 10+2*n<M/2) {
    M=11+2*n;
  }
  if(M!=Mold) {
    p=new char [M];
    if(!p) HALT("MT::Mem failed memory allocation of " <<M <<"bytes");
    if(copy) memmove(p, pold, N<n?N:n);
    if(Mold) delete[] pold;
  }
  N=n;
  p[N]=0;
  resetIstream();
}

void MT::String::init() { p=0; N=0; M=0; buffer.string=this; flushCallback=NULL; }

/// standard constructor
MT::String::String():std::iostream(&buffer) { init(); clearStream(); }

/// copy constructor
MT::String::String(const String& s):std::iostream(&buffer) { init(); this->operator=(s); }

/// copy constructor for an ordinary C-string (needs to be 0-terminated)
MT::String::String(const char *s):std::iostream(&buffer) { init(); this->operator=(s); }

MT::String::~String() { if(M) delete[] p; }

/// returns a reference to this
std::iostream& MT::String::stream() { return (std::iostream&)(*this); }

/// returns a reference to this
MT::String& MT::String::operator()() { return *this; }

/** @brief returns the true memory buffer (C-string) of this class (which is
always kept 0-terminated) */
MT::String::operator char*() { return p; }

/// as above but const
MT::String::operator const char*() const { return p; }

/// returns the i-th char
char& MT::String::operator()(uint i) const { CHECK(i<=N, "String range error (" <<i <<"<=" <<N <<")"); return p[i]; }

/// return the substring from `start` to (exclusive) `end`.
MT::String MT::String::getSubString(uint start, uint end) const {
  CHECK(start < end, "getSubString: start should be smaller than end");
  clip(end, uint(0), N);
  String tmp;
  for (uint i = start; i < end; i++) {
    tmp.append((*this)(i));
  }
  return tmp;
}

/**
 * @brief Return the last `n` chars of the string.
 * @param n number of chars to return
 */
MT::String MT::String::getLastN(uint n) const {
  clip(n, uint(0), N);
  return getSubString(N-n, N);
}

/**
 * @brief Return the first `n` chars of the string.
 * @param n number of chars to return.
 */
MT::String MT::String::getFirstN(uint n) const {
  clip(n, uint(0), N);
  return getSubString(0, n);
}

/// copy operator
MT::String& MT::String::operator=(const String& s) {
  resize(s.N, false);
  memmove(p, s.p, N);
  return *this;
}

/// copies from the C-string
void MT::String::operator=(const char *s) {
  if(!s){  clear();  return;  }
  uint ls = strlen(s);
  if(!ls){  clear();  return;  }
  if(s>=p && s<=p+N){ //s points to a substring within this string!
    memmove(p, s, ls);
    resize(ls, true);
  }else{
    resize(ls, false);
    memmove(p, s, ls);
  }
}

void MT::String::set(const char *s, uint n) { resize(n, false); memmove(p, s, n); }

/// shorthand for the !strcmp command
bool MT::String::operator==(const char *s) const { return p && !strcmp(p, s); }
/// shorthand for the !strcmp command
bool MT::String::operator==(const String& s) const { return p && s.p && !strcmp(p, s.p); }
bool MT::String::operator!=(const char *s) const { return !operator==(s); }
bool MT::String::operator!=(const String& s) const { return !(operator==(s)); }
bool MT::String::operator<(const String& s) const { return p && s.p && strcmp(p, s.p)<0; }

bool MT::String::contains(const String& substring) const {
  char* p = strstr(this->p, substring.p);
  return p != NULL;
}

/// Return true iff the string starts with `substring`.
bool MT::String::startsWith(const String& substring) const {
  return this->getFirstN(substring.N) == substring;
}
/// Return true iff the string starts with `substring`.
bool MT::String::startsWith(const char* substring) const {
  return this->startsWith(MT::String(substring));
}

/// Return true iff the string ends with `substring`.
bool MT::String::endsWith(const String& substring) const {
  return this->getLastN(substring.N) == substring;
}
/// Return true iff the string ends with `substring`.
bool MT::String::endsWith(const char* substring) const {
  return this->endsWith(MT::String(substring));
}

/// deletes all memory and resets all stream flags
MT::String& MT::String::clear() { resize(0, false); return *this; }

/// call IOstream::clear();
MT::String& MT::String::clearStream() { std::iostream::clear(); return *this; }

/** @brief when using this String as an istream (to read other variables
  from it), this method resets the reading-pointer to the beginning
  of the string and also clears all flags of the stream */
MT::String& MT::String::resetIstream() { buffer.setIpos(p); clearStream(); return *this; }

/// writes the string into some ostream
void MT::String::write(std::ostream& os) const { if(N) os <<p; }

/** @brief reads the string from some istream: first skip until one of the stopSymbols
is encountered (default: newline symbols) */
uint MT::String::read(std::istream& is, const char* skipSymbols, const char *stopSymbols, int eatStopSymbol) {
  if(!skipSymbols) skipSymbols=readSkipSymbols;
  if(!stopSymbols) stopSymbols=readStopSymbols;
  if(eatStopSymbol==-1) eatStopSymbol=readEatStopSymbol;
  MT::skip(is, skipSymbols);
  clear();
  char c=is.get();
  while(c!=-1 && is.good() && !MT::contains(stopSymbols, c)) {
    append(c);
    c=is.get();
  }
  if(c==-1) is.clear();
  if(c!=-1 && !eatStopSymbol) is.putback(c);
  return N;
}

//===========================================================================
//
// string-filling routines

/** @brief fills the string with the date and time in the format
 * yy-mm-dd--hh-mm-mm */
MT::String MT::getNowString() {
  time_t t = time(0);
  struct tm *now = localtime(&t);

  MT::String str;
  str.resize(19, false); //-- just enough
  sprintf(str.p, "%02d-%02d-%02d--%02d-%02d-%02d",
    now->tm_year-100,
    now->tm_mon+1,
    now->tm_mday,
    now->tm_hour,
    now->tm_min,
    now->tm_sec);
  return str;
}


//===========================================================================
//
// FileToken
//

MT::FileToken::FileToken(const char* filename, bool change_dir): os(NULL), is(NULL){
  name=filename;
  if(change_dir){
    decomposeFilename();
    if(path.N){
      cwd.resize(200, false);
      if(!getcwd(cwd.p, 200)) HALT("couldn't get current dir");
      cwd.resize(strlen(cwd.p), true);
      log() <<"entering path `" <<path<<"' from '" <<cwd <<"'" <<std::endl;
      if(chdir(path)) HALT("couldn't change to directory " <<path <<"(current dir: " <<cwd <<")");
    }
  }
}

MT::FileToken::~FileToken(){
  if(is){ is->close(); delete is; is=NULL; }
  if(os){ os->close(); delete os; os=NULL; }
  if(cwd.N){
    log() <<"leaving path `" <<path<<"' back to '" <<cwd <<"'" <<std::endl;
    if(chdir(cwd)) HALT("couldn't change back to directory " <<cwd);
  }
}

/// change to the directory of the given filename
void MT::FileToken::decomposeFilename() {
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

std::ofstream& MT::FileToken::getOs(){
  CHECK(!is,"don't use a FileToken both as input and output");
  if(!os){
    os=new std::ofstream;
    os->open(name);
    log() <<"opening output file `" <<name <<"'" <<std::endl;
    if(!os->good()) MT_MSG("could not open file `" <<name <<"' for output");
  }
  return *os;
}

std::ifstream& MT::FileToken::getIs(){
  CHECK(!os,"don't use a FileToken both as input and output");
  if(!is){
    is=new std::ifstream;
    is->open(name);
    log() <<"opening input file `" <<name <<"'" <<std::endl;
    if(!is->good()) HALT("could not open file `" <<name <<"' for input");
  }
  return *is;
}


//===========================================================================
//
// random number generator
//

namespace MT { Rnd rnd; }

uint32_t MT::Rnd::seed(uint32_t n) {
  uint32_t s, c;
  if(n>12345) { s=n; c=n%113; } else { s=12345; c=n; }
  while(c--) s*=65539;
  s=s>>1;
  seed250(s);
  ready=true;
  return n;
}

uint32_t MT::Rnd::seed() {
  return seed(MT::getParameter<uint32_t>("seed", 0));
}

uint32_t MT::Rnd::clockSeed() {
  uint32_t s;
#ifndef MT_TIMEB
  timeval t; gettimeofday(&t, 0); s=1000000L*t.tv_sec+t.tv_usec;
#else
  _timeb t; _ftime(&t); s=1000L*t.time+t.millitm;
#endif
  log() <<"random clock seed: " <<s <<std::endl;
  return seed(s);
}

double MT::Rnd::gauss() {
  double w, v, rsq, fac;
  do {
    v   = 2 * uni() - 1;
    w   = 2 * uni() - 1;
    rsq = v*v + w*w;
  } while(rsq >= 1 || rsq == 0);
  fac  = ::sqrt(-2 * ::log(rsq) / rsq);
  return v*fac;
}

uint32_t MT::Rnd::poisson(double mean) {
  if(mean>100) {
    uint32_t i=(uint32_t)::floor(mean+::sqrt(mean)*gauss()+.5);
    return (i>0)?(uint32_t)i:0;
  }
  uint32_t count = 0;
  double bound, product;
  if(mean>=0) {
    bound=::exp(-mean);
    product=uni();
    while(product>=bound) {
      count++;
      product*=uni();
    }
  }
  return count;
}

void  MT::Rnd::seed250(int32_t seed) {
  int32_t      i;
  int32_t     j, k;
  
  if(seed<=0) seed=1;
  
  for(i=0; i<250; ++i) {  // Schleife ueber Zufallsfeld
    k = seed / 127773;          // Modulozufallszahlengenerator
    seed = 16807 * (seed - k*127773) - 2836 * k;
    if(seed<0) seed += 0x7FFFFFFF;
    rfield[i] = seed;
  }
  
  // Masken ueberlagern
  k = 0x7FFFFFFF;
  j = 0x40000000;
  for(i=1; i<250; i+=8) rfield[i] = (rfield[i] & k) | j;
  
  // rpoint initialisieren
  rpoint = 249;
  
  // Anfangszahlen verwerfen
  for(i=0; i<4711; ++i) rnd250();
}


//===========================================================================
//
// Inotify
//

Inotify::Inotify(const char* filename): fd(0), wd(0){
  fd = inotify_init();
  if(fd<0) HALT("Couldn't initialize inotify");
  fil = new MT::FileToken(filename, false);
  fil->decomposeFilename();
  wd = inotify_add_watch( fd, fil->path,
			  IN_MODIFY | IN_CREATE | IN_DELETE );
  if(wd == -1) HALT("Couldn't add watch to " <<filename);
  buffer_size = 10*(sizeof(struct inotify_event)+64); 
  buffer = new char[buffer_size];
}

Inotify::~Inotify(){
  inotify_rm_watch( fd, wd );
  close( fd );
  delete buffer;
  delete fil;
}

bool Inotify::pollForModification(bool block, bool verbose){
  if(!block){
    struct pollfd fd_poll = {fd, POLLIN, 0};
    int r = poll(&fd_poll, 1, 0);
    CHECK(r>=0,"poll failed");
    if(!r) return false;
  }

  int length = read( fd, buffer, buffer_size );
  CHECK(length>=0, "read failed");

  //-- process event list
  for(int i=0;i<length;){
    struct inotify_event *event = ( struct inotify_event * ) &buffer[ i ];
    if(verbose){
      if(event->len) {
        if(event->mask & IN_CREATE)
          cout << "The "
               <<(event->mask&IN_ISDIR?"directory ":"file ")
              <<event->name <<" was created." <<endl;
        if ( event->mask & IN_DELETE )
          cout << "The "
               <<(event->mask&IN_ISDIR?"directory ":"file ")
              <<event->name <<" was deleted." <<endl;
        if ( event->mask & IN_MODIFY )
          cout << "The "
               <<(event->mask&IN_ISDIR?"directory ":"file ")
              <<event->name <<" was modified." <<endl;
      }else{
        cout <<"event of zero length" <<endl;
      }
    }
    if(event->len
       && (event->mask & (IN_MODIFY|IN_CREATE|IN_DELETE))
       && !strcmp(event->name, fil->name.p) ) return true; //report modification on specific file
    i += sizeof(struct inotify_event) + event->len;
  }

  return false;
}



//===========================================================================
//
// Mutex
//

#define MUTEX_DUMP(x) //x

#ifndef MT_MSVC
Mutex::Mutex() {
  pthread_mutexattr_t atts;
  int rc;
  rc = pthread_mutexattr_init(&atts);  if(rc) HALT("pthread failed with err " <<rc <<strerror(rc));
  rc = pthread_mutexattr_settype(&atts, PTHREAD_MUTEX_RECURSIVE_NP);  if(rc) HALT("pthread failed with err " <<rc <<strerror(rc));
  rc = pthread_mutex_init(&mutex, &atts);
  //mutex = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
  state=0;
  recursive=0;
}

Mutex::~Mutex() {
  CHECK(!state, "Mutex destroyed without unlocking first");
  int rc = pthread_mutex_destroy(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void Mutex::lock() {
  int rc = pthread_mutex_lock(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  recursive++;
  state=syscall(SYS_gettid);
  MUTEX_DUMP(cout <<"Mutex-lock: " <<state <<" (rec: " <<recursive << ")" <<endl);
}

void Mutex::unlock() {
  MUTEX_DUMP(cout <<"Mutex-unlock: " <<state <<" (rec: " <<recursive << ")" <<endl);
  if(--recursive == 0)
    state=0;
  int rc = pthread_mutex_unlock(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}
#else//MT_MSVC
Mutex::Mutex() {}
Mutex::~Mutex() {}
void Mutex::lock() {}
void Mutex::unlock() {}
#endif

//===========================================================================
//
// gnuplot calls
//

static FILE *MT_gp=NULL;
void gnuplotClose() {
  if(MT_gp) { fflush(MT_gp); fclose(MT_gp); }
}
void gnuplot(const char *command, bool pauseMouse, bool persist, const char *PDFfile) {
if(!MT::getInteractivity()){
  pauseMouse=false;
  persist=false;
}
#ifndef MT_MSVC
  if(!MT_gp) {
    if(!persist) MT_gp=popen("env gnuplot -noraise -geometry 600x600-0-0 2> /dev/null", "w");
    else         MT_gp=popen("env gnuplot -noraise -persist -geometry 600x600-0-0 2> /dev/null", "w");
    CHECK(MT_gp, "could not open gnuplot pipe");
  }
  
  MT::String cmd;
  cmd <<"set style data lines\n";
  
  // run standard files
  if(!access("~/gnuplot.cfg", R_OK)) cmd <<"load '~/gnuplot.cfg'\n";
  if(!access("gnuplot.cfg", R_OK)) cmd <<"load 'gnuplot.cfg'\n";
  
  cmd <<"set title '(Gui/plot.h -> gnuplot pipe)'\n"
      <<command <<std::endl;
      
  if(PDFfile) {
    cmd <<"set terminal push\n"
        <<"set terminal pdfcairo\n"
        <<"set output '" <<PDFfile <<"'\n"
        <<command <<std::endl
        <<"\nset terminal pop\n";
  }
  
  if(pauseMouse) cmd <<"\n pause mouse" <<std::endl;
  FILE("z.plotcmd") <<cmd; //for debugging..
  fputs(cmd.p, MT_gp);
  fflush(MT_gp) ;
#else
  NIY;
#endif
}


//===========================================================================
//
// Cumulative probability for the Standard Normal Distribution
//

namespace MT {
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
  return .5*(1.+erf(x/MT_SQRT2));
}

/// expectation \f$\int_x^\infty {\cal N}(x) x dx\f$ when integrated from -infty to x
double gaussIntExpectation(double x) {
  double norm=gaussInt(x) / (::sqrt(MT_2PI));
  return - norm*MT::approxExp(-.5*x*x);
}
}

//===========================================================================
// MISC

/**
 * @brief Return the current working dir as std::string.
 */
std::string getcwd_string() {
   char buff[PATH_MAX];
   char *succ=getcwd( buff, PATH_MAX );
   CHECK(succ,"could not call getcwd: errno=" <<errno <<' ' <<strerror(errno));
   return std::string(buff);
}

//===========================================================================
//
// explicit instantiations
//

#include "util_t.h"
template void MT::getParameter(int&, const char*);
template void MT::getParameter(int&, const char*, const int&);
template void MT::getParameter(uint&, const char*);
template void MT::getParameter(uint&, const char*, const uint&);
template void MT::getParameter(bool&, const char*, const bool&);
template void MT::getParameter(double&, const char*);
template void MT::getParameter(double&, const char*, const double&);
template void MT::getParameter(MT::String&, const char*, const MT::String&);
template void MT::getParameter(MT::String&, const char*);

template int MT::getParameter<int>(const char*);
template int MT::getParameter<int>(const char*, const int&);
template uint MT::getParameter(const char*);
template uint MT::getParameter<uint>(const char*, const uint&);
template float MT::getParameter<float>(const char*);
template double MT::getParameter<double>(const char*);
template double MT::getParameter<double>(const char*, const double&);
template bool MT::getParameter<bool>(const char*);
template bool MT::getParameter<bool>(const char*, const bool&);
template long MT::getParameter<long>(const char*);
template MT::String MT::getParameter<MT::String>(const char*);
template MT::String MT::getParameter<MT::String>(const char*, const MT::String&);

template bool MT::checkParameter<uint>(const char*);
template bool MT::checkParameter<bool>(const char*);

template void MT::Parameter<MT::String>::initialize();
template void MT::Parameter<bool>::initialize();
template void MT::Parameter<double>::initialize();
template void MT::Parameter<int>::initialize();
template void MT::Parameter<uint>::initialize();

template std::map<std::string,int> MT::ParameterMap<int>::m;
template std::map<std::string,double> MT::ParameterMap<double>::m;
template std::map<std::string,unsigned int> MT::ParameterMap<unsigned int>::m;
template std::map<std::string,float> MT::ParameterMap<float>::m;
template std::map<std::string,bool> MT::ParameterMap<bool>::m;
template std::map<std::string,long> MT::ParameterMap<long>::m;
template std::map<std::string,MT::String> MT::ParameterMap<MT::String>::m;
template std::map<std::string,std::string> MT::ParameterMap<std::string>::m;
