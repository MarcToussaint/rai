#include <Core/thread.h>

TStream tout(cout);
Mutex m;

//===========================================================================

// Normal Thread struct
struct MyThread: Thread{
  VariableData<double>& x;
  uint n;
  MyThread(VariableData<double>& x, uint n, double beat):Thread(STRING("MyThread_"<<n), beat), x(x), n(n){
    threadOpen();
  }
  void open(){}
  void close(){}
  void step(){
    x.set(this)++;
    COUT <<mlr::realTime() <<"sec Thread " <<n <<" is counting:" <<x.get() <<endl;
  }
};

void TEST(Thread){
  VariableData<double> x;
  x.value = 0.;
  MyThread t1(x, 1, .5), t2(x, 2, -1);

  t1.threadLoop();
  t2.listenTo(x); //whenever t1 modifies x, t2 is stepped
  
  mlr::wait(3.);

  t1.threadClose();
  t2.threadClose();

  CHECK(x.get()>=11. && x.get()<=15.,"");
}

//===========================================================================
//
// not using any 'system' code, directly creating the modules and 'completing'
// the variables

struct MyType{
  int i;
};

struct ComputeSum : Thread {
  ACCESS(arr, x)    //input
  ACCESS(double, s) //output
  ACCESS(MyType, i)

  ComputeSum():Thread("ComputeSum"){}
  virtual ~ComputeSum(){}

  void open(){}
  void close(){}
  void step(){
    s.set() = sum(x.get()());
  }
};

//===========================================================================
//
// mini test Way0

void TEST(Way0){
  ComputeSum C;
  C.x.set() = ARR(1., 2., 3.);
  C.open();
  C.step();
  C.close();
  cout <<C.s.get() <<endl;
}

//===========================================================================
//
// direct execution - Way1
//

void TEST(Way1){
  Thread *m = new ComputeSum;

  Access<arr> x(NULL, "x");
  Access<double> s(NULL, "s");

  x.set() = {1., 2., 3.};

#if 0 //serial
  m->open();
  m->step();
  m->close();
#else //threaded
  m->threadOpen();
  m->threadStep();
  mlr::wait(.001); //give it a tiny tiny bit of time to at least step once (it could happen that the condition variable has not waken up to detect the step BEFORE threadClose chanes the state again)
  m->threadClose();
#endif

  cout <<"result = " <<s.get() <<endl;
  delete m;
}

//===========================================================================
//
// direct execution - Way2
//

struct MySystem{
  Access<arr> x = Access<arr>(NULL, "x");
  Access<double> s = Access<double>(NULL, "s");
  ComputeSum cs;
};

void TEST(Way2){
  cout <<registry() <<endl;

  MySystem S;

  S.x.set() = {1., 2., 3.};

  openModules();
  stepModules();
  closeModules();

  cout <<"result = " <<S.s.get() <<endl;
};

//===========================================================================
//
// testing
//

void TEST(SystemConnect) {
  cout <<"**** ENTER_MAIN" <<endl;

  cout <<registry() <<endl;

  ComputeSum CS;

  cout <<registry() <<endl;

  registry()->displayDot();

//  GraphView gv(registry());
//  gv.watch();
}

//===========================================================================
//
// test with a pair-wise distributed sorter
//

struct PairSorter:Thread{
  Access<int> a;
  Access<int> b;
  PairSorter(const char *a_name, const char* b_name)
    : Thread(STRING("S_"<<a_name<<"_"<<b_name)),
      a(this, a_name),
      b(this, b_name){}
  PairSorter():Thread("S"), a(this, "a"), b(this, "b"){}
  void open(){}
  void close(){}
  void step();
};

//==============================================================================

void TEST(ModuleSorter1){
  uint N=20;

  cout <<registry() <<endl <<"----------------------------" <<endl;
  mlr::Array<PairSorter*> ps;
  for(uint i=0;i<N-1;i++)
    ps.append( new PairSorter(STRING("int"<<i), STRING("int"<<i+1)) );
  cout <<registry() <<endl <<"----------------------------" <<endl;
  auto vars = getVariablesOfType<int>();
  CHECK_EQ(vars.N, N, "");

  threadOpenModules(true);

  for(uint i=0;i<N;i++) vars(i)->set() = rnd(100);

  for(uint k=0;k<20;k++){
    if(moduleShutdown()->getStatus()) break;
    for(uint i=0;i<N;i++) cout <<vars(i)->get() <<' ';  cout <<endl;
    stepModules();
    mlr::wait(.1);
  }

  threadCloseModules();

  for(auto& x:ps){ delete x; x=NULL; }

}

//==============================================================================

void TEST(ModuleSorter2){
  uint N=20;

  mlr::Array<PairSorter*> ps;

  for(uint i=0;i<N-1;i++)
    ps.append( new PairSorter(STRING("int"<<i), STRING("int"<<i+1)) );

  cout <<registry() <<endl;

  for(uint i=0;i<N-1;i++) ps(i)->a.set() = rnd(100);
  ps.last()->b.set() = rnd(100);

  openModules();

  for(uint k=0;k<20;k++){
    for(uint i=0;i<N-1;i++) cout <<ps(i)->a.get() <<' ';  cout <<endl;
    stepModules();
    mlr::wait(.1);
  }

  closeModules();

  for(auto& x:ps){ delete x; x=NULL; }
}

//==============================================================================


void PairSorter::step(){
  int xa = a.get();
  int xb = b.get();//->get_x(this);
  if(xa>xb){  //swap numbers
    a.writeAccess(); // we want to lock both to ensure no number is lost!
    b.writeAccess();
    xa = a();
    xb = b();
    if(xa>xb){  //still?
      a() = xb;
      b() = xa;
    }
    b.deAccess();
    a.deAccess();
  }
}

//==============================================================================
//
// logging with threads
//


// Normal Thread struct
struct MyLogThread: Thread{
  uint n;
  MyLogThread(uint n, double beatIntervalSec=0.):Thread(STRING("MyThread_"<<n), beatIntervalSec), n(n){}
  void open(){}
  void close(){}
  void step(){
    LOG(0) <<mlr::realTime() <<"sec Thread " <<n;
  }
};

void TEST(Logging){
  MyLogThread t1(1, .5), t2(2, .25);

  t1.threadLoop();
  t2.threadLoop();

  LOG(0) <<"starting to wait";
  mlr::wait(3.);

  LOG(0) <<"done with wait";

  t1.threadClose();
  t2.threadClose();
}

//===========================================================================

int MAIN(int argc,char** argv){
  mlr::initCmdLine(argc, argv);

  testThread();
  testWay0();
  testWay1();
  testWay2();
  testSystemConnect();
  testModuleSorter1();
  testModuleSorter2();
  testLogging();

  return 0;
}
