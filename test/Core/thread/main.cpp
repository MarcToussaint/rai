#include <Core/thread.h>

//===========================================================================

// Normal Thread struct
struct MyThread: Thread{
  Var<double> x;
  uint n;
  MyThread(Var<double>& _x, uint n, double beat)
    : Thread(STRING("MyThread_"<<n), beat), x(this, _x), n(n){
    threadOpen();
  }
  void step(){
    if(n<=1) x.set()++; //only thread 1 should modify x!
    COUT <<"Thread " <<n <<": " <<rai::realTime() <<"sec  - is counting:" <<x.get() <<endl;
  }
};

void TEST(Thread){
  Var<double> x;
  x.set() = 0.;
  MyThread t1(x, 1, .5), t2(x, 2, -1);

  t1.threadLoop();
  t2.event.listenTo(x); //whenever t1 modifies x, t2 is stepped
  
  rai::wait(3.);

  t1.threadClose();
  t2.threadClose();

  CHECK(x.get()>=5. && x.get()<=7.,"");
}

//===========================================================================
//
// not using any 'system' code, directly creating the modules and 'completing'
// the variables

struct MyType{
  int i;
};

struct ComputeSum : Thread {
  Var<arr> x;    //input
  Var<double> s; //output
  Var<MyType> i;

  ComputeSum():Thread("ComputeSum"){}
  virtual ~ComputeSum(){ threadClose(); }

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
  auto m = make_shared<ComputeSum>();

  Var<arr> x(m->x);
  Var<double> s(m->s);

  x.set() = {1., 2., 3.};

#if 0 //serial
  m->open();
  m->step();
  m->close();
#else //threaded
  m->threadOpen();
  m->threadStep();
  rai::wait(.001); //give it a tiny tiny bit of time to at least step once (it could happen that the condition variable has not waken up to detect the step BEFORE threadClose chanes the state again)
  m->threadClose();
#endif

  cout <<"result = " <<s.get() <<endl;
}

//===========================================================================
//
// test with a pair-wise distributed sorter
//

struct PairSorter:Thread{
  Var<int> a;
  Var<int> b;
  PairSorter(Var<int>& _a, Var<int>& _b)
    : Thread(STRING("S_"<<_a.name()<<"_"<<_b.name())),
      a(this, _a),
      b(this, _b){}
  ~PairSorter(){
    threadClose();
  }
  void step();
};

//==============================================================================

void TEST(Sorter){
  uint N=20;

  rai::Array<Var<int>> x(N);
  rai::Array<ptr<PairSorter>> ps(N-1);
  for(uint i=0;i<N-1;i++)
    ps(i) = make_shared<PairSorter>(x(i), x(i+1));

  {
    for(auto& s:ps) s->threadOpen();

    for(uint i=0;i<N;i++) x(i).set() = rnd(100);

    for(uint k=0;k<20;k++){
      //if(moduleShutdown()->getStatus()) break;
      for(uint i=0;i<N;i++) cout <<x(i).get() <<' ';
      cout <<endl;
      for(auto& s:ps) s->threadStep();
      rai::wait(.1);
    }

    for(auto& s:ps) s->threadClose();
  }

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
    LOG(0) <<rai::realTime() <<"sec Thread " <<n;
  }
};

void TEST(Logging){
  MyLogThread t1(1, .5), t2(2, .25);

  t1.threadLoop();
  t2.threadLoop();

  LOG(0) <<"starting to wait";
  rai::wait(3.);

  LOG(0) <<"done with wait";

  t1.threadClose();
  t2.threadClose();
}

//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc, argv);

  testThread();
  testSorter();

  testWay0();
  testWay1();
  testLogging();

  return 0;
}
